/**
 * @file rtp_bbg.c
 * @brief BBG Vehicle Gateway — I2C reader + UDP/RTP sender.
 *
 * Single process, two threads:
 *
 *   Thread 1 — I2C reader
 *     Opens /dev/i2c-2 as master, reads 84-byte frames from STM32.
 *     Validates frame (sync byte, msg_type).
 *     Writes frame to shared slot, signals RTP sender thread.
 *
 *   Thread 2 — RTP sender
 *     Waits on condvar for new frame from I2C reader.
 *     Builds 12-byte RTP header (seq++, timestamp, SSRC).
 *     Sends 96-byte UDP datagram to fleet server over 4G/WiFi.
 *
 * Communication between threads:
 *   frame_slot_t — single frame slot with mutex + condvar.
 *   Not a ring buffer: if the RTP sender is slow, the I2C reader
 *   overwrites with the latest frame (newest data always sent).
 *
 * Compile:  gcc rtp_bbg.c -o rtp_bbg -lpthread -lm
 * Run:      sudo ./rtp_bbg <SERVER_IP> <SERVER_PORT>
 *           e.g. sudo ./rtp_bbg 85.64.12.34 5004
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <linux/i2c-dev.h>

#include "../common/common.h"
#include "rtp_bbg.h"

/* ── Globals ─────────────────────────────────────────────────────────── */
static volatile int  g_running = 1;
static frame_slot_t  g_slot;

/* ── Signal handler ──────────────────────────────────────────────────── */
static void handle_signal(int s) { (void)s; g_running = 0; }

/* ────────────────────────────────────────────────────────────────────────
 * RTP header builder
 * ───────────────────────────────────────────────────────────────────── */

/**
 * @brief Build a 12-byte RTP header into buf (big-endian fields).
 */
static void rtp_build_header(uint8_t *buf,
                              uint16_t seq,
                              uint32_t timestamp,
                              uint32_t ssrc)
{
    buf[0]  = 0x80;                         /* V=2, P=0, X=0, CC=0 */
    buf[1]  = RTP_PAYLOAD_TYPE & 0x7F;      /* M=0, PT=96          */
    buf[2]  = (seq >> 8) & 0xFF;            /* seq high byte       */
    buf[3]  = seq & 0xFF;                   /* seq low byte        */
    buf[4]  = (timestamp >> 24) & 0xFF;
    buf[5]  = (timestamp >> 16) & 0xFF;
    buf[6]  = (timestamp >>  8) & 0xFF;
    buf[7]  =  timestamp        & 0xFF;
    buf[8]  = (ssrc >> 24) & 0xFF;
    buf[9]  = (ssrc >> 16) & 0xFF;
    buf[10] = (ssrc >>  8) & 0xFF;
    buf[11] =  ssrc        & 0xFF;
}

/* ────────────────────────────────────────────────────────────────────────
 * Thread 2 — RTP sender
 * ───────────────────────────────────────────────────────────────────── */

static void *rtp_sender_thread(void *arg)
{
    rtp_thread_arg_t *a = (rtp_thread_arg_t *)arg;

    /* ── Open UDP socket ──────────────────────────────────────────── */
    int udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_fd < 0) {
        printf("[RTP] socket() failed: %s\n", strerror(errno));
        g_running = 0;
        return NULL;
    }

    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family      = AF_INET;
    dest.sin_port        = htons((uint16_t)a->server_port);
    if (inet_pton(AF_INET, a->server_ip, &dest.sin_addr) <= 0) {
        printf("[RTP] Invalid server IP: %s\n", a->server_ip);
        close(udp_fd);
        g_running = 0;
        return NULL;
    }

    printf("[RTP] Sender ready → %s:%d\n", a->server_ip, a->server_port);

    uint16_t seq       = 0;
    uint32_t timestamp = 0;
    long     sent      = 0;

    while (g_running) {
        /* ── Wait for I2C reader to signal new frame ──────────────── */
        pthread_mutex_lock(&a->slot->mutex);
        while (!a->slot->has_data && g_running)
            pthread_cond_wait(&a->slot->cond, &a->slot->mutex);

        if (!g_running) {
            pthread_mutex_unlock(&a->slot->mutex);
            break;
        }

        telemetry_frame_t frame = a->slot->frame;  /* copy under lock */
        a->slot->has_data = 0;
        pthread_mutex_unlock(&a->slot->mutex);

        /* ── Build RTP packet ─────────────────────────────────────── */
        uint8_t pkt[RTP_PACKET_SIZE];

        rtp_build_header(pkt, seq, timestamp, RTP_SSRC_CAR001);
        memcpy(pkt + RTP_HEADER_SIZE, &frame, TELEMETRY_FRAME_SIZE);

        /* ── Send UDP datagram ────────────────────────────────────── */
        ssize_t n = sendto(udp_fd, pkt, RTP_PACKET_SIZE, 0,
                           (struct sockaddr *)&dest, sizeof(dest));
        if (n < 0) {
            printf("[RTP] sendto failed: %s\n", strerror(errno));
        } else {
            sent++;
            if (sent % 60 == 0)
                printf("[RTP] Sent %ld packets | seq=%u | type=%c\n",
                       sent, seq, frame.msg_type);
        }

        seq++;
        timestamp++;   /* 1 unit = 1 second at 1 Hz */
    }

    close(udp_fd);
    printf("[RTP] Sender thread stopped. Total sent: %ld\n", sent);
    return NULL;
}

/* ────────────────────────────────────────────────────────────────────────
 * Thread 1 — I2C reader
 * ───────────────────────────────────────────────────────────────────── */

static void *i2c_reader_thread(void *arg)
{
    (void)arg;

    /* ── Open I2C bus ─────────────────────────────────────────────── */
    int fd = open(I2C_DEV, O_RDWR);
    if (fd < 0) {
        printf("[I2C] Cannot open %s: %s\n", I2C_DEV, strerror(errno));
        g_running = 0;
        return NULL;
    }
    if (ioctl(fd, I2C_SLAVE, STM32_SLAVE_ADDR) < 0) {
        printf("[I2C] ioctl I2C_SLAVE failed: %s\n", strerror(errno));
        close(fd);
        g_running = 0;
        return NULL;
    }

    printf("[I2C] Opened %s slave=0x%02X frame=%d bytes\n",
           I2C_DEV, STM32_SLAVE_ADDR, TELEMETRY_FRAME_SIZE);

    /* Verify frame size */
    if ((int)sizeof(telemetry_frame_t) != TELEMETRY_FRAME_SIZE) {
        printf("[I2C] ERROR: sizeof(telemetry_frame_t)=%zu but expected %d\n",
               sizeof(telemetry_frame_t), TELEMETRY_FRAME_SIZE);
        close(fd);
        g_running = 0;
        return NULL;
    }

    long frames = 0;

    while (g_running) {
        uint8_t buf[TELEMETRY_FRAME_SIZE];
        memset(buf, 0, sizeof(buf));

        /* ── Read full frame (loop for partial reads) ─────────────── */
        int total = 0;
        while (total < TELEMETRY_FRAME_SIZE) {
            int n = read(fd, buf + total, TELEMETRY_FRAME_SIZE - total);
            if (n < 0) {
                printf("[I2C] read error: %s\n", strerror(errno));
                total = -1;
                break;
            }
            if (n == 0) break;
            total += n;
        }
        if (total < 0) { sleep(1); continue; }
        if (total < TELEMETRY_FRAME_SIZE) {
            printf("[I2C] Short read: %d/%d\n", total, TELEMETRY_FRAME_SIZE);
            continue;
        }

        /* ── Validate frame alignment using msg_type at offset 72 ─── */
        #define MSG_TYPE_OFFSET 72
        if (buf[MSG_TYPE_OFFSET] != MSG_TRIP_START &&
            buf[MSG_TYPE_OFFSET] != MSG_TRIP_DATA  &&
            buf[MSG_TYPE_OFFSET] != MSG_TRIP_END) {

            printf("[I2C] Misaligned frame buf[76]=0x%02X — resyncing\n",
                   buf[MSG_TYPE_OFFSET]);

            /* Shift 1 byte at a time until buf[76] is valid */
            int attempts = 0;
            while (attempts < TELEMETRY_FRAME_SIZE * 3) {
                memmove(buf, buf + 1, TELEMETRY_FRAME_SIZE - 1);
                uint8_t b = 0;
                if (read(fd, &b, 1) != 1) break;
                buf[TELEMETRY_FRAME_SIZE - 1] = b;
                attempts++;
                if (buf[MSG_TYPE_OFFSET] == MSG_TRIP_START ||
                    buf[MSG_TYPE_OFFSET] == MSG_TRIP_DATA  ||
                    buf[MSG_TYPE_OFFSET] == MSG_TRIP_END) {
                    printf("[I2C] Resynced after %d bytes\n", attempts);
                    break;
                }
            }
            if (buf[MSG_TYPE_OFFSET] != MSG_TRIP_START &&
                buf[MSG_TYPE_OFFSET] != MSG_TRIP_DATA  &&
                buf[MSG_TYPE_OFFSET] != MSG_TRIP_END)
                continue;
        }

        /* ── Write frame to shared slot → signal RTP sender ──────── */
        pthread_mutex_lock(&g_slot.mutex);
        memcpy(&g_slot.frame, buf, TELEMETRY_FRAME_SIZE);
        g_slot.frame.car_id[7]    = '\0';
        g_slot.frame.trip_id[19]  = '\0';
        g_slot.has_data = 1;
        pthread_cond_signal(&g_slot.cond);
        pthread_mutex_unlock(&g_slot.mutex);

        frames++;
        if (frames % 60 == 0)
            printf("[I2C] %ld frames received | type=%c car=%s\n",
                   frames,
                   ((telemetry_frame_t *)buf)->msg_type,
                   ((telemetry_frame_t *)buf)->car_id);
    }

    close(fd);
    printf("[I2C] Reader thread stopped\n");
    return NULL;
}

/* ────────────────────────────────────────────────────────────────────────
 * Main
 * ───────────────────────────────────────────────────────────────────── */

int main(int argc, char *argv[])
{
    if (argc < 3) {
        printf("Usage: %s <SERVER_IP> <SERVER_PORT>\n", argv[0]);
        printf("  e.g. %s 85.64.12.34 5004\n", argv[0]);
        return 1;
    }

    signal(SIGINT,  handle_signal);
    signal(SIGTERM, handle_signal);

    /* Initialise shared slot */
    pthread_mutex_init(&g_slot.mutex, NULL);
    pthread_cond_init (&g_slot.cond,  NULL);
    g_slot.has_data = 0;

    printf("=== Vehicle Fleet BBG Gateway starting ===\n");
    printf("Server : %s:%s\n", argv[1], argv[2]);
    printf("I2C    : %s  slave=0x%02X\n", I2C_DEV, STM32_SLAVE_ADDR);
    printf("RTP    : UDP payload=%d bytes  total=%d bytes\n\n",
           TELEMETRY_FRAME_SIZE, RTP_PACKET_SIZE);

    /* Prepare RTP sender arguments */
    rtp_thread_arg_t rtp_arg;
    rtp_arg.server_ip   = argv[1];
    rtp_arg.server_port = atoi(argv[2]);
    rtp_arg.slot        = &g_slot;

    /* Start threads */
    pthread_t i2c_tid, rtp_tid;
    pthread_create(&i2c_tid, NULL, i2c_reader_thread, NULL);
    pthread_create(&rtp_tid,  NULL, rtp_sender_thread, &rtp_arg);

    pthread_join(i2c_tid, NULL);
    pthread_join(rtp_tid,  NULL);

    pthread_mutex_destroy(&g_slot.mutex);
    pthread_cond_destroy (&g_slot.cond);

    printf("[BBG] Shutdown complete\n");
    return 0;
}
