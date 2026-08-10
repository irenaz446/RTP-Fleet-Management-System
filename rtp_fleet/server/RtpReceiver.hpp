/**
 * @file RtpReceiver.hpp
 * @brief UDP/RTP receiver — listens for telemetry packets from the BBG.
 *
 * One blocking recvfrom() loop receives 96-byte UDP datagrams.
 * Each datagram is validated as an RTP packet, the telemetry payload
 * is extracted, and statistics (lost packets, jitter) are tracked.
 */

#pragma once

#include <string>
#include <functional>
#include <cstring>
#include <cstdio>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cerrno>

#include "../common/common.h"
#include "../common/Logger.hpp"
#include "RtpHeader.hpp"

/** @brief Called by RtpReceiver for each valid received telemetry frame. */
using FrameCallback = std::function<void(const telemetry_frame_t &,
                                         uint16_t seq,
                                         uint32_t ssrc,
                                         int lost)>;

class RtpReceiver {
public:
    explicit RtpReceiver(int port) : port_(port) {}

    ~RtpReceiver() { if (fd_ >= 0) ::close(fd_); }

    /**
     * @brief Open UDP socket and bind to port.
     * @return true on success.
     */
    bool open()
    {
        fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) {
            Logger::error("RtpReceiver: socket() failed: " +
                          std::string(strerror(errno)));
            return false;
        }

        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port        = htons((uint16_t)port_);

        if (bind(fd_, (sockaddr *)&addr, sizeof(addr)) < 0) {
            Logger::error("RtpReceiver: bind() port " +
                          std::to_string(port_) + " failed: " +
                          std::string(strerror(errno)));
            ::close(fd_);
            fd_ = -1;
            return false;
        }

        Logger::info("RtpReceiver: UDP socket open on port " +
                     std::to_string(port_));
        return true;
    }

    /**
     * @brief Receive loop — blocks until running_ is false.
     *
     * For each received datagram:
     *   1. Validate as RTP (header check)
     *   2. Detect lost packets via sequence number gap
     *   3. Extract telemetry_frame_t from payload
     *   4. Call the frame callback
     *
     * @param running  Reference to server running flag.
     * @param cb       Callback for each valid frame.
     */
    void run(const volatile bool &running, FrameCallback cb)
    {
        uint8_t  buf[RTP_PACKET_SIZE + 16];
        uint16_t last_seq    = 0;
        bool     first       = true;
        long     total_rx    = 0;
        long     total_lost  = 0;

        Logger::info("RtpReceiver: waiting for RTP packets...");

        while (running) {
            sockaddr_in src{};
            socklen_t   src_len = sizeof(src);

            ssize_t n = recvfrom(fd_, buf, sizeof(buf), 0,
                                 (sockaddr *)&src, &src_len);
            if (n < 0) {
                if (errno == EINTR) continue;
                Logger::error("RtpReceiver: recvfrom failed: " +
                              std::string(strerror(errno)));
                break;
            }

            if (n < RTP_PACKET_SIZE) {
                Logger::warn("RtpReceiver: short packet " +
                             std::to_string(n) + " bytes — discarding");
                continue;
            }

            /* ── Parse RTP header ──────────────────────────────── */
            uint16_t seq;
            uint32_t ts, ssrc;
            if (!RtpHeader::parse(buf, seq, ts, ssrc)) {
                Logger::warn("RtpReceiver: invalid RTP header — discarding");
                continue;
            }

            /* ── Detect lost packets ────────────────────────────── */
            int lost = 0;
            if (!first) {
                lost = RtpHeader::lostPackets(last_seq, seq);
                if (lost > 0) {
                    total_lost += lost;
                    Logger::warn("RtpReceiver: " +
                                 std::to_string(lost) +
                                 " packet(s) lost (seq " +
                                 std::to_string(last_seq) +
                                 " → " + std::to_string(seq) + ")");
                }
            }
            last_seq = seq;
            first    = false;
            total_rx++;

            /* ── Extract telemetry payload ──────────────────────── */
            telemetry_frame_t frame;
            std::memcpy(&frame, buf + RTP_HEADER_SIZE,
                        TELEMETRY_FRAME_SIZE);

            /* Null-terminate string fields */
            frame.car_id[7]    = '\0';
            frame.driver_id[7] = '\0';
            frame.trip_id[19]  = '\0';

            /* Log every 100 packets */
            if (total_rx % 100 == 0) {
                Logger::info("RtpReceiver: rx=" +
                             std::to_string(total_rx) +
                             " lost=" + std::to_string(total_lost) +
                             " ssrc=0x" +
                             std::to_string(ssrc));
            }

            /* ── Deliver to application layer ───────────────────── */
            cb(frame, seq, ssrc, lost);
        }

        Logger::info("RtpReceiver: stopped. total_rx=" +
                     std::to_string(total_rx) +
                     " total_lost=" + std::to_string(total_lost));
    }

    void stop() { running_local_ = false; }

private:
    int  fd_            = -1;
    int  port_;
    bool running_local_ = true;
};
