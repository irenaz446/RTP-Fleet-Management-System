/**
 * @file RtpHeader.hpp
 * @brief RTP header construction and parsing (RFC 3550).
 *
 * Handles network byte order (big-endian) conversion for all
 * multi-byte RTP header fields.
 */

#pragma once

#include <cstring>
#include <cstdint>
#include <arpa/inet.h>   /* htons, htonl, ntohs, ntohl */
#include "../common/common.h"

class RtpHeader {
public:
    /**
     * @brief Build a 12-byte RTP header into buf.
     *
     * @param buf       Output buffer, at least 12 bytes.
     * @param seq       Sequence number (host byte order).
     * @param timestamp RTP timestamp (host byte order).
     * @param ssrc      SSRC identifier (host byte order).
     */
    static void build(uint8_t *buf,
                      uint16_t seq,
                      uint32_t timestamp,
                      uint32_t ssrc)
    {
        rtp_header_t h;
        h.first     = 0x80;                    /* V=2, P=0, X=0, CC=0 */
        h.second    = RTP_PAYLOAD_TYPE & 0x7F; /* M=0, PT=96          */
        h.seq       = htons(seq);
        h.timestamp = htonl(timestamp);
        h.ssrc      = htonl(ssrc);
        std::memcpy(buf, &h, RTP_HEADER_SIZE);
    }

    /**
     * @brief Parse a 12-byte RTP header from buf.
     *
     * @param buf   Input buffer, at least 12 bytes.
     * @param seq   Output sequence number (host byte order).
     * @param ts    Output timestamp (host byte order).
     * @param ssrc  Output SSRC (host byte order).
     * @return true if header is valid (V=2, correct PT).
     */
    static bool parse(const uint8_t *buf,
                      uint16_t &seq,
                      uint32_t &ts,
                      uint32_t &ssrc)
    {
        rtp_header_t h;
        std::memcpy(&h, buf, RTP_HEADER_SIZE);

        /* Validate version = 2 */
        if ((h.first >> 6) != 2) return false;

        /* Validate payload type */
        if ((h.second & 0x7F) != RTP_PAYLOAD_TYPE) return false;

        seq  = ntohs(h.seq);
        ts   = ntohl(h.timestamp);
        ssrc = ntohl(h.ssrc);
        return true;
    }

    /**
     * @brief Check for lost packets using sequence numbers.
     *
     * @param last_seq  Last received sequence number.
     * @param new_seq   Current received sequence number.
     * @return Number of lost packets (0 = no loss, >0 = gap detected).
     */
    static int lostPackets(uint16_t last_seq, uint16_t new_seq)
    {
        /* Handle uint16 wrap-around (65535 → 0) */
        int diff = (int)new_seq - (int)last_seq;
        if (diff < 0) diff += 65536;
        return (diff > 1) ? diff - 1 : 0;
    }
};
