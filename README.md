# Vehicle Fleet Management System — V2
## Design Document: Real Car, UDP/RTP Transport
### Real-Time Group (RTG)

---

## 1. What Changed from V1

| Aspect          | V1 (previous)              | V2 (this document)              |
|-----------------|----------------------------|---------------------------------|
| Cars            | 100 simulated              | 1 car (real scenario)           |
| Transport       | TCP                        | UDP + RTP (RFC 3550)            |
| Network         | Ethernet cable             | 4G/LTE mobile network           |
| BBG design      | 101 threads                | 1 process, 2 threads            |
| Server input    | TCP poll() loop            | UDP socket, RTP parser          |
| Packet loss     | Not handled (TCP)          | Detected via RTP sequence number|
| Frame fields    | car_id, driver_id, lat/lon | car_id, lat/lon/altitude        |
| driver_id       | Present                    | Removed (1 car = 1 driver)      |
| GPS parameters  | latitude, longitude        | latitude, longitude, altitude   |

---

## 2. System Architecture

```
┌──────────────────────────────────────────────────┐
│  STM32 (in car)                                  │
│  Simulates accel + gyro + GPS at 1 Hz            │
│  Sends 84-byte telemetry_frame_t via I2C slave   │
└─────────────────────┬────────────────────────────┘
                      │ I2C  84 bytes @ 1 Hz
                      ▼
┌──────────────────────────────────────────────────┐
│  BBG (in car, single process, 2 threads)         │
│                                                  │
│  Thread 1 — I2C reader                           │
│    reads frame from STM32 via /dev/i2c-2         │
│    validates msg_type at offset 72               │
│    writes to in-process frame_slot_t             │
│                                                  │
│  Thread 2 — RTP sender                           │
│    waits on condvar for new frame                │
│    builds RTP header (seq++, timestamp, SSRC)    │
│    sends 96-byte UDP datagram to server          │
│    uses 4G/LTE network interface (e.g. wwan0)    │
└─────────────────────┬────────────────────────────┘
                      │ UDP/RTP  96 bytes @ 1 Hz
                      │ 4G mobile network
                      │ Server public IP : port 5004
                      ▼
┌──────────────────────────────────────────────────┐
│  Fleet Server (Linux PC)                         │
│                                                  │
│  UDP socket on port 5004                         │
│  RTP parser: validate header, seq number         │
│  Detects lost packets (gap in seq number)        │
│  Feeds TelemetryParser → TripManager             │
│  ScoringEngine → CompletedTrip → TripQueue       │
│  DB thread → SQLite                              │
└──────────────────────────────────────────────────┘
```

---

## 3. RTP Packet Format

Standard RTP header (RFC 3550) = 12 bytes
Telemetry payload = 84 bytes
**Total UDP datagram = 96 bytes**

```
Byte  0:    V=2, P=0, X=0, CC=0      → 0x80
Byte  1:    M=0, PT=96               → 0x60
Byte  2-3:  Sequence number          uint16, big-endian, increments per packet
Byte  4-7:  Timestamp                uint32, big-endian, seconds since trip start
Byte  8-11: SSRC                     uint32, big-endian, car ID = 0x00000001
Byte 12-95: telemetry_frame_t        84 bytes payload
```

---

## 4. Telemetry Frame

84 bytes, natural compiler layout — same struct on STM32, BBG, and server.
No `#pragma pack` needed — verified with `offsetof` on all targets.

### Changes from V1
- `driver_id[8]` removed — one car means one driver, field was redundant
- `altitude` added as third GPS parameter (standard WGS-84: lat, lon, alt)
- `trip_id` moved up to offset 8 (was 16, after driver_id)
- `msg_type` moved to offset 72 (was 76, after driver_id removal)
- `reserved` expanded from `[5]` to `[9]` to keep total at 84 bytes

### Layout

```
Offset  Size  Field            Type      Description
──────  ────  ───────────────  ────────  ─────────────────────────────
  [0]    8    car_id           char[8]   "CAR-001\0"
  [8]   20    trip_id          char[20]  "T1234567001\0"
 [28]    4    latitude         float     degrees (WGS-84)
 [32]    4    longitude        float     degrees (WGS-84)
 [36]    4    altitude         float     metres above sea level ← NEW
 [40]    4    gps_speed_kmh    float     km/h from GPS
 [44]    4    accel_x          float     m/s²  forward/back
 [48]    4    accel_y          float     m/s²  lateral
 [52]    4    accel_z          float     m/s²  vertical
 [56]    4    gyro_x           float     °/s   pitch
 [60]    4    gyro_y           float     °/s   roll
 [64]    4    gyro_z           float     °/s   yaw (turn rate)
 [68]    4    timestamp_sec    uint32    Unix timestamp
 [72]    1    msg_type         uint8     'S'=start 'D'=data 'E'=end
 [73]    1    gps_fix          uint8     0=no fix  1=fix
 [74]    1    satellites       uint8     satellites in view
 [75]    9    reserved[9]      uint8[]   [0]=0xAB sync marker
──────  ────
Total  84 bytes
```

### Sync marker

`reserved[0] = 0xAB` is set by the STM32 on every frame.
The BBG checks `buf[72]` (msg_type) for a valid value `'S'`,`'D'`,`'E'`
to detect frame misalignment on the I2C bus and trigger resync.

### Altitude simulation (STM32)

Since GPS altitude is not available in simulation, use a realistic
value for the car's city:

```c
frame->altitude = 30.0f + lcg_float(&seed, -5.0f, 5.0f);  /* Tel Aviv ~30m ASL */
```

---

## 5. File Structure

```
rtp_fleet/
├── common/
│   ├── common.h              telemetry_frame_t (84B), rtp_header_t, constants
│   ├── Logger.hpp            reuse from V1
│   └── Config.hpp            reuse from V1
├── server/
│   ├── server_main.cpp       entry point — RTP thread + DB thread
│   ├── RtpReceiver.hpp       UDP socket, recvfrom() loop, RTP validation
│   ├── RtpHeader.hpp         RTP header build/parse, lost packet detection
│   ├── TelemetryParser.hpp   binary frame → TelemetrySample (no driver_id)
│   ├── TripManager.hpp       active trip tracking, scoring on trip END
│   ├── TripQueue.hpp         std::queue between RTP thread and DB thread
│   ├── ViolationDetector.hpp real-time alert logging
│   └── Database.hpp/.cpp     SQLite (trips, violations, cars tables)
├── bbg/
│   ├── rtp_bbg.c             single process, 2 threads, I2C + RTP/UDP
│   └── rtp_bbg.h             frame_slot_t, rtp_thread_arg_t
├── stm32/
│   └── main.c                I2C slave, 1 car, fleet_data.c simulation
├── config/
│   └── rtp_fleet.cfg
└── Makefile
```

---

## 6. Packet Loss Handling

Over 4G, packet loss of 1–5% is normal. RTP sequence numbers allow
the server to detect gaps:

```
Server receives: seq=1, seq=2, seq=4  ← seq=3 missing!

lost = new_seq - last_seq - 1 = 4 - 2 - 1 = 1 lost packet

Server actions:
  a) Log the gap                        (always)
  b) Increment lost_packets counter     (always)
  c) Penalise driving score             (−1 point per lost packet)

Server tracks per trip:
  last_seq      — last received sequence number
  lost_packets  — cumulative gaps detected
  total_rx      — total packets received
```

uint16 wrap-around is handled: if `new_seq < last_seq`, the difference
is computed modulo 65536.

---

## 7. 4G Network Considerations

The BBG connects to the internet via a 4G USB modem or SIM module.
The network interface is typically `wwan0` or `usb0`.

The server needs a **public IP address** so the BBG can reach it
from the mobile network:

| Scenario | Solution |
|----------|----------|
| Production | Static public IP on server |
| Development | Dynamic DNS (no-ip.com, dyndns.org) |
| Testing | Same WiFi/Ethernet network — code is identical |

For development and testing, WiFi or Ethernet can be used instead
of 4G. Only the network interface on the BBG changes — the UDP/RTP
code is completely identical.

---

## 8. Scoring

```
score = 100
      − harsh_brake_count × 5   (accel_x < −4.0 m/s²)
      − harsh_accel_count × 3   (accel_x >  3.0 m/s²)
      − harsh_turn_count  × 4   (|gyro_z| > 45 °/s)
      − overspeed_count   × 2   (gps_speed > 60 km/h)
      − lost_rtp_packets  × 1   (4G connectivity quality)
      minimum 0

Grade:  90–100  Excellent
        75–89   Good
        60–74   Acceptable
        40–59   Poor
         0–39   Dangerous
```

The `lost_rtp_packets` penalty is new in V2 — it rewards drivers
whose vehicle has good 4G coverage and penalises areas with poor
signal, giving fleet operators visibility into both driving quality
and network infrastructure quality.

---

## 9. SQLite Schema

```sql
CREATE TABLE cars (
    car_id       TEXT PRIMARY KEY,
    total_trips  INTEGER DEFAULT 0,
    avg_score    REAL    DEFAULT 0,
    total_km     REAL    DEFAULT 0
);

CREATE TABLE trips (
    id                 INTEGER PRIMARY KEY AUTOINCREMENT,
    trip_id            TEXT UNIQUE,
    car_id             TEXT,
    start_time         TEXT,
    end_time           TEXT,
    distance_km        REAL,
    score              INTEGER,
    grade              TEXT,
    harsh_brake_count  INTEGER,
    harsh_accel_count  INTEGER,
    harsh_turn_count   INTEGER,
    overspeed_count    INTEGER,
    max_speed_kmh      REAL,
    max_altitude_m     REAL,
    lost_rtp_packets   INTEGER
);

CREATE TABLE violations (
    id           INTEGER PRIMARY KEY AUTOINCREMENT,
    trip_id      TEXT,
    car_id       TEXT,
    timestamp    TEXT,
    type         TEXT,
    severity     REAL,
    latitude     REAL,
    longitude    REAL,
    altitude     REAL
);
```

Note: `driver_id` column removed from all tables.
`cars` table replaces the previous `drivers` table, keyed by `car_id`.
`altitude` added to `violations` for precise incident location.

---

## 10. Development Phases

| Phase | Task | Status |
|-------|------|--------|
| 1 | BBG RTP sender (`rtp_bbg.c`) — I2C reader + UDP sender | Done |
| 2 | Server UDP/RTP receiver (`RtpReceiver.hpp`) | Done |
| 3 | TripManager, ViolationDetector, Database (adapted from V1) | Done |
| 4 | Test over WiFi (same code, no 4G needed) | Next |
| 5 | Test over real 4G with SIM module | After WiFi test passes |
