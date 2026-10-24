# Vehicle Fleet Management System — RTP Fleet V3

Real-time vehicle telemetry: physical sensors on STM32 → I2C to BeagleBone Green →
RTP/UDP over 4G → C++ server.

---

## System Architecture

```
┌─────────────────────────────────────────────────────┐
│  Vehicle hardware                                    │
│                                                      │
│  MPU-6050 ──I2C1──► STM32F4 ──I2C2──► BeagleBone   │
│  NEO-6M   ──UART──►  slave            Green (BBG)   │
│                     (84-byte frame)    RTP/UDP       │
└─────────────────────────────────────────────────────┘
                                              │ 4G / LTE
                                              ▼
                                   ┌─────────────────┐
                                   │  Linux Server   │
                                   │  C++ (RTP/UDP)  │
                                   │  SQLite + REST  │
                                   └─────────────────┘
```

### Data flow, step by step

1. **MPU-6050** — STM32 reads 14 bytes via `HAL_I2C_Mem_Read` on I2C1 every second.
   Produces: acceleration (m/s²), angular rate (°/s).
2. **NEO-6M GPS** — NMEA sentences arrive at 9600 baud on USART1.
   `HAL_UART_RxCpltCallback` pushes each byte into a ring buffer.
   `fleet_data_update()` drains the buffer and parses `$GPGGA` (lat, lon, alt, fix)
   and `$GPRMC` (lat, lon, speed) sentences.
3. **STM32 I2C2 slave** — BBG polls the STM32 once per second and reads the
   84-byte `telemetry_frame_t` via `HAL_I2C_Slave_Transmit_IT`.
   A double-buffer guarantees the BBG always sees a consistent snapshot.
4. **BBG (`rtp_bbg.c`)** — wraps the frame in RTP (sequence number, timestamp,
   SSRC) and sends it over UDP to the server.
5. **Server** — receives RTP/UDP datagrams, strips the RTP header, stores
   `telemetry_frame_t` in SQLite, exposes REST API for dashboards.

---

## Hardware

### Components

| Component | Part | Interface | Voltage |
|-----------|------|-----------|---------|
| MCU | STM32F401RE (Nucleo-64) | — | 3.3 V |
| IMU | MPU-6050 breakout | I2C | 3.3 V |
| GPS | NEO-6M or NEO-8M module | UART | 3.3 V |
| SBC | BeagleBone Green | I2C master | 3.3 V |

### Wiring

#### MPU-6050 → STM32

| MPU-6050 pin | STM32 pin | Notes |
|--------------|-----------|-------|
| VCC | 3.3 V | |
| GND | GND | |
| SDA | **PB7** (I2C1 SDA) | 4.7 kΩ pull-up to 3.3 V |
| SCL | **PB6** (I2C1 SCL) | 4.7 kΩ pull-up to 3.3 V |
| AD0 | GND | I2C address = 0x68 |
| INT | — | not used |

> **Pull-ups:** If the MPU-6050 breakout board has no on-board pull-ups,
> add two 4.7 kΩ resistors between SDA↔3.3 V and SCL↔3.3 V.

#### NEO-6M / NEO-8M GPS → STM32

| GPS pin | STM32 pin | Notes |
|---------|-----------|-------|
| VCC | 3.3 V (or 5 V if module has regulator) | |
| GND | GND | |
| TX | **PA10** (USART1 RX) | GPS transmits; STM32 receives |
| RX | **PA9** (USART1 TX) | optional — only needed to send UBX config |

> The GPS TX line runs at 3.3 V logic; connect directly to PA10.
> If the module runs at 5 V and has no level-shifter, use a 1 kΩ / 2 kΩ
> voltage divider on the TX→PA10 line.

#### STM32 I2C2 slave → BeagleBone Green (unchanged from V2)

| STM32 pin | BBG pin | Notes |
|-----------|---------|-------|
| **PB10** (I2C2 SCL) | P9\_19 | 4.7 kΩ pull-up to 3.3 V |
| **PB11** (I2C2 SDA) | P9\_20 | 4.7 kΩ pull-up to 3.3 V |
| GND | P9\_1 | common ground |

STM32 slave address on I2C2: **0x30** (defined as `BBG_I2C_SLAVE_ADDR` in `main.c`).

---

## STM32 CubeMX Configuration

Open your `.ioc` file and set the following. Everything not listed stays at default.

### Clock Configuration

| Setting | Value |
|---------|-------|
| HSE input | 8 MHz (Nucleo uses ST-LINK MCO) |
| PLL M | 8 |
| PLL N | 336 |
| PLL P | 4 → **SYSCLK = 84 MHz** |
| AHB Prescaler | 1 → HCLK = 84 MHz |
| APB1 Prescaler | 2 → APB1 = 42 MHz (I2C2, TIM2, USART3) |
| APB2 Prescaler | 1 → APB2 = 84 MHz (USART1) |

### Peripherals

#### I2C1 — MPU-6050 master (PB6, PB7)

| Parameter | Value |
|-----------|-------|
| Mode | I2C |
| Speed Mode | Standard Mode |
| Clock Speed | 100 000 Hz |

Enable **I2C1 event interrupt** and **I2C1 error interrupt** in NVIC tab.

#### I2C2 — BBG slave (PB10, PB11)

| Parameter | Value |
|-----------|-------|
| Mode | I2C |
| Speed Mode | Standard Mode |
| Clock Speed | 100 000 Hz |
| Own Address 1 | 0x30 |

Enable **I2C2 event interrupt** and **I2C2 error interrupt** in NVIC tab.

#### USART1 — GPS (PA10 = RX)

| Parameter | Value |
|-----------|-------|
| Mode | Asynchronous |
| Baud Rate | 9 600 |
| Word Length | 8 Bits |
| Stop Bits | 1 |
| Parity | None |
| Direction | Receive Only |

Enable **USART1 global interrupt** in NVIC tab.

#### USART3 — debug console (PD8 = TX, PD9 = RX)

| Parameter | Value |
|-----------|-------|
| Mode | Asynchronous |
| Baud Rate | 115 200 |
| Word Length | 8 Bits |
| Direction | Receive and Transmit |

#### TIM2 — 1 Hz tick

| Parameter | Value |
|-----------|-------|
| Clock Source | Internal Clock |
| Prescaler | 8399 |
| Counter Period | 9999 |
| auto-reload preload | Disable |

Calculation: TIM2 clock = 84 MHz, Prescaler = 8400 → 10 kHz tick, Period = 10 000 → 1 Hz.  
Enable **TIM2 global interrupt** in NVIC tab.

---

## Source Files

### Files that changed (V2 → V3)

| File | Change |
|------|--------|
| `Core/Src/main.c` | Added `MX_I2C1_Init`, `MX_USART1_UART_Init`, `fleet_sensor_init(&hi2c1, &huart1)` |
| `Core/Src/fleet_data.c` | **Replaced** simulated values with real MPU-6050 reads and NMEA parser |
| `Core/Inc/fleet_data.h` | Added MPU-6050 register defines, GPS buffer externs, unchanged `telemetry_frame_t` |

### Files that did NOT change

| File | Description |
|------|-------------|
| `rtp_bbg.c / .h` | BBG I2C master — reads same 84-byte frame |
| `server/` | All C++ server code unchanged |
| `common/common.h` | `telemetry_frame_t` struct layout identical |
| `config/rtp_fleet.cfg` | IP, port, SSRC settings unchanged |
| `Makefile` | No new source files; `-lm` was already present |

The BBG and server are completely unaware that real sensors replaced the simulation.
The 84-byte frame format is bit-for-bit identical.

---

## Build and Flash (STM32)

### Prerequisites

```bash
# ARM GCC toolchain
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi

# ST-LINK flash tool
sudo apt install stlink-tools
```

### Build

```bash
cd stm32/
make -j4
# Output: build/rtp_fleet.elf, build/rtp_fleet.bin
```

### Flash

```bash
st-flash write build/rtp_fleet.bin 0x08000000
```

Or use **STM32CubeIDE**: Run → Debug, which builds and flashes in one step.

### Debug console

```bash
# Open minicom or screen on the ST-LINK virtual COM port (115200 8N1)
screen /dev/ttyACM0 115200
# Expected output after power-on:
# [INIT] MPU-6050 awake on I2C1
# [INIT] GPS UART interrupt receive started
# === Fleet V3 started — real sensors ===
# [DATA] t=1s  lat=32.08450  lon=34.78920  alt=35.2m  spd=0.0km/h  ax=0.012 ay=-0.034 az=9.811
```

---

## Build and Run (BBG)

```bash
# On BeagleBone Green
cd rtp_bbg/
make
sudo ./rtp_bbg config/rtp_fleet.cfg
```

BBG reads the STM32 I2C2 slave at address 0x30 on `/dev/i2c-2`, wraps each
84-byte frame in an RTP header, and sends it as a UDP datagram to the server.

---

## Build and Run (Server)

```bash
cd server/
make
./rtp_server config/rtp_fleet.cfg
# Listening on UDP 5004, storing to SQLite, REST on port 8080
```

---

## Telemetry Frame Layout

All fields little-endian (ARM native). Layout must match between STM32 and BBG.

```
Offset  Size  Type     Field              Source
------  ----  ------   -----------------  --------------------------
  0      8    char[]   car_id             hardcoded "VEH-001"
  8     20    char[]   trip_id            hardcoded "TRIP-20261023-001"
 28      4    float    latitude           GPS $GPGGA / $GPRMC (decimal °)
 32      4    float    longitude          GPS $GPGGA / $GPRMC (decimal °)
 36      4    float    altitude           GPS $GPGGA field 9  (metres ASL)
 40      4    float    gps_speed_kmh      GPS $GPRMC (knots × 1.852)
 44      4    float    accel_x            MPU-6050 (m/s²)
 48      4    float    accel_y            MPU-6050 (m/s²)
 52      4    float    accel_z            MPU-6050 (m/s²)
 56      4    float    gyro_x             MPU-6050 (°/s)
 60      4    float    gyro_y             MPU-6050 (°/s)
 64      4    float    gyro_z             MPU-6050 (°/s)
 68      4    uint32   timestamp_sec      seconds since STM32 boot
 72      1    uint8    msg_type           'S'=start, 'D'=data, 'E'=end
 73      1    uint8    gps_fix            0=no fix, 1=GPS, 2=DGPS
 74      1    uint8    satellites         number of SVs in use
 75      9    uint8[]  reserved           [0]=0xAB magic, rest 0x00
------  ----
Total = 84 bytes
```

`_Static_assert(sizeof(telemetry_frame_t) == 84)` is enforced at compile time in
`fleet_data.h`.

---

## MPU-6050 Sensor Details

| Setting | Value |
|---------|-------|
| I2C address | 0x68 (AD0 = GND) |
| Accelerometer range | ±2 g (default, register ACCEL_CONFIG = 0x00) |
| Gyroscope range | ±250 °/s (default, register GYRO_CONFIG = 0x00) |
| Accel sensitivity | 16 384 LSB/g |
| Gyro sensitivity | 131 LSB/°/s |
| Sample rate | 1 Hz (one `HAL_I2C_Mem_Read` per TIM2 tick) |
| Data register | 0x3B — 14 bytes burst (accel XYZ, temp, gyro XYZ) |
| Wake-up | Write 0x00 to register 0x6B (PWR_MGMT_1) |

Raw-to-SI conversion:

```
accel_m_s2 = (raw_int16 / 16384.0f) × 9.81f
gyro_deg_s  =  raw_int16 / 131.0f
```

---

## GPS Module Details

| Setting | Value |
|---------|-------|
| Default baud rate | 9 600 baud |
| Output sentences | $GPGGA, $GPRMC, $GPGSV, … |
| Sentences used | $GPGGA (lat, lon, alt, fix, sats) and $GPRMC (lat, lon, speed) |
| Cold start TTFF | ~30 s (clear sky), up to 12.5 min (first boot, no almanac) |
| Hot start TTFF | < 1 s |

The NEO-6M baud rate can be changed to 115 200 by sending a UBX-CFG-PRT message.
Leave it at 9 600 for simplicity unless you need higher update rates.

The `$GPRMC` status field must be `'A'` (active) for the position to be accepted.
Until a fix is acquired the GPS globals stay at 0.0 and `gps_fix = 0`.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `MPU-6050 wake FAILED` | Wrong wiring or missing pull-ups | Check SDA/SCL, add 4.7 kΩ pull-ups |
| `accel_z` reads ~0 instead of 9.81 | I2C reading zeros | Verify `AD0 = GND`; try `MPU6050_ADDR = 0x69` if `AD0 = VCC` |
| `GPS UART interrupt receive started` but lat/lon stay 0 | No GPS fix yet | Wait 30–60 s in open sky |
| GPS data looks wrong | TX/RX crossed | Swap PA10 and PA9 connections |
| BBG I2C read returns garbage | Ground loop | Connect STM32 GND to BBG GND |
| STM32 hardfault at startup | Stack overflow from large local arrays | Check `NMEA_MAX_LEN` and `GPS_BUF_SIZE` fit in RAM |

---

*Irena Zelinsky — Real-Time Embedded Systems*
