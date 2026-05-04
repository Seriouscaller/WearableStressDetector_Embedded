# ESP32 Stress-Monitoring Wearable

**Last Updated:** May 4, 2026  
**Primary Hardware:** Seeed XIAO ESP32S3

---

## Table of Contents
1. [Project Overview](#project-overview)
2. [System Architecture](#system-architecture)
3. [Directory Structure & File Mapping](#directory-structure--file-mapping)
4. [FreeRTOS Task Orchestration](#freertos-task-orchestration)
5. [Data Flow Pipeline](#data-flow-pipeline)
6. [Configuration & Control](#configuration--control)
7. [Key Dependencies & Libraries](#key-dependencies--libraries)
8. [Development Setup](#development-setup)
9. [Debugging & Troubleshooting](#debugging--troubleshooting)
10. [Critical Parameters & Thresholds](#critical-parameters--thresholds)

---

## Project Overview

This is a **real-time physiological stress monitoring system** that combines multi-modal sensor fusion with on-device ML classification. The system:

- **Acquires** 4 physiological signals at high frequency (200Hz PPG, variable-rate GSR/IMU)
- **Processes** signals in real-time with artifact detection and feature extraction
- **Classifies** stress state (STRESS / REST / NEUTRAL) using a Self-Organizing Map (SOM) model
- **Transmits** fragmented BLE payloads to companion applications
- **Stores** training data to SPIFFS for transfer learning

**Target Users:** Wearable developers, ML researchers, embedded systems engineers

---

## System Architecture

### Hardware Communication Buses

```
┌─────────────────────────────────────────────────┐
│        Seeed XIAO ESP32S3 (Main MCU)            │
└─────────────────────────────────────────────────┘
         │              │              │
    ┌────┴───┐      ┌───┴────┐    ┌───┴────┐
    │ I2C    │      │ SPI    │    │ ADC    │
    │ Bus    │      │ Bus    │    │        │
    └────┬───┘      └───┬────┘    └───┬────┘
         │              │             │
    ┌────┴────────┬─────┴────┐   ┌───┴──────┐
    │             │          │   │          │
┌───┴──┐  ┌──────┴──┐  ┌────┴─┐ │Battery  │
│MAX30 │  │ BMI260  │  │ GSR  │ │Voltage  │
│(PPG) │  │ (IMU)   │  │(EDA) │ │Monitor  │
└──────┘  └─────────┘  └──────┘ └─────────┘
    │         │           │
    └─────────┴───────────┘
    Heart Rate, HRV,
    Motion Data, EDA Metrics
```

### Communication Protocols

| Bus    | Device        | Function               | Speed    | Notes                    |
|--------|---------------|------------------------|----------|--------------------------|
| **I2C**| MAX30101      | PPG (optical HR/HRV)   | 400 kHz  | Heart rate sensing       |
| **I2C**| BMI260        | 6-axis IMU            | 400 kHz  | Motion & artifact detect |
| **I2C**| TMP117        | Temperature           | 400 kHz  | Skin temp monitoring     |
| **SPI**| GSR Custom    | Electrodermal activity | 10 MHz   | EDA/Skin conductance    |
| **ADC**| Battery Div   | Voltage monitoring     | 1-shot   | Battery % calculation    |
| **BLE**| NimBLE Stack  | Data transmission      | 2.4 GHz  | Wireless to companion    |

---

## Directory Structure & File Mapping

```
Stress_Det/
├── src/
│   └── main.c                    # Entry point: Hardware init, task creation
│
├── lib/
│   ├── board_config/
│   │   └── board_config.c        # GPIO pin definitions, device_config struct
│   │
│   ├── i2c_common/
│   │   ├── i2c_common.h          # I2C abstraction layer
│   │   ├── i2c_common.c          # I2C read/write helpers
│   │   └── scanner.c             # I2C device discovery tool
│   │
│   ├── spi_common/
│   │   ├── spi_common.h          # SPI initialization
│   │   └── spi_common.c          # SPI bus setup
│   │
│   ├── max/
│   │   ├── max30101.h            # PPG sensor interface
│   │   └── max30101.c            # PPG initialization & raw data read
│   │
│   ├── bmi/
│   │   ├── bmi260.h              # IMU sensor interface
│   │   ├── bmi260.c              # IMU initialization & data read
│   │   └── bmi260_config_file.h  # 8KB BMI260 config blob
│   │
│   ├── tmp/
│   │   ├── tmp117.h              # Temperature sensor interface
│   │   └── tmp117.c              # Temp sensor init & read
│   │
│   ├── gsr/
│   │   ├── gsr.h                 # GSR/EDA sensor interface
│   │   └── gsr.c                 # SPI-based GSR read logic
│   │
│   ├── adc/
│   │   ├── adc.h                 # Battery voltage monitoring
│   │   └── adc.c                 # ADC initialization & reading
│   │
│   ├── ppg/
│   │   ├── ppg_processing.h      # PPG signal conditioning interface
│   │   ├── ppg_processing.c      # DC removal, filtering, peak detection
│   │   └── ppg_hrv.h             # HRV calculation (RMSSD)
│   │
│   ├── eda/
│   │   ├── eda_processing.h      # EDA signal decomposition interface
│   │   └── eda_processing.c      # Tonic/Phasic EDA extraction
│   │
│   ├── signal_processing/
│   │   ├── signal_processing.h   # Feature extraction entry point
│   │   └── signal_processing.c   # Combines PPG, EDA, IMU → feature vector
│   │
│   ├── som/
│   │   ├── som.h                 # Self-Organizing Map classifier
│   │   └── som.c                 # SOM inference (STRESS/REST/NEUTRAL)
│   │
│   ├── tasks/
│   │   ├── tasks.h               # FreeRTOS task declarations
│   │   └── tasks.c               # All 7 FreeRTOS tasks (see below)
│   │
│   ├── bluetooth/
│   │   ├── ble_server.h          # BLE initialization & GAP/GATT setup
│   │   ├── ble_server.c          # NimBLE stack management
│   │   ├── ble_commands.h        # BLE command handler interface
│   │   ├── ble_commands.c        # Device control via BLE
│   │   ├── gatt.h                # GATT service & characteristic defs
│   │   └── gatt.c                # Custom 128-bit UUID definitions
│   │
│   ├── storage/
│   │   ├── storage.h             # Storage initialization interface
│   │   └── storage.c             # PSRAM ring buffer & SPIFFS setup
│   │
│   ├── types/
│   │   └── types.h               # All data structures (see section below)
│   │
│   └── esp_dsp_dotprod/
│       └── [DSP utility functions]
│
├── sdkconfig.defaults            # ESP-IDF configuration (BT, SPIRAM, etc.)
├── CMakeLists.txt
└── platformio.ini

```

### Core Type Definitions (types.h)

All data structures are tightly packed for memory efficiency:

```c
// Raw sensor sample: 18 bytes
typedef struct __attribute__((packed)) {
    int64_t time_stamp;
    uint32_t ppg_raw;
    float ppg_filtered;
    uint16_t gsr;
} raw_log_data_t;

// Single sensor snapshot: 50+ bytes (includes IMU 12B, motion flag)
typedef struct __attribute__((packed)) {
    int64_t time_stamp;
    uint32_t ppg_raw;
    float ppg_filtered;
    uint16_t gsr;
    bmi_data_t bmi_data;          // 12B (ax,ay,az,gx,gy,gz)
    bool has_movement_artifact;
} raw_data_t;

// Extracted features: 20 bytes
typedef struct __attribute__((packed)) {
    float hr;                      // Heart Rate (BPM)
    float hrv_rmssd;               // Heart Rate Variability (ms)
    float scr;                     // Skin Conductance Response (count)
    float tonic;                   // Tonic EDA (µS)
    float phasic;                  // Phasic EDA (µS)
} som_input_t;

// Complete logged window: 2830 bytes
typedef struct __attribute__((packed)) {
    uint32_t timestamp;
    raw_log_data_t raw_samples[200];  // 200 samples @ 200Hz = 1 sec
    som_input_t features;
    uint8_t stress_class;             // 0=STRESS, 1=REST, 2=NEUTRAL
    uint8_t experiment_phase;
} complete_log_t;

// BLE fragmented payloads
typedef struct __attribute__((packed)) {
    uint32_t timestamp;
    raw_log_data_t raw_samples[24];   // 424 bytes
} ble_payload_bulk_t;

typedef struct __attribute__((packed)) {
    uint32_t timestamp;
    raw_log_data_t raw_samples[8];
    float hr, rmssd, scr, tonic, phasic;
    uint8_t stress_class;
    uint8_t experiment_phase;
} ble_payload_final_t;                // 450 bytes total
```

---

## FreeRTOS Task Orchestration

The system runs **7 parallel FreeRTOS tasks** (defined in `lib/tasks/tasks.c`). Each task has a specific role and inter-task communication method:

### Task Dependency Graph

```
                    ┌──────────────────────────────┐
                    │    MAIN (app_main)           │
                    │  Creates all tasks & queues  │
                    └──────────────────────────────┘
                                 │
       ┌─────────────┬───────────┼───────────┬─────────────┐
       │             │           │           │             │
       ▼             ▼           ▼           ▼             ▼
┌────────────┐ ┌──────────┐ ┌────────┐ ┌─────────┐ ┌──────────┐
│ sensor_    │ │ imu_     │ │temp_   │ │battery_ │ │telemetry_│
│sampling_   │ │sampling_ │ │task    │ │task     │ │task      │
│task        │ │task      │ │        │ │         │ │          │
│ (200 Hz)   │ │ (100 Hz) │ │ (1 Hz) │ │ (1 Hz)  │ │ (200 Hz) │
└──────┬─────┘ └────┬─────┘ └────────┘ └─────────┘ └────┬─────┘
       │            │                                   │
       │ writes     │ writes                            │ reads
       │ PPG + GSR  │ IMU data ────┐                    │
       │            │              │                    │
       └────┬───────┴─────┬────────┘                    │
            │             │                              │
            ▼             ▼                              │
       ┌──────────────────────────────┐                 │
       │ PSRAM Ring Buffer             │                 │
       │ (raw_data_ringbuf)            │◄────────────────┘
       │ Holds last 200 PPG samples    │
       └──────────────────────────────┘
            │
            ▼
       ┌──────────────────────────────────────┐
       │ feature_extraction_task               │
       │ • Computes HR from PPG peaks         │
       │ • Calculates HRV_RMSSD              │
       │ • Decomposes EDA (tonic/phasic)     │
       │ • Runs SOM classifier               │
       └──────────────────────┬───────────────┘
                              │
                        writes complete_log
                              │
                    ┌─────────┴─────────┐
                    ▼                   ▼
            ┌────────────────┐  ┌───────────────┐
            │ logging_task   │  │ ble_update_   │
            │ • Stores to    │  │ task          │
            │   SPIFFS       │  │ • Fragments   │
            │ • Ring buffer  │  │   payloads    │
            │   control      │  │ • Sends via   │
            │                │  │   NimBLE      │
            └────────────────┘  └───────────────┘
                                      │
                                      ▼
                              ┌──────────────────┐
                              │ BLE Client       │
                              │ (Companion App)  │
                              └──────────────────┘
```

### Task Details

| Task Name | Priority | Frequency | Input | Output | Purpose |
|-----------|----------|-----------|-------|--------|---------|
| **sensor_sampling_task** | HIGH | 200 Hz | MAX30101, GSR | `raw_data_ringbuf` | Acquire PPG @ 200Hz, GSR async reads |
| **imu_sampling_task** | HIGH | ~100 Hz | BMI260 | `raw_data_ringbuf` | 6-axis motion data (accel + gyro) |
| **feature_extraction_task** | MEDIUM | 200 Hz | `raw_data_ringbuf` | `data_log_queue` | HR/HRV/EDA/SOM classification |
| **logging_task** | LOW | Event-driven | `data_log_queue` | SPIFFS | Training data persistence |
| **ble_update_task** | MEDIUM | Event-driven | `data_log_queue` | BLE GATT | Fragment & transmit over BLE |
| **telemetry_task** | LOW | 200 Hz | `raw_data_ringbuf` | UART/Debug | Raw sample streaming for testing |
| **battery_task** | LOW | 1 Hz | ADC | UART/Log | Battery voltage & % estimation |
| **temperature_task** | LOW | 1 Hz | TMP117 | UART/Log | Skin temperature logging |

### Inter-Task Communication Mechanisms

1. **Ring Buffer** (`raw_data_ringbuf`)
   - **Type:** FreeRTOS Ring Buffer (PSRAM-backed)
   - **Size:** ~5 KB (holds ~100 raw samples)
   - **Writers:** `sensor_sampling_task`, `imu_sampling_task`
   - **Readers:** `feature_extraction_task`, `telemetry_task`
   - **Purpose:** High-frequency data buffering without blocking

2. **Message Queue** (`data_log_queue`)
   - **Type:** FreeRTOS Queue
   - **Size:** 5 complete_log_t pointers
   - **Writers:** `feature_extraction_task`
   - **Readers:** `logging_task`, `ble_update_task`
   - **Purpose:** Complete feature vectors + ML results

3. **Telemetry Queue** (`telemetry_queue`)
   - **Type:** FreeRTOS Queue
   - **Size:** 20 raw_data_t items
   - **Writers:** `telemetry_task`
   - **Readers:** Debug UART output
   - **Purpose:** Real-time debug streaming

4. **Mutexes**
   - **ble_payload_mutex:** Protects fragmented BLE payload buffer during assembly/transmission
   - **experiment_phase_mutex:** Protects shared experiment phase state (for test control)

### Critical Timing Constraints

```c
// Sampling: 200 Hz = 5 ms between acquisitions
#define SAMPLING_INTERVAL_MS  5

// Feature extraction: 1-second window (200 samples @ 200 Hz)
#define FEATURE_WINDOW_SIZE   200

// Ring buffer capacity: 5 seconds of history
#define RINGBUF_SIZE          (200 * 5 * sizeof(raw_data_t))

// BLE MTU: 244 bytes (limits payload size)
#define BLE_NUM_OF_SAMPLES_PER_PAYLOAD  24  // 18B * 24 = 432B < 512B
#define BLE_NUM_OF_BULK_PAYLOADS        8   // 8 payloads + 1 final = 9 characteristics
```

---

## Data Flow Pipeline

### Complete Processing Chain (Per 1-Second Window)

```
Timeline: 0 ms → 1000 ms (at 200 Hz)

0 ms      ┌─────────────────────────────────────────────┐
│         │ Sampling Phase (Continuous)                 │
│         │ • PPG raw sample @ 200 Hz (every 5ms)      │
│         │ • GSR read (async, variable rate)           │
│         │ • IMU @ ~100 Hz (every 10ms)                │
│         │ → All written to raw_data_ringbuf          │
│         └─────────────────────────────────────────────┘
│                        │
│                        ▼
│         ┌─────────────────────────────────────────────┐
1000 ms   │ Feature Extraction (at 1000 ms)             │
│         │ 1. Read last 200 samples from ring buffer   │
│         │ 2. PPG Processing:                          │
│         │    - DC removal (baseline tracking)         │
│         │    - Jitter filter (α=0.30)                 │
│         │    - Signal inversion                       │
│         │ 3. Peak Detection → Calculate HR            │
│         │ 4. RR-interval analysis → HRV_RMSSD        │
│         │ 5. EDA Decomposition (Tonic/Phasic)        │
│         │ 6. Motion artifact detection                │
│         │ → Assemble som_input_t struct (5 features)  │
│         └─────────────────────────────────────────────┘
│                        │
│                        ▼
│         ┌─────────────────────────────────────────────┐
~1001ms   │ ML Classification                           │
│         │ • SOM inference with feature vector         │
│         │ • Output: stress_class {0,1,2}              │
│         │ → Assemble complete_log_t (2830B)          │
│         │ → Push to data_log_queue                    │
│         └─────────────────────────────────────────────┘
│                        │
│          ┌─────────────┴─────────────┐
│          ▼                           ▼
│   ┌─────────────────┐         ┌──────────────┐
│   │ Logging Task    │         │ BLE Update   │
│   │ • Buffer in RAM │         │ • Fragment   │
│   │ • When full:    │         │ • MTU check  │
│   │   Write to      │         │ • Notify     │
│   │   SPIFFS        │         │   client     │
│   │ • 28B aligned   │         └──────────────┘
│   └─────────────────┘
│
└──────────────────────────────────────────────→ Time
     (Repeat every 1 second)
```

### PPG Signal Conditioning Flow

```
Raw PPG (IR LED)
    │
    ▼
┌─────────────────────────────────┐
│ DC Baseline Removal             │
│ (Subtract running average)      │
└─────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────┐
│ Jitter Filter (α = 0.30)        │
│ smooth = α*new + (1-α)*smooth   │
└─────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────┐
│ Signal Inversion (flip polarity)│
└─────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────┐
│ Peak Detection (systolic peaks)  │
│ → RR-intervals (beat-to-beat)   │
└─────────────────────────────────┘
    │
    ├──────────────────┬──────────────┐
    ▼                  ▼              ▼
┌─────────────┐ ┌──────────────┐ ┌──────────┐
│ HR (BPM)    │ │ HRV_RMSSD    │ │ Quality  │
│ = 60/RR_avg │ │ = sqrt(mean  │ │ Metric   │
│             │ │   (RR_diff²))│ │          │
└─────────────┘ └──────────────┘ └──────────┘
```

### EDA Signal Decomposition

```
Raw GSR (skin conductance via SPI)
    │
    ▼
┌─────────────────────────────────────┐
│ Two-component Model                 │
│ EDA = Tonic + Phasic                │
└─────────────────────────────────────┘
    │
    ├─────────────────────┬────────────────────┐
    ▼                     ▼                    ▼
┌─────────────────┐ ┌────────────────┐ ┌─────────────┐
│ Tonic EDA (µS)  │ │ Phasic EDA (µS)│ │ SCR_COUNT   │
│ = Slow baseline │ │ = Fast changes │ │ = Event cnt │
│ (long-term)     │ │ (spikes)       │ │ (triggers)  │
└─────────────────┘ └────────────────┘ └─────────────┘
```

---

## Configuration & Control

### Device Configuration Structure

The `device_control_t` struct controls all runtime behavior:

```c
typedef struct {
    bool show_telemetry;          // Print raw samples to UART
    bool show_logged_values;      // Print extracted features
    bool show_battery_log;        // Battery voltage logs
    bool show_gsr_debugging;      // EDA decomposition debug
    bool show_spiff_status;       // SPIFFS usage stats
    bool enable_imu;              // Activate BMI260
    bool enable_ppg;              // Activate MAX30101
    bool enable_gsr;              // Activate GSR sensor
    bool enable_temp;             // Activate TMP117
} device_control_t;
```

**Location:** Defined in `board_config.c`, referenced globally as `device_config`

### Where Configuration is Set

1. **Compile-Time:** `board_config.c` static initializer
   ```c
   device_control_t device_config = {
       .show_telemetry = true,
       .enable_ppg = true,
       .enable_gsr = true,
       .enable_imu = true,
       .enable_temp = true,
   };
   ```

2. **Runtime:** BLE command handler
   - **File:** `lib/bluetooth/ble_commands.c`
   - **Function:** `control_write_cb()`
   - **Input:** BLE write to control characteristic
   - **Effect:** Updates `device_config` flags dynamically

### BLE Command Protocol

Companion app writes commands via:
- **Service UUID:** Custom 128-bit (see `gatt.c`)
- **Characteristic:** `ble_command_chr_val_handle`
- **Payload Format:** Binary encoded commands
- **Response:** Immediate effect on firmware behavior

**Example Commands:**
- Toggle telemetry streaming on/off
- Start/stop SPIFFS logging
- Change experiment phase marker
- Reset ring buffer

---

## Key Dependencies & Libraries

### ESP-IDF Components

| Component | Version | Purpose | Config |
|-----------|---------|---------|--------|
| **FreeRTOS** | Built-in | Real-time task scheduling | `sdkconfig` |
| **NimBLE** | Built-in | BLE stack (GAP/GATT) | `CONFIG_BT_NIMBLE_ENABLED=y` |
| **PSRAM** | Built-in | External SPI RAM | `CONFIG_SPIRAM=y` |
| **SPIFFS** | Built-in | SPI Flash File System | Partition table |
| **I2C Master** | Built-in | I2C bus driver | 400 kHz |
| **SPI Master** | Built-in | SPI bus driver | 10 MHz |
| **ADC** | Built-in | Analog-to-Digital | 1-shot mode |
| **ESP_LOG** | Built-in | Debug logging | Tag-based |

### Custom Libraries

| Module | Files | Responsibility |
|--------|-------|-----------------|
| **PPG** | `ppg_processing.{h,c}` + `ppg_hrv.h` | HR & HRV calculation |
| **EDA** | `eda_processing.{h,c}` | Tonic/Phasic decomposition |
| **SOM** | `som.{h,c}` | Stress classifier (hardcoded weights) |
| **Signal Proc** | `signal_processing.{h,c}` | Feature aggregation |
| **Storage** | `storage.{h,c}` | Ring buffer + SPIFFS init |
| **BLE Server** | `ble_server.{h,c}` + `gatt.{h,c}` | NimBLE management |

### Important SDK Configuration

```ini
# sdkconfig.defaults
CONFIG_BT_ENABLED=y                         # Enable Bluetooth
CONFIG_BT_NIMBLE_ENABLED=y                  # NimBLE stack
CONFIG_SPIRAM=y                             # External PSRAM
CONFIG_SPIRAM_TYPE_ESP32S3=y                # Octal SPI
CONFIG_SPIRAM_MODE_OCTAL=y
CONFIG_SPIRAM_BOOT_INIT=y
CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL=32768 # 32KB internal reserve
CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=16384   # Critical allocations
```

---

## Development Setup

### Prerequisites

1. **Hardware**
   - Seeed XIAO ESP32S3
   - MAX30101 PPG sensor (I2C)
   - BMI260 IMU (I2C)
   - TMP117 Temperature (I2C)
   - Custom GSR module (SPI)
   - USB-to-UART adapter (for serial logs)

2. **Software**
   ```bash
   # ESP-IDF installation (v5.x)
   git clone https://github.com/espressif/esp-idf.git
   cd esp-idf && git checkout v5.1
   ./install.sh esp32s3
   . ./export.sh

   # PlatformIO (alternative to idf.py)
   pip install platformio
   ```

3. **Python Dependencies**
   ```bash
   pip install esptool pyserial
   ```

### Build & Flash

**Option A: Using ESP-IDF**
```bash
cd Stress_Det
idf.py build
idf.py -p /dev/ttyUSB0 flash
idf.py -p /dev/ttyUSB0 monitor
```

**Option B: Using PlatformIO**
```bash
cd Stress_Det
pio run -t upload -p /dev/ttyUSB0
pio device monitor -p /dev/ttyUSB0
```

### UART Serial Output

Expected startup sequence:
```
I (0) boot: ESP-IDF v5.1 2nd stage bootloader
I (150) boot: Loaded app from partition at offset 0x10000
I (200) MAIN: All sensors initialized.
I (250) BLE_SRV: NimBLE Host Task started
I (300) BLE_SRV: Advertisement started
I (350) SENSOR: Sampling task started
I (400) FEATURE: Feature extraction task running
```

### Key Debug Macros

Enable in code or via serial monitor:
```c
#define DEBUG_PPG           1  // Peak detection output
#define DEBUG_EDA           1  // Tonic/Phasic values
#define DEBUG_SOM           1  // Classification confidence
#define DEBUG_BLE_FRAG      1  // BLE payload assembly
```

---

## Debugging & Troubleshooting

### Common Issues & Solutions

#### 1. Sensors Not Detected on I2C

**Symptom:** `I2C initialization OK but sensor read fails`

**Diagnosis:**
```bash
# Use I2C scanner tool
# Build: Stress_Det/lib/i2c_common/scanner.c
# Output: Detected addresses for each device
```

**Solutions:**
- Check pull-up resistors (4.7 kΩ on SDA/SCL)
- Verify I2C frequency in `board_config.c`: `I2C_MASTER_FREQ_HZ = 400000`
- Confirm sensor I2C addresses match (MAX30101=0x57, BMI260=0x68, TMP117=0x48)

#### 2. Ring Buffer Overflow

**Symptom:** `[WARNING] Ring buffer overflowing!`

**Root Cause:** Feature extraction task slower than sampling

**Solutions:**
- Increase ring buffer size (edit `RINGBUF_SIZE` in `storage.h`)
- Optimize feature calculation (reduce EDA decomposition iterations)
- Check task priority inversion (`feature_extraction_task` should be MEDIUM+)

#### 3. BLE Connection Drops

**Symptom:** Client disconnects randomly

**Root Cause:** MTU exceeded or callback blocking

**Solutions:**
- Verify payload fragmentation: `ble_payloads_bulk[i]` in `tasks.c` line ~295
- Check `ble_payload_mutex` is properly released after notify
- Monitor UART for BLE error codes from NimBLE

#### 4. SPIFFS Full / Write Failures

**Symptom:** `Failed to open file for appending`

**Root Cause:** Partition full or corrupted file system

**Solutions:**
```bash
# Erase SPIFFS partition
idf.py erase_flash  # Full erase
idf.py flash         # Re-flash firmware

# Or selective erase via NVS
```

- Monitor `check_spiffs_status()` output (line ~340 in tasks.c)
- Reduce logging frequency: `STORAGE_BUFFER_SIZE` in `types.h`

#### 5. Incorrect Heart Rate / HRV

**Symptom:** HR = 0 BPM or wildly fluctuating

**Root Cause:** PPG signal noise or poor contact

**Solutions:**
- Check PPG signal conditioning:
  - DC offset removal in `ppg_processing.c`
  - Jitter filter alpha (α=0.30): tune if needed
- Verify MAX30101 LED current (should be 50-100 mA)
- Check sensor contact: optical sensors are sensitive to placement
- View raw samples via telemetry: set `show_telemetry = true`

#### 6. SOM Classification Stuck at One State

**Symptom:** Always outputs STRESS or REST

**Root Cause:** Feature scaling issue or hardcoded weights mismatch

**Solutions:**
- Check feature ranges in `signal_processing.c`
- Verify SOM weight matrix loaded (see `som.c`)
- Retrain SOM model if dataset changed

### Serial Monitor Decoding

**PPG Debug Output:**
```
>HR:72 HRV:45  # Heart rate & variability
```

**EDA Debug Output:**
```
>EDA_T:15.4 EDA_P:2.1 SCR:3  # Tonic, Phasic, SCR count
```

**BLE Debug Output:**
```
>BLE_FRAG: Payload A (424B)
>BLE_FRAG: Payload B (424B)
>BLE_FRAG: Final payload (450B)
```

**SOM Debug Output:**
```
>SOM_CLASS: 1 (REST) conf:0.92
```

---

## Critical Parameters & Thresholds

### Sampling Configuration

```c
// PPG & GSR sampling
#define PPG_SAMPLE_RATE          200    // Hertz
#define GSR_SAMPLE_RATE          100    // Hertz (approximate)
#define FEATURE_EXTRACTION_FREQ  1      // Hz (every 1 second)

// Ring buffer
#define RAW_DATA_HISTORY_SECONDS 5      // Keep 5 seconds in RAM
#define RINGBUF_CAPACITY         (200 * 5 * sizeof(raw_data_t))
```

### Signal Processing

```c
// PPG jitter filter
#define PPG_JITTER_ALPHA         0.30   // Exponential smoothing

// Peak detection (from ppg_hrv.h)
#define MIN_PEAK_HEIGHT          100    // ADC counts (tunable)
#define MIN_PEAK_DISTANCE        40     // Samples @ 200Hz (~200ms min interval)

// HRV calculation window
#define RR_INTERVAL_SAMPLES      200    // 1 second @ 200Hz
#define MIN_HR                   30     // BPM lower bound
#define MAX_HR                   200    // BPM upper bound
```

### EDA Decomposition

```c
// From eda_processing.c
#define EDA_TONIC_WINDOW         3      // Seconds (low-pass cutoff)
#define EDA_PHASIC_THRESHOLD     0.05   // µS (noise floor)
#define SCR_PEAK_THRESHOLD       0.1    // µS (significant event)
```

### BLE Transmission

```c
// Payload sizing (from types.h)
#define BLE_NUM_OF_SAMPLES_PER_PAYLOAD  24   // 18B × 24 = 432B
#define BLE_NUM_OF_BULK_PAYLOADS        8    // 8 × 432B bulk payloads
#define BLE_FINAL_PAYLOAD_SAMPLES       8    // Last 8 samples + features

// MTU constraint
#define BLE_MTU_DEFAULT          244     // Effective payload: 242B
// Actual: 424B bulk < 512B BLE max, fragmented into characteristics
```

### SOM Model

```c
// From som.c (if accessible)
#define SOM_GRID_WIDTH           10     // Map dimensions
#define SOM_GRID_HEIGHT          10
#define SOM_NUM_CLASSES          3      // {0: STRESS, 1: REST, 2: NEUTRAL}
#define SOM_LEARNING_RATE        0.05   // (inference only, no online learning)
```

### SPIFFS Logging

```c
// From storage.c & tasks.c
#define STORAGE_BUFFER_SIZE      50     // Samples per write
#define LOG_FILE_PATH            "/spiffs/training_log.dat"
#define PARTITION_NAME           "storage"

// File format: array of som_input_transfer_learning_t (28B each)
// ~50 samples × 28B = 1.4 KB per write
```

### Motion Artifact Detection

```c
// From signal_processing.c or bmi/bmi260.c
#define MOTION_ARTIFACT_THRESHOLD   500   // Accel+Gyro magnitude
#define MOTION_MASKING_ENABLED      true  // Ignore PPG if moving
```

### Battery Monitoring

```c
// From adc.c
#define BATTERY_ADC_CHANNEL      ADC_CHANNEL_0
#define VOLTAGE_DIVIDER_RATIO    2.0     // (Upper / Lower resistor)
#define BATTERY_SAMPLE_PERIOD_MS 1000    // 1 Hz updates
```

---

## Contact & Support

For questions on specific modules:
- **Sensors/Hardware:** See device datasheets in hardware folder
- **FreeRTOS:** [ESP-IDF FreeRTOS Docs](https://docs.espressif.com/projects/esp-idf)
- **NimBLE:** [NimBLE Docs](https://mynewt.apache.org/latest/network/)
- **Signal Processing:** See inline comments in `ppg_processing.c`, `eda_processing.c`
- **ML Model:** Consult original SOM training scripts (separate repo)

---

**Document Version:** 1.0  
**Last Updated:** May 4, 2026  
