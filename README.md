# ESP32 Stress-Monitoring Wearable (Seeed XIAO ESP32S3)

This project is a sophisticated wearable system designed to monitor human stress levels in real-time. Built on the Seeed XIAO ESP32S3, it integrates multi-modal physiological sensors with an on-device Machine Learning (ML) model to classify stress states locally.

## Key Features

* **On-Device Classification**: Implements a Self-Organizing Map (SOM) model to classify user states into **STRESS**, **REST**, or **NEUTRAL** directly on the MCU.
* **Multi-Modal Sensing**:
    * **PPG (MAX30101)**: High-precision optical blood volume pulse sensing for heart rate and HRV.
    * **GSR (Galvanic Skin Response)**: Measures electrodermal activity (EDA) via a custom SPI interface.
    * **IMU (BMI260)**: 6-axis motion tracking for artifact rejection and movement intensity scoring.
    * **Temp (TMP117)**: High-accuracy skin temperature monitoring.
* **Advanced Signal Processing**: Real-time artifact detection (motion-masking), baseline tracking (DC-offset removal), and signal conditioning for peak detection.
* **BLE Data Pipeline**: Fragments large physiological datasets into multiple BLE characteristics to fit within MTU limits for transmission to a companion application.

## Hardware Architecture

The system is built on a modular driver structure interacting with various communication buses:
* **I2C Bus**: Interfaces with the MAX30101 (PPG), BMI260 (IMU), and TMP117 (Temp).
* **SPI Bus**: High-speed communication with the GSR sensor.
* **ADC**: Monitors battery voltage via a voltage divider to calculate remaining charge percentage.

## Extracted Features

The ML model relies on the following derived metrics:

| Feature | Data Type | Description | Unit / Metric |
| :--- | :--- | :--- | :--- |
| **HR** | `float` | **Heart Rate:** Calculated by detecting systolic peaks in the filtered PPG signal. | Beats Per Minute (BPM) |
| **HRV_RMSSD** | `float` | **Root Mean Square of Successive Differences:** The primary metric for Heart Rate Variability. | Milliseconds (ms) |
| **SCR_COUNT** | `float` | **Skin Conductance Response:** Tracks rapid, stimulus-induced changes in skin conductance. | Event Count |
| **EDA_TONIC** | `float` | **Tonic EDA:** Represents the long-term baseline level of skin conductance. | Microsiemens ($\mu S$) |
| **EDA_PHASIC** | `float` | **Phasic EDA:** Represents the fast-changing component of EDA. | Microsiemens ($\mu S$) |

## Software Pipeline

### 1. Sensor Sampling & Processing
The system uses **FreeRTOS** to manage high-frequency tasks:
* **Sampling Task**: Runs at 200Hz, collecting PPG, GSR, and IMU data.
* **Motion Masking**: Includes a mechanism that masks sensor artifacts if movement intensity exceeds a set threshold.
* **PPG Conditioning**: Removes DC baseline offsets and applies a jitter filter ($\alpha=0.30$) before signal inversion.

### 2. Data Storage & BLE
* **PSRAM Ring Buffer**: Stores raw sensor data in high-speed PSRAM for windowed processing.
* **SPIFFS**: Logs training data locally for transfer learning and debugging.
* **GATT Table**: Defines custom 128-bit UUID services to push bulk data to a connected client.
