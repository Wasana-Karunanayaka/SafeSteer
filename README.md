# SafeSteer: Smart Anti-Drowsiness Steering Wheel

SafeSteer is a hybrid safety system designed to prevent accidents caused by driver drowsiness and sudden health issues (e.g., heart attacks or seizures). It combines **grip sensors** on a steering wheel with **computer vision** for robust driver monitoring.

## Overview

The system operates on two parallel layers of protection:
1.  **Grip Detection (Arduino):** Monitors the driver's hold on the wheel using touch sensors. Loss of grip triggers a "Dead Man's Switch" fail-safe.
2.  **Vision Monitoring (Raspberry Pi):** Analyzes facial landmarks to detect eye closure (drowsiness) and signals the Arduino to alert the driver.

If potential danger is detected, the system provides multi-sensory feedback (haptic vibration, buzzer alarms, visual LEDs) and can actively slow down the simulated vehicle motor.

## Alert Mechanism

The system employs a graduated response system to handle drowsiness or lack of control:

-   **Level 1 (Warning):**
    -   **Trigger:** First signs of drowsiness (eyes closed > 2s) or fail-safe trigger.
    -   **Action:** Mild vibration (Haptic Feedback) to alert the driver without startling them.
    
-   **Level 2 (Critical):**
    -   **Trigger:** Persistent drowsiness or lack of response to Level 1.
    -   **Action:** Stronger vibration + Intermittent Siren (Audible Alarm).

-   **Level 3 (Emergency):**
    -   **Trigger:** Continuous non-response.
    -   **Action:** Maximum vibration + Continuous Siren + **Automatic Motor Deceleration** to bring the vehicle to a safe stop.

## Hardware Architecture

-   **Microcontroller:** Arduino Uno (R3)
-   **Processor:** Raspberry Pi 4 Model B (4GB)
-   **Sensors:**
    -   MAX30105 (Heart Rate & SpO2)
    -   ADXL345 (Accelerometer for jerk/crash detection)
    -   Capacitive Touch Sensors (Grip detection)
-   **Actuators:**
    -   Vibration Motors (Haptic feedback)
    -   Active Buzzer (Audible alarm)
    -   WS2812B LED Strip (Visual indicators)
    -   DC Motor (Simulated vehicle engine)
-   **Communication:** Arduino connects to Raspberry Pi via USB Serial (115200 baud).

## Software Prerequisites

### Arduino
-   **IDE:** Arduino IDE or VS Code with Arduino Extension.
-   **Libraries:**
    -   `FastLED`
    -   `LiquidCrystal_I2C`
    -   `SparkFun_ADXL345`
    -   `SparkFun_MAX3010x`

### Raspberry Pi (Python)
-   Python 3.7+
-   **Libraries:**
    ```bash
    pip install opencv-python mediapipe pyserial
    ```

## Setup Guide

### 1. Arduino Setup
1.  Open `SafeSteer.ino`.
2.  Install the required libraries via the Arduino Library Manager.
3.  Select **Arduino Uno** as the board and the correct COM port.
4.  Upload the sketch.

### 2. Raspberry Pi Setup
1.  Connect the Arduino to the Pi via USB.
2.  Connect a USB Webcam or Pi Camera.
3.  Install dependencies:
    ```bash
    pip install opencv-python mediapipe pyserial
    ```
4.  Run the detection script:
    ```bash
    python3 drowsiness_detector.py
    ```

## Fail-Safe Mechanism

SafeSteer includes a robust "Dead Man's Switch" designed for high reliability:

-   **Heartbeat Monitoring:** The Arduino continuously listens for a "No" signal from the Raspberry Pi every second.
-   **Fail-Safe Mode:** If the Pi crashes or the camera disconnects (timeout > 5s), the Arduino enters **Fail-Safe Mode**.
-   **Behavior:** In this mode, it relies strictly on the touch sensors. If the driver releases the wheel (fewer than 2 sensors active) for more than 3 seconds, the system assumes incapacitation and autonomously triggers the emergency stop sequence (Siren + Vibration + Motor Power Cut).

---

**Author:** Wasana Karunanayaka  
**Version:** 1.6  
**License:** MIT
