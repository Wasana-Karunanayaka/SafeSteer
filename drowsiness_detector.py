"""
SafeSteer - Drowsiness Detection Module
=======================================
Analyzes driver facial landmarks using MediaPipe to detect drowsiness 
via Eye Aspect Ratio (EAR). Communicates with the Arduino controller 
via Serial to trigger safety alerts.

@project SafeSteer
@author Wasana Karunanayaka
@date 13th July 2025
@version 1.6
"""

import cv2
import mediapipe as mp
import time
import math
import serial
import threading
import csv
import os
from datetime import datetime

# ==========================================
# INSTALLATION INSTRUCTIONS
# ==========================================
# Run the following on your Raspberry Pi:
# pip install opencv-python mediapipe myserial

# ==========================================
# CONFIGURATION
# ==========================================
SERIAL_PORT = '/dev/ttyUSB0'  # Default for Arduino on Pi
BAUD_RATE = 115200            # Matches SafeSteer.ino
EAR_THRESHOLD = 0.25          # Eye Aspect Ratio threshold for closed eyes
CLOSED_EYE_TIME_LIMIT = 2.0   # Seconds to trigger alert
HEARTBEAT_INTERVAL = 1.0      # Seconds between "No" heartbeats
LOG_FILE = 'drive_log.csv'

# ==========================================
# MEDIAPIPE SETUP
# ==========================================
mp_face_mesh = mp.solutions.face_mesh
# Landmark indices for Left and Right eyes
# (P1, P2, P3, P4, P5, P6) -> P1, P4 are corners; P2, P3 top; P5, P6 bottom
LEFT_EYE = [362, 385, 387, 263, 373, 380]
RIGHT_EYE = [33, 160, 158, 133, 153, 144]

# ==========================================
# GLOBAL STATE
# ==========================================
ser = None
running = True
last_heartbeat_time = 0

def setup_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to Serial: {SERIAL_PORT}")
        return True
    except serial.SerialException as e:
        print(f"Serial Error: {e}")
        # Try a fallback port if on Windows for testing
        if os.name == 'nt':
            try:
                # Replace 'COM3' with your actual Windows COM port if testing locally
                ser = serial.Serial('COM3', BAUD_RATE, timeout=1) 
                print("Connected to Fallback Serial: COM3")
                return True
            except:
                pass
        return False

def calculate_ear(landmarks, indices, img_w, img_h):
    """
    Calculates the Eye Aspect Ratio (EAR) to estimate eye openness.
    
    Formula: EAR = (|p2-p6| + |p3-p5|) / (2 * |p1-p4|)
    Where p1..p6 are specific 2D landmark points around the eye.
    
    :param landmarks: List of MediaPipe landmarks
    :param indices: List of 6 landmark indices for the eye
    :return: Calculated EAR float where < 0.25 typically means closed
    """
    # Retrieve coordinates
    coords = []
    for idx in indices:
        lm = landmarks[idx]
        coords.append((int(lm.x * img_w), int(lm.y * img_h)))

    # Vertical distances
    d_A = math.dist(coords[1], coords[5])
    d_B = math.dist(coords[2], coords[4])
    # Horizontal distance
    d_C = math.dist(coords[0], coords[3])

    # Eye Aspect Ratio
    ear = (d_A + d_B) / (2.0 * d_C)
    return ear

def log_data(hr, alert_level):
    file_exists = os.path.isfile(LOG_FILE)
    try:
        with open(LOG_FILE, mode='a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(['Timestamp', 'HeartRate', 'AlertLevel'])
            writer.writerow([datetime.now().strftime('%Y-%m-%d %H:%M:%S'), hr, alert_level])
    except Exception as e:
        print(f"Logging Error: {e}")

def serial_thread_func():
    global ser, last_heartbeat_time
    
    while running:
        current_time = time.time()
        
        if ser and ser.is_open:
            # 1. Send Heartbeat
            if current_time - last_heartbeat_time >= HEARTBEAT_INTERVAL:
                try:
                    ser.write(b"No\n")
                    last_heartbeat_time = current_time
                except Exception as e:
                    print(f"Send Error: {e}")
                    # Attempt reconnect logic could go here

            # 2. Read Incoming Data
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("SENSOR_DATA:"):
                        # SENSOR_DATA:HR=75,SPO2=0,ACCEL_X=0,...
                        print(f"Received: {line}")
                        parts = line.split(',')
                        hr = 0
                        alert = 0
                        for part in parts:
                            if "HR=" in part:
                                hr = part.split('=')[1]
                            elif "ALERT_LEVEL=" in part:
                                alert = part.split('=')[1]
                        
                        log_data(hr, alert)
            except Exception as e:
                print(f"Read Error: {e}")
        
        else:
            # Try to reconnect occasionally
            if current_time - last_heartbeat_time >= 5.0:
                print("Attempting to reconnect serial...")
                setup_serial()
                last_heartbeat_time = current_time

        time.sleep(0.01) # Small sleep to prevent CPU hogging

def main():
    global running, ser
    
    # Init Serial
    setup_serial()
    
    # Start Serial Thread
    t_serial = threading.Thread(target=serial_thread_func)
    t_serial.daemon = True
    t_serial.start()

    # Init Camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        return

    # drowsiness state
    eyes_closed_start_time = None
    alert_triggered = False

    with mp_face_mesh.FaceMesh(
        max_num_faces=1,
        refine_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    ) as face_mesh:
        
        print("Starting Video Loop...")
        try:
            while cap.isOpened():
                success, image = cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    continue

                # Performance optimization
                image.flags.writeable = False
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                results = face_mesh.process(image)

                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                
                img_h, img_w, _ = image.shape

                if results.multi_face_landmarks:
                    for face_landmarks in results.multi_face_landmarks:
                        landmarks = face_landmarks.landmark
                        
                        # Calculate EAR
                        left_ear = calculate_ear(landmarks, LEFT_EYE, img_w, img_h)
                        right_ear = calculate_ear(landmarks, RIGHT_EYE, img_w, img_h)
                        avg_ear = (left_ear + right_ear) / 2.0

                        # Display EAR
                        cv2.putText(image, f'EAR: {avg_ear:.2f}', (30, 30), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                        # Check Threshold
                        if avg_ear < EAR_THRESHOLD:
                            if eyes_closed_start_time is None:
                                eyes_closed_start_time = time.time()
                            
                            duration = time.time() - eyes_closed_start_time
                            cv2.putText(image, f'Closed: {duration:.1f}s', (30, 70), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                            
                            if duration > CLOSED_EYE_TIME_LIMIT:
                                cv2.putText(image, "DROWSINESS DETECTED!", (100, img_h // 2), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
                                
                                # Send Alert once per closure event or continuously?
                                # Requirement says "If eyes are closed... trigger"
                                # We'll send it continuously while condition is met
                                if ser and ser.is_open:
                                    try:
                                        ser.write(b"Level 01\n")
                                        print("Sent: Level 01")
                                    except:
                                        pass
                        else:
                            eyes_closed_start_time = None
                            alert_triggered = False

                cv2.imshow('SafeSteer Drowsiness Detector', image)
                
                if cv2.waitKey(5) & 0xFF == 27:
                    break
        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            running = False
            if ser and ser.is_open:
                ser.close()
            cap.release()
            cv2.destroyAllWindows()
            t_serial.join()
            print("Clean exit.")

if __name__ == "__main__":
    main()
