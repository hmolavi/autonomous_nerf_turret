#!/usr/bin/env python3
"""
Simple YOLO camera viewer for tennis-ball detection, integrated with stepper motor control.

This script opens a camera, runs the YOLO model to detect tennis balls,
and sends commands via a serial port to a connected Arduino to control
stepper motors for a turret, based on the detected ball's position.

Usage: python tennis_ball_demo.py

Using Apple silicon GPU:
    python3 tennis_ball_demo.py -d mps --half

Resize and skip frames to increase throughput (trade detection latency for speed):
    python3 tennis_ball_demo.py --imgsz 416 --skip 2

Both:
    python3 tennis_ball_demo.py --imgsz 416 --skip 2 -d mps --half
"""

import time
import argparse
import threading
import queue
import serial
import struct

import cv2
from ultralytics import YOLO


# --- YOLO and Turret Parameters ---
DEFAULT_MODEL = "runs/detect/tennis_v6_nano/weights/best.pt"

# Turret parameters
HORIZONTAL_FOV = 70.0  # degrees
VERTICAL_FOV = 52.0  # degrees
YAW_PULSES_PER_DEGREE = 50  # 1280.0 / 17.0  # X - Axis
PITCH_PULSES_PER_DEGREE = 10  # 5120.0 / 189.0  # Y - Axis
# Visual/aiming shift: move crosshair (and aiming reference) horizontally by this many pixels
# Negative -> shift left, Positive -> shift right
CROSSHAIR_SHIFT_X = -50

# --- Serial Port Configuration (from joystick_stepper.py) ---
SERIAL_PORT = "/dev/cu.usbmodem31201"
BAUD_RATE = 115200

# Command Type Definitions (Must match Arduino's main.cpp)
CMD_MOVE_X = 0x01
CMD_MOVE_Y = 0x02
CMD_SHOOT_GUN = 0x03
CMD_ENABLE_GUN = 0x04
CMD_DISABLE_GUN = 0x05

# --- Adjustable Control Variable ---
# Adjust this variable to change the speed of the stepper motors.
# A smaller value means commands are sent more frequently (faster).
# A larger value means commands are sent less frequently (slower).
step_send_interval = 0.2  # seconds


# --- Global Variables for Communication ---
# The queue is used to pass the latest yaw and pitch pulses to the
# serial communication thread.
pulse_queue = queue.Queue(maxsize=1)
ser = None
exit_flag = False
# Gun control state
gun_enabled = False  # whether we have sent ENABLE to the Arduino
gun_forced_off = False  # safety lock controlled by spacebar; when True gun must remain disabled
last_shot_time = 0.0
SHOOT_COOLDOWN = 2  # seconds between automatic shoot commands to avoid flooding

# Detection debounce: require sustained detection before enabling the gun
DETECTION_ENABLE_THRESHOLD = 0.5  # seconds of continuous detection required to enable gun
detection_start_time = None
# Wind-up requirement: require gun to be enabled for this many seconds before shooting
GUN_WINDUP = 5  # seconds required after enable before first shot
gun_enabled_time = None


# --- Function to send a command via serial ---
def send_command(command_type, value=0):
    """
    Sends a serial command to the Arduino.
    :param command_type: The command byte (e.g., CMD_MOVE_X).
    :param value: The 16-bit signed integer value to send (e.g., step count).
    """
    global ser
    if ser is None or not ser.is_open:
        # print("Serial port not open. Cannot send command.")
        return

    packet = bytes([0xFF, command_type])  # Start byte + Command Type

    # Add data bytes if the command type requires a step count
    if command_type in [CMD_MOVE_X, CMD_MOVE_Y]:
        # Pack the 16-bit signed integer value into two bytes (big-endian)
        # '>h' means big-endian ('<h' for little-endian) short (2 bytes, signed)
        packet += struct.pack(">h", int(value))

    packet += bytes([0xFE])  # End byte

    try:
        ser.write(packet)
        # print(f"Sent: {packet.hex()}")  # Uncomment for debugging
    except serial.SerialException as e:
        print(f"Error sending data: {e}")


# --- Thread to handle serial communication ---
class TurretControlThread(threading.Thread):
    def __init__(self, pulse_q, interval, exit_f):
        super().__init__()
        self.pulse_queue = pulse_q
        self.interval = interval
        self.exit_flag = exit_f
        self.last_sent_time = 0

    def run(self):
        global ser
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            time.sleep(2)  # Wait for serial connection to establish

            while not self.exit_flag:
                # Check for new pulses from the main thread
                try:
                    yaw_pulses, pitch_pulses = self.pulse_queue.get(block=False)

                    now = time.time()
                    if now - self.last_sent_time >= self.interval:
                        # Send commands only if there are significant movements
                        if abs(yaw_pulses) > 1:
                            send_command(CMD_MOVE_X, yaw_pulses)
                        if abs(pitch_pulses) > 1:
                            send_command(CMD_MOVE_Y, pitch_pulses)

                        self.last_sent_time = now

                except queue.Empty:
                    # No new data, just sleep for a short while
                    pass

                time.sleep(0.01)  # Small delay to prevent busy-waiting

        except serial.SerialException:
            # Suppress serial open errors; send_command will report write failures.
            pass
        except Exception:
            # Suppress other control thread errors to minimize stdout noise.
            pass
        finally:
            if ser and ser.is_open:
                ser.close()


# --- Main Application Logic ---
def parse_args():
    p = argparse.ArgumentParser(description="YOLO camera viewer (rotated 180°)")
    p.add_argument("--camera", "-c", type=int, default=0, help="camera index (default 0)")
    p.add_argument("--model", "-m", default=DEFAULT_MODEL, help="path to YOLO weights")
    p.add_argument("--conf", type=float, default=0.35, help="confidence threshold")
    p.add_argument("--no-rotate", dest="rotate", action="store_false", help="disable 180° rotation")
    p.add_argument("--device", "-d", default=None, help="device for inference (e.g. 'cpu','cuda','mps'); default auto")
    p.add_argument("--imgsz", type=int, default=416, help="resize shorter side to this for faster inference (default 640)")
    p.add_argument("--half", action="store_true", help="use half precision if supported (faster on GPU)")
    p.add_argument("--skip", type=int, default=0, help="skip N frames between inferences (0 = no skip)")
    p.add_argument("--force-1080p", action="store_true", help="try to set capture to 1920x1080 and report result")
    return p.parse_args()


def main():
    # Mutate module-level state from this function
    global exit_flag, gun_enabled, gun_forced_off, last_shot_time, detection_start_time, gun_enabled_time, last_detection_time, shot_count, shot_visual_time
    args = parse_args()

    # Start the serial communication thread
    turret_thread = TurretControlThread(pulse_queue, step_send_interval, exit_flag)
    turret_thread.daemon = True
    turret_thread.start()

    # Load a model
    model = YOLO(args.model)

    # Open camera
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        # Camera couldn't be opened; exit silently
        return

    # Get camera resolution
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Calculate degrees per pixel
    horizontal_degrees_per_pixel = HORIZONTAL_FOV / width
    vertical_degrees_per_pixel = VERTICAL_FOV / height

    # The queue is used to process frames in a separate thread
    frame_queue = queue.Queue(maxsize=1)
    results_queue = queue.Queue(maxsize=1)

    def process_frames():
        while True:
            frame = frame_queue.get()
            if frame is None:
                break
            results = model(
                frame,
                stream=False,
                device=args.device,
                imgsz=args.imgsz,
                conf=args.conf,
                verbose=False,
            )
            try:
                results_queue.put(results, block=False)
            except queue.Full:
                pass

    process_thread = threading.Thread(target=process_frames)
    process_thread.daemon = True
    process_thread.start()

    # Main loop
    prev_time = time.time()
    fps = 0.0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Rotate frame 180 degrees if requested
            if args.rotate:
                try:
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
                except Exception:
                    # fallback: flip both axes
                    frame = cv2.flip(frame, -1)

            # Send frame to the processing thread
            try:
                frame_queue.put(frame, block=False)
            except queue.Full:
                pass

            # Get results from the processing thread
            try:
                results = results_queue.get(block=False)
                display = results[0].orig_img.copy()

                # Draw center crosshair lines (always visible on the display)
                center_x = int(width / 2) + CROSSHAIR_SHIFT_X
                center_y = int(height / 2)
                cv2.line(display, (center_x, 0), (center_x, height), (255, 0, 0), 1)
                cv2.line(display, (0, center_y), (width, center_y), (255, 0, 0), 1)

                for r in results:
                    boxes = r.boxes.xyxy
                    clss = r.boxes.cls
                    confs = r.boxes.conf
                    names = r.names

                    # Track whether any object was detected in this results set
                    detection_present = False

                    if len(boxes) > 0:
                        # Focus on the first detected object (e.g., a tennis ball)
                        box = boxes[0]
                        conf = confs[0]
                        cls_idx = clss[0]
                        x1, y1, x2, y2 = box
                        cx = int((x1 + x2) / 2)
                        cy = int((y1 + y2) / 2)

                        # Get frame center (apply shift to adjust aiming)
                        frame_center_x = (width / 2) + CROSSHAIR_SHIFT_X
                        frame_center_y = height / 2

                        # Calculate pixel offset from center
                        x_offset_pixels = cx - frame_center_x
                        y_offset_pixels = cy - frame_center_y

                        # Convert pixels to degrees
                        yaw_angle_degrees = x_offset_pixels * horizontal_degrees_per_pixel
                        pitch_angle_degrees = y_offset_pixels * vertical_degrees_per_pixel

                        # Calculate pulses
                        yaw_pulses = yaw_angle_degrees * YAW_PULSES_PER_DEGREE
                        pitch_pulses = pitch_angle_degrees * PITCH_PULSES_PER_DEGREE

                        # Send the pulses to the control thread
                        try:
                            pulse_queue.put((yaw_pulses, pitch_pulses), block=False)
                        except queue.Full:
                            pass

                        # Mark that we have at least one detection on-screen
                        detection_present = True

                        # Gun enable/disable logic with debounce
                        # Only enable the gun if the object has been detected continuously
                        # for at least DETECTION_ENABLE_THRESHOLD seconds.
                        global gun_enabled, gun_forced_off, last_shot_time, detection_start_time, gun_enabled_time
                        now_det = time.time()
                        if detection_start_time is None:
                            detection_start_time = now_det
                        else:
                            # detection has been present since detection_start_time
                            if (now_det - detection_start_time) >= DETECTION_ENABLE_THRESHOLD:
                                if not gun_forced_off and not gun_enabled:
                                    send_command(CMD_ENABLE_GUN)
                                    gun_enabled = True
                                    gun_enabled_time = time.time()

                        # If there is no detection later we will disable the gun (see after results loop)

                        # Shooting: If the (shifted) frame center lies inside the detected bounding box -> locked
                        frame_center_x = (width / 2) + CROSSHAIR_SHIFT_X
                        frame_center_y = height / 2
                        locked = (x1 <= frame_center_x <= x2) and (y1 <= frame_center_y <= y2)

                        # Only shoot if gun is enabled and not forced off
                        if locked and gun_enabled and not gun_forced_off:
                            now_shot = time.time()
                            # ensure gun has had required wind-up time since enabled
                            if gun_enabled_time is None or (now_shot - gun_enabled_time) < GUN_WINDUP:
                                # Not yet wound up; skip shooting
                                pass
                            else:
                                if now_shot - last_shot_time >= SHOOT_COOLDOWN:
                                    send_command(CMD_SHOOT_GUN)
                                    last_shot_time = now_shot

                        # suppressed debug print

                        # Calibration logic removed to avoid overlay and prints

                        # Draw bounding box and center point on display
                        cv2.rectangle(display, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        cv2.circle(display, (cx, cy), 4, (0, 0, 255), -1)

                        label = f"{conf:.2f}"
                        if names is not None and cls_idx is not None:
                            label = f"{names.get(int(cls_idx), cls_idx)} {conf:.2f}"
                        cv2.putText(display, label, (int(x1), int(y1) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

                        # Show pixel offsets from the detected ball to the frame center
                        offset_text = f"X offset: {int(x_offset_pixels)} px, Y offset: {int(y_offset_pixels)} px"
                        # Place the text near the top-left so it doesn't overlap the box
                        cv2.putText(display, offset_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                # After processing all results, if nothing detected disable the gun
                try:
                    any_detection = detection_present
                except NameError:
                    any_detection = False

                if not any_detection:
                    # Reset detection start time since nothing is detected now
                    detection_start_time = None
                if not any_detection and gun_enabled:
                    # No objects: disable gun unless forcibly locked off already
                    send_command(CMD_DISABLE_GUN)
                    gun_enabled = False
                    gun_enabled_time = None

                # FPS and display
                now = time.time()
                fps = 0.9 * fps + 0.1 * (1.0 / (now - prev_time)) if now != prev_time else fps
                prev_time = now
                cv2.putText(display, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

                # Overlay gun status
                status_text = f"Gun: {'ENABLED' if gun_enabled else 'DISABLED'}"
                if gun_forced_off:
                    status_text += " (FORCED OFF)"
                cv2.putText(display, status_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255) if gun_forced_off else (0, 200, 0), 2)
                cv2.imshow("YOLO Camera Viewer", display)

            except queue.Empty:
                pass

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == 32:  # Space bar toggles forced-off safety
                # Toggle forced-off state
                gun_forced_off = not gun_forced_off
                if gun_forced_off:
                    # Immediately disable the gun and prevent re-enabling until space pressed again
                    if gun_enabled:
                        send_command(CMD_DISABLE_GUN)
                        gun_enabled = False
                else:
                    # Releasing forced-off allows the normal detection-based enabling to resume
                    pass
    finally:
        exit_flag = True
        frame_queue.put(None)
        process_thread.join()
        if turret_thread.is_alive():
            turret_thread.join()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
