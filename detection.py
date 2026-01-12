from ultralytics import YOLO
import cv2
import numpy as np
# import serial
import time
import math

# Load the trained YOLO model
model = YOLO(r'C:\Users\rnave\Downloads\Plastic recyclable detection.v2i.yolov11\best.pt')

# Initialize video capture
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# Initialize serial communication (update COM port if needed)
# arduino = serial.Serial('COM4', 115200)
# time.sleep(2)  # Allow time for Arduino to reset

# Class names
class_names = model.names

# Arm segment lengths in mm
L1 = 100  # base to shoulder height
L2 = 120  # shoulder to elbow
L3 = 100  # elbow to wrist

def pixel_to_real(x_pixel, y_pixel):
    scale_x, scale_y = 0.5, 0.5
    offset_x, offset_y = -160, -120
    x_mm = (x_pixel + offset_x) * scale_x
    y_mm = (y_pixel + offset_y) * scale_y
    z_mm = 50  # fixed height
    return x_mm, y_mm, z_mm

def inverse_kinematics(x, y, z):
    try:
        theta1 = math.degrees(math.atan2(y, x))
        r = math.sqrt(x**2 + y**2)
        z_offset = z - L1

        D = (r**2 + z_offset**2 - L2**2 - L3**2) / (2 * L2 * L3)
        if D < -1 or D > 1:
            #print("Target out of reach")
            return [90]*6

        theta3 = math.degrees(math.acos(D))
        theta2 = math.degrees(math.atan2(z_offset, r) - math.atan2(L3 * math.sin(math.radians(theta3)),
                                                                    L2 + L3 * math.cos(math.radians(theta3))))
        theta4 = 180 - (theta2 + theta3)
        theta5 = 90  # fixed wrist roll
        theta6 = 20  # open gripper

        return [int(theta1), int(theta2), int(theta3), int(theta4), int(theta5), int(theta6)]

    except Exception as e:
        #print(f"IK Error: {e}")
        return [90]*6

previous_detections = set()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model.predict(source=frame, conf=0.5, verbose=False)
    annotated_frame = results[0].plot()

    boxes = results[0].boxes
    current_detections = set()

    if boxes is not None and boxes.cls.numel() > 0:
        detected_classes = boxes.cls.cpu().numpy().astype(int)

        for idx, cls in enumerate(detected_classes):
            x1, y1, x2, y2 = boxes.xyxy[idx].cpu().numpy()
            x_center = int((x1 + x2) / 2)
            y_center = int((y1 + y2) / 2)
            box_width = int(x2 - x1)
            box_height = int(y2 - y1)

            x_real, y_real, z_real = pixel_to_real(x_center, y_center)

            # Print pixel center and real world coordinates
            print(f"Object: {class_names[cls]} | Pixel center: ({x_center}, {y_center}) | Real-world coords (mm): X={x_real:.2f}, Y={y_real:.2f}, Z={z_real:.2f}")

            current_detections.add((class_names[cls], x_real, y_real, z_real, box_width, box_height))

            # Compute inverse kinematics and send angles to Arduino
            joint_angles = inverse_kinematics(x_real, y_real, z_real)
            angle_str = ",".join(str(min(max(a, 0), 180)) for a in joint_angles) + "\n"
            # arduino.write(angle_str.encode())
            print(f"angles: {angle_str.strip()}")

    else:
        print("No object detected.")

    previous_detections = current_detections

    cv2.namedWindow("Real-Time Detection", cv2.WINDOW_AUTOSIZE)
    cv2.resizeWindow("Real-Time Detection", 640, 480)
    cv2.moveWindow("Real-Time Detection", 10, 10)
    cv2.imshow("Real-Time Detection", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
# arduino.close()
