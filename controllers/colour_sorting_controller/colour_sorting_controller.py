"""UR5e colour-sorting controller for Webots.

Reads a distance sensor, a wrist position sensor and an overhead camera,
classifies the object colour with OpenCV HSV masks, and drives the UR5e arm
and Robotiq 3F gripper through a five-state machine:
WAITING -> GRASPING -> ROTATING -> RELEASING -> ROTATING_BACK.

Author: Akinpelumi Michael Ajayi
"""
import sys
from controller import Robot
import cv2
import numpy as np

# STATES
WAITING = 0
GRASPING = 1
ROTATING = 2
RELEASING = 3
ROTATING_BACK = 4

TIME_STEP = 32

robot = Robot()
timestep = int(robot.getBasicTimeStep())

counter = 0
state = WAITING

target_positns_blue = [-1.88, -2.14, -2.38, -1.51]
target_positns_red = [-1.88, 2.14, -2.38, -1.51]
target_positns_green = [-1.88, -1.14, -2.38, -1.51]
# Fallback for unknown or yellow objects. Without this line the controller
# crashed with a NameError. Tune these joint angles to a reject area in your world.
target_positns_default = [-1.88, 0.0, -2.38, -1.51]

# Speed
speed = 1.0
if len(sys.argv) == 2:
    speed = float(sys.argv[1])

# Gripper Motors
hand_motors = [
    robot.getDevice("finger_1_joint_1"),
    robot.getDevice("finger_2_joint_1"),
    robot.getDevice("finger_middle_joint_1")
]

# UR Motors
ur_motors = [
    robot.getDevice("shoulder_lift_joint"),
    robot.getDevice("elbow_joint"),
    robot.getDevice("wrist_1_joint"),
    robot.getDevice("wrist_2_joint")
]

for m in ur_motors:
    m.setVelocity(speed)

# Sensors
distance_sensor = robot.getDevice("distance sensor")
distance_sensor.enable(TIME_STEP)

position_sensor = robot.getDevice("wrist_1_joint_sensor")
position_sensor.enable(TIME_STEP)

# Camera
camera = robot.getDevice("camera")
camera.enable(timestep)
width = camera.getWidth()
height = camera.getHeight()


#  Image and colour detection(HSV MASK)
def get_object_image(robot_camera):
    raw = robot_camera.getImage()
    img = np.frombuffer(raw, np.uint8).reshape((height, width, 4))
    return img[:, :, :3]  # BGR

def detect_colour_hsv(img):
    img = cv2.GaussianBlur(img, (7, 7), 0)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv[:, :, 2] = cv2.equalizeHist(hsv[:, :, 2])

    # Red
    red1 = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255))
    red2 = cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))
    red_mask = red1 | red2
    # Green
    green_mask = cv2.inRange(hsv, (40, 70, 70), (80, 255, 255))
    # Blue
    blue_mask = cv2.inRange(hsv, (100, 180, 80), (130, 255, 255))
    # Default
    default_mask = cv2.inRange(hsv, (20, 100, 100), (30, 255, 255))

    # Pixel counts
    pixel_counts = {
        "RED": np.sum(red_mask > 0) * 1.5,
        "GREEN": np.sum(green_mask > 0),
        "BLUE": np.sum(blue_mask > 0),
        "DEFAULT": np.sum(default_mask > 0) * 1.3
    }
    # Threshold
    if max(pixel_counts.values()) < 500:
        return None
    return max(pixel_counts, key=pixel_counts.get)
    
def clamp_release_positn(postn):
    return max(postn, 0.05)
    
#  main loop 
while robot.step(TIME_STEP) != -1:
    img = get_object_image(camera)
    if counter <= 0:
        #print(counter)
        
        # WAITING
        if state == WAITING:
            print(f"Waiting... Distance sensor: {distance_sensor.getValue()}")

            if distance_sensor.getValue() < 500:
                state = GRASPING
                counter = 8
                print("Object detected: Grasping...")
                
                # close gripper
                for m in hand_motors:
                    m.setPosition(0.85)  

        # GRASPING: detect colour and move arm
        elif state == GRASPING:
            colour_name = detect_colour_hsv(img)
            print(f"Detected colour: {colour_name}")
            
            if colour_name == "RED":
                targets = target_positns_red
                print("red basket")

            elif colour_name == "GREEN":
                targets = target_positns_green
                print("green basket")
                
            elif colour_name == "BLUE":
                targets = target_positns_blue
                print("blue basket")
            
            else:
                targets = target_positns_default
                print("Unknown")

            for i in range(4):
                ur_motors[i].setPosition(targets[i])

            state = ROTATING
            counter = 8
            print("Rotating arm...")

        # ROTATING: stop when joint gets here
        elif state == ROTATING:
            if position_sensor.getValue() < -2.3:
                counter = 8
                print("Releasing object...")
                state = RELEASING

                for m in hand_motors:
                    release_postn = clamp_release_positn(0.0)
                    m.setPosition(release_postn)

        # RELEASING
        elif state == RELEASING:
            for m in ur_motors:
                m.setPosition(0.0)

            state = ROTATING_BACK
            counter = 8
            print("Rotating arm back...")

        # ROTATING BACK: return to WAITING
        elif state == ROTATING_BACK:
            if position_sensor.getValue() > -0.1:
                state = WAITING
                print("Waiting for object...")

    counter -= 1
