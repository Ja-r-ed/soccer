# defiendo porteria amarilla y no me meto a la azul
import math
import time
import utime
import image
import sensor
from machine import UART

# --- UART CONFIG FOR RT1062 ---
# id = 3 means UART3 (same as pyb.UART(3,...))
# Adjust tx/rx pins if necessary, default pins work on OpenMV RT1062
uart = UART(1, baudrate=115200, timeout_char=0)

# Thresholds_ball = (16, 100, 12, 127, -11, 127)
Thresholds_ball = (27, 42, 53, 127, 41, 56)
thresholds_yellow_goal = (0,0,0,0,0,0)
tresholds_blue_goal = (0,0,0,0,0,0)
# thresholds_yellow_goal = (53, 89, 7, 60, 16, 89)
# tresholds_blue_goal = (8, 36, -3, 14, -23, -5)
thresholds_line = (86, 100, -128, 127, -128, 127)

FRAME_HEIGHT = sensor.height()
FRAME_WIDTH = sensor.width()
FRAME_ROBOT = 20
FRAME_CIRCLE = 150
PIXEL_SIZE_HEIGHT = 0.2726875
PIXEL_SIZE_WIDTH = 0.25
CAMERA_HEIGHT = 5
FLOOR_HEIGHT = 15
CM_CONVERTION = 0.1
minor_x = 0
minor_y = 0

distance_ball = 0
distance_pixels = 0
angle_ball = 0
distance_goal = 0
angle_goal = 0
final_goal_distance = 0

def initialize_sensor():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_vflip(False)
    sensor.set_hmirror(True)
    sensor.skip_frames(time=100)
    sensor.set_brightness(-3)
    sensor.set_contrast(-2)
    sensor.set_saturation(-3)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)


def locate_blob(img):
    blobs1 = img.find_blobs([Thresholds_ball], area_threshold=1, merge=True)
    blobs2 = img.find_blobs([thresholds_yellow_goal], area_threshold=1000, merge=True)
    blobAzul = img.find_blobs([tresholds_blue_goal], area_threshold=1000, merge=True)

    for blob in blobs1:
        img.draw_rectangle(blob.rect(), color=(0, 255, 0))
        img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))

    for blob in blobs2:
        img.draw_ellipse(blob.enclosed_ellipse(), color=(255, 255, 0))
        img.draw_cross(blob.cx(), blob.cy(), color=(255, 255, 0))

    for blob in blobAzul:
        img.draw_ellipse(blob.enclosed_ellipse(), color=(0, 0, 255))
        img.draw_cross(blob.cx(), blob.cy(), color=(255, 255, 0))

    return blobs1, blobs2, blobAzul

def locate_biggest_blob(img):
    # Find blobs
    blobs_ball = img.find_blobs([Thresholds_ball], area_threshold=1, merge=True)
    blobs_yellow_goal = img.find_blobs([thresholds_yellow_goal], area_threshold=1000, merge=True)
    blobs_blue_goal = img.find_blobs([tresholds_blue_goal], area_threshold=1000, merge=True)

    # Initialize largest blobs
    largest_ball = None
    largest_yellow_goal = None
    largest_blue_goal = None

    # Select largest ball blob
    if blobs_ball:
        largest_ball = max(blobs_ball, key=lambda b: b.pixels())
        img.draw_rectangle(largest_ball.rect(), color=(0, 255, 0))
        img.draw_cross(largest_ball.cx(), largest_ball.cy(), color=(0, 255, 0))

    # Select largest yellow goal blob
    if blobs_yellow_goal:
        largest_yellow_goal = max(blobs_yellow_goal, key=lambda b: b.pixels())
        img.draw_ellipse(largest_yellow_goal.enclosed_ellipse(), color=(255, 255, 0))
        img.draw_cross(largest_yellow_goal.cx(), largest_yellow_goal.cy(), color=(255, 255, 0))

    # Select largest blue goal blob
    if blobs_blue_goal:
        largest_blue_goal = max(blobs_blue_goal, key=lambda b: b.pixels())
        img.draw_ellipse(largest_blue_goal.enclosed_ellipse(), color=(0, 0, 255))
        img.draw_cross(largest_blue_goal.cx(), largest_blue_goal.cy(), color=(255, 255, 0))

    return largest_ball, largest_yellow_goal, largest_blue_goal

def calculate_distance(blob):
    relative_cx = blob.cx() - FRAME_HEIGHT
    relative_cy = blob.cy() - FRAME_WIDTH
    magnitude_distance = math.sqrt(relative_cx**2 + relative_cy**2)
    total_distance = 0.6217 * math.exp(0.0166 * magnitude_distance)
    print(total_distance)
    return total_distance


def calculate_opp_distance(goal_distance, goal_angle):
    if goal_angle > 270:
        goal_angle = 360 - goal_angle
    distance_final = goal_distance * math.sin(math.radians(goal_angle))
    return distance_final


def calculate_distance_goal(blob):
    relative_cx = blob.cx() - FRAME_HEIGHT
    relative_cy = blob.cy() - FRAME_WIDTH
    magnitude_distance = math.sqrt(relative_cx**2 + relative_cy**2)
    total_distance = 11.83 * math.exp(0.0245 * magnitude_distance)
    return total_distance


def calculate_angle(blob):
    relative_cx = blob.cx() - FRAME_HEIGHT
    relative_cy = blob.cy() - FRAME_WIDTH
    angle = math.atan2(relative_cy, relative_cx)
    angle_degrees = math.degrees(angle)
    angle_degrees += 8
    if angle_degrees < 0:
        angle_degrees += 360
    return angle_degrees


def main():
    initialize_sensor()
    clock = time.clock()
    global distance_ball, angle_ball, angle_goal, distance_pixels

    while True:
        distance_ball = 0
        angle_ball = 0
        distance_goal = 0
        angle_goal = 0
        final_goal_distance = 0

        clock.tick()
        img = sensor.snapshot()
        ball, yellowGoal, blueGoal = locate_blob(img)

        if ball:
            for blob in ball:
                distance_ball = calculate_distance(blob)
                angle_ball = calculate_angle(blob)

        if yellowGoal:
            for blob in yellowGoal:
                distance_pixels = calculate_distance_goal(blob)
                angle_goal = calculate_angle(blob)

        if blueGoal:
            for blob in blueGoal:
                distance_pixels = calculate_distance_goal(blob)
                angle_goal = calculate_angle(blob)

        uart.write("{:.2f} {:.2f} {:.2f} {:.2f}\n".format(
            distance_ball, angle_ball, angle_goal, distance_pixels
        )
        )

        time.sleep_ms(50)


if __name__ == "__main__":
    main()
