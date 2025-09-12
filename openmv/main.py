# Defiendo porteria amarilla, solo envía posición x,y de la bola y la portería más grande
import time
import sensor
import image
from machine import UART

# --- UART CONFIG FOR RT1062 ---
uart = UART(1, baudrate=115200, timeout_char=0)

# COLOR THRESHOLDS
Thresholds_ball = (27, 42, 53, 127, 41, 56)
thresholds_yellow_goal = (6, 58, 14, 53, 11, 69)
thresholds_blue_goal = (18, 40, 41, 66, -96, -61)

loop = 0
cycle = 0

# CAMERA SETTINGS
def initialize_sensor():
    # sensor.reset()
    # sensor.set_pixformat(sensor.RGB565)
    # sensor.set_framesize(sensor.QVGA)
    # sensor.set_vflip(False)
    # sensor.set_hmirror(True)
    # sensor.skip_frames(time=100)
    # sensor.set_brightness(-3)
    # sensor.set_contrast(-2)
    # sensor.set_saturation(-3)
    # sensor.set_auto_gain(False)
    # sensor.set_auto_whitebal(False)

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

# LOCATE LARGEST BLOB
def locate_biggest_blob(img, thresholds, area_thresh=1):
    blobs = img.find_blobs([thresholds], area_threshold=area_thresh, merge=True)
    if blobs:
        largest = max(blobs, key=lambda b: b.pixels())
        # Draw for debugging
        img.draw_rectangle(largest.rect(), color=(0, 255, 0))
        img.draw_cross(largest.cx(), largest.cy(), color=(0, 255, 0))
        return largest
    return None

def main():
    global loop
    global cycle
    initialize_sensor()
    clock = time.clock()

    while True:
        ballx = 0
        bally = 0
        goalx = 0
        goaly = 0

        clock.tick()
        img = sensor.snapshot()

        # Get largest ball
        ball_blob = locate_biggest_blob(img, Thresholds_ball)

        # Get largest yellow and blue goal blobs
        yellow_blob = locate_biggest_blob(img, thresholds_yellow_goal, area_thresh=1000)
        blue_blob = locate_biggest_blob(img, thresholds_blue_goal, area_thresh=1000)

        # Decide which goal is bigger
        goal_blob = None
        if yellow_blob and blue_blob:
            goal_blob = yellow_blob if yellow_blob.pixels() >= blue_blob.pixels() else blue_blob
        elif yellow_blob:
            goal_blob = yellow_blob
        elif blue_blob:
            goal_blob = blue_blob

        if ball_blob:
            ballx, bally = ball_blob.cx(), ball_blob.cy()

        if goal_blob:
            goalx, goaly = goal_blob.cx(), goal_blob.cy()

        # Only send if we found both a ball and a goal
        if ball_blob or goal_blob:
            uart.write("{:.2f} {:.2f} {:.2f} {:.2f}\n".format(
                ballx, bally,
                goalx, goaly
            ))
            print("sent: {:.2f} {:.2f} {:.2f} {:.2f}".format(
                ballx, bally,
                goalx, goaly
            ))
        if not ball_blob or not goal_blob:
            uart.write("{:.2f} {:.2f} {:.2f} {:.2f}\n".format(
                0, 0,
                0, 0
            ))
            print("sent: {:.2f} {:.2f} {:.2f} {:.2f}".format(
                0, 0,
                0, 0
            ))

        # cycle += 1
        # loop += 1
        # if loop == 50:
        #     try:
        #         img.save("frame_%d.jpg" % cycle)
        #         print("frame_%d.jpg" % cycle)
        #     except:
        #         print("Error saving image")

        #     loop = 0


if __name__ == "__main__":
    main()
