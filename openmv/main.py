import sensor, image, time

from machine import SPI, Pin

# Camera setup
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()

# Ball color threshold for orange in L*a*b*
orange_threshold = (20, 100, 10, 90, 20, 70)

# SPI master setup
spi = SPI(1)  # SPI bus 1
spi.init(baudrate=1000000, polarity=0, phase=0, bits=8)

# CS tied low since single master/slave
cs = Pin("P3", Pin.OUT)
cs.value(0)

START_BYTE = 0xAA

while True:
    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs([orange_threshold], pixels_threshold=200, area_threshold=200, merge=True)

    # Default coordinates if no blob found
    cx, cy = 0, 0

    if blobs:
        largest_blob = max(blobs, key=lambda b: b.pixels())
        cx, cy = largest_blob.cx(), largest_blob.cy()
        img.draw_cross(cx, cy, color=(255, 0, 0), size=10)
        print("Blob X:", cx, "Y:", cy)

    # Send coordinates via SPI: start byte + X high + X low + Y high + Y low
    spi.write(bytearray([START_BYTE, cx >> 8, cx & 0xFF, cy >> 8, cy & 0xFF]))

    time.sleep_ms(50)
