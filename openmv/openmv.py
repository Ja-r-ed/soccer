import sensor, image, time
from machine import SPI, Pin

# Camera setup
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
clock = time.clock()

# Color threshold (tune for your target)
orange_threshold = (30, 100, 30, 100, 10, 60)

# SPI Master: use bus 2 (pins P0=SCK, P1=MOSI, P2=MISO)
spi = SPI(2, baudrate=1000000, polarity=0, phase=0, bits=8, firstbit=SPI.MSB)

cx = 0
cy = 0

while True:
    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs([orange_threshold], area_threshold=150)
    if blobs:
        largest_blob = max(blobs, key=lambda b: b.pixels())
        img.draw_rectangle(largest_blob.rect())
        img.draw_cross(largest_blob.cx(), largest_blob.cy())
        cx = largest_blob.cx()
        cy = largest_blob.cy()
    else:
        cx, cy = 0, 0

    # Build packet (optional sync marker 0xFF, 0xFF)
    packet = bytearray([0xFF, 0xFF, cx >> 8, cx & 0xFF, cy >> 8, cy & 0xFF])

    # Send packet
    spi.write(packet)

    time.sleep_ms(50)  # control rate
