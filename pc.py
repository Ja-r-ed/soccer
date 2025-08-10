import socket
import struct
import time
import pygame

pygame.init()
pygame.joystick.init()

TCP_IP = '10.92.32.32'
TCP_PORT = 10000

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

if pygame.joystick.get_count() == 0:
    print("No joystick detected.")
else:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick detected: {joystick.get_name()}")

    try:
        while True:
            pygame.event.pump()
            # Get left stick (axes 0,1) and right stick (axes 2,3)
            left_x = joystick.get_axis(0)
            left_y = joystick.get_axis(1)
            right_x = joystick.get_axis(2)
            right_y = joystick.get_axis(3)
            print(f"Left Stick: ({left_x:.2f}, {left_y:.2f}), Right Stick: ({right_x:.2f}, {right_y:.2f})")
            # Pack as 4 floats and send
            data = struct.pack('ffff', left_x, left_y, right_x, right_y)
            s.sendall(data)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass

    joystick.quit()

s.close()
pygame.quit()