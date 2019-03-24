import socket
import netifaces
import pygame
import time
from pygame.locals import *

# Port to send joystick data
PORT = 4141

# Get the bot's IP address
gws = netifaces.gateways()
IP_OTIS = gws['default'][netifaces.AF_INET][0]

# Setup the joystick
axes = 0
is_joystick = False
joystick = None
jostick_name = None  

pygame.display.init()
pygame.joystick.init()

joystick_count = pygame.joystick.get_count()
for i in range(joystick_count):
    joystick = pygame.joystick.Joystick(i)
    joystick.init()
    name = joystick.get_name()
    axes = joystick.get_numaxes()

    if axes == 4:
        is_joystick = True
        joystick = joystick
        joystick_name = name
        print("Registering %s" % name)

while(True):
    if is_joystick:
        pygame.event.pump()
        tiltf = joystick.get_axis(1)
        yawf = joystick.get_axis(3)
        tiltu = int((tiltf + 1.0)/2 * (2**16-1))
        yawu = int((yawf + 1.0)/2 * (2**16-1))

        y0 = yawu & 0x00ff
        y1 = yawu >> 8

        t0 = tiltu & 0x00ff
        t1 = tiltu >> 8
        print('%d %d' % (tiltu, yawu))

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((IP_OTIS, PORT))
            byte_data = bytearray()
            byte_data.append(t0)
            byte_data.append(t1)
            byte_data.append(y0)
            byte_data.append(y1)
            s.send(byte_data)

        time.sleep(1/10)



