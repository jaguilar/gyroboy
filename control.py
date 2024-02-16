from math import degrees, radians
from pybrickspc.messaging import BluetoothMailboxClient, Mailbox
import pygame
import struct
import sys
import time
import config


# This is the address of the server EV3 we are connecting to.
SERVER = "f0:45:da:13:1c:8a"

_mailbox_client = BluetoothMailboxClient()

print("establishing connection...")
_mailbox_client.connect(SERVER)
print("connected!")

_direction = Mailbox(config.direction_channel, _mailbox_client)


def input_gain(x: float):
    return 0.4 * x + 0.6 * x**3


last_send = None


def handle_stick_input(j: pygame.joystick.JoystickType):
    global last_send
    if last_send is not None:
        if time.time() - last_send < 0.1:
            return
        last_send = time.time()
    left_right = int(100 * j.get_axis(0))
    forward_backward = int(100 * j.get_axis(1))
    _direction.send(struct.pack(config.direction_format, left_right, forward_backward))


print("pygame init")
pygame.init()
print("pygame init done")


def main():
    # Used to manage how fast the screen updates.
    clock = pygame.time.Clock()

    joysticks = {}
    while True:
        # Event processing step.
        # Possible joystick events: JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN,
        # JOYBUTTONUP, JOYHATMOTION, JOYDEVICEADDED, JOYDEVICEREMOVED
        for event in pygame.event.get():
            # Handle hotplugging
            if event.type == pygame.JOYDEVICEADDED:
                # This event will be generated when the program starts for every
                # joystick, filling up the list without needing to create them manually.
                joy = pygame.joystick.Joystick(event.device_index)
                joysticks[joy.get_instance_id()] = joy
                print(f"Joystick {joy.get_instance_id()} connencted")

            if event.type == pygame.JOYDEVICEREMOVED:
                del joysticks[event.instance_id]
                print(f"Joystick {event.instance_id} disconnected")

        if pygame.joystick.get_count() == 0:
            # Don't do anything unless there is a joystick
            clock.tick(1)
            print("no joy")
            continue

        for j in joysticks.values():
            handle_stick_input(j)

        # Limit to 30 frames per second.
        clock.tick(60)


if __name__ == "__main__":
    main()
    # If you forget this line, the program will 'hang'
    # on exit if running from IDLE.
    pygame.quit()
