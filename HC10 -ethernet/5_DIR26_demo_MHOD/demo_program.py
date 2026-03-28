"""DIR demo program"""
__author__ = "Matej Hodnik"
__version__ = "28.03.26"
__email__ = "mh2078@student.uni-lj.si"


########################################################################################################################
# Import statements

from yrc_high_speed_ethernet import ClientOfYRC, UDPRequest, UDPAnswer
from time import sleep

########################################################################################################################
# Initialisation

# TODO: User Configuration Parameters
local = False

# Open UDP connection
if local:
    yrc = ClientOfYRC('localhost')
    print('Connection with virtual controller established')
else:
    yrc = ClientOfYRC('172.16.0.1')
    print('Connection with YRC1000 established')


########################################################################################################################
# Test

X = 500843
Y = 162045
Z = 531355
TX = 1799513
TY = -2807
TZ = -29

STEP_10CM = 100000   # 100 mm
SPEED = 200          # 20.0 mm/s

print("Initial status:")
print(yrc.status_information_reading())

print("\nServo ON:")
print(yrc.servo_on())
print("Servos ON")

sleep(1)

print("\nMove +10 cm in X:")
print(yrc.move_straight(X + STEP_10CM, Y, Z, TX, TY, TZ, SPEED))

sleep(8)

print("\nMove back to start:")
print(yrc.move_straight(X, Y, Z, TX, TY, TZ, SPEED))

sleep(8)

print("\nServo OFF:")
print(yrc.servo_off())
print("Servos OFF")