"""DIR demo program"""
__author__ = "Matej Hodnik"
__version__ = "28.03.26"
__email__ = "mh2078@student.uni-lj.si"



########################################################################################################################
# Import statements

from yrc_high_speed_ethernet import ClientOfYRC, UDPRequest, UDPAnswer
from time import sleep
import struct


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

# Examples
print(yrc.status_information_reading())

# ----------------------------------------------------------------------------------------------------------------------
# INT VARIABLE TEST

print("\n--- INT VARIABLE TEST ---")

# Inicializacija (I10 = 0)
print("Set I30 = 0")
print(yrc.int_variable_write(0, index=30))
sleep(1)

# Sprememba (I10 = 123)
print("Set I31 = 123")
print(yrc.int_variable_write(123, index=31))
sleep(1)

# Sprememba (I10 = -50)
print("Set I32 = -50")
print(yrc.int_variable_write(-50, index=32))
sleep(1)


# ----------------------------------------------------------------------------------------------------------------------
# REAL VARIABLE TEST

print("\n--- REAL VARIABLE TEST ---")

# Inicializacija (R1 = 0.0)
print("Set R10 = 0.0")
print(yrc.real_type_variable_write(0.0, index=10))
sleep(1)

# Sprememba (R1 = 3.14)
print("Set R11 = 3.14")
print(yrc.real_type_variable_write(3.14, index=11))
sleep(1)

# Sprememba (R1 = -1.5)
print("Set R12 = -1.5")
print(yrc.real_type_variable_write(-1.5, index=12))
sleep(1)


# ----------------------------------------------------------------------------------------------------------------------
# OPTIONAL: SERVO TEST (pusti če hočeš)

print("\nServo ON:")
print(yrc.servo_on())
sleep(2)

print("\nServo OFF:")
print(yrc.servo_off())