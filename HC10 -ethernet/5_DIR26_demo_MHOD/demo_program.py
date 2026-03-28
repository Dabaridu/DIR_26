"""DIR demo program"""
__author__ = "Matej Hodnik"
__version__ = "18.08.22"
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

# Examples
print(yrc.status_information_reading())
#print(yrc.real_type_variable_write(3.20, index=1))



print(yrc.servo_on())
print('Servos ON')
sleep(3)
print(yrc.servo_off())
print('Servos OFF')

