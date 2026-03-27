"""Gripper test"""
__author__ = "Gaja Zumer & Jernej Puc"
__version__ = "18.08.22"
__email__ = "colette.dubois8@gmail.com, nejc.puc@gmail.com"


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
    yrc = ClientOfYRC('192.168.65.249')
    print('Connection with YRC1000 established')


########################################################################################################################
# Test

# Examples
print(yrc.status_information_reading())
#print(yrc.real_type_variable_write(3.20, index=1))



print(yrc.servo_on())
sleep(3)
print(yrc.servo_off())

