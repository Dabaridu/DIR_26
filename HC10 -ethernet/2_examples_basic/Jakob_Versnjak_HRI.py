"""Gripper test"""
__author__ = "Gaja Zumer & Jernej Puc"
__version__ = "18.08.22"
__email__ = "colette.dubois8@gmail.com, nejc.puc@gmail.com"


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
#print(yrc.real_type_variable_write(3.20, index=1))
data = yrc.robot_position_data_read()
a = data['fourth_axis_data']

#print(data)


print(struct.unpack('l', a))




print(yrc.servo_on())
yrc.move_straight(645000, -32000+100000, 210000, 1594030, -160240, -375770, 1800)
sleep(3)
yrc.move_straight(645000-100000, -32000+100000, 210000, 1594030, -160240, -375770, 1800)
sleep(3)
yrc.move_straight(645000-100000, -32000-100000, 210000, 1594030, -160240, -375770, 1800)
sleep(3)
yrc.move_straight(645000, -32000-100000, 210000, 1594030, -160240, -375770, 1800)
sleep(3)

print(yrc.servo_off())

