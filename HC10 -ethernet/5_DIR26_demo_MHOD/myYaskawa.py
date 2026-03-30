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

#####################################CONSTANTS##########################################################################
ROT_PREE = 10000 #0.0001 degres -> 1deg
TRAN_PREE = 1000 # 0.001 mm -> 1mm

#################USER FUNCTIONS##########################################################################################
def wait_move_finnish():
    status = yrc.status_information_reading()
    while(status['Running'] != 0):
        print("Waiting for movement to finish...", end="\r")
        sleep(0.5)
        status = yrc.status_information_reading()

def gripper_grab():
    ok_select = yrc.job_select("DIR26CILCLOSE", 0)
    print("Select DIR26CILCLOSE:", ok_select)

    if ok_select:
        ok_start = yrc.job_start()
        print("Start DIR26CILCLOSE:", ok_start)
    else:
        print("Failed to select DIR26CILCLOSE job")

def gripper_release():
    ok_select = yrc.job_select("DIR26CILOPEN", 0)
    print("Select DIR26CILOPEN:", ok_select)

    if ok_select:
        ok_start = yrc.job_start()
        print("Start DIR26CILOPEN:", ok_start)
    else:
        print("Failed to select DIR26CILOPEN job")

########################################################################################################################
# Test

#position over table
X = 500*TRAN_PREE #in mm
Y = -150*TRAN_PREE
Z = 530*TRAN_PREE
TX = 180*ROT_PREE #in deg
TY = 1*ROT_PREE
TZ = 60*ROT_PREE


#position up high
X0 = 360*TRAN_PREE
Y0 = -124*TRAN_PREE
Z0 = 693*TRAN_PREE
TX0 = 180*ROT_PREE
TY0 = 0*ROT_PREE
TZ0 = 140*ROT_PREE



STEP_10CM = 100000   # 100 mm
SPEED = 2000          # 20.0 mm/s

print("Initial status:")
print(yrc.status_information_reading())


print("\nServo ON:")
print(yrc.servo_on())
print("Servos ON")

sleep(1)

# print("\nMove ")
# print(yrc.move_link(X, Y, Z, TX, TY, TZ, SPEED))
# wait_move_finnish()


# print("\nMove back to start:")
# print(yrc.move_straight(X0, Y0, Z0, TX0, TY0, TZ0, SPEED))
# wait_move_finnish()

#
#   DIR26CILOPEN
#   DIR26CILCLOSE
#

gripper_grab()

sleep(5)

# gripper_release()

# sleep(1)





print("\nServo OFF:")
print(yrc.servo_off())
print("Servos OFF")


