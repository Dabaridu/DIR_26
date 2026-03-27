"""Local server for limited offline debugging of the YRC1000 High-speed Ethernet server function"""
__author__ = "Gaja Zumer, Jernej Puc"
__version__ = "18.03.12"
__email__ = "colette.dubois8@gmail.com, nejc.puc@gmail.com"


########################################################################################################################
# Import statements

import socket


########################################################################################################################
# Null UDP packet definition

data_size = 32

output_data = bytes([0, 0, 0, 0,                        # identifier
                     0, 0,                              # head size
                     data_size, 0,                      # data size
                     0,                                 # reserve 1
                     0,                                 # proc div
                     0,                                 # ACK
                     0,                                 # req id
                     0, 0, 0, 0,                        # block number
                     0, 0, 0, 0, 0, 0, 0, 0,            # reserve 2
                     0,                                 # service
                     0,                                 # status
                     0,                                 # added status size
                     0,                                 # padding 1
                     0, 0,                              # added status
                     0, 0                               # padding 2
                     ])

output_data += bytes([0 for i in range(data_size)])     # data


########################################################################################################################
# Socket initialisation

link = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = ('localhost', 10040)
link.bind(server_address)

print('\nCHANNEL TO ', server_address, ' OPEN\n')


########################################################################################################################
# Print raw data

while True:
    (input_data, client_address) = link.recvfrom(512)

    if input_data:
        print(input_data, '\n')
        link.sendto(output_data, client_address)
