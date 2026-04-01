"""Python implementation of YRC1000 High-speed Ethernet server function"""
__author__ = "Gaja Zumer, Jernej Puc after kromau and Luka Kresnik"
__version__ = "21.03.2019"
__email__ = "colette.dubois8@gmail.com, nejc.puc@gmail.com, luka.kresnik@hotmail.com"


########################################################################################################################
# Import statements

import socket
import struct
import logging
import pprint
from threading import Lock


########################################################################################################################
# UDP packet definitions

class UDPRequest:
    """UDP request packet header, subheader and data"""

    def __init__(self, sub_header, data=[], proc_div=1):

        # Header
        self.identifier = list('YERC'.encode('latin1'))     # Identifier         4 Byte      Fixed to YERC
        self.headSize = [0x20, 0x00]                        # Header part size   2 Byte      Fixed to 0x20
        self.dataSize = [len(data), 0x00]                   # Data part size     2 Byte      Variable
        self.reserve1 = [3]                                 # Reserve 1          1 Byte      Fixed to "3"
        self.procDiv = [proc_div]                           # Processing div     1 Byte      1: robot control
        self.ACK = [0]                                      # ACK                1 Byte      0: Request, 1: Other
        self.reqID = [0]                                    # Request ID         1 Byte      ID for command session
        self.blockNo = [0, 0, 0, 0]                         # Block No.          4 Byte      0: Request
        self.reserve2 = [9, 9, 9, 9, 9, 9, 9, 9]            # Reserve 2          8 Byte      Fixed to "99999999"

        # Sub-header
        self.commandNo = sub_header['commandNo']
        self.instance = sub_header['instance']
        self.attribute = sub_header['attribute']
        self.service = sub_header['service']

        # Padding
        self.padding = [0, 0]

        # Data
        self.data = data

    def to_bytes(self):
        """Returns byte representation of data, ready to be sent via socket"""

        return bytes(self.identifier +
                     self.headSize +
                     self.dataSize +
                     self.reserve1 +
                     self.procDiv +
                     self.ACK +
                     self.reqID +
                     self.blockNo +
                     self.reserve2 +
                     self.commandNo +
                     self.instance +
                     self.attribute +
                     self.service +
                     self.padding +
                     self.data
                     )


class UDPAnswer:
    """Answer UDP packet sub-header and data"""

    def __init__(self, ans_packet):

        # Header
        self.identifier = ans_packet[0:4]
        self.headSize = ans_packet[4:6]
        self.dataSize = ans_packet[6:8]
        self.reserve1 = [ans_packet[8]]
        self.procDiv = [ans_packet[9]]
        self.ACK = [ans_packet[10]]
        self.reqID = [ans_packet[11]]
        self.blockNo = ans_packet[12:16]
        self.reserve2 = ans_packet[16:24]

        # Sub-header
        self.service = [ans_packet[24]]
        self.status = [ans_packet[25]]
        self.added_status_size = [ans_packet[26]]
        self.padding1 = [ans_packet[27]]
        self.added_status = ans_packet[28:30]
        self.padding2 = ans_packet[30:32]

        # Data
        self.data = ans_packet[32:]


########################################################################################################################
# Client definition

class ClientOfYRC:
    """Instance of the client connected to the YRC1000 controller"""

    def __init__(self, ip='localhost', port=10040):

        # Create UDP socket
        self.com = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.com.settimeout(2)

        # Socket data
        self.address = (ip, port)

        #mutex for multithread applications
        self.mutex = Lock()

        # Controller status
        self.status = {}

    def yrc_response_to_packet(self, sub_header, data, proc_div=1):
        """Sends data to the YRC1000 controller and returns the response"""

        # Debug
        # print('\nSub-header: ', sub_header)
        # print('Data: ', data)

        # Request
        req_packet = UDPRequest(sub_header, data, proc_div)

        # Answer
        ans_packet = self.get_yrc_response_to(req_packet)

        # Return
        if ans_packet is None:
            return None

        else:
            return UDPAnswer(ans_packet)

    def get_yrc_response_to(self, req_packet):
        """Separate function for communication in case of errors blocking further execution"""

        self.mutex.acquire()
        try:
            # Debug
            # print('Binary: \n', req_packet.to_bytes())

            # Send request
            self.com.sendto(req_packet.to_bytes(), self.address)

            # Wait for response
            (ans_packet, address) = self.com.recvfrom(512)

        except socket.timeout as e:
            print('Socket timeout: ' + str(e))
            logging.exception(str(e))
            return None

        except socket.gaierror as e:
            print('Socket address error')
            logging.exception(str(e))
            return None

        except socket.error as e:
            print('Socket related error')
            logging.exception(str(e))
            return None

        else:
            return ans_packet

        finally:
            self.mutex.release()

########################################################################################################################
    #HSE commands for robot control

    def status_information_reading(self):
        """See command 3.3.3, page 73"""

        # Sub-header
        sub_header = {'commandNo': [0x72, 0x00],
                      'instance': [1, 0],
                      'attribute': [0],
                      'service': [0x01]
                      }

        # Data
        data = []

        # YRC response
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])

        # Parse answer data
        byte1 = ans_packet.data[0]
        byte2 = ans_packet.data[4]

        def status_of(int_type, offset):	
            mask = 1 << offset
            if (int_type & mask) == mask:
                return True
            else:
                return False

        self.status['Step'] = status_of(byte1, 0)
        self.status['Cycle'] = status_of(byte1, 1)
        self.status['Auto'] = status_of(byte1, 2)
        self.status['Running'] = status_of(byte1, 3)
        self.status['InGuard'] = status_of(byte1, 4)
        self.status['Teach'] = status_of(byte1, 5)
        self.status['Play'] = status_of(byte1, 6)
        self.status['Remote'] = status_of(byte1, 7)

        self.status['Hold_PP'] = status_of(byte2, 1)
        self.status['Hold_Ext'] = status_of(byte2, 2)
        self.status['Hold_Cmd'] = status_of(byte2, 3)
        self.status['Alarm'] = status_of(byte2, 4)
        self.status['Error'] = status_of(byte2, 5)
        self.status['ServoOn'] = status_of(byte2, 6)

        return self.status

    def robot_position_data_read(self):
        """See command 3.3.6, page 78"""

        # Sub-header
        sub_header = {'commandNo': [0x75, 0x00],
                      'instance': [101, 0],
                      'attribute': [0],
                      'service': [0x01]
                      }

        # Data
        data = []

        # YRC response
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])

        return {'data_type': ans_packet.data[0:4],
                'type': ans_packet.data[4:8],
                'tool_no': ans_packet.data[8:12],
                'user_coordinate_no': ans_packet.data[12:16],
                'extended_type': ans_packet.data[16:20],
                'first_axis_data': ans_packet.data[20:24],
                'second_axis_data': ans_packet.data[24:28],
                'third_axis_data': ans_packet.data[28:32],
                'fourth_axis_data': ans_packet.data[32:36],
                'fifth_axis_data': ans_packet.data[36:40],
                'sixth_axis_data': ans_packet.data[40:44]}

    def io_data_write(self, value, number=2701):
        """See command 3.3.9, page 83"""

        # Sub-header
        sub_header = {'commandNo': [0x78, 0x00],
                      'instance': list(struct.pack('=h', number)),
                      'attribute': [1],
                      'service': [0x10]
                      }

        # Data
        print("stevilka je = " + str(number))
        data = [value]

        # YRC response
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])

        # Confirm successful operation
        return ans_packet.data  # ans_packet.status[0] == 0

    def io_data_read(self, number):
        """See command 3.3.9, page 83"""

        # Sub-header
        sub_header = {'commandNo': [0x78, 0x00],
                      'instance': list(struct.pack('=h', number)),
                      'attribute': [1],
                      'service': [0x0E]
                      }

        # Data
        data = []

        # YRC response
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])

        # Confirm successful operation
        return ans_packet.data  # ans_packet.status[0] == 0
    
    def byte_variable_read(self, index=1):
        """See command 3.3.11, page 85"""

        # Sub-header
        sub_header = {'commandNo': [0x7a, 0x00],
                      'instance': [index, 0],
                      'attribute': [1],
                      'service': [0x0e]
                      }

        # Data
        data = []

        # YRC response
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])

        return ans_packet.data

    def byte_variable_write(self, value, index=1):
        """See command 3.3.11, page 85"""

        # Sub-header
        sub_header = {'commandNo': [0x7a, 0x00],
                      'instance': [index, 0],
                      'attribute': [1],
                      'service': [0x10]
                      }

        # Data
        data = [value]

        # YRC response
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])

        # Confirm successful operation
        return ans_packet.status[0] == 0

    def int_variable_write(self, value, index=1):
        """See command 3.3.12, page 86"""

        # Sub-header
        sub_header = {'commandNo': [0x7b, 0x00],
                      'instance': [index, 0],
                      'attribute': [1],
                      'service': [0x10]
                      }

        # Data
        data = list(struct.pack("=h", value))

        # YRC response
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])

        # Confirm successful operation
        return ans_packet.status[0] == 0

    def double_int_variable_write(self, value, index=1):
        """See command 3.3.13, page 87"""

        # Sub-header
        sub_header = {'commandNo': [0x7c, 0x00],
                      'instance': [index, 0],
                      'attribute': [1],
                      'service': [0x10]
                      }

        # Data
        data = list(struct.pack("=l", value))

        # YRC response
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])

        # Confirm successful operation
        return ans_packet.status[0] == 0

    def real_type_variable_write(self, value, index=1):
        """See command 3.3.14, page 88"""

        # Sub-header
        sub_header = {'commandNo': [0x7d, 0x00],
                      'instance': [index, 0],
                      'attribute': [1],
                      'service': [0x10]
                      }

        # Data
        data = list(struct.pack("=f", value))

        # YRC response
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])

        # Confirm successful operation
        return ans_packet.status[0] == 0

    def robot_position_type_variable_write(self, x, y, z, tx, ty, tz, index=1):
        """See command 3.3.16, page 90"""

        # Sub-header
        sub_header = {'commandNo': [0x7f, 0x00],
                      'instance': [index, 0],
                      'attribute': [0],
                      'service': [0x02]
                      }

        # Data
        data = [16, 0, 0, 0,                    # Data type
                0, 0, 0, 0,                     # Figure
                0, 0, 0, 0,                     # Tool number
                0, 0, 0, 0,                     # User coordinae number
                0, 0, 0, 0]                     # Extended figure
        data += list(struct.pack('=l', x))      # First coordinate data
        data += list(struct.pack('=l', y))      # Second coordinate data
        data += list(struct.pack('=l', z))      # Third coordinate data
        data += list(struct.pack('=l', tx))     # Fourth coordinate data
        data += list(struct.pack('=l', ty))     # Fifth coordinate data
        data += list(struct.pack('=l', tz))     # Sixth coordinate data
        data += [0, 0, 0, 0,                    # Seventh coordinate data
                 0, 0, 0, 0]                    # Eighth coordinate data

        # YRC response
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])

        # Confirm successful operation
        return ans_packet.status[0] == 0

    def alarm_reset(self):
        """See command 3.3.19, page 97"""

        # Sub-header
        sub_header = {'commandNo': [0x82, 0x00],
                      'instance': [1, 0],
                      'attribute': [1],
                      'service': [0x10]
                      }

        # Data
        data = [1, 0, 0, 0]

        # YRC response
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])

        # Confirm successful operation
        return ans_packet.status[0] == 0

    def hold_on(self):
        """See command 3.3.20, page 3-52 (65)"""

        # Sub-header
        sub_header = {'commandNo': [0x83, 0x00],
                      'instance': [1, 0],
                      'attribute': [1],
                      'service': [0x10]
                      }

        # Data
        data = [1, 0, 0, 0]

        # YRC response
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])

        # Confirm successful operation
        return ans_packet.status[0] == 0

    def hold_off(self):
        """See command 3.3.20, page 98"""

        # Sub-header
        sub_header = {'commandNo': [0x83, 0x00],
                      'instance': [1, 0],
                      'attribute': [1],
                      'service': [0x10]
                      }

        # Data
        data = [2, 0, 0, 0]

        # YRC response
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])

        # Confirm successful operation
        return ans_packet.status[0] == 0

    def servo_on(self):
        """See command 3.3.20, page 98"""

        # Sub-header
        sub_header = {'commandNo': [0x83, 0x00],
                      'instance': [2, 0],
                      'attribute': [1],
                      'service': [0x10]
                      }

        # Data
        data = [1, 0, 0, 0]

        # YRC response
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])

        # Confirm successful operation
        return ans_packet.status[0] == 0

    def servo_off(self):
        """See command 3.3.20, page 98"""

        # Sub-header
        sub_header = {'commandNo': [0x83, 0x00],
                      'instance': [2, 0],
                      'attribute': [1],
                      'service': [0x10]
                      }

        # Data
        data = [2, 0, 0, 0]

        # YRC response
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])

        # Confirm successful operation
        return ans_packet.status[0] == 0

    def job_start(self):
        """See command 3.3.23, page 101"""

        # Sub-header
        sub_header = {'commandNo': [0x86, 0x00],
                      'instance': [1, 0],
                      'attribute': [1],
                      'service': [0x10]
                      }

        # Data
        data = [1, 0, 0, 0]

        # YRC response
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])

        # Confirm successful operation
        return ans_packet.status[0] == 0

    def move_link(self, x, y, z, tx, ty, tz, speed):
        """See command 3.3.39, page 128"""

        # Sub-header
        sub_header = {'commandNo': [0x8a, 0x00],
                      'instance': [1, 0],
                      'attribute': [1],
                      'service': [0x02]
                      }

        # Data
        data = [1, 0, 0, 0,                         # Control group (robot)
                0, 0, 0, 0,                         # Control group (station)
                1, 0, 0, 0]                         # Classification of operations
        data += list(struct.pack('=l', speed))      # Speed in 0.1 mm/s and 0.1 degrees/s
        data += [16, 0, 0, 0]               # Operation coordinate 16-base, 17-robot, 18-user, 19-tool
        data += list(struct.pack('=l', x))          # X coordinate in micrometers
        data += list(struct.pack('=l', y))          # Y coordinate
        data += list(struct.pack('=l', z))          # Z coordinate
        data += list(struct.pack('=l', tx))         # Tx coordinate in 0.0001 degrees
        data += list(struct.pack('=l', ty))         # Ty coordinate
        data += list(struct.pack('=l', tz))         # Tz coordinate
        data += [0, 0, 0, 0,                        # Reservation
                 0, 0, 0, 0,                        # Reservation
                 0, 0, 0, 0,                        # Type
                 0, 0, 0, 0,                        # Expanded type
                 0, 0, 0, 0,                        # Tool No
                 1, 0, 0, 0,                        # User coordinate No
                 0, 0, 0, 0,                        # Base 1st axis position
                 0, 0, 0, 0,                        # Base 2nd axis position
                 0, 0, 0, 0,                        # Base 3rd axis position
                 0, 0, 0, 0,                        # Station 1st axis position
                 0, 0, 0, 0,                        # Station 2nd axis position
                 0, 0, 0, 0,                        # Station 3rd axis position
                 0, 0, 0, 0,                        # Station 4th axis position
                 0, 0, 0, 0,                        # Station 5th axis position
                 0, 0, 0, 0                         # Station 6th axis position
                 ]

        # YRC response
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])

        # Confirm successful operation
        return ans_packet.status[0] == 0

    def move_straight(self, x, y, z, tx, ty, tz, speed):
        """See command 3.3.39, page 128"""

        # Sub-header
        sub_header = {'commandNo': [0x8a, 0x00],
                      'instance': [2, 0],
                      'attribute': [1],
                      'service': [0x02]
                      }

        # Data
        data = [1, 0, 0, 0,                         # Control group (robot)
                0, 0, 0, 0,                         # Control group (station)
                1, 0, 0, 0]                         # Classification of operations
        data += list(struct.pack('=l', speed))      # Speed in 0.1 mm/s and 0.1 degrees/s
        data += [16, 0, 0, 0]                       # Operation coordinate
        data += list(struct.pack('=l', x))          # X coordinate in micrometers
        data += list(struct.pack('=l', y))          # Y coordinate
        data += list(struct.pack('=l', z))          # Z coordinate
        data += list(struct.pack('=l', tx))         # Tx coordinate in 0.0001 degrees
        data += list(struct.pack('=l', ty))         # Ty coordinate
        data += list(struct.pack('=l', tz))         # Tz coordinate
        data += [0, 0, 0, 0,                        # Reservation
                 0, 0, 0, 0,                        # Reservation
                 0, 0, 0, 0,                        # Type
                 0, 0, 0, 0,                        # Expanded type
                 0, 0, 0, 0,                        # Tool No
                 1, 0, 0, 0,                        # User coordinate No
                 0, 0, 0, 0,                        # Base 1st axis position
                 0, 0, 0, 0,                        # Base 2nd axis position
                 0, 0, 0, 0,                        # Base 3rd axis position
                 0, 0, 0, 0,                        # Station 1st axis position
                 0, 0, 0, 0,                        # Station 2nd axis position
                 0, 0, 0, 0,                        # Station 3rd axis position
                 0, 0, 0, 0,                        # Station 4th axis position
                 0, 0, 0, 0,                        # Station 5th axis position
                 0, 0, 0, 0                         # Station 6th axis position
                 ]

        # YRC response
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])

        # Confirm successful operation
        return ans_packet.status[0] == 0

    def move_increment(self, x, y, z, tx, ty, tz, speed):
        """See command 3.3.39, page 128"""

        # Sub-header
        sub_header = {'commandNo': [0x8a, 0x00],
                      'instance': [3, 0],
                      'attribute': [1],
                      'service': [0x02]
                      }

        # Data
        data = [1, 0, 0, 0,                         # Control group (robot)
                0, 0, 0, 0,                         # Control group (station)
                1, 0, 0, 0]                         # Classification of operations
        data += list(struct.pack('=l', speed))      # Speed in 0.1 mm/s and 0.1 degrees/s
        data += [16, 0, 0, 0]                       # Operation coordinate
        data += list(struct.pack('=l', x))          # X coordinate in micrometers
        data += list(struct.pack('=l', y))          # Y coordinate
        data += list(struct.pack('=l', z))          # Z coordinate
        data += list(struct.pack('=l', tx))         # Tx coordinate in 0.0001 degrees
        data += list(struct.pack('=l', ty))         # Ty coordinate
        data += list(struct.pack('=l', tz))         # Tz coordinate
        data += [0, 0, 0, 0,                        # Reservation
                 0, 0, 0, 0,                        # Reservation
                 0, 0, 0, 0,                        # Type
                 0, 0, 0, 0,                        # Expanded type
                 0, 0, 0, 0,                        # Tool No
                 1, 0, 0, 0,                        # User coordinate No
                 0, 0, 0, 0,                        # Base 1st axis position
                 0, 0, 0, 0,                        # Base 2nd axis position
                 0, 0, 0, 0,                        # Base 3rd axis position
                 0, 0, 0, 0,                        # Station 1st axis position
                 0, 0, 0, 0,                        # Station 2nd axis position
                 0, 0, 0, 0,                        # Station 3rd axis position
                 0, 0, 0, 0,                        # Station 4th axis position
                 0, 0, 0, 0,                        # Station 5th axis position
                 0, 0, 0, 0                         # Station 6th axis position
                 ]

        # YRC response
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])

        # Confirm successful operation
        return ans_packet.status[0] == 0

    def P_variable_read(self, index=1, log=0):
        """See command 3.3.3.16, page 90"""

        # Sub-header
        sub_header = {'commandNo': [0x7f, 0x00],
                      'instance': [index, 0],
                      'attribute': [0],
                      'service': [0x01]
                      }

        # Data
        data = []

        # YRC response
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        
       

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])
        
        

        #Parse data from answer
        data1 = int.from_bytes(ans_packet.data[0:4],"little")                    #Data Type
        data2 = int.from_bytes(ans_packet.data[4:8],"little")                    #Figure
        data3 = int.from_bytes(ans_packet.data[8:12],"little")                   #Tool number
        data4 = int.from_bytes(ans_packet.data[12:16],"little")                  #User coordinate num
        data5 = int.from_bytes(ans_packet.data[16:20],"little")                  #Extended figure
        data6 = int.from_bytes(ans_packet.data[20:24],"little", signed=True)     #First coordinate data
        data7 = int.from_bytes(ans_packet.data[24:28],"little", signed=True)     #Second coordinate data
        data8 = int.from_bytes(ans_packet.data[28:32],"little", signed=True)     #Third coordinate data
        data9 = int.from_bytes(ans_packet.data[32:36],"little", signed=True)     #Foruth coordinate data
        data10 = int.from_bytes(ans_packet.data[36:40],"little", signed=True)    #Fifth coordinate data
        data11 = int.from_bytes(ans_packet.data[40:44],"little", signed=True)    #Sixth coordinate data   
        data12 = int.from_bytes(ans_packet.data[44:48],"little")
        data13 = int.from_bytes(ans_packet.data[48:52],"little")
        
        
        #supplementary function
        
        def status_of(int_type, offset):
            mask = 1 << offset
            if (int_type & mask) == mask:
                return True
            else:
                return False

        #Data 1 - Data Type
        self.status['Data_type'] = data1

        #Data 2 - Type
        self.status['Front/Back'] = status_of(data2, 0)
       
        self.status['lower/upper arm'] = status_of(data2, 1)
       
        self.status['flip/noflip'] = status_of(data2, 2)
       
        self.status['eR<180'] = status_of(data2, 3)
       
        self.status['eT<180'] = status_of(data2, 4)
       
        self.status['eS<180'] = status_of(data2, 5)
       
        self.status['redudant front/back'] = status_of(data2, 6)
       
        self.status['bit 7'] = status_of(data2, 7)
       

        #Data 3 - Tool Number
        self.status['Tool Number'] = data3
       
        
        #Data 4 - User coordinate number
        self.status['User coordinate number'] = data4
       

        #Data 5 - Extended type
        
        self.status['өL<180'] = status_of(data5, 0)
        self.status['өU<180'] = status_of(data5, 1)
        self.status['өB<180'] = status_of(data5, 2)
        self.status['өE<180'] = status_of(data5, 3)
        self.status['өW<180'] = status_of(data5, 4)
       
            #biti 5,6,7 reserved for further use
        
        #Data 6 - First coordinate data
        self.status['First coordinate data'] = data6/1000

        #Data 7 - Second coordinate data
        self.status['Second coordinate data'] = data7/1000
       
        #Data 8 - Third coordinate data
        self.status['Third coordinate data'] = data8/1000
       
        #Data 9 - Fourth coordinate data
        self.status['Fourth coordinate data'] = data9/10000
       
        #Data 10 - Fifth coordinate data
        self.status['Fifth coordinate data'] = data10/10000
       
        #Data 11 - Sixth coordinate data
        self.status['Sixth coordinate data'] = data11/10000
       
        #Data 12 - Seventh coordinate data
        self.status['Seventh coordinate data'] = data12/1000
       
        #Data 13 - Eighth coordinate data
        self.status['Eighth coordinate data'] = data13/1000

        
        del self.status['Auto']
        del self.status['Alarm']
        del self.status['Cycle']
        del self.status['Error']
        del self.status['Hold_Cmd']
        del self.status['Hold_Ext']
        del self.status['Hold_PP']
        del self.status['InGuard']
        del self.status['Play']
        del self.status['Remote']
        del self.status['Running']
        del self.status['ServoOn']
        del self.status['Step']
        del self.status['Teach']        
        
        #Logging option
        if log >= 1:
            print('\n')
            print('  log P variable')
            print('###################')
            print("#   Tool Num : " + str(self.status['Tool Number']))
            print("#   User Coor. Num : " + str(self.status['User coordinate number']))
            print("#")
            print("#   X  = " + str(format(self.status['First coordinate data'],'.3f')))
            print("#   Y  = " + str(format(self.status['Second coordinate data'],'.3f')))
            print("#   Z  = " + str(format(self.status['Third coordinate data'],'.3f')))
            print("#   Rx = " + str(format(self.status['Fourth coordinate data'],'.4f')))
            print("#   Ry = " + str(format(self.status['Fifth coordinate data'],'.4f')))
            print("#   Rz = " + str(format(self.status['Sixth coordinate data'],'.4f')))
             
            print("#")
            print("#   FRONT = ", end='')
            print("өS >= 180") if self.status["eS<180"] == True else print("eS < 180")
            print("#   UP    = ", end='')
            print("өR >= 180") if self.status["eR<180"] == True else print("eR < 180")
            print("#   FLIP  = ", end='')
            print("өT >= 180") if self.status["eT<180"] == True else print("eT < 180")
            print('###################')
            print('\n')


        
        return self.status
   
    def P_Variable_write(self, data_type, x, y, z, Rx, Ry, Rz, Tool_num, User_coor_num, Front, Up, Flip, index=1):
        """See command 3.3.3.16, page 90"""
        """mistake in PDF !!! Data type 
                                        18: Tool coorinated value
                                        19: User coordinated value
        """
        # Sub-header
        sub_header = {'commandNo': [0x7f, 0x00],
                      'instance': [index, 0],
                      'attribute': [0],
                      'service': [0x02]
                      }
        
        
        def change_bit(bytee, bit, status):
            #input bytee, bit - position, status 0/1
            maskk = 0
            if status == 1:
                maskk = 1 << bit
                return (bytee | maskk)
            else:
                maskk = ~(1 << bit)
                return(bytee & maskk)
        
        Figure = 0
        Extended_Figure = 0
        Figure = change_bit(Figure, 3, Up)
        Figure = change_bit(Figure, 4, Flip)
        Figure = change_bit(Figure, 5, Front)
        
                # Data
        data = []
        data += list(struct.pack('l',data_type))       # Data type
        data += list(struct.pack('l',Figure))          # Figure
        data += list(struct.pack('l',Tool_num))        # Tool number
        data += list(struct.pack('l',User_coor_num))      # User coordinae number
        data += list(struct.pack('l',Extended_Figure))    # Extended figure
        data += list(struct.pack('=l', (int)(x*1000)))          # First coordinate data
        data += list(struct.pack('=l', (int)(y*1000)))          # Second coordinate data
        data += list(struct.pack('=l', (int)(z*1000)))          # Third coordinate data
        data += list(struct.pack('=l', (int)(Rx*10000)))        # Fourth coordinate data
        data += list(struct.pack('=l', (int)(Ry*10000)))        # Fifth coordinate data
        data += list(struct.pack('=l', (int)(Rz*10000)))        # Sixth coordinate data
        data += [0, 0, 0, 0,                                    # Seventh coordinate data
                 0, 0, 0, 0]                                    # Eighth coordinate data
        
        #print("izpis sendanih podatkov")

        
        #pprint.pprint(data, width=1200)
        #print("konec izpisa")

        # YRC response
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])

        # Confirm successful operation
        return ans_packet.status[0] == 0
    
    def job_select(self, job_name, line_number):
        """See command 3.3.3.24, page 102"""

        # Sub-header
        sub_header = {'commandNo': [0x87, 0x00],
                      'instance': [1, 0],
                      'attribute': [0],
                      'service': [0x02]
                      }
        data = []
        
        for i in job_name :
            #print(i)
            data += list(struct.pack('c', i.encode()))
        
        #fill in rest of bytes with 0
        for i in range(len(job_name), 36):
            data+=[0]

        
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])

        # Confirm successful operation
        return ans_packet.status[0] == 0
        
    def Job_start(self,):
        """See command 3.3.3.23, page 101"""

        # Sub-header
        sub_header = {'commandNo': [0x86, 0x00],
                      'instance': [1, 0],
                      'attribute': [1],
                      'service': [0x10]
                      }
        data = []
        data += list(struct.pack('l', 1))
        
        ans_packet = self.yrc_response_to_packet(sub_header, data)

        # Handle error
        if ans_packet is None:
            return None
        elif ans_packet.status[0] != 0:
            return ans_packet.status[0], hex(struct.unpack('=H', bytes(ans_packet.added_status))[0])

        # Confirm successful operation
        return ans_packet.status[0] == 0
        
        
        
########################################################################################################################
# Testing

#if __name__ == '__main__':

    # yrc = ClientOfYRC('localhost')
#    yrc = ClientOfYRC('192.168.255.1')

#    print('Response: ', yrc.status_information_reading(), '\n')
#    print('Response: ', yrc.real_type_variable_write(3.14, index=1), '\n')
#    print('Response: ', yrc.servo_on(), '\n')
