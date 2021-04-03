#
# *********     Read and Write Example      *********
#
#
# Available DXL model on this example : All models using Protocol 1.0
# This example is tested with a DXL MX-28, and an USB2DYNAMIXEL
# Be sure that DXL MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
#


from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_AX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_AX_GOAL_POSITION      = 30
ADDR_AX_PRESENT_POSITION   = 36

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 1                # Dynamixel ID : 1
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/tty.usbserial-FT4TFQEI'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 450           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 550            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position


def enable_torque(DXL_ID):
    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel has been successfully connected")
    return dxl_error

def disable_torque(DXL_ID):
    # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_AX_TORQUE_ENABLE,
                                                              TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    return dxl_error


def set_goal_position (DXL_ID, goal_position):
    # Write goal position
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_GOAL_POSITION, goal_position)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    return dxl_error

def get_present_position (DXL_ID):
    # Read present position
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID,
                                                                                   ADDR_AX_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    return dxl_present_position, dxl_error

# returns the value according to the degrees
def degrees_to_value(d):
    value = (150 + d) * 1023 / 300
    return round(value)

def setJoints(i,Q):
    if i == 1 or i == 3:
        Q = [-Q[0], -Q[1], -Q[2]]
    elif i == 2 or i == 4:
        Q = [Q[0], -Q[1], -Q[2]]

    # idx of firts joint of the leg
    i = (i-1)*3
    dxl_error = set_goal_position(i + 1,degrees_to_value(Q[0]))
    dxl_error = set_goal_position(i + 2,degrees_to_value(Q[1]))
    dxl_error = set_goal_position(i + 3,degrees_to_value(Q[2]))

    return dxl_error

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

enable_torque(10)
enable_torque(11)
enable_torque(12)

print("Press any key to continue! (or press ESC to quit!)")

#dxl_error = set_goal_position(i + 1,degrees_to_value(Q[0]))
setJoints(4,[30,-25,35])

disable_torque(10)
disable_torque(11)
disable_torque(12)


# Close port
portHandler.closePort()