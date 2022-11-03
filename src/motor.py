import math
import os
from os.path import join

from dataclasses import dataclass
from dynamixel_sdk import *

# dataname:(address,size)
ADDR_TABLE={
    "Model Number":(0,2),
    "Model Information":(2,4),
    "Firmware Version":(6,1),
    "ID":(7,1),
    "Baud Rate":(8,1),
    "Return Delay Time":(9,1),
    "Drive Mode":(10,1),
    "Operating Mode":(11,1),
    "Secondary(Shadow) ID":(12,1),
    "Protocol Type":(13,1),
    "Homing Offset":(20,4),
    "Moving Threshold":(24,4),
    "Temperature Limit":(31,1),
    "Max Voltage Limit":(32,2),
    "Min Voltage Limit":(34,2),
    "PWM Limit":(36,2),
    "Velocity Limit":(44,4),
    "Max Position Limit":(48,4),
    "Min Position Limit":(52,4),
    "Startup Configuration":(60,1),
    "Shutdown":(63,1),
    "Torque Enable":(64,1),
    "LED":(65,1),
    "Status Return Level":(68,1),
    "Registered Instruction":(69,1),
    "Hardware Error Status":(70,1),
    "Velocity I Gain":(76,2),
    "Velocity P Gain":(78,2),
    "Position D Gain":(80,2),
    "Position I Gain":(82,2),
    "Position P Gain":(84,2),
    "Feedforward 2nd Gain":(88,2),
    "Feedforward 1st Gain":(90,2),
    "Bus Watchdog":(98,1),
    "Goal PWM":(100,2),
    "Goal Velocity":(104,4),
    "Profile Acceleration":(108,4),
    "Profile Velocity":(112,4),
    "Goal Position":(116,4),
    "Realtime Tick":(120,2),
    "Moving":(122,1),
    "Moving Status":(123,1),
    "Present PWM":(124,2),
    "Present Load":(126,2),
    "Present Velocity":(128,4),
    "Present Position":(132,4),
    "Velocity Trajectory":(136,4),
    "Position Trajectory":(140,4),
    "Present Input Voltage":(144,2),
    "Present Temperature":(146,1),
    "Backup Ready":(147,1),
}

@dataclass
class dynamixel_device:
    port_handler:PortHandler
    # only protocal 2 is used
    packet_handler:Protocol2PacketHandler

# read data from the specific address defined by the data_name
# returns the read result on success, None on fail
def read_dynamixel(device:dynamixel_device, data_name, device_id=1):
    size=ADDR_TABLE[data_name][1]
    if(size==1):
        dxl_read_result, dxl_comm_result, dxl_error=device.packet_handler.read1ByteTxRx(device.port_handler,device_id,ADDR_TABLE[data_name][0])
    elif(size==2):
        dxl_read_result, dxl_comm_result, dxl_error=device.packet_handler.read2ByteTxRx(device.port_handler,device_id,ADDR_TABLE[data_name][0])
    else:
        dxl_read_result, dxl_comm_result, dxl_error=device.packet_handler.read4ByteTxRx(device.port_handler,device_id,ADDR_TABLE[data_name][0])
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % device.packet_handler.getTxRxResult(dxl_comm_result))
        return None
    elif dxl_error != 0:
        print("%s" % device.packet_handler.getRxPacketError(dxl_error))
        return None
    
    return dxl_read_result

# write data to the specific address defined by the data_name
# returns True on success, False on fail
def write_dynamixel(device:dynamixel_device, data_name, data, device_id=1):
    size=ADDR_TABLE[data_name][1]
    if(size==1):
        dxl_comm_result, dxl_error=device.packet_handler.write1ByteTxRx(device.port_handler,device_id,ADDR_TABLE[data_name][0],data)
    elif(size==2):
        dxl_comm_result, dxl_error=device.packet_handler.write2ByteTxRx(device.port_handler,device_id,ADDR_TABLE[data_name][0],data)
    else:
        dxl_comm_result, dxl_error=device.packet_handler.write4ByteTxRx(device.port_handler,device_id,ADDR_TABLE[data_name][0],data)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % device.packet_handler.getTxRxResult(dxl_comm_result))
        return False
    elif dxl_error != 0:
        print("%s" % device.packet_handler.getRxPacketError(dxl_error))
        return False
    return True

def reboot_dynamixel(device:dynamixel_device, device_id=1):
    device.packet_handler.reboot(device.port_handler,device_id)

def initialize(port=0):
    if(port==-1):
        # enumerate all usb devices and find the first device with ID "0403:6014"
        # the function below is from https://stackoverflow.com/a/10030819
        def find_tty_usb(idVendor, idProduct):
            for dnbase in os.listdir('/sys/bus/usb/devices'):
                dn = join('/sys/bus/usb/devices', dnbase)
                if not os.path.exists(join(dn, 'idVendor')):
                    continue
                idv = open(join(dn, 'idVendor')).read().strip()
                if idv != idVendor:
                    continue
                idp = open(join(dn, 'idProduct')).read().strip()
                if idp != idProduct:
                    continue
                for subdir in os.listdir(dn):
                    if subdir.startswith(dnbase+':'):
                        for subsubdir in os.listdir(join(dn, subdir)):
                            if subsubdir.startswith('ttyUSB'):
                                return join('/dev', subsubdir)
            return None
        device_name=find_tty_usb('0403','6014')
    else:
        device_name = '/dev/ttyUSB'+str(port)
        
    if(not device_name):
        print("unable to find device!")
        return None

    # open port and set baud rate to match the default baud rate on the device
    portHandler = PortHandler(device_name)
    if not portHandler.openPort():
        print("failed to open the port!")
        return None
    if not portHandler.setBaudRate(57600):
        print("failed to set baud rate!")
        return None

    packetHandler = PacketHandler(2.0)

    device=dynamixel_device(portHandler,packetHandler)

    # check if the device is 'XL430-W250-T'
    model_number=read_dynamixel(device,"Model Number")
    if(model_number!=1060):
        print("un-supported device!")
        return None
    return device

def finalize(device:dynamixel_device):
    if(not device):
        return
    # disable torque
    write_dynamixel(device,"Torque Enable",0)
    # close port
    device.port_handler.closePort()

def set_velocity_rads(device:dynamixel_device, velocity):
    vel=int(velocity*60/0.229)
    if(vel==0 and velocity!=0):
        vel=1
    return write_dynamixel(device,"Profile Velocity",vel)

def set_goal_rad(device:dynamixel_device, goal):
    return write_dynamixel(device,"Goal Position",int(goal*4096/(2*math.pi))%4096)

if __name__ == "__main__":
    initialize(-1)