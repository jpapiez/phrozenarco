####################################
#Project Name: 
#Chip Type: 
#Function: 
# Developer: Lan Caigang
#Development Date: 20230830
####################################

import os
import numpy as np

import logging
import json
import time
import serial
from .base import *




# c class type number data
from ctypes import *


# （2）Python class up can use use use, because is to class and to, code show down show :
# class item:
#     def __init__(self):
#         self.name = ''
#         self.size = 10
#         self.list = []
# from up show can output class is full, in class change variable, change variable is class 。 and and change variable number data class type is 、 number and list table, normal is number data class type 。
# class can number value set has and action use to, code show down show :
# a = item()
# a.name = 'cup'
# a.size = 8
# a.list.append('water')


# number
# in C canstruct class type, data internal empty, use internal, because therefore can address number 。 and C, in NumPy to number action 。 to NumPy and C, NumPycan address readC number number data, switch is NumPy number 。need to number, hasname, age and salary。 in NumPy can down :
# import numpy as np
# MyType=np.dtype({
#     'names':['name','age','salary'],
# 'formats':['S32','i','f']# add s, and S write
# })
# a=np.array([("tang",23,130.2),("wang",22,100.2)],
# dtype=MyType)
# # or Data=np.array([(‘zero’,0.,0.)]*10,dtype=MyType) #createData[2]
# #Date[0]['name']="tang"

# in Python use use c_typewhen output little-endian
# item project need to and bottom layer signal number data, is therefore use use Pythonc_type class internal,, number data down,
# name length(bytes)
# command 2
# class type 1
# number data 1 4
# number data 2 2
# Python,
# class EProfile(Structure):
#     _pack_ = 1
#     _fields_ = [('command_id', c_short),
#                 ('type', c_ubyte),
#                 ('data1', c_int),
#                 ('data2', c_short)]
# c_short is 2, c_ubyte is 1, c_int is 4。but is number data, switch is byte streamafterdetected, is use little-endian。, command value if is 1000, switch 2hex number is 0x03E8, byte stream input output is 0xE803。
# : set class StructureBigEndianStructure。


# union in internal has internal empty, emptyunion number data class type, union in initializewhen, union value, after has parameter number
# from ctypes import *
# print "aaa:"
# value = raw_input()
# v=int(value)
# vv=long(value)
# vvv=value
# class aaa(Union):
#     _fields_=[
#        ("aaa",c_int),
#        ("bbb",c_long),
#        ("ccc",c_char),       ]
# print "aaaaaaa:%s" %value
# a=aaa(v,vv,vvv)
# print "aaa: %d" %a.aaa
# print "bbb: %ld" %a.bbb
# print "ccc: %s" %a.ccc
# test1
# c:\Python27>python D:\jincheng\workspace\GrayHatPython\chapter1.py
# aaa:
# 6
# aaaaaaa:6
# aaa: 54
# bbb: 54
# ccc: 6
# test2
# c:\Python27>python D:\jincheng\workspace\GrayHatPython\chapter1.py
# aaa:
# 66
# aaaaaaa:66
# aaa: 13878
# bbb: 13878
# ccc: 66
# modify
# from ctypes import *
# print "aaa:"
# value = raw_input()
# v=int(value)
# print "v %d" %v
# print "bbb:"
# val=raw_input()
# vv=long(val)
# vvv=value
# class aaa(Union):
#     _fields_=[
#        ("aaa",c_int),
#        ("bbb",c_long),
#        ("ccc",c_char * 6),       ]
# print "aaaaaaa:%s" %value
# a=aaa(v,vv)
# s=a.aaa
# ss=int(s)
# print "ss %d" %ss
# print "aaa: %d" %a.aaa
# print "bbb: %ld" %a.bbb
# print "ccc: %s" %a.ccc
# test
# D:\Python27>python d:\demo.py
# aaa:
# 66
# v 66
# bbb:
# 55
# aaaaaaa:66
# ss 55
# aaa: 55
# bbb: 55
# ccc: 7


# // to signal info
# typedef struct St_SystemSimpleStatus {
# // info (CMD_RSP_SYSTEM_STATE)
#     uint8_t InfoFlag;
# // current ID
#     int CurrentDeviceId;
# // after ID, has？ unit network
#     int EndDeviceId;
# // mode(multi-materialmode: 00 refillmode: 01)
#     uint8_t DeviceMode;
# // multi-materialmode down state code
#     uint8_t MCStateMachine : 4;
# // refillmode down state code
#     uint8_t MAStateMachine : 4;
# } St_SystemSimpleStatus;
####################################
#Class Name: 
# Description: Lan Caigang-20230830
####################################
class AMSSimpleInfoSt(LittleEndianStructure):# little-endian mode
    _pack_ = 1
    _fields_ = [
        ("info_flag", c_uint8),# 8bit; flag
        ("dev_id", c_uint8),# 8bit; multi-materialid
        ("end_dev_id", c_uint8),# 8bit; after multi-materialid
        ("dev_mode", c_uint8),# 8bit; multi-materialmode
        ("mc_state", c_uint8, 4),# 4bit; mcstate
        ("ma_state", c_uint8, 4),# 4bit; mastate
    ]

####################################
#Class Name: 
# Description: Lan Caigang-20230830
####################################
class AMSSimpleInfoBytes(Union):# multi-materialmainboardstate
    _fields_ = [
        ("field", AMSSimpleInfoSt),
        ("whole", c_uint8 * sizeof(AMSSimpleInfoSt)),
    ]

# // signal info
# typedef struct St_SystemDetailStatus {
# // info (CMD_RSP_SYSTEM_STATE)
#     uint8_t InfoFlag;
# // current ID
#     int8_t CurrentDeviceId;
# // after ID, has？ unit network
#     int8_t EndDeviceId;
# // move ID, #？ unit is current move
#     int8_t ActiveDeviceId;
# // project ID, #？ unit is need toto
#     int8_t TargetDeviceId;
# // (use keep reserve)
#     uint8_t Others;
# // mode(multi-materialmode: 00 refillmode: 01)
#     uint8_t DeviceMode : 2;
# // machine in (full all stop : 0)
#     uint8_t IfAnyMotorRuning : 1;
# // bufferempty(send : 1 send : 0)
#     uint8_t CacheEmptyIfTrigger : 1;
# // bufferfull(send : 1 send : 0)
#     uint8_t CacheFullIfTrigger : 1;
# // bufferexists(send : 1 send : 0)
#     uint8_t CacheExistIfTrigger : 1;
# // keep reserve
#     uint8_t Reserve : 2;
# // multi-materialmode down state code
#     uint8_t MCStateMachine : 4;
# // refillmode down state code
#     uint8_t MAStateMachine : 4;
# // entry device state(bit is table state, send : 1 send : 0)
#     uint32_t EntryPositionIfTriggerBitMask;
# // park position device state(bit is table state, send : 1 send : 0)
#     uint32_t ParkPositionIfTriggerBitMask;
# } St_SystemDetailStatus;
####################################
#Class Name: 
# Description: Lan Caigang-20230830
####################################
class AMSDetailInfoSt(LittleEndianStructure):# little-endian mode
    _pack_ = 1
    _fields_ = [
        ("info_flag", c_uint8),# 8bit; flag
        ("dev_id", c_uint8),#8bit; 
        ("end_dev_id", c_int8),#8bit; 
        ("active_dev_id", c_int8),#8bit; 
        ("target_dev_id", c_int8),#8bit; 
        ("others", c_uint8),#8bit; 

        ("dev_mode", c_uint8, 2),# 2bit; multi-materialmode
        ("any_motor_runing", c_uint8, 1),# 1bit; # is no has machine in
        ("cache_empty", c_uint8, 1),# 1bit; #bufferemptystate
        ("cache_full", c_uint8, 1),# 1bit; #bufferfullstate
        ("cache_exist", c_uint8, 1),# 1bit; #bufferexistsstate
        ("reserve", c_uint8, 2),#2bit; 

        ("mc_state", c_uint8, 4),# 4bit; #mcstate
        ("ma_state", c_uint8, 4),# 4bit; #mastate

        ("entry_state", c_uint32),# 32bit; #feedentrystate
        ("park_state", c_uint32),# 32bit; #park positionstate
    ]

####################################
#Class Name: 
# Description: Lan Caigang-20230830
####################################
class AMSDetailInfoBytes(Union):# multi-materialmainboardstate
    _fields_ = [
        ("field", AMSDetailInfoSt),
        ("whole", c_uint8 * sizeof(AMSDetailInfoSt)),
    ]

####################################
#Class Name: 
# Description: Lan Caigang-20230830
####################################
class Commands(Base):
    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    #Initialize constructor
    def __init__(self, config):
        super(Commands, self).__init__(config)

        # ttyserial port is no connect
        self.G_SerialPort1OpenFlag = False

        # ttyserial port is no connect
        self.G_SerialPort2OpenFlag = False


        # MC first time
        self.G_ChangeChannelFirstFilaFlag = True
        # toolhead first time feed
        self.G_ToolheadFirstInputFila = False  # first feed
        # klipper is no pause
        self.G_KlipperIfPaused = False
        # move move because
        self.G_MovementSpeedFactor = 1.0 / 60
        # toolhead after position
        self.G_ToolheadLastPosition = None
        # AMS action mode
        self.G_AMSDeviceWorkMode = AMS_WORK_MODE_UNKNOW  # default action modeUnknown mode

        # 1、python () table show, is change sequence list
        # 1）create: tuple = (1,2,3) get number data tuple[0]...... tuple[0,2].....tuple[1,2]......
        # 2)modify: is modify
        # 3）delete del tuple
        # 4） internal set number :
        # cmp（tuple1, tuple2）: compare
        # len(tuple):length
        # max（tuple）: value
        # min（tuple）: value
        # tuple（seq）: set list table switch is
        # 2、python [] table show list table, list table is change sequence list
        # 1）create list table l = [1,2,3,4] get number data l[0]........
        # 2) list table modify
        # 3） internal set number
        # cmp（list1, list2）: compare
        # len(list):length
        # max（list）: value
        # min（list）: value
        # list（seq）: set switch is list table
        # list.append(obj): in list table new to
        # list.pop(): move number data
        # list.remove: move list table # value
        # list.sort(): sequence
        # list.reverse(): switch list table
        # list.count(bj): to in list table output time number
        # list.insert(index,obj) : in position to
        # 3、python {}; is change device, use use compare
        # 1）create: dict = {"a":1,"b":2}. is to : key, value value to get number data dict['a'],
        # 2）modify
        # 3）delete: del dict["a"] delete to number data del dict delete dict.clear()all project
        # 4） internal set number
        # cmp（dict1, dict2）: compare
        # len(dict):length
        # dict.clear():delete number data
        # dict.get(key, default=None): value, ifnodefault value
        # dict.has_key(key): value is no exists, true, false
        # dict.item（） list table value （, value ）
        # dict.key（）allkey value

        # python
        #P9 X190.290 Y238.700 W2.010 H11.200 D1
        # toolhead in filamentwait parameter number
        self.G_DictChangeChannelWaitAreaParam = {# python {} key-value value to
            "T": self.G_ChangeChannelTimeout,     # timeoutwhen (), default120
            "A": 0,         # move action Action
            "D": 0,         # defaultX or Y
            "X": 0.0,       # waitX
            #lancaigang20231020: 
            "Y": 20.0,      # waitY
            "W": 0.0,       # waitX
            "H": 0.0,       # waitY
        }



        # down parameter number in connect value
        # toolhead
        self.G_ProzenToolhead = None
        #manual toolhead movement
        self.G_ToolheadManualMovement = None
        # Wait for toolhead movement
        self.G_ToolheadWaitMovementEnd = None


        # lancaigang231115: timeout, add Function
        # lancaigang231216: default use use channel0, unknownchannel
        self.G_ChangeChannelTimeoutOldChan=-1
        self.G_ChangeChannelTimeoutOldGcmd=None
        # lancaigang240912:new AMS, use use new old channel
        self.G_ChangeChannelTimeoutNewChan=-1
        self.G_ChangeChannelTimeoutNewGcmd=None


        # lancaigang231206: During resumenot allowedpause
        self.G_ChangeChannelResumeFlag = False

        # path
        self.ChangeWaitMoveArea = []  # pathqueue


        # lancaigang231216: waitXY
        self.G_XBasePosition=0
        self.G_YBasePosition=0

        # lancaigang231216: ifpause, z, to pausewhen, set z keep, error
        self.G_IfZPositionLiftUpFlag = False

        #lancaigang231219:
        self.G_SerialPort1Obj=None

        #lancaigang241029:
        self.G_SerialPort2Obj=None

        #lancaigang241030:
        self.G_SerialPortHaveOpenedCount=0
        self.G_SerialPortIsOpenCount=0







    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_MoveTo(self, pos, velocity):
        #Unable to get the toolhead object
        if self.G_ProzenToolhead is None:
            return

        #Wait for toolhead movement
        self.G_ProzenToolhead.wait_moves()
        #Get the last toolhead position
        self.G_ToolheadLastPosition = self.G_ProzenToolhead.get_position()

        for index, p in enumerate(pos):
            self.G_ToolheadLastPosition[index] = p

        #manual toolhead movement
        self.G_ProzenToolhead.manual_move(self.G_ToolheadLastPosition, velocity * self.G_MovementSpeedFactor)
        #Wait for toolhead movement
        self.G_ProzenToolhead.wait_moves()
    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    #Send via serial without waiting for a response
    def Cmds_AMSSerial1Send(self, cmd):
        if self.G_SerialPort1OpenFlag==False:
            self.G_PhrozenFluiddRespondInfo("[WARN] AMS1/tty1 not connected; send P28 first.")
            try:
                self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.py)Cmds_AMSSerial1Send]Reinitializing serial port 1")
                self.G_SerialPort1Obj = serial.Serial(self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3)
                #Serial port opened successfully
                if self.G_SerialPort1Obj is not None:
                    if self.G_SerialPort1Obj.is_open:
                        self.G_SerialPort1OpenFlag = True
                        self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 1successful")
                        # lancaigang231213: openserial port
                        self.G_SerialPort1Obj.flushInput()  # clean serial write cache
                        self.G_SerialPort1Obj.flush()
                        self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 buffers cleared")
                        self.G_PhrozenFluiddRespondInfo("[INFO] Re-registering serial port 1 callback")
                        self.G_SerialPort1RecvTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerUart1Recv, self.G_PhrozenReactor.NOW)
            except:
                self.G_PhrozenFluiddRespondInfo("[ERROR] Unable to open tty1. Check the USB connection or try rebooting.")
            return

        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_AMSSerial1Send]Sending command: cmd=%s" % cmd)

        try:
            self.G_PhrozenFluiddRespondInfo("[INFO] before sending, Reinitializing serial port 1")
            self.G_SerialPort1Obj = serial.Serial(self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3)
            #Serial port opened successfully
            if self.G_SerialPort1Obj is not None:
                if self.G_SerialPort1Obj.is_open:
                    # tty1Serial sendcommand
                    #Lock
                    if self.Base_AMSSerialCmdLock():
                        self.G_SerialPort1Obj.flush()
                        #tty1Serial send
                        self.G_SerialPort1Obj.write(cmd.encode())#.encode()
                        self.G_PhrozenFluiddRespondInfo("[DEBUG] self.G_SerialPort1Obj.write")
                        self.G_SerialPort1Obj.flush()
                        #Unlock
                        self.Base_AMSSerialCmdUnlock()
                        
        except:
            self.G_PhrozenFluiddRespondInfo("[ERROR] Unable to open tty1. Check the USB connection or try rebooting.")
            #Unlock
            self.Base_AMSSerialCmdUnlock()



    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_AMSSerial2Send(self, cmd):
        if self.G_SerialPort2OpenFlag==False:
            self.G_PhrozenFluiddRespondInfo("[DEBUG] AMS2/tty2 not available; skipping serial port 2.")
            try:
                self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.py)Cmds_AMSSerial2Send]Reinitializing serial port 2")
                self.G_SerialPort2Obj = serial.Serial(self.G_Serialport2Define, SERIAL_PORT_BAUD, timeout=3)
                #Serial port opened successfully
                if self.G_SerialPort2Obj is not None:
                    if self.G_SerialPort2Obj.is_open:
                        self.G_SerialPort2OpenFlag = True
                        self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 2successful")
                        self.G_SerialPort2Obj.flushInput()  # clean serial write cache
                        self.G_SerialPort2Obj.flush()
                        self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 buffers cleared")
                        self.G_PhrozenFluiddRespondInfo("[INFO] Re-registering serial port 2 callback")
                        self.G_SerialPort2RecvTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerUart2Recv, self.G_PhrozenReactor.NOW)
            except:
                self.G_PhrozenFluiddRespondInfo("[DEBUG] AMS2/tty2 not available; skipping serial port 2.")
            return
        
        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_AMSSerial2Send]Sending command: cmd=%s" % cmd)


        try:
            self.G_PhrozenFluiddRespondInfo("[INFO] before sending, Reinitializing serial port 2")
            self.G_SerialPort2Obj = serial.Serial(self.G_Serialport2Define, SERIAL_PORT_BAUD, timeout=3)
            #Serial port opened successfully
            if self.G_SerialPort2Obj is not None:
                if self.G_SerialPort2Obj.is_open:
                    # tty1Serial sendcommand
                    #Lock
                    if self.Base_AMSSerialCmdLock():
                        self.G_SerialPort2Obj.flush()
                        #tty1Serial send
                        self.G_SerialPort2Obj.write(cmd.encode())#.encode()
                        self.G_PhrozenFluiddRespondInfo("[DEBUG] self.G_SerialPort2Obj.write")
                        self.G_SerialPort2Obj.flush()
                        #Unlock
                        self.Base_AMSSerialCmdUnlock()
                        self.G_PhrozenFluiddRespondInfo("[DEBUG] self.G_SerialPort2Obj.write")
        except:
            self.G_PhrozenFluiddRespondInfo("[DEBUG] AMS2/tty2 not available; skipping serial port 2.")
            #Unlock
            self.Base_AMSSerialCmdUnlock()




    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    #Send via serial and wait for a response
    def Cmds_AMSSerialPort1SendWaitRsp(self, cmd, res_len):
        if self.G_SerialPort1OpenFlag==False:
                self.G_PhrozenFluiddRespondInfo("[WARN] AMS1/tty1 not connected; send P28 first.")
                return
        
        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_AMSSerialPort1SendWaitRsp]Sending command: cmd=%s" % cmd)
        
            
        try:
            #Serial port opened successfully
            if self.G_SerialPort1Obj is not None:
                if self.G_SerialPort1Obj.is_open:
                    # get command
                    #Lock
                    if self.Base_AMSSerialCmdLock():
                        # tty1Serial sendbyte stream
                        self.G_SerialPort1Obj.write(cmd.encode())
                        self.G_PhrozenFluiddRespondInfo("[DEBUG] self.G_SerialPort1Obj.write")
                        self.G_SerialPort1Obj.flush()
                        #tty1Read byte stream from serial
                        resp = self.G_SerialPort1Obj.read(res_len)
                        #Unlock
                        self.Base_AMSSerialCmdUnlock()
                        return resp
        except:
            self.G_PhrozenFluiddRespondInfo("[ERROR] Unable to open tty1. Check the USB connection or try rebooting.")
            #Unlock
            self.Base_AMSSerialCmdUnlock()
        
    
    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    #Send via serial and wait for a response
    def Cmds_AMSSerialPort2SendWaitRsp(self, cmd, res_len):
        if self.G_SerialPort2OpenFlag==False:
                self.G_PhrozenFluiddRespondInfo("[DEBUG] AMS2/tty2 not available; skipping serial port 2.")
                return
        
        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_AMSSerialPort2SendWaitRsp]Sending command: cmd=%s" % cmd)

        # # get command
        # #Lock
        # if self.Base_AMSSerialCmdLock():
        # #tty2Serial sendbyte stream
        #     self.G_SerialPort2Obj.write(cmd.encode())
        #     self.G_SerialPort2Obj.flush()
        #     #tty2Read byte stream from serial
        #     resp = self.G_SerialPort2Obj.read(res_len)
        #     #Unlock
        #     self.Base_AMSSerialCmdUnlock()
        #     return resp
        try:
            #Serial port opened successfully
            if self.G_SerialPort2Obj is not None:
                if self.G_SerialPort2Obj.is_open:
                    # get command
                    #Lock
                    if self.Base_AMSSerialCmdLock():
                        # tty2Serial sendbyte stream
                        self.G_SerialPort2Obj.write(cmd.encode())
                        self.G_PhrozenFluiddRespondInfo("[DEBUG] self.G_SerialPort2Obj.write")
                        self.G_SerialPort2Obj.flush()
                        #tty2Read byte stream from serial
                        resp = self.G_SerialPort2Obj.read(res_len)
                        #Unlock
                        self.Base_AMSSerialCmdUnlock()
                        return resp
        except:
            self.G_PhrozenFluiddRespondInfo("[DEBUG] AMS2/tty2 not available; skipping serial port 2.")
            #Unlock
            self.Base_AMSSerialCmdUnlock()
    
    

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # P1 E[n]; n:1~32(no network, get 1~4); channelfilament before switch, need to get output toolhead up filament tube Yes; ====="E?";
    def Cmds_P1EnForceForward(self, chan, gcmd):
        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1EnForceForward]Sending command: E%d" % chan)

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+E:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()
        #lancaigang241030:
        if chan in range(1, 5):
            self.Cmds_AMSSerial1Send("E%d" % chan)
            self.G_PhrozenFluiddRespondInfo("Serial port 1 sending command: E%d" % chan)
        elif chan in range(5, 9):
            self.Cmds_AMSSerial2Send("E%d" % (chan-4))
            self.G_PhrozenFluiddRespondInfo("Serial port 2 sending command: E%d" % (chan-4))


        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+E:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # lancaigang240228: toolheadretract, need tostm32first
    # P1 G[n]; n:1~32(no network, get 1~4); channelfilament Yes; ====="G?";
    def Cmds_P1GnExtruderBack(self, chan, gcmd):
        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1EnForceForward]Sending command: G%d" % chan)

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+G:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()
        #lancaigang241030:
        if chan in range(1, 5):
            self.Cmds_AMSSerial1Send("G%d" % chan)
            self.G_PhrozenFluiddRespondInfo("Serial port 1 sending command: G%d" % chan)
        elif chan in range(5, 9):
            self.Cmds_AMSSerial2Send("G%d" % (chan-4))
            self.G_PhrozenFluiddRespondInfo("Serial port 2 sending command: G%d" % (chan-4))


        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+G:1,%d" % self.G_ChangeChannelTimeoutNewChan)
    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_P1HnSpecialInfila(self, chan, gcmd):
        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1HnSpecialInfila]Sending command: H%d" % chan)

         # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+H:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()
        #lancaigang241030:
        if chan in range(1, 5):
            self.Cmds_AMSSerial1Send("H%d" % chan)
            self.G_PhrozenFluiddRespondInfo("Serial port 1 sending command: H%d" % chan)
        elif chan in range(5, 9):
            self.Cmds_AMSSerial2Send("H%d" % (chan-4))
            self.G_PhrozenFluiddRespondInfo("Serial port 2 sending command: H%d" % (chan-4))


        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+H:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_P1InExtrudeManualIn(self, value):
        command_string = """
                        M106 S0
                        M83
                        G92 E0
                        G1 E%f F300
                        """ % (
                        value,
                    )
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1InExtrudeManualIn]G-code command: %s" % command_string)


    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # =====P1 J[n]; multi-materialmanualpurge; bufferfullwhen refill;
    def Cmds_P1JnManualSpitFila(self,chan, gcmd):
        self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_P1JnManualSpitFila]Sending command P1J?")
        self.G_PhrozenFluiddRespondInfo("chan=%d;" % chan)
        self.G_PhrozenFluiddRespondInfo("+J:0,%d" % self.G_ChangeChannelTimeoutNewChan)


        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()
        #lancaigang241030:
        if chan in range(1, 5):
            self.Cmds_AMSSerial1Send("J%d" % chan)
            self.G_PhrozenFluiddRespondInfo("Serial port 1 sending command: J%d" % chan)
        elif chan in range(5, 9):
            self.Cmds_AMSSerial2Send("J%d" % (chan-4))
            self.G_PhrozenFluiddRespondInfo("Serial port 2 sending command: J%d" % (chan-4))




        self.G_PhrozenFluiddRespondInfo("+J:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # =====P1 I[n]; manualextrudewhen stm32need torefill; ====="I?"; ?-extrude or retract
    # I2 table show extrude, I3 table show retract, I0 table show idle
    def Cmds_P1InExtruderBack(self, value, gcmd):
        self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_P1InExtruderBack]Sending command I?")
        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()
         # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+I:0,%d" % self.G_ChangeChannelTimeoutNewChan)

        # lancaigang0415: I2 table show extrude, I3 table show retract
        if value>0:
            self.G_PhrozenFluiddRespondInfo("[INFO] Sending command: I2; extrude")

            #lancaigang241031:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("I2")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: I2")
            elif self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("I2")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: I2")

            # lancaigang240516: preventtime too close
            # lancaigang240705: prevent time; or time too close
            self.G_ProzenToolhead.dwell(0.5)

            #time.sleep(2)
            self.G_PhrozenFluiddRespondInfo("[INFO] time.sleep(2)")
        elif value<0:
            self.G_PhrozenFluiddRespondInfo("[INFO] Sending command: I3; retract")

            #lancaigang241031:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("I3")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: I3")
            elif self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("I3")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: I3")

            # lancaigang240516: preventtime too close
            # lancaigang240705: prevent time; or time too close
            self.G_ProzenToolhead.dwell(0.52)

            #time.sleep(2)
            self.G_PhrozenFluiddRespondInfo("[INFO] time.sleep(2)")
        elif value==0:
            self.G_PhrozenFluiddRespondInfo('[INFO] Sending command: AT+IDLE; IDLE state')

            #lancaigang241031:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("AT+IDLE")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: AT+IDLE")
            elif self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("AT+IDLE")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: AT+IDLE")

            # lancaigang240516: preventtime too close
            # lancaigang240705: prevent time; or time too close
            self.G_ProzenToolhead.dwell(0.5)

            #time.sleep(2)
            self.G_PhrozenFluiddRespondInfo("[INFO] time.sleep(2)")
        else:
            self.G_PhrozenFluiddRespondInfo("[INFO] Sending command: none")


        self.Cmds_P1InExtrudeManualIn(value)


        #self.Cmds_AMSSerial1Send("AT+IDLE")
        # self.G_PhrozenFluiddRespondInfo("[DEBUG] [(cmds.python)Cmds_P1EnForceForward]Sending command: AT+IDLE; IDLEstate")
        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+I:1,%d" % self.G_ChangeChannelTimeoutNewChan)




    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # P1 B?;filament
    def Cmds_P1BnWholeRollbackAction(self, chan, gcmd):
        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1BnWholeRollbackAction]Sending command: B%d" % chan)
        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()
         # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+B:0,%d" % self.G_ChangeChannelTimeoutNewChan)


        #lancaigang241030:
        if chan in range(1, 5):
            self.Cmds_AMSSerial1Send("B%d" % chan)
            self.G_PhrozenFluiddRespondInfo("Serial port 1 sending command: B%d" % chan)
        elif chan in range(5, 9):
            self.Cmds_AMSSerial2Send("B%d" % (chan-4))
            self.G_PhrozenFluiddRespondInfo("Serial port 2 sending command: B%d" % (chan-4))



        # lancaigang240115: ifcurrent channel is filament tubechannel, canchecktoolheadcut filament
        if self.G_ChangeChannelTimeoutNewChan == chan:
            # lancaigang240113: toolheadhasfilamentdetect
            if self.G_ToolheadIfHaveFilaFlag == True:
                self.G_ProzenToolhead.dwell(6.0)
                # lancaigang231201: checkcut filament after is no normal normal unload filament, normal normal then pause
                self.Cmds_CutFilaIfNormalCheck()

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+B:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # P1 D?;filamentto park position
    def Cmds_P1DnMoveToParkPositonAction(self, chan, gcmd):
        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1DnMoveToParkPositonAction]Sending command: P%d" % chan)
        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()
         # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+D:0,%d" % self.G_ChangeChannelTimeoutNewChan)

         #lancaigang241030:
        if chan in range(1, 5):
            self.Cmds_AMSSerial1Send("P%d" % chan)
            self.G_PhrozenFluiddRespondInfo("Serial port 1 sending command: P%d" % chan)
        elif chan in range(5, 9):
            self.Cmds_AMSSerial2Send("P%d" % (chan-4))
            self.G_PhrozenFluiddRespondInfo("Serial port 2 sending command: P%d" % (chan-4))


        # lancaigang240115: ifcurrent channel is filament tubechannel, canchecktoolheadcut filament
        if self.G_ChangeChannelTimeoutNewChan == chan:
            # lancaigang240113: toolheadhasfilamentdetect
            if self.G_ToolheadIfHaveFilaFlag == True:
                self.G_ProzenToolhead.dwell(6.0)
                # lancaigang231201: checkcut filament after is no normal normal unload filament, normal normal then pause
                self.Cmds_CutFilaIfNormalCheck()

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+D:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_MoveToCutFilaPrepare(self):
        self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_MoveToCutFilaPrepare]Prepare before cutting filament")

        # self.Cmds_AMSSerial1Send("H%d" % self.G_ChangeChannelTimeoutNewChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]cut filamentbeforespecial refillstate: H%d" % self.G_ChangeChannelTimeoutNewChan)
        # time.sleep(1)

        # #lancaigang240319: after, first reserve toolheadfilament, preventsmall pellet
        # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)]External macro command-PG102; cut filamentbefore, first reserve toolheadfilament, preventsmall pellet")
        # self.PG102Flag=True
        # self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)]self.Flag=True")
        # command_string = """
        # PG102
        # """
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)]command_string='%s'" % command_string)
        
        # # for i in range(15):
        # # self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)]purge, wait")
        # # #lancaigang20231013: is 4when
        # # #lancaigang231115: is 1s
        # #     self.G_ProzenToolhead.dwell(1.0)
        # # #lancaigang240125: cannot use sleep, will thread
        # #     #time.sleep(1)
        # self.PG102Flag=False
        # self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)]self.Flag=False")

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    #Move to the filament-cut position
    def Cmds_MoveToCutFilaAction(self, gcmd):
        # // [(cmds.python)Cmds_MoveToCutFilaAction]cut filament;gcodecommand=
        # // G91
        # // G1 Z1.200000 F3000
        # // [(cmds.python)Cmds_MoveToCutFilaAction]cut filament;gcodecommand=
        # // G90
        # // G1 X301.500000 Y0.000000 F24000
        # // G1 X308.500000 F600
        # // G1 X301.500000 F7200
        self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_MoveToCutFilaAction]cut filament;sending command')

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+Cut:0,%d" % self.G_ChangeChannelTimeoutNewChan)

        # #lancaigang231208: pauseresumeerror, first disable disable
        # # # 0=cut filament before default internal all gcode
        # #lancaigang231208: z+ normal number will up
        # #lancaigang231215: Z axis up after down
        # #if self.G_ChangeChannelIfZLiftingUpByGcode == 0:
        # command_string = """
        #     G91
        #     G1 Z%f F500
        #     """ % (
        #     self.G_AMSFilaCutZPositionLiftingUp,
        # )
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # #lancaigang231216: ifpause, z, to pausewhen, set z keep, error
        # self.G_IfZPositionLiftUpFlag = True
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_MoveToCutFilaAction]cut filament; Z axisraiseraise;gcodecommand=%s" % command_string)
        # self.G_ProzenToolhead.wait_moves()
        # self.G_ProzenToolhead.dwell(1.0)



        # self.Cmds_AMSSerial1Send("H%d" % self.G_ChangeChannelTimeoutNewChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]cut filamentbeforespecial refillstate: H%d" % self.G_ChangeChannelTimeoutNewChan)
        # time.sleep(1)



        # to cut filamentX Yposition, cut filament, then
        # // G91
        # // G1 Z5.000000 F5000
        # // G90
        # // G1 X302.000000 Y244.100000 F5000
        # // G1 X309.000000 F500
        # // G1 X302.000000 F5000
        # // G1 X290 F5000
        command_string = """
            G91
            G1 Z%f F8000
            G90
            G1 X%f Y%f F10000
            G1 X%f Y%f F500
            G4 P500
            G1 X%f Y%f F8000
            G1 X%f F5000
            G91
            G1 Z-%f F8000
            """ % (
            self.G_AMSFilaCutZPositionLiftingUp,
            self.G_AMSFilaCutXPosition-7,
            self.G_AMSFilaCutYPosition,#lancaigang240409: 
            self.G_AMSFilaCutXPosition,
            self.G_AMSFilaCutYPosition,
            self.G_AMSFilaCutXPosition-7,
            self.G_AMSFilaCutYPosition,#lancaigang241217: 
            self.G_AMSFilaCutXPosition-30,#lancaigang250807:
            self.G_AMSFilaCutZPositionLiftingUp,
        )
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.G_PhrozenFluiddRespondInfo('Z axisraiseraise and cut filament;gcodecommand=%s' % command_string)
        self.G_ProzenToolhead.wait_moves()

        #self.G_IfZPositionLiftUpFlag = True


        # #lancaigang240110: waitwaitbefore, first External macro command, move move to positionwait
        # command_string = """
        #     PG101
        #     """
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_MoveToCutFilaAction]External macro command-to positionpurge; command_string='%s'" % command_string)
        # self.IfDoPG102Flag=True

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+Cut:1,%d" % self.G_ChangeChannelTimeoutNewChan)

        # #lancaigang231207: preventcut filamentpress against the cutter, push downward0.5; pauseresumewhen, disable disable
        # command = """
        #     G92 E0
        #     G1 E0.5 F300
        #     G92 E0
        # """
        # self.G_PhrozenGCode.run_script_from_command(command)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_MoveToCutFilaAction]cut filament;gcodecommand=%s" % command)
        # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_MoveToCutFilaAction]preventcut filamentpress against the cutter, push downward0.5")

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_MoveToCutFilaAbsolutePositionNotReset(self, gcmd):
        self.G_PhrozenFluiddRespondInfo('[(cmds.python)Cmds_MoveToCutFilaAbsolutePositionNotReset]home/resetcut filament, absolute position;sending command=%s' % (gcmd.get_commandline()))

        # #lancaigang231208: z+ normal number will up
        # # command_string = """
        # #     G91
        # #     G1 Z10 F3000
        # #     """
        # # self.G_PhrozenGCode.run_script_from_command(command_string)
        # #lancaigang231215: Z axis up after down
        # command_string = """
        #     G91
        #     G1 Z%f F500
        #     """ % (
        #     self.G_AMSFilaCutZPositionLiftingUp,
        # )
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # #lancaigang231216: ifpause, z, to pausewhen, set z keep, error
        # self.G_IfZPositionLiftUpFlag = True
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_MoveToCutFilaAbsolutePositionNotReset]Z axis up 10mm=%s" % command_string)
        # self.G_ProzenToolhead.wait_moves()
        # self.G_ProzenToolhead.dwell(1.0)


        # self.Cmds_AMSSerial1Send("H%d" % self.G_ChangeChannelTimeoutNewChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]cut filamentbeforespecial refillstate: H%d" % self.G_ChangeChannelTimeoutNewChan)
        # time.sleep(1)

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+Cut:0,%d" % self.G_ChangeChannelTimeoutNewChan)

        # to cut filamentX Yposition, cut filament, then
        # // G91
        # // G1 Z5.000000 F5000
        # // G90
        # // G1 X302.000000 Y244.100000 F5000
        # // G1 X309.000000 F500
        # // G1 X302.000000 F5000
        # // G1 X290 F5000
        command_string = """
            G91
            G1 Z%f F8000
            G90
            G1 X%f Y%f F10000
            G1 X%f Y%f F500
            G4 P500
            G1 X%f Y%f F8000
            G1 X%f F5000
            G91
            G1 Z-%f F8000
            """ % (
            self.G_AMSFilaCutZPositionLiftingUp,
            self.G_AMSFilaCutXPosition-7,
            self.G_AMSFilaCutYPosition,#lancaigang240409: 
            self.G_AMSFilaCutXPosition,
            self.G_AMSFilaCutYPosition,
            self.G_AMSFilaCutXPosition-7,
            self.G_AMSFilaCutYPosition,#lancaigang241217: 
            self.G_AMSFilaCutXPosition-30,#lancaigang250807:
            self.G_AMSFilaCutZPositionLiftingUp,
        )
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.G_PhrozenFluiddRespondInfo('Z axisraiseraise and cut filament;gcodecommand=%s' % command_string)
        self.G_ProzenToolhead.wait_moves()

        #self.G_IfZPositionLiftUpFlag = True
        
        # #lancaigang240110: waitwaitbefore, first External macro command, wipe nozzle
        # command_string = """
        #     PG107
        #     """
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # self.G_PhrozenFluiddRespondInfo("External macro command-wipe nozzle; command_string='%s'" % command_string)
        # self.IfDoPG102Flag=True

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+Cut:1,%d" % self.G_ChangeChannelTimeoutNewChan)

        # #lancaigang231207: preventcut filamentpress against the cutter, push downward0.5; pauseresumewhen, disable disable
        # command = """
        #     G92 E0
        #     G1 E0.5 F300
        #     G92 E0
        # """
        # self.G_PhrozenGCode.run_script_from_command(command)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_MoveToCutFilaAction]cut filament;gcodecommand=%s" % command)
        # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_MoveToCutFilaAction]preventcut filamentpress against the cutter, push downward0.5")

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_MoveToCutFilaAbsolutePositionNotResetAndRollback(self, gcmd):
        self.G_PhrozenFluiddRespondInfo('[(cmds.python)Cmds_MoveToCutFilaAbsolutePositionNotResetAndRollback]home/resetcut filament, absolute position;sending command=%s' % (gcmd.get_commandline()))
        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()
        # #lancaigang231208: z+ normal number will up
        # # command_string = """
        # #     G91
        # #     G1 Z10 F3000
        # #     """
        # # self.G_PhrozenGCode.run_script_from_command(command_string)
        # #lancaigang231215: Z axis up after down
        # command_string = """
        #     G91
        #     G1 Z%f F500
        #     """ % (
        #     self.G_AMSFilaCutZPositionLiftingUp,
        # )
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # #lancaigang231216: ifpause, z, to pausewhen, set z keep, error
        # self.G_IfZPositionLiftUpFlag = True
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_MoveToCutFilaAbsolutePositionNotReset]Z axis up 10mm=%s" % command_string)
        # self.G_ProzenToolhead.wait_moves()
        # self.G_ProzenToolhead.dwell(1.0)

        # self.Cmds_AMSSerial1Send("H%d" % self.G_ChangeChannelTimeoutNewChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]cut filamentbeforespecial refillstate: H%d" % self.G_ChangeChannelTimeoutNewChan)
        # time.sleep(1)

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+Cut:0,%d" % self.G_ChangeChannelTimeoutNewChan)

       # to cut filamentX Yposition, cut filament, then
        # // G91
        # // G1 Z5.000000 F5000
        # // G90
        # // G1 X302.000000 Y244.100000 F5000
        # // G1 X309.000000 F500
        # // G1 X302.000000 F5000
        # // G1 X290 F5000
        command_string = """
            G91
            G1 Z%f F8000
            G90
            G1 X%f Y%f F10000
            G1 X%f Y%f F500
            G4 P500
            G1 X%f Y%f F8000
            G1 X%f F5000
            G91
            G1 Z-%f F8000
            """ % (
            self.G_AMSFilaCutZPositionLiftingUp,
            self.G_AMSFilaCutXPosition-7,
            self.G_AMSFilaCutYPosition,#lancaigang240409: 
            self.G_AMSFilaCutXPosition,
            self.G_AMSFilaCutYPosition,
            self.G_AMSFilaCutXPosition-7,
            self.G_AMSFilaCutYPosition,#lancaigang241217: 
            self.G_AMSFilaCutXPosition-30,#lancaigang250807:
            self.G_AMSFilaCutZPositionLiftingUp,
        )
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.G_PhrozenFluiddRespondInfo('Z axisraiseraise and cut filament;gcodecommand=%s' % command_string)
        self.G_ProzenToolhead.wait_moves()

        # #self.G_IfZPositionLiftUpFlag = True
        # #lancaigang240110: waitwaitbefore, first External macro command, wipe nozzle
        # command_string = """
        #     PG107
        #     """
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # self.G_PhrozenFluiddRespondInfo("External macro command-wipe nozzle; command_string='%s'" % command_string)
        # self.IfDoPG102Flag=True


        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+Cut:1,%d" % self.G_ChangeChannelTimeoutNewChan)


        # #lancaigang231207: preventcut filamentpress against the cutter, push downward0.5; pauseresumewhen, disable disable
        # command = """
        #     G92 E0
        #     G1 E0.5 F300
        #     G92 E0
        # """
        # self.G_PhrozenGCode.run_script_from_command(command)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_MoveToCutFilaAction]cut filament;gcodecommand=%s" % command)
        # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_MoveToCutFilaAction]preventcut filamentpress against the cutter, push downward0.5")


        #lancaigang241031:
        if self.G_SerialPort1OpenFlag == True:
            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: AP, fully retract to the park position")
            # // retract allto park position; //===== P2 A1 allfilamentto park position Yes; "AP";
            self.Cmds_AMSSerial1Send("AP")
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2Sending command: AP, fully retract to the park position")
            # // retract allto park position; //===== P2 A1 allfilamentto park position Yes; "AP";
            self.Cmds_AMSSerial2Send("AP")

        # lancaigang240913: set to external
        self.G_ProzenToolhead.dwell(6.0)
        # lancaigang231201: checkcut filament after is no normal normal unload filament, normal normal then pause
        self.Cmds_CutFilaIfNormalCheck()

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_MoveToCutFilaAndNotRollback(self, gcmd):
        self.G_PhrozenFluiddRespondInfo('[(cmds.python)Cmds_MoveToCutFilaAndNotRollback]cut filament;sending command=%s' % (gcmd.get_commandline()))
        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+Zero:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # lancaigang20231019: machine error, Automatic filament changeifdetected #1 channeltoolhead reserve up time filament, need to and allfilament
        # lancaigang20231020: first detecttoolheadhas
        #if self.G_ToolheadIfHaveFilaFlag:
        # 0=cut filament before default internal all gcode
        # lancaigang231128: G28 is PG28
        #if self.G_ChangeChannelIfZLiftingUpByGcode == 0:
        self.G_PhrozenFluiddRespondInfo("[INFO] allhomeandcut filament")
        command_string = """
        G28
        """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.G_PhrozenFluiddRespondInfo('home/reset=%s' % command_string)
        # lancaigang20231020: output head retractgcode, retract before need totoolhead, when compare, logical, Automatic filament change and cut filament
        # G92 E0
        # G1 E0.0000 F600
        # G91
        # G1 E-0.385 F8000
        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+Zero:1,%d" % self.G_ChangeChannelTimeoutNewChan)

        self.G_PhrozenFluiddRespondInfo("[INFO] cut filament")


        #lancaigang20231013: cut filament
        self.Cmds_MoveToCutFilaAction(gcmd)

####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_MoveToCutFilaAndHomingXY(self, gcmd):
        self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_MoveToCutFilaAndHomingXY]cut filament;XYreset")
        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+Zero:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # lancaigang20231019: machine error, Automatic filament changeifdetected #1 channeltoolhead reserve up time filament, need to and allfilament
        # lancaigang20231020: first detecttoolheadhas
        #if self.G_ToolheadIfHaveFilaFlag:
        # 0=cut filament before default internal all gcode
        # lancaigang231128: G28 is PG28
        #if self.G_ChangeChannelIfZLiftingUpByGcode == 0:
        self.G_PhrozenFluiddRespondInfo("[INFO] G28homeYandcut filament")
        command_string = """
        G28 Y0
        """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.G_PhrozenFluiddRespondInfo("Y0 reset=%s" % command_string)
        self.G_PhrozenFluiddRespondInfo("[INFO] G28homeXandcut filament")
        command_string = """
        G28 X0
        """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.G_PhrozenFluiddRespondInfo("X0 reset=%s" % command_string)
        # lancaigang20231020: output head retractgcode, retract before need totoolhead, when compare, logical, Automatic filament change and cut filament
        # G92 E0
        # G1 E0.0000 F600
        # G91
        # G1 E-0.385 F8000
        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+Zero:1,%d" % self.G_ChangeChannelTimeoutNewChan)

        self.G_PhrozenFluiddRespondInfo("[INFO] cut filament")


        #lancaigang20231013: cut filament
        self.Cmds_MoveToCutFilaAction(gcmd)


    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_MoveToCutFilaAndRollback(self, gcmd):
        number=50;
        self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_MoveToCutFilaAndRollback]cut filament;sending command')

        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()

        # lancaigang20231019: machine error, Automatic filament changeifdetected #1 channeltoolhead reserve up time filament, need to and allfilament
        # lancaigang20231020: first detecttoolheadhas
        #if self.G_ToolheadIfHaveFilaFlag:
        # # 0=cut filament before default internal all gcode
        # lancaigang231128: G28 is PG28
        # lancaigang240319: G-codehasPG28, do notPG28
        # lancaigang240323: # layer, first disable disable
        # if self.G_ChangeChannelIfZLiftingUpByGcode == 0:
        # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_MoveToCutFilaAndRollback]allhomeandcut filament")
        #     command_string = """
        #     PG28
        #     """
        #     self.G_PhrozenGCode.run_script_from_command(command_string)
        # #lancaigang20231020: output head retractgcode, retract before need totoolhead, when compare, logical, Automatic filament change and cut filament
        #     # G92 E0
        #     # G1 E0.0000 F600
        #     # G91
        #     # G1 E-0.385 F8000

        self.G_PhrozenFluiddRespondInfo("[INFO] cut filament")
        #lancaigang20231013: cut filament
        self.Cmds_MoveToCutFilaAction(gcmd)


        self.G_ProzenToolhead.dwell(2.0)


        #lancaigang241031:
        if self.G_SerialPort1OpenFlag == True:
            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: AP, fully retract to the park position")
            # // retract allto park position; //===== P2 A1 allfilamentto park position Yes; "AP";
            self.Cmds_AMSSerial1Send("AP")
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2Sending command: AP, fully retract to the park position")
            # // retract allto park position; //===== P2 A1 allfilamentto park position Yes; "AP";
            self.Cmds_AMSSerial2Send("AP")




        # lancaigang240913: set to external
        self.G_ProzenToolhead.dwell(6.0)
        # lancaigang231201: checkcut filament after is no normal normal unload filament, normal normal then pause
        self.Cmds_CutFilaIfNormalCheck()



        # if self.G_ToolheadIfHaveFilaFlag:
        #     for i in range(number):
        #             time.sleep(1)
        #             i += 1
        # self.G_PhrozenFluiddRespondInfo('toolhead has, APcommand;i=%d' % i)

        #             if i >= number:
        # self.G_PhrozenFluiddRespondInfo('APcommandtimeout;number=%d' % number)
        #                 break

    

    
    
    

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_PhrozenKlipperPauseCommon(self):
        self.G_PhrozenFluiddRespondInfo("[WARN] =====[(cmds.python)Cmds_PhrozenKlipperPauseCommon]klipperpause")
        self.G_PhrozenFluiddRespondInfo("[WARN] =====PAUSE=====")
        self.G_PhrozenFluiddRespondInfo("=====PAUSE=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====PAUSE=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)

        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        #lancaigang250517: 
        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)

        # lancaigang250526: pause, not allowednew gcodecommand, need waitpausecomplete
        self.G_KlipperInPausing = True
        self.G_PhrozenFluiddRespondInfo('[INFO] pause, not allowednew gcodecommand, need pausecomplete')

        #lancaigang240229:
        if self.IfDoPG102Flag==True:
            self.G_PhrozenFluiddRespondInfo("[INFO] self.IfDoPG102Flag==True")
            self.IfDoPG102Flag=False

            # self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)Cmds_PhrozenKlipperPauseCommon]External macro command-PG104")
            # command_string = """
            # PG104
            # """
            # self.G_PhrozenGCode.run_script_from_command(command_string)
            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPauseCommon]External macro command-; command_string='%s'" % command_string)
            # command = """
            #     G91
            #     G1 Z5 F5000
            #     G90
            #     G1 X240 Y280 F8000
            #     G91
            #     G1 Z-5 F5000
            #     G90
            # """
            # self.G_PhrozenGCode.run_script_from_command(command)
            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)]command=%s" % (command))


        # lancaigang241030: has in state down pause
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:#0
            self.G_PhrozenFluiddRespondInfo('[WARN] in printingmode, do not executepausePAUSEcommand')
        else:
            Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
            self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
            self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
            #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
            if Lo_PauseStatus['is_paused'] == True:
                self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
            else:
                self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")

                # #lancaigang250527: Pause in the waiting area
                # self.G_PhrozenFluiddRespondInfo("[WARN] startcalling External macro command-PRZ_PAUSE_WAITINGAREA")
                # command = """
                # PRZ_PAUSE_WAITINGAREA
                # """
                # self.G_PhrozenGCode.run_script_from_command(command)
                # self.G_PhrozenFluiddRespondInfo("calling External macro command:command=%s" % (command))

                # lancaigang250527: pause
                if self.G_KlipperQuickPause == True:
                    self.G_KlipperQuickPause = False

                    
                    self.G_PhrozenFluiddRespondInfo("[WARN] startcalling External macro command-PRZ_PAUSE_WAITINGAREA")
                    command = """
                    PRZ_PAUSE_WAITINGAREA
                    """
                    self.G_PhrozenGCode.run_script_from_command(command)

                    # lancaigang240119: pause use cfg set table command
                    self.G_PhrozenFluiddRespondInfo("[WARN] startcalling External macro command-PAUSE_PRINTING")
                    command = """
                    PAUSE_PRINTING
                    """
                    self.G_PhrozenGCode.run_script_from_command(command)
                    self.G_PhrozenFluiddRespondInfo("[INFO] self.G_ProzenToolhead.wait_moves()")
                    self.G_ProzenToolhead.wait_moves()
                    self.G_PhrozenFluiddRespondInfo('calling External macro command:command=%s' % (command))
                    # self.G_PhrozenFluiddRespondInfo("[WARN] preventpause, add command; send_pause_command")
                    #self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                else:
                    self.G_KlipperQuickPause = False

                    # #lancaigang250716: need tofirst to pause
                    # self.G_PhrozenFluiddRespondInfo("[WARN] startcalling External macro command-PRZ_PAUSE_WAITINGAREA")
                    # command = """
                    # PRZ_PAUSE_WAITINGAREA
                    # """
                    # self.G_PhrozenGCode.run_script_from_command(command)

                    # lancaigang240119: pause use cfg set table command
                    self.G_PhrozenFluiddRespondInfo("[WARN] startcalling External macro command-PAUSE")
                    command = """
                    PAUSE
                    """
                    self.G_PhrozenGCode.run_script_from_command(command)
                    self.G_PhrozenFluiddRespondInfo("[INFO] self.G_ProzenToolhead.wait_moves()")
                    self.G_ProzenToolhead.wait_moves()
                    self.G_PhrozenFluiddRespondInfo('calling External macro command:command=%s' % (command))
                    # self.G_PhrozenFluiddRespondInfo("[WARN] preventpause, add command; send_pause_command")
                    #self.G_PhrozenPrinterCancelPauseResume.send_pause_command()

                # #lancaigang250527: Pause in the waiting area
                # self.G_PhrozenFluiddRespondInfo("[WARN] startcalling External macro command-PRZ_PAUSE_WAITINGAREA")
                # command = """
                # PRZ_PAUSE_WAITINGAREA
                # """
                # self.G_PhrozenGCode.run_script_from_command(command)
                # self.G_PhrozenFluiddRespondInfo("calling External macro command:command=%s" % (command))

                # lancaigang240125: pause, not allowedstm23 move reportthen time pause
                self.STM32ReprotPauseFlag=1
                self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=1")

                self.G_KlipperIfPaused = True
                self.G_PhrozenFluiddRespondInfo("[WARN] self.G_KlipperIfPaused = True")
                self.G_PhrozenFluiddRespondInfo("[INFO] klipperpause; ")




        # lancaigang240325: failed, cannotresume
        self.G_MCModeCanResumeFlag = False

        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        #lancaigang250517: 
        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)

         #lancaigang250526: 
        self.G_KlipperInPausing = False
        self.G_PhrozenFluiddRespondInfo('[INFO] pausecomplete, allowednew gcodecommand')


        # lancaigang250607: state; 1-unload filament; 2-feed; 3-; 4-pause
        self.G_KlipperPrintStatus= 4

        # # # move move to before action
        # # command = """
        # #     G90
        # #     G1 X150 Y10 F5400
        # # """
        # # self.G_PhrozenGCode.run_script_from_command(command)
        # # self.G_PhrozenFluiddRespondInfo("[(cmds.python)]command=%s" % (command))
        # # # gcodecommandafter to add down wait_moves number
        # # #lancaigang231202: wait_movesklipperno pause
        # # #lancaigang231207: cannot use wait_moves, no then keep gcodecommanderror
        # # self.G_ProzenToolhead.wait_moves()
        # # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)] move move to before action ")
        # #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_MoveToCutFilaAction]cut filament;gcodecommand=%s" % command)
        # #klipperpausecommand; keep current x y z
        # #lancaigang240108: to time pause keep number data is no normal normal, after to
        # self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PRZ_PAUSE_STATE")
        # self.G_PhrozenFluiddRespondInfo("[DEBUG] [(cmds.python)]SAVE_GCODE_STATE")
        # self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
        # #self.G_ProzenToolhead.wait_moves()
        # self.G_ProzenToolhead.dwell(1.0)

        # #time.sleep(1)
        # #self.G_ProzenToolhead.wait_moves()
        # #self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)]wait_moves")
        # #lancaigang231219: is is pause move move klippererror
        # #lancaigang230103: pausehaswhen
        # # move move to before action
        # # G91
        # # G1 Z10 F7000
        # # G90
        # command = """
        # G91
        # G1 X150 Y10 F6000
        # G90
        # """
        # self.G_PhrozenGCode.run_script_from_command(command)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)]command=%s" % (command))
        # self.G_ProzenToolhead.dwell(1.0)

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_PhrozenKlipperPauseM2M3ToSTM32(self, gcmd):
        _ = gcmd

        # lancaigang231115: to first gcmdcommand is no is empty, no then will klippererror
        if gcmd is None:
            self.G_PhrozenFluiddRespondInfo('[WARN] [(cmds.python)Cmds_PhrozenKlipperPauseM2M3ToSTM32]self.G_PhrozenFluiddRespondInfo;gcmd to is empty')
            self.G_PhrozenFluiddRespondInfo("[INFO] self.G_PhrozenFluiddRespondInfo;klipperpause")
            #pass
        if gcmd is not None:
            self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPauseM2M3ToSTM32]command='%s'" % (gcmd.get_commandline(),))
            self.G_PhrozenFluiddRespondInfo("[INFO] klipperpause")
            #pass
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPauseM2M3ToSTM32]command='%s'" % (gcmd.get_commandline(),))

        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()

        # lancaigang250526: pause, not allowednew gcodecommand, need waitpausecomplete
        self.G_KlipperInPausing = True
        self.G_PhrozenFluiddRespondInfo('[INFO] pause, not allowednew gcodecommand, need pausecomplete')

        # lancaigang231129: pausewhen toolhead move move to position
        # self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PRZ_PAUSE_STATE")
        # self.G_KlipperIfPaused = True
        # #toolheadwait move move
        # lancaigang231202: wait_movesklipperno pause
        # lancaigang231207: cannot use wait_moves, no then keep gcodecommanderror
        # self.G_ProzenToolhead.wait_moves()


        #time.sleep(1)
        # lancaigang231201: klipperpausewhen, pausestm32 machine
        #// AT+PAUSE
        #// AT+PAUSE=8
        #// AT+PAUSE=9


        #lancaigang241031:
        if self.G_SerialPort1OpenFlag == True:
            self.Cmds_AMSSerial1Send("AT+PAUSE")
            self.G_PhrozenFluiddRespondInfo('[WARN] serial port 1 send AT+PAUSEpausestm32 machine')
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.Cmds_AMSSerial2Send("AT+PAUSE")
            self.G_PhrozenFluiddRespondInfo('[WARN] serial port 2 send AT+PAUSEpausestm32 machine')


        # lancaigang240125: number
        self.Cmds_PhrozenKlipperPauseCommon()

     
        self.G_KlipperIfPaused = True
        self.G_PhrozenFluiddRespondInfo("[WARN] self.G_KlipperIfPaused = True")
        self.G_PhrozenFluiddRespondInfo("[INFO] klipperpause; ")

        #lancaigang250526: 
        self.G_KlipperInPausing = False
        self.G_PhrozenFluiddRespondInfo('[INFO] pausecomplete, allowednew gcodecommand')

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_PhrozenKlipperPauseMAToSTM32(self, gcmd):
        _ = gcmd

        # lancaigang231115: to first gcmdcommand is no is empty, no then will klippererror
        if gcmd is None:
            self.G_PhrozenFluiddRespondInfo('[WARN] [(cmds.python)Cmds_PhrozenKlipperPauseMAToSTM32]self.G_PhrozenFluiddRespondInfo;gcmd to is empty')
            self.G_PhrozenFluiddRespondInfo("[INFO] self.G_PhrozenFluiddRespondInfo;klipperpause")
            #pass
        if gcmd is not None:
            self.G_PhrozenFluiddRespondInfo("command='%s'" % (gcmd.get_commandline(),))
            self.G_PhrozenFluiddRespondInfo("[INFO] klipperpause")
            #pass
        # self.G_PhrozenFluiddRespondInfo("command='%s'" % (gcmd.get_commandline(),))

        # lancaigang250526: pause, not allowednew gcodecommand, need waitpausecomplete
        self.G_KlipperInPausing = True
        self.G_PhrozenFluiddRespondInfo('[INFO] pause, not allowednew gcodecommand, need pausecomplete')


        #time.sleep(1)

        # #lancaigang241031:
        # if self.G_SerialPort1OpenFlag == True:
        #     self.Cmds_AMSSerial1Send("AT+PAUSE")
        # self.G_PhrozenFluiddRespondInfo("[WARN] serial port 1 send AT+PAUSEpausestm32 machine ")
        # #lancaigang241030:
        # if self.G_SerialPort2OpenFlag == True:
        #     self.Cmds_AMSSerial2Send("AT+PAUSE")
        # self.G_PhrozenFluiddRespondInfo("[WARN] serial port 2 send AT+PAUSEpausestm32 machine ")

        #lancaigang240229:
        if self.IfDoPG102Flag==True:
            self.G_PhrozenFluiddRespondInfo("[INFO] self.IfDoPG102Flag==True")
            self.IfDoPG102Flag=False

        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")

        # lancaigang241030: has in state down pause
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:#0
            self.G_PhrozenFluiddRespondInfo('[WARN] in printingmode, do not executepausePAUSEcommand')
        else:
 
            # lancaigang240119: pause use cfg set table command
            self.G_PhrozenFluiddRespondInfo("[WARN] startcalling External macro command-PAUSEMA")
            command = """
            PAUSEMA
            """
            self.G_PhrozenGCode.run_script_from_command(command)
            self.G_PhrozenFluiddRespondInfo('calling command:command=%s' % (command))
            # self.G_PhrozenFluiddRespondInfo("[WARN] preventpause, add command; send_pause_command")
            #self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
            self.G_PhrozenFluiddRespondInfo("[WARN] finishcalling External macro command-PAUSEMA")

            # lancaigang240125: pause, not allowedstm23 move reportthen time pause
            self.STM32ReprotPauseFlag=1
            self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=1")
            self.G_KlipperIfPaused = True
            self.G_PhrozenFluiddRespondInfo("[WARN] self.G_KlipperIfPaused = True")
            self.G_PhrozenFluiddRespondInfo("[INFO] klipperpause; ")


        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")


        # lancaigang240325: failed, cannotresume
        self.G_MCModeCanResumeFlag = False

        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)

        #lancaigang250526: 
        self.G_KlipperInPausing = False
        self.G_PhrozenFluiddRespondInfo('[INFO] pausecomplete, allowednew gcodecommand')
    

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_PhrozenKlipperPauseM2M3NoneCmdToSTM32(self, gcmd):
        _ = gcmd

        # lancaigang231115: to first gcmdcommand is no is empty, no then will klippererror
        if gcmd is None:
            self.G_PhrozenFluiddRespondInfo('[WARN] [(cmds.python)Cmds_PhrozenKlipperPauseM2M3NoneCmdToSTM32]self.G_PhrozenFluiddRespondInfo;gcmd to is empty')
            self.G_PhrozenFluiddRespondInfo("[INFO] self.G_PhrozenFluiddRespondInfo;klipperpause")
            #pass
        if gcmd is not None:
            self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPauseM2M3NoneCmdToSTM32]command='%s'" % (gcmd.get_commandline(),))
            self.G_PhrozenFluiddRespondInfo("[INFO] klipperpause")
            #pass
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPauseM2M3NoneCmdToSTM32]command='%s'" % (gcmd.get_commandline(),))


        # lancaigang231129: pausewhen toolhead move move to position
        # self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PRZ_PAUSE_STATE")
        # self.G_KlipperIfPaused = True
        # #toolheadwait move move
        # lancaigang231202: wait_movesklipperno pause
        # lancaigang231207: cannot use wait_moves, no then keep gcodecommanderror
        # self.G_ProzenToolhead.wait_moves()

        # lancaigang250526: pause, not allowednew gcodecommand, need waitpausecomplete
        self.G_KlipperInPausing = True
        self.G_PhrozenFluiddRespondInfo('[INFO] pause, not allowednew gcodecommand, need pausecomplete')


        # lancaigang240125: number
        self.Cmds_PhrozenKlipperPauseCommon()


        self.G_KlipperIfPaused = True
        self.G_PhrozenFluiddRespondInfo("[WARN] self.G_KlipperIfPaused = True")
        self.G_PhrozenFluiddRespondInfo("[INFO] klipperpause; ")

        #lancaigang250526: 
        self.G_KlipperInPausing = False
        self.G_PhrozenFluiddRespondInfo('[INFO] pausecomplete, allowednew gcodecommand')

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # PRZ_PAUSEpause(commandpause move action and non-pause, in when use use)
    # AT+PAUSE
    def Cmds_PhrozenKlipperPauseNoneCmdToSTM32(self, gcmd):
        _ = gcmd

        # lancaigang231130: disable disable, time pause down
        # if self.G_KlipperIfPaused == True:
        # self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)Cmds_PhrozenKlipperPauseNoneCmdToSTM32] in klipperpausestate")
        #     return


        # lancaigang231115: to first gcmdcommand is no is empty, no then will klippererror
        if gcmd is None:
            self.G_PhrozenFluiddRespondInfo('[WARN] [(cmds.python)Cmds_PhrozenKlipperPauseNoneCmdToSTM32]self.G_PhrozenFluiddRespondInfo;gcmd to is empty')
            self.G_PhrozenFluiddRespondInfo("[INFO] self.G_PhrozenFluiddRespondInfo;klipperpause")
            #pass
        if gcmd is not None:
            self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPauseNoneCmdToSTM32]command='%s'" % (gcmd.get_commandline(),))
            self.G_PhrozenFluiddRespondInfo("[INFO] klipperpause")
            #pass
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPauseNoneCmdToSTM32]command='%s'" % (gcmd.get_commandline(),))


        # lancaigang231129: pausewhen toolhead move move to position
        # self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PRZ_PAUSE_STATE")
        # self.G_KlipperIfPaused = True
        # #toolheadwait move move
        # lancaigang231202: wait_movesklipperno pause
        # lancaigang231207: cannot use wait_moves, no then keep gcodecommanderror
        # self.G_ProzenToolhead.wait_moves()

        # lancaigang250526: pause, not allowednew gcodecommand, need waitpausecomplete
        self.G_KlipperInPausing = True
        self.G_PhrozenFluiddRespondInfo('[INFO] pause, not allowednew gcodecommand, need pausecomplete')


        # lancaigang240125: number
        self.Cmds_PhrozenKlipperPauseCommon()


        self.G_KlipperIfPaused = True
        self.G_PhrozenFluiddRespondInfo("[WARN] self.G_KlipperIfPaused = True")
        self.G_PhrozenFluiddRespondInfo("[INFO] klipperpause; ")

        #lancaigang250526: 
        self.G_KlipperInPausing = False
        self.G_PhrozenFluiddRespondInfo('[INFO] pausecomplete, allowednew gcodecommand')



    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_PhrozenKlipperPauseToolheadCutFailsure(self, gcmd):
        _ = gcmd

        # lancaigang231115: to first gcmdcommand is no is empty, no then will klippererror
        if gcmd is None:
            self.G_PhrozenFluiddRespondInfo('[WARN] [(cmds.python)Cmds_PhrozenKlipperPauseToolheadCutFailsure]self.G_PhrozenFluiddRespondInfo;gcmd to is empty')
            self.G_PhrozenFluiddRespondInfo("[INFO] self.G_PhrozenFluiddRespondInfo;klipperpause")
            #pass
        if gcmd is not None:
            self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPauseToolheadCutFailsure]command='%s'" % (gcmd.get_commandline(),))
            self.G_PhrozenFluiddRespondInfo("[INFO] klipperpause")
            #pass
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPauseToolheadCutFailsure]command='%s'" % (gcmd.get_commandline(),))
        

        # lancaigang231129: pausewhen toolhead move move to position
        # self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PRZ_PAUSE_STATE")
        # self.G_KlipperIfPaused = True
        # #toolheadwait move move
        # lancaigang231207: cannot use wait_moves, no then keep gcodecommanderror
        # self.G_ProzenToolhead.wait_moves()


        # lancaigang250526: pause, not allowednew gcodecommand, need waitpausecomplete
        self.G_KlipperInPausing = True
        self.G_PhrozenFluiddRespondInfo('[INFO] pause, not allowednew gcodecommand, need pausecomplete')


        # lancaigang240125: number
        self.Cmds_PhrozenKlipperPauseCommon()

        self.G_KlipperIfPaused = True
        self.G_PhrozenFluiddRespondInfo("[WARN] self.G_KlipperIfPaused = True")
        self.G_PhrozenFluiddRespondInfo("[INFO] klipperpause; ")

        #lancaigang250526: 
        self.G_KlipperInPausing = False
        self.G_PhrozenFluiddRespondInfo('[INFO] pausecomplete, allowednew gcodecommand')


    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_PhrozenKlipperPauseChangeChannelTimeout(self, gcmd):
        _ = gcmd


        # lancaigang231115: to first gcmdcommand is no is empty, no then will klippererror
        if gcmd is None:
            self.G_PhrozenFluiddRespondInfo('[WARN] [(cmds.python)Cmds_PhrozenKlipperPauseChangeChannelTimeout]self.G_PhrozenFluiddRespondInfo;gcmd to is empty')
            self.G_PhrozenFluiddRespondInfo("[INFO] self.G_PhrozenFluiddRespondInfo;klipperpause")
            #pass
        if gcmd is not None:
            self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPauseChangeChannelTimeout]command='%s'" % (gcmd.get_commandline(),))
            self.G_PhrozenFluiddRespondInfo("[INFO] klipperpause")
            #pass
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPauseChangeChannelTimeout]command='%s'" % (gcmd.get_commandline(),))
        

        # lancaigang231129: pausewhen toolhead move move to position
        # self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PRZ_PAUSE_STATE")
        # self.G_KlipperIfPaused = True
        # #toolheadwait move move
        # lancaigang231207: cannot use wait_moves, no then keep gcodecommanderror
        # self.G_ProzenToolhead.wait_moves()


        # lancaigang250526: pause, not allowednew gcodecommand, need waitpausecomplete
        self.G_KlipperInPausing = True
        self.G_PhrozenFluiddRespondInfo('[INFO] pause, not allowednew gcodecommand, need pausecomplete')


        # lancaigang240125: number
        self.Cmds_PhrozenKlipperPauseCommon()

        self.G_KlipperIfPaused = True
        self.G_PhrozenFluiddRespondInfo("[WARN] self.G_KlipperIfPaused = True")
        self.G_PhrozenFluiddRespondInfo("[INFO] klipperpause; ")

        #lancaigang250526: 
        self.G_KlipperInPausing = False
        self.G_PhrozenFluiddRespondInfo('[INFO] pausecomplete, allowednew gcodecommand')


    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # PRZ_PAUSEpause(commandpause move action and non-pause, in when use use)
    # AT+PAUSE
    def Cmds_PhrozenKlipperPause(self, gcmd):
        _ = gcmd
        self.G_PhrozenFluiddRespondInfo("[WARN] =====[(cmds.python)Cmds_PhrozenKlipperPause]klipperpause")
        # lancaigang231130: disable disable, time pause down
        # if self.G_KlipperIfPaused == True:
        # self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)Cmds_PhrozenKlipperPause] in klipperpausestate")
        #     return

        # if self.G_ChangeChannelResumeFlag:
        # self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)Cmds_PhrozenKlipperPause]in progressresume up time move action, not allowedpause")
        #     return


        # #lancaigang231216: 
        # eventtime = self.G_PhrozenReactor.monotonic()
        # # Determine "printing" status
        # idle_timeout = self.G_PhrozenPrinter.lookup_object("idle_timeout")
        # is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
        # if is_printing:
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPause]in progress; command='%s'" % (gcmd.get_commandline(),))
        # else:
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPause] in; command='%s; return'" % (gcmd.get_commandline(),))
        #     return


        # lancaigang231115: to first gcmdcommand is no is empty, no then will klippererror
        if gcmd is None:
            self.G_PhrozenFluiddRespondInfo('[WARN] [(cmds.python)Cmds_PhrozenKlipperPause]self.G_PhrozenFluiddRespondInfo;gcmd to is empty')
            self.G_PhrozenFluiddRespondInfo("[INFO] self.G_PhrozenFluiddRespondInfo;klipperpause")
            #pass
        if gcmd is not None:
            self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPause]command='%s'" % (gcmd.get_commandline(),))
            self.G_PhrozenFluiddRespondInfo("[INFO] klipperpause")
            #pass
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPause]command='%s'" % (gcmd.get_commandline(),))
        
        self.G_PhrozenFluiddRespondInfo("[INFO] Current mode")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:0,unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:1,MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:2,MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo("[WARN] +Mode:3,RUNOUT")
        else:
            self.G_PhrozenFluiddRespondInfo("[ERROR] +Mode:-1,error")

        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()

        # lancaigang250526: pause, not allowednew gcodecommand, need waitpausecomplete
        self.G_KlipperInPausing = True
        self.G_PhrozenFluiddRespondInfo('[INFO] pause, not allowednew gcodecommand, need pausecomplete')



        # lancaigang250526: pause, not allowednew gcodecommand, need waitpausecomplete
        self.G_KlipperInPausing = True



        #time.sleep(1)

        # lancaigang231201: klipperpausewhen, pausestm32 machine
        #// AT+PAUSE
        #// AT+PAUSE=8
        #// AT+PAUSE=9
        

        #lancaigang241031:
        if self.G_SerialPort1OpenFlag == True:
            self.Cmds_AMSSerial1Send("AT+PAUSE")
            self.G_PhrozenFluiddRespondInfo('[WARN] serial port 1 send AT+PAUSEpausestm32 machine')
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.Cmds_AMSSerial2Send("AT+PAUSE")
            self.G_PhrozenFluiddRespondInfo('[WARN] serial port 2 send AT+PAUSEpausestm32 machine')

        # lancaigang231129: pausewhen toolhead move move to position
        # self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PRZ_PAUSE_STATE")
        # self.G_KlipperIfPaused = True
        # #toolheadwait move move
        # lancaigang231207: cannot use wait_moves, no then keep gcodecommanderror
        # self.G_ProzenToolhead.wait_moves()





        # lancaigang240125: number
        self.Cmds_PhrozenKlipperPauseCommon()


        self.G_KlipperIfPaused = True
        self.G_PhrozenFluiddRespondInfo("[WARN] self.G_KlipperIfPaused = True")
        self.G_PhrozenFluiddRespondInfo("[INFO] klipperpause; ")

        #lancaigang250526: 
        self.G_KlipperInPausing = False
        self.G_PhrozenFluiddRespondInfo('[INFO] pausecomplete, allowednew gcodecommand')

    
    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # PRZ_RESUME resume(and PRZ_PAUSE to use use)
    def Cmds_PhrozenKlipperResumeCommon(self):
        self.G_PhrozenFluiddRespondInfo("[INFO] =====[(cmds.python)Cmds_PhrozenKlipperResumeCommon]klipperresume")
        
        # #lancaigang240103: when then resume
        # #self.G_ProzenToolhead.dwell(3.0)
        # velocity = 2400
        # self.G_PhrozenGCode.run_script_from_command(
        #     "RESTORE_GCODE_STATE NAME=PRZ_PAUSE_STATE MOVE=1 MOVE_SPEED=%.4f"
        #     % (velocity)
        # )
        # self.G_PhrozenFluiddRespondInfo("[DEBUG] [(cmds.python)Cmds_PhrozenKlipperResumeCommon]RESTORE_GCODE_STATE")
        # self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)Cmds_PhrozenKlipperResumeCommon]NAME=PRZ_PAUSE_STATE")
        # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_PhrozenKlipperResumeCommon]send_resume_command")
        # #klipperresumecommand
        # self.G_PhrozenPrinterCancelPauseResume.send_resume_command()
        # #self.G_ProzenToolhead.wait_moves()
        # self.G_ProzenToolhead.dwell(2.0)
        # # #lancaigang240103: Z axis up after down, to pausewhen up raise
        # # command_string = """
        # #     G90
        # #     G91
        # #     G1 Z-10 F3000
        # #     """
        # # self.G_PhrozenGCode.run_script_from_command(command_string)


        self.G_PhrozenFluiddRespondInfo("[INFO] Current mode")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:0,unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:1,MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:2,MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo("[WARN] +Mode:3,RUNOUT")
        else:
            self.G_PhrozenFluiddRespondInfo("[ERROR] +Mode:-1,error")


        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo('[INFO] is pause state, need to resume')
            # lancaigang240119: pause use cfg set table command
            self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-RESUME")
            command = """
            RESUME
            """
            self.G_PhrozenGCode.run_script_from_command(command)
            self.G_PhrozenFluiddRespondInfo('calling command:command=%s' % (command))

            self.G_PauseToLCDString=""
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused, do notthen resume")

        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])





        # lancaigang240325: resume number data after, need to
        self.G_MCModeCanResumeFlag == False
        # lancaigang240108: pausestate
        self.G_KlipperIfPaused = False
        # lancaigang240124: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0

        # lancaigang250607: state; 1-unload filament; 2-feed; 3-; 4-pause
        self.G_KlipperPrintStatus= 3

        # lancaigang250619: ifusbconnect10s, then report error pause
        self.G_ASM1DisconnectErrorCount= 0




    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # PRZ_RESUME resume(and PRZ_PAUSE to use use)
    def Cmds_PhrozenKlipperResume(self, gcmd):
        _ = gcmd
        self.G_PhrozenFluiddRespondInfo("[INFO] =====[(cmds.py)Cmds_PhrozenKlipperResume]")
        self.G_PhrozenFluiddRespondInfo("+RESUME:1,%d" % self.G_ChangeChannelTimeoutNewChan)
        self.G_PhrozenFluiddRespondInfo("[INFO] =====RESUME=====")
        self.G_PhrozenFluiddRespondInfo("=====RESUME=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====RESUME=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)

        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        #lancaigang250517: 
        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")




        # lancaigang240511: resumewhen, initialize down serial port, preventAMSserial porterror
        try:
            self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.py)Cmds_PhrozenKlipperResume]Reinitializing serial port 1")
            self.G_SerialPort1Obj = serial.Serial(self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3)
            #Serial port opened successfully
            if self.G_SerialPort1Obj is not None:
                if self.G_SerialPort1Obj.is_open:
                    self.G_SerialPort1OpenFlag = True
                    self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 1successful")
                    # lancaigang231213: openserial port
                    self.G_SerialPort1Obj.flushInput()  # clean serial write cache
                    self.G_SerialPort1Obj.flush()
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 buffers cleared")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Re-registering serial port 1 callback")
                    self.G_SerialPort1RecvTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerUart1Recv, self.G_PhrozenReactor.NOW)
        except:
            self.G_PhrozenFluiddRespondInfo("[ERROR] Unable to open tty1. Check the USB connection or try rebooting.")

        try:
            self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.py)Cmds_PhrozenKlipperResume]Reinitializing serial port 2")
            self.G_SerialPort2Obj = serial.Serial(self.G_Serialport2Define, SERIAL_PORT_BAUD, timeout=3)
            #Serial port opened successfully
            if self.G_SerialPort2Obj is not None:
                if self.G_SerialPort2Obj.is_open:
                    self.G_SerialPort2OpenFlag = True
                    self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 2successful")
                    self.G_SerialPort2Obj.flushInput()  # clean serial write cache
                    self.G_SerialPort2Obj.flush()
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 buffers cleared")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Re-registering serial port 2 callback")
                    self.G_SerialPort2RecvTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerUart2Recv, self.G_PhrozenReactor.NOW)
        except:
            self.G_PhrozenFluiddRespondInfo("[DEBUG] AMS2/tty2 not available; skipping serial port 2.")




        self.G_PhrozenFluiddRespondInfo("[INFO] Current mode")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:0,unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:1,MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:2,MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo("[WARN] +Mode:3,RUNOUT")
        else:
            self.G_PhrozenFluiddRespondInfo("[ERROR] +Mode:-1,error")


        # lancaigang241108: has in state down pause
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:#0
            self.G_PhrozenFluiddRespondInfo('[INFO] in printingmode, do not executeresume,return')
            self.G_PhrozenFluiddRespondInfo("+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan)
            return


        #lancaigang250510: 
        if self.PG102Flag==True:
            self.G_PhrozenFluiddRespondInfo('[INFO] normal in purge, not allowedresume')
            self.G_PhrozenFluiddRespondInfo("+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan)
            return


        # #lancaigang231216: 
        # eventtime = self.G_PhrozenReactor.monotonic()
        # # Determine "printing" status
        # idle_timeout = self.G_PhrozenPrinter.lookup_object("idle_timeout")
        # is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
        # if is_printing:
        # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_PhrozenKlipperResume]in progressprinting; command='%s'")
        # else:
        # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_PhrozenKlipperResume] in printing; command='%s; return'")
        #     return
        self.G_PhrozenFluiddRespondInfo("[INFO] klipperresume")

        # lancaigang240325: MCmode logical, and time pause time output
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.G_PhrozenFluiddRespondInfo('[INFO] multi-materialmoderesume, #1 time pause is, after output time pausedo not executeresume')
        
            # lancaigang241011: first logical resumeAMS move reportpause
            self.STM32ReprotPauseFlag=0

        else:
            self.G_PhrozenFluiddRespondInfo("[INFO] single-color、single-color refill moderesume")
            # lancaigang240108: pausestate
            self.G_KlipperIfPaused = False
            # lancaigang240124: stm32 move report, canpause1 time
            self.STM32ReprotPauseFlag=0

            #lancaigang241106: 
            #self.G_P0M2MAStartPrintFlag=0


        #lancaigang240325:
        # lancaigang240426: resume after set false
        self.G_ResumeProcessCheckPauseStatus=False
        # lancaigang231207:+PAUSE:1feed
        self.G_IfInFilaBlockFlag=False
        # lancaigang240321: PG102 pause
        self.PG102DelayPauseFlag=False
        # lancaigang240325: resumestate
        self.G_ChangeChannelResumeFlag=True
        # lancaigang231207: P1 C?Automatic filament changewhen, if to resume, from #1 time channel
        self.G_ChangeChannelFirstFilaFlag=True
        #self.G_PhrozenFluiddRespondInfo("+RESUME:1,%d" % self.G_ChangeChannelTimeoutNewChan)
        
        # lancaigang250812:single-colorfilament runoutdetect, to pause
        self.G_RetryToPauseAreaFlag = False
        self.G_RetryToPauseAreaCount = 0





        # =====lancaigang231212: serial port disable or network move pauseresume, iftoolheadhasdetectto filament, then resume, do notstm32then feed
        # lancaigang240108: move pauseneed totoolhead is no hasfilament, after to logical
        if self.G_IfToolheadHaveFilaInitiativePauseFlag  == True:
            self.G_IfToolheadHaveFilaInitiativePauseFlag=False

            # lancaigang240103: single-colorM2MArefillmode, need to send stm32resumestate, preventstm32errorno refill
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                self.G_PhrozenFluiddRespondInfo('[INFO] M2MAmoderesume, move pauseresume')
                #lancaigang240122: 
                self.AMSRunoutPauseTimeCount = 0
                #lancaigang240123: 
                self.AMSRunoutPauseTimeoutFlag=0

                # hasfilamentcanresume
                if self.G_ToolheadIfHaveFilaFlag:
                    self.G_M2MAModeResumeFlag=True
                    # lancaigang240412: single-colormode, ifhasAMSmulti-material, need toresumeAMS
                    if self.G_AMSDevice1IfNormal==True or self.G_AMSDevice2IfNormal==True:
                        self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, has AMSmulti-material, do notfilament, need to sending commandSTM32resume after state')
                        # #self.Cmds_CmdP8(gcmd)
                        # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("FA")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1 send FA")
                        # elif self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("FA")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2 send FA")

                        #lancaigang250607:
                        self.G_PhrozenFluiddRespondInfo('[INFO] canresume, STM32printing report error')
                        self.G_KlipperQuickPause = True
                        # #lancaigang250427: 
                        # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-AMSfinishtimingbuffer-full time")
                        # if self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-AMSfinishtimingbuffer-full time")
                        # self.G_ProzenToolhead.dwell(1.5)

                        #lancaigang250522: 
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG109-heat up")
                        command_string = """
                            PG109
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-PG109-; command_string='%s'" % command_string)
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+RESTORE")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1-resume")
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+RESTORE")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2-resume")
                        # #lancaigang250611: 
                        # self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG108-heat uppurgewipe nozzle")
                        # command_string = """
                        #     PG108
                        #     """
                        # self.G_PhrozenGCode.run_script_from_command(command_string)
                        # self.G_PhrozenFluiddRespondInfo("[WARN] serial port disable or network move pauseresume, toolheadhasfilament, resumedo notthen feed")
                        # lancaigang240125: number
                        self.Cmds_PhrozenKlipperResumeCommon()
                        self.G_ChangeChannelResumeFlag=False
                        self.G_PhrozenFluiddRespondInfo("+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan)
                    else:
                        self.G_KlipperQuickPause = False
                        self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, has has AMSmulti-material, resume')
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port disable or network move pauseresume, toolheadhas filament, resumedo notthen feed')

                        #lancaigang250522: 
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG109-heat up")
                        command_string = """
                            PG109
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-PG109-; command_string='%s'" % command_string)
                        # #lancaigang250611: 
                        # self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG108-heat uppurgewipe nozzle")
                        # command_string = """
                        #     PG108
                        #     """
                        # self.G_PhrozenGCode.run_script_from_command(command_string)
                        # lancaigang240125: number
                        self.Cmds_PhrozenKlipperResumeCommon()
                        self.G_ChangeChannelResumeFlag=False
                        self.G_PhrozenFluiddRespondInfo("+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan)
                else:
                    if self.G_AMSDevice1IfNormal==True or self.G_AMSDevice2IfNormal==True:
                        self.G_PhrozenFluiddRespondInfo('[INFO] toolheadnonefilament, has AMSmulti-material, P8feed')
                        #lancaigang241106: 
                        self.G_P0M2MAStartPrintFlag=0

                        # lancaigang250522: not allowedM3filament runoutdetect
                        self.G_IfChangeFilaOngoing = True

                        #lancaigang241106: 
                        self.Cmds_CmdP8(gcmd)
                        # lancaigang250619:checkAMS is no re connectsuccessful
                        self.Cmds_USBConnectErrorCheck()
                        # lancaigang241106:toolheadsuccessfulfeed
                        if self.G_P0M2MAStartPrintFlag==1:
                            #lancaigang250607:
                            self.G_PhrozenFluiddRespondInfo('[INFO] canresume, STM32printing report error')
                            self.G_KlipperQuickPause = True
                            # #lancaigang250427: 
                            # if self.G_SerialPort1OpenFlag == True:
                            #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                            # self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-AMSfinishtimingbuffer-full time")
                            # if self.G_SerialPort2OpenFlag == True:
                            #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                            # self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-AMSfinishtimingbuffer-full time")
                            # self.G_ProzenToolhead.dwell(1.5)
                            # lancaigang250423: feedsuccessful, purge, AMSwhen, ifpurge5buffer is state, head
                            #self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
                            # self.G_PhrozenFluiddRespondInfo("[INFO] AMSstart timingbuffer-full time")
                            if self.G_SerialPort1OpenFlag == True:
                                self.Cmds_AMSSerial1Send("AT+EBLOCKCHECK")
                                self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1-AMSstart timingbuffer-full time")
                            if self.G_SerialPort2OpenFlag == True:
                                self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
                                self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2-AMSstart timingbuffer-full time")
                            #self.G_ProzenToolhead.dwell(1)
                            # lancaigang251120: purge, add, preventPG108purge toolheadnofilamentpause, pauseposition in purge, resumewhen will to purge;
                            self.G_PG108Ingoing=1
                            #lancaigang250611: 
                            self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG108-heat uppurgewipe nozzle")
                            command_string = """
                                PG108
                                """
                            self.G_PhrozenGCode.run_script_from_command(command_string)
                            self.G_PG108Ingoing=0
                            #lancaigang250427: 
                            if self.G_SerialPort1OpenFlag == True:
                                self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                                self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1-AMSfinishtimingbuffer-full time")
                            if self.G_SerialPort2OpenFlag == True:
                                self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                                self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2-AMSfinishtimingbuffer-full time")
                            
                            if self.STM32ReprotPauseFlag == 1:
                                self.G_PhrozenFluiddRespondInfo('[INFO] STM32 up report pause, cannotresume')
                                # lancaigang240125: number
                                #self.Cmds_PhrozenKlipperResumeCommon()
                                self.G_ChangeChannelResumeFlag=False
                                self.G_PhrozenFluiddRespondInfo("+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                            else:
                                self.G_PhrozenFluiddRespondInfo("[INFO] toolheadhas filament, resume")
                                # lancaigang240125: number
                                self.Cmds_PhrozenKlipperResumeCommon()
                                self.G_ChangeChannelResumeFlag=False
                                self.G_PhrozenFluiddRespondInfo("+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan)
                        else:
                            self.G_KlipperQuickPause = False
                            self.G_PhrozenFluiddRespondInfo('[INFO] toolheadnonefilament, single-color refill modepause')
                            if self.G_KlipperIfPaused == False:
                                self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE")
                                self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)]PAUSE")
                                self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                                self.G_ProzenToolhead.wait_moves()
                                self.G_KlipperIfPaused=True
                                #self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                                self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                            else:
                                self.G_PhrozenFluiddRespondInfo('[INFO] already pause, do notpause')
                            self.G_ChangeChannelResumeFlag=False
                            self.G_PhrozenFluiddRespondInfo("+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                    else:
                        self.G_PhrozenFluiddRespondInfo('[INFO] toolheadnonefilament, has AMSmulti-material, single-color refill modepause')
                        if self.G_KlipperIfPaused == False:
                            self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE")
                            self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)]PAUSE")
                            self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                            self.G_ProzenToolhead.wait_moves()
                            self.G_KlipperIfPaused=True
                            #self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                            self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        else:
                            self.G_PhrozenFluiddRespondInfo('[INFO] already pause, do notpause')
                        self.G_ChangeChannelResumeFlag=False
                        self.G_PhrozenFluiddRespondInfo("+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan)

                return


            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                self.G_PhrozenFluiddRespondInfo('[INFO] M3moderesume, move pauseresume')
                # #lancaigang241106:hasAMSmulti-material
                # if self.G_AMSDevice1IfNormal==True or self.G_AMSDevice2IfNormal==True:
                # self.G_PhrozenFluiddRespondInfo("[DEBUG] single-colorM3mode, hasAMSmulti-material, need to send stm32resumestate")
                #     # #lancaigang240416:
                #     # if self.G_SerialPort1OpenFlag == True:
                #     #     self.Cmds_AMSSerial1Send("MA")
                # #     self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-MA")
                #     # #lancaigang241030:
                #     # elif self.G_SerialPort2OpenFlag == True:
                #     #     self.Cmds_AMSSerial2Send("MA")
                # #     self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-MA")

                #     # time.sleep(2)

                #     # #lancaigang240416:
                #     # if self.G_SerialPort1OpenFlag == True:
                #     #     self.Cmds_AMSSerial1Send("FA")
                # #     self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-FA")
                #     # #lancaigang241030:
                #     # elif self.G_SerialPort2OpenFlag == True:
                #     #     self.Cmds_AMSSerial2Send("FA")
                # #     self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-FA")

                #     #lancaigang241106: 
                #     self.Cmds_CmdP8(gcmd)
                # #lancaigang241106:toolheadsuccessfulfeed
                #     if self.G_P0M2MAStartPrintFlag==1:
                # self.G_PhrozenFluiddRespondInfo("[WARN] serial port disable or network move pauseresume, toolheadhasfilament, resumedo notthen feed")
                # #lancaigang240125: number
                #         self.Cmds_PhrozenKlipperResumeCommon()
                #     else:
                # self.G_PhrozenFluiddRespondInfo("[WARN] toolheadnonefilament, single-colorrefillpause")
                #         self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                #         self.G_PhrozenFluiddRespondInfo("[WARN] send_pause_command")
                # #no filamentpause
                #         self.G_KlipperIfPaused=True
                #         self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                # else:
                #     if self.G_ToolheadIfHaveFilaFlag==True:
                # self.G_PhrozenFluiddRespondInfo("[INFO] single-colorM3mode, noAMSmulti-material, klipperresume")
                # self.G_PhrozenFluiddRespondInfo("[WARN] serial port disable or network move pauseresume, toolheadhasfilament, resumedo notthen feed")
                # #lancaigang240125: number
                #         self.Cmds_PhrozenKlipperResumeCommon()
                #     else:
                # self.G_PhrozenFluiddRespondInfo("[WARN] toolheadnonefilament, single-colorrefillpause")
                #         self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                #         self.G_PhrozenFluiddRespondInfo("[WARN] send_pause_command")
                # #no filamentpause
                #         self.G_KlipperIfPaused=True
                #         self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)

                # hasfilamentcanresume
                if self.G_ToolheadIfHaveFilaFlag:
                    # lancaigang240412: M3mode, ifhasAMSmulti-material, need toresumeAMS
                    if self.G_AMSDevice1IfNormal==True or self.G_AMSDevice2IfNormal==True:
                        self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, has AMSmulti-material, do notfilament, need to sending commandSTM32resume after state')
                        # #self.Cmds_CmdP8(gcmd)
                        # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("FA")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1 send FA")
                        # elif self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("FA")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2 send FA")
                        #lancaigang250607:
                        self.G_PhrozenFluiddRespondInfo('[INFO] canresume, STM32printing report error')
                        self.G_KlipperQuickPause = True
                        # #lancaigang250427: 
                        # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-AMSfinishtimingbuffer-full time")
                        # if self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-AMSfinishtimingbuffer-full time")
                        # self.G_ProzenToolhead.dwell(1.5)

                        #lancaigang250522: 
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG109-heat up")
                        command_string = """
                            PG109
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-PG109-; command_string='%s'" % command_string)
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+RESTORE")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1-resume")
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+RESTORE")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2-resume")

                        # #lancaigang250611: 
                        # self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG108-heat uppurgewipe nozzle")
                        # command_string = """
                        #     PG108
                        #     """
                        # self.G_PhrozenGCode.run_script_from_command(command_string)
                        # self.G_PhrozenFluiddRespondInfo("[WARN] serial port disable or network move pauseresume, toolheadhasfilament, resumedo notthen feed")
                        # lancaigang240125: number
                        self.Cmds_PhrozenKlipperResumeCommon()
                        self.G_ChangeChannelResumeFlag=False
                        self.G_PhrozenFluiddRespondInfo("+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan)
                    else:
                        self.G_KlipperQuickPause = False
                        self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, has has AMSmulti-material, resume')
                        #lancaigang250522: 
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG109-heat up")
                        command_string = """
                            PG109
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-PG109-; command_string='%s'" % command_string)
                        # self.G_PhrozenFluiddRespondInfo("[WARN] serial port disable or network move pauseresume, toolheadhasfilament, resumedo notthen feed")
                        # #lancaigang250409: resumewhen then purge
                        # command_string = """
                        # PG108
                        # """
                        # self.G_PhrozenGCode.run_script_from_command(command_string)
                        # self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)
                        # self.G_PhrozenFluiddRespondInfo("[INFO] purgecomplete, toolheaddetectto hasfilamentresumeprinting")
                        # lancaigang240125: number
                        self.Cmds_PhrozenKlipperResumeCommon()
                        self.G_ChangeChannelResumeFlag=False
                        self.G_PhrozenFluiddRespondInfo("+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan)
                else:
                    if self.G_AMSDevice1IfNormal==True or self.G_AMSDevice2IfNormal==True:
                        self.G_PhrozenFluiddRespondInfo('[INFO] toolheadnonefilament, has AMSmulti-material, P8feed')
                        #lancaigang241106: 
                        self.G_P0M2MAStartPrintFlag=0

                        # lancaigang250522: not allowedM3filament runoutdetect
                        self.G_IfChangeFilaOngoing = True

                        #lancaigang241106: 
                        self.Cmds_CmdP8(gcmd)
                        # lancaigang250619:checkAMS is no re connectsuccessful
                        self.Cmds_USBConnectErrorCheck()
                        # lancaigang241106:toolheadsuccessfulfeed
                        if self.G_P0M2MAStartPrintFlag==1:
                            #lancaigang250607:
                            self.G_PhrozenFluiddRespondInfo('[INFO] canresume, STM32printing report error')
                            self.G_KlipperQuickPause = True
                            # #lancaigang250427: 
                            # if self.G_SerialPort1OpenFlag == True:
                            #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                            # self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-AMSfinishtimingbuffer-full time")
                            # if self.G_SerialPort2OpenFlag == True:
                            #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                            # self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-AMSfinishtimingbuffer-full time")
                            # self.G_ProzenToolhead.dwell(1.5)
                            if self.G_SerialPort1OpenFlag == True:
                                self.Cmds_AMSSerial1Send("AT+EBLOCKCHECK")
                                self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1-AMSstart timingbuffer-full time")
                            if self.G_SerialPort2OpenFlag == True:
                                self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
                                self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2-AMSstart timingbuffer-full time")
                            #self.G_ProzenToolhead.dwell(1)
                            # lancaigang251120: purge, add, preventPG108purge toolheadnofilamentpause, pauseposition in purge, resumewhen will to purge;
                            self.G_PG108Ingoing=1
                            #lancaigang250611: 
                            self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG108-heat uppurgewipe nozzle")
                            command_string = """
                                PG108
                                """
                            self.G_PhrozenGCode.run_script_from_command(command_string)
                            self.G_PG108Ingoing=0
                            #lancaigang250427: 
                            if self.G_SerialPort1OpenFlag == True:
                                self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                                self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1-AMSfinishtimingbuffer-full time")
                            if self.G_SerialPort2OpenFlag == True:
                                self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                                self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2-AMSfinishtimingbuffer-full time")
                            if self.STM32ReprotPauseFlag == 1:
                                self.G_PhrozenFluiddRespondInfo('[INFO] STM32 up report pause, cannotresume')
                                # lancaigang240125: number
                                #self.Cmds_PhrozenKlipperResumeCommon()
                                self.G_ChangeChannelResumeFlag=False
                                self.G_PhrozenFluiddRespondInfo("+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                            else:
                                self.G_PhrozenFluiddRespondInfo("[INFO] toolheadhas filament, resume")
                                # lancaigang240125: number
                                self.Cmds_PhrozenKlipperResumeCommon()
                                self.G_ChangeChannelResumeFlag=False
                                self.G_PhrozenFluiddRespondInfo("+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan)
                        else:
                            self.G_KlipperQuickPause = False
                            self.G_PhrozenFluiddRespondInfo('[INFO] toolheadnonefilament, M3moderefillpause')
                            if self.G_KlipperIfPaused == False:
                                self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE")
                                self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)]PAUSE")
                                self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                                self.G_ProzenToolhead.wait_moves()
                                self.G_KlipperIfPaused=True
                                #self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                                self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                            else:
                                self.G_PhrozenFluiddRespondInfo('[INFO] already pause, do notpause')
                            self.G_ChangeChannelResumeFlag=False
                            self.G_PhrozenFluiddRespondInfo("+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                    else:
                        self.G_KlipperQuickPause = False
                        self.G_PhrozenFluiddRespondInfo('[INFO] toolheadnonefilament, has AMSmulti-material, M3modepause')
                        self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE")
                        self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)]PAUSE")
                        self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                        self.G_ProzenToolhead.wait_moves()
                        # no filamentpause
                        self.G_KlipperIfPaused=True
                        #self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                        self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        self.G_ChangeChannelResumeFlag=False
                        self.G_PhrozenFluiddRespondInfo("+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan)

                        


                return


            # lancaigang240115: move pauseresume, resumestm32stateto refillstate
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
                self.G_PhrozenFluiddRespondInfo("[INFO] M1MCmoderesume")
                # lancaigang240521: resumewhen, ifdetectedAMSerrorrestart, can is is AMS, unload filament
                if self.G_ResumeCheckAMS1ErrorRestartFlag == True:
                    self.G_ResumeCheckAMS1ErrorRestartFlag=False
                    self.G_PhrozenFluiddRespondInfo('[INFO] serial port disable or network move pauseresume;multi-materialMCmode; detectedAMSerrorrestart, resume')
                else:
                    self.G_PhrozenFluiddRespondInfo('[INFO] serial port disable or network move pauseresume;multi-materialMCmode, stm32resumeprintingrefillstate')
                    #self.Cmds_AMSSerial1Send("AT+MCRS=F")
                    #self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_PhrozenKlipperResume]AT+MCRS=F")
                    # lancaigang240115: if disable move pause, action manualcommand, stm32state will change change, no pause before state, re P1 C?command
                    # iftoolheadhasfilament, stm32can switch to refillstate
                    if self.G_ToolheadIfHaveFilaFlag==True:
                        # #lancaigang241030:
                        # if self.G_ChangeChannelTimeoutNewChan in range(1, 4):
                        # #lancaigang0427: is no hasmanualcommand, resumestm32when, is refillstate
                        # self.Cmds_AMSSerial1Send("AT+MCRS=5,%d" % self.G_ChangeChannelTimeoutNewChan)#05=refillstate
                        #     self.G_PhrozenFluiddRespondInfo("AT+MCRS=5,%d" % self.G_ChangeChannelTimeoutNewChan)
                        # elif self.G_ChangeChannelTimeoutNewChan in range(5, 8):
                        # self.Cmds_AMSSerial2Send("AT+MCRS=5,%d" % self.G_ChangeChannelTimeoutNewChan-4)#05=refillstate
                        #     self.G_PhrozenFluiddRespondInfo("AT+MCRS=5,%d" % self.G_ChangeChannelTimeoutNewChan-4)

                        self.G_PhrozenFluiddRespondInfo('[INFO] multi-materialMCmode, send stm32resumeprintingrefillstate')
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port disable or network move pauseresume, toolheadhas filament')
                        
                        # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("AT+RESTORE")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-resume")
                        # if self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("AT+RESTORE")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-resume")

                        # #lancaigang241012: move pauseneed tore feed, preventAMSstateerror

                        # #lancaigang240125: number
                        # self.Cmds_PhrozenKlipperResumeCommon()
                        # self.G_ChangeChannelResumeFlag=False
                        # self.G_PhrozenFluiddRespondInfo("+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan)
                        # return
                    
                    else:
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port disable or network move pauseresume;toolheadhas filament, resume')




        # =====lancaigang231229: MAsingle-colorrefill logical, and machine single-color
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.G_PhrozenFluiddRespondInfo("[INFO] M2MAmoderesume")
            #lancaigang240122: 
            self.AMSRunoutPauseTimeCount = 0
            #lancaigang240123: 
            self.AMSRunoutPauseTimeoutFlag=0

            # # #lancaigang240416:
            # # if self.G_SerialPort1OpenFlag == True:
            # #     self.Cmds_AMSSerial1Send("MA")
            # #     self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-MA")
            # # #lancaigang241030:
            # # elif self.G_SerialPort2OpenFlag == True:
            # #     self.Cmds_AMSSerial2Send("MA")
            # #     self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-MA")

            # # #lancaigang240115:when 1, prevent
            # # time.sleep(2)

            # #hasfilamentcanresume
            # if self.G_ToolheadIfHaveFilaFlag:
            # # #lancaigang231228: resumeafter, sending commandstm32resume up time state machine state
            # # #resumestateRS=F,resume up time state
            # # #resumestateRS=0,resume up MASTATEMACHINE_IDLE_STANDBYstate
            # # #resumestateRS=X,...
            # # #resumestateRS=Y,...
            # # #resumestateRS=Z,...
            # # #lancaigang240108: first send
            #     # #self.Cmds_AMSSerial1Send("AT+MARS=F")
            #     # #self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_PhrozenKlipperResume]AT+MARS=F")
            # # #lancaigang240108: re old action channel, filament tubefilament is filament runouterror
            # # #lancaigang240226: toolheadhasfilament, do not send FA
            #     # #lancaigang240416:
            #     # if self.G_SerialPort1OpenFlag == True:
            #     #     self.Cmds_AMSSerial1Send("FA")
            # #     self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-FA")
            #     # #lancaigang241030:
            #     # elif self.G_SerialPort2OpenFlag == True:
            #     #     self.Cmds_AMSSerial2Send("FA")
            # #     self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-FA")

            # # self.G_PhrozenFluiddRespondInfo("[INFO] single-colorM2MArefillmode, toolheadhasfilamentresume")
            # # #lancaigang240108: extrudecompletethen resume
            #     # #if self.P0M3FilaRunoutSpittingFinished == True:
            # # #self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_PhrozenKlipperResume]extrudecomplete, canresume")


            #     #lancaigang241106: 
            #     self.Cmds_CmdP8(gcmd)
            # #lancaigang241106:toolheadsuccessfulfeed
            #     if self.G_P0M2MAStartPrintFlag==1:
            # self.G_PhrozenFluiddRespondInfo("[INFO] toolheadhasfilament, then feed")
            # #lancaigang240125: number
            #         self.Cmds_PhrozenKlipperResumeCommon()
            #     else:
            # self.G_PhrozenFluiddRespondInfo("[WARN] toolheadnonefilament, single-colorrefillpause")
            #         self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
            #         self.G_PhrozenFluiddRespondInfo("[WARN] send_pause_command")
            # #no filamentpause
            #         self.G_KlipperIfPaused=True
            #         self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d" % self.G_ChangeChannelTimeoutNewChan)

            #     #lancaigang241106: 
            #     #self.Cmds_CmdP8(gcmd)
            #     #self.Cmds_PhrozenKlipperResumeCommon()


            # #lancaigang240108: toolheadhasfilament, canresume
            #     self.G_M2MAModeResumeFlag=True

            #     self.G_ChangeChannelResumeFlag=False
            #     self.G_PhrozenFluiddRespondInfo("+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan)

            #     return
            # #nofilamentneed tore feed
            # else:
            # #lancaigang240108: resumewhen, allowedfilament runoutextrude
            #     self.P0M3FilaRunoutSpittingFinished = False
            #     self.G_ToolheadFirstInputFila=False
            # #lancaigang240108: toolheadhasfilament, canresume
            # #lancaigang240109: feed is no timeout to toolheaddetectto filament, canresume action
            #     self.G_M2MAModeResumeFlag=True

            # self.G_PhrozenFluiddRespondInfo("[INFO] single-colorM2MArefillmode, toolheadnonefilamentneed tore feed new filament")



            # # #lancaigang240103: toolheadno filament, need tore feed, re feed sequence, single-colorautomaticrefillF8
            #     # #ttyUSB0Serial send: FA
            # # #lancaigang240108: first send FA
            #     # #lancaigang240416:
            #     # if self.G_SerialPort1OpenFlag == True:
            #     #     self.Cmds_AMSSerial1Send("FA")
            # #     self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-FA")
            #     # #lancaigang241030:
            #     # elif self.G_SerialPort2OpenFlag == True:
            #     #     self.Cmds_AMSSerial2Send("FA")
            # #     self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-FA")
            # # #lancaigang231229: number
            #     # self.Cmds_MARetryInFila(gcmd)

            #     #lancaigang241106: 
            #     self.Cmds_CmdP8(gcmd)
            # #lancaigang241106:toolheadsuccessfulfeed
            #     if self.G_P0M2MAStartPrintFlag==1:
            # self.G_PhrozenFluiddRespondInfo("[INFO] toolheadhasfilament, then feed")
            # #lancaigang240125: number
            #         self.Cmds_PhrozenKlipperResumeCommon()
            #     else:
            # self.G_PhrozenFluiddRespondInfo("[WARN] toolheadnonefilament, single-colorrefillpause")
            #         self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
            #         self.G_PhrozenFluiddRespondInfo("[WARN] send_pause_command")
            # #no filamentpause
            #         self.G_KlipperIfPaused=True
            #         self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d" % self.G_ChangeChannelTimeoutNewChan)

            # hasfilamentcanresume
            if self.G_ToolheadIfHaveFilaFlag:
                self.G_M2MAModeResumeFlag=True
                # lancaigang240412: M2MAmode, ifhasAMSmulti-material, need toresumeAMS
                if self.G_AMSDevice1IfNormal==True or self.G_AMSDevice2IfNormal==True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, has AMSmulti-material, but is P8feed, prevent stop normal')
                    self.G_P0M2MAStartPrintFlag=0

                    # lancaigang250522: not allowedM3filament runoutdetect
                    self.G_IfChangeFilaOngoing = True

                    self.Cmds_CmdP8(gcmd)
                    # lancaigang250619:checkAMS is no re connectsuccessful
                    self.Cmds_USBConnectErrorCheck()
                    # if self.G_SerialPort1OpenFlag == True:
                    #     self.Cmds_AMSSerial1Send("FA")
                    # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1 send FA")
                    # elif self.G_SerialPort2OpenFlag == True:
                    #     self.Cmds_AMSSerial2Send("FA")
                    # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2 send FA")
                    # lancaigang241106:toolheadsuccessfulfeed
                    if self.G_P0M2MAStartPrintFlag==1:
                        #lancaigang250607:
                        self.G_PhrozenFluiddRespondInfo('[INFO] canresume, STM32printing report error')
                        self.G_KlipperQuickPause = True
                        # #lancaigang250427: 
                        # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-AMSfinishtimingbuffer-full time")
                        # if self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-AMSfinishtimingbuffer-full time")
                        # self.G_ProzenToolhead.dwell(1.5)
                        #lancaigang250522: 
                        # self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG109-heat up")
                        # command_string = """
                        #     PG109
                        #     """
                        # self.G_PhrozenGCode.run_script_from_command(command_string)
                        # self.G_PhrozenFluiddRespondInfo("External macro command-PG109-; command_string='%s'" % command_string)
                        # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("AT+RESTORE")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-resume")
                        # if self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("AT+RESTORE")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-resume")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] toolheadhasfilament, resume")
                        # #lancaigang250611: 
                        # self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG108-heat uppurgewipe nozzle")
                        # command_string = """
                        #     PG108
                        #     """
                        # self.G_PhrozenGCode.run_script_from_command(command_string)
                        # #lancaigang240125: number
                        # self.Cmds_PhrozenKlipperResumeCommon()
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+EBLOCKCHECK")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1-AMSstart timingbuffer-full time")
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2-AMSstart timingbuffer-full time")
                        #self.G_ProzenToolhead.dwell(1)
                        #lancaigang250522: 
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG109-heat up")
                        command_string = """
                            PG109
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-PG109-; command_string='%s'" % command_string)
                        # lancaigang251120: purge, add, preventPG108purge toolheadnofilamentpause, pauseposition in purge, resumewhen will to purge;
                        self.G_PG108Ingoing=1
                        #lancaigang250611: 
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG108-heat uppurgewipe nozzle")
                        command_string = """
                            PG108
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PG108Ingoing=0
                        #lancaigang250427: 
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1-AMSfinishtimingbuffer-full time")
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2-AMSfinishtimingbuffer-full time")
                        if self.STM32ReprotPauseFlag == 1:
                            self.G_PhrozenFluiddRespondInfo('[INFO] STM32 up report pause, cannotresume')
                            # lancaigang240125: number
                            #self.Cmds_PhrozenKlipperResumeCommon()
                            self.G_ChangeChannelResumeFlag=False
                            self.G_PhrozenFluiddRespondInfo("+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                        else:
                            self.G_PhrozenFluiddRespondInfo("[INFO] toolheadhas filament, resume")
                            # lancaigang240125: number
                            self.Cmds_PhrozenKlipperResumeCommon()
                            self.G_ChangeChannelResumeFlag=False
                            self.G_PhrozenFluiddRespondInfo("+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan)
                    else:
                        self.G_KlipperQuickPause = False
                        self.G_PhrozenFluiddRespondInfo('[INFO] toolheadnonefilament, M2MAmodepause')
                        if self.G_KlipperIfPaused == False:
                            self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE")
                            self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)]PAUSE")
                            self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                            self.G_ProzenToolhead.wait_moves()
                            # no filamentpause
                            self.G_KlipperIfPaused=True
                            #self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                            self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        else:
                            self.G_PhrozenFluiddRespondInfo('[INFO] already pause, do notpause')
                        self.G_ChangeChannelResumeFlag=False
                        self.G_PhrozenFluiddRespondInfo("+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                else:
                    self.G_KlipperQuickPause = False
                    self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, has has AMSmulti-material, resume')
                    self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, resumedo notthen feed')

                    #lancaigang250522: 
                    self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG109-heat up")
                    command_string = """
                        PG109
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.G_PhrozenFluiddRespondInfo("External macro command-PG109-; command_string='%s'" % command_string)
                    # lancaigang251120: purge, add, preventPG108purge toolheadnofilamentpause, pauseposition in purge, resumewhen will to purge;
                    self.G_PG108Ingoing=1
                    #lancaigang250611: 
                    self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG108-heat uppurgewipe nozzle")
                    command_string = """
                        PG108
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.G_PG108Ingoing=0
                    # lancaigang240125: number
                    self.Cmds_PhrozenKlipperResumeCommon()
                    self.G_ChangeChannelResumeFlag=False
                    self.G_PhrozenFluiddRespondInfo("+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan)
            else:
                if self.G_AMSDevice1IfNormal==True or self.G_AMSDevice2IfNormal==True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] toolheadnonefilament, has AMSmulti-material, P8feed')
                    #lancaigang241106: 
                    self.G_P0M2MAStartPrintFlag=0

                    # lancaigang250522: not allowedM3filament runoutdetect
                    self.G_IfChangeFilaOngoing = True

                    #lancaigang241106: 
                    self.Cmds_CmdP8(gcmd)
                    # lancaigang250619:checkAMS is no re connectsuccessful
                    self.Cmds_USBConnectErrorCheck()
                    # lancaigang241106:toolheadsuccessfulfeed
                    if self.G_P0M2MAStartPrintFlag==1:
                        #lancaigang250607:
                        self.G_PhrozenFluiddRespondInfo('[INFO] canresume, STM32printing report error')
                        self.G_KlipperQuickPause = True
                        # #lancaigang250427: 
                        # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-AMSfinishtimingbuffer-full time")
                        # if self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-AMSfinishtimingbuffer-full time")
                        # self.G_ProzenToolhead.dwell(1.5)
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+EBLOCKCHECK")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1-AMSstart timingbuffer-full time")
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2-AMSstart timingbuffer-full time")
                        #self.G_ProzenToolhead.dwell(1)
                        #lancaigang250522: 
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG109-heat up")
                        command_string = """
                            PG109
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-PG109-; command_string='%s'" % command_string)
                        # lancaigang251120: purge, add, preventPG108purge toolheadnofilamentpause, pauseposition in purge, resumewhen will to purge;
                        self.G_PG108Ingoing=1
                        #lancaigang250611: 
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG108-heat uppurgewipe nozzle")
                        command_string = """
                            PG108
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PG108Ingoing=0
                        #lancaigang250427: 
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1-AMSfinishtimingbuffer-full time")
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2-AMSfinishtimingbuffer-full time")
                        if self.STM32ReprotPauseFlag == 1:
                            self.G_PhrozenFluiddRespondInfo('[INFO] STM32 up report pause, cannotresume')
                            # lancaigang240125: number
                            #self.Cmds_PhrozenKlipperResumeCommon()
                            self.G_ChangeChannelResumeFlag=False
                            self.G_PhrozenFluiddRespondInfo("+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                        else:
                            self.G_PhrozenFluiddRespondInfo("[INFO] toolheadhas filament, resume")
                            # lancaigang240125: number
                            self.Cmds_PhrozenKlipperResumeCommon()
                            self.G_ChangeChannelResumeFlag=False
                            self.G_PhrozenFluiddRespondInfo("+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan)
                    else:
                        self.G_KlipperQuickPause = False
                        self.G_PhrozenFluiddRespondInfo('[INFO] toolheadnonefilament, M2MAmodepause')
                        if self.G_KlipperIfPaused == False:
                            self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE")
                            self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)]PAUSE")
                            self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                            self.G_ProzenToolhead.wait_moves()
                            self.G_KlipperIfPaused=True
                            #self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                            self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        else:
                            self.G_PhrozenFluiddRespondInfo('[INFO] already pause, do notpause')
                        self.G_ChangeChannelResumeFlag=False
                        self.G_PhrozenFluiddRespondInfo("+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                else:
                    self.G_KlipperQuickPause = False
                    self.G_PhrozenFluiddRespondInfo('[INFO] toolheadnonefilament, has AMSmulti-material, M2MApause')
                    self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE")
                    self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)]PAUSE")
                    self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                    self.G_ProzenToolhead.wait_moves()
                    self.G_KlipperIfPaused=True
                    #self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d" % self.G_ChangeChannelTimeoutNewChan)
                    self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    self.G_ChangeChannelResumeFlag=False
                    self.G_PhrozenFluiddRespondInfo("+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan)







            return
            




        # =====lancaigang231220: M3single-color, need tomanualrefill, hastoolheaddetectto filamentcanresume
        # machine M3filament runout logical mode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo("[INFO] M3moderesume")
            # #hasfilamentcanresume
            # if self.G_ToolheadIfHaveFilaFlag:
            # #lancaigang240412: single-colormode, ifhasAMSmulti-material, need toresumeAMS
            #     if self.G_AMSDevice1IfNormal==True or self.G_AMSDevice2IfNormal==True:
            #         # #lancaigang240416:
            #         # if self.G_SerialPort1OpenFlag == True:
            #         #     self.Cmds_AMSSerial1Send("MA")
            # #     self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-MA")
            #         # #lancaigang241030:
            #         # elif self.G_SerialPort2OpenFlag == True:
            #         #     self.Cmds_AMSSerial2Send("MA")
            # #     self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-MA")

            # # #lancaigang240115:when 1, prevent
            #         # time.sleep(2)

            # # #lancaigang241030:FA use
            #         # if self.G_SerialPort1OpenFlag == True:
            #         #     self.Cmds_AMSSerial1Send("FA")
            # #     self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-FA")
            #         # #lancaigang241030:
            #         # elif self.G_SerialPort2OpenFlag == True:
            #         #     self.Cmds_AMSSerial2Send("FA")
            # #     self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-FA")

            # #lancaigang241106: P8 full new feed, preventAMSerror or because restart
            #         self.Cmds_CmdP8(gcmd)
            # #lancaigang241106:toolheadsuccessfulfeed
            #         if self.G_P0M2MAStartPrintFlag==1:
            # self.G_PhrozenFluiddRespondInfo("[INFO] single-colorM3mode, AMSmulti-materialtoolheadhasfilamentresume")
            # #lancaigang240125: number
            #             self.Cmds_PhrozenKlipperResumeCommon()
            #         else:
            # self.G_PhrozenFluiddRespondInfo("[WARN] standaloneM3mode, hasAMSmulti-materialtoolheadnonefilamentpause, please manual add ")
            #             self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
            #             self.G_PhrozenFluiddRespondInfo("[WARN] send_pause_command")
            # #no filamentpause
            #             self.G_KlipperIfPaused=True
            #             self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
            #     else:
            # self.G_PhrozenFluiddRespondInfo("[INFO] standaloneM3mode, noAMSmulti-materialtoolheadhasfilamentresume")
            # # #lancaigang240411: manualextrudethen resume
            #         # command = """
            #         #     G90
            #         #     G1 X250 Y10 F10000 
            #         #     G91
            #         #     G1 Z10 F500
            #         #     G92 E0
            #         #     G1 E100 F500
            #         #     G92 E0
            #         #     G0
            #         # """
            #         # self.G_PhrozenGCode.run_script_from_command(command)
            #         # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]command=%s" % command)
            # self.G_PhrozenFluiddRespondInfo("[INFO] calling external all -PG108-single-colorM3modestartpurge")
            # #lancaigang240407: calling purgeFunction, in purgebefore, preventtoolheadfeed up output, time command report error
            # self.P0M3FilaRunoutSpittingFinished = True#purgecomplete, prevent time calling command
            #         # command = """
            #         #     G90
            #         #     G1 X250 Y10 F10000 
            #         #     G91
            #         #     G1 Z10 F500
            #         #     G92 E0
            #         #     G1 E100 F500
            #         #     G92 E0
            #         #     G0
            #         # """
            #         # self.G_PhrozenGCode.run_script_from_command(command)
            #         # self.G_PhrozenFluiddRespondInfo("[(dev.python)Cmds_PhrozenKlipperResume]command=%s" % command)
            #         command_string = """
            #         PG108
            #         """
            #         self.G_PhrozenGCode.run_script_from_command(command_string)
            #         self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)


            # #lancaigang240125: number
            #         self.Cmds_PhrozenKlipperResumeCommon()

            #     # #lancaigang241106: 
            #     # self.Cmds_CmdP8(gcmd)
            #     # self.Cmds_PhrozenKlipperResumeCommon()

            #     self.G_ChangeChannelResumeFlag=False
            #     self.G_PhrozenFluiddRespondInfo("+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan)


            #     return
            

            # #nofilamentpause
            # else:
            # #lancaigang240412: single-colormode, ifhasAMSmulti-material, need toresumeAMS
            #     if self.G_AMSDevice1IfNormal==True or self.G_AMSDevice2IfNormal==True:
            #         # #lancaigang240416:
            #         # if self.G_SerialPort1OpenFlag == True:
            #         #     self.Cmds_AMSSerial1Send("MA")
            # #     self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-MA")
            #         # #lancaigang241030:
            #         # elif self.G_SerialPort2OpenFlag == True:
            #         #     self.Cmds_AMSSerial2Send("MA")
            # #     self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-MA")

            # # #lancaigang240115:when 1, prevent
            #         # time.sleep(2)

            # # self.G_PhrozenFluiddRespondInfo("[INFO] single-colorM3mode, hasAMSmulti-materialtoolheadnonefilamentneed tore feed new filament")
            # # #lancaigang240103: toolheadno filament, need tore feed, re feed sequence, single-colorautomaticrefillF8
            #         # #ttyUSB0Serial send: FA
            # # #lancaigang240108: first send FA
            #         # #lancaigang240416:
            #         # if self.G_SerialPort1OpenFlag == True:
            #         #     self.Cmds_AMSSerial1Send("FA")
            # #     self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-FA")
            #         # #lancaigang241030:
            #         # elif self.G_SerialPort2OpenFlag == True:
            #         #     self.Cmds_AMSSerial2Send("FA")
            # #     self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-FA")

            # # #lancaigang231229: number
            #         # self.Cmds_MARetryInFila(gcmd)

            # #lancaigang241106: P8 full new feed, preventAMSerror or because restart
            #         self.Cmds_CmdP8(gcmd)
            # #lancaigang241106:toolheadsuccessfulfeed
            #         if self.G_P0M2MAStartPrintFlag==1:
            # self.G_PhrozenFluiddRespondInfo("[INFO] single-colorM3mode, AMSmulti-materialtoolheadhasfilamentresume")
            # #lancaigang240125: number
            #             self.Cmds_PhrozenKlipperResumeCommon()
            #         else:
            # self.G_PhrozenFluiddRespondInfo("[WARN] standaloneM3mode, hasAMSmulti-materialtoolheadnonefilamentpause, please manual add ")
            #             self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
            #             self.G_PhrozenFluiddRespondInfo("[WARN] send_pause_command")
            # #no filamentpause
            #             self.G_KlipperIfPaused=True
            #             self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)

            #     else:
            # self.G_PhrozenFluiddRespondInfo("[WARN] standaloneM3mode, noAMSmulti-materialtoolheadnonefilamentpause, please manual add ")
            #         self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
            #         self.G_PhrozenFluiddRespondInfo("[WARN] send_pause_command")
            # #no filamentpause
            #         self.G_KlipperIfPaused=True
            #         self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)



            # hasfilamentcanresume
            if self.G_ToolheadIfHaveFilaFlag:
                # lancaigang240412: M3mode, ifhasAMSmulti-material, need toresumeAMS
                if self.G_AMSDevice1IfNormal==True or self.G_AMSDevice2IfNormal==True:
                    # self.G_PhrozenFluiddRespondInfo("[DEBUG] toolheadhasfilament, hasAMSmulti-material, do notfilament, need tosending commandSTM32resume after state")
                    # #self.Cmds_CmdP8(gcmd)
                    # if self.G_SerialPort1OpenFlag == True:
                    #     self.Cmds_AMSSerial1Send("FA")
                    # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1 send FA")
                    # elif self.G_SerialPort2OpenFlag == True:
                    #     self.Cmds_AMSSerial2Send("FA")
                    # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2 send FA")
                    # self.G_PhrozenFluiddRespondInfo("[INFO] toolheadhasfilament, resumedo notthen feed")
                    self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, has AMSmulti-material, but is P8feed, prevent stop normal')
                    self.G_P0M2MAStartPrintFlag=0

                    # lancaigang250522: not allowedM3filament runoutdetect
                    self.G_IfChangeFilaOngoing = True

                    self.Cmds_CmdP8(gcmd)
                    # lancaigang250619:checkAMS is no re connectsuccessful
                    self.Cmds_USBConnectErrorCheck()
                    # if self.G_SerialPort1OpenFlag == True:
                    #     self.Cmds_AMSSerial1Send("FA")
                    # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1 send FA")
                    # elif self.G_SerialPort2OpenFlag == True:
                    #     self.Cmds_AMSSerial2Send("FA")
                    # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2 send FA")
                    # lancaigang241106:toolheadsuccessfulfeed
                    if self.G_P0M2MAStartPrintFlag==1:
                        #lancaigang250607:
                        self.G_PhrozenFluiddRespondInfo('[INFO] canresume, STM32printing report error')
                        self.G_KlipperQuickPause = True
                        # #lancaigang250427: 
                        # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-AMSfinishtimingbuffer-full time")
                        # if self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-AMSfinishtimingbuffer-full time")
                        # self.G_ProzenToolhead.dwell(1.5)
                        #lancaigang250522: 
                        # self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG109-heat up")
                        # command_string = """
                        #     PG109
                        #     """
                        # self.G_PhrozenGCode.run_script_from_command(command_string)
                        # self.G_PhrozenFluiddRespondInfo("External macro command-PG109-; command_string='%s'" % command_string)
                        # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("AT+RESTORE")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-resume")
                        # if self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("AT+RESTORE")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-resume")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] toolheadhasfilament, resume")
                        # #lancaigang250611: 
                        # self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG108-heat uppurgewipe nozzle")
                        # command_string = """
                        #     PG108
                        #     """
                        # self.G_PhrozenGCode.run_script_from_command(command_string)
                        # #lancaigang240125: number
                        # self.Cmds_PhrozenKlipperResumeCommon()
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+EBLOCKCHECK")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1-AMSstart timingbuffer-full time")
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2-AMSstart timingbuffer-full time")
                        #self.G_ProzenToolhead.dwell(1)
                        #lancaigang250522: 
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG109-heat up")
                        command_string = """
                            PG109
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-PG109-; command_string='%s'" % command_string)
                        # lancaigang251120: purge, add, preventPG108purge toolheadnofilamentpause, pauseposition in purge, resumewhen will to purge;
                        self.G_PG108Ingoing=1
                        #lancaigang250611: 
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG108-heat uppurgewipe nozzle")
                        command_string = """
                            PG108
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PG108Ingoing=0
                        #lancaigang250427: 
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1-AMSfinishtimingbuffer-full time")
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2-AMSfinishtimingbuffer-full time")
                        if self.STM32ReprotPauseFlag == 1:
                            self.G_PhrozenFluiddRespondInfo('[INFO] STM32 up report pause, cannotresume')
                            # lancaigang240125: number
                            #self.Cmds_PhrozenKlipperResumeCommon()
                            self.G_ChangeChannelResumeFlag=False
                            self.G_PhrozenFluiddRespondInfo("+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                        else:
                            self.G_PhrozenFluiddRespondInfo("[INFO] toolheadhas filament, resume")
                            # lancaigang240125: number
                            self.Cmds_PhrozenKlipperResumeCommon()
                            self.G_ChangeChannelResumeFlag=False
                            self.G_PhrozenFluiddRespondInfo("+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan)
                    else:
                        self.G_KlipperQuickPause = False
                        self.G_PhrozenFluiddRespondInfo('[INFO] toolheadnonefilament, M3modepause')
                        if self.G_KlipperIfPaused == False:
                            self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE")
                            self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)]PAUSE")
                            self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                            self.G_ProzenToolhead.wait_moves()
                            self.G_KlipperIfPaused=True
                            #self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                            self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        else:
                            self.G_PhrozenFluiddRespondInfo('[INFO] already pause, do notpause')
                        self.G_ChangeChannelResumeFlag=False
                        self.G_PhrozenFluiddRespondInfo("+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                else:
                    self.G_KlipperQuickPause = False
                    self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, has has AMSmulti-material, resume')
                    self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, resumedo notthen feed')
                    #lancaigang250522: 
                    self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG109-heat up")
                    command_string = """
                        PG109
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.G_PhrozenFluiddRespondInfo("External macro command-PG109-; command_string='%s'" % command_string)
                    # lancaigang251120: purge, add, preventPG108purge toolheadnofilamentpause, pauseposition in purge, resumewhen will to purge;
                    self.G_PG108Ingoing=1
                    # lancaigang250409: resumewhen then purge
                    command_string = """
                    PG108
                    """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.G_PG108Ingoing=0
                    self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)
                    self.G_PhrozenFluiddRespondInfo("[INFO] purgecomplete, toolheaddetectedhas filamentresume printing")
                    # lancaigang240125: number
                    self.Cmds_PhrozenKlipperResumeCommon()
                    self.G_ChangeChannelResumeFlag=False
                    self.G_PhrozenFluiddRespondInfo("+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan)
            else:
                if self.G_AMSDevice1IfNormal==True or self.G_AMSDevice2IfNormal==True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] toolheadnonefilament, has AMSmulti-material, P8feed')
                    #lancaigang241106: 
                    self.G_P0M2MAStartPrintFlag=0

                    # lancaigang250522: not allowedM3filament runoutdetect
                    self.G_IfChangeFilaOngoing = True

                    #lancaigang241106: 
                    self.Cmds_CmdP8(gcmd)
                    # lancaigang250619:checkAMS is no re connectsuccessful
                    self.Cmds_USBConnectErrorCheck()
                    # lancaigang241106:toolheadsuccessfulfeed
                    if self.G_P0M2MAStartPrintFlag==1:
                        #lancaigang250607:
                        self.G_PhrozenFluiddRespondInfo('[INFO] canresume, STM32printing report error')
                        self.G_KlipperQuickPause = True
                        # #lancaigang250427: 
                        # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-AMSfinishtimingbuffer-full time")
                        # if self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-AMSfinishtimingbuffer-full time")
                        # self.G_ProzenToolhead.dwell(1.5)
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+EBLOCKCHECK")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1-AMSstart timingbuffer-full time")
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2-AMSstart timingbuffer-full time")
                        #self.G_ProzenToolhead.dwell(1)
                        #lancaigang250522: 
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG109-heat up")
                        command_string = """
                            PG109
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-PG109-; command_string='%s'" % command_string)
                        # lancaigang251120: purge, add, preventPG108purge toolheadnofilamentpause, pauseposition in purge, resumewhen will to purge;
                        self.G_PG108Ingoing=1
                        #lancaigang250611: 
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG108-heat uppurgewipe nozzle")
                        command_string = """
                            PG108
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PG108Ingoing=0
                        #lancaigang250427: 
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1-AMSfinishtimingbuffer-full time")
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2-AMSfinishtimingbuffer-full time")
                        if self.STM32ReprotPauseFlag == 1:
                            self.G_PhrozenFluiddRespondInfo('[INFO] STM32 up report pause, cannotresume')
                            # lancaigang240125: number
                            #self.Cmds_PhrozenKlipperResumeCommon()
                            self.G_ChangeChannelResumeFlag=False
                            self.G_PhrozenFluiddRespondInfo("+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                        else:
                            self.G_PhrozenFluiddRespondInfo("[INFO] toolheadhas filament, resume")
                            # lancaigang240125: number
                            self.Cmds_PhrozenKlipperResumeCommon()
                            self.G_ChangeChannelResumeFlag=False
                            self.G_PhrozenFluiddRespondInfo("+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan)
                    else:
                        self.G_KlipperQuickPause = False
                        self.G_PhrozenFluiddRespondInfo('[INFO] toolheadnonefilament, M3modepause')
                        if self.G_KlipperIfPaused == False:
                            self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE")
                            self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)]PAUSE")
                            self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                            self.G_ProzenToolhead.wait_moves()
                            # no filamentpause
                            self.G_KlipperIfPaused=True
                            #self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                            self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        else:
                            self.G_PhrozenFluiddRespondInfo('[INFO] already pause, do notpause')
                        self.G_ChangeChannelResumeFlag=False
                        self.G_PhrozenFluiddRespondInfo("+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                            
                else:
                    self.G_KlipperQuickPause = False
                    self.G_PhrozenFluiddRespondInfo('[INFO] toolheadnonefilament, has AMSmulti-material, M3pause')
                    self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE")
                    self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)]PAUSE")
                    self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                    self.G_ProzenToolhead.wait_moves()
                    # no filamentpause
                    self.G_KlipperIfPaused=True
                    #self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                    self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    self.G_ChangeChannelResumeFlag=False
                    self.G_PhrozenFluiddRespondInfo("+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan)




            # self.G_ChangeChannelResumeFlag=False
            # self.G_PhrozenFluiddRespondInfo("+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan)


            return






        # #lancaigang240319: toolhead up hasfilamentspecial refillH?
        # if self.G_ToolheadIfHaveFilaFlag:
        # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_PhrozenKlipperResume]toolhead up hasfilament")
        # #lancaigang240319: cut filamentbefore move action
        #     #self.Cmds_MoveToCutFilaPrepare()
        #     self.Cmds_AMSSerial1Send("H%d" % self.G_ChangeChannelTimeoutNewChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]cut filamentbeforespecial refillstate: H%d" % self.G_ChangeChannelTimeoutNewChan)
        #     time.sleep(1)


        # #lancaigang240423: resumewhen, first retractfilament
        # self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutNewChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]G%d" % self.G_ChangeChannelTimeoutNewChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]AMSfirst, after toolheadthen retractmm: G%d" % self.G_ChangeChannelTimeoutNewChan)
        # self.G_PauseTriggerWhileChangeChannelFlag=False




        self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG104- get before full change variable')
        command_string = """
            PG104
            """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.G_PhrozenFluiddRespondInfo("External macro command-PG104- get before full change variable; command_string='%s'" % command_string)
        self.IfDoPG102Flag=True


        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG101-retract")
        command_string = """
            PG101
            """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.G_PhrozenFluiddRespondInfo("External macro command-to waiting areapositionpurge; command_string='%s'" % command_string)
        self.IfDoPG102Flag=True





        # lancaigang240328: manualcommanddo not executepurge
        if self.ManualCmdFlag==True:
            self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG106; move command, do not executepurgeFunction')
        else:
            # lancaigang240319: after, first reserve toolheadfilament, preventsmall pellet
            self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG106; cut filament before, first reserve toolheadfilament, prevent stop small pellet')
            self.PG102Flag=True
            self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
            command_string = """
            PG106
            """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)
            self.PG102Flag=False
            self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=False")

        # lancaigang241012: after PG102
        self.IfDoPG102Flag=True

        # lancaigang250717:first, buffer and down
        self.G_ProzenToolhead.dwell(8)

        #lancaigang250519:
        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PRZ_WIPEMOUTH")
        command_string = """
            PRZ_WIPEMOUTH
            """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.G_PhrozenFluiddRespondInfo("External macro command-to waiting areaposition; command_string='%s'" % command_string)




        # lancaigang20231205: cutter cut filament
        # lancaigang231215: Z axis up after down
        self.Cmds_MoveToCutFilaAction(gcmd)




        # lancaigang231216: ifzno down, need to down then pause
        if self.G_IfZPositionLiftUpFlag==True:
            command_string = """
                G90
                G91
                G1 Z-%f F8000
                """ % (
                self.G_AMSFilaCutZPositionLiftingUp,
            )
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_IfZPositionLiftUpFlag = False
            self.G_PhrozenFluiddRespondInfo("Z axis down; command_string='%s'" % command_string)




        # lancaigang240226: cut filament after AMSmainboardfilament, when after toolheadretract20mm
        #time.sleep(2)
        self.G_ProzenToolhead.dwell(0.5)



        # #lancaigang240328: manualcommanddo not executepurge
        # if self.ManualCmdFlag==True:
        # self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG106; manualcommand, do not executepurgeFunction")
        # else:
        # #lancaigang240319: after, first reserve toolheadfilament, preventsmall pellet
        # self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG106; cut filamentbefore, first reserve toolheadfilament, preventsmall pellet")
        #     self.PG102Flag=True
        #     self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
        #     command_string = """
        #     PG106
        #     """
        #     self.G_PhrozenGCode.run_script_from_command(command_string)
        #     self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)
        #     self.PG102Flag=False
        #     self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=False")

        # #lancaigang241012: after PG102
        # self.IfDoPG102Flag=True

        # #lancaigang240906: new AMS, cut filament after, up time channel
        # #lancaigang20231013: stm32
        # #lancaigang231129: stm32 internal all klipper, stm32 internal all, klipperiftoolheadcut filamenterrorno unload filament, klippererrorempty
        # self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutOldChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]G%d" % self.G_ChangeChannelTimeoutOldChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]AMS old channelfirst : G%d" % self.G_ChangeChannelTimeoutOldChan)
        
        # # #lancaigang240906: in up machine when wait, stm32channel and refill
        # # for i in range(5):#
        # # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_P1TnManualChangeChannel]wait old channel")
        # #     self.G_ProzenToolhead.dwell(1.0)
        # #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]i=%d;T=%d" % (i,chan))

        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()
        #lancaigang240416:
        if self.G_SerialPort1OpenFlag == True:
            # lancaigang240913: resumewhen, project is, can full all all, prevent old channelerror, new channelfeederror
            self.Cmds_AMSSerial1Send("AP")
            self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: AP; all filament')
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.Cmds_AMSSerial2Send("AP")
            self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2Sending command: AP; all filament')


        # self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutOldChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]G%d" % self.G_ChangeChannelTimeoutOldChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]AMS old channelfirst : G%d" % self.G_ChangeChannelTimeoutOldChan)
        
        # self.G_ProzenToolhead.dwell(5)


        # self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutNewChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]G%d" % self.G_ChangeChannelTimeoutNewChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]AMSnew channelfirst : G%d" % self.G_ChangeChannelTimeoutNewChan)
        
        # self.G_ProzenToolhead.dwell(5)


        # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_PhrozenKlipperResume]External macro command-PG101")
        # command_string = """
        #     PG101
        #     """
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]External macro command-to waiting areapositionwaitpurge; command_string='%s'" % command_string)
        # self.IfDoPG102Flag=True



        # lancaigang231216: ifpause, z, to pausewhen, set z keep, error
        # lancaigang231216: ifzno down, need to down then pause
        if self.G_IfZPositionLiftUpFlag==True:
            command_string = """
                G90
                G91
                G1 Z-%f F8000
                """ % (
                self.G_AMSFilaCutZPositionLiftingUp,
            )
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_IfZPositionLiftUpFlag = False
            self.G_PhrozenFluiddRespondInfo("Z axis down; command_string='%s'" % command_string)

        # lancaigang240920: resumefilament after,
        #self.ToolheadCutFlag=False

        #lancaigang250519:
        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PRZ_CUT_WAITINGAREA")
        command_string = """
            PRZ_CUT_WAITINGAREA
            """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.G_PhrozenFluiddRespondInfo("External macro command-to waiting areaposition; command_string='%s'" % command_string)


        # lancaigang240913: set to external
        self.G_ProzenToolhead.dwell(6)
        # lancaigang240911: Gcommandafterwhen 5checktoolhead is no hasfilament
        # lancaigang231201: checkcut filament after old channelfilament is no normal normal unload filament, normal normal then pause
        self.Cmds_CutFilaIfNormalCheck()
        # lancaigang240912: is pause, resumewhen will detectto is pause, will
        # lancaigang250109:becausemulti-materialMCresumeneed tore feed。cannotpause
        # if self.G_KlipperIfPaused == True:
        # self.G_PhrozenFluiddRespondInfo("[ERROR] [(cmds.python)]cut filament6toolheaddetectto filament, cutter error, please check cutter, pauseklipperprinting")
        #     #Lo_ChangeChannelIfSuccess = False
        #     return
        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")


        #lancaigang250712: 
        if self.G_ChangeChannelTimeoutOldChan==-1 and self.G_ChangeChannelTimeoutNewChan==-1:
            self.G_PhrozenFluiddRespondInfo('[INFO] multi-materialprinting, printingwhen pause, new old, ifP2A1 normal normal, then resume, down command')
            if self.G_ToolheadIfHaveFilaFlag == False:
                self.G_PhrozenFluiddRespondInfo('[INFO] toolhead5toolheaddetectedfilament, filamentalready, resume;')
                # lancaigang240125: number
                self.Cmds_PhrozenKlipperResumeCommon()
                self.G_ChangeChannelResumeFlag=False
                self.G_ChangeChannelFirstFilaFlag=True
                self.G_IfChangeFilaOngoing= False

                self.G_PhrozenFluiddRespondInfo("[INFO] return")
                return



        # lancaigang250102: resumewhen,, prevent switch
        #self.G_ProzenToolhead.dwell(0.5)
        self.G_PrintCountNum=self.G_PrintCountNum-1


        # lancaigang231115:; resume and beforechannel
        # toolheadhasfilament
        if self.G_ToolheadIfHaveFilaFlag:
            self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, canresume printing')
            # lancaigang240323: first, then resume #1 time channel logical
            self.Cmds_P1CnAutoChangeChannel(self.G_ChangeChannelTimeoutNewChan, self.G_ChangeChannelTimeoutNewGcmd)
            # lancaigang240325: successful, can number data resume
            if self.G_MCModeCanResumeFlag == True:
                self.G_PhrozenFluiddRespondInfo('[INFO] successful, canresume number data')
                # lancaigang240125: number
                self.Cmds_PhrozenKlipperResumeCommon()

                self.G_ProzenToolhead.dwell(1)
                self.G_ChangeChannelResumeFlag=False
                self.G_PhrozenFluiddRespondInfo("+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan)
            else:
                self.G_PhrozenFluiddRespondInfo('[INFO] successful, canresume number data')

                #lancaigang250527: Pause in the waiting area
                self.G_PhrozenFluiddRespondInfo("[WARN] startcalling External macro command-PRZ_PAUSE_WAITINGAREA")
                command = """
                PRZ_PAUSE_WAITINGAREA
                """
                self.G_PhrozenGCode.run_script_from_command(command)
                self.G_PhrozenFluiddRespondInfo('calling External macro command:command=%s' % (command))
                
                self.G_ProzenToolhead.dwell(1)
                self.G_ChangeChannelResumeFlag=False
                self.G_PhrozenFluiddRespondInfo("+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan)

        # toolheadnofilament
        else:
            self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, re action')
            # lancaigang240323: first, then resume #1 time channel logical
            self.Cmds_P1CnAutoChangeChannel(self.G_ChangeChannelTimeoutNewChan, self.G_ChangeChannelTimeoutNewGcmd)
            # lancaigang240325: successful, can number data resume
            if self.G_MCModeCanResumeFlag == True:
                self.G_PhrozenFluiddRespondInfo('[INFO] successful, canresume number data')
                # lancaigang240125: number
                self.Cmds_PhrozenKlipperResumeCommon()

                self.G_ProzenToolhead.dwell(1)
                self.G_ChangeChannelResumeFlag=False
                self.G_PhrozenFluiddRespondInfo("+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan)

            else:
                self.G_PhrozenFluiddRespondInfo('[INFO] successful, canresume number data')

                #lancaigang250527: Pause in the waiting area
                self.G_PhrozenFluiddRespondInfo("[WARN] startcalling External macro command-PRZ_PAUSE_WAITINGAREA")
                command = """
                PRZ_PAUSE_WAITINGAREA
                """
                self.G_PhrozenGCode.run_script_from_command(command)
                self.G_PhrozenFluiddRespondInfo('calling External macro command:command=%s' % (command))

                # lancaigang240509: disable disable
                # #lancaigang240426: resumefailed, need toreportpause
                # if len(self.G_PauseToLCDString)==0:
                # #lancaigang0429: prevent time report pause
                #     #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                #     self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d" % self.G_ChangeChannelTimeoutNewChan)
                self.G_ProzenToolhead.dwell(1)
                self.G_ChangeChannelResumeFlag=False
                self.G_PhrozenFluiddRespondInfo("+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan)




        # lancaigang250102: resumewhen,, prevent switch
        #self.G_ProzenToolhead.dwell(0.5)
        self.G_PrintCountNum=self.G_PrintCountNum-1
        # lancaigang250102: time number; #1 time
        if self.G_PrintCountNum<=0:
            self.G_PrintCountNum=0
            self.G_PhrozenFluiddRespondInfo('[INFO] resumefinish, #1 time')
        else:
            command_string = """
                M106 S255
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("resume,; command_string='%s'" % command_string)
            self.G_PhrozenFluiddRespondInfo("self.G_PrintCountNum='%d'" % self.G_PrintCountNum)
        #self.G_ProzenToolhead.dwell(0.5)


    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_PhrozenKlipperPauseScreen(self, gcmd):
        _ = gcmd

        self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)Cmds_PhrozenKlipperPauseScreen]")
            # // lancaigang231202:+PAUSE:1,ch;1-feed use, pause
            # // lancaigang231202:+PAUSE:2,ch;2-pauseACK
            # // lancaigang231204:+PAUSE:3,ch;3-new channel refilltimeout10s, pause
            # // lancaigang231205:+PAUSE:4,ch;4-new channelfeedtimeout50s, pause
            # // lancaigang231205:+PAUSE:5,ch;5-new channel refilltimeout10s, pause
            # // lancaigang231205:+PAUSE:6,ch;6-entryto park positiontimeout10s, pause
            # // lancaigang231205:+PAUSE:7,ch;7-bufferfullstatetimeout30s, pause
            # // lancaigang231205:+PAUSE:8,ch;8-toolhead cutter or device error, pause
            # // lancaigang231205:+PAUSE:9,ch;9-timeout120s, pause
            # // lancaigang231202:+PAUSE:a,ch;a-park positionto bufferentrytimeout10s, pause
            # // lancaigang231202:+PAUSE:b,ch;b- reserve
            # // lancaigang231202:+PAUSE:c,ch;c- reserve
            # // lancaigang231202:+PAUSE:d,ch;d- reserve
            # // lancaigang231202:+PAUSE:10,ch;10- disable or fluidd network move pause
        # klipper move pause
        self.G_PhrozenFluiddRespondInfo("[WARN] =====PAUSE=====")
        self.G_PhrozenFluiddRespondInfo("=====PAUSE=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====PAUSE=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)

        # lancaigang250527: pause
        self.G_KlipperQuickPause = False
        
        # lancaigang250516: purge pause
        if self.PG102Flag==True:
            self.G_PhrozenFluiddRespondInfo('[INFO] purge, pause')
            #self.G_PhrozenFluiddRespondInfo("+PAUSE:10,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
            return



        # lancaigang231228: hasMCmodeallowedZ axis move action
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            # lancaigang231216: ifpause, z, to pausewhen, set z keep, error
            # lancaigang231216: ifzno down, need to down then pause
            if self.G_IfZPositionLiftUpFlag==True:
                command_string = """
                    G90
                    G91
                    G1 Z-%f F8000
                    """ % (
                    self.G_AMSFilaCutZPositionLiftingUp,
                )
                self.G_PhrozenGCode.run_script_from_command(command_string)
                self.G_IfZPositionLiftUpFlag = False
                self.G_PhrozenFluiddRespondInfo("Z axis down; command_string='%s'" % command_string)


        if self.G_ToolheadIfHaveFilaFlag==True:
            self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, set resumedo notthen feed')
            # lancaigang240116: MAmodeneed topausestm32
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:#MA
                if self.G_KlipperInPausing == False:
                    self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                    #lancaigang250607:
                    self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                    self.G_KlipperQuickPause = True
                    # lancaigang241012: temporary when pauseAMS
                    self.Cmds_PhrozenKlipperPause(None)
                    #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                else:
                    self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")
                
            # lancaigang240412: single-colormode, ifhasAMSmulti-materialfeed, need topauseAMS
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:#M3
                if self.G_AMSDevice1IfNormal==True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, single-colorM3modehas AMSmulti-material, need to stm32pause')
                    if self.G_KlipperInPausing == False:
                        self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                        #lancaigang250607:
                        self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                        self.G_KlipperQuickPause = True
                        # lancaigang241012: temporary when pauseAMS
                        self.Cmds_PhrozenKlipperPause(None)
                        #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")
                else:
                    self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, single-colorM3modehas AMSmulti-material, need to stm32pause')
                    if self.G_KlipperInPausing == False:
                        self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                        #lancaigang250607:
                        self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                        self.G_KlipperQuickPause = True
                        self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")
                    
            else:#MC
                # lancaigang240427: disable disable
                #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                # lancaigang240427: disable move pause, need tostm32pause
                self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, MCmulti-materialmodehas AMSmulti-material, need to stm32pause')
                if self.G_KlipperInPausing == False:
                    self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                    #lancaigang250607:
                    self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                    self.G_KlipperQuickPause = True
                    # lancaigang241012: temporary when pauseAMS
                    self.Cmds_PhrozenKlipperPause(None)
                    #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                else:
                    self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")

                
            self.G_IfToolheadHaveFilaInitiativePauseFlag  = True
        else:
            # lancaigang231216: ifin progress move pause, because already z, resumewhen no resume all z
            self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, set resume need to STM32then feed')
            if self.G_KlipperInPausing == False:
                self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                #lancaigang250607:
                self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                self.G_KlipperQuickPause = True
                # lancaigang241012: temporary when pauseAMS
                self.Cmds_PhrozenKlipperPause(None)
                #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
            else:
                self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")

            

        #lancaigang250527: Pause in the waiting area
        self.G_PhrozenFluiddRespondInfo("[WARN] startcalling External macro command-PRZ_PAUSE_WAITINGAREA")
        command = """
        PRZ_PAUSE_WAITINGAREA
        """
        self.G_PhrozenGCode.run_script_from_command(command)
        self.G_PhrozenFluiddRespondInfo('calling External macro command:command=%s' % (command))

        self.G_PhrozenFluiddRespondInfo('[INFO] disable or fluidd network move pause, pause')
        #self.G_PhrozenFluiddRespondInfo("+PAUSE:10,%d" % self.G_ChangeChannelTimeoutNewChan)
        self.G_PhrozenFluiddRespondInfo("+PAUSE:10,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))



    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # PRZ_CANCEL cancel
    def Cmds_PhrozenKlipperCancel(self, gcmd):
        self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_PhrozenKlipperCancel]klippercancelprinting;')

        self.G_PhrozenFluiddRespondInfo("+CANCEL:0,%d" % self.G_ChangeChannelTimeoutNewChan)

        self.G_PhrozenFluiddRespondInfo("[INFO] =====CANCEL=====")
        self.G_PhrozenFluiddRespondInfo("=====CANCEL=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====CANCEL=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)




        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command: CANCEL_PRINT")
        # lancaigang240120: pause use cfg set table command
        command = """
        CANCEL_PRINT
        """
        self.G_PhrozenGCode.run_script_from_command(command)
        self.G_PhrozenFluiddRespondInfo('calling command:command=%s' % (command))


        #Unlock
        self.Base_AMSSerialCmdUnlock()


        # #lancaigang231216: 
        # eventtime = self.G_PhrozenReactor.monotonic()
        # # Determine "printing" status
        # idle_timeout = self.G_PhrozenPrinter.lookup_object("idle_timeout")
        # is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
        # if is_printing:
        # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_PhrozenKlipperCancel]in progressprinting; command='%s'")
        # else:
        # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_PhrozenKlipperCancel] in printing; command='%s; return'")
        #     return


        # lancaigang231207:+PAUSE:1feed
        self.G_IfInFilaBlockFlag=False
        # lancaigang240321: PG102 pause
        self.PG102DelayPauseFlag=False
        # lancaigang240426: resume after set false
        self.G_ResumeProcessCheckPauseStatus=False
        #lancaigang240410: 
        self.G_CancelFlag=True
        # lancaigang240411: ifnoreceived P0 M3command, use use filament runoutdetect machine
        self.G_P0M3Flag = False

        self.ManualCmdFlag=False
        # lancaigang250805: cutter
        self.G_CutCheckTest=False

        # lancaigang240427: AMSerrorrestart, need to
        self.G_AMS1ErrorRestartFlag = False
        self.G_AMS1ErrorRestartCount = 0

        #lancaigang241030:
        self.G_AMS2ErrorRestartFlag = False
        self.G_AMS2ErrorRestartCount = 0

        # lancaigang240124: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0

        #lancaigang250526: 
        self.G_IfToolheadHaveFilaInitiativePauseFlag=False
        # lancaigang250526: pause, not allowednew gcodecommand, need waitpausecomplete
        self.G_KlipperInPausing = False
        # lancaigang250527: pause
        self.G_KlipperQuickPause = False
        # lancaigang250607: state; 1-unload filament; 2-feed; 3-; 4-pause
        self.G_KlipperPrintStatus= -1
        self.G_ASM1DisconnectErrorCount=0
        # lancaigang250812:single-colorfilament runoutdetect, to pause
        self.G_RetryToPauseAreaFlag = False
        self.G_RetryToPauseAreaCount = 0
        self.G_P10SpitNum=0
        self.G_IfChangeFilaOngoing= False
        # lancaigang240223: toolheadcut filamentfailed
        self.ToolheadCutFlag = False





        #lancaigang250515: 
        self.G_P0M1MCNoneAMS=0
        self.G_PhrozenFluiddRespondInfo("[INFO] self.G_P0M1MCNoneAMS=0")

        # lancaigang250515:clearserial port disable set number data
        self.Cmds_GetUartScreenCfgClear()


        # lancaigang250807:cancelthen clearpausestate
        self.G_PhrozenPrinterCancelPauseResume.cmd_CLEAR_PAUSE(None)
        self.G_PhrozenFluiddRespondInfo('[INFO] clearpause state')





        #lancaigang241016:
        #self.ToolheadCutFlag=False

        # #AMSmulti-material stop
        # #self.Cmds_CmdP4(None)
        # #lancaigang240125: 
        # #lancaigang240507: send pausecommand, send M0command
        # #lancaigang240516: cancel, do not executeFunction
        # self.Cmds_AMSSerial1Send("AT+PAUSE")
        # self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)Cmds_PhrozenKlipperCancel]AT+PAUSEpausestm32 machine ")

        # #klipper move pause
        # self.Cmds_PhrozenKlipperPause(None)
        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)

        self.G_ProzenToolhead.dwell(1.0)

        #lancaigang240416:
        # lancaigang240516: cancel, do not executeFunction
        # if self.G_SerialPort1OpenFlag == True:
        #     self.Cmds_AMSSerial1Send("M0")
        # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_PhrozenKlipperCancel]sending command: M0")

        # #AMSmulti-material stop
        # #self.Cmds_CmdP4(None)
        # #lancaigang240125: 
        # #lancaigang240507: send pausecommand, send M0command
        # #lancaigang240516: cancel, do not executeFunction
        # self.Cmds_AMSSerial1Send("AT+PAUSE")
        # self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)Cmds_PhrozenKlipperCancel]AT+PAUSEpausestm32 machine ")

        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()

        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.G_PhrozenFluiddRespondInfo('[INFO] sending command:MAmulti-materialmode-MC-AMSemptymode')
            #lancaigang241031:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.G_PhrozenFluiddRespondInfo('[INFO] sending command:M2single-color refill modemode-MA-AMSemptymode')
            #lancaigang241031:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MA")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MA")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MA")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo('[INFO] sending command:M3single-colormode-MA-AMSemptymode')
            #lancaigang241031:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MA")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MA")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MA")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MA")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Unknown mode, pauseAMS")
            
            #lancaigang241031:
            if self.G_SerialPort1OpenFlag == True:
                # lancaigang240516: Unknown mode, pauseAMS
                self.Cmds_AMSSerial1Send("AT+PAUSE")
                self.G_PhrozenFluiddRespondInfo('[WARN] serial port 1 send AT+PAUSEpausestm32 machine')
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("AT+PAUSE")
                self.G_PhrozenFluiddRespondInfo('[WARN] serial port 2 send AT+PAUSEpausestm32 machine')


        #lancaigang241106: 
        self.G_P0M2MAStartPrintFlag=0
        # lancaigang250104: P2A3
        self.G_P2A3Flag = 0

        # lancaigang250102: layer number
        self.G_PrintCountNum=0


        # lancaigang20231013: disconnectconnect
        #self.Device_DisconnectAMSDevice()
        # lancaigang250712: disable disable, do notdisconnectconnect
        # lancaigang250815: disable disable, preventcancel after serial porterror
        self.Cmds_CmdP29(None)

        # lancaigang250815:mode set is Unknown mode
        self.G_AMSDeviceWorkMode = AMS_WORK_MODE_UNKNOW



        self.G_PhrozenFluiddRespondInfo("+CANCEL:1,%d" % self.G_ChangeChannelTimeoutNewChan)


    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_USBConnectErrorCheck(self):
        self.G_PhrozenFluiddRespondInfo("[ERROR] [(cmds.python)Cmds_USBConnectErrorCheck]")


        self.G_PhrozenFluiddRespondInfo("self.G_CancelFlag='%s'" % self.G_CancelFlag)
        self.G_PhrozenFluiddRespondInfo("[INFO] Current mode")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:0,unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:1,MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:2,MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo("[WARN] +Mode:3,RUNOUT")
        else:
            self.G_PhrozenFluiddRespondInfo("[ERROR] +Mode:-1,error")

        try:
            self.G_PhrozenFluiddRespondInfo("[ERROR] [(cmds.py)Cmds_USBConnectErrorCheck]Reinitializing serial port 1")
            self.G_SerialPort1Obj = serial.Serial(self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3)
            #Serial port opened successfully
            if self.G_SerialPort1Obj is not None:
                if self.G_SerialPort1Obj.is_open:
                    self.G_SerialPort1OpenFlag = True
                    self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 1successful")
                    # lancaigang231213: openserial port
                    self.G_SerialPort1Obj.flushInput()  # clean serial write cache
                    self.G_SerialPort1Obj.flush()
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 buffers cleared")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Re-registering serial port 1 callback")
                    self.G_SerialPort1RecvTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerUart1Recv, self.G_PhrozenReactor.NOW)
                    
                    if "+PAUSE:g" in self.G_PauseToLCDString:
                        self.G_PhrozenFluiddRespondInfo('[INFO] if is USBfilament runouterror, clear report error signal info')
                        # lancaigang250902: cannot is empty, prevent after pause report error signal info is emptyno output pause
                        #self.G_PauseToLCDString=""
                        self.G_PauseToLCDString="+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                        self.G_PhrozenFluiddRespondInfo("len(self.G_PauseToLCDString)='%d'" % len(self.G_PauseToLCDString))

        except:
            self.G_PhrozenFluiddRespondInfo("[ERROR] Unable to open tty1. Check the USB connection or try rebooting.")
            self.G_SerialPort1OpenFlag = False

            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW or self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                self.G_PhrozenFluiddRespondInfo('[WARN] single-colorM3 or Unknown mode, do notnew pause signal info')
            else:
                if len(self.G_PauseToLCDString)==0:
                    self.G_PhrozenFluiddRespondInfo('[INFO] new pause signal info')
                    self.G_PhrozenFluiddRespondInfo("[WARN] pause:+PAUSE:g")
                    self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                else:
                    #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                    self.G_PhrozenFluiddRespondInfo('[INFO] new pause signal info')
                    self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                    self.G_PhrozenFluiddRespondInfo("[WARN] pause:+PAUSE:g")

        try:
            self.G_PhrozenFluiddRespondInfo("[ERROR] [(cmds.py)Cmds_USBConnectErrorCheck]Reinitializing serial port 2")
            self.G_SerialPort2Obj = serial.Serial(self.G_Serialport2Define, SERIAL_PORT_BAUD, timeout=3)
            #Serial port opened successfully
            if self.G_SerialPort2Obj is not None:
                if self.G_SerialPort2Obj.is_open:
                    self.G_SerialPort2OpenFlag = True
                    self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 2successful")
                    self.G_SerialPort2Obj.flushInput()  # clean serial write cache
                    self.G_SerialPort2Obj.flush()
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 buffers cleared")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Re-registering serial port 2 callback")
                    self.G_SerialPort2RecvTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerUart2Recv, self.G_PhrozenReactor.NOW)
                    # if "+PAUSE:g" in self.G_PauseToLCDString:
                    #     self.G_PauseToLCDString=""
        except:
            self.G_PhrozenFluiddRespondInfo("[DEBUG] AMS2/tty2 not available; skipping serial port 2.")
            self.G_SerialPort2OpenFlag = False

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_CutFilaIfNormalCheck(self):
        self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CutFilaIfNormalCheck]")

        #lancaigang241016:
        self.ToolheadCutFlag=False

        # lancaigang250527: pause
        self.G_KlipperQuickPause = False
        
        # sending command8S after checkfilament is no reserve in toolhead, normal normal 8stoolheadhasfilament
        # lancaigang20231013: is 8swhen
        # lancaigang231201: is 5
        # lancaigang240912: klipperwhen to stm32 old channelwhen
        #self.G_ProzenToolhead.dwell(6.0)

        self.G_PhrozenFluiddRespondInfo('[INFO] when detected is no has filament???')
        # lancaigang240125: cannot use sleep, will thread
        #time.sleep(5)
        # filament8s after, toolhead is no hasfilament, normal normal is will hasfilament
        if self.G_ToolheadIfHaveFilaFlag:
            # raise self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]checktoolhead is no hasfilament, but toolhead up detectto filament; cmd='%s'" % (gcmd.get_commandline()))
            self.G_PhrozenFluiddRespondInfo('[INFO] toolhead5toolheaddetectedfilament, please check cutter is no error, move failed;klipperpause')
            # // lancaigang231202:+PAUSE:1,ch;1-feed use, pause
            # // lancaigang231202:+PAUSE:2,ch;2-pauseACK
            # // lancaigang231204:+PAUSE:3,ch;3-new channel refilltimeout10s, pause
            # // lancaigang231205:+PAUSE:4,ch;4-new channelfeedtimeout50s, pause
            # // lancaigang231205:+PAUSE:5,ch;5-new channel refilltimeout10s, pause
            # // lancaigang231205:+PAUSE:6,ch;6-entryto park positiontimeout10s, pause
            # // lancaigang231205:+PAUSE:7,ch;7-bufferfullstatetimeout30s, pause
            # // lancaigang231205:+PAUSE:8,ch;8-toolhead cutter or device error, pause
            # // lancaigang231205:+PAUSE:9,ch;9-timeout120s, pause
            # // lancaigang231202:+PAUSE:a,ch;a-park positionto bufferentrytimeout10s, pause
            # // lancaigang231202:+PAUSE:b,ch;b- reserve
            # // lancaigang231202:+PAUSE:c,ch;c- reserve
            # // lancaigang231202:+PAUSE:d,ch;d- reserve
            # // lancaigang231202:+PAUSE:10,ch;10- disable or fluidd network move pause
            # lancaigang240223: beforeZcut filamentalready failed, pausebeforefirst to Z down
            if self.G_IfZPositionLiftUpFlag==True:
                command_string = """
                    G90
                    G91
                    G1 Z-%f F8000
                    """ % (
                    self.G_AMSFilaCutZPositionLiftingUp,
                )
                self.G_PhrozenGCode.run_script_from_command(command_string)
                self.G_IfZPositionLiftUpFlag = False
                self.G_PhrozenFluiddRespondInfo("Z axis down; command_string='%s'" % command_string)
                

            #lancaigang240223:
            self.ToolheadCutFlag=True

            # lancaigang240322: if before already pause, do notthen report error
            if self.STM32ReprotPauseFlag==1:
                self.G_PhrozenFluiddRespondInfo('[INFO] already pause, do notpause')
                self.G_PhrozenFluiddRespondInfo('[INFO] toolhead cutter or device error, pause')
                #self.G_PhrozenFluiddRespondInfo("+PAUSE:8,%d" % self.G_ChangeChannelTimeoutOldChan)
                #lancaigang250414:
                #self.G_PhrozenFluiddRespondInfo("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                
                # lancaigang250619:checkAMS is no re connectsuccessful
                self.Cmds_USBConnectErrorCheck()
                
                
                #lancaigang250721: 
                if len(self.G_PauseToLCDString)==0:
                    self.G_PhrozenFluiddRespondInfo("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                else:
                    #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                    self.G_PhrozenFluiddRespondInfo('[INFO] new pause signal info')
                    # lancaigang250721: ifAMSerror, is errorpause8 action is after pause
                    self.G_PauseToLCDString="+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                    self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)

                

                
            else:
                # lancaigang240328: if is manualcommand, pause
                if self.ManualCmdFlag==True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] move command, klipperdo not executepause')
                    #lancaigang241031:
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("AT+PAUSE")
                        self.G_PhrozenFluiddRespondInfo('[WARN] serial port 1 send AT+PAUSEpausestm32 machine')
                    #lancaigang241030:
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AT+PAUSE")
                        self.G_PhrozenFluiddRespondInfo('[WARN] serial port 2 send AT+PAUSEpausestm32 machine')

                    #lancaigang250805:
                    #self.G_PhrozenFluiddRespondInfo("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    if len(self.G_PauseToLCDString)==0:
                        self.G_PhrozenFluiddRespondInfo("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    else:
                        #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                        self.G_PhrozenFluiddRespondInfo('[INFO] new pause signal info')
                        # lancaigang250721: ifAMSerror, is errorpause8 action is after pause
                        self.G_PauseToLCDString="+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                        self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                elif self.G_CutCheckTest==True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] cutter, klipperdo not executepause')
                    #lancaigang241031:
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("AT+PAUSE")
                        self.G_PhrozenFluiddRespondInfo('[WARN] serial port 1 send AT+PAUSEpausestm32 machine')
                    #lancaigang241030:
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AT+PAUSE")
                        self.G_PhrozenFluiddRespondInfo('[WARN] serial port 2 send AT+PAUSEpausestm32 machine')

                    #lancaigang250805:
                    #self.G_PhrozenFluiddRespondInfo("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    if len(self.G_PauseToLCDString)==0:
                        self.G_PhrozenFluiddRespondInfo("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    else:
                        #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                        self.G_PhrozenFluiddRespondInfo('[INFO] new pause signal info')
                        # lancaigang250721: ifAMSerror, is errorpause8 action is after pause
                        self.G_PauseToLCDString="+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                        self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                else:
                    self.G_PhrozenFluiddRespondInfo('[INFO] toolhead cutter or device error, pause')


                    if self.G_KlipperInPausing == False:
                        self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                        #lancaigang250607:
                        self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                        self.G_KlipperQuickPause = True
                        # klipper move pause
                        self.Cmds_PhrozenKlipperPause(None)
                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")
                
                    #self.G_PhrozenFluiddRespondInfo("+PAUSE:8,%d" % self.G_ChangeChannelTimeoutOldChan)
                    # lancaigang250619:checkAMS is no re connectsuccessful
                    self.Cmds_USBConnectErrorCheck()
                    
                    #lancaigang250414:
                    #self.G_PhrozenFluiddRespondInfo("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    if len(self.G_PauseToLCDString)==0:
                        self.G_PhrozenFluiddRespondInfo("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    else:
                        #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                        self.G_PhrozenFluiddRespondInfo('[INFO] new pause signal info')
                        # lancaigang250721: ifAMSerror, is errorpause8 action is after pause
                        self.G_PauseToLCDString="+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                        self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)

            

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # channel
    def Cmds_P1TnManualChangeChannel(self, chan, gcmd):
        self.G_PhrozenFluiddRespondInfo("[INFO] =====[(cmds.python)Cmds_P1TnManualChangeChannel]")

        if gcmd is None:
            self.G_PhrozenFluiddRespondInfo("[INFO] gcmd is None")
            #self.G_PhrozenFluiddRespondInfo("[INFO] return")
            #return
            #pass
        if gcmd is not None:
            self.G_PhrozenFluiddRespondInfo("[INFO] gcmd is not None:")
            self.G_PhrozenFluiddRespondInfo("=====[(cmds.python)Cmds_P1TnManualChangeChannel]cmd='%s'" % (gcmd.get_commandline()))

        # #lancaigang20231101: first toolhead is no hasfilament, has then first unload filament
        # if self.G_ToolheadIfHaveFilaFlag:
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]toolheadhasfilament, preventfirst unload filament; cmd='%s'" % (gcmd.get_commandline()))
        # #// retract allto park position; //===== P2 A1 allfilamentto park position Yes; "AP";
        #     self.Cmds_AMSSerial1Send("AP")
        # self.G_PhrozenFluiddRespondInfo("[INFO] Sending command: AP, fully retract to the park position")


        # lancaigang231216: manualgcodechannel and command
        # get channel and gcmd to
        #self.G_ChangeChannelTimeoutOldChan=chan
        #self.G_ChangeChannelTimeoutOldGcmd=gcmd


        self.G_IfChangeFilaOngoing= True

        self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))



        # lancaigang240229: preventsending command
        #time.sleep(1)
        self.G_ProzenToolhead.dwell(0.5)

        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()

        # lancaigang240223: ifcut filamentfailed, up number already Z axis down
        if self.ToolheadCutFlag==True:
            self.ToolheadCutFlag=False
            self.G_PhrozenFluiddRespondInfo('[INFO] before cut filamenterror, failed')
            self.G_ChangeChannelFirstFilaFlag=True
            self.G_IfChangeFilaOngoing= False

            # stm32reportpausepause1 time, cannotpause
            self.STM32ReprotPauseFlag=1
            # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
            self.G_ChangeChannelFirstFilaFlag=True

            # #lancaigang250308: resumealready cut filamenterror, report cut filamenterror
            # #self.G_PhrozenFluiddRespondInfo("+PAUSE:8,%d" % self.G_ChangeChannelTimeoutNewChan)
            # #self.G_PhrozenFluiddRespondInfo("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
            # if len(self.G_PauseToLCDString)==0:
            #     self.G_PhrozenFluiddRespondInfo("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
            # else:
            #     self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)

            self.G_PauseToLCDString="+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

            #lancaigang240416:
            if self.G_SerialPort1OpenFlag == True:
                # lancaigang240603: preventAMS
                self.Cmds_AMSSerial1Send("AT+PAUSE")
                self.G_PhrozenFluiddRespondInfo('[WARN] serial port 1-AT+PAUSEpausestm32 machine')
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("AT+PAUSE")
                self.G_PhrozenFluiddRespondInfo('[WARN] serial port 2-AT+PAUSEpausestm32 machine')

            if self.G_KlipperInPausing == False:
                self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                #lancaigang250607:
                self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                self.G_KlipperQuickPause = True
                # klipper move pause
                self.Cmds_PhrozenKlipperPause(None)
            else:
                self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")

            self.G_KlipperIfPaused = True

            # lancaigang240325: failed, cannotresume
            self.G_MCModeCanResumeFlag = False

            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+T:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            
            #lancaigang250529:
            if len(self.G_PauseToLCDString)==0:
                self.G_PhrozenFluiddRespondInfo("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
            else:
                self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)

            self.G_PhrozenFluiddRespondInfo("[INFO] return")
            return

        self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))

        #lancaigang241030:
        if self.G_ChangeChannelTimeoutNewChan in range(1, 5):
            # lancaigang240911: new AMS code disable, T?commandfeed
            # send manualcommand
            self.Cmds_AMSSerial1Send("T%d" % self.G_ChangeChannelTimeoutNewChan)
            self.G_PhrozenFluiddRespondInfo('serial port 1Sending command: T%d' % self.G_ChangeChannelTimeoutNewChan)
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+T:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        elif self.G_ChangeChannelTimeoutNewChan in range(5, 9):
            self.Cmds_AMSSerial2Send("T%d" % self.G_ChangeChannelTimeoutNewChan-4)
            self.G_PhrozenFluiddRespondInfo('serial port 2Sending command: T%d' % self.G_ChangeChannelTimeoutNewChan-4)
            self.G_PhrozenFluiddRespondInfo("+T:0,%d" % self.G_ChangeChannelTimeoutNewChan-4)

        #lancaigang250322: 
        if self.ManualCmdFlag==True:
            self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG105; move command, do not executepurgeFunction')
            self.IfDoPG102Flag=True
        # lancaigang250805: cutter
        elif self.G_CutCheckTest == True:
            # lancaigang240319: after, first reserve toolheadfilament, preventsmall pellet
            self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG105; cut filament after, toolheadheat upwhen AMS')
            self.PG102Flag=True
            self.IfDoPG102Flag=True
            self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
            command_string = """
            PG105
            """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)
            self.PG102Flag=False
            self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=False")
        else:
            # lancaigang240319: after, first reserve toolheadfilament, preventsmall pellet
            self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG105; cut filament after, toolheadheat upwhen AMS')
            self.PG102Flag=True
            self.IfDoPG102Flag=True
            self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
            command_string = """
            PG105
            """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)
            self.PG102Flag=False
            self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=False")

        self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))

        # lancaigang240328: manualcommanddo not executepurge
        if self.ManualCmdFlag==True:
            self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG110; move command, do not execute')
            self.IfDoPG102Flag=True
        # lancaigang250805: cutter
        elif self.G_CutCheckTest == True:
            # lancaigang240319: after, first reserve toolheadfilament, preventsmall pellet
            self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG110; STM32feed after, klipperstartpurgefeed')
            self.PG102Flag=True
            self.IfDoPG102Flag=True
            self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
            command_string = """
            PG110
            """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)
            self.PG102Flag=False
            self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=False")
        else:
            # lancaigang240319: after, first reserve toolheadfilament, preventsmall pellet
            self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG110; STM32feed after, klipperstartpurgefeed')
            self.PG102Flag=True
            self.IfDoPG102Flag=True
            self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
            command_string = """
            PG110
            """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)
            self.PG102Flag=False
            self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=False")

        self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))
        # #lancaigang240328: manualcommanddo not executepurge
        # if self.ManualCmdFlag==True:
        # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)]External macro command-PG110; manualcommand, do not execute")
        # else:
        # #lancaigang240319: after, first reserve toolheadfilament, preventsmall pellet
        # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)]External macro command-PG110; STM32feedafter, klipper up startpurge")
        #     self.PG102Flag=True
        #     self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)]self.Flag=True")
        #     command_string = """
        #     PG110
        #     """
        #     self.G_PhrozenGCode.run_script_from_command(command_string)
        #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)]command_string='%s'" % command_string)
        #     self.PG102Flag=False
        #     self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)]self.Flag=False")

        

        # #lancaigang240226: cut filament after AMSmainboardfilament, when after toolheadretract20mm
        # time.sleep(2)
        # #lancaigang231208: E head - number, output head retractfilament
        # command_string = """
        # G92 E0
        # G1 E0.0000 F600
        # G91
        # G1 E-50 F8000
        # """
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]when 2s; E number toolheadretractfilament50mm;G-codecommand:command_string='%s'" % command_string)


        #lancaigang20231013: 8
        # lancaigang231115: temporary when use use printer.cfg set timeoutwhen, use use python internal all defaulttimeoutwhen
        timeout = self.G_DictChangeChannelWaitAreaParam["T"] - 8

        # #lancaigang240125: wait, z after then down
        # #lancaigang231208: z+ normal number will up
        # command_string = """
        #     G91
        #     G1 Z%f F8000
        #     """ % (
        #     self.G_AMSFilaCutZPositionLiftingUp,
        # )
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]Z axisraiseraise;gcodecommand=%s" % command_string)


        #lancaigang240619: 
        # #lancaigang240306: move move to cut filament code
        # #lancaigang240110: waitwaitbefore, first External macro command, move move to positionwait
        # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_P1TnManualChangeChannel]External macro command-PG101")
        # command_string = """
        #     PG101
        #     """
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]External macro command; command_string='%s'" % command_string)
        # self.IfDoPG102Flag=True


        # lancaigang240223: becauseresumewhen, to P9wait, will to type
        command_string = """
                        G90
                        G91
                        G1 Z%f F8000
                        """ % (
                        self.G_AMSFilaCutZPositionLiftingUp,
                    )
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.Lo_ThisIfZPositionLiftUpFlag = True
        self.G_PhrozenFluiddRespondInfo("Z axistemporary; command_string='%s'" % command_string)

        #lancaigang240325:
        #self.G_ResumeProcessCheckPauseStatus=False


        #lancaigang250519:
        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PRZ_SPITTING_SCRAPE")
        command_string = """
            PRZ_SPITTING_SCRAPE
            """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)

        self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))
        # set
        Lo_ChangeChannelIfSuccess = False
        # lancaigang250607: state; 1-unload filament; 2-feed; 3-; 4-pause
        self.G_KlipperPrintStatus= 2
        # lancaigang20231013: timeoutwhen
        # lancaigang231114: in printer.cfg set text file timeoutwhen, timeout
        # detect #2 time feedfilament is no to toolhead
        for i in range(CHANGE_CHANNEL_WAIT_TIMEOUT):
            # self.G_XBasePosition+=2
            # self.G_YBasePosition+=2
            # lancaigang240325: if in resumestate, canreportpausestate
            #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]self.G_ResumeProcessCheckPauseStatus='%d'" % self.G_ResumeProcessCheckPauseStatus)
            if self.G_ChangeChannelResumeFlag==True:
                if self.STM32ReprotPauseFlag==1:
                    self.G_PhrozenFluiddRespondInfo('[INFO] in resumestate, detected up time pause')
                    if self.G_ResumeProcessCheckPauseStatus==True:
                        # lancaigang240430: to after failed logical
                        #self.G_ResumeProcessCheckPauseStatus=False
                        self.G_PhrozenFluiddRespondInfo('[INFO] has time pause state up report, exitresume')
                        self.G_ChangeChannelFirstFilaFlag=True
                        Lo_ChangeChannelIfSuccess = False


                        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
                        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
                        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
                        if Lo_PauseStatus['is_paused'] == True:
                            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
                        else:
                            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")

                        break
                    #else:
                        # self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)Cmds_P1TnManualChangeChannel]no time pausestatereport, resume")

            else:
                # lancaigang231202: ifSTM32 move reportpause, need toklipperpause
                if self.STM32ReprotPauseFlag==1:
                    self.G_ChangeChannelFirstFilaFlag=True
                    self.G_PhrozenFluiddRespondInfo('[INFO], stm32 move up report pause')
                    Lo_ChangeChannelIfSuccess = False


                    Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                    self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
                    self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
                    #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
                    if Lo_PauseStatus['is_paused'] == True:
                        self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
                    else:
                        self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")

                    break
            

            # #lancaigang231216: 
            # if self.G_XBasePosition==0 and self.G_YBasePosition==0:
            # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_P1TnManualChangeChannel]wait, XY is 0")
            #     command_string = """
            #         G90
            #         G1 X%.03f Y%.03f F5000
            #         """ % (
            #         150+(i%2),
            #         260+(i%2)
            #     )
            # #lancaigang231129: move move
            #     self.G_PhrozenGCode.run_script_from_command(command_string)
            # else:
            # #lancaigang231216: resumewhen, need to move prevent
            # #lancaigang231214: waitX YW H move move, purgeFunction
            #     command_string = """
            #         G90
            #         G1 X%.03f Y%.03f F5000
            #         """ % (
            #         self.G_XBasePosition+(i%2),
            #         self.G_YBasePosition+(i%2)
            #     )
            # #lancaigang231129: move move
            #     self.G_PhrozenGCode.run_script_from_command(command_string)
            # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_P1TnManualChangeChannel]wait, XY is P9 set ")
            # #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]wait move; command_string='%s'" % command_string)
            
            
            # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_P1TnManualChangeChannel]wait, use use External macro command")


            # lancaigang240223: becauseresumewhen, to P9wait, will to type, time and down time
            if self.Lo_ThisIfZPositionLiftUpFlag == True:
                command_string = """
                                G90
                                G91
                                G1 Z-%f F8000
                                """ % (
                                self.G_AMSFilaCutZPositionLiftingUp,
                            )
                self.G_PhrozenGCode.run_script_from_command(command_string)
                self.Lo_ThisIfZPositionLiftUpFlag = False
                self.G_PhrozenFluiddRespondInfo("Z axistemporary down; command_string='%s'" % command_string)

            # lancaigang20231013: is 4when
            # lancaigang231115: is 1s
            self.G_ProzenToolhead.dwell(1)
            # lancaigang240125: cannot use sleep, will thread
            #time.sleep(1)


            self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))

            self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG110; STM32feed after, klipperstartpurgefeed')
            command_string = """
            PG110
            """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)


            
            self.G_PhrozenFluiddRespondInfo("[INFO] Current mode")
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:0,unkown")
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
                self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:1,MC")
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:2,MA")
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                self.G_PhrozenFluiddRespondInfo("[WARN] +Mode:3,RUNOUT")
            else:
                self.G_PhrozenFluiddRespondInfo("[ERROR] +Mode:-1,error")


            Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
            self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
            self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
            #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
            if Lo_PauseStatus['is_paused'] == True:
                self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
            else:
                self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")



            #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]i=%d;T=%d" % (i,self.G_ChangeChannelTimeoutNewChan))

            # detectnew channelfilamentfeed, is no hasfilamentto toolhead
            if self.G_ToolheadIfHaveFilaFlag:
                Lo_ChangeChannelIfSuccess = True
                break

        # #lancaigang240125: wait, z after then down
        # command_string = """
        #     G91
        #     G1 -Z%f F8000
        #     """ % (
        #     self.G_AMSFilaCutZPositionLiftingUp,
        # )
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]Z axis down;gcodecommand=%s" % command_string)




        # lancaigang240318: preventerror up down
        if self.Lo_ThisIfZPositionLiftUpFlag == True:
            command_string = """
                            G90
                            G91
                            G1 Z-%f F8000
                            """ % (
                            self.G_AMSFilaCutZPositionLiftingUp,
                        )
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.Lo_ThisIfZPositionLiftUpFlag = False
            self.G_PhrozenFluiddRespondInfo("Z axistemporary down; command_string='%s'" % command_string)


        self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))

        # normal normal; successful
        if Lo_ChangeChannelIfSuccess:
            self.G_PhrozenFluiddRespondInfo('successful: T%d' % self.G_ChangeChannelTimeoutNewChan)
            self.G_IfChangeFilaOngoing= False

            # lancaigang250424: preventAMSbufferhasfull
            self.G_ProzenToolhead.dwell(0.5)

            # lancaigang250619:checkAMS is no re connectsuccessful
            self.Cmds_USBConnectErrorCheck()
            # lancaigang250423: feedsuccessful, purge, AMSwhen, ifpurge5buffer is state, head
            #self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
            # self.G_PhrozenFluiddRespondInfo("[INFO] AMSstart timingbuffer-full time")
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("AT+EBLOCKCHECK")
                self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1-AMSstart timingbuffer-full time")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
                self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2-AMSstart timingbuffer-full time")
            self.G_ProzenToolhead.dwell(1)

            #lancaigang240229:
            if self.IfDoPG102Flag==True:
                self.IfDoPG102Flag=False

                self.G_PhrozenFluiddRespondInfo("[INFO] purgestart")
                self.G_PhrozenFluiddRespondInfo("+MSG:1,0,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))

                # lancaigang240328: manualcommanddo not executepurge
                if self.ManualCmdFlag==True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG102; move command, do not executepurgeFunction')
                    # lancaigang250409: get AMS
                    self.Cmds_CmdP114(None)
                else:
                    # self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG102")
                    # self.PG102Flag=True
                    # self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                    # command_string = """
                    # PG102
                    # """
                    # self.G_PhrozenGCode.run_script_from_command(command_string)
                    # self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)

                    # lancaigang241031: purge time number
                    if self.G_P10SpitNum==0:
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG113")
                        self.PG102Flag=True
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                        command_string = """
                        PG114
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)
                    elif self.G_P10SpitNum==1:
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG111")
                        self.PG102Flag=True
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                        command_string = """
                        PG114
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)
                    elif self.G_P10SpitNum==2:
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG112")
                        self.PG102Flag=True
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                        command_string = """
                        PG113
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)
                    elif self.G_P10SpitNum==3:
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG113")
                        self.PG102Flag=True
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                        command_string = """
                        PG113
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)
                    elif self.G_P10SpitNum==4:
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG114")
                        self.PG102Flag=True
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                        command_string = """
                        PG114
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)
                    elif self.G_P10SpitNum==5:
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG115")
                        self.PG102Flag=True
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                        command_string = """
                        PG115
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)
                    #lancaigang250528: 
                    elif self.G_P10SpitNum==6:
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG116")
                        self.PG102Flag=True
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                        command_string = """
                        PG116
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)
                    elif self.G_P10SpitNum==7:
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG117")
                        self.PG102Flag=True
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                        command_string = """
                        PG117
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)
                    elif self.G_P10SpitNum==8:
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG118")
                        self.PG102Flag=True
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                        command_string = """
                        PG118
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)
                    elif self.G_P10SpitNum==9:
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG119")
                        self.PG102Flag=True
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                        command_string = """
                        PG119
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)


                    self.PG102Flag=False
                    self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=False")

                self.G_PhrozenFluiddRespondInfo("[INFO] purgefinish")
                self.G_PhrozenFluiddRespondInfo("+MSG:1,1,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))


                # lancaigang240323: # layer, first disable disable
                # #lancaigang240321: purgecomplete after, move move to, preventresumewhen from Y305positionto pause, toolheadMCUerror
                # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_P1TnManualChangeChannel]External macro command-PG105; move move to, preventresumepath")
                # command_string = """
                # PG105
                # """
                # self.G_PhrozenGCode.run_script_from_command(command_string)
                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]External macro command-PG105; move move to, preventresumepath; command_string='%s'" % command_string)
                


                # for i in range(15):
                # self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)]purge, wait")
                # #lancaigang20231013: is 4when
                # #lancaigang231115: is 1s
                #     self.G_ProzenToolhead.dwell(1.0)
                # #lancaigang240125: cannot use sleep, will thread
                #     #time.sleep(1)
                if self.PG102DelayPauseFlag==True:
                    self.PG102DelayPauseFlag=False

                    # lancaigang250619:checkAMS is no re connectsuccessful
                    self.Cmds_USBConnectErrorCheck()
                    #lancaigang250427: 
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                        self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1-AMSfinishtimingbuffer-full time")
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                        self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2-AMSfinishtimingbuffer-full time")

                    self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                    self.G_KlipperQuickPause = True
                    self.G_PhrozenFluiddRespondInfo('[INFO] purge, STM32 send filament runoutpause')
                    # lancaigang231209: timer logical, will error, after to use thread logical interrupt business logic
                    self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause1 time')


                    

                    if self.G_KlipperInPausing == False:
                        self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                        #lancaigang250607:
                        self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                        self.G_KlipperQuickPause = True
                        # klipper move pause
                        self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")




                    self.G_KlipperIfPaused = True
                    # stm32 move pausepause1 time, cannotpause
                    self.STM32ReprotPauseFlag=1
                    # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                    self.G_ChangeChannelFirstFilaFlag=True
                    
                    self.G_ProzenToolhead.dwell(1.5)
                    self.G_PhrozenFluiddRespondInfo("+MSG:1,1,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    # lancaigang240524: use UIUX move
                    self.G_PhrozenFluiddRespondInfo("+C:1,%d" % self.G_ChangeChannelTimeoutNewChan)
                    
                    #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)

                    #lancaigang250529:
                    if len(self.G_PauseToLCDString)==0:
                        self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    else:
                        self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)


                    # lancaigang250607: state; 1-unload filament; 2-feed; 3-; 4-pause
                    self.G_KlipperPrintStatus= 3
                    self.G_PauseToLCDString=""
                    
                    self.G_PhrozenFluiddRespondInfo("[INFO] return")
                    return
                else:
                    # lancaigang240325: is pause, use use pause1
                    if self.G_PauseTriggerWhileChangeChannelFlag==True:
                        # lancaigang231209: timer logical, will error, after to use thread logical interrupt business logic
                        self.G_PhrozenFluiddRespondInfo('[INFO] purge, STM32 send pause')
                        #self.G_PhrozenFluiddRespondInfo("+PAUSE:1,%d" % self.G_ChangeChannelTimeoutNewChan)
                        #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                        #lancaigang250529:
                        if len(self.G_PauseToLCDString)==0:
                            self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        else:
                            self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                        # lancaigang240325: failed, cannotresume
                        self.G_MCModeCanResumeFlag = False
                        # lancaigang250527: pause
                        self.G_KlipperQuickPause = False
                    else:
                        # lancaigang240325: successful, resume
                        self.G_MCModeCanResumeFlag = True
                        self.G_PhrozenFluiddRespondInfo('[INFO] purge normal normal, enterprinting')
                        # lancaigang250527: pause
                        self.G_KlipperQuickPause = True
            else:
                # lancaigang240325: successful, resume
                self.G_MCModeCanResumeFlag = True
                # lancaigang250527: pause
                #self.G_KlipperQuickPause = True
            # lancaigang250619:checkAMS is no re connectsuccessful
            self.Cmds_USBConnectErrorCheck()
            #lancaigang250427: 
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1-AMSfinishtimingbuffer-full time")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2-AMSfinishtimingbuffer-full time")
            self.G_ProzenToolhead.dwell(1.5)
            
            # #lancaigang240318: preventerror up down
            # if self.Lo_ThisIfZPositionLiftUpFlag == True:
            #     command_string = """
            #                     G90
            #                     G91
            #                     G1 Z-%f F8000
            #                     """ % (
            #                     self.G_AMSFilaCutZPositionLiftingUp,
            #                 )
            #     self.G_PhrozenGCode.run_script_from_command(command_string)
            #     self.Lo_ThisIfZPositionLiftUpFlag = False
            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]Z axistemporary down; command_string='%s'" % command_string)

            self.G_ResumeProcessCheckPauseStatus=False
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+T:1,%d" % self.G_ChangeChannelTimeoutNewChan)

            # lancaigang250607: state; 1-unload filament; 2-feed; 3-; 4-pause
            self.G_KlipperPrintStatus= 3

            self.G_PauseToLCDString=""

            return
        

        self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))

        # failed
        if self.G_DictChangeChannelWaitAreaParam["A"] == 0:
            # lancaigang250619:checkAMS is no re connectsuccessful
            self.Cmds_USBConnectErrorCheck()

            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]failed; filamentfeedto toolheadtimeout; cmd='%s', full all filament, klipperpause" % (gcmd.get_commandline()))
            # #// retract allto park position; //===== P2 A1 allfilamentto park position Yes; "AP";
            # self.Cmds_AMSSerial1Send("AP")
            # self.G_PhrozenFluiddRespondInfo("[INFO] Sending command: AP, fully retract to the park position")
            # lancaigang231209: stm32 move report then report9
            if self.G_KlipperIfPaused==False:
                # lancaigang240328: if is manualcommand, pause
                if self.ManualCmdFlag==True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] move command, klipperdo not executepause')
                    # lancaigang250409: get AMS
                    self.Cmds_CmdP114(None)
                elif self.G_CutCheckTest==True:
                    self.G_PhrozenFluiddRespondInfo('[DEBUG] cutter Test command, klipperdo not executepause')
                    # lancaigang250409: get AMS
                    self.Cmds_CmdP114(None)
                else:
                    #lancaigang250702: 
                    if self.G_KlipperInPausing == False:
                        self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                        self.STM32ReprotPauseFlag=1
                        # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                        self.G_ChangeChannelFirstFilaFlag=True

                        self.G_PhrozenFluiddRespondInfo('[INFO] timeout60s, pause')
                        #self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        
                        #lancaigang240416:
                        if self.G_SerialPort1OpenFlag == True:
                            # lancaigang240603: preventAMS
                            self.Cmds_AMSSerial1Send("AT+PAUSE")
                            self.G_PhrozenFluiddRespondInfo('[WARN] serial port 1-AT+PAUSEpausestm32 machine')
                        #lancaigang241030:
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+PAUSE")
                            self.G_PhrozenFluiddRespondInfo('[WARN] serial port 2-AT+PAUSEpausestm32 machine')

                        self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                        self.G_KlipperQuickPause = True
                        # klipper move pause
                        self.Cmds_PhrozenKlipperPause(None)
                        self.G_KlipperIfPaused = True

                        #lancaigang250529:
                        if len(self.G_PauseToLCDString)==0:
                            self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        else:
                            self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)

                        # lancaigang240325: failed, cannotresume
                        self.G_MCModeCanResumeFlag = False
                        # lancaigang250527: pause
                        self.G_KlipperQuickPause = False
                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")
            # lancaigang240124: cannotpause
            else:
                self.G_PhrozenFluiddRespondInfo('[INFO] already pause, do notpause')
                # lancaigang240509: disable disable
                # #lancaigang240326: reportpause
                # #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                # if len(self.G_PauseToLCDString)==0:
                #     self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d" % self.G_ChangeChannelTimeoutNewChan)
                # lancaigang240417: preventstm32reportwhen G_PauseToLCDString is empty
                #self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d" % self.G_ChangeChannelTimeoutNewChan)
                # lancaigang240325: failed, cannotresume
                self.G_MCModeCanResumeFlag = False
                # lancaigang250527: pause
                self.G_KlipperQuickPause = False

                # lancaigang240429: ifDuring resumestm32 and noreportpausestate, need toreportpause
                if self.G_ResumeProcessCheckPauseStatus==False:
                    self.G_PhrozenFluiddRespondInfo('[INFO] AMShas up report pause, klipperpause, need to up report pause')
                    #lancaigang250529:
                    if len(self.G_PauseToLCDString)==0:
                        self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    else:
                        self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                    #lancaigang240416:
                    if self.G_SerialPort1OpenFlag == True:
                        # lancaigang240603: preventAMS
                        self.Cmds_AMSSerial1Send("AT+PAUSE")
                        self.G_PhrozenFluiddRespondInfo('[WARN] serial port 1-AT+PAUSEpausestm32 machine')
                    #lancaigang241030:
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AT+PAUSE")
                        self.G_PhrozenFluiddRespondInfo('[WARN] serial port 2-AT+PAUSEpausestm32 machine')
                else:#True
                    self.G_PhrozenFluiddRespondInfo('[INFO] AMShas up report pause, klipper need to need to up report pause')
                    self.G_ResumeProcessCheckPauseStatus=False

                    #lancaigang250529:
                    if len(self.G_PauseToLCDString)==0:
                        self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    else:
                        self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)


                # self.G_PhrozenFluiddRespondInfo("[ERROR] already pause, then pause1 time, preventbeforepauseerror")
                # #lancaigang250423: is preventerrorpause, pause1 time
                # #klipper move pause
                # self.Cmds_PhrozenKlipperPause(None)

            # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
            self.G_ChangeChannelFirstFilaFlag=True

            self.G_IfChangeFilaOngoing= False

            self.G_ResumeProcessCheckPauseStatus=False
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+T:1,%d" % self.G_ChangeChannelTimeoutNewChan)

            # lancaigang250607: state; 1-unload filament; 2-feed; 3-; 4-pause
            self.G_KlipperPrintStatus= -1

            return

        # normal normal; Action normal normal
        if self.G_DictChangeChannelWaitAreaParam["A"] == 1:
            pass


    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
   # P1 C[n] n:1~32(no network, get 1~4) automaticto channel(move action command,cut filament,, wait)
    def Cmds_P1CnAutoChangeChannel(self, chan, gcmd):
            # lancaigang241030: is P1 C1to P1 C32, in 1to 32
            # unit 1: 1 2 3 4
            # unit 2: 5 6 7 8
            # #3 unit : 9 10 11 12
            # #4 unit : 13 14 15 16
            # #5 unit : 17 18 19 20
            # #6 unit : 21 22 23 24
            # #7 unit : 25 26 27 28
            # #8 unit : 29 30 31 32
        self.G_PhrozenFluiddRespondInfo("[INFO] =====[(cmds.python)Cmds_P1CnAutoChangeChannel]")
        self.G_PhrozenFluiddRespondInfo('===== up time self.G_ChangeChannelTimeoutOldChan=%d' % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo('===== up time self.G_ChangeChannelTimeoutNewChan=%d' % self.G_ChangeChannelTimeoutNewChan)
        if gcmd is None:
            self.G_PhrozenFluiddRespondInfo("[INFO] gcmd is None")
            #self.G_PhrozenFluiddRespondInfo("[INFO] return")
            #return
            #pass
        if gcmd is not None:
            self.G_PhrozenFluiddRespondInfo("[INFO] gcmd is not None:")
            self.G_PhrozenFluiddRespondInfo("===== up time '%s';self.G_ChangeChannelTimeoutOldChan=%d" % (gcmd.get_commandline(),self.G_ChangeChannelTimeoutOldChan))
            self.G_PhrozenFluiddRespondInfo("===== up time '%s';self.G_ChangeChannelTimeoutNewChan=%d" % (gcmd.get_commandline(),self.G_ChangeChannelTimeoutNewChan))
        
        self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))




        #lancaigang250824:
        self.G_PhrozenFluiddRespondInfo("[INFO] self.G_ProzenToolhead.wait_moves()")
        self.G_ProzenToolhead.wait_moves()





        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        #lancaigang250517: 
        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")

        self.G_PhrozenFluiddRespondInfo("[INFO] Current mode")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:0,unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:1,MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:2,MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo("[WARN] +Mode:3,RUNOUT")
        else:
            self.G_PhrozenFluiddRespondInfo("[ERROR] +Mode:-1,error")


        #Unlock
        self.Base_AMSSerialCmdUnlock()


        #lancaigang250605:
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo('[WARN] ifreceived T?command, but mode is Unknown mode, switch is MCmulti-materialmode')
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC

        self.G_PhrozenFluiddRespondInfo("[INFO] Current mode")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:0,unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:1,MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:2,MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo("[WARN] +Mode:3,RUNOUT")
        else:
            self.G_PhrozenFluiddRespondInfo("[ERROR] +Mode:-1,error")


        # lancaigang250526: pause, not allowednew gcodecommand, need waitpausecomplete
        if self.G_KlipperInPausing == True:
            self.G_PhrozenFluiddRespondInfo('[INFO] pause, not allowednew gcodecommand, need pausecomplete')
            for num in range(30):
                # lancaigang231115: is 1s
                self.G_PhrozenFluiddRespondInfo("[INFO] self.G_ProzenToolhead.dwell(1)")
                self.G_ProzenToolhead.dwell(1)
                self.G_PhrozenFluiddRespondInfo('[INFO] pause, not allowednew gcodecommand, need pausecomplete')
                Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
                self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
                
                #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
                if Lo_PauseStatus['is_paused'] == True:
                    self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
                else:
                    self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")

                if self.G_KlipperInPausing == False:
                    self.G_PhrozenFluiddRespondInfo("[INFO] pausefinish")
                    Lo_ChangeChannelIfSuccess = True

                    Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                    self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
                    self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
                    #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
                    if Lo_PauseStatus['is_paused'] == True:
                        self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
                    else:
                        self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
                        # klipperpausecommand; keep current x y z
                        # lancaigang240108: to time pause keep number data is no normal normal, after to
                        self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE")
                        self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)]PAUSE")
                        self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                        self.G_ProzenToolhead.wait_moves()
                        self.G_ProzenToolhead.dwell(1.0)

                    self.G_PhrozenFluiddRespondInfo("[INFO] break")
                    break

            # lancaigang250725: if, is nopause, then up pause
            if self.G_KlipperInPausing == True:
                self.G_PhrozenFluiddRespondInfo('[INFO] =====pause, received new command, but pausecomplete, pause')
                Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
                self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
                #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
                if Lo_PauseStatus['is_paused'] == True:
                    self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
                else:
                    self.G_PhrozenFluiddRespondInfo('[WARN] =====Not currently paused, pause action')
                    # klipperpausecommand; keep current x y z
                    # lancaigang240108: to time pause keep number data is no normal normal, after to
                    self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE")
                    self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)]PAUSE")
                    self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                    self.G_ProzenToolhead.wait_moves()
                    self.G_ProzenToolhead.dwell(1.0)

        else:
            self.G_PhrozenFluiddRespondInfo('[INFO] in pause state')
            self.G_PhrozenFluiddRespondInfo("[INFO] self.G_KlipperInPausing == False")

        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")

        # lancaigang250512: pause1 time, preventbeforenopausesuccessful
        if self.G_KlipperIfPaused == True:
            # is resumestate
            if self.G_ChangeChannelResumeFlag==False:
                self.G_PhrozenFluiddRespondInfo('[INFO] is resumestate')
                self.G_PhrozenFluiddRespondInfo('[INFO] klipperpause, but received command')
                # lancaigang250508:preventpauseerror
                self.G_PhrozenFluiddRespondInfo('[INFO] klipperpause, but received command, then time pause')
                self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                self.G_PauseToLCDString="+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                #self.Cmds_PhrozenKlipperPause(None)

                Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
                self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
                #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
                if Lo_PauseStatus['is_paused'] == True:
                    self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
                else:
                    self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
                    # klipperpausecommand; keep current x y z
                    # lancaigang240108: to time pause keep number data is no normal normal, after to
                    self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE")
                    self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)]PAUSE")
                    self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                    self.G_ProzenToolhead.wait_moves()
                    self.G_ProzenToolhead.dwell(1.0)

                #lancaigang250524:
                self.G_PhrozenFluiddRespondInfo('[INFO] pause, received new gcodecommand, need to new new old, prevent stop')
                self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
                self.G_ChangeChannelTimeoutOldGcmd=self.G_ChangeChannelTimeoutNewGcmd
                self.G_ChangeChannelTimeoutNewChan=chan
                self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)
                self.G_ChangeChannelTimeoutNewGcmd=gcmd
                self.Cmds_PrintMode(self.G_AMSDeviceWorkMode)

                Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
                self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
                #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
                if Lo_PauseStatus['is_paused'] == True:
                    self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
                else:
                    self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")

                self.G_PhrozenFluiddRespondInfo("[INFO] return")
                return
            

       # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()
        if self.G_SerialPort1Obj is not None:
            if self.G_SerialPort1Obj.is_open:
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 is open")
                #self.G_SerialPort1Obj.flushInput()
                # self.G_PhrozenFluiddRespondInfo("[INFO] G_SerialPort1Obj.flushInputserial portclear")
        if self.G_SerialPort2Obj is not None:
            #lancaigang241030:
            if self.G_SerialPort2Obj.is_open:
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 is open")

        self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))

        # lancaigang240226: cut filament after AMSmainboardfilament, when after toolheadretract20mm
        #time.sleep(2)
        #self.G_ProzenToolhead.dwell(2.0)

        self.G_PauseTriggerWhileChangeChannelFlag=False
        self.G_PhrozenFluiddRespondInfo("+C:0,%d" % chan)

        self.G_ASM1DisconnectErrorCount=0



        # #lancaigang240322: pause and when send, ifG?commandalready detectedfilament runoutpause,
        # if self.STM32ReprotPauseFlag==1:
        #     self.G_PhrozenFluiddRespondInfo("self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)
        #     self.G_PhrozenFluiddRespondInfo("self.G_Pause1Channel=%d" % self.G_Pause1Channel)
        #     if self.G_PauseTriggerWhileChangeChannelFlag==True:
        # self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)Cmds_P1CnAutoChangeChannel]G?commanddetectedpausereport, pause")
        # #lancaigang240325: is filament runoutno get output disable, use pause1 table show
        #         #self.G_PhrozenFluiddRespondInfo("+PAUSE:1,%d" % self.G_Pause1Channel)
        #         if "+PAUSE:1" in self.G_PauseToLCDString:
        #             self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
        #         #else:
        #             #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
        #         self.G_ChangeChannelFirstFilaFlag=True
        #         self.G_IfChangeFilaOngoing= False
        # #lancaigang240524: use UIUX move
        #         self.G_PhrozenFluiddRespondInfo("+C:1,%d" % self.G_ChangeChannelTimeoutNewChan)
        #         return
        #     else:
        # self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)Cmds_P1CnAutoChangeChannel]G?commandnodetectedpausereport, need tofeedresume")
        #         self.G_ChangeChannelFirstFilaFlag=True
        #         self.G_IfChangeFilaOngoing= False
        # #lancaigang240325: nofilament runout, canfeed
        #         #return


        self.G_IfChangeFilaOngoing= True



        # lancaigang250102: number
        self.G_PrintCountNum=self.G_PrintCountNum+1
        self.G_PhrozenFluiddRespondInfo('time number =%d' % self.G_PrintCountNum)


        # first time #1 channelfilament
        if self.G_ChangeChannelFirstFilaFlag:
            # #lancaigang240314: # channelfirst move move to position
            # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_P1CnAutoChangeChannel]External macro command-PG104")
            # command_string = """
            # PG104
            # """
            # self.G_PhrozenGCode.run_script_from_command(command_string)
            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]External macro command- #1 channel move move to position; command_string='%s'" % command_string)


            #lancaigang240125: 
            self.G_PhrozenFluiddRespondInfo('[INFO] first time #1;pauseresume #1')

            # #lancaigang240124: stm32 move report, canpause1 time
            # self.STM32ReprotPauseFlag=0
            # self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)Cmds_P1CnAutoChangeChannel]self.STM32ReprotPauseFlag=0")



            
            # lancaigang231202: if #1 channelfeed after klippererrorpause value false, then resume after no #1 time
            self.G_ChangeChannelFirstFilaFlag = False

            # is resumestate, then need tocut filament
            if self.G_ChangeChannelResumeFlag==False:
                self.G_PhrozenFluiddRespondInfo('[INFO] first layer printing, is pauseresume')
                self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                self.G_PhrozenFluiddRespondInfo('===== time self.G_ChangeChannelTimeoutOldChan=%d' % self.G_ChangeChannelTimeoutOldChan)
                self.G_ChangeChannelTimeoutOldGcmd=self.G_ChangeChannelTimeoutNewGcmd
                self.G_ChangeChannelTimeoutNewChan=chan
                self.G_PhrozenFluiddRespondInfo('===== time self.G_ChangeChannelTimeoutNewChan=%d' % self.G_ChangeChannelTimeoutNewChan)
                self.G_ChangeChannelTimeoutNewGcmd=gcmd
                self.Cmds_PrintMode(self.G_AMSDeviceWorkMode)




                # lancaigang250619:checkAMS is no re connectsuccessful
                self.Cmds_USBConnectErrorCheck()
                #lancaigang241030:
                if self.G_ChangeChannelTimeoutOldChan in range(1, 5):# 1 2 3 4
                    # #lancaigang241011: beforeAMSfirst, then PG101; to pauseresumeretractcut filament
                    self.G_PhrozenFluiddRespondInfo("+H:0,%d" % self.G_ChangeChannelTimeoutOldChan)
                    self.Cmds_AMSSerial1Send("H%d" % self.G_ChangeChannelTimeoutOldChan)
                    self.G_PhrozenFluiddRespondInfo('serial port 1Sending command: H%d' % self.G_ChangeChannelTimeoutOldChan)
                    self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1 before AMSretract')
                    self.G_PhrozenFluiddRespondInfo("+H:1,%d" % self.G_ChangeChannelTimeoutOldChan)
                elif self.G_ChangeChannelTimeoutOldChan in range(5, 9):# 5 6 7 8
                    self.G_PhrozenFluiddRespondInfo("+H:0,%d" % self.G_ChangeChannelTimeoutOldChan-4)
                    self.Cmds_AMSSerial2Send("H%d" % self.G_ChangeChannelTimeoutOldChan-4)
                    self.G_PhrozenFluiddRespondInfo('serial port 2Sending command: H%d' % self.G_ChangeChannelTimeoutOldChan-4)
                    self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2 before AMSretract')
                    self.G_PhrozenFluiddRespondInfo("+H:1,%d" % self.G_ChangeChannelTimeoutOldChan-4)
                else:
                    self.G_PhrozenFluiddRespondInfo('[INFO] error, all filament')
                    if self.G_SerialPort1OpenFlag == True:
                        # lancaigang240913: resumewhen, project is, can full all all, prevent old channelerror, new channelfeederror
                        self.Cmds_AMSSerial1Send("AP")
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: AP; all filament')
                    #lancaigang241030:
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AP")
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2Sending command: AP; all filament')

                    # lancaigang240913: set to external
                    self.G_ProzenToolhead.dwell(6)







                # #lancaigang241011: PG101beforeG?command, after PG101toolheadretract action, in after MCG?command
                # self.Cmds_AMSSerial1Send("MC")
                # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_P1CnAutoChangeChannel]sending command: MC")
                # self.G_PhrozenFluiddRespondInfo("[INFO] AMS")


                self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))

                self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG104- get before full change variable')
                command_string = """
                    PG104
                    """
                self.G_PhrozenGCode.run_script_from_command(command_string)
                self.G_PhrozenFluiddRespondInfo("External macro command-PG104- get before full change variable; command_string='%s'" % command_string)
                self.IfDoPG102Flag=True


                # lancaigang240510: before, first to waiting area
                # lancaigang240306: move move to cut filament code
                # lancaigang240110: waitwaitbefore, first External macro command, move move to positionwait
                # lancaigang240515: before, first first to to waiting area
                self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG101-retract")
                command_string = """
                    PG101
                    """
                self.G_PhrozenGCode.run_script_from_command(command_string)
                self.G_PhrozenFluiddRespondInfo("External macro command-to waiting areapositionpurge; command_string='%s'" % command_string)
                self.IfDoPG102Flag=True

                #lancaigang250323: 
                if self.G_ToolheadIfHaveFilaFlag==True:
                    self.G_PhrozenFluiddRespondInfo("[INFO] toolheadhas filament")



                    # lancaigang240909:cut filament move action in PG106before
                    # for i in range(15):
                    # self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)]purge complete")
                    # #lancaigang20231013: is 4when
                    # #lancaigang231115: is 1s
                    #     self.G_ProzenToolhead.dwell(1.0)
                    # #lancaigang240125: cannot use sleep, will thread
                    #     #time.sleep(1)
                    # lancaigang240319: cut filamentbefore move action
                    #self.Cmds_MoveToCutFilaPrepare()
                    # lancaigang20231205: cutter cut filament
                    self.Cmds_MoveToCutFilaAction(gcmd)

                    #lancaigang250519:
                    self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PRZ_CUT_WAITINGAREA")
                    command_string = """
                        PRZ_CUT_WAITINGAREA
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.G_PhrozenFluiddRespondInfo("External macro command-to waiting areaposition; command_string='%s'" % command_string)


                    # lancaigang240226: cut filament after AMSmainboardfilament, when after toolheadretract20mm
                    #time.sleep(2)
                    self.G_ProzenToolhead.dwell(0.5)


                    # lancaigang250619:checkAMS is no re connectsuccessful
                    self.Cmds_USBConnectErrorCheck()
                    #lancaigang241030:
                    if self.G_ChangeChannelTimeoutOldChan in range(1, 5):
                        # lancaigang240906: new AMS, cut filament after, up time channel
                        # lancaigang20231013: stm32
                        # lancaigang231129: stm32 internal all klipper, stm32 internal all, klipperiftoolheadcut filamenterrorno unload filament, klippererrorempty
                        self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutOldChan)
                        self.G_PhrozenFluiddRespondInfo("G%d" % self.G_ChangeChannelTimeoutOldChan)
                        self.G_PhrozenFluiddRespondInfo('serial port 1-AMS old first : G%d' % self.G_ChangeChannelTimeoutOldChan)
                    elif self.G_ChangeChannelTimeoutOldChan in range(5, 9):
                        self.Cmds_AMSSerial2Send("G%d" % self.G_ChangeChannelTimeoutOldChan-4)
                        self.G_PhrozenFluiddRespondInfo("G%d" % self.G_ChangeChannelTimeoutOldChan-4)
                        self.G_PhrozenFluiddRespondInfo('serial port 2-AMS old first : G%d' % self.G_ChangeChannelTimeoutOldChan-4)


                    self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))



                    # get channel and gcmd to
                    #self.G_ChangeChannelTimeoutOldChan=chan
                    #self.G_ChangeChannelTimeoutOldGcmd=gcmd

                    self.G_ProzenToolhead.dwell(0.5)

                    


                    # lancaigang240913: set to external
                    self.G_ProzenToolhead.dwell(6.5)
                    # lancaigang240911: Gcommandafterwhen 6checktoolhead is no hasfilament
                    # lancaigang231201: checkcut filament after old channelfilament is no normal normal unload filament, normal normal then pause
                    self.Cmds_CutFilaIfNormalCheck()
                    if self.G_KlipperIfPaused == True:
                        self.G_PhrozenFluiddRespondInfo('[INFO] cut filament？toolheaddetectedfilament, cutter error, please check cutter, pauseklipperprinting')
                        #Lo_ChangeChannelIfSuccess = False
                        return
                # else:
                # self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PRZ_WAITINGAREA-wait")
                #     command_string = """
                #         PRZ_WAITINGAREA
                #         """
                #     self.G_PhrozenGCode.run_script_from_command(command_string)
                #     self.G_PhrozenFluiddRespondInfo("External macro command-PRZ_WAITINGAREA; command_string='%s'" % command_string)



                self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))

                # lancaigang240328: manualcommanddo not executepurge
                if self.ManualCmdFlag==True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG106; move command, do not executepurgeFunction')
                else:
                    # lancaigang240319: after, first reserve toolheadfilament, preventsmall pellet
                    self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG106; cut filament after, toolheadheat upwhen AMS')
                    self.PG102Flag=True
                    self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                    command_string = """
                    PG106
                    """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)
                    self.PG102Flag=False
                    self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=False")


                # lancaigang231216: ifpause, z, to pausewhen, set z keep, error
                # lancaigang231216: ifzno down, need to down then pause
                if self.G_IfZPositionLiftUpFlag==True:
                    command_string = """
                        G90
                        G91
                        G1 Z-%f F8000
                        """ % (
                        self.G_AMSFilaCutZPositionLiftingUp,
                    )
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.G_IfZPositionLiftUpFlag = False
                    self.G_PhrozenFluiddRespondInfo("Z axis down; command_string='%s'" % command_string)
            

                self.G_ProzenToolhead.dwell(0.5)

                # lancaigang20231013: manual
                self.Cmds_P1TnManualChangeChannel(self.G_ChangeChannelTimeoutNewChan, self.G_ChangeChannelTimeoutNewGcmd)
            # lancaigang240912:pauseresume, use use old old channel and new channel
            else:
                self.G_PhrozenFluiddRespondInfo('[INFO] is first layer printing, is pauseresume')
                # lancaigang20231013: manual
                self.Cmds_P1TnManualChangeChannel(self.G_ChangeChannelTimeoutNewChan, self.G_ChangeChannelTimeoutNewGcmd)









        # after #n channelfilament
        else:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_P1CnAutoChangeChannel] after #n; else')
            self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
            self.G_PhrozenFluiddRespondInfo('===== time self.G_ChangeChannelTimeoutOldChan=%d' % self.G_ChangeChannelTimeoutOldChan)
            self.G_ChangeChannelTimeoutOldGcmd=self.G_ChangeChannelTimeoutNewGcmd
            self.G_ChangeChannelTimeoutNewChan=chan
            self.G_PhrozenFluiddRespondInfo('===== time self.G_ChangeChannelTimeoutNewChan=%d' % self.G_ChangeChannelTimeoutNewChan)
            self.G_ChangeChannelTimeoutNewGcmd=gcmd
            self.Cmds_PrintMode(self.G_AMSDeviceWorkMode)



            # lancaigang240124: stm32 move report, canpause1 time
            self.STM32ReprotPauseFlag=0
            self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)Cmds_P1CnAutoChangeChannel]self.STM32ReprotPauseFlag=0")
            self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))

            # lancaigang250619:checkAMS is no re connectsuccessful
            self.Cmds_USBConnectErrorCheck()
            #lancaigang241030:
            if self.G_ChangeChannelTimeoutOldChan in range(1, 5):
                # #lancaigang241011: beforeAMSfirst, then PG101; to pauseresumeretractcut filament
                self.G_PhrozenFluiddRespondInfo("+H:0,%d" % self.G_ChangeChannelTimeoutOldChan)
                self.Cmds_AMSSerial1Send("H%d" % self.G_ChangeChannelTimeoutOldChan)
                self.G_PhrozenFluiddRespondInfo('serial port 1Sending command: H%d' % self.G_ChangeChannelTimeoutOldChan)
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1 before AMSretract')
                self.G_PhrozenFluiddRespondInfo("+H:1,%d" % self.G_ChangeChannelTimeoutOldChan)
            elif self.G_ChangeChannelTimeoutOldChan in range(5, 9):
                self.G_PhrozenFluiddRespondInfo("+H:0,%d" % self.G_ChangeChannelTimeoutOldChan-4)
                self.Cmds_AMSSerial2Send("H%d" % self.G_ChangeChannelTimeoutOldChan-4)
                self.G_PhrozenFluiddRespondInfo('serial port 2Sending command: H%d' % self.G_ChangeChannelTimeoutOldChan-4)
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2 before AMSretract')
                self.G_PhrozenFluiddRespondInfo("+H:1,%d" % self.G_ChangeChannelTimeoutOldChan-4)
            else:
                    self.G_PhrozenFluiddRespondInfo('[INFO] error, all filament')
                    if self.G_SerialPort1OpenFlag == True:
                        # lancaigang240913: resumewhen, project is, can full all all, prevent old channelerror, new channelfeederror
                        self.Cmds_AMSSerial1Send("AP")
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: AP; all filament')
                    #lancaigang241030:
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AP")
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2Sending command: AP; all filament')

                    # lancaigang240913: set to external
                    self.G_ProzenToolhead.dwell(6)

            


            #lancaigang250824:
            self.G_PhrozenFluiddRespondInfo("[INFO] self.G_ProzenToolhead.wait_moves()")
            self.G_ProzenToolhead.wait_moves()


            self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG104- get before full change variable')
            command_string = """
                PG104
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command-PG104- get before full change variable; command_string='%s'" % command_string)
            self.IfDoPG102Flag=True
            
            
            # lancaigang240510: before, first to waiting area
            # lancaigang240306: move move to cut filament code
            # lancaigang240110: waitwaitbefore, first External macro command, move move to positionwait
            # lancaigang240515: before, first first to to waiting area
            self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_P1CnAutoChangeChannel]External macro command-PG101-retract")
            command_string = """
                PG101
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]External macro command-to waiting areapositionpurge; command_string='%s'" % command_string)
            self.IfDoPG102Flag=True


            self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))



            #lancaigang250824:
            self.G_PhrozenFluiddRespondInfo("[INFO] self.G_ProzenToolhead.wait_moves()")
            self.G_ProzenToolhead.wait_moves()


            #lancaigang250323: 
            #if self.G_ToolheadIfHaveFilaFlag==True:
                # self.G_PhrozenFluiddRespondInfo("[INFO] toolheadhasfilament")
                # lancaigang20231013: cutter cut filament, Zto X Ycut filamentposition
            self.Cmds_MoveToCutFilaAction(gcmd)
            #else:
            # self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PRZ_WAITINGAREA-wait")
            #    command_string = """
            #        PRZ_WAITINGAREA
            #        """
            #    self.G_PhrozenGCode.run_script_from_command(command_string)
            #    self.G_PhrozenFluiddRespondInfo("External macro command-PRZ_WAITINGAREA; command_string='%s'" % command_string)


            #lancaigang250824:
            self.G_PhrozenFluiddRespondInfo("[INFO] self.G_ProzenToolhead.wait_moves()")
            self.G_ProzenToolhead.wait_moves()



            #lancaigang250519:
            self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PRZ_CUT_WAITINGAREA")
            command_string = """
                PRZ_CUT_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command-to waiting areaposition; command_string='%s'" % command_string)


            self.G_ProzenToolhead.dwell(0.5)


            #lancaigang250824:
            self.G_PhrozenFluiddRespondInfo("[INFO] self.G_ProzenToolhead.wait_moves()")
            self.G_ProzenToolhead.wait_moves()

            self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))

            # lancaigang250619:checkAMS is no re connectsuccessful
            self.Cmds_USBConnectErrorCheck()
            #lancaigang241030:
            if self.G_ChangeChannelTimeoutOldChan in range(1, 5):
                # lancaigang240906: cut filament after, up time channel
                # lancaigang20231013: stm32
                # lancaigang231129: stm32 internal all klipper, stm32 internal all, klipperiftoolheadcut filamenterrorno unload filament, klippererrorempty
                self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutOldChan)
                self.G_PhrozenFluiddRespondInfo("G%d" % self.G_ChangeChannelTimeoutOldChan)
                self.G_PhrozenFluiddRespondInfo('serial port 1-AMS old first : G%d' % self.G_ChangeChannelTimeoutOldChan)
            elif self.G_ChangeChannelTimeoutOldChan in range(5, 9):
                self.Cmds_AMSSerial2Send("G%d" % self.G_ChangeChannelTimeoutOldChan-4)
                self.G_PhrozenFluiddRespondInfo("G%d" % self.G_ChangeChannelTimeoutOldChan-4)
                self.G_PhrozenFluiddRespondInfo('serial port 2-AMS old first : G%d' % self.G_ChangeChannelTimeoutOldChan-4)


            #lancaigang250824:
            self.G_PhrozenFluiddRespondInfo("[INFO] self.G_ProzenToolhead.wait_moves()")
            self.G_ProzenToolhead.wait_moves()

            # lancaigang250322: PG106 to, do notwhen, but before prompt is PG106
            # lancaigang240913: set to external
            self.G_ProzenToolhead.dwell(6.5)
            #lancaigang250823: 
            self.G_PhrozenFluiddRespondInfo("[INFO] self.G_ProzenToolhead.wait_moves()")
            self.G_ProzenToolhead.wait_moves()
            # lancaigang231201: checkcut filament after is no normal normal unload filament, normal normal then pause
            # lancaigang231215: Z axis up after down
            # lancaigang231216: wait6check is no cut filamentsuccessful
            self.Cmds_CutFilaIfNormalCheck()
            if self.G_KlipperIfPaused == True:
                self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_P1CnAutoChangeChannel]cut filament？toolheaddetectedfilament, cutter error, please check cutter, pauseklipperprinting')
                #self.Cmds_PhrozenKlipperPause(None)

                Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
                self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
                #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
                if Lo_PauseStatus['is_paused'] == True:
                    self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
                else:
                    self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
                    # klipperpausecommand; keep current x y z
                    # lancaigang240108: to time pause keep number data is no normal normal, after to
                    self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE")
                    self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)]PAUSE")
                    self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                    self.G_ProzenToolhead.wait_moves()
                    self.G_ProzenToolhead.dwell(1.0)

                    self.G_PhrozenFluiddRespondInfo('[INFO] toolhead cutter or device error, pause')
                    #self.G_PhrozenFluiddRespondInfo("+PAUSE:8,%d" % self.G_ChangeChannelTimeoutOldChan)
                    #lancaigang250414:
                    #self.G_PhrozenFluiddRespondInfo("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    if len(self.G_PauseToLCDString)==0:
                        self.G_PhrozenFluiddRespondInfo("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    else:
                        self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                # #lancaigang250524:
                # self.G_PhrozenFluiddRespondInfo("[WARN] pause, received new gcodecommand, need tonew new old channel, prevent")
                # self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                # self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
                # self.G_ChangeChannelTimeoutOldGcmd=self.G_ChangeChannelTimeoutNewGcmd
                # self.G_ChangeChannelTimeoutNewChan=chan
                # self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)
                # self.G_ChangeChannelTimeoutNewGcmd=gcmd

                Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
                self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
                #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
                if Lo_PauseStatus['is_paused'] == True:
                    self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
                else:
                    self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")

                self.G_PhrozenFluiddRespondInfo("[INFO] return")
                return

            # lancaigang240229: preventsending command
            #time.sleep(1)
            self.G_ProzenToolhead.dwell(0.5)



            self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))

            # lancaigang240319: after, first reserve toolheadfilament, preventsmall pellet
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)]External macro command-PG106; cut filament after, toolheadheat upwhen AMS')
            self.PG102Flag=True
            self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)]self.Flag=True")
            command_string = """
            PG106
            """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("[(cmds.python)]command_string='%s'" % command_string)
            self.PG102Flag=False
            self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)]self.Flag=False")
            
            # lancaigang250619:checkAMS is no re connectsuccessful
            self.Cmds_USBConnectErrorCheck()
            #lancaigang241030:
            if self.G_ChangeChannelTimeoutNewChan in range(1, 5):
                # lancaigang240911: new AMS, Tcommandfeed
                # lancaigang20231013: stm32
                # lancaigang231129: stm32 internal all klipper, stm32 internal all, klipperiftoolheadcut filamenterrorno unload filament, klippererrorempty
                self.Cmds_AMSSerial1Send("T%d" % self.G_ChangeChannelTimeoutNewChan)
                self.G_PhrozenFluiddRespondInfo('serial port 1Sending command: T%d' % self.G_ChangeChannelTimeoutNewChan)
            elif self.G_ChangeChannelTimeoutNewChan in range(5, 9):
                self.Cmds_AMSSerial2Send("T%d" % self.G_ChangeChannelTimeoutNewChan-4)
                self.G_PhrozenFluiddRespondInfo('serial port 2Sending command: T%d' % self.G_ChangeChannelTimeoutNewChan-4)






            #lancaigang240322: 
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)]External macro command-PG105; cut filament after, toolheadheat upwhen AMS')
            self.PG102Flag=True
            self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)]self.Flag=True")
            command_string = """
            PG105
            """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("[(cmds.python)]command_string='%s'" % command_string)
            self.PG102Flag=False
            self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)]self.Flag=False")

            self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))

            # lancaigang240328: manualcommanddo not executepurge
            if self.ManualCmdFlag==True:
                self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG110; move command, do not execute')
            else:
                # lancaigang240319: after, first reserve toolheadfilament, preventsmall pellet
                self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG110; STM32feed after, klipperstartpurgefeed')
                self.PG102Flag=True
                self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                command_string = """
                PG110
                """
                self.G_PhrozenGCode.run_script_from_command(command_string)
                self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)
                self.PG102Flag=False
                self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=False")


            # lancaigang240229: z down, do notto waiting area
            if self.G_IfZPositionLiftUpFlag==True:
                command_string = """
                    G90
                    G91
                    G1 Z-%f F8000
                    """ % (
                    self.G_AMSFilaCutZPositionLiftingUp,
                )
                self.G_PhrozenGCode.run_script_from_command(command_string)
                self.G_IfZPositionLiftUpFlag = False
                self.G_PhrozenFluiddRespondInfo("Z axis down; command_string='%s'" % command_string)



            # lancaigang240223: ifcut filamentfailed, up number already Z axis down, already pausedo not execute down action
            if self.ToolheadCutFlag==True:
                self.ToolheadCutFlag=False
                self.G_PhrozenFluiddRespondInfo('[INFO] before cut filamenterror, failed')
                self.G_ChangeChannelFirstFilaFlag=True
                self.G_IfChangeFilaOngoing= False

                # stm32reportpausepause1 time, cannotpause
                self.STM32ReprotPauseFlag=1
                # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                self.G_ChangeChannelFirstFilaFlag=True

                # lancaigang250308: resumealready cut filamenterror, report cut filamenterror
                #self.G_PhrozenFluiddRespondInfo("+PAUSE:8,%d" % self.G_ChangeChannelTimeoutNewChan)
                #self.G_PhrozenFluiddRespondInfo("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                
                #lancaigang240416:
                if self.G_SerialPort1OpenFlag == True:
                    # lancaigang240603: preventAMS
                    self.Cmds_AMSSerial1Send("AT+PAUSE")
                    self.G_PhrozenFluiddRespondInfo('[WARN] serial port 1-AT+PAUSEpausestm32 machine')
                #lancaigang241030:
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("AT+PAUSE")
                    self.G_PhrozenFluiddRespondInfo('[WARN] serial port 2-AT+PAUSEpausestm32 machine')



                if self.G_KlipperInPausing == False:
                    self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                    #lancaigang250607:
                    self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                    self.G_KlipperQuickPause = True
                    # klipper move pause
                    self.Cmds_PhrozenKlipperPause(None)
                else:
                    self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")



                self.G_KlipperIfPaused = True

                # lancaigang240325: failed, cannotresume
                self.G_MCModeCanResumeFlag = False

                if len(self.G_PauseToLCDString)==0:
                    self.G_PhrozenFluiddRespondInfo("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                else:
                    self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)

                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+C:1,%d" % self.G_ChangeChannelTimeoutNewChan)
                
                self.G_PhrozenFluiddRespondInfo("[INFO] return")
                return

            self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))

            # #lancaigang231208: z- number will down
            # #lancaigang231213: F7200 is 300
            # #lancaigang231215: Z axis up after down
            # command_string = """
            #     G90
            #     G1 X%.3f Y%.3f F8000
            #     G91
            #     G1 Z-%f F8000
            #     """ % (
            #     self.G_DictChangeChannelWaitAreaParam["X"],
            #     self.G_DictChangeChannelWaitAreaParam["Y"],
            #     self.G_AMSFilaCutZPositionLiftingUp,
            # )
            # self.G_PhrozenGCode.run_script_from_command(command_string)
            # #lancaigang231216: ifpause, z, to pausewhen, set z keep, error
            # self.G_IfZPositionLiftUpFlag = False
            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel] internal all gcodeto waitx y z down; command_string='%s'" % command_string)

            
            # set
            Lo_ChangeChannelIfSuccess = False
            self.G_PhrozenFluiddRespondInfo("[INFO] Lo_ChangeChannelIfSuccess = False")


            # lancaigang231202: ifP9commandmovepath number is empty, len number errorklipper
            # is 0 then path
            # lancaigang231206: UIifpauseresume, is noP9, move number is empty, codedump
            if self.ChangeWaitMoveArea is None:
                self.G_PhrozenFluiddRespondInfo('[INFO] waiting area move move error;klipperpause')
                Lo_ChangeChannelIfSuccess = False
                pass

            if self.ChangeWaitMoveArea is not None:
                # empty list table
                if len(self.ChangeWaitMoveArea) == 0:
                    self.G_PhrozenFluiddRespondInfo('[INFO] return;waiting area move move error, path;if len(self.ChangeWaitMoveArea) == 0')
                    # lancaigang231206: down
                    #return
                else:
                    self.G_PhrozenFluiddRespondInfo('[INFO] for;waiting area move move normal normal, pathqueue, new filamentto toolhead')


                # #lancaigang240306: move move to cut filament code
                # #lancaigang240110: waitwaitbefore, first External macro command, move move to positionwait
                # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_P1CnAutoChangeChannel]External macro command-PG101")
                # command_string = """
                #     PG101
                #     """
                # self.G_PhrozenGCode.run_script_from_command(command_string)
                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]External macro command-to positionwaitpurge; command_string='%s'" % command_string)
                # self.IfDoPG102Flag=True

                #lancaigang250519:
                self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PRZ_SPITTING_SCRAPE")
                command_string = """
                    PRZ_SPITTING_SCRAPE
                    """
                self.G_PhrozenGCode.run_script_from_command(command_string)
                self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)


                # lancaigang250607: state; 1-unload filament; 2-feed; 3-; 4-pause
                self.G_KlipperPrintStatus= 2
                # Python enumerate() number
                # enumerate() number use set number data to (list table 、 or) is sequence list, when list output number data and number data down, use in for 。
                # Python 2.3. up version use, 2.6 add start parameter number 。
                # for use use enumerate
                # >>> seq = ['one', 'two', 'three']
                # >>> for i, element in enumerate(seq):
                # ...     print i, element
                # ...
                # 0 one
                # 1 two
                # 2 three
                # wait move, 80timeout; move move
                # for i in range(CHANGE_CHANNEL_WAIT_TIMEOUT):#120
                #for num, point in enumerate(self.ChangeWaitMoveArea):
                for num in range(CHANGE_CHANNEL_WAIT_TIMEOUT):
                    # lancaigang231202: ifSTM32 move reportpause, need toklipperpause
                    if self.STM32ReprotPauseFlag==1:
                        # Lo_ChangeChannelIfSuccess = False
                        # break
                        # lancaigang231205: ifwaithasstm32 move reportpause, when output, do not down pause
                        self.G_PhrozenFluiddRespondInfo('[INFO], stm32 move up report pause')

                        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
                        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
                        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
                        if Lo_PauseStatus['is_paused'] == True:
                            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
                        else:
                            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
                        Lo_ChangeChannelIfSuccess = False
                        break


                    # #lancaigang231214: waitX YW H move move, purgeFunction
                    # command_string = """
                    #     G90
                    #     G1 X%.03f Y%.03f F%d
                    #     """ % (
                    # point[0]+(num%2),#X; lancaigang231215: waitx move mm, prevent normal normal toolheadto
                    # point[1]+(num%2),#Y
                    # int(self.G_WaitAreaEachStepDist / self.G_MovementSpeedFactor),#
                    #     #500
                    # )
                    # #lancaigang231129: move move
                    # self.G_PhrozenGCode.run_script_from_command(command_string)
                    # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_P1CnAutoChangeChannel]wait, XY is P9 set ")
                    
                    
                    # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_P1CnAutoChangeChannel]wait, use use External macro command")


                    #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]num='%d'" % num)
                    # lancaigang20231014: Wait for toolhead movementto position, will when, 1
                    self.G_ProzenToolhead.wait_moves()
                    #self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_P1CnAutoChangeChannel]self.G_ProzenToolhead.wait_moves()")

                    # lancaigang231219: is dwell
                    #lancaigang231209
                    #time.sleep(2)
                    # lancaigang231115: is 1s
                    self.G_ProzenToolhead.dwell(1)

                    self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))

                    self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG110; STM32feed after, klipperstartpurgefeed')
                    command_string = """
                    PG110
                    """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)


        
                    self.G_PhrozenFluiddRespondInfo("[INFO] Current mode")
                    if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                        self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:0,unkown")
                    elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
                        self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:1,MC")
                    elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                        self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:2,MA")
                    elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                        self.G_PhrozenFluiddRespondInfo("[WARN] +Mode:3,RUNOUT")
                    else:
                        self.G_PhrozenFluiddRespondInfo("[ERROR] +Mode:-1,error")


                    Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                    self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
                    self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
                    #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
                    if Lo_PauseStatus['is_paused'] == True:
                        self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
                    else:
                        self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")



                    # lancaigang250111:forpurge output machine move, preventfeed in output


                    # lancaigang240125: cannot use sleep, will thread
                    #time.sleep(1)

                    # #lancaigang231129: if after detectedtoolheaddetectto filament, toolheadcut filamenterror, need topauseklipper
                    # if num == 3 and point[2] and self.G_ToolheadIfHaveFilaFlag:
                    # self.G_PhrozenFluiddRespondInfo("[ERROR] [(cmds.python)Cmds_P1CnAutoChangeChannel]cut filament5toolheaddetectto filament, cutter error, please check cutter, pauseklipperprinting")
                    #     Lo_ChangeChannelIfSuccess = False
                    #     break
                    # elif num > 3:
                    # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_P1CnAutoChangeChannel]cut filamentsuccessful, waitnew filament")

                    # 10 after and alloweddetect and detectto filament
                    # lancaigang20231013: 10 is 8
                    # lancaigang231129: ifcut filamentsuccessful, stm32 will keep reserve up time command move action, but up, ifdetectto toolheadhasfilament, will klippernofilamentempty
                    # lancaigang231129: is after detecttoolhead is no hasfilament, normal normal when, 5 internal cutter cut filament, stm32 machine and filament, when toolhead is no detectto filament, 30 after then detect is no hasnew filament
                    if num > 1 and self.G_ToolheadIfHaveFilaFlag:
                        self.G_PhrozenFluiddRespondInfo('[INFO] has new filament, successful, canprinting')
                        Lo_ChangeChannelIfSuccess = True
                        break



            self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))

            # if is truesuccessful, then
            if Lo_ChangeChannelIfSuccess:
                self.G_PhrozenFluiddRespondInfo('[INFO] successful;')
                self.G_PhrozenFluiddRespondInfo('[INFO] successful')
                self.G_IfChangeFilaOngoing= False

                # lancaigang250424: preventAMSbufferhasfull
                self.G_ProzenToolhead.dwell(0.5)

                # lancaigang250619:checkAMS is no re connectsuccessful
                self.Cmds_USBConnectErrorCheck()
                # lancaigang250423: feedsuccessful, purge, AMSwhen, ifpurge5buffer is state, head
                #self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
                # self.G_PhrozenFluiddRespondInfo("[INFO] AMSstart timingbuffer-full time")
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("AT+EBLOCKCHECK")
                    self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1-AMSstart timingbuffer-full time")
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
                    self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2-AMSstart timingbuffer-full time")
                self.G_ProzenToolhead.dwell(1)

                #lancaigang240229:
                if self.IfDoPG102Flag==True:
                    self.IfDoPG102Flag=False

                    self.G_PhrozenFluiddRespondInfo("[INFO] purgestart")
                    self.G_PhrozenFluiddRespondInfo("+MSG:1,0,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))


                    # lancaigang241031: purge time number
                    # lancaigang250324: default is PG113, purge3 time
                    if self.G_P10SpitNum==0:
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG113")
                        self.PG102Flag=True
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                        command_string = """
                        PG113
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)
                    elif self.G_P10SpitNum==1:
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG111")
                        self.PG102Flag=True
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                        command_string = """
                        PG111
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)
                    elif self.G_P10SpitNum==2:
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG112")
                        self.PG102Flag=True
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                        command_string = """
                        PG112
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)
                    elif self.G_P10SpitNum==3:
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG113")
                        self.PG102Flag=True
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                        command_string = """
                        PG113
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)
                    elif self.G_P10SpitNum==4:
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG114")
                        self.PG102Flag=True
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                        command_string = """
                        PG114
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)
                    #lancaigang250528: 
                    elif self.G_P10SpitNum==5:
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG115")
                        self.PG102Flag=True
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                        command_string = """
                        PG115
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)
                    #lancaigang250528: 
                    elif self.G_P10SpitNum==6:
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG116")
                        self.PG102Flag=True
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                        command_string = """
                        PG116
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)
                    elif self.G_P10SpitNum==7:
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG117")
                        self.PG102Flag=True
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                        command_string = """
                        PG117
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)
                    elif self.G_P10SpitNum==8:
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG118")
                        self.PG102Flag=True
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                        command_string = """
                        PG118
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)
                    elif self.G_P10SpitNum==9:
                        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG119")
                        self.PG102Flag=True
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
                        command_string = """
                        PG119
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("External macro command-; command_string='%s'" % command_string)

                    self.PG102Flag=False
                    self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=False")
                    
                    self.G_PhrozenFluiddRespondInfo("[INFO] purgefinish")
   
                    # for i in range(15):
                    # self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)]purge, wait")
                    # #lancaigang20231013: is 4when
                    # #lancaigang231115: is 1s
                    #     self.G_ProzenToolhead.dwell(1.0)
                    # #lancaigang240125: cannot use sleep, will thread
                    #     #time.sleep(1)
                    if self.PG102DelayPauseFlag==True:
                        self.PG102DelayPauseFlag=False

                        # lancaigang250619:checkAMS is no re connectsuccessful
                        self.Cmds_USBConnectErrorCheck()
                        #lancaigang250427: 
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1-AMSfinishtimingbuffer-full time")
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2-AMSfinishtimingbuffer-full time")

                        self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                        self.G_KlipperQuickPause = True
                        self.G_PhrozenFluiddRespondInfo('[INFO] purge, STM32 send filament runoutpause')
                        # lancaigang231209: timer logical, will error, after to use thread logical interrupt business logic

                        if self.G_KlipperInPausing == False:
                            self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                            #lancaigang250607:
                            self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                            self.G_KlipperQuickPause = True
                            # klipper move pause
                            self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause1 time')
                            self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                        else:
                            self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")

                        self.G_KlipperIfPaused = True
                        # stm32 move pausepause1 time, cannotpause
                        self.STM32ReprotPauseFlag=1
                        # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                        self.G_ChangeChannelFirstFilaFlag=True
                        
                        self.G_ProzenToolhead.dwell(1.5)
                        self.G_PhrozenFluiddRespondInfo("+MSG:1,1,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        # lancaigang240524: use UIUX move
                        self.G_PhrozenFluiddRespondInfo("+C:1,%d" % self.G_ChangeChannelTimeoutNewChan)
                        

                        #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                        #lancaigang250529:
                        if len(self.G_PauseToLCDString)==0:
                            self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        else:
                            self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)

                        # lancaigang250607: state; 1-unload filament; 2-feed; 3-; 4-pause
                        self.G_KlipperPrintStatus= 3
                        self.G_PauseToLCDString=""
                        
                        self.G_PhrozenFluiddRespondInfo("[INFO] return")
                        return

                        # lancaigang240326: purgepause, use use pause1 table show
                        #self.G_PhrozenFluiddRespondInfo("+PAUSE:1,%d" % self.G_ChangeChannelTimeoutNewChan)
                    else:
                        self.G_PhrozenFluiddRespondInfo('[INFO] purge normal normal, enterprinting')
                        # lancaigang250527: successful and gocde text file
                        # lancaigang250527: pause
                        self.G_KlipperQuickPause = True


                self.G_PhrozenFluiddRespondInfo("+MSG:1,1,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))

                # lancaigang250619:checkAMS is no re connectsuccessful
                self.Cmds_USBConnectErrorCheck()
                #lancaigang250427: 
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                    self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1-AMSfinishtimingbuffer-full time")
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                    self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2-AMSfinishtimingbuffer-full time")
                self.G_ProzenToolhead.dwell(1.5)



                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+C:1,%d" % self.G_ChangeChannelTimeoutNewChan)


                # lancaigang250607: state; 1-unload filament; 2-feed; 3-; 4-pause
                self.G_KlipperPrintStatus= 3

                self.G_PauseToLCDString=""

                self.G_PhrozenFluiddRespondInfo('[INFO] normal normal enterprinting')

                return
            





            self.G_PhrozenFluiddRespondInfo('[INFO] failed')
            self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))
            # failed
            if self.G_DictChangeChannelWaitAreaParam["A"] == 0:
                self.G_PhrozenFluiddRespondInfo("[INFO] failed; filamentfeedtimeout; cmd='%s', allfilament, klipperpause")
                self.G_PhrozenFluiddRespondInfo("failed; current command='%s';klipperpause" % (self.G_ChangeChannelTimeoutOldGcmd.get_commandline()))
                
                # lancaigang250527: pause
                self.G_KlipperQuickPause = False

                # lancaigang250619:checkAMS is no re connectsuccessful
                self.Cmds_USBConnectErrorCheck()

                # #lancaigang231129: klipperpausewhen, toolhead move move to z=10; x=150; y=10
                # command_string = """
                # G91
                # G1 z10 F600
                # G90
                # G1 X150 F600
                # G1 Y10 F600
                # """
                # self.G_PhrozenGCode.run_script_from_command(command_string)
                
                # lancaigang231201: klipperpausewhen stm32 machine cannot move
                # gcmd.respond_info("Sending command: AP, retract allto park position")
                # #// retract allto park position; //===== P2 A1 allfilamentto park position Yes; "AP";
                # self.Cmds_AMSSerial1Send("AP")
                # logging.info("SendCmd: AP")
                if self.G_KlipperIfPaused==False:
                    #lancaigang250702: 
                    if self.G_KlipperInPausing == False:
                        self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                        self.STM32ReprotPauseFlag=1
                        # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                        self.G_ChangeChannelFirstFilaFlag=True

                        
                        #lancaigang240416:
                        if self.G_SerialPort1OpenFlag == True:
                            # lancaigang240603: preventAMS
                            self.Cmds_AMSSerial1Send("AT+PAUSE")
                            self.G_PhrozenFluiddRespondInfo('[WARN] serial port 1-AT+PAUSEpausestm32 machine')
                        #lancaigang241030:
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+PAUSE")
                            self.G_PhrozenFluiddRespondInfo('[WARN] serial port 2-AT+PAUSEpausestm32 machine')

                        self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                        self.G_KlipperQuickPause = True
                        # klipper move pause
                        self.Cmds_PhrozenKlipperPause(None)
                        self.G_KlipperIfPaused = True
                        #self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d" % self.G_ChangeChannelTimeoutNewChan)

                        self.G_PhrozenFluiddRespondInfo('[INFO] timeout60s, pause')
                        #self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        #self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        #lancaigang250529:
                        if len(self.G_PauseToLCDString)==0:
                            self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        else:
                            self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)

                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")
                # lancaigang240124: cannotpause
                else:
                    self.G_PhrozenFluiddRespondInfo('[INFO] already pause, do notpause')
                    # #lancaigang250529:
                    # if len(self.G_PauseToLCDString)==0:
                    #     self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    # else:
                    #     self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)

                    #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                    # lancaigang240417: preventstm32reportwhen G_PauseToLCDString is empty
                    #if len(self.G_PauseToLCDString)==0:
                    #    self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    #lancaigang250529:
                    if len(self.G_PauseToLCDString)==0:
                        self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    else:
                        self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                    #lancaigang240416:
                    if self.G_SerialPort1OpenFlag == True:
                        # lancaigang240603: preventAMS
                        self.Cmds_AMSSerial1Send("AT+PAUSE")
                        self.G_PhrozenFluiddRespondInfo('[WARN] serial port 1-AT+PAUSEpausestm32 machine')
                    #lancaigang241030:
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AT+PAUSE")
                        self.G_PhrozenFluiddRespondInfo('[WARN] serial port 2-AT+PAUSEpausestm32 machine')
                    # lancaigang240429: ifDuring resumestm32 and noreportpausestate, need toreportpause
                    # if self.G_ResumeProcessCheckPauseStatus==False:
                    #     self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d" % self.G_ChangeChannelTimeoutNewChan)

                    if self.G_ResumeProcessCheckPauseStatus==False:
                        self.G_PhrozenFluiddRespondInfo('[INFO] AMShas up report pause, klipperpause, need to up report pause')
                        #self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        #lancaigang250529:
                        if len(self.G_PauseToLCDString)==0:
                            self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        else:
                            self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                        
                        #lancaigang240416:
                        if self.G_SerialPort1OpenFlag == True:
                            # lancaigang240603: preventAMS
                            self.Cmds_AMSSerial1Send("AT+PAUSE")
                            self.G_PhrozenFluiddRespondInfo('[WARN] serial port 1-AT+PAUSEpausestm32 machine')
                        #lancaigang241030:
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+PAUSE")
                            self.G_PhrozenFluiddRespondInfo('[WARN] serial port 2-AT+PAUSEpausestm32 machine')
                    else:
                        self.G_PhrozenFluiddRespondInfo('[INFO] AMShas up report pause, klipper need to need to up report pause')
                        self.G_ResumeProcessCheckPauseStatus=False

                    # self.G_PhrozenFluiddRespondInfo("[ERROR] already pause, then pause1 time, preventbeforepauseerror")
                    # #lancaigang250423: is preventerrorpause, pause1 time
                    # #klipper move pause
                    # self.Cmds_PhrozenKlipperPause(None)

                # lancaigang231207: P1 C?Automatic filament changewhen, if to resume, from #1 time channel
                self.G_ChangeChannelFirstFilaFlag=True
                self.G_IfChangeFilaOngoing= False

                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+C:1,%d" % self.G_ChangeChannelTimeoutNewChan)

                # lancaigang250607: state; 1-unload filament; 2-feed; 3-; 4-pause
                self.G_KlipperPrintStatus= -1

                return
            
            # lancaigang20231013: Action normal normal =1
            if self.G_DictChangeChannelWaitAreaParam["A"] == 1:
                pass



    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_CmdOrcaPre(self):
        self.G_PhrozenFluiddRespondInfo('=====[(cmds.python)Cmds_CmdOrcaPre]orca before set move action' )

        # lancaigang250912: toolhead; first to, after send GPIOtoolhead normal switch, toolhead normal switch to has and machine

        # lancaigang250515: machine multi-material, logical T?
        if self.G_P0M1MCNoneAMS == 1:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdOrcaPre]standalonemulti-material, logical T?')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            #self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)

            return
        
        #lancaigang250429: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdOrcaPre]single-colormode, logical T?')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            #self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        
        #lancaigang250514: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_MA :
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdOrcaPre]single-color refill modemode, logical T?')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            #self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return

        # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
            self.G_PhrozenFluiddRespondInfo('[WARN] ifreceived T?command, but mode is Unknown mode, switch is MCmulti-materialmode')
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            #lancaigang241030:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")
        
            self.G_ProzenToolhead.dwell(2)



    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_CmdT0(self,gcmd):
        self.G_PhrozenFluiddRespondInfo('=====[(cmds.python)Cmds_CmdT0 +1]orcaAMS' )

        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)

        # lancaigang250515: machine multi-material, logical T?
        if self.G_P0M1MCNoneAMS == 1:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT1]standalonemulti-material, logical T?')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250429: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT1]single-colormode, logical T0')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250514: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_MA :
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT1]single-color refill modemode, logical T0')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return

        # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
            self.G_PhrozenFluiddRespondInfo('[WARN] ifreceived T?command, but mode is Unknown mode, switch is MCmulti-materialmode')
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            #lancaigang241030:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")

            self.G_ProzenToolhead.dwell(2)


        #lancaigang250912:
        #self.Cmds_CmdOrcaPre()

        
        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.G_PhrozenFluiddRespondInfo("[INFO] Automatic filament change")
        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T0:0,%d" % self.G_ChangeChannelTimeoutNewChan)

        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        #lancaigang250517: 
        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
        # lancaigang240113: manualcommand
        self.ManualCmdFlag=False
        # lancaigang240221: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0
        self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")

        # lancaigang241030: is P1 C1to P1 C32, in 1to 32
        # unit 1: 1 2 3 4
        # unit 2: 5 6 7 8
        # #3 unit : 9 10 11 12
        # #4 unit : 13 14 15 16
        # #5 unit : 17 18 19 20
        # #6 unit : 21 22 23 24
        # #7 unit : 25 26 27 28
        # #8 unit : 29 30 31 32
        #Automatic filament change
        chan=0+1
        self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)

        # lancaigang250515:serial port disable set color to channel
        if self.G_ChromaKitAccessT0 > 0:
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT0=%d" % self.G_ChromaKitAccessT0)
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT0, gcmd)
        else:
            self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T0:1,%d" % self.G_ChangeChannelTimeoutNewChan)


    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_CmdT1(self,gcmd):
        self.G_PhrozenFluiddRespondInfo('=====[(cmds.python)Cmds_CmdT1 +1]orcaAMS' )

        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)

        # lancaigang250515: machine multi-material, logical T?
        if self.G_P0M1MCNoneAMS == 1:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT1]standalonemulti-material, logical T?')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250429: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT1]single-colormode, logical T1')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250514: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_MA :
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT1]single-color refill modemode, logical T1')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return

        # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
            self.G_PhrozenFluiddRespondInfo('[WARN] ifreceived T?command, but mode is Unknown mode, switch is MCmulti-materialmode')
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            #lancaigang241030:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")

            self.G_ProzenToolhead.dwell(2)
        
        # lancaigang250912:returnerror
        #self.Cmds_CmdOrcaPre()



        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.G_PhrozenFluiddRespondInfo("[INFO] Automatic filament change")
        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T1:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        #lancaigang250517: 
        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
        # lancaigang240113: manualcommand
        self.ManualCmdFlag=False
        # lancaigang240221: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0
        self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")

        # lancaigang241030: is P1 C1to P1 C32, in 1to 32
        # unit 1: 1 2 3 4
        # unit 2: 5 6 7 8
        # #3 unit : 9 10 11 12
        # #4 unit : 13 14 15 16
        # #5 unit : 17 18 19 20
        # #6 unit : 21 22 23 24
        # #7 unit : 25 26 27 28
        # #8 unit : 29 30 31 32
        #Automatic filament change
        chan=1+1
        self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)

        # lancaigang250515:serial port disable set color to channel
        if self.G_ChromaKitAccessT1 > 0:
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT1=%d" % self.G_ChromaKitAccessT1)
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT1, gcmd)
        else:
            self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T1:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_CmdT2(self,gcmd):
        self.G_PhrozenFluiddRespondInfo('=====[(cmds.python)Cmds_CmdT2 +1]orcaAMS' )

        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)

        # lancaigang250515: machine multi-material, logical T?
        if self.G_P0M1MCNoneAMS == 1:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT2]standalonemulti-material, logical T?')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250429: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT2]single-colormode, logical T2')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250514: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_MA :
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT2]single-color refill modemode, logical T2')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return

        # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
            self.G_PhrozenFluiddRespondInfo('[WARN] ifreceived T?command, but mode is Unknown mode, switch is MCmulti-materialmode')
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            #lancaigang241030:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")

            self.G_ProzenToolhead.dwell(2)

        # lancaigang250912:returnerror
        #self.Cmds_CmdOrcaPre()

        
        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.G_PhrozenFluiddRespondInfo("[INFO] Automatic filament change")
        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T2:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        #lancaigang250517: 
        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
        # lancaigang240113: manualcommand
        self.ManualCmdFlag=False
        # lancaigang240221: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0
        self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")

        # lancaigang241030: is P1 C1to P1 C32, in 1to 32
        # unit 1: 1 2 3 4
        # unit 2: 5 6 7 8
        # #3 unit : 9 10 11 12
        # #4 unit : 13 14 15 16
        # #5 unit : 17 18 19 20
        # #6 unit : 21 22 23 24
        # #7 unit : 25 26 27 28
        # #8 unit : 29 30 31 32
        #Automatic filament change
        chan=2+1
        self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
        # lancaigang250515:serial port disable set color to channel
        if self.G_ChromaKitAccessT2 > 0:
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT2=%d" % self.G_ChromaKitAccessT2)
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT2, gcmd)
        else:
            self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T2:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_CmdT3(self,gcmd):
        self.G_PhrozenFluiddRespondInfo('=====[(cmds.python)Cmds_CmdT3 +1]orcaAMS' )

        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)


        # lancaigang250515: machine multi-material, logical T?
        if self.G_P0M1MCNoneAMS == 1:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT3]standalonemulti-material, logical T?')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250429: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT3]single-colormode, logical T3')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250514: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_MA :
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT3]single-color refill modemode, logical T3')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return

        # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
            self.G_PhrozenFluiddRespondInfo('[WARN] ifreceived T?command, but mode is Unknown mode, switch is MCmulti-materialmode')
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            #lancaigang241030:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")

            self.G_ProzenToolhead.dwell(2)

        #lancaigang250912:
        #self.Cmds_CmdOrcaPre()

        
        
        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.G_PhrozenFluiddRespondInfo("[INFO] Automatic filament change")
        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T3:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        #lancaigang250517: 
        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
        # lancaigang240113: manualcommand
        self.ManualCmdFlag=False
        # lancaigang240221: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0
        self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")

        # lancaigang241030: is P1 C1to P1 C32, in 1to 32
        # unit 1: 1 2 3 4
        # unit 2: 5 6 7 8
        # #3 unit : 9 10 11 12
        # #4 unit : 13 14 15 16
        # #5 unit : 17 18 19 20
        # #6 unit : 21 22 23 24
        # #7 unit : 25 26 27 28
        # #8 unit : 29 30 31 32
        #Automatic filament change
        chan=3+1
        self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
        # lancaigang250515:serial port disable set color to channel
        if self.G_ChromaKitAccessT3 > 0:
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT3=%d" % self.G_ChromaKitAccessT3)
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT3, gcmd)
        else:
            self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T3:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_CmdT4(self,gcmd):
        self.G_PhrozenFluiddRespondInfo('=====[(cmds.python)Cmds_CmdT4 +1]orcaAMS' )

        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)

        # lancaigang250515: machine multi-material, logical T?
        if self.G_P0M1MCNoneAMS == 1:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT4]standalonemulti-material, logical T?')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250429: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT4]single-colormode, logical T4')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250514: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_MA :
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT4]single-color refill modemode, logical T4')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return

        # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
            self.G_PhrozenFluiddRespondInfo('[WARN] ifreceived T?command, but mode is Unknown mode, switch is MCmulti-materialmode')
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            #lancaigang241030:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")

            self.G_ProzenToolhead.dwell(2)

        # lancaigang250912:returnerror
        #self.Cmds_CmdOrcaPre()

        
        
        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.G_PhrozenFluiddRespondInfo("[INFO] Automatic filament change")
        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T4:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        #lancaigang250517: 
        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
        # lancaigang240113: manualcommand
        self.ManualCmdFlag=False
        # lancaigang240221: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0
        self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")

        # lancaigang241224: no # AMS, do not execute;
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #2 AMS, command')
        else:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #2 AMS, do not executecommand')
            self.G_PhrozenFluiddRespondInfo("+T4:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return

        # lancaigang241030: is P1 C1to P1 C32, in 1to 32
        # unit 1: 1 2 3 4
        # unit 2: 5 6 7 8
        # #3 unit : 9 10 11 12
        # #4 unit : 13 14 15 16
        # #5 unit : 17 18 19 20
        # #6 unit : 21 22 23 24
        # #7 unit : 25 26 27 28
        # #8 unit : 29 30 31 32
        #Automatic filament change
        chan=4+1
        self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
        # lancaigang250515:serial port disable set color to channel
        if self.G_ChromaKitAccessT4 > 0:
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT4=%d" % self.G_ChromaKitAccessT4)
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT4, gcmd)
        else:
            self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T4:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_CmdT5(self,gcmd):
        self.G_PhrozenFluiddRespondInfo('=====[(cmds.python)Cmds_CmdT5 +1]orcaAMS' )

        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)

        # lancaigang250515: machine multi-material, logical T?
        if self.G_P0M1MCNoneAMS == 1:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT5]standalonemulti-material, logical T?')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250429: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT5]single-colormode, logical T5')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250514: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_MA :
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT5]single-color refill modemode, logical T5')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return

        # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
            self.G_PhrozenFluiddRespondInfo('[WARN] ifreceived T?command, but mode is Unknown mode, switch is MCmulti-materialmode')
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            #lancaigang241030:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")

            self.G_ProzenToolhead.dwell(2)

        # lancaigang250912:returnerror
        #self.Cmds_CmdOrcaPre()

        

        
        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.G_PhrozenFluiddRespondInfo("[INFO] Automatic filament change")
        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T5:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        #lancaigang250517: 
        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
        # lancaigang240113: manualcommand
        self.ManualCmdFlag=False
        # lancaigang240221: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0
        self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")

        # lancaigang241224: no # AMS, do not execute;
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #2 AMS, command')
        else:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #2 AMS, do not executecommand')
            self.G_PhrozenFluiddRespondInfo("+T5:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return


        # lancaigang241030: is P1 C1to P1 C32, in 1to 32
        # unit 1: 1 2 3 4
        # unit 2: 5 6 7 8
        # #3 unit : 9 10 11 12
        # #4 unit : 13 14 15 16
        # #5 unit : 17 18 19 20
        # #6 unit : 21 22 23 24
        # #7 unit : 25 26 27 28
        # #8 unit : 29 30 31 32
        #Automatic filament change
        chan=5+1
        self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
        # lancaigang250515:serial port disable set color to channel
        if self.G_ChromaKitAccessT5 > 0:
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT5=%d" % self.G_ChromaKitAccessT5)
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT5, gcmd)
        else:
            self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T5:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_CmdT6(self,gcmd):
        self.G_PhrozenFluiddRespondInfo('=====[(cmds.python)Cmds_CmdT6 +1]orcaAMS' )

        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)

        # lancaigang250515: machine multi-material, logical T?
        if self.G_P0M1MCNoneAMS == 1:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT6]standalonemulti-material, logical T?')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250429: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT6]single-colormode, logical T6')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250514: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_MA :
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT6]single-color refill modemode, logical T6')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return

        # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
            self.G_PhrozenFluiddRespondInfo('[WARN] ifreceived T?command, but mode is Unknown mode, switch is MCmulti-materialmode')
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            #lancaigang241030:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")

            self.G_ProzenToolhead.dwell(2)

        # lancaigang250912:returnerror
        #self.Cmds_CmdOrcaPre()

        

        
        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.G_PhrozenFluiddRespondInfo("[INFO] Automatic filament change")
        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T6:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        #lancaigang250517: 
        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
        # lancaigang240113: manualcommand
        self.ManualCmdFlag=False
        # lancaigang240221: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0
        self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")


        # lancaigang241224: no # AMS, do not execute;
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #2 AMS, command')
        else:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #2 AMS, do not executecommand')
            self.G_PhrozenFluiddRespondInfo("+T6:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return



        # lancaigang241030: is P1 C1to P1 C32, in 1to 32
        # unit 1: 1 2 3 4
        # unit 2: 5 6 7 8
        # #3 unit : 9 10 11 12
        # #4 unit : 13 14 15 16
        # #5 unit : 17 18 19 20
        # #6 unit : 21 22 23 24
        # #7 unit : 25 26 27 28
        # #8 unit : 29 30 31 32
        #Automatic filament change
        chan=6+1
        self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
        # lancaigang250515:serial port disable set color to channel
        if self.G_ChromaKitAccessT6 > 0:
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT6=%d" % self.G_ChromaKitAccessT6)
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT6, gcmd)
        else:
            self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T6:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_CmdT7(self,gcmd):
        self.G_PhrozenFluiddRespondInfo('=====[(cmds.python)Cmds_CmdT7 +1]orcaAMS' )

        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)

        # lancaigang250515: machine multi-material, logical T?
        if self.G_P0M1MCNoneAMS == 1:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT7]standalonemulti-material, logical T?')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250429: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT7]single-colormode, logical T7')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250514: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_MA :
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT7]single-color refill modemode, logical T7')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return

        # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
            self.G_PhrozenFluiddRespondInfo('[WARN] ifreceived T?command, but mode is Unknown mode, switch is MCmulti-materialmode')
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            #lancaigang241030:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")

            self.G_ProzenToolhead.dwell(2)

        # lancaigang250912:returnerror
        #self.Cmds_CmdOrcaPre()

        

        
        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.G_PhrozenFluiddRespondInfo("[INFO] Automatic filament change")
        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T7:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        #lancaigang250517: 
        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
        # lancaigang240113: manualcommand
        self.ManualCmdFlag=False
        # lancaigang240221: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0
        self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")


        # lancaigang241224: no # AMS, do not execute;
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #2 AMS, command')
        else:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #2 AMS, do not executecommand')
            self.G_PhrozenFluiddRespondInfo("+T7:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return

        # lancaigang241030: is P1 C1to P1 C32, in 1to 32
        # unit 1: 1 2 3 4
        # unit 2: 5 6 7 8
        # #3 unit : 9 10 11 12
        # #4 unit : 13 14 15 16
        # #5 unit : 17 18 19 20
        # #6 unit : 21 22 23 24
        # #7 unit : 25 26 27 28
        # #8 unit : 29 30 31 32
        #Automatic filament change
        chan=7+1
        self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
        # lancaigang250515:serial port disable set color to channel
        if self.G_ChromaKitAccessT7 > 0:
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT7=%d" % self.G_ChromaKitAccessT7)
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT7, gcmd)
        else:
            self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T7:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_CmdT8(self,gcmd):
        self.G_PhrozenFluiddRespondInfo('=====[(cmds.python)Cmds_CmdT8 +1]orcaAMS' )

        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)

        # lancaigang250515: machine multi-material, logical T?
        if self.G_P0M1MCNoneAMS == 1:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT8]standalonemulti-material, logical T?')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250429: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT8]single-colormode, logical T8')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250514: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_MA :
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT8]single-color refill modemode, logical T8')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        
        # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
            self.G_PhrozenFluiddRespondInfo('[WARN] ifreceived T?command, but mode is Unknown mode, switch is MCmulti-materialmode')
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            #lancaigang241030:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")

            self.G_ProzenToolhead.dwell(2)

        # lancaigang250912:returnerror
        #self.Cmds_CmdOrcaPre()

        

        
        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.G_PhrozenFluiddRespondInfo("[INFO] Automatic filament change")
        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T8:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        #lancaigang250517: 
        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
        # lancaigang240113: manualcommand
        self.ManualCmdFlag=False
        # lancaigang240221: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0
        self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")


        # lancaigang241224: no #3 AMS, do not execute;
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #3 AMS, command')
        else:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #3 AMS, do not executecommand')
            self.G_PhrozenFluiddRespondInfo("+T8:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return

        # lancaigang241030: is P1 C1to P1 C32, in 1to 32
        # unit 1: 1 2 3 4
        # unit 2: 5 6 7 8
        # #3 unit : 9 10 11 12
        # #4 unit : 13 14 15 16
        # #5 unit : 17 18 19 20
        # #6 unit : 21 22 23 24
        # #7 unit : 25 26 27 28
        # #8 unit : 29 30 31 32
        #Automatic filament change
        chan=8+1
        self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
        # lancaigang250515:serial port disable set color to channel
        if self.G_ChromaKitAccessT8 > 0:
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT8=%d" % self.G_ChromaKitAccessT8)
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT8, gcmd)
        else:
            self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T8:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_CmdT9(self,gcmd):
        self.G_PhrozenFluiddRespondInfo('=====[(cmds.python)Cmds_CmdT9 +1]orcaAMS' )

        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)

        # lancaigang250515: machine multi-material, logical T?
        if self.G_P0M1MCNoneAMS == 1:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT9]standalonemulti-material, logical T?')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250429: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT9]single-colormode, logical T9')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250514: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_MA :
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT9]single-color refill modemode, logical T9')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        
        # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
            self.G_PhrozenFluiddRespondInfo('[WARN] ifreceived T?command, but mode is Unknown mode, switch is MCmulti-materialmode')
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            #lancaigang241030:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")

            self.G_ProzenToolhead.dwell(2)


        # #lancaigang250912:
        # self.Cmds_CmdOrcaPre()

        



        
        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.G_PhrozenFluiddRespondInfo("[INFO] Automatic filament change")
        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T9:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        #lancaigang250517: 
        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
        # lancaigang240113: manualcommand
        self.ManualCmdFlag=False
        # lancaigang240221: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0
        self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")


        # lancaigang241224: no #3 AMS, do not execute;
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #3 AMS, command')
        else:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #3 AMS, do not executecommand')
            self.G_PhrozenFluiddRespondInfo("+T9:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return


        # lancaigang241030: is P1 C1to P1 C32, in 1to 32
        # unit 1: 1 2 3 4
        # unit 2: 5 6 7 8
        # #3 unit : 9 10 11 12
        # #4 unit : 13 14 15 16
        # #5 unit : 17 18 19 20
        # #6 unit : 21 22 23 24
        # #7 unit : 25 26 27 28
        # #8 unit : 29 30 31 32
        #Automatic filament change
        chan=9+1
        self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
        # lancaigang250515:serial port disable set color to channel
        if self.G_ChromaKitAccessT9 > 0:
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT9=%d" % self.G_ChromaKitAccessT9)
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT9, gcmd)
        else:
            self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T9:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_CmdT10(self,gcmd):
        self.G_PhrozenFluiddRespondInfo('=====[(cmds.python)Cmds_CmdT10 +1]orcaAMS' )

        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)

        # lancaigang250515: machine multi-material, logical T?
        if self.G_P0M1MCNoneAMS == 1:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT10]standalonemulti-material, logical T?')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250429: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT10]single-colormode, logical T10')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250514: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_MA :
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT10]single-color refill modemode, logical T10')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        
        # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
            self.G_PhrozenFluiddRespondInfo('[WARN] ifreceived T?command, but mode is Unknown mode, switch is MCmulti-materialmode')
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            #lancaigang241030:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")

            self.G_ProzenToolhead.dwell(2)

        # lancaigang250912:returnerror
        #self.Cmds_CmdOrcaPre()

        

        
        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.G_PhrozenFluiddRespondInfo("[INFO] Automatic filament change")
        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T10:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        #lancaigang250517: 
        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
        # lancaigang240113: manualcommand
        self.ManualCmdFlag=False
        # lancaigang240221: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0
        self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")

        # lancaigang241224: no #3 AMS, do not execute;
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #3 AMS, command')
        else:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #3 AMS, do not executecommand')
            self.G_PhrozenFluiddRespondInfo("+T10:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return

        # lancaigang241030: is P1 C1to P1 C32, in 1to 32
        # unit 1: 1 2 3 4
        # unit 2: 5 6 7 8
        # #3 unit : 9 10 11 12
        # #4 unit : 13 14 15 16
        # #5 unit : 17 18 19 20
        # #6 unit : 21 22 23 24
        # #7 unit : 25 26 27 28
        # #8 unit : 29 30 31 32
        #Automatic filament change
        chan=10+1
        self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
        # lancaigang250515:serial port disable set color to channel
        if self.G_ChromaKitAccessT10 > 0:
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT10=%d" % self.G_ChromaKitAccessT10)
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT10, gcmd)
        else:
            self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T10:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_CmdT11(self,gcmd):
        self.G_PhrozenFluiddRespondInfo('=====[(cmds.python)Cmds_CmdT11 +1]orcaAMS' )

        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)

        # lancaigang250515: machine multi-material, logical T?
        if self.G_P0M1MCNoneAMS == 1:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT11]standalonemulti-material, logical T?')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250429: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT11]single-colormode, logical T11')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250514: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_MA :
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT11]single-color refill modemode, logical T11')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        
        # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
            self.G_PhrozenFluiddRespondInfo('[WARN] ifreceived T?command, but mode is Unknown mode, switch is MCmulti-materialmode')
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            #lancaigang241030:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")

            self.G_ProzenToolhead.dwell(2)

        # lancaigang250912:returnerror
        #self.Cmds_CmdOrcaPre()

        

        
        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.G_PhrozenFluiddRespondInfo("[INFO] Automatic filament change")
        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T11:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        #lancaigang250517: 
        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
        # lancaigang240113: manualcommand
        self.ManualCmdFlag=False
        # lancaigang240221: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0
        self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")

        # lancaigang241224: no #3 AMS, do not execute;
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #3 AMS, command')
        else:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #3 AMS, do not executecommand')
            self.G_PhrozenFluiddRespondInfo("+T11:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return


        # lancaigang241030: is P1 C1to P1 C32, in 1to 32
        # unit 1: 1 2 3 4
        # unit 2: 5 6 7 8
        # #3 unit : 9 10 11 12
        # #4 unit : 13 14 15 16
        # #5 unit : 17 18 19 20
        # #6 unit : 21 22 23 24
        # #7 unit : 25 26 27 28
        # #8 unit : 29 30 31 32
        #Automatic filament change
        chan=11+1
        self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
        # lancaigang250515:serial port disable set color to channel
        if self.G_ChromaKitAccessT11 > 0:
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT11=%d" % self.G_ChromaKitAccessT11)
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT11, gcmd)
        else:
            self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T11:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_CmdT12(self,gcmd):
        self.G_PhrozenFluiddRespondInfo('=====[(cmds.python)Cmds_CmdT12 +1]orcaAMS' )

        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)

        # lancaigang250515: machine multi-material, logical T?
        if self.G_P0M1MCNoneAMS == 1:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT12]standalonemulti-material, logical T?')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250429: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT12]single-colormode, logical T12')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250514: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_MA :
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT12]single-color refill modemode, logical T12')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        
        # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
            self.G_PhrozenFluiddRespondInfo('[WARN] ifreceived T?command, but mode is Unknown mode, switch is MCmulti-materialmode')
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            #lancaigang241030:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")

            self.G_ProzenToolhead.dwell(2)

        # lancaigang250912:returnerror
        #self.Cmds_CmdOrcaPre()

        

        
        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.G_PhrozenFluiddRespondInfo("[INFO] Automatic filament change")
        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T12:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        #lancaigang250517: 
        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
        # lancaigang240113: manualcommand
        self.ManualCmdFlag=False
        # lancaigang240221: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0
        self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")

        # lancaigang241224: no #4 AMS, do not execute;
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #4 AMS, command')
        else:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #4 AMS, do not executecommand')
            self.G_PhrozenFluiddRespondInfo("+T12:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return


        # lancaigang241030: is P1 C1to P1 C32, in 1to 32
        # unit 1: 1 2 3 4
        # unit 2: 5 6 7 8
        # #3 unit : 9 10 11 12
        # #4 unit : 13 14 15 16
        # #5 unit : 17 18 19 20
        # #6 unit : 21 22 23 24
        # #7 unit : 25 26 27 28
        # #8 unit : 29 30 31 32
        #Automatic filament change
        chan=12+1
        self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
        # lancaigang250515:serial port disable set color to channel
        if self.G_ChromaKitAccessT12 > 0:
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT12=%d" % self.G_ChromaKitAccessT12)
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT12, gcmd)
        else:
            self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T12:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_CmdT13(self,gcmd):
        self.G_PhrozenFluiddRespondInfo('=====[(cmds.python)Cmds_CmdT13 +1]orcaAMS' )

        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)

        # lancaigang250515: machine multi-material, logical T?
        if self.G_P0M1MCNoneAMS == 1:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT13]standalonemulti-material, logical T?')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250429: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT13]single-colormode, logical T13')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250514: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_MA :
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT13]single-color refill modemode, logical T13')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        
        # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
            self.G_PhrozenFluiddRespondInfo('[WARN] ifreceived T?command, but mode is Unknown mode, switch is MCmulti-materialmode')
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            #lancaigang241030:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")

            self.G_ProzenToolhead.dwell(2)

        # lancaigang250912:returnerror
        #self.Cmds_CmdOrcaPre()

        

        
        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.G_PhrozenFluiddRespondInfo("[INFO] Automatic filament change")
        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T13:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        #lancaigang250517: 
        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
        # lancaigang240113: manualcommand
        self.ManualCmdFlag=False
        # lancaigang240221: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0
        self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")

        # lancaigang241224: no #4 AMS, do not execute;
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #4 AMS, command')
        else:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #4 AMS, do not executecommand')
            self.G_PhrozenFluiddRespondInfo("+T13:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return


        # lancaigang241030: is P1 C1to P1 C32, in 1to 32
        # unit 1: 1 2 3 4
        # unit 2: 5 6 7 8
        # #3 unit : 9 10 11 12
        # #4 unit : 13 14 15 16
        # #5 unit : 17 18 19 20
        # #6 unit : 21 22 23 24
        # #7 unit : 25 26 27 28
        # #8 unit : 29 30 31 32
        #Automatic filament change
        chan=13+1
        self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
        # lancaigang250515:serial port disable set color to channel
        if self.G_ChromaKitAccessT13 > 0:
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT13=%d" % self.G_ChromaKitAccessT13)
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT13, gcmd)
        else:
            self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T13:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_CmdT14(self,gcmd):
        self.G_PhrozenFluiddRespondInfo('=====[(cmds.python)Cmds_CmdT14 +1]orcaAMS' )

        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)

        # lancaigang250515: machine multi-material, logical T?
        if self.G_P0M1MCNoneAMS == 1:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT14]standalonemulti-material, logical T?')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250429: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT14]single-colormode, logical T14')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250514: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_MA :
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT14]single-color refill modemode, logical T14')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        
        # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
            self.G_PhrozenFluiddRespondInfo('[WARN] ifreceived T?command, but mode is Unknown mode, switch is MCmulti-materialmode')
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            #lancaigang241030:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")

            self.G_ProzenToolhead.dwell(2)

        # lancaigang250912:returnerror
        #self.Cmds_CmdOrcaPre()

        

        
        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.G_PhrozenFluiddRespondInfo("[INFO] Automatic filament change")
        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T14:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        #lancaigang250517: 
        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
        # lancaigang240113: manualcommand
        self.ManualCmdFlag=False
        # lancaigang240221: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0
        self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")

        # lancaigang241224: no #4 AMS, do not execute;
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #4 AMS, command')
        else:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #4 AMS, do not executecommand')
            self.G_PhrozenFluiddRespondInfo("+T14:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return


        # lancaigang241030: is P1 C1to P1 C32, in 1to 32
        # unit 1: 1 2 3 4
        # unit 2: 5 6 7 8
        # #3 unit : 9 10 11 12
        # #4 unit : 13 14 15 16
        # #5 unit : 17 18 19 20
        # #6 unit : 21 22 23 24
        # #7 unit : 25 26 27 28
        # #8 unit : 29 30 31 32
        #Automatic filament change
        chan=14+1
        self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
        # lancaigang250515:serial port disable set color to channel
        if self.G_ChromaKitAccessT14 > 0:
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT14=%d" % self.G_ChromaKitAccessT14)
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT14, gcmd)
        else:
            self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T14:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_CmdT15(self,gcmd):
        self.G_PhrozenFluiddRespondInfo('=====[(cmds.python)Cmds_CmdT15 +1]orcaAMS' )

        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)

        # lancaigang250515: machine multi-material, logical T?
        if self.G_P0M1MCNoneAMS == 1:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT15]standalonemulti-material, logical T?')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250429: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT15]single-colormode, logical T15')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        #lancaigang250514: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_MA :
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT15]single-color refill modemode, logical T15')
            #lancaigang250828:
            self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            command_string = """
                PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)
            return
        
        # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
            self.G_PhrozenFluiddRespondInfo('[WARN] ifreceived T?command, but mode is Unknown mode, switch is MCmulti-materialmode')
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            #lancaigang241030:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")

            self.G_ProzenToolhead.dwell(2)

        #lancaigang250912:
        #self.Cmds_CmdOrcaPre()

        

        
        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.G_PhrozenFluiddRespondInfo("[INFO] Automatic filament change")
        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T15:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        #lancaigang250517: 
        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
        # lancaigang240113: manualcommand
        self.ManualCmdFlag=False
        # lancaigang240221: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0
        self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")

        # lancaigang241224: no #4 AMS, do not execute;
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #4 AMS, command')
        else:
            self.G_PhrozenFluiddRespondInfo('[INFO] has #4 AMS, do not executecommand')
            self.G_PhrozenFluiddRespondInfo("+T15:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return

        # lancaigang241030: is P1 C1to P1 C32, in 1to 32
        # unit 1: 1 2 3 4
        # unit 2: 5 6 7 8
        # #3 unit : 9 10 11 12
        # #4 unit : 13 14 15 16
        # #5 unit : 17 18 19 20
        # #6 unit : 21 22 23 24
        # #7 unit : 25 26 27 28
        # #8 unit : 29 30 31 32
        #Automatic filament change
        chan=15+1
        self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
        # lancaigang250515:serial port disable set color to channel
        if self.G_ChromaKitAccessT15 > 0:
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT15=%d" % self.G_ChromaKitAccessT15)
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT15, gcmd)
        else:
            self.G_PhrozenFluiddRespondInfo("chan=%d" % chan)
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+T15:1,%d" % self.G_ChangeChannelTimeoutNewChan)




    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # PRZ_VERSION version
    def Cmds_PhrozenVersion(self, gcmd):
        # ASCIIhas128 (up table), code ()from 0to 127(is from 0000 0000to 0111 1111,
        # hex is from 0x00to 0x7F), is 0。 :
        # 0~31: show or use, 0x07(BEL) will machine send output 、0x00(NULempty, is empty)
        # normal use show 、0x0D(CR) and 0x0A(LF) use show machine head to first () and move to down ();
        # : set use or or use is “”, up has, up “” table show is move action or is,
        # because therefore show 。
        # 32: show but empty;
        # 33~126: show, 48~57 is 0-9 number, 65~90 is 26 write text, 97~122 is 26 write text,
        # is 、;
        # 127: show DEL。


        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenVersion]command='%s'" % (gcmd.get_commandline(),))


        self.G_PhrozenFluiddRespondInfo("[INFO] Current mode")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:0,unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:1,MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:2,MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo("[WARN] +Mode:3,RUNOUT")
        else:
            self.G_PhrozenFluiddRespondInfo("[ERROR] +Mode:-1,error")



        self.G_PhrozenFluiddRespondInfo("%s" % (gcmd.get_commandline(),))
        # #lancaigang240224:
        # command = """
        # PAUSE
        # """
        # self.G_PhrozenGCode.run_script_from_command(command)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.Cmds_PhrozenVersion)]calling command:command=%s" % (command))
        # self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.Cmds_PhrozenVersion)]preventpause, add command; send_pause_command")
        # self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()
        #lancaigang241031:
        if self.G_SerialPort1OpenFlag == True:
            # lancaigang240524: readAMSmainboardversion、16HUBmainboardversion
            self.Cmds_AMSSerial1Send("AT+SB=0")
            self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: AT+SB=0; get AMSmainboard、16HUBmainboard')
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.Cmds_AMSSerial2Send("AT+SB=0")
            self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2Sending command: AT+SB=0; get AMSmainboard、16HUBmainboard')

        # lancaigang240529: phrozen file version
        self.G_PhrozenFluiddRespondInfo("V-H%s-I%s-F%s" % (HW_VERSION,IMAGE_VERSION,FW_VERSION))


        #emb_filename = "/home/prz/hdlDat/DriveCodeJson.dat"
        #json_data = json.load(emb_filename)
        # self.G_PhrozenFluiddRespondInfo("json_data=%s" % json_data)
        # DriveCode = json_data['DriveCode']
        # DriveImageType = json_data['DriveImageType']
        # DriveHwVersion = json_data['DriveHwVersion']
        # DriveFwVersion = json_data['DriveFwVersion']
        # DriveId = json_data['DriveId']
        # self.G_PhrozenFluiddRespondInfo("DriveCode=%s" % DriveCode)
        # self.G_PhrozenFluiddRespondInfo("DriveImageType=%s" % DriveImageType)
        # self.G_PhrozenFluiddRespondInfo("DriveHwVersion=%s" % DriveHwVersion)
        # self.G_PhrozenFluiddRespondInfo("DriveFwVersion=%s" % DriveFwVersion)
        # self.G_PhrozenFluiddRespondInfo("DriveId=%s" % DriveId)



        #self.G_ProzenToolhead.dwell(1.5)


        #=====DriveCodeFile.dat
        # 1 , 18 , 24053 , 18 , 0# // AMSmainboard1firmware-18
        # 2 , 18 , 24053 , 18 , 0# // AMSmainboard2firmware-18
        # 3 , 18 , 24053 , 18 , 0# // AMSmainboard3firmware-18
        # 4 , 18 , 24053 , 18 , 0# // AMSmainboard4firmware-18
        # 5, 5, 24046, 5, 0# // OTA sequence -AMSserial portupdate sequence -5 5
        # 6, 0, 0, 0, 0# // bufferfirmware-6 6 keep reserve
        # 7, 7, 24051, 7, 0# // 16HUBfirmware-7 7
        # 8 , 0 , 0 , 0 , 0
        # 9 , 0 , 0 , 0 , 0
        # 10, 10, 24054, 10, 0# // OTA sequence -serial port disable after unit sequence -10
        # 11, 11, 24047, 11, 0# // serial port disable before unit HMIfirmware-11
        # 12 , 0 , 0 , 0 , 0
        # 13 , 0 , 0 , 0 , 0
        # 14 , 0 , 0 , 0 , 0
        # 15 , 15 , 25042 , 15 , 0
        # 16 , 16 , 25042 , 16 , 0
        # 17 , ? , 25042 , ? , 0
        # 18 , ? , 25042 , ? , 0
        # 19 , ? , 25042 , ? , 0
        # 20 , ? , 25042 , ? , 0
        # lancaigang240530: versionwrite todat text file; DriveCodeJson.dat
               #lancaigang250724: Read the system image ID to distinguish product, mainboard, and firmware variants
        #lancaigang250724:Read image ID
        self.Cmds_GetImageId()
        if self.G_ImageId==16:
            self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==16: ARCO300-MKS-RK3328-STM32F407VET6-I16")
            filename='/home/mks/hdlDat/DriveCodeFile.dat'
            self.G_PhrozenFluiddRespondInfo("filename=%s" % filename)
        elif self.G_ImageId==31:
            self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==31: ARCO300-phrozen-RK3308-STM32F407VET6-I31")
            filename='/home/prz/hdlDat/DriveCodeFile.dat'
            self.G_PhrozenFluiddRespondInfo("filename=%s" % filename)
        elif self.G_ImageId==-1:
            self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==-1, default: ARCO300-MKS-RK3328-STM32F407VET6-I16")
            filename='/home/mks/hdlDat/DriveCodeFile.dat'
            self.G_PhrozenFluiddRespondInfo("filename=%s" % filename)
        else:
            self.G_PhrozenFluiddRespondInfo("[INFO] Image IDcould not be read, default: ARCO300-MKS-RK3328-STM32F407VET6-I16")
            filename='/home/mks/hdlDat/DriveCodeFile.dat'
            self.G_PhrozenFluiddRespondInfo("filename=%s" % filename)

        Lo_AllLine=""
        #data = [{"DriveCode":16,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":15,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":14,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":13,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":12,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":11,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":10,"DriveImageType":10,"DriveHwVersion":10,"DriveFwVersion":24045,"DriveId":0},{"DriveCode":9,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":8,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":7,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":6,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":5,"DriveImageType":5,"DriveHwVersion":5,"DriveFwVersion":24046,"DriveId":0},{"DriveCode":4,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":3,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":2,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":1,"DriveImageType":1,"DriveHwVersion":1,"DriveFwVersion":24042,"DriveId":0}]
        #f = open(filename, 'a')
        # json.dump(data, f) # to sequence list is byte stream
        #f.close()
        with open(filename,'r') as file:
            #for line in file:
            # # realine() read internal, "\n"
            # self.G_PhrozenFluiddRespondInfo(file.readline().strip())
            # #time.sleep(1)
            Lo_FileDataList=file.readlines()
            for line in Lo_FileDataList:
                #split = [i[:-1].split(',') for i in file.readlines()]
                #self.G_PhrozenFluiddRespondInfo(type(split))
                #self.G_PhrozenFluiddRespondInfo(split[1])
                #self.G_PhrozenFluiddRespondInfo(split[2])
                #self.G_PhrozenFluiddRespondInfo(split[3])
                #line_strip=line.strip()
                #self.G_PhrozenFluiddRespondInfo(line)
                #self.G_PhrozenFluiddRespondInfo("line.count=%d" % line.count)
                split=line.split(',')
                #self.G_PhrozenFluiddRespondInfo(type(split))
                #self.G_PhrozenFluiddRespondInfo("".join(split))
                #self.G_PhrozenFluiddRespondInfo(split[0])
                split[0]=split[0].strip()# move
                split[1]=split[1].strip()# file id
                split[2]=split[2].strip()# firmwareversion
                split[3]=split[3].strip()# id
                split[4]=split[4].strip()# is no in

                # lancaigang240617: first set id=1 and 7 in state set 0, ifAMShas, then set 1 in

                if split[0]=="16":
                    self.G_PhrozenFluiddRespondInfo(split[0])
                    self.G_PhrozenFluiddRespondInfo(split[1])
                    self.G_PhrozenFluiddRespondInfo(split[2])
                    self.G_PhrozenFluiddRespondInfo(split[3])
                    self.G_PhrozenFluiddRespondInfo(split[4])
                    #line=("%d,%d,%d," % (HW_VERSION,,))
                    line_modify=split[0]+','+'16'+','+str(FW_VERSION)+','+'16'+','+'1'
                    self.G_PhrozenFluiddRespondInfo(line_modify)
                    Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
                elif split[0]=="1":
                    self.G_PhrozenFluiddRespondInfo(split[0])
                    self.G_PhrozenFluiddRespondInfo(split[1])
                    self.G_PhrozenFluiddRespondInfo(split[2])
                    self.G_PhrozenFluiddRespondInfo(split[3])
                    self.G_PhrozenFluiddRespondInfo(split[4])
                    line_modify=split[0]+','+'1'+','+"00000"+','+'1'+','+'0'
                    self.G_PhrozenFluiddRespondInfo(line_modify)
                    Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
                elif split[0]=="2":
                    self.G_PhrozenFluiddRespondInfo(split[0])
                    self.G_PhrozenFluiddRespondInfo(split[1])
                    self.G_PhrozenFluiddRespondInfo(split[2])
                    self.G_PhrozenFluiddRespondInfo(split[3])
                    self.G_PhrozenFluiddRespondInfo(split[4])
                    line_modify=split[0]+','+'1'+','+"00000"+','+'1'+','+'0'
                    self.G_PhrozenFluiddRespondInfo(line_modify)
                    Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
                elif split[0]=="3":
                    self.G_PhrozenFluiddRespondInfo(split[0])
                    self.G_PhrozenFluiddRespondInfo(split[1])
                    self.G_PhrozenFluiddRespondInfo(split[2])
                    self.G_PhrozenFluiddRespondInfo(split[3])
                    self.G_PhrozenFluiddRespondInfo(split[4])
                    line_modify=split[0]+','+'1'+','+"00000"+','+'1'+','+'0'
                    self.G_PhrozenFluiddRespondInfo(line_modify)
                    Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
                elif split[0]=="4":
                    self.G_PhrozenFluiddRespondInfo(split[0])
                    self.G_PhrozenFluiddRespondInfo(split[1])
                    self.G_PhrozenFluiddRespondInfo(split[2])
                    self.G_PhrozenFluiddRespondInfo(split[3])
                    self.G_PhrozenFluiddRespondInfo(split[4])
                    line_modify=split[0]+','+'1'+','+"00000"+','+'1'+','+'0'
                    self.G_PhrozenFluiddRespondInfo(line_modify)
                    Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
                elif split[0]=="5":
                    self.G_PhrozenFluiddRespondInfo(split[0])
                    self.G_PhrozenFluiddRespondInfo(split[1])
                    self.G_PhrozenFluiddRespondInfo(split[2])
                    self.G_PhrozenFluiddRespondInfo(split[3])
                    self.G_PhrozenFluiddRespondInfo(split[4])
                    line_modify=split[0]+','+split[1]+','+split[2]+','+split[3]+','+'1'
                    self.G_PhrozenFluiddRespondInfo(line_modify)
                    Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
                elif split[0]=="10":
                    self.G_PhrozenFluiddRespondInfo(split[0])
                    self.G_PhrozenFluiddRespondInfo(split[1])
                    self.G_PhrozenFluiddRespondInfo(split[2])
                    self.G_PhrozenFluiddRespondInfo(split[3])
                    self.G_PhrozenFluiddRespondInfo(split[4])
                    line_modify=split[0]+','+split[1]+','+split[2]+','+split[3]+','+'1'
                    self.G_PhrozenFluiddRespondInfo(line_modify)
                    Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
                elif split[0]=="7":
                    self.G_PhrozenFluiddRespondInfo(split[0])
                    self.G_PhrozenFluiddRespondInfo(split[1])
                    self.G_PhrozenFluiddRespondInfo(split[2])
                    self.G_PhrozenFluiddRespondInfo(split[3])
                    self.G_PhrozenFluiddRespondInfo(split[4])
                    line_modify=split[0]+','+'7'+','+"00000"+','+'7'+','+'0'
                    self.G_PhrozenFluiddRespondInfo(line_modify)
                    Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A

                elif split[0]=="17":
                    self.G_PhrozenFluiddRespondInfo(split[0])
                    self.G_PhrozenFluiddRespondInfo(split[1])
                    self.G_PhrozenFluiddRespondInfo(split[2])
                    self.G_PhrozenFluiddRespondInfo(split[3])
                    self.G_PhrozenFluiddRespondInfo(split[4])
                    #line=("%d,%d,%d," % (HW_VERSION,,))
                    line_modify=split[0]+','+'18'+','+"00000"+','+'18'+','+'0'
                    self.G_PhrozenFluiddRespondInfo(line_modify)
                    Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
                elif split[0]=="18":
                    self.G_PhrozenFluiddRespondInfo(split[0])
                    self.G_PhrozenFluiddRespondInfo(split[1])
                    self.G_PhrozenFluiddRespondInfo(split[2])
                    self.G_PhrozenFluiddRespondInfo(split[3])
                    self.G_PhrozenFluiddRespondInfo(split[4])
                    #line=("%d,%d,%d," % (HW_VERSION,,))
                    line_modify=split[0]+','+'18'+','+"00000"+','+'18'+','+'0'
                    self.G_PhrozenFluiddRespondInfo(line_modify)
                    Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
                elif split[0]=="19":
                    self.G_PhrozenFluiddRespondInfo(split[0])
                    self.G_PhrozenFluiddRespondInfo(split[1])
                    self.G_PhrozenFluiddRespondInfo(split[2])
                    self.G_PhrozenFluiddRespondInfo(split[3])
                    self.G_PhrozenFluiddRespondInfo(split[4])
                    #line=("%d,%d,%d," % (HW_VERSION,,))
                    line_modify=split[0]+','+'18'+','+"00000"+','+'18'+','+'0'
                    self.G_PhrozenFluiddRespondInfo(line_modify)
                    Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
                elif split[0]=="20":
                    self.G_PhrozenFluiddRespondInfo(split[0])
                    self.G_PhrozenFluiddRespondInfo(split[1])
                    self.G_PhrozenFluiddRespondInfo(split[2])
                    self.G_PhrozenFluiddRespondInfo(split[3])
                    self.G_PhrozenFluiddRespondInfo(split[4])
                    #line=("%d,%d,%d," % (HW_VERSION,,))
                    line_modify=split[0]+','+'18'+','+"00000"+','+'18'+','+'0'
                    self.G_PhrozenFluiddRespondInfo(line_modify)
                    Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A


                else:
                    Lo_AllLine=Lo_AllLine+line
        #self.G_PhrozenFluiddRespondInfo(Lo_AllLine)
        with open(filename,"w+") as file_w:
	        file_w.write(Lo_AllLine)




        #self.G_PhrozenFluiddRespondInfo("PRZ_DEV_VER: %s" % FW_VERSION)
        #self.G_PhrozenFluiddRespondInfo("V-H'%s'-I'%s'-F'%s'" % (HW_VERSION,IMAGE_VERSION,FW_VERSION))




    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: # PRZ_ADC head filament device value (use)
    ####################################
    #lancaigang251020: 
    def Cmds_PhrozenAdc(self,gcmd):
        self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_PhrozenAdc]command=PRZ_ADC")

        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)
       
        # readtoolhead after ADC value
        value, timestamp = self.G_ToolheadAdc.get_last_value()
        
        self.G_PhrozenFluiddRespondInfo('PRZ_ADC: read get ADC value %.6f (timestamp %.3f) fila_exist:%r' % (value, timestamp, self.G_ToolheadIfHaveFilaFlag))
        self.G_PhrozenFluiddRespondInfo("self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag))
        self.G_PhrozenFluiddRespondInfo("self.G_AMS1ErrorRestartCount=%d" % (self.G_AMS1ErrorRestartCount))

        self.G_PhrozenFluiddRespondInfo("[INFO] Current mode")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:0,unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:1,MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:2,MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo("[WARN] +Mode:3,RUNOUT")
        else:
            self.G_PhrozenFluiddRespondInfo("[ERROR] +Mode:-1,error")


        self.G_PhrozenFluiddRespondInfo("self.G_P0M3Flag=%d" % (self.G_P0M3Flag))
        self.G_PhrozenFluiddRespondInfo("self.G_KlipperIfPaused=%d" % (self.G_KlipperIfPaused))
        self.G_PhrozenFluiddRespondInfo("self.G_CancelFlag=%d" % (self.G_CancelFlag))
        self.G_PhrozenFluiddRespondInfo("self.G_IfChangeFilaOngoing=%d" % (self.G_IfChangeFilaOngoing))
        self.G_PhrozenFluiddRespondInfo("self.G_SerialPort1OpenFlag=%d" % (self.G_SerialPort1OpenFlag))
        self.G_PhrozenFluiddRespondInfo("self.G_P0M2MAStartPrintFlag=%d" % (self.G_P0M2MAStartPrintFlag))
        self.G_PhrozenFluiddRespondInfo("self.G_M2MAModeResumeFlag=%d" % (self.G_M2MAModeResumeFlag))
        self.G_PhrozenFluiddRespondInfo("self.G_KlipperPrintStatus=%d" % (self.G_KlipperPrintStatus))
        self.G_PhrozenFluiddRespondInfo("self.G_SerialPort1OpenFlag=%d" % (self.G_SerialPort1OpenFlag))
        self.G_PhrozenFluiddRespondInfo("self.G_SerialPort2OpenFlag=%d" % (self.G_SerialPort2OpenFlag))
        self.G_PhrozenFluiddRespondInfo("self.ManualCmdFlag=%d" % (self.ManualCmdFlag))
        self.G_PhrozenFluiddRespondInfo("self.STM32ReprotPauseFlag=%d" % (self.STM32ReprotPauseFlag))
        self.G_PhrozenFluiddRespondInfo("self.PG102Flag=%d" % (self.PG102Flag))
        self.G_PhrozenFluiddRespondInfo("self.G_IfInFilaBlockFlag=%d" % (self.G_IfInFilaBlockFlag))
        self.G_PhrozenFluiddRespondInfo("self.PG102DelayPauseFlag=%d" % (self.PG102DelayPauseFlag))
        self.G_PhrozenFluiddRespondInfo("self.G_KlipperQuickPause=%d" % (self.G_KlipperQuickPause))
        self.G_PhrozenFluiddRespondInfo("self.G_PauseToLCDString=%s" % (self.G_PauseToLCDString))






        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)

        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")


        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()
        #lancaigang241031:
        if self.G_SerialPort1OpenFlag == True:
            # lancaigang240524: readAMSmainboardversion、16HUBmainboardversion
            self.Cmds_AMSSerial1Send("AT+SB=0")
            self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: AT+SB=0; get AMSmainboard、16HUBmainboard')
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            # lancaigang240524: readAMSmainboardversion、16HUBmainboardversion
            self.Cmds_AMSSerial2Send("AT+SB=0")
            self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2Sending command: AT+SB=0; get AMSmainboard、16HUBmainboard')


        # lancaigang240529: phrozen file version
        self.G_PhrozenFluiddRespondInfo("V-H%s-I%s-F%s-SN1" % (HW_VERSION,IMAGE_VERSION,FW_VERSION))

        #time.sleep(1)

        #self.Cmds_AMSSerial1Send("AT+SB=1")
        # self.G_PhrozenFluiddRespondInfo("[DEBUG] Sending command: AT+SB=1; get AMSmainboardstate")



        #PRZ_PwrDownResumePrint
        try:
            self.G_PhrozenFluiddRespondInfo("[INFO] try")
            # lancaigang240530: versionwrite todat text file; DriveCodeJson.dat

            #lancaigang250724: Read the system image ID to distinguish product, mainboard, and firmware variants
            #lancaigang250724:Read image ID
            self.Cmds_GetImageId()
            if self.G_ImageId==16:
                self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==16: ARCO300-MKS-RK3328-STM32F407VET6-I16")
                with open('/home/mks/hdlDat/Phrozen_Dev.json', 'r', encoding='utf-8') as file:
                    FileRead = file.read()
                    self.G_PhrozenFluiddRespondInfo('[INFO] read get json text file')
                    # JSON number data
                    json_data = json.loads(FileRead)
                    self.G_PhrozenFluiddRespondInfo("json_data['mode']=%d" % (json_data['mode']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc1']=%d" % (json_data['nc1']))
            elif self.G_ImageId==31:
                self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==31: ARCO300-phrozen-RK3308-STM32F407VET6-I31")
                with open('/home/prz/hdlDat/Phrozen_Dev.json', 'r', encoding='utf-8') as file:
                    FileRead = file.read()
                    self.G_PhrozenFluiddRespondInfo('[INFO] read get json text file')
                    # JSON number data
                    json_data = json.loads(FileRead)
                    self.G_PhrozenFluiddRespondInfo("json_data['mode']=%d" % (json_data['mode']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc1']=%d" % (json_data['nc1']))
            elif self.G_ImageId==-1:
                self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==-1, default: ARCO300-MKS-RK3328-STM32F407VET6-I16")
                with open('/home/mks/hdlDat/Phrozen_Dev.json', 'r', encoding='utf-8') as file:
                    FileRead = file.read()
                    self.G_PhrozenFluiddRespondInfo('[INFO] read get json text file')
                    # JSON number data
                    json_data = json.loads(FileRead)
                    self.G_PhrozenFluiddRespondInfo("json_data['mode']=%d" % (json_data['mode']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc1']=%d" % (json_data['nc1']))
            else:
                self.G_PhrozenFluiddRespondInfo("[INFO] Image IDcould not be read, default: ARCO300-MKS-RK3328-STM32F407VET6-I16")
                with open('/home/mks/hdlDat/Phrozen_Dev.json', 'r', encoding='utf-8') as file:
                    FileRead = file.read()
                    self.G_PhrozenFluiddRespondInfo('[INFO] read get json text file')
                    # JSON number data
                    json_data = json.loads(FileRead)
                    self.G_PhrozenFluiddRespondInfo("json_data['mode']=%d" % (json_data['mode']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc1']=%d" % (json_data['nc1']))
        except:
            self.G_PhrozenFluiddRespondInfo("[INFO] except")



    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_PhrozenBM1(self, gcmd):
        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenBM1]command='%s'" % (gcmd.get_commandline(),))

        self.G_PhrozenFluiddRespondInfo("%s" % (gcmd.get_commandline(),))

        # lancaigang250327: multi-materialbefore, not allowedAMSmulti-materialpause
        self.ManualCmdFlag=True
        self.G_PhrozenFluiddRespondInfo("[INFO] self.ManualCmdFlag=True")

        #  #lancaigang250514: Read the JSON file for monochrome refill settings and channel/color mappings
        # #/home/prz/klipper/klippy/extras/phrozen_dev/serial-screen/test.json
        # self.Cmds_GetUartScreenCfg()

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_PhrozenBM0(self, gcmd):
        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenBM0]command='%s'" % (gcmd.get_commandline(),))

        self.G_PhrozenFluiddRespondInfo("%s" % (gcmd.get_commandline(),))
        # lancaigang250327: multi-materialbefore, not allowedAMSmulti-materialpause
        self.ManualCmdFlag=True
        self.G_PhrozenFluiddRespondInfo("[INFO] self.ManualCmdFlag=True")

        #  #lancaigang250514: Read the JSON file for monochrome refill settings and channel/color mappings
        # #/home/prz/klipper/klippy/extras/phrozen_dev/serial-screen/test.json
        # self.Cmds_GetUartScreenCfg()
    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    #PRZ_PRINT_START
    def Cmds_PrzPrintStart(self, gcmd):
        self.G_PhrozenFluiddRespondInfo("[(cmds.python)CmdsPrzPrintStart]command='%s'" % (gcmd.get_commandline(),))

        self.G_PhrozenFluiddRespondInfo("%s" % (gcmd.get_commandline(),))

        # #lancaigang250514: Read the JSON file for monochrome refill settings and channel/color mappings
        # #/home/prz/klipper/klippy/extras/phrozen_dev/serial-screen/test.json
        # self.Cmds_GetUartScreenCfg()

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_HomingOverrideEnd(self, gcmd):
        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_HomingOverrideEnd]command='%s'" % (gcmd.get_commandline(),))

        self.G_PhrozenFluiddRespondInfo("%s" % (gcmd.get_commandline(),))

        # #lancaigang250514: Read the JSON file for monochrome refill settings and channel/color mappings
        # #/home/prz/klipper/klippy/extras/phrozen_dev/serial-screen/test.json
        # self.Cmds_GetUartScreenCfg()


    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_PrzPrintEnd(self, gcmd):
        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PrzPrintEnd]command='%s'" % (gcmd.get_commandline(),))

        self.G_PhrozenFluiddRespondInfo("%s" % (gcmd.get_commandline(),))

####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_PrintMode(self,mode):
        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PrintMode]Sending command: self.G_AMSDeviceWorkMode=%d" % self.G_AMSDeviceWorkMode)
        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PrintMode]Sending command: mode=%d" % mode)

        try:
            self.G_PhrozenFluiddRespondInfo("[INFO] try")
            Phrozen_Dev = {"mode": self.G_AMSDeviceWorkMode, "nc1": self.G_ChangeChannelTimeoutOldChan, "nc2": self.G_ChangeChannelTimeoutNewChan,"nc3":0,"nc4":0,"nc5":0,"nc6":0}
            
            #lancaigang250724: Read the system image ID to distinguish product, mainboard, and firmware variants
            #lancaigang250724:Read image ID
            self.Cmds_GetImageId()
            if self.G_ImageId==16:
                self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==16: ARCO300-MKS-RK3328-STM32F407VET6-I16")
                with open('/home/mks/hdlDat/Phrozen_Dev.json', 'w') as file:
                    json.dump(Phrozen_Dev, file)
                    self.G_PhrozenFluiddRespondInfo('[INFO] write json text file')
                with open('/home/mks/hdlDat/Phrozen_Dev.json', 'r', encoding='utf-8') as file:
                    FileRead = file.read()
                    self.G_PhrozenFluiddRespondInfo('[INFO] read get json text file')
                    # JSON number data
                    json_data = json.loads(FileRead)
                    self.G_PhrozenFluiddRespondInfo("json_data['mode']=%d" % (json_data['mode']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc1']=%d" % (json_data['nc1']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc2']=%d" % (json_data['nc2']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc3']=%d" % (json_data['nc3']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc4']=%d" % (json_data['nc4']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc5']=%d" % (json_data['nc5']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc6']=%d" % (json_data['nc6']))
            elif self.G_ImageId==31:
                self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==31: ARCO300-phrozen-RK3308-STM32F407VET6-I31")
                with open('/home/prz/hdlDat/Phrozen_Dev.json', 'w') as file:
                    json.dump(Phrozen_Dev, file)
                    self.G_PhrozenFluiddRespondInfo('[INFO] write json text file')
                with open('/home/prz/hdlDat/Phrozen_Dev.json', 'r', encoding='utf-8') as file:
                    FileRead = file.read()
                    self.G_PhrozenFluiddRespondInfo('[INFO] read get json text file')
                    # JSON number data
                    json_data = json.loads(FileRead)
                    self.G_PhrozenFluiddRespondInfo("json_data['mode']=%d" % (json_data['mode']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc1']=%d" % (json_data['nc1']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc2']=%d" % (json_data['nc2']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc3']=%d" % (json_data['nc3']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc4']=%d" % (json_data['nc4']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc5']=%d" % (json_data['nc5']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc6']=%d" % (json_data['nc6']))
            elif self.G_ImageId==-1:
                self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==-1, default: ARCO300-MKS-RK3328-STM32F407VET6-I16")
                with open('/home/mks/hdlDat/Phrozen_Dev.json', 'w') as file:
                    json.dump(Phrozen_Dev, file)
                    self.G_PhrozenFluiddRespondInfo('[INFO] write json text file')
                with open('/home/mks/hdlDat/Phrozen_Dev.json', 'r', encoding='utf-8') as file:
                    FileRead = file.read()
                    self.G_PhrozenFluiddRespondInfo('[INFO] read get json text file')
                    # JSON number data
                    json_data = json.loads(FileRead)
                    self.G_PhrozenFluiddRespondInfo("json_data['mode']=%d" % (json_data['mode']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc1']=%d" % (json_data['nc1']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc2']=%d" % (json_data['nc2']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc3']=%d" % (json_data['nc3']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc4']=%d" % (json_data['nc4']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc5']=%d" % (json_data['nc5']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc6']=%d" % (json_data['nc6']))
            else:
                self.G_PhrozenFluiddRespondInfo("[INFO] Image IDcould not be read, default: ARCO300-MKS-RK3328-STM32F407VET6-I16")
                with open('/home/mks/hdlDat/Phrozen_Dev.json', 'w') as file:
                    json.dump(Phrozen_Dev, file)
                    self.G_PhrozenFluiddRespondInfo('[INFO] write json text file')
                with open('/home/mks/hdlDat/Phrozen_Dev.json', 'r', encoding='utf-8') as file:
                    FileRead = file.read()
                    self.G_PhrozenFluiddRespondInfo('[INFO] read get json text file')
                    # JSON number data
                    json_data = json.loads(FileRead)
                    self.G_PhrozenFluiddRespondInfo("json_data['mode']=%d" % (json_data['mode']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc1']=%d" % (json_data['nc1']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc2']=%d" % (json_data['nc2']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc3']=%d" % (json_data['nc3']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc4']=%d" % (json_data['nc4']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc5']=%d" % (json_data['nc5']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc6']=%d" % (json_data['nc6']))
        except:
            self.G_PhrozenFluiddRespondInfo("[INFO] except")


    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    #PRZ_RESTORE
    def Cmds_PrzATRestore(self, gcmd):
        self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_PrzATRestore]")

        #self.G_PhrozenFluiddRespondInfo("%s" % (gcmd.get_commandline(),))

        self.G_PhrozenFluiddRespondInfo("[INFO] Current mode")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:0,unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:1,MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:2,MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo("[WARN] +Mode:3,RUNOUT")
        else:
            self.G_PhrozenFluiddRespondInfo("[ERROR] +Mode:-1,error")


        

        #PRZ_PwrDownResumePrint
        try:
            self.G_PhrozenFluiddRespondInfo("[INFO] try")

            #lancaigang250724: Read the system image ID to distinguish product, mainboard, and firmware variants
            #lancaigang250724:Read image ID
            self.Cmds_GetImageId()
            if self.G_ImageId==16:
                self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==16: ARCO300-MKS-RK3328-STM32F407VET6-I16")
                with open('/home/mks/hdlDat/Phrozen_Dev.json', 'r', encoding='utf-8') as file:
                    FileRead = file.read()
                    self.G_PhrozenFluiddRespondInfo('[INFO] read get json text file')
                    # JSON number data
                    json_data = json.loads(FileRead)
                    self.G_PhrozenFluiddRespondInfo("json_data['mode']=%d" % (json_data['mode']))
                    self.G_AMSDeviceWorkMode = json_data['mode']
                    self.G_PhrozenFluiddRespondInfo("self.G_AMSDeviceWorkMode=%d" % (self.G_AMSDeviceWorkMode))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc1']=%d" % (json_data['nc1']))
                    self.G_ChangeChannelTimeoutOldChan=json_data['nc1']
                    self.G_PhrozenFluiddRespondInfo("self.G_ChangeChannelTimeoutOldChan=%d" % (self.G_ChangeChannelTimeoutOldChan))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc2']=%d" % (json_data['nc2']))
                    self.G_ChangeChannelTimeoutNewChan=json_data['nc2']
                    self.G_PhrozenFluiddRespondInfo("self.G_ChangeChannelTimeoutNewChan=%d" % (self.G_ChangeChannelTimeoutNewChan))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc3']=%d" % (json_data['nc3']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc4']=%d" % (json_data['nc4']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc5']=%d" % (json_data['nc5']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc6']=%d" % (json_data['nc6']))
            elif self.G_ImageId==31:
                self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==31: ARCO300-phrozen-RK3308-STM32F407VET6-I31")
                with open('/home/prz/hdlDat/Phrozen_Dev.json', 'r', encoding='utf-8') as file:
                    FileRead = file.read()
                    self.G_PhrozenFluiddRespondInfo('[INFO] read get json text file')
                    # JSON number data
                    json_data = json.loads(FileRead)
                    self.G_PhrozenFluiddRespondInfo("json_data['mode']=%d" % (json_data['mode']))
                    self.G_AMSDeviceWorkMode = json_data['mode']
                    self.G_PhrozenFluiddRespondInfo("self.G_AMSDeviceWorkMode=%d" % (self.G_AMSDeviceWorkMode))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc1']=%d" % (json_data['nc1']))
                    self.G_ChangeChannelTimeoutOldChan=json_data['nc1']
                    self.G_PhrozenFluiddRespondInfo("self.G_ChangeChannelTimeoutOldChan=%d" % (self.G_ChangeChannelTimeoutOldChan))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc2']=%d" % (json_data['nc2']))
                    self.G_ChangeChannelTimeoutNewChan=json_data['nc2']
                    self.G_PhrozenFluiddRespondInfo("self.G_ChangeChannelTimeoutNewChan=%d" % (self.G_ChangeChannelTimeoutNewChan))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc3']=%d" % (json_data['nc3']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc4']=%d" % (json_data['nc4']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc5']=%d" % (json_data['nc5']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc6']=%d" % (json_data['nc6']))
            elif self.G_ImageId==-1:
                self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==-1, default: ARCO300-MKS-RK3328-STM32F407VET6-I16")
                with open('/home/mks/hdlDat/Phrozen_Dev.json', 'r', encoding='utf-8') as file:
                    FileRead = file.read()
                    self.G_PhrozenFluiddRespondInfo('[INFO] read get json text file')
                    # JSON number data
                    json_data = json.loads(FileRead)
                    self.G_PhrozenFluiddRespondInfo("json_data['mode']=%d" % (json_data['mode']))
                    self.G_AMSDeviceWorkMode = json_data['mode']
                    self.G_PhrozenFluiddRespondInfo("self.G_AMSDeviceWorkMode=%d" % (self.G_AMSDeviceWorkMode))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc1']=%d" % (json_data['nc1']))
                    self.G_ChangeChannelTimeoutOldChan=json_data['nc1']
                    self.G_PhrozenFluiddRespondInfo("self.G_ChangeChannelTimeoutOldChan=%d" % (self.G_ChangeChannelTimeoutOldChan))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc2']=%d" % (json_data['nc2']))
                    self.G_ChangeChannelTimeoutNewChan=json_data['nc2']
                    self.G_PhrozenFluiddRespondInfo("self.G_ChangeChannelTimeoutNewChan=%d" % (self.G_ChangeChannelTimeoutNewChan))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc3']=%d" % (json_data['nc3']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc4']=%d" % (json_data['nc4']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc5']=%d" % (json_data['nc5']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc6']=%d" % (json_data['nc6']))
            else:
                self.G_PhrozenFluiddRespondInfo("[INFO] Image IDcould not be read, default: ARCO300-MKS-RK3328-STM32F407VET6-I16")
                with open('/home/mks/hdlDat/Phrozen_Dev.json', 'r', encoding='utf-8') as file:
                    FileRead = file.read()
                    self.G_PhrozenFluiddRespondInfo('[INFO] read get json text file')
                    # JSON number data
                    json_data = json.loads(FileRead)
                    self.G_PhrozenFluiddRespondInfo("json_data['mode']=%d" % (json_data['mode']))
                    self.G_AMSDeviceWorkMode = json_data['mode']
                    self.G_PhrozenFluiddRespondInfo("self.G_AMSDeviceWorkMode=%d" % (self.G_AMSDeviceWorkMode))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc1']=%d" % (json_data['nc1']))
                    self.G_ChangeChannelTimeoutOldChan=json_data['nc1']
                    self.G_PhrozenFluiddRespondInfo("self.G_ChangeChannelTimeoutOldChan=%d" % (self.G_ChangeChannelTimeoutOldChan))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc2']=%d" % (json_data['nc2']))
                    self.G_ChangeChannelTimeoutNewChan=json_data['nc2']
                    self.G_PhrozenFluiddRespondInfo("self.G_ChangeChannelTimeoutNewChan=%d" % (self.G_ChangeChannelTimeoutNewChan))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc3']=%d" % (json_data['nc3']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc4']=%d" % (json_data['nc4']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc5']=%d" % (json_data['nc5']))
                    self.G_PhrozenFluiddRespondInfo("json_data['nc6']=%d" % (json_data['nc6']))

            


            try:
                self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.py)Cmds_PrzATRestore]Reinitializing serial port 1")
                self.G_SerialPort1Obj = serial.Serial(self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3)
                #Serial port opened successfully
                if self.G_SerialPort1Obj is not None:
                    if self.G_SerialPort1Obj.is_open:
                        self.G_SerialPort1OpenFlag = True
                        self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 1successful")
                        # lancaigang231213: openserial port
                        self.G_SerialPort1Obj.flushInput()  # clean serial write cache
                        self.G_SerialPort1Obj.flush()
                        self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 buffers cleared")
                        self.G_PhrozenFluiddRespondInfo("[INFO] Re-registering serial port 1 callback")
                        self.G_SerialPort1RecvTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerUart1Recv, self.G_PhrozenReactor.NOW)
            except:
                self.G_PhrozenFluiddRespondInfo("[ERROR] Unable to open tty1. Check the USB connection or try rebooting.")
                self.G_SerialPort1OpenFlag = False

            try:
                self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.py)Cmds_PrzATRestore]Reinitializing serial port 2")
                self.G_SerialPort2Obj = serial.Serial(self.G_Serialport2Define, SERIAL_PORT_BAUD, timeout=3)
                #Serial port opened successfully
                if self.G_SerialPort2Obj is not None:
                    if self.G_SerialPort2Obj.is_open:
                        self.G_SerialPort2OpenFlag = True
                        self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 2successful")
                        self.G_SerialPort2Obj.flushInput()  # clean serial write cache
                        self.G_SerialPort2Obj.flush()
                        self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 buffers cleared")
                        self.G_PhrozenFluiddRespondInfo("[INFO] Re-registering serial port 2 callback")
                        self.G_SerialPort2RecvTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerUart2Recv, self.G_PhrozenReactor.NOW)
            except:
                self.G_PhrozenFluiddRespondInfo("[DEBUG] AMS2/tty2 not available; skipping serial port 2.")
                self.G_SerialPort2OpenFlag = False


            # lancaigang250619:checkAMS is no re connectsuccessful
            self.Cmds_USBConnectErrorCheck()
            #lancaigang240416:
            if self.G_SerialPort1OpenFlag == True:
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: AT+RESTORE")
                self.Cmds_AMSSerial1Send("AT+RESTORE")
                
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: AT+RESTORE")
                self.Cmds_AMSSerial2Send("AT+RESTORE")
                

            self.G_ProzenToolhead.dwell(2)


            

            self.G_PhrozenFluiddRespondInfo("[INFO] Current mode")
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:0,unkown")



            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
                self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:1,MC")
                # lancaigang250607: state; 1-unload filament; 2-feed; 3-; 4-pause
                #self.G_KlipperPrintStatus = 3
                if self.G_SerialPort1OpenFlag == False and self.G_SerialPort2OpenFlag == False:
                    self.G_PhrozenFluiddRespondInfo('[INFO] noneconnectedAMS, pause')
                    if self.G_KlipperInPausing == False:
                        self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                        self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                        self.G_KlipperIfPaused = True
                        #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                        self.STM32ReprotPauseFlag=1
                        # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                        self.G_ChangeChannelFirstFilaFlag=True
                        self.G_PhrozenFluiddRespondInfo("+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")


            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:2,MA")
                self.G_P0M2MAStartPrintFlag=1
                #self.G_ToolheadFirstInputFila=False
                #self.P0M3FilaRunoutSpittingFinished=True
                if self.G_ToolheadIfHaveFilaFlag:
                    self.G_PhrozenFluiddRespondInfo('[INFO] has filament, canprinting')
                    # lancaigang250607: state; 1-unload filament; 2-feed; 3-; 4-pause
                    self.G_KlipperPrintStatus= 3
                    #lancaigang250522: 
                    self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG108")
                    # lancaigang251120: purge, add, preventPG108purge toolheadnofilamentpause, pauseposition in purge, resumewhen will to purge;
                    self.G_PG108Ingoing=1
                    command_string = """
                        PG108
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.G_PG108Ingoing=0
                    self.G_PhrozenFluiddRespondInfo("External macro command-PG108; command_string='%s'" % command_string)

                    #lancaigang250607:
                    self.G_PhrozenFluiddRespondInfo('[INFO] canresume, STM32printing report error')
                    self.G_KlipperQuickPause = True
                    # #lancaigang250427: 
                    # if self.G_SerialPort1OpenFlag == True:
                    #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                    # self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-AMSfinishtimingbuffer-full time")
                    # if self.G_SerialPort2OpenFlag == True:
                    #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                    # self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-AMSfinishtimingbuffer-full time")
                    # #self.G_ProzenToolhead.dwell(1.5)
                else:
                    self.G_PhrozenFluiddRespondInfo('[INFO] nonefilament, need to pause')
                    # self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-RESUME")
                    # command = """
                    # RESUME
                    # """
                    # self.G_PhrozenGCode.run_script_from_command(command)
                    # self.G_PhrozenFluiddRespondInfo("calling command:command=%s" % (command))
                    # lancaigang240125: number
                    self.Cmds_PhrozenKlipperResumeCommon()

                    #lancaigang250522: 
                    self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG109-heat up")
                    command_string = """
                        PG109
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.G_PhrozenFluiddRespondInfo("External macro command-PG109-; command_string='%s'" % command_string)

                    


                    

                    if self.G_KlipperInPausing == False:
                        self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                        #lancaigang250607:
                        # self.G_PhrozenFluiddRespondInfo("[WARN] use pause")
                        #self.G_KlipperQuickPause = True
                        # klipper move pause
                        self.Cmds_PhrozenKlipperPauseMAToSTM32(None)
                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")




                    self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))

                    # if self.G_SerialPort1OpenFlag==True or self.G_SerialPort2OpenFlag==True:
                    # self.G_PhrozenFluiddRespondInfo("[INFO] toolheadnonefilament, hasAMSmulti-material, P8feed")
                    #     #lancaigang241106: 
                    #     self.G_P0M2MAStartPrintFlag=0

                    # #lancaigang250522: not allowedM3filament runoutdetect
                    #     self.G_IfChangeFilaOngoing = True

                    #     #lancaigang241106: 
                    #     self.Cmds_CmdP8(gcmd)
                    # #lancaigang241106:toolheadsuccessfulfeed
                    #     if self.G_P0M2MAStartPrintFlag==1:
                    #         #lancaigang250607:
                    # self.G_PhrozenFluiddRespondInfo("[INFO] canresume, STM32printing report error ")
                    #         self.G_KlipperQuickPause = True
                    # self.G_PhrozenFluiddRespondInfo("[INFO] toolheadhasfilament, resume")
                    # #lancaigang240125: number
                    #         self.Cmds_PhrozenKlipperResumeCommon()
                    #     else:
                    #         self.G_KlipperQuickPause = False
                    # self.G_PhrozenFluiddRespondInfo("[WARN] toolheadnonefilament, refillpause")
                    #         if self.G_KlipperIfPaused == False:
                    #             self.Cmds_PhrozenKlipperPauseMAToSTM32(None)
                    #             self.G_KlipperIfPaused=True
                    #             #self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                    #             self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    #         else:
                    # self.G_PhrozenFluiddRespondInfo("[WARN] already pause, do notpause")
                    # else:
                    #     self.G_KlipperQuickPause = False
                    # self.G_PhrozenFluiddRespondInfo("[WARN] toolheadnonefilament, noAMSmulti-material, pause")
                    #     self.Cmds_PhrozenKlipperPauseMAToSTM32(None)
                    # #no filamentpause
                    #     self.G_KlipperIfPaused=True
                    # #lancaigang250521:hasAMSmulti-material
                    #     #if self.G_AMSDevice1IfNormal==True or self.G_AMSDevice2IfNormal==True:
                    #     self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    #     #else:
                    #     #self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))




            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                self.G_PhrozenFluiddRespondInfo("[WARN] +Mode:3,RUNOUT")
                self.G_P0M3Flag = True
                #self.G_ToolheadFirstInputFila = True
                # lancaigang240415: toolheadhasfilament, # time do notpurge
                #self.G_P0M3ToolheadHaveFilaNotSpittingFlag = True
                # self.P0M3FilaRunoutSpittingFinished==True:#purgecomplete

                if self.G_ToolheadIfHaveFilaFlag:
                    self.G_PhrozenFluiddRespondInfo('[INFO] has filament, canprinting')
                    self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG108")
                    # lancaigang251120: purge, add, preventPG108purge toolheadnofilamentpause, pauseposition in purge, resumewhen will to purge;
                    self.G_PG108Ingoing=1
                    command_string = """
                        PG108
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.G_PG108Ingoing=0
                    self.G_PhrozenFluiddRespondInfo("External macro command-PG108; command_string='%s'" % command_string)

                    #lancaigang250607:
                    self.G_PhrozenFluiddRespondInfo('[INFO] canresume, STM32printing report error')
                    #self.G_KlipperQuickPause = True
                    # #lancaigang250427: 
                    # if self.G_SerialPort1OpenFlag == True:
                    #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                    # self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-AMSfinishtimingbuffer-full time")
                    # if self.G_SerialPort2OpenFlag == True:
                    #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                    # self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-AMSfinishtimingbuffer-full time")
                    # #self.G_ProzenToolhead.dwell(1.5)
                else:
                    self.G_PhrozenFluiddRespondInfo('[INFO] nonefilament, need to pause')
                    # self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-RESUME")
                    # command = """
                    # RESUME
                    # """
                    # self.G_PhrozenGCode.run_script_from_command(command)
                    # self.G_PhrozenFluiddRespondInfo("calling command:command=%s" % (command))
                    # lancaigang240125: number
                    self.Cmds_PhrozenKlipperResumeCommon()

                    #lancaigang250522: 
                    self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG109-heat up")
                    command_string = """
                        PG109
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.G_PhrozenFluiddRespondInfo("External macro command-PG109-; command_string='%s'" % command_string)


                    if self.G_KlipperInPausing == False:
                        self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                        #lancaigang250607:
                        # self.G_PhrozenFluiddRespondInfo("[WARN] use pause")
                        #self.G_KlipperQuickPause = True
                        # klipper move pause
                        self.Cmds_PhrozenKlipperPauseMAToSTM32(None)
                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")
                    



                    self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))

                    # if self.G_SerialPort1OpenFlag==True or self.G_SerialPort2OpenFlag==True:
                    # self.G_PhrozenFluiddRespondInfo("[INFO] toolheadnonefilament, hasAMSmulti-material, P8feed")
                    #     #lancaigang241106: 
                    #     self.G_P0M2MAStartPrintFlag=0

                    # #lancaigang250522: not allowedM3filament runoutdetect
                    #     self.G_IfChangeFilaOngoing = True

                    #     #lancaigang241106: 
                    #     self.Cmds_CmdP8(gcmd)
                    # #lancaigang241106:toolheadsuccessfulfeed
                    #     if self.G_P0M2MAStartPrintFlag==1:
                    #         #lancaigang250607:
                    # self.G_PhrozenFluiddRespondInfo("[INFO] canresume, STM32printing report error ")
                    #         self.G_KlipperQuickPause = True
                    # self.G_PhrozenFluiddRespondInfo("[INFO] toolheadhasfilament, resume")
                    # #lancaigang240125: number
                    #         self.Cmds_PhrozenKlipperResumeCommon()
                    #     else:
                    #         self.G_KlipperQuickPause = False
                    # self.G_PhrozenFluiddRespondInfo("[WARN] toolheadnonefilament, refillpause")
                    #         if self.G_KlipperIfPaused == False:
                    #             self.Cmds_PhrozenKlipperPauseMAToSTM32(None)
                    #             self.G_KlipperIfPaused=True
                    #             #self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                    #             self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    #         else:
                    # self.G_PhrozenFluiddRespondInfo("[WARN] already pause, do notpause")
                    # else:
                    #     self.G_KlipperQuickPause = False
                    # self.G_PhrozenFluiddRespondInfo("[WARN] toolheadnonefilament, noAMSmulti-material, pause")
                    #     self.Cmds_PhrozenKlipperPauseMAToSTM32(None)
                    # #no filamentpause
                    #     self.G_KlipperIfPaused=True
                    # #lancaigang250521:hasAMSmulti-material
                    #     #if self.G_AMSDevice1IfNormal==True or self.G_AMSDevice2IfNormal==True:
                    #     self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    #     #else:
                    #     #self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))

            else:
                self.G_PhrozenFluiddRespondInfo("[ERROR] +Mode:-1,error")

        except:
            self.G_PhrozenFluiddRespondInfo("[INFO] except")





    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_PrzATIdle(self, gcmd):
        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PrzATIdle]command='%s'" % (gcmd.get_commandline(),))

        self.G_PhrozenFluiddRespondInfo("%s" % (gcmd.get_commandline(),))
        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()
        #lancaigang240416:
        if self.G_SerialPort1OpenFlag == True:
            self.Cmds_AMSSerial1Send("AT+IDLE")
            self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: AT+IDLE")
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.Cmds_AMSSerial2Send("AT+IDLE")
            self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: AT+IDLE")

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_MARetryInFila(self, gcmd):
        self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)Cmds_MARetryInFila]")


        self.G_IfChangeFilaOngoing= True


        #lancaigang250522: 
        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG109-heat up")
        command_string = """
            PG109
            """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.G_PhrozenFluiddRespondInfo("External macro command-PG109-; command_string='%s'" % command_string)
        self.IfDoPG102Flag=True


        # lancaigang231228: need waitstm32FA after, toolheaddetectto filament
        # set
        Lo_ChangeChannelIfSuccess = False
        # lancaigang250607: state; 1-unload filament; 2-feed; 3-; 4-pause
        self.G_KlipperPrintStatus= 2
        # lancaigang20231013: timeoutwhen
        # lancaigang231114: in printer.cfg set text file timeoutwhen, timeout
        # detect #2 time feedfilament is no to toolhead
        for i in range(CHANGE_CHANNEL_WAIT_TIMEOUT+50):# 130
            # self.G_XBasePosition+=2
            # self.G_YBasePosition+=2
            # lancaigang231202: ifSTM32 move reportpause, need toklipperpause
            if self.STM32ReprotPauseFlag==1:
                self.G_ChangeChannelFirstFilaFlag=True
                self.G_PhrozenFluiddRespondInfo('[INFO], stm32 move up report pause')
                Lo_ChangeChannelIfSuccess = False


                Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
                self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
                #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
                if Lo_PauseStatus['is_paused'] == True:
                    self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
                else:
                    self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")

                break
            
            #lancaigang231216: 
            if self.G_XBasePosition==0 and self.G_YBasePosition==0:
                self.G_PhrozenFluiddRespondInfo('[INFO], XY is 0')
            else:
                # lancaigang231216: resumewhen, need to move prevent
                # lancaigang231214: waitX YW H move move, purgeFunction
                command_string = """
                    G90
                    G1 X%.03f Y%.03f F1000
                    """ % (
                    self.G_XBasePosition+(i%2),
                    self.G_YBasePosition+(i%2)
                )
                # lancaigang231129: move move
                self.G_PhrozenGCode.run_script_from_command(command_string)
                self.G_PhrozenFluiddRespondInfo('[INFO], XY is P9 set')


            # lancaigang20231013: is 4when
            # lancaigang231115: is 1s
            self.G_ProzenToolhead.dwell(1)
            # lancaigang240222: cannot use use time.sleep, will errorcodedump
            #time.sleep(1)



            # self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG110; STM32feedafter, klipperstartpurgefeed")
            # command_string = """
            # PG110
            # """
            # self.G_PhrozenGCode.run_script_from_command(command_string)
            # self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)




            # detectnew channelfilamentfeed, is no hasfilamentto toolhead
            if self.G_ToolheadIfHaveFilaFlag:
                Lo_ChangeChannelIfSuccess = True
                break



        # normal normal
        if Lo_ChangeChannelIfSuccess==True:
            self.G_PhrozenFluiddRespondInfo('[INFO] successful')
            self.G_IfChangeFilaOngoing= False

            # lancaigang240108: toolheadhasfilament, canresume
            self.G_M2MAModeResumeFlag=True
            
            # lancaigang241106: successfulfeed
            self.G_P0M2MAStartPrintFlag=1

            # lancaigang250607: state; 1-unload filament; 2-feed; 3-; 4-pause
            self.G_KlipperPrintStatus= 3

            self.G_PauseToLCDString=""


            # #lancaigang250611: 
            # self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG108-heat uppurgewipe nozzle")
            # command_string = """
            #     PG108
            #     """
            # self.G_PhrozenGCode.run_script_from_command(command_string)

            #lancaigang250607:
            self.G_PhrozenFluiddRespondInfo('[INFO] canresume, STM32printing report error')
            self.G_KlipperQuickPause = True
            # #lancaigang250427: 
            # if self.G_SerialPort1OpenFlag == True:
            #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
            # self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-AMSfinishtimingbuffer-full time")
            # if self.G_SerialPort2OpenFlag == True:
            #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
            # self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-AMSfinishtimingbuffer-full time")
            # #self.G_ProzenToolhead.dwell(1.5)


            return
        
        self.G_PhrozenFluiddRespondInfo('[INFO] failed')
        # expire:timeoutwhen,
        # (default60)
        # A0:timeout,(default)
        # A1:timeout after stop
        # timeout
        # lancaigang20231013: A0:timeout
        if self.G_DictChangeChannelWaitAreaParam["A"] == 0:
            # lancaigang231209: stm32 move report then report9
            if self.G_KlipperIfPaused==False:
                self.G_PhrozenFluiddRespondInfo('[INFO] timeout100s, pause')
                
                #lancaigang250702: 
                if self.G_KlipperInPausing == False:
                    self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")

                    # lancaigang240104: do not send stm32pause
                    # klipper move pause
                    self.Cmds_PhrozenKlipperPauseM2M3ToSTM32(None)

                    #self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d" % self.G_ChangeChannelTimeoutNewChan)
                    self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                else:
                    self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")

            # lancaigang240123: ifalready is pausestate, not allowedthen pause
            else:
                self.G_PhrozenFluiddRespondInfo('[INFO] already pause, not allowedpause')

            # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
            self.G_ChangeChannelFirstFilaFlag=True
            self.G_IfChangeFilaOngoing= False

            # lancaigang241106: feedfailed
            self.G_P0M2MAStartPrintFlag=0

            # lancaigang250607: state; 1-unload filament; 2-feed; 3-; 4-pause
            self.G_KlipperPrintStatus= -1

            return
        
        # normal normal; Action normal normal
        if self.G_DictChangeChannelWaitAreaParam["A"] == 1:
            pass

        self.G_IfChangeFilaOngoing= False


    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # P114 S; no parameter number when state,has parameter number Swhen state after; "SB";
    # P114 S; no parameter number when state,has parameter number Swhen state after; "SD";
    def Cmds_CmdP114(self, gcmd):
        _ = gcmd

        if gcmd is None:
            self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP114]commandP114-None")
        if gcmd is not None:
            self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]command='%s'" % (gcmd.get_commandline(),))

            # get P114command parameter number
            params = gcmd.get_command_parameters()

        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
        self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)

        #lancaigang240510: 
        self.G_PhrozenFluiddRespondInfo("[INFO] +P114:0")
        
        self.G_PhrozenFluiddRespondInfo("[INFO] Current mode")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:0,unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:1,MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:2,MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo("[WARN] +Mode:3,RUNOUT")
        else:
            self.G_PhrozenFluiddRespondInfo("[ERROR] +Mode:-1,error")


        #Unlock
        self.Base_AMSSerialCmdUnlock()




        self.G_PhrozenFluiddRespondInfo("self.G_CancelFlag='%s'" % self.G_CancelFlag)
        #lancaigang250712:
        #self.Cmds_CmdP29(None)


        # lancaigang240511: resumewhen, initialize down serial port, preventAMSserial porterror
        try:
            self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.py)Cmds_CmdP114]Reinitializing serial port 1")
            self.G_SerialPort1Obj = serial.Serial(self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3)
            #Serial port opened successfully
            if self.G_SerialPort1Obj is not None:
                if self.G_SerialPort1Obj.is_open:
                    self.G_SerialPort1OpenFlag = True
                    self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 1successful")
                    # lancaigang231213: openserial port
                    self.G_SerialPort1Obj.flushInput()  # clean serial write cache
                    self.G_SerialPort1Obj.flush()
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 buffers cleared")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Re-registering serial port 1 callback")
                    self.G_SerialPort1RecvTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerUart1Recv, self.G_PhrozenReactor.NOW)
        except:
            self.G_PhrozenFluiddRespondInfo("[ERROR] Unable to open tty1. Check the USB connection or try rebooting.")

        try:
            self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.py)Cmds_CmdP114]Reinitializing serial port 2")
            self.G_SerialPort2Obj = serial.Serial(self.G_Serialport2Define, SERIAL_PORT_BAUD, timeout=3)
            #Serial port opened successfully
            if self.G_SerialPort2Obj is not None:
                if self.G_SerialPort2Obj.is_open:
                    self.G_SerialPort2OpenFlag = True
                    self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 2successful")
                    self.G_SerialPort2Obj.flushInput()  # clean serial write cache
                    self.G_SerialPort2Obj.flush()
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 buffers cleared")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Re-registering serial port 2 callback")
                    self.G_SerialPort2RecvTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerUart2Recv, self.G_PhrozenReactor.NOW)
        except:
            self.G_PhrozenFluiddRespondInfo("[DEBUG] AMS2/tty2 not available; skipping serial port 2.")


        # lancaigang240524: prevent; clearserial port number data
        self.G_ProzenToolhead.dwell(0.5)

        if self.G_SerialPort1OpenFlag==True:
            if self.G_SerialPort1Obj.is_open:
                #self.G_SerialPort1Obj.flushInput()  # clean serial write cache
                #self.G_SerialPort1Obj.flush()
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 is open")

        if self.G_SerialPort2OpenFlag==True:
            if self.G_SerialPort2Obj.is_open:
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 is open")

        # # parameter number is S, readmulti-materialmainboard parameter number
        # if "S" in params:
        #     # #ttyUSB0Send via serial and wait for a response
        #     # Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort1SendWaitRsp("SB", sizeof(AMSSimpleInfoBytes))
        #     # if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSSimpleInfoBytes):
        # # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]no command '%s'" % (gcmd.get_commandline(),))
        #     #     return

        #     # Lo_AMSDeviceStateInfo = AMSSimpleInfoBytes()
        #     # Lo_AMSDeviceStateInfo.whole[:] = bytearray(Lo_AMSDeviceStateRspInfo)
        # # #pythonempty
        #     # Lo_AMSSimpleState = {}
        #     # self.G_AMS1DeviceState["dev_id"] = Lo_AMSSimpleState["dev_id"] = Lo_AMSDeviceStateInfo.field.dev_id
        #     # self.G_AMS1DeviceState["dev_mode"] = Lo_AMSSimpleState["dev_mode"] = Lo_AMSDeviceStateInfo.field.dev_mode
        #     # self.G_AMS1DeviceState["mc_state"] = Lo_AMSSimpleState["mc_state"] = Lo_AMSDeviceStateInfo.field.mc_state
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_STANDBY:
        # # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]current statestate== machine ==%d" % MC_STANDBY)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_PREPARTION:
        # # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]current statestate====%d" % MC_STANDBY)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_CHANGING_P1:
        # # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]current statestate==1==%d" % MC_CHANGING_P1)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_CHANGING_P2:
        # # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]current statestate==2==%d" % MC_CHANGING_P2)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_FORCE_FEED:
        # # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]current statestate==refill==%d" % MC_FORCE_FEED)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_PRINTING:
        # # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]current statestate==refill==%d" % MC_PRINTING)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_ROLLBACK:
        # # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]current statestate== full unload filament==%d" % MC_ROLLBACK)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_PARKBACK:
        # # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]current statestate==unload filamentto park position==%d" % MC_PARKBACK)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_PARKALL:
        # # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]current statestate== full all unload filamentto park position==%d" % MC_PARKALL)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_CLEANING:
        # # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]current statestate==allfilamentclear==%d" % MC_CLEANING)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_ERR_TIMEOUT:
        # # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]current statestate==timeout output error state==%d" % MC_ERR_TIMEOUT)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_ERR_RUNOUT:
        # # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]current statestate==filament runout output error state==%d" % MC_ERR_RUNOUT)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_ERR_BLOCKUP:
        # # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]current statestate== output error state==%d" % MC_ERR_BLOCKUP)
        #     # self.G_AMS1DeviceState["ma_state"] = Lo_AMSSimpleState["ma_state"] = Lo_AMSDeviceStateInfo.field.ma_state
        # # # number data json switch
        #     # self.G_PhrozenFluiddRespondInfo(json.dumps(Lo_AMSSimpleState))

        #     self.G_PhrozenFluiddRespondInfo("[INFO] +P114:1")
        #     #self.G_P114RunFlag=0
        #     return
        
        # lancaigang250619:checkAMS is no re connectsuccessful
        #self.Cmds_USBConnectErrorCheck()

        # get multi-materialmainboardstate
        # lancaigang240430: stm32 will when report, use use withrsp, will time too close; not allowed send P114
        #Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort1SendWaitRsp("SD", sizeof(AMSDetailInfoBytes))
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]P114serial port receive : %s" % Lo_AMSDeviceStateRspInfo)
         # lancaigang240524: is send after receive
        if self.G_SerialPort1OpenFlag == True:
            self.Cmds_AMSSerial1Send("SD")
            self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: SD")

        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.Cmds_AMSSerial2Send("SD")
            self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: SD")


        # if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]AMS, please checkAMS '%s'" % (gcmd.get_commandline(),))
        #     #lancaigang240510: 
        #     self.G_PhrozenFluiddRespondInfo("[INFO] +P114:-1")
        #     #self.G_P114RunFlag=0
        # #lancaigang240412:AMSmulti-material
        #     self.G_AMSDevice1IfNormal=False
        #     return
        
        # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP114]AMShas")


        # #lancaigang240412:AMSmulti-material
        # self.G_AMSDevice1IfNormal=True

        # Lo_AMSDeviceStateInfo = AMSDetailInfoBytes()
        # Lo_AMSDeviceStateInfo.whole[:] = bytearray(Lo_AMSDeviceStateRspInfo)
        # #pythonempty
        # Lo_AMSDetailState = {}
        # self.G_AMSG_AMS1DeviceStateDeviceState["dev_id"] = Lo_AMSDetailState["dev_id"] = Lo_AMSDeviceStateInfo.field.dev_id
        # self.G_AMS1DeviceState["dev_mode"] = Lo_AMSDetailState["dev_mode"] = Lo_AMSDeviceStateInfo.field.dev_mode
        # #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]buffer device filamentstate(bool)==%d" % Lo_AMSDeviceStateInfo.field.cache_exist)
        # self.G_AMS1DeviceState["mc_state"] = Lo_AMSDetailState["mc_state"] = Lo_AMSDeviceStateInfo.field.mc_state
        # self.G_AMS1DeviceState["ma_state"] = Lo_AMSDetailState["ma_state"] = Lo_AMSDeviceStateInfo.field.ma_state
        # #lancaigang240524: use use, value -1
        # self.G_AMS1DeviceState["active_dev_id"] = Lo_AMSDetailState["active_dev_id"] = Lo_AMSDeviceStateInfo.field.dev_id
        # self.G_AMS1DeviceState["cache_empty"] = Lo_AMSDetailState["cache_empty"] = Lo_AMSDeviceStateInfo.field.cache_empty
        # #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]buffer device emptystate(bool)==%d" % Lo_AMSDeviceStateInfo.field.cache_empty)
        # self.G_AMS1DeviceState["cache_full"] = Lo_AMSDetailState["cache_full"] = Lo_AMSDeviceStateInfo.field.cache_full
        # #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]buffer device fullstate(bool)==%d" % Lo_AMSDeviceStateInfo.field.cache_full)
        # self.G_AMS1DeviceState["cache_exist"] = Lo_AMSDetailState["cache_exist"] = Lo_AMSDeviceStateInfo.field.cache_exist
        # self.G_AMS1DeviceState["entry_state"] = Lo_AMSDetailState["entry_state"] = Lo_AMSDeviceStateInfo.field.entry_state
        # #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]entry device state(bit)==%d" % Lo_AMSDeviceStateInfo.field.entry_state)
        # self.G_AMS1DeviceState["park_state"] = Lo_AMSDetailState["park_state"] = Lo_AMSDeviceStateInfo.field.park_state
        # #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]park position device state(bit)==%d" % Lo_AMSDeviceStateInfo.field.park_state)
        
        # # number data json switch
        # self.G_PhrozenFluiddRespondInfo(json.dumps(Lo_AMSDetailState))

        # lancaigang240123: prevent file readstm32
        #time.sleep(1)
        # lancaigang240229: cannot use time.sleep, will time to close
        #self.G_ProzenToolhead.dwell(0.5)
        #lancaigang240510: 
        #self.G_PhrozenFluiddRespondInfo("[INFO] +P114:1")
        #self.G_P114RunFlag=False

        self.G_P114RunFlag=1

        return


    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # P30 automaticID(use automatic network); "I"; logical automaticIDcommand
    def Cmds_CmdP30(self, gcmd):
        if not self.G_SerialPort1OpenFlag:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdP30]AMSmulti-material not connected, send P28 first')
            return
        
        self.G_PhrozenFluiddRespondInfo("command='%s'" % (gcmd.get_commandline(),))

        mcu_cmd = G_DictPhrozenCmdP30["mcu_cmd"][0] + "0"
        self.Cmds_AMSSerial1Send(mcu_cmd)
        self.G_PhrozenFluiddRespondInfo("Sending command: %s" % mcu_cmd)

        logging.info("SendCmd: %s" % mcu_cmd)

    


    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # P29 disconnectconnect
    def Cmds_CmdP29(self, gcmd):
        self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP29]command")

        try:
            if self.G_SerialPort1Obj is not None:
                if self.G_SerialPort1Obj.is_open:
                    # tty1close
                    self.G_SerialPort1Obj.close()
        except:
            self.G_PhrozenFluiddRespondInfo("[WARN] AMS1/tty1 not connected; send P28 first.")
        self.G_SerialPort1OpenFlag = False
        self.G_PhrozenFluiddRespondInfo('[INFO] AMS1clear')
        self.G_SerialPort1Obj=None

        try:
            if self.G_SerialPort2Obj is not None:
                if self.G_SerialPort2Obj.is_open:
                    self.G_SerialPort2Obj.close()
        except:
            self.G_PhrozenFluiddRespondInfo("[DEBUG] AMS2/tty2 not available; skipping serial port 2.")
        self.G_SerialPort2OpenFlag = False
        self.G_PhrozenFluiddRespondInfo('[INFO] AMS2clear')
        self.G_SerialPort2Obj=None


        if self.G_SerialPort1RecvTimmer:
            #Unregister
            self.G_PhrozenReactor.unregister_timer(self.G_SerialPort1RecvTimmer)
            # cleartimer
            self.G_SerialPort1RecvTimmer = None

        #lancaigang241030
        if self.G_SerialPort2RecvTimmer:
            #Unregister
            self.G_PhrozenReactor.unregister_timer(self.G_SerialPort2RecvTimmer)
            # cleartimer
            self.G_SerialPort2RecvTimmer = None

        #lancaigang250515: 
        self.G_P0M1MCNoneAMS=0
        self.G_PhrozenFluiddRespondInfo("[INFO] self.G_P0M1MCNoneAMS=0")



        # lancaigang231122: use use ttyafter, need to after unit IAPupdate sequence hdl_zigbee_gateway
        #os.system('sh /home/prz/klipper/klippy/extras/phrozen_dev/start.sh &')
        #self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP29]sh /home/prz/klipper/klippy/extras/phrozen_dev/start.sh &")

        #self.G_ProzenToolhead.dwell(1.0)



    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_GetImageId(self):
        self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)Cmds_GetImageId]")

        current_directory = os.getcwd()
        #current_directory=/home/mks/klipper
        #current_directory=/home/prz/klipper
        self.G_PhrozenFluiddRespondInfo("current_directory=%s" % (current_directory))

        # lancaigang250514: read set table
        try:
            self.G_PhrozenFluiddRespondInfo("[INFO] try")
            
            # openJSON text file and read internal
            #with open('.././hdlDat/ImageId.json', 'r', encoding='utf-8') as file:
            self.G_PhrozenFluiddRespondInfo("[INFO] /etc/ImageId.json")
            with open('/etc/ImageId.json', 'r', encoding='utf-8') as file:
                ImageData = file.read()
            self.G_PhrozenFluiddRespondInfo("[INFO] with open")
            #self.G_PhrozenFluiddRespondInfo("ImageData=%s" % (ImageData))
            # JSON number data
            json_data = json.loads(ImageData)
            #self.G_PhrozenFluiddRespondInfo("json_data=%s" % (json_data))
            self.G_PhrozenFluiddRespondInfo("json_data['ImageId']=%d" % (json_data['ImageId']))
            self.G_ImageId= json_data['ImageId']
            self.G_PhrozenFluiddRespondInfo("self.G_ImageId=%d" % (self.G_ImageId))
            self.G_PhrozenFluiddRespondInfo("json_data['HwId']=%d" % (json_data['HwId']))
            self.HwId= json_data['HwId']
            self.G_PhrozenFluiddRespondInfo("self.HwId=%d" % (self.HwId))
            self.G_PhrozenFluiddRespondInfo("json_data['FwId']=%d" % (json_data['FwId']))
            self.G_PhrozenFluiddRespondInfo("json_data['NC0']=%d" % (json_data['NC0']))
            self.G_PhrozenFluiddRespondInfo("json_data['NC1']=%d" % (json_data['NC1']))
            self.G_PhrozenFluiddRespondInfo("json_data['NC2']=%d" % (json_data['NC2']))
            self.G_PhrozenFluiddRespondInfo("json_data['NC3']=%d" % (json_data['NC3']))
            self.G_PhrozenFluiddRespondInfo("json_data['NC4']=%d" % (json_data['NC4']))

            if self.G_ImageId==16:
                self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==16: ARCO300-MKS-RK3328-STM32F407VET6-I16")
            elif self.G_ImageId==31:
                self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==31: ARCO300-phrozen-RK3308-STM32F407VET6-I31")
            elif self.G_ImageId==-1:
                self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==-1, default: ARCO300-MKS-RK3328-STM32F407VET6-I16")
            else:
                self.G_PhrozenFluiddRespondInfo("[INFO] Image IDcould not be read, default: ARCO300-MKS-RK3328-STM32F407VET6-I16")
        except Exception as e:
            self.G_PhrozenFluiddRespondInfo('[INFO] read get text file number data error')
            self.G_PhrozenFluiddRespondInfo("self.G_ImageId=%d" % (self.G_ImageId))
            #self.G_PhrozenFluiddRespondInfo("self.HwId=%d" % (self.HwId))


    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_GetUartScreenCfg(self):
        self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_GetUartScreenCfg]")


        # lancaigang250514: read set table
        try:
            self.G_PhrozenFluiddRespondInfo("[INFO] try")

            #lancaigang250724: Read the system image ID to distinguish product, mainboard, and firmware variants
            #lancaigang250724:Read image ID
            self.Cmds_GetImageId()
            if self.G_ImageId==16:
                self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==16: ARCO300-MKS-RK3328-STM32F407VET6-I16")
                # openJSON text file and read internal
                with open('/home/mks/klipper/klippy/extras/phrozen_dev/serial-screen/plr_print_precfg.json', 'r', encoding='utf-8') as file:
                    UartScreenCfgData = file.read()
            elif self.G_ImageId==31:
                self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==31: ARCO300-phrozen-RK3308-STM32F407VET6-I31")
                # openJSON text file and read internal
                with open('/home/prz/klipper/klippy/extras/phrozen_dev/serial-screen/plr_print_precfg.json', 'r', encoding='utf-8') as file:
                    UartScreenCfgData = file.read()
            elif self.G_ImageId==-1:
                self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==-1, default: ARCO300-MKS-RK3328-STM32F407VET6-I16")
                # openJSON text file and read internal
                with open('/home/mks/klipper/klippy/extras/phrozen_dev/serial-screen/plr_print_precfg.json', 'r', encoding='utf-8') as file:
                    UartScreenCfgData = file.read()
            else:
                self.G_PhrozenFluiddRespondInfo("[INFO] Image IDcould not be read, default: ARCO300-MKS-RK3328-STM32F407VET6-I16")
                # openJSON text file and read internal
                with open('/home/mks/klipper/klippy/extras/phrozen_dev/serial-screen/plr_print_precfg.json', 'r', encoding='utf-8') as file:
                    UartScreenCfgData = file.read()

            self.G_PhrozenFluiddRespondInfo("[INFO] with open")
            # {
            # 	"Auto_Replace_state":	0,
            # 	"Chroma_Kit_state":	0,
            # 	"Chroma_Kit_num":	4,
            # 	"Chroma_Kit_access":	{
            # 		"T0":	1,
            # 		"T1":	2,
            # 		"T2":	3,
            # 		"T3":	4,
            # 		"T4":	-1,
            # 		"T5":	-1,
            # 		"T6":	-1,
            # 		"T7":	-1,
            # 		"T8":	-1,
            # 		"T9":	-1,
            # 		"T10":	-1,
            # 		"T11":	-1,
            # 		"T12":	-1,
            # 		"T13":	-1,
            # 		"T14":	-1,
            # 		"T15":	-1
            # 	}
            # }
            # JSON number data
            json_data = json.loads(UartScreenCfgData)
            #print(json_data['Auto_Replace_state'])
            #print(json_data['Chroma_Kit_num'])
            self.G_PhrozenFluiddRespondInfo("json_data['Auto_Replace_state']=%d" % (json_data['Auto_Replace_state']))
            self.G_PhrozenFluiddRespondInfo("json_data['Chroma_Kit_num']=%d" % (json_data['Chroma_Kit_num']))
            self.G_PhrozenFluiddRespondInfo("json_data['Chroma_Kit_access']['T0']=%d" % (json_data['Chroma_Kit_access']['T0']))
            self.G_PhrozenFluiddRespondInfo("json_data['Chroma_Kit_access']['T1']=%d" % (json_data['Chroma_Kit_access']['T1']))
            self.G_PhrozenFluiddRespondInfo("json_data['Chroma_Kit_access']['T2']=%d" % (json_data['Chroma_Kit_access']['T2']))
            self.G_PhrozenFluiddRespondInfo("json_data['Chroma_Kit_access']['T3']=%d" % (json_data['Chroma_Kit_access']['T3']))
            self.G_PhrozenFluiddRespondInfo("json_data['Chroma_Kit_access']['T4']=%d" % (json_data['Chroma_Kit_access']['T4']))
            self.G_PhrozenFluiddRespondInfo("json_data['Chroma_Kit_access']['T5']=%d" % (json_data['Chroma_Kit_access']['T5']))
            self.G_PhrozenFluiddRespondInfo("json_data['Chroma_Kit_access']['T6']=%d" % (json_data['Chroma_Kit_access']['T6']))
            self.G_PhrozenFluiddRespondInfo("json_data['Chroma_Kit_access']['T7']=%d" % (json_data['Chroma_Kit_access']['T7']))
            self.G_PhrozenFluiddRespondInfo("json_data['Chroma_Kit_access']['T8']=%d" % (json_data['Chroma_Kit_access']['T8']))
            self.G_PhrozenFluiddRespondInfo("json_data['Chroma_Kit_access']['T9']=%d" % (json_data['Chroma_Kit_access']['T9']))
            self.G_PhrozenFluiddRespondInfo("json_data['Chroma_Kit_access']['T10']=%d" % (json_data['Chroma_Kit_access']['T10']))
            self.G_PhrozenFluiddRespondInfo("json_data['Chroma_Kit_access']['T11']=%d" % (json_data['Chroma_Kit_access']['T11']))
            self.G_PhrozenFluiddRespondInfo("json_data['Chroma_Kit_access']['T12']=%d" % (json_data['Chroma_Kit_access']['T12']))
            self.G_PhrozenFluiddRespondInfo("json_data['Chroma_Kit_access']['T13']=%d" % (json_data['Chroma_Kit_access']['T13']))
            self.G_PhrozenFluiddRespondInfo("json_data['Chroma_Kit_access']['T14']=%d" % (json_data['Chroma_Kit_access']['T14']))
            self.G_PhrozenFluiddRespondInfo("json_data['Chroma_Kit_access']['T15']=%d" % (json_data['Chroma_Kit_access']['T15']))


            self.G_AutoReplaceState = json_data['Auto_Replace_state']
            self.G_ChromaKitNum = json_data['Chroma_Kit_num']
            self.G_ChromaKitAccessT0 = json_data['Chroma_Kit_access']['T0']
            self.G_ChromaKitAccessT1 = json_data['Chroma_Kit_access']['T1']
            self.G_ChromaKitAccessT2 = json_data['Chroma_Kit_access']['T2']
            self.G_ChromaKitAccessT3 = json_data['Chroma_Kit_access']['T3']
            self.G_ChromaKitAccessT4 = json_data['Chroma_Kit_access']['T4']
            self.G_ChromaKitAccessT5 = json_data['Chroma_Kit_access']['T5']
            self.G_ChromaKitAccessT6 = json_data['Chroma_Kit_access']['T6']
            self.G_ChromaKitAccessT7 = json_data['Chroma_Kit_access']['T7']
            self.G_ChromaKitAccessT8 = json_data['Chroma_Kit_access']['T8']
            self.G_ChromaKitAccessT9 = json_data['Chroma_Kit_access']['T9']
            self.G_ChromaKitAccessT10 = json_data['Chroma_Kit_access']['T10']
            self.G_ChromaKitAccessT11 = json_data['Chroma_Kit_access']['T11']
            self.G_ChromaKitAccessT12 = json_data['Chroma_Kit_access']['T12']
            self.G_ChromaKitAccessT13 = json_data['Chroma_Kit_access']['T13']
            self.G_ChromaKitAccessT14 = json_data['Chroma_Kit_access']['T14']
            self.G_ChromaKitAccessT15 = json_data['Chroma_Kit_access']['T15']

            self.G_PhrozenFluiddRespondInfo("self.G_AutoReplaceState=%d" % (self.G_AutoReplaceState))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitNum=%d" % (self.G_ChromaKitNum))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT0=%d" % (self.G_ChromaKitAccessT0))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT1=%d" % (self.G_ChromaKitAccessT1))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT2=%d" % (self.G_ChromaKitAccessT2))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT3=%d" % (self.G_ChromaKitAccessT3))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT4=%d" % (self.G_ChromaKitAccessT4))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT5=%d" % (self.G_ChromaKitAccessT5))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT6=%d" % (self.G_ChromaKitAccessT6))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT7=%d" % (self.G_ChromaKitAccessT7))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT8=%d" % (self.G_ChromaKitAccessT8))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT9=%d" % (self.G_ChromaKitAccessT9))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT10=%d" % (self.G_ChromaKitAccessT10))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT11=%d" % (self.G_ChromaKitAccessT11))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT12=%d" % (self.G_ChromaKitAccessT12))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT13=%d" % (self.G_ChromaKitAccessT13))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT14=%d" % (self.G_ChromaKitAccessT14))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT15=%d" % (self.G_ChromaKitAccessT15))
            
        except:
            self.G_PhrozenFluiddRespondInfo('[INFO] number data error, but number data get')

####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_GetUartScreenCfgClear(self):
        self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_GetUartScreenCfgClear]")


        # lancaigang250514: read set table
        try:
            self.G_PhrozenFluiddRespondInfo("[INFO] try")
            # {
            # 	"Auto_Replace_state":	0,
            # 	"Chroma_Kit_state":	0,
            # 	"Chroma_Kit_num":	4,
            # 	"Chroma_Kit_access":	{
            # 		"T0":	1,
            # 		"T1":	2,
            # 		"T2":	3,
            # 		"T3":	4,
            # 		"T4":	-1,
            # 		"T5":	-1,
            # 		"T6":	-1,
            # 		"T7":	-1,
            # 		"T8":	-1,
            # 		"T9":	-1,
            # 		"T10":	-1,
            # 		"T11":	-1,
            # 		"T12":	-1,
            # 		"T13":	-1,
            # 		"T14":	-1,
            # 		"T15":	-1
            # 	}
            # }
            self.G_AutoReplaceState = -1
            self.G_ChromaKitNum = -1
            self.G_ChromaKitAccessT0 = -1
            self.G_ChromaKitAccessT1 = -1
            self.G_ChromaKitAccessT2 = -1
            self.G_ChromaKitAccessT3 = -1
            self.G_ChromaKitAccessT4 = -1
            self.G_ChromaKitAccessT5 = -1
            self.G_ChromaKitAccessT6 = -1
            self.G_ChromaKitAccessT7 = -1
            self.G_ChromaKitAccessT8 = -1
            self.G_ChromaKitAccessT9 = -1
            self.G_ChromaKitAccessT10 = -1
            self.G_ChromaKitAccessT11 = -1
            self.G_ChromaKitAccessT12 = -1
            self.G_ChromaKitAccessT13 = -1
            self.G_ChromaKitAccessT14 = -1
            self.G_ChromaKitAccessT15 = -1

            self.G_PhrozenFluiddRespondInfo("self.G_AutoReplaceState=%d" % (self.G_AutoReplaceState))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitNum=%d" % (self.G_ChromaKitNum))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT0=%d" % (self.G_ChromaKitAccessT0))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT1=%d" % (self.G_ChromaKitAccessT1))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT2=%d" % (self.G_ChromaKitAccessT2))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT3=%d" % (self.G_ChromaKitAccessT3))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT4=%d" % (self.G_ChromaKitAccessT4))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT5=%d" % (self.G_ChromaKitAccessT5))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT6=%d" % (self.G_ChromaKitAccessT6))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT7=%d" % (self.G_ChromaKitAccessT7))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT8=%d" % (self.G_ChromaKitAccessT8))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT9=%d" % (self.G_ChromaKitAccessT9))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT10=%d" % (self.G_ChromaKitAccessT10))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT11=%d" % (self.G_ChromaKitAccessT11))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT12=%d" % (self.G_ChromaKitAccessT12))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT13=%d" % (self.G_ChromaKitAccessT13))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT14=%d" % (self.G_ChromaKitAccessT14))
            self.G_PhrozenFluiddRespondInfo("self.G_ChromaKitAccessT15=%d" % (self.G_ChromaKitAccessT15))
            
        except:
            self.G_PhrozenFluiddRespondInfo('[INFO] serial port disable set number data error')

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # P0 M1; multi-materialmodemode(need connect external all) Yes; "MC";P0 M1;P28;P2 A1;
    # P0 M2; multi-material single-colorrefillmode(need connect external all); "MA";P0 M2;P28;P8;
    # P0 M3; single-colorfilament runoutmode;P0 M3;
    # P28 connect
    def Cmds_CmdP28(self, gcmd):
        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP28]command='%s'" % (gcmd.get_commandline(),))

        self.G_PhrozenFluiddRespondInfo("self.G_CancelFlag='%s'" % self.G_CancelFlag)
        # #lancaigang250712: 
        # self.G_CancelFlag=False
        # self.G_PhrozenFluiddRespondInfo("self.G_CancelFlag='%s'" % self.G_CancelFlag)


        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        #lancaigang250517: 
        #self.G_PhrozenPrinterCancelPauseResume.cmd_CLEAR_PAUSE
        # #cancelcancelcommand
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        #lancaigang250517: 
        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)

        # #lancaigang250807:cannotclearpausestate, # send P28
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CLEAR_PAUSE(None)
        # self.G_PhrozenFluiddRespondInfo("[WARN] clearpausestate")

        Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
        self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
        #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus['is_paused'] == True:
            self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
        else:
            self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")

        self.G_PhrozenFluiddRespondInfo("[INFO] Current mode")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:0,unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:1,MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:2,MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo("[WARN] +Mode:3,RUNOUT")
        else:
            self.G_PhrozenFluiddRespondInfo("[ERROR] +Mode:-1,error")


        #Unlock
        self.Base_AMSSerialCmdUnlock()

        # lancaigang250607: state; 1-unload filament; 2-feed; 3-; 4-pause
        if self.G_KlipperPrintStatus == 3:
            self.G_PhrozenFluiddRespondInfo('[INFO] printing, logical P28！！！')
            return

        #lancaigang250724:Read image ID
        self.Cmds_GetImageId()

        #lancaigang250514: Read the JSON file for monochrome refill settings and channel/color mappings
        #/home/prz/klipper/klippy/extras/phrozen_dev/serial-screen/test.json
        self.Cmds_GetUartScreenCfg()
        


        #lancaigang231220: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdP28]single-colormode, logical P28')
            self.G_PhrozenFluiddRespondInfo("V-H%s-I%s-F%s" % (HW_VERSION,IMAGE_VERSION,FW_VERSION))
            return

        #lancaigang250610: 
        if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_MA:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdP28]single-color refill modemode, logical P28')
            self.G_PhrozenFluiddRespondInfo("V-H%s-I%s-F%s" % (HW_VERSION,IMAGE_VERSION,FW_VERSION))
            return


        # #lancaigang231122: Before using ttyUSB0, stop the background IAP updater process hdl_zigbee_gateway
        # os.system('sh /home/prz/klipper/klippy/extras/phrozen_dev/stop.sh &')
        # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP28]sh /home/prz/klipper/klippy/extras/phrozen_dev/stop.sh &")

        #lancaigang231205: 
        self.G_KlipperIfPaused = False
        # lancaigang250526: pause, not allowednew gcodecommand, need waitpausecomplete
        self.G_KlipperInPausing = False
        #lancaigang250526: 
        self.G_IfToolheadHaveFilaInitiativePauseFlag=False
        # lancaigang240223: toolheadcut filamentfailed
        self.ToolheadCutFlag = False












        if self.G_SerialPort1Obj is not None:
            # lancaigang231219: ifserial portalready open, cannotthen down
            if self.G_SerialPort1Obj.is_open:
                self.G_PhrozenFluiddRespondInfo('[INFO] P28serial port 1already open')
                # self.G_PhrozenFluiddRespondInfo("V-H%s-I%s-F%s" % (HW_VERSION,IMAGE_VERSION,FW_VERSION))
                # #lancaigang240104: disable
                # self.G_PhrozenFluiddRespondInfo("[INFO] +AMSCONNECT:0")

                #self.G_SerialPort1Obj.flushInput()  # clean serial write cache
                #self.G_SerialPort1Obj.flush()
                #self.G_SerialPort1OpenFlag = True
                # lancaigang240524: is is None, serial porttimerregister
                #if self.G_SerialPort1RecvTimmer is None:
                # timerthread
                self.G_PhrozenFluiddRespondInfo("[INFO] Re-registering serial port 1 callback")
                self.G_Serial1PortRecvTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerUart1Recv, self.G_PhrozenReactor.NOW)

                # lancaigang240511: before after error, MA M0 MA, AMSrestart
                # if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
                # self.G_PhrozenFluiddRespondInfo("[INFO] Sending command: M0mode")
                #     self.Cmds_AMSSerial1Send("M0")

                # if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                # self.G_PhrozenFluiddRespondInfo("[INFO] Sending command: M0mode")
                #     self.Cmds_AMSSerial1Send("M0")

                self.G_ProzenToolhead.dwell(0.5)

                # lancaigang250619:checkAMS is no re connectsuccessful
                self.Cmds_USBConnectErrorCheck()
                if self.G_SerialPort1OpenFlag == True:
                    # # get multi-materialmainboardstate
                    # Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort1SendWaitRsp("SD", sizeof(AMSDetailInfoBytes))
                    # self.G_PhrozenFluiddRespondInfo("ttyserial port 1 receive : %s" % Lo_AMSDeviceStateRspInfo)
                    
                    # if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                    # self.G_PhrozenFluiddRespondInfo("AMS1, please checkAMS '%s'" % (gcmd.get_commandline(),))
                    # #lancaigang240412:AMSmulti-material
                    #     self.G_AMSDevice1IfNormal=False
                    # else:
                    # self.G_PhrozenFluiddRespondInfo("AMS1connectsuccessful '%s'" % (gcmd.get_commandline(),))
                    #     self.G_PhrozenFluiddRespondInfo("[INFO] self.G_AMSDevice1IfNormal=True")

                    # #lancaigang240412:AMSmulti-material
                    #     self.G_AMSDevice1IfNormal=True
                    self.Cmds_AMSSerial1Send("SD")
                    self.G_PhrozenFluiddRespondInfo("[INFO] SD")

                self.G_ProzenToolhead.dwell(2)


                # if self.G_SerialPort2Obj is not None:
                #     if self.G_SerialPort2Obj.is_open:
                # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2open, ")
                #     else:
                #         self.G_PhrozenFluiddRespondInfo("V-H%s-I%s-F%s" % (HW_VERSION,IMAGE_VERSION,FW_VERSION))
                #         self.G_PhrozenFluiddRespondInfo("[INFO] +AMSCONNECT:0")
                # #
                #         return
                self.G_SerialPortHaveOpenedCount=self.G_SerialPortHaveOpenedCount+1





        if self.G_SerialPort2Obj is not None:
            if self.G_SerialPort2Obj.is_open:
                self.G_PhrozenFluiddRespondInfo('[INFO] P28serial port 2already open')
                self.G_PhrozenFluiddRespondInfo("[INFO] Re-registering serial port 2 callback")
                self.G_SerialPort2RecvTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerUart2Recv, self.G_PhrozenReactor.NOW)
                
                #self.G_PhrozenFluiddRespondInfo("V-H%s-I%s-F%s" % (HW_VERSION,IMAGE_VERSION,FW_VERSION))
                #self.G_PhrozenFluiddRespondInfo("[INFO] +AMSCONNECT:0")

                self.G_ProzenToolhead.dwell(0.5)

                if self.G_SerialPort2OpenFlag == True:
                    # # get multi-materialmainboardstate
                    # Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort2SendWaitRsp("SD", sizeof(AMSDetailInfoBytes))
                    # self.G_PhrozenFluiddRespondInfo("ttyserial port 2 receive : %s" % Lo_AMSDeviceStateRspInfo)
                    # if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                    # self.G_PhrozenFluiddRespondInfo("AMS2, please checkAMS '%s'" % (gcmd.get_commandline(),))
                    #     self.G_AMSDevice2IfNormal=False
                    # else:
                    # self.G_PhrozenFluiddRespondInfo("AMS2connectsuccessful '%s'" % (gcmd.get_commandline(),))
                    #     self.G_AMSDevice2IfNormal=True
                    #     self.G_PhrozenFluiddRespondInfo("[INFO] self.G_AMSDevice2IfNormal=True")
                    self.Cmds_AMSSerial2Send("SD")
                    self.G_PhrozenFluiddRespondInfo("[INFO] SD")

                self.G_ProzenToolhead.dwell(2)

                self.G_SerialPortHaveOpenedCount=self.G_SerialPortHaveOpenedCount+1

                #return


        #lancaigang241030:
        if self.G_SerialPortHaveOpenedCount>0:
            self.G_PhrozenFluiddRespondInfo("has unit AMSalready openserial port='%d'" % (self.G_SerialPortHaveOpenedCount,))
            self.G_SerialPortHaveOpenedCount=0
            self.G_PhrozenFluiddRespondInfo("V-H%s-I%s-F%s" % (HW_VERSION,IMAGE_VERSION,FW_VERSION))
            self.G_PhrozenFluiddRespondInfo("[INFO] +AMSCONNECT:0")

            #lancaigang241031:
            if self.G_SerialPort1OpenFlag == True:
                # lancaigang240524: readAMSmainboardversion、16HUBmainboardversion
                self.Cmds_AMSSerial1Send("AT+SB=0")
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: AT+SB=0; get AMSmainboard、16HUBmainboard')
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                # lancaigang240524: readAMSmainboardversion、16HUBmainboardversion
                self.Cmds_AMSSerial2Send("AT+SB=0")
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2Sending command: AT+SB=0; get AMSmainboard、16HUBmainboard')

            self.G_PhrozenFluiddRespondInfo('[INFO] return')
            # 
            return


        # lancaigang240511: is 0.5, preventklipper move time too close
        time.sleep(0.5)
        
        # #lancaigang20231019: machine error, Automatic filament changeifdetected #1 channeltoolhead reserve up time filament, need to and allfilament
        # #lancaigang20231020: first detecttoolheadhas
        # #if self.G_ToolheadIfHaveFilaFlag:
        # # # 0=cut filament before default internal all gcode
        # lancaigang231128: G28 is PG28
        # if self.G_ChangeChannelIfZLiftingUpByGcode == 0:
        # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP28]allhomeandcut filament")
        #     command_string = """
        #     PG28
        #     """
        #     self.G_PhrozenGCode.run_script_from_command(command_string)
        # #lancaigang20231020: output head retractgcode, retract before need totoolhead, when compare, logical, Automatic filament change and cut filament
        #     # G92 E0
        #     # G1 E0.0000 F600
        #     # G91
        #     # G1 E-0.385 F8000
        # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP28]cut filament")
        # #lancaigang20231013: cut filament
        # self.Cmds_MoveToCutFilaAction(gcmd)
        # #self.G_PhrozenFluiddRespondInfo("[INFO] Sending command: AP, fully retract to the park position")
        # #// retract allto park position; //===== P2 A1 allfilamentto park position Yes; "AP";
        # #Klipper state: Shutdown
        # #!! Internal error on command:"P28"
        # #ifttyUSB0open send, klippr report error
        # #self.Cmds_AMSSerial1Send("AP")






        # lancaigang241030:serial port1
        try:
            # openttyserial port, 19200
            self.G_SerialPort1Obj = serial.Serial(self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3)
            #Serial port opened successfully
            if self.G_SerialPort1Obj.is_open:
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1 #1 time opensuccessful')
                # lancaigang231213: openserial port
                self.G_SerialPort1Obj.flushInput()  # clean serial write cache
                self.G_SerialPort1Obj.flush()
                self.G_SerialPort1OpenFlag = True
                # lancaigang240524: is is None, serial porttimerregister
                #if self.G_SerialPort1RecvTimmer is None:
                # timerthread
                self.G_SerialPort1RecvTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerUart1Recv, self.G_PhrozenReactor.NOW)

                # lancaigang240306: ifmode is M1-MC, then send MCto stm32
                if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
                    self.G_PhrozenFluiddRespondInfo("[INFO] AMS_WORK_MODE_MC; Sending command: M1-MC, MCmode")
                    self.Cmds_AMSSerial1Send("MC")

                # lancaigang241031: Unknown mode, then defaultMC
                if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                    self.G_PhrozenFluiddRespondInfo("[INFO] AMS_WORK_MODE_UNKNOW; Sending command: M1-MC, MCmode")
                    self.Cmds_AMSSerial1Send("MC")


                if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                    self.G_PhrozenFluiddRespondInfo("[INFO] AMS_WORK_MODE_MA; Sending command: M2-MA, MAmode")
                    self.Cmds_AMSSerial1Send("MA")

                if self.G_ToolheadIfHaveFilaFlag:
                    self.G_PhrozenFluiddRespondInfo('[INFO] toolhead up has filament')
                    # lancaigang240113: MCmode or Unknown mode AMS_WORK_MODE_UNKNOW,
                    if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
                        # lancaigang240319: cut filamentbefore move action
                        #self.Cmds_MoveToCutFilaPrepare()
                    #if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
                        self.G_PhrozenFluiddRespondInfo('[INFO] PG107; add before wipe nozzle')
                        command_string = """
                        PG107
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)
                        # lancaigang240323: cut filamentbeforefirst wipe nozzle
                        # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)]External macro command-PRZ_CZ; cut filamentbefore, first wipe nozzle")
                        # command_string = """
                        # PRZ_CZ
                        # """
                        # self.G_PhrozenGCode.run_script_from_command(command_string)
                        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)]command_string='%s'" % command_string)
                        # lancaigang231202: home/resetcut filament and
                        self.Cmds_MoveToCutFilaAndRollback(gcmd)
                    # lancaigang240104: single-colorM2MArefillmodecannotcut filament
                    #if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                    # #lancaigang231202: home/resetcut filament and
                    #    self.Cmds_MoveToCutFilaAndNotRollback(gcmd)

                    # when 20s, preventp28 after commandno logical
                    #time.sleep(20)
                    # raise gcmd.error("[(cmds.python)Cmds_CmdP28]AMSmulti-materialconnectfailed")
                
                self.G_ProzenToolhead.dwell(2)

                if self.G_SerialPort1OpenFlag == True:
                    # # get multi-materialmainboardstate
                    # Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort1SendWaitRsp("SD", sizeof(AMSDetailInfoBytes))
                    # self.G_PhrozenFluiddRespondInfo("tty1serial port receive : %s" % Lo_AMSDeviceStateRspInfo)
                    # if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                    # self.G_PhrozenFluiddRespondInfo("AMS1, please checkAMS '%s'" % (gcmd.get_commandline(),))
                    # #lancaigang240412:AMSmulti-material
                    #     self.G_AMSDevice1IfNormal=False
                    # else:
                    # #lancaigang240412:AMSmulti-material
                    #     self.G_AMSDevice1IfNormal=True
                    self.Cmds_AMSSerial1Send("SD")
                    self.G_PhrozenFluiddRespondInfo("[INFO] SD")

                self.G_ProzenToolhead.dwell(2)


                self.G_SerialPortIsOpenCount=self.G_SerialPortIsOpenCount+1

                # self.G_PhrozenFluiddRespondInfo("V-H%s-I%s-F%s" % (HW_VERSION,IMAGE_VERSION,FW_VERSION))
                # self.G_PhrozenFluiddRespondInfo("[INFO] +AMSCONNECT:0")

            else:
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1 #1 time openfailed')
                #self.G_PhrozenFluiddRespondInfo("[INFO] +AMSCONNECT:1")
                self.G_SerialPort1OpenFlag = False
                # gcmd.respond_info("Unable to connect to Phrozen devs")
                # lancaigang231207: 1-AMSmulti-materialconnectfailed
                # lancaigang231207: 2-AMSmulti-materialserial portttyopenfailed
                self.G_PhrozenFluiddRespondInfo("[ERROR] +AMSERROR:1")
                self.G_PhrozenFluiddRespondInfo("[INFO] AMS1multi-materialconnectedfailed")
                # [WARN] AMS1/tty1 not connected; send P28 first.
        except:
            self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1 #1 time openfailed')
            #self.G_PhrozenFluiddRespondInfo("[INFO] +AMSCONNECT:2")
            # gcmd.respond_info("Unable open USB serial port, Please check USB port connect first")
            # lancaigang231207: 1-AMSmulti-materialconnectfailed
            # lancaigang231207: 2-AMSmulti-materialserial portttyopenfailed
            self.G_PhrozenFluiddRespondInfo("[ERROR] +AMSERROR:2")
            self.G_PhrozenFluiddRespondInfo("[ERROR] Unable to open tty1. Check the USB connection or try rebooting.")
            # [WARN] AMS1/tty1 not connected; send P28 first.
        







        # lancaigang241030:serial port2
        try:
            # openttyserial port, 19200
            self.G_SerialPort2Obj = serial.Serial(self.G_Serialport2Define, SERIAL_PORT_BAUD, timeout=3)
            #Serial port opened successfully
            if self.G_SerialPort2Obj.is_open:
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2 #1 time opensuccessful')
                self.G_SerialPort2Obj.flushInput()  # clean serial write cache
                self.G_SerialPort2Obj.flush()
                self.G_SerialPort2OpenFlag = True
                self.G_SerialPort2RecvTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerUart2Recv, self.G_PhrozenReactor.NOW)

                self.G_ProzenToolhead.dwell(0.5)

                if self.G_SerialPort2OpenFlag == True:
                    # # get multi-materialmainboardstate
                    # Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort2SendWaitRsp("SD", sizeof(AMSDetailInfoBytes))
                    # self.G_PhrozenFluiddRespondInfo("tty2serial port receive : %s" % Lo_AMSDeviceStateRspInfo)
                    # if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                    # self.G_PhrozenFluiddRespondInfo("AMS2, please checkAMS '%s'" % (gcmd.get_commandline(),))
                    #     self.G_AMSDevice2IfNormal=False
                    # else:
                    # #lancaigang240412:AMSmulti-material
                    #     self.G_AMSDevice2IfNormal=True
                    self.Cmds_AMSSerial2Send("SD")
                    self.G_PhrozenFluiddRespondInfo("[INFO] SD")

                self.G_ProzenToolhead.dwell(2)

                self.G_SerialPortIsOpenCount=self.G_SerialPortIsOpenCount+1


                # self.G_PhrozenFluiddRespondInfo("V-H%s-I%s-F%s" % (HW_VERSION,IMAGE_VERSION,FW_VERSION))
                # self.G_PhrozenFluiddRespondInfo("[INFO] +AMSCONNECT:0")

            else:
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2 #1 time openfailed')
                #self.G_PhrozenFluiddRespondInfo("[INFO] +AMSCONNECT:1")
                self.G_SerialPort2OpenFlag = False
                self.G_PhrozenFluiddRespondInfo("[ERROR] +AMSERROR:1")
                self.G_PhrozenFluiddRespondInfo("[INFO] AMS2multi-materialconnectedfailed")
                # [DEBUG] AMS2/tty2 not available; skipping serial port 2.
        except:
            self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2 #1 time openfailed')
            #self.G_PhrozenFluiddRespondInfo("[INFO] +AMSCONNECT:2")
            self.G_PhrozenFluiddRespondInfo("[ERROR] +AMSERROR:2")
            self.G_PhrozenFluiddRespondInfo("[DEBUG] AMS2/tty2 not available; skipping serial port 2.")
            # [WARN] AMS1/tty1 not connected; send P28 first.




        # lancaigang241030: to successfulopen serial port, can use use multi-material
        if self.G_SerialPortIsOpenCount>0:
            self.G_PhrozenFluiddRespondInfo('successfulopenAMSAMShas unit =%d' % self.G_SerialPortIsOpenCount)
            self.G_SerialPortIsOpenCount=0
            
            self.G_PhrozenFluiddRespondInfo("V-H%s-I%s-F%s" % (HW_VERSION,IMAGE_VERSION,FW_VERSION))
            self.G_PhrozenFluiddRespondInfo("[INFO] +AMSCONNECT:0")

            #lancaigang241031:
            if self.G_SerialPort1OpenFlag == True:
                # lancaigang240524: readAMSmainboardversion、16HUBmainboardversion
                self.Cmds_AMSSerial1Send("AT+SB=0")
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: AT+SB=0; get AMSmainboard、16HUBmainboard')
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                # lancaigang240524: readAMSmainboardversion、16HUBmainboardversion
                self.Cmds_AMSSerial2Send("AT+SB=0")
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2Sending command: AT+SB=0; get AMSmainboard、16HUBmainboard')

        # if is 0, nosuccessfulopen serial port
        else:
            #self.G_PhrozenFluiddRespondInfo("[INFO] +AMSCONNECT:2")
            self.G_PhrozenFluiddRespondInfo("[ERROR] +AMSERROR:2")
            self.G_PhrozenFluiddRespondInfo('[INFO] opentty port, please checkUSB port or try rebooting')

            raise gcmd.error('has connectAMSAMS, connectAMSfailed')



    ####################################
    #Function Name: python
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20241101
    ####################################
    #lancaigang241101: 
    # P10 S? parameter number S[1,5]:purge time number, S1-purge1 time, S2-purge2 time ..., purge5 time
    def Cmds_CmdP10(self, gcmd):
        # get command parameter number
        params = gcmd.get_command_parameters()

        if self.G_KlipperIfPaused == True:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdP10]klipperpause, but received command')

        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP10]command='%s'" % (gcmd.get_commandline(),))

        self.G_PhrozenFluiddRespondInfo("command: '%s'" % (gcmd.get_commandline(),))

        if "S" in params:
            Lo_SpitNum = int(params["S"])
            if not Lo_SpitNum in [1, 2, 3,4,5,6,7,8,9]:
                raise gcmd.error("no parameter number command;cmd '%s', parameter number S need in [1/2/3/4/5/6/7/8/9]" % (gcmd.get_commandline(),))

            self.G_P10SpitNum=Lo_SpitNum




        #lancaigang250519:
        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PRZ_CUT_WAITINGAREA")
        command_string = """
            PRZ_CUT_WAITINGAREA
            """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.G_PhrozenFluiddRespondInfo("External macro command-to waiting areaposition; command_string='%s'" % command_string)






        self.G_PhrozenFluiddRespondInfo("purge time number : '%d'" % (self.G_P10SpitNum,))

    ####################################
    #Function Name: python
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20241101
    ####################################
    # P11 T?;multi-material cutter
    def Cmds_CmdP11(self, gcmd):
        if gcmd is None:
            self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP11]gcmd-None")
            self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP11]return")
            return
        if gcmd is not None:
            self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP11]command='%s'" % (gcmd.get_commandline(),))
        
        # get command parameter number
        params = gcmd.get_command_parameters()

        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP11]command='%s'; AMS cutter" % (gcmd.get_commandline(),))

        self.G_PhrozenFluiddRespondInfo("command: '%s'" % (gcmd.get_commandline(),))

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+P11:0,%d" % self.G_ChangeChannelTimeoutNewChan)

        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()
        
        # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
            #lancaigang241030:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")

            self.G_ProzenToolhead.dwell(2)

        # lancaigang231209: manual call do notpause
        self.G_KlipperIfPaused=False
        # lancaigang240221: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0
        self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")

        # lancaigang250805: cutter
        self.G_CutCheckTest=True
        self.ManualCmdFlag=False


        #if self.G_ToolheadIfHaveFilaFlag:
        self.G_PhrozenFluiddRespondInfo('[INFO] reset, cut filament; all AMSfirst')
        # lancaigang231205: home/resetcut filament
        self.Cmds_MoveToCutFilaAndHomingXY(gcmd)



        self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG104- get before full change variable')
        command_string = """
            PG104
            """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.G_PhrozenFluiddRespondInfo("External macro command-PG104- get before full change variable; command_string='%s'" % command_string)
        self.IfDoPG102Flag=True


        # lancaigang240510: before, first to waiting area
        # lancaigang240306: move move to cut filament code
        # lancaigang240110: waitwaitbefore, first External macro command, move move to positionwait
        # lancaigang240515: before, first first to to waiting area
        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG101-retract")
        command_string = """
            PG101
            """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.G_PhrozenFluiddRespondInfo("External macro command-to waiting areapositionpurge; command_string='%s'" % command_string)
        self.IfDoPG102Flag=True



        # lancaigang240319: after, first reserve toolheadfilament, preventsmall pellet
        self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG106; cut filament before, first reserve toolheadfilament, prevent stop small pellet')
        self.PG102Flag=True
        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
        command_string = """
        PG106
        """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)
        self.PG102Flag=False
        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=False")




        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()


        if self.G_SerialPort1OpenFlag == True:
            self.Cmds_AMSSerial1Send("AP")
            self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: AP")
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.Cmds_AMSSerial2Send("AP")
            self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: AP")

        self.G_ProzenToolhead.dwell(0.5)


        # lancaigang240913: set to external
        self.G_ProzenToolhead.dwell(6.0)
        # lancaigang231201: checkcut filament after old channelfilament is no normal normal unload filament, normal normal then pause
        self.Cmds_CutFilaIfNormalCheck()
        if self.G_KlipperIfPaused == True:
            self.G_PhrozenFluiddRespondInfo('[INFO] cut filament5toolheaddetectedfilament, cutter error, please check cutter, pauseklipperprinting')
            #Lo_ChangeChannelIfSuccess = False
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P11:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            # lancaigang250805: cutter
            self.G_CutCheckTest=False
            return


        #lancaigang231207: 
        if self.G_IfInFilaBlockFlag:
            self.G_PhrozenFluiddRespondInfo('[INFO] feed, first move P1 E?from toolhead up filament tube get output and move prz_resumeresume')
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P11:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            # lancaigang250805: cutter
            self.G_CutCheckTest=False
            return


        if "T" in params:
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P11 Tn:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
            self.G_ChangeChannelTimeoutOldGcmd=self.G_ChangeChannelTimeoutNewGcmd
            self.G_ChangeChannelTimeoutNewChan=int(params["T"])
            self.G_ChangeChannelTimeoutNewGcmd=gcmd


            self.G_P10SpitNum=1

            # lancaigang241030: is P1 C1to P1 C32, in 1to 32
            # unit 1: 1 2 3 4
            # unit 2: 5 6 7 8
            # #3 unit : 9 10 11 12
            # #4 unit : 13 14 15 16
            # #5 unit : 17 18 19 20
            # #6 unit : 21 22 23 24
            # #7 unit : 25 26 27 28
            # #8 unit : 29 30 31 32
            # manual
            self.Cmds_P1TnManualChangeChannel(int(params["T"]), gcmd)
            # lancaigang240524: use UIUX move
            #self.G_PhrozenFluiddRespondInfo("+P11 Tn:1,%d" % self.G_ChangeChannelTimeoutNewChan)


            self.Cmds_MoveToCutFilaAction(gcmd)

            #lancaigang250519:
            self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PRZ_CUT_WAITINGAREA")
            command_string = """
                PRZ_CUT_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command-to waiting areaposition; command_string='%s'" % command_string)


            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("AP")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: AP")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("AP")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: AP")

            self.G_ProzenToolhead.dwell(0.5)




            # lancaigang240913: set to external
            self.G_ProzenToolhead.dwell(6.0)
            # lancaigang231201: checkcut filament after old channelfilament is no normal normal unload filament, normal normal then pause
            self.Cmds_CutFilaIfNormalCheck()
            if self.G_KlipperIfPaused == True:
                self.G_PhrozenFluiddRespondInfo('[INFO] cut filament5toolheaddetectedfilament, cutter error, please check cutter, pauseklipperprinting')
                #Lo_ChangeChannelIfSuccess = False
                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P11:1,%d" % self.G_ChangeChannelTimeoutNewChan)
                # lancaigang250805: cutter
                self.G_CutCheckTest=False
                return




        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+P11:1,%d" % self.G_ChangeChannelTimeoutNewChan)
        # lancaigang250805: cutter
        self.G_CutCheckTest=False

    ####################################
    #Function Name: python
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20241101
    ####################################
    # P12 T?;multi-material cutter
    def Cmds_CmdP12(self, gcmd):
        if gcmd is None:
            self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP12]gcmd-None")
            self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP12]return")
            return
        if gcmd is not None:
            self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP12]command='%s'" % (gcmd.get_commandline(),))
        
        # get command parameter number
        params = gcmd.get_command_parameters()

        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP12]command='%s'; AMS cutter" % (gcmd.get_commandline(),))

        self.G_PhrozenFluiddRespondInfo("command: '%s'" % (gcmd.get_commandline(),))

        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+P12:0,%d" % self.G_ChangeChannelTimeoutNewChan)

        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()
        
        # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
            #lancaigang241030:
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")

            self.G_ProzenToolhead.dwell(2)

        # lancaigang231209: manual call do notpause
        self.G_KlipperIfPaused=False
        # lancaigang240221: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0
        self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")

        # lancaigang250805: cutter
        self.G_CutCheckTest=True
        self.ManualCmdFlag=False


        # #if self.G_ToolheadIfHaveFilaFlag:
        # self.G_PhrozenFluiddRespondInfo("[INFO] reset, cut filament; allAMSfirst ")
        # #lancaigang231205: home/resetcut filament
        # self.Cmds_MoveToCutFilaAndHomingXY(gcmd)



        self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG104- get before full change variable')
        command_string = """
            PG104
            """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.G_PhrozenFluiddRespondInfo("External macro command-PG104- get before full change variable; command_string='%s'" % command_string)
        self.IfDoPG102Flag=True


        # lancaigang240510: before, first to waiting area
        # lancaigang240306: move move to cut filament code
        # lancaigang240110: waitwaitbefore, first External macro command, move move to positionwait
        # lancaigang240515: before, first first to to waiting area
        self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG101-retract")
        command_string = """
            PG101
            """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.G_PhrozenFluiddRespondInfo("External macro command-to waiting areapositionpurge; command_string='%s'" % command_string)
        self.IfDoPG102Flag=True



        # lancaigang240319: after, first reserve toolheadfilament, preventsmall pellet
        self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG106; cut filament before, first reserve toolheadfilament, prevent stop small pellet')
        self.PG102Flag=True
        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
        command_string = """
        PG106
        """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)
        self.PG102Flag=False
        self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=False")




        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()


        if self.G_SerialPort1OpenFlag == True:
            self.Cmds_AMSSerial1Send("AP")
            self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: AP")
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.Cmds_AMSSerial2Send("AP")
            self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: AP")

        self.G_ProzenToolhead.dwell(0.5)


        # lancaigang240913: set to external
        self.G_ProzenToolhead.dwell(6.0)
        # lancaigang231201: checkcut filament after old channelfilament is no normal normal unload filament, normal normal then pause
        self.Cmds_CutFilaIfNormalCheck()
        if self.G_KlipperIfPaused == True:
            self.G_PhrozenFluiddRespondInfo('[INFO] cut filament5toolheaddetectedfilament, cutter error, please check cutter, pauseklipperprinting')
            #Lo_ChangeChannelIfSuccess = False
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P12:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            # lancaigang250805: cutter
            self.G_CutCheckTest=False
            return


        #lancaigang231207: 
        if self.G_IfInFilaBlockFlag:
            self.G_PhrozenFluiddRespondInfo('[INFO] feed, first move P1 E?from toolhead up filament tube get output and move prz_resumeresume')
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P12:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            # lancaigang250805: cutter
            self.G_CutCheckTest=False
            return


        if "T" in params:
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P12 Tn:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
            self.G_ChangeChannelTimeoutOldGcmd=self.G_ChangeChannelTimeoutNewGcmd
            self.G_ChangeChannelTimeoutNewChan=int(params["T"])
            self.G_ChangeChannelTimeoutNewGcmd=gcmd


            self.G_P10SpitNum=1

            # lancaigang241030: is P1 C1to P1 C32, in 1to 32
            # unit 1: 1 2 3 4
            # unit 2: 5 6 7 8
            # #3 unit : 9 10 11 12
            # #4 unit : 13 14 15 16
            # #5 unit : 17 18 19 20
            # #6 unit : 21 22 23 24
            # #7 unit : 25 26 27 28
            # #8 unit : 29 30 31 32
            # manual
            self.Cmds_P1TnManualChangeChannel(int(params["T"]), gcmd)
            # lancaigang240524: use UIUX move
            #self.G_PhrozenFluiddRespondInfo("+P12 Tn:1,%d" % self.G_ChangeChannelTimeoutNewChan)


            self.Cmds_MoveToCutFilaAction(gcmd)

            #lancaigang250519:
            self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PRZ_CUT_WAITINGAREA")
            command_string = """
                PRZ_CUT_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command-to waiting areaposition; command_string='%s'" % command_string)


            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("AP")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: AP")
            #lancaigang241030:
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("AP")
                self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: AP")

            self.G_ProzenToolhead.dwell(0.5)




            # lancaigang240913: set to external
            self.G_ProzenToolhead.dwell(6.0)
            # lancaigang231201: checkcut filament after old channelfilament is no normal normal unload filament, normal normal then pause
            self.Cmds_CutFilaIfNormalCheck()
            if self.G_KlipperIfPaused == True:
                self.G_PhrozenFluiddRespondInfo('[INFO] cut filament5toolheaddetectedfilament, cutter error, please check cutter, pauseklipperprinting')
                #Lo_ChangeChannelIfSuccess = False
                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P12:1,%d" % self.G_ChangeChannelTimeoutNewChan)
                # lancaigang250805: cutter
                self.G_CutCheckTest=False
                return




        # lancaigang240524: use UIUX move
        self.G_PhrozenFluiddRespondInfo("+P12:1,%d" % self.G_ChangeChannelTimeoutNewChan)
        # lancaigang250805: cutter
        self.G_CutCheckTest=False




    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    #'P9 X195.940 Y242.500 W3.010 H41.450 D?'
    # wait logical
    # P9 
    # X[x_pos] x_pos:waitX
    # Y[y_pos] y_pos:waitY
    # W[width] width:wait
    # H[height] height:wait
    # D[0/5] D?: keep

    # P9 
    # T[expire]
    # A[0/1];
    # expire:timeoutwhen,(default60)
    # A0:timeout,(default) A1:timeout after stop waittimeout and logical
    def Cmds_CmdP9(self, gcmd):
        # get command parameter number
        params = gcmd.get_command_parameters()

        if self.G_KlipperIfPaused == True:
            self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdP9]klipperpause, but received command')

        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP9]command='%s'" % (gcmd.get_commandline(),))

        # lancaigang20231016: P9 after parameter number XYWH; X; Y; Wwait; Hwait
        #'P9 X195.940 Y242.500 W3.010 H41.450 D0'
        for flag in "XYWH":
            if flag in params:
                self.G_DictChangeChannelWaitAreaParam[flag] = float(params[flag])

        self.G_PhrozenFluiddRespondInfo("command: '%s'" % (gcmd.get_commandline(),))

        # parameter number D # D0:X action move move Y number (default) D1:Y action move move X number wait
        #'P9 X195.940 Y242.500 W3.010 H41.450 D0'
        if "D" in params:
            direction = int(params["D"])
            # if not direction in [0, 10]:
            # raise gcmd.error("no wait set, D parameter number is [0/1] '%s'" % (gcmd.get_commandline(),))
            self.G_DictChangeChannelWaitAreaParam["D"] = direction

        #lancaigang241031: 
        self.G_PhrozenFluiddRespondInfo("P9 parameter number;self.G_DictChangeChannelWaitAreaParam[D]='%d'" % (self.G_DictChangeChannelWaitAreaParam["D"],))


        # parameter number T # expire:timeoutwhen,(default60)
        #'P9 X195.940 Y242.500 W3.010 H41.450 D0'
        if "T" in params:
            expire = int(params["T"])
            # lancaigang20231016: is 60
            if expire < 60:
                self.G_PhrozenFluiddRespondInfo("no timeoutwhen, is 60 internal '%s'" % (gcmd.get_commandline(),))
            self.G_DictChangeChannelWaitAreaParam["T"] = expire
            self.G_PhrozenFluiddRespondInfo('Sending command: expire=%d' % expire)

        # parameter number A# A0:timeout,(default) A1:timeout after stop waittimeout and logical
        #'P9 X195.940 Y242.500 W3.010 H41.450 D0'
        if "A" in params:
            action = int(params["A"])
            if not action in [0, 1]:
                self.G_PhrozenFluiddRespondInfo("no timeout logical, A parameter number is [0/1] '%s'" % (gcmd.get_commandline(),))
            self.G_DictChangeChannelWaitAreaParam["A"] = action

        # Python list table (List)
        # sequence list is Python number data 。 sequence list number - position, or, # is 0, # is 1, therefore class 。
        # Pythonhas6 sequence list internal set class type, but normal is list table and 。
        # sequence list can action,, add,, check。
        # therefore external, Pythonalready internal set sequence list length and and 。
        # list table is normal use Python number data class type, can action is internal value output 。
        # list table number data item need tohas class type
        # create list table, to set number data item use use 。 down show :
        # list1 = ['physics', 'chemistry', 1997, 2000]
        # list2 = [1, 2, 3, 4, 5 ]
        # list3 = ["a", "b", "c", "d"]
        # and, list table from 0。 list table can get 、。
        self.ChangeWaitMoveArea = []
        # defaultmm; cfg set or internal all default
        Lo_LineWidth = self.G_ChangeChannelWaitLineWidth  
        # wait
        Lo_WaitAreaWidth, Lo_WaitAreaHeight = abs(self.G_DictChangeChannelWaitAreaParam["W"]), abs(self.G_DictChangeChannelWaitAreaParam["H"])
        # waitX Y
        Lo_XBasePosition, Lo_YBasePosition = self.G_DictChangeChannelWaitAreaParam["X"], self.G_DictChangeChannelWaitAreaParam["Y"]
        #lancaigang231216
        self.G_XBasePosition=Lo_XBasePosition
        self.G_YBasePosition=Lo_YBasePosition

        # move move
        Lo_TotalMovingDist = (Lo_WaitAreaWidth * Lo_WaitAreaHeight / Lo_LineWidth)
        # ;# mm/s
        self.G_WaitAreaEachStepDist = min(Lo_TotalMovingDist / self.G_DictChangeChannelWaitAreaParam["T"], self.G_ChangeChannelWaitMaxMovementSpeed* self.G_MovementSpeedFactor) 

        # D0:X action move move Y number (default) D1:Y action move move X number wait
        if self.G_DictChangeChannelWaitAreaParam["D"] == 1:
            Lo_WaitAreaWidth, Lo_WaitAreaHeight = Lo_WaitAreaHeight, Lo_WaitAreaWidth

        if self.G_WaitAreaEachStepDist > Lo_WaitAreaWidth:
             # lancaigang231129: output wait
             self.G_PhrozenFluiddRespondInfo("no parameter number;cmd='%s', that less than minstep: %.03f"% (gcmd.get_commandline(), self.G_WaitAreaEachStepDist))

        # wait number data
        for index, y in enumerate(np.arange(0.0, Lo_WaitAreaHeight, Lo_LineWidth)):
            #
            if len(self.ChangeWaitMoveArea) >= self.G_DictChangeChannelWaitAreaParam["T"]:
                break
            if index % 2 == 0:
                for x in np.arange(0, Lo_WaitAreaWidth, self.G_WaitAreaEachStepDist):
                    if x < Lo_WaitAreaWidth - self.G_WaitAreaEachStepDist / 2:
                        self.ChangeWaitMoveArea.append([x, y, True])
                    else:
                        self.ChangeWaitMoveArea.append([Lo_WaitAreaWidth, y, True])
                        if y + Lo_LineWidth < Lo_WaitAreaHeight:
                            self.ChangeWaitMoveArea.append((Lo_WaitAreaWidth, y + Lo_LineWidth, False))
                        break
            else:
                for x in np.arange(Lo_WaitAreaWidth - self.G_WaitAreaEachStepDist, 0.0, -self.G_WaitAreaEachStepDist):
                    if x > self.G_WaitAreaEachStepDist / 2:
                        self.ChangeWaitMoveArea.append([x, y, True])
                    else:
                        self.ChangeWaitMoveArea.append([0, y, False])
                        break

        # D0:X action move move Y number (default) D1:Y action move move X number wait
        if self.G_DictChangeChannelWaitAreaParam["D"] == 1:
            self.ChangeWaitMoveArea = [[y, x, b] for [x, y, b] in self.ChangeWaitMoveArea]

        # W
        if self.G_DictChangeChannelWaitAreaParam["W"] < 0:
            self.ChangeWaitMoveArea = [[-x, y, b] for [x, y, b] in self.ChangeWaitMoveArea]

        # H
        if self.G_DictChangeChannelWaitAreaParam["H"] < 0:
            self.ChangeWaitMoveArea = [[x, -y, b] for [x, y, b] in self.ChangeWaitMoveArea]

        self.ChangeWaitMoveArea = [[x + Lo_XBasePosition, y + Lo_YBasePosition, b] for [x, y, b] in self.ChangeWaitMoveArea]


####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_CmdP0M3P8FA(self, AMSNum,gcmd):
        # if not self.G_SerialPort1OpenFlag:
        # self.G_PhrozenFluiddRespondInfo("[INFO] AMSmulti-material not connected, send P28 first")
        #     return
        
        self.G_ProzenToolhead.dwell(2.0)

        self.G_PhrozenFluiddRespondInfo('[(cmds.python)Cmds_CmdP0M3P8FA]command=P8FA' )

        Lo_MCUSTM32Cmd = G_DictPhrozenCmdP8["mcu_cmd"][0]
        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()
        #lancaigang241030:
        if AMSNum==1:
            self.Cmds_AMSSerial1Send("MA")
            self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1 send MA')
        elif AMSNum==2:
            self.Cmds_AMSSerial2Send("MA")
            self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2 send MA')

        # lancaigang240124: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0


        # lancaigang240123: iftoolheadhasfilament, do not send FAto stm32, waittoolheadcompleteafterpausethen send FA, and automaticresume
        if self.G_ToolheadIfHaveFilaFlag==False:
            self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, FA')
            # lancaigang240115:when 2, prevent
            #time.sleep(2)
            self.G_ProzenToolhead.dwell(2.0)

            #lancaigang241030:
            if AMSNum==1:
                self.Cmds_AMSSerial1Send("FA")
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1 send FA')
            elif AMSNum==2:
                self.Cmds_AMSSerial2Send("FA")
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2 send FA')

            # lancaigang231229: number, waitfeed
            self.Cmds_MARetryInFila(gcmd)
            # lancaigang240108: P8commanddo not logical resumecommand
            self.G_M2MAModeResumeFlag=False



        else:# toolheadhasfilament
            self.G_PhrozenFluiddRespondInfo("[INFO] toolheadhas filament, FB")
            #time.sleep(2)
            self.G_ProzenToolhead.dwell(2.0)

            #lancaigang241030:
            if AMSNum==1:
                self.Cmds_AMSSerial1Send("FB")
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1 send FB')
            elif AMSNum==2:
                self.Cmds_AMSSerial2Send("FB")
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2 send FB')

            self.G_M2MAModeResumeFlag=False

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_P8AMS1AutoSelectChannel(self):
        self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_P8AMS1AutoSelectChannel]")

        bitmask1=0b0001
        bitmask2=0b0010
        bitmask4=0b0100
        bitmask8=0b1000
        if self.G_AMS1DeviceState["entry_state"] == 0:
            if self.G_AMS1DeviceState["park_state"] & bitmask1 == 1:#0001
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T1')
                self.Cmds_AMSSerial1Send("T1")
                if self.G_ChangeChannelTimeoutOldChan<0 or self.G_ChangeChannelTimeoutNewChan<0:
                    self.G_ChangeChannelTimeoutOldChan=1
                else:
                    self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan=1
                self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
            elif self.G_AMS1DeviceState["park_state"] & bitmask2 == 2:#0010
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T2')
                self.Cmds_AMSSerial1Send("T2")
                if self.G_ChangeChannelTimeoutOldChan<0 or self.G_ChangeChannelTimeoutNewChan<0:
                    self.G_ChangeChannelTimeoutOldChan=2
                else:
                    self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan=2
                self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
            elif self.G_AMS1DeviceState["park_state"] & bitmask4 == 4:#0100
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T3')
                self.Cmds_AMSSerial1Send("T3")
                if self.G_ChangeChannelTimeoutOldChan<0 or self.G_ChangeChannelTimeoutNewChan<0:
                    self.G_ChangeChannelTimeoutOldChan=3
                else:
                    self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan=3
                self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,3")
            elif self.G_AMS1DeviceState["park_state"] & bitmask8 == 8:#1000
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T4')
                self.Cmds_AMSSerial1Send("T4")
                if self.G_ChangeChannelTimeoutOldChan<0 or self.G_ChangeChannelTimeoutNewChan<0:
                    self.G_ChangeChannelTimeoutOldChan=4
                else:
                    self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan=4
                self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,4")
            else:
                self.G_PhrozenFluiddRespondInfo("[INFO] nonefilament")
        elif self.G_AMS1DeviceState["entry_state"] & bitmask1 == 1:#0001
            self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T1')
            self.Cmds_AMSSerial1Send("T1")
            if self.G_ChangeChannelTimeoutOldChan<0 or self.G_ChangeChannelTimeoutNewChan<0:
                    self.G_ChangeChannelTimeoutOldChan=1
            else:
                self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
            self.G_ChangeChannelTimeoutNewChan=1
            self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        elif self.G_AMS1DeviceState["entry_state"] & bitmask2 == 2:#0010
            if self.G_AMS1DeviceState["park_state"] & bitmask1 == 1:#0001
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T1')
                self.Cmds_AMSSerial1Send("T1")
                if self.G_ChangeChannelTimeoutOldChan<0 or self.G_ChangeChannelTimeoutNewChan<0:
                    self.G_ChangeChannelTimeoutOldChan=1
                else:
                    self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan=1
                self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
            elif self.G_AMS1DeviceState["park_state"] & bitmask2 == 2:#0010
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T2')
                self.Cmds_AMSSerial1Send("T2")
                if self.G_ChangeChannelTimeoutOldChan<0 or self.G_ChangeChannelTimeoutNewChan<0:
                    self.G_ChangeChannelTimeoutOldChan=2
                else:
                    self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan=2
                self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
            else:
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T2')
                self.Cmds_AMSSerial1Send("T2")
                if self.G_ChangeChannelTimeoutOldChan<0 or self.G_ChangeChannelTimeoutNewChan<0:
                    self.G_ChangeChannelTimeoutOldChan=2
                else:
                    self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan=2
                self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
        elif self.G_AMS1DeviceState["entry_state"] & bitmask4 == 4:#0100
            if self.G_AMS1DeviceState["park_state"] & bitmask1 == 1:#0001
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T1')
                self.Cmds_AMSSerial1Send("T1")
                if self.G_ChangeChannelTimeoutOldChan<0 or self.G_ChangeChannelTimeoutNewChan<0:
                    self.G_ChangeChannelTimeoutOldChan=1
                else:
                    self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan=1
                self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
            elif self.G_AMS1DeviceState["park_state"] & bitmask2 == 2:#0010
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T2')
                self.Cmds_AMSSerial1Send("T2")
                if self.G_ChangeChannelTimeoutOldChan<0 or self.G_ChangeChannelTimeoutNewChan<0:
                    self.G_ChangeChannelTimeoutOldChan=2
                else:
                    self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan=2
                self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
            elif self.G_AMS1DeviceState["park_state"] & bitmask4 == 4:#0100
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T3')
                self.Cmds_AMSSerial1Send("T3")
                if self.G_ChangeChannelTimeoutOldChan<0 or self.G_ChangeChannelTimeoutNewChan<0:
                    self.G_ChangeChannelTimeoutOldChan=3
                else:
                    self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan=3
                self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,3")
            else:
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T3')
                self.Cmds_AMSSerial1Send("T3")
                if self.G_ChangeChannelTimeoutOldChan<0 or self.G_ChangeChannelTimeoutNewChan<0:
                    self.G_ChangeChannelTimeoutOldChan=3
                else:
                    self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan=3
                self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,3")
        elif self.G_AMS1DeviceState["entry_state"] & bitmask8 ==8:#1000
            if self.G_AMS1DeviceState["park_state"] & bitmask1 == 1:#0001
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T1')
                self.Cmds_AMSSerial1Send("T1")
                if self.G_ChangeChannelTimeoutOldChan<0 or self.G_ChangeChannelTimeoutNewChan<0:
                    self.G_ChangeChannelTimeoutOldChan=1
                else:
                    self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan=1
                self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
            elif self.G_AMS1DeviceState["park_state"] & bitmask2 == 2:#0010
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T2')
                self.Cmds_AMSSerial1Send("T2")
                if self.G_ChangeChannelTimeoutOldChan<0 or self.G_ChangeChannelTimeoutNewChan<0:
                    self.G_ChangeChannelTimeoutOldChan=2
                else:
                    self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan=2
                self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
            elif self.G_AMS1DeviceState["park_state"] & bitmask4 == 4:#0100
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T3')
                self.Cmds_AMSSerial1Send("T3")
                if self.G_ChangeChannelTimeoutOldChan<0 or self.G_ChangeChannelTimeoutNewChan<0:
                    self.G_ChangeChannelTimeoutOldChan=3
                else:
                    self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan=3
                self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,3")
            elif self.G_AMS1DeviceState["park_state"] & bitmask8 == 8:#1000
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T4')
                self.Cmds_AMSSerial1Send("T4")
                if self.G_ChangeChannelTimeoutOldChan<0 or self.G_ChangeChannelTimeoutNewChan<0:
                    self.G_ChangeChannelTimeoutOldChan=4
                else:
                    self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan=4
                self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,4")
            else:
                self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T4')
                self.Cmds_AMSSerial1Send("T4")
                if self.G_ChangeChannelTimeoutOldChan<0 or self.G_ChangeChannelTimeoutNewChan<0:
                    self.G_ChangeChannelTimeoutOldChan=4
                else:
                    self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan=4
                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,4")




    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # P8 automaticrefill Yes; "FA";
    def Cmds_CmdP8(self,gcmd):
        # if not self.G_SerialPort1OpenFlag:
        # self.G_PhrozenFluiddRespondInfo("[INFO] AMSmulti-material not connected, send P28 first")
        #     return
        # lancaigang250522: not allowedM3filament runoutdetect
        self.G_IfChangeFilaOngoing = True

        self.G_ProzenToolhead.dwell(2.0)

        self.G_PhrozenFluiddRespondInfo('[(cmds.python)Cmds_CmdP8]command=P8' )


        self.G_PhrozenFluiddRespondInfo("[INFO] Current mode")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:0,unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:1,MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:2,MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo("[WARN] +Mode:3,RUNOUT")
        else:
            self.G_PhrozenFluiddRespondInfo("[ERROR] +Mode:-1,error")



        Lo_MCUSTM32Cmd = G_DictPhrozenCmdP8["mcu_cmd"][0]


        # lancaigang240511: resumewhen, initialize down serial port, preventAMSserial porterror
        try:
            self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP8]Reinitializing serial port 1")
            self.G_SerialPort1Obj = serial.Serial(self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3)
            #Serial port opened successfully
            if self.G_SerialPort1Obj is not None:
                if self.G_SerialPort1Obj.is_open:
                    self.G_SerialPort1OpenFlag = True
                    self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 1successful")
                    # lancaigang231213: openserial port
                    self.G_SerialPort1Obj.flushInput()  # clean serial write cache
                    self.G_SerialPort1Obj.flush()
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 buffers cleared")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Re-registering serial port 1 callback")
                    self.G_SerialPort1RecvTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerUart1Recv, self.G_PhrozenReactor.NOW)
        except:
            self.G_PhrozenFluiddRespondInfo("[ERROR] Unable to open tty1. Check the USB connection or try rebooting.")

        try:
            self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.py)Cmds_PhrozenKlipperResume]Reinitializing serial port 2")
            self.G_SerialPort2Obj = serial.Serial(self.G_Serialport2Define, SERIAL_PORT_BAUD, timeout=3)
            #Serial port opened successfully
            if self.G_SerialPort2Obj is not None:
                if self.G_SerialPort2Obj.is_open:
                    self.G_SerialPort2OpenFlag = True
                    self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 2successful")
                    self.G_SerialPort2Obj.flushInput()  # clean serial write cache
                    self.G_SerialPort2Obj.flush()
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 buffers cleared")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Re-registering serial port 2 callback")
                    self.G_SerialPort2RecvTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerUart2Recv, self.G_PhrozenReactor.NOW)
        except:
            self.G_PhrozenFluiddRespondInfo("[DEBUG] AMS2/tty2 not available; skipping serial port 2.")







        #lancaigang241030:
        if self.G_SerialPort1OpenFlag == True:
            self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1 send MA')
            self.Cmds_AMSSerial1Send("MA")

        if self.G_SerialPort2OpenFlag == True:
            self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2 send MA')
            self.Cmds_AMSSerial2Send("MA")


        # lancaigang240124: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0

        # #lancaigang240123: iftoolheadhasfilament, do not send FAto stm32, waittoolheadcompleteafterpausethen send FA, and automaticresume
        # if self.G_ToolheadIfHaveFilaFlag==False:
        # self.G_PhrozenFluiddRespondInfo("[INFO] toolheadnofilament, FA")
        # #lancaigang240115:when 2, prevent
        #     #time.sleep(2)
        #     self.G_ProzenToolhead.dwell(2.0)

        #     #lancaigang241030:
        #     if self.G_SerialPort1OpenFlag == True:
        #         self.Cmds_AMSSerial1Send("FA")
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1 send FA")
        #     elif self.G_SerialPort2OpenFlag == True:
        #         self.Cmds_AMSSerial2Send("FA")
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2 send FA")

        # #lancaigang231229: number
        #     self.Cmds_MARetryInFila(gcmd)
        # #lancaigang240108: P8commanddo not logical resumecommand
        #     self.G_M2MAModeResumeFlag=False



        # else:#toolheadhasfilament
        # self.G_PhrozenFluiddRespondInfo("[INFO] toolheadhasfilament, FB")
        #     #time.sleep(2)
        #     self.G_ProzenToolhead.dwell(2.0)

        #     #lancaigang241030:
        #     if self.G_SerialPort1OpenFlag == True:
        #         self.Cmds_AMSSerial1Send("FB")
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1 send FB")
        #     elif self.G_SerialPort2OpenFlag == True:
        #         self.Cmds_AMSSerial2Send("FB")
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2 send FB")

        #     self.G_M2MAModeResumeFlag=False



        # lancaigang241105: ifrestart after, current toolheadfilament is AMS channel, first allchannel
        # lancaigang231205: home/resetcut filament
        self.Cmds_MoveToCutFilaAndRollback(gcmd)


        # #lancaigang231205: home/resetcut filament
        # self.Cmds_MoveToCutFilaAndNotRollback(gcmd)
        # self.G_PhrozenFluiddRespondInfo("[INFO] allAMSfirst ")
        # #lancaigang241030:
        # if self.G_SerialPort1OpenFlag == True:
        #     self.Cmds_AMSSerial1Send("AP")
        #     self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: AP")
        # #lancaigang241030:
        # if self.G_SerialPort2OpenFlag == True:
        #     self.Cmds_AMSSerial2Send("AP")
        #     self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: AP")
        # #lancaigang240913: set to external
        # self.G_ProzenToolhead.dwell(6.0)
        # #lancaigang231201: checkcut filament after old channelfilament is no normal normal unload filament, normal normal then pause
        # self.Cmds_CutFilaIfNormalCheck()


        if self.G_KlipperIfPaused == True:
            self.G_PhrozenFluiddRespondInfo('[INFO] cut filament5toolheaddetectedfilament, cutter error, please check cutter, pauseklipperprinting')
            #Lo_ChangeChannelIfSuccess = False
            self.G_PauseToLCDString="+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
            self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
            self.G_IfChangeFilaOngoing= False
            return


        # ifcut filament normal normal, then first # AMS # channelfeed
        if self.G_KlipperIfPaused == False:
            self.G_ProzenToolhead.dwell(2.0)

            if self.G_SerialPort1OpenFlag == True:
                try:
                    self.G_PhrozenFluiddRespondInfo("[DEBUG] try;Lo_AMSDeviceStateRspInfo")
                    # get multi-materialmainboardstate
                    Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort1SendWaitRsp("SD", sizeof(AMSDetailInfoBytes))
                    self.G_PhrozenFluiddRespondInfo('ttyserial port 1 receive : %s' % Lo_AMSDeviceStateRspInfo)
                    if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                        self.G_PhrozenFluiddRespondInfo('[INFO] AMS1, please checkAMS')
                        # lancaigang240412:AMSmulti-material
                        self.G_AMSDevice1IfNormal=False
                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] AMS1connectedsuccessful")
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.G_AMSDevice1IfNormal=True")
                        # lancaigang240412:AMSmulti-material
                        self.G_AMSDevice1IfNormal=True

                        Lo_AMSDeviceStateInfo = AMSDetailInfoBytes()
                        Lo_AMSDeviceStateInfo.whole[:] = bytearray(Lo_AMSDeviceStateRspInfo)
                        #self.G_PhrozenFluiddRespondInfo("%s" % SerialRxASCIIStr)
                        # pythonempty
                        Lo_AMSDetailState = {}
                        self.G_AMS1DeviceState["dev_id"] = Lo_AMSDetailState["dev_id"] = Lo_AMSDeviceStateInfo.field.dev_id
                        self.G_AMS1DeviceState["active_dev_id"] = Lo_AMSDetailState["active_dev_id"] = Lo_AMSDeviceStateInfo.field.active_dev_id
                        self.G_AMS1DeviceState["dev_mode"] = Lo_AMSDetailState["dev_mode"] = Lo_AMSDeviceStateInfo.field.dev_mode
                        self.G_AMS1DeviceState["cache_empty"] = Lo_AMSDetailState["cache_empty"] = Lo_AMSDeviceStateInfo.field.cache_empty
                        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]buffer device emptystate(bool)==%d" % Lo_AMSDeviceStateInfo.field.cache_empty)
                        self.G_AMS1DeviceState["cache_full"] = Lo_AMSDetailState["cache_full"] = Lo_AMSDeviceStateInfo.field.cache_full
                        self.G_PhrozenFluiddRespondInfo('buffer device fullstate(bool)==%d' % Lo_AMSDeviceStateInfo.field.cache_full)
                        self.G_AMS1DeviceState["cache_exist"] = Lo_AMSDetailState["cache_exist"] = Lo_AMSDeviceStateInfo.field.cache_exist
                        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]buffer device filamentstate(bool)==%d" % Lo_AMSDeviceStateInfo.field.cache_exist)
                        self.G_AMS1DeviceState["mc_state"] = Lo_AMSDetailState["mc_state"] = Lo_AMSDeviceStateInfo.field.mc_state
                        self.G_AMS1DeviceState["ma_state"] = Lo_AMSDetailState["ma_state"] = Lo_AMSDeviceStateInfo.field.ma_state
                        self.G_AMS1DeviceState["entry_state"] = Lo_AMSDetailState["entry_state"] = Lo_AMSDeviceStateInfo.field.entry_state
                        self.G_PhrozenFluiddRespondInfo('entry device state(bit)==%d' % Lo_AMSDeviceStateInfo.field.entry_state)
                        self.G_AMS1DeviceState["park_state"] = Lo_AMSDetailState["park_state"] = Lo_AMSDeviceStateInfo.field.park_state
                        self.G_PhrozenFluiddRespondInfo('park position device state(bit)==%d' % Lo_AMSDeviceStateInfo.field.park_state)
                except:
                    self.G_PhrozenFluiddRespondInfo("[DEBUG] except;Lo_AMSDeviceStateRspInfo")
                



            if self.G_SerialPort2OpenFlag == True:
                try:
                    self.G_PhrozenFluiddRespondInfo("[DEBUG] try;Lo_AMSDeviceStateRspInfo")
                    Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort2SendWaitRsp("SD", sizeof(AMSDetailInfoBytes))
                    self.G_PhrozenFluiddRespondInfo('ttyserial port 2 receive : %s' % Lo_AMSDeviceStateRspInfo)
                    if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                        self.G_PhrozenFluiddRespondInfo('[INFO] AMS2, please checkAMS')
                        # lancaigang240412:AMSmulti-material
                        self.G_AMSDevice2IfNormal=False
                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] AMS2connectedsuccessful")
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.G_AMSDevice2IfNormal=True")
                        # lancaigang240412:AMSmulti-material
                        self.G_AMSDevice2IfNormal=True

                        Lo_AMSDeviceStateInfo = AMSDetailInfoBytes()
                        Lo_AMSDeviceStateInfo.whole[:] = bytearray(Lo_AMSDeviceStateRspInfo)
                        # pythonempty
                        Lo_AMSDetailState = {}
                        self.G_AMS2DeviceState["dev_id"] = Lo_AMSDetailState["dev_id"] = Lo_AMSDeviceStateInfo.field.dev_id
                        self.G_AMS2DeviceState["active_dev_id"] = Lo_AMSDetailState["active_dev_id"] = Lo_AMSDeviceStateInfo.field.active_dev_id
                        self.G_AMS2DeviceState["dev_mode"] = Lo_AMSDetailState["dev_mode"] = Lo_AMSDeviceStateInfo.field.dev_mode
                        self.G_AMS2DeviceState["cache_empty"] = Lo_AMSDetailState["cache_empty"] = Lo_AMSDeviceStateInfo.field.cache_empty
                        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]buffer device emptystate(bool)==%d" % Lo_AMSDeviceStateInfo.field.cache_empty)
                        self.G_AMS2DeviceState["cache_full"] = Lo_AMSDetailState["cache_full"] = Lo_AMSDeviceStateInfo.field.cache_full
                        self.G_PhrozenFluiddRespondInfo('buffer device fullstate(bool)==%d' % Lo_AMSDeviceStateInfo.field.cache_full)
                        self.G_AMS2DeviceState["cache_exist"] = Lo_AMSDetailState["cache_exist"] = Lo_AMSDeviceStateInfo.field.cache_exist
                        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]buffer device filamentstate(bool)==%d" % Lo_AMSDeviceStateInfo.field.cache_exist)
                        self.G_AMS2DeviceState["mc_state"] = Lo_AMSDetailState["mc_state"] = Lo_AMSDeviceStateInfo.field.mc_state
                        self.G_AMS2DeviceState["ma_state"] = Lo_AMSDetailState["ma_state"] = Lo_AMSDeviceStateInfo.field.ma_state
                        self.G_AMS2DeviceState["entry_state"] = Lo_AMSDetailState["entry_state"] = Lo_AMSDeviceStateInfo.field.entry_state
                        self.G_PhrozenFluiddRespondInfo('entry device state(bit)==%d' % Lo_AMSDeviceStateInfo.field.entry_state)
                        self.G_AMS2DeviceState["park_state"] = Lo_AMSDetailState["park_state"] = Lo_AMSDeviceStateInfo.field.park_state
                        self.G_PhrozenFluiddRespondInfo('park position device state(bit)==%d' % Lo_AMSDeviceStateInfo.field.park_state)

                except:
                    self.G_PhrozenFluiddRespondInfo("[DEBUG] except;Lo_AMSDeviceStateRspInfo")




        self.G_ProzenToolhead.dwell(2.0)


        if self.G_AMSDevice1IfNormal==True:

            # lancaigang241106:first # AMS # channel
            if self.G_AMS1DeviceState["entry_state"] > 0 or self.G_AMS1DeviceState["park_state"] > 0:
                self.G_PhrozenFluiddRespondInfo('[INFO] #1 AMShas filament')
                # lancaigang250711: if disable colorchannel, use channelfirst;
                # =====M3mode
                if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:#M3 M2
                    #if self.G_ChromaKitNum>0:
                    self.G_PhrozenFluiddRespondInfo('[INFO] M3modesingle-color type, use multi-material printingsingle-color type;')
                    if self.G_ChromaKitAccessT0>0:
                        self.Cmds_AMSSerial1Send("T%d" % self.G_ChromaKitAccessT0)
                        self.G_PhrozenFluiddRespondInfo('serial port 1Sending command: T%d' % self.G_ChromaKitAccessT0)
                        if self.G_ChangeChannelTimeoutOldChan<0 or self.G_ChangeChannelTimeoutNewChan<0:
                            self.G_ChangeChannelTimeoutOldChan=self.G_ChromaKitAccessT0
                        else:
                            self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan=self.G_ChromaKitAccessT0
                    elif self.G_ChromaKitAccessT1>0:
                        self.Cmds_AMSSerial1Send("T%d" % self.G_ChromaKitAccessT1)
                        self.G_PhrozenFluiddRespondInfo('serial port 1Sending command: T%d' % self.G_ChromaKitAccessT1)
                        if self.G_ChangeChannelTimeoutOldChan<0 or self.G_ChangeChannelTimeoutNewChan<0:
                            self.G_ChangeChannelTimeoutOldChan=self.G_ChromaKitAccessT1
                        else:
                            self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan=self.G_ChromaKitAccessT1
                    elif self.G_ChromaKitAccessT2>0:
                        self.Cmds_AMSSerial1Send("T%d" % self.G_ChromaKitAccessT2)
                        self.G_PhrozenFluiddRespondInfo('serial port 1Sending command: T%d' % self.G_ChromaKitAccessT2)
                        if self.G_ChangeChannelTimeoutOldChan<0 or self.G_ChangeChannelTimeoutNewChan<0:
                            self.G_ChangeChannelTimeoutOldChan=self.G_ChromaKitAccessT2
                        else:
                            self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan=self.G_ChromaKitAccessT2
                    elif self.G_ChromaKitAccessT3>0:
                        self.Cmds_AMSSerial1Send("T%d" % self.G_ChromaKitAccessT3)
                        self.G_PhrozenFluiddRespondInfo('serial port 1Sending command: T%d' % self.G_ChromaKitAccessT3)
                        if self.G_ChangeChannelTimeoutOldChan<0 or self.G_ChangeChannelTimeoutNewChan<0:
                            self.G_ChangeChannelTimeoutOldChan=self.G_ChromaKitAccessT3
                        else:
                            self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan=self.G_ChromaKitAccessT3
                    else:
                        self.G_PhrozenFluiddRespondInfo('[INFO] M3modesingle-color type, use has multi-material printingsingle-color type; move printingsingle-color type')
                        self.Cmds_P8AMS1AutoSelectChannel()
                else:
                    self.G_PhrozenFluiddRespondInfo('[INFO] modesingle-color type, use has multi-material printingsingle-color type; move printingsingle-color type')
                    self.Cmds_P8AMS1AutoSelectChannel()
            else:
                self.G_PhrozenFluiddRespondInfo('[INFO] #1 AMSnonefilament')


        if self.G_AMSDevice2IfNormal==True:
            if self.G_AMS2DeviceState["entry_state"]>0:
                self.G_PhrozenFluiddRespondInfo('[INFO] #2 AMShas filament')

        self.Cmds_PrintMode(self.G_AMSDeviceWorkMode)

         # lancaigang231229: number, waitfeed
        self.Cmds_MARetryInFila(gcmd)

        # lancaigang240108: P8commanddo not logical resumecommand
        self.G_M2MAModeResumeFlag=False

####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_CmdP8Infila(self):
        #self.G_ProzenToolhead.dwell(2.0)

        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP8Infila]" )

        self.G_PhrozenFluiddRespondInfo("[INFO] Current mode")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:0,unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:1,MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:2,MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo("[WARN] +Mode:3,RUNOUT")
        else:
            self.G_PhrozenFluiddRespondInfo("[ERROR] +Mode:-1,error")

        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()
        #lancaigang241030:
        if self.G_SerialPort1OpenFlag == True:
            self.Cmds_AMSSerial1Send("MB")
            self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1 send MB')
        elif self.G_SerialPort2OpenFlag == True:
            self.Cmds_AMSSerial2Send("MB")
            self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2 send MB')

        # lancaigang240124: stm32 move report, canpause1 time
        self.STM32ReprotPauseFlag=0

        self.G_ProzenToolhead.dwell(2.5)

        if self.G_SerialPort1OpenFlag == True:
            try:
                self.G_PhrozenFluiddRespondInfo("[DEBUG] try;Lo_AMSDeviceStateRspInfo")
                # get multi-materialmainboardstate
                Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort1SendWaitRsp("SD", sizeof(AMSDetailInfoBytes))
                self.G_PhrozenFluiddRespondInfo('ttyserial port 1 receive : %s' % Lo_AMSDeviceStateRspInfo)
                if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                    self.G_PhrozenFluiddRespondInfo('[INFO] AMS1, please checkAMS')
                    # lancaigang240412:AMSmulti-material
                    self.G_AMSDevice1IfNormal=False
                else:
                    self.G_PhrozenFluiddRespondInfo("[INFO] AMS1connectedsuccessful")
                    self.G_PhrozenFluiddRespondInfo("[INFO] self.G_AMSDevice1IfNormal=True")
                    # lancaigang240412:AMSmulti-material
                    self.G_AMSDevice1IfNormal=True

                    Lo_AMSDeviceStateInfo = AMSDetailInfoBytes()
                    Lo_AMSDeviceStateInfo.whole[:] = bytearray(Lo_AMSDeviceStateRspInfo)
                    #self.G_PhrozenFluiddRespondInfo("%s" % SerialRxASCIIStr)
                    # pythonempty
                    Lo_AMSDetailState = {}
                    self.G_AMS1DeviceState["dev_id"] = Lo_AMSDetailState["dev_id"] = Lo_AMSDeviceStateInfo.field.dev_id
                    self.G_AMS1DeviceState["active_dev_id"] = Lo_AMSDetailState["active_dev_id"] = Lo_AMSDeviceStateInfo.field.active_dev_id
                    self.G_AMS1DeviceState["dev_mode"] = Lo_AMSDetailState["dev_mode"] = Lo_AMSDeviceStateInfo.field.dev_mode
                    self.G_AMS1DeviceState["cache_empty"] = Lo_AMSDetailState["cache_empty"] = Lo_AMSDeviceStateInfo.field.cache_empty
                    # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]buffer device emptystate(bool)==%d" % Lo_AMSDeviceStateInfo.field.cache_empty)
                    self.G_AMS1DeviceState["cache_full"] = Lo_AMSDetailState["cache_full"] = Lo_AMSDeviceStateInfo.field.cache_full
                    self.G_PhrozenFluiddRespondInfo('buffer device fullstate(bool)==%d' % Lo_AMSDeviceStateInfo.field.cache_full)
                    self.G_AMS1DeviceState["cache_exist"] = Lo_AMSDetailState["cache_exist"] = Lo_AMSDeviceStateInfo.field.cache_exist
                    # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]buffer device filamentstate(bool)==%d" % Lo_AMSDeviceStateInfo.field.cache_exist)
                    self.G_AMS1DeviceState["mc_state"] = Lo_AMSDetailState["mc_state"] = Lo_AMSDeviceStateInfo.field.mc_state
                    self.G_AMS1DeviceState["ma_state"] = Lo_AMSDetailState["ma_state"] = Lo_AMSDeviceStateInfo.field.ma_state
                    self.G_AMS1DeviceState["entry_state"] = Lo_AMSDetailState["entry_state"] = Lo_AMSDeviceStateInfo.field.entry_state
                    self.G_PhrozenFluiddRespondInfo('entry device state(bit)==%d' % Lo_AMSDeviceStateInfo.field.entry_state)
                    self.G_AMS1DeviceState["park_state"] = Lo_AMSDetailState["park_state"] = Lo_AMSDeviceStateInfo.field.park_state
                    self.G_PhrozenFluiddRespondInfo('park position device state(bit)==%d' % Lo_AMSDeviceStateInfo.field.park_state)
            except:
                self.G_PhrozenFluiddRespondInfo("[DEBUG] except;Lo_AMSDeviceStateRspInfo")




        if self.G_SerialPort2OpenFlag == True:
            try:
                self.G_PhrozenFluiddRespondInfo("[DEBUG] try;Lo_AMSDeviceStateRspInfo")
                Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort2SendWaitRsp("SD", sizeof(AMSDetailInfoBytes))
                self.G_PhrozenFluiddRespondInfo('ttyserial port 2 receive : %s' % Lo_AMSDeviceStateRspInfo)
                if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                    self.G_PhrozenFluiddRespondInfo('[INFO] AMS2, please checkAMS')
                    # lancaigang240412:AMSmulti-material
                    self.G_AMSDevice2IfNormal=False
                else:
                    self.G_PhrozenFluiddRespondInfo("[INFO] AMS2connectedsuccessful")
                    self.G_PhrozenFluiddRespondInfo("[INFO] self.G_AMSDevice2IfNormal=True")
                    # lancaigang240412:AMSmulti-material
                    self.G_AMSDevice2IfNormal=True

                    Lo_AMSDeviceStateInfo = AMSDetailInfoBytes()
                    Lo_AMSDeviceStateInfo.whole[:] = bytearray(Lo_AMSDeviceStateRspInfo)
                    # pythonempty
                    Lo_AMSDetailState = {}
                    self.G_AMS2DeviceState["dev_id"] = Lo_AMSDetailState["dev_id"] = Lo_AMSDeviceStateInfo.field.dev_id
                    self.G_AMS2DeviceState["active_dev_id"] = Lo_AMSDetailState["active_dev_id"] = Lo_AMSDeviceStateInfo.field.active_dev_id
                    self.G_AMS2DeviceState["dev_mode"] = Lo_AMSDetailState["dev_mode"] = Lo_AMSDeviceStateInfo.field.dev_mode
                    self.G_AMS2DeviceState["cache_empty"] = Lo_AMSDetailState["cache_empty"] = Lo_AMSDeviceStateInfo.field.cache_empty
                    # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]buffer device emptystate(bool)==%d" % Lo_AMSDeviceStateInfo.field.cache_empty)
                    self.G_AMS2DeviceState["cache_full"] = Lo_AMSDetailState["cache_full"] = Lo_AMSDeviceStateInfo.field.cache_full
                    self.G_PhrozenFluiddRespondInfo('buffer device fullstate(bool)==%d' % Lo_AMSDeviceStateInfo.field.cache_full)
                    self.G_AMS2DeviceState["cache_exist"] = Lo_AMSDetailState["cache_exist"] = Lo_AMSDeviceStateInfo.field.cache_exist
                    # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]buffer device filamentstate(bool)==%d" % Lo_AMSDeviceStateInfo.field.cache_exist)
                    self.G_AMS2DeviceState["mc_state"] = Lo_AMSDetailState["mc_state"] = Lo_AMSDeviceStateInfo.field.mc_state
                    self.G_AMS2DeviceState["ma_state"] = Lo_AMSDetailState["ma_state"] = Lo_AMSDeviceStateInfo.field.ma_state
                    self.G_AMS2DeviceState["entry_state"] = Lo_AMSDetailState["entry_state"] = Lo_AMSDeviceStateInfo.field.entry_state
                    self.G_PhrozenFluiddRespondInfo('entry device state(bit)==%d' % Lo_AMSDeviceStateInfo.field.entry_state)
                    self.G_AMS2DeviceState["park_state"] = Lo_AMSDetailState["park_state"] = Lo_AMSDeviceStateInfo.field.park_state
                    self.G_PhrozenFluiddRespondInfo('park position device state(bit)==%d' % Lo_AMSDeviceStateInfo.field.park_state)
            except:
                self.G_PhrozenFluiddRespondInfo("[DEBUG] except;Lo_AMSDeviceStateRspInfo")
                
        # if self.G_AMSDevice1IfNormal==True:
        # #lancaigang241106:first # AMS # channel
        #     if self.G_AMS1DeviceState["entry_state"] > 0 or self.G_AMS1DeviceState["park_state"] > 0:
        # self.G_PhrozenFluiddRespondInfo("[INFO] #1 AMShasfilament")
        #         # if self.G_AMS1DeviceState["entry_state"]==0 or self.G_AMS1DeviceState["park_state"]==0:
        # # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #         #     self.Cmds_AMSSerial1Send("T1")
        #         #     self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #         #     self.G_ChangeChannelTimeoutNewChan=1
        # # #lancaigang240524: use UIUX move
        #         #     self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #         if self.G_AMS1DeviceState["entry_state"]==1:#0001
        #         #if self.G_AMS1DeviceState["park_state"]==1:#0001
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #             self.Cmds_AMSSerial1Send("T1")
        #             self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #             self.G_ChangeChannelTimeoutNewChan=1
        #             self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #         elif self.G_AMS1DeviceState["entry_state"]==2:#0010
        #             if self.G_AMS1DeviceState["park_state"]==1:#0001
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==2:#0010
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
        #             else:
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
        #         elif self.G_AMS1DeviceState["entry_state"]==3:#0011
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #             self.Cmds_AMSSerial1Send("T1")
        #             self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #             self.G_ChangeChannelTimeoutNewChan=1
        # #lancaigang240524: use UIUX move
        #             self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #         elif self.G_AMS1DeviceState["entry_state"]==4:#0100
        #             if self.G_AMS1DeviceState["park_state"]==1:#0001
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==2:#0010
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
        #             elif self.G_AMS1DeviceState["park_state"]==3:#0011
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==4:#0100
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T3")
        #                 self.Cmds_AMSSerial1Send("T3")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=3
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,3")
        #             elif self.G_AMS1DeviceState["park_state"]==5:#0101
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==6:#0110
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
        #             elif self.G_AMS1DeviceState["park_state"]==7:#0111
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #             else:
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T3")
        #                 self.Cmds_AMSSerial1Send("T3")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=3
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,3")
        #         elif self.G_AMS1DeviceState["entry_state"]==5:#0101
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #             self.Cmds_AMSSerial1Send("T1")
        #             self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #             self.G_ChangeChannelTimeoutNewChan=1
        # #lancaigang240524: use UIUX move
        #             self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #         elif self.G_AMS1DeviceState["entry_state"]==6:#0110
        #             if self.G_AMS1DeviceState["park_state"]==1:#0001
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==2:#0010
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
        #             else:
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        # #lancaigang240524: use UIUX move
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
        #         elif self.G_AMS1DeviceState["entry_state"]==7:#0111
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #             self.Cmds_AMSSerial1Send("T1")
        #             self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #             self.G_ChangeChannelTimeoutNewChan=1
        # #lancaigang240524: use UIUX move
        #             self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #         elif self.G_AMS1DeviceState["entry_state"]==8:#1000
        #             if self.G_AMS1DeviceState["park_state"]==1:#0001
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==2:#0010
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
        #             elif self.G_AMS1DeviceState["park_state"]==3:#0011
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==4:#0100
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T3")
        #                 self.Cmds_AMSSerial1Send("T3")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=3
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,3")
        #             elif self.G_AMS1DeviceState["park_state"]==5:#0101
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==6:#0110
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
        #             elif self.G_AMS1DeviceState["park_state"]==7:#0111
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==8:#1000
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T4")
        #                 self.Cmds_AMSSerial1Send("T4")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=4
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,4")
        #             elif self.G_AMS1DeviceState["park_state"]==9:#1001
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==10:#1010
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
        #             elif self.G_AMS1DeviceState["park_state"]==11:#1011
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==12:#1100
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T3")
        #                 self.Cmds_AMSSerial1Send("T3")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=3
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,3")
        #             elif self.G_AMS1DeviceState["park_state"]==13:#1101
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==14:#1110
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
        #             elif self.G_AMS1DeviceState["park_state"]==15:#1111
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #             else:
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T4")
        #                 self.Cmds_AMSSerial1Send("T4")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=4
        # #lancaigang240524: use UIUX move
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,4")
        #         elif self.G_AMS1DeviceState["entry_state"]==9:#1001
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #             self.Cmds_AMSSerial1Send("T1")
        #             self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #             self.G_ChangeChannelTimeoutNewChan=1
        # #lancaigang240524: use UIUX move
        #             self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #         elif self.G_AMS1DeviceState["entry_state"]==10:#1010
        #             if self.G_AMS1DeviceState["park_state"]==1:#0001
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==2:#0010
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
        #             else:
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        # #lancaigang240524: use UIUX move
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
        #         elif self.G_AMS1DeviceState["entry_state"]==11:#1011
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #             self.Cmds_AMSSerial1Send("T1")
        #             self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #             self.G_ChangeChannelTimeoutNewChan=1
        # #lancaigang240524: use UIUX move
        #             self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #         elif self.G_AMS1DeviceState["entry_state"]==12:#1100
        #             if self.G_AMS1DeviceState["park_state"]==1:#0001
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==2:#0010
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
        #             elif self.G_AMS1DeviceState["park_state"]==3:#0011
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==4:#0100
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T3")
        #                 self.Cmds_AMSSerial1Send("T3")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=3
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,3")
        #             elif self.G_AMS1DeviceState["park_state"]==5:#0101
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==6:#0110
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
        #             elif self.G_AMS1DeviceState["park_state"]==7:#0111
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #             else:
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T3")
        #                 self.Cmds_AMSSerial1Send("T3")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=3
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,3")
        #         elif self.G_AMS1DeviceState["entry_state"]==13:#1101
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #             self.Cmds_AMSSerial1Send("T1")
        #             self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #             self.G_ChangeChannelTimeoutNewChan=1
        # #lancaigang240524: use UIUX move
        #             self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #         elif self.G_AMS1DeviceState["entry_state"]==14:#1110
        #             if self.G_AMS1DeviceState["park_state"]==1:#0001
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==2:#0010
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
        #             else:
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        # #lancaigang240524: use UIUX move
        #                 self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
        #         elif self.G_AMS1DeviceState["entry_state"]==15:#1111
        # self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: T1")
        #             self.Cmds_AMSSerial1Send("T1")
        #             self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #             self.G_ChangeChannelTimeoutNewChan=1
        # #lancaigang240524: use UIUX move
        #             self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
        if self.G_AMSDevice1IfNormal==True:
            bitmask1=0b0001
            bitmask2=0b0010
            bitmask4=0b0100
            bitmask8=0b1000
            # lancaigang241106:first # AMS # channel
            if self.G_AMS1DeviceState["entry_state"] > 0 or self.G_AMS1DeviceState["park_state"] > 0:
                self.G_PhrozenFluiddRespondInfo('[INFO] #1 AMShas filament')
                if self.G_AMS1DeviceState["entry_state"] == 0:
                    if self.G_AMS1DeviceState["park_state"] & bitmask1 == 1:#0001
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T1')
                        self.Cmds_AMSSerial1Send("T1")
                        self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan=1
                        self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
                    elif self.G_AMS1DeviceState["park_state"] & bitmask2 == 2:#0010
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T2')
                        self.Cmds_AMSSerial1Send("T2")
                        self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan=2
                        self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
                    elif self.G_AMS1DeviceState["park_state"] & bitmask4 == 4:#0100
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T3')
                        self.Cmds_AMSSerial1Send("T3")
                        self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan=3
                        self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,3")
                    elif self.G_AMS1DeviceState["park_state"] & bitmask8 == 8:#1000
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T4')
                        self.Cmds_AMSSerial1Send("T4")
                        self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan=4
                        self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,4")
                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] nonefilament")
                elif self.G_AMS1DeviceState["entry_state"] & bitmask1 == 1:#0001
                    self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T1')
                    self.Cmds_AMSSerial1Send("T1")
                    self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                    self.G_ChangeChannelTimeoutNewChan=1
                    self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
                elif self.G_AMS1DeviceState["entry_state"] & bitmask2 == 2:#0010
                    if self.G_AMS1DeviceState["park_state"] & bitmask1 == 1:#0001
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T1')
                        self.Cmds_AMSSerial1Send("T1")
                        self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan=1
                        self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
                    elif self.G_AMS1DeviceState["park_state"] & bitmask2 == 2:#0010
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T2')
                        self.Cmds_AMSSerial1Send("T2")
                        self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan=2
                        self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
                    else:
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T2')
                        self.Cmds_AMSSerial1Send("T2")
                        self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan=2
                        self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
                elif self.G_AMS1DeviceState["entry_state"] & bitmask4 == 4:#0100
                    if self.G_AMS1DeviceState["park_state"] & bitmask1 == 1:#0001
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T1')
                        self.Cmds_AMSSerial1Send("T1")
                        self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan=1
                        self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
                    elif self.G_AMS1DeviceState["park_state"] & bitmask2 == 2:#0010
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T2')
                        self.Cmds_AMSSerial1Send("T2")
                        self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan=2
                        self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
                    elif self.G_AMS1DeviceState["park_state"] & bitmask4 == 4:#0100
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T3')
                        self.Cmds_AMSSerial1Send("T3")
                        self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan=3
                        self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,3")
                    else:
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T3')
                        self.Cmds_AMSSerial1Send("T3")
                        self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan=3
                        self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,3")
                elif self.G_AMS1DeviceState["entry_state"] & bitmask8 ==8:#1000
                    if self.G_AMS1DeviceState["park_state"] & bitmask1 == 1:#0001
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T1')
                        self.Cmds_AMSSerial1Send("T1")
                        self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan=1
                        self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,1")
                    elif self.G_AMS1DeviceState["park_state"] & bitmask2 == 2:#0010
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T2')
                        self.Cmds_AMSSerial1Send("T2")
                        self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan=2
                        self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,2")
                    elif self.G_AMS1DeviceState["park_state"] & bitmask4 == 4:#0100
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T3')
                        self.Cmds_AMSSerial1Send("T3")
                        self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan=3
                        self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,3")
                    elif self.G_AMS1DeviceState["park_state"] & bitmask8 == 8:#1000
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T4')
                        self.Cmds_AMSSerial1Send("T4")
                        self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan=4
                        self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,4")
                    else:
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: T4')
                        self.Cmds_AMSSerial1Send("T4")
                        self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan=4
                        # lancaigang240524: use UIUX move
                        self.G_PhrozenFluiddRespondInfo("[INFO] +T:0,4")
            else:
                self.G_PhrozenFluiddRespondInfo('[INFO] #1 AMSnonefilament')

        self.Cmds_PrintMode(self.G_AMSDeviceWorkMode)

        if self.G_AMSDevice2IfNormal==True:
            if self.G_AMS2DeviceState["entry_state"]>0:
                self.G_PhrozenFluiddRespondInfo('[INFO] #2 AMShas filament')

        # lancaigang240108: P8commanddo not logical resumecommand
        self.G_M2MAModeResumeFlag=False


    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # P4 stop; stop Stopcommand(time first): "SP";
    def Cmds_CmdP4(self, gcmd):
        # if not self.G_SerialPort1OpenFlag:
        # self.G_PhrozenFluiddRespondInfo("[INFO] AMSmulti-material not connected, send P28 first")
        #     return
        
        self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.py)Cmds_CmdP4]command: stop')

        mcu_cmd = G_DictPhrozenCmdP4["mcu_cmd"][0]
        self.G_PhrozenFluiddRespondInfo("[INFO] mcucommand")



        #lancaigang241031:
        if self.G_SerialPort1OpenFlag == True:
            # lancaigang231207: stm32pause
            self.Cmds_AMSSerial1Send(mcu_cmd)
            self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1sending command')
        #lancaigang241030:
        if self.G_SerialPort2OpenFlag == True:
            self.Cmds_AMSSerial2Send(mcu_cmd)
            self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2sending command')

        #lancaigang240125: 
        # lancaigang231207: klipperpause+stm32pause
        # klipper move pause
        

        if self.G_KlipperInPausing == False:
            self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
            #lancaigang250607:
            self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
            self.G_KlipperQuickPause = True
            # klipper move pause
            self.Cmds_PhrozenKlipperPause(None)
        else:
            self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")
        # logging.info("SendCmd: %s" % mcu_cmd)
        # logging.info("stop dev running at once")



    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # P2 A1 allfilamentretractto park positionposition Yes; ====="AP";
    # P2 A2; output allfilament Yes; "CL";
    # P2 A3 filament
    # P2 A4 filament and filament
    # P2 A7 filament and filament, detectpause, use completeAMSallfilament
    def Cmds_CmdP2(self, gcmd):
        # if not self.G_SerialPort1OpenFlag:
        # self.G_PhrozenFluiddRespondInfo("[INFO] AMSmulti-material not connected, send P28 first")
        #     return
        
        self.G_PhrozenFluiddRespondInfo("[(cmds.py)Cmds_CmdP2]command='%s'" % (gcmd.get_commandline(),))


        # get command parameter number
        params = gcmd.get_command_parameters()


        self.G_PhrozenFluiddRespondInfo("[INFO] Current mode")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:0,unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:1,MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:2,MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo("[WARN] +Mode:3,RUNOUT")
        else:
            self.G_PhrozenFluiddRespondInfo("[ERROR] +Mode:-1,error")


        # if not "A" in params:
        #     return


        #time.sleep(0.5)
        self.G_PhrozenFluiddRespondInfo('[INFO] when 0.5')
        self.G_ProzenToolhead.dwell(0.5)



        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()




        if "A" in params:
            action = int(params["A"])
            if not action in [1, 2, 3,4,5,6,7]:
                raise gcmd.error("no parameter number command;cmd '%s', that must is A[1/2/3/4/5/6/7]" % (gcmd.get_commandline(),))
            # P2 A1 allfilamentretractto park positionposition Yes; ====="AP";
            if action == 1:
                # lancaigang250515: machine multi-material, logical P2A?
                if self.G_P0M1MCNoneAMS == 1:
                    self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdP2]standalonemulti-material, logical P2A?')
                    return
                #lancaigang250427: 
                if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
                    self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdP2]P0M3single-colormode, logical P2 A1')
                    return
                if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_MA:
                    self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdP2]P0M2MAsingle-color refill modemode, logical P2 A1')
                    return


                self.G_PhrozenFluiddRespondInfo("command='%s'; all filamentto park position" % (gcmd.get_commandline(),))
                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P2A1:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                # lancaigang231201: complete after, ifhome/reset, will to type, cannothome/resetbut need tocut filamentfilament
                
                #lancaigang250323: 
                if self.G_ToolheadIfHaveFilaFlag==True:
                    # lancaigang231205: home/resetcut filament
                    #self.Cmds_MoveToCutFilaAndNotRollback(gcmd)
                    self.G_PhrozenFluiddRespondInfo("[INFO] toolheadhas filament")
                    # lancaigang20231024home/resetcut filament; cannotto type
                    # lancaigang240109: toolheadhasfilamentallowedcut filament
                    #if self.G_ToolheadIfHaveFilaFlag==True:
                    # lancaigang240319: cut filamentbefore move action
                    #self.Cmds_MoveToCutFilaPrepare()
                    self.Cmds_MoveToCutFilaAbsolutePositionNotReset(gcmd)
                    #lancaigang241031:
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("AP")
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: AP; all filamentto the park position')
                    #lancaigang241030:
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AP")
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2Sending command: AP; all filamentto the park position')

                    self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PRZ_WAITINGAREA-waiting area')
                    command_string = """
                        PRZ_WAITINGAREA
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.G_PhrozenFluiddRespondInfo("External macro command-PRZ_WAITINGAREA; command_string='%s'" % command_string)

                    # lancaigang240913: set to external
                    self.G_ProzenToolhead.dwell(6.0)

                    # lancaigang231201: checkcut filament after is no normal normal unload filament, normal normal then pause
                    # lancaigang231225: when klippercompletehominghomeerror, first disable disable
                    # lancaigang240224: need tocheck is no cut filamentsuccessful
                    self.Cmds_CutFilaIfNormalCheck()
                else:
                    #lancaigang241031:
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("AP")
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: AP; all filamentto the park position')
                    #lancaigang241030:
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AP")
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2Sending command: AP; all filamentto the park position')

                    self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PRZ_WAITINGAREA-waiting area')
                    command_string = """
                        PRZ_WAITINGAREA
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.G_PhrozenFluiddRespondInfo("External macro command-PRZ_WAITINGAREA; command_string='%s'" % command_string)






                # lancaigang240113: manualcommand
                self.ManualCmdFlag=False


                # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP2]allhomeandcut filament")
                # command_string = """
                # PG28
                # """
                # self.G_PhrozenGCode.run_script_from_command(command_string)

                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P2A1:1,%d" % self.G_ChangeChannelTimeoutNewChan)

                #time.sleep(0.5)
                self.G_PhrozenFluiddRespondInfo('[INFO] when 0.5')
                self.G_ProzenToolhead.dwell(0.5)

                # lancaigang250409: get AMS
                self.Cmds_CmdP114(None)

                #time.sleep(0.5)
                self.G_PhrozenFluiddRespondInfo('[INFO] when 0.5')
                self.G_ProzenToolhead.dwell(0.5)

                return





            # P2 A2; output allfilament Yes; "CL";
            if action == 2:
                # lancaigang250515: machine multi-material, logical P2A?
                if self.G_P0M1MCNoneAMS == 1:
                    self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdP2]standalonemulti-material, logical P2A?')
                    return
                #lancaigang250427: 
                if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
                    self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdP2]single-colormode, logical P2 A2')
                    return
                self.G_PhrozenFluiddRespondInfo("command='%s'; all filament full output" % (gcmd.get_commandline(),))
                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P2A2:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                # lancaigang240319: cut filamentbefore move action
                #self.Cmds_MoveToCutFilaPrepare()


                if self.G_ToolheadIfHaveFilaFlag:
                    # lancaigang231205: home/resetcut filament
                    self.Cmds_MoveToCutFilaAndNotRollback(gcmd)
                    self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, all AMSfirst')
                    # self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutOldChan)
                    # self.G_PhrozenFluiddRespondInfo("G%d" % self.G_ChangeChannelTimeoutOldChan)
                    # self.G_PhrozenFluiddRespondInfo("AMS old channelfirst : G%d" % self.G_ChangeChannelTimeoutOldChan)
                    #lancaigang241030:
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("AP")
                        self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: AP")
                    #lancaigang241030:
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AP")
                        self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: AP")

                    self.G_ProzenToolhead.dwell(0.5)

                    # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP1]External macro command-PG101")
                    # command_string = """
                    #     PG101
                    #     """
                    # self.G_PhrozenGCode.run_script_from_command(command_string)
                    # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]External macro command-to waiting areapositionwaitpurge; command_string='%s'" % command_string)
                    # self.IfDoPG102Flag=True

                    # lancaigang240913: set to external
                    self.G_ProzenToolhead.dwell(6.0)
                    # lancaigang231201: checkcut filament after old channelfilament is no normal normal unload filament, normal normal then pause
                    self.Cmds_CutFilaIfNormalCheck()
                    if self.G_KlipperIfPaused == True:
                            self.G_PhrozenFluiddRespondInfo('[INFO] cut filament5toolheaddetectedfilament, cutter error, please check cutter, pauseklipperprinting')
                            #Lo_ChangeChannelIfSuccess = False
                            return



                # lancaigang250619:checkAMS is no re connectsuccessful
                self.Cmds_USBConnectErrorCheck()
                #lancaigang241031:
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("CL")
                    self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: CL")
                #lancaigang241030:
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("CL")
                    self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2Sending command: CL")




                # #lancaigang240913: set to external
                # self.G_ProzenToolhead.dwell(6.0)
                # #lancaigang231201: checkcut filament after is no normal normal unload filament, normal normal then pause
                # self.Cmds_CutFilaIfNormalCheck()

                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P2A2:1,%d" % self.G_ChangeChannelTimeoutNewChan)

                #time.sleep(0.5)
                self.G_PhrozenFluiddRespondInfo('[INFO] when 0.5')
                self.G_ProzenToolhead.dwell(0.5)

                return






            # P2 A3 filament
            if action == 3:
                # #lancaigang250515: machine multi-material, logical P2A?
                # if self.G_P0M1MCNoneAMS == 1:
                # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP2]standalonemulti-material, logical P2A?")
                #     return
                self.G_PhrozenFluiddRespondInfo("command='%s'; cut filament" % (gcmd.get_commandline(),))
                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P2A3:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                # lancaigang240319: cut filamentbefore move action
                #self.Cmds_MoveToCutFilaPrepare()

                self.Cmds_MoveToCutFilaAbsolutePositionNotReset(gcmd)
                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P2A3:1,%d" % self.G_ChangeChannelTimeoutNewChan)

                # lancaigang250104: P2A3
                self.G_P2A3Flag = 1
                # lancaigang240516: prevent
                #time.sleep(0.5)
                self.G_PhrozenFluiddRespondInfo('[INFO] when 0.5')
                self.G_ProzenToolhead.dwell(0.5)


            # P2 A4 filament and filament
            if action == 4:
                # lancaigang250515: machine multi-material, logical P2A?
                if self.G_P0M1MCNoneAMS == 1:
                    self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdP2]standalonemulti-material, logical P2A?')
                    return
                self.G_PhrozenFluiddRespondInfo("command='%s'; cut filament and filamentto park position" % (gcmd.get_commandline(),))
                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P2A4:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                # lancaigang240319: cut filamentbefore move action
                #self.Cmds_MoveToCutFilaPrepare()

                self.Cmds_MoveToCutFilaAbsolutePositionNotReset(gcmd)
                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P2A4:1,%d" % self.G_ChangeChannelTimeoutNewChan)






            # P2 A5 completefilament and filament, cannotto type
            if action == 5:
                # lancaigang250515: machine multi-material, logical P2A?
                if self.G_P0M1MCNoneAMS == 1:
                    self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdP2]standalonemulti-material, logical P2A?')
                    return
                self.G_PhrozenFluiddRespondInfo("command='%s'completefilament runout and filament, cannotto type" % (gcmd.get_commandline(),))
                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P2A5:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                # lancaigang240319: cut filamentbefore move action
                #self.Cmds_MoveToCutFilaPrepare()

                self.Cmds_MoveToCutFilaAbsolutePositionNotReset(gcmd)
                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P2A5:0,%d" % self.G_ChangeChannelTimeoutNewChan)


            # P2 A6 home/reset and cut filament
            if action == 6:
                # lancaigang250515: machine multi-material, logical P2A?
                if self.G_P0M1MCNoneAMS == 1:
                    self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdP2]standalonemulti-material, logical P2A?')
                    return
                self.G_PhrozenFluiddRespondInfo("command='%s'; home/reset and cut filament" % (gcmd.get_commandline(),))
                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P2A6:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                # lancaigang231201: complete after, ifhome/reset, will to type, cannothome/resetbut need tocut filamentfilament
                
                #lancaigang250323: 
                if self.G_ToolheadIfHaveFilaFlag==True:
                    # lancaigang231205: home/resetcut filament
                    #self.Cmds_MoveToCutFilaAndNotRollback(gcmd)
                    self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, resetXYcut filamentand')
                    #self.Cmds_MoveToCutFilaAndNotRollback(gcmd)
                    self.Cmds_MoveToCutFilaAndHomingXY(gcmd)
                    #lancaigang241031:
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("AP")
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: AP; all filamentto the park position')
                    #lancaigang241030:
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AP")
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2Sending command: AP; all filamentto the park position')

                    # self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PRZ_WAITINGAREA-wait")
                    # command_string = """
                    #     PRZ_WAITINGAREA
                    #     """
                    # self.G_PhrozenGCode.run_script_from_command(command_string)
                    # self.G_PhrozenFluiddRespondInfo("External macro command-PRZ_WAITINGAREA; command_string='%s'" % command_string)

                    # lancaigang240913: set to external
                    self.G_ProzenToolhead.dwell(6.0)

                    # lancaigang231201: checkcut filament after is no normal normal unload filament, normal normal then pause
                    # lancaigang231225: when klippercompletehominghomeerror, first disable disable
                    # lancaigang240224: need tocheck is no cut filamentsuccessful
                    self.Cmds_CutFilaIfNormalCheck()
                else:
                    #lancaigang241031:
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("AP")
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: AP; all filamentto the park position')
                    #lancaigang241030:
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AP")
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2Sending command: AP; all filamentto the park position')

                    # self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PRZ_WAITINGAREA-wait")
                    # command_string = """
                    #     PRZ_WAITINGAREA
                    #     """
                    # self.G_PhrozenGCode.run_script_from_command(command_string)
                    # self.G_PhrozenFluiddRespondInfo("External macro command-PRZ_WAITINGAREA; command_string='%s'" % command_string)






                #lancaigang240113: 
                self.ManualCmdFlag=True


                # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP2]allhomeandcut filament")
                # command_string = """
                # PG28
                # """
                # self.G_PhrozenGCode.run_script_from_command(command_string)

                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P2A6:1,%d" % self.G_ChangeChannelTimeoutNewChan)

                #time.sleep(0.5)
                self.G_PhrozenFluiddRespondInfo('[INFO] when 0.5')
                self.G_ProzenToolhead.dwell(0.5)

                # lancaigang250409: get AMS
                #self.Cmds_CmdP114(None)

                #time.sleep(0.5)
                self.G_PhrozenFluiddRespondInfo('[INFO] when 0.5')
                self.G_ProzenToolhead.dwell(0.5)

                return

            # P2 A7 filament and filament, detectpause, use completeAMSallfilament
            if action == 7:
                # lancaigang251014: complete; clear;
                self.G_P0M1MCNoneAMS = 0
                # #lancaigang250515: machine multi-material, logical P2A?
                # if self.G_P0M1MCNoneAMS == 1:
                # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP2]standalonemulti-material, logical P2A?")
                #     return
                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P2A7:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                
                #lancaigang250427: 
                if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_UNKNOW:
                    self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)Cmds_CmdP2]P0 M0Unknown mode")
                    #return
                
                # lancaigang250618:single-color and single-colorrefillfilament runoutdetect output
                self.G_P0M3Flag = False
                self.G_P0M2MAStartPrintFlag=0

                # lancaigang250619:checkAMS is no re connectsuccessful
                self.Cmds_USBConnectErrorCheck()

                #lancaigang250427: 
                if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
                    self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP2]P0 M3;single-colormode")

                    
                #lancaigang2521:
                if self.G_SerialPort1OpenFlag == True:
                    try:
                        self.G_PhrozenFluiddRespondInfo("[DEBUG] try;Lo_AMSDeviceStateRspInfo")
                        # get multi-materialmainboardstate
                        Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort1SendWaitRsp("SD", sizeof(AMSDetailInfoBytes))
                        self.G_PhrozenFluiddRespondInfo('tty1serial port receive : %s' % Lo_AMSDeviceStateRspInfo)
                        if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                            self.G_PhrozenFluiddRespondInfo("AMS1, please checkAMS1='%s'" % (gcmd.get_commandline(),))
                            # lancaigang240412:AMSmulti-material
                            self.G_AMSDevice1IfNormal=False
                        else:
                            # lancaigang240412:AMSmulti-material
                            self.G_AMSDevice1IfNormal=True
                    except:
                        self.G_PhrozenFluiddRespondInfo("[DEBUG] except;Lo_AMSDeviceStateRspInfo")

                if self.G_SerialPort2OpenFlag == True:
                    try:
                        self.G_PhrozenFluiddRespondInfo("[DEBUG] try;Lo_AMSDeviceStateRspInfo")
                        # get multi-materialmainboardstate
                        Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort2SendWaitRsp("SD", sizeof(AMSDetailInfoBytes))
                        self.G_PhrozenFluiddRespondInfo('tty2serial port receive : %s' % Lo_AMSDeviceStateRspInfo)
                        if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                            self.G_PhrozenFluiddRespondInfo("AMS2, please checkAMS2='%s'" % (gcmd.get_commandline(),))
                            self.G_AMSDevice2IfNormal=False
                        else:
                            self.G_AMSDevice2IfNormal=True
                    except:
                        self.G_PhrozenFluiddRespondInfo("[DEBUG] except;Lo_AMSDeviceStateRspInfo")

                #lancaigang241107:
                if self.G_AMSDevice1IfNormal==True or self.G_AMSDevice2IfNormal==True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] has AMSmulti-material, logical P2 A7')
                else:
                    self.G_PhrozenFluiddRespondInfo('[INFO] has AMSmulti-material, logical P2 A7')
                    # lancaigang250619:checkAMS is no re connectsuccessful
                    self.Cmds_USBConnectErrorCheck()
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("M0")
                        self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: M0")
                    #lancaigang241030:
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("M0")
                        self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2Sending command: M0")
                    
                    # lancaigang240524: use UIUX move
                    self.G_PhrozenFluiddRespondInfo("+P2A7:1,%d" % self.G_ChangeChannelTimeoutNewChan)
                    self.G_PhrozenFluiddRespondInfo("[INFO] return")
                    return

                self.G_PhrozenFluiddRespondInfo("command='%s'; all filamentto park position" % (gcmd.get_commandline(),))
                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P2A7:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                # lancaigang231201: complete after, ifhome/reset, will to type, cannothome/resetbut need tocut filamentfilament
                
                #lancaigang250323: 
                if self.G_ToolheadIfHaveFilaFlag==True:
                    # lancaigang231205: home/resetcut filament
                    #self.Cmds_MoveToCutFilaAndNotRollback(gcmd)
                    self.G_PhrozenFluiddRespondInfo("[INFO] toolheadhas filament")
                    # lancaigang20231024home/resetcut filament; cannotto type
                    # lancaigang240109: toolheadhasfilamentallowedcut filament
                    #if self.G_ToolheadIfHaveFilaFlag==True:
                    # lancaigang240319: cut filamentbefore move action
                    #self.Cmds_MoveToCutFilaPrepare()
                    self.Cmds_MoveToCutFilaAbsolutePositionNotReset(gcmd)
                    #lancaigang241031:
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("RD")
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: RD; all filamentto the park position')
                    #lancaigang241030:
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("RD")
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2Sending command: RD; all filamentto the park position')

                    self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PRZ_WAITINGAREA-waiting area')
                    command_string = """
                        PRZ_WAITINGAREA
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.G_PhrozenFluiddRespondInfo("External macro command-PRZ_WAITINGAREA; command_string='%s'" % command_string)

                    self.G_PhrozenFluiddRespondInfo('[INFO] when 16')
                    # #lancaigang240913: set to external
                    self.G_ProzenToolhead.dwell(16)
                    self.G_PhrozenFluiddRespondInfo('[INFO] when 16')

                    # lancaigang250619:checkAMS is no re connectsuccessful
                    self.Cmds_USBConnectErrorCheck()
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("M0")
                        self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: M0")
                    #lancaigang241030:
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("M0")
                        self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2Sending command: M0")
                else:
                    #lancaigang241031:
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("RD")
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1Sending command: RD; all filamentto the park position')
                    #lancaigang241030:
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("RD")
                        self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2Sending command: RD; all filamentto the park position')

                    self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PRZ_WAITINGAREA-waiting area')
                    command_string = """
                        PRZ_WAITINGAREA
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.G_PhrozenFluiddRespondInfo("External macro command-PRZ_WAITINGAREA; command_string='%s'" % command_string)

                    self.G_PhrozenFluiddRespondInfo('[INFO] when 12')
                    # #lancaigang240913: set to external
                    self.G_ProzenToolhead.dwell(12)
                    self.G_PhrozenFluiddRespondInfo('[INFO] when 12')

                    # lancaigang250619:checkAMS is no re connectsuccessful
                    self.Cmds_USBConnectErrorCheck()
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("M0")
                        self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1Sending command: M0")
                    #lancaigang241030:
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("M0")
                        self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2Sending command: M0")





                # lancaigang240113: manualcommand
                self.ManualCmdFlag=False


                # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP2]allhomeandcut filament")
                # command_string = """
                # PG28
                # """
                # self.G_PhrozenGCode.run_script_from_command(command_string)

                

                #time.sleep(0.5)
                self.G_PhrozenFluiddRespondInfo('[INFO] when 0.5')
                self.G_ProzenToolhead.dwell(0.5)

                # lancaigang250409: get AMS
                self.Cmds_CmdP114(None)

                #time.sleep(0.5)
                #time.sleep(0.5)
                self.G_PhrozenFluiddRespondInfo('[INFO] when 0.5')
                self.G_ProzenToolhead.dwell(0.5)

                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P2A7:1,%d" % self.G_ChangeChannelTimeoutNewChan)

                return




        #lancaigang240801: P2 B?
        if "B" in params:
            #self.Cmds_P1CnAutoChangeChannel(int(params["C"]), gcmd)
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo(gcmd.get_commandline())


        # lancaigang240516: prevent
        #time.sleep(0.5)
        self.G_PhrozenFluiddRespondInfo('[INFO] when 0.5')
        self.G_ProzenToolhead.dwell(0.5)








    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # P1 S0 allfilamentfeedto park position; ====="RD";
    # P1 T[n]n:1 ~32(no network, get 1 ~4)manualto channel,(use); ====="T";
    # P1 B[n]n:1 ~32(no network, get 1 ~4)channelfilament full output Yes; ====="B";
    # P1 D[n]; n:1~32(no network, get 1~4); channelfilamentretract in park positionstate Yes; ====="P";
    # P1 C[n] n:1~32(no network, get 1~4) automaticto channel(move action command,cut filament,, wait); ====="T";
    #lancaigang231202:
    # P1 E[n]; n:1~32(no network, get 1~4); channelfilament before switch, need to get output toolhead up filament tube Yes; ====="E?";
    # lancaigang240228: toolheadretract, need tostm32first
    # P1 G[n]; n:1~32(no network, get 1~4); channelfilament Yes; ====="G?";
    # lancaigang240319: 
    # =====P1 H[n]; n:1~32(no network, get 1~4); before retract Yes; ====="H?";
    # lancaigang240329: use
    # =====P1 I[n]; manualextrudewhen stm32need torefill; ====="I?";
    # =====P1 J[n]; multi-materialmanualpurge; bufferfullwhen refill;
    # =====P1 K[n]; 
    # =====P1 L[n]; 
    # =====P1 M[n]; 
    # =====P1 N[n]; 
    # =====P1 O[n]; 
    # =====P1 Q[n]; 
    # =====P1 U[n]; 
    #lancaigang240418: 
    # =====P1 V[n]; use version
    # =====P1 W[n]; 
    # =====P1 X[n]; 
    # =====P1 Y[n]; 
    # =====P1 Z[n]; 
    def Cmds_CmdP1(self, gcmd):
        self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]command='%s'" % (gcmd.get_commandline(),))

        if self.G_AMSDevice1IfNormal==False and self.G_AMSDevice2IfNormal==False:
            self.G_PhrozenFluiddRespondInfo('[INFO] has AMSmulti-material, all AMSmulti-material not connected, logical P1 ??command')



            # #lancaigang250828:
            # self.G_PhrozenFluiddRespondInfo("[WARN] External macro command-PRZ_PAUSE_WAITINGAREA")
            # command_string = """
            #     PRZ_PAUSE_WAITINGAREA
            #     """
            # self.G_PhrozenGCode.run_script_from_command(command_string)
            # self.G_PhrozenFluiddRespondInfo("External macro command; command_string='%s'" % command_string)




            # get command parameter number
            params = gcmd.get_command_parameters()

            # =====P1 I[n]; manualextrudewhen stm32need torefill; ====="I?";
            if "I" in params:
                self.G_PhrozenFluiddRespondInfo('[INFO] AMSmulti-materialP28not connected, move extrude use use single-colorM3mode')
                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P1In:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                if not int(params["I"]) in range(-1000, 1000):
                    raise gcmd.error("no parameter number command;cmd '%s', move extrude" % (gcmd.get_commandline()))
                # command_string = """
                #                 M106 S0
                #                 M83
                #                 G92 E0
                #                 G1 E%f F300
                #                 """ %(int(params["I"]),)
                # self.G_PhrozenGCode.run_script_from_command(command_string)
                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]G-code command: %s" % command_string)

                # lancaigang240705: AMS, use use gcodecommand,
                self.Cmds_P1InExtrudeManualIn(int(params["I"]))


                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P1In:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return
        


        # #lancaigang240705: ifP114noAMS, then extrude
        # if self.G_AMSDevice1IfNormal==False:
        #     self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP1]self.G_AMSDevice1IfNormal==False")
        # # get command parameter number
        #     params = gcmd.get_command_parameters()
        # # =====P1 I[n]; manualextrudewhen stm32need torefill; ====="I?";
        #     if "I" in params:
        # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP1]AMSmulti-materialP114not connected, extrude")
        # #lancaigang240524: use UIUX move
        #         self.G_PhrozenFluiddRespondInfo("+P1In:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        #         if not int(params["I"]) in range(-1000, 1000):
        # raise gcmd.error("no parameter number command;cmd '%s', manualextrude" % (gcmd.get_commandline()))
        #         # command_string = """
        #         #                 M106 S0
        #         #                 M83
        #         #                 G92 E0
        #         #                 G1 E%f F300
        #         #                 """ %(int(params["I"]),)
        #         # self.G_PhrozenGCode.run_script_from_command(command_string)
        #         # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]G-code command: %s" % command_string)

        # #lancaigang240705: number
        #         self.Cmds_P1InExtrudeManualIn(int(params["I"]))

        # #lancaigang240524: use UIUX move
        #         self.G_PhrozenFluiddRespondInfo("+P1In:1,%d" % self.G_ChangeChannelTimeoutNewChan)

        #         return

        self.G_PhrozenFluiddRespondInfo("[INFO] Current mode")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:0,unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:1,MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:2,MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.G_PhrozenFluiddRespondInfo("[WARN] +Mode:3,RUNOUT")
        else:
            self.G_PhrozenFluiddRespondInfo("[ERROR] +Mode:-1,error")


        # lancaigang240529: phrozen file version
        self.G_PhrozenFluiddRespondInfo("V-H%s-I%s-F%s-SN1" % (HW_VERSION,IMAGE_VERSION,FW_VERSION))


        # lancaigang231228: prevent after commandstm32ATcommand
        #time.sleep(0.5)
        # lancaigang240516: preventtime too close
        #self.G_ProzenToolhead.dwell(0.5)


        # lancaigang240105: add ATcommand
        self.G_PhrozenFluiddRespondInfo("+P1START:0,%d" % self.G_ChangeChannelTimeoutNewChan)

        # lancaigang20231019: ifdetectedtoolhead up detectto filament, first filamentthen output allthreadto park position
        #if self.G_ToolheadIfHaveFilaFlag:
        # gcmd.respond_info("[(cmds.python)Cmds_CmdP1]detectto toolhead up hasfilament, first filament, then output allthreadto park position")
        # gcmd.respond_info("[(cmds.python)Cmds_CmdP1]toolheadcut filament")
        #    self.Cmds_MoveToCutFilaAction(gcmd)
        #    self.Cmds_AMSSerial1Send("AP")
        # gcmd.respond_info("Sending command: AP, retract allto park position")

        # get command parameter number
        params = gcmd.get_command_parameters()


        # lancaigang250619:checkAMS is no re connectsuccessful
        self.Cmds_USBConnectErrorCheck()





        # P1 S0; allfilamentfeedto park position
        # P1 S1;
        if "S" in params:
            self.G_PhrozenFluiddRespondInfo("[INFO] Cmds_CmdP1]P1 S?")


            # #lancaigang250515: machine multi-material, logical T?
            # if self.G_P0M1MCNoneAMS == 1:
            # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdT0]standalonemulti-material, logical T?")
            #     return
            # #lancaigang250429: 
            # if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
            # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdT0]single-colormode, logical T0")
            #     return
            # #lancaigang250514: 
            # if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_MA :
            # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdT0]single-colorrefillmode, logical T0")
            #     return



            # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
                #lancaigang241030:
                if self.G_SerialPort1OpenFlag == True:
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
                    self.Cmds_AMSSerial1Send("MC")
                    
                #lancaigang241030:
                if self.G_SerialPort2OpenFlag == True:
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")
                    self.Cmds_AMSSerial2Send("MC")
                    

                self.G_ProzenToolhead.dwell(2)

                

            if self.G_ToolheadIfHaveFilaFlag:
                # lancaigang231205: home/resetcut filament
                self.Cmds_MoveToCutFilaAndNotRollback(gcmd)
                self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, all AMSfirst')
                # self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutOldChan)
                # self.G_PhrozenFluiddRespondInfo("G%d" % self.G_ChangeChannelTimeoutOldChan)
                # self.G_PhrozenFluiddRespondInfo("AMS old channelfirst : G%d" % self.G_ChangeChannelTimeoutOldChan)
                #lancaigang241030:
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("AP")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: AP")
                #lancaigang241030:
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("AP")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: AP")

                self.G_ProzenToolhead.dwell(0.5)

                # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP1]External macro command-PG101")
                # command_string = """
                #     PG101
                #     """
                # self.G_PhrozenGCode.run_script_from_command(command_string)
                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]External macro command-to waiting areapositionwaitpurge; command_string='%s'" % command_string)
                # self.IfDoPG102Flag=True

                # lancaigang240913: set to external
                self.G_ProzenToolhead.dwell(6.0)
                # lancaigang231201: checkcut filament after old channelfilament is no normal normal unload filament, normal normal then pause
                self.Cmds_CutFilaIfNormalCheck()
                if self.G_KlipperIfPaused == True:
                        self.G_PhrozenFluiddRespondInfo('[INFO] cut filament5toolheaddetectedfilament, cutter error, please check cutter, pauseklipperprinting')
                        #Lo_ChangeChannelIfSuccess = False
                        return

                
            self.G_PhrozenFluiddRespondInfo("command='%s';" % (gcmd.get_commandline(),))
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P1S:0,%d" % self.G_ChangeChannelTimeoutNewChan)

            #lancaigang231207: 
            if self.G_IfInFilaBlockFlag:
                self.G_PhrozenFluiddRespondInfo('[INFO] feed, first move P1 E?from toolhead up filament tube get output and move prz_resumeresume')
                self.G_PhrozenFluiddRespondInfo("+P1END:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P1S:1,%d" % self.G_ChangeChannelTimeoutNewChan)
                return

            if int(params["S"])==0:
                # feedto park position
                #lancaigang241030:
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("RD")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: RD")
                #lancaigang241030:
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("RD")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: RD")

                self.G_PhrozenFluiddRespondInfo('[INFO] sending command=RD; all filamentfeedto the park position')
            if int(params["S"])==1:
                # feedto park position
                #lancaigang241030:
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("RB")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: RB")
                #lancaigang241030:
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("RB")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: RB")
                self.G_PhrozenFluiddRespondInfo('[INFO] sending command=RB;')




            # lancaigang240113: manualcommand
            self.ManualCmdFlag=True


            # lancaigang231201: checkcut filament after is no normal normal unload filament, normal normal then pause
            # lancaigang240528: disable disable detectcut filament
            #self.Cmds_CutFilaIfNormalCheck()

            self.G_PhrozenFluiddRespondInfo("+P1END:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P1S:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return




        #lancaigang20231013: Automatic filament change
        # P1 C[n] Automatic filament change
        if "C" in params:
            self.G_PhrozenFluiddRespondInfo("[INFO] P1 C?")
            self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
            self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)

            # lancaigang250515: machine multi-material, logical T?
            if self.G_P0M1MCNoneAMS == 1:
                self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT0]standalonemulti-material, logical T?')
                return
            #lancaigang250429: 
            if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
                self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT0]single-colormode, logical T?')
                return
            #lancaigang250514: 
            if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_MA :
                self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_CmdT0]single-color refill modemode, logical T?')
                return

            # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
                self.G_PhrozenFluiddRespondInfo('[WARN] ifreceived T?command, but mode is Unknown mode, switch is MCmulti-materialmode')
                self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
                #lancaigang241030:
                if self.G_SerialPort1OpenFlag == True:
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
                    self.Cmds_AMSSerial1Send("MC")
                    
                #lancaigang241030:
                if self.G_SerialPort2OpenFlag == True:
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")
                    self.Cmds_AMSSerial2Send("MC")
                    
            
                self.G_ProzenToolhead.dwell(2)

            # lancaigang250913:returnerror
            #self.Cmds_CmdOrcaPre()
            
            self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
            self.G_PhrozenFluiddRespondInfo("[INFO] Automatic filament change")
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P1Cn:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            if not int(params["C"]) in range(1, AMS_MAX_CHANNEL + 1):
                raise gcmd.error("no parameter number command;cmd '%s', that must is C?" % (gcmd.get_commandline()))
            
            # lancaigang240113: manualcommand
            self.ManualCmdFlag=False
            # lancaigang240221: stm32 move report, canpause1 time
            self.STM32ReprotPauseFlag=0
            self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")
            # #cancelcancelcommand
            # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
            #lancaigang250517: 
            Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
            self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
            self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
            #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
            if Lo_PauseStatus['is_paused'] == True:
                self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
            else:
                self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
            # lancaigang240113: manualcommand
            self.ManualCmdFlag=False
            # lancaigang240221: stm32 move report, canpause1 time
            self.STM32ReprotPauseFlag=0
            self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")

            # lancaigang241030: is P1 C1to P1 C32, in 1to 32
            # unit 1: 1 2 3 4
            # unit 2: 5 6 7 8
            # #3 unit : 9 10 11 12
            # #4 unit : 13 14 15 16
            # #5 unit : 17 18 19 20
            # #6 unit : 21 22 23 24
            # #7 unit : 25 26 27 28
            # #8 unit : 29 30 31 32
            #Automatic filament change
            #self.Cmds_P1CnAutoChangeChannel(int(params["C"]), gcmd)
            chan=int(params["C"])

            if chan==1:
                # lancaigang250515:serial port disable set color to channel
                if self.G_ChromaKitAccessT0 > 0:
                    self.G_PhrozenFluiddRespondInfo('use; self.G_ChromaKitAccessT0=%d' % self.G_ChromaKitAccessT0)
                    self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT0, gcmd)
                else:
                    self.G_PhrozenFluiddRespondInfo('use, default; chan=%d' % chan)
                    self.Cmds_P1CnAutoChangeChannel(chan, gcmd)
            elif chan==2:
                # lancaigang250515:serial port disable set color to channel
                if self.G_ChromaKitAccessT1 > 0:
                    self.G_PhrozenFluiddRespondInfo('use; self.G_ChromaKitAccessT1=%d' % self.G_ChromaKitAccessT1)
                    self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT1, gcmd)
                else:
                    self.G_PhrozenFluiddRespondInfo('use, default; chan=%d' % chan)
                    self.Cmds_P1CnAutoChangeChannel(chan, gcmd)
            elif chan==3:
                # lancaigang250515:serial port disable set color to channel
                if self.G_ChromaKitAccessT2 > 0:
                    self.G_PhrozenFluiddRespondInfo('use; self.G_ChromaKitAccessT2=%d' % self.G_ChromaKitAccessT2)
                    self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT2, gcmd)
                else:
                    self.G_PhrozenFluiddRespondInfo('use, default; chan=%d' % chan)
                    self.Cmds_P1CnAutoChangeChannel(chan, gcmd)
            elif chan==4:
                # lancaigang250515:serial port disable set color to channel
                if self.G_ChromaKitAccessT3 > 0:
                    self.G_PhrozenFluiddRespondInfo('use; self.G_ChromaKitAccessT3=%d' % self.G_ChromaKitAccessT3)
                    self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT3, gcmd)
                else:
                    self.G_PhrozenFluiddRespondInfo('use, default; chan=%d' % chan)
                    self.Cmds_P1CnAutoChangeChannel(chan, gcmd)


            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P1Cn:1,%d" % self.G_ChangeChannelTimeoutNewChan)















        # lancaigang20231013: manual
        # P1 T[n] manual
        if "T" in params:
            self.G_PhrozenFluiddRespondInfo("[INFO] P1 T?")
            # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
                #lancaigang241030:
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("MC")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
                #lancaigang241030:
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("MC")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")

                self.G_ProzenToolhead.dwell(2)
            
            self.G_PhrozenFluiddRespondInfo("command='%s'; move" % (gcmd.get_commandline(),))
            self.G_PhrozenFluiddRespondInfo('[INFO] move')
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P1Tn:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            if not int(params["T"]) in range(1, AMS_MAX_CHANNEL + 1):
                raise gcmd.error("no parameter number command;cmd '%s', that must is T?" % (gcmd.get_commandline()))
            
            # lancaigang231209: manual call do notpause
            self.G_KlipperIfPaused=False
            # lancaigang240221: stm32 move report, canpause1 time
            self.STM32ReprotPauseFlag=0
            self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")

            # lancaigang240113: manualcommand
            self.ManualCmdFlag=True
            # #lancaigang240529: refill pause
            # self.Cmds_AMSSerial1Send("M0")
            # self.G_PhrozenFluiddRespondInfo("[INFO] sending command=M0")
            # self.G_ProzenToolhead.dwell(1)

            #lancaigang231207: 
            if self.G_IfInFilaBlockFlag:
                self.G_PhrozenFluiddRespondInfo('[INFO] feed, first move P1 E?from toolhead up filament tube get output and move prz_resumeresume')
                self.G_PhrozenFluiddRespondInfo("+P1END:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P1Tn:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                return

            # lancaigang231202: home/resetcut filament
            #self.Cmds_MoveToCutFilaAbsolutePositionNotReset(gcmd)
            # lancaigang240319: cut filamentbefore move action
            #self.G_ChangeChannelTimeoutOldChan=int(params["T"])
            #self.Cmds_MoveToCutFilaPrepare()


            if self.G_ToolheadIfHaveFilaFlag:
                # lancaigang231205: home/resetcut filament
                self.Cmds_MoveToCutFilaAndNotRollback(gcmd)
                self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, all AMSfirst')
                # self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutOldChan)
                # self.G_PhrozenFluiddRespondInfo("G%d" % self.G_ChangeChannelTimeoutOldChan)
                # self.G_PhrozenFluiddRespondInfo("AMS old channelfirst : G%d" % self.G_ChangeChannelTimeoutOldChan)
                #lancaigang241030:
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("AP")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: AP")
                #lancaigang241030:
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("AP")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: AP")

                self.G_ProzenToolhead.dwell(0.5)

                # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP1]External macro command-PG101")
                # command_string = """
                #     PG101
                #     """
                # self.G_PhrozenGCode.run_script_from_command(command_string)
                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]External macro command-to waiting areapositionwaitpurge; command_string='%s'" % command_string)
                # self.IfDoPG102Flag=True

                # lancaigang240913: set to external
                self.G_ProzenToolhead.dwell(6.0)
                # lancaigang231201: checkcut filament after old channelfilament is no normal normal unload filament, normal normal then pause
                self.Cmds_CutFilaIfNormalCheck()
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] cut filament5toolheaddetectedfilament, cutter error, please check cutter, pauseklipperprinting')
                    #Lo_ChangeChannelIfSuccess = False
                    return




            # self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG109; cut filament after, first add and reserve toolheadfilament")
            # self.PG102Flag=True
            # self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=True")
            # command_string = """
            # PG109
            # """
            # self.G_PhrozenGCode.run_script_from_command(command_string)
            # self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)
            # self.PG102Flag=False
            # self.G_PhrozenFluiddRespondInfo("[INFO] self.Flag=False")




            self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
            self.G_ChangeChannelTimeoutOldGcmd=self.G_ChangeChannelTimeoutNewGcmd
            self.G_ChangeChannelTimeoutNewChan=int(params["T"])
            self.G_ChangeChannelTimeoutNewGcmd=gcmd

            # lancaigang241030: is P1 C1to P1 C32, in 1to 32
            # unit 1: 1 2 3 4
            # unit 2: 5 6 7 8
            # #3 unit : 9 10 11 12
            # #4 unit : 13 14 15 16
            # #5 unit : 17 18 19 20
            # #6 unit : 21 22 23 24
            # #7 unit : 25 26 27 28
            # #8 unit : 29 30 31 32
            # manual
            self.Cmds_P1TnManualChangeChannel(int(params["T"]), gcmd)



            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P1Tn:1,%d" % self.G_ChangeChannelTimeoutNewChan)





        # P1 B[n] manual full unload filament
        if "B" in params:
            self.G_PhrozenFluiddRespondInfo("[INFO] P1 B?")
            # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
                #lancaigang241030:
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("MC")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
                #lancaigang241030:
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("MC")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")

                self.G_ProzenToolhead.dwell(2)
            
            self.G_PhrozenFluiddRespondInfo("command='%s'; filament full output" % (gcmd.get_commandline(),))
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P1Bn:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            if not int(params["B"]) in range(1, AMS_MAX_CHANNEL + 1):
                raise gcmd.error("no parameter number command;cmd '%s', that must is B?" % (gcmd.get_commandline()))
            
            # lancaigang231209: manual call do notpause
            self.G_KlipperIfPaused=False
            # lancaigang240221: stm32 move report, canpause1 time
            self.STM32ReprotPauseFlag=0
            self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")

            # lancaigang240113: manualcommand
            self.ManualCmdFlag=True
            # #lancaigang240529: refill pause
            # self.Cmds_AMSSerial1Send("M0")
            # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP1]sending command=M0")
            # self.G_ProzenToolhead.dwell(1)

            # lancaigang240319: cut filamentbefore move action
            #self.G_ChangeChannelTimeoutNewChan=int(params["B"])
            #self.Cmds_MoveToCutFilaPrepare()

            # #lancaigang231202: home/resetcut filament
            # #self.Cmds_MoveToCutFilaAbsolutePositionNotReset(gcmd)
            # #lancaigang231205: home/resetcut filament
            # #lancaigang240328: home/resetcut filament
            # self.Cmds_MoveToCutFilaAndNotRollback(gcmd)


            if self.G_ToolheadIfHaveFilaFlag==True:
                # lancaigang231205: home/resetcut filament
                self.Cmds_MoveToCutFilaAndNotRollback(gcmd)
                self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, all AMSfirst')
                # self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutOldChan)
                # self.G_PhrozenFluiddRespondInfo("G%d" % self.G_ChangeChannelTimeoutOldChan)
                # self.G_PhrozenFluiddRespondInfo("AMS old channelfirst : G%d" % self.G_ChangeChannelTimeoutOldChan)
                #lancaigang241030:
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("AP")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: AP")
                #lancaigang241030:
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("AP")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: AP")

                self.G_ProzenToolhead.dwell(0.5)

                # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP1]External macro command-PG101")
                # command_string = """
                #     PG101
                #     """
                # self.G_PhrozenGCode.run_script_from_command(command_string)
                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]External macro command-to waiting areapositionwaitpurge; command_string='%s'" % command_string)
                # self.IfDoPG102Flag=True

                # lancaigang240913: set to external
                self.G_ProzenToolhead.dwell(6.0)
                # lancaigang231201: checkcut filament after old channelfilament is no normal normal unload filament, normal normal then pause
                self.Cmds_CutFilaIfNormalCheck()
                if self.G_KlipperIfPaused == True:
                        self.G_PhrozenFluiddRespondInfo('[INFO] cut filament5toolheaddetectedfilament, cutter error, please check cutter, pauseklipperprinting')
                        #Lo_ChangeChannelIfSuccess = False
                        return


            # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)]External macro command-PG109; cut filament after, first reserve toolheadfilament")
            # self.PG102Flag=True
            # self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)]self.Flag=True")
            # command_string = """
            # PG109
            # """
            # self.G_PhrozenGCode.run_script_from_command(command_string)
            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)]command_string='%s'" % command_string)
            # self.PG102Flag=False
            # self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)]self.Flag=False")



            #lancaigang231207: 
            if self.G_IfInFilaBlockFlag:
                self.G_PhrozenFluiddRespondInfo('[INFO] feed, first move P1 E?from toolhead up filament tube get output and move prz_resumeresume')
                self.G_PhrozenFluiddRespondInfo("+P1END:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P1Bn:1,%d" % self.G_ChangeChannelTimeoutNewChan)

                return

            
            self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
            self.G_ChangeChannelTimeoutOldGcmd=self.G_ChangeChannelTimeoutNewGcmd
            self.G_ChangeChannelTimeoutNewChan=int(params["B"])
            self.G_ChangeChannelTimeoutNewGcmd=gcmd


            # manual full unload filament
            self.Cmds_P1BnWholeRollbackAction(int(params["B"]), gcmd)

            # lancaigang240115:when 1
            time.sleep(1)
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P1Bn:1,%d" % self.G_ChangeChannelTimeoutNewChan)





        # P1 D[n] manualto park position
        if "D" in params:
            self.G_PhrozenFluiddRespondInfo("[INFO] P1 D?")
            # lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, because to action move command, defaultSTM32enterMCmulti-materialmode')
                #lancaigang241030:
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("MC")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")
                #lancaigang241030:
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("MC")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")
                
                self.G_ProzenToolhead.dwell(2)
            
            self.G_PhrozenFluiddRespondInfo("command='%s'; move to park position" % (gcmd.get_commandline(),))
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P1Dn:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            if not int(params["D"]) in range(1, AMS_MAX_CHANNEL + 1):
                raise gcmd.error("no parameter number command;cmd '%s', that must is D?" % (gcmd.get_commandline()))
            
            # lancaigang231209: manual call do notpause
            self.G_KlipperIfPaused=False
            # lancaigang240221: stm32 move report, canpause1 time
            self.STM32ReprotPauseFlag=0
            self.G_PhrozenFluiddRespondInfo("[WARN] self.STM32ReprotPauseFlag=0")

            # lancaigang240113: manualcommand
            self.ManualCmdFlag=True
            # #lancaigang240529: refill pause
            # self.Cmds_AMSSerial1Send("M0")
            # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP1]sending command=M0")
            # self.G_ProzenToolhead.dwell(1)

            # lancaigang240319: cut filamentbefore move action
            #self.G_ChangeChannelTimeoutNewChan=int(params["D"])
            #self.Cmds_MoveToCutFilaPrepare()



            if self.G_ToolheadIfHaveFilaFlag:
                # lancaigang231205: home/resetcut filament
                self.Cmds_MoveToCutFilaAndNotRollback(gcmd)
                self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, all AMSfirst')
                # self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutOldChan)
                # self.G_PhrozenFluiddRespondInfo("G%d" % self.G_ChangeChannelTimeoutOldChan)
                # self.G_PhrozenFluiddRespondInfo("AMS old channelfirst : G%d" % self.G_ChangeChannelTimeoutOldChan)
                #lancaigang241030:
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("AP")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: AP")
                #lancaigang241030:
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("AP")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: AP")

                self.G_ProzenToolhead.dwell(0.5)

                # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP1]External macro command-PG101")
                # command_string = """
                #     PG101
                #     """
                # self.G_PhrozenGCode.run_script_from_command(command_string)
                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]External macro command-to waiting areapositionwaitpurge; command_string='%s'" % command_string)
                # self.IfDoPG102Flag=True

                # lancaigang240913: set to external
                self.G_ProzenToolhead.dwell(6.0)
                # lancaigang231201: checkcut filament after old channelfilament is no normal normal unload filament, normal normal then pause
                self.Cmds_CutFilaIfNormalCheck()
                if self.G_KlipperIfPaused == True:
                        self.G_PhrozenFluiddRespondInfo('[INFO] cut filament5toolheaddetectedfilament, cutter error, please check cutter, pauseklipperprinting')
                        #Lo_ChangeChannelIfSuccess = False
                        return


            # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)]External macro command-PG109; cut filament after, first reserve toolheadfilament")
            # self.PG102Flag=True
            # self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)]self.Flag=True")
            # command_string = """
            # PG109
            # """
            # self.G_PhrozenGCode.run_script_from_command(command_string)
            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)]command_string='%s'" % command_string)
            # self.PG102Flag=False
            # self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)]self.Flag=False")

            #lancaigang231207: 
            if self.G_IfInFilaBlockFlag:
                self.G_PhrozenFluiddRespondInfo('[INFO] feed, first move P1 E?from toolhead up filament tube get output and move prz_resumeresume')
                self.G_PhrozenFluiddRespondInfo("+P1END:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                # lancaigang240524: use UIUX move
                self.G_PhrozenFluiddRespondInfo("+P1Dn:1,%d" % self.G_ChangeChannelTimeoutNewChan)
                return


            self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
            self.G_ChangeChannelTimeoutOldGcmd=self.G_ChangeChannelTimeoutNewGcmd
            self.G_ChangeChannelTimeoutNewChan=int(params["D"])
            self.G_ChangeChannelTimeoutNewGcmd=gcmd

            # manualto park position
            self.Cmds_P1DnMoveToParkPositonAction(int(params["D"]), gcmd)

            # lancaigang240115:when 1
            time.sleep(1)

            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P1Dn:1,%d" % self.G_ChangeChannelTimeoutNewChan)





        # lancaigang231202: before switch machine, get output toolheadfilament tube output
        # P1 E[n]
        if "E" in params:
            self.G_PhrozenFluiddRespondInfo("[INFO] P1 E?")
            # #lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
            # if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            # self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)Cmds_CmdP1]Unknown mode, because to action manualcommand, defaultSTM32enterMCmulti-materialmode")
            #     self.Cmds_AMSSerial1Send("MC")
            # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP1]sending command: MC")
            
            self.G_PhrozenFluiddRespondInfo("command='%s'; before switch machine, get output toolheadfilament tube output" % (gcmd.get_commandline(),))
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P1En:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            if not int(params["E"]) in range(1, AMS_MAX_CHANNEL + 1):
                raise gcmd.error("no parameter number command;cmd '%s', that must is E?" % (gcmd.get_commandline()))
            
            # lancaigang231202: home/resetcut filament
            # lancaigang231214: toolhead get output filament, do notautomaticcut filament, need tocut filament
            #self.Cmds_MoveToCutFilaAbsolutePositionNotReset(gcmd)

            # lancaigang240603: prevent
            #time.sleep(2)

            # manualto park position
            self.Cmds_P1EnForceForward(int(params["E"]), gcmd)

            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P1En:1,%d" % self.G_ChangeChannelTimeoutNewChan)





        # lancaigang240228: toolheadretract, need tostm32first
        # P1 G[n]; n:1~32(no network, get 1~4); channelfilament Yes; ====="G?";
        if "G" in params:
            self.G_PhrozenFluiddRespondInfo("[INFO] P1 G?")
            # #lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
            # if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            # self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)Cmds_CmdP1]Unknown mode, because to action manualcommand, defaultSTM32enterMCmulti-materialmode")
            #     self.Cmds_AMSSerial1Send("MC")
            # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP1]sending command: MC")
            
            self.G_PhrozenFluiddRespondInfo("command='%s'AMSfirst" % (gcmd.get_commandline(),))
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P1Gn:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            if not int(params["G"]) in range(1, AMS_MAX_CHANNEL + 1):
                raise gcmd.error("no parameter number command;cmd '%s', that must is G?" % (gcmd.get_commandline()))
            

            if self.G_ToolheadIfHaveFilaFlag:
                # lancaigang231205: home/resetcut filament
                self.Cmds_MoveToCutFilaAndNotRollback(gcmd)
                self.G_PhrozenFluiddRespondInfo('[INFO] toolheadhas filament, all AMSfirst')
                # self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutOldChan)
                # self.G_PhrozenFluiddRespondInfo("G%d" % self.G_ChangeChannelTimeoutOldChan)
                # self.G_PhrozenFluiddRespondInfo("AMS old channelfirst : G%d" % self.G_ChangeChannelTimeoutOldChan)
                #lancaigang241030:
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("AP")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: AP")
                #lancaigang241030:
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("AP")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: AP")

                self.G_ProzenToolhead.dwell(0.5)

                # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP1]External macro command-PG101")
                # command_string = """
                #     PG101
                #     """
                # self.G_PhrozenGCode.run_script_from_command(command_string)
                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]External macro command-to waiting areapositionwaitpurge; command_string='%s'" % command_string)
                # self.IfDoPG102Flag=True

                # lancaigang240913: set to external
                self.G_ProzenToolhead.dwell(6.0)
                # lancaigang231201: checkcut filament after old channelfilament is no normal normal unload filament, normal normal then pause
                self.Cmds_CutFilaIfNormalCheck()
                if self.G_KlipperIfPaused == True:
                        self.G_PhrozenFluiddRespondInfo('[INFO] cut filament5toolheaddetectedfilament, cutter error, please check cutter, pauseklipperprinting')
                        #Lo_ChangeChannelIfSuccess = False
                        return

            

            # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)]External macro command-PG109; cut filament after, first reserve toolheadfilament")
            # self.PG102Flag=True
            # self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)]self.Flag=True")
            # command_string = """
            # PG109
            # """
            # self.G_PhrozenGCode.run_script_from_command(command_string)
            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)]command_string='%s'" % command_string)
            # self.PG102Flag=False
            # self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)]self.Flag=False")



            self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
            self.G_ChangeChannelTimeoutOldGcmd=self.G_ChangeChannelTimeoutNewGcmd
            self.G_ChangeChannelTimeoutNewChan=int(params["G"])
            self.G_ChangeChannelTimeoutNewGcmd=gcmd

            self.Cmds_P1GnExtruderBack(int(params["G"]), gcmd)

            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P1Gn:1,%d" % self.G_ChangeChannelTimeoutNewChan)





        if "H" in params:
            self.G_PhrozenFluiddRespondInfo("[INFO] P1 H?")
            # #lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
            # if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            # self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)Cmds_CmdP1]Unknown mode, because to action manualcommand, defaultSTM32enterMCmulti-materialmode")
            #     self.Cmds_AMSSerial1Send("MC")
            # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP1]sending command: MC")
            
            self.G_PhrozenFluiddRespondInfo("command='%s'special refill" % (gcmd.get_commandline(),))
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P1Hn:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            if not int(params["H"]) in range(1, AMS_MAX_CHANNEL + 1):
                raise gcmd.error("no parameter number command;cmd '%s', that must is H?" % (gcmd.get_commandline()))
            
            

            self.Cmds_P1HnSpecialInfila(int(params["H"]), gcmd)


            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P1Hn:1,%d" % self.G_ChangeChannelTimeoutNewChan)





        # =====P1 I[n]; manualextrudewhen stm32need torefill; ====="I?";
        if "I" in params:
            self.G_PhrozenFluiddRespondInfo("[INFO] P1 I?")
            # #lancaigang240527: Unknown mode, because to action manualcommand, defaultSTM32MCmulti-materialmode
            # if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            # self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)Cmds_CmdP1]Unknown mode, because to action manualcommand, defaultSTM32enterMCmulti-materialmode")
            #     self.Cmds_AMSSerial1Send("MC")
            # self.G_PhrozenFluiddRespondInfo("[INFO] [(cmds.python)Cmds_CmdP1]sending command: MC")
            
            self.G_PhrozenFluiddRespondInfo("command='%s' move extrudewhen stm32 need to refill" % (gcmd.get_commandline(),))
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P1In:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            if not int(params["I"]) in range(-1000, 1000):
                raise gcmd.error("no parameter number command;cmd '%s', move extrude" % (gcmd.get_commandline()))
            
            

            self.Cmds_P1InExtruderBack(int(params["I"]), gcmd)


            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P1In:1,%d" % self.G_ChangeChannelTimeoutNewChan)


        # =====P1 J[n]; multi-materialmanualpurge; bufferfullwhen refill;
        if "J" in params:
            self.G_PhrozenFluiddRespondInfo("[INFO] P1 J?")

            self.G_PhrozenFluiddRespondInfo("command='%s' move extrudewhen stm32 need to refill" % (gcmd.get_commandline(),))
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P1Jn:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            if not int(params["J"]) in range(-1000, 1000):
                raise gcmd.error("no parameter number command;cmd '%s', move extrude parameter number error" % (gcmd.get_commandline()))
            
            

            self.Cmds_P1JnManualSpitFila(int(params["J"]), gcmd)


            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo("+P1Jn:1,%d" % self.G_ChangeChannelTimeoutNewChan)






        # lancaigang240105: command after add completeATcommand
        self.G_PhrozenFluiddRespondInfo("+P1END:0,%d" % self.G_ChangeChannelTimeoutNewChan)
    


    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # P0 M1; multi-materialmodemode(need connect external all) Yes; "MC";P0 M1;P28;P2 A1;
    # P0 M2; multi-material single-colorrefillmode(need connect external all); "MA";P0 M2;P28;P8;
    # P0 M3; single-colorfilament runoutmode;P0 M3;
    #lancaigang240801:
    # P0 B?
    def Cmds_CmdP0(self, gcmd):
        self.G_PhrozenFluiddRespondInfo("[(cmds.py)Cmds_CmdP0]command='%s'" % (gcmd.get_commandline(),))

        self.G_PhrozenFluiddRespondInfo("V-H%s-I%s-F%s" % (HW_VERSION,IMAGE_VERSION,FW_VERSION))

        # get command parameter number M?
        params = gcmd.get_command_parameters()

        # if not "M" in params:
        #     return


        #Unlock
        self.Base_AMSSerialCmdUnlock()







        #lancaigang240801: P2 M?
        if "M" in params:
            # lancaigang250522: clearAMSconnect
            self.G_AMSDevice1IfNormal=False
            self.G_AMSDevice2IfNormal=False

            #lancaigang250526: 
            self.G_IfToolheadHaveFilaInitiativePauseFlag=False
            # lancaigang250526: pause, not allowednew gcodecommand, need waitpausecomplete
            self.G_KlipperInPausing = False
            # lancaigang250527: pause
            self.G_KlipperQuickPause = False
            # lancaigang250607: state; 1-unload filament; 2-feed; 3-; 4-pause
            self.G_KlipperPrintStatus= -1
            self.G_ASM1DisconnectErrorCount=0
            # lancaigang250812:single-colorfilament runoutdetect, to pause
            self.G_RetryToPauseAreaFlag = False
            self.G_RetryToPauseAreaCount = 0
            self.G_P10SpitNum=0
            self.G_IfChangeFilaOngoing= False
            # lancaigang240223: toolheadcut filamentfailed
            self.ToolheadCutFlag = False











            # lancaigang250618: single-colorM3filament runoutdetect
            self.G_P0M3Flag = False
            # lancaigang250618: single-colorrefillM2filament runoutdetect
            self.G_P0M2MAStartPrintFlag = 0
            self.ManualCmdFlag=False
            # lancaigang250805: cutter
            self.G_CutCheckTest=False


            # lancaigang250515:clearserial port disable set number data
            self.Cmds_GetUartScreenCfgClear()

            #lancaigang250514: Read the JSON file for monochrome refill settings and channel/color mappings
            #/home/prz/klipper/klippy/extras/phrozen_dev/serial-screen/test.json
            self.Cmds_GetUartScreenCfg()
            
            self.G_PhrozenFluiddRespondInfo("[INFO] Current mode")
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:0,unkown")
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
                self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:1,MC")
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:2,MA")
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                self.G_PhrozenFluiddRespondInfo("[WARN] +Mode:3,RUNOUT")
            else:
                self.G_PhrozenFluiddRespondInfo("[ERROR] +Mode:-1,error")


            #time.sleep(0.5)
            self.G_PhrozenFluiddRespondInfo('[INFO] when 1')
            self.G_ProzenToolhead.dwell(1)

            # lancaigang250619:checkAMS is no re connectsuccessful
            self.Cmds_USBConnectErrorCheck()


            # #cancelcancelcommand
            # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
            #lancaigang250517: 
            #self.G_PhrozenPrinterCancelPauseResume.cmd_CLEAR_PAUSE
            # #cancelcancelcommand
            # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
            #lancaigang250517: 
            Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
            self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
            self.G_PhrozenPrinterCancelPauseResume.cmd_CLEAR_PAUSE(None)
            self.G_PhrozenFluiddRespondInfo('[INFO] clearpause state')
            Lo_PauseStatus=self.G_PhrozenPrinterCancelPauseResume.get_status(None)
            self.G_PhrozenFluiddRespondInfo("Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus)
            self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
            #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
            if Lo_PauseStatus['is_paused'] == True:
                self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
            else:
                self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
            # lancaigang240511: initialize down serial port, preventAMSserial porterror
            try:
                self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 1")
                self.G_SerialPort1Obj = serial.Serial(self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3)
                #Serial port opened successfully
                if self.G_SerialPort1Obj is not None:
                    if self.G_SerialPort1Obj.is_open:
                        self.G_SerialPort1OpenFlag = True
                        self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 1successful")
                        # lancaigang231213: openserial port
                        self.G_SerialPort1Obj.flushInput()  # clean serial write cache
                        self.G_SerialPort1Obj.flush()
                        self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 buffers cleared")
                        self.G_PhrozenFluiddRespondInfo("[INFO] Re-registering serial port 1 callback")
                        self.G_SerialPort1RecvTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerUart1Recv, self.G_PhrozenReactor.NOW)
            except:
                self.G_PhrozenFluiddRespondInfo("[ERROR] Unable to open tty1. Check the USB connection or try rebooting.")

            try:
                self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 2")
                self.G_SerialPort2Obj = serial.Serial(self.G_Serialport2Define, SERIAL_PORT_BAUD, timeout=3)
                #Serial port opened successfully
                if self.G_SerialPort2Obj is not None:
                    if self.G_SerialPort2Obj.is_open:
                        self.G_SerialPort2OpenFlag = True
                        self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 2successful")
                        self.G_SerialPort2Obj.flushInput()  # clean serial write cache
                        self.G_SerialPort2Obj.flush()
                        self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 buffers cleared")
                        self.G_PhrozenFluiddRespondInfo("[INFO] Re-registering serial port 2 callback")
                        self.G_SerialPort2RecvTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerUart2Recv, self.G_PhrozenReactor.NOW)
            except:
                self.G_PhrozenFluiddRespondInfo("[DEBUG] AMS2/tty2 not available; skipping serial port 2.")




            #lancaigang250323: 
            self.G_PhrozenFluiddRespondInfo('[INFO] External macro command-PG103- get toolhead; full')
            command_string = """
                PG103
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_PhrozenFluiddRespondInfo("External macro command-PG103; command_string='%s'" % command_string)





            # get commandmode parameter number
            Lo_AMSWorkMode = int(params["M"])

            if not Lo_AMSWorkMode in [
                AMS_WORK_MODE_UNKNOW,#Unknown mode 0
                AMS_WORK_MODE_MC,# MCmode 1
                AMS_WORK_MODE_MA,# single-colorMAmode 2
                AMS_WORK_MODE_FILA_RUNOUT,# filament runout logical mode 3
            ]:
                raise gcmd.error("no parameter number command;cmd '%s', that must is M[0/1/2/3]" % (gcmd.get_commandline(),))
            



            self.G_AMSDeviceWorkMode = Lo_AMSWorkMode
            self.Cmds_PrintMode(self.G_AMSDeviceWorkMode)


        
            self.G_PhrozenFluiddRespondInfo("[INFO] Current mode")
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:0,unkown")
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
                self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:1,MC")
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                self.G_PhrozenFluiddRespondInfo("[INFO] +Mode:2,MA")
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                self.G_PhrozenFluiddRespondInfo("[WARN] +Mode:3,RUNOUT")
            else:
                self.G_PhrozenFluiddRespondInfo("[ERROR] +Mode:-1,error")



            #lancaigang240410: 
            self.G_CancelFlag=False
            # lancaigang240413: pausestate
            self.G_KlipperIfPaused = False
            # lancaigang240413: stm32 move report, canpause1 time
            self.STM32ReprotPauseFlag=0

            #lancaigang250515: 
            self.G_P0M1MCNoneAMS=0
            self.G_PhrozenFluiddRespondInfo("[INFO] self.G_P0M1MCNoneAMS=0")


            self.G_PhrozenFluiddRespondInfo('[INFO] when 1')
            self.G_ProzenToolhead.dwell(1)


            #=====M0;Unknown mode
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:#0
                self.G_ToolheadFirstInputFila = False
                self.G_PhrozenFluiddRespondInfo("[WARN] P0 M0Unknown mode")

                # lancaigang240411: ifnoreceived P0 M3command, use use filament runoutdetect machine
                self.G_P0M3Flag = False
                
                self.G_P0M1MCNoneAMS=0
                self.G_PhrozenFluiddRespondInfo("[INFO] self.G_P0M1MCNoneAMS=0")


                # lancaigang250327: multi-materialbefore, not allowedAMSmulti-materialpause
                self.ManualCmdFlag=True
                self.G_PhrozenFluiddRespondInfo("[INFO] self.ManualCmdFlag=True")

                # lancaigang250104: P2A3
                if self.G_P2A3Flag == 1:
                    self.G_P2A3Flag = 0
                    #lancaigang240416:
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("AT+IDLE")
                        self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: AT+IDLE")
                    #lancaigang241030:
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AT+IDLE")
                        self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: AT+IDLE")
                
                else:
                    #lancaigang240416:
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("M0")
                        self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: M0")
                    #lancaigang241030:
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("M0")
                        self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: M0")

                self.G_ProzenToolhead.dwell(0.5)

            # =====M1;multi-materialmode
            #P0 M1
            #P28
            #P2 A1
            #T?
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:  #1
                self.G_PhrozenFluiddRespondInfo("[INFO] P0 M1multi-materialmode: MC")

                # lancaigang250520:
                self.ManualCmdFlag=False

                # lancaigang250304: time P0M1initializemulti-materialchannel, preventchannelerror
                self.G_ChangeChannelTimeoutOldChan=-1
                self.G_PhrozenFluiddRespondInfo("[INFO] self.G_ChangeChannelTimeoutOldChan=-1")
                self.G_ChangeChannelTimeoutOldGcmd=None
                self.G_PhrozenFluiddRespondInfo("[INFO] self.G_ChangeChannelTimeoutOldGcmd=None")
                self.G_ChangeChannelTimeoutNewChan=-1
                self.G_PhrozenFluiddRespondInfo("[INFO] self.G_ChangeChannelTimeoutNewChan=-1")
                self.G_ChangeChannelTimeoutNewGcmd=None
                self.G_PhrozenFluiddRespondInfo("[INFO] self.G_ChangeChannelTimeoutNewGcmd=None")


                # lancaigang250102: time number
                self.G_PrintCountNum=0

                # lancaigang240125: do notP28connectcanfirst home/resetcut filament
                # lancaigang231219: home/resetcut filament
                # lancaigang240319: PG28home/reset will error
                #self.Cmds_MoveToCutFilaAndNotRollback(gcmd)


                #lancaigang240416:
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("MC")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MC")

                #lancaigang241030:
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("MC")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MC")



                self.G_ChangeChannelFirstFilaFlag = True


                #time.sleep(0.5)
                self.G_ProzenToolhead.dwell(2.5)


                self.G_PhrozenFluiddRespondInfo('[INFO] check is no has AMS')

                if self.G_SerialPort1OpenFlag == True:
                    try:
                        self.G_PhrozenFluiddRespondInfo("[DEBUG] try;Lo_AMSDeviceStateRspInfo")
                        # get multi-materialmainboardstate
                        Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort1SendWaitRsp("SD", sizeof(AMSDetailInfoBytes))
                        self.G_PhrozenFluiddRespondInfo('tty1serial port receive : %s' % Lo_AMSDeviceStateRspInfo)
                        if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                            self.G_PhrozenFluiddRespondInfo("AMS1, please checkAMS1='%s'" % (gcmd.get_commandline(),))
                            # lancaigang240412:AMSmulti-material
                            self.G_AMSDevice1IfNormal=False
                        else:
                            # lancaigang240412:AMSmulti-material
                            self.G_AMSDevice1IfNormal=True
                    except:
                        self.G_PhrozenFluiddRespondInfo("[DEBUG] except;Lo_AMSDeviceStateRspInfo")

                if self.G_SerialPort2OpenFlag == True:
                    try:
                        self.G_PhrozenFluiddRespondInfo("[DEBUG] try;Lo_AMSDeviceStateRspInfo")
                        # get multi-materialmainboardstate
                        Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort2SendWaitRsp("SD", sizeof(AMSDetailInfoBytes))
                        self.G_PhrozenFluiddRespondInfo('tty2serial port receive : %s' % Lo_AMSDeviceStateRspInfo)
                        if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                            self.G_PhrozenFluiddRespondInfo("AMS2, please checkAMS2='%s'" % (gcmd.get_commandline(),))
                            self.G_AMSDevice2IfNormal=False
                        else:
                            self.G_AMSDevice2IfNormal=True
                    except:
                        self.G_PhrozenFluiddRespondInfo("[DEBUG] except;Lo_AMSDeviceStateRspInfo")



                self.G_ProzenToolhead.dwell(1.0)



                # lancaigang250515:P0 M1multi-material type, need toAMS logical
                if self.G_AMSDevice1IfNormal==True or self.G_AMSDevice2IfNormal==True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] =====multi-materialmulti-material type P0 M1, has AMS')
                    self.G_PhrozenFluiddRespondInfo('[INFO] =====multi-materialmulti-material type P0 M1, P2 A1')
                    self.G_PhrozenFluiddRespondInfo('[INFO] =====multi-materialmulti-material type P0 M1, T?')
                    # lancaigang250722: multi-materialautomaticrefill; 
                    if self.G_AutoReplaceState == 1:
                        self.G_PhrozenFluiddRespondInfo('[INFO] =====multi-materialmulti-material type automatic refill;P0 M1 move switch is P0 M2, refill')
                        self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MA

                        # #lancaigang240416:
                        # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("MA")
                        #     self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MA")
                        # #lancaigang241030:
                        # if self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("MA")
                        #     self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MA")

                        self.Cmds_PrintMode(self.G_AMSDeviceWorkMode)
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MA")
                        self.G_PhrozenFluiddRespondInfo("[INFO] P8")
                        #lancaigang241106: 
                        self.Cmds_CmdP8(gcmd)

                        if self.G_ToolheadIfHaveFilaFlag:
                            #lancaigang250607:
                            self.G_PhrozenFluiddRespondInfo('[INFO] canresume, STM32printing report error')
                            self.G_KlipperQuickPause = True
                            # #lancaigang250427: 
                            # if self.G_SerialPort1OpenFlag == True:
                            #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                            # self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-AMSfinishtimingbuffer-full time")
                            # if self.G_SerialPort2OpenFlag == True:
                            #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                            # self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-AMSfinishtimingbuffer-full time")
                            # #self.G_ProzenToolhead.dwell(1.5)
                            # lancaigang251120: purge, add, preventPG108purge toolheadnofilamentpause, pauseposition in purge, resumewhen will to purge;
                            self.G_PG108Ingoing=1
                            command_string = """
                            PG108
                            """
                            self.G_PhrozenGCode.run_script_from_command(command_string)
                            self.G_PG108Ingoing=0
                            self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)
                            self.G_PhrozenFluiddRespondInfo("[INFO] purgecomplete, toolheaddetectedhas filament, printing")

                    else:
                        self.G_PhrozenFluiddRespondInfo('[INFO] =====multi-materialmulti-material type')
                else:
                    self.G_PhrozenFluiddRespondInfo('[INFO] =====single-colormulti-material type P0 M1, has AMS, P0 M3single-colorprinting, disable disable P2 A1 and T?')
                    self.G_PhrozenFluiddRespondInfo('[INFO] =====P0 M1 switch is P0 M3')
                    self.G_AMSDeviceWorkMode = AMS_WORK_MODE_FILA_RUNOUT
                    self.Cmds_PrintMode(self.G_AMSDeviceWorkMode)
                    self.G_PhrozenFluiddRespondInfo("[WARN] self.G_AMSDeviceWorkMode = AMS_WORK_MODE_FILA_RUNOUT")
                    self.G_P0M1MCNoneAMS=1
                    self.G_PhrozenFluiddRespondInfo("[INFO] self.G_P0M1MCNoneAMS=1")
                    # lancaigang240411: use use filament runoutdetect machine
                    self.G_P0M3Flag = True
                    self.G_PhrozenFluiddRespondInfo("[INFO] self.G_P0M3Flag = True")
                    if self.G_AutoReplaceState == 1:
                        self.G_PhrozenFluiddRespondInfo('[INFO] =====single-colormulti-material type automatic refill;')
                    else:
                        self.G_PhrozenFluiddRespondInfo('[INFO] =====single-colormulti-material type')

                    self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
                    #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
                    if Lo_PauseStatus['is_paused'] == True:
                        self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
                    else:
                        self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")

                    if self.G_ToolheadIfHaveFilaFlag==True:
                        self.G_PhrozenFluiddRespondInfo("[INFO] detectedfilament, startprinting")
                        # lancaigang240412: ifhasmulti-materialAMS, then MAsingle-colorrefill
                        # lancaigang241030: unit AMS, sequence first from unit 1, 1 unit
                        if self.G_AMSDevice1IfNormal==True:
                            self.G_PhrozenFluiddRespondInfo("[WARN] AMS1/tty1 not connected; send P28 first.")
                            self.G_ChangeChannelFirstFilaFlag = True
                        elif self.G_AMSDevice2IfNormal==True:
                            self.G_PhrozenFluiddRespondInfo("[DEBUG] AMS2/tty2 not available; skipping serial port 2.")
                            self.G_ChangeChannelFirstFilaFlag = True
                        else:
                            self.G_PhrozenFluiddRespondInfo('[INFO] detectedfilament, AMSmulti-material not connected, please move filament')
                            # lancaigang240411: use use filament runoutdetect machine
                            self.G_P0M3Flag = True
                            # lancaigang240415: toolheadhasfilament, # time do notpurge
                            self.G_P0M3ToolheadHaveFilaNotSpittingFlag = True
                        # lancaigang251120: purge, add, preventPG108purge toolheadnofilamentpause, pauseposition in purge, resumewhen will to purge;
                        self.G_PG108Ingoing=1
                        command_string = """
                        PG108
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PG108Ingoing=0
                        self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)
                        self.G_PhrozenFluiddRespondInfo("[INFO] purgecomplete, toolheaddetectedhas filament, printing")

                    else:
                        self.G_PhrozenFluiddRespondInfo('[INFO] detectedfilament, pause')
                        # lancaigang240407: call before cannotpause
                        self.Cmds_PhrozenKlipperPauseM2M3NoneCmdToSTM32(None)
                        # lancaigang240411: use use filament runoutdetect machine
                        self.G_P0M3Flag = True
                        #self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                        self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))

                self.G_ProzenToolhead.dwell(0.5)



            # =====M2;multi-material single-colorautomaticrefillmode
            #P0 M2
            #P28
            #P8
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:  #2
                self.G_PhrozenFluiddRespondInfo("[INFO] =====P0 M2single-color refill modemode: MA")

                # lancaigang250520:
                self.ManualCmdFlag=False


                # lancaigang250102: time number
                self.G_PrintCountNum=0

                #lancaigang240416:
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("MA")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 sending command: MA")
                #lancaigang241030:
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("MA")
                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 2 sending command: MA")
    
                self.G_ChangeChannelFirstFilaFlag = True



                self.G_ProzenToolhead.dwell(0.5)

                #time.sleep(1)
                # lancaigang240104: single-colorM2MArefillmodecannotcut filament
                # lancaigang20231219: home/resetcut filament
                #self.Cmds_MoveToCutFilaAndNotRollback(gcmd)


            # =====M3;single-colorfilament runoutdetectmode
            # lancaigang250511: ifexistsAMSmulti-material, then automatic use automaticrefillmode; P0 M3 switch is P0 M2
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:#3
                self.G_PhrozenFluiddRespondInfo('[INFO] P0 M3single-colorfilament')

                # lancaigang250520:
                self.ManualCmdFlag=False

                # lancaigang250102: time number
                self.G_PrintCountNum=0

                if self.G_SerialPort1OpenFlag == True:
                    try:
                        self.G_PhrozenFluiddRespondInfo("[DEBUG] try;Lo_AMSDeviceStateRspInfo")
                        # get multi-materialmainboardstate
                        Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort1SendWaitRsp("SD", sizeof(AMSDetailInfoBytes))
                        self.G_PhrozenFluiddRespondInfo('tty1serial port receive : %s' % Lo_AMSDeviceStateRspInfo)
                        if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                            self.G_PhrozenFluiddRespondInfo("AMS1, please checkAMS1='%s'" % (gcmd.get_commandline(),))
                            # lancaigang240412:AMSmulti-material
                            self.G_AMSDevice1IfNormal=False
                        else:
                            # lancaigang240412:AMSmulti-material
                            self.G_AMSDevice1IfNormal=True
                    except:
                        self.G_PhrozenFluiddRespondInfo("[DEBUG] except;Lo_AMSDeviceStateRspInfo")



                if self.G_SerialPort2OpenFlag == True:
                    try:
                        self.G_PhrozenFluiddRespondInfo("[DEBUG] try;Lo_AMSDeviceStateRspInfo")
                        # get multi-materialmainboardstate
                        Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort2SendWaitRsp("SD", sizeof(AMSDetailInfoBytes))
                        self.G_PhrozenFluiddRespondInfo('tty2serial port receive : %s' % Lo_AMSDeviceStateRspInfo)
                        if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                            self.G_PhrozenFluiddRespondInfo("AMS2, please checkAMS2='%s'" % (gcmd.get_commandline(),))
                            self.G_AMSDevice2IfNormal=False
                        else:
                            self.G_AMSDevice2IfNormal=True
                    except:
                        self.G_PhrozenFluiddRespondInfo("[DEBUG] except;Lo_AMSDeviceStateRspInfo")
                        


                #lancaigang241107:
                if self.G_AMSDevice1IfNormal==True or self.G_AMSDevice2IfNormal==True:
                    #lancaigang241106: 
                    self.G_P0M2MAStartPrintFlag=0

                    # lancaigang250104: preventserial porterror
                    #self.Cmds_CmdP28(None)


                    self.G_PhrozenFluiddRespondInfo('[INFO] =====multi-materialsingle-color type P0 M3')
                    self.G_PhrozenFluiddRespondInfo("self.G_AutoReplaceState=%d" % self.G_AutoReplaceState)



                    # lancaigang250514:ifsingle-colorautomaticrefill, then P0 M3 switch is P0 M2
                    if self.G_AutoReplaceState == 1:
                        # lancaigang250511: ifhasAMSmulti-material, switch is single-colorautomaticrefillmode
                        #P0 M2
                        #P8
                        self.G_PhrozenFluiddRespondInfo('[INFO] =====multi-materialsingle-color type automatic refill;P0 M3 move switch is P0 M2, refill')
                        self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MA
                        self.Cmds_PrintMode(self.G_AMSDeviceWorkMode)
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MA")
                        self.G_PhrozenFluiddRespondInfo("[INFO] P8")
                        #lancaigang241106: 
                        self.Cmds_CmdP8(gcmd)

                        if self.G_ToolheadIfHaveFilaFlag:
                            #lancaigang250607:
                            self.G_PhrozenFluiddRespondInfo('[INFO] canresume, STM32printing report error')
                            self.G_KlipperQuickPause = True
                            # #lancaigang250427: 
                            # if self.G_SerialPort1OpenFlag == True:
                            #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                            # self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-AMSfinishtimingbuffer-full time")
                            # if self.G_SerialPort2OpenFlag == True:
                            #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                            # self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-AMSfinishtimingbuffer-full time")
                            # #self.G_ProzenToolhead.dwell(1.5)

                    else:
                        self.G_PhrozenFluiddRespondInfo('[INFO] =====multi-materialsingle-colorP0 M3, filament runout')
                        self.G_PhrozenFluiddRespondInfo("[INFO] P8")
                        #lancaigang241106: 
                        self.Cmds_CmdP8(gcmd)
                        if self.G_ToolheadIfHaveFilaFlag:
                            #lancaigang250607:
                            self.G_PhrozenFluiddRespondInfo('[INFO] canresume, STM32printing report error')
                            self.G_KlipperQuickPause = True
                            # #lancaigang250427: 
                            # if self.G_SerialPort1OpenFlag == True:
                            #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                            # self.G_PhrozenFluiddRespondInfo("[INFO] serial port1-AMSfinishtimingbuffer-full time")
                            # if self.G_SerialPort2OpenFlag == True:
                            #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                            # self.G_PhrozenFluiddRespondInfo("[INFO] serial port2-AMSfinishtimingbuffer-full time")
                            # #self.G_ProzenToolhead.dwell(1.5)
                        # lancaigang240411: use use filament runoutdetect machine
                        self.G_P0M3Flag = True
                        self.G_PhrozenFluiddRespondInfo("[INFO] self.G_P0M3Flag = True")

                    # #lancaigang250511: ifhasAMSmulti-material, single-colormode
                    # #P0 M3
                    # #P8
                    # #lancaigang250514: multi-materialsingle-color
                    # self.G_PhrozenFluiddRespondInfo("[INFO] P8")
                    # #lancaigang241106: 
                    # self.Cmds_CmdP8(gcmd)

                    # lancaigang241106:toolheadsuccessfulfeed
                    if self.G_P0M2MAStartPrintFlag==1:
                        self.G_PhrozenFluiddRespondInfo("[INFO] toolheadhas filament")
                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] toolheadnonefilament")
                else:
                    self.G_PhrozenFluiddRespondInfo('[INFO] =====single-colorsingle-color type P0 M3')
                    self.G_PhrozenFluiddRespondInfo("self.G_AutoReplaceState=%d" % self.G_AutoReplaceState)


                # lancaigang231220: hasfilament
                for i in range(10):#
                    self.G_ProzenToolhead.dwell(1.0)
                    #time.sleep(1)
                    self.G_PhrozenFluiddRespondInfo('move filament; i=%d' % (i))

                    if self.G_ToolheadIfHaveFilaFlag==True:
                        self.G_PhrozenFluiddRespondInfo("[INFO] detectedfilament, startprinting")

                        # lancaigang240412: ifhasmulti-materialAMS, then MAsingle-colorrefill
                        # lancaigang241030: unit AMS, sequence first from unit 1, 1 unit
                        if self.G_AMSDevice1IfNormal==True:
                            self.G_PhrozenFluiddRespondInfo("[WARN] AMS1/tty1 not connected; send P28 first.")
                            self.G_ChangeChannelFirstFilaFlag = True
                        elif self.G_AMSDevice2IfNormal==True:
                            self.G_PhrozenFluiddRespondInfo("[DEBUG] AMS2/tty2 not available; skipping serial port 2.")
                            self.G_ChangeChannelFirstFilaFlag = True
                        else:
                            self.G_PhrozenFluiddRespondInfo('[INFO] detectedfilament, AMSmulti-material not connected, please move filament')
                            # lancaigang240411: use use filament runoutdetect machine
                            self.G_P0M3Flag = True
                            # lancaigang240415: toolheadhasfilament, # time do notpurge
                            self.G_P0M3ToolheadHaveFilaNotSpittingFlag = True

                        # lancaigang251120: purge, add, preventPG108purge toolheadnofilamentpause, pauseposition in purge, resumewhen will to purge;
                        self.G_PG108Ingoing=1
                        command_string = """
                        PG108
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)
                        self.G_PhrozenFluiddRespondInfo("[INFO] purgecomplete, toolheaddetectedhas filament, printing")
                        self.G_PG108Ingoing=0
                        break
                    if i>=9:
                        # lancaigang240412: ifhasmulti-materialAMS, then MAsingle-colorrefill
                        if self.G_AMSDevice1IfNormal==True:
                            self.G_PhrozenFluiddRespondInfo("[WARN] AMS1/tty1 not connected; send P28 first.")
                            self.G_ChangeChannelFirstFilaFlag = True
                        elif self.G_AMSDevice2IfNormal==True:
                            self.G_PhrozenFluiddRespondInfo("[DEBUG] AMS2/tty2 not available; skipping serial port 2.")
                            self.G_ChangeChannelFirstFilaFlag = True
                        else:
                            self.G_PhrozenFluiddRespondInfo('[INFO] AMSmulti-material not connected, please move filament')
                            self.G_PhrozenFluiddRespondInfo('[INFO] filamentfeedtimeout;pause')
                            

                            self.G_PhrozenFluiddRespondInfo("Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus['is_paused'])
                            #// Current pause state-Lo_PauseStatus='{'is_paused': True}'
                            if Lo_PauseStatus['is_paused'] == True:
                                self.G_PhrozenFluiddRespondInfo("[WARN] Already paused")
                            else:
                                self.G_PhrozenFluiddRespondInfo("[WARN] Not currently paused")
                            
                                # lancaigang240407: call before cannotpause
                                self.Cmds_PhrozenKlipperPauseM2M3NoneCmdToSTM32(None)
                                # lancaigang240411: use use filament runoutdetect machine
                                self.G_P0M3Flag = True

                                #self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                                self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))

                            return
                        
                self.G_ProzenToolhead.dwell(0.5)

            self.G_PhrozenFluiddRespondInfo('[INFO] when 0.5')
            # lancaigang231228: prevent after commandstm32ATcommand
            self.G_ProzenToolhead.dwell(0.5)
            #self.G_ProzenToolhead.dwell(1.0)


        #lancaigang240801: P0 B?
        if "B" in params:
            #self.Cmds_P1CnAutoChangeChannel(int(params["C"]), gcmd)
            # lancaigang240524: use UIUX move
            self.G_PhrozenFluiddRespondInfo(gcmd.get_commandline())







    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Cmds_RegisterCmds(self):
        self.G_PhrozenFluiddRespondInfo('[INFO] [(cmds.python)Cmds_RegisterCmds]registerphrozengcodecommand')
        # P114 S; state; "SB";
        # P114; allstate; "SD";
        self.G_PhrozenGCode.register_command(G_DictPhrozenCmdP114["cmd"],self.Cmds_CmdP114,desc=G_DictPhrozenCmdP114["desc"])
        # P28 connect
        self.G_PhrozenGCode.register_command(G_DictPhrozenCmdP28["cmd"],self.Cmds_CmdP28,desc=G_DictPhrozenCmdP28["desc"])
        # P29 disconnectconnect
        self.G_PhrozenGCode.register_command(G_DictPhrozenCmdP29["cmd"],self.Cmds_CmdP29,desc=G_DictPhrozenCmdP29["desc"])
        # P30 automaticID(use automatic network); "I"; logical automaticIDcommand
        self.G_PhrozenGCode.register_command(G_DictPhrozenCmdP30["cmd"],self.Cmds_CmdP30,desc=G_DictPhrozenCmdP30["desc"])
        # P0 M1; multi-materialmodemode(need connect external all) Yes; "MC";
        # P0 M2; multi-materialsingle-colorrefillmode(need connect external all); "MA";
        # P0 M3; single-colorfilament runoutmode
        #lancaigang240801:
        # P0 B?;;
        self.G_PhrozenGCode.register_command(G_DictPhrozenP0["cmd"],self.Cmds_CmdP0,desc=G_DictPhrozenP0["desc"])
        # P2 A1 allfilamentretractto park positionposition Yes; ====="AP";
        # P2 A2; output allfilament Yes; "CL";
        # P2 A3 filament
        # P2 A4 filament and filament
        # P2 A5 completefilament and filament, cannotto type
        self.G_PhrozenGCode.register_command(G_DictPhrozenCmdP2["cmd"],self.Cmds_CmdP2,desc=G_DictPhrozenCmdP2["desc"])
        # P1 S0 allchannel in park positionfeedto machine state, canfeedto park position or retractto park position; ====="RD";
        # P1 T[n]n:1 ~32(no network, get 1 ~4)manualto channel,(use); ====="T";
        # P1 B[n]n:1 ~32(no network, get 1 ~4)channelfilament full output Yes; ====="B";
        # P1 D[n]; n:1~32(no network, get 1~4); channelfilamentretract in park positionstate Yes; ====="P";
        # P1 C[n] n:1~32(no network, get 1~4) automaticto channel(move action command,cut filament,, wait); ====="T";
        #lancaigang231202:
        # P1 E[n]; n:1~32(no network, get 1~4); channelfilament before switch, need to get output toolhead up filament tube Yes; ====="E?";
        # lancaigang240228: toolheadretract, need tostm32first
        # P1 G[n]; n:1~32(no network, get 1~4); channelfilament Yes; ====="G?";
        # lancaigang240319: special refill, bufferfullrefill
        # =====P1 H[n]; n:1~32(no network, get 1~4); special refill, bufferfullrefill Yes; ====="H?";
        # lancaigang240329: use
        # =====P1 I[n]; manualextrudewhen stm32need torefill; ====="I?";
        # =====P1 J[n]; multi-materialmanualpurge; bufferfullwhen refill;
        # =====P1 K[n]; 
        # =====P1 L[n]; 
        # =====P1 M[n]; 
        # =====P1 N[n]; 
        # =====P1 O[n]; 
        # =====P1 Q[n]; 
        # =====P1 U[n]; 
        #lancaigang240418: 
        # =====P1 V[n]; use version
        # =====P1 W[n]; 
        # =====P1 X[n]; 
        # =====P1 Y[n]; 
        # =====P1 Z[n]; 
        self.G_PhrozenGCode.register_command(G_DictPhrozenCmdP1["cmd"],self.Cmds_CmdP1,desc=G_DictPhrozenCmdP1["desc"])
        # P9 X[x_pos]Y[y_pos]W[width]H[height]D[0/1];x_pos:waitXy_pos:waitYwidth:wait
        # height:waitD0:X action move move Y number (default)D1:Y action move move X number wait
        # P9 T[expire]A[0/1];expire:timeoutwhen,(default60)A0:timeout,(default)A1:timeout after stop waittimeout and logical
        self.G_PhrozenGCode.register_command(G_DictPhrozenCmdP9["cmd"],self.Cmds_CmdP9,desc=G_DictPhrozenCmdP9["desc"])

        #lancaigang241101: 
        # P10 S? parameter number S[1,5]:purge time number, S1-purge1 time, S2-purge2 time ..., purge5 time
        self.G_PhrozenGCode.register_command(G_DictPhrozenCmdP10["cmd"],self.Cmds_CmdP10,desc=G_DictPhrozenCmdP10["desc"])

        #lancaigang250805: 
        # P11 multi-material cutter
        self.G_PhrozenGCode.register_command("P11",self.Cmds_CmdP11,desc="P11")
        #lancaigang250805: 
        # P12 multi-material cutter
        self.G_PhrozenGCode.register_command("P12",self.Cmds_CmdP12,desc="P12")

        # P8 automaticrefill Yes; "FA";
        self.G_PhrozenGCode.register_command(G_DictPhrozenCmdP8["cmd"],self.Cmds_CmdP8,desc=G_DictPhrozenCmdP8["desc"])
        # PRZ_ADC
        self.G_PhrozenGCode.register_command(G_DictPhrozenCmdToolheadAdc["cmd"],self.Cmds_PhrozenAdc,desc=G_DictPhrozenCmdToolheadAdc["desc"])
        # PRZ_PAUSEpause
        self.G_PhrozenGCode.register_command("PRZ_PAUSE",self.Cmds_PhrozenKlipperPauseScreen,desc="PHROZEN_PAUSE")
        # PRZ_RESUME resume
        self.G_PhrozenGCode.register_command("PRZ_RESUME",self.Cmds_PhrozenKlipperResume,desc="PHROZEN_RESUME")
        # PRZ_CANCEL cancel
        self.G_PhrozenGCode.register_command("PRZ_CANCEL",self.Cmds_PhrozenKlipperCancel,desc="PHROZEN_CANCEL")
        # PRZ_VERSION version
        self.G_PhrozenGCode.register_command("PRZ_VERSION",self.Cmds_PhrozenVersion,desc="PHROZEN_VERSION")
        # P4 stop; stop Stopcommand(time first): "SP";
        self.G_PhrozenGCode.register_command(G_DictPhrozenCmdP4["cmd"],self.Cmds_CmdP4,desc=G_DictPhrozenCmdP4["desc"])

        self.G_PhrozenGCode.register_command("PRZ_BM1",self.Cmds_PhrozenBM1,desc="PRZ_BM1")
        self.G_PhrozenGCode.register_command("PRZ_BM0",self.Cmds_PhrozenBM0,desc="PRZ_BM0")

        self.G_PhrozenGCode.register_command("PRZ_PRINT_START",self.Cmds_PrzPrintStart,desc="PRZ_PRINT_START")
        
        #self.G_PhrozenGCode.register_command("PRINT_END",self.Cmds_PrzPrintEnd,desc="PRINT_END")
        #lancaigang250514:
        self.G_PhrozenGCode.register_command("homing_override_end",self.Cmds_HomingOverrideEnd,desc="homing_override_end")

        #lancaigang250115: 
        self.G_PhrozenGCode.register_command("PRZ_RESTORE",self.Cmds_PrzATRestore,desc="PRZ_RESTORE")
        self.G_PhrozenGCode.register_command("PRZ_IDLE",self.Cmds_PrzATIdle,desc="PRZ_IDLE")


        # lancaigang250324: orca file T0 T1 T2 T3command
        self.G_PhrozenGCode.register_command("T0",self.Cmds_CmdT0,desc="T0")
        self.G_PhrozenGCode.register_command("T1",self.Cmds_CmdT1,desc="T1")
        self.G_PhrozenGCode.register_command("T2",self.Cmds_CmdT2,desc="T2")
        self.G_PhrozenGCode.register_command("T3",self.Cmds_CmdT3,desc="T3")
        self.G_PhrozenGCode.register_command("T4",self.Cmds_CmdT4,desc="T4")
        self.G_PhrozenGCode.register_command("T5",self.Cmds_CmdT5,desc="T5")
        self.G_PhrozenGCode.register_command("T6",self.Cmds_CmdT6,desc="T6")
        self.G_PhrozenGCode.register_command("T7",self.Cmds_CmdT7,desc="T7")
        self.G_PhrozenGCode.register_command("T8",self.Cmds_CmdT8,desc="T8")
        self.G_PhrozenGCode.register_command("T9",self.Cmds_CmdT9,desc="T9")
        self.G_PhrozenGCode.register_command("T10",self.Cmds_CmdT10,desc="T10")
        self.G_PhrozenGCode.register_command("T11",self.Cmds_CmdT11,desc="T11")
        self.G_PhrozenGCode.register_command("T12",self.Cmds_CmdT12,desc="T12")
        self.G_PhrozenGCode.register_command("T13",self.Cmds_CmdT13,desc="T13")
        self.G_PhrozenGCode.register_command("T14",self.Cmds_CmdT14,desc="T14")
        self.G_PhrozenGCode.register_command("T15",self.Cmds_CmdT15,desc="T15")







        
