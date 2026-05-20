import os
import numpy as np

import logging
import json
import time
import serial
from .base import *

# ctypedata
from ctypes import *

# Binary payload structures used by the Phrozen AMS serial protocol.
# The original vendor file included long tutorial notes and C snippets.
# Those notes are condensed here to keep behavior unchanged and docs readable.


# Simplified AMS state payload (compact status frame).
class AMSSimpleInfoSt(LittleEndianStructure):  #
    _pack_ = 1
    _fields_ = [
        ("info_flag", c_uint8),  # 8bit;flaglabel
        ("dev_id", c_uint8),  # 8bit;deviceid
        ("end_dev_id", c_uint8),  # 8bit;deviceid
        ("dev_mode", c_uint8),  # 8bit;device
        ("mc_state", c_uint8, 4),  # 4bit;mc
        ("ma_state", c_uint8, 4),  # 4bit;ma
    ]


class AMSSimpleInfoBytes(Union):  #
    _fields_ = [
        ("field", AMSSimpleInfoSt),
        ("whole", c_uint8 * sizeof(AMSSimpleInfoSt)),
    ]


# Detailed AMS state payload (extended status frame).
#     // messageflag(CMD_RSP_SYSTEM_STATE)
#     uint8_t InfoFlag;
#     // currentdeviceID
#     int8_t CurrentDeviceId;
#     // deviceID, ?device
#     int8_t EndDeviceId;
#     //  deviceID, ?devicecurrentdevice
#     int8_t ActiveDeviceId;
#     // deviceID,?deviceneeddevice
#     int8_t TargetDeviceId;
#     // ()
#     uint8_t Others;
#     // (:00  refill mode:01)
#     uint8_t DeviceMode : 2;
#     // (:0)
#     uint8_t IfAnyMotorRuning : 1;
#     // (trigger:1  trigger:0)
#     uint8_t CacheEmptyIfTrigger : 1;
#     // (trigger:1  trigger:0)
#     uint8_t CacheFullIfTrigger : 1;
#     // (trigger:1  trigger:0)
#     uint8_t CacheExistIfTrigger : 1;
#     //
#     uint8_t Reserve : 2;
#     //
#     uint8_t MCStateMachine : 4;
#     // refill mode
#     uint8_t MAStateMachine : 4;
#     // sensor(bit, trigger:1  trigger:0)
#     uint32_t EntryPositionIfTriggerBitMask;
#     // sensor(bit, trigger:1  trigger:0)
#     uint32_t ParkPositionIfTriggerBitMask;
# } St_SystemDetailStatus;
class AMSDetailInfoSt(LittleEndianStructure):  #
    _pack_ = 1
    _fields_ = [
        ("info_flag", c_uint8),  # 8bit;flaglabel
        ("dev_id", c_uint8),  # 8bit;
        ("end_dev_id", c_int8),  # 8bit;
        ("active_dev_id", c_int8),  # 8bit;
        ("target_dev_id", c_int8),  # 8bit;
        ("others", c_uint8),  # 8bit;
        ("dev_mode", c_uint8, 2),  # 2bit;device
        ("any_motor_runing", c_uint8, 1),  # 1bit;#
        ("cache_empty", c_uint8, 1),  # 1bit;#
        ("cache_full", c_uint8, 1),  # 1bit;#
        ("cache_exist", c_uint8, 1),  # 1bit;#
        ("reserve", c_uint8, 2),  # 2bit;
        ("mc_state", c_uint8, 4),  # 4bit;#mc
        ("ma_state", c_uint8, 4),  # 4bit;#ma
        ("entry_state", c_uint32),  # 32bit;#
        ("park_state", c_uint32),  # 32bit;#
    ]


class AMSDetailInfoBytes(Union):  #
    _fields_ = [
        ("field", AMSDetailInfoSt),
        ("whole", c_uint8 * sizeof(AMSDetailInfoSt)),
    ]


class Commands(Base):
    # constructor initialization
    def __init__(self, config):
        super(Commands, self).__init__(config)

        # tty
        self.G_SerialPort1OpenFlag = False

        # tty
        self.G_SerialPort2OpenFlag = False

        # MCfilament change
        self.G_ChangeChannelFirstFilaFlag = True
        #
        self.G_ToolheadFirstInputFila = False  #
        # klipper
        self.G_KlipperIfPaused = False
        # speed
        self.G_MovementSpeedFactor = 1.0 / 60
        # position
        self.G_ToolheadLastPosition = None
        # AMS
        self.G_AMSDeviceWorkMode = AMS_WORK_MODE_UNKNOW  # default

        # 1python (),
        #  1)create:tuple = (1,2,3) data tuple[0]......  tuple[0,2].....tuple[1,2]......
        # 2)modify:modify
        # 3) del tuple
        # 4)function:
        # cmp(tuple1,tuple2):
        # len(tuple):calculate
        # max(tuple):
        # min(tuple):
        # tuple(seq):list
        # 2python []list,list
        # 1)createlistl = [1,2,3,4]datal[0]........
        # 2)listmodify
        # 3)function
        # cmp(list1,list2):
        # len(list):calculate
        # max(list):
        # min(list):
        # list(seq):list
        # list.append(obj):listobject
        # list.pop():data
        # list.remove:list
        # list.sort():
        # list.reverse():list
        # list.count(bj):calculateobjectlist
        # list.insert(index,obj) :positionobject
        # 3python {} dict;dict,
        # 1)createdict:dict = {"a":1,"b":2}. dict:key, value datadict['a'],
        # 2)modify
        # 3):del dict["a"] data  del dict dict dict.clear()dict
        # 4)function
        # cmp(dict1,dict2):
        # len(dict):calculate
        # dict.clear():dictdata
        # dict.get(key, default=None):return,ifreturndefault
        # dict.has_key(key):check,returntrue,false
        # dict.item()listreturnreturn(,)
        # dict.key()returndictkey

        # pythondict
        # P9 X190.290 Y238.700 W2.010 H11.200 D1
        # filamentduringparameter
        self.G_DictChangeChannelWaitAreaParam = {  # python {} dict key-value
            "T": self.G_ChangeChannelTimeout,  # filament changetime(),default120
            "A": 0,  # Action
            "D": 0,  # defaultXY
            "X": 0.0,  # Xcoordinates
            # Vendor note (20231020): "Y": 20.0,      # Ycoordinates
            "W": 0.0,  # X
            "H": 0.0,  # Y
        }

        # parameterconnect
        #
        self.G_ProzenToolhead = None
        # toolhead manual move
        self.G_ToolheadManualMovement = None
        # end
        self.G_ToolheadWaitMovementEnd = None

        # Vendor note (231115): filament change,
        # Vendor note (231216): default0,
        self.G_ChangeChannelTimeoutOldChan = -1
        self.G_ChangeChannelTimeoutOldGcmd = None
        # Vendor note (240912): AMS,log
        self.G_ChangeChannelTimeoutNewChan = -1
        self.G_ChangeChannelTimeoutNewGcmd = None

        # Vendor note (231206): allow
        self.G_ChangeChannelResumeFlag = False

        #
        self.ChangeWaitMoveArea = []  # queue

        # Vendor note (231216): filament changewaiting areaXYcoordinates
        self.G_XBasePosition = 0
        self.G_YBasePosition = 0

        # Vendor note (231216): iffilament changeduring,filament changeduringz,execute,z,
        self.G_IfZPositionLiftUpFlag = False

        self.G_SerialPort1Obj = None

        self.G_SerialPort2Obj = None

        self.G_SerialPortHaveOpenedCount = 0
        self.G_SerialPortIsOpenCount = 0

    def Cmds_MoveTo(self, pos, velocity):
        # object
        if self.G_ProzenToolhead is None:
            return

        #
        self.G_ProzenToolhead.wait_moves()
        # position
        self.G_ToolheadLastPosition = self.G_ProzenToolhead.get_position()

        for index, p in enumerate(pos):
            self.G_ToolheadLastPosition[index] = p

        # toolhead manual move
        self.G_ProzenToolhead.manual_move(
            self.G_ToolheadLastPosition, velocity * self.G_MovementSpeedFactor
        )
        #
        self.G_ProzenToolhead.wait_moves()

    #
    def Cmds_AMSSerial1Send(self, cmd):
        if self.G_SerialPort1OpenFlag == False:
            self.kaos_log("DEBUG", "AMS1/tty1 not connected; send P28 first.", "SERIAL")
            try:
                self.kaos_log(
                    "DEBUG", "[(cmds.py)Cmds_AMSSerial1Send]Reinitializing serial port 1", "SERIAL"
                )
                self.G_SerialPort1Obj = serial.Serial(
                    self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3
                )
                # serial port opened successfully
                if self.G_SerialPort1Obj is not None:
                    if self.G_SerialPort1Obj.is_open:
                        self.G_SerialPort1OpenFlag = True
                        self.kaos_log("DEBUG", "Reinitializing serial port 1 successful", "SERIAL")
                        # Vendor note (231213): open serial port
                        self.G_SerialPort1Obj.flushInput()  # clean serial write cache
                        self.G_SerialPort1Obj.flush()
                        self.kaos_log("DEBUG", "Serial port 1 buffers cleared", "SERIAL")
                        self.kaos_log("DEBUG", "Re-registering serial port 1 callback", "SERIAL")
                        self.G_SerialPort1RecvTimmer = self.G_PhrozenReactor.register_timer(
                            self.Device_TimmerUart1Recv, self.G_PhrozenReactor.NOW
                        )
            except:
                self.kaos_log(
                    "DEBUG",
                    "Unable to open tty1. Check the USB connection or try rebooting.",
                    "SERIAL",
                )
            return

        self.kaos_log(
            "DEBUG", "[(cmds.python)Cmds_AMSSerial1Send]Sending command: cmd=%s" % cmd, "SERIAL"
        )

        try:
            self.kaos_log("DEBUG", "before sending, Reinitializing serial port 1", "SERIAL")
            self.G_SerialPort1Obj = serial.Serial(
                self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3
            )
            # serial port opened successfully
            if self.G_SerialPort1Obj is not None:
                if self.G_SerialPort1Obj.is_open:
                    # tty1
                    # lock
                    if self.Base_AMSSerialCmdLock():
                        self.G_SerialPort1Obj.flush()
                        # tty1
                        self.G_SerialPort1Obj.write(cmd.encode())  # .encode()
                        self.kaos_log("DEBUG", "self.G_SerialPort1Obj.write", "SERIAL")
                        self.G_SerialPort1Obj.flush()
                        # unlock
                        self.Base_AMSSerialCmdUnlock()

        except:
            self.kaos_log(
                "DEBUG", "Unable to open tty1. Check the USB connection or try rebooting.", "SERIAL"
            )
            # [Translated vendor note] 1, python (),
            # [Translated vendor note] 1): tuple = (1,2,3) tuple[0]...... tuple[0,2].....tuple[1,2]......
            # [Translated vendor note] 2):
            # [Translated vendor note] 3) del tuple
            # [Translated vendor note] 4):
            # [Translated vendor note] cmp(tuple1, tuple2):
            # [Translated vendor note] len(tuple):
            # [Translated vendor note] max(tuple):
            # [Translated vendor note] min(tuple):
            # [Translated vendor note] tuple(seq):
            # [Translated vendor note] 2, python [],
            # [Translated vendor note] 1)l = [1,2,3,4]l[0]........
            # [Translated vendor note] 2)
            # [Translated vendor note] 3)
            # [Translated vendor note] cmp(list1, list2):
            # [Translated vendor note] len(list):
            # [Translated vendor note] max(list):
            # [Translated vendor note] min(list):
            # [Translated vendor note] list(seq):
            # [Translated vendor note] list.append(obj):
            # [Translated vendor note] list.pop():
            # [Translated vendor note] list.remove:
            # [Translated vendor note] list.sort():
            # [Translated vendor note] list.reverse():
            # [Translated vendor note] list.count(bj):
            # [Translated vendor note] list.insert(index,obj) :
            # [Translated vendor note] 3, python {} ; ,
            # [Translated vendor note] 1): dict = {"a":1,"b":2}. : key, value dict['a'],
            # [Translated vendor note] 2)
            # [Translated vendor note] 3): del dict["a"] del dict dict.clear()
            # [Translated vendor note] 4)
            # [Translated vendor note] cmp(dict1, dict2):
            # [Translated vendor note] len(dict):
            # [Translated vendor note] dict.clear():
            # [Translated vendor note] dict.get(key, default=None):, default
            # [Translated vendor note] dict.has_key(key):, true, false
            # [Translated vendor note] dict.item()(, )
            # [Translated vendor note] dict.key()key

            # unlock
            self.Base_AMSSerialCmdUnlock()

    def Cmds_AMSSerial2Send(self, cmd):
        if self.G_SerialPort2OpenFlag == False:
            self.kaos_log("DEBUG", "AMS2/tty2 not available; skipping serial port 2.", "SERIAL")
            try:
                self.kaos_log(
                    "DEBUG", "[(cmds.py)Cmds_AMSSerial2Send]Reinitializing serial port 2", "SERIAL"
                )
                self.G_SerialPort2Obj = serial.Serial(
                    self.G_Serialport2Define, SERIAL_PORT_BAUD, timeout=3
                )
                # serial port opened successfully
                if self.G_SerialPort2Obj is not None:
                    if self.G_SerialPort2Obj.is_open:
                        self.G_SerialPort2OpenFlag = True
                        self.kaos_log("DEBUG", "Reinitializing serial port 2 successful", "SERIAL")
                        self.G_SerialPort2Obj.flushInput()  # clean serial write cache
                        self.G_SerialPort2Obj.flush()
                        self.kaos_log("DEBUG", "Serial port 2 buffers cleared", "SERIAL")
                        self.kaos_log("DEBUG", "Re-registering serial port 2 callback", "SERIAL")
                        self.G_SerialPort2RecvTimmer = self.G_PhrozenReactor.register_timer(
                            self.Device_TimmerUart2Recv, self.G_PhrozenReactor.NOW
                        )
            except:
                self.kaos_log(
                    "DEBUG",
                    "Unable to open tty2. Check the USB connection or try rebooting.",
                    "SERIAL",
                )
            return

        self.kaos_log(
            "DEBUG", "[(cmds.python)Cmds_AMSSerial2Send]Sending command: cmd=%s" % cmd, "SERIAL"
        )

        try:
            self.kaos_log("DEBUG", "before sending, Reinitializing serial port 2", "SERIAL")
            self.G_SerialPort2Obj = serial.Serial(
                self.G_Serialport2Define, SERIAL_PORT_BAUD, timeout=3
            )
            # serial port opened successfully
            if self.G_SerialPort2Obj is not None:
                if self.G_SerialPort2Obj.is_open:
                    # tty1
                    # lock
                    if self.Base_AMSSerialCmdLock():
                        self.G_SerialPort2Obj.flush()
                        # tty1
                        self.G_SerialPort2Obj.write(cmd.encode())  # .encode()
                        self.kaos_log("DEBUG", "self.G_SerialPort2Obj.write", "SERIAL")
                        self.G_SerialPort2Obj.flush()
                        # unlock
                        self.Base_AMSSerialCmdUnlock()
                        self.kaos_log("DEBUG", "self.G_SerialPort2Obj.write", "SERIAL")
        except:
            self.kaos_log(
                "DEBUG", "Unable to open tty2. Check the USB connection or try rebooting.", "SERIAL"
            )
            # unlock
            self.Base_AMSSerialCmdUnlock()

    # serial send and wait for response
    def Cmds_AMSSerialPort1SendWaitRsp(self, cmd, res_len):
        if self.G_SerialPort1OpenFlag == False:
            self.kaos_log("DEBUG", "AMS1/tty1 not connected; send P28 first.", "SERIAL")
            return

        self.kaos_log(
            "DEBUG",
            "[(cmds.python)Cmds_AMSSerialPort1SendWaitRsp]Sending command: cmd=%s" % cmd,
            "SERIAL",
        )

        try:
            # serial port opened successfully
            if self.G_SerialPort1Obj is not None:
                if self.G_SerialPort1Obj.is_open:
                    #
                    # lock
                    if self.Base_AMSSerialCmdLock():
                        # tty1
                        self.G_SerialPort1Obj.write(cmd.encode())
                        self.kaos_log("DEBUG", "self.G_SerialPort1Obj.write", "SERIAL")
                        self.G_SerialPort1Obj.flush()
                        # tty1read
                        resp = self.G_SerialPort1Obj.read(res_len)
                        # unlock
                        self.Base_AMSSerialCmdUnlock()
                        return resp
        except:
            self.kaos_log(
                "DEBUG", "Unable to open tty1. Check the USB connection or try rebooting.", "SERIAL"
            )
            # unlock
            self.Base_AMSSerialCmdUnlock()

    # serial send and wait for response
    def Cmds_AMSSerialPort2SendWaitRsp(self, cmd, res_len):
        if self.G_SerialPort2OpenFlag == False:
            self.kaos_log("DEBUG", "AMS2/tty2 not available; skipping serial port 2.", "SERIAL")
            return

        self.kaos_log(
            "DEBUG",
            "[(cmds.python)Cmds_AMSSerialPort2SendWaitRsp]Sending command: cmd=%s" % cmd,
            "SERIAL",
        )

        # #
        # #lock
        # if self.Base_AMSSerialCmdLock():
        #     #tty2
        #     self.G_SerialPort2Obj.write(cmd.encode())
        #     self.G_SerialPort2Obj.flush()
        #     #tty2read
        #     resp = self.G_SerialPort2Obj.read(res_len)
        #     #unlock
        #     self.Base_AMSSerialCmdUnlock()
        #     return resp
        try:
            # serial port opened successfully
            if self.G_SerialPort2Obj is not None:
                if self.G_SerialPort2Obj.is_open:
                    #
                    # lock
                    if self.Base_AMSSerialCmdLock():
                        # tty2
                        self.G_SerialPort2Obj.write(cmd.encode())
                        self.kaos_log("DEBUG", "self.G_SerialPort2Obj.write", "SERIAL")
                        self.G_SerialPort2Obj.flush()
                        # tty2read
                        resp = self.G_SerialPort2Obj.read(res_len)
                        # unlock
                        self.Base_AMSSerialCmdUnlock()
                        return resp
        except:
            self.kaos_log(
                "DEBUG", "Unable to open tty2. Check the USB connection or try rebooting.", "SERIAL"
            )
            # unlock
            self.Base_AMSSerialCmdUnlock()

    # P1 E[n];n:1~32(device,1~4);,need Yes;====="E?";
    def Cmds_P1EnForceForward(self, chan, gcmd):
        self.kaos_log(
            "DEBUG", "[(cmds.python)Cmds_P1EnForceForward]Sending command: E%d" % chan, "SERIAL"
        )

        # for UIUX dynamic interface
        self.emit_channel_op("E", 0, self.G_ChangeChannelTimeoutNewChan)
        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()
        if chan in range(1, 5):
            self.Cmds_AMSSerial1Send("E%d" % chan)
            self.kaos_log("DEBUG", "Serial port 1 sending command: E%d" % chan, "SERIAL")
        elif chan in range(5, 9):
            self.Cmds_AMSSerial2Send("E%d" % (chan - 4))
            self.kaos_log("DEBUG", "Serial port 2 sending command: E%d" % (chan - 4), "SERIAL")

        # for UIUX dynamic interface
        self.emit_channel_op("E", 1, self.G_ChangeChannelTimeoutNewChan)

    # Vendor note (240228): distance,needstm32distance
    # P1 G[n];n:1~32(device,1~4);distance Yes;====="G?";
    def Cmds_P1GnExtruderBack(self, chan, gcmd):
        self.kaos_log(
            "DEBUG", "[(cmds.python)Cmds_P1EnForceForward]Sending command: G%d" % chan, "SERIAL"
        )

        # for UIUX dynamic interface
        self.emit_channel_op("G", 0, self.G_ChangeChannelTimeoutNewChan)
        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()
        if chan in range(1, 5):
            self.Cmds_AMSSerial1Send("G%d" % chan)
            self.kaos_log("DEBUG", "Serial port 1 sending command: G%d" % chan, "SERIAL")
        elif chan in range(5, 9):
            self.Cmds_AMSSerial2Send("G%d" % (chan - 4))
            self.kaos_log("DEBUG", "Serial port 2 sending command: G%d" % (chan - 4), "SERIAL")

        # for UIUX dynamic interface
        self.emit_channel_op("G", 1, self.G_ChangeChannelTimeoutNewChan)

    def Cmds_P1HnSpecialInfila(self, chan, gcmd):
        self.kaos_log(
            "DEBUG", "[(cmds.python)Cmds_P1HnSpecialInfila]Sending command: H%d" % chan, "SERIAL"
        )

        # for UIUX dynamic interface
        self.emit_channel_op("H", 0, self.G_ChangeChannelTimeoutNewChan)
        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()
        if chan in range(1, 5):
            self.Cmds_AMSSerial1Send("H%d" % chan)
            self.kaos_log("DEBUG", "Serial port 1 sending command: H%d" % chan, "SERIAL")
        elif chan in range(5, 9):
            self.Cmds_AMSSerial2Send("H%d" % (chan - 4))
            self.kaos_log("DEBUG", "Serial port 2 sending command: H%d" % (chan - 4), "SERIAL")

        # for UIUX dynamic interface
        self.emit_channel_op("H", 1, self.G_ChangeChannelTimeoutNewChan)

    def Cmds_P1InExtrudeManualIn(self, value):
        command_string = """
                        # M106 S0
                        # M83
                        # G92 E0
                        G1 E%f F300
                        """ % (
            # value,
        )
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.kaos_log(
            "DEBUG",
            "[(cmds.python)Cmds_P1InExtrudeManualIn]G-code command: %s" % command_string,
            "SERIAL",
        )

    # =====P1 J[n];;;
    def Cmds_P1JnManualSpitFila(self, chan, gcmd):
        self.kaos_log(
            "DEBUG", "[(cmds.python)Cmds_P1JnManualSpitFila]Sending command P1J?", "SERIAL"
        )
        self.kaos_log("DEBUG", "chan=%d;" % chan, "SERIAL")
        self.emit_channel_op("J", 0, self.G_ChangeChannelTimeoutNewChan)

        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()
        if chan in range(1, 5):
            self.Cmds_AMSSerial1Send("J%d" % chan)
            self.kaos_log("DEBUG", "Serial port 1 sending command: J%d" % chan, "SERIAL")
        elif chan in range(5, 9):
            self.Cmds_AMSSerial2Send("J%d" % (chan - 4))
            self.kaos_log("DEBUG", "Serial port 2 sending command: J%d" % (chan - 4), "SERIAL")

        self.emit_channel_op("J", 1, self.G_ChangeChannelTimeoutNewChan)

    # =====P1 I[n];stm32need;====="I?";?-
    # I2,I3,I0idle
    def Cmds_P1InExtruderBack(self, value, gcmd):
        self.kaos_log("DEBUG", "[(cmds.python)Cmds_P1InExtruderBack]Sending command I?", "SERIAL")
        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()
        # for UIUX dynamic interface
        self.emit_channel_op("I", 0, self.G_ChangeChannelTimeoutNewChan)

        # Vendor note (0415): I2,I3
        if value > 0:
            self.kaos_log("DEBUG", "Sending command: I2; extrude", "SERIAL")

            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("I2")
                self.kaos_log("DEBUG", "Serial port 1 sending command: I2", "SERIAL")
            elif self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("I2")
                self.kaos_log("DEBUG", "Serial port 2 sending command: I2", "SERIAL")

            # Vendor note (240516): preventtime too close
            # Vendor note (240705): prevent;time too close
            self.G_ProzenToolhead.dwell(0.5)

            # time.sleep(2)
            self.kaos_log("DEBUG", "time.sleep(2)", "SERIAL")
        elif value < 0:
            self.kaos_log("DEBUG", "Sending command: I3; retract", "SERIAL")

            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("I3")
                self.kaos_log("DEBUG", "Serial port 1 sending command: I3", "SERIAL")
            elif self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("I3")
                self.kaos_log("DEBUG", "Serial port 2 sending command: I3", "SERIAL")

            # Vendor note (240516): preventtime too close
            # Vendor note (240705): prevent;time too close
            self.G_ProzenToolhead.dwell(0.52)

            # time.sleep(2)
            self.kaos_log("DEBUG", "time.sleep(2)", "SERIAL")
        elif value == 0:
            self.kaos_log("DEBUG", "Sending command: AT+IDLE; IDLE state", "SERIAL")

            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("AT+IDLE")
                self.kaos_log("DEBUG", "Serial port 1 sending command: AT+IDLE", "SERIAL")
            elif self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("AT+IDLE")
                self.kaos_log("DEBUG", "Serial port 2 sending command: AT+IDLE", "SERIAL")

            # Vendor note (240516): preventtime too close
            # Vendor note (240705): prevent;time too close
            self.G_ProzenToolhead.dwell(0.5)

            # time.sleep(2)
            self.kaos_log("DEBUG", "time.sleep(2)", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Sending command: none", "SERIAL")

        self.Cmds_P1InExtrudeManualIn(value)

        # self.Cmds_AMSSerial1Send("AT+IDLE")
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1EnForceForward]Sending command: AT+IDLE; IDLE state")
        # for UIUX dynamic interface
        self.emit_channel_op("I", 1, self.G_ChangeChannelTimeoutNewChan)

    # P1 B?;filament
    def Cmds_P1BnWholeRollbackAction(self, chan, gcmd):
        self.kaos_log(
            "DEBUG",
            "[(cmds.python)Cmds_P1BnWholeRollbackAction]Sending command: B%d" % chan,
            "SERIAL",
        )
        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()
        # for UIUX dynamic interface
        self.emit_channel_op("B", 0, self.G_ChangeChannelTimeoutNewChan)

        if chan in range(1, 5):
            self.Cmds_AMSSerial1Send("B%d" % chan)
            self.kaos_log("DEBUG", "Serial port 1 sending command: B%d" % chan, "SERIAL")
        elif chan in range(5, 9):
            self.Cmds_AMSSerial2Send("B%d" % (chan - 4))
            self.kaos_log("DEBUG", "Serial port 2 sending command: B%d" % (chan - 4), "SERIAL")

        # Vendor note (240115): ifcurrent,cancheck
        if self.G_ChangeChannelTimeoutNewChan == chan:
            # Vendor note (240113): filamentdetect
            if self.G_ToolheadIfHaveFilaFlag == True:
                self.G_ProzenToolhead.dwell(6.0)
                # Vendor note (231201): checknormal,normal
                self.Cmds_CutFilaIfNormalCheck()

        # for UIUX dynamic interface
        self.emit_channel_op("B", 1, self.G_ChangeChannelTimeoutNewChan)

    # P1 D?;filament
    def Cmds_P1DnMoveToParkPositonAction(self, chan, gcmd):
        self.kaos_log(
            "DEBUG",
            "[(cmds.python)Cmds_P1DnMoveToParkPositonAction]Sending command: P%d" % chan,
            "SERIAL",
        )
        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()
        # for UIUX dynamic interface
        self.emit_channel_op("D", 0, self.G_ChangeChannelTimeoutNewChan)

        if chan in range(1, 5):
            self.Cmds_AMSSerial1Send("P%d" % chan)
            self.kaos_log("DEBUG", "Serial port 1 sending command: P%d" % chan, "SERIAL")
        elif chan in range(5, 9):
            self.Cmds_AMSSerial2Send("P%d" % (chan - 4))
            self.kaos_log("DEBUG", "Serial port 2 sending command: P%d" % (chan - 4), "SERIAL")

        # Vendor note (240115): ifcurrent,cancheck
        if self.G_ChangeChannelTimeoutNewChan == chan:
            # Vendor note (240113): filamentdetect
            if self.G_ToolheadIfHaveFilaFlag == True:
                self.G_ProzenToolhead.dwell(6.0)
                # Vendor note (231201): checknormal,normal
                self.Cmds_CutFilaIfNormalCheck()

        # for UIUX dynamic interface
        self.emit_channel_op("D", 1, self.G_ChangeChannelTimeoutNewChan)

    def Cmds_MoveToCutFilaPrepare(self):
        self.kaos_log(
            "DEBUG",
            "[(cmds.python)Cmds_MoveToCutFilaPrepare]Prepare before cutting filament",
            "SERIAL",
        )

        # self.Cmds_AMSSerial1Send("H%d" % self.G_ChangeChannelTimeoutNewChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]Special refill state before cut: H%d" % self.G_ChangeChannelTimeoutNewChan)
        # time.sleep(1)

        # # Vendor note (240319): ,filament,prevent
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)]External macro command-PG102; before cut, purge residual filament to prevent small bits")
        # self.PG102Flag=True
        # self.G_PhrozenFluiddRespondInfo("[(dev.python)]self.Flag=True")
        # command_string = """
        # PG102
        # """
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)]command_string='%s'" % command_string)

        # # for i in range(15):
        # #     self.G_PhrozenFluiddRespondInfo("[(dev.python)]Purging, waiting")
        # #     # Vendor note (20231013): 4
        # #     # Vendor note (231115): 1s
        # #     self.G_ProzenToolhead.dwell(1.0)
        # #     # Vendor note (240125): cannotsleep,block
        # #     #time.sleep(1)
        # self.PG102Flag=False
        # self.G_PhrozenFluiddRespondInfo("[(dev.python)]self.Flag=False")

    # position
    def Cmds_MoveToCutFilaAction(self, gcmd):
        # // [(cmds.python)Cmds_MoveToCutFilaAction];gcode=
        # // G91
        # // G1 Z1.200000 F3000
        # // [(cmds.python)Cmds_MoveToCutFilaAction];gcode=
        # // G90
        # // G1 X301.500000 Y0.000000 F24000
        # // G1 X308.500000 F600
        # // G1 X301.500000 F7200
        self.kaos_log(
            "DEBUG", "[(cmds.python)Cmds_MoveToCutFilaAction]cut filament;sending command", "SERIAL"
        )

        # for UIUX dynamic interface
        self.emit_channel_op("Cut", 0, self.G_ChangeChannelTimeoutNewChan)

        # # Vendor note (231208): ,
        # # # 0=defaultgcodeexecute
        # # Vendor note (231208): z+
        # # Vendor note (231215): Z
        # #if self.G_ChangeChannelIfZLiftingUpByGcode == 0:
        # command_string = """
        #     G91
        #     G1 Z%f F500
        #     """ % (
        #     self.G_AMSFilaCutZPositionLiftingUp,
        # )
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # # Vendor note (231216): iffilament changeduring,filament changeduringz,execute,z,
        # self.G_IfZPositionLiftUpFlag = True
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_MoveToCutFilaAction]Cut filament; Z-axis raise; gcode command=%s" % command_string)
        # self.G_ProzenToolhead.wait_moves()
        # self.G_ProzenToolhead.dwell(1.0)

        # self.Cmds_AMSSerial1Send("H%d" % self.G_ChangeChannelTimeoutNewChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]Special refill state before cut: H%d" % self.G_ChangeChannelTimeoutNewChan)
        # time.sleep(1)

        # X Yposition,,return
        # // G91
        # // G1 Z5.000000 F5000
        # // G90
        # // G1 X302.000000 Y244.100000 F5000
        # // G1 X309.000000 F500
        # // G1 X302.000000 F5000
        # // G1 X290 F5000
        command_string = """
            # G91
            G1 Z%f F8000
            # G90
            G1 X%f Y%f F10000
            G1 X%f Y%f F500
            # G4 P500
            G1 X%f Y%f F8000
            G1 X%f F5000
            # G91
            G1 Z-%f F8000
            """ % (
            self.G_AMSFilaCutZPositionLiftingUp,
            self.G_AMSFilaCutXPosition - 7,
            self.G_AMSFilaCutYPosition,  # Vendor note (240409): self.G_AMSFilaCutXPosition,
            self.G_AMSFilaCutYPosition,
            self.G_AMSFilaCutXPosition - 7,
            self.G_AMSFilaCutYPosition,  # Vendor note (241217): self.G_AMSFilaCutXPosition-30,# Vendor note (250807): self.G_AMSFilaCutZPositionLiftingUp,
        )
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.kaos_log(
            "DEBUG", "[TOOLHEAD] Z raised and filament cut; gcode=%s" % command_string, "SERIAL"
        )
        self.G_ProzenToolhead.wait_moves()

        # self.G_IfZPositionLiftUpFlag = True

        # # Vendor note (240110): waiting areabefore,execute,position
        # command_string = """
        #     PG101
        #     """
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_MoveToCutFilaAction]-position;command_string='%s'" % command_string)
        # self.IfDoPG102Flag=True

        # for UIUX dynamic interface
        self.emit_channel_op("Cut", 1, self.G_ChangeChannelTimeoutNewChan)

        # # Vendor note (231207): prevent,0.5;speed,
        # command = """
        #     G92 E0
        #     G1 E0.5 F300
        #     G92 E0
        # """
        # self.G_PhrozenGCode.run_script_from_command(command)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_MoveToCutFilaAction]Cut filament; gcode command=%s" % command)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_MoveToCutFilaAction]Prevent filament jamming cutter, extrude down 0.5mm")

    def Cmds_MoveToCutFilaAbsolutePositionNotReset(self, gcmd):
        self.kaos_log(
            "DEBUG",
            "[(cmds.python)Cmds_MoveToCutFilaAbsolutePositionNotReset]home/reset cut filament, absolute position;sending command=%s"
            % (gcmd.get_commandline()),
            "SERIAL",
        )

        # # Vendor note (231208): z+
        # # command_string = """
        # #     G91
        # #     G1 Z10 F3000
        # #     """
        # # self.G_PhrozenGCode.run_script_from_command(command_string)
        # # Vendor note (231215): Z
        # command_string = """
        #     G91
        #     G1 Z%f F500
        #     """ % (
        #     self.G_AMSFilaCutZPositionLiftingUp,
        # )
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # # Vendor note (231216): iffilament changeduring,filament changeduringz,execute,z,
        # self.G_IfZPositionLiftUpFlag = True
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_MoveToCutFilaAbsolutePositionNotReset]Z-axis raise 10mm=%s" % command_string)
        # self.G_ProzenToolhead.wait_moves()
        # self.G_ProzenToolhead.dwell(1.0)

        # self.Cmds_AMSSerial1Send("H%d" % self.G_ChangeChannelTimeoutNewChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]Special refill state before cut: H%d" % self.G_ChangeChannelTimeoutNewChan)
        # time.sleep(1)

        # for UIUX dynamic interface
        self.emit_channel_op("Cut", 0, self.G_ChangeChannelTimeoutNewChan)

        # X Yposition,,return
        # // G91
        # // G1 Z5.000000 F5000
        # // G90
        # // G1 X302.000000 Y244.100000 F5000
        # // G1 X309.000000 F500
        # // G1 X302.000000 F5000
        # // G1 X290 F5000
        command_string = """
            # G91
            G1 Z%f F8000
            # G90
            G1 X%f Y%f F10000
            G1 X%f Y%f F500
            # G4 P500
            G1 X%f Y%f F8000
            G1 X%f F5000
            # G91
            G1 Z-%f F8000
            """ % (
            self.G_AMSFilaCutZPositionLiftingUp,
            self.G_AMSFilaCutXPosition - 7,
            self.G_AMSFilaCutYPosition,  # Vendor note (240409): self.G_AMSFilaCutXPosition,
            self.G_AMSFilaCutYPosition,
            self.G_AMSFilaCutXPosition - 7,
            self.G_AMSFilaCutYPosition,  # Vendor note (241217): self.G_AMSFilaCutXPosition-30,# Vendor note (250807): self.G_AMSFilaCutZPositionLiftingUp,
        )
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.kaos_log(
            "DEBUG", "[TOOLHEAD] Z raised and filament cut; gcode=%s" % command_string, "SERIAL"
        )
        self.G_ProzenToolhead.wait_moves()

        # self.G_IfZPositionLiftUpFlag = True

        # # Vendor note (240110): waiting areabefore,execute,
        # command_string = """
        #     PG107
        #     """
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # self.G_PhrozenFluiddRespondInfo("-;command_string='%s'" % command_string)
        # self.IfDoPG102Flag=True

        # for UIUX dynamic interface
        self.emit_channel_op("Cut", 1, self.G_ChangeChannelTimeoutNewChan)

        # # Vendor note (231207): prevent,0.5;speed,
        # command = """
        #     G92 E0
        #     G1 E0.5 F300
        #     G92 E0
        # """
        # self.G_PhrozenGCode.run_script_from_command(command)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_MoveToCutFilaAction]Cut filament; gcode command=%s" % command)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_MoveToCutFilaAction]Prevent filament jamming cutter, extrude down 0.5mm")

    def Cmds_MoveToCutFilaAbsolutePositionNotResetAndRollback(self, gcmd):
        self.kaos_log(
            "DEBUG",
            "[(cmds.python)Cmds_MoveToCutFilaAbsolutePositionNotResetAndRollback]home/reset cut filament, absolute position;sending command=%s"
            % (gcmd.get_commandline()),
            "SERIAL",
        )
        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()
        # # Vendor note (231208): z+
        # # command_string = """
        # #     G91
        # #     G1 Z10 F3000
        # #     """
        # # self.G_PhrozenGCode.run_script_from_command(command_string)
        # # Vendor note (231215): Z
        # command_string = """
        #     G91
        #     G1 Z%f F500
        #     """ % (
        #     self.G_AMSFilaCutZPositionLiftingUp,
        # )
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # # Vendor note (231216): iffilament changeduring,filament changeduringz,execute,z,
        # self.G_IfZPositionLiftUpFlag = True
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_MoveToCutFilaAbsolutePositionNotReset]Z-axis raise 10mm=%s" % command_string)
        # self.G_ProzenToolhead.wait_moves()
        # self.G_ProzenToolhead.dwell(1.0)

        # self.Cmds_AMSSerial1Send("H%d" % self.G_ChangeChannelTimeoutNewChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]Special refill state before cut: H%d" % self.G_ChangeChannelTimeoutNewChan)
        # time.sleep(1)

        # for UIUX dynamic interface
        self.emit_channel_op("Cut", 0, self.G_ChangeChannelTimeoutNewChan)

        # X Yposition,,return
        # // G91
        # // G1 Z5.000000 F5000
        # // G90
        # // G1 X302.000000 Y244.100000 F5000
        # // G1 X309.000000 F500
        # // G1 X302.000000 F5000
        # // G1 X290 F5000
        command_string = """
            # G91
            G1 Z%f F8000
            # G90
            G1 X%f Y%f F10000
            G1 X%f Y%f F500
            # G4 P500
            G1 X%f Y%f F8000
            G1 X%f F5000
            # G91
            G1 Z-%f F8000
            """ % (
            self.G_AMSFilaCutZPositionLiftingUp,
            self.G_AMSFilaCutXPosition - 7,
            self.G_AMSFilaCutYPosition,  # Vendor note (240409): self.G_AMSFilaCutXPosition,
            self.G_AMSFilaCutYPosition,
            self.G_AMSFilaCutXPosition - 7,
            self.G_AMSFilaCutYPosition,  # Vendor note (241217): self.G_AMSFilaCutXPosition-30,# Vendor note (250807): self.G_AMSFilaCutZPositionLiftingUp,
        )
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.kaos_log(
            "DEBUG", "[TOOLHEAD] Z raised and filament cut; gcode=%s" % command_string, "SERIAL"
        )
        self.G_ProzenToolhead.wait_moves()

        # #self.G_IfZPositionLiftUpFlag = True
        # # Vendor note (240110): waiting areabefore,execute,
        # command_string = """
        #     PG107
        #     """
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # self.G_PhrozenFluiddRespondInfo("-;command_string='%s'" % command_string)
        # self.IfDoPG102Flag=True

        # for UIUX dynamic interface
        self.emit_channel_op("Cut", 1, self.G_ChangeChannelTimeoutNewChan)

        # # Vendor note (231207): prevent,0.5;speed,
        # command = """
        #     G92 E0
        #     G1 E0.5 F300
        #     G92 E0
        # """
        # self.G_PhrozenGCode.run_script_from_command(command)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_MoveToCutFilaAction]Cut filament; gcode command=%s" % command)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_MoveToCutFilaAction]Prevent filament jamming cutter, extrude down 0.5mm")

        if self.G_SerialPort1OpenFlag == True:
            self.kaos_log(
                "DEBUG",
                "serial port 1Sending command: AP, fully retract to the park position",
                "SERIAL",
            )
            # // all retract to park;//===== P2 A1  Yes;"AP";
            self.Cmds_AMSSerial1Send("AP")
        if self.G_SerialPort2OpenFlag == True:
            self.kaos_log(
                "DEBUG",
                "serial port 2Sending command: AP, fully retract to the park position",
                "SERIAL",
            )
            # // all retract to park;//===== P2 A1  Yes;"AP";
            self.Cmds_AMSSerial2Send("AP")

        # Vendor note (240913): self.G_ProzenToolhead.dwell(6.0)
        # Vendor note (231201): checknormal,normal
        self.Cmds_CutFilaIfNormalCheck()

    def Cmds_MoveToCutFilaAndNotRollback(self, gcmd):
        self.kaos_log(
            "DEBUG",
            "[(cmds.python)Cmds_MoveToCutFilaAndNotRollback]cut filament;sending command=%s"
            % (gcmd.get_commandline()),
            "SERIAL",
        )
        # for UIUX dynamic interface
        self.emit_channel_op("Zero", 0, self.G_ChangeChannelTimeoutNewChan)
        # Vendor note (20231019): ,auto filament changeif1filament,needfilament
        # Vendor note (20231020): detect
        # if self.G_ToolheadIfHaveFilaFlag:
        # 0=defaultgcodeexecute
        # Vendor note (231128): G28PG28
        # if self.G_ChangeChannelIfZLiftingUpByGcode == 0:
        self.kaos_log("DEBUG", "allhomeandcut filament", "SERIAL")
        command_string = """
        # G28
        """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.kaos_log("DEBUG", "home/reset=%s" % command_string, "SERIAL")
        # Vendor note (20231020): gcode,need,time,,auto filament change
        # G92 E0
        # G1 E0.0000 F600
        # G91
        # G1 E-0.385 F8000
        # for UIUX dynamic interface
        self.emit_channel_op("Zero", 1, self.G_ChangeChannelTimeoutNewChan)

        self.kaos_log("DEBUG", "cut filament", "SERIAL")

        # Vendor note (20231013): self.Cmds_MoveToCutFilaAction(gcmd)

    def Cmds_MoveToCutFilaAndHomingXY(self, gcmd):
        self.kaos_log(
            "DEBUG", "[(cmds.python)Cmds_MoveToCutFilaAndHomingXY]cut filament;XYreset", "SERIAL"
        )
        # for UIUX dynamic interface
        self.emit_channel_op("Zero", 0, self.G_ChangeChannelTimeoutNewChan)
        # Vendor note (20231019): ,auto filament changeif1filament,needfilament
        # Vendor note (20231020): detect
        # if self.G_ToolheadIfHaveFilaFlag:
        # 0=defaultgcodeexecute
        # Vendor note (231128): G28PG28
        # if self.G_ChangeChannelIfZLiftingUpByGcode == 0:
        self.kaos_log("DEBUG", "G28homeYandcut filament", "SERIAL")
        command_string = """
        # G28 Y0
        """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.kaos_log("DEBUG", "Y0 reset=%s" % command_string, "SERIAL")
        self.kaos_log("DEBUG", "G28homeXandcut filament", "SERIAL")
        command_string = """
        # G28 X0
        """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.kaos_log("DEBUG", "X0 reset=%s" % command_string, "SERIAL")
        # Vendor note (20231020): gcode,need,time,,auto filament change
        # G92 E0
        # G1 E0.0000 F600
        # G91
        # G1 E-0.385 F8000
        # for UIUX dynamic interface
        self.emit_channel_op("Zero", 1, self.G_ChangeChannelTimeoutNewChan)

        self.kaos_log("DEBUG", "cut filament", "SERIAL")

        # Vendor note (20231013): self.Cmds_MoveToCutFilaAction(gcmd)

    def Cmds_MoveToCutFilaAndRollback(self, gcmd):
        number = 50
        self.kaos_log(
            "DEBUG",
            "[(cmds.python)Cmds_MoveToCutFilaAndRollback]cut filament;sending command",
            "SERIAL",
        )

        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()

        # Vendor note (20231019): ,auto filament changeif1filament,needfilament
        # Vendor note (20231020): detect
        # if self.G_ToolheadIfHaveFilaFlag:
        # # 0=defaultgcodeexecute
        # Vendor note (231128): G28PG28
        # Vendor note (240319): GCODEPG28,PG28
        # Vendor note (240323): ,
        # if self.G_ChangeChannelIfZLiftingUpByGcode == 0:
        #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_MoveToCutFilaAndRollback]Home all and cut filament")
        #     command_string = """
        #     PG28
        #     """
        #     self.G_PhrozenGCode.run_script_from_command(command_string)
        #     # Vendor note (20231020): gcode,need,time,,auto filament change
        #     # G92 E0
        #     # G1 E0.0000 F600
        #     # G91
        #     # G1 E-0.385 F8000

        self.kaos_log("DEBUG", "cut filament", "SERIAL")
        # Vendor note (20231013): self.Cmds_MoveToCutFilaAction(gcmd)

        self.G_ProzenToolhead.dwell(2.0)

        if self.G_SerialPort1OpenFlag == True:
            self.kaos_log(
                "DEBUG",
                "serial port 1Sending command: AP, fully retract to the park position",
                "SERIAL",
            )
            # // all retract to park;//===== P2 A1  Yes;"AP";
            self.Cmds_AMSSerial1Send("AP")
        if self.G_SerialPort2OpenFlag == True:
            self.kaos_log(
                "DEBUG",
                "serial port 2Sending command: AP, fully retract to the park position",
                "SERIAL",
            )
            # // all retract to park;//===== P2 A1  Yes;"AP";
            self.Cmds_AMSSerial2Send("AP")

        # Vendor note (240913): self.G_ProzenToolhead.dwell(6.0)
        # Vendor note (231201): checknormal,normal
        self.Cmds_CutFilaIfNormalCheck()

        # if self.G_ToolheadIfHaveFilaFlag:
        #     for i in range(number):
        #             time.sleep(1)
        #             i += 1
        #             self.G_PhrozenFluiddRespondInfo('Filament in toolhead, AP command retracting; i=%d' % i)

        #             if i >= number:
        #                 self.G_PhrozenFluiddRespondInfo('AP command timeout; number=%d' % number)
        #                 break

    def Cmds_PhrozenKlipperPauseCommon(self):
        self.kaos_log(
            "DEBUG", "=====[(cmds.python)Cmds_PhrozenKlipperPauseCommon]klipper pause", "SERIAL"
        )
        self.kaos_log("DEBUG", "=====PAUSE=====", "SERIAL")
        self.kaos_log(
            "DEBUG",
            "=====PAUSE=====self.G_ChangeChannelTimeoutOldChan=%d"
            % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====PAUSE=====self.G_ChangeChannelTimeoutNewChan=%d"
            % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")

        # Vendor note (250526): ,allowgcode,complete
        self.G_KlipperInPausing = True
        self.kaos_log("DEBUG", "pause, not allowednew gcode command, need pause complete", "SERIAL")

        if self.IfDoPG102Flag == True:
            self.kaos_log("DEBUG", "self.IfDoPG102Flag==True", "SERIAL")
            self.IfDoPG102Flag = False

            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPauseCommon]External macro command-PG104")
            # command_string = """
            # PG104
            # """
            # self.G_PhrozenGCode.run_script_from_command(command_string)
            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPauseCommon]-;command_string='%s'" % command_string)
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

        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:  # 0
            self.kaos_log("DEBUG", "in printingmode, do not executepausePAUSEcommand", "SERIAL")
        else:
            Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
            self.kaos_log(
                "DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
            )
            # // current-Lo_PauseStatus='{'is_paused': True}'
            if Lo_PauseStatus["is_paused"] == True:
                self.kaos_log("DEBUG", "Already paused", "SERIAL")
            else:
                self.kaos_log("DEBUG", "Not currently paused", "SERIAL")

                # # Vendor note (250527): pause at feed waiting area
                # self.G_PhrozenFluiddRespondInfo("[SERVICE] Start external macro PRZ_PAUSE_WAITINGAREA")
                # command = """
                # PRZ_PAUSE_WAITINGAREA
                # """
                # self.G_PhrozenGCode.run_script_from_command(command)
                # self.G_PhrozenFluiddRespondInfo("[SERVICE] End external macro: command=%s" % (command))

                # Vendor note (250527): execute
                if self.G_KlipperQuickPause == True:
                    self.G_KlipperQuickPause = False

                    self.kaos_log(
                        "DEBUG", "[SERVICE] Start external macro PRZ_PAUSE_WAITINGAREA", "SERIAL"
                    )
                    command = """
                    # PRZ_PAUSE_WAITINGAREA
                    """
                    self.G_PhrozenGCode.run_script_from_command(command)

                    # Vendor note (240119): cfgconfig
                    self.kaos_log("DEBUG", "[PRINT] Start external macro PAUSE_PRINTING", "SERIAL")
                    command = """
                    # PAUSE_PRINTING
                    """
                    self.G_PhrozenGCode.run_script_from_command(command)
                    self.kaos_log("DEBUG", "self.G_ProzenToolhead.wait_moves()", "SERIAL")
                    self.G_ProzenToolhead.wait_moves()
                    self.kaos_log(
                        "DEBUG", "[SERVICE] End external macro: command=%s" % (command), "SERIAL"
                    )
                    # self.G_PhrozenFluiddRespondInfo("Prevent pause failure, extra command; send_pause_command")
                    # self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                else:
                    self.G_KlipperQuickPause = False

                    # # Vendor note (250716): needpause zone
                    # self.G_PhrozenFluiddRespondInfo("[SERVICE] Start external macro PRZ_PAUSE_WAITINGAREA")
                    # command = """
                    # PRZ_PAUSE_WAITINGAREA
                    # """
                    # self.G_PhrozenGCode.run_script_from_command(command)

                    # Vendor note (240119): cfgconfig
                    self.kaos_log("DEBUG", "[PRINT] Start external macro PAUSE", "SERIAL")
                    command = """
                    # PAUSE
                    """
                    self.G_PhrozenGCode.run_script_from_command(command)
                    self.kaos_log("DEBUG", "self.G_ProzenToolhead.wait_moves()", "SERIAL")
                    self.G_ProzenToolhead.wait_moves()
                    self.kaos_log(
                        "DEBUG", "[SERVICE] End external macro: command=%s" % (command), "SERIAL"
                    )
                    # self.G_PhrozenFluiddRespondInfo("Prevent pause failure, extra command; send_pause_command")
                    # self.G_PhrozenPrinterCancelPauseResume.send_pause_command()

                # # Vendor note (250527): pause at feed waiting area
                # self.G_PhrozenFluiddRespondInfo("[SERVICE] Start external macro PRZ_PAUSE_WAITINGAREA")
                # command = """
                # PRZ_PAUSE_WAITINGAREA
                # """
                # self.G_PhrozenGCode.run_script_from_command(command)
                # self.G_PhrozenFluiddRespondInfo("[SERVICE] End external macro: command=%s" % (command))

                # Vendor note (240125): ,allowstm23
                self.STM32ReprotPauseFlag = 1
                self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=1", "SERIAL")

                self.G_KlipperIfPaused = True
                self.kaos_log("DEBUG", "self.G_KlipperIfPaused = True", "SERIAL")
                self.kaos_log("DEBUG", "klipper pause;", "SERIAL")

        # Vendor note (240325): filament changefailed,cannotexecute
        self.G_MCModeCanResumeFlag = False

        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")

        self.G_KlipperInPausing = False
        self.kaos_log("DEBUG", "pause complete, allowednew gcode command", "SERIAL")

        # Vendor note (250607): print status: 1=retracting; 2=feeding; 3=printing; 4=paused
        self.G_KlipperPrintStatus = 4

        # # #
        # # command = """
        # #     G90
        # #     G1 X150 Y10 F5400
        # # """
        # # self.G_PhrozenGCode.run_script_from_command(command)
        # # self.G_PhrozenFluiddRespondInfo("[(cmds.python)]command=%s" % (command))
        # # #gcodeafterwait_movesfunction
        # # # Vendor note (231202): wait_movesklipper
        # # # Vendor note (231207): cannotwait_moves,gcode
        # # self.G_ProzenToolhead.wait_moves()
        # # self.G_PhrozenFluiddRespondInfo("[(cmds.python)]Move to front for user access")
        # #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_MoveToCutFilaAction]Cut filament; gcode command=%s" % command)
        # #klipper pause;currentx y zcoordinates
        # # Vendor note (240108): datanormal,validate
        # self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PRZ_PAUSE_STATE")
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)]SAVE_GCODE_STATE")
        # self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
        # #self.G_ProzenToolhead.wait_moves()
        # self.G_ProzenToolhead.dwell(1.0)

        # #time.sleep(1)
        # #self.G_ProzenToolhead.wait_moves()
        # #self.G_PhrozenFluiddRespondInfo("[(cmds.python)]wait_moves")
        # # Vendor note (231219): klipper
        # # Vendor note (230103): # #
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

    def Cmds_PhrozenKlipperPauseM2M3ToSTM32(self, gcmd):
        _ = gcmd

        # Vendor note (231115): checkgcmd,klipper
        if gcmd is None:
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)Cmds_PhrozenKlipperPauseM2M3ToSTM32]self.G_PhrozenFluiddRespondInfo;gcmd to is empty",
                "SERIAL",
            )
            self.kaos_log("DEBUG", "self.G_PhrozenFluiddRespondInfo;klipper pause", "SERIAL")
            # pass
        if gcmd is not None:
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)Cmds_PhrozenKlipperPauseM2M3ToSTM32]command='%s'"
                % (gcmd.get_commandline(),),
                "SERIAL",
            )
            self.kaos_log("DEBUG", "klipper pause", "SERIAL")
            # pass
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPauseM2M3ToSTM32]command='%s'" % (gcmd.get_commandline(),))

        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()

        # Vendor note (250526): ,allowgcode,complete
        self.G_KlipperInPausing = True
        self.kaos_log("DEBUG", "pause, not allowednew gcode command, need pause complete", "SERIAL")

        # Vendor note (231129): position
        # self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PRZ_PAUSE_STATE")
        # self.G_KlipperIfPaused = True
        # #
        # Vendor note (231202): wait_movesklipper
        # Vendor note (231207): cannotwait_moves,gcode
        # self.G_ProzenToolhead.wait_moves()

        # time.sleep(1)
        # Vendor note (231201): klipper pause,stm32
        # // AT+PAUSE
        # // AT+PAUSE=8
        # // AT+PAUSE=9

        if self.G_SerialPort1OpenFlag == True:
            self.Cmds_AMSSerial1Send("AT+PAUSE")
            self.kaos_log("DEBUG", "serial port 1 send AT+PAUSEpausestm32 machine", "SERIAL")
        if self.G_SerialPort2OpenFlag == True:
            self.Cmds_AMSSerial2Send("AT+PAUSE")
            self.kaos_log("DEBUG", "serial port 2 send AT+PAUSEpausestm32 machine", "SERIAL")

        # Vendor note (240125): encapsulated function
        self.Cmds_PhrozenKlipperPauseCommon()

        self.G_KlipperIfPaused = True
        self.kaos_log("DEBUG", "self.G_KlipperIfPaused = True", "SERIAL")
        self.kaos_log("DEBUG", "klipper pause;", "SERIAL")

        self.G_KlipperInPausing = False
        self.kaos_log("DEBUG", "pause complete, allowednew gcode command", "SERIAL")

    def Cmds_PhrozenKlipperPauseMAToSTM32(self, gcmd):
        _ = gcmd

        # Vendor note (231115): checkgcmd,klipper
        if gcmd is None:
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)Cmds_PhrozenKlipperPauseMAToSTM32]self.G_PhrozenFluiddRespondInfo;gcmd to is empty",
                "SERIAL",
            )
            self.kaos_log("DEBUG", "self.G_PhrozenFluiddRespondInfo;klipper pause", "SERIAL")
            # pass
        if gcmd is not None:
            self.kaos_log("DEBUG", "command='%s'" % (gcmd.get_commandline(),), "SERIAL")
            self.kaos_log("DEBUG", "klipper pause", "SERIAL")
            # pass
        # self.G_PhrozenFluiddRespondInfo("command='%s'" % (gcmd.get_commandline(),))

        # Vendor note (250526): ,allowgcode,complete
        self.G_KlipperInPausing = True
        self.kaos_log("DEBUG", "pause, not allowednew gcode command, need pause complete", "SERIAL")

        # time.sleep(1)

        # # Vendor note (241031): # if self.G_SerialPort1OpenFlag == True:
        #     self.Cmds_AMSSerial1Send("AT+PAUSE")
        #     self.G_PhrozenFluiddRespondInfo("serial port 1 send AT+PAUSEpausestm32 machine")
        # # Vendor note (241030): # if self.G_SerialPort2OpenFlag == True:
        #     self.Cmds_AMSSerial2Send("AT+PAUSE")
        #     self.G_PhrozenFluiddRespondInfo("serial port 2 send AT+PAUSEpausestm32 machine")

        if self.IfDoPG102Flag == True:
            self.kaos_log("DEBUG", "self.IfDoPG102Flag==True", "SERIAL")
            self.IfDoPG102Flag = False

        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")

        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:  # 0
            self.kaos_log("DEBUG", "in printingmode, do not executepausePAUSEcommand", "SERIAL")
        else:

            # Vendor note (240119): cfgconfig
            self.kaos_log("DEBUG", "[PRINT] Start external macro PAUSEMA", "SERIAL")
            command = """
            # PAUSEMA
            """
            self.G_PhrozenGCode.run_script_from_command(command)
            self.kaos_log("DEBUG", "[PRINT] Calling macro command=%s" % (command), "SERIAL")
            # self.G_PhrozenFluiddRespondInfo("Prevent pause failure, extra command; send_pause_command")
            # self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
            self.kaos_log("DEBUG", "[PRINT] End external macro PAUSEMA", "SERIAL")

            # Vendor note (240125): ,allowstm23
            self.STM32ReprotPauseFlag = 1
            self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=1", "SERIAL")
            self.G_KlipperIfPaused = True
            self.kaos_log("DEBUG", "self.G_KlipperIfPaused = True", "SERIAL")
            self.kaos_log("DEBUG", "klipper pause;", "SERIAL")

        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")

        # Vendor note (240325): filament changefailed,cannotexecute
        self.G_MCModeCanResumeFlag = False

        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        self.G_KlipperInPausing = False
        self.kaos_log("DEBUG", "pause complete, allowednew gcode command", "SERIAL")

    def Cmds_PhrozenKlipperPauseM2M3NoneCmdToSTM32(self, gcmd):
        _ = gcmd

        # Vendor note (231115): checkgcmd,klipper
        if gcmd is None:
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)Cmds_PhrozenKlipperPauseM2M3NoneCmdToSTM32]self.G_PhrozenFluiddRespondInfo;gcmd to is empty",
                "SERIAL",
            )
            self.kaos_log("DEBUG", "self.G_PhrozenFluiddRespondInfo;klipper pause", "SERIAL")
            # pass
        if gcmd is not None:
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)Cmds_PhrozenKlipperPauseM2M3NoneCmdToSTM32]command='%s'"
                % (gcmd.get_commandline(),),
                "SERIAL",
            )
            self.kaos_log("DEBUG", "klipper pause", "SERIAL")
            # pass
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPauseM2M3NoneCmdToSTM32]command='%s'" % (gcmd.get_commandline(),))

        # Vendor note (231129): position
        # self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PRZ_PAUSE_STATE")
        # self.G_KlipperIfPaused = True
        # #
        # Vendor note (231202): wait_movesklipper
        # Vendor note (231207): cannotwait_moves,gcode
        # self.G_ProzenToolhead.wait_moves()

        # Vendor note (250526): ,allowgcode,complete
        self.G_KlipperInPausing = True
        self.kaos_log("DEBUG", "pause, not allowednew gcode command, need pause complete", "SERIAL")

        # Vendor note (240125): encapsulated function
        self.Cmds_PhrozenKlipperPauseCommon()

        self.G_KlipperIfPaused = True
        self.kaos_log("DEBUG", "self.G_KlipperIfPaused = True", "SERIAL")
        self.kaos_log("DEBUG", "klipper pause;", "SERIAL")

        self.G_KlipperInPausing = False
        self.kaos_log("DEBUG", "pause complete, allowednew gcode command", "SERIAL")

    # PRZ_PAUSE(system,)
    # AT+PAUSE
    def Cmds_PhrozenKlipperPauseNoneCmdToSTM32(self, gcmd):
        _ = gcmd

        # Vendor note (231130): ,execute
        # if self.G_KlipperIfPaused == True:
        #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPauseNoneCmdToSTM32]Already in Klipper pause state")
        #     return

        # Vendor note (231115): checkgcmd,klipper
        if gcmd is None:
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)Cmds_PhrozenKlipperPauseNoneCmdToSTM32]self.G_PhrozenFluiddRespondInfo;gcmd to is empty",
                "SERIAL",
            )
            self.kaos_log("DEBUG", "self.G_PhrozenFluiddRespondInfo;klipper pause", "SERIAL")
            # pass
        if gcmd is not None:
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)Cmds_PhrozenKlipperPauseNoneCmdToSTM32]command='%s'"
                % (gcmd.get_commandline(),),
                "SERIAL",
            )
            self.kaos_log("DEBUG", "klipper pause", "SERIAL")
            # pass
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPauseNoneCmdToSTM32]command='%s'" % (gcmd.get_commandline(),))

        # Vendor note (231129): position
        # self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PRZ_PAUSE_STATE")
        # self.G_KlipperIfPaused = True
        # #
        # Vendor note (231202): wait_movesklipper
        # Vendor note (231207): cannotwait_moves,gcode
        # self.G_ProzenToolhead.wait_moves()

        # Vendor note (250526): ,allowgcode,complete
        self.G_KlipperInPausing = True
        self.kaos_log("DEBUG", "pause, not allowednew gcode command, need pause complete", "SERIAL")

        # Vendor note (240125): encapsulated function
        self.Cmds_PhrozenKlipperPauseCommon()

        self.G_KlipperIfPaused = True
        self.kaos_log("DEBUG", "self.G_KlipperIfPaused = True", "SERIAL")
        self.kaos_log("DEBUG", "klipper pause;", "SERIAL")

        self.G_KlipperInPausing = False
        self.kaos_log("DEBUG", "pause complete, allowednew gcode command", "SERIAL")

    def Cmds_PhrozenKlipperPauseToolheadCutFailsure(self, gcmd):
        _ = gcmd

        # Vendor note (231115): checkgcmd,klipper
        if gcmd is None:
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)Cmds_PhrozenKlipperPauseToolheadCutFailsure]self.G_PhrozenFluiddRespondInfo;gcmd to is empty",
                "SERIAL",
            )
            self.kaos_log("DEBUG", "self.G_PhrozenFluiddRespondInfo;klipper pause", "SERIAL")
            # pass
        if gcmd is not None:
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)Cmds_PhrozenKlipperPauseToolheadCutFailsure]command='%s'"
                % (gcmd.get_commandline(),),
                "SERIAL",
            )
            self.kaos_log("DEBUG", "klipper pause", "SERIAL")
            # pass
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPauseToolheadCutFailsure]command='%s'" % (gcmd.get_commandline(),))

        # Vendor note (231129): position
        # self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PRZ_PAUSE_STATE")
        # self.G_KlipperIfPaused = True
        # #
        # Vendor note (231207): cannotwait_moves,gcode
        # self.G_ProzenToolhead.wait_moves()

        # Vendor note (250526): ,allowgcode,complete
        self.G_KlipperInPausing = True
        self.kaos_log("DEBUG", "pause, not allowednew gcode command, need pause complete", "SERIAL")

        # Vendor note (240125): encapsulated function
        self.Cmds_PhrozenKlipperPauseCommon()

        self.G_KlipperIfPaused = True
        self.kaos_log("DEBUG", "self.G_KlipperIfPaused = True", "SERIAL")
        self.kaos_log("DEBUG", "klipper pause;", "SERIAL")

        self.G_KlipperInPausing = False
        self.kaos_log("DEBUG", "pause complete, allowednew gcode command", "SERIAL")

    def Cmds_PhrozenKlipperPauseChangeChannelTimeout(self, gcmd):
        _ = gcmd

        # Vendor note (231115): checkgcmd,klipper
        if gcmd is None:
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)Cmds_PhrozenKlipperPauseChangeChannelTimeout]self.G_PhrozenFluiddRespondInfo;gcmd to is empty",
                "SERIAL",
            )
            self.kaos_log("DEBUG", "self.G_PhrozenFluiddRespondInfo;klipper pause", "SERIAL")
            # pass
        if gcmd is not None:
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)Cmds_PhrozenKlipperPauseChangeChannelTimeout]command='%s'"
                % (gcmd.get_commandline(),),
                "SERIAL",
            )
            self.kaos_log("DEBUG", "klipper pause", "SERIAL")
            # pass
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPauseChangeChannelTimeout]command='%s'" % (gcmd.get_commandline(),))

        # Vendor note (231129): position
        # self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PRZ_PAUSE_STATE")
        # self.G_KlipperIfPaused = True
        # #
        # Vendor note (231207): cannotwait_moves,gcode
        # self.G_ProzenToolhead.wait_moves()

        # Vendor note (250526): ,allowgcode,complete
        self.G_KlipperInPausing = True
        self.kaos_log("DEBUG", "pause, not allowednew gcode command, need pause complete", "SERIAL")

        # Vendor note (240125): encapsulated function
        self.Cmds_PhrozenKlipperPauseCommon()

        self.G_KlipperIfPaused = True
        self.kaos_log("DEBUG", "self.G_KlipperIfPaused = True", "SERIAL")
        self.kaos_log("DEBUG", "klipper pause;", "SERIAL")

        self.G_KlipperInPausing = False
        self.kaos_log("DEBUG", "pause complete, allowednew gcode command", "SERIAL")

    # PRZ_PAUSE(system,)
    # AT+PAUSE
    def Cmds_PhrozenKlipperPause(self, gcmd):
        _ = gcmd
        self.kaos_log(
            "DEBUG", "=====[(cmds.python)Cmds_PhrozenKlipperPause]klipper pause", "SERIAL"
        )
        # Vendor note (231130): ,execute
        # if self.G_KlipperIfPaused == True:
        #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPause]Already in Klipper pause state")
        #     return

        # if self.G_ChangeChannelResumeFlag:
        #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPause]Resuming previous action, pause not allowed")
        #     return

        # # Vendor note (231216): # eventtime = self.G_PhrozenReactor.monotonic()
        # # Determine "printing" status
        # idle_timeout = self.G_PhrozenPrinter.lookup_object("idle_timeout")
        # is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
        # if is_printing:
        #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPause];='%s'" % (gcmd.get_commandline(),))
        # else:
        #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPause];='%s;return'" % (gcmd.get_commandline(),))
        #     return

        # Vendor note (231115): checkgcmd,klipper
        if gcmd is None:
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)Cmds_PhrozenKlipperPause]self.G_PhrozenFluiddRespondInfo;gcmd to is empty",
                "SERIAL",
            )
            self.kaos_log("DEBUG", "self.G_PhrozenFluiddRespondInfo;klipper pause", "SERIAL")
            # pass
        if gcmd is not None:
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)Cmds_PhrozenKlipperPause]command='%s'" % (gcmd.get_commandline(),),
                "SERIAL",
            )
            self.kaos_log("DEBUG", "klipper pause", "SERIAL")
            # pass
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperPause]command='%s'" % (gcmd.get_commandline(),))

        self.kaos_log("DEBUG", "Current mode", "SERIAL")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.emit_mode(0, "unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.emit_mode(1, "MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.emit_mode(2, "MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.emit_mode(3, "RUNOUT")
        else:
            self.emit_mode(-1, "error")

        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()

        # Vendor note (250526): ,allowgcode,complete
        self.G_KlipperInPausing = True
        self.kaos_log("DEBUG", "pause, not allowednew gcode command, need pause complete", "SERIAL")

        # Vendor note (250526): ,allowgcode,complete
        self.G_KlipperInPausing = True

        # time.sleep(1)

        # Vendor note (231201): klipper pause,stm32
        # // AT+PAUSE
        # // AT+PAUSE=8
        # // AT+PAUSE=9

        if self.G_SerialPort1OpenFlag == True:
            self.Cmds_AMSSerial1Send("AT+PAUSE")
            self.kaos_log("DEBUG", "serial port 1 send AT+PAUSEpausestm32 machine", "SERIAL")
        if self.G_SerialPort2OpenFlag == True:
            self.Cmds_AMSSerial2Send("AT+PAUSE")
            self.kaos_log("DEBUG", "serial port 2 send AT+PAUSEpausestm32 machine", "SERIAL")

        # Vendor note (231129): position
        # self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PRZ_PAUSE_STATE")
        # self.G_KlipperIfPaused = True
        # #
        # Vendor note (231207): cannotwait_moves,gcode
        # self.G_ProzenToolhead.wait_moves()

        # Vendor note (240125): encapsulated function
        self.Cmds_PhrozenKlipperPauseCommon()

        self.G_KlipperIfPaused = True
        self.kaos_log("DEBUG", "self.G_KlipperIfPaused = True", "SERIAL")
        self.kaos_log("DEBUG", "klipper pause;", "SERIAL")

        self.G_KlipperInPausing = False
        self.kaos_log("DEBUG", "pause complete, allowednew gcode command", "SERIAL")

    # PRZ_RESUME (PRZ_PAUSE)
    def Cmds_PhrozenKlipperResumeCommon(self):
        self.kaos_log(
            "DEBUG", "=====[(cmds.python)Cmds_PhrozenKlipperResumeCommon]klipper resume", "SERIAL"
        )

        # # Vendor note (240103): # #self.G_ProzenToolhead.dwell(3.0)
        # velocity = 2400
        # self.G_PhrozenGCode.run_script_from_command(
        #     "RESTORE_GCODE_STATE NAME=PRZ_PAUSE_STATE MOVE=1 MOVE_SPEED=%.4f"
        #     % (velocity)
        # )
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResumeCommon]RESTORE_GCODE_STATE")
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResumeCommon]NAME=PRZ_PAUSE_STATE")
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResumeCommon]send_resume_command")
        # #klipper
        # self.G_PhrozenPrinterCancelPauseResume.send_resume_command()
        # #self.G_ProzenToolhead.wait_moves()
        # self.G_ProzenToolhead.dwell(2.0)
        # # # Vendor note (240103): Z,
        # # command_string = """
        # #     G90
        # #     G91
        # #     G1 Z-10 F3000
        # #     """
        # # self.G_PhrozenGCode.run_script_from_command(command_string)

        self.kaos_log("DEBUG", "Current mode", "SERIAL")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.emit_mode(0, "unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.emit_mode(1, "MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.emit_mode(2, "MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.emit_mode(3, "RUNOUT")
        else:
            self.emit_mode(-1, "error")

        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "is pause state, need to resume", "SERIAL")
            # Vendor note (240119): cfgconfig
            self.kaos_log("DEBUG", "[PRINT] External macro RESUME", "SERIAL")
            command = """
            # RESUME
            """
            self.G_PhrozenGCode.run_script_from_command(command)
            self.kaos_log("DEBUG", "[PRINT] Calling macro command=%s" % (command), "SERIAL")

            self.G_PauseToLCDString = ""
        else:
            self.kaos_log("DEBUG", "Not currently paused, do notthen resume", "SERIAL")

        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )

        # Vendor note (240325): dataafter,needflag
        self.G_MCModeCanResumeFlag == False
        self.G_KlipperIfPaused = False
        # Vendor note (240124): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0

        # Vendor note (250607): print status: 1=retracting; 2=feeding; 3=printing; 4=paused
        self.G_KlipperPrintStatus = 3

        # Vendor note (250619): ifusb10s,
        self.G_ASM1DisconnectErrorCount = 0

    # PRZ_RESUME (PRZ_PAUSE)
    def Cmds_PhrozenKlipperResume(self, gcmd):
        _ = gcmd
        self.kaos_log("DEBUG", "=====[(cmds.py)Cmds_PhrozenKlipperResume]", "SERIAL")
        self.emit_resume(1, self.G_ChangeChannelTimeoutNewChan)
        self.kaos_log("DEBUG", "=====RESUME=====", "SERIAL")
        self.kaos_log(
            "DEBUG",
            "=====RESUME=====self.G_ChangeChannelTimeoutOldChan=%d"
            % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====RESUME=====self.G_ChangeChannelTimeoutNewChan=%d"
            % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")

        # Vendor note (240511): on resume, reinitialize serial to handle AMS hot-plug serial errors
        try:
            self.kaos_log(
                "DEBUG",
                "[(cmds.py)Cmds_PhrozenKlipperResume]Reinitializing serial port 1",
                "SERIAL",
            )
            self.G_SerialPort1Obj = serial.Serial(
                self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3
            )
            # serial port opened successfully
            if self.G_SerialPort1Obj is not None:
                if self.G_SerialPort1Obj.is_open:
                    self.G_SerialPort1OpenFlag = True
                    self.kaos_log("DEBUG", "Reinitializing serial port 1 successful", "SERIAL")
                    # Vendor note (231213): open serial port
                    self.G_SerialPort1Obj.flushInput()  # clean serial write cache
                    self.G_SerialPort1Obj.flush()
                    self.kaos_log("DEBUG", "Serial port 1 buffers cleared", "SERIAL")
                    self.kaos_log("DEBUG", "Re-registering serial port 1 callback", "SERIAL")
                    self.G_SerialPort1RecvTimmer = self.G_PhrozenReactor.register_timer(
                        self.Device_TimmerUart1Recv, self.G_PhrozenReactor.NOW
                    )
        except:
            self.kaos_log(
                "DEBUG", "Unable to open tty1. Check the USB connection or try rebooting.", "SERIAL"
            )

        try:
            self.kaos_log(
                "DEBUG",
                "[(cmds.py)Cmds_PhrozenKlipperResume]Reinitializing serial port 2",
                "SERIAL",
            )
            self.G_SerialPort2Obj = serial.Serial(
                self.G_Serialport2Define, SERIAL_PORT_BAUD, timeout=3
            )
            # serial port opened successfully
            if self.G_SerialPort2Obj is not None:
                if self.G_SerialPort2Obj.is_open:
                    self.G_SerialPort2OpenFlag = True
                    self.kaos_log("DEBUG", "Reinitializing serial port 2 successful", "SERIAL")
                    self.G_SerialPort2Obj.flushInput()  # clean serial write cache
                    self.G_SerialPort2Obj.flush()
                    self.kaos_log("DEBUG", "Serial port 2 buffers cleared", "SERIAL")
                    self.kaos_log("DEBUG", "Re-registering serial port 2 callback", "SERIAL")
                    self.G_SerialPort2RecvTimmer = self.G_PhrozenReactor.register_timer(
                        self.Device_TimmerUart2Recv, self.G_PhrozenReactor.NOW
                    )
        except:
            self.kaos_log(
                "DEBUG", "Unable to open tty2. Check the USB connection or try rebooting.", "SERIAL"
            )

        self.kaos_log("DEBUG", "Current mode", "SERIAL")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.emit_mode(0, "unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.emit_mode(1, "MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.emit_mode(2, "MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.emit_mode(3, "RUNOUT")
        else:
            self.emit_mode(-1, "error")

        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:  # 0
            self.kaos_log("DEBUG", "in printingmode, do not executeresume,return", "SERIAL")
            self.emit_resume(2, self.G_ChangeChannelTimeoutNewChan)
            return

        if self.PG102Flag == True:
            self.kaos_log("DEBUG", "normal in purge, not allowedresume", "SERIAL")
            self.emit_resume(2, self.G_ChangeChannelTimeoutNewChan)
            return

        # # Vendor note (231216): # eventtime = self.G_PhrozenReactor.monotonic()
        # # Determine "printing" status
        # idle_timeout = self.G_PhrozenPrinter.lookup_object("idle_timeout")
        # is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
        # if is_printing:
        #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume];='%s'")
        # else:
        #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume];='%s;return'")
        #     return
        self.kaos_log("DEBUG", "klipper resume", "SERIAL")

        # Vendor note (240325): MC,
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.kaos_log(
                "DEBUG",
                "multi-material mode resume, #1 time pause is, after output time pausedo not executeresume",
                "SERIAL",
            )

            # Vendor note (241011): duringAMS
            self.STM32ReprotPauseFlag = 0

        else:
            self.kaos_log("DEBUG", "single-color,single-color refill mode resume", "SERIAL")
            self.G_KlipperIfPaused = False
            # Vendor note (240124): STM32 active report, allow one pause
            self.STM32ReprotPauseFlag = 0

            # Vendor note (241106): #self.G_P0M2MAStartPrintFlag=0

        # Vendor note (240325): # Vendor note (240426): set flagfalse
        self.G_ResumeProcessCheckPauseStatus = False
        # Vendor note (231207): +PAUSE:1label
        self.G_IfInFilaBlockFlag = False
        # Vendor note (240321): PG102label
        self.PG102DelayPauseFlag = False
        self.G_ChangeChannelResumeFlag = True
        # Vendor note (231207): P1 C?auto filament change,if,continue1start
        self.G_ChangeChannelFirstFilaFlag = True
        # self.emit_resume(1, self.G_ChangeChannelTimeoutNewChan)

        # Vendor note (250812): single-color runout detection, return to pause zone
        self.G_RetryToPauseAreaFlag = False
        self.G_RetryToPauseAreaCount = 0

        # =====lancaigang231212:,ifdetectfilament,,stm32
        # Vendor note (240108): needfilament,
        if self.G_IfToolheadHaveFilaInitiativePauseFlag == True:
            self.G_IfToolheadHaveFilaInitiativePauseFlag = False

            # Vendor note (240103): M2MArefill mode,needstm32,preventstm32
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                self.kaos_log("DEBUG", "M2MA mode resume, move pause/resume", "SERIAL")
                self.AMSRunoutPauseTimeCount = 0
                self.AMSRunoutPauseTimeoutFlag = 0

                # filament required to resume print
                if self.G_ToolheadIfHaveFilaFlag:
                    self.G_M2MAModeResumeFlag = True
                    # Vendor note (240412): ,ifAMS multi-material present,needAMS
                    if self.G_AMSDevice1IfNormal == True or self.G_AMSDevice2IfNormal == True:
                        self.kaos_log(
                            "DEBUG",
                            "toolhead has filament, has AMS multi-material, do notfilament, need to sending commandSTM32resume after state",
                            "SERIAL",
                        )
                        # #self.Cmds_CmdP8(gcmd)
                        # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("FA")
                        #     self.G_PhrozenFluiddRespondInfo("serial port 1 send FA")
                        # elif self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("FA")
                        #     self.G_PhrozenFluiddRespondInfo("serial port 2 send FA")

                        # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("can resume, STM32printing report error")
                        self.G_KlipperQuickPause = True
                        # # Vendor note (250427): # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                        #     self.G_PhrozenFluiddRespondInfo("serial port 1-AMSfinishtimingbuffer-full time")
                        # if self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                        #     self.G_PhrozenFluiddRespondInfo("serial port 2-AMSfinishtimingbuffer-full time")
                        # self.G_ProzenToolhead.dwell(1.5)

                        # Vendor note (250522): self.G_PhrozenFluiddRespondInfo("[FIRMWARE] External macro PG109 heat-up")
                        command_string = """
                            # PG109
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[FIRMWARE] External macro PG109 heat-up; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+RESTORE")
                            self.kaos_log("DEBUG", "serial port 1-resume", "SERIAL")
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+RESTORE")
                            self.kaos_log("DEBUG", "serial port 2-resume", "SERIAL")
                        # # Vendor note (250611): # self.G_PhrozenFluiddRespondInfo("[PURGE] External macro PG108; heat, purge, wipe")
                        # command_string = """
                        #     PG108
                        #     """
                        # self.G_PhrozenGCode.run_script_from_command(command_string)
                        # self.G_PhrozenFluiddRespondInfo("serial port disable or network move pause/resume, toolhead has filament, resumedo notthen feed")
                        # Vendor note (240125): encapsulated function
                        self.Cmds_PhrozenKlipperResumeCommon()
                        self.G_ChangeChannelResumeFlag = False
                        self.G_PhrozenFluiddRespondInfo(
                            "+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan
                        )
                    else:
                        self.G_KlipperQuickPause = False
                        self.kaos_log(
                            "DEBUG",
                            "toolhead has filament, has has AMS multi-material, resume",
                            "SERIAL",
                        )
                        self.kaos_log(
                            "DEBUG",
                            "serial port disable or network move pause/resume, toolhead has filament, resumedo notthen feed",
                            "SERIAL",
                        )

                        # Vendor note (250522): self.G_PhrozenFluiddRespondInfo("[FIRMWARE] External macro PG109 heat-up")
                        command_string = """
                            # PG109
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[FIRMWARE] External macro PG109 heat-up; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                        # # Vendor note (250611): # self.G_PhrozenFluiddRespondInfo("[PURGE] External macro PG108; heat, purge, wipe")
                        # command_string = """
                        #     PG108
                        #     """
                        # self.G_PhrozenGCode.run_script_from_command(command_string)
                        # Vendor note (240125): encapsulated function
                        self.Cmds_PhrozenKlipperResumeCommon()
                        self.G_ChangeChannelResumeFlag = False
                        self.G_PhrozenFluiddRespondInfo(
                            "+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan
                        )
                else:
                    if self.G_AMSDevice1IfNormal == True or self.G_AMSDevice2IfNormal == True:
                        self.kaos_log(
                            "DEBUG",
                            "toolhead no filament, has AMS multi-material, P8 feed",
                            "SERIAL",
                        )
                        self.G_P0M2MAStartPrintFlag = 0

                        # Vendor note (250522): allowM3detect
                        self.G_IfChangeFilaOngoing = True

                        # Vendor note (241106): self.Cmds_CmdP8(gcmd)
                        # Vendor note (250619): check if AMS reconnected successfully
                        self.Cmds_USBConnectErrorCheck()
                        # Vendor note (241106): toolhead feed successful
                        if self.G_P0M2MAStartPrintFlag == 1:
                            # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("can resume, STM32printing report error")
                            self.G_KlipperQuickPause = True
                            # # Vendor note (250427): # if self.G_SerialPort1OpenFlag == True:
                            #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                            #     self.G_PhrozenFluiddRespondInfo("serial port 1-AMSfinishtimingbuffer-full time")
                            # if self.G_SerialPort2OpenFlag == True:
                            #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                            #     self.G_PhrozenFluiddRespondInfo("serial port 2-AMSfinishtimingbuffer-full time")
                            # self.G_ProzenToolhead.dwell(1.5)
                            # Vendor note (250423): successful,start,AMSstart,if5,
                            # self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
                            # self.G_PhrozenFluiddRespondInfo("AMS start timing buffer-full time")
                            if self.G_SerialPort1OpenFlag == True:
                                self.Cmds_AMSSerial1Send("AT+EBLOCKCHECK")
                                self.kaos_log(
                                    "DEBUG",
                                    "serial port 1-AMSstart timingbuffer-full time",
                                    "SERIAL",
                                )
                            if self.G_SerialPort2OpenFlag == True:
                                self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
                                self.kaos_log(
                                    "DEBUG",
                                    "serial port 2-AMSstart timingbuffer-full time",
                                    "SERIAL",
                                )
                            # self.G_ProzenToolhead.dwell(1)
                            # Vendor note (251120): entering purge, set flag to prevent PG108 purge Hall-sensor pause at purge zone;
                            self.G_PG108Ingoing = 1
                            # Vendor note (250611): self.G_PhrozenFluiddRespondInfo("[PURGE] External macro PG108; heat, purge, wipe")
                            command_string = """
                                # PG108
                                """
                            self.G_PhrozenGCode.run_script_from_command(command_string)
                            self.G_PG108Ingoing = 0
                            if self.G_SerialPort1OpenFlag == True:
                                self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                                self.kaos_log(
                                    "DEBUG",
                                    "serial port 1-AMSfinishtimingbuffer-full time",
                                    "SERIAL",
                                )
                            if self.G_SerialPort2OpenFlag == True:
                                self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                                self.kaos_log(
                                    "DEBUG",
                                    "serial port 2-AMSfinishtimingbuffer-full time",
                                    "SERIAL",
                                )

                            if self.STM32ReprotPauseFlag == 1:
                                self.kaos_log(
                                    "DEBUG", "STM32 up report pause, cannot resume", "SERIAL"
                                )
                                # Vendor note (240125): encapsulated function
                                # self.Cmds_PhrozenKlipperResumeCommon()
                                self.G_ChangeChannelResumeFlag = False
                                self.G_PhrozenFluiddRespondInfo(
                                    "+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan
                                )
                            else:
                                self.kaos_log("DEBUG", "toolhead has filament, resume", "SERIAL")
                                # Vendor note (240125): encapsulated function
                                self.Cmds_PhrozenKlipperResumeCommon()
                                self.G_ChangeChannelResumeFlag = False
                                self.G_PhrozenFluiddRespondInfo(
                                    "+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan
                                )
                        else:
                            self.G_KlipperQuickPause = False
                            self.kaos_log(
                                "DEBUG",
                                "toolhead no filament, single-color refill mode pause",
                                "SERIAL",
                            )
                            if self.G_KlipperIfPaused == False:
                                self.G_PhrozenGCode.run_script_from_command(
                                    "SAVE_GCODE_STATE NAME=PAUSE"
                                )
                                self.kaos_log("DEBUG", "[(cmds.python)]PAUSE", "SERIAL")
                                self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                                self.G_ProzenToolhead.wait_moves()
                                self.G_KlipperIfPaused = True
                                # self.emit_protocol("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                                self.G_PhrozenFluiddRespondInfo(
                                    "+PAUSE:4,%d,%d"
                                    % (
                                        self.G_ChangeChannelTimeoutOldChan,
                                        self.G_ChangeChannelTimeoutNewChan,
                                    )
                                )
                            else:
                                self.kaos_log("DEBUG", "already pause, do notpause", "SERIAL")
                            self.G_ChangeChannelResumeFlag = False
                            self.G_PhrozenFluiddRespondInfo(
                                "+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan
                            )
                    else:
                        self.kaos_log(
                            "DEBUG",
                            "toolhead no filament, has AMS multi-material, single-color refill mode pause",
                            "SERIAL",
                        )
                        if self.G_KlipperIfPaused == False:
                            self.G_PhrozenGCode.run_script_from_command(
                                "SAVE_GCODE_STATE NAME=PAUSE"
                            )
                            self.kaos_log("DEBUG", "[(cmds.python)]PAUSE", "SERIAL")
                            self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                            self.G_ProzenToolhead.wait_moves()
                            self.G_KlipperIfPaused = True
                            # self.emit_protocol("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                            self.G_PhrozenFluiddRespondInfo(
                                "+PAUSE:4,%d,%d"
                                % (
                                    self.G_ChangeChannelTimeoutOldChan,
                                    self.G_ChangeChannelTimeoutNewChan,
                                )
                            )
                        else:
                            self.kaos_log("DEBUG", "already pause, do notpause", "SERIAL")
                        self.G_ChangeChannelResumeFlag = False
                        self.G_PhrozenFluiddRespondInfo(
                            "+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan
                        )

                return

            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                self.kaos_log("DEBUG", "M3 mode resume, move pause/resume", "SERIAL")
                # # Vendor note (241106): AMS multi-material present
                # if self.G_AMSDevice1IfNormal==True or self.G_AMSDevice2IfNormal==True:
                #     self.G_PhrozenFluiddRespondInfo("Single-color M3 mode, AMS present, need to send STM32 resume state")
                #     # # Vendor note (240416): #     # if self.G_SerialPort1OpenFlag == True:
                #     #     self.Cmds_AMSSerial1Send("MA")
                #     #     self.G_PhrozenFluiddRespondInfo("Serial port 1-MA")
                #     # # Vendor note (241030): #     # elif self.G_SerialPort2OpenFlag == True:
                #     #     self.Cmds_AMSSerial2Send("MA")
                #     #     self.G_PhrozenFluiddRespondInfo("Serial port 2-MA")

                #     # time.sleep(2)

                #     # # Vendor note (240416): #     # if self.G_SerialPort1OpenFlag == True:
                #     #     self.Cmds_AMSSerial1Send("FA")
                #     #     self.G_PhrozenFluiddRespondInfo("Serial port 1-FA")
                #     # # Vendor note (241030): #     # elif self.G_SerialPort2OpenFlag == True:
                #     #     self.Cmds_AMSSerial2Send("FA")
                #     #     self.G_PhrozenFluiddRespondInfo("Serial port 2-FA")

                #     # Vendor note (241106): #     self.Cmds_CmdP8(gcmd)
                #     # Vendor note (241106): toolhead feed successful
                #     if self.G_P0M2MAStartPrintFlag==1:
                #         self.G_PhrozenFluiddRespondInfo("serial port disable or network move pause/resume, toolhead has filament, resumedo notthen feed")
                #         # Vendor note (240125): encapsulated function
                #         self.Cmds_PhrozenKlipperResumeCommon()
                #     else:
                #         self.G_PhrozenFluiddRespondInfo("toolhead no filament, single-color refill mode pause")
                #         self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                #         self.G_PhrozenFluiddRespondInfo("send_pause_command")
                #         #no filament, continue pausing
                #         self.G_KlipperIfPaused=True
                #         self.emit_protocol("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                # else:
                #     if self.G_ToolheadIfHaveFilaFlag==True:
                #         self.G_PhrozenFluiddRespondInfo("Single-color M3 mode, no AMS, Klipper resume directly")
                #         self.G_PhrozenFluiddRespondInfo("serial port disable or network move pause/resume, toolhead has filament, resumedo notthen feed")
                #         # Vendor note (240125): encapsulated function
                #         self.Cmds_PhrozenKlipperResumeCommon()
                #     else:
                #         self.G_PhrozenFluiddRespondInfo("toolhead no filament, single-color refill mode pause")
                #         self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                #         self.G_PhrozenFluiddRespondInfo("send_pause_command")
                #         #no filament, continue pausing
                #         self.G_KlipperIfPaused=True
                #         self.emit_protocol("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)

                # filament required to resume print
                if self.G_ToolheadIfHaveFilaFlag:
                    # Vendor note (240412): M3,ifAMS multi-material present,needAMS
                    if self.G_AMSDevice1IfNormal == True or self.G_AMSDevice2IfNormal == True:
                        self.kaos_log(
                            "DEBUG",
                            "toolhead has filament, has AMS multi-material, do notfilament, need to sending commandSTM32resume after state",
                            "SERIAL",
                        )
                        # #self.Cmds_CmdP8(gcmd)
                        # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("FA")
                        #     self.G_PhrozenFluiddRespondInfo("serial port 1 send FA")
                        # elif self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("FA")
                        #     self.G_PhrozenFluiddRespondInfo("serial port 2 send FA")
                        # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("can resume, STM32printing report error")
                        self.G_KlipperQuickPause = True
                        # # Vendor note (250427): # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                        #     self.G_PhrozenFluiddRespondInfo("serial port 1-AMSfinishtimingbuffer-full time")
                        # if self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                        #     self.G_PhrozenFluiddRespondInfo("serial port 2-AMSfinishtimingbuffer-full time")
                        # self.G_ProzenToolhead.dwell(1.5)

                        # Vendor note (250522): self.G_PhrozenFluiddRespondInfo("[FIRMWARE] External macro PG109 heat-up")
                        command_string = """
                            # PG109
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[FIRMWARE] External macro PG109 heat-up; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+RESTORE")
                            self.kaos_log("DEBUG", "serial port 1-resume", "SERIAL")
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+RESTORE")
                            self.kaos_log("DEBUG", "serial port 2-resume", "SERIAL")

                        # # Vendor note (250611): # self.G_PhrozenFluiddRespondInfo("[PURGE] External macro PG108; heat, purge, wipe")
                        # command_string = """
                        #     PG108
                        #     """
                        # self.G_PhrozenGCode.run_script_from_command(command_string)
                        # self.G_PhrozenFluiddRespondInfo("serial port disable or network move pause/resume, toolhead has filament, resumedo notthen feed")
                        # Vendor note (240125): encapsulated function
                        self.Cmds_PhrozenKlipperResumeCommon()
                        self.G_ChangeChannelResumeFlag = False
                        self.G_PhrozenFluiddRespondInfo(
                            "+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan
                        )
                    else:
                        self.G_KlipperQuickPause = False
                        self.kaos_log(
                            "DEBUG",
                            "toolhead has filament, has has AMS multi-material, resume",
                            "SERIAL",
                        )
                        # Vendor note (250522): self.G_PhrozenFluiddRespondInfo("[FIRMWARE] External macro PG109 heat-up")
                        command_string = """
                            # PG109
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[FIRMWARE] External macro PG109 heat-up; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                        # self.G_PhrozenFluiddRespondInfo("serial port disable or network move pause/resume, toolhead has filament, resumedo notthen feed")
                        # # Vendor note (250409): # command_string = """
                        # PG108
                        # """
                        # self.G_PhrozenGCode.run_script_from_command(command_string)
                        # self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)
                        # self.G_PhrozenFluiddRespondInfo("purge complete, toolhead detectedhas filamentresume printing")
                        # Vendor note (240125): encapsulated function
                        self.Cmds_PhrozenKlipperResumeCommon()
                        self.G_ChangeChannelResumeFlag = False
                        self.G_PhrozenFluiddRespondInfo(
                            "+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan
                        )
                else:
                    if self.G_AMSDevice1IfNormal == True or self.G_AMSDevice2IfNormal == True:
                        self.kaos_log(
                            "DEBUG",
                            "toolhead no filament, has AMS multi-material, P8 feed",
                            "SERIAL",
                        )
                        self.G_P0M2MAStartPrintFlag = 0

                        # Vendor note (250522): allowM3detect
                        self.G_IfChangeFilaOngoing = True

                        # Vendor note (241106): self.Cmds_CmdP8(gcmd)
                        # Vendor note (250619): check if AMS reconnected successfully
                        self.Cmds_USBConnectErrorCheck()
                        # Vendor note (241106): toolhead feed successful
                        if self.G_P0M2MAStartPrintFlag == 1:
                            # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("can resume, STM32printing report error")
                            self.G_KlipperQuickPause = True
                            # # Vendor note (250427): # if self.G_SerialPort1OpenFlag == True:
                            #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                            #     self.G_PhrozenFluiddRespondInfo("serial port 1-AMSfinishtimingbuffer-full time")
                            # if self.G_SerialPort2OpenFlag == True:
                            #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                            #     self.G_PhrozenFluiddRespondInfo("serial port 2-AMSfinishtimingbuffer-full time")
                            # self.G_ProzenToolhead.dwell(1.5)
                            if self.G_SerialPort1OpenFlag == True:
                                self.Cmds_AMSSerial1Send("AT+EBLOCKCHECK")
                                self.kaos_log(
                                    "DEBUG",
                                    "serial port 1-AMSstart timingbuffer-full time",
                                    "SERIAL",
                                )
                            if self.G_SerialPort2OpenFlag == True:
                                self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
                                self.kaos_log(
                                    "DEBUG",
                                    "serial port 2-AMSstart timingbuffer-full time",
                                    "SERIAL",
                                )
                            # self.G_ProzenToolhead.dwell(1)
                            # Vendor note (251120): entering purge, set flag to prevent PG108 purge Hall-sensor pause at purge zone;
                            self.G_PG108Ingoing = 1
                            # Vendor note (250611): self.G_PhrozenFluiddRespondInfo("[PURGE] External macro PG108; heat, purge, wipe")
                            command_string = """
                                # PG108
                                """
                            self.G_PhrozenGCode.run_script_from_command(command_string)
                            self.G_PG108Ingoing = 0
                            if self.G_SerialPort1OpenFlag == True:
                                self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                                self.kaos_log(
                                    "DEBUG",
                                    "serial port 1-AMSfinishtimingbuffer-full time",
                                    "SERIAL",
                                )
                            if self.G_SerialPort2OpenFlag == True:
                                self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                                self.kaos_log(
                                    "DEBUG",
                                    "serial port 2-AMSfinishtimingbuffer-full time",
                                    "SERIAL",
                                )
                            if self.STM32ReprotPauseFlag == 1:
                                self.kaos_log(
                                    "DEBUG", "STM32 up report pause, cannot resume", "SERIAL"
                                )
                                # Vendor note (240125): encapsulated function
                                # self.Cmds_PhrozenKlipperResumeCommon()
                                self.G_ChangeChannelResumeFlag = False
                                self.G_PhrozenFluiddRespondInfo(
                                    "+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan
                                )
                            else:
                                self.kaos_log("DEBUG", "toolhead has filament, resume", "SERIAL")
                                # Vendor note (240125): encapsulated function
                                self.Cmds_PhrozenKlipperResumeCommon()
                                self.G_ChangeChannelResumeFlag = False
                                self.G_PhrozenFluiddRespondInfo(
                                    "+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan
                                )
                        else:
                            self.G_KlipperQuickPause = False
                            self.kaos_log(
                                "DEBUG", "toolhead no filament, M3 mode refill pause", "SERIAL"
                            )
                            if self.G_KlipperIfPaused == False:
                                self.G_PhrozenGCode.run_script_from_command(
                                    "SAVE_GCODE_STATE NAME=PAUSE"
                                )
                                self.kaos_log("DEBUG", "[(cmds.python)]PAUSE", "SERIAL")
                                self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                                self.G_ProzenToolhead.wait_moves()
                                self.G_KlipperIfPaused = True
                                # self.emit_protocol("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                                self.G_PhrozenFluiddRespondInfo(
                                    "+PAUSE:4,%d,%d"
                                    % (
                                        self.G_ChangeChannelTimeoutOldChan,
                                        self.G_ChangeChannelTimeoutNewChan,
                                    )
                                )
                            else:
                                self.kaos_log("DEBUG", "already pause, do notpause", "SERIAL")
                            self.G_ChangeChannelResumeFlag = False
                            self.G_PhrozenFluiddRespondInfo(
                                "+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan
                            )
                    else:
                        self.G_KlipperQuickPause = False
                        self.kaos_log(
                            "DEBUG",
                            "toolhead no filament, has AMS multi-material, M3 mode pause",
                            "SERIAL",
                        )
                        self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE")
                        self.kaos_log("DEBUG", "[(cmds.python)]PAUSE", "SERIAL")
                        self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                        self.G_ProzenToolhead.wait_moves()
                        # no filament, continue pausing
                        self.G_KlipperIfPaused = True
                        # self.emit_protocol("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                        self.G_PhrozenFluiddRespondInfo(
                            "+PAUSE:b,%d,%d"
                            % (
                                self.G_ChangeChannelTimeoutOldChan,
                                self.G_ChangeChannelTimeoutNewChan,
                            )
                        )
                        self.G_ChangeChannelResumeFlag = False
                        self.G_PhrozenFluiddRespondInfo(
                            "+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan
                        )

                return

            # Vendor note (240115): ,stm32
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
                self.kaos_log("DEBUG", "M1MC mode resume", "SERIAL")
                # Vendor note (240521): on resume: if AMS restart detected (hot-plug), execute full retract/change
                if self.G_ResumeCheckAMS1ErrorRestartFlag == True:
                    self.G_ResumeCheckAMS1ErrorRestartFlag = False
                    self.kaos_log(
                        "DEBUG",
                        "serial port disable or network move pause/resume;multi-material MC mode; detectedAMS errorrestart, resume",
                        "SERIAL",
                    )
                else:
                    self.kaos_log(
                        "DEBUG",
                        "serial port disable or network move pause/resume;multi-material MC mode, stm32 resumeprinting refill state",
                        "SERIAL",
                    )
                    # self.Cmds_AMSSerial1Send("AT+MCRS=F")
                    # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]AT+MCRS=F")
                    # Vendor note (240115): ifduring,manual command,stm32,,executeP1 C?
                    # iffilament,stm32canconvert
                    if self.G_ToolheadIfHaveFilaFlag == True:
                        # # Vendor note (241030): # if self.G_ChangeChannelTimeoutNewChan in range(1, 4):
                        #     # Vendor note (0427): manual command,stm32,
                        #     self.Cmds_AMSSerial1Send("AT+MCRS=5,%d" % self.G_ChangeChannelTimeoutNewChan)#05=
                        #     self.G_PhrozenFluiddRespondInfo("AT+MCRS=5,%d" % self.G_ChangeChannelTimeoutNewChan)
                        # elif self.G_ChangeChannelTimeoutNewChan in range(5, 8):
                        #     self.Cmds_AMSSerial2Send("AT+MCRS=5,%d" % self.G_ChangeChannelTimeoutNewChan-4)#05=
                        #     self.G_PhrozenFluiddRespondInfo("AT+MCRS=5,%d" % self.G_ChangeChannelTimeoutNewChan-4)

                        self.kaos_log(
                            "DEBUG",
                            "multi-material MC mode, send stm32 resumeprinting refill state",
                            "SERIAL",
                        )
                        self.kaos_log(
                            "DEBUG",
                            "serial port disable or network move pause/resume, toolhead has filament",
                            "SERIAL",
                        )

                        # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("AT+RESTORE")
                        #     self.G_PhrozenFluiddRespondInfo("serial port 1-resume")
                        # if self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("AT+RESTORE")
                        #     self.G_PhrozenFluiddRespondInfo("serial port 2-resume")

                        # # Vendor note (241012): needre-feed,preventAMS

                        # # Vendor note (240125): encapsulated function
                        # self.Cmds_PhrozenKlipperResumeCommon()
                        # self.G_ChangeChannelResumeFlag=False
                        # self.emit_resume(2, self.G_ChangeChannelTimeoutNewChan)
                        # return

                    else:
                        self.kaos_log(
                            "DEBUG",
                            "serial port disable or network move pause/resume;toolhead has filament, resume",
                            "SERIAL",
                        )

        # =====lancaigang231229:MA,
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.kaos_log("DEBUG", "M2MA mode resume", "SERIAL")
            self.AMSRunoutPauseTimeCount = 0
            self.AMSRunoutPauseTimeoutFlag = 0

            # # # Vendor note (240416): # # if self.G_SerialPort1OpenFlag == True:
            # #     self.Cmds_AMSSerial1Send("MA")
            # #     self.G_PhrozenFluiddRespondInfo("Serial port 1-MA")
            # # # Vendor note (241030): # # elif self.G_SerialPort2OpenFlag == True:
            # #     self.Cmds_AMSSerial2Send("MA")
            # #     self.G_PhrozenFluiddRespondInfo("Serial port 2-MA")

            # # # Vendor note (240115): delay 1s to prevent packet adhesion
            # # time.sleep(2)

            # #filament required to resume print
            # if self.G_ToolheadIfHaveFilaFlag:
            #     # # Vendor note (231228): after resume, send command to STM32 to restore last state machine state
            #     # #resume state RS=F, restore last state
            #     # #RS=0,MASTATEMACHINE_IDLE_STANDBY
            #     # #resume state RS=X,...
            #     # #resume state RS=Y,...
            #     # #resume state RS=Z,...
            #     # # Vendor note (240108): #     # #self.Cmds_AMSSerial1Send("AT+MARS=F")
            #     # #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]AT+MARS=F")
            #     # # Vendor note (240108): ,filament
            #     # # Vendor note (240226): filament,FA
            #     # # Vendor note (240416): #     # if self.G_SerialPort1OpenFlag == True:
            #     #     self.Cmds_AMSSerial1Send("FA")
            #     #     self.G_PhrozenFluiddRespondInfo("Serial port 1-FA")
            #     # # Vendor note (241030): #     # elif self.G_SerialPort2OpenFlag == True:
            #     #     self.Cmds_AMSSerial2Send("FA")
            #     #     self.G_PhrozenFluiddRespondInfo("Serial port 2-FA")

            #     # self.G_PhrozenFluiddRespondInfo("Single-color M2 MA refill mode, toolhead has filament, resume directly")
            #     # # Vendor note (240108): complete
            #     # #if self.P0M3FilaRunoutSpittingFinished == True:
            #     # #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]Extrusion done, can resume")

            #     # Vendor note (241106): #     self.Cmds_CmdP8(gcmd)
            #     # Vendor note (241106): toolhead feed successful
            #     if self.G_P0M2MAStartPrintFlag==1:
            #         self.G_PhrozenFluiddRespondInfo("Toolhead has filament, retract then re-feed")
            #         # Vendor note (240125): encapsulated function
            #         self.Cmds_PhrozenKlipperResumeCommon()
            #     else:
            #         self.G_PhrozenFluiddRespondInfo("toolhead no filament, single-color refill mode pause")
            #         self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
            #         self.G_PhrozenFluiddRespondInfo("send_pause_command")
            #         #no filament, continue pausing
            #         self.G_KlipperIfPaused=True
            #         self.emit_protocol("+PAUSE:4,%d" % self.G_ChangeChannelTimeoutNewChan)

            #     # Vendor note (241106): #     #self.Cmds_CmdP8(gcmd)
            #     #self.Cmds_PhrozenKlipperResumeCommon()

            #     # Vendor note (240108): filament,can
            #     self.G_M2MAModeResumeFlag=True

            #     self.G_ChangeChannelResumeFlag=False
            #     self.emit_resume(2, self.G_ChangeChannelTimeoutNewChan)

            #     return
            # #filamentneedre-feed
            # else:
            #     # Vendor note (240108): ,allow
            #     self.P0M3FilaRunoutSpittingFinished = False
            #     self.G_ToolheadFirstInputFila=False
            #     # Vendor note (240108): filament,can
            #     # Vendor note (240109): detectfilament,can
            #     self.G_M2MAModeResumeFlag=True

            #     self.G_PhrozenFluiddRespondInfo("Single-color M2 MA refill mode, toolhead no filament, need to re-feed")

            #     # # Vendor note (240103): toolhead no filament, re-feed, reschedule, execute single-color auto-refill F8
            #     # #ttyUSB0 serial send: FA
            #     # # Vendor note (240108): do not send FA yet
            #     # # Vendor note (240416): #     # if self.G_SerialPort1OpenFlag == True:
            #     #     self.Cmds_AMSSerial1Send("FA")
            #     #     self.G_PhrozenFluiddRespondInfo("Serial port 1-FA")
            #     # # Vendor note (241030): #     # elif self.G_SerialPort2OpenFlag == True:
            #     #     self.Cmds_AMSSerial2Send("FA")
            #     #     self.G_PhrozenFluiddRespondInfo("Serial port 2-FA")
            #     # # Vendor note (231229): encapsulated function
            #     # self.Cmds_MARetryInFila(gcmd)

            #     # Vendor note (241106): #     self.Cmds_CmdP8(gcmd)
            #     # Vendor note (241106): toolhead feed successful
            #     if self.G_P0M2MAStartPrintFlag==1:
            #         self.G_PhrozenFluiddRespondInfo("Toolhead has filament, retract then re-feed")
            #         # Vendor note (240125): encapsulated function
            #         self.Cmds_PhrozenKlipperResumeCommon()
            #     else:
            #         self.G_PhrozenFluiddRespondInfo("toolhead no filament, single-color refill mode pause")
            #         self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
            #         self.G_PhrozenFluiddRespondInfo("send_pause_command")
            #         #no filament, continue pausing
            #         self.G_KlipperIfPaused=True
            #         self.emit_protocol("+PAUSE:4,%d" % self.G_ChangeChannelTimeoutNewChan)

            # filament required to resume print
            if self.G_ToolheadIfHaveFilaFlag:
                self.G_M2MAModeResumeFlag = True
                # Vendor note (240412): M2MA,ifAMS multi-material present,needAMS
                if self.G_AMSDevice1IfNormal == True or self.G_AMSDevice2IfNormal == True:
                    self.kaos_log(
                        "DEBUG",
                        "toolhead has filament, has AMS multi-material, but is P8 feed, prevent stop normal",
                        "SERIAL",
                    )
                    self.G_P0M2MAStartPrintFlag = 0

                    # Vendor note (250522): allowM3detect
                    self.G_IfChangeFilaOngoing = True

                    self.Cmds_CmdP8(gcmd)
                    # Vendor note (250619): check if AMS reconnected successfully
                    self.Cmds_USBConnectErrorCheck()
                    # if self.G_SerialPort1OpenFlag == True:
                    #     self.Cmds_AMSSerial1Send("FA")
                    #     self.G_PhrozenFluiddRespondInfo("serial port 1 send FA")
                    # elif self.G_SerialPort2OpenFlag == True:
                    #     self.Cmds_AMSSerial2Send("FA")
                    #     self.G_PhrozenFluiddRespondInfo("serial port 2 send FA")
                    # Vendor note (241106): toolhead feed successful
                    if self.G_P0M2MAStartPrintFlag == 1:
                        # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("can resume, STM32printing report error")
                        self.G_KlipperQuickPause = True
                        # # Vendor note (250427): # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                        #     self.G_PhrozenFluiddRespondInfo("serial port 1-AMSfinishtimingbuffer-full time")
                        # if self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                        #     self.G_PhrozenFluiddRespondInfo("serial port 2-AMSfinishtimingbuffer-full time")
                        # self.G_ProzenToolhead.dwell(1.5)
                        # Vendor note (250522): # self.G_PhrozenFluiddRespondInfo("[FIRMWARE] External macro PG109 heat-up")
                        # command_string = """
                        #     PG109
                        #     """
                        # self.G_PhrozenGCode.run_script_from_command(command_string)
                        # self.G_PhrozenFluiddRespondInfo("[FIRMWARE] External macro PG109 heat-up; command_string='%s'" % command_string)
                        # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("AT+RESTORE")
                        #     self.G_PhrozenFluiddRespondInfo("serial port 1-resume")
                        # if self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("AT+RESTORE")
                        #     self.G_PhrozenFluiddRespondInfo("serial port 2-resume")
                        # self.G_PhrozenFluiddRespondInfo("toolhead has filament, resume")
                        # # Vendor note (250611): # self.G_PhrozenFluiddRespondInfo("[PURGE] External macro PG108; heat, purge, wipe")
                        # command_string = """
                        #     PG108
                        #     """
                        # self.G_PhrozenGCode.run_script_from_command(command_string)
                        # # Vendor note (240125): encapsulated function
                        # self.Cmds_PhrozenKlipperResumeCommon()
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+EBLOCKCHECK")
                            self.kaos_log(
                                "DEBUG", "serial port 1-AMSstart timingbuffer-full time", "SERIAL"
                            )
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
                            self.kaos_log(
                                "DEBUG", "serial port 2-AMSstart timingbuffer-full time", "SERIAL"
                            )
                        # self.G_ProzenToolhead.dwell(1)
                        # Vendor note (250522): self.G_PhrozenFluiddRespondInfo("[FIRMWARE] External macro PG109 heat-up")
                        command_string = """
                            # PG109
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[FIRMWARE] External macro PG109 heat-up; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                        # Vendor note (251120): entering purge, set flag to prevent PG108 purge Hall-sensor pause at purge zone;
                        self.G_PG108Ingoing = 1
                        # Vendor note (250611): self.G_PhrozenFluiddRespondInfo("[PURGE] External macro PG108; heat, purge, wipe")
                        command_string = """
                            # PG108
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PG108Ingoing = 0
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                            self.kaos_log(
                                "DEBUG", "serial port 1-AMSfinishtimingbuffer-full time", "SERIAL"
                            )
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                            self.kaos_log(
                                "DEBUG", "serial port 2-AMSfinishtimingbuffer-full time", "SERIAL"
                            )
                        if self.STM32ReprotPauseFlag == 1:
                            self.kaos_log("DEBUG", "STM32 up report pause, cannot resume", "SERIAL")
                            # Vendor note (240125): encapsulated function
                            # self.Cmds_PhrozenKlipperResumeCommon()
                            self.G_ChangeChannelResumeFlag = False
                            self.G_PhrozenFluiddRespondInfo(
                                "+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan
                            )
                        else:
                            self.kaos_log("DEBUG", "toolhead has filament, resume", "SERIAL")
                            # Vendor note (240125): encapsulated function
                            self.Cmds_PhrozenKlipperResumeCommon()
                            self.G_ChangeChannelResumeFlag = False
                            self.G_PhrozenFluiddRespondInfo(
                                "+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan
                            )
                    else:
                        self.G_KlipperQuickPause = False
                        self.kaos_log("DEBUG", "toolhead no filament, M2MA mode pause", "SERIAL")
                        if self.G_KlipperIfPaused == False:
                            self.G_PhrozenGCode.run_script_from_command(
                                "SAVE_GCODE_STATE NAME=PAUSE"
                            )
                            self.kaos_log("DEBUG", "[(cmds.python)]PAUSE", "SERIAL")
                            self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                            self.G_ProzenToolhead.wait_moves()
                            # no filament, continue pausing
                            self.G_KlipperIfPaused = True
                            # self.emit_protocol("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                            self.G_PhrozenFluiddRespondInfo(
                                "+PAUSE:4,%d,%d"
                                % (
                                    self.G_ChangeChannelTimeoutOldChan,
                                    self.G_ChangeChannelTimeoutNewChan,
                                )
                            )
                        else:
                            self.kaos_log("DEBUG", "already pause, do notpause", "SERIAL")
                        self.G_ChangeChannelResumeFlag = False
                        self.G_PhrozenFluiddRespondInfo(
                            "+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan
                        )
                else:
                    self.G_KlipperQuickPause = False
                    self.kaos_log(
                        "DEBUG",
                        "toolhead has filament, has has AMS multi-material, resume",
                        "SERIAL",
                    )
                    self.kaos_log("DEBUG", "toolhead has filament, resumedo notthen feed", "SERIAL")

                    # Vendor note (250522): self.G_PhrozenFluiddRespondInfo("[FIRMWARE] External macro PG109 heat-up")
                    command_string = """
                        # PG109
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.kaos_log(
                        "DEBUG",
                        "[FIRMWARE] External macro PG109 heat-up; command_string='%s'"
                        % command_string,
                        "SERIAL",
                    )
                    # Vendor note (251120): entering purge, set flag to prevent PG108 purge Hall-sensor pause at purge zone;
                    self.G_PG108Ingoing = 1
                    # Vendor note (250611): self.G_PhrozenFluiddRespondInfo("[PURGE] External macro PG108; heat, purge, wipe")
                    command_string = """
                        # PG108
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.G_PG108Ingoing = 0
                    # Vendor note (240125): encapsulated function
                    self.Cmds_PhrozenKlipperResumeCommon()
                    self.G_ChangeChannelResumeFlag = False
                    self.G_PhrozenFluiddRespondInfo(
                        "+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan
                    )
            else:
                if self.G_AMSDevice1IfNormal == True or self.G_AMSDevice2IfNormal == True:
                    self.kaos_log(
                        "DEBUG", "toolhead no filament, has AMS multi-material, P8 feed", "SERIAL"
                    )
                    self.G_P0M2MAStartPrintFlag = 0

                    # Vendor note (250522): allowM3detect
                    self.G_IfChangeFilaOngoing = True

                    # Vendor note (241106): self.Cmds_CmdP8(gcmd)
                    # Vendor note (250619): check if AMS reconnected successfully
                    self.Cmds_USBConnectErrorCheck()
                    # Vendor note (241106): toolhead feed successful
                    if self.G_P0M2MAStartPrintFlag == 1:
                        # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("can resume, STM32printing report error")
                        self.G_KlipperQuickPause = True
                        # # Vendor note (250427): # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                        #     self.G_PhrozenFluiddRespondInfo("serial port 1-AMSfinishtimingbuffer-full time")
                        # if self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                        #     self.G_PhrozenFluiddRespondInfo("serial port 2-AMSfinishtimingbuffer-full time")
                        # self.G_ProzenToolhead.dwell(1.5)
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+EBLOCKCHECK")
                            self.kaos_log(
                                "DEBUG", "serial port 1-AMSstart timingbuffer-full time", "SERIAL"
                            )
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
                            self.kaos_log(
                                "DEBUG", "serial port 2-AMSstart timingbuffer-full time", "SERIAL"
                            )
                        # self.G_ProzenToolhead.dwell(1)
                        # Vendor note (250522): self.G_PhrozenFluiddRespondInfo("[FIRMWARE] External macro PG109 heat-up")
                        command_string = """
                            # PG109
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[FIRMWARE] External macro PG109 heat-up; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                        # Vendor note (251120): entering purge, set flag to prevent PG108 purge Hall-sensor pause at purge zone;
                        self.G_PG108Ingoing = 1
                        # Vendor note (250611): self.G_PhrozenFluiddRespondInfo("[PURGE] External macro PG108; heat, purge, wipe")
                        command_string = """
                            # PG108
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PG108Ingoing = 0
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                            self.kaos_log(
                                "DEBUG", "serial port 1-AMSfinishtimingbuffer-full time", "SERIAL"
                            )
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                            self.kaos_log(
                                "DEBUG", "serial port 2-AMSfinishtimingbuffer-full time", "SERIAL"
                            )
                        if self.STM32ReprotPauseFlag == 1:
                            self.kaos_log("DEBUG", "STM32 up report pause, cannot resume", "SERIAL")
                            # Vendor note (240125): encapsulated function
                            # self.Cmds_PhrozenKlipperResumeCommon()
                            self.G_ChangeChannelResumeFlag = False
                            self.G_PhrozenFluiddRespondInfo(
                                "+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan
                            )
                        else:
                            self.kaos_log("DEBUG", "toolhead has filament, resume", "SERIAL")
                            # Vendor note (240125): encapsulated function
                            self.Cmds_PhrozenKlipperResumeCommon()
                            self.G_ChangeChannelResumeFlag = False
                            self.G_PhrozenFluiddRespondInfo(
                                "+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan
                            )
                    else:
                        self.G_KlipperQuickPause = False
                        self.kaos_log("DEBUG", "toolhead no filament, M2MA mode pause", "SERIAL")
                        if self.G_KlipperIfPaused == False:
                            self.G_PhrozenGCode.run_script_from_command(
                                "SAVE_GCODE_STATE NAME=PAUSE"
                            )
                            self.kaos_log("DEBUG", "[(cmds.python)]PAUSE", "SERIAL")
                            self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                            self.G_ProzenToolhead.wait_moves()
                            self.G_KlipperIfPaused = True
                            # self.emit_protocol("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                            self.G_PhrozenFluiddRespondInfo(
                                "+PAUSE:4,%d,%d"
                                % (
                                    self.G_ChangeChannelTimeoutOldChan,
                                    self.G_ChangeChannelTimeoutNewChan,
                                )
                            )
                        else:
                            self.kaos_log("DEBUG", "already pause, do notpause", "SERIAL")
                        self.G_ChangeChannelResumeFlag = False
                        self.G_PhrozenFluiddRespondInfo(
                            "+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan
                        )
                else:
                    self.G_KlipperQuickPause = False
                    self.kaos_log(
                        "DEBUG", "toolhead no filament, has AMS multi-material, M2MApause", "SERIAL"
                    )
                    self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE")
                    self.kaos_log("DEBUG", "[(cmds.python)]PAUSE", "SERIAL")
                    self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                    self.G_ProzenToolhead.wait_moves()
                    self.G_KlipperIfPaused = True
                    # self.emit_protocol("+PAUSE:4,%d" % self.G_ChangeChannelTimeoutNewChan)
                    self.G_PhrozenFluiddRespondInfo(
                        "+PAUSE:4,%d,%d"
                        % (
                            self.G_ChangeChannelTimeoutOldChan,
                            self.G_ChangeChannelTimeoutNewChan,
                        )
                    )
                    self.G_ChangeChannelResumeFlag = False
                    self.G_PhrozenFluiddRespondInfo(
                        "+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan
                    )

            return

        # =====lancaigang231220:M3,need,detectfilamentcan
        # M3
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.kaos_log("DEBUG", "M3 mode resume", "SERIAL")
            # #filament required to resume print
            # if self.G_ToolheadIfHaveFilaFlag:
            #     # Vendor note (240412): ,ifAMS multi-material present,needAMS
            #     if self.G_AMSDevice1IfNormal==True or self.G_AMSDevice2IfNormal==True:
            #         # # Vendor note (240416): #         # if self.G_SerialPort1OpenFlag == True:
            #         #     self.Cmds_AMSSerial1Send("MA")
            #         #     self.G_PhrozenFluiddRespondInfo("Serial port 1-MA")
            #         # # Vendor note (241030): #         # elif self.G_SerialPort2OpenFlag == True:
            #         #     self.Cmds_AMSSerial2Send("MA")
            #         #     self.G_PhrozenFluiddRespondInfo("Serial port 2-MA")

            #         # # Vendor note (240115): delay 1s to prevent packet adhesion
            #         # time.sleep(2)

            #         # # Vendor note (241030): FA used for
            #         # if self.G_SerialPort1OpenFlag == True:
            #         #     self.Cmds_AMSSerial1Send("FA")
            #         #     self.G_PhrozenFluiddRespondInfo("Serial port 1-FA")
            #         # # Vendor note (241030): #         # elif self.G_SerialPort2OpenFlag == True:
            #         #     self.Cmds_AMSSerial2Send("FA")
            #         #     self.G_PhrozenFluiddRespondInfo("Serial port 2-FA")

            #         # Vendor note (241106): execute P8 full feed to handle AMS power-loss restart
            #         self.Cmds_CmdP8(gcmd)
            #         # Vendor note (241106): toolhead feed successful
            #         if self.G_P0M2MAStartPrintFlag==1:
            #             self.G_PhrozenFluiddRespondInfo("Single-color M3 mode, AMS present, toolhead has filament, resuming")
            #             # Vendor note (240125): encapsulated function
            #             self.Cmds_PhrozenKlipperResumeCommon()
            #         else:
            #             self.G_PhrozenFluiddRespondInfo("Standalone M3 mode, AMS present but toolhead no filament, pausing, please load manually")
            #             self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
            #             self.G_PhrozenFluiddRespondInfo("send_pause_command")
            #             #no filament, continue pausing
            #             self.G_KlipperIfPaused=True
            #             self.emit_protocol("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
            #     else:
            #         self.G_PhrozenFluiddRespondInfo("Standalone M3 mode, no AMS, toolhead has filament, resuming")
            #         # # Vendor note (240411): manual extrude then resume
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
            #         self.G_PhrozenFluiddRespondInfo("Calling external macro PG108 - single-color M3 mode start purge")
            #         # Vendor note (240407): placed before purge call to prevent immediate feed-then-purge multi-command error
            #         self.P0M3FilaRunoutSpittingFinished = True#complete,preventcall
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

            #         # Vendor note (240125): encapsulated function
            #         self.Cmds_PhrozenKlipperResumeCommon()

            #     # # Vendor note (241106): #     # self.Cmds_CmdP8(gcmd)
            #     # self.Cmds_PhrozenKlipperResumeCommon()

            #     self.G_ChangeChannelResumeFlag=False
            #     self.emit_resume(2, self.G_ChangeChannelTimeoutNewChan)

            #     return

            # #filamentcontinue
            # else:
            #     # Vendor note (240412): ,ifAMS multi-material present,needAMS
            #     if self.G_AMSDevice1IfNormal==True or self.G_AMSDevice2IfNormal==True:
            #         # # Vendor note (240416): #         # if self.G_SerialPort1OpenFlag == True:
            #         #     self.Cmds_AMSSerial1Send("MA")
            #         #     self.G_PhrozenFluiddRespondInfo("Serial port 1-MA")
            #         # # Vendor note (241030): #         # elif self.G_SerialPort2OpenFlag == True:
            #         #     self.Cmds_AMSSerial2Send("MA")
            #         #     self.G_PhrozenFluiddRespondInfo("Serial port 2-MA")

            #         # # Vendor note (240115): delay 1s to prevent packet adhesion
            #         # time.sleep(2)

            #         # self.G_PhrozenFluiddRespondInfo("Single-color M3 mode, AMS present, toolhead no filament, need to re-feed")
            #         # # Vendor note (240103): toolhead no filament, re-feed, reschedule, execute single-color auto-refill F8
            #         # #ttyUSB0 serial send: FA
            #         # # Vendor note (240108): do not send FA yet
            #         # # Vendor note (240416): #         # if self.G_SerialPort1OpenFlag == True:
            #         #     self.Cmds_AMSSerial1Send("FA")
            #         #     self.G_PhrozenFluiddRespondInfo("Serial port 1-FA")
            #         # # Vendor note (241030): #         # elif self.G_SerialPort2OpenFlag == True:
            #         #     self.Cmds_AMSSerial2Send("FA")
            #         #     self.G_PhrozenFluiddRespondInfo("Serial port 2-FA")

            #         # # Vendor note (231229): encapsulated function
            #         # self.Cmds_MARetryInFila(gcmd)

            #         # Vendor note (241106): execute P8 full feed to handle AMS power-loss restart
            #         self.Cmds_CmdP8(gcmd)
            #         # Vendor note (241106): toolhead feed successful
            #         if self.G_P0M2MAStartPrintFlag==1:
            #             self.G_PhrozenFluiddRespondInfo("Single-color M3 mode, AMS present, toolhead has filament, resuming")
            #             # Vendor note (240125): encapsulated function
            #             self.Cmds_PhrozenKlipperResumeCommon()
            #         else:
            #             self.G_PhrozenFluiddRespondInfo("Standalone M3 mode, AMS present but toolhead no filament, pausing, please load manually")
            #             self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
            #             self.G_PhrozenFluiddRespondInfo("send_pause_command")
            #             #no filament, continue pausing
            #             self.G_KlipperIfPaused=True
            #             self.emit_protocol("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)

            #     else:
            #         self.G_PhrozenFluiddRespondInfo("Standalone M3 mode, no AMS, toolhead no filament, pausing, please load manually")
            #         self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
            #         self.G_PhrozenFluiddRespondInfo("send_pause_command")
            #         #no filament, continue pausing
            #         self.G_KlipperIfPaused=True
            #         self.emit_protocol("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)

            # filament required to resume print
            if self.G_ToolheadIfHaveFilaFlag:
                # Vendor note (240412): M3,ifAMS multi-material present,needAMS
                if self.G_AMSDevice1IfNormal == True or self.G_AMSDevice2IfNormal == True:
                    # self.G_PhrozenFluiddRespondInfo("toolhead has filament, has AMS multi-material, do notfilament, need to sending commandSTM32resume after state")
                    # #self.Cmds_CmdP8(gcmd)
                    # if self.G_SerialPort1OpenFlag == True:
                    #     self.Cmds_AMSSerial1Send("FA")
                    #     self.G_PhrozenFluiddRespondInfo("serial port 1 send FA")
                    # elif self.G_SerialPort2OpenFlag == True:
                    #     self.Cmds_AMSSerial2Send("FA")
                    #     self.G_PhrozenFluiddRespondInfo("serial port 2 send FA")
                    # self.G_PhrozenFluiddRespondInfo("toolhead has filament, resumedo notthen feed")
                    self.kaos_log(
                        "DEBUG",
                        "toolhead has filament, has AMS multi-material, but is P8 feed, prevent stop normal",
                        "SERIAL",
                    )
                    self.G_P0M2MAStartPrintFlag = 0

                    # Vendor note (250522): allowM3detect
                    self.G_IfChangeFilaOngoing = True

                    self.Cmds_CmdP8(gcmd)
                    # Vendor note (250619): check if AMS reconnected successfully
                    self.Cmds_USBConnectErrorCheck()
                    # if self.G_SerialPort1OpenFlag == True:
                    #     self.Cmds_AMSSerial1Send("FA")
                    #     self.G_PhrozenFluiddRespondInfo("serial port 1 send FA")
                    # elif self.G_SerialPort2OpenFlag == True:
                    #     self.Cmds_AMSSerial2Send("FA")
                    #     self.G_PhrozenFluiddRespondInfo("serial port 2 send FA")
                    # Vendor note (241106): toolhead feed successful
                    if self.G_P0M2MAStartPrintFlag == 1:
                        # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("can resume, STM32printing report error")
                        self.G_KlipperQuickPause = True
                        # # Vendor note (250427): # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                        #     self.G_PhrozenFluiddRespondInfo("serial port 1-AMSfinishtimingbuffer-full time")
                        # if self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                        #     self.G_PhrozenFluiddRespondInfo("serial port 2-AMSfinishtimingbuffer-full time")
                        # self.G_ProzenToolhead.dwell(1.5)
                        # Vendor note (250522): # self.G_PhrozenFluiddRespondInfo("[FIRMWARE] External macro PG109 heat-up")
                        # command_string = """
                        #     PG109
                        #     """
                        # self.G_PhrozenGCode.run_script_from_command(command_string)
                        # self.G_PhrozenFluiddRespondInfo("[FIRMWARE] External macro PG109 heat-up; command_string='%s'" % command_string)
                        # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("AT+RESTORE")
                        #     self.G_PhrozenFluiddRespondInfo("serial port 1-resume")
                        # if self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("AT+RESTORE")
                        #     self.G_PhrozenFluiddRespondInfo("serial port 2-resume")
                        # self.G_PhrozenFluiddRespondInfo("toolhead has filament, resume")
                        # # Vendor note (250611): # self.G_PhrozenFluiddRespondInfo("[PURGE] External macro PG108; heat, purge, wipe")
                        # command_string = """
                        #     PG108
                        #     """
                        # self.G_PhrozenGCode.run_script_from_command(command_string)
                        # # Vendor note (240125): encapsulated function
                        # self.Cmds_PhrozenKlipperResumeCommon()
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+EBLOCKCHECK")
                            self.kaos_log(
                                "DEBUG", "serial port 1-AMSstart timingbuffer-full time", "SERIAL"
                            )
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
                            self.kaos_log(
                                "DEBUG", "serial port 2-AMSstart timingbuffer-full time", "SERIAL"
                            )
                        # self.G_ProzenToolhead.dwell(1)
                        # Vendor note (250522): self.G_PhrozenFluiddRespondInfo("[FIRMWARE] External macro PG109 heat-up")
                        command_string = """
                            # PG109
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[FIRMWARE] External macro PG109 heat-up; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                        # Vendor note (251120): entering purge, set flag to prevent PG108 purge Hall-sensor pause at purge zone;
                        self.G_PG108Ingoing = 1
                        # Vendor note (250611): self.G_PhrozenFluiddRespondInfo("[PURGE] External macro PG108; heat, purge, wipe")
                        command_string = """
                            # PG108
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PG108Ingoing = 0
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                            self.kaos_log(
                                "DEBUG", "serial port 1-AMSfinishtimingbuffer-full time", "SERIAL"
                            )
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                            self.kaos_log(
                                "DEBUG", "serial port 2-AMSfinishtimingbuffer-full time", "SERIAL"
                            )
                        if self.STM32ReprotPauseFlag == 1:
                            self.kaos_log("DEBUG", "STM32 up report pause, cannot resume", "SERIAL")
                            # Vendor note (240125): encapsulated function
                            # self.Cmds_PhrozenKlipperResumeCommon()
                            self.G_ChangeChannelResumeFlag = False
                            self.G_PhrozenFluiddRespondInfo(
                                "+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan
                            )
                        else:
                            self.kaos_log("DEBUG", "toolhead has filament, resume", "SERIAL")
                            # Vendor note (240125): encapsulated function
                            self.Cmds_PhrozenKlipperResumeCommon()
                            self.G_ChangeChannelResumeFlag = False
                            self.G_PhrozenFluiddRespondInfo(
                                "+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan
                            )
                    else:
                        self.G_KlipperQuickPause = False
                        self.kaos_log("DEBUG", "toolhead no filament, M3 mode pause", "SERIAL")
                        if self.G_KlipperIfPaused == False:
                            self.G_PhrozenGCode.run_script_from_command(
                                "SAVE_GCODE_STATE NAME=PAUSE"
                            )
                            self.kaos_log("DEBUG", "[(cmds.python)]PAUSE", "SERIAL")
                            self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                            self.G_ProzenToolhead.wait_moves()
                            self.G_KlipperIfPaused = True
                            # self.emit_protocol("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                            self.G_PhrozenFluiddRespondInfo(
                                "+PAUSE:4,%d,%d"
                                % (
                                    self.G_ChangeChannelTimeoutOldChan,
                                    self.G_ChangeChannelTimeoutNewChan,
                                )
                            )
                        else:
                            self.kaos_log("DEBUG", "already pause, do notpause", "SERIAL")
                        self.G_ChangeChannelResumeFlag = False
                        self.G_PhrozenFluiddRespondInfo(
                            "+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan
                        )
                else:
                    self.G_KlipperQuickPause = False
                    self.kaos_log(
                        "DEBUG",
                        "toolhead has filament, has has AMS multi-material, resume",
                        "SERIAL",
                    )
                    self.kaos_log("DEBUG", "toolhead has filament, resumedo notthen feed", "SERIAL")
                    # Vendor note (250522): self.G_PhrozenFluiddRespondInfo("[FIRMWARE] External macro PG109 heat-up")
                    command_string = """
                        # PG109
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.kaos_log(
                        "DEBUG",
                        "[FIRMWARE] External macro PG109 heat-up; command_string='%s'"
                        % command_string,
                        "SERIAL",
                    )
                    # Vendor note (251120): entering purge, set flag to prevent PG108 purge Hall-sensor pause at purge zone;
                    self.G_PG108Ingoing = 1
                    command_string = """
                    # PG108
                    """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.G_PG108Ingoing = 0
                    self.kaos_log("DEBUG", "command_string='%s'" % command_string, "SERIAL")
                    self.kaos_log(
                        "DEBUG",
                        "purge complete, toolhead detectedhas filamentresume printing",
                        "SERIAL",
                    )
                    # Vendor note (240125): encapsulated function
                    self.Cmds_PhrozenKlipperResumeCommon()
                    self.G_ChangeChannelResumeFlag = False
                    self.G_PhrozenFluiddRespondInfo(
                        "+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan
                    )
            else:
                if self.G_AMSDevice1IfNormal == True or self.G_AMSDevice2IfNormal == True:
                    self.kaos_log(
                        "DEBUG", "toolhead no filament, has AMS multi-material, P8 feed", "SERIAL"
                    )
                    self.G_P0M2MAStartPrintFlag = 0

                    # Vendor note (250522): allowM3detect
                    self.G_IfChangeFilaOngoing = True

                    # Vendor note (241106): self.Cmds_CmdP8(gcmd)
                    # Vendor note (250619): check if AMS reconnected successfully
                    self.Cmds_USBConnectErrorCheck()
                    # Vendor note (241106): toolhead feed successful
                    if self.G_P0M2MAStartPrintFlag == 1:
                        # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("can resume, STM32printing report error")
                        self.G_KlipperQuickPause = True
                        # # Vendor note (250427): # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                        #     self.G_PhrozenFluiddRespondInfo("serial port 1-AMSfinishtimingbuffer-full time")
                        # if self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                        #     self.G_PhrozenFluiddRespondInfo("serial port 2-AMSfinishtimingbuffer-full time")
                        # self.G_ProzenToolhead.dwell(1.5)
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+EBLOCKCHECK")
                            self.kaos_log(
                                "DEBUG", "serial port 1-AMSstart timingbuffer-full time", "SERIAL"
                            )
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
                            self.kaos_log(
                                "DEBUG", "serial port 2-AMSstart timingbuffer-full time", "SERIAL"
                            )
                        # self.G_ProzenToolhead.dwell(1)
                        # Vendor note (250522): self.G_PhrozenFluiddRespondInfo("[FIRMWARE] External macro PG109 heat-up")
                        command_string = """
                            # PG109
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[FIRMWARE] External macro PG109 heat-up; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                        # Vendor note (251120): entering purge, set flag to prevent PG108 purge Hall-sensor pause at purge zone;
                        self.G_PG108Ingoing = 1
                        # Vendor note (250611): self.G_PhrozenFluiddRespondInfo("[PURGE] External macro PG108; heat, purge, wipe")
                        command_string = """
                            # PG108
                            """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PG108Ingoing = 0
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                            self.kaos_log(
                                "DEBUG", "serial port 1-AMSfinishtimingbuffer-full time", "SERIAL"
                            )
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                            self.kaos_log(
                                "DEBUG", "serial port 2-AMSfinishtimingbuffer-full time", "SERIAL"
                            )
                        if self.STM32ReprotPauseFlag == 1:
                            self.kaos_log("DEBUG", "STM32 up report pause, cannot resume", "SERIAL")
                            # Vendor note (240125): encapsulated function
                            # self.Cmds_PhrozenKlipperResumeCommon()
                            self.G_ChangeChannelResumeFlag = False
                            self.G_PhrozenFluiddRespondInfo(
                                "+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan
                            )
                        else:
                            self.kaos_log("DEBUG", "toolhead has filament, resume", "SERIAL")
                            # Vendor note (240125): encapsulated function
                            self.Cmds_PhrozenKlipperResumeCommon()
                            self.G_ChangeChannelResumeFlag = False
                            self.G_PhrozenFluiddRespondInfo(
                                "+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan
                            )
                    else:
                        self.G_KlipperQuickPause = False
                        self.kaos_log("DEBUG", "toolhead no filament, M3 mode pause", "SERIAL")
                        if self.G_KlipperIfPaused == False:
                            self.G_PhrozenGCode.run_script_from_command(
                                "SAVE_GCODE_STATE NAME=PAUSE"
                            )
                            self.kaos_log("DEBUG", "[(cmds.python)]PAUSE", "SERIAL")
                            self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                            self.G_ProzenToolhead.wait_moves()
                            # no filament, continue pausing
                            self.G_KlipperIfPaused = True
                            # self.emit_protocol("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                            self.G_PhrozenFluiddRespondInfo(
                                "+PAUSE:4,%d,%d"
                                % (
                                    self.G_ChangeChannelTimeoutOldChan,
                                    self.G_ChangeChannelTimeoutNewChan,
                                )
                            )
                        else:
                            self.kaos_log("DEBUG", "already pause, do notpause", "SERIAL")
                        self.G_ChangeChannelResumeFlag = False
                        self.G_PhrozenFluiddRespondInfo(
                            "+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan
                        )

                else:
                    self.G_KlipperQuickPause = False
                    self.kaos_log(
                        "DEBUG", "toolhead no filament, has AMS multi-material, M3pause", "SERIAL"
                    )
                    self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE")
                    self.kaos_log("DEBUG", "[(cmds.python)]PAUSE", "SERIAL")
                    self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                    self.G_ProzenToolhead.wait_moves()
                    # no filament, continue pausing
                    self.G_KlipperIfPaused = True
                    # self.emit_protocol("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                    self.G_PhrozenFluiddRespondInfo(
                        "+PAUSE:b,%d,%d"
                        % (
                            self.G_ChangeChannelTimeoutOldChan,
                            self.G_ChangeChannelTimeoutNewChan,
                        )
                    )
                    self.G_ChangeChannelResumeFlag = False
                    self.G_PhrozenFluiddRespondInfo(
                        "+RESUME:0,%d" % self.G_ChangeChannelTimeoutNewChan
                    )

            # self.G_ChangeChannelResumeFlag=False
            # self.emit_resume(2, self.G_ChangeChannelTimeoutNewChan)

            return

        # # Vendor note (240319): filamentH?
        # if self.G_ToolheadIfHaveFilaFlag:
        #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]Filament present in toolhead")
        #     # Vendor note (240319): before
        #     #self.Cmds_MoveToCutFilaPrepare()
        #     self.Cmds_AMSSerial1Send("H%d" % self.G_ChangeChannelTimeoutNewChan)
        #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]Special refill state before cut: H%d" % self.G_ChangeChannelTimeoutNewChan)
        #     time.sleep(1)

        # # Vendor note (240423): ,filament
        # self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutNewChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]G%d" % self.G_ChangeChannelTimeoutNewChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]AMS retract a distance, then toolhead retract mm: G%d" % self.G_ChangeChannelTimeoutNewChan)
        # self.G_PauseTriggerWhileChangeChannelFlag=False

        self.kaos_log(
            "DEBUG",
            "[FIRMWARE] External macro PG104; capture variables before toolchange",
            "SERIAL",
        )
        command_string = """
            # PG104
            """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.kaos_log(
            "DEBUG",
            "[FIRMWARE] External macro PG104; command_string='%s'" % command_string,
            "SERIAL",
        )
        self.IfDoPG102Flag = True

        self.kaos_log("DEBUG", "[TOOLCHANGE] External macro PG101 retract/pre-cut", "SERIAL")
        command_string = """
            # PG101
            """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.kaos_log(
            "DEBUG",
            "[SERVICE] Move to waiting area for purge; command_string='%s'" % command_string,
            "SERIAL",
        )
        self.IfDoPG102Flag = True

        # Vendor note (240328): manual commandexecute
        if self.ManualCmdFlag == True:
            self.kaos_log(
                "DEBUG",
                "[TOOLCHANGE] External macro PG106; manual command, purge disabled",
                "SERIAL",
            )
        else:
            # Vendor note (240319): ,filament,prevent
            self.kaos_log(
                "DEBUG", "[TOOLCHANGE] External macro PG106; purge residue before cut", "SERIAL"
            )
            self.PG102Flag = True
            self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
            command_string = """
            # PG106
            """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log("DEBUG", "command_string='%s'" % command_string, "SERIAL")
            self.PG102Flag = False
            self.kaos_log("DEBUG", "self.Flag=False", "SERIAL")

        # Vendor note (241012): executePG102
        self.IfDoPG102Flag = True

        # Vendor note (250717): ,
        self.G_ProzenToolhead.dwell(8)

        # Vendor note (250519): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_WIPEMOUTH")
        command_string = """
            # PRZ_WIPEMOUTH
            """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.kaos_log(
            "DEBUG",
            "[SERVICE] Move to service/waiting position; command_string='%s'" % command_string,
            "SERIAL",
        )

        # Vendor note (20231205): # Vendor note (231215): Z
        self.Cmds_MoveToCutFilaAction(gcmd)

        # Vendor note (231216): ifz,need
        if self.G_IfZPositionLiftUpFlag == True:
            command_string = """
                # G90
                # G91
                G1 Z-%f F8000
                """ % (
                self.G_AMSFilaCutZPositionLiftingUp,
            )
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_IfZPositionLiftUpFlag = False
            self.kaos_log(
                "DEBUG", "[TOOLHEAD] Z lowered; command_string='%s'" % command_string, "SERIAL"
            )

        # Vendor note (240226): AMSfilament,20mm
        # time.sleep(2)
        self.G_ProzenToolhead.dwell(0.5)

        # # Vendor note (240328): manual commandexecute
        # if self.ManualCmdFlag==True:
        #     self.G_PhrozenFluiddRespondInfo("[TOOLCHANGE] External macro PG106; manual command, purge disabled")
        # else:
        #     # Vendor note (240319): ,filament,prevent
        #     self.G_PhrozenFluiddRespondInfo("[TOOLCHANGE] External macro PG106; purge residue before cut")
        #     self.PG102Flag=True
        #     self.G_PhrozenFluiddRespondInfo("self.Flag=True")
        #     command_string = """
        #     PG106
        #     """
        #     self.G_PhrozenGCode.run_script_from_command(command_string)
        #     self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)
        #     self.PG102Flag=False
        #     self.G_PhrozenFluiddRespondInfo("self.Flag=False")

        # # Vendor note (241012): executePG102
        # self.IfDoPG102Flag=True

        # # Vendor note (240906): AMS,,lastdistance
        # # Vendor note (20231013): stm32filament change
        # # Vendor note (231129): stm32filament changeklipperfilament change,stm32filament change,klipperif,klipper
        # self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutOldChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]G%d" % self.G_ChangeChannelTimeoutOldChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]AMS old channel retract a distance first: G%d" % self.G_ChangeChannelTimeoutOldChan)

        # # # Vendor note (240906): host computer,stm32
        # # for i in range(5):#
        # #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]Waiting for old channel to retract")
        # #     self.G_ProzenToolhead.dwell(1.0)
        # #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]i=%d;T=%d" % (i,chan))

        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()
        if self.G_SerialPort1OpenFlag == True:
            # Vendor note (240913): ,,candistance,prevent,
            self.Cmds_AMSSerial1Send("AP")
            self.kaos_log("DEBUG", "serial port 1Sending command: AP; all filament", "SERIAL")
        if self.G_SerialPort2OpenFlag == True:
            self.Cmds_AMSSerial2Send("AP")
            self.kaos_log("DEBUG", "serial port 2Sending command: AP; all filament", "SERIAL")

        # self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutOldChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]G%d" % self.G_ChangeChannelTimeoutOldChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]AMS old channel retract a distance first: G%d" % self.G_ChangeChannelTimeoutOldChan)

        # self.G_ProzenToolhead.dwell(5)

        # self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutNewChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]G%d" % self.G_ChangeChannelTimeoutNewChan)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]AMS new channel retract a distance first: G%d" % self.G_ChangeChannelTimeoutNewChan)

        # self.G_ProzenToolhead.dwell(5)

        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]External macro command-PG101")
        # command_string = """
        #     PG101
        #     """
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperResume]-feed waiting zoneposition;command_string='%s'" % command_string)
        # self.IfDoPG102Flag=True

        # Vendor note (231216): iffilament changeduring,filament changeduringz,execute,z,
        # Vendor note (231216): ifz,need
        if self.G_IfZPositionLiftUpFlag == True:
            command_string = """
                # G90
                # G91
                G1 Z-%f F8000
                """ % (
                self.G_AMSFilaCutZPositionLiftingUp,
            )
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.G_IfZPositionLiftUpFlag = False
            self.kaos_log(
                "DEBUG", "[TOOLHEAD] Z lowered; command_string='%s'" % command_string, "SERIAL"
            )

        # Vendor note (240920): filament,flag
        # self.ToolheadCutFlag=False

        # Vendor note (250519): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_CUT_WAITINGAREA")
        command_string = """
            # PRZ_CUT_WAITINGAREA
            """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.kaos_log(
            "DEBUG",
            "[SERVICE] Move to service/waiting position; command_string='%s'" % command_string,
            "SERIAL",
        )

        # Vendor note (240913): self.G_ProzenToolhead.dwell(6)
        # Vendor note (240911): Gafter5checkfilament
        # Vendor note (231201): checkfilamentnormal,normal
        self.Cmds_CutFilaIfNormalCheck()
        # Vendor note (240912): ,detect,return
        # Vendor note (250109): becauseMCneedre-feedcannotreturn
        # if self.G_KlipperIfPaused == True:
        #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)]6s after cut, toolhead still detects filament, cutter error, check cutter, pause Klipper")
        #     #Lo_ChangeChannelIfSuccess = False
        #     return
        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        # [Translated vendor note] #Vendor note (241106): P8feed, AMS
        # self.Cmds_CmdP8(gcmd)
        # [Translated vendor note] #Vendor note (241106):toolheadfeed
        # if self.G_P0M2MAStartPrintFlag==1:
        # [Translated vendor note] self.G_PhrozenFluiddRespondInfo("M3mode, AMStoolheadfilament")
        # [Translated vendor note] #Vendor note (240125):
        # self.Cmds_PhrozenKlipperResumeCommon()
        # else:
        # [Translated vendor note] self.G_PhrozenFluiddRespondInfo("M3mode, AMStoolheadfilamentpause, ")
        # self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
        # self.G_PhrozenFluiddRespondInfo("send_pause_command")
        # [Translated vendor note] #filamentpause
        # self.G_KlipperIfPaused=True
        # self.emit_protocol("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)

        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")

        if self.G_ChangeChannelTimeoutOldChan == -1 and self.G_ChangeChannelTimeoutNewChan == -1:
            self.kaos_log(
                "DEBUG",
                "multi-material printing, printingwhen pause, new old, ifP2A1 normal normal, then resume, down command",
                "SERIAL",
            )
            if self.G_ToolheadIfHaveFilaFlag == False:
                self.kaos_log(
                    "DEBUG",
                    "toolhead5toolhead detectedfilament, filamentalready, resume;",
                    "SERIAL",
                )
                # Vendor note (240125): encapsulated function
                self.Cmds_PhrozenKlipperResumeCommon()
                self.G_ChangeChannelResumeFlag = False
                self.G_ChangeChannelFirstFilaFlag = True
                self.G_IfChangeFilaOngoing = False

                self.kaos_log("DEBUG", "return", "SERIAL")
                return

        # Vendor note (250102): ,enable,prevent
        # self.G_ProzenToolhead.dwell(0.5)
        self.G_PrintCountNum = self.G_PrintCountNum - 1

        # Vendor note (231115): logic;before
        # filament
        if self.G_ToolheadIfHaveFilaFlag:
            self.kaos_log("DEBUG", "toolhead has filament, can resume printing", "SERIAL")
            # Vendor note (240323): ,1
            self.Cmds_P1CnAutoChangeChannel(
                self.G_ChangeChannelTimeoutNewChan, self.G_ChangeChannelTimeoutNewGcmd
            )
            # Vendor note (240325): filament changesuccessful,candata
            if self.G_MCModeCanResumeFlag == True:
                self.kaos_log("DEBUG", "successful, can resume number data", "SERIAL")
                # Vendor note (240125): encapsulated function
                self.Cmds_PhrozenKlipperResumeCommon()

                self.G_ProzenToolhead.dwell(1)
                self.G_ChangeChannelResumeFlag = False
                self.emit_resume(2, self.G_ChangeChannelTimeoutNewChan)
            else:
                self.kaos_log("DEBUG", "successful, can resume number data", "SERIAL")

                # Vendor note (250527): pause at feed waiting area
                self.kaos_log(
                    "DEBUG", "[SERVICE] Start external macro PRZ_PAUSE_WAITINGAREA", "SERIAL"
                )
                command = """
                # PRZ_PAUSE_WAITINGAREA
                """
                self.G_PhrozenGCode.run_script_from_command(command)
                self.kaos_log(
                    "DEBUG", "[SERVICE] End external macro: command=%s" % (command), "SERIAL"
                )

                self.G_ProzenToolhead.dwell(1)
                self.G_ChangeChannelResumeFlag = False
                self.emit_resume(0, self.G_ChangeChannelTimeoutNewChan)

        # filament
        else:
            self.kaos_log("DEBUG", "toolhead has filament, re action", "SERIAL")
            # Vendor note (240323): ,1
            self.Cmds_P1CnAutoChangeChannel(
                self.G_ChangeChannelTimeoutNewChan, self.G_ChangeChannelTimeoutNewGcmd
            )
            # Vendor note (240325): filament changesuccessful,candata
            if self.G_MCModeCanResumeFlag == True:
                self.kaos_log("DEBUG", "successful, can resume number data", "SERIAL")
                # Vendor note (240125): encapsulated function
                self.Cmds_PhrozenKlipperResumeCommon()

                self.G_ProzenToolhead.dwell(1)
                self.G_ChangeChannelResumeFlag = False
                self.emit_resume(2, self.G_ChangeChannelTimeoutNewChan)

            else:
                self.kaos_log("DEBUG", "successful, can resume number data", "SERIAL")

                # Vendor note (250527): pause at feed waiting area
                self.kaos_log(
                    "DEBUG", "[SERVICE] Start external macro PRZ_PAUSE_WAITINGAREA", "SERIAL"
                )
                command = """
                # PRZ_PAUSE_WAITINGAREA
                """
                self.G_PhrozenGCode.run_script_from_command(command)
                self.kaos_log(
                    "DEBUG", "[SERVICE] End external macro: command=%s" % (command), "SERIAL"
                )

                # Vendor note (240509): # # Vendor note (240426): failed,need
                # if len(self.G_PauseToLCDString)==0:
                #     # Vendor note (0429): prevent
                #     #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                #     self.emit_protocol("+PAUSE:4,%d" % self.G_ChangeChannelTimeoutNewChan)
                self.G_ProzenToolhead.dwell(1)
                self.G_ChangeChannelResumeFlag = False
                self.emit_resume(0, self.G_ChangeChannelTimeoutNewChan)

        # Vendor note (250102): ,enable,prevent
        # self.G_ProzenToolhead.dwell(0.5)
        self.G_PrintCountNum = self.G_PrintCountNum - 1
        # Vendor note (250102): filament changecalculate;1filament change
        if self.G_PrintCountNum <= 0:
            self.G_PrintCountNum = 0
            self.kaos_log("DEBUG", "resumefinish, #1 time", "SERIAL")
        else:
            command_string = """
                # M106 S255
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG",
                "[PRINT] Resume complete, turn fans on; command_string='%s'" % command_string,
                "SERIAL",
            )
            self.kaos_log("DEBUG", "self.G_PrintCountNum='%d'" % self.G_PrintCountNum, "SERIAL")
        # self.G_ProzenToolhead.dwell(0.5)

    def Cmds_PhrozenKlipperPauseScreen(self, gcmd):
        _ = gcmd

        self.kaos_log("DEBUG", "[(cmds.python)Cmds_PhrozenKlipperPauseScreen]", "SERIAL")
        # // lancaigang231202:+PAUSE:1,ch;1-feed used up, jam, pause
        # // lancaigang231202:+PAUSE:2,ch;2-pause ACK
        # // lancaigang231204:+PAUSE:3,ch;3-new channel slow-refill during print timeout 10s, pause
        # // lancaigang231205:+PAUSE:4,ch;4-new channel feed timeout 50s, pause
        # // lancaigang231205:+PAUSE:5,ch;5-new channel fast-refill during print timeout 10s, pause
        # // lancaigang231205:+PAUSE:6,ch;6-entry to park timeout 10s, pause
        # // lancaigang231205:+PAUSE:7,ch;7-buffer full timeout 30s, pause
        # // lancaigang231205:+PAUSE:8,ch;8-toolhead cutter or sensor error, pause
        # // lancaigang231205:+PAUSE:9,ch;9-filament change timeout 120s, pause
        # // lancaigang231202:+PAUSE:a,ch;a-park to buffer entry timeout 10s, pause
        # // lancaigang231202:+PAUSE:b,ch;b-reserved
        # // lancaigang231202:+PAUSE:c,ch;c-reserved
        # // lancaigang231202:+PAUSE:d,ch;d-reserved
        # // lancaigang231202:+PAUSE:10,ch;10-fluidd
        # klipper active pause
        self.kaos_log("DEBUG", "=====PAUSE=====", "SERIAL")
        self.kaos_log(
            "DEBUG",
            "=====PAUSE=====self.G_ChangeChannelTimeoutOldChan=%d"
            % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====PAUSE=====self.G_ChangeChannelTimeoutNewChan=%d"
            % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        # Vendor note (250527): execute
        self.G_KlipperQuickPause = False

        if self.PG102Flag == True:
            self.kaos_log("DEBUG", "purge, pause", "SERIAL")
            # self.emit_protocol("+PAUSE:10,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
            return

        # Vendor note (231228): MCallowZ
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            # Vendor note (231216): iffilament changeduring,filament changeduringz,execute,z,
            # Vendor note (231216): ifz,need
            if self.G_IfZPositionLiftUpFlag == True:
                command_string = """
                    # G90
                    # G91
                    G1 Z-%f F8000
                    """ % (
                    self.G_AMSFilaCutZPositionLiftingUp,
                )
                self.G_PhrozenGCode.run_script_from_command(command_string)
                self.G_IfZPositionLiftUpFlag = False
                self.kaos_log(
                    "DEBUG", "[TOOLHEAD] Z lowered; command_string='%s'" % command_string, "SERIAL"
                )

        if self.G_ToolheadIfHaveFilaFlag == True:
            self.kaos_log("DEBUG", "toolhead has filament, set resumedo notthen feed", "SERIAL")
            # Vendor note (240116): MAneedstm32
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:  # MA
                if self.G_KlipperInPausing == False:
                    self.kaos_log(
                        "DEBUG", "No pause is in progress; a new pause is allowed", "SERIAL"
                    )
                    # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("Quick pause enabled")
                    self.G_KlipperQuickPause = True
                    # Vendor note (241012): AMS
                    self.Cmds_PhrozenKlipperPause(None)
                    # self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                else:
                    self.kaos_log(
                        "DEBUG",
                        "A pause is already in progress; a new pause is not allowed",
                        "SERIAL",
                    )

            # Vendor note (240412): ,ifAMS multi-material present,needAMS
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:  # M3
                if self.G_AMSDevice1IfNormal == True:
                    self.kaos_log(
                        "DEBUG",
                        "Toolhead has filament, single-color M3 mode with AMS multi-material, need to pause STM32",
                        "SERIAL",
                    )
                    if self.G_KlipperInPausing == False:
                        self.kaos_log(
                            "DEBUG", "No pause is in progress; a new pause is allowed", "SERIAL"
                        )
                        # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("Quick pause enabled")
                        self.G_KlipperQuickPause = True
                        # Vendor note (241012): AMS
                        self.Cmds_PhrozenKlipperPause(None)
                        # self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                    else:
                        self.kaos_log(
                            "DEBUG",
                            "A pause is already in progress; a new pause is not allowed",
                            "SERIAL",
                        )
                else:
                    self.kaos_log(
                        "DEBUG",
                        "Toolhead has filament, single-color M3 mode, no AMS multi-material, STM32 pause not needed",
                        "SERIAL",
                    )
                    if self.G_KlipperInPausing == False:
                        self.kaos_log(
                            "DEBUG", "No pause is in progress; a new pause is allowed", "SERIAL"
                        )
                        # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("Quick pause enabled")
                        self.G_KlipperQuickPause = True
                        self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                    else:
                        self.kaos_log(
                            "DEBUG",
                            "A pause is already in progress; a new pause is not allowed",
                            "SERIAL",
                        )

            else:  # MC
                # Vendor note (240427): #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                # Vendor note (240427): ,needstm32
                self.kaos_log(
                    "DEBUG",
                    "toolhead has filament, MCmulti-material mode has AMS multi-material, need to stm32 pause",
                    "SERIAL",
                )
                if self.G_KlipperInPausing == False:
                    self.kaos_log(
                        "DEBUG", "No pause is in progress; a new pause is allowed", "SERIAL"
                    )
                    # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("Quick pause enabled")
                    self.G_KlipperQuickPause = True
                    # Vendor note (241012): AMS
                    self.Cmds_PhrozenKlipperPause(None)
                    # self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                else:
                    self.kaos_log(
                        "DEBUG",
                        "A pause is already in progress; a new pause is not allowed",
                        "SERIAL",
                    )

            self.G_IfToolheadHaveFilaInitiativePauseFlag = True
        else:
            # Vendor note (231216): iffilament change,becausefilament changealreadyz,z
            self.kaos_log(
                "DEBUG", "toolhead has filament, set resume need to STM32then feed", "SERIAL"
            )
            if self.G_KlipperInPausing == False:
                self.kaos_log("DEBUG", "No pause is in progress; a new pause is allowed", "SERIAL")
                # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("Quick pause enabled")
                self.G_KlipperQuickPause = True
                # Vendor note (241012): AMS
                self.Cmds_PhrozenKlipperPause(None)
                # self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
            else:
                self.kaos_log(
                    "DEBUG", "A pause is already in progress; a new pause is not allowed", "SERIAL"
                )

        # Vendor note (250527): pause at feed waiting area
        self.kaos_log("DEBUG", "[SERVICE] Start external macro PRZ_PAUSE_WAITINGAREA", "SERIAL")
        command = """
        # PRZ_PAUSE_WAITINGAREA
        """
        self.G_PhrozenGCode.run_script_from_command(command)
        self.kaos_log("DEBUG", "[SERVICE] End external macro: command=%s" % (command), "SERIAL")

        self.kaos_log("DEBUG", "disable or fluidd network move pause, pause", "SERIAL")
        # self.emit_protocol("+PAUSE:10,%d" % self.G_ChangeChannelTimeoutNewChan)
        self.G_PhrozenFluiddRespondInfo(
            "+PAUSE:10,%d,%d"
            % (self.G_ChangeChannelTimeoutOldChan, self.G_ChangeChannelTimeoutNewChan)
        )

    # PRZ_CANCEL
    def Cmds_PhrozenKlipperCancel(self, gcmd):
        self.kaos_log(
            "DEBUG", "[(cmds.python)Cmds_PhrozenKlipperCancel]klipper cancelprinting;", "SERIAL"
        )

        self.emit_protocol("+CANCEL:0,%d" % self.G_ChangeChannelTimeoutNewChan)

        self.kaos_log("DEBUG", "=====CANCEL=====", "SERIAL")
        self.kaos_log(
            "DEBUG",
            "=====CANCEL=====self.G_ChangeChannelTimeoutOldChan=%d"
            % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====CANCEL=====self.G_ChangeChannelTimeoutNewChan=%d"
            % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        self.kaos_log("DEBUG", "External macro command: CANCEL_PRINT", "SERIAL")
        # Vendor note (240120): cfgconfig
        command = """
        # CANCEL_PRINT
        """
        self.G_PhrozenGCode.run_script_from_command(command)
        self.kaos_log("DEBUG", "[PRINT] Calling macro command=%s" % (command), "SERIAL")

        # unlock
        self.Base_AMSSerialCmdUnlock()

        # # Vendor note (231216): # eventtime = self.G_PhrozenReactor.monotonic()
        # # Determine "printing" status
        # idle_timeout = self.G_PhrozenPrinter.lookup_object("idle_timeout")
        # is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
        # if is_printing:
        #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperCancel];='%s'")
        # else:
        #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperCancel];='%s;return'")
        #     return

        # Vendor note (231207): +PAUSE:1label
        self.G_IfInFilaBlockFlag = False
        # Vendor note (240321): PG102label
        self.PG102DelayPauseFlag = False
        # Vendor note (240426): set flagfalse
        self.G_ResumeProcessCheckPauseStatus = False
        self.G_CancelFlag = True
        # Vendor note (240411): ifno P0 M3 command received, skip runout detection
        self.G_P0M3Flag = False

        self.ManualCmdFlag = False
        self.G_CutCheckTest = False

        # Vendor note (240427): AMS error restart, needs logging
        self.G_AMS1ErrorRestartFlag = False
        self.G_AMS1ErrorRestartCount = 0

        self.G_AMS2ErrorRestartFlag = False
        self.G_AMS2ErrorRestartCount = 0

        # Vendor note (240124): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0

        self.G_IfToolheadHaveFilaInitiativePauseFlag = False
        # Vendor note (250526): ,allowgcode,complete
        self.G_KlipperInPausing = False
        # Vendor note (250527): execute
        self.G_KlipperQuickPause = False
        # Vendor note (250607): print status: 1=retracting; 2=feeding; 3=printing; 4=paused
        self.G_KlipperPrintStatus = -1
        self.G_ASM1DisconnectErrorCount = 0
        # Vendor note (250812): single-color runout detection, return to pause zone
        self.G_RetryToPauseAreaFlag = False
        self.G_RetryToPauseAreaCount = 0
        self.G_P10SpitNum = 0
        self.G_IfChangeFilaOngoing = False
        # Vendor note (240223): failed
        self.ToolheadCutFlag = False

        self.G_P0M1MCNoneAMS = 0
        self.kaos_log("DEBUG", "self.G_P0M1MCNoneAMS=0", "SERIAL")

        # Vendor note (250515): clearconfigdata
        self.Cmds_GetUartScreenCfgClear()

        # Vendor note (250807): clear
        self.G_PhrozenPrinterCancelPauseResume.cmd_CLEAR_PAUSE(None)
        self.kaos_log("DEBUG", "clearpause state", "SERIAL")

        # Vendor note (241016): #self.ToolheadCutFlag=False

        # #AMS
        # #self.Cmds_CmdP4(None)
        # # Vendor note (240125): # # Vendor note (240507): ,M0
        # # Vendor note (240516): ,execute
        # self.Cmds_AMSSerial1Send("AT+PAUSE")
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperCancel]AT+PAUSE: pause STM32 motor")

        # #klipper active pause
        # self.Cmds_PhrozenKlipperPause(None)
        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)

        self.G_ProzenToolhead.dwell(1.0)

        # Vendor note (240416): # Vendor note (240516): ,execute
        # if self.G_SerialPort1OpenFlag == True:
        #     self.Cmds_AMSSerial1Send("M0")
        #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperCancel]Sending command: M0")

        # #AMS
        # #self.Cmds_CmdP4(None)
        # # Vendor note (240125): # # Vendor note (240507): ,M0
        # # Vendor note (240516): ,execute
        # self.Cmds_AMSSerial1Send("AT+PAUSE")
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_PhrozenKlipperCancel]AT+PAUSE: pause STM32 motor")

        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()

        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.kaos_log(
                "DEBUG", "sending command:MAmulti-material mode-MC-AMSemptymode", "SERIAL"
            )
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.kaos_log(
                "DEBUG", "sending command:M2single-color refill mode-MA-AMSemptymode", "SERIAL"
            )
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MA")
                self.kaos_log("DEBUG", "Serial port 1 sending command: MA", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MA")
                self.kaos_log("DEBUG", "Serial port 2 sending command: MA", "SERIAL")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.kaos_log("DEBUG", "sending command:M3single-color mode-MA-AMSemptymode", "SERIAL")
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MA")
                self.kaos_log("DEBUG", "Serial port 1 sending command: MA", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MA")
                self.kaos_log("DEBUG", "Serial port 2 sending command: MA", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Unknown mode, pause AMS", "SERIAL")

            if self.G_SerialPort1OpenFlag == True:
                # Vendor note (240516): ,AMS
                self.Cmds_AMSSerial1Send("AT+PAUSE")
                self.kaos_log("DEBUG", "serial port 1 send AT+PAUSEpausestm32 machine", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("AT+PAUSE")
                self.kaos_log("DEBUG", "serial port 2 send AT+PAUSEpausestm32 machine", "SERIAL")

        self.G_P0M2MAStartPrintFlag = 0
        # Vendor note (250104): P2A3flag
        self.G_P2A3Flag = 0

        # Vendor note (250102): calculate
        self.G_PrintCountNum = 0

        # Vendor note (20231013): disconnect
        # self.Device_DisconnectAMSDevice()
        # Vendor note (250712): ,disconnect
        # Vendor note (250815): ,prevent
        self.Cmds_CmdP29(None)

        self.G_AMSDeviceWorkMode = AMS_WORK_MODE_UNKNOW

        self.emit_protocol("+CANCEL:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    def Cmds_USBConnectErrorCheck(self):
        self.kaos_log("DEBUG", "[(cmds.python)Cmds_USBConnectErrorCheck]", "SERIAL")

        self.kaos_log("DEBUG", "self.G_CancelFlag='%s'" % self.G_CancelFlag, "SERIAL")
        self.kaos_log("DEBUG", "Current mode", "SERIAL")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.emit_mode(0, "unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.emit_mode(1, "MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.emit_mode(2, "MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.emit_mode(3, "RUNOUT")
        else:
            self.emit_mode(-1, "error")

        try:
            self.kaos_log(
                "DEBUG",
                "[(cmds.py)Cmds_USBConnectErrorCheck]Reinitializing serial port 1",
                "SERIAL",
            )
            self.G_SerialPort1Obj = serial.Serial(
                self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3
            )
            # serial port opened successfully
            if self.G_SerialPort1Obj is not None:
                if self.G_SerialPort1Obj.is_open:
                    self.G_SerialPort1OpenFlag = True
                    self.kaos_log("DEBUG", "Reinitializing serial port 1 successful", "SERIAL")
                    # Vendor note (231213): open serial port
                    self.G_SerialPort1Obj.flushInput()  # clean serial write cache
                    self.G_SerialPort1Obj.flush()
                    self.kaos_log("DEBUG", "Serial port 1 buffers cleared", "SERIAL")
                    self.kaos_log("DEBUG", "Re-registering serial port 1 callback", "SERIAL")
                    self.G_SerialPort1RecvTimmer = self.G_PhrozenReactor.register_timer(
                        self.Device_TimmerUart1Recv, self.G_PhrozenReactor.NOW
                    )

                    if "+PAUSE:g" in self.G_PauseToLCDString:
                        self.kaos_log(
                            "DEBUG",
                            "if is USBfilament runouterror, clear report error signal info",
                            "SERIAL",
                        )
                        # Vendor note (250902): cannot,prevent
                        # self.G_PauseToLCDString=""
                        self.G_PauseToLCDString = "+PAUSE:4,%d,%d" % (
                            self.G_ChangeChannelTimeoutOldChan,
                            self.G_ChangeChannelTimeoutNewChan,
                        )
                        self.kaos_log(
                            "DEBUG",
                            "len(self.G_PauseToLCDString)='%d'" % len(self.G_PauseToLCDString),
                            "SERIAL",
                        )

        except:
            self.kaos_log(
                "DEBUG", "Unable to open tty1. Check the USB connection or try rebooting.", "SERIAL"
            )
            self.G_SerialPort1OpenFlag = False

            if (
                self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW
                or self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT
            ):
                self.kaos_log(
                    "DEBUG",
                    "single-color M3 or Unknown mode, do notnew pause signal info",
                    "SERIAL",
                )
            else:
                if len(self.G_PauseToLCDString) == 0:
                    self.kaos_log("DEBUG", "new pause signal info", "SERIAL")
                    self.kaos_log("DEBUG", "pause:+PAUSE:g", "SERIAL")
                    self.G_PauseToLCDString = "+PAUSE:g,%d,%d" % (
                        self.G_ChangeChannelTimeoutOldChan,
                        self.G_ChangeChannelTimeoutNewChan,
                    )
                else:
                    # self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                    self.kaos_log("DEBUG", "new pause signal info", "SERIAL")
                    self.G_PauseToLCDString = "+PAUSE:g,%d,%d" % (
                        self.G_ChangeChannelTimeoutOldChan,
                        self.G_ChangeChannelTimeoutNewChan,
                    )
                    self.kaos_log("DEBUG", "pause:+PAUSE:g", "SERIAL")

        try:
            self.kaos_log(
                "DEBUG",
                "[(cmds.py)Cmds_USBConnectErrorCheck]Reinitializing serial port 2",
                "SERIAL",
            )
            self.G_SerialPort2Obj = serial.Serial(
                self.G_Serialport2Define, SERIAL_PORT_BAUD, timeout=3
            )
            # serial port opened successfully
            if self.G_SerialPort2Obj is not None:
                if self.G_SerialPort2Obj.is_open:
                    self.G_SerialPort2OpenFlag = True
                    self.kaos_log("DEBUG", "Reinitializing serial port 2 successful", "SERIAL")
                    self.G_SerialPort2Obj.flushInput()  # clean serial write cache
                    self.G_SerialPort2Obj.flush()
                    self.kaos_log("DEBUG", "Serial port 2 buffers cleared", "SERIAL")
                    self.kaos_log("DEBUG", "Re-registering serial port 2 callback", "SERIAL")
                    self.G_SerialPort2RecvTimmer = self.G_PhrozenReactor.register_timer(
                        self.Device_TimmerUart2Recv, self.G_PhrozenReactor.NOW
                    )
                    # if "+PAUSE:g" in self.G_PauseToLCDString:
                    #     self.G_PauseToLCDString=""
        except:
            self.kaos_log(
                "DEBUG", "Unable to open tty2. Check the USB connection or try rebooting.", "SERIAL"
            )
            self.G_SerialPort2OpenFlag = False

    def Cmds_CutFilaIfNormalCheck(self):
        self.kaos_log("DEBUG", "[(cmds.python)Cmds_CutFilaIfNormalCheck]", "SERIAL")

        self.ToolheadCutFlag = False

        # Vendor note (250527): execute
        self.G_KlipperQuickPause = False

        # 8Scheckfilament,normal8sfilament
        # Vendor note (20231013): 8s
        # Vendor note (231201): 5
        # Vendor note (240912): klipperstm32time
        # self.G_ProzenToolhead.dwell(6.0)

        self.kaos_log("DEBUG", "when detected is no has filament???", "SERIAL")
        # Vendor note (240125): cannotsleep,block
        # time.sleep(5)
        # filament8s,filament,normalfilament
        if self.G_ToolheadIfHaveFilaFlag:
            # raise self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]filament changecheckfilament,detectfilament;cmd='%s'" % (gcmd.get_commandline()))
            self.kaos_log(
                "DEBUG",
                "toolhead5toolhead detectedfilament, please check cutter is no error, move failed;klipper pause",
                "SERIAL",
            )
            # // lancaigang231202:+PAUSE:1,ch;1-feed used up, jam, pause
            # // lancaigang231202:+PAUSE:2,ch;2-pause ACK
            # // lancaigang231204:+PAUSE:3,ch;3-new channel slow-refill during print timeout 10s, pause
            # // lancaigang231205:+PAUSE:4,ch;4-new channel feed timeout 50s, pause
            # // lancaigang231205:+PAUSE:5,ch;5-new channel fast-refill during print timeout 10s, pause
            # // lancaigang231205:+PAUSE:6,ch;6-entry to park timeout 10s, pause
            # // lancaigang231205:+PAUSE:7,ch;7-buffer full timeout 30s, pause
            # // lancaigang231205:+PAUSE:8,ch;8-toolhead cutter or sensor error, pause
            # // lancaigang231205:+PAUSE:9,ch;9-filament change timeout 120s, pause
            # // lancaigang231202:+PAUSE:a,ch;a-park to buffer entry timeout 10s, pause
            # // lancaigang231202:+PAUSE:b,ch;b-reserved
            # // lancaigang231202:+PAUSE:c,ch;c-reserved
            # // lancaigang231202:+PAUSE:d,ch;d-reserved
            # // lancaigang231202:+PAUSE:10,ch;10-fluidd
            # Vendor note (240223): beforeZalreadyfailed,beforeZ
            if self.G_IfZPositionLiftUpFlag == True:
                command_string = """
                    # G90
                    # G91
                    G1 Z-%f F8000
                    """ % (
                    self.G_AMSFilaCutZPositionLiftingUp,
                )
                self.G_PhrozenGCode.run_script_from_command(command_string)
                self.G_IfZPositionLiftUpFlag = False
                self.kaos_log(
                    "DEBUG", "[TOOLHEAD] Z lowered; command_string='%s'" % command_string, "SERIAL"
                )

            self.ToolheadCutFlag = True

            # Vendor note (240322): ifalready,
            if self.STM32ReprotPauseFlag == 1:
                self.kaos_log("DEBUG", "already pause, do notpause", "SERIAL")
                self.kaos_log("DEBUG", "toolhead cutter or device error, pause", "SERIAL")
                # self.emit_protocol("+PAUSE:8,%d" % self.G_ChangeChannelTimeoutOldChan)
                # Vendor note (250414): #self.emit_protocol("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))

                # Vendor note (250619): check if AMS reconnected successfully
                self.Cmds_USBConnectErrorCheck()

                if len(self.G_PauseToLCDString) == 0:
                    self.G_PhrozenFluiddRespondInfo(
                        "+PAUSE:8,%d,%d"
                        % (
                            self.G_ChangeChannelTimeoutOldChan,
                            self.G_ChangeChannelTimeoutNewChan,
                        )
                    )
                else:
                    # self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                    self.kaos_log("DEBUG", "new pause signal info", "SERIAL")
                    # Vendor note (250721): ifAMS,8
                    self.G_PauseToLCDString = "+PAUSE:8,%d,%d" % (
                        self.G_ChangeChannelTimeoutOldChan,
                        self.G_ChangeChannelTimeoutNewChan,
                    )
                    self.kaos_log("DEBUG", self.G_PauseToLCDString, "SERIAL")

            else:
                # Vendor note (240328): ifmanual command,
                if self.ManualCmdFlag == True:
                    self.kaos_log("DEBUG", "move command, klipperdo not executepause", "SERIAL")
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("AT+PAUSE")
                        self.kaos_log(
                            "DEBUG", "serial port 1 send AT+PAUSEpausestm32 machine", "SERIAL"
                        )
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AT+PAUSE")
                        self.kaos_log(
                            "DEBUG", "serial port 2 send AT+PAUSEpausestm32 machine", "SERIAL"
                        )

                    # Vendor note (250805): #self.emit_protocol("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    if len(self.G_PauseToLCDString) == 0:
                        self.G_PhrozenFluiddRespondInfo(
                            "+PAUSE:8,%d,%d"
                            % (
                                self.G_ChangeChannelTimeoutOldChan,
                                self.G_ChangeChannelTimeoutNewChan,
                            )
                        )
                    else:
                        # self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                        self.kaos_log("DEBUG", "new pause signal info", "SERIAL")
                        # Vendor note (250721): ifAMS,8
                        self.G_PauseToLCDString = "+PAUSE:8,%d,%d" % (
                            self.G_ChangeChannelTimeoutOldChan,
                            self.G_ChangeChannelTimeoutNewChan,
                        )
                        self.kaos_log("DEBUG", self.G_PauseToLCDString, "SERIAL")
                elif self.G_CutCheckTest == True:
                    self.kaos_log("DEBUG", "cutter, klipperdo not executepause", "SERIAL")
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("AT+PAUSE")
                        self.kaos_log(
                            "DEBUG", "serial port 1 send AT+PAUSEpausestm32 machine", "SERIAL"
                        )
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AT+PAUSE")
                        self.kaos_log(
                            "DEBUG", "serial port 2 send AT+PAUSEpausestm32 machine", "SERIAL"
                        )

                    # Vendor note (250805): #self.emit_protocol("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    if len(self.G_PauseToLCDString) == 0:
                        self.G_PhrozenFluiddRespondInfo(
                            "+PAUSE:8,%d,%d"
                            % (
                                self.G_ChangeChannelTimeoutOldChan,
                                self.G_ChangeChannelTimeoutNewChan,
                            )
                        )
                    else:
                        # self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                        self.kaos_log("DEBUG", "new pause signal info", "SERIAL")
                        # Vendor note (250721): ifAMS,8
                        self.G_PauseToLCDString = "+PAUSE:8,%d,%d" % (
                            self.G_ChangeChannelTimeoutOldChan,
                            self.G_ChangeChannelTimeoutNewChan,
                        )
                        self.kaos_log("DEBUG", self.G_PauseToLCDString, "SERIAL")
                else:
                    self.kaos_log("DEBUG", "toolhead cutter or device error, pause", "SERIAL")

                    if self.G_KlipperInPausing == False:
                        self.kaos_log(
                            "DEBUG", "No pause is in progress; a new pause is allowed", "SERIAL"
                        )
                        # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("Quick pause enabled")
                        self.G_KlipperQuickPause = True
                        # klipper active pause
                        self.Cmds_PhrozenKlipperPause(None)
                    else:
                        self.kaos_log(
                            "DEBUG",
                            "A pause is already in progress; a new pause is not allowed",
                            "SERIAL",
                        )

                    # self.emit_protocol("+PAUSE:8,%d" % self.G_ChangeChannelTimeoutOldChan)
                    # Vendor note (250619): check if AMS reconnected successfully
                    self.Cmds_USBConnectErrorCheck()

                    # Vendor note (250414): #self.emit_protocol("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    if len(self.G_PauseToLCDString) == 0:
                        self.G_PhrozenFluiddRespondInfo(
                            "+PAUSE:8,%d,%d"
                            % (
                                self.G_ChangeChannelTimeoutOldChan,
                                self.G_ChangeChannelTimeoutNewChan,
                            )
                        )
                    else:
                        # self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                        self.kaos_log("DEBUG", "new pause signal info", "SERIAL")
                        # Vendor note (250721): ifAMS,8
                        self.G_PauseToLCDString = "+PAUSE:8,%d,%d" % (
                            self.G_ChangeChannelTimeoutOldChan,
                            self.G_ChangeChannelTimeoutNewChan,
                        )
                        self.kaos_log("DEBUG", self.G_PauseToLCDString, "SERIAL")

    # filament change
    def Cmds_P1TnManualChangeChannel(self, chan, gcmd):
        self.kaos_log("DEBUG", "=====[(cmds.python)Cmds_P1TnManualChangeChannel]", "SERIAL")

        if gcmd is None:
            self.kaos_log("DEBUG", "gcmd is None", "SERIAL")
            # self.G_PhrozenFluiddRespondInfo("return")
            # return
            # pass
        if gcmd is not None:
            self.kaos_log("DEBUG", "gcmd is not None:", "SERIAL")
            self.kaos_log(
                "DEBUG",
                "=====[(cmds.python)Cmds_P1TnManualChangeChannel]cmd='%s'"
                % (gcmd.get_commandline()),
                "SERIAL",
            )

        # # Vendor note (20231101): checkfilament,
        # if self.G_ToolheadIfHaveFilaFlag:
        #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]filament,prevent;cmd='%s'" % (gcmd.get_commandline()))
        #     #// all retract to park;//===== P2 A1  Yes;"AP";
        #     self.Cmds_AMSSerial1Send("AP")
        #     self.G_PhrozenFluiddRespondInfo("Sending command: AP, retract all to park position")

        # Vendor note (231216): filament changeloggcode
        # channel numbergcmdobject
        # self.G_ChangeChannelTimeoutOldChan=chan
        # self.G_ChangeChannelTimeoutOldGcmd=gcmd

        self.G_IfChangeFilaOngoing = True

        self.kaos_log(
            "DEBUG", "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag), "SERIAL"
        )

        # Vendor note (240229): prevent
        # time.sleep(1)
        self.G_ProzenToolhead.dwell(0.5)

        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()

        # Vendor note (240223): iffailed,functionalreadyZ
        if self.ToolheadCutFlag == True:
            self.ToolheadCutFlag = False
            self.kaos_log("DEBUG", "before cut filament error, failed", "SERIAL")
            self.G_ChangeChannelFirstFilaFlag = True
            self.G_IfChangeFilaOngoing = False

            # STM32-reported pause: allow only once,cannot
            self.STM32ReprotPauseFlag = 1
            # Vendor note (231202): P1 C?auto filament change,if1,if,continue1start
            self.G_ChangeChannelFirstFilaFlag = True

            # # Vendor note (250308): already,
            # #self.emit_protocol("+PAUSE:8,%d" % self.G_ChangeChannelTimeoutNewChan)
            # #self.emit_protocol("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
            # if len(self.G_PauseToLCDString)==0:
            #     self.emit_protocol("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
            # else:
            #     self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)

            self.G_PauseToLCDString = "+PAUSE:8,%d,%d" % (
                self.G_ChangeChannelTimeoutOldChan,
                self.G_ChangeChannelTimeoutNewChan,
            )

            if self.G_SerialPort1OpenFlag == True:
                # Vendor note (240603): preventAMS
                self.Cmds_AMSSerial1Send("AT+PAUSE")
                self.kaos_log("DEBUG", "serial port 1-AT+PAUSEpausestm32 machine", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("AT+PAUSE")
                self.kaos_log("DEBUG", "serial port 2-AT+PAUSEpausestm32 machine", "SERIAL")

            if self.G_KlipperInPausing == False:
                self.kaos_log("DEBUG", "No pause is in progress; a new pause is allowed", "SERIAL")
                # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("Quick pause enabled")
                self.G_KlipperQuickPause = True
                # klipper active pause
                self.Cmds_PhrozenKlipperPause(None)
            else:
                self.kaos_log(
                    "DEBUG", "A pause is already in progress; a new pause is not allowed", "SERIAL"
                )

            self.G_KlipperIfPaused = True

            # Vendor note (240325): filament changefailed,cannotexecute
            self.G_MCModeCanResumeFlag = False

            # for UIUX dynamic interface
            self.emit_protocol("+T:1,%d" % self.G_ChangeChannelTimeoutNewChan)

            if len(self.G_PauseToLCDString) == 0:
                self.G_PhrozenFluiddRespondInfo(
                    "+PAUSE:8,%d,%d"
                    % (
                        self.G_ChangeChannelTimeoutOldChan,
                        self.G_ChangeChannelTimeoutNewChan,
                    )
                )
            else:
                self.kaos_log("DEBUG", self.G_PauseToLCDString, "SERIAL")

            self.kaos_log("DEBUG", "return", "SERIAL")
            return

        self.kaos_log(
            "DEBUG", "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag), "SERIAL"
        )

        if self.G_ChangeChannelTimeoutNewChan in range(1, 5):
            # Vendor note (240911): AMS,T?
            # filament change
            self.Cmds_AMSSerial1Send("T%d" % self.G_ChangeChannelTimeoutNewChan)
            self.kaos_log(
                "DEBUG",
                "serial port 1Sending command: T%d" % self.G_ChangeChannelTimeoutNewChan,
                "SERIAL",
            )
            # for UIUX dynamic interface
            self.emit_protocol("+T:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        elif self.G_ChangeChannelTimeoutNewChan in range(5, 9):
            self.Cmds_AMSSerial2Send("T%d" % self.G_ChangeChannelTimeoutNewChan - 4)
            self.kaos_log(
                "DEBUG",
                "serial port 2Sending command: T%d" % self.G_ChangeChannelTimeoutNewChan - 4,
                "SERIAL",
            )
            self.emit_protocol("+T:0,%d" % self.G_ChangeChannelTimeoutNewChan - 4)

        if self.ManualCmdFlag == True:
            self.kaos_log(
                "DEBUG", "[FIRMWARE] External macro PG105; manual command, purge disabled", "SERIAL"
            )
            self.IfDoPG102Flag = True
        elif self.G_CutCheckTest == True:
            # Vendor note (240319): ,filament,prevent
            self.kaos_log(
                "DEBUG",
                "[FIRMWARE] External macro PG105; heat toolhead while AMS feeds after cut",
                "SERIAL",
            )
            self.PG102Flag = True
            self.IfDoPG102Flag = True
            self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
            command_string = """
            # PG105
            """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log("DEBUG", "command_string='%s'" % command_string, "SERIAL")
            self.PG102Flag = False
            self.kaos_log("DEBUG", "self.Flag=False", "SERIAL")
        else:
            # Vendor note (240319): ,filament,prevent
            self.kaos_log(
                "DEBUG",
                "[FIRMWARE] External macro PG105; heat toolhead while AMS feeds after cut",
                "SERIAL",
            )
            self.PG102Flag = True
            self.IfDoPG102Flag = True
            self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
            command_string = """
            # PG105
            """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log("DEBUG", "command_string='%s'" % command_string, "SERIAL")
            self.PG102Flag = False
            self.kaos_log("DEBUG", "self.Flag=False", "SERIAL")

        self.kaos_log(
            "DEBUG", "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag), "SERIAL"
        )

        # Vendor note (240328): manual commandexecute
        if self.ManualCmdFlag == True:
            self.kaos_log(
                "DEBUG", "[FIRMWARE] External macro PG110; manual command, skipped", "SERIAL"
            )
            self.IfDoPG102Flag = True
        elif self.G_CutCheckTest == True:
            # Vendor note (240319): ,filament,prevent
            self.kaos_log(
                "DEBUG",
                "[FIRMWARE] External macro PG110; Klipper purges after STM32 feed",
                "SERIAL",
            )
            self.PG102Flag = True
            self.IfDoPG102Flag = True
            self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
            command_string = """
            # PG110
            """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log("DEBUG", "command_string='%s'" % command_string, "SERIAL")
            self.PG102Flag = False
            self.kaos_log("DEBUG", "self.Flag=False", "SERIAL")
        else:
            # Vendor note (240319): ,filament,prevent
            self.kaos_log(
                "DEBUG",
                "[FIRMWARE] External macro PG110; Klipper purges after STM32 feed",
                "SERIAL",
            )
            self.PG102Flag = True
            self.IfDoPG102Flag = True
            self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
            command_string = """
            # PG110
            """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log("DEBUG", "command_string='%s'" % command_string, "SERIAL")
            self.PG102Flag = False
            self.kaos_log("DEBUG", "self.Flag=False", "SERIAL")

        self.kaos_log(
            "DEBUG", "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag), "SERIAL"
        )
        # # Vendor note (240328): manual commandexecute
        # if self.ManualCmdFlag==True:
        #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)]External macro command-PG110; manual command, skip")
        # else:
        #     # Vendor note (240319): ,filament,prevent
        #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)]External macro command-PG110; after STM32 feeds, Klipper immediately purges")
        #     self.PG102Flag=True
        #     self.G_PhrozenFluiddRespondInfo("[(dev.python)]self.Flag=True")
        #     command_string = """
        #     PG110
        #     """
        #     self.G_PhrozenGCode.run_script_from_command(command_string)
        #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)]command_string='%s'" % command_string)
        #     self.PG102Flag=False
        #     self.G_PhrozenFluiddRespondInfo("[(dev.python)]self.Flag=False")

        # # Vendor note (240226): AMSfilament,20mm
        # time.sleep(2)
        # # Vendor note (231208): E-,filament
        # command_string = """
        # G92 E0
        # G1 E0.0000 F600
        # G91
        # G1 E-50 F8000
        # """
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]2s;Efilament50mm;GCODE:command_string='%s'" % command_string)

        # Vendor note (20231013): 8
        # Vendor note (231115): printer.cfgconfigtime,pythondefaulttime
        timeout = self.G_DictChangeChannelWaitAreaParam["T"] - 8

        # # Vendor note (240125): filament changeduring,z
        # # Vendor note (231208): z+
        # command_string = """
        #     G91
        #     G1 Z%f F8000
        #     """ % (
        #     self.G_AMSFilaCutZPositionLiftingUp,
        # )
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]Z-axis raise; gcode command=%s" % command_string)

        # Vendor note (240619): # # Vendor note (240306): # # Vendor note (240110): waiting areabefore,execute,position
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]External macro command-PG101")
        # command_string = """
        #     PG101
        #     """
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel];command_string='%s'" % command_string)
        # self.IfDoPG102Flag=True

        # Vendor note (240223): because,P9,
        command_string = """
                        # G90
                        # G91
                        G1 Z%f F8000
                        """ % (
            self.G_AMSFilaCutZPositionLiftingUp,
        )
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.Lo_ThisIfZPositionLiftUpFlag = True
        self.kaos_log(
            "DEBUG", "[TOOLHEAD] Z temporary lift; command_string='%s'" % command_string, "SERIAL"
        )

        # Vendor note (240325): #self.G_ResumeProcessCheckPauseStatus=False

        # Vendor note (250519): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_SPITTING_SCRAPE")
        command_string = """
            # PRZ_SPITTING_SCRAPE
            """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.kaos_log(
            "DEBUG",
            "[SERVICE] External scrape macro; command_string='%s'" % command_string,
            "SERIAL",
        )

        self.kaos_log(
            "DEBUG", "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag), "SERIAL"
        )
        # set flaglabel
        Lo_ChangeChannelIfSuccess = False
        # Vendor note (250607): print status: 1=retracting; 2=feeding; 3=printing; 4=paused
        self.G_KlipperPrintStatus = 2
        # Vendor note (20231013): time
        # Vendor note (231114): printer.cfgconfigfilament changetime,timeout
        # loopdetect2filament
        for i in range(CHANGE_CHANNEL_WAIT_TIMEOUT):
            # self.G_XBasePosition+=2
            # self.G_YBasePosition+=2
            # Vendor note (240325): if,cancheck
            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]self.G_ResumeProcessCheckPauseStatus='%d'" % self.G_ResumeProcessCheckPauseStatus)
            if self.G_ChangeChannelResumeFlag == True:
                if self.STM32ReprotPauseFlag == 1:
                    self.kaos_log("DEBUG", "in resumestate, detected up time pause", "SERIAL")
                    if self.G_ResumeProcessCheckPauseStatus == True:
                        # Vendor note (240430): failed
                        # self.G_ResumeProcessCheckPauseStatus=False
                        self.kaos_log(
                            "DEBUG", "has time pause state up report, exitresume", "SERIAL"
                        )
                        self.G_ChangeChannelFirstFilaFlag = True
                        Lo_ChangeChannelIfSuccess = False

                        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                        self.kaos_log(
                            "DEBUG",
                            "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus,
                            "SERIAL",
                        )
                        self.kaos_log(
                            "DEBUG",
                            "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"],
                            "SERIAL",
                        )
                        # // current-Lo_PauseStatus='{'is_paused': True}'
                        if Lo_PauseStatus["is_paused"] == True:
                            self.kaos_log("DEBUG", "Already paused", "SERIAL")
                        else:
                            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")

                        break
                    # else:
                    # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]No pause status report this time, continue resume")

            else:
                # Vendor note (231202): ifSTM32,needklipper pause
                if self.STM32ReprotPauseFlag == 1:
                    self.G_ChangeChannelFirstFilaFlag = True
                    self.kaos_log("DEBUG", ", stm32 move up report pause", "SERIAL")
                    Lo_ChangeChannelIfSuccess = False

                    Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                    self.kaos_log(
                        "DEBUG",
                        "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus,
                        "SERIAL",
                    )
                    self.kaos_log(
                        "DEBUG",
                        "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"],
                        "SERIAL",
                    )
                    # // current-Lo_PauseStatus='{'is_paused': True}'
                    if Lo_PauseStatus["is_paused"] == True:
                        self.kaos_log("DEBUG", "Already paused", "SERIAL")
                    else:
                        self.kaos_log("DEBUG", "Not currently paused", "SERIAL")

                    break

            # # Vendor note (231216): # if self.G_XBasePosition==0 and self.G_YBasePosition==0:
            #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]During filament change wait, base XY at 0")
            #     command_string = """
            #         G90
            #         G1 X%.03f Y%.03f F5000
            #         """ % (
            #         150+(i%2),
            #         260+(i%2)
            #     )
            #     # Vendor note (231129): #     self.G_PhrozenGCode.run_script_from_command(command_string)
            # else:
            #     # Vendor note (231216): ,needprevent
            #     # Vendor note (231214): waiting areaX YW H,
            #     command_string = """
            #         G90
            #         G1 X%.03f Y%.03f F5000
            #         """ % (
            #         self.G_XBasePosition+(i%2),
            #         self.G_YBasePosition+(i%2)
            #     )
            #     # Vendor note (231129): #     self.G_PhrozenGCode.run_script_from_command(command_string)
            #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]During filament change wait, base XY from P9 config")
            #     #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel];command_string='%s'" % command_string)

            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]During filament change wait, using external macro")

            # Vendor note (240223): because,P9,,
            if self.Lo_ThisIfZPositionLiftUpFlag == True:
                command_string = """
                                # G90
                                # G91
                                G1 Z-%f F8000
                                """ % (
                    self.G_AMSFilaCutZPositionLiftingUp,
                )
                self.G_PhrozenGCode.run_script_from_command(command_string)
                self.Lo_ThisIfZPositionLiftUpFlag = False
                self.kaos_log(
                    "DEBUG",
                    "[TOOLHEAD] Z temporary lower; command_string='%s'" % command_string,
                    "SERIAL",
                )

            # Vendor note (20231013): 4
            # Vendor note (231115): 1s
            self.G_ProzenToolhead.dwell(1)
            # Vendor note (240125): cannotsleep,block
            # time.sleep(1)

            self.kaos_log(
                "DEBUG",
                "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag),
                "SERIAL",
            )

            self.kaos_log(
                "DEBUG",
                "[FIRMWARE] External macro PG110; Klipper purges after STM32 feed",
                "SERIAL",
            )
            command_string = """
            # PG110
            """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log("DEBUG", "command_string='%s'" % command_string, "SERIAL")

            self.kaos_log("DEBUG", "Current mode", "SERIAL")
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                self.emit_mode(0, "unkown")
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
                self.emit_mode(1, "MC")
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                self.emit_mode(2, "MA")
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                self.emit_mode(3, "RUNOUT")
            else:
                self.emit_mode(-1, "error")

            Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
            self.kaos_log(
                "DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
            )
            # // current-Lo_PauseStatus='{'is_paused': True}'
            if Lo_PauseStatus["is_paused"] == True:
                self.kaos_log("DEBUG", "Already paused", "SERIAL")
            else:
                self.kaos_log("DEBUG", "Not currently paused", "SERIAL")

            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]i=%d;T=%d" % (i,self.G_ChangeChannelTimeoutNewChan))

            # detectfilament,filament
            if self.G_ToolheadIfHaveFilaFlag:
                Lo_ChangeChannelIfSuccess = True
                break

        # # Vendor note (240125): filament changeduring,z
        # command_string = """
        #     G91
        #     G1 -Z%f F8000
        #     """ % (
        #     self.G_AMSFilaCutZPositionLiftingUp,
        # )
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]Z-axis lower; gcode command=%s" % command_string)

        # Vendor note (240318): prevent
        if self.Lo_ThisIfZPositionLiftUpFlag == True:
            command_string = """
                            # G90
                            # G91
                            G1 Z-%f F8000
                            """ % (
                self.G_AMSFilaCutZPositionLiftingUp,
            )
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.Lo_ThisIfZPositionLiftUpFlag = False
            self.kaos_log(
                "DEBUG",
                "[TOOLHEAD] Z temporary lower; command_string='%s'" % command_string,
                "SERIAL",
            )

        self.kaos_log(
            "DEBUG", "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag), "SERIAL"
        )

        # normalfilament change;filament changesuccessful
        if Lo_ChangeChannelIfSuccess:
            self.kaos_log("DEBUG", "successful: T%d" % self.G_ChangeChannelTimeoutNewChan, "SERIAL")
            self.G_IfChangeFilaOngoing = False

            # Vendor note (250424): preventAMS
            self.G_ProzenToolhead.dwell(0.5)

            # Vendor note (250619): check if AMS reconnected successfully
            self.Cmds_USBConnectErrorCheck()
            # Vendor note (250423): successful,start,AMSstart,if5,
            # self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
            # self.G_PhrozenFluiddRespondInfo("AMS start timing buffer-full time")
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("AT+EBLOCKCHECK")
                self.kaos_log("DEBUG", "serial port 1-AMSstart timingbuffer-full time", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
                self.kaos_log("DEBUG", "serial port 2-AMSstart timingbuffer-full time", "SERIAL")
            self.G_ProzenToolhead.dwell(1)

            if self.IfDoPG102Flag == True:
                self.IfDoPG102Flag = False

                self.kaos_log("DEBUG", "purgestart", "SERIAL")
                self.G_PhrozenFluiddRespondInfo(
                    "+MSG:1,0,%d,%d"
                    % (
                        self.G_ChangeChannelTimeoutOldChan,
                        self.G_ChangeChannelTimeoutNewChan,
                    )
                )

                # Vendor note (240328): manual commandexecute
                if self.ManualCmdFlag == True:
                    self.kaos_log(
                        "DEBUG",
                        "[PURGE] External macro PG102; manual command, purge disabled",
                        "SERIAL",
                    )
                    # Vendor note (250409): AMS
                    self.Cmds_CmdP114(None)
                else:
                    # self.G_PhrozenFluiddRespondInfo("External macro command-PG102")
                    # self.PG102Flag=True
                    # self.G_PhrozenFluiddRespondInfo("self.Flag=True")
                    # command_string = """
                    # PG102
                    # """
                    # self.G_PhrozenGCode.run_script_from_command(command_string)
                    # self.G_PhrozenFluiddRespondInfo("[PURGE] External purge/waste macro; command_string='%s'" % command_string)

                    # Vendor note (241031): control
                    if self.G_P10SpitNum == 0:
                        self.kaos_log("DEBUG", "[PURGE] External macro PG113", "SERIAL")
                        self.PG102Flag = True
                        self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
                        command_string = """
                        # PG114
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[PURGE] External purge/waste macro; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                    elif self.G_P10SpitNum == 1:
                        self.kaos_log("DEBUG", "[PURGE] External macro PG111", "SERIAL")
                        self.PG102Flag = True
                        self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
                        command_string = """
                        # PG114
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[PURGE] External purge/waste macro; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                    elif self.G_P10SpitNum == 2:
                        self.kaos_log("DEBUG", "[PURGE] External macro PG112", "SERIAL")
                        self.PG102Flag = True
                        self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
                        command_string = """
                        # PG113
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[PURGE] External purge/waste macro; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                    elif self.G_P10SpitNum == 3:
                        self.kaos_log("DEBUG", "[PURGE] External macro PG113", "SERIAL")
                        self.PG102Flag = True
                        self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
                        command_string = """
                        # PG113
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[PURGE] External purge/waste macro; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                    elif self.G_P10SpitNum == 4:
                        self.kaos_log("DEBUG", "[PURGE] External macro PG114", "SERIAL")
                        self.PG102Flag = True
                        self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
                        command_string = """
                        # PG114
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[PURGE] External purge/waste macro; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                    elif self.G_P10SpitNum == 5:
                        self.kaos_log("DEBUG", "[PURGE] External macro PG115", "SERIAL")
                        self.PG102Flag = True
                        self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
                        command_string = """
                        # PG115
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[PURGE] External purge/waste macro; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                    elif self.G_P10SpitNum == 6:
                        self.kaos_log("DEBUG", "[PURGE] External macro PG116", "SERIAL")
                        self.PG102Flag = True
                        self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
                        command_string = """
                        # PG116
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[PURGE] External purge/waste macro; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                    elif self.G_P10SpitNum == 7:
                        self.kaos_log("DEBUG", "[PURGE] External macro PG117", "SERIAL")
                        self.PG102Flag = True
                        self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
                        command_string = """
                        # PG117
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[PURGE] External purge/waste macro; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                    elif self.G_P10SpitNum == 8:
                        self.kaos_log("DEBUG", "[PURGE] External macro PG118", "SERIAL")
                        self.PG102Flag = True
                        self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
                        command_string = """
                        # PG118
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[PURGE] External purge/waste macro; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                    elif self.G_P10SpitNum == 9:
                        self.kaos_log("DEBUG", "[PURGE] External macro PG119", "SERIAL")
                        self.PG102Flag = True
                        self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
                        command_string = """
                        # PG119
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[PURGE] External purge/waste macro; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )

                    self.PG102Flag = False
                    self.kaos_log("DEBUG", "self.Flag=False", "SERIAL")

                self.kaos_log("DEBUG", "purgefinish", "SERIAL")
                self.G_PhrozenFluiddRespondInfo(
                    "+MSG:1,1,%d,%d"
                    % (
                        self.G_ChangeChannelTimeoutOldChan,
                        self.G_ChangeChannelTimeoutNewChan,
                    )
                )

                # Vendor note (240323): ,
                # # Vendor note (240321): complete,heated bed,preventY305position,MCU
                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]External macro command-PG105; move to bed center, prevent long resume path")
                # command_string = """
                # PG105
                # """
                # self.G_PhrozenGCode.run_script_from_command(command_string)
                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]-PG105;heated bed,prevent;command_string='%s'" % command_string)

                # for i in range(15):
                #     self.G_PhrozenFluiddRespondInfo("[(dev.python)]Purging, waiting")
                #     # Vendor note (20231013): 4
                #     # Vendor note (231115): 1s
                #     self.G_ProzenToolhead.dwell(1.0)
                #     # Vendor note (240125): cannotsleep,block
                #     #time.sleep(1)
                if self.PG102DelayPauseFlag == True:
                    self.PG102DelayPauseFlag = False

                    # Vendor note (250619): check if AMS reconnected successfully
                    self.Cmds_USBConnectErrorCheck()
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                        self.kaos_log(
                            "DEBUG", "serial port 1-AMSfinishtimingbuffer-full time", "SERIAL"
                        )
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                        self.kaos_log(
                            "DEBUG", "serial port 2-AMSfinishtimingbuffer-full time", "SERIAL"
                        )

                    self.kaos_log("DEBUG", "Quick pause enabled", "SERIAL")
                    self.G_KlipperQuickPause = True
                    self.kaos_log("DEBUG", "purge, STM32 send filament runoutpause", "SERIAL")
                    # Vendor note (231209): handling business in timer causes errors; use thread for interrupt later
                    self.kaos_log("DEBUG", "stm32 move pause up report, pause1 time", "SERIAL")

                    if self.G_KlipperInPausing == False:
                        self.kaos_log(
                            "DEBUG", "No pause is in progress; a new pause is allowed", "SERIAL"
                        )
                        # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("Quick pause enabled")
                        self.G_KlipperQuickPause = True
                        # klipper active pause
                        self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                    else:
                        self.kaos_log(
                            "DEBUG",
                            "A pause is already in progress; a new pause is not allowed",
                            "SERIAL",
                        )

                    self.G_KlipperIfPaused = True
                    # stm321,cannot
                    self.STM32ReprotPauseFlag = 1
                    # Vendor note (231202): P1 C?auto filament change,if1,if,continue1start
                    self.G_ChangeChannelFirstFilaFlag = True

                    self.G_ProzenToolhead.dwell(1.5)
                    self.G_PhrozenFluiddRespondInfo(
                        "+MSG:1,1,%d,%d"
                        % (
                            self.G_ChangeChannelTimeoutOldChan,
                            self.G_ChangeChannelTimeoutNewChan,
                        )
                    )
                    # for UIUX dynamic interface
                    self.emit_protocol("+C:1,%d" % self.G_ChangeChannelTimeoutNewChan)

                    # self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)

                    if len(self.G_PauseToLCDString) == 0:
                        self.G_PhrozenFluiddRespondInfo(
                            "+PAUSE:4,%d,%d"
                            % (
                                self.G_ChangeChannelTimeoutOldChan,
                                self.G_ChangeChannelTimeoutNewChan,
                            )
                        )
                    else:
                        self.kaos_log("DEBUG", self.G_PauseToLCDString, "SERIAL")

                    # Vendor note (250607): print status: 1=retracting; 2=feeding; 3=printing; 4=paused
                    self.G_KlipperPrintStatus = 3
                    self.G_PauseToLCDString = ""

                    self.kaos_log("DEBUG", "return", "SERIAL")
                    return
                else:
                    # Vendor note (240325): ,1
                    if self.G_PauseTriggerWhileChangeChannelFlag == True:
                        # Vendor note (231209): handling business in timer causes errors; use thread for interrupt later
                        self.kaos_log("DEBUG", "purge, STM32 send pause", "SERIAL")
                        # self.emit_protocol("+PAUSE:1,%d" % self.G_ChangeChannelTimeoutNewChan)
                        # self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                        if len(self.G_PauseToLCDString) == 0:
                            self.G_PhrozenFluiddRespondInfo(
                                "+PAUSE:4,%d,%d"
                                % (
                                    self.G_ChangeChannelTimeoutOldChan,
                                    self.G_ChangeChannelTimeoutNewChan,
                                )
                            )
                        else:
                            self.kaos_log("DEBUG", self.G_PauseToLCDString, "SERIAL")
                        # Vendor note (240325): filament changefailed,cannotexecute
                        self.G_MCModeCanResumeFlag = False
                        # Vendor note (250527): execute
                        self.G_KlipperQuickPause = False
                    else:
                        # Vendor note (240325): filament changesuccessful,execute
                        self.G_MCModeCanResumeFlag = True
                        self.kaos_log("DEBUG", "purge normal normal, enterprinting", "SERIAL")
                        # Vendor note (250527): execute
                        self.G_KlipperQuickPause = True
            else:
                # Vendor note (240325): filament changesuccessful,execute
                self.G_MCModeCanResumeFlag = True
                # Vendor note (250527): execute
                # self.G_KlipperQuickPause = True
            # Vendor note (250619): check if AMS reconnected successfully
            self.Cmds_USBConnectErrorCheck()
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                self.kaos_log("DEBUG", "serial port 1-AMSfinishtimingbuffer-full time", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                self.kaos_log("DEBUG", "serial port 2-AMSfinishtimingbuffer-full time", "SERIAL")
            self.G_ProzenToolhead.dwell(1.5)

            # # Vendor note (240318): prevent
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
            #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]Z;command_string='%s'" % command_string)

            self.G_ResumeProcessCheckPauseStatus = False
            # for UIUX dynamic interface
            self.emit_protocol("+T:1,%d" % self.G_ChangeChannelTimeoutNewChan)

            # Vendor note (250607): print status: 1=retracting; 2=feeding; 3=printing; 4=paused
            self.G_KlipperPrintStatus = 3

            self.G_PauseToLCDString = ""

            return

        self.kaos_log(
            "DEBUG", "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag), "SERIAL"
        )

        # filament changefailed
        if self.G_DictChangeChannelWaitAreaParam["A"] == 0:
            # Vendor note (250619): check if AMS reconnected successfully
            self.Cmds_USBConnectErrorCheck()

            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]filament changefailed;filament;cmd='%s', filament,klipper pause" % (gcmd.get_commandline()))
            # #// all retract to park;//===== P2 A1  Yes;"AP";
            # self.Cmds_AMSSerial1Send("AP")
            # self.G_PhrozenFluiddRespondInfo("Sending command: AP, retract all to park position")
            # Vendor note (231209): stm329
            if self.G_KlipperIfPaused == False:
                # Vendor note (240328): ifmanual command,
                if self.ManualCmdFlag == True:
                    self.kaos_log("DEBUG", "move command, klipperdo not executepause", "SERIAL")
                    # Vendor note (250409): AMS
                    self.Cmds_CmdP114(None)
                elif self.G_CutCheckTest == True:
                    self.kaos_log(
                        "DEBUG", "cutter Test command, klipperdo not executepause", "SERIAL"
                    )
                    # Vendor note (250409): AMS
                    self.Cmds_CmdP114(None)
                else:
                    if self.G_KlipperInPausing == False:
                        self.kaos_log(
                            "DEBUG", "No pause is in progress; a new pause is allowed", "SERIAL"
                        )
                        self.STM32ReprotPauseFlag = 1
                        # Vendor note (231202): P1 C?auto filament change,if1,if,continue1start
                        self.G_ChangeChannelFirstFilaFlag = True

                        self.kaos_log("DEBUG", "timeout60s, pause", "SERIAL")
                        # self.emit_protocol("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))

                        if self.G_SerialPort1OpenFlag == True:
                            # Vendor note (240603): preventAMS
                            self.Cmds_AMSSerial1Send("AT+PAUSE")
                            self.kaos_log(
                                "DEBUG", "serial port 1-AT+PAUSEpausestm32 machine", "SERIAL"
                            )
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+PAUSE")
                            self.kaos_log(
                                "DEBUG", "serial port 2-AT+PAUSEpausestm32 machine", "SERIAL"
                            )

                        self.kaos_log("DEBUG", "Quick pause enabled", "SERIAL")
                        self.G_KlipperQuickPause = True
                        # klipper active pause
                        self.Cmds_PhrozenKlipperPause(None)
                        self.G_KlipperIfPaused = True

                        if len(self.G_PauseToLCDString) == 0:
                            self.G_PhrozenFluiddRespondInfo(
                                "+PAUSE:4,%d,%d"
                                % (
                                    self.G_ChangeChannelTimeoutOldChan,
                                    self.G_ChangeChannelTimeoutNewChan,
                                )
                            )
                        else:
                            self.kaos_log("DEBUG", self.G_PauseToLCDString, "SERIAL")

                        # Vendor note (240325): filament changefailed,cannotexecute
                        self.G_MCModeCanResumeFlag = False
                        # Vendor note (250527): execute
                        self.G_KlipperQuickPause = False
                    else:
                        self.kaos_log(
                            "DEBUG",
                            "A pause is already in progress; a new pause is not allowed",
                            "SERIAL",
                        )
            # Vendor note (240124): cannot
            else:
                self.kaos_log("DEBUG", "already pause, do notpause", "SERIAL")
                # Vendor note (240509): # # Vendor note (240326): # #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                # if len(self.G_PauseToLCDString)==0:
                #     self.emit_protocol("+PAUSE:4,%d" % self.G_ChangeChannelTimeoutNewChan)
                # Vendor note (240417): preventstm32G_PauseToLCDString
                # self.emit_protocol("+PAUSE:4,%d" % self.G_ChangeChannelTimeoutNewChan)
                # Vendor note (240325): filament changefailed,cannotexecute
                self.G_MCModeCanResumeFlag = False
                # Vendor note (250527): execute
                self.G_KlipperQuickPause = False

                # Vendor note (240429): ifstm32,need
                if self.G_ResumeProcessCheckPauseStatus == False:
                    self.kaos_log(
                        "DEBUG",
                        "AMS has up report pause, klipper pause, need to up report pause",
                        "SERIAL",
                    )
                    if len(self.G_PauseToLCDString) == 0:
                        self.G_PhrozenFluiddRespondInfo(
                            "+PAUSE:4,%d,%d"
                            % (
                                self.G_ChangeChannelTimeoutOldChan,
                                self.G_ChangeChannelTimeoutNewChan,
                            )
                        )
                    else:
                        self.kaos_log("DEBUG", self.G_PauseToLCDString, "SERIAL")
                    if self.G_SerialPort1OpenFlag == True:
                        # Vendor note (240603): preventAMS
                        self.Cmds_AMSSerial1Send("AT+PAUSE")
                        self.kaos_log("DEBUG", "serial port 1-AT+PAUSEpausestm32 machine", "SERIAL")
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AT+PAUSE")
                        self.kaos_log("DEBUG", "serial port 2-AT+PAUSEpausestm32 machine", "SERIAL")
                else:  # True
                    self.kaos_log(
                        "DEBUG",
                        "AMS has up report pause, klipper need to need to up report pause",
                        "SERIAL",
                    )
                    self.G_ResumeProcessCheckPauseStatus = False

                    if len(self.G_PauseToLCDString) == 0:
                        self.G_PhrozenFluiddRespondInfo(
                            "+PAUSE:4,%d,%d"
                            % (
                                self.G_ChangeChannelTimeoutOldChan,
                                self.G_ChangeChannelTimeoutNewChan,
                            )
                        )
                    else:
                        self.kaos_log("DEBUG", self.G_PauseToLCDString, "SERIAL")

                # self.G_PhrozenFluiddRespondInfo("Already paused, pause once more to prevent prior pause anomaly")
                # # Vendor note (250423): prevent,1
                # #klipper active pause
                # self.Cmds_PhrozenKlipperPause(None)

            # Vendor note (231202): P1 C?auto filament change,if1,if,continue1start
            self.G_ChangeChannelFirstFilaFlag = True

            self.G_IfChangeFilaOngoing = False

            self.G_ResumeProcessCheckPauseStatus = False
            # for UIUX dynamic interface
            self.emit_protocol("+T:1,%d" % self.G_ChangeChannelTimeoutNewChan)

            # Vendor note (250607): print status: 1=retracting; 2=feeding; 3=printing; 4=paused
            self.G_KlipperPrintStatus = -1

            return

        # normalfilament change;Actionnormal
        if self.G_DictChangeChannelWaitAreaParam["A"] == 1:
            pass

    # P1 C[n] n:1~32(device,1~4) (,, , )
    def Cmds_P1CnAutoChangeChannel(self, chan, gcmd):
        # Vendor note (241030): P1 C1P1 C32,132
        # 1:1 2 3 4
        # 2:5 6 7 8
        # 3:9 10 11 12
        # 4:13 14 15 16
        # 5:17 18 19 20
        # 6:21 22 23 24
        # 7:25 26 27 28
        # 8:29 30 31 32
        self.kaos_log("DEBUG", "=====[(cmds.python)Cmds_P1CnAutoChangeChannel]", "SERIAL")
        self.kaos_log(
            "DEBUG",
            "===== up time self.G_ChangeChannelTimeoutOldChan=%d"
            % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "===== up time self.G_ChangeChannelTimeoutNewChan=%d"
            % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )
        if gcmd is None:
            self.kaos_log("DEBUG", "gcmd is None", "SERIAL")
            # self.G_PhrozenFluiddRespondInfo("return")
            # return
            # pass
        if gcmd is not None:
            self.kaos_log("DEBUG", "gcmd is not None:", "SERIAL")
            self.kaos_log(
                "DEBUG",
                "===== up time '%s';self.G_ChangeChannelTimeoutOldChan=%d"
                % (gcmd.get_commandline(), self.G_ChangeChannelTimeoutOldChan),
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "===== up time '%s';self.G_ChangeChannelTimeoutNewChan=%d"
                % (gcmd.get_commandline(), self.G_ChangeChannelTimeoutNewChan),
                "SERIAL",
            )

        self.kaos_log(
            "DEBUG", "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag), "SERIAL"
        )

        # Vendor note (250824): self.G_PhrozenFluiddRespondInfo("self.G_ProzenToolhead.wait_moves()")
        self.G_ProzenToolhead.wait_moves()

        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")

        self.kaos_log("DEBUG", "Current mode", "SERIAL")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.emit_mode(0, "unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.emit_mode(1, "MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.emit_mode(2, "MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.emit_mode(3, "RUNOUT")
        else:
            self.emit_mode(-1, "error")

        # unlock
        # [Translated vendor note] self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]filament change, XY0")
        # command_string = """
        # G90
        # G1 X%.03f Y%.03f F5000
        # """ % (
        # 150+(i%2),
        # 260+(i%2)
        # )
        # [Translated vendor note] #Vendor note (231129):
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # else:
        # [Translated vendor note] #Vendor note (231216): ,
        # [Translated vendor note] #Vendor note (231214): X YW H, purge
        # command_string = """
        # G90
        # G1 X%.03f Y%.03f F5000
        # """ % (
        # self.G_XBasePosition+(i%2),
        # self.G_YBasePosition+(i%2)
        # )
        # [Translated vendor note] #Vendor note (231129):
        # self.G_PhrozenGCode.run_script_from_command(command_string)
        # [Translated vendor note] self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]filament change, XYP9")
        # [Translated vendor note] #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1TnManualChangeChannel]; command_string='%s'" % command_string)

        self.Base_AMSSerialCmdUnlock()

        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.kaos_log(
                "DEBUG",
                "ifreceived T?command, but mode is Unknown mode, switch is MCmulti-material mode",
                "SERIAL",
            )
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC

        self.kaos_log("DEBUG", "Current mode", "SERIAL")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.emit_mode(0, "unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.emit_mode(1, "MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.emit_mode(2, "MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.emit_mode(3, "RUNOUT")
        else:
            self.emit_mode(-1, "error")

        # Vendor note (250526): ,allowgcode,complete
        if self.G_KlipperInPausing == True:
            self.kaos_log(
                "DEBUG", "pause, not allowednew gcode command, need pause complete", "SERIAL"
            )
            for num in range(30):
                # Vendor note (231115): 1s
                self.kaos_log("DEBUG", "self.G_ProzenToolhead.dwell(1)", "SERIAL")
                self.G_ProzenToolhead.dwell(1)
                self.kaos_log(
                    "DEBUG", "pause, not allowednew gcode command, need pause complete", "SERIAL"
                )
                Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                self.kaos_log(
                    "DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL"
                )
                self.kaos_log(
                    "DEBUG",
                    "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"],
                    "SERIAL",
                )

                # // current-Lo_PauseStatus='{'is_paused': True}'
                if Lo_PauseStatus["is_paused"] == True:
                    self.kaos_log("DEBUG", "Already paused", "SERIAL")
                else:
                    self.kaos_log("DEBUG", "Not currently paused", "SERIAL")

                if self.G_KlipperInPausing == False:
                    self.kaos_log("DEBUG", "pausefinish", "SERIAL")
                    Lo_ChangeChannelIfSuccess = True

                    Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                    self.kaos_log(
                        "DEBUG",
                        "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus,
                        "SERIAL",
                    )
                    self.kaos_log(
                        "DEBUG",
                        "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"],
                        "SERIAL",
                    )
                    # // current-Lo_PauseStatus='{'is_paused': True}'
                    if Lo_PauseStatus["is_paused"] == True:
                        self.kaos_log("DEBUG", "Already paused", "SERIAL")
                    else:
                        self.kaos_log("DEBUG", "Not currently paused", "SERIAL")
                        # klipper pause;currentx y zcoordinates
                        # Vendor note (240108): datanormal,validate
                        self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE")
                        self.kaos_log("DEBUG", "[(cmds.python)]PAUSE", "SERIAL")
                        self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                        self.G_ProzenToolhead.wait_moves()
                        self.G_ProzenToolhead.dwell(1.0)

                    self.kaos_log("DEBUG", "break", "SERIAL")
                    break

            # Vendor note (250725): ifloopend,execute,execute
            if self.G_KlipperInPausing == True:
                self.kaos_log(
                    "DEBUG", "=====pause, received new command, but pause complete, pause", "SERIAL"
                )
                Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                self.kaos_log(
                    "DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL"
                )
                self.kaos_log(
                    "DEBUG",
                    "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"],
                    "SERIAL",
                )
                # // current-Lo_PauseStatus='{'is_paused': True}'
                if Lo_PauseStatus["is_paused"] == True:
                    self.kaos_log("DEBUG", "Already paused", "SERIAL")
                else:
                    self.kaos_log("DEBUG", "=====Not currently paused, pause action", "SERIAL")
                    # klipper pause;currentx y zcoordinates
                    # Vendor note (240108): datanormal,validate
                    self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE")
                    self.kaos_log("DEBUG", "[(cmds.python)]PAUSE", "SERIAL")
                    self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                    self.G_ProzenToolhead.wait_moves()
                    self.G_ProzenToolhead.dwell(1.0)

        else:
            self.kaos_log("DEBUG", "in pause state", "SERIAL")
            self.kaos_log("DEBUG", "self.G_KlipperInPausing == False", "SERIAL")

        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")

        # Vendor note (250512): 1,preventbeforesuccessful
        if self.G_KlipperIfPaused == True:
            #
            if self.G_ChangeChannelResumeFlag == False:
                self.kaos_log("DEBUG", "is resumestate", "SERIAL")
                self.kaos_log("DEBUG", "klipper pause, but received command", "SERIAL")
                # Vendor note (250508): prevent
                self.kaos_log(
                    "DEBUG", "klipper pause, but received command, then time pause", "SERIAL"
                )
                self.G_PhrozenFluiddRespondInfo(
                    "+PAUSE:4,%d,%d"
                    % (
                        self.G_ChangeChannelTimeoutOldChan,
                        self.G_ChangeChannelTimeoutNewChan,
                    )
                )
                self.G_PauseToLCDString = "+PAUSE:4,%d,%d" % (
                    self.G_ChangeChannelTimeoutOldChan,
                    self.G_ChangeChannelTimeoutNewChan,
                )
                # self.Cmds_PhrozenKlipperPause(None)

                Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                self.kaos_log(
                    "DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL"
                )
                self.kaos_log(
                    "DEBUG",
                    "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"],
                    "SERIAL",
                )
                # // current-Lo_PauseStatus='{'is_paused': True}'
                if Lo_PauseStatus["is_paused"] == True:
                    self.kaos_log("DEBUG", "Already paused", "SERIAL")
                else:
                    self.kaos_log("DEBUG", "Not currently paused", "SERIAL")
                    # klipper pause;currentx y zcoordinates
                    # Vendor note (240108): datanormal,validate
                    self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE")
                    self.kaos_log("DEBUG", "[(cmds.python)]PAUSE", "SERIAL")
                    self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                    self.G_ProzenToolhead.wait_moves()
                    self.G_ProzenToolhead.dwell(1.0)

                # Vendor note (250524): self.G_PhrozenFluiddRespondInfo("pause, received new gcode command, need to new new old, prevent stop")
                self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                self.kaos_log(
                    "DEBUG",
                    "=====self.G_ChangeChannelTimeoutOldChan=%d"
                    % self.G_ChangeChannelTimeoutOldChan,
                    "SERIAL",
                )
                self.G_ChangeChannelTimeoutOldGcmd = self.G_ChangeChannelTimeoutNewGcmd
                self.G_ChangeChannelTimeoutNewChan = chan
                self.kaos_log(
                    "DEBUG",
                    "=====self.G_ChangeChannelTimeoutNewChan=%d"
                    % self.G_ChangeChannelTimeoutNewChan,
                    "SERIAL",
                )
                self.G_ChangeChannelTimeoutNewGcmd = gcmd
                self.Cmds_PrintMode(self.G_AMSDeviceWorkMode)

                Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                self.kaos_log(
                    "DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL"
                )
                self.kaos_log(
                    "DEBUG",
                    "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"],
                    "SERIAL",
                )
                # // current-Lo_PauseStatus='{'is_paused': True}'
                if Lo_PauseStatus["is_paused"] == True:
                    self.kaos_log("DEBUG", "Already paused", "SERIAL")
                else:
                    self.kaos_log("DEBUG", "Not currently paused", "SERIAL")

                self.kaos_log("DEBUG", "return", "SERIAL")
                return

        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()
        if self.G_SerialPort1Obj is not None:
            if self.G_SerialPort1Obj.is_open:
                self.kaos_log("DEBUG", "Serial port 1 is open", "SERIAL")
                # self.G_SerialPort1Obj.flushInput()
                # self.G_PhrozenFluiddRespondInfo("G_SerialPort1Obj.flushInput: flush serial buffer")
        if self.G_SerialPort2Obj is not None:
            if self.G_SerialPort2Obj.is_open:
                self.kaos_log("DEBUG", "Serial port 2 is open", "SERIAL")

        self.kaos_log(
            "DEBUG", "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag), "SERIAL"
        )

        # Vendor note (240226): AMSfilament,20mm
        # time.sleep(2)
        # self.G_ProzenToolhead.dwell(2.0)

        self.G_PauseTriggerWhileChangeChannelFlag = False
        self.emit_protocol("+C:0,%d" % chan)

        self.G_ASM1DisconnectErrorCount = 0

        # # Vendor note (240322): filament change,ifG?already,return
        # if self.STM32ReprotPauseFlag==1:
        #     self.G_PhrozenFluiddRespondInfo("self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)
        #     self.G_PhrozenFluiddRespondInfo("self.G_Pause1Channel=%d" % self.G_Pause1Channel)
        #     if self.G_PauseTriggerWhileChangeChannelFlag==True:
        #         self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]G? command detected pause report, continue pausing")
        #         # Vendor note (240325): for continuous screen tap after runout without removal, use pause=1
        #         #self.emit_protocol("+PAUSE:1,%d" % self.G_Pause1Channel)
        #         if "+PAUSE:1" in self.G_PauseToLCDString:
        #             self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
        #         #else:
        #             #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
        #         self.G_ChangeChannelFirstFilaFlag=True
        #         self.G_IfChangeFilaOngoing= False
        #         # Vendor note (240524): for UIUX dynamic interface
        #         self.emit_protocol("+C:1,%d" % self.G_ChangeChannelTimeoutNewChan)
        #         return
        #     else:
        #         self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]G? command no pause report, need feed and resume")
        #         self.G_ChangeChannelFirstFilaFlag=True
        #         self.G_IfChangeFilaOngoing= False
        #         # Vendor note (240325): no runout, continue feeding
        #         #return

        self.G_IfChangeFilaOngoing = True

        # Vendor note (250102): filament changecalculate
        self.G_PrintCountNum = self.G_PrintCountNum + 1
        self.kaos_log("DEBUG", "time number =%d" % self.G_PrintCountNum, "SERIAL")

        # filament change1filament
        if self.G_ChangeChannelFirstFilaFlag:
            # # Vendor note (240314): position
            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]External macro command-PG104")
            # command_string = """
            # PG104
            # """
            # self.G_PhrozenGCode.run_script_from_command(command_string)
            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]-1position;command_string='%s'" % command_string)

            # Vendor note (240125): self.G_PhrozenFluiddRespondInfo("first time #1;pause/resume #1")

            # # Vendor note (240124): STM32 active report, allow one pause
            # self.STM32ReprotPauseFlag=0
            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]self.STM32ReprotPauseFlag=0")

            # Vendor note (231202): if1klipperfalse,1filament change
            self.G_ChangeChannelFirstFilaFlag = False

            # ,need
            if self.G_ChangeChannelResumeFlag == False:
                self.kaos_log("DEBUG", "first layer printing, is pause/resume", "SERIAL")
                self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                self.kaos_log(
                    "DEBUG",
                    "===== time self.G_ChangeChannelTimeoutOldChan=%d"
                    % self.G_ChangeChannelTimeoutOldChan,
                    "SERIAL",
                )
                self.G_ChangeChannelTimeoutOldGcmd = self.G_ChangeChannelTimeoutNewGcmd
                self.G_ChangeChannelTimeoutNewChan = chan
                self.kaos_log(
                    "DEBUG",
                    "===== time self.G_ChangeChannelTimeoutNewChan=%d"
                    % self.G_ChangeChannelTimeoutNewChan,
                    "SERIAL",
                )
                self.G_ChangeChannelTimeoutNewGcmd = gcmd
                self.Cmds_PrintMode(self.G_AMSDeviceWorkMode)

                # Vendor note (250619): check if AMS reconnected successfully
                self.Cmds_USBConnectErrorCheck()
                if self.G_ChangeChannelTimeoutOldChan in range(1, 5):  # 1 2 3 4
                    # # Vendor note (241011): filament changebeforeAMSexecutedistance,executePG101;
                    self.emit_channel_op("H", 0, self.G_ChangeChannelTimeoutOldChan)
                    self.Cmds_AMSSerial1Send("H%d" % self.G_ChangeChannelTimeoutOldChan)
                    self.kaos_log(
                        "DEBUG",
                        "serial port 1Sending command: H%d" % self.G_ChangeChannelTimeoutOldChan,
                        "SERIAL",
                    )
                    self.kaos_log("DEBUG", "serial port 1 before AMSretract", "SERIAL")
                    self.emit_channel_op("H", 1, self.G_ChangeChannelTimeoutOldChan)
                elif self.G_ChangeChannelTimeoutOldChan in range(5, 9):  # 5 6 7 8
                    self.G_PhrozenFluiddRespondInfo(
                        "+H:0,%d" % self.G_ChangeChannelTimeoutOldChan - 4
                    )
                    self.Cmds_AMSSerial2Send("H%d" % self.G_ChangeChannelTimeoutOldChan - 4)
                    self.kaos_log(
                        "DEBUG",
                        "serial port 2Sending command: H%d" % self.G_ChangeChannelTimeoutOldChan
                        - 4,
                        "SERIAL",
                    )
                    self.kaos_log("DEBUG", "serial port 2 before AMSretract", "SERIAL")
                    self.G_PhrozenFluiddRespondInfo(
                        "+H:1,%d" % self.G_ChangeChannelTimeoutOldChan - 4
                    )
                else:
                    self.kaos_log("DEBUG", "error, all filament", "SERIAL")
                    if self.G_SerialPort1OpenFlag == True:
                        # Vendor note (240913): ,,candistance,prevent,
                        self.Cmds_AMSSerial1Send("AP")
                        self.kaos_log(
                            "DEBUG", "serial port 1Sending command: AP; all filament", "SERIAL"
                        )
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AP")
                        self.kaos_log(
                            "DEBUG", "serial port 2Sending command: AP; all filament", "SERIAL"
                        )

                    # Vendor note (240913): self.G_ProzenToolhead.dwell(6)

                # # Vendor note (241011): PG101beforeG?,executePG101,executeMCG?
                # self.Cmds_AMSSerial1Send("MC")
                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]Sending command: MC")
                # self.G_PhrozenFluiddRespondInfo("Force interrupt AMS retract distance")

                self.kaos_log(
                    "DEBUG",
                    "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag),
                    "SERIAL",
                )

                self.kaos_log(
                    "DEBUG",
                    "[FIRMWARE] External macro PG104; capture variables before toolchange",
                    "SERIAL",
                )
                command_string = """
                    # PG104
                    """
                self.G_PhrozenGCode.run_script_from_command(command_string)
                self.kaos_log(
                    "DEBUG",
                    "[FIRMWARE] External macro PG104; command_string='%s'" % command_string,
                    "SERIAL",
                )
                self.IfDoPG102Flag = True

                # Vendor note (240510): before,feed waiting zone
                # Vendor note (240306): # Vendor note (240110): waiting areabefore,execute,position
                # Vendor note (240515): before,feed waiting zone
                self.kaos_log(
                    "DEBUG", "[TOOLCHANGE] External macro PG101 retract/pre-cut", "SERIAL"
                )
                command_string = """
                    # PG101
                    """
                self.G_PhrozenGCode.run_script_from_command(command_string)
                self.kaos_log(
                    "DEBUG",
                    "[SERVICE] Move to waiting area for purge; command_string='%s'"
                    % command_string,
                    "SERIAL",
                )
                self.IfDoPG102Flag = True

                if self.G_ToolheadIfHaveFilaFlag == True:
                    self.kaos_log("DEBUG", "toolhead has filament", "SERIAL")

                    # Vendor note (240909): PG106before
                    # for i in range(15):
                    #     self.G_PhrozenFluiddRespondInfo("[(dev.python)]Purge in progress complete")
                    #     # Vendor note (20231013): 4
                    #     # Vendor note (231115): 1s
                    #     self.G_ProzenToolhead.dwell(1.0)
                    #     # Vendor note (240125): cannotsleep,block
                    #     #time.sleep(1)
                    # Vendor note (240319): before
                    # self.Cmds_MoveToCutFilaPrepare()
                    # Vendor note (20231205): self.Cmds_MoveToCutFilaAction(gcmd)

                    # Vendor note (250519): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_CUT_WAITINGAREA")
                    command_string = """
                        # PRZ_CUT_WAITINGAREA
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.kaos_log(
                        "DEBUG",
                        "[SERVICE] Move to service/waiting position; command_string='%s'"
                        % command_string,
                        "SERIAL",
                    )

                    # Vendor note (240226): AMSfilament,20mm
                    # time.sleep(2)
                    self.G_ProzenToolhead.dwell(0.5)

                    # Vendor note (250619): check if AMS reconnected successfully
                    self.Cmds_USBConnectErrorCheck()
                    if self.G_ChangeChannelTimeoutOldChan in range(1, 5):
                        # Vendor note (240906): AMS,,lastdistance
                        # Vendor note (20231013): stm32filament change
                        # Vendor note (231129): stm32filament changeklipperfilament change,stm32filament change,klipperif,klipper
                        self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutOldChan)
                        self.kaos_log("DEBUG", "G%d" % self.G_ChangeChannelTimeoutOldChan, "SERIAL")
                        self.kaos_log(
                            "DEBUG",
                            "serial port 1-AMS old first : G%d"
                            % self.G_ChangeChannelTimeoutOldChan,
                            "SERIAL",
                        )
                    elif self.G_ChangeChannelTimeoutOldChan in range(5, 9):
                        self.Cmds_AMSSerial2Send("G%d" % self.G_ChangeChannelTimeoutOldChan - 4)
                        self.kaos_log(
                            "DEBUG", "G%d" % self.G_ChangeChannelTimeoutOldChan - 4, "SERIAL"
                        )
                        self.kaos_log(
                            "DEBUG",
                            "serial port 2-AMS old first : G%d" % self.G_ChangeChannelTimeoutOldChan
                            - 4,
                            "SERIAL",
                        )

                    self.kaos_log(
                        "DEBUG",
                        "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag),
                        "SERIAL",
                    )

                    # channel numbergcmdobject
                    # self.G_ChangeChannelTimeoutOldChan=chan
                    # self.G_ChangeChannelTimeoutOldGcmd=gcmd

                    self.G_ProzenToolhead.dwell(0.5)

                    # Vendor note (240913): self.G_ProzenToolhead.dwell(6.5)
                    # Vendor note (240911): Gafter6checkfilament
                    # Vendor note (231201): checkfilamentnormal,normal
                    self.Cmds_CutFilaIfNormalCheck()
                    if self.G_KlipperIfPaused == True:
                        self.kaos_log(
                            "DEBUG",
                            "cut filament ?toolhead detectedfilament, cutter error, please check cutter, pauseklipperprinting",
                            "SERIAL",
                        )
                        # Lo_ChangeChannelIfSuccess = False
                        return
                # else:
                #     self.G_PhrozenFluiddRespondInfo("External macro command-PRZ_WAITINGAREA-waiting area")
                #     command_string = """
                #         PRZ_WAITINGAREA
                #         """
                #     self.G_PhrozenGCode.run_script_from_command(command_string)
                #     self.G_PhrozenFluiddRespondInfo("External macro command-PRZ_WAITINGAREA; command_string='%s'" % command_string)

                self.kaos_log(
                    "DEBUG",
                    "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag),
                    "SERIAL",
                )

                # Vendor note (240328): manual commandexecute
                if self.ManualCmdFlag == True:
                    self.kaos_log(
                        "DEBUG",
                        "[TOOLCHANGE] External macro PG106; manual command, purge disabled",
                        "SERIAL",
                    )
                else:
                    # Vendor note (240319): ,filament,prevent
                    self.kaos_log(
                        "DEBUG",
                        "[TOOLCHANGE] External macro PG106; heat toolhead while AMS retracts after cut",
                        "SERIAL",
                    )
                    self.PG102Flag = True
                    self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
                    command_string = """
                    # PG106
                    """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.kaos_log("DEBUG", "command_string='%s'" % command_string, "SERIAL")
                    self.PG102Flag = False
                    self.kaos_log("DEBUG", "self.Flag=False", "SERIAL")

                # Vendor note (231216): iffilament changeduring,filament changeduringz,execute,z,
                # Vendor note (231216): ifz,need
                if self.G_IfZPositionLiftUpFlag == True:
                    command_string = """
                        # G90
                        # G91
                        G1 Z-%f F8000
                        """ % (
                        self.G_AMSFilaCutZPositionLiftingUp,
                    )
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.G_IfZPositionLiftUpFlag = False
                    self.kaos_log(
                        "DEBUG",
                        "[TOOLHEAD] Z lowered; command_string='%s'" % command_string,
                        "SERIAL",
                    )

                self.G_ProzenToolhead.dwell(0.5)

                # Vendor note (20231013): filament change
                self.Cmds_P1TnManualChangeChannel(
                    self.G_ChangeChannelTimeoutNewChan,
                    self.G_ChangeChannelTimeoutNewGcmd,
                )
            # Vendor note (240912): ,log
            else:
                self.kaos_log("DEBUG", "is first layer printing, is pause/resume", "SERIAL")
                # Vendor note (20231013): filament change
                self.Cmds_P1TnManualChangeChannel(
                    self.G_ChangeChannelTimeoutNewChan,
                    self.G_ChangeChannelTimeoutNewGcmd,
                )

        # nfilament
        else:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_P1CnAutoChangeChannel] after #n; else", "SERIAL"
            )
            self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
            self.kaos_log(
                "DEBUG",
                "===== time self.G_ChangeChannelTimeoutOldChan=%d"
                % self.G_ChangeChannelTimeoutOldChan,
                "SERIAL",
            )
            self.G_ChangeChannelTimeoutOldGcmd = self.G_ChangeChannelTimeoutNewGcmd
            self.G_ChangeChannelTimeoutNewChan = chan
            self.kaos_log(
                "DEBUG",
                "===== time self.G_ChangeChannelTimeoutNewChan=%d"
                % self.G_ChangeChannelTimeoutNewChan,
                "SERIAL",
            )
            self.G_ChangeChannelTimeoutNewGcmd = gcmd
            self.Cmds_PrintMode(self.G_AMSDeviceWorkMode)

            # Vendor note (240124): STM32 active report, allow one pause
            self.STM32ReprotPauseFlag = 0
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)Cmds_P1CnAutoChangeChannel]self.STM32ReprotPauseFlag=0",
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag),
                "SERIAL",
            )

            # Vendor note (250619): check if AMS reconnected successfully
            self.Cmds_USBConnectErrorCheck()
            if self.G_ChangeChannelTimeoutOldChan in range(1, 5):
                # # Vendor note (241011): filament changebeforeAMSexecutedistance,executePG101;
                self.emit_channel_op("H", 0, self.G_ChangeChannelTimeoutOldChan)
                self.Cmds_AMSSerial1Send("H%d" % self.G_ChangeChannelTimeoutOldChan)
                self.kaos_log(
                    "DEBUG",
                    "serial port 1Sending command: H%d" % self.G_ChangeChannelTimeoutOldChan,
                    "SERIAL",
                )
                self.kaos_log("DEBUG", "serial port 1 before AMSretract", "SERIAL")
                self.emit_channel_op("H", 1, self.G_ChangeChannelTimeoutOldChan)
            elif self.G_ChangeChannelTimeoutOldChan in range(5, 9):
                self.emit_channel_op("H", 0, self.G_ChangeChannelTimeoutOldChan - 4)
                self.Cmds_AMSSerial2Send("H%d" % (self.G_ChangeChannelTimeoutOldChan - 4))
                self.kaos_log(
                    "DEBUG",
                    "serial port 2Sending command: H%d" % (self.G_ChangeChannelTimeoutOldChan - 4),
                    "SERIAL",
                )
                self.kaos_log("DEBUG", "serial port 2 before AMSretract", "SERIAL")
                self.emit_channel_op("H", 1, self.G_ChangeChannelTimeoutOldChan - 4)
            else:
                self.kaos_log("DEBUG", "error, all filament", "SERIAL")
                if self.G_SerialPort1OpenFlag == True:
                    # Vendor note (240913): ,,candistance,prevent,
                    self.Cmds_AMSSerial1Send("AP")
                    self.kaos_log(
                        "DEBUG", "serial port 1Sending command: AP; all filament", "SERIAL"
                    )
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("AP")
                    self.kaos_log(
                        "DEBUG", "serial port 2Sending command: AP; all filament", "SERIAL"
                    )

                # Vendor note (240913): self.G_ProzenToolhead.dwell(6)

            # Vendor note (250824): self.G_PhrozenFluiddRespondInfo("self.G_ProzenToolhead.wait_moves()")
            self.G_ProzenToolhead.wait_moves()

            self.kaos_log(
                "DEBUG",
                "[FIRMWARE] External macro PG104; capture variables before toolchange",
                "SERIAL",
            )
            command_string = """
                # PG104
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG",
                "[FIRMWARE] External macro PG104; command_string='%s'" % command_string,
                "SERIAL",
            )
            self.IfDoPG102Flag = True

            # Vendor note (240510): before,feed waiting zone
            # Vendor note (240306): # Vendor note (240110): waiting areabefore,execute,position
            # Vendor note (240515): before,feed waiting zone
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)Cmds_P1CnAutoChangeChannel]External macro command-PG101-retract",
                "SERIAL",
            )
            command_string = """
                # PG101
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)Cmds_P1CnAutoChangeChannel]External macro command-to waiting areapositionpurge; command_string='%s'"
                % command_string,
                "SERIAL",
            )
            self.IfDoPG102Flag = True

            self.kaos_log(
                "DEBUG",
                "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag),
                "SERIAL",
            )

            # Vendor note (250824): self.G_PhrozenFluiddRespondInfo("self.G_ProzenToolhead.wait_moves()")
            self.G_ProzenToolhead.wait_moves()

            # Vendor note (250323): #if self.G_ToolheadIfHaveFilaFlag==True:
            # self.G_PhrozenFluiddRespondInfo("toolhead has filament")
            # Vendor note (20231013): ,ZX Yposition
            self.Cmds_MoveToCutFilaAction(gcmd)
            # else:
            #    self.G_PhrozenFluiddRespondInfo("External macro command-PRZ_WAITINGAREA-waiting area")
            #    command_string = """
            #        PRZ_WAITINGAREA
            #        """
            #    self.G_PhrozenGCode.run_script_from_command(command_string)
            #    self.G_PhrozenFluiddRespondInfo("External macro command-PRZ_WAITINGAREA; command_string='%s'" % command_string)

            # Vendor note (250824): self.G_PhrozenFluiddRespondInfo("self.G_ProzenToolhead.wait_moves()")
            self.G_ProzenToolhead.wait_moves()

            # Vendor note (250519): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_CUT_WAITINGAREA")
            command_string = """
                # PRZ_CUT_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG",
                "[SERVICE] Move to service/waiting position; command_string='%s'" % command_string,
                "SERIAL",
            )

            self.G_ProzenToolhead.dwell(0.5)

            # Vendor note (250824): self.G_PhrozenFluiddRespondInfo("self.G_ProzenToolhead.wait_moves()")
            self.G_ProzenToolhead.wait_moves()

            self.kaos_log(
                "DEBUG",
                "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag),
                "SERIAL",
            )

            # Vendor note (250619): check if AMS reconnected successfully
            self.Cmds_USBConnectErrorCheck()
            if self.G_ChangeChannelTimeoutOldChan in range(1, 5):
                # Vendor note (240906): ,lastdistance
                # Vendor note (20231013): stm32filament change
                # Vendor note (231129): stm32filament changeklipperfilament change,stm32filament change,klipperif,klipper
                self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutOldChan)
                self.kaos_log("DEBUG", "G%d" % self.G_ChangeChannelTimeoutOldChan, "SERIAL")
                self.kaos_log(
                    "DEBUG",
                    "serial port 1-AMS old first : G%d" % self.G_ChangeChannelTimeoutOldChan,
                    "SERIAL",
                )
            elif self.G_ChangeChannelTimeoutOldChan in range(5, 9):
                self.Cmds_AMSSerial2Send("G%d" % self.G_ChangeChannelTimeoutOldChan - 4)
                self.kaos_log("DEBUG", "G%d" % self.G_ChangeChannelTimeoutOldChan - 4, "SERIAL")
                self.kaos_log(
                    "DEBUG",
                    "serial port 2-AMS old first : G%d" % self.G_ChangeChannelTimeoutOldChan - 4,
                    "SERIAL",
                )

            # Vendor note (250824): self.G_PhrozenFluiddRespondInfo("self.G_ProzenToolhead.wait_moves()")
            self.G_ProzenToolhead.wait_moves()

            # Vendor note (250322): PG106,,PG106
            # Vendor note (240913): self.G_ProzenToolhead.dwell(6.5)
            # Vendor note (250823): self.G_PhrozenFluiddRespondInfo("self.G_ProzenToolhead.wait_moves()")
            self.G_ProzenToolhead.wait_moves()
            # Vendor note (231201): checknormal,normal
            # Vendor note (231215): Z
            # Vendor note (231216): 6checksuccessful
            self.Cmds_CutFilaIfNormalCheck()
            if self.G_KlipperIfPaused == True:
                self.kaos_log(
                    "DEBUG",
                    "[(cmds.python)Cmds_P1CnAutoChangeChannel]cut filament ?toolhead detectedfilament, cutter error, please check cutter, pauseklipperprinting",
                    "SERIAL",
                )
                # self.Cmds_PhrozenKlipperPause(None)

                Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                self.kaos_log(
                    "DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL"
                )
                self.kaos_log(
                    "DEBUG",
                    "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"],
                    "SERIAL",
                )
                # // current-Lo_PauseStatus='{'is_paused': True}'
                if Lo_PauseStatus["is_paused"] == True:
                    self.kaos_log("DEBUG", "Already paused", "SERIAL")
                else:
                    self.kaos_log("DEBUG", "Not currently paused", "SERIAL")
                    # klipper pause;currentx y zcoordinates
                    # Vendor note (240108): datanormal,validate
                    self.G_PhrozenGCode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE")
                    self.kaos_log("DEBUG", "[(cmds.python)]PAUSE", "SERIAL")
                    self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
                    self.G_ProzenToolhead.wait_moves()
                    self.G_ProzenToolhead.dwell(1.0)

                    self.kaos_log("DEBUG", "toolhead cutter or device error, pause", "SERIAL")
                    # self.emit_protocol("+PAUSE:8,%d" % self.G_ChangeChannelTimeoutOldChan)
                    # Vendor note (250414): #self.emit_protocol("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    if len(self.G_PauseToLCDString) == 0:
                        self.G_PhrozenFluiddRespondInfo(
                            "+PAUSE:8,%d,%d"
                            % (
                                self.G_ChangeChannelTimeoutOldChan,
                                self.G_ChangeChannelTimeoutNewChan,
                            )
                        )
                    else:
                        self.kaos_log("DEBUG", self.G_PauseToLCDString, "SERIAL")
                # # Vendor note (250524): # self.G_PhrozenFluiddRespondInfo("pause, received new gcode command, need to new new old, prevent stop")
                # self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
                # self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan)
                # self.G_ChangeChannelTimeoutOldGcmd=self.G_ChangeChannelTimeoutNewGcmd
                # self.G_ChangeChannelTimeoutNewChan=chan
                # self.G_PhrozenFluiddRespondInfo("=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)
                # self.G_ChangeChannelTimeoutNewGcmd=gcmd

                Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                self.kaos_log(
                    "DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL"
                )
                self.kaos_log(
                    "DEBUG",
                    "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"],
                    "SERIAL",
                )
                # // current-Lo_PauseStatus='{'is_paused': True}'
                if Lo_PauseStatus["is_paused"] == True:
                    self.kaos_log("DEBUG", "Already paused", "SERIAL")
                else:
                    self.kaos_log("DEBUG", "Not currently paused", "SERIAL")

                self.kaos_log("DEBUG", "return", "SERIAL")
                return

            # Vendor note (240229): prevent
            # time.sleep(1)
            self.G_ProzenToolhead.dwell(0.5)

            self.kaos_log(
                "DEBUG",
                "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag),
                "SERIAL",
            )

            # Vendor note (240319): ,filament,prevent
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)]External macro command-PG106; cut filament after, toolheadheat upwhen AMS",
                "SERIAL",
            )
            self.PG102Flag = True
            self.kaos_log("DEBUG", "[(dev.python)]self.Flag=True", "SERIAL")
            command_string = """
            # PG106
            """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log("DEBUG", "[(cmds.python)]command_string='%s'" % command_string, "SERIAL")
            self.PG102Flag = False
            self.kaos_log("DEBUG", "[(dev.python)]self.Flag=False", "SERIAL")

            # Vendor note (250619): check if AMS reconnected successfully
            self.Cmds_USBConnectErrorCheck()
            if self.G_ChangeChannelTimeoutNewChan in range(1, 5):
                # Vendor note (240911): AMS,T
                # Vendor note (20231013): stm32filament change
                # Vendor note (231129): stm32filament changeklipperfilament change,stm32filament change,klipperif,klipper
                self.Cmds_AMSSerial1Send("T%d" % self.G_ChangeChannelTimeoutNewChan)
                self.kaos_log(
                    "DEBUG",
                    "serial port 1Sending command: T%d" % self.G_ChangeChannelTimeoutNewChan,
                    "SERIAL",
                )
            elif self.G_ChangeChannelTimeoutNewChan in range(5, 9):
                self.Cmds_AMSSerial2Send("T%d" % self.G_ChangeChannelTimeoutNewChan - 4)
                self.kaos_log(
                    "DEBUG",
                    "serial port 2Sending command: T%d" % self.G_ChangeChannelTimeoutNewChan - 4,
                    "SERIAL",
                )

            # Vendor note (240322): self.G_PhrozenFluiddRespondInfo("[(cmds.python)]External macro command-PG105; cut filament after, toolheadheat upwhen AMS")
            self.PG102Flag = True
            self.kaos_log("DEBUG", "[(dev.python)]self.Flag=True", "SERIAL")
            command_string = """
            # PG105
            """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log("DEBUG", "[(cmds.python)]command_string='%s'" % command_string, "SERIAL")
            self.PG102Flag = False
            self.kaos_log("DEBUG", "[(dev.python)]self.Flag=False", "SERIAL")

            self.kaos_log(
                "DEBUG",
                "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag),
                "SERIAL",
            )

            # Vendor note (240328): manual commandexecute
            if self.ManualCmdFlag == True:
                self.kaos_log(
                    "DEBUG", "[FIRMWARE] External macro PG110; manual command, skipped", "SERIAL"
                )
            else:
                # Vendor note (240319): ,filament,prevent
                self.kaos_log(
                    "DEBUG",
                    "[FIRMWARE] External macro PG110; Klipper purges after STM32 feed",
                    "SERIAL",
                )
                self.PG102Flag = True
                self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
                command_string = """
                # PG110
                """
                self.G_PhrozenGCode.run_script_from_command(command_string)
                self.kaos_log("DEBUG", "command_string='%s'" % command_string, "SERIAL")
                self.PG102Flag = False
                self.kaos_log("DEBUG", "self.Flag=False", "SERIAL")

            # Vendor note (240229): z,feed waiting zone
            if self.G_IfZPositionLiftUpFlag == True:
                command_string = """
                    # G90
                    # G91
                    G1 Z-%f F8000
                    """ % (
                    self.G_AMSFilaCutZPositionLiftingUp,
                )
                self.G_PhrozenGCode.run_script_from_command(command_string)
                self.G_IfZPositionLiftUpFlag = False
                self.kaos_log(
                    "DEBUG", "[TOOLHEAD] Z lowered; command_string='%s'" % command_string, "SERIAL"
                )

            # Vendor note (240223): iffailed,functionalreadyZ,alreadyexecute
            if self.ToolheadCutFlag == True:
                self.ToolheadCutFlag = False
                self.kaos_log("DEBUG", "before cut filament error, failed", "SERIAL")
                self.G_ChangeChannelFirstFilaFlag = True
                self.G_IfChangeFilaOngoing = False

                # STM32-reported pause: allow only once,cannot
                self.STM32ReprotPauseFlag = 1
                # Vendor note (231202): P1 C?auto filament change,if1,if,continue1start
                self.G_ChangeChannelFirstFilaFlag = True

                # Vendor note (250308): already,
                # self.emit_protocol("+PAUSE:8,%d" % self.G_ChangeChannelTimeoutNewChan)
                # self.emit_protocol("+PAUSE:8,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))

                if self.G_SerialPort1OpenFlag == True:
                    # Vendor note (240603): preventAMS
                    self.Cmds_AMSSerial1Send("AT+PAUSE")
                    self.kaos_log("DEBUG", "serial port 1-AT+PAUSEpausestm32 machine", "SERIAL")
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("AT+PAUSE")
                    self.kaos_log("DEBUG", "serial port 2-AT+PAUSEpausestm32 machine", "SERIAL")

                if self.G_KlipperInPausing == False:
                    self.kaos_log(
                        "DEBUG", "No pause is in progress; a new pause is allowed", "SERIAL"
                    )
                    # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("Quick pause enabled")
                    self.G_KlipperQuickPause = True
                    # klipper active pause
                    self.Cmds_PhrozenKlipperPause(None)
                else:
                    self.kaos_log(
                        "DEBUG",
                        "A pause is already in progress; a new pause is not allowed",
                        "SERIAL",
                    )

                self.G_KlipperIfPaused = True

                # Vendor note (240325): filament changefailed,cannotexecute
                self.G_MCModeCanResumeFlag = False

                if len(self.G_PauseToLCDString) == 0:
                    self.G_PhrozenFluiddRespondInfo(
                        "+PAUSE:8,%d,%d"
                        % (
                            self.G_ChangeChannelTimeoutOldChan,
                            self.G_ChangeChannelTimeoutNewChan,
                        )
                    )
                else:
                    self.kaos_log("DEBUG", self.G_PauseToLCDString, "SERIAL")

                # for UIUX dynamic interface
                self.emit_protocol("+C:1,%d" % self.G_ChangeChannelTimeoutNewChan)

                self.kaos_log("DEBUG", "return", "SERIAL")
                return

            self.kaos_log(
                "DEBUG",
                "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag),
                "SERIAL",
            )

            # # Vendor note (231208): z-
            # # Vendor note (231213): F7200300
            # # Vendor note (231215): Z
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
            # # Vendor note (231216): iffilament changeduring,filament changeduringz,execute,z,
            # self.G_IfZPositionLiftUpFlag = False
            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]gcodex y z;command_string='%s'" % command_string)

            # set flaglabel
            Lo_ChangeChannelIfSuccess = False
            self.kaos_log("DEBUG", "Lo_ChangeChannelIfSuccess = False", "SERIAL")

            # Vendor note (231202): ifP9movepatharray,lenfunctionklipper
            # 0
            # Vendor note (231206): UIif,P9,array,codedump
            if self.ChangeWaitMoveArea is None:
                self.kaos_log("DEBUG", "waiting area move move error;klipper pause", "SERIAL")
                Lo_ChangeChannelIfSuccess = False
                pass

            if self.ChangeWaitMoveArea is not None:
                # list
                if len(self.ChangeWaitMoveArea) == 0:
                    self.kaos_log(
                        "DEBUG",
                        "return;waiting area move move error, path;if len(self.ChangeWaitMoveArea) == 0",
                        "SERIAL",
                    )
                    # Vendor note (231206): execute
                    # return
                else:
                    self.kaos_log(
                        "DEBUG",
                        "for;waiting area move move normal normal, pathqueue, new filamentto toolhead",
                        "SERIAL",
                    )

                # # Vendor note (240306): # # Vendor note (240110): waiting areabefore,execute,position
                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]External macro command-PG101")
                # command_string = """
                #     PG101
                #     """
                # self.G_PhrozenGCode.run_script_from_command(command_string)
                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]-position;command_string='%s'" % command_string)
                # self.IfDoPG102Flag=True

                # Vendor note (250519): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_SPITTING_SCRAPE")
                command_string = """
                    # PRZ_SPITTING_SCRAPE
                    """
                self.G_PhrozenGCode.run_script_from_command(command_string)
                self.kaos_log(
                    "DEBUG",
                    "[SERVICE] External scrape macro; command_string='%s'" % command_string,
                    "SERIAL",
                )

                # Vendor note (250607): print status: 1=retracting; 2=feeding; 3=printing; 4=paused
                self.G_KlipperPrintStatus = 2
                # Python enumerate() function
                # enumerate() functiondataobject(list)index,datadata, for loop
                # Python 2.3. ,2.6  start parameter
                # for loop enumerate
                # >>> seq = ['one', 'two', 'three']
                # >>> for i, element in enumerate(seq):
                # ...     print i, element
                # ...
                # 0 one
                # 1 two
                # 2 three
                # waiting area,80;
                # for i in range(CHANGE_CHANNEL_WAIT_TIMEOUT):#120
                # for num, point in enumerate(self.ChangeWaitMoveArea):
                for num in range(CHANGE_CHANNEL_WAIT_TIMEOUT):
                    # Vendor note (231202): ifSTM32,needklipper pause
                    if self.STM32ReprotPauseFlag == 1:
                        # Lo_ChangeChannelIfSuccess = False
                        # break
                        # Vendor note (231205): ifduringstm32,exit,
                        self.kaos_log("DEBUG", ", stm32 move up report pause", "SERIAL")

                        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                        self.kaos_log(
                            "DEBUG",
                            "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus,
                            "SERIAL",
                        )
                        self.kaos_log(
                            "DEBUG",
                            "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"],
                            "SERIAL",
                        )
                        # // current-Lo_PauseStatus='{'is_paused': True}'
                        if Lo_PauseStatus["is_paused"] == True:
                            self.kaos_log("DEBUG", "Already paused", "SERIAL")
                        else:
                            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")
                        Lo_ChangeChannelIfSuccess = False
                        break

                    # # Vendor note (231214): waiting areaX YW H,
                    # command_string = """
                    #     G90
                    #     G1 X%.03f Y%.03f F%d
                    #     """ % (
                    #     point[0]+(num%2),#Xcoordinates;lancaigang231215:waiting areaxcoordinatesmm,preventnormal
                    #     point[1]+(num%2),#Ycoordinates
                    #     int(self.G_WaitAreaEachStepDist / self.G_MovementSpeedFactor),#
                    #     #500
                    # )
                    # # Vendor note (231129): # self.G_PhrozenGCode.run_script_from_command(command_string)
                    # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]During filament change wait, base XY from P9 config")

                    # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]During filament change wait, using external macro")

                    # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]num='%d'" % num)
                    # Vendor note (20231014): position,time,1
                    self.G_ProzenToolhead.wait_moves()
                    # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]self.G_ProzenToolhead.wait_moves()")

                    # Vendor note (231219): dwell
                    # Vendor note (231209): #time.sleep(2)
                    # Vendor note (231115): 1s
                    self.G_ProzenToolhead.dwell(1)

                    self.kaos_log(
                        "DEBUG",
                        "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag),
                        "SERIAL",
                    )

                    self.kaos_log(
                        "DEBUG",
                        "[FIRMWARE] External macro PG110; Klipper purges after STM32 feed",
                        "SERIAL",
                    )
                    command_string = """
                    # PG110
                    """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.kaos_log("DEBUG", "command_string='%s'" % command_string, "SERIAL")

                    self.kaos_log("DEBUG", "Current mode", "SERIAL")
                    if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                        self.emit_mode(0, "unkown")
                    elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
                        self.emit_mode(1, "MC")
                    elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                        self.emit_mode(2, "MA")
                    elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                        self.emit_mode(3, "RUNOUT")
                    else:
                        self.emit_mode(-1, "error")

                    Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                    self.kaos_log(
                        "DEBUG",
                        "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus,
                        "SERIAL",
                    )
                    self.kaos_log(
                        "DEBUG",
                        "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"],
                        "SERIAL",
                    )
                    # // current-Lo_PauseStatus='{'is_paused': True}'
                    if Lo_PauseStatus["is_paused"] == True:
                        self.kaos_log("DEBUG", "Already paused", "SERIAL")
                    else:
                        self.kaos_log("DEBUG", "Not currently paused", "SERIAL")

                    # Vendor note (250111): forloop,prevent

                    # Vendor note (240125): cannotsleep,block
                    # time.sleep(1)

                    # # Vendor note (231129): ifdetectfilament,,needklipper
                    # if num == 3 and point[2] and self.G_ToolheadIfHaveFilaFlag:
                    #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]5s after cut, toolhead still detects filament, cutter error, check cutter, pause Klipper")
                    #     Lo_ChangeChannelIfSuccess = False
                    #     break
                    # elif num > 3:
                    #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]Cut successful, waiting for new filament")

                    # 10allowdetectdetectfilament
                    # Vendor note (20231013): 108
                    # Vendor note (231129): ifsuccessful,stm32continueexecutefilament change,,ifdetectfilament,continueklipperfilament
                    # Vendor note (231129): detectfilament,normalfilament change,5,stm32filament,detectfilament,30detectfilament
                    if num > 1 and self.G_ToolheadIfHaveFilaFlag:
                        self.kaos_log(
                            "DEBUG", "has new filament, successful, canprinting", "SERIAL"
                        )
                        Lo_ChangeChannelIfSuccess = True
                        break

            self.kaos_log(
                "DEBUG",
                "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag),
                "SERIAL",
            )

            # iftruesuccessful,return
            if Lo_ChangeChannelIfSuccess:
                self.kaos_log("DEBUG", "successful;", "SERIAL")
                self.kaos_log("DEBUG", "successful", "SERIAL")
                self.G_IfChangeFilaOngoing = False

                # Vendor note (250424): preventAMS
                self.G_ProzenToolhead.dwell(0.5)

                # Vendor note (250619): check if AMS reconnected successfully
                self.Cmds_USBConnectErrorCheck()
                # Vendor note (250423): successful,start,AMSstart,if5,
                # self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
                # self.G_PhrozenFluiddRespondInfo("AMS start timing buffer-full time")
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("AT+EBLOCKCHECK")
                    self.kaos_log(
                        "DEBUG", "serial port 1-AMSstart timingbuffer-full time", "SERIAL"
                    )
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
                    self.kaos_log(
                        "DEBUG", "serial port 2-AMSstart timingbuffer-full time", "SERIAL"
                    )
                self.G_ProzenToolhead.dwell(1)

                if self.IfDoPG102Flag == True:
                    self.IfDoPG102Flag = False

                    self.kaos_log("DEBUG", "purgestart", "SERIAL")
                    self.G_PhrozenFluiddRespondInfo(
                        "+MSG:1,0,%d,%d"
                        % (
                            self.G_ChangeChannelTimeoutOldChan,
                            self.G_ChangeChannelTimeoutNewChan,
                        )
                    )

                    # Vendor note (241031): control
                    # Vendor note (250324): defaultPG113,3
                    if self.G_P10SpitNum == 0:
                        self.kaos_log("DEBUG", "[PURGE] External macro PG113", "SERIAL")
                        self.PG102Flag = True
                        self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
                        command_string = """
                        # PG113
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[PURGE] External purge/waste macro; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                    elif self.G_P10SpitNum == 1:
                        self.kaos_log("DEBUG", "[PURGE] External macro PG111", "SERIAL")
                        self.PG102Flag = True
                        self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
                        command_string = """
                        # PG111
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[PURGE] External purge/waste macro; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                    elif self.G_P10SpitNum == 2:
                        self.kaos_log("DEBUG", "[PURGE] External macro PG112", "SERIAL")
                        self.PG102Flag = True
                        self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
                        command_string = """
                        # PG112
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[PURGE] External purge/waste macro; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                    elif self.G_P10SpitNum == 3:
                        self.kaos_log("DEBUG", "[PURGE] External macro PG113", "SERIAL")
                        self.PG102Flag = True
                        self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
                        command_string = """
                        # PG113
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[PURGE] External purge/waste macro; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                    elif self.G_P10SpitNum == 4:
                        self.kaos_log("DEBUG", "[PURGE] External macro PG114", "SERIAL")
                        self.PG102Flag = True
                        self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
                        command_string = """
                        # PG114
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[PURGE] External purge/waste macro; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                    elif self.G_P10SpitNum == 5:
                        self.kaos_log("DEBUG", "[PURGE] External macro PG115", "SERIAL")
                        self.PG102Flag = True
                        self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
                        command_string = """
                        # PG115
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[PURGE] External purge/waste macro; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                    elif self.G_P10SpitNum == 6:
                        self.kaos_log("DEBUG", "[PURGE] External macro PG116", "SERIAL")
                        self.PG102Flag = True
                        self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
                        command_string = """
                        # PG116
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[PURGE] External purge/waste macro; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                    elif self.G_P10SpitNum == 7:
                        self.kaos_log("DEBUG", "[PURGE] External macro PG117", "SERIAL")
                        self.PG102Flag = True
                        self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
                        command_string = """
                        # PG117
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[PURGE] External purge/waste macro; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                    elif self.G_P10SpitNum == 8:
                        self.kaos_log("DEBUG", "[PURGE] External macro PG118", "SERIAL")
                        self.PG102Flag = True
                        self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
                        command_string = """
                        # PG118
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[PURGE] External purge/waste macro; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )
                    elif self.G_P10SpitNum == 9:
                        self.kaos_log("DEBUG", "[PURGE] External macro PG119", "SERIAL")
                        self.PG102Flag = True
                        self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
                        command_string = """
                        # PG119
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log(
                            "DEBUG",
                            "[PURGE] External purge/waste macro; command_string='%s'"
                            % command_string,
                            "SERIAL",
                        )

                    self.PG102Flag = False
                    self.kaos_log("DEBUG", "self.Flag=False", "SERIAL")

                    self.kaos_log("DEBUG", "purgefinish", "SERIAL")

                    # for i in range(15):
                    #     self.G_PhrozenFluiddRespondInfo("[(dev.python)]Purging, waiting")
                    #     # Vendor note (20231013): 4
                    #     # Vendor note (231115): 1s
                    #     self.G_ProzenToolhead.dwell(1.0)
                    #     # Vendor note (240125): cannotsleep,block
                    #     #time.sleep(1)
                    if self.PG102DelayPauseFlag == True:
                        self.PG102DelayPauseFlag = False

                        # Vendor note (250619): check if AMS reconnected successfully
                        self.Cmds_USBConnectErrorCheck()
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                            self.kaos_log(
                                "DEBUG", "serial port 1-AMSfinishtimingbuffer-full time", "SERIAL"
                            )
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                            self.kaos_log(
                                "DEBUG", "serial port 2-AMSfinishtimingbuffer-full time", "SERIAL"
                            )

                        self.kaos_log("DEBUG", "Quick pause enabled", "SERIAL")
                        self.G_KlipperQuickPause = True
                        self.kaos_log("DEBUG", "purge, STM32 send filament runoutpause", "SERIAL")
                        # Vendor note (231209): handling business in timer causes errors; use thread for interrupt later

                        if self.G_KlipperInPausing == False:
                            self.kaos_log(
                                "DEBUG", "No pause is in progress; a new pause is allowed", "SERIAL"
                            )
                            # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("Quick pause enabled")
                            self.G_KlipperQuickPause = True
                            # klipper active pause
                            self.kaos_log(
                                "DEBUG", "stm32 move pause up report, pause1 time", "SERIAL"
                            )
                            self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                        else:
                            self.kaos_log(
                                "DEBUG",
                                "A pause is already in progress; a new pause is not allowed",
                                "SERIAL",
                            )

                        self.G_KlipperIfPaused = True
                        # stm321,cannot
                        self.STM32ReprotPauseFlag = 1
                        # Vendor note (231202): P1 C?auto filament change,if1,if,continue1start
                        self.G_ChangeChannelFirstFilaFlag = True

                        self.G_ProzenToolhead.dwell(1.5)
                        self.G_PhrozenFluiddRespondInfo(
                            "+MSG:1,1,%d,%d"
                            % (
                                self.G_ChangeChannelTimeoutOldChan,
                                self.G_ChangeChannelTimeoutNewChan,
                            )
                        )
                        # for UIUX dynamic interface
                        self.G_PhrozenFluiddRespondInfo(
                            "+C:1,%d" % self.G_ChangeChannelTimeoutNewChan
                        )

                        # self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                        if len(self.G_PauseToLCDString) == 0:
                            self.G_PhrozenFluiddRespondInfo(
                                "+PAUSE:4,%d,%d"
                                % (
                                    self.G_ChangeChannelTimeoutOldChan,
                                    self.G_ChangeChannelTimeoutNewChan,
                                )
                            )
                        else:
                            self.kaos_log("DEBUG", self.G_PauseToLCDString, "SERIAL")

                        # Vendor note (250607): print status: 1=retracting; 2=feeding; 3=printing; 4=paused
                        self.G_KlipperPrintStatus = 3
                        self.G_PauseToLCDString = ""

                        self.kaos_log("DEBUG", "return", "SERIAL")
                        return

                        # Vendor note (240326): during,1
                        # self.emit_protocol("+PAUSE:1,%d" % self.G_ChangeChannelTimeoutNewChan)
                    else:
                        self.kaos_log("DEBUG", "purge normal normal, enterprinting", "SERIAL")
                        # Vendor note (250527): filament changesuccessfulgocde
                        # Vendor note (250527): execute
                        self.G_KlipperQuickPause = True

                self.G_PhrozenFluiddRespondInfo(
                    "+MSG:1,1,%d,%d"
                    % (
                        self.G_ChangeChannelTimeoutOldChan,
                        self.G_ChangeChannelTimeoutNewChan,
                    )
                )

                # Vendor note (250619): check if AMS reconnected successfully
                self.Cmds_USBConnectErrorCheck()
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                    self.kaos_log(
                        "DEBUG", "serial port 1-AMSfinishtimingbuffer-full time", "SERIAL"
                    )
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                    self.kaos_log(
                        "DEBUG", "serial port 2-AMSfinishtimingbuffer-full time", "SERIAL"
                    )
                self.G_ProzenToolhead.dwell(1.5)

                # for UIUX dynamic interface
                self.emit_protocol("+C:1,%d" % self.G_ChangeChannelTimeoutNewChan)

                # Vendor note (250607): print status: 1=retracting; 2=feeding; 3=printing; 4=paused
                self.G_KlipperPrintStatus = 3

                self.G_PauseToLCDString = ""

                self.kaos_log("DEBUG", "normal normal enterprinting", "SERIAL")

                return

            self.kaos_log("DEBUG", "failed", "SERIAL")
            self.kaos_log(
                "DEBUG",
                "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag),
                "SERIAL",
            )
            # filament changefailed
            if self.G_DictChangeChannelWaitAreaParam["A"] == 0:
                self.kaos_log(
                    "DEBUG",
                    "failed; filamentfeedtimeout; cmd='%s', allfilament, klipper pause",
                    "SERIAL",
                )
                self.kaos_log(
                    "DEBUG",
                    "failed; current command='%s';klipper pause"
                    % (self.G_ChangeChannelTimeoutOldGcmd.get_commandline()),
                    "SERIAL",
                )

                # Vendor note (250527): execute
                self.G_KlipperQuickPause = False

                # Vendor note (250619): check if AMS reconnected successfully
                self.Cmds_USBConnectErrorCheck()

                # # Vendor note (231129): klipper pause,z=10;x=150;y=10
                # command_string = """
                # G91
                # G1 z10 F600
                # G90
                # G1 X150 F600
                # G1 Y10 F600
                # """
                # self.G_PhrozenGCode.run_script_from_command(command_string)

                # Vendor note (231201): klipper pausestm32cannot
                # gcmd.respond_info("Sending command: AP, retract all to park position")
                # #// all retract to park;//===== P2 A1  Yes;"AP";
                # self.Cmds_AMSSerial1Send("AP")
                # logging.info("SendCmd: AP")
                if self.G_KlipperIfPaused == False:
                    if self.G_KlipperInPausing == False:
                        self.kaos_log(
                            "DEBUG", "No pause is in progress; a new pause is allowed", "SERIAL"
                        )
                        self.STM32ReprotPauseFlag = 1
                        # Vendor note (231202): P1 C?auto filament change,if1,if,continue1start
                        self.G_ChangeChannelFirstFilaFlag = True

                        if self.G_SerialPort1OpenFlag == True:
                            # Vendor note (240603): preventAMS
                            self.Cmds_AMSSerial1Send("AT+PAUSE")
                            self.kaos_log(
                                "DEBUG", "serial port 1-AT+PAUSEpausestm32 machine", "SERIAL"
                            )
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+PAUSE")
                            self.kaos_log(
                                "DEBUG", "serial port 2-AT+PAUSEpausestm32 machine", "SERIAL"
                            )

                        self.kaos_log("DEBUG", "Quick pause enabled", "SERIAL")
                        self.G_KlipperQuickPause = True
                        # klipper active pause
                        self.Cmds_PhrozenKlipperPause(None)
                        self.G_KlipperIfPaused = True
                        # self.emit_protocol("+PAUSE:4,%d" % self.G_ChangeChannelTimeoutNewChan)

                        self.kaos_log("DEBUG", "timeout60s, pause", "SERIAL")
                        # self.emit_protocol("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        # self.emit_protocol("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        if len(self.G_PauseToLCDString) == 0:
                            self.G_PhrozenFluiddRespondInfo(
                                "+PAUSE:4,%d,%d"
                                % (
                                    self.G_ChangeChannelTimeoutOldChan,
                                    self.G_ChangeChannelTimeoutNewChan,
                                )
                            )
                        else:
                            self.kaos_log("DEBUG", self.G_PauseToLCDString, "SERIAL")

                    else:
                        self.kaos_log(
                            "DEBUG",
                            "A pause is already in progress; a new pause is not allowed",
                            "SERIAL",
                        )
                # Vendor note (240124): cannot
                else:
                    self.kaos_log("DEBUG", "already pause, do notpause", "SERIAL")
                    # # Vendor note (250529): # if len(self.G_PauseToLCDString)==0:
                    #     self.emit_protocol("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    # else:
                    #     self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)

                    # self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                    # Vendor note (240417): preventstm32G_PauseToLCDString
                    # if len(self.G_PauseToLCDString)==0:
                    #    self.emit_protocol("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    if len(self.G_PauseToLCDString) == 0:
                        self.G_PhrozenFluiddRespondInfo(
                            "+PAUSE:4,%d,%d"
                            % (
                                self.G_ChangeChannelTimeoutOldChan,
                                self.G_ChangeChannelTimeoutNewChan,
                            )
                        )
                    else:
                        self.kaos_log("DEBUG", self.G_PauseToLCDString, "SERIAL")
                    if self.G_SerialPort1OpenFlag == True:
                        # Vendor note (240603): preventAMS
                        self.Cmds_AMSSerial1Send("AT+PAUSE")
                        self.kaos_log("DEBUG", "serial port 1-AT+PAUSEpausestm32 machine", "SERIAL")
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AT+PAUSE")
                        self.kaos_log("DEBUG", "serial port 2-AT+PAUSEpausestm32 machine", "SERIAL")
                    # Vendor note (240429): ifstm32,need
                    # if self.G_ResumeProcessCheckPauseStatus==False:
                    #     self.emit_protocol("+PAUSE:4,%d" % self.G_ChangeChannelTimeoutNewChan)

                    if self.G_ResumeProcessCheckPauseStatus == False:
                        self.kaos_log(
                            "DEBUG",
                            "AMS has up report pause, klipper pause, need to up report pause",
                            "SERIAL",
                        )
                        # self.emit_protocol("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        if len(self.G_PauseToLCDString) == 0:
                            self.G_PhrozenFluiddRespondInfo(
                                "+PAUSE:4,%d,%d"
                                % (
                                    self.G_ChangeChannelTimeoutOldChan,
                                    self.G_ChangeChannelTimeoutNewChan,
                                )
                            )
                        else:
                            self.kaos_log("DEBUG", self.G_PauseToLCDString, "SERIAL")

                        if self.G_SerialPort1OpenFlag == True:
                            # Vendor note (240603): preventAMS
                            self.Cmds_AMSSerial1Send("AT+PAUSE")
                            self.kaos_log(
                                "DEBUG", "serial port 1-AT+PAUSEpausestm32 machine", "SERIAL"
                            )
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+PAUSE")
                            self.kaos_log(
                                "DEBUG", "serial port 2-AT+PAUSEpausestm32 machine", "SERIAL"
                            )
                    else:
                        self.kaos_log(
                            "DEBUG",
                            "AMS has up report pause, klipper need to need to up report pause",
                            "SERIAL",
                        )
                        self.G_ResumeProcessCheckPauseStatus = False

                    # self.G_PhrozenFluiddRespondInfo("Already paused, pause once more to prevent prior pause anomaly")
                    # # Vendor note (250423): prevent,1
                    # #klipper active pause
                    # self.Cmds_PhrozenKlipperPause(None)

                # Vendor note (231207): P1 C?auto filament change,if,continue1start
                self.G_ChangeChannelFirstFilaFlag = True
                self.G_IfChangeFilaOngoing = False

                # for UIUX dynamic interface
                self.emit_protocol("+C:1,%d" % self.G_ChangeChannelTimeoutNewChan)

                # Vendor note (250607): print status: 1=retracting; 2=feeding; 3=printing; 4=paused
                self.G_KlipperPrintStatus = -1

                return

            # Vendor note (20231013): Actionnormalfilament change=1
            if self.G_DictChangeChannelWaitAreaParam["A"] == 1:
                pass

    def Cmds_CmdOrcaPre(self):
        self.kaos_log(
            "DEBUG", "=====[(cmds.python)Cmds_CmdOrcaPre]orca before set move action", "SERIAL"
        )

        # Vendor note (250912): ;coordinates,triggerGPIO,control

        # Vendor note (250515): ,T?
        if self.G_P0M1MCNoneAMS == 1:
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)Cmds_CmdOrcaPre]standalone multi-material, logical T?",
                "SERIAL",
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            # self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro; command_string='%s'" % command_string)

            return

        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdOrcaPre]single-color mode, logical T?", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            # self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro; command_string='%s'" % command_string)
            return

        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)Cmds_CmdOrcaPre]single-color refill mode, logical T?",
                "SERIAL",
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            # self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro; command_string='%s'" % command_string)
            return

        # Vendor note (240527): ,becausemanual command,defaultSTM32MC
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.kaos_log(
                "DEBUG",
                "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "ifreceived T?command, but mode is Unknown mode, switch is MCmulti-material mode",
                "SERIAL",
            )
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")

            self.G_ProzenToolhead.dwell(2)

    def Cmds_CmdT0(self, gcmd):
        self.kaos_log("DEBUG", "=====[(cmds.python)Cmds_CmdT0 +1]orcaAMS", "SERIAL")

        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        # Vendor note (250515): ,T?
        if self.G_P0M1MCNoneAMS == 1:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT1]standalone multi-material, logical T?", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT1]single-color mode, logical T0", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT1]single-color refill mode, logical T0", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return

        # Vendor note (240527): ,becausemanual command,defaultSTM32MC
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.kaos_log(
                "DEBUG",
                "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "ifreceived T?command, but mode is Unknown mode, switch is MCmulti-material mode",
                "SERIAL",
            )
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")

            self.G_ProzenToolhead.dwell(2)

        # Vendor note (250912): #self.Cmds_CmdOrcaPre()

        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.kaos_log("DEBUG", "Automatic filament change", "SERIAL")
        # for UIUX dynamic interface
        self.emit_protocol("+T0:0,%d" % self.G_ChangeChannelTimeoutNewChan)

        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")
        # Vendor note (240113): manual commandflag
        self.ManualCmdFlag = False
        # Vendor note (240221): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0
        self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")

        # Vendor note (241030): P1 C1P1 C32,132
        # 1:1 2 3 4
        # 2:5 6 7 8
        # 3:9 10 11 12
        # 4:13 14 15 16
        # 5:17 18 19 20
        # 6:21 22 23 24
        # 7:25 26 27 28
        # 8:29 30 31 32
        # auto filament change
        chan = 0 + 1
        self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")

        # Vendor note (250515): checkconfigcolor
        if self.G_ChromaKitAccessT0 > 0:
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT0=%d" % self.G_ChromaKitAccessT0, "SERIAL"
            )
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT0, gcmd)
        else:
            self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # for UIUX dynamic interface
        self.emit_protocol("+T0:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    def Cmds_CmdT1(self, gcmd):
        self.kaos_log("DEBUG", "=====[(cmds.python)Cmds_CmdT1 +1]orcaAMS", "SERIAL")

        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        # Vendor note (250515): ,T?
        if self.G_P0M1MCNoneAMS == 1:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT1]standalone multi-material, logical T?", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT1]single-color mode, logical T1", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT1]single-color refill mode, logical T1", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return

        # Vendor note (240527): ,becausemanual command,defaultSTM32MC
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.kaos_log(
                "DEBUG",
                "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "ifreceived T?command, but mode is Unknown mode, switch is MCmulti-material mode",
                "SERIAL",
            )
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            if self.G_SerialPort1OpenFlag == True:
                # [Translated vendor note] #Vendor note (231214): X YW H, purge
                # command_string = """
                # G90
                # G1 X%.03f Y%.03f F%d
                # """ % (
                # [Translated vendor note] point[0]+(num%2),#X; Vendor note (231215): xmm, printtoolhead
                # [Translated vendor note] point[1]+(num%2),#Y
                # [Translated vendor note] int(self.G_WaitAreaEachStepDist / self.G_MovementSpeedFactor),#
                # #500
                # )
                # [Translated vendor note] #Vendor note (231129):
                # self.G_PhrozenGCode.run_script_from_command(command_string)
                # [Translated vendor note] self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_P1CnAutoChangeChannel]filament change, XYP9")

                self.Cmds_AMSSerial1Send("MC")
                self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")

            self.G_ProzenToolhead.dwell(2)

        # Vendor note (250912): return
        # self.Cmds_CmdOrcaPre()

        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.kaos_log("DEBUG", "Automatic filament change", "SERIAL")
        # for UIUX dynamic interface
        self.emit_protocol("+T1:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")
        # Vendor note (240113): manual commandflag
        self.ManualCmdFlag = False
        # Vendor note (240221): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0
        self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")

        # Vendor note (241030): P1 C1P1 C32,132
        # 1:1 2 3 4
        # 2:5 6 7 8
        # 3:9 10 11 12
        # 4:13 14 15 16
        # 5:17 18 19 20
        # 6:21 22 23 24
        # 7:25 26 27 28
        # 8:29 30 31 32
        # auto filament change
        chan = 1 + 1
        self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")

        # Vendor note (250515): checkconfigcolor
        if self.G_ChromaKitAccessT1 > 0:
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT1=%d" % self.G_ChromaKitAccessT1, "SERIAL"
            )
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT1, gcmd)
        else:
            self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # for UIUX dynamic interface
        self.emit_protocol("+T1:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    def Cmds_CmdT2(self, gcmd):
        self.kaos_log("DEBUG", "=====[(cmds.python)Cmds_CmdT2 +1]orcaAMS", "SERIAL")

        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        # Vendor note (250515): ,T?
        if self.G_P0M1MCNoneAMS == 1:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT2]standalone multi-material, logical T?", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT2]single-color mode, logical T2", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT2]single-color refill mode, logical T2", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return

        # Vendor note (240527): ,becausemanual command,defaultSTM32MC
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.kaos_log(
                "DEBUG",
                "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "ifreceived T?command, but mode is Unknown mode, switch is MCmulti-material mode",
                "SERIAL",
            )
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")

            self.G_ProzenToolhead.dwell(2)

        # Vendor note (250912): return
        # self.Cmds_CmdOrcaPre()

        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.kaos_log("DEBUG", "Automatic filament change", "SERIAL")
        # for UIUX dynamic interface
        self.emit_protocol("+T2:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")
        # Vendor note (240113): manual commandflag
        self.ManualCmdFlag = False
        # Vendor note (240221): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0
        self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")

        # Vendor note (241030): P1 C1P1 C32,132
        # 1:1 2 3 4
        # 2:5 6 7 8
        # 3:9 10 11 12
        # 4:13 14 15 16
        # 5:17 18 19 20
        # 6:21 22 23 24
        # 7:25 26 27 28
        # 8:29 30 31 32
        # auto filament change
        chan = 2 + 1
        self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
        # Vendor note (250515): checkconfigcolor
        if self.G_ChromaKitAccessT2 > 0:
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT2=%d" % self.G_ChromaKitAccessT2, "SERIAL"
            )
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT2, gcmd)
        else:
            self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # for UIUX dynamic interface
        self.emit_protocol("+T2:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    def Cmds_CmdT3(self, gcmd):
        self.kaos_log("DEBUG", "=====[(cmds.python)Cmds_CmdT3 +1]orcaAMS", "SERIAL")

        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        # Vendor note (250515): ,T?
        if self.G_P0M1MCNoneAMS == 1:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT3]standalone multi-material, logical T?", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT3]single-color mode, logical T3", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT3]single-color refill mode, logical T3", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return

        # Vendor note (240527): ,becausemanual command,defaultSTM32MC
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.kaos_log(
                "DEBUG",
                "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "ifreceived T?command, but mode is Unknown mode, switch is MCmulti-material mode",
                "SERIAL",
            )
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")

            self.G_ProzenToolhead.dwell(2)

        # Vendor note (250912): #self.Cmds_CmdOrcaPre()

        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.kaos_log("DEBUG", "Automatic filament change", "SERIAL")
        # for UIUX dynamic interface
        self.emit_protocol("+T3:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")
        # Vendor note (240113): manual commandflag
        self.ManualCmdFlag = False
        # Vendor note (240221): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0
        self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")

        # Vendor note (241030): P1 C1P1 C32,132
        # 1:1 2 3 4
        # 2:5 6 7 8
        # 3:9 10 11 12
        # 4:13 14 15 16
        # 5:17 18 19 20
        # 6:21 22 23 24
        # 7:25 26 27 28
        # 8:29 30 31 32
        # auto filament change
        chan = 3 + 1
        self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
        # Vendor note (250515): checkconfigcolor
        if self.G_ChromaKitAccessT3 > 0:
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT3=%d" % self.G_ChromaKitAccessT3, "SERIAL"
            )
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT3, gcmd)
        else:
            self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # for UIUX dynamic interface
        self.emit_protocol("+T3:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    def Cmds_CmdT4(self, gcmd):
        self.kaos_log("DEBUG", "=====[(cmds.python)Cmds_CmdT4 +1]orcaAMS", "SERIAL")

        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        # Vendor note (250515): ,T?
        if self.G_P0M1MCNoneAMS == 1:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT4]standalone multi-material, logical T?", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT4]single-color mode, logical T4", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT4]single-color refill mode, logical T4", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return

        # Vendor note (240527): ,becausemanual command,defaultSTM32MC
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.kaos_log(
                "DEBUG",
                "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "ifreceived T?command, but mode is Unknown mode, switch is MCmulti-material mode",
                "SERIAL",
            )
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")

            self.G_ProzenToolhead.dwell(2)

        # Vendor note (250912): return
        # self.Cmds_CmdOrcaPre()

        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.kaos_log("DEBUG", "Automatic filament change", "SERIAL")
        # for UIUX dynamic interface
        self.emit_protocol("+T4:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")
        # Vendor note (240113): manual commandflag
        self.ManualCmdFlag = False
        # Vendor note (240221): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0
        self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")

        # Vendor note (241224): AMS,execute;
        if self.G_SerialPort2OpenFlag == True:
            self.kaos_log("DEBUG", "has #2 AMS, command", "SERIAL")
        else:
            self.kaos_log("DEBUG", "has #2 AMS, do not executecommand", "SERIAL")
            self.emit_protocol("+T4:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return

        # Vendor note (241030): P1 C1P1 C32,132
        # 1:1 2 3 4
        # 2:5 6 7 8
        # 3:9 10 11 12
        # 4:13 14 15 16
        # 5:17 18 19 20
        # 6:21 22 23 24
        # 7:25 26 27 28
        # 8:29 30 31 32
        # auto filament change
        chan = 4 + 1
        self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
        # Vendor note (250515): checkconfigcolor
        if self.G_ChromaKitAccessT4 > 0:
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT4=%d" % self.G_ChromaKitAccessT4, "SERIAL"
            )
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT4, gcmd)
        else:
            self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # for UIUX dynamic interface
        self.emit_protocol("+T4:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    def Cmds_CmdT5(self, gcmd):
        self.kaos_log("DEBUG", "=====[(cmds.python)Cmds_CmdT5 +1]orcaAMS", "SERIAL")

        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        # Vendor note (250515): ,T?
        if self.G_P0M1MCNoneAMS == 1:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT5]standalone multi-material, logical T?", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT5]single-color mode, logical T5", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT5]single-color refill mode, logical T5", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return

        # Vendor note (240527): ,becausemanual command,defaultSTM32MC
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.kaos_log(
                "DEBUG",
                "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "ifreceived T?command, but mode is Unknown mode, switch is MCmulti-material mode",
                "SERIAL",
            )
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")

            self.G_ProzenToolhead.dwell(2)

        # Vendor note (250912): return
        # self.Cmds_CmdOrcaPre()

        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.kaos_log("DEBUG", "Automatic filament change", "SERIAL")
        # for UIUX dynamic interface
        self.emit_protocol("+T5:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")
        # Vendor note (240113): manual commandflag
        self.ManualCmdFlag = False
        # Vendor note (240221): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0
        self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")

        # Vendor note (241224): AMS,execute;
        if self.G_SerialPort2OpenFlag == True:
            self.kaos_log("DEBUG", "has #2 AMS, command", "SERIAL")
        else:
            self.kaos_log("DEBUG", "has #2 AMS, do not executecommand", "SERIAL")
            self.emit_protocol("+T5:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return

        # Vendor note (241030): P1 C1P1 C32,132
        # 1:1 2 3 4
        # 2:5 6 7 8
        # 3:9 10 11 12
        # 4:13 14 15 16
        # 5:17 18 19 20
        # 6:21 22 23 24
        # 7:25 26 27 28
        # 8:29 30 31 32
        # auto filament change
        chan = 5 + 1
        self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
        # Vendor note (250515): checkconfigcolor
        if self.G_ChromaKitAccessT5 > 0:
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT5=%d" % self.G_ChromaKitAccessT5, "SERIAL"
            )
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT5, gcmd)
        else:
            self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # for UIUX dynamic interface
        self.emit_protocol("+T5:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    def Cmds_CmdT6(self, gcmd):
        self.kaos_log("DEBUG", "=====[(cmds.python)Cmds_CmdT6 +1]orcaAMS", "SERIAL")

        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        # Vendor note (250515): ,T?
        if self.G_P0M1MCNoneAMS == 1:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT6]standalone multi-material, logical T?", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT6]single-color mode, logical T6", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT6]single-color refill mode, logical T6", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return

        # Vendor note (240527): ,becausemanual command,defaultSTM32MC
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.kaos_log(
                "DEBUG",
                "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "ifreceived T?command, but mode is Unknown mode, switch is MCmulti-material mode",
                "SERIAL",
            )
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")

            self.G_ProzenToolhead.dwell(2)

        # Vendor note (250912): return
        # self.Cmds_CmdOrcaPre()

        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.kaos_log("DEBUG", "Automatic filament change", "SERIAL")
        # for UIUX dynamic interface
        self.emit_protocol("+T6:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")
        # Vendor note (240113): manual commandflag
        self.ManualCmdFlag = False
        # Vendor note (240221): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0
        self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")

        # Vendor note (241224): AMS,execute;
        if self.G_SerialPort2OpenFlag == True:
            self.kaos_log("DEBUG", "has #2 AMS, command", "SERIAL")
        else:
            self.kaos_log("DEBUG", "has #2 AMS, do not executecommand", "SERIAL")
            self.emit_protocol("+T6:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return

        # Vendor note (241030): P1 C1P1 C32,132
        # 1:1 2 3 4
        # 2:5 6 7 8
        # 3:9 10 11 12
        # 4:13 14 15 16
        # 5:17 18 19 20
        # 6:21 22 23 24
        # 7:25 26 27 28
        # 8:29 30 31 32
        # auto filament change
        # Channel map (P1 C1..P1 C32; valid range 1..32)
        # Unit 1:  1  2  3  4
        # Unit 2:  5  6  7  8
        # Unit 3:  9 10 11 12
        # Unit 4: 13 14 15 16
        # Unit 5: 17 18 19 20
        # Unit 6: 21 22 23 24
        # Unit 7: 25 26 27 28
        # Unit 8: 29 30 31 32
        # Automatic filament-change path
        chan = 6 + 1
        self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
        # Vendor note (250515): checkconfigcolor
        if self.G_ChromaKitAccessT6 > 0:
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT6=%d" % self.G_ChromaKitAccessT6, "SERIAL"
            )
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT6, gcmd)
        else:
            self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # for UIUX dynamic interface
        self.emit_protocol("+T6:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    def Cmds_CmdT7(self, gcmd):
        self.kaos_log("DEBUG", "=====[(cmds.python)Cmds_CmdT7 +1]orcaAMS", "SERIAL")

        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        # Vendor note (250515): ,T?
        if self.G_P0M1MCNoneAMS == 1:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT7]standalone multi-material, logical T?", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT7]single-color mode, logical T7", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT7]single-color refill mode, logical T7", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return

        # Vendor note (240527): ,becausemanual command,defaultSTM32MC
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.kaos_log(
                "DEBUG",
                "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "ifreceived T?command, but mode is Unknown mode, switch is MCmulti-material mode",
                "SERIAL",
            )
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            # [Translated vendor note] Vendor note (241030): P1 C1P1 C32, 132
            # [Translated vendor note] 1: 1 2 3 4
            # [Translated vendor note] 2: 5 6 7 8
            # [Translated vendor note] 3: 9 10 11 12
            # [Translated vendor note] 4: 13 14 15 16
            # [Translated vendor note] 5: 17 18 19 20
            # [Translated vendor note] 6: 21 22 23 24
            # [Translated vendor note] 7: 25 26 27 28
            # [Translated vendor note] 8: 29 30 31 32
            # [Translated vendor note] filament change

            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")

            self.G_ProzenToolhead.dwell(2)

        # Vendor note (250912): return
        # self.Cmds_CmdOrcaPre()

        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.kaos_log("DEBUG", "Automatic filament change", "SERIAL")
        # for UIUX dynamic interface
        self.emit_protocol("+T7:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")
        # Vendor note (240113): manual commandflag
        self.ManualCmdFlag = False
        # Vendor note (240221): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0
        self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")

        # Vendor note (241224): AMS,execute;
        if self.G_SerialPort2OpenFlag == True:
            self.kaos_log("DEBUG", "has #2 AMS, command", "SERIAL")
        else:
            self.kaos_log("DEBUG", "has #2 AMS, do not executecommand", "SERIAL")
            self.emit_protocol("+T7:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return

        # Vendor note (241030): P1 C1P1 C32,132
        # 1:1 2 3 4
        # 2:5 6 7 8
        # 3:9 10 11 12
        # 4:13 14 15 16
        # 5:17 18 19 20
        # 6:21 22 23 24
        # 7:25 26 27 28
        # 8:29 30 31 32
        # auto filament change
        # Channel map (P1 C1..P1 C32; valid range 1..32)
        # Unit 1:  1  2  3  4
        # Unit 2:  5  6  7  8
        # Unit 3:  9 10 11 12
        # Unit 4: 13 14 15 16
        # Unit 5: 17 18 19 20
        # Unit 6: 21 22 23 24
        # Unit 7: 25 26 27 28
        # Unit 8: 29 30 31 32
        # Automatic filament-change path
        chan = 7 + 1
        self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
        # Vendor note (250515): checkconfigcolor
        if self.G_ChromaKitAccessT7 > 0:
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT7=%d" % self.G_ChromaKitAccessT7, "SERIAL"
            )
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT7, gcmd)
        else:
            self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # for UIUX dynamic interface
        self.emit_protocol("+T7:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    def Cmds_CmdT8(self, gcmd):
        self.kaos_log("DEBUG", "=====[(cmds.python)Cmds_CmdT8 +1]orcaAMS", "SERIAL")

        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        # Vendor note (250515): ,T?
        if self.G_P0M1MCNoneAMS == 1:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT8]standalone multi-material, logical T?", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT8]single-color mode, logical T8", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT8]single-color refill mode, logical T8", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return

        # Vendor note (240527): ,becausemanual command,defaultSTM32MC
        # [Translated vendor note] Vendor note (241030): P1 C1P1 C32, 132
        # [Translated vendor note] 1: 1 2 3 4
        # [Translated vendor note] 2: 5 6 7 8
        # [Translated vendor note] 3: 9 10 11 12
        # [Translated vendor note] 4: 13 14 15 16
        # [Translated vendor note] 5: 17 18 19 20
        # [Translated vendor note] 6: 21 22 23 24
        # [Translated vendor note] 7: 25 26 27 28
        # [Translated vendor note] 8: 29 30 31 32
        # [Translated vendor note] filament change

        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.kaos_log(
                "DEBUG",
                "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "ifreceived T?command, but mode is Unknown mode, switch is MCmulti-material mode",
                "SERIAL",
            )
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")

            self.G_ProzenToolhead.dwell(2)

        # Vendor note (250912): return
        # self.Cmds_CmdOrcaPre()

        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.kaos_log("DEBUG", "Automatic filament change", "SERIAL")
        # for UIUX dynamic interface
        self.emit_protocol("+T8:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")
        # Vendor note (240113): manual commandflag
        self.ManualCmdFlag = False
        # Vendor note (240221): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0
        self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")

        # Vendor note (241224): 3AMS,execute;
        if self.G_SerialPort2OpenFlag == True:
            self.kaos_log("DEBUG", "has #3 AMS, command", "SERIAL")
        else:
            self.kaos_log("DEBUG", "has #3 AMS, do not executecommand", "SERIAL")
            self.emit_protocol("+T8:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return

        # Vendor note (241030): P1 C1P1 C32,132
        # 1:1 2 3 4
        # 2:5 6 7 8
        # 3:9 10 11 12
        # 4:13 14 15 16
        # 5:17 18 19 20
        # 6:21 22 23 24
        # 7:25 26 27 28
        # 8:29 30 31 32
        # auto filament change
        # Channel map (P1 C1..P1 C32; valid range 1..32)
        # Unit 1:  1  2  3  4
        # Unit 2:  5  6  7  8
        # Unit 3:  9 10 11 12
        # Unit 4: 13 14 15 16
        # Unit 5: 17 18 19 20
        # Unit 6: 21 22 23 24
        # Unit 7: 25 26 27 28
        # Unit 8: 29 30 31 32
        # Automatic filament-change path
        chan = 8 + 1
        self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
        # Vendor note (250515): checkconfigcolor
        if self.G_ChromaKitAccessT8 > 0:
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT8=%d" % self.G_ChromaKitAccessT8, "SERIAL"
            )
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT8, gcmd)
        else:
            self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # for UIUX dynamic interface
        self.emit_protocol("+T8:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    def Cmds_CmdT9(self, gcmd):
        self.kaos_log("DEBUG", "=====[(cmds.python)Cmds_CmdT9 +1]orcaAMS", "SERIAL")

        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        # Vendor note (250515): ,T?
        if self.G_P0M1MCNoneAMS == 1:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT9]standalone multi-material, logical T?", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT9]single-color mode, logical T9", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT9]single-color refill mode, logical T9", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
        #[Translated vendor note] Vendor note (241030): P1 C1P1 C32, 132
        #[Translated vendor note] 1: 1 2 3 4
        #[Translated vendor note] 2: 5 6 7 8
        #[Translated vendor note] 3: 9 10 11 12
        #[Translated vendor note] 4: 13 14 15 16
        #[Translated vendor note] 5: 17 18 19 20
        #[Translated vendor note] 6: 21 22 23 24
        #[Translated vendor note] 7: 25 26 27 28
        #[Translated vendor note] 8: 29 30 31 32
        #[Translated vendor note] filament change

                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return

        # Vendor note (240527): ,becausemanual command,defaultSTM32MC
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.kaos_log(
                "DEBUG",
                "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "ifreceived T?command, but mode is Unknown mode, switch is MCmulti-material mode",
                "SERIAL",
            )
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")

            self.G_ProzenToolhead.dwell(2)

        # # Vendor note (250912): # self.Cmds_CmdOrcaPre()

        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.kaos_log("DEBUG", "Automatic filament change", "SERIAL")
        # for UIUX dynamic interface
        self.emit_protocol("+T9:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")
        # Vendor note (240113): manual commandflag
        self.ManualCmdFlag = False
        # Vendor note (240221): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0
        self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")

        # Vendor note (241224): 3AMS,execute;
        if self.G_SerialPort2OpenFlag == True:
            self.kaos_log("DEBUG", "has #3 AMS, command", "SERIAL")
        else:
            self.kaos_log("DEBUG", "has #3 AMS, do not executecommand", "SERIAL")
            self.emit_protocol("+T9:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return

        # Vendor note (241030): P1 C1P1 C32,132
        # 1:1 2 3 4
        # 2:5 6 7 8
        # 3:9 10 11 12
        # 4:13 14 15 16
        # 5:17 18 19 20
        # 6:21 22 23 24
        # 7:25 26 27 28
        # 8:29 30 31 32
        # auto filament change
        # Channel map (P1 C1..P1 C32; valid range 1..32)
        # Unit 1:  1  2  3  4
        # Unit 2:  5  6  7  8
        # Unit 3:  9 10 11 12
        # Unit 4: 13 14 15 16
        # Unit 5: 17 18 19 20
        # Unit 6: 21 22 23 24
        # Unit 7: 25 26 27 28
        # Unit 8: 29 30 31 32
        # Automatic filament-change path
        chan = 9 + 1
        self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
        # Vendor note (250515): checkconfigcolor
        if self.G_ChromaKitAccessT9 > 0:
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT9=%d" % self.G_ChromaKitAccessT9, "SERIAL"
            )
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT9, gcmd)
        else:
            self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # for UIUX dynamic interface
        self.emit_protocol("+T9:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    def Cmds_CmdT10(self, gcmd):
        self.kaos_log("DEBUG", "=====[(cmds.python)Cmds_CmdT10 +1]orcaAMS", "SERIAL")

        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        # Vendor note (250515): ,T?
        if self.G_P0M1MCNoneAMS == 1:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT10]standalone multi-material, logical T?", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT10]single-color mode, logical T10", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            # [Translated vendor note] Vendor note (241030): P1 C1P1 C32, 132
            # [Translated vendor note] 1: 1 2 3 4
            # [Translated vendor note] 2: 5 6 7 8
            # [Translated vendor note] 3: 9 10 11 12
            # [Translated vendor note] 4: 13 14 15 16
            # [Translated vendor note] 5: 17 18 19 20
            # [Translated vendor note] 6: 21 22 23 24
            # [Translated vendor note] 7: 25 26 27 28
            # [Translated vendor note] 8: 29 30 31 32
            # [Translated vendor note] filament change

            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT10]single-color refill mode, logical T10", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return

        # Vendor note (240527): ,becausemanual command,defaultSTM32MC
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.kaos_log(
                "DEBUG",
                "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "ifreceived T?command, but mode is Unknown mode, switch is MCmulti-material mode",
                "SERIAL",
            )
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")

            self.G_ProzenToolhead.dwell(2)

        # Vendor note (250912): return
        # self.Cmds_CmdOrcaPre()

        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.kaos_log("DEBUG", "Automatic filament change", "SERIAL")
        # for UIUX dynamic interface
        self.emit_protocol("+T10:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")
        # Vendor note (240113): manual commandflag
        self.ManualCmdFlag = False
        # Vendor note (240221): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0
        self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")

        # Vendor note (241224): 3AMS,execute;
        if self.G_SerialPort2OpenFlag == True:
            self.kaos_log("DEBUG", "has #3 AMS, command", "SERIAL")
        else:
            self.kaos_log("DEBUG", "has #3 AMS, do not executecommand", "SERIAL")
            self.emit_protocol("+T10:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return

        # Vendor note (241030): P1 C1P1 C32,132
        # 1:1 2 3 4
        # 2:5 6 7 8
        # 3:9 10 11 12
        # 4:13 14 15 16
        # 5:17 18 19 20
        # 6:21 22 23 24
        # 7:25 26 27 28
        # 8:29 30 31 32
        # auto filament change
        # Channel map (P1 C1..P1 C32; valid range 1..32)
        # Unit 1:  1  2  3  4
        # Unit 2:  5  6  7  8
        # Unit 3:  9 10 11 12
        # Unit 4: 13 14 15 16
        # Unit 5: 17 18 19 20
        # Unit 6: 21 22 23 24
        # Unit 7: 25 26 27 28
        # Unit 8: 29 30 31 32
        # Automatic filament-change path
        chan = 10 + 1
        self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
        # Vendor note (250515): checkconfigcolor
        if self.G_ChromaKitAccessT10 > 0:
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT10=%d" % self.G_ChromaKitAccessT10, "SERIAL"
            )
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT10, gcmd)
        else:
            self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # for UIUX dynamic interface
        self.emit_protocol("+T10:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    def Cmds_CmdT11(self, gcmd):
        self.kaos_log("DEBUG", "=====[(cmds.python)Cmds_CmdT11 +1]orcaAMS", "SERIAL")

        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        # Vendor note (250515): ,T?
        if self.G_P0M1MCNoneAMS == 1:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT11]standalone multi-material, logical T?", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT11]single-color mode, logical T11", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            # [Translated vendor note] Vendor note (241030): P1 C1P1 C32, 132
            # [Translated vendor note] 1: 1 2 3 4
            # [Translated vendor note] 2: 5 6 7 8
            # [Translated vendor note] 3: 9 10 11 12
            # [Translated vendor note] 4: 13 14 15 16
            # [Translated vendor note] 5: 17 18 19 20
            # [Translated vendor note] 6: 21 22 23 24
            # [Translated vendor note] 7: 25 26 27 28
            # [Translated vendor note] 8: 29 30 31 32
            # [Translated vendor note] filament change

            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT11]single-color refill mode, logical T11", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return

        # Vendor note (240527): ,becausemanual command,defaultSTM32MC
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.kaos_log(
                "DEBUG",
                "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "ifreceived T?command, but mode is Unknown mode, switch is MCmulti-material mode",
                "SERIAL",
            )
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")

            self.G_ProzenToolhead.dwell(2)

        # Vendor note (250912): return
        # self.Cmds_CmdOrcaPre()

        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.kaos_log("DEBUG", "Automatic filament change", "SERIAL")
        # for UIUX dynamic interface
        self.emit_protocol("+T11:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")
        # Vendor note (240113): manual commandflag
        self.ManualCmdFlag = False
        # Vendor note (240221): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0
        self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")

        # Vendor note (241224): 3AMS,execute;
        if self.G_SerialPort2OpenFlag == True:
            self.kaos_log("DEBUG", "has #3 AMS, command", "SERIAL")
        else:
            self.kaos_log("DEBUG", "has #3 AMS, do not executecommand", "SERIAL")
            self.emit_protocol("+T11:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return

        # Vendor note (241030): P1 C1P1 C32,132
        # 1:1 2 3 4
        # 2:5 6 7 8
        # 3:9 10 11 12
        # 4:13 14 15 16
        # 5:17 18 19 20
        # 6:21 22 23 24
        # 7:25 26 27 28
        # 8:29 30 31 32
        # auto filament change
        # Channel map (P1 C1..P1 C32; valid range 1..32)
        # Unit 1:  1  2  3  4
        # Unit 2:  5  6  7  8
        # Unit 3:  9 10 11 12
        # Unit 4: 13 14 15 16
        # Unit 5: 17 18 19 20
        # Unit 6: 21 22 23 24
        # Unit 7: 25 26 27 28
        # Unit 8: 29 30 31 32
        # Automatic filament-change path
        chan = 11 + 1
        self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
        # Vendor note (250515): checkconfigcolor
        if self.G_ChromaKitAccessT11 > 0:
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT11=%d" % self.G_ChromaKitAccessT11, "SERIAL"
            )
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT11, gcmd)
        else:
            self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # for UIUX dynamic interface
        self.emit_protocol("+T11:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    def Cmds_CmdT12(self, gcmd):
        self.kaos_log("DEBUG", "=====[(cmds.python)Cmds_CmdT12 +1]orcaAMS", "SERIAL")

        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        # Vendor note (250515): ,T?
        if self.G_P0M1MCNoneAMS == 1:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT12]standalone multi-material, logical T?", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT12]single-color mode, logical T12", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT12]single-color refill mode, logical T12", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            # [Translated vendor note] Vendor note (241030): P1 C1P1 C32, 132
            # [Translated vendor note] 1: 1 2 3 4
            # [Translated vendor note] 2: 5 6 7 8
            # [Translated vendor note] 3: 9 10 11 12
            # [Translated vendor note] 4: 13 14 15 16
            # [Translated vendor note] 5: 17 18 19 20
            # [Translated vendor note] 6: 21 22 23 24
            # [Translated vendor note] 7: 25 26 27 28
            # [Translated vendor note] 8: 29 30 31 32
            # [Translated vendor note] filament change

            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return

        # Vendor note (240527): ,becausemanual command,defaultSTM32MC
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.kaos_log(
                "DEBUG",
                "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "ifreceived T?command, but mode is Unknown mode, switch is MCmulti-material mode",
                "SERIAL",
            )
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")

            self.G_ProzenToolhead.dwell(2)

        # Vendor note (250912): return
        # self.Cmds_CmdOrcaPre()

        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.kaos_log("DEBUG", "Automatic filament change", "SERIAL")
        # for UIUX dynamic interface
        self.emit_protocol("+T12:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")
        # Vendor note (240113): manual commandflag
        self.ManualCmdFlag = False
        # Vendor note (240221): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0
        self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")

        # Vendor note (241224): 4AMS,execute;
        if self.G_SerialPort2OpenFlag == True:
            self.kaos_log("DEBUG", "has #4 AMS, command", "SERIAL")
        else:
            self.kaos_log("DEBUG", "has #4 AMS, do not executecommand", "SERIAL")
            self.emit_protocol("+T12:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return

        # Vendor note (241030): P1 C1P1 C32,132
        # 1:1 2 3 4
        # 2:5 6 7 8
        # 3:9 10 11 12
        # 4:13 14 15 16
        # 5:17 18 19 20
        # 6:21 22 23 24
        # 7:25 26 27 28
        # 8:29 30 31 32
        # auto filament change
        # Channel map (P1 C1..P1 C32; valid range 1..32)
        # Unit 1:  1  2  3  4
        # Unit 2:  5  6  7  8
        # Unit 3:  9 10 11 12
        # Unit 4: 13 14 15 16
        # Unit 5: 17 18 19 20
        # Unit 6: 21 22 23 24
        # Unit 7: 25 26 27 28
        # Unit 8: 29 30 31 32
        # Automatic filament-change path
        chan = 12 + 1
        self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
        # Vendor note (250515): checkconfigcolor
        if self.G_ChromaKitAccessT12 > 0:
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT12=%d" % self.G_ChromaKitAccessT12, "SERIAL"
            )
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT12, gcmd)
        else:
            self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # for UIUX dynamic interface
        self.emit_protocol("+T12:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    def Cmds_CmdT13(self, gcmd):
        self.kaos_log("DEBUG", "=====[(cmds.python)Cmds_CmdT13 +1]orcaAMS", "SERIAL")

        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        # Vendor note (250515): ,T?
        if self.G_P0M1MCNoneAMS == 1:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT13]standalone multi-material, logical T?", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT13]single-color mode, logical T13", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT13]single-color refill mode, logical T13", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return

        # Vendor note (240527): ,becausemanual command,defaultSTM32MC
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.kaos_log(
                "DEBUG",
                "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                "SERIAL",
            )
            # [Translated vendor note] Vendor note (241030): P1 C1P1 C32, 132
            # [Translated vendor note] 1: 1 2 3 4
            # [Translated vendor note] 2: 5 6 7 8
            # [Translated vendor note] 3: 9 10 11 12
            # [Translated vendor note] 4: 13 14 15 16
            # [Translated vendor note] 5: 17 18 19 20
            # [Translated vendor note] 6: 21 22 23 24
            # [Translated vendor note] 7: 25 26 27 28
            # [Translated vendor note] 8: 29 30 31 32
            # [Translated vendor note] filament change

            self.kaos_log(
                "DEBUG",
                "ifreceived T?command, but mode is Unknown mode, switch is MCmulti-material mode",
                "SERIAL",
            )
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")

            self.G_ProzenToolhead.dwell(2)

        # Vendor note (250912): return
        # self.Cmds_CmdOrcaPre()

        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.kaos_log("DEBUG", "Automatic filament change", "SERIAL")
        # for UIUX dynamic interface
        self.emit_protocol("+T13:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")
        # Vendor note (240113): manual commandflag
        self.ManualCmdFlag = False
        # Vendor note (240221): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0
        self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")

        # Vendor note (241224): 4AMS,execute;
        if self.G_SerialPort2OpenFlag == True:
            self.kaos_log("DEBUG", "has #4 AMS, command", "SERIAL")
        else:
            self.kaos_log("DEBUG", "has #4 AMS, do not executecommand", "SERIAL")
            self.emit_protocol("+T13:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return

        # Vendor note (241030): P1 C1P1 C32,132
        # 1:1 2 3 4
        # 2:5 6 7 8
        # 3:9 10 11 12
        # 4:13 14 15 16
        # 5:17 18 19 20
        # 6:21 22 23 24
        # 7:25 26 27 28
        # 8:29 30 31 32
        # auto filament change
        # Channel map (P1 C1..P1 C32; valid range 1..32)
        # Unit 1:  1  2  3  4
        # Unit 2:  5  6  7  8
        # Unit 3:  9 10 11 12
        # Unit 4: 13 14 15 16
        # Unit 5: 17 18 19 20
        # Unit 6: 21 22 23 24
        # Unit 7: 25 26 27 28
        # Unit 8: 29 30 31 32
        # Automatic filament-change path
        chan = 13 + 1
        self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
        # Vendor note (250515): checkconfigcolor
        if self.G_ChromaKitAccessT13 > 0:
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT13=%d" % self.G_ChromaKitAccessT13, "SERIAL"
            )
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT13, gcmd)
        else:
            self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # for UIUX dynamic interface
        self.emit_protocol("+T13:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    def Cmds_CmdT14(self, gcmd):
        self.kaos_log("DEBUG", "=====[(cmds.python)Cmds_CmdT14 +1]orcaAMS", "SERIAL")

        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        # Vendor note (250515): ,T?
        if self.G_P0M1MCNoneAMS == 1:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT14]standalone multi-material, logical T?", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT14]single-color mode, logical T14", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT14]single-color refill mode, logical T14", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return

        # Vendor note (240527): ,becausemanual command,defaultSTM32MC
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.kaos_log(
                "DEBUG",
                "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "ifreceived T?command, but mode is Unknown mode, switch is MCmulti-material mode",
                "SERIAL",
            )
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
            # [Translated vendor note] Vendor note (241030): P1 C1P1 C32, 132
            # [Translated vendor note] 1: 1 2 3 4
            # [Translated vendor note] 2: 5 6 7 8
            # [Translated vendor note] 3: 9 10 11 12
            # [Translated vendor note] 4: 13 14 15 16
            # [Translated vendor note] 5: 17 18 19 20
            # [Translated vendor note] 6: 21 22 23 24
            # [Translated vendor note] 7: 25 26 27 28
            # [Translated vendor note] 8: 29 30 31 32
            # [Translated vendor note] filament change

            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")

            self.G_ProzenToolhead.dwell(2)

        # Vendor note (250912): return
        # self.Cmds_CmdOrcaPre()

        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.kaos_log("DEBUG", "Automatic filament change", "SERIAL")
        # for UIUX dynamic interface
        self.emit_protocol("+T14:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")
        # Vendor note (240113): manual commandflag
        self.ManualCmdFlag = False
        # Vendor note (240221): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0
        self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")

        # Vendor note (241224): 4AMS,execute;
        if self.G_SerialPort2OpenFlag == True:
            self.kaos_log("DEBUG", "has #4 AMS, command", "SERIAL")
        else:
            self.kaos_log("DEBUG", "has #4 AMS, do not executecommand", "SERIAL")
            self.emit_protocol("+T14:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return

        # Vendor note (241030): P1 C1P1 C32,132
        # 1:1 2 3 4
        # 2:5 6 7 8
        # 3:9 10 11 12
        # 4:13 14 15 16
        # 5:17 18 19 20
        # 6:21 22 23 24
        # 7:25 26 27 28
        # 8:29 30 31 32
        # auto filament change
        # Channel map (P1 C1..P1 C32; valid range 1..32)
        # Unit 1:  1  2  3  4
        # Unit 2:  5  6  7  8
        # Unit 3:  9 10 11 12
        # Unit 4: 13 14 15 16
        # Unit 5: 17 18 19 20
        # Unit 6: 21 22 23 24
        # Unit 7: 25 26 27 28
        # Unit 8: 29 30 31 32
        # Automatic filament-change path
        chan = 14 + 1
        self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
        # Vendor note (250515): checkconfigcolor
        if self.G_ChromaKitAccessT14 > 0:
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT14=%d" % self.G_ChromaKitAccessT14, "SERIAL"
            )
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT14, gcmd)
        else:
            self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # for UIUX dynamic interface
        self.emit_protocol("+T14:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    def Cmds_CmdT15(self, gcmd):
        self.kaos_log("DEBUG", "=====[(cmds.python)Cmds_CmdT15 +1]orcaAMS", "SERIAL")

        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        # Vendor note (250515): ,T?
        if self.G_P0M1MCNoneAMS == 1:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT15]standalone multi-material, logical T?", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT15]single-color mode, logical T15", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdT15]single-color refill mode, logical T15", "SERIAL"
            )
            # Vendor note (250828): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            command_string = """
                # PRZ_PAUSE_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG", "[SERVICE] External macro; command_string='%s'" % command_string, "SERIAL"
            )
            return

        # Vendor note (240527): ,becausemanual command,defaultSTM32MC
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.kaos_log(
                "DEBUG",
                "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "ifreceived T?command, but mode is Unknown mode, switch is MCmulti-material mode",
                "SERIAL",
            )
            self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")

            self.G_ProzenToolhead.dwell(2)

        # Vendor note (250912): #self.Cmds_CmdOrcaPre()

        # self.G_PhrozenFluiddRespondInfo("command='%s'; Automatic filament change" % (gcmd.get_commandline(),))
        self.kaos_log("DEBUG", "Automatic filament change", "SERIAL")
        # for UIUX dynamic interface
        self.emit_protocol("+T15:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")
        # Vendor note (240113): manual commandflag
        self.ManualCmdFlag = False
        # Vendor note (240221): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0
        self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")

        # Vendor note (241224): 4AMS,execute;
        if self.G_SerialPort2OpenFlag == True:
            self.kaos_log("DEBUG", "has #4 AMS, command", "SERIAL")
        else:
            self.kaos_log("DEBUG", "has #4 AMS, do not executecommand", "SERIAL")
            self.emit_protocol("+T15:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return

        # Vendor note (241030): P1 C1P1 C32,132
        # 1:1 2 3 4
        # 2:5 6 7 8
        # 3:9 10 11 12
        # 4:13 14 15 16
        # 5:17 18 19 20
        # 6:21 22 23 24
        # 7:25 26 27 28
        # 8:29 30 31 32
        # auto filament change
        # Channel map (P1 C1..P1 C32; valid range 1..32)
        # Unit 1:  1  2  3  4
        # Unit 2:  5  6  7  8
        # Unit 3:  9 10 11 12
        # Unit 4: 13 14 15 16
        # Unit 5: 17 18 19 20
        # Unit 6: 21 22 23 24
        # Unit 7: 25 26 27 28
        # Unit 8: 29 30 31 32
        # Automatic filament-change path
        chan = 15 + 1
        self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
        # Vendor note (250515): checkconfigcolor
        if self.G_ChromaKitAccessT15 > 0:
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT15=%d" % self.G_ChromaKitAccessT15, "SERIAL"
            )
            self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT15, gcmd)
        else:
            self.kaos_log("DEBUG", "chan=%d" % chan, "SERIAL")
            self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

        # for UIUX dynamic interface
        self.emit_protocol("+T15:1,%d" % self.G_ChangeChannelTimeoutNewChan)

    # PRZ_VERSION query
    def Cmds_PhrozenVersion(self, gcmd):
        # ASCII128(),()0127(0000 00000111 1111,
        # 0x000x7F),0:
        # 0~31:control,0x07(BEL)calculate0x00(NUL,)
        # end0x0D(CR)0x0A(LF)()();
        # : controlcontrol“”,,“”,
        #
        # 32:;
        # 33~126:,48~570-9,65~9026,97~12226,
        # ;
        # 127:controlDEL

        self.kaos_log(
            "DEBUG",
            "[(cmds.python)Cmds_PhrozenVersion]command='%s'" % (gcmd.get_commandline(),),
            "SERIAL",
        )

        self.kaos_log("DEBUG", "Current mode", "SERIAL")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.emit_mode(0, "unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.emit_mode(1, "MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.emit_mode(2, "MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.emit_mode(3, "RUNOUT")
        else:
            self.emit_mode(-1, "error")

        self.kaos_log("DEBUG", "%s" % (gcmd.get_commandline(),), "SERIAL")
        # # Vendor note (240224): # command = """
        # PAUSE
        # """
        # self.G_PhrozenGCode.run_script_from_command(command)
        # self.G_PhrozenFluiddRespondInfo("[(cmds.Cmds_PhrozenVersion)]calling macro command: command=%s" % (command))
        # self.G_PhrozenFluiddRespondInfo("[(cmds.Cmds_PhrozenVersion)]prevent pause failure, extra command; send_pause_command")
        # self.G_PhrozenPrinterCancelPauseResume.send_pause_command()
        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()
        if self.G_SerialPort1OpenFlag == True:
            # Vendor note (240524): readAMS16HUB
            self.Cmds_AMSSerial1Send("AT+SB=0")
            self.kaos_log(
                "DEBUG",
                "serial port 1Sending command: AT+SB=0; get AMSmainboard,16HUBmainboard",
                "SERIAL",
            )
        if self.G_SerialPort2OpenFlag == True:
            self.Cmds_AMSSerial2Send("AT+SB=0")
            self.kaos_log(
                "DEBUG",
                "serial port 2Sending command: AT+SB=0; get AMSmainboard,16HUBmainboard",
                "SERIAL",
            )

        # Vendor note (240529): phrozen
        self.G_PhrozenFluiddRespondInfo("V-H%s-I%s-F%s" % (HW_VERSION, IMAGE_VERSION, FW_VERSION))

        # emb_filename = "/home/prz/hdlDat/DriveCodeJson.dat"
        # json_data = json.load(emb_filename)
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

        # self.G_ProzenToolhead.dwell(1.5)

        # =====DriveCodeFile.dat
        # 1 , 18 , 24053 , 18 , 0# // AMS board 1 firmware-18
        # 2 , 18 , 24053 , 18 , 0# // AMS board 2 firmware-18
        # 3 , 18 , 24053 , 18 , 0# // AMS board 3 firmware-18
        # 4 , 18 , 24053 , 18 , 0# // AMS board 4 firmware-18
        # 5 , 5 , 24046 , 5 , 0# // OTA sub-program - AMS serial upgrade-5 5
        # 6 , 0 , 0 , 0 , 0# // buffer board firmware-6 6
        # 7 , 7 , 24051 , 7 , 0# // 16-color HUB board firmware-7 7
        # 8 , 0 , 0 , 0 , 0
        # 9 , 0 , 0 , 0 , 0
        # 10 , 10 , 24054 , 10 , 0# // OTA sub-program - TJC display background-10
        # 11 , 11 , 24047 , 11 , 0# // TJC serial display HMI firmware-11
        # 12 , 0 , 0 , 0 , 0
        # 13 , 0 , 0 , 0 , 0
        # 14 , 0 , 0 , 0 , 0
        # 15 , 15 , 25042 , 15 , 0
        # 16 , 16 , 25042 , 16 , 0
        # 17 , ? , 25042 , ? , 0
        # 18 , ? , 25042 , ? , 0
        # 19 , ? , 25042 , ? , 0
        # 20 , ? , 25042 , ? , 0
        # Vendor note (240530): write version to DriveCodeJson.dat
        # Vendor note (250724): read system image id to distinguish product/board/firmware variants
        # Vendor note (250724): read image id
        self.Cmds_GetImageId()
        if self.G_ImageId == 16:
            self.kaos_log("DEBUG", "Image ID==16: ARCO300-MKS-RK3328-STM32F407VET6-I16", "SERIAL")
            filename = "/home/mks/hdlDat/DriveCodeFile.dat"
            self.kaos_log("DEBUG", "filename=%s" % filename, "SERIAL")
        elif self.G_ImageId == 31:
            self.kaos_log(
                "DEBUG", "Image ID==31: ARCO300-phrozen-RK3308-STM32F407VET6-I31", "SERIAL"
            )
            filename = "/home/prz/hdlDat/DriveCodeFile.dat"
            self.kaos_log("DEBUG", "filename=%s" % filename, "SERIAL")
        elif self.G_ImageId == -1:
            self.kaos_log(
                "DEBUG", "Image ID==-1, default: ARCO300-MKS-RK3328-STM32F407VET6-I16", "SERIAL"
            )
            filename = "/home/mks/hdlDat/DriveCodeFile.dat"
            self.kaos_log("DEBUG", "filename=%s" % filename, "SERIAL")
        else:
            self.kaos_log(
                "DEBUG",
                "Image ID could not be read, default: ARCO300-MKS-RK3328-STM32F407VET6-I16",
                "SERIAL",
            )
            filename = "/home/mks/hdlDat/DriveCodeFile.dat"
            self.kaos_log("DEBUG", "filename=%s" % filename, "SERIAL")

        Lo_AllLine = ""
        # data = [{"DriveCode":16,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":15,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":14,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":13,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":12,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":11,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":10,"DriveImageType":10,"DriveHwVersion":10,"DriveFwVersion":24045,"DriveId":0},{"DriveCode":9,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":8,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":7,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":6,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":5,"DriveImageType":5,"DriveHwVersion":5,"DriveFwVersion":24046,"DriveId":0},{"DriveCode":4,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":3,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":2,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":1,"DriveImageType":1,"DriveHwVersion":1,"DriveFwVersion":24042,"DriveId":0}]
        # f = open(filename, 'a')
        # json.dump(data, f)  #object
        # f.close()
        with open(filename, "r") as file:
            # for line in file:
            # # realine() read, "\n"
            # self.G_PhrozenFluiddRespondInfo(file.readline().strip())
            # #time.sleep(1)
            Lo_FileDataList = file.readlines()
            for line in Lo_FileDataList:
                # split = [i[:-1].split(',') for i in file.readlines()]
                # self.G_PhrozenFluiddRespondInfo(type(split))
                # self.G_PhrozenFluiddRespondInfo(split[1])
                # self.G_PhrozenFluiddRespondInfo(split[2])
                # self.G_PhrozenFluiddRespondInfo(split[3])
                # line_strip=line.strip()
                # self.G_PhrozenFluiddRespondInfo(line)
                # self.G_PhrozenFluiddRespondInfo("line.count=%d" % line.count)
                split = line.split(",")
                # self.G_PhrozenFluiddRespondInfo(type(split))
                # self.G_PhrozenFluiddRespondInfo("".join(split))
                # self.G_PhrozenFluiddRespondInfo(split[0])
                split[0] = split[0].strip()  # driver number
                split[1] = split[1].strip()  # hardware id
                split[2] = split[2].strip()  # firmware version
                split[3] = split[3].strip()  # image id
                split[4] = split[4].strip()  # online status

                # Vendor note (240617): image id=17set flag0,ifAMS,set flag1

                if split[0] == "16":
                    self.kaos_log("DEBUG", split[0], "SERIAL")
                    self.kaos_log("DEBUG", split[1], "SERIAL")
                    self.kaos_log("DEBUG", split[2], "SERIAL")
                    self.kaos_log("DEBUG", split[3], "SERIAL")
                    self.kaos_log("DEBUG", split[4], "SERIAL")
                    # line=("%d,%d,%d," % (HW_VERSION,,))
                    line_modify = (
                        split[0] + "," + "16" + "," + str(FW_VERSION) + "," + "16" + "," + "1"
                    )
                    self.kaos_log("DEBUG", line_modify, "SERIAL")
                    Lo_AllLine = Lo_AllLine + line_modify + "\r\n"  # 0x0D 0x0A
                elif split[0] == "1":
                    self.kaos_log("DEBUG", split[0], "SERIAL")
                    self.kaos_log("DEBUG", split[1], "SERIAL")
                    self.kaos_log("DEBUG", split[2], "SERIAL")
                    self.kaos_log("DEBUG", split[3], "SERIAL")
                    self.kaos_log("DEBUG", split[4], "SERIAL")
                    line_modify = split[0] + "," + "1" + "," + "00000" + "," + "1" + "," + "0"
                    self.kaos_log("DEBUG", line_modify, "SERIAL")
                    Lo_AllLine = Lo_AllLine + line_modify + "\r\n"  # 0x0D 0x0A
                elif split[0] == "2":
                    self.kaos_log("DEBUG", split[0], "SERIAL")
                    self.kaos_log("DEBUG", split[1], "SERIAL")
                    self.kaos_log("DEBUG", split[2], "SERIAL")
                    self.kaos_log("DEBUG", split[3], "SERIAL")
                    self.kaos_log("DEBUG", split[4], "SERIAL")
                    line_modify = split[0] + "," + "1" + "," + "00000" + "," + "1" + "," + "0"
                    self.kaos_log("DEBUG", line_modify, "SERIAL")
                    Lo_AllLine = Lo_AllLine + line_modify + "\r\n"  # 0x0D 0x0A
                elif split[0] == "3":
                    self.kaos_log("DEBUG", split[0], "SERIAL")
                    self.kaos_log("DEBUG", split[1], "SERIAL")
                    # [Translated vendor note] Vendor note (241030): P1 C1P1 C32, 132
                    # [Translated vendor note] 1: 1 2 3 4
                    # [Translated vendor note] 2: 5 6 7 8
                    # [Translated vendor note] 3: 9 10 11 12
                    # [Translated vendor note] 4: 13 14 15 16
                    # [Translated vendor note] 5: 17 18 19 20
                    # [Translated vendor note] 6: 21 22 23 24
                    # [Translated vendor note] 7: 25 26 27 28
                    # [Translated vendor note] 8: 29 30 31 32
                    # [Translated vendor note] filament change

                    self.kaos_log("DEBUG", split[2], "SERIAL")
                    self.kaos_log("DEBUG", split[3], "SERIAL")
                    self.kaos_log("DEBUG", split[4], "SERIAL")
                    line_modify = split[0] + "," + "1" + "," + "00000" + "," + "1" + "," + "0"
                    self.kaos_log("DEBUG", line_modify, "SERIAL")
                    Lo_AllLine = Lo_AllLine + line_modify + "\r\n"  # 0x0D 0x0A
                elif split[0] == "4":
                    self.kaos_log("DEBUG", split[0], "SERIAL")
                    self.kaos_log("DEBUG", split[1], "SERIAL")
                    self.kaos_log("DEBUG", split[2], "SERIAL")
                    self.kaos_log("DEBUG", split[3], "SERIAL")
                    self.kaos_log("DEBUG", split[4], "SERIAL")
                    line_modify = split[0] + "," + "1" + "," + "00000" + "," + "1" + "," + "0"
                    self.kaos_log("DEBUG", line_modify, "SERIAL")
                    Lo_AllLine = Lo_AllLine + line_modify + "\r\n"  # 0x0D 0x0A
                elif split[0] == "5":
                    self.kaos_log("DEBUG", split[0], "SERIAL")
                    self.kaos_log("DEBUG", split[1], "SERIAL")
                    self.kaos_log("DEBUG", split[2], "SERIAL")
                    self.kaos_log("DEBUG", split[3], "SERIAL")
                    self.kaos_log("DEBUG", split[4], "SERIAL")
                    line_modify = (
                        split[0] + "," + split[1] + "," + split[2] + "," + split[3] + "," + "1"
                    )
                    self.kaos_log("DEBUG", line_modify, "SERIAL")
                    Lo_AllLine = Lo_AllLine + line_modify + "\r\n"  # 0x0D 0x0A
                elif split[0] == "10":
                    self.kaos_log("DEBUG", split[0], "SERIAL")
                    self.kaos_log("DEBUG", split[1], "SERIAL")
                    self.kaos_log("DEBUG", split[2], "SERIAL")
                    self.kaos_log("DEBUG", split[3], "SERIAL")
                    self.kaos_log("DEBUG", split[4], "SERIAL")
                    line_modify = (
                        split[0] + "," + split[1] + "," + split[2] + "," + split[3] + "," + "1"
                    )
                    self.kaos_log("DEBUG", line_modify, "SERIAL")
                    Lo_AllLine = Lo_AllLine + line_modify + "\r\n"  # 0x0D 0x0A
                elif split[0] == "7":
                    self.kaos_log("DEBUG", split[0], "SERIAL")
                    self.kaos_log("DEBUG", split[1], "SERIAL")
                    self.kaos_log("DEBUG", split[2], "SERIAL")
                    self.kaos_log("DEBUG", split[3], "SERIAL")
                    self.kaos_log("DEBUG", split[4], "SERIAL")
                    line_modify = split[0] + "," + "7" + "," + "00000" + "," + "7" + "," + "0"
                    self.kaos_log("DEBUG", line_modify, "SERIAL")
                    Lo_AllLine = Lo_AllLine + line_modify + "\r\n"  # 0x0D 0x0A

                elif split[0] == "17":
                    self.kaos_log("DEBUG", split[0], "SERIAL")
                    self.kaos_log("DEBUG", split[1], "SERIAL")
                    self.kaos_log("DEBUG", split[2], "SERIAL")
                    self.kaos_log("DEBUG", split[3], "SERIAL")
                    self.kaos_log("DEBUG", split[4], "SERIAL")
                    # line=("%d,%d,%d," % (HW_VERSION,,))
                    line_modify = split[0] + "," + "18" + "," + "00000" + "," + "18" + "," + "0"
                    self.kaos_log("DEBUG", line_modify, "SERIAL")
                    Lo_AllLine = Lo_AllLine + line_modify + "\r\n"  # 0x0D 0x0A
                elif split[0] == "18":
                    self.kaos_log("DEBUG", split[0], "SERIAL")
                    self.kaos_log("DEBUG", split[1], "SERIAL")
                    self.kaos_log("DEBUG", split[2], "SERIAL")
                    self.kaos_log("DEBUG", split[3], "SERIAL")
                    self.kaos_log("DEBUG", split[4], "SERIAL")
                    # line=("%d,%d,%d," % (HW_VERSION,,))
                    line_modify = split[0] + "," + "18" + "," + "00000" + "," + "18" + "," + "0"
                    self.kaos_log("DEBUG", line_modify, "SERIAL")
                    Lo_AllLine = Lo_AllLine + line_modify + "\r\n"  # 0x0D 0x0A
                elif split[0] == "19":
                    self.kaos_log("DEBUG", split[0], "SERIAL")
                    self.kaos_log("DEBUG", split[1], "SERIAL")
                    self.kaos_log("DEBUG", split[2], "SERIAL")
                    self.kaos_log("DEBUG", split[3], "SERIAL")
                    self.kaos_log("DEBUG", split[4], "SERIAL")
                    # line=("%d,%d,%d," % (HW_VERSION,,))
                    line_modify = split[0] + "," + "18" + "," + "00000" + "," + "18" + "," + "0"
                    self.kaos_log("DEBUG", line_modify, "SERIAL")
                    Lo_AllLine = Lo_AllLine + line_modify + "\r\n"  # 0x0D 0x0A
                elif split[0] == "20":
                    self.kaos_log("DEBUG", split[0], "SERIAL")
                    self.kaos_log("DEBUG", split[1], "SERIAL")
                    self.kaos_log("DEBUG", split[2], "SERIAL")
                    self.kaos_log("DEBUG", split[3], "SERIAL")
                    self.kaos_log("DEBUG", split[4], "SERIAL")
                    # line=("%d,%d,%d," % (HW_VERSION,,))
                    line_modify = split[0] + "," + "18" + "," + "00000" + "," + "18" + "," + "0"
                    self.kaos_log("DEBUG", line_modify, "SERIAL")
                    Lo_AllLine = Lo_AllLine + line_modify + "\r\n"  # 0x0D 0x0A

                else:
                    Lo_AllLine = Lo_AllLine + line
        # self.G_PhrozenFluiddRespondInfo(Lo_AllLine)
        with open(filename, "w+") as file_w:
            file_w.write(Lo_AllLine)

        # self.G_PhrozenFluiddRespondInfo("PRZ_DEV_VER: %s" % FW_VERSION)
        # self.G_PhrozenFluiddRespondInfo("V-H'%s'-I'%s'-F'%s'" % (HW_VERSION,IMAGE_VERSION,FW_VERSION))

    def Cmds_PhrozenAdc(self, gcmd):
        self.kaos_log("DEBUG", "[(cmds.python)Cmds_PhrozenAdc]command=PRZ_ADC", "SERIAL")

        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        # readADC
        value, timestamp = self.G_ToolheadAdc.get_last_value()

        self.kaos_log(
            "DEBUG",
            "PRZ_ADC: read get ADC value %.6f (timestamp %.3f) fila_exist:%r"
            % (value, timestamp, self.G_ToolheadIfHaveFilaFlag),
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG", "self.G_ToolheadIfHaveFilaFlag=%d" % (self.G_ToolheadIfHaveFilaFlag), "SERIAL"
        )
        self.kaos_log(
            "DEBUG", "self.G_AMS1ErrorRestartCount=%d" % (self.G_AMS1ErrorRestartCount), "SERIAL"
        )

        self.kaos_log("DEBUG", "Current mode", "SERIAL")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.emit_mode(0, "unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.emit_mode(1, "MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.emit_mode(2, "MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.emit_mode(3, "RUNOUT")
        # [Translated vendor note] Vendor note (241030): P1 C1P1 C32, 132
        # [Translated vendor note] 1: 1 2 3 4
        # [Translated vendor note] 2: 5 6 7 8
        # [Translated vendor note] 3: 9 10 11 12
        # [Translated vendor note] 4: 13 14 15 16
        # [Translated vendor note] 5: 17 18 19 20
        # [Translated vendor note] 6: 21 22 23 24
        # [Translated vendor note] 7: 25 26 27 28
        # [Translated vendor note] 8: 29 30 31 32
        # [Translated vendor note] filament change

        else:
            self.emit_mode(-1, "error")

        self.kaos_log("DEBUG", "self.G_P0M3Flag=%d" % (self.G_P0M3Flag), "SERIAL")
        self.kaos_log("DEBUG", "self.G_KlipperIfPaused=%d" % (self.G_KlipperIfPaused), "SERIAL")
        self.kaos_log("DEBUG", "self.G_CancelFlag=%d" % (self.G_CancelFlag), "SERIAL")
        self.kaos_log(
            "DEBUG", "self.G_IfChangeFilaOngoing=%d" % (self.G_IfChangeFilaOngoing), "SERIAL"
        )
        self.kaos_log(
            "DEBUG", "self.G_SerialPort1OpenFlag=%d" % (self.G_SerialPort1OpenFlag), "SERIAL"
        )
        self.kaos_log(
            "DEBUG", "self.G_P0M2MAStartPrintFlag=%d" % (self.G_P0M2MAStartPrintFlag), "SERIAL"
        )
        self.kaos_log(
            "DEBUG", "self.G_M2MAModeResumeFlag=%d" % (self.G_M2MAModeResumeFlag), "SERIAL"
        )
        self.kaos_log(
            "DEBUG", "self.G_KlipperPrintStatus=%d" % (self.G_KlipperPrintStatus), "SERIAL"
        )
        self.kaos_log(
            "DEBUG", "self.G_SerialPort1OpenFlag=%d" % (self.G_SerialPort1OpenFlag), "SERIAL"
        )
        self.kaos_log(
            "DEBUG", "self.G_SerialPort2OpenFlag=%d" % (self.G_SerialPort2OpenFlag), "SERIAL"
        )
        self.kaos_log("DEBUG", "self.ManualCmdFlag=%d" % (self.ManualCmdFlag), "SERIAL")
        self.kaos_log(
            "DEBUG", "self.STM32ReprotPauseFlag=%d" % (self.STM32ReprotPauseFlag), "SERIAL"
        )
        self.kaos_log("DEBUG", "self.PG102Flag=%d" % (self.PG102Flag), "SERIAL")
        self.kaos_log("DEBUG", "self.G_IfInFilaBlockFlag=%d" % (self.G_IfInFilaBlockFlag), "SERIAL")
        self.kaos_log("DEBUG", "self.PG102DelayPauseFlag=%d" % (self.PG102DelayPauseFlag), "SERIAL")
        self.kaos_log("DEBUG", "self.G_KlipperQuickPause=%d" % (self.G_KlipperQuickPause), "SERIAL")
        self.kaos_log("DEBUG", "self.G_PauseToLCDString=%s" % (self.G_PauseToLCDString), "SERIAL")

        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")

        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")

        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()
        if self.G_SerialPort1OpenFlag == True:
            # Vendor note (240524): readAMS16HUB
            self.Cmds_AMSSerial1Send("AT+SB=0")
            self.kaos_log(
                "DEBUG",
                "serial port 1Sending command: AT+SB=0; get AMSmainboard,16HUBmainboard",
                "SERIAL",
            )
        if self.G_SerialPort2OpenFlag == True:
            # Vendor note (240524): readAMS16HUB
            self.Cmds_AMSSerial2Send("AT+SB=0")
            self.kaos_log(
                "DEBUG",
                "serial port 2Sending command: AT+SB=0; get AMSmainboard,16HUBmainboard",
                "SERIAL",
            )

        # Vendor note (240529): phrozen
        self.G_PhrozenFluiddRespondInfo(
            "V-H%s-I%s-F%s-SN1" % (HW_VERSION, IMAGE_VERSION, FW_VERSION)
        )

        # time.sleep(1)

        # self.Cmds_AMSSerial1Send("AT+SB=1")
        # self.G_PhrozenFluiddRespondInfo("Sending command: AT+SB=1; get AMS board status")

        # PRZ_PwrDownResumePrint
        try:
            self.kaos_log("DEBUG", "try", "SERIAL")
            # Vendor note (240530): write version to DriveCodeJson.dat

            # Vendor note (250724): read system image id to distinguish product/board/firmware variants
            # Vendor note (250724): read image id
            self.Cmds_GetImageId()
            if self.G_ImageId == 16:
                self.kaos_log(
                    "DEBUG", "Image ID==16: ARCO300-MKS-RK3328-STM32F407VET6-I16", "SERIAL"
                )
                with open("/home/mks/hdlDat/Phrozen_Dev.json", "r", encoding="utf-8") as file:
                    FileRead = file.read()
                    self.kaos_log("DEBUG", "read get json text file", "SERIAL")
                    # parseJSONdata
                    json_data = json.loads(FileRead)
                    self.kaos_log("DEBUG", "json_data['mode']=%d" % (json_data["mode"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc1']=%d" % (json_data["nc1"]), "SERIAL")
            elif self.G_ImageId == 31:
                self.kaos_log(
                    "DEBUG", "Image ID==31: ARCO300-phrozen-RK3308-STM32F407VET6-I31", "SERIAL"
                )
                with open("/home/prz/hdlDat/Phrozen_Dev.json", "r", encoding="utf-8") as file:
                    FileRead = file.read()
                    self.kaos_log("DEBUG", "read get json text file", "SERIAL")
                    # parseJSONdata
                    json_data = json.loads(FileRead)
                    self.kaos_log("DEBUG", "json_data['mode']=%d" % (json_data["mode"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc1']=%d" % (json_data["nc1"]), "SERIAL")
            elif self.G_ImageId == -1:
                self.kaos_log(
                    "DEBUG", "Image ID==-1, default: ARCO300-MKS-RK3328-STM32F407VET6-I16", "SERIAL"
                )
                with open("/home/mks/hdlDat/Phrozen_Dev.json", "r", encoding="utf-8") as file:
                    FileRead = file.read()
                    self.kaos_log("DEBUG", "read get json text file", "SERIAL")
                    # parseJSONdata
                    json_data = json.loads(FileRead)
                    self.kaos_log("DEBUG", "json_data['mode']=%d" % (json_data["mode"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc1']=%d" % (json_data["nc1"]), "SERIAL")
            else:
                self.kaos_log(
                    "DEBUG",
                    "Image ID could not be read, default: ARCO300-MKS-RK3328-STM32F407VET6-I16",
                    "SERIAL",
                )
                with open("/home/mks/hdlDat/Phrozen_Dev.json", "r", encoding="utf-8") as file:
                    FileRead = file.read()
                    self.kaos_log("DEBUG", "read get json text file", "SERIAL")
                    # parseJSONdata
                    json_data = json.loads(FileRead)
                    self.kaos_log("DEBUG", "json_data['mode']=%d" % (json_data["mode"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc1']=%d" % (json_data["nc1"]), "SERIAL")
        except:
            self.kaos_log("DEBUG", "except", "SERIAL")

    def Cmds_PhrozenBM1(self, gcmd):
        self.kaos_log(
            "DEBUG",
            "[(cmds.python)Cmds_PhrozenBM1]command='%s'" % (gcmd.get_commandline(),),
            "SERIAL",
        )

        self.kaos_log("DEBUG", "%s" % (gcmd.get_commandline(),), "SERIAL")

        # Vendor note (250327): filament changebefore,allowAMS
        self.ManualCmdFlag = True
        self.kaos_log("DEBUG", "self.ManualCmdFlag=True", "SERIAL")

        #  # Vendor note (250514): read json file for single-color refill config and channel-color mapping
        # #/home/prz/klipper/klippy/extras/phrozen_dev/serial-screen/test.json
        # self.Cmds_GetUartScreenCfg()

    def Cmds_PhrozenBM0(self, gcmd):
        self.kaos_log(
            "DEBUG",
            "[(cmds.python)Cmds_PhrozenBM0]command='%s'" % (gcmd.get_commandline(),),
            "SERIAL",
        )

        self.kaos_log("DEBUG", "%s" % (gcmd.get_commandline(),), "SERIAL")
        # Vendor note (250327): filament changebefore,allowAMS
        self.ManualCmdFlag = True
        self.kaos_log("DEBUG", "self.ManualCmdFlag=True", "SERIAL")

        #  # Vendor note (250514): read json file for single-color refill config and channel-color mapping
        # #/home/prz/klipper/klippy/extras/phrozen_dev/serial-screen/test.json
        # self.Cmds_GetUartScreenCfg()

    # PRZ_PRINT_START
    def Cmds_PrzPrintStart(self, gcmd):
        self.kaos_log(
            "DEBUG",
            "[(cmds.python)CmdsPrzPrintStart]command='%s'" % (gcmd.get_commandline(),),
            "SERIAL",
        )

        self.kaos_log("DEBUG", "%s" % (gcmd.get_commandline(),), "SERIAL")

        # # Vendor note (250514): read json file for single-color refill config and channel-color mapping
        # #/home/prz/klipper/klippy/extras/phrozen_dev/serial-screen/test.json
        # self.Cmds_GetUartScreenCfg()

    def Cmds_HomingOverrideEnd(self, gcmd):
        self.kaos_log(
            "DEBUG",
            "[(cmds.python)Cmds_HomingOverrideEnd]command='%s'" % (gcmd.get_commandline(),),
            "SERIAL",
        )

        self.kaos_log("DEBUG", "%s" % (gcmd.get_commandline(),), "SERIAL")

        # # Vendor note (250514): read json file for single-color refill config and channel-color mapping
        # #/home/prz/klipper/klippy/extras/phrozen_dev/serial-screen/test.json
        # self.Cmds_GetUartScreenCfg()

    def Cmds_PrzPrintEnd(self, gcmd):
        self.kaos_log(
            "DEBUG",
            "[(cmds.python)Cmds_PrzPrintEnd]command='%s'" % (gcmd.get_commandline(),),
            "SERIAL",
        )

        self.kaos_log("DEBUG", "%s" % (gcmd.get_commandline(),), "SERIAL")

    def Cmds_PrintMode(self, mode):
        self.kaos_log(
            "DEBUG",
            "[(cmds.python)Cmds_PrintMode]Sending command: self.G_AMSDeviceWorkMode=%d"
            % self.G_AMSDeviceWorkMode,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG", "[(cmds.python)Cmds_PrintMode]Sending command: mode=%d" % mode, "SERIAL"
        )

        try:
            self.kaos_log("DEBUG", "try", "SERIAL")
            Phrozen_Dev = {
                "mode": self.G_AMSDeviceWorkMode,
                "nc1": self.G_ChangeChannelTimeoutOldChan,
                "nc2": self.G_ChangeChannelTimeoutNewChan,
                "nc3": 0,
                "nc4": 0,
                "nc5": 0,
                "nc6": 0,
            }

            # Vendor note (250724): read system image id to distinguish product/board/firmware variants
            # Vendor note (250724): read image id
            self.Cmds_GetImageId()
            if self.G_ImageId == 16:
                self.kaos_log(
                    "DEBUG", "Image ID==16: ARCO300-MKS-RK3328-STM32F407VET6-I16", "SERIAL"
                )
                with open("/home/mks/hdlDat/Phrozen_Dev.json", "w") as file:
                    json.dump(Phrozen_Dev, file)
                    self.kaos_log("DEBUG", "write json text file", "SERIAL")
                with open("/home/mks/hdlDat/Phrozen_Dev.json", "r", encoding="utf-8") as file:
                    FileRead = file.read()
                    self.kaos_log("DEBUG", "read get json text file", "SERIAL")
                    # parseJSONdata
                    json_data = json.loads(FileRead)
                    self.kaos_log("DEBUG", "json_data['mode']=%d" % (json_data["mode"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc1']=%d" % (json_data["nc1"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc2']=%d" % (json_data["nc2"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc3']=%d" % (json_data["nc3"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc4']=%d" % (json_data["nc4"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc5']=%d" % (json_data["nc5"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc6']=%d" % (json_data["nc6"]), "SERIAL")
            elif self.G_ImageId == 31:
                self.kaos_log(
                    "DEBUG", "Image ID==31: ARCO300-phrozen-RK3308-STM32F407VET6-I31", "SERIAL"
                )
                with open("/home/prz/hdlDat/Phrozen_Dev.json", "w") as file:
                    json.dump(Phrozen_Dev, file)
                    self.kaos_log("DEBUG", "write json text file", "SERIAL")
                with open("/home/prz/hdlDat/Phrozen_Dev.json", "r", encoding="utf-8") as file:
                    FileRead = file.read()
                    self.kaos_log("DEBUG", "read get json text file", "SERIAL")
                    # parseJSONdata
                    json_data = json.loads(FileRead)
                    self.kaos_log("DEBUG", "json_data['mode']=%d" % (json_data["mode"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc1']=%d" % (json_data["nc1"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc2']=%d" % (json_data["nc2"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc3']=%d" % (json_data["nc3"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc4']=%d" % (json_data["nc4"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc5']=%d" % (json_data["nc5"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc6']=%d" % (json_data["nc6"]), "SERIAL")
            elif self.G_ImageId == -1:
                self.kaos_log(
                    "DEBUG", "Image ID==-1, default: ARCO300-MKS-RK3328-STM32F407VET6-I16", "SERIAL"
                )
                with open("/home/mks/hdlDat/Phrozen_Dev.json", "w") as file:
                    json.dump(Phrozen_Dev, file)
                    self.kaos_log("DEBUG", "write json text file", "SERIAL")
                with open("/home/mks/hdlDat/Phrozen_Dev.json", "r", encoding="utf-8") as file:
                    FileRead = file.read()
                    self.kaos_log("DEBUG", "read get json text file", "SERIAL")
                    # parseJSONdata
                    json_data = json.loads(FileRead)
                    self.kaos_log("DEBUG", "json_data['mode']=%d" % (json_data["mode"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc1']=%d" % (json_data["nc1"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc2']=%d" % (json_data["nc2"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc3']=%d" % (json_data["nc3"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc4']=%d" % (json_data["nc4"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc5']=%d" % (json_data["nc5"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc6']=%d" % (json_data["nc6"]), "SERIAL")
            else:
                self.kaos_log(
                    "DEBUG",
                    "Image ID could not be read, default: ARCO300-MKS-RK3328-STM32F407VET6-I16",
                    "SERIAL",
                )
                with open("/home/mks/hdlDat/Phrozen_Dev.json", "w") as file:
                    json.dump(Phrozen_Dev, file)
                    self.kaos_log("DEBUG", "write json text file", "SERIAL")
                with open("/home/mks/hdlDat/Phrozen_Dev.json", "r", encoding="utf-8") as file:
                    FileRead = file.read()
                    self.kaos_log("DEBUG", "read get json text file", "SERIAL")
                    # parseJSONdata
                    json_data = json.loads(FileRead)
                    self.kaos_log("DEBUG", "json_data['mode']=%d" % (json_data["mode"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc1']=%d" % (json_data["nc1"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc2']=%d" % (json_data["nc2"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc3']=%d" % (json_data["nc3"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc4']=%d" % (json_data["nc4"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc5']=%d" % (json_data["nc5"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc6']=%d" % (json_data["nc6"]), "SERIAL")
        except:
            self.kaos_log("DEBUG", "except", "SERIAL")

    # PRZ_RESTORE
    def Cmds_PrzATRestore(self, gcmd):
        self.kaos_log("DEBUG", "[(cmds.python)Cmds_PrzATRestore]", "SERIAL")
        # [Translated vendor note] Vendor note (241030): P1 C1P1 C32, 132
        # [Translated vendor note] 1: 1 2 3 4
        # [Translated vendor note] 2: 5 6 7 8
        # [Translated vendor note] 3: 9 10 11 12
        # [Translated vendor note] 4: 13 14 15 16
        # [Translated vendor note] 5: 17 18 19 20
        # [Translated vendor note] 6: 21 22 23 24
        # [Translated vendor note] 7: 25 26 27 28
        # [Translated vendor note] 8: 29 30 31 32
        # [Translated vendor note] filament change

        # self.G_PhrozenFluiddRespondInfo("%s" % (gcmd.get_commandline(),))

        self.kaos_log("DEBUG", "Current mode", "SERIAL")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.emit_mode(0, "unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.emit_mode(1, "MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.emit_mode(2, "MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.emit_mode(3, "RUNOUT")
        else:
            self.emit_mode(-1, "error")

        # PRZ_PwrDownResumePrint
        try:
            self.kaos_log("DEBUG", "try", "SERIAL")

            # Vendor note (250724): read system image id to distinguish product/board/firmware variants
            # Vendor note (250724): read image id
            self.Cmds_GetImageId()
            if self.G_ImageId == 16:
                self.kaos_log(
                    "DEBUG", "Image ID==16: ARCO300-MKS-RK3328-STM32F407VET6-I16", "SERIAL"
                )
                with open("/home/mks/hdlDat/Phrozen_Dev.json", "r", encoding="utf-8") as file:
                    FileRead = file.read()
                    self.kaos_log("DEBUG", "read get json text file", "SERIAL")
                    # parseJSONdata
                    json_data = json.loads(FileRead)
                    self.kaos_log("DEBUG", "json_data['mode']=%d" % (json_data["mode"]), "SERIAL")
                    self.G_AMSDeviceWorkMode = json_data["mode"]
                    self.kaos_log(
                        "DEBUG",
                        "self.G_AMSDeviceWorkMode=%d" % (self.G_AMSDeviceWorkMode),
                        "SERIAL",
                    )
                    self.kaos_log("DEBUG", "json_data['nc1']=%d" % (json_data["nc1"]), "SERIAL")
                    self.G_ChangeChannelTimeoutOldChan = json_data["nc1"]
                    self.kaos_log(
                        "DEBUG",
                        "self.G_ChangeChannelTimeoutOldChan=%d"
                        % (self.G_ChangeChannelTimeoutOldChan),
                        "SERIAL",
                    )
                    self.kaos_log("DEBUG", "json_data['nc2']=%d" % (json_data["nc2"]), "SERIAL")
                    self.G_ChangeChannelTimeoutNewChan = json_data["nc2"]
                    self.kaos_log(
                        "DEBUG",
                        "self.G_ChangeChannelTimeoutNewChan=%d"
                        % (self.G_ChangeChannelTimeoutNewChan),
                        "SERIAL",
                    )
                    self.kaos_log("DEBUG", "json_data['nc3']=%d" % (json_data["nc3"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc4']=%d" % (json_data["nc4"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc5']=%d" % (json_data["nc5"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc6']=%d" % (json_data["nc6"]), "SERIAL")
            elif self.G_ImageId == 31:
                self.kaos_log(
                    "DEBUG", "Image ID==31: ARCO300-phrozen-RK3308-STM32F407VET6-I31", "SERIAL"
                )
                with open("/home/prz/hdlDat/Phrozen_Dev.json", "r", encoding="utf-8") as file:
                    FileRead = file.read()
                    self.kaos_log("DEBUG", "read get json text file", "SERIAL")
                    # parseJSONdata
                    json_data = json.loads(FileRead)
                    self.kaos_log("DEBUG", "json_data['mode']=%d" % (json_data["mode"]), "SERIAL")
                    self.G_AMSDeviceWorkMode = json_data["mode"]
                    self.kaos_log(
                        "DEBUG",
                        "self.G_AMSDeviceWorkMode=%d" % (self.G_AMSDeviceWorkMode),
                        "SERIAL",
                    )
                    self.kaos_log("DEBUG", "json_data['nc1']=%d" % (json_data["nc1"]), "SERIAL")
                    self.G_ChangeChannelTimeoutOldChan = json_data["nc1"]
                    self.kaos_log(
                        "DEBUG",
                        "self.G_ChangeChannelTimeoutOldChan=%d"
                        % (self.G_ChangeChannelTimeoutOldChan),
                        "SERIAL",
                    )
                    self.kaos_log("DEBUG", "json_data['nc2']=%d" % (json_data["nc2"]), "SERIAL")
                    self.G_ChangeChannelTimeoutNewChan = json_data["nc2"]
                    self.kaos_log(
                        "DEBUG",
                        "self.G_ChangeChannelTimeoutNewChan=%d"
                        % (self.G_ChangeChannelTimeoutNewChan),
                        "SERIAL",
                    )
                    self.kaos_log("DEBUG", "json_data['nc3']=%d" % (json_data["nc3"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc4']=%d" % (json_data["nc4"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc5']=%d" % (json_data["nc5"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc6']=%d" % (json_data["nc6"]), "SERIAL")
            elif self.G_ImageId == -1:
                self.kaos_log(
                    "DEBUG", "Image ID==-1, default: ARCO300-MKS-RK3328-STM32F407VET6-I16", "SERIAL"
                )
                with open("/home/mks/hdlDat/Phrozen_Dev.json", "r", encoding="utf-8") as file:
                    FileRead = file.read()
                    self.kaos_log("DEBUG", "read get json text file", "SERIAL")
                    # parseJSONdata
                    json_data = json.loads(FileRead)
                    self.kaos_log("DEBUG", "json_data['mode']=%d" % (json_data["mode"]), "SERIAL")
                    self.G_AMSDeviceWorkMode = json_data["mode"]
                    self.kaos_log(
                        "DEBUG",
                        "self.G_AMSDeviceWorkMode=%d" % (self.G_AMSDeviceWorkMode),
                        "SERIAL",
                    )
                    self.kaos_log("DEBUG", "json_data['nc1']=%d" % (json_data["nc1"]), "SERIAL")
                    self.G_ChangeChannelTimeoutOldChan = json_data["nc1"]
                    self.kaos_log(
                        "DEBUG",
                        "self.G_ChangeChannelTimeoutOldChan=%d"
                        % (self.G_ChangeChannelTimeoutOldChan),
                        "SERIAL",
                    )
                    self.kaos_log("DEBUG", "json_data['nc2']=%d" % (json_data["nc2"]), "SERIAL")
                    self.G_ChangeChannelTimeoutNewChan = json_data["nc2"]
                    self.kaos_log(
                        "DEBUG",
                        "self.G_ChangeChannelTimeoutNewChan=%d"
                        % (self.G_ChangeChannelTimeoutNewChan),
                        "SERIAL",
                    )
                    self.kaos_log("DEBUG", "json_data['nc3']=%d" % (json_data["nc3"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc4']=%d" % (json_data["nc4"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc5']=%d" % (json_data["nc5"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc6']=%d" % (json_data["nc6"]), "SERIAL")
            else:
                self.kaos_log(
                    "DEBUG",
                    "Image ID could not be read, default: ARCO300-MKS-RK3328-STM32F407VET6-I16",
                    "SERIAL",
                )
                with open("/home/mks/hdlDat/Phrozen_Dev.json", "r", encoding="utf-8") as file:
                    FileRead = file.read()
                    self.kaos_log("DEBUG", "read get json text file", "SERIAL")
                    # parseJSONdata
                    json_data = json.loads(FileRead)
                    self.kaos_log("DEBUG", "json_data['mode']=%d" % (json_data["mode"]), "SERIAL")
                    self.G_AMSDeviceWorkMode = json_data["mode"]
                    self.kaos_log(
                        "DEBUG",
                        "self.G_AMSDeviceWorkMode=%d" % (self.G_AMSDeviceWorkMode),
                        "SERIAL",
                    )
                    self.kaos_log("DEBUG", "json_data['nc1']=%d" % (json_data["nc1"]), "SERIAL")
                    self.G_ChangeChannelTimeoutOldChan = json_data["nc1"]
                    self.kaos_log(
                        "DEBUG",
                        "self.G_ChangeChannelTimeoutOldChan=%d"
                        % (self.G_ChangeChannelTimeoutOldChan),
                        "SERIAL",
                    )
                    self.kaos_log("DEBUG", "json_data['nc2']=%d" % (json_data["nc2"]), "SERIAL")
                    self.G_ChangeChannelTimeoutNewChan = json_data["nc2"]
                    self.kaos_log(
                        "DEBUG",
                        "self.G_ChangeChannelTimeoutNewChan=%d"
                        % (self.G_ChangeChannelTimeoutNewChan),
                        "SERIAL",
                    )
                    self.kaos_log("DEBUG", "json_data['nc3']=%d" % (json_data["nc3"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc4']=%d" % (json_data["nc4"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc5']=%d" % (json_data["nc5"]), "SERIAL")
                    self.kaos_log("DEBUG", "json_data['nc6']=%d" % (json_data["nc6"]), "SERIAL")

            try:
                self.kaos_log(
                    "DEBUG", "[(cmds.py)Cmds_PrzATRestore]Reinitializing serial port 1", "SERIAL"
                )
                self.G_SerialPort1Obj = serial.Serial(
                    self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3
                )
                # serial port opened successfully
                if self.G_SerialPort1Obj is not None:
                    if self.G_SerialPort1Obj.is_open:
                        self.G_SerialPort1OpenFlag = True
                        self.kaos_log("DEBUG", "Reinitializing serial port 1 successful", "SERIAL")
                        # Vendor note (231213): open serial port
                        self.G_SerialPort1Obj.flushInput()  # clean serial write cache
                        self.G_SerialPort1Obj.flush()
                        self.kaos_log("DEBUG", "Serial port 1 buffers cleared", "SERIAL")
                        self.kaos_log("DEBUG", "Re-registering serial port 1 callback", "SERIAL")
                        # [Translated vendor note] Vendor note (241030): P1 C1P1 C32, 132
                        # [Translated vendor note] 1: 1 2 3 4
                        # [Translated vendor note] 2: 5 6 7 8
                        # [Translated vendor note] 3: 9 10 11 12
                        # [Translated vendor note] 4: 13 14 15 16
                        # [Translated vendor note] 5: 17 18 19 20
                        # [Translated vendor note] 6: 21 22 23 24
                        # [Translated vendor note] 7: 25 26 27 28
                        # [Translated vendor note] 8: 29 30 31 32
                        # [Translated vendor note] filament change

                        self.G_SerialPort1RecvTimmer = self.G_PhrozenReactor.register_timer(
                            self.Device_TimmerUart1Recv, self.G_PhrozenReactor.NOW
                        )
            except:
                self.kaos_log(
                    "DEBUG",
                    "Unable to open tty1. Check the USB connection or try rebooting.",
                    "SERIAL",
                )
                self.G_SerialPort1OpenFlag = False

            try:
                self.kaos_log(
                    "DEBUG", "[(cmds.py)Cmds_PrzATRestore]Reinitializing serial port 2", "SERIAL"
                )
                self.G_SerialPort2Obj = serial.Serial(
                    self.G_Serialport2Define, SERIAL_PORT_BAUD, timeout=3
                )
                # serial port opened successfully
                if self.G_SerialPort2Obj is not None:
                    if self.G_SerialPort2Obj.is_open:
                        self.G_SerialPort2OpenFlag = True
                        self.kaos_log("DEBUG", "Reinitializing serial port 2 successful", "SERIAL")
                        self.G_SerialPort2Obj.flushInput()  # clean serial write cache
                        self.G_SerialPort2Obj.flush()
                        self.kaos_log("DEBUG", "Serial port 2 buffers cleared", "SERIAL")
                        self.kaos_log("DEBUG", "Re-registering serial port 2 callback", "SERIAL")
                        self.G_SerialPort2RecvTimmer = self.G_PhrozenReactor.register_timer(
                            self.Device_TimmerUart2Recv, self.G_PhrozenReactor.NOW
                        )
            except:
                self.kaos_log(
                    "DEBUG",
                    "Unable to open tty2. Check the USB connection or try rebooting.",
                    "SERIAL",
                )
                self.G_SerialPort2OpenFlag = False

            # Vendor note (250619): check if AMS reconnected successfully
            self.Cmds_USBConnectErrorCheck()
            if self.G_SerialPort1OpenFlag == True:
                self.kaos_log("DEBUG", "Serial port 1 sending command: AT+RESTORE", "SERIAL")
                self.Cmds_AMSSerial1Send("AT+RESTORE")

            if self.G_SerialPort2OpenFlag == True:
                self.kaos_log("DEBUG", "Serial port 2 sending command: AT+RESTORE", "SERIAL")
                self.Cmds_AMSSerial2Send("AT+RESTORE")

            self.G_ProzenToolhead.dwell(2)

            self.kaos_log("DEBUG", "Current mode", "SERIAL")
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                self.emit_mode(0, "unkown")

            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
                self.emit_mode(1, "MC")
                # Vendor note (250607): print status: 1=retracting; 2=feeding; 3=printing; 4=paused
                # self.G_KlipperPrintStatus = 3
                if self.G_SerialPort1OpenFlag == False and self.G_SerialPort2OpenFlag == False:
                    self.kaos_log("DEBUG", "noneconnectedAMS, pause", "SERIAL")
                    if self.G_KlipperInPausing == False:
                        self.kaos_log(
                            "DEBUG", "No pause is in progress; a new pause is allowed", "SERIAL"
                        )
                        self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                        self.G_KlipperIfPaused = True
                        # self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                        self.STM32ReprotPauseFlag = 1
                        # Vendor note (231202): P1 C?auto filament change,if1,if,continue1start
                        self.G_ChangeChannelFirstFilaFlag = True
                        self.G_PhrozenFluiddRespondInfo(
                            "+PAUSE:g,%d,%d"
                            % (
                                self.G_ChangeChannelTimeoutOldChan,
                                self.G_ChangeChannelTimeoutNewChan,
                            )
                        )
                        self.G_PauseToLCDString = "+PAUSE:g,%d,%d" % (
                            self.G_ChangeChannelTimeoutOldChan,
                            self.G_ChangeChannelTimeoutNewChan,
                        )
                    else:
                        self.kaos_log(
                            "DEBUG",
                            "A pause is already in progress; a new pause is not allowed",
                            "SERIAL",
                        )

            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                self.emit_mode(2, "MA")
                self.G_P0M2MAStartPrintFlag = 1
                # self.G_ToolheadFirstInputFila=False
                # self.P0M3FilaRunoutSpittingFinished=True
                if self.G_ToolheadIfHaveFilaFlag:
                    self.kaos_log("DEBUG", "has filament, canprinting", "SERIAL")
                    # Vendor note (250607): print status: 1=retracting; 2=feeding; 3=printing; 4=paused
                    self.G_KlipperPrintStatus = 3
                    # Vendor note (250522): self.G_PhrozenFluiddRespondInfo("[PURGE] External macro PG108")
                    # Vendor note (251120): entering purge, set flag to prevent PG108 purge Hall-sensor pause at purge zone;
                    self.G_PG108Ingoing = 1
                    command_string = """
                        # PG108
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.G_PG108Ingoing = 0
                    self.kaos_log(
                        "DEBUG",
                        "[PURGE] External macro PG108; command_string='%s'" % command_string,
                        "SERIAL",
                    )

                    # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("can resume, STM32printing report error")
                    self.G_KlipperQuickPause = True
                    # # Vendor note (250427): # if self.G_SerialPort1OpenFlag == True:
                    #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                    #     self.G_PhrozenFluiddRespondInfo("serial port 1-AMSfinishtimingbuffer-full time")
                    # if self.G_SerialPort2OpenFlag == True:
                    #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                    #     self.G_PhrozenFluiddRespondInfo("serial port 2-AMSfinishtimingbuffer-full time")
                    # #self.G_ProzenToolhead.dwell(1.5)
                else:
                    self.kaos_log("DEBUG", "no filament, need to pause", "SERIAL")
                    # self.G_PhrozenFluiddRespondInfo("[PRINT] External macro RESUME")
                    # command = """
                    # RESUME
                    # """
                    # self.G_PhrozenGCode.run_script_from_command(command)
                    # self.G_PhrozenFluiddRespondInfo("[PRINT] Calling macro command=%s" % (command))
                    # Vendor note (240125): encapsulated function
                    self.Cmds_PhrozenKlipperResumeCommon()

                    # Vendor note (250522): self.G_PhrozenFluiddRespondInfo("[FIRMWARE] External macro PG109 heat-up")
                    command_string = """
                        # PG109
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.kaos_log(
                        "DEBUG",
                        "[FIRMWARE] External macro PG109 heat-up; command_string='%s'"
                        % command_string,
                        "SERIAL",
                    )

                    if self.G_KlipperInPausing == False:
                        self.kaos_log(
                            "DEBUG", "No pause is in progress; a new pause is allowed", "SERIAL"
                        )
                        # Vendor note (250607): #self.G_PhrozenFluiddRespondInfo("Quick pause enabled")
                        # self.G_KlipperQuickPause = True
                        # klipper active pause
                        self.Cmds_PhrozenKlipperPauseMAToSTM32(None)
                    else:
                        self.kaos_log(
                            "DEBUG",
                            "A pause is already in progress; a new pause is not allowed",
                            "SERIAL",
                        )

                    self.G_PhrozenFluiddRespondInfo(
                        "+PAUSE:4,%d,%d"
                        % (
                            self.G_ChangeChannelTimeoutOldChan,
                            self.G_ChangeChannelTimeoutNewChan,
                        )
                    )

                    # if self.G_SerialPort1OpenFlag==True or self.G_SerialPort2OpenFlag==True:
                    #     self.G_PhrozenFluiddRespondInfo("toolhead no filament, has AMS multi-material, P8 feed")
                    #     # Vendor note (241106): #     self.G_P0M2MAStartPrintFlag=0

                    #     # Vendor note (250522): allowM3detect
                    #     self.G_IfChangeFilaOngoing = True

                    #     # Vendor note (241106): #     self.Cmds_CmdP8(gcmd)
                    #     # Vendor note (241106): toolhead feed successful
                    #     if self.G_P0M2MAStartPrintFlag==1:
                    #         # Vendor note (250607): #         self.G_PhrozenFluiddRespondInfo("can resume, STM32printing report error")
                    #         self.G_KlipperQuickPause = True
                    #         self.G_PhrozenFluiddRespondInfo("toolhead has filament, resume")
                    #         # Vendor note (240125): encapsulated function
                    #         self.Cmds_PhrozenKlipperResumeCommon()
                    #     else:
                    #         self.G_KlipperQuickPause = False
                    #         self.G_PhrozenFluiddRespondInfo("Toolhead no filament, refill, continue pausing")
                    #         if self.G_KlipperIfPaused == False:
                    #             self.Cmds_PhrozenKlipperPauseMAToSTM32(None)
                    #             self.G_KlipperIfPaused=True
                    #             #self.emit_protocol("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                    #             self.emit_protocol("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    #         else:
                    #             self.G_PhrozenFluiddRespondInfo("already pause, do notpause")
                    # else:
                    #     self.G_KlipperQuickPause = False
                    #     self.G_PhrozenFluiddRespondInfo("Toolhead no filament, no AMS, continue pausing")
                    #     self.Cmds_PhrozenKlipperPauseMAToSTM32(None)
                    #     #no filament, continue pausing
                    #     self.G_KlipperIfPaused=True
                    #     # Vendor note (250521): AMS multi-material present
                    #     #if self.G_AMSDevice1IfNormal==True or self.G_AMSDevice2IfNormal==True:
                    #     self.emit_protocol("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    #     #else:
                    #     #self.emit_protocol("+PAUSE:b,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))

            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                self.emit_mode(3, "RUNOUT")
                self.G_P0M3Flag = True
                # self.G_ToolheadFirstInputFila = True
                # Vendor note (240415): filament,
                # self.G_P0M3ToolheadHaveFilaNotSpittingFlag = True
                # self.P0M3FilaRunoutSpittingFinished==True:#complete

                if self.G_ToolheadIfHaveFilaFlag:
                    self.kaos_log("DEBUG", "has filament, canprinting", "SERIAL")
                    self.kaos_log("DEBUG", "[PURGE] External macro PG108", "SERIAL")
                    # Vendor note (251120): entering purge, set flag to prevent PG108 purge Hall-sensor pause at purge zone;
                    self.G_PG108Ingoing = 1
                    command_string = """
                        # PG108
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.G_PG108Ingoing = 0
                    self.kaos_log(
                        "DEBUG",
                        "[PURGE] External macro PG108; command_string='%s'" % command_string,
                        "SERIAL",
                    )

                    # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("can resume, STM32printing report error")
                    # self.G_KlipperQuickPause = True
                    # # Vendor note (250427): # if self.G_SerialPort1OpenFlag == True:
                    #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                    #     self.G_PhrozenFluiddRespondInfo("serial port 1-AMSfinishtimingbuffer-full time")
                    # if self.G_SerialPort2OpenFlag == True:
                    #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                    #     self.G_PhrozenFluiddRespondInfo("serial port 2-AMSfinishtimingbuffer-full time")
                    # #self.G_ProzenToolhead.dwell(1.5)
                else:
                    self.kaos_log("DEBUG", "no filament, need to pause", "SERIAL")
                    # self.G_PhrozenFluiddRespondInfo("[PRINT] External macro RESUME")
                    # command = """
                    # RESUME
                    # """
                    # self.G_PhrozenGCode.run_script_from_command(command)
                    # self.G_PhrozenFluiddRespondInfo("[PRINT] Calling macro command=%s" % (command))
                    # Vendor note (240125): encapsulated function
                    self.Cmds_PhrozenKlipperResumeCommon()

                    # Vendor note (250522): self.G_PhrozenFluiddRespondInfo("[FIRMWARE] External macro PG109 heat-up")
                    command_string = """
                        # PG109
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.kaos_log(
                        "DEBUG",
                        "[FIRMWARE] External macro PG109 heat-up; command_string='%s'"
                        % command_string,
                        "SERIAL",
                    )

                    if self.G_KlipperInPausing == False:
                        self.kaos_log(
                            "DEBUG", "No pause is in progress; a new pause is allowed", "SERIAL"
                        )
                        # Vendor note (250607): #self.G_PhrozenFluiddRespondInfo("Quick pause enabled")
                        # self.G_KlipperQuickPause = True
                        # klipper active pause
                        self.Cmds_PhrozenKlipperPauseMAToSTM32(None)
                    else:
                        self.kaos_log(
                            "DEBUG",
                            "A pause is already in progress; a new pause is not allowed",
                            "SERIAL",
                        )

                    self.G_PhrozenFluiddRespondInfo(
                        "+PAUSE:b,%d,%d"
                        % (
                            self.G_ChangeChannelTimeoutOldChan,
                            self.G_ChangeChannelTimeoutNewChan,
                        )
                    )

                    # if self.G_SerialPort1OpenFlag==True or self.G_SerialPort2OpenFlag==True:
                    #     self.G_PhrozenFluiddRespondInfo("toolhead no filament, has AMS multi-material, P8 feed")
                    #     # Vendor note (241106): #     self.G_P0M2MAStartPrintFlag=0

                    #     # Vendor note (250522): allowM3detect
                    #     self.G_IfChangeFilaOngoing = True

                    #     # Vendor note (241106): #     self.Cmds_CmdP8(gcmd)
                    #     # Vendor note (241106): toolhead feed successful
                    #     if self.G_P0M2MAStartPrintFlag==1:
                    #         # Vendor note (250607): #         self.G_PhrozenFluiddRespondInfo("can resume, STM32printing report error")
                    #         self.G_KlipperQuickPause = True
                    #         self.G_PhrozenFluiddRespondInfo("toolhead has filament, resume")
                    #         # Vendor note (240125): encapsulated function
                    #         self.Cmds_PhrozenKlipperResumeCommon()
                    #     else:
                    #         self.G_KlipperQuickPause = False
                    #         self.G_PhrozenFluiddRespondInfo("Toolhead no filament, refill, continue pausing")
                    #         if self.G_KlipperIfPaused == False:
                    #             self.Cmds_PhrozenKlipperPauseMAToSTM32(None)
                    #             self.G_KlipperIfPaused=True
                    #             #self.emit_protocol("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                    #             self.emit_protocol("+PAUSE:b,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    #         else:
                    #             self.G_PhrozenFluiddRespondInfo("already pause, do notpause")
                    # else:
                    #     self.G_KlipperQuickPause = False
                    #     self.G_PhrozenFluiddRespondInfo("Toolhead no filament, no AMS, continue pausing")
                    #     self.Cmds_PhrozenKlipperPauseMAToSTM32(None)
                    #     #no filament, continue pausing
                    #     self.G_KlipperIfPaused=True
                    #     # Vendor note (250521): AMS multi-material present
                    #     #if self.G_AMSDevice1IfNormal==True or self.G_AMSDevice2IfNormal==True:
                    #     self.emit_protocol("+PAUSE:b,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    #     #else:
                    #     #self.emit_protocol("+PAUSE:b,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))

            else:
                self.emit_mode(-1, "error")

        except:
            self.kaos_log("DEBUG", "except", "SERIAL")

    def Cmds_PrzATIdle(self, gcmd):
        self.kaos_log(
            "DEBUG",
            "[(cmds.python)Cmds_PrzATIdle]command='%s'" % (gcmd.get_commandline(),),
            "SERIAL",
        )

        self.kaos_log("DEBUG", "%s" % (gcmd.get_commandline(),), "SERIAL")
        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()
        if self.G_SerialPort1OpenFlag == True:
            self.Cmds_AMSSerial1Send("AT+IDLE")
            self.kaos_log("DEBUG", "Serial port 1 sending command: AT+IDLE", "SERIAL")
        if self.G_SerialPort2OpenFlag == True:
            self.Cmds_AMSSerial2Send("AT+IDLE")
            self.kaos_log("DEBUG", "Serial port 2 sending command: AT+IDLE", "SERIAL")

    # Function: retry filament feeding in MA mode
    # Parameters: gcmd (G-code command context)
    # Notes: retries feed with timeout/attempt guards to recover from transient runout
    def Cmds_MARetryInFila(self, gcmd):
        self.kaos_log("DEBUG", "[(cmds.python)Cmds_MARetryInFila]", "SERIAL")

        self.G_IfChangeFilaOngoing = True

        # Vendor note (250522): self.G_PhrozenFluiddRespondInfo("[FIRMWARE] External macro PG109 heat-up")
        command_string = """
            # PG109
            """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.kaos_log(
            "DEBUG",
            "[FIRMWARE] External macro PG109 heat-up; command_string='%s'" % command_string,
            "SERIAL",
        )
        self.IfDoPG102Flag = True

        # Vendor note (231228): stm32executeFA,detectfilamentstart
        # set flaglabel
        Lo_ChangeChannelIfSuccess = False
        # Vendor note (250607): print status: 1=retracting; 2=feeding; 3=printing; 4=paused
        self.G_KlipperPrintStatus = 2
        # Vendor note (20231013): time
        # Vendor note (231114): printer.cfgconfigfilament changetime,timeout
        # loopdetect2filament
        for i in range(CHANGE_CHANNEL_WAIT_TIMEOUT + 50):  # 130
            # self.G_XBasePosition+=2
            # self.G_YBasePosition+=2
            # Vendor note (231202): ifSTM32,needklipper pause
            if self.STM32ReprotPauseFlag == 1:
                self.G_ChangeChannelFirstFilaFlag = True
                self.kaos_log("DEBUG", ", stm32 move up report pause", "SERIAL")
                Lo_ChangeChannelIfSuccess = False

                Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
                self.kaos_log(
                    "DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL"
                )
                self.kaos_log(
                    "DEBUG",
                    "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"],
                    "SERIAL",
                )
                # // current-Lo_PauseStatus='{'is_paused': True}'
                if Lo_PauseStatus["is_paused"] == True:
                    self.kaos_log("DEBUG", "Already paused", "SERIAL")
                else:
                    self.kaos_log("DEBUG", "Not currently paused", "SERIAL")

                break

            if self.G_XBasePosition == 0 and self.G_YBasePosition == 0:
                self.kaos_log("DEBUG", ", XY is 0", "SERIAL")
            else:
                # Vendor note (231216): ,needprevent
                # Vendor note (231214): waiting areaX YW H,
                command_string = """
                    # G90
                    G1 X%.03f Y%.03f F1000
                    """ % (
                    self.G_XBasePosition + (i % 2),
                    self.G_YBasePosition + (i % 2),
                )
                # Vendor note (231129): self.G_PhrozenGCode.run_script_from_command(command_string)
                self.kaos_log("DEBUG", ", XY is P9 set", "SERIAL")

            # Vendor note (20231013): 4
            # Vendor note (231115): 1s
            self.G_ProzenToolhead.dwell(1)
            # Vendor note (240222): cannottime.sleep,codedump
            # time.sleep(1)

            # self.G_PhrozenFluiddRespondInfo("[FIRMWARE] External macro PG110; Klipper purges after STM32 feed")
            # command_string = """
            # PG110
            # """
            # self.G_PhrozenGCode.run_script_from_command(command_string)
            # self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)

            # detectfilament,filament
            if self.G_ToolheadIfHaveFilaFlag:
                Lo_ChangeChannelIfSuccess = True
                break

        # normalfilament change
        if Lo_ChangeChannelIfSuccess == True:
            self.kaos_log("DEBUG", "successful", "SERIAL")
            self.G_IfChangeFilaOngoing = False

            # Vendor note (240108): filament,can
            self.G_M2MAModeResumeFlag = True

            # Vendor note (241106): successful
            self.G_P0M2MAStartPrintFlag = 1

            # Vendor note (250607): print status: 1=retracting; 2=feeding; 3=printing; 4=paused
            self.G_KlipperPrintStatus = 3

            self.G_PauseToLCDString = ""

            # # Vendor note (250611): # self.G_PhrozenFluiddRespondInfo("[PURGE] External macro PG108; heat, purge, wipe")
            # command_string = """
            #     PG108
            #     """
            # self.G_PhrozenGCode.run_script_from_command(command_string)

            # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("can resume, STM32printing report error")
            self.G_KlipperQuickPause = True
            # # Vendor note (250427): # if self.G_SerialPort1OpenFlag == True:
            #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
            #     self.G_PhrozenFluiddRespondInfo("serial port 1-AMSfinishtimingbuffer-full time")
            # if self.G_SerialPort2OpenFlag == True:
            #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
            #     self.G_PhrozenFluiddRespondInfo("serial port 2-AMSfinishtimingbuffer-full time")
            # #self.G_ProzenToolhead.dwell(1.5)

            return

        self.kaos_log("DEBUG", "failed", "SERIAL")
        # expire:time,
        # (default60)
        # A0:,continue(default)
        # A1:
        # filament change
        # Vendor note (20231013): A0:
        if self.G_DictChangeChannelWaitAreaParam["A"] == 0:
            # Vendor note (231209): stm329
            if self.G_KlipperIfPaused == False:
                self.kaos_log("DEBUG", "timeout100s, pause", "SERIAL")

                if self.G_KlipperInPausing == False:
                    self.kaos_log(
                        "DEBUG", "No pause is in progress; a new pause is allowed", "SERIAL"
                    )

                    # Vendor note (240104): stm32
                    # klipper active pause
                    self.Cmds_PhrozenKlipperPauseM2M3ToSTM32(None)

                    # self.emit_protocol("+PAUSE:4,%d" % self.G_ChangeChannelTimeoutNewChan)
                    self.G_PhrozenFluiddRespondInfo(
                        "+PAUSE:4,%d,%d"
                        % (
                            self.G_ChangeChannelTimeoutOldChan,
                            self.G_ChangeChannelTimeoutNewChan,
                        )
                    )
                else:
                    self.kaos_log(
                        "DEBUG",
                        "A pause is already in progress; a new pause is not allowed",
                        "SERIAL",
                    )

            # Vendor note (240123): ifalready,allow
            else:
                self.kaos_log("DEBUG", "already pause, not allowedpause", "SERIAL")

            # Vendor note (231202): P1 C?auto filament change,if1,if,continue1start
            self.G_ChangeChannelFirstFilaFlag = True
            self.G_IfChangeFilaOngoing = False

            # Vendor note (241106): failed
            self.G_P0M2MAStartPrintFlag = 0

            # Vendor note (250607): print status: 1=retracting; 2=feeding; 3=printing; 4=paused
            self.G_KlipperPrintStatus = -1

            return

        # normalfilament change;Actionnormal
        if self.G_DictChangeChannelWaitAreaParam["A"] == 1:
            pass

        self.G_IfChangeFilaOngoing = False

    # P114 S;parameterquerydevice,parameterSquery;"SB";
    # P114 S;parameterquerydevice,parameterSquery ;"SD";
    def Cmds_CmdP114(self, gcmd):
        _ = gcmd

        if gcmd is None:
            self.kaos_log("DEBUG", "[(cmds.python)Cmds_CmdP114]commandP114-None", "SERIAL")
        if gcmd is not None:
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)Cmds_CmdP114]command='%s'" % (gcmd.get_commandline(),),
                "SERIAL",
            )

            # P114parameter
            params = gcmd.get_command_parameters()

        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan,
            "SERIAL",
        )
        self.kaos_log(
            "DEBUG",
            "=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan,
            "SERIAL",
        )

        # Vendor note (240510): self.emit_protocol("+P114:0")

        self.kaos_log("DEBUG", "Current mode", "SERIAL")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.emit_mode(0, "unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.emit_mode(1, "MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.emit_mode(2, "MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.emit_mode(3, "RUNOUT")
        else:
            self.emit_mode(-1, "error")

        # unlock
        self.Base_AMSSerialCmdUnlock()

        self.kaos_log("DEBUG", "self.G_CancelFlag='%s'" % self.G_CancelFlag, "SERIAL")
        # Vendor note (250712): #self.Cmds_CmdP29(None)

        # Vendor note (240511): on resume, reinitialize serial to handle AMS hot-plug serial errors
        try:
            self.kaos_log("DEBUG", "[(cmds.py)Cmds_CmdP114]Reinitializing serial port 1", "SERIAL")
            self.G_SerialPort1Obj = serial.Serial(
                self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3
            )
            # serial port opened successfully
            if self.G_SerialPort1Obj is not None:
                if self.G_SerialPort1Obj.is_open:
                    self.G_SerialPort1OpenFlag = True
                    self.kaos_log("DEBUG", "Reinitializing serial port 1 successful", "SERIAL")
                    # Vendor note (231213): open serial port
                    self.G_SerialPort1Obj.flushInput()  # clean serial write cache
                    self.G_SerialPort1Obj.flush()
                    self.kaos_log("DEBUG", "Serial port 1 buffers cleared", "SERIAL")
                    self.kaos_log("DEBUG", "Re-registering serial port 1 callback", "SERIAL")
                    self.G_SerialPort1RecvTimmer = self.G_PhrozenReactor.register_timer(
                        self.Device_TimmerUart1Recv, self.G_PhrozenReactor.NOW
                    )
        except:
            self.kaos_log(
                "DEBUG", "Unable to open tty1. Check the USB connection or try rebooting.", "SERIAL"
            )

        try:
            self.kaos_log("DEBUG", "[(cmds.py)Cmds_CmdP114]Reinitializing serial port 2", "SERIAL")
            self.G_SerialPort2Obj = serial.Serial(
                self.G_Serialport2Define, SERIAL_PORT_BAUD, timeout=3
            )
            # serial port opened successfully
            if self.G_SerialPort2Obj is not None:
                if self.G_SerialPort2Obj.is_open:
                    self.G_SerialPort2OpenFlag = True
                    self.kaos_log("DEBUG", "Reinitializing serial port 2 successful", "SERIAL")
                    self.G_SerialPort2Obj.flushInput()  # clean serial write cache
                    self.G_SerialPort2Obj.flush()
                    self.kaos_log("DEBUG", "Serial port 2 buffers cleared", "SERIAL")
                    self.kaos_log("DEBUG", "Re-registering serial port 2 callback", "SERIAL")
                    self.G_SerialPort2RecvTimmer = self.G_PhrozenReactor.register_timer(
                        self.Device_TimmerUart2Recv, self.G_PhrozenReactor.NOW
                    )
        except:
            self.kaos_log(
                "DEBUG", "Unable to open tty2. Check the USB connection or try rebooting.", "SERIAL"
            )

        # Vendor note (240524): prevent;cleardata
        self.G_ProzenToolhead.dwell(0.5)

        if self.G_SerialPort1OpenFlag == True:
            if self.G_SerialPort1Obj.is_open:
                # self.G_SerialPort1Obj.flushInput()  # clean serial write cache
                # self.G_SerialPort1Obj.flush()
                self.kaos_log("DEBUG", "Serial port 1 is open", "SERIAL")

        if self.G_SerialPort2OpenFlag == True:
            if self.G_SerialPort2Obj.is_open:
                self.kaos_log("DEBUG", "Serial port 2 is open", "SERIAL")

        # # parameterS,readparameter
        # if "S" in params:
        #     # #ttyUSB0serial send and wait for response
        #     # Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort1SendWaitRsp("SB", sizeof(AMSSimpleInfoBytes))
        #     # if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSSimpleInfoBytes):
        #     #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114] '%s'" % (gcmd.get_commandline(),))
        #     #     return

        #     # Lo_AMSDeviceStateInfo = AMSSimpleInfoBytes()
        #     # Lo_AMSDeviceStateInfo.whole[:] = bytearray(Lo_AMSDeviceStateRspInfo)
        #     # #empty Python dict
        #     # Lo_AMSSimpleState = {}
        #     # self.G_AMS1DeviceState["dev_id"] = Lo_AMSSimpleState["dev_id"] = Lo_AMSDeviceStateInfo.field.dev_id
        #     # self.G_AMS1DeviceState["dev_mode"] = Lo_AMSSimpleState["dev_mode"] = Lo_AMSDeviceStateInfo.field.dev_mode
        #     # self.G_AMS1DeviceState["mc_state"] = Lo_AMSSimpleState["mc_state"] = Lo_AMSDeviceStateInfo.field.mc_state
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_STANDBY:
        #     #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Current state == standby phase == %d" % MC_STANDBY)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_PREPARTION:
        #     #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Current state == prepare filament park phase == %d" % MC_STANDBY)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_CHANGING_P1:
        #     #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Current state == filament change phase 1 == %d" % MC_CHANGING_P1)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_CHANGING_P2:
        #     #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Current state == filament change phase 2 == %d" % MC_CHANGING_P2)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_FORCE_FEED:
        #     #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Current state == filament change phase force refill == %d" % MC_FORCE_FEED)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_PRINTING:
        #     #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Current state == printing phase refill == %d" % MC_PRINTING)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_ROLLBACK:
        #     #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Current state == fully retracted == %d" % MC_ROLLBACK)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_PARKBACK:
        #     #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Current state == retracted to park == %d" % MC_PARKBACK)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_PARKALL:
        #     #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Current state == all retracted to park == %d" % MC_PARKALL)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_CLEANING:
        #     #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Current state == all filaments cleared == %d" % MC_CLEANING)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_ERR_TIMEOUT:
        #     #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Current state == timeout error state == %d" % MC_ERR_TIMEOUT)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_ERR_RUNOUT:
        #     #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Current state == runout error state == %d" % MC_ERR_RUNOUT)
        #     # if int(Lo_AMSDeviceStateInfo.field.mc_state) is MC_ERR_BLOCKUP:
        #     #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Current state == jam error state == %d" % MC_ERR_BLOCKUP)
        #     # self.G_AMS1DeviceState["ma_state"] = Lo_AMSSimpleState["ma_state"] = Lo_AMSDeviceStateInfo.field.ma_state
        #     # # response data JSON conversion
        #     # self.G_PhrozenFluiddRespondInfo(json.dumps(Lo_AMSSimpleState))

        #     self.emit_protocol("+P114:1")
        #     #self.G_P114RunFlag=0
        #     return

        # Vendor note (250619): check if AMS reconnected successfully
        # self.Cmds_USBConnectErrorCheck()

        # get detailed AMS board state
        # Vendor note (240430): stm32,withrsp,time too close;allowP114
        # Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort1SendWaitRsp("SD", sizeof(AMSDetailInfoBytes))
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]P114 blocking serial receive: %s" % Lo_AMSDeviceStateRspInfo)
        # Vendor note (240524): async
        if self.G_SerialPort1OpenFlag == True:
            self.Cmds_AMSSerial1Send("SD")
            self.kaos_log("DEBUG", "Serial port 1 sending command: SD", "SERIAL")

        if self.G_SerialPort2OpenFlag == True:
            self.Cmds_AMSSerial2Send("SD")
            self.kaos_log("DEBUG", "Serial port 2 sending command: SD", "SERIAL")

        # if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
        #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]AMS,checkAMS '%s'" % (gcmd.get_commandline(),))
        #     # Vendor note (240510): #     self.emit_protocol("+P114:-1")
        #     #self.G_P114RunFlag=0
        #     # Vendor note (240412): AMSlabel
        #     self.G_AMSDevice1IfNormal=False
        #     return

        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]AMS responded")

        # # Vendor note (240412): AMSlabel
        # self.G_AMSDevice1IfNormal=True

        # Lo_AMSDeviceStateInfo = AMSDetailInfoBytes()
        # Lo_AMSDeviceStateInfo.whole[:] = bytearray(Lo_AMSDeviceStateRspInfo)
        # #empty Python dict
        # Lo_AMSDetailState = {}
        # self.G_AMSG_AMS1DeviceStateDeviceState["dev_id"] = Lo_AMSDetailState["dev_id"] = Lo_AMSDeviceStateInfo.field.dev_id
        # self.G_AMS1DeviceState["dev_mode"] = Lo_AMSDetailState["dev_mode"] = Lo_AMSDeviceStateInfo.field.dev_mode
        # #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Buffer sensor filament state (bool) == %d" % Lo_AMSDeviceStateInfo.field.cache_exist)
        # self.G_AMS1DeviceState["mc_state"] = Lo_AMSDetailState["mc_state"] = Lo_AMSDeviceStateInfo.field.mc_state
        # self.G_AMS1DeviceState["ma_state"] = Lo_AMSDetailState["ma_state"] = Lo_AMSDeviceStateInfo.field.ma_state
        # # Vendor note (240524): ,-1
        # self.G_AMS1DeviceState["active_dev_id"] = Lo_AMSDetailState["active_dev_id"] = Lo_AMSDeviceStateInfo.field.dev_id
        # self.G_AMS1DeviceState["cache_empty"] = Lo_AMSDetailState["cache_empty"] = Lo_AMSDeviceStateInfo.field.cache_empty
        # #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Buffer sensor empty state (bool) == %d" % Lo_AMSDeviceStateInfo.field.cache_empty)
        # self.G_AMS1DeviceState["cache_full"] = Lo_AMSDetailState["cache_full"] = Lo_AMSDeviceStateInfo.field.cache_full
        # #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Buffer sensor full state (bool) == %d" % Lo_AMSDeviceStateInfo.field.cache_full)
        # self.G_AMS1DeviceState["cache_exist"] = Lo_AMSDetailState["cache_exist"] = Lo_AMSDeviceStateInfo.field.cache_exist
        # self.G_AMS1DeviceState["entry_state"] = Lo_AMSDetailState["entry_state"] = Lo_AMSDeviceStateInfo.field.entry_state
        # #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Entry position sensor state (bit) == %d" % Lo_AMSDeviceStateInfo.field.entry_state)
        # self.G_AMS1DeviceState["park_state"] = Lo_AMSDetailState["park_state"] = Lo_AMSDeviceStateInfo.field.park_state
        # #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Park position sensor state (bit) == %d" % Lo_AMSDeviceStateInfo.field.park_state)

        # # response data JSON conversion
        # self.G_PhrozenFluiddRespondInfo(json.dumps(Lo_AMSDetailState))

        # Vendor note (240123): preventreadstm32
        # time.sleep(1)
        # Vendor note (240229): cannottime.sleep,time to close
        # self.G_ProzenToolhead.dwell(0.5)
        # Vendor note (240510): #self.emit_protocol("+P114:1")
        # self.G_P114RunFlag=False

        self.G_P114RunFlag = 1

        return

    # P30 deviceID(device);"I";deviceID
    def Cmds_CmdP30(self, gcmd):
        if not self.G_SerialPort1OpenFlag:
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)Cmds_CmdP30]AMS multi-material not connected, send P28 first",
                "SERIAL",
            )
            return

        self.kaos_log("DEBUG", "command='%s'" % (gcmd.get_commandline(),), "SERIAL")

        mcu_cmd = G_DictPhrozenCmdP30["mcu_cmd"][0] + "0"
        self.Cmds_AMSSerial1Send(mcu_cmd)
        self.kaos_log("DEBUG", "Sending command: %s" % mcu_cmd, "SERIAL")

        logging.info("SendCmd: %s" % mcu_cmd)

    # P29 disconnect
    def Cmds_CmdP29(self, gcmd):
        self.kaos_log("DEBUG", "[(cmds.python)Cmds_CmdP29]command", "SERIAL")

        try:
            if self.G_SerialPort1Obj is not None:
                if self.G_SerialPort1Obj.is_open:
                    # tty1 closed
                    self.G_SerialPort1Obj.close()
        except:
            self.kaos_log("DEBUG", "AMS1/tty1 not connected; send P28 first.", "SERIAL")
        self.G_SerialPort1OpenFlag = False
        self.kaos_log("DEBUG", "AMS1 clear", "SERIAL")
        self.G_SerialPort1Obj = None

        try:
            if self.G_SerialPort2Obj is not None:
                if self.G_SerialPort2Obj.is_open:
                    self.G_SerialPort2Obj.close()
        except:
            self.kaos_log("DEBUG", "AMS2/tty2 not available; skipping serial port 2.", "SERIAL")
        self.G_SerialPort2OpenFlag = False
        self.kaos_log("DEBUG", "AMS2 clear", "SERIAL")
        self.G_SerialPort2Obj = None

        if self.G_SerialPort1RecvTimmer:
            # unregister
            self.G_PhrozenReactor.unregister_timer(self.G_SerialPort1RecvTimmer)
            # clear
            self.G_SerialPort1RecvTimmer = None

        if self.G_SerialPort2RecvTimmer:
            # unregister
            self.G_PhrozenReactor.unregister_timer(self.G_SerialPort2RecvTimmer)
            # clear
            self.G_SerialPort2RecvTimmer = None

        self.G_P0M1MCNoneAMS = 0
        self.kaos_log("DEBUG", "self.G_P0M1MCNoneAMS=0", "SERIAL")

        # Vendor note (231122): ttyafter,needenableIAPhdl_zigbee_gateway
        # os.system('sh /home/prz/klipper/klippy/extras/phrozen_dev/start.sh &')
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP29]sh /home/prz/klipper/klippy/extras/phrozen_dev/start.sh &")

        # self.G_ProzenToolhead.dwell(1.0)

    def Cmds_GetImageId(self):
        self.kaos_log("DEBUG", "[(dev.python)Cmds_GetImageId]", "SERIAL")

        current_directory = os.getcwd()
        # current_directory=/home/mks/klipper
        # current_directory=/home/prz/klipper
        self.kaos_log("DEBUG", "current_directory=%s" % (current_directory), "SERIAL")

        # Vendor note (250514): readconfig
        try:
            self.kaos_log("DEBUG", "try", "SERIAL")

            # JSONread
            # with open('.././hdlDat/ImageId.json', 'r', encoding='utf-8') as file:
            self.kaos_log("DEBUG", "/etc/ImageId.json", "SERIAL")
            with open("/etc/ImageId.json", "r", encoding="utf-8") as file:
                ImageData = file.read()
            self.kaos_log("DEBUG", "with open", "SERIAL")
            # self.G_PhrozenFluiddRespondInfo("ImageData=%s" % (ImageData))
            # parseJSONdata
            json_data = json.loads(ImageData)
            # self.G_PhrozenFluiddRespondInfo("json_data=%s" % (json_data))
            self.kaos_log("DEBUG", "json_data['ImageId']=%d" % (json_data["ImageId"]), "SERIAL")
            self.G_ImageId = json_data["ImageId"]
            self.kaos_log("DEBUG", "self.G_ImageId=%d" % (self.G_ImageId), "SERIAL")
            self.kaos_log("DEBUG", "json_data['HwId']=%d" % (json_data["HwId"]), "SERIAL")
            self.HwId = json_data["HwId"]
            self.kaos_log("DEBUG", "self.HwId=%d" % (self.HwId), "SERIAL")
            self.kaos_log("DEBUG", "json_data['FwId']=%d" % (json_data["FwId"]), "SERIAL")
            self.kaos_log("DEBUG", "json_data['NC0']=%d" % (json_data["NC0"]), "SERIAL")
            self.kaos_log("DEBUG", "json_data['NC1']=%d" % (json_data["NC1"]), "SERIAL")
            self.kaos_log("DEBUG", "json_data['NC2']=%d" % (json_data["NC2"]), "SERIAL")
            self.kaos_log("DEBUG", "json_data['NC3']=%d" % (json_data["NC3"]), "SERIAL")
            self.kaos_log("DEBUG", "json_data['NC4']=%d" % (json_data["NC4"]), "SERIAL")

            if self.G_ImageId == 16:
                self.kaos_log(
                    "DEBUG", "Image ID==16: ARCO300-MKS-RK3328-STM32F407VET6-I16", "SERIAL"
                )
            elif self.G_ImageId == 31:
                self.kaos_log(
                    "DEBUG", "Image ID==31: ARCO300-phrozen-RK3308-STM32F407VET6-I31", "SERIAL"
                )
            elif self.G_ImageId == -1:
                self.kaos_log(
                    "DEBUG", "Image ID==-1, default: ARCO300-MKS-RK3328-STM32F407VET6-I16", "SERIAL"
                )
            else:
                self.kaos_log(
                    "DEBUG",
                    "Image ID could not be read, default: ARCO300-MKS-RK3328-STM32F407VET6-I16",
                    "SERIAL",
                )
        except Exception as e:
            self.kaos_log("DEBUG", "read get text file number data error", "SERIAL")
            self.kaos_log("DEBUG", "self.G_ImageId=%d" % (self.G_ImageId), "SERIAL")
            # self.G_PhrozenFluiddRespondInfo("self.HwId=%d" % (self.HwId))

    def Cmds_GetUartScreenCfg(self):
        self.kaos_log("DEBUG", "[(cmds.python)Cmds_GetUartScreenCfg]", "SERIAL")

        # Vendor note (250514): readconfig
        try:
            self.kaos_log("DEBUG", "try", "SERIAL")

            # Vendor note (250724): read system image id to distinguish product/board/firmware variants
            # Vendor note (250724): read image id
            self.Cmds_GetImageId()
            if self.G_ImageId == 16:
                self.kaos_log(
                    "DEBUG", "Image ID==16: ARCO300-MKS-RK3328-STM32F407VET6-I16", "SERIAL"
                )
                # JSONread
                with open(
                    "/home/mks/klipper/klippy/extras/phrozen_dev/serial-screen/plr_print_precfg.json",
                    "r",
                    encoding="utf-8",
                ) as file:
                    UartScreenCfgData = file.read()
            elif self.G_ImageId == 31:
                self.kaos_log(
                    "DEBUG", "Image ID==31: ARCO300-phrozen-RK3308-STM32F407VET6-I31", "SERIAL"
                )
                # JSONread
                with open(
                    "/home/prz/klipper/klippy/extras/phrozen_dev/serial-screen/plr_print_precfg.json",
                    "r",
                    encoding="utf-8",
                ) as file:
                    UartScreenCfgData = file.read()
            elif self.G_ImageId == -1:
                self.kaos_log(
                    "DEBUG", "Image ID==-1, default: ARCO300-MKS-RK3328-STM32F407VET6-I16", "SERIAL"
                )
                # JSONread
                with open(
                    "/home/mks/klipper/klippy/extras/phrozen_dev/serial-screen/plr_print_precfg.json",
                    "r",
                    encoding="utf-8",
                ) as file:
                    UartScreenCfgData = file.read()
            else:
                self.kaos_log(
                    "DEBUG",
                    "Image ID could not be read, default: ARCO300-MKS-RK3328-STM32F407VET6-I16",
                    "SERIAL",
                )
                # JSONread
                with open(
                    "/home/mks/klipper/klippy/extras/phrozen_dev/serial-screen/plr_print_precfg.json",
                    "r",
                    encoding="utf-8",
                ) as file:
                    UartScreenCfgData = file.read()

            self.kaos_log("DEBUG", "with open", "SERIAL")
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
            # parseJSONdata
            json_data = json.loads(UartScreenCfgData)
            # print(json_data['Auto_Replace_state'])
            # print(json_data['Chroma_Kit_num'])
            self.kaos_log(
                "DEBUG",
                "json_data['Auto_Replace_state']=%d" % (json_data["Auto_Replace_state"]),
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG", "json_data['Chroma_Kit_num']=%d" % (json_data["Chroma_Kit_num"]), "SERIAL"
            )
            self.kaos_log(
                "DEBUG",
                "json_data['Chroma_Kit_access']['T0']=%d" % (json_data["Chroma_Kit_access"]["T0"]),
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "json_data['Chroma_Kit_access']['T1']=%d" % (json_data["Chroma_Kit_access"]["T1"]),
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "json_data['Chroma_Kit_access']['T2']=%d" % (json_data["Chroma_Kit_access"]["T2"]),
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "json_data['Chroma_Kit_access']['T3']=%d" % (json_data["Chroma_Kit_access"]["T3"]),
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "json_data['Chroma_Kit_access']['T4']=%d" % (json_data["Chroma_Kit_access"]["T4"]),
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "json_data['Chroma_Kit_access']['T5']=%d" % (json_data["Chroma_Kit_access"]["T5"]),
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "json_data['Chroma_Kit_access']['T6']=%d" % (json_data["Chroma_Kit_access"]["T6"]),
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "json_data['Chroma_Kit_access']['T7']=%d" % (json_data["Chroma_Kit_access"]["T7"]),
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "json_data['Chroma_Kit_access']['T8']=%d" % (json_data["Chroma_Kit_access"]["T8"]),
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "json_data['Chroma_Kit_access']['T9']=%d" % (json_data["Chroma_Kit_access"]["T9"]),
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "json_data['Chroma_Kit_access']['T10']=%d"
                % (json_data["Chroma_Kit_access"]["T10"]),
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "json_data['Chroma_Kit_access']['T11']=%d"
                % (json_data["Chroma_Kit_access"]["T11"]),
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "json_data['Chroma_Kit_access']['T12']=%d"
                % (json_data["Chroma_Kit_access"]["T12"]),
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "json_data['Chroma_Kit_access']['T13']=%d"
                % (json_data["Chroma_Kit_access"]["T13"]),
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "json_data['Chroma_Kit_access']['T14']=%d"
                % (json_data["Chroma_Kit_access"]["T14"]),
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "json_data['Chroma_Kit_access']['T15']=%d"
                % (json_data["Chroma_Kit_access"]["T15"]),
                "SERIAL",
            )

            self.G_AutoReplaceState = json_data["Auto_Replace_state"]
            self.G_ChromaKitNum = json_data["Chroma_Kit_num"]
            self.G_ChromaKitAccessT0 = json_data["Chroma_Kit_access"]["T0"]
            self.G_ChromaKitAccessT1 = json_data["Chroma_Kit_access"]["T1"]
            self.G_ChromaKitAccessT2 = json_data["Chroma_Kit_access"]["T2"]
            self.G_ChromaKitAccessT3 = json_data["Chroma_Kit_access"]["T3"]
            self.G_ChromaKitAccessT4 = json_data["Chroma_Kit_access"]["T4"]
            self.G_ChromaKitAccessT5 = json_data["Chroma_Kit_access"]["T5"]
            self.G_ChromaKitAccessT6 = json_data["Chroma_Kit_access"]["T6"]
            self.G_ChromaKitAccessT7 = json_data["Chroma_Kit_access"]["T7"]
            self.G_ChromaKitAccessT8 = json_data["Chroma_Kit_access"]["T8"]
            self.G_ChromaKitAccessT9 = json_data["Chroma_Kit_access"]["T9"]
            self.G_ChromaKitAccessT10 = json_data["Chroma_Kit_access"]["T10"]
            self.G_ChromaKitAccessT11 = json_data["Chroma_Kit_access"]["T11"]
            self.G_ChromaKitAccessT12 = json_data["Chroma_Kit_access"]["T12"]
            self.G_ChromaKitAccessT13 = json_data["Chroma_Kit_access"]["T13"]
            self.G_ChromaKitAccessT14 = json_data["Chroma_Kit_access"]["T14"]
            self.G_ChromaKitAccessT15 = json_data["Chroma_Kit_access"]["T15"]

            self.kaos_log(
                "DEBUG", "self.G_AutoReplaceState=%d" % (self.G_AutoReplaceState), "SERIAL"
            )
            self.kaos_log("DEBUG", "self.G_ChromaKitNum=%d" % (self.G_ChromaKitNum), "SERIAL")
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT0=%d" % (self.G_ChromaKitAccessT0), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT1=%d" % (self.G_ChromaKitAccessT1), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT2=%d" % (self.G_ChromaKitAccessT2), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT3=%d" % (self.G_ChromaKitAccessT3), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT4=%d" % (self.G_ChromaKitAccessT4), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT5=%d" % (self.G_ChromaKitAccessT5), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT6=%d" % (self.G_ChromaKitAccessT6), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT7=%d" % (self.G_ChromaKitAccessT7), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT8=%d" % (self.G_ChromaKitAccessT8), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT9=%d" % (self.G_ChromaKitAccessT9), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT10=%d" % (self.G_ChromaKitAccessT10), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT11=%d" % (self.G_ChromaKitAccessT11), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT12=%d" % (self.G_ChromaKitAccessT12), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT13=%d" % (self.G_ChromaKitAccessT13), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT14=%d" % (self.G_ChromaKitAccessT14), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT15=%d" % (self.G_ChromaKitAccessT15), "SERIAL"
            )

        except:
            self.kaos_log("DEBUG", "number data error, but number data get", "SERIAL")

    # Function: clear UART-screen cached configuration state
    # Purpose: force fresh screen-config sync on next query
    def Cmds_GetUartScreenCfgClear(self):
        self.kaos_log("DEBUG", "[(cmds.python)Cmds_GetUartScreenCfgClear]", "SERIAL")

        # Vendor note (250514): readconfig
        try:
            self.kaos_log("DEBUG", "try", "SERIAL")
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

            self.kaos_log(
                "DEBUG", "self.G_AutoReplaceState=%d" % (self.G_AutoReplaceState), "SERIAL"
            )
            self.kaos_log("DEBUG", "self.G_ChromaKitNum=%d" % (self.G_ChromaKitNum), "SERIAL")
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT0=%d" % (self.G_ChromaKitAccessT0), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT1=%d" % (self.G_ChromaKitAccessT1), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT2=%d" % (self.G_ChromaKitAccessT2), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT3=%d" % (self.G_ChromaKitAccessT3), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT4=%d" % (self.G_ChromaKitAccessT4), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT5=%d" % (self.G_ChromaKitAccessT5), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT6=%d" % (self.G_ChromaKitAccessT6), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT7=%d" % (self.G_ChromaKitAccessT7), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT8=%d" % (self.G_ChromaKitAccessT8), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT9=%d" % (self.G_ChromaKitAccessT9), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT10=%d" % (self.G_ChromaKitAccessT10), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT11=%d" % (self.G_ChromaKitAccessT11), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT12=%d" % (self.G_ChromaKitAccessT12), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT13=%d" % (self.G_ChromaKitAccessT13), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT14=%d" % (self.G_ChromaKitAccessT14), "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "self.G_ChromaKitAccessT15=%d" % (self.G_ChromaKitAccessT15), "SERIAL"
            )

        except:
            self.kaos_log("DEBUG", "serial port disable set number data error", "SERIAL")

    # P0 M1;(device) Yes;"MC";P0 M1;P28;P2 A1;
    # P0 M2;refill mode(device);"MA";P0 M2;P28;P8;
    # P0 M3; ;P0 M3;
    # P28 device
    def Cmds_CmdP28(self, gcmd):
        self.kaos_log(
            "DEBUG", "[(cmds.python)Cmds_CmdP28]command='%s'" % (gcmd.get_commandline(),), "SERIAL"
        )

        # KAOS: skip AMS connection when ams_attached is false
        if not self.G_AmsAttached:
            self.kaos_log("DEBUG", "P28 skipped: ams_attached is false", "SERIAL")
            return

        self.kaos_log("DEBUG", "self.G_CancelFlag='%s'" % self.G_CancelFlag, "SERIAL")
        # # Vendor note (250712): # self.G_CancelFlag=False
        # self.G_PhrozenFluiddRespondInfo("self.G_CancelFlag='%s'" % self.G_CancelFlag)

        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        # Vendor note (250517): #self.G_PhrozenPrinterCancelPauseResume.cmd_CLEAR_PAUSE
        # #cancel
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")

        # # Vendor note (250807): cannotclear,P28
        # self.G_PhrozenPrinterCancelPauseResume.cmd_CLEAR_PAUSE(None)
        # self.G_PhrozenFluiddRespondInfo("clearpause state")

        Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
        self.kaos_log("DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL")
        self.kaos_log(
            "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
        )
        # // current-Lo_PauseStatus='{'is_paused': True}'
        if Lo_PauseStatus["is_paused"] == True:
            self.kaos_log("DEBUG", "Already paused", "SERIAL")
        else:
            self.kaos_log("DEBUG", "Not currently paused", "SERIAL")

        self.kaos_log("DEBUG", "Current mode", "SERIAL")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.emit_mode(0, "unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.emit_mode(1, "MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.emit_mode(2, "MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.emit_mode(3, "RUNOUT")
        else:
            self.emit_mode(-1, "error")

        # unlock
        self.Base_AMSSerialCmdUnlock()

        # Vendor note (250607): print status: 1=retracting; 2=feeding; 3=printing; 4=paused
        if self.G_KlipperPrintStatus == 3:
            self.kaos_log("DEBUG", "printing, logical P28!!!", "SERIAL")
            return

        # Vendor note (250724): read image id
        self.Cmds_GetImageId()

        # Vendor note (250514): read json file for single-color refill config and channel-color mapping
        # /home/prz/klipper/klippy/extras/phrozen_dev/serial-screen/test.json
        self.Cmds_GetUartScreenCfg()

        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdP28]single-color mode, logical P28", "SERIAL"
            )
            self.G_PhrozenFluiddRespondInfo(
                "V-H%s-I%s-F%s" % (HW_VERSION, IMAGE_VERSION, FW_VERSION)
            )
            return

        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdP28]single-color refill mode, logical P28", "SERIAL"
            )
            self.G_PhrozenFluiddRespondInfo(
                "V-H%s-I%s-F%s" % (HW_VERSION, IMAGE_VERSION, FW_VERSION)
            )
            return

        # # Vendor note (231122): ttyUSB0before,needIAPhdl_zigbee_gateway
        # os.system('sh /home/prz/klipper/klippy/extras/phrozen_dev/stop.sh &')
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP28]sh /home/prz/klipper/klippy/extras/phrozen_dev/stop.sh &")

        self.G_KlipperIfPaused = False
        # Vendor note (250526): ,allowgcode,complete
        self.G_KlipperInPausing = False
        self.G_IfToolheadHaveFilaInitiativePauseFlag = False
        # Vendor note (240223): failed
        self.ToolheadCutFlag = False

        if self.G_SerialPort1Obj is not None:
            # Vendor note (231219): ifalready,cannot
            if self.G_SerialPort1Obj.is_open:
                self.kaos_log("DEBUG", "P28 serial port 1 already open", "SERIAL")
                # self.G_PhrozenFluiddRespondInfo("V-H%s-I%s-F%s" % (HW_VERSION,IMAGE_VERSION,FW_VERSION))
                # # Vendor note (240104): return
                # self.emit_protocol("+AMSCONNECT:0")

                # self.G_SerialPort1Obj.flushInput()  # clean serial write cache
                # self.G_SerialPort1Obj.flush()
                # self.G_SerialPort1OpenFlag = True
                # Vendor note (240524): None,
                # if self.G_SerialPort1RecvTimmer is None:
                # period
                self.kaos_log("DEBUG", "Re-registering serial port 1 callback", "SERIAL")
                self.G_Serial1PortRecvTimmer = self.G_PhrozenReactor.register_timer(
                    self.Device_TimmerUart1Recv, self.G_PhrozenReactor.NOW
                )

                # Vendor note (240511): ,MA M0 MA,AMS
                # if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
                #     self.G_PhrozenFluiddRespondInfo("Sending command: M0 mode")
                #     self.Cmds_AMSSerial1Send("M0")

                # if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                #     self.G_PhrozenFluiddRespondInfo("Sending command: M0 mode")
                #     self.Cmds_AMSSerial1Send("M0")

                self.G_ProzenToolhead.dwell(0.5)

                # Vendor note (250619): check if AMS reconnected successfully
                self.Cmds_USBConnectErrorCheck()
                if self.G_SerialPort1OpenFlag == True:
                    # #get detailed AMS board state
                    # Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort1SendWaitRsp("SD", sizeof(AMSDetailInfoBytes))
                    # self.G_PhrozenFluiddRespondInfo("ttyserial port 1 receive : %s" % Lo_AMSDeviceStateRspInfo)

                    # if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                    #     self.G_PhrozenFluiddRespondInfo("AMS1,checkAMS '%s'" % (gcmd.get_commandline(),))
                    #     # Vendor note (240412): AMSlabel
                    #     self.G_AMSDevice1IfNormal=False
                    # else:
                    #     self.G_PhrozenFluiddRespondInfo("AMS1successful '%s'" % (gcmd.get_commandline(),))
                    #     self.G_PhrozenFluiddRespondInfo("self.G_AMSDevice1IfNormal=True")

                    #     # Vendor note (240412): AMSlabel
                    #     self.G_AMSDevice1IfNormal=True
                    self.Cmds_AMSSerial1Send("SD")
                    self.kaos_log("DEBUG", "SD", "SERIAL")

                self.G_ProzenToolhead.dwell(2)

                # if self.G_SerialPort2Obj is not None:
                #     if self.G_SerialPort2Obj.is_open:
                #         self.G_PhrozenFluiddRespondInfo("Serial port 2 already open, continue")
                #     else:
                #         self.G_PhrozenFluiddRespondInfo("V-H%s-I%s-F%s" % (HW_VERSION,IMAGE_VERSION,FW_VERSION))
                #         self.emit_protocol("+AMSCONNECT:0")
                #         #return
                #         return
                self.G_SerialPortHaveOpenedCount = self.G_SerialPortHaveOpenedCount + 1

        if self.G_SerialPort2Obj is not None:
            if self.G_SerialPort2Obj.is_open:
                self.kaos_log("DEBUG", "P28 serial port 2 already open", "SERIAL")
                self.kaos_log("DEBUG", "Re-registering serial port 2 callback", "SERIAL")
                self.G_SerialPort2RecvTimmer = self.G_PhrozenReactor.register_timer(
                    self.Device_TimmerUart2Recv, self.G_PhrozenReactor.NOW
                )

                # self.G_PhrozenFluiddRespondInfo("V-H%s-I%s-F%s" % (HW_VERSION,IMAGE_VERSION,FW_VERSION))
                # self.emit_protocol("+AMSCONNECT:0")

                self.G_ProzenToolhead.dwell(0.5)

                if self.G_SerialPort2OpenFlag == True:
                    # #get detailed AMS board state
                    # Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort2SendWaitRsp("SD", sizeof(AMSDetailInfoBytes))
                    # self.G_PhrozenFluiddRespondInfo("ttyserial port 2 receive : %s" % Lo_AMSDeviceStateRspInfo)
                    # if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                    #     self.G_PhrozenFluiddRespondInfo("AMS2,checkAMS '%s'" % (gcmd.get_commandline(),))
                    #     self.G_AMSDevice2IfNormal=False
                    # else:
                    #     self.G_PhrozenFluiddRespondInfo("AMS2successful '%s'" % (gcmd.get_commandline(),))
                    #     self.G_AMSDevice2IfNormal=True
                    #     self.G_PhrozenFluiddRespondInfo("self.G_AMSDevice2IfNormal=True")
                    self.Cmds_AMSSerial2Send("SD")
                    self.kaos_log("DEBUG", "SD", "SERIAL")

                self.G_ProzenToolhead.dwell(2)

                self.G_SerialPortHaveOpenedCount = self.G_SerialPortHaveOpenedCount + 1

                # return

        if self.G_SerialPortHaveOpenedCount > 0:
            self.kaos_log(
                "DEBUG",
                "has unit AMSalready openserial port='%d'" % (self.G_SerialPortHaveOpenedCount,),
                "SERIAL",
            )
            self.G_SerialPortHaveOpenedCount = 0
            self.G_PhrozenFluiddRespondInfo(
                "V-H%s-I%s-F%s" % (HW_VERSION, IMAGE_VERSION, FW_VERSION)
            )
            self.emit_protocol("+AMSCONNECT:0")

            if self.G_SerialPort1OpenFlag == True:
                # Vendor note (240524): readAMS16HUB
                self.Cmds_AMSSerial1Send("AT+SB=0")
                self.kaos_log(
                    "DEBUG",
                    "serial port 1Sending command: AT+SB=0; get AMSmainboard,16HUBmainboard",
                    "SERIAL",
                )
            if self.G_SerialPort2OpenFlag == True:
                # Vendor note (240524): readAMS16HUB
                self.Cmds_AMSSerial2Send("AT+SB=0")
                self.kaos_log(
                    "DEBUG",
                    "serial port 2Sending command: AT+SB=0; get AMSmainboard,16HUBmainboard",
                    "SERIAL",
                )

            self.kaos_log("DEBUG", "return", "SERIAL")
            # return
            return

        # Vendor note (240511): 0.5,preventklippertime too close
        time.sleep(0.5)

        # # Vendor note (20231019): ,auto filament changeif1filament,needfilament
        # # Vendor note (20231020): detect
        # #if self.G_ToolheadIfHaveFilaFlag:
        # # # 0=defaultgcodeexecute
        # Vendor note (231128): G28PG28
        # if self.G_ChangeChannelIfZLiftingUpByGcode == 0:
        #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP28]Home all and cut filament")
        #     command_string = """
        #     PG28
        #     """
        #     self.G_PhrozenGCode.run_script_from_command(command_string)
        #     # Vendor note (20231020): gcode,need,time,,auto filament change
        #     # G92 E0
        #     # G1 E0.0000 F600
        #     # G91
        #     # G1 E-0.385 F8000
        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP28]Cut filament")
        # # Vendor note (20231013): # self.Cmds_MoveToCutFilaAction(gcmd)
        # #self.G_PhrozenFluiddRespondInfo("Sending command: AP, retract all to park position")
        # #// all retract to park;//===== P2 A1  Yes;"AP";
        # #Klipper state: Shutdown
        # #!! Internal error on command:"P28"
        # #ifttyUSB0,klipprsystem
        # #self.Cmds_AMSSerial1Send("AP")

        # Vendor note (241030): 1
        try:
            # tty,19200
            self.G_SerialPort1Obj = serial.Serial(
                self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3
            )
            # serial port opened successfully
            if self.G_SerialPort1Obj.is_open:
                self.kaos_log("DEBUG", "serial port 1 #1 time opensuccessful", "SERIAL")
                # Vendor note (231213): open serial port
                self.G_SerialPort1Obj.flushInput()  # clean serial write cache
                self.G_SerialPort1Obj.flush()
                self.G_SerialPort1OpenFlag = True
                # Vendor note (240524): None,
                # if self.G_SerialPort1RecvTimmer is None:
                # period
                self.G_SerialPort1RecvTimmer = self.G_PhrozenReactor.register_timer(
                    self.Device_TimmerUart1Recv, self.G_PhrozenReactor.NOW
                )

                # Vendor note (240306): ifM1-MC,MCstm32
                if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
                    self.kaos_log(
                        "DEBUG", "AMS_WORK_MODE_MC; Sending command: M1-MC, MC mode", "SERIAL"
                    )
                    self.Cmds_AMSSerial1Send("MC")

                # Vendor note (241031): ,defaultMC
                if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                    self.kaos_log(
                        "DEBUG", "AMS_WORK_MODE_UNKNOW; Sending command: M1-MC, MC mode", "SERIAL"
                    )
                    self.Cmds_AMSSerial1Send("MC")

                if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                    self.kaos_log(
                        "DEBUG", "AMS_WORK_MODE_MA; Sending command: M2-MA, MA mode", "SERIAL"
                    )
                    self.Cmds_AMSSerial1Send("MA")

                if self.G_ToolheadIfHaveFilaFlag:
                    self.kaos_log("DEBUG", "toolhead up has filament", "SERIAL")
                    # Vendor note (240113): MC AMS_WORK_MODE_UNKNOW,
                    if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
                        # Vendor note (240319): before
                        # self.Cmds_MoveToCutFilaPrepare()
                        # if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
                        self.kaos_log("DEBUG", "PG107; add before wipe nozzle", "SERIAL")
                        command_string = """
                        # PG107
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log("DEBUG", "command_string='%s'" % command_string, "SERIAL")
                        # Vendor note (240323): before
                        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)]External macro command-PRZ_CZ; before cut, wipe nozzle")
                        # command_string = """
                        # PRZ_CZ
                        # """
                        # self.G_PhrozenGCode.run_script_from_command(command_string)
                        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)]command_string='%s'" % command_string)
                        # Vendor note (231202): self.Cmds_MoveToCutFilaAndRollback(gcmd)
                    # Vendor note (240104): M2MArefill modecannot
                    # if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                    #    # Vendor note (231202): #    self.Cmds_MoveToCutFilaAndNotRollback(gcmd)

                    # 20s,preventp28
                    # time.sleep(20)
                    # raise gcmd.error("[(cmds.python)Cmds_CmdP28]AMSfailed")

                self.G_ProzenToolhead.dwell(2)

                if self.G_SerialPort1OpenFlag == True:
                    # #get detailed AMS board state
                    # Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort1SendWaitRsp("SD", sizeof(AMSDetailInfoBytes))
                    # self.G_PhrozenFluiddRespondInfo("tty1 serial port receive : %s" % Lo_AMSDeviceStateRspInfo)
                    # if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                    #     self.G_PhrozenFluiddRespondInfo("AMS1,checkAMS '%s'" % (gcmd.get_commandline(),))
                    #     # Vendor note (240412): AMSlabel
                    #     self.G_AMSDevice1IfNormal=False
                    # else:
                    #     # Vendor note (240412): AMSlabel
                    #     self.G_AMSDevice1IfNormal=True
                    self.Cmds_AMSSerial1Send("SD")
                    self.kaos_log("DEBUG", "SD", "SERIAL")

                self.G_ProzenToolhead.dwell(2)

                self.G_SerialPortIsOpenCount = self.G_SerialPortIsOpenCount + 1

                # self.G_PhrozenFluiddRespondInfo("V-H%s-I%s-F%s" % (HW_VERSION,IMAGE_VERSION,FW_VERSION))
                # self.emit_protocol("+AMSCONNECT:0")

            else:
                self.kaos_log("DEBUG", "serial port 1 #1 time openfailed", "SERIAL")
                # self.emit_protocol("+AMSCONNECT:1")
                self.G_SerialPort1OpenFlag = False
                # gcmd.respond_info("Unable to connect to Phrozen devs")
                # Vendor note (231207): 1-AMSfailed
                # Vendor note (231207): 2-AMSttyfailed
                self.emit_protocol("+AMSERROR:1")
                self.kaos_log("DEBUG", "AMS1 multi-material connectedfailed", "SERIAL")
                # raise gcmd.error("AMS1 multi-material connectedfailed")
        except:
            self.kaos_log("DEBUG", "serial port 1 #1 time openfailed", "SERIAL")
            # self.emit_protocol("+AMSCONNECT:2")
            # gcmd.respond_info("Unable open USB serial port, Please check USB port connect first")
            # Vendor note (231207): 1-AMSfailed
            # Vendor note (231207): 2-AMSttyfailed
            self.emit_protocol("+AMSERROR:2")
            self.kaos_log(
                "DEBUG", "Unable to open tty1. Check the USB connection or try rebooting.", "SERIAL"
            )
            # raise gcmd.error("AMS1 multi-material connectedfailed")

        # Vendor note (241030): 2
        try:
            # tty,19200
            self.G_SerialPort2Obj = serial.Serial(
                self.G_Serialport2Define, SERIAL_PORT_BAUD, timeout=3
            )
            # serial port opened successfully
            if self.G_SerialPort2Obj.is_open:
                self.kaos_log("DEBUG", "serial port 2 #1 time opensuccessful", "SERIAL")
                self.G_SerialPort2Obj.flushInput()  # clean serial write cache
                self.G_SerialPort2Obj.flush()
                self.G_SerialPort2OpenFlag = True
                self.G_SerialPort2RecvTimmer = self.G_PhrozenReactor.register_timer(
                    self.Device_TimmerUart2Recv, self.G_PhrozenReactor.NOW
                )

                self.G_ProzenToolhead.dwell(0.5)

                if self.G_SerialPort2OpenFlag == True:
                    # #get detailed AMS board state
                    # Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort2SendWaitRsp("SD", sizeof(AMSDetailInfoBytes))
                    # self.G_PhrozenFluiddRespondInfo("tty2 serial port receive : %s" % Lo_AMSDeviceStateRspInfo)
                    # if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                    #     self.G_PhrozenFluiddRespondInfo("AMS2,checkAMS '%s'" % (gcmd.get_commandline(),))
                    #     self.G_AMSDevice2IfNormal=False
                    # else:
                    #     # Vendor note (240412): AMSlabel
                    #     self.G_AMSDevice2IfNormal=True
                    self.Cmds_AMSSerial2Send("SD")
                    self.kaos_log("DEBUG", "SD", "SERIAL")

                self.G_ProzenToolhead.dwell(2)

                self.G_SerialPortIsOpenCount = self.G_SerialPortIsOpenCount + 1

                # self.G_PhrozenFluiddRespondInfo("V-H%s-I%s-F%s" % (HW_VERSION,IMAGE_VERSION,FW_VERSION))
                # self.emit_protocol("+AMSCONNECT:0")

            # Function name:
            # Input parameters:
            # [Translated vendor note] :
            # [Translated vendor note] Description: -20230830
            ####################################
            # [Translated vendor note] P114 S; ,S; "SB";
            # [Translated vendor note] P114 S; ,S ; "SD";

            else:
                self.kaos_log("DEBUG", "serial port 2 #1 time openfailed", "SERIAL")
                # self.emit_protocol("+AMSCONNECT:1")
                self.G_SerialPort2OpenFlag = False
                self.emit_protocol("+AMSERROR:1")
                self.kaos_log("DEBUG", "AMS2 multi-material connectedfailed", "SERIAL")
                # raise gcmd.error("AMS2 multi-material connectedfailed")
        except:
            self.kaos_log("DEBUG", "serial port 2 #1 time openfailed", "SERIAL")
            # self.emit_protocol("+AMSCONNECT:2")
            self.emit_protocol("+AMSERROR:2")
            self.kaos_log(
                "DEBUG", "Unable to open tty2. Check the USB connection or try rebooting.", "SERIAL"
            )
            # raise gcmd.error("AMS1 multi-material connectedfailed")

        # Vendor note (241030): successful,can
        if self.G_SerialPortIsOpenCount > 0:
            self.kaos_log(
                "DEBUG",
                "successfulopenAMSAMS has unit =%d" % self.G_SerialPortIsOpenCount,
                "SERIAL",
            )
            self.G_SerialPortIsOpenCount = 0

            self.G_PhrozenFluiddRespondInfo(
                "V-H%s-I%s-F%s" % (HW_VERSION, IMAGE_VERSION, FW_VERSION)
            )
            self.emit_protocol("+AMSCONNECT:0")

            if self.G_SerialPort1OpenFlag == True:
                # Vendor note (240524): readAMS16HUB
                self.Cmds_AMSSerial1Send("AT+SB=0")
                self.kaos_log(
                    "DEBUG",
                    "serial port 1Sending command: AT+SB=0; get AMSmainboard,16HUBmainboard",
                    "SERIAL",
                )
            if self.G_SerialPort2OpenFlag == True:
                # Vendor note (240524): readAMS16HUB
                self.Cmds_AMSSerial2Send("AT+SB=0")
                self.kaos_log(
                    "DEBUG",
                    "serial port 2Sending command: AT+SB=0; get AMSmainboard,16HUBmainboard",
                    "SERIAL",
                )

        # if0,successful
        else:
            # self.emit_protocol("+AMSCONNECT:2")
            self.emit_protocol("+AMSERROR:2")
            self.kaos_log("DEBUG", "opentty port, please checkUSB port or try rebooting", "SERIAL")

            raise gcmd.error("has connectAMSAMS, connectAMSfailed")

    # Vendor note (241101): # P10 S?    parameterS[1,5]:control,S1-1,S2-2...,5
    def Cmds_CmdP10(self, gcmd):
        # get command parameters
        params = gcmd.get_command_parameters()

        if self.G_KlipperIfPaused == True:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdP10]klipper pause, but received command", "SERIAL"
            )

        self.kaos_log(
            "DEBUG", "[(cmds.python)Cmds_CmdP10]command='%s'" % (gcmd.get_commandline(),), "SERIAL"
        )

        self.kaos_log("DEBUG", "command: '%s'" % (gcmd.get_commandline(),), "SERIAL")

        if "S" in params:
            Lo_SpitNum = int(params["S"])
            if not Lo_SpitNum in [1, 2, 3, 4, 5, 6, 7, 8, 9]:
                raise gcmd.error(
                    "no parameter number command;cmd '%s', parameter number S need in [1/2/3/4/5/6/7/8/9]"
                    % (gcmd.get_commandline(),)
                )

            self.G_P10SpitNum = Lo_SpitNum

        # Vendor note (250519): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_CUT_WAITINGAREA")
        command_string = """
            # PRZ_CUT_WAITINGAREA
            """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.kaos_log(
            "DEBUG",
            "[SERVICE] Move to service/waiting position; command_string='%s'" % command_string,
            "SERIAL",
        )

        self.kaos_log("DEBUG", "purge time number : '%d'" % (self.G_P10SpitNum,), "SERIAL")

    # P11 T?;
    def Cmds_CmdP11(self, gcmd):
        if gcmd is None:
            self.kaos_log("DEBUG", "[(cmds.python)Cmds_CmdP11]gcmd-None", "SERIAL")
            self.kaos_log("DEBUG", "[(cmds.python)Cmds_CmdP11]return", "SERIAL")
            return
        if gcmd is not None:
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)Cmds_CmdP11]command='%s'" % (gcmd.get_commandline(),),
                "SERIAL",
            )

        # get command parameters
        params = gcmd.get_command_parameters()

        self.kaos_log(
            "DEBUG",
            "[(cmds.python)Cmds_CmdP11]command='%s'; AMS cutter" % (gcmd.get_commandline(),),
            "SERIAL",
        )

        self.kaos_log("DEBUG", "command: '%s'" % (gcmd.get_commandline(),), "SERIAL")

        # for UIUX dynamic interface
        self.emit_protocol("+P11:0,%d" % self.G_ChangeChannelTimeoutNewChan)

        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()

        # Vendor note (240527): ,becausemanual command,defaultSTM32MC
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.kaos_log(
                "DEBUG",
                "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                "SERIAL",
            )
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")

            self.G_ProzenToolhead.dwell(2)

        self.G_KlipperIfPaused = False
        # Vendor note (240221): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0
        self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")

        self.G_CutCheckTest = True
        self.ManualCmdFlag = False

        # if self.G_ToolheadIfHaveFilaFlag:
        self.kaos_log("DEBUG", "reset, cut filament; all AMSfirst", "SERIAL")
        # Vendor note (231205): self.Cmds_MoveToCutFilaAndHomingXY(gcmd)

        self.kaos_log(
            "DEBUG",
            "[FIRMWARE] External macro PG104; capture variables before toolchange",
            "SERIAL",
        )
        command_string = """
            # PG104
            """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.kaos_log(
            "DEBUG",
            "[FIRMWARE] External macro PG104; command_string='%s'" % command_string,
            "SERIAL",
        )
        self.IfDoPG102Flag = True

        # Vendor note (240510): before,feed waiting zone
        # Vendor note (240306): # Vendor note (240110): waiting areabefore,execute,position
        # Vendor note (240515): before,feed waiting zone
        self.kaos_log("DEBUG", "[TOOLCHANGE] External macro PG101 retract/pre-cut", "SERIAL")
        command_string = """
            # PG101
            """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.kaos_log(
            "DEBUG",
            "[SERVICE] Move to waiting area for purge; command_string='%s'" % command_string,
            "SERIAL",
        )
        self.IfDoPG102Flag = True

        # Vendor note (240319): ,filament,prevent
        self.kaos_log(
            "DEBUG", "[TOOLCHANGE] External macro PG106; purge residue before cut", "SERIAL"
        )
        self.PG102Flag = True
        self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
        command_string = """
        # PG106
        """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.kaos_log("DEBUG", "command_string='%s'" % command_string, "SERIAL")
        self.PG102Flag = False
        self.kaos_log("DEBUG", "self.Flag=False", "SERIAL")

        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()

        if self.G_SerialPort1OpenFlag == True:
            self.Cmds_AMSSerial1Send("AP")
            self.kaos_log("DEBUG", "Serial port 1 sending command: AP", "SERIAL")
        if self.G_SerialPort2OpenFlag == True:
            self.Cmds_AMSSerial2Send("AP")
            self.kaos_log("DEBUG", "Serial port 2 sending command: AP", "SERIAL")

        self.G_ProzenToolhead.dwell(0.5)

        # Vendor note (240913): self.G_ProzenToolhead.dwell(6.0)
        # Vendor note (231201): checkfilamentnormal,normal
        self.Cmds_CutFilaIfNormalCheck()
        if self.G_KlipperIfPaused == True:
            self.kaos_log(
                "DEBUG",
                "cut filament 5toolhead detectedfilament, cutter error, please check cutter, pauseklipperprinting",
                "SERIAL",
            )
            # Lo_ChangeChannelIfSuccess = False
            # for UIUX dynamic interface
            self.emit_protocol("+P11:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            self.G_CutCheckTest = False
            return

        if self.G_IfInFilaBlockFlag:
            self.kaos_log(
                "DEBUG",
                "feed, first move P1 E?from toolhead up filament tube get output and move prz_resumeresume",
                "SERIAL",
            )
            # for UIUX dynamic interface
            self.emit_protocol("+P11:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            self.G_CutCheckTest = False
            return

        if "T" in params:
            # for UIUX dynamic interface
            self.emit_protocol("+P11 Tn:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
            self.G_ChangeChannelTimeoutOldGcmd = self.G_ChangeChannelTimeoutNewGcmd
            self.G_ChangeChannelTimeoutNewChan = int(params["T"])
            self.G_ChangeChannelTimeoutNewGcmd = gcmd

            self.G_P10SpitNum = 1

            # Vendor note (241030): P1 C1P1 C32,132
            # 1:1 2 3 4
            # 2:5 6 7 8
            # 3:9 10 11 12
            # 4:13 14 15 16
            # 5:17 18 19 20
            # 6:21 22 23 24
            # 7:25 26 27 28
            # 8:29 30 31 32
            # filament change
            self.Cmds_P1TnManualChangeChannel(int(params["T"]), gcmd)
            # for UIUX dynamic interface
            # self.emit_protocol("+P11 Tn:1,%d" % self.G_ChangeChannelTimeoutNewChan)

            self.Cmds_MoveToCutFilaAction(gcmd)

            # Vendor note (250519): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_CUT_WAITINGAREA")
            command_string = """
                # PRZ_CUT_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG",
                "[SERVICE] Move to service/waiting position; command_string='%s'" % command_string,
                "SERIAL",
            )

            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("AP")
                self.kaos_log("DEBUG", "Serial port 1 sending command: AP", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("AP")
                self.kaos_log("DEBUG", "Serial port 2 sending command: AP", "SERIAL")

            self.G_ProzenToolhead.dwell(0.5)

            # Vendor note (240913): self.G_ProzenToolhead.dwell(6.0)
            # Vendor note (231201): checkfilamentnormal,normal
            self.Cmds_CutFilaIfNormalCheck()
            if self.G_KlipperIfPaused == True:
                self.kaos_log(
                    "DEBUG",
                    "cut filament 5toolhead detectedfilament, cutter error, please check cutter, pauseklipperprinting",
                    "SERIAL",
                )
                # Lo_ChangeChannelIfSuccess = False
                # for UIUX dynamic interface
                self.emit_protocol("+P11:1,%d" % self.G_ChangeChannelTimeoutNewChan)
                self.G_CutCheckTest = False
                return

        # for UIUX dynamic interface
        self.emit_protocol("+P11:1,%d" % self.G_ChangeChannelTimeoutNewChan)
        self.G_CutCheckTest = False

    # P12 T?;loop
    # Function: handle P12 command workflow
    # Parameters: gcmd (G-code command context)
    def Cmds_CmdP12(self, gcmd):
        if gcmd is None:
            self.kaos_log("DEBUG", "[(cmds.python)Cmds_CmdP12]gcmd-None", "SERIAL")
            self.kaos_log("DEBUG", "[(cmds.python)Cmds_CmdP12]return", "SERIAL")
            return
        if gcmd is not None:
            self.kaos_log(
                "DEBUG",
                "[(cmds.python)Cmds_CmdP12]command='%s'" % (gcmd.get_commandline(),),
                "SERIAL",
            )

        # get command parameters
        params = gcmd.get_command_parameters()

        self.kaos_log(
            "DEBUG",
            "[(cmds.python)Cmds_CmdP12]command='%s'; AMS cutter" % (gcmd.get_commandline(),),
            "SERIAL",
        )

        self.kaos_log("DEBUG", "command: '%s'" % (gcmd.get_commandline(),), "SERIAL")

        # for UIUX dynamic interface
        self.emit_protocol("+P12:0,%d" % self.G_ChangeChannelTimeoutNewChan)

        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()

        # Vendor note (240527): ,becausemanual command,defaultSTM32MC
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.kaos_log(
                "DEBUG",
                "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                "SERIAL",
            )
            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("MC")
                self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("MC")
                self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")

            self.G_ProzenToolhead.dwell(2)

        self.G_KlipperIfPaused = False
        # Vendor note (240221): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0
        self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")

        self.G_CutCheckTest = True
        self.ManualCmdFlag = False

        # #if self.G_ToolheadIfHaveFilaFlag:
        # self.G_PhrozenFluiddRespondInfo("reset, cut filament; all AMSfirst")
        # # Vendor note (231205): # self.Cmds_MoveToCutFilaAndHomingXY(gcmd)

        self.kaos_log(
            "DEBUG",
            "[FIRMWARE] External macro PG104; capture variables before toolchange",
            "SERIAL",
        )
        command_string = """
            # PG104
            """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.kaos_log(
            "DEBUG",
            "[FIRMWARE] External macro PG104; command_string='%s'" % command_string,
            "SERIAL",
        )
        self.IfDoPG102Flag = True

        # Vendor note (240510): before,feed waiting zone
        # Vendor note (240306): # Vendor note (240110): waiting areabefore,execute,position
        # Vendor note (240515): before,feed waiting zone
        self.kaos_log("DEBUG", "[TOOLCHANGE] External macro PG101 retract/pre-cut", "SERIAL")
        command_string = """
            # PG101
            """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.kaos_log(
            "DEBUG",
            "[SERVICE] Move to waiting area for purge; command_string='%s'" % command_string,
            "SERIAL",
        )
        self.IfDoPG102Flag = True

        # Vendor note (240319): ,filament,prevent
        self.kaos_log(
            "DEBUG", "[TOOLCHANGE] External macro PG106; purge residue before cut", "SERIAL"
        )
        self.PG102Flag = True
        self.kaos_log("DEBUG", "self.Flag=True", "SERIAL")
        command_string = """
        # PG106
        """
        self.G_PhrozenGCode.run_script_from_command(command_string)
        self.kaos_log("DEBUG", "command_string='%s'" % command_string, "SERIAL")
        self.PG102Flag = False
        self.kaos_log("DEBUG", "self.Flag=False", "SERIAL")

        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()

        if self.G_SerialPort1OpenFlag == True:
            self.Cmds_AMSSerial1Send("AP")
            self.kaos_log("DEBUG", "Serial port 1 sending command: AP", "SERIAL")
        if self.G_SerialPort2OpenFlag == True:
            self.Cmds_AMSSerial2Send("AP")
            self.kaos_log("DEBUG", "Serial port 2 sending command: AP", "SERIAL")

        self.G_ProzenToolhead.dwell(0.5)

        # Vendor note (240913): self.G_ProzenToolhead.dwell(6.0)
        # Vendor note (231201): checkfilamentnormal,normal
        self.Cmds_CutFilaIfNormalCheck()
        if self.G_KlipperIfPaused == True:
            self.kaos_log(
                "DEBUG",
                "cut filament 5toolhead detectedfilament, cutter error, please check cutter, pauseklipperprinting",
                "SERIAL",
            )
            # Lo_ChangeChannelIfSuccess = False
            # for UIUX dynamic interface
            self.emit_protocol("+P12:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            self.G_CutCheckTest = False
            return

        if self.G_IfInFilaBlockFlag:
            self.kaos_log(
                "DEBUG",
                "feed, first move P1 E?from toolhead up filament tube get output and move prz_resumeresume",
                "SERIAL",
            )
            # for UIUX dynamic interface
            self.emit_protocol("+P12:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            self.G_CutCheckTest = False
            return

        if "T" in params:
            # for UIUX dynamic interface
            self.emit_protocol("+P12 Tn:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
            self.G_ChangeChannelTimeoutOldGcmd = self.G_ChangeChannelTimeoutNewGcmd
            self.G_ChangeChannelTimeoutNewChan = int(params["T"])
            self.G_ChangeChannelTimeoutNewGcmd = gcmd

            self.G_P10SpitNum = 1

            # Vendor note (241030): P1 C1P1 C32,132
            # 1:1 2 3 4
            # 2:5 6 7 8
            # 3:9 10 11 12
            # 4:13 14 15 16
            # 5:17 18 19 20
            # 6:21 22 23 24
            # 7:25 26 27 28
            # 8:29 30 31 32
            # filament change
            self.Cmds_P1TnManualChangeChannel(int(params["T"]), gcmd)
            # for UIUX dynamic interface
            # self.emit_protocol("+P12 Tn:1,%d" % self.G_ChangeChannelTimeoutNewChan)

            self.Cmds_MoveToCutFilaAction(gcmd)

            # Vendor note (250519): self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_CUT_WAITINGAREA")
            command_string = """
                # PRZ_CUT_WAITINGAREA
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG",
                "[SERVICE] Move to service/waiting position; command_string='%s'" % command_string,
                "SERIAL",
            )

            if self.G_SerialPort1OpenFlag == True:
                self.Cmds_AMSSerial1Send("AP")
                self.kaos_log("DEBUG", "Serial port 1 sending command: AP", "SERIAL")
            if self.G_SerialPort2OpenFlag == True:
                self.Cmds_AMSSerial2Send("AP")
                self.kaos_log("DEBUG", "Serial port 2 sending command: AP", "SERIAL")

            self.G_ProzenToolhead.dwell(0.5)

            # Vendor note (240913): self.G_ProzenToolhead.dwell(6.0)
            # Vendor note (231201): checkfilamentnormal,normal
            self.Cmds_CutFilaIfNormalCheck()
            if self.G_KlipperIfPaused == True:
                self.kaos_log(
                    "DEBUG",
                    "cut filament 5toolhead detectedfilament, cutter error, please check cutter, pauseklipperprinting",
                    "SERIAL",
                )
                # Lo_ChangeChannelIfSuccess = False
                # for UIUX dynamic interface
                self.emit_protocol("+P12:1,%d" % self.G_ChangeChannelTimeoutNewChan)
                self.G_CutCheckTest = False
                return

        # for UIUX dynamic interface
        self.emit_protocol("+P12:1,%d" % self.G_ChangeChannelTimeoutNewChan)
        self.G_CutCheckTest = False

    #'P9 X195.940 Y242.500 W3.010 H41.450 D?'
    # waiting area
    # P9
    # X[x_pos] x_pos:Xcoordinates
    # Y[y_pos] y_pos:Ycoordinates
    # W[width] width:
    # H[height] height:
    # D[0/5] D?:

    # P9
    # T[expire]
    # A[0/1];
    # expire:time,(default60)
    # A0:,continue(default)   A1:
    # Function: handle P9 command workflow
    # Parameters: gcmd (G-code command context)
    def Cmds_CmdP9(self, gcmd):
        # get command parameters
        params = gcmd.get_command_parameters()

        if self.G_KlipperIfPaused == True:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdP9]klipper pause, but received command", "SERIAL"
            )

        self.kaos_log(
            "DEBUG", "[(cmds.python)Cmds_CmdP9]command='%s'" % (gcmd.get_commandline(),), "SERIAL"
        )

        # Vendor note (20231016): P9parameterXYWH;Xcoordinates;Ycoordinates;W;H
        #'P9 X195.940 Y242.500 W3.010 H41.450 D0'
        for flag in "XYWH":
            if flag in params:
                self.G_DictChangeChannelWaitAreaParam[flag] = float(params[flag])

        self.kaos_log("DEBUG", "command: '%s'" % (gcmd.get_commandline(),), "SERIAL")

        # parameterD # D0:XYcount(default) D1:YXcountwaiting area
        #'P9 X195.940 Y242.500 W3.010 H41.450 D0'
        if "D" in params:
            direction = int(params["D"])
            # if not direction in [0, 10]:
            #     raise gcmd.error("waiting area,Dparameter[0/1] '%s'" % (gcmd.get_commandline(),))
            self.G_DictChangeChannelWaitAreaParam["D"] = direction

        # Vendor note (241031): self.G_PhrozenFluiddRespondInfo("P9 parameter number;self.G_DictChangeChannelWaitAreaParam[D]='%d'" % (self.G_DictChangeChannelWaitAreaParam["D"],))

        # parameterT # expire:time,(default60)
        #'P9 X195.940 Y242.500 W3.010 H41.450 D0'
        if "T" in params:
            expire = int(params["T"])
            # Vendor note (20231016): 60
            if expire < 60:
                self.kaos_log(
                    "DEBUG",
                    "no timeoutwhen, is 60 internal '%s'" % (gcmd.get_commandline(),),
                    "SERIAL",
                )
            self.G_DictChangeChannelWaitAreaParam["T"] = expire
            self.kaos_log("DEBUG", "Sending command: expire=%d" % expire, "SERIAL")

        # parameter A# A0:,continue(default)   A1:
        #'P9 X195.940 Y242.500 W3.010 H41.450 D0'
        if "A" in params:
            action = int(params["A"])
            if not action in [0, 1]:
                self.kaos_log(
                    "DEBUG",
                    "no timeout logical, A parameter number is [0/1] '%s'"
                    % (gcmd.get_commandline(),),
                    "SERIAL",
                )
            self.G_DictChangeChannelWaitAreaParam["A"] = action

        # Python list(List)
        # Pythondata - position,index,index0,index1,
        # Python6type,list
        # canindex,,,,check
        # ,Pythonalreadymethod
        # listPythondatatype,can
        # listdataneedtype
        # createlist,differentdata:
        # list1 = ['physics', 'chemistry', 1997, 2000]
        # list2 = [1, 2, 3, 4, 5 ]
        # list3 = ["a", "b", "c", "d"]
        # index,listindex0startlistcan
        self.ChangeWaitMoveArea = []
        # defaultmm;cfgconfigdefault
        Lo_LineWidth = self.G_ChangeChannelWaitLineWidth
        # waiting area
        Lo_WaitAreaWidth, Lo_WaitAreaHeight = abs(self.G_DictChangeChannelWaitAreaParam["W"]), abs(
            self.G_DictChangeChannelWaitAreaParam["H"]
        )
        # waiting areaXcoordinates Ycoordinates
        Lo_XBasePosition, Lo_YBasePosition = (
            self.G_DictChangeChannelWaitAreaParam["X"],
            self.G_DictChangeChannelWaitAreaParam["Y"],
        )
        self.G_XBasePosition = Lo_XBasePosition
        self.G_YBasePosition = Lo_YBasePosition

        # distance
        Lo_TotalMovingDist = Lo_WaitAreaWidth * Lo_WaitAreaHeight / Lo_LineWidth
        # ;# mm/s
        self.G_WaitAreaEachStepDist = min(
            Lo_TotalMovingDist / self.G_DictChangeChannelWaitAreaParam["T"],
            self.G_ChangeChannelWaitMaxMovementSpeed * self.G_MovementSpeedFactor,
        )

        # D0:XYcount(default) D1:YXcountwaiting area
        if self.G_DictChangeChannelWaitAreaParam["D"] == 1:
            Lo_WaitAreaWidth, Lo_WaitAreaHeight = Lo_WaitAreaHeight, Lo_WaitAreaWidth

        if self.G_WaitAreaEachStepDist > Lo_WaitAreaWidth:
            # Vendor note (231129): continuecontinue
            self.kaos_log(
                "DEBUG",
                "no parameter number;cmd='%s', that less than minstep: %.03f"
                % (gcmd.get_commandline(), self.G_WaitAreaEachStepDist),
                "SERIAL",
            )

        # waiting areadata
        for index, y in enumerate(np.arange(0.0, Lo_WaitAreaHeight, Lo_LineWidth)):
            #
            # Function name:
            # Input parameters:
            # [Translated vendor note] :
            # [Translated vendor note] Description: -20230830
            ####################################
            # [Translated vendor note] P0 M1; modemode() Yes; "MC";P0 M1;P28;P2 A1;
            # [Translated vendor note] P0 M2; mode(); "MA";P0 M2;P28;P8;
            # [Translated vendor note] P0 M3; mode;P0 M3;
            # [Translated vendor note] P28

            if len(self.ChangeWaitMoveArea) >= self.G_DictChangeChannelWaitAreaParam["T"]:
                break
            if index % 2 == 0:
                for x in np.arange(0, Lo_WaitAreaWidth, self.G_WaitAreaEachStepDist):
                    if x < Lo_WaitAreaWidth - self.G_WaitAreaEachStepDist / 2:
                        self.ChangeWaitMoveArea.append([x, y, True])
                    else:
                        self.ChangeWaitMoveArea.append([Lo_WaitAreaWidth, y, True])
                        if y + Lo_LineWidth < Lo_WaitAreaHeight:
                            self.ChangeWaitMoveArea.append(
                                (Lo_WaitAreaWidth, y + Lo_LineWidth, False)
                            )
                        break
            else:
                for x in np.arange(
                    Lo_WaitAreaWidth - self.G_WaitAreaEachStepDist,
                    0.0,
                    -self.G_WaitAreaEachStepDist,
                ):
                    if x > self.G_WaitAreaEachStepDist / 2:
                        self.ChangeWaitMoveArea.append([x, y, True])
                    else:
                        self.ChangeWaitMoveArea.append([0, y, False])
                        break

        # D0:XYcount(default) D1:YXcountwaiting area
        if self.G_DictChangeChannelWaitAreaParam["D"] == 1:
            self.ChangeWaitMoveArea = [[y, x, b] for [x, y, b] in self.ChangeWaitMoveArea]

        # W
        if self.G_DictChangeChannelWaitAreaParam["W"] < 0:
            self.ChangeWaitMoveArea = [[-x, y, b] for [x, y, b] in self.ChangeWaitMoveArea]

        # H
        if self.G_DictChangeChannelWaitAreaParam["H"] < 0:
            self.ChangeWaitMoveArea = [[x, -y, b] for [x, y, b] in self.ChangeWaitMoveArea]

        self.ChangeWaitMoveArea = [
            [x + Lo_XBasePosition, y + Lo_YBasePosition, b] for [x, y, b] in self.ChangeWaitMoveArea
        ]

    def Cmds_CmdP0M3P8FA(self, AMSNum, gcmd):
        # if not self.G_SerialPort1OpenFlag:
        #     self.G_PhrozenFluiddRespondInfo("AMS multi-material not connected, please send P28 first")
        #     return

        self.G_ProzenToolhead.dwell(2.0)

        self.kaos_log("DEBUG", "[(cmds.python)Cmds_CmdP0M3P8FA]command=P8FA", "SERIAL")

        Lo_MCUSTM32Cmd = G_DictPhrozenCmdP8["mcu_cmd"][0]
        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()
        if AMSNum == 1:
            self.Cmds_AMSSerial1Send("MA")
            self.kaos_log("DEBUG", "serial port 1 send MA", "SERIAL")
        elif AMSNum == 2:
            self.Cmds_AMSSerial2Send("MA")
            self.kaos_log("DEBUG", "serial port 2 send MA", "SERIAL")

        # Vendor note (240124): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0

        # Vendor note (240123): iffilament,FAstm32,completeafterFA,
        if self.G_ToolheadIfHaveFilaFlag == False:
            self.kaos_log("DEBUG", "toolhead has filament, FA", "SERIAL")
            # Vendor note (240115): 2,prevent
            # time.sleep(2)
            self.G_ProzenToolhead.dwell(2.0)

            if AMSNum == 1:
                self.Cmds_AMSSerial1Send("FA")
                self.kaos_log("DEBUG", "serial port 1 send FA", "SERIAL")
            elif AMSNum == 2:
                self.Cmds_AMSSerial2Send("FA")
                self.kaos_log("DEBUG", "serial port 2 send FA", "SERIAL")

            # Vendor note (231229): encapsulated function,
            self.Cmds_MARetryInFila(gcmd)
            # Vendor note (240108): P8
            self.G_M2MAModeResumeFlag = False

        else:  # filament
            self.kaos_log("DEBUG", "toolhead has filament, FB", "SERIAL")
            # time.sleep(2)
            self.G_ProzenToolhead.dwell(2.0)

            if AMSNum == 1:
                self.Cmds_AMSSerial1Send("FB")
                self.kaos_log("DEBUG", "serial port 1 send FB", "SERIAL")
            elif AMSNum == 2:
                self.Cmds_AMSSerial2Send("FB")
                self.kaos_log("DEBUG", "serial port 2 send FB", "SERIAL")

            self.G_M2MAModeResumeFlag = False

    def Cmds_P8AMS1AutoSelectChannel(self):
        self.kaos_log("DEBUG", "[(cmds.python)Cmds_P8AMS1AutoSelectChannel]", "SERIAL")

        bitmask1 = 0b0001
        bitmask2 = 0b0010
        bitmask4 = 0b0100
        bitmask8 = 0b1000
        if self.G_AMS1DeviceState["entry_state"] == 0:
            if self.G_AMS1DeviceState["park_state"] & bitmask1 == 1:  # 0001
                self.kaos_log("DEBUG", "serial port 1Sending command: T1", "SERIAL")
                self.Cmds_AMSSerial1Send("T1")
                if self.G_ChangeChannelTimeoutOldChan < 0 or self.G_ChangeChannelTimeoutNewChan < 0:
                    self.G_ChangeChannelTimeoutOldChan = 1
                else:
                    self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan = 1
                self.emit_protocol("+T:0,1")
            elif self.G_AMS1DeviceState["park_state"] & bitmask2 == 2:  # 0010
                self.kaos_log("DEBUG", "serial port 1Sending command: T2", "SERIAL")
                self.Cmds_AMSSerial1Send("T2")
                if self.G_ChangeChannelTimeoutOldChan < 0 or self.G_ChangeChannelTimeoutNewChan < 0:
                    self.G_ChangeChannelTimeoutOldChan = 2
                else:
                    self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan = 2
                self.emit_protocol("+T:0,2")
            elif self.G_AMS1DeviceState["park_state"] & bitmask4 == 4:  # 0100
                self.kaos_log("DEBUG", "serial port 1Sending command: T3", "SERIAL")
                self.Cmds_AMSSerial1Send("T3")
                if self.G_ChangeChannelTimeoutOldChan < 0 or self.G_ChangeChannelTimeoutNewChan < 0:
                    self.G_ChangeChannelTimeoutOldChan = 3
                else:
                    self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan = 3
                self.emit_protocol("+T:0,3")
            elif self.G_AMS1DeviceState["park_state"] & bitmask8 == 8:  # 1000
                self.kaos_log("DEBUG", "serial port 1Sending command: T4", "SERIAL")
                self.Cmds_AMSSerial1Send("T4")
                if self.G_ChangeChannelTimeoutOldChan < 0 or self.G_ChangeChannelTimeoutNewChan < 0:
                    self.G_ChangeChannelTimeoutOldChan = 4
                else:
                    self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan = 4
                self.emit_protocol("+T:0,4")
            else:
                self.kaos_log("DEBUG", "no filament", "SERIAL")
        elif self.G_AMS1DeviceState["entry_state"] & bitmask1 == 1:  # 0001
            self.kaos_log("DEBUG", "serial port 1Sending command: T1", "SERIAL")
            self.Cmds_AMSSerial1Send("T1")
            if self.G_ChangeChannelTimeoutOldChan < 0 or self.G_ChangeChannelTimeoutNewChan < 0:
                self.G_ChangeChannelTimeoutOldChan = 1
            else:
                self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
            self.G_ChangeChannelTimeoutNewChan = 1
            self.emit_protocol("+T:0,1")
        elif self.G_AMS1DeviceState["entry_state"] & bitmask2 == 2:  # 0010
            if self.G_AMS1DeviceState["park_state"] & bitmask1 == 1:  # 0001
                self.kaos_log("DEBUG", "serial port 1Sending command: T1", "SERIAL")
                self.Cmds_AMSSerial1Send("T1")
                if self.G_ChangeChannelTimeoutOldChan < 0 or self.G_ChangeChannelTimeoutNewChan < 0:
                    self.G_ChangeChannelTimeoutOldChan = 1
                else:
                    self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan = 1
                self.emit_protocol("+T:0,1")
            elif self.G_AMS1DeviceState["park_state"] & bitmask2 == 2:  # 0010
                self.kaos_log("DEBUG", "serial port 1Sending command: T2", "SERIAL")
                self.Cmds_AMSSerial1Send("T2")
                if self.G_ChangeChannelTimeoutOldChan < 0 or self.G_ChangeChannelTimeoutNewChan < 0:
                    self.G_ChangeChannelTimeoutOldChan = 2
                else:
                    self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan = 2
                self.emit_protocol("+T:0,2")
            else:
                self.kaos_log("DEBUG", "serial port 1Sending command: T2", "SERIAL")
                self.Cmds_AMSSerial1Send("T2")
                if self.G_ChangeChannelTimeoutOldChan < 0 or self.G_ChangeChannelTimeoutNewChan < 0:
                    self.G_ChangeChannelTimeoutOldChan = 2
                else:
                    self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan = 2
                self.emit_protocol("+T:0,2")
        elif self.G_AMS1DeviceState["entry_state"] & bitmask4 == 4:  # 0100
            if self.G_AMS1DeviceState["park_state"] & bitmask1 == 1:  # 0001
                self.kaos_log("DEBUG", "serial port 1Sending command: T1", "SERIAL")
                self.Cmds_AMSSerial1Send("T1")
                if self.G_ChangeChannelTimeoutOldChan < 0 or self.G_ChangeChannelTimeoutNewChan < 0:
                    self.G_ChangeChannelTimeoutOldChan = 1
                else:
                    self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan = 1
                self.emit_protocol("+T:0,1")
            elif self.G_AMS1DeviceState["park_state"] & bitmask2 == 2:  # 0010
                self.kaos_log("DEBUG", "serial port 1Sending command: T2", "SERIAL")
                self.Cmds_AMSSerial1Send("T2")
                if self.G_ChangeChannelTimeoutOldChan < 0 or self.G_ChangeChannelTimeoutNewChan < 0:
                    self.G_ChangeChannelTimeoutOldChan = 2
                else:
                    self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan = 2
                self.emit_protocol("+T:0,2")
            elif self.G_AMS1DeviceState["park_state"] & bitmask4 == 4:  # 0100
                self.kaos_log("DEBUG", "serial port 1Sending command: T3", "SERIAL")
                self.Cmds_AMSSerial1Send("T3")
                if self.G_ChangeChannelTimeoutOldChan < 0 or self.G_ChangeChannelTimeoutNewChan < 0:
                    self.G_ChangeChannelTimeoutOldChan = 3
                else:
                    self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan = 3
                self.emit_protocol("+T:0,3")
            else:
                self.kaos_log("DEBUG", "serial port 1Sending command: T3", "SERIAL")
                self.Cmds_AMSSerial1Send("T3")
                if self.G_ChangeChannelTimeoutOldChan < 0 or self.G_ChangeChannelTimeoutNewChan < 0:
                    self.G_ChangeChannelTimeoutOldChan = 3
                else:
                    self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan = 3
                self.emit_protocol("+T:0,3")
        elif self.G_AMS1DeviceState["entry_state"] & bitmask8 == 8:  # 1000
            if self.G_AMS1DeviceState["park_state"] & bitmask1 == 1:  # 0001
                self.kaos_log("DEBUG", "serial port 1Sending command: T1", "SERIAL")
                self.Cmds_AMSSerial1Send("T1")
                if self.G_ChangeChannelTimeoutOldChan < 0 or self.G_ChangeChannelTimeoutNewChan < 0:
                    self.G_ChangeChannelTimeoutOldChan = 1
                else:
                    self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan = 1
                self.emit_protocol("+T:0,1")
            elif self.G_AMS1DeviceState["park_state"] & bitmask2 == 2:  # 0010
                self.kaos_log("DEBUG", "serial port 1Sending command: T2", "SERIAL")
                self.Cmds_AMSSerial1Send("T2")
                if self.G_ChangeChannelTimeoutOldChan < 0 or self.G_ChangeChannelTimeoutNewChan < 0:
                    self.G_ChangeChannelTimeoutOldChan = 2
                else:
                    self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan = 2
                self.emit_protocol("+T:0,2")
            # [Translated vendor note] #Vendor note (20231019): print, filament change1channeltoolheadfilament, filament
            # [Translated vendor note] #Vendor note (20231020): toolhead
            # #if self.G_ToolheadIfHaveFilaFlag:
            # [Translated vendor note] # # 0=defaultgcode
            # [Translated vendor note] Vendor note (231128): G28PG28
            # if self.G_ChangeChannelIfZLiftingUpByGcode == 0:
            # [Translated vendor note] self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP28]")
            # command_string = """
            # PG28
            # """
            # self.G_PhrozenGCode.run_script_from_command(command_string)
            # [Translated vendor note] #Vendor note (20231020): gcode, toolhead, , , filament change
            # # G92 E0
            # # G1 E0.0000 F600
            # # G91
            # # G1 E-0.385 F8000
            # [Translated vendor note] self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP28]")
            # [Translated vendor note] #Vendor note (20231013):
            # self.Cmds_MoveToCutFilaAction(gcmd)
            # [Translated vendor note] #self.G_PhrozenFluiddRespondInfo(": AP, park position")
            # [Translated vendor note] #// park position; //===== P2 A1 park positionprint Yes; "AP";
            # #Klipper state: Shutdown
            # #!! Internal error on command:"P28"
            # [Translated vendor note] #ttyUSB0, klippr
            # #self.Cmds_AMSSerial1Send("AP")

            elif self.G_AMS1DeviceState["park_state"] & bitmask4 == 4:  # 0100
                self.kaos_log("DEBUG", "serial port 1Sending command: T3", "SERIAL")
                self.Cmds_AMSSerial1Send("T3")
                if self.G_ChangeChannelTimeoutOldChan < 0 or self.G_ChangeChannelTimeoutNewChan < 0:
                    self.G_ChangeChannelTimeoutOldChan = 3
                else:
                    self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan = 3
                self.emit_protocol("+T:0,3")
            elif self.G_AMS1DeviceState["park_state"] & bitmask8 == 8:  # 1000
                self.kaos_log("DEBUG", "serial port 1Sending command: T4", "SERIAL")
                self.Cmds_AMSSerial1Send("T4")
                if self.G_ChangeChannelTimeoutOldChan < 0 or self.G_ChangeChannelTimeoutNewChan < 0:
                    self.G_ChangeChannelTimeoutOldChan = 4
                else:
                    self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan = 4
                self.emit_protocol("+T:0,4")
            else:
                self.kaos_log("DEBUG", "serial port 1Sending command: T4", "SERIAL")
                self.Cmds_AMSSerial1Send("T4")
                if self.G_ChangeChannelTimeoutOldChan < 0 or self.G_ChangeChannelTimeoutNewChan < 0:
                    self.G_ChangeChannelTimeoutOldChan = 4
                else:
                    self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                self.G_ChangeChannelTimeoutNewChan = 4
                # for UIUX dynamic interface
                self.emit_protocol("+T:0,4")

    # P8 execute Yes;"FA";
    def Cmds_CmdP8(self, gcmd):
        # if not self.G_SerialPort1OpenFlag:
        #     self.G_PhrozenFluiddRespondInfo("AMS multi-material not connected, please send P28 first")
        #     return
        # Vendor note (250522): allowM3detect
        self.G_IfChangeFilaOngoing = True

        self.G_ProzenToolhead.dwell(2.0)

        self.kaos_log("DEBUG", "[(cmds.python)Cmds_CmdP8]command=P8", "SERIAL")

        self.kaos_log("DEBUG", "Current mode", "SERIAL")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.emit_mode(0, "unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.emit_mode(1, "MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.emit_mode(2, "MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.emit_mode(3, "RUNOUT")
        else:
            self.emit_mode(-1, "error")

        Lo_MCUSTM32Cmd = G_DictPhrozenCmdP8["mcu_cmd"][0]

        # Vendor note (240511): on resume, reinitialize serial to handle AMS hot-plug serial errors
        try:
            self.kaos_log(
                "DEBUG", "[(cmds.python)Cmds_CmdP8]Reinitializing serial port 1", "SERIAL"
            )
            self.G_SerialPort1Obj = serial.Serial(
                self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3
            )
            # serial port opened successfully
            if self.G_SerialPort1Obj is not None:
                if self.G_SerialPort1Obj.is_open:
                    self.G_SerialPort1OpenFlag = True
                    self.kaos_log("DEBUG", "Reinitializing serial port 1 successful", "SERIAL")
                    # Vendor note (231213): open serial port
                    self.G_SerialPort1Obj.flushInput()  # clean serial write cache
                    self.G_SerialPort1Obj.flush()
                    self.kaos_log("DEBUG", "Serial port 1 buffers cleared", "SERIAL")
                    self.kaos_log("DEBUG", "Re-registering serial port 1 callback", "SERIAL")
                    self.G_SerialPort1RecvTimmer = self.G_PhrozenReactor.register_timer(
                        self.Device_TimmerUart1Recv, self.G_PhrozenReactor.NOW
                    )
        except:
            self.kaos_log(
                "DEBUG", "Unable to open tty1. Check the USB connection or try rebooting.", "SERIAL"
            )

        try:
            self.kaos_log(
                "DEBUG",
                "[(cmds.py)Cmds_PhrozenKlipperResume]Reinitializing serial port 2",
                "SERIAL",
            )
            self.G_SerialPort2Obj = serial.Serial(
                self.G_Serialport2Define, SERIAL_PORT_BAUD, timeout=3
            )
            # serial port opened successfully
            if self.G_SerialPort2Obj is not None:
                if self.G_SerialPort2Obj.is_open:
                    self.G_SerialPort2OpenFlag = True
                    self.kaos_log("DEBUG", "Reinitializing serial port 2 successful", "SERIAL")
                    self.G_SerialPort2Obj.flushInput()  # clean serial write cache
                    self.G_SerialPort2Obj.flush()
                    self.kaos_log("DEBUG", "Serial port 2 buffers cleared", "SERIAL")
                    self.kaos_log("DEBUG", "Re-registering serial port 2 callback", "SERIAL")
                    self.G_SerialPort2RecvTimmer = self.G_PhrozenReactor.register_timer(
                        self.Device_TimmerUart2Recv, self.G_PhrozenReactor.NOW
                    )
        except:
            self.kaos_log(
                "DEBUG", "Unable to open tty2. Check the USB connection or try rebooting.", "SERIAL"
            )

        if self.G_SerialPort1OpenFlag == True:
            self.kaos_log("DEBUG", "serial port 1 send MA", "SERIAL")
            self.Cmds_AMSSerial1Send("MA")

        if self.G_SerialPort2OpenFlag == True:
            self.kaos_log("DEBUG", "serial port 2 send MA", "SERIAL")
            self.Cmds_AMSSerial2Send("MA")

        # Vendor note (240124): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0

        # # Vendor note (240123): iffilament,FAstm32,completeafterFA,
        # if self.G_ToolheadIfHaveFilaFlag==False:
        #     self.G_PhrozenFluiddRespondInfo("toolhead has filament, FA")
        #     # Vendor note (240115): 2,prevent
        #     #time.sleep(2)
        #     self.G_ProzenToolhead.dwell(2.0)

        #     # Vendor note (241030): #     if self.G_SerialPort1OpenFlag == True:
        #         self.Cmds_AMSSerial1Send("FA")
        #         self.G_PhrozenFluiddRespondInfo("serial port 1 send FA")
        #     elif self.G_SerialPort2OpenFlag == True:
        #         self.Cmds_AMSSerial2Send("FA")
        #         self.G_PhrozenFluiddRespondInfo("serial port 2 send FA")

        #     # Vendor note (231229): encapsulated function
        #     self.Cmds_MARetryInFila(gcmd)
        #     # Vendor note (240108): P8
        #     self.G_M2MAModeResumeFlag=False

        # else:#filament
        #     self.G_PhrozenFluiddRespondInfo("toolhead has filament, FB")
        #     #time.sleep(2)
        #     self.G_ProzenToolhead.dwell(2.0)

        #     # Vendor note (241030): #     if self.G_SerialPort1OpenFlag == True:
        #         self.Cmds_AMSSerial1Send("FB")
        #         self.G_PhrozenFluiddRespondInfo("serial port 1 send FB")
        #     elif self.G_SerialPort2OpenFlag == True:
        #         self.Cmds_AMSSerial2Send("FB")
        #         self.G_PhrozenFluiddRespondInfo("serial port 2 send FB")

        #     self.G_M2MAModeResumeFlag=False

        # Vendor note (241105): if,currentfilamentAMS,
        # Vendor note (231205): self.Cmds_MoveToCutFilaAndRollback(gcmd)

        # # Vendor note (231205): # self.Cmds_MoveToCutFilaAndNotRollback(gcmd)
        # self.G_PhrozenFluiddRespondInfo("All AMS retract first")
        # # Vendor note (241030): # if self.G_SerialPort1OpenFlag == True:
        #     self.Cmds_AMSSerial1Send("AP")
        #     self.G_PhrozenFluiddRespondInfo("Serial port 1 sending command: AP")
        # # Vendor note (241030): # if self.G_SerialPort2OpenFlag == True:
        #     self.Cmds_AMSSerial2Send("AP")
        #     self.G_PhrozenFluiddRespondInfo("Serial port 2 sending command: AP")
        # # Vendor note (240913): # self.G_ProzenToolhead.dwell(6.0)
        # # Vendor note (231201): checkfilamentnormal,normal
        # self.Cmds_CutFilaIfNormalCheck()

        if self.G_KlipperIfPaused == True:
            self.kaos_log(
                "DEBUG",
                "cut filament 5toolhead detectedfilament, cutter error, please check cutter, pauseklipperprinting",
                "SERIAL",
            )
            # Lo_ChangeChannelIfSuccess = False
            self.G_PauseToLCDString = "+PAUSE:8,%d,%d" % (
                self.G_ChangeChannelTimeoutOldChan,
                self.G_ChangeChannelTimeoutNewChan,
            )
            self.kaos_log("DEBUG", self.G_PauseToLCDString, "SERIAL")
            self.G_IfChangeFilaOngoing = False
            return

        # ifnormal,AMS
        if self.G_KlipperIfPaused == False:
            self.G_ProzenToolhead.dwell(2.0)

            if self.G_SerialPort1OpenFlag == True:
                try:
                    self.kaos_log("DEBUG", "try;Lo_AMSDeviceStateRspInfo", "SERIAL")
                    # get detailed AMS board state
                    Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort1SendWaitRsp(
                        "SD", sizeof(AMSDetailInfoBytes)
                    )
                    self.kaos_log(
                        "DEBUG",
                        "ttyserial port 1 receive : %s" % Lo_AMSDeviceStateRspInfo,
                        "SERIAL",
                    )
                    if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                        self.kaos_log("DEBUG", "AMS1, please checkAMS", "SERIAL")
                        # Vendor note (240412): AMSlabel
                        self.G_AMSDevice1IfNormal = False
                    else:
                        self.kaos_log("DEBUG", "AMS1 connectedsuccessful", "SERIAL")
                        self.kaos_log("DEBUG", "self.G_AMSDevice1IfNormal=True", "SERIAL")
                        # Vendor note (240412): AMSlabel
                        self.G_AMSDevice1IfNormal = True

                        Lo_AMSDeviceStateInfo = AMSDetailInfoBytes()
                        Lo_AMSDeviceStateInfo.whole[:] = bytearray(Lo_AMSDeviceStateRspInfo)
                        # self.G_PhrozenFluiddRespondInfo("%s" % SerialRxASCIIStr)
                        # empty Python dict
                        Lo_AMSDetailState = {}
                        self.G_AMS1DeviceState["dev_id"] = Lo_AMSDetailState["dev_id"] = (
                            Lo_AMSDeviceStateInfo.field.dev_id
                        )
                        self.G_AMS1DeviceState["active_dev_id"] = Lo_AMSDetailState[
                            "active_dev_id"
                        ] = Lo_AMSDeviceStateInfo.field.active_dev_id
                        self.G_AMS1DeviceState["dev_mode"] = Lo_AMSDetailState["dev_mode"] = (
                            Lo_AMSDeviceStateInfo.field.dev_mode
                        )
                        self.G_AMS1DeviceState["cache_empty"] = Lo_AMSDetailState["cache_empty"] = (
                            Lo_AMSDeviceStateInfo.field.cache_empty
                        )
                        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Buffer sensor empty state (bool) == %d" % Lo_AMSDeviceStateInfo.field.cache_empty)
                        self.G_AMS1DeviceState["cache_full"] = Lo_AMSDetailState["cache_full"] = (
                            Lo_AMSDeviceStateInfo.field.cache_full
                        )
                        self.kaos_log(
                            "DEBUG",
                            "buffer device fullstate(bool)==%d"
                            % Lo_AMSDeviceStateInfo.field.cache_full,
                            "SERIAL",
                        )
                        self.G_AMS1DeviceState["cache_exist"] = Lo_AMSDetailState["cache_exist"] = (
                            Lo_AMSDeviceStateInfo.field.cache_exist
                        )
                        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Buffer sensor filament state (bool) == %d" % Lo_AMSDeviceStateInfo.field.cache_exist)
                        self.G_AMS1DeviceState["mc_state"] = Lo_AMSDetailState["mc_state"] = (
                            Lo_AMSDeviceStateInfo.field.mc_state
                        )
                        self.G_AMS1DeviceState["ma_state"] = Lo_AMSDetailState["ma_state"] = (
                            Lo_AMSDeviceStateInfo.field.ma_state
                        )
                        self.G_AMS1DeviceState["entry_state"] = Lo_AMSDetailState["entry_state"] = (
                            Lo_AMSDeviceStateInfo.field.entry_state
                        )
                        self.kaos_log(
                            "DEBUG",
                            "entry device state(bit)==%d" % Lo_AMSDeviceStateInfo.field.entry_state,
                            "SERIAL",
                        )
                        self.G_AMS1DeviceState["park_state"] = Lo_AMSDetailState["park_state"] = (
                            Lo_AMSDeviceStateInfo.field.park_state
                        )
                        self.kaos_log(
                            "DEBUG",
                            "park position device state(bit)==%d"
                            % Lo_AMSDeviceStateInfo.field.park_state,
                            "SERIAL",
                        )
                except:
                    self.kaos_log("DEBUG", "except;Lo_AMSDeviceStateRspInfo", "SERIAL")

            if self.G_SerialPort2OpenFlag == True:
                try:
                    self.kaos_log("DEBUG", "try;Lo_AMSDeviceStateRspInfo", "SERIAL")
                    Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort2SendWaitRsp(
                        "SD", sizeof(AMSDetailInfoBytes)
                    )
                    self.kaos_log(
                        "DEBUG",
                        "ttyserial port 2 receive : %s" % Lo_AMSDeviceStateRspInfo,
                        "SERIAL",
                    )
                    if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                        self.kaos_log("DEBUG", "AMS2, please checkAMS", "SERIAL")
                        # Vendor note (240412): AMSlabel
                        self.G_AMSDevice2IfNormal = False
                    else:
                        self.kaos_log("DEBUG", "AMS2 connectedsuccessful", "SERIAL")
                        self.kaos_log("DEBUG", "self.G_AMSDevice2IfNormal=True", "SERIAL")
                        # Vendor note (240412): AMSlabel
                        self.G_AMSDevice2IfNormal = True

                        Lo_AMSDeviceStateInfo = AMSDetailInfoBytes()
                        Lo_AMSDeviceStateInfo.whole[:] = bytearray(Lo_AMSDeviceStateRspInfo)
                        # empty Python dict
                        Lo_AMSDetailState = {}
                        self.G_AMS2DeviceState["dev_id"] = Lo_AMSDetailState["dev_id"] = (
                            Lo_AMSDeviceStateInfo.field.dev_id
                        )
                        self.G_AMS2DeviceState["active_dev_id"] = Lo_AMSDetailState[
                            "active_dev_id"
                        ] = Lo_AMSDeviceStateInfo.field.active_dev_id
                        self.G_AMS2DeviceState["dev_mode"] = Lo_AMSDetailState["dev_mode"] = (
                            Lo_AMSDeviceStateInfo.field.dev_mode
                        )
                        self.G_AMS2DeviceState["cache_empty"] = Lo_AMSDetailState["cache_empty"] = (
                            Lo_AMSDeviceStateInfo.field.cache_empty
                        )
                        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Buffer sensor empty state (bool) == %d" % Lo_AMSDeviceStateInfo.field.cache_empty)
                        self.G_AMS2DeviceState["cache_full"] = Lo_AMSDetailState["cache_full"] = (
                            Lo_AMSDeviceStateInfo.field.cache_full
                        )
                        self.kaos_log(
                            "DEBUG",
                            "buffer device fullstate(bool)==%d"
                            % Lo_AMSDeviceStateInfo.field.cache_full,
                            "SERIAL",
                        )
                        self.G_AMS2DeviceState["cache_exist"] = Lo_AMSDetailState["cache_exist"] = (
                            Lo_AMSDeviceStateInfo.field.cache_exist
                        )
                        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Buffer sensor filament state (bool) == %d" % Lo_AMSDeviceStateInfo.field.cache_exist)
                        self.G_AMS2DeviceState["mc_state"] = Lo_AMSDetailState["mc_state"] = (
                            Lo_AMSDeviceStateInfo.field.mc_state
                        )
                        self.G_AMS2DeviceState["ma_state"] = Lo_AMSDetailState["ma_state"] = (
                            Lo_AMSDeviceStateInfo.field.ma_state
                        )
                        self.G_AMS2DeviceState["entry_state"] = Lo_AMSDetailState["entry_state"] = (
                            Lo_AMSDeviceStateInfo.field.entry_state
                        )
                        self.kaos_log(
                            "DEBUG",
                            "entry device state(bit)==%d" % Lo_AMSDeviceStateInfo.field.entry_state,
                            "SERIAL",
                        )
                        self.G_AMS2DeviceState["park_state"] = Lo_AMSDetailState["park_state"] = (
                            Lo_AMSDeviceStateInfo.field.park_state
                        )
                        self.kaos_log(
                            "DEBUG",
                            "park position device state(bit)==%d"
                            % Lo_AMSDeviceStateInfo.field.park_state,
                            "SERIAL",
                        )

                except:
                    self.kaos_log("DEBUG", "except;Lo_AMSDeviceStateRspInfo", "SERIAL")

        self.G_ProzenToolhead.dwell(2.0)

        if self.G_AMSDevice1IfNormal == True:

            # Vendor note (241106): AMS
            if (
                self.G_AMS1DeviceState["entry_state"] > 0
                or self.G_AMS1DeviceState["park_state"] > 0
            ):
                self.kaos_log("DEBUG", "#1 AMS has filament", "SERIAL")
                # Vendor note (250711): ifcolor,;
                # =====M3
                if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:  # M3 M2
                    # if self.G_ChromaKitNum>0:
                    self.kaos_log(
                        "DEBUG",
                        "M3 mode single-color model, user selected multi-material single-channel to print single-color;",
                        "SERIAL",
                    )
                    if self.G_ChromaKitAccessT0 > 0:
                        self.Cmds_AMSSerial1Send("T%d" % self.G_ChromaKitAccessT0)
                        self.kaos_log(
                            "DEBUG",
                            "serial port 1Sending command: T%d" % self.G_ChromaKitAccessT0,
                            "SERIAL",
                        )
                        if (
                            self.G_ChangeChannelTimeoutOldChan < 0
                            or self.G_ChangeChannelTimeoutNewChan < 0
                        ):
                            self.G_ChangeChannelTimeoutOldChan = self.G_ChromaKitAccessT0
                        else:
                            self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan = self.G_ChromaKitAccessT0
                    elif self.G_ChromaKitAccessT1 > 0:
                        self.Cmds_AMSSerial1Send("T%d" % self.G_ChromaKitAccessT1)
                        self.kaos_log(
                            "DEBUG",
                            "serial port 1Sending command: T%d" % self.G_ChromaKitAccessT1,
                            "SERIAL",
                        )
                        if (
                            self.G_ChangeChannelTimeoutOldChan < 0
                            or self.G_ChangeChannelTimeoutNewChan < 0
                        ):
                            self.G_ChangeChannelTimeoutOldChan = self.G_ChromaKitAccessT1
                        else:
                            self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan = self.G_ChromaKitAccessT1
                    elif self.G_ChromaKitAccessT2 > 0:
                        self.Cmds_AMSSerial1Send("T%d" % self.G_ChromaKitAccessT2)
                        self.kaos_log(
                            "DEBUG",
                            "serial port 1Sending command: T%d" % self.G_ChromaKitAccessT2,
                            "SERIAL",
                        )
                        if (
                            self.G_ChangeChannelTimeoutOldChan < 0
                            or self.G_ChangeChannelTimeoutNewChan < 0
                        ):
                            self.G_ChangeChannelTimeoutOldChan = self.G_ChromaKitAccessT2
                        else:
                            self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan = self.G_ChromaKitAccessT2
                    elif self.G_ChromaKitAccessT3 > 0:
                        self.Cmds_AMSSerial1Send("T%d" % self.G_ChromaKitAccessT3)
                        self.kaos_log(
                            "DEBUG",
                            "serial port 1Sending command: T%d" % self.G_ChromaKitAccessT3,
                            "SERIAL",
                        )
                        if (
                            self.G_ChangeChannelTimeoutOldChan < 0
                            or self.G_ChangeChannelTimeoutNewChan < 0
                        ):
                            self.G_ChangeChannelTimeoutOldChan = self.G_ChromaKitAccessT3
                        else:
                            self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan = self.G_ChromaKitAccessT3
                    else:
                        self.kaos_log(
                            "DEBUG",
                            "M3 mode single-color model, user did not select multi-material channel; auto-select channel",
                            "SERIAL",
                        )
                        self.Cmds_P8AMS1AutoSelectChannel()
                else:
                    self.kaos_log(
                        "DEBUG",
                        "mode single-color type, use has multi-material printing single-color type; move printing single-color type",
                        "SERIAL",
                    )
                    self.Cmds_P8AMS1AutoSelectChannel()
            else:
                self.kaos_log("DEBUG", "#1 AMSno filament", "SERIAL")

        if self.G_AMSDevice2IfNormal == True:
            if self.G_AMS2DeviceState["entry_state"] > 0:
                self.kaos_log("DEBUG", "#2 AMS has filament", "SERIAL")

        self.Cmds_PrintMode(self.G_AMSDeviceWorkMode)

        # Vendor note (231229): encapsulated function,
        self.Cmds_MARetryInFila(gcmd)

        # Vendor note (240108): P8
        self.G_M2MAModeResumeFlag = False

    def Cmds_CmdP8Infila(self):
        # self.G_ProzenToolhead.dwell(2.0)

        self.kaos_log("DEBUG", "[(cmds.python)Cmds_CmdP8Infila]", "SERIAL")

        self.kaos_log("DEBUG", "Current mode", "SERIAL")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.emit_mode(0, "unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.emit_mode(1, "MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.emit_mode(2, "MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.emit_mode(3, "RUNOUT")
        else:
            self.emit_mode(-1, "error")

        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()
        if self.G_SerialPort1OpenFlag == True:
            self.Cmds_AMSSerial1Send("MB")
            self.kaos_log("DEBUG", "serial port 1 send MB", "SERIAL")
        elif self.G_SerialPort2OpenFlag == True:
            self.Cmds_AMSSerial2Send("MB")
            self.kaos_log("DEBUG", "serial port 2 send MB", "SERIAL")

        # Vendor note (240124): STM32 active report, allow one pause
        self.STM32ReprotPauseFlag = 0

        self.G_ProzenToolhead.dwell(2.5)

        if self.G_SerialPort1OpenFlag == True:
            try:
                self.kaos_log("DEBUG", "try;Lo_AMSDeviceStateRspInfo", "SERIAL")
                # get detailed AMS board state
                Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort1SendWaitRsp(
                    "SD", sizeof(AMSDetailInfoBytes)
                )
                self.kaos_log(
                    "DEBUG", "ttyserial port 1 receive : %s" % Lo_AMSDeviceStateRspInfo, "SERIAL"
                )
                if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                    self.kaos_log("DEBUG", "AMS1, please checkAMS", "SERIAL")
                    # Vendor note (240412): AMSlabel
                    self.G_AMSDevice1IfNormal = False
                else:
                    self.kaos_log("DEBUG", "AMS1 connectedsuccessful", "SERIAL")
                    self.kaos_log("DEBUG", "self.G_AMSDevice1IfNormal=True", "SERIAL")
                    # Vendor note (240412): AMSlabel
                    self.G_AMSDevice1IfNormal = True

                    Lo_AMSDeviceStateInfo = AMSDetailInfoBytes()
                    Lo_AMSDeviceStateInfo.whole[:] = bytearray(Lo_AMSDeviceStateRspInfo)
                    # self.G_PhrozenFluiddRespondInfo("%s" % SerialRxASCIIStr)
                    # empty Python dict
                    Lo_AMSDetailState = {}
                    self.G_AMS1DeviceState["dev_id"] = Lo_AMSDetailState["dev_id"] = (
                        Lo_AMSDeviceStateInfo.field.dev_id
                    )
                    self.G_AMS1DeviceState["active_dev_id"] = Lo_AMSDetailState["active_dev_id"] = (
                        Lo_AMSDeviceStateInfo.field.active_dev_id
                    )
                    self.G_AMS1DeviceState["dev_mode"] = Lo_AMSDetailState["dev_mode"] = (
                        Lo_AMSDeviceStateInfo.field.dev_mode
                    )
                    self.G_AMS1DeviceState["cache_empty"] = Lo_AMSDetailState["cache_empty"] = (
                        Lo_AMSDeviceStateInfo.field.cache_empty
                    )
                    # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Buffer sensor empty state (bool) == %d" % Lo_AMSDeviceStateInfo.field.cache_empty)
                    self.G_AMS1DeviceState["cache_full"] = Lo_AMSDetailState["cache_full"] = (
                        Lo_AMSDeviceStateInfo.field.cache_full
                    )
                    self.kaos_log(
                        "DEBUG",
                        "buffer device fullstate(bool)==%d"
                        % Lo_AMSDeviceStateInfo.field.cache_full,
                        "SERIAL",
                    )
                    self.G_AMS1DeviceState["cache_exist"] = Lo_AMSDetailState["cache_exist"] = (
                        Lo_AMSDeviceStateInfo.field.cache_exist
                    )
                    # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Buffer sensor filament state (bool) == %d" % Lo_AMSDeviceStateInfo.field.cache_exist)
                    self.G_AMS1DeviceState["mc_state"] = Lo_AMSDetailState["mc_state"] = (
                        Lo_AMSDeviceStateInfo.field.mc_state
                    )
                    self.G_AMS1DeviceState["ma_state"] = Lo_AMSDetailState["ma_state"] = (
                        Lo_AMSDeviceStateInfo.field.ma_state
                    )
                    self.G_AMS1DeviceState["entry_state"] = Lo_AMSDetailState["entry_state"] = (
                        Lo_AMSDeviceStateInfo.field.entry_state
                    )
                    self.kaos_log(
                        "DEBUG",
                        "entry device state(bit)==%d" % Lo_AMSDeviceStateInfo.field.entry_state,
                        "SERIAL",
                    )
                    self.G_AMS1DeviceState["park_state"] = Lo_AMSDetailState["park_state"] = (
                        Lo_AMSDeviceStateInfo.field.park_state
                    )
                    self.kaos_log(
                        "DEBUG",
                        "park position device state(bit)==%d"
                        % Lo_AMSDeviceStateInfo.field.park_state,
                        "SERIAL",
                    )
            except:
                self.kaos_log("DEBUG", "except;Lo_AMSDeviceStateRspInfo", "SERIAL")

        if self.G_SerialPort2OpenFlag == True:
            try:
                self.kaos_log("DEBUG", "try;Lo_AMSDeviceStateRspInfo", "SERIAL")
                Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort2SendWaitRsp(
                    "SD", sizeof(AMSDetailInfoBytes)
                )
                self.kaos_log(
                    "DEBUG", "ttyserial port 2 receive : %s" % Lo_AMSDeviceStateRspInfo, "SERIAL"
                )
                if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                    self.kaos_log("DEBUG", "AMS2, please checkAMS", "SERIAL")
                    # Vendor note (240412): AMSlabel
                    self.G_AMSDevice2IfNormal = False
                else:
                    self.kaos_log("DEBUG", "AMS2 connectedsuccessful", "SERIAL")
                    self.kaos_log("DEBUG", "self.G_AMSDevice2IfNormal=True", "SERIAL")
                    # Vendor note (240412): AMSlabel
                    # [Translated vendor note] Vendor note (241030): P1 C1P1 C32, 132
                    # [Translated vendor note] 1: 1 2 3 4
                    # [Translated vendor note] 2: 5 6 7 8
                    # [Translated vendor note] 3: 9 10 11 12
                    # [Translated vendor note] 4: 13 14 15 16
                    # [Translated vendor note] 5: 17 18 19 20
                    # [Translated vendor note] 6: 21 22 23 24
                    # [Translated vendor note] 7: 25 26 27 28
                    # [Translated vendor note] 8: 29 30 31 32
                    # [Translated vendor note] filament change

                    self.G_AMSDevice2IfNormal = True

                    Lo_AMSDeviceStateInfo = AMSDetailInfoBytes()
                    Lo_AMSDeviceStateInfo.whole[:] = bytearray(Lo_AMSDeviceStateRspInfo)
                    # empty Python dict
                    Lo_AMSDetailState = {}
                    self.G_AMS2DeviceState["dev_id"] = Lo_AMSDetailState["dev_id"] = (
                        Lo_AMSDeviceStateInfo.field.dev_id
                    )
                    self.G_AMS2DeviceState["active_dev_id"] = Lo_AMSDetailState["active_dev_id"] = (
                        Lo_AMSDeviceStateInfo.field.active_dev_id
                    )
                    self.G_AMS2DeviceState["dev_mode"] = Lo_AMSDetailState["dev_mode"] = (
                        Lo_AMSDeviceStateInfo.field.dev_mode
                    )
                    self.G_AMS2DeviceState["cache_empty"] = Lo_AMSDetailState["cache_empty"] = (
                        Lo_AMSDeviceStateInfo.field.cache_empty
                    )
                    # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Buffer sensor empty state (bool) == %d" % Lo_AMSDeviceStateInfo.field.cache_empty)
                    self.G_AMS2DeviceState["cache_full"] = Lo_AMSDetailState["cache_full"] = (
                        Lo_AMSDeviceStateInfo.field.cache_full
                    )
                    self.kaos_log(
                        "DEBUG",
                        "buffer device fullstate(bool)==%d"
                        % Lo_AMSDeviceStateInfo.field.cache_full,
                        "SERIAL",
                    )
                    self.G_AMS2DeviceState["cache_exist"] = Lo_AMSDetailState["cache_exist"] = (
                        Lo_AMSDeviceStateInfo.field.cache_exist
                    )
                    # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]Buffer sensor filament state (bool) == %d" % Lo_AMSDeviceStateInfo.field.cache_exist)
                    self.G_AMS2DeviceState["mc_state"] = Lo_AMSDetailState["mc_state"] = (
                        Lo_AMSDeviceStateInfo.field.mc_state
                    )
                    self.G_AMS2DeviceState["ma_state"] = Lo_AMSDetailState["ma_state"] = (
                        Lo_AMSDeviceStateInfo.field.ma_state
                    )
                    self.G_AMS2DeviceState["entry_state"] = Lo_AMSDetailState["entry_state"] = (
                        Lo_AMSDeviceStateInfo.field.entry_state
                    )
                    self.kaos_log(
                        "DEBUG",
                        "entry device state(bit)==%d" % Lo_AMSDeviceStateInfo.field.entry_state,
                        "SERIAL",
                    )
                    self.G_AMS2DeviceState["park_state"] = Lo_AMSDetailState["park_state"] = (
                        Lo_AMSDeviceStateInfo.field.park_state
                    )
                    self.kaos_log(
                        "DEBUG",
                        "park position device state(bit)==%d"
                        % Lo_AMSDeviceStateInfo.field.park_state,
                        "SERIAL",
                    )
            except:
                self.kaos_log("DEBUG", "except;Lo_AMSDeviceStateRspInfo", "SERIAL")

        # if self.G_AMSDevice1IfNormal==True:
        #     # Vendor note (241106): AMS
        #     if self.G_AMS1DeviceState["entry_state"] > 0 or self.G_AMS1DeviceState["park_state"] > 0:
        #         self.G_PhrozenFluiddRespondInfo("#1 AMS has filament")
        #         # if self.G_AMS1DeviceState["entry_state"]==0 or self.G_AMS1DeviceState["park_state"]==0:
        #         #     self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #         #     self.Cmds_AMSSerial1Send("T1")
        #         #     self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #         #     self.G_ChangeChannelTimeoutNewChan=1
        #         #     # Vendor note (240524): for UIUX dynamic interface
        #         #     self.emit_protocol("+T:0,1")
        #         if self.G_AMS1DeviceState["entry_state"]==1:#0001
        #         #if self.G_AMS1DeviceState["park_state"]==1:#0001
        #             self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #             self.Cmds_AMSSerial1Send("T1")
        #             self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #             self.G_ChangeChannelTimeoutNewChan=1
        #             self.emit_protocol("+T:0,1")
        #         elif self.G_AMS1DeviceState["entry_state"]==2:#0010
        #             if self.G_AMS1DeviceState["park_state"]==1:#0001
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.emit_protocol("+T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==2:#0010
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.emit_protocol("+T:0,2")
        #             else:
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.emit_protocol("+T:0,2")
        #         elif self.G_AMS1DeviceState["entry_state"]==3:#0011
        #             self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #             self.Cmds_AMSSerial1Send("T1")
        #             self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #             self.G_ChangeChannelTimeoutNewChan=1
        #             # Vendor note (240524): for UIUX dynamic interface
        #             self.emit_protocol("+T:0,1")
        #         elif self.G_AMS1DeviceState["entry_state"]==4:#0100
        #             if self.G_AMS1DeviceState["park_state"]==1:#0001
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.emit_protocol("+T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==2:#0010
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.emit_protocol("+T:0,2")
        #             elif self.G_AMS1DeviceState["park_state"]==3:#0011
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.emit_protocol("+T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==4:#0100
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T3")
        #                 self.Cmds_AMSSerial1Send("T3")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=3
        #                 self.emit_protocol("+T:0,3")
        #             elif self.G_AMS1DeviceState["park_state"]==5:#0101
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.emit_protocol("+T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==6:#0110
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.emit_protocol("+T:0,2")
        #             elif self.G_AMS1DeviceState["park_state"]==7:#0111
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.emit_protocol("+T:0,1")
        #             else:
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T3")
        #                 self.Cmds_AMSSerial1Send("T3")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=3
        #                 self.emit_protocol("+T:0,3")
        #         elif self.G_AMS1DeviceState["entry_state"]==5:#0101
        #             self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #             self.Cmds_AMSSerial1Send("T1")
        #             self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #             self.G_ChangeChannelTimeoutNewChan=1
        #             # Vendor note (240524): for UIUX dynamic interface
        #             self.emit_protocol("+T:0,1")
        #         elif self.G_AMS1DeviceState["entry_state"]==6:#0110
        #             if self.G_AMS1DeviceState["park_state"]==1:#0001
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.emit_protocol("+T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==2:#0010
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.emit_protocol("+T:0,2")
        #             else:
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 # Vendor note (240524): for UIUX dynamic interface
        #                 self.emit_protocol("+T:0,2")
        #         elif self.G_AMS1DeviceState["entry_state"]==7:#0111
        #             self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #             self.Cmds_AMSSerial1Send("T1")
        #             self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #             self.G_ChangeChannelTimeoutNewChan=1
        #             # Vendor note (240524): for UIUX dynamic interface
        #             self.emit_protocol("+T:0,1")
        #         elif self.G_AMS1DeviceState["entry_state"]==8:#1000
        #             if self.G_AMS1DeviceState["park_state"]==1:#0001
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.emit_protocol("+T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==2:#0010
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.emit_protocol("+T:0,2")
        #             elif self.G_AMS1DeviceState["park_state"]==3:#0011
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.emit_protocol("+T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==4:#0100
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T3")
        #                 self.Cmds_AMSSerial1Send("T3")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=3
        #                 self.emit_protocol("+T:0,3")
        #             elif self.G_AMS1DeviceState["park_state"]==5:#0101
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.emit_protocol("+T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==6:#0110
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.emit_protocol("+T:0,2")
        #             elif self.G_AMS1DeviceState["park_state"]==7:#0111
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.emit_protocol("+T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==8:#1000
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T4")
        #                 self.Cmds_AMSSerial1Send("T4")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=4
        #                 self.emit_protocol("+T:0,4")
        #             elif self.G_AMS1DeviceState["park_state"]==9:#1001
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.emit_protocol("+T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==10:#1010
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.emit_protocol("+T:0,2")
        #             elif self.G_AMS1DeviceState["park_state"]==11:#1011
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.emit_protocol("+T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==12:#1100
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T3")
        #                 self.Cmds_AMSSerial1Send("T3")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=3
        #                 self.emit_protocol("+T:0,3")
        #             elif self.G_AMS1DeviceState["park_state"]==13:#1101
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.emit_protocol("+T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==14:#1110
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.emit_protocol("+T:0,2")
        #             elif self.G_AMS1DeviceState["park_state"]==15:#1111
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.emit_protocol("+T:0,1")
        #             else:
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T4")
        #                 self.Cmds_AMSSerial1Send("T4")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=4
        #                 # Vendor note (240524): for UIUX dynamic interface
        #                 self.emit_protocol("+T:0,4")
        #         elif self.G_AMS1DeviceState["entry_state"]==9:#1001
        #             self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #             self.Cmds_AMSSerial1Send("T1")
        #             self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #             self.G_ChangeChannelTimeoutNewChan=1
        #             # Vendor note (240524): for UIUX dynamic interface
        #             self.emit_protocol("+T:0,1")
        #         elif self.G_AMS1DeviceState["entry_state"]==10:#1010
        #             if self.G_AMS1DeviceState["park_state"]==1:#0001
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.emit_protocol("+T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==2:#0010
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.emit_protocol("+T:0,2")
        #             else:
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 # Vendor note (240524): for UIUX dynamic interface
        #                 self.emit_protocol("+T:0,2")
        #         elif self.G_AMS1DeviceState["entry_state"]==11:#1011
        #             self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #             self.Cmds_AMSSerial1Send("T1")
        #             self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #             self.G_ChangeChannelTimeoutNewChan=1
        #             # Vendor note (240524): for UIUX dynamic interface
        #             self.emit_protocol("+T:0,1")
        #         elif self.G_AMS1DeviceState["entry_state"]==12:#1100
        #             if self.G_AMS1DeviceState["park_state"]==1:#0001
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.emit_protocol("+T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==2:#0010
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.emit_protocol("+T:0,2")
        #             elif self.G_AMS1DeviceState["park_state"]==3:#0011
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.emit_protocol("+T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==4:#0100
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T3")
        #                 self.Cmds_AMSSerial1Send("T3")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=3
        #                 self.emit_protocol("+T:0,3")
        #             elif self.G_AMS1DeviceState["park_state"]==5:#0101
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.emit_protocol("+T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==6:#0110
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.emit_protocol("+T:0,2")
        #             elif self.G_AMS1DeviceState["park_state"]==7:#0111
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.emit_protocol("+T:0,1")
        #             else:
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T3")
        #                 self.Cmds_AMSSerial1Send("T3")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=3
        #                 self.emit_protocol("+T:0,3")
        #         elif self.G_AMS1DeviceState["entry_state"]==13:#1101
        #             self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #             self.Cmds_AMSSerial1Send("T1")
        #             self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #             self.G_ChangeChannelTimeoutNewChan=1
        #             # Vendor note (240524): for UIUX dynamic interface
        #             self.emit_protocol("+T:0,1")
        #         elif self.G_AMS1DeviceState["entry_state"]==14:#1110
        #             if self.G_AMS1DeviceState["park_state"]==1:#0001
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #                 self.Cmds_AMSSerial1Send("T1")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=1
        #                 self.emit_protocol("+T:0,1")
        #             elif self.G_AMS1DeviceState["park_state"]==2:#0010
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 self.emit_protocol("+T:0,2")
        #             else:
        #                 self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T2")
        #                 self.Cmds_AMSSerial1Send("T2")
        #                 self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #                 self.G_ChangeChannelTimeoutNewChan=2
        #                 # Vendor note (240524): for UIUX dynamic interface
        #                 self.emit_protocol("+T:0,2")
        #         elif self.G_AMS1DeviceState["entry_state"]==15:#1111
        #             self.G_PhrozenFluiddRespondInfo("serial port 1Sending command: T1")
        #             self.Cmds_AMSSerial1Send("T1")
        #             self.G_ChangeChannelTimeoutOldChan=self.G_ChangeChannelTimeoutNewChan
        #             self.G_ChangeChannelTimeoutNewChan=1
        #             # Vendor note (240524): for UIUX dynamic interface
        #             self.emit_protocol("+T:0,1")
        if self.G_AMSDevice1IfNormal == True:
            bitmask1 = 0b0001
            bitmask2 = 0b0010
            bitmask4 = 0b0100
            bitmask8 = 0b1000
            # Vendor note (241106): AMS
            if (
                self.G_AMS1DeviceState["entry_state"] > 0
                or self.G_AMS1DeviceState["park_state"] > 0
            ):
                self.kaos_log("DEBUG", "#1 AMS has filament", "SERIAL")
                if self.G_AMS1DeviceState["entry_state"] == 0:
                    if self.G_AMS1DeviceState["park_state"] & bitmask1 == 1:  # 0001
                        self.kaos_log("DEBUG", "serial port 1Sending command: T1", "SERIAL")
                        self.Cmds_AMSSerial1Send("T1")
                        self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan = 1
                        self.emit_protocol("+T:0,1")
                    elif self.G_AMS1DeviceState["park_state"] & bitmask2 == 2:  # 0010
                        self.kaos_log("DEBUG", "serial port 1Sending command: T2", "SERIAL")
                        self.Cmds_AMSSerial1Send("T2")
                        self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan = 2
                        self.emit_protocol("+T:0,2")
                    elif self.G_AMS1DeviceState["park_state"] & bitmask4 == 4:  # 0100
                        self.kaos_log("DEBUG", "serial port 1Sending command: T3", "SERIAL")
                        self.Cmds_AMSSerial1Send("T3")
                        self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan = 3
                        self.emit_protocol("+T:0,3")
                    elif self.G_AMS1DeviceState["park_state"] & bitmask8 == 8:  # 1000
                        self.kaos_log("DEBUG", "serial port 1Sending command: T4", "SERIAL")
                        self.Cmds_AMSSerial1Send("T4")
                        self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan = 4
                        self.emit_protocol("+T:0,4")
                    else:
                        self.kaos_log("DEBUG", "no filament", "SERIAL")
                elif self.G_AMS1DeviceState["entry_state"] & bitmask1 == 1:  # 0001
                    self.kaos_log("DEBUG", "serial port 1Sending command: T1", "SERIAL")
                    self.Cmds_AMSSerial1Send("T1")
                    self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                    self.G_ChangeChannelTimeoutNewChan = 1
                    self.emit_protocol("+T:0,1")
                elif self.G_AMS1DeviceState["entry_state"] & bitmask2 == 2:  # 0010
                    if self.G_AMS1DeviceState["park_state"] & bitmask1 == 1:  # 0001
                        self.kaos_log("DEBUG", "serial port 1Sending command: T1", "SERIAL")
                        self.Cmds_AMSSerial1Send("T1")
                        self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan = 1
                        self.emit_protocol("+T:0,1")
                    elif self.G_AMS1DeviceState["park_state"] & bitmask2 == 2:  # 0010
                        self.kaos_log("DEBUG", "serial port 1Sending command: T2", "SERIAL")
                        self.Cmds_AMSSerial1Send("T2")
                        self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan = 2
                        self.emit_protocol("+T:0,2")
                    else:
                        self.kaos_log("DEBUG", "serial port 1Sending command: T2", "SERIAL")
                        self.Cmds_AMSSerial1Send("T2")
                        self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan = 2
                        self.emit_protocol("+T:0,2")
                elif self.G_AMS1DeviceState["entry_state"] & bitmask4 == 4:  # 0100
                    if self.G_AMS1DeviceState["park_state"] & bitmask1 == 1:  # 0001
                        self.kaos_log("DEBUG", "serial port 1Sending command: T1", "SERIAL")
                        self.Cmds_AMSSerial1Send("T1")
                        self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan = 1
                        self.emit_protocol("+T:0,1")
                    elif self.G_AMS1DeviceState["park_state"] & bitmask2 == 2:  # 0010
                        self.kaos_log("DEBUG", "serial port 1Sending command: T2", "SERIAL")
                        self.Cmds_AMSSerial1Send("T2")
                        self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan = 2
                        self.emit_protocol("+T:0,2")
                    elif self.G_AMS1DeviceState["park_state"] & bitmask4 == 4:  # 0100
                        self.kaos_log("DEBUG", "serial port 1Sending command: T3", "SERIAL")
                        self.Cmds_AMSSerial1Send("T3")
                        self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan = 3
                        self.emit_protocol("+T:0,3")
                    else:
                        self.kaos_log("DEBUG", "serial port 1Sending command: T3", "SERIAL")
                        self.Cmds_AMSSerial1Send("T3")
                        self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan = 3
                        self.emit_protocol("+T:0,3")
                elif self.G_AMS1DeviceState["entry_state"] & bitmask8 == 8:  # 1000
                    if self.G_AMS1DeviceState["park_state"] & bitmask1 == 1:  # 0001
                        self.kaos_log("DEBUG", "serial port 1Sending command: T1", "SERIAL")
                        self.Cmds_AMSSerial1Send("T1")
                        self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan = 1
                        self.emit_protocol("+T:0,1")
                    elif self.G_AMS1DeviceState["park_state"] & bitmask2 == 2:  # 0010
                        self.kaos_log("DEBUG", "serial port 1Sending command: T2", "SERIAL")
                        self.Cmds_AMSSerial1Send("T2")
                        self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan = 2
                        self.emit_protocol("+T:0,2")
                    elif self.G_AMS1DeviceState["park_state"] & bitmask4 == 4:  # 0100
                        self.kaos_log("DEBUG", "serial port 1Sending command: T3", "SERIAL")
                        self.Cmds_AMSSerial1Send("T3")
                        self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan = 3
                        self.emit_protocol("+T:0,3")
                    elif self.G_AMS1DeviceState["park_state"] & bitmask8 == 8:  # 1000
                        self.kaos_log("DEBUG", "serial port 1Sending command: T4", "SERIAL")
                        self.Cmds_AMSSerial1Send("T4")
                        self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan = 4
                        self.emit_protocol("+T:0,4")
                    else:
                        self.kaos_log("DEBUG", "serial port 1Sending command: T4", "SERIAL")
                        self.Cmds_AMSSerial1Send("T4")
                        self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
                        self.G_ChangeChannelTimeoutNewChan = 4
                        # for UIUX dynamic interface
                        self.emit_protocol("+T:0,4")
            else:
                self.kaos_log("DEBUG", "#1 AMSno filament", "SERIAL")

        self.Cmds_PrintMode(self.G_AMSDeviceWorkMode)

        if self.G_AMSDevice2IfNormal == True:
            if self.G_AMS2DeviceState["entry_state"] > 0:
                self.kaos_log("DEBUG", "#2 AMS has filament", "SERIAL")

        # Vendor note (240108): P8
        self.G_M2MAModeResumeFlag = False

    # P4 device;Stop():"SP";
    # Function: handle P4 command workflow
    # Parameters: gcmd (G-code command context)
    def Cmds_CmdP4(self, gcmd):
        # if not self.G_SerialPort1OpenFlag:
        #     self.G_PhrozenFluiddRespondInfo("AMS multi-material not connected, please send P28 first")
        #     return

        self.kaos_log("DEBUG", "[(cmds.py)Cmds_CmdP4]command: stop", "SERIAL")

        mcu_cmd = G_DictPhrozenCmdP4["mcu_cmd"][0]
        self.kaos_log("DEBUG", "mcu command", "SERIAL")

        if self.G_SerialPort1OpenFlag == True:
            # Vendor note (231207): stm32
            self.Cmds_AMSSerial1Send(mcu_cmd)
            self.kaos_log("DEBUG", "serial port 1 sending command", "SERIAL")
        if self.G_SerialPort2OpenFlag == True:
            self.Cmds_AMSSerial2Send(mcu_cmd)
            self.kaos_log("DEBUG", "serial port 2 sending command", "SERIAL")

        # Vendor note (240125): # Vendor note (231207): klipper pause+stm32
        # klipper active pause

        if self.G_KlipperInPausing == False:
            self.kaos_log("DEBUG", "No pause is in progress; a new pause is allowed", "SERIAL")
            # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("Quick pause enabled")
            self.G_KlipperQuickPause = True
            # klipper active pause
            self.Cmds_PhrozenKlipperPause(None)
        else:
            self.kaos_log(
                "DEBUG", "A pause is already in progress; a new pause is not allowed", "SERIAL"
            )
        # logging.info("SendCmd: %s" % mcu_cmd)
        # logging.info("stop dev running at once")

    # P2 A1 retract to parkposition Yes;====="AP";
    # P2 A2;exitfilament Yes;"CL";
    # P2 A3 filament
    # P2 A4 filamentfilament
    # P2 A7 filamentfilament,detect,completeAMSfilament
    def Cmds_CmdP2(self, gcmd):
        # if not self.G_SerialPort1OpenFlag:
        #     self.G_PhrozenFluiddRespondInfo("AMS multi-material not connected, please send P28 first")
        #     return

        self.kaos_log(
            "DEBUG", "[(cmds.py)Cmds_CmdP2]command='%s'" % (gcmd.get_commandline(),), "SERIAL"
        )

        # KAOS: skip AMS retract when ams_attached is false
        if not self.G_AmsAttached:
            self.kaos_log("DEBUG", "P2 skipped: ams_attached is false", "SERIAL")
            return

        # get command parameters
        params = gcmd.get_command_parameters()

        self.kaos_log("DEBUG", "Current mode", "SERIAL")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.emit_mode(0, "unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.emit_mode(1, "MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.emit_mode(2, "MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.emit_mode(3, "RUNOUT")
        else:
            self.emit_mode(-1, "error")

        # if not "A" in params:
        #     return

        # time.sleep(0.5)
        self.kaos_log("DEBUG", "when 0.5", "SERIAL")
        self.G_ProzenToolhead.dwell(0.5)

        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()

        if "A" in params:
            action = int(params["A"])
            if not action in [1, 2, 3, 4, 5, 6, 7]:
                raise gcmd.error(
                    "no parameter number command;cmd '%s', that must is A[1/2/3/4/5/6/7]"
                    % (gcmd.get_commandline(),)
                )
            # P2 A1 retract to parkposition Yes;====="AP";
            if action == 1:
                # Vendor note (250515): ,P2A?
                if self.G_P0M1MCNoneAMS == 1:
                    self.kaos_log(
                        "DEBUG",
                        "[(cmds.python)Cmds_CmdP2]standalone multi-material, logical P2A?",
                        "SERIAL",
                    )
                    return
                if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                    self.kaos_log(
                        "DEBUG",
                        "[(cmds.python)Cmds_CmdP2]P0M3single-color mode, logical P2 A1",
                        "SERIAL",
                    )
                    return
                if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                    self.kaos_log(
                        "DEBUG",
                        "[(cmds.python)Cmds_CmdP2]P0M2MAsingle-color refill mode, logical P2 A1",
                        "SERIAL",
                    )
                    return

                self.kaos_log(
                    "DEBUG",
                    "command='%s'; all filamentto park position" % (gcmd.get_commandline(),),
                    "SERIAL",
                )
                # for UIUX dynamic interface
                self.emit_protocol("+P2A1:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                # Vendor note (231201): complete,if,,cannotneedfilament

                if self.G_ToolheadIfHaveFilaFlag == True:
                    # Vendor note (231205): #self.Cmds_MoveToCutFilaAndNotRollback(gcmd)
                    self.kaos_log("DEBUG", "toolhead has filament", "SERIAL")
                    # Vendor note (20231024): ;cannot
                    # Vendor note (240109): filamentallow
                    # if self.G_ToolheadIfHaveFilaFlag==True:
                    # Vendor note (240319): before
                    # self.Cmds_MoveToCutFilaPrepare()
                    self.Cmds_MoveToCutFilaAbsolutePositionNotReset(gcmd)
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("AP")
                        self.kaos_log(
                            "DEBUG",
                            "serial port 1Sending command: AP; all filamentto the park position",
                            "SERIAL",
                        )
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AP")
                        self.kaos_log(
                            "DEBUG",
                            "serial port 2Sending command: AP; all filamentto the park position",
                            "SERIAL",
                        )

                    self.kaos_log(
                        "DEBUG", "External macro command-PRZ_WAITINGAREA-waiting area", "SERIAL"
                    )
                    command_string = """
                        # PRZ_WAITINGAREA
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.kaos_log(
                        "DEBUG",
                        "External macro command-PRZ_WAITINGAREA; command_string='%s'"
                        % command_string,
                        "SERIAL",
                    )

                    # Vendor note (240913): self.G_ProzenToolhead.dwell(6.0)

                    # Vendor note (231201): checknormal,normal
                    # Vendor note (231225): klippercompletehoming,
                    # Vendor note (240224): needchecksuccessful
                    self.Cmds_CutFilaIfNormalCheck()
                else:
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("AP")
                        self.kaos_log(
                            "DEBUG",
                            "serial port 1Sending command: AP; all filamentto the park position",
                            "SERIAL",
                        )
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AP")
                        self.kaos_log(
                            "DEBUG",
                            "serial port 2Sending command: AP; all filamentto the park position",
                            "SERIAL",
                        )

                    self.kaos_log(
                        "DEBUG", "External macro command-PRZ_WAITINGAREA-waiting area", "SERIAL"
                    )
                    command_string = """
                        # PRZ_WAITINGAREA
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.kaos_log(
                        "DEBUG",
                        "External macro command-PRZ_WAITINGAREA; command_string='%s'"
                        % command_string,
                        "SERIAL",
                    )

                # Vendor note (240113): manual commandflag
                self.ManualCmdFlag = False

                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP2]Home all and cut filament")
                # command_string = """
                # PG28
                # """
                # self.G_PhrozenGCode.run_script_from_command(command_string)

                # for UIUX dynamic interface
                self.emit_protocol("+P2A1:1,%d" % self.G_ChangeChannelTimeoutNewChan)

                # time.sleep(0.5)
                self.kaos_log("DEBUG", "when 0.5", "SERIAL")
                self.G_ProzenToolhead.dwell(0.5)

                # Vendor note (250409): AMS
                self.Cmds_CmdP114(None)

                # time.sleep(0.5)
                self.kaos_log("DEBUG", "when 0.5", "SERIAL")
                self.G_ProzenToolhead.dwell(0.5)

                return

            # P2 A2;exitfilament Yes;"CL";
            if action == 2:
                # Vendor note (250515): ,P2A?
                if self.G_P0M1MCNoneAMS == 1:
                    self.kaos_log(
                        "DEBUG",
                        "[(cmds.python)Cmds_CmdP2]standalone multi-material, logical P2A?",
                        "SERIAL",
                    )
                    return
                if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                    self.kaos_log(
                        "DEBUG",
                        "[(cmds.python)Cmds_CmdP2]single-color mode, logical P2 A2",
                        "SERIAL",
                    )
                    return
                self.kaos_log(
                    "DEBUG",
                    "command='%s'; all filament full output" % (gcmd.get_commandline(),),
                    "SERIAL",
                )
                # for UIUX dynamic interface
                self.emit_protocol("+P2A2:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                # Vendor note (240319): before
                # self.Cmds_MoveToCutFilaPrepare()

                if self.G_ToolheadIfHaveFilaFlag:
                    # Vendor note (231205): self.Cmds_MoveToCutFilaAndNotRollback(gcmd)
                    self.kaos_log("DEBUG", "toolhead has filament, all AMSfirst", "SERIAL")
                    # self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutOldChan)
                    # self.G_PhrozenFluiddRespondInfo("G%d" % self.G_ChangeChannelTimeoutOldChan)
                    # self.G_PhrozenFluiddRespondInfo("AMS old channel retract a distance first: G%d" % self.G_ChangeChannelTimeoutOldChan)
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("AP")
                        self.kaos_log("DEBUG", "Serial port 1 sending command: AP", "SERIAL")
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AP")
                        self.kaos_log("DEBUG", "Serial port 2 sending command: AP", "SERIAL")

                    self.G_ProzenToolhead.dwell(0.5)

                    # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]External macro command-PG101")
                    # command_string = """
                    #     PG101
                    #     """
                    # self.G_PhrozenGCode.run_script_from_command(command_string)
                    # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]-feed waiting zoneposition;command_string='%s'" % command_string)
                    # self.IfDoPG102Flag=True

                    # Vendor note (240913): self.G_ProzenToolhead.dwell(6.0)
                    # Vendor note (231201): checkfilamentnormal,normal
                    self.Cmds_CutFilaIfNormalCheck()
                    if self.G_KlipperIfPaused == True:
                        self.kaos_log(
                            "DEBUG",
                            "cut filament 5toolhead detectedfilament, cutter error, please check cutter, pauseklipperprinting",
                            "SERIAL",
                        )
                        # Lo_ChangeChannelIfSuccess = False
                        return

                # Vendor note (250619): check if AMS reconnected successfully
                self.Cmds_USBConnectErrorCheck()
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("CL")
                    self.kaos_log("DEBUG", "serial port 1Sending command: CL", "SERIAL")
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("CL")
                    self.kaos_log("DEBUG", "serial port 2Sending command: CL", "SERIAL")

                # # Vendor note (240913): # self.G_ProzenToolhead.dwell(6.0)
                # # Vendor note (231201): checknormal,normal
                # self.Cmds_CutFilaIfNormalCheck()

                # for UIUX dynamic interface
                self.emit_protocol("+P2A2:1,%d" % self.G_ChangeChannelTimeoutNewChan)

                # time.sleep(0.5)
                self.kaos_log("DEBUG", "when 0.5", "SERIAL")
                self.G_ProzenToolhead.dwell(0.5)

                return

            # P2 A3 filament
            if action == 3:
                # # Vendor note (250515): ,P2A?
                # if self.G_P0M1MCNoneAMS == 1:
                #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP2]standalone multi-material, logical P2A?")
                #     return
                self.kaos_log(
                    "DEBUG", "command='%s'; cut filament" % (gcmd.get_commandline(),), "SERIAL"
                )
                # for UIUX dynamic interface
                self.emit_protocol("+P2A3:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                # Vendor note (240319): before
                # self.Cmds_MoveToCutFilaPrepare()

                self.Cmds_MoveToCutFilaAbsolutePositionNotReset(gcmd)
                # for UIUX dynamic interface
                self.emit_protocol("+P2A3:1,%d" % self.G_ChangeChannelTimeoutNewChan)

                # Vendor note (250104): P2A3flag
                self.G_P2A3Flag = 1
                # Vendor note (240516): prevent
                # time.sleep(0.5)
                self.kaos_log("DEBUG", "when 0.5", "SERIAL")
                self.G_ProzenToolhead.dwell(0.5)

            # P2 A4 filamentfilament
            if action == 4:
                # Vendor note (250515): ,P2A?
                if self.G_P0M1MCNoneAMS == 1:
                    self.kaos_log(
                        "DEBUG",
                        "[(cmds.python)Cmds_CmdP2]standalone multi-material, logical P2A?",
                        "SERIAL",
                    )
                    return
                self.kaos_log(
                    "DEBUG",
                    "command='%s'; cut filament and filamentto park position"
                    % (gcmd.get_commandline(),),
                    "SERIAL",
                )
                # for UIUX dynamic interface
                self.emit_protocol("+P2A4:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                # Vendor note (240319): before
                # self.Cmds_MoveToCutFilaPrepare()

                self.Cmds_MoveToCutFilaAbsolutePositionNotReset(gcmd)
                # for UIUX dynamic interface
                self.emit_protocol("+P2A4:1,%d" % self.G_ChangeChannelTimeoutNewChan)

            # P2 A5 completefilamentfilament,cannot
            if action == 5:
                # Vendor note (250515): ,P2A?
                if self.G_P0M1MCNoneAMS == 1:
                    self.kaos_log(
                        "DEBUG",
                        "[(cmds.python)Cmds_CmdP2]standalone multi-material, logical P2A?",
                        "SERIAL",
                    )
                    return
                self.kaos_log(
                    "DEBUG",
                    "command='%s'completefilament runout and filament, cannotto type"
                    % (gcmd.get_commandline(),),
                    "SERIAL",
                )
                # for UIUX dynamic interface
                self.emit_protocol("+P2A5:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                # Vendor note (240319): before
                # self.Cmds_MoveToCutFilaPrepare()

                self.Cmds_MoveToCutFilaAbsolutePositionNotReset(gcmd)
                # for UIUX dynamic interface
                self.emit_protocol("+P2A5:0,%d" % self.G_ChangeChannelTimeoutNewChan)

            # P2 A6
            if action == 6:
                # Vendor note (250515): ,P2A?
                if self.G_P0M1MCNoneAMS == 1:
                    self.kaos_log(
                        "DEBUG",
                        "[(cmds.python)Cmds_CmdP2]standalone multi-material, logical P2A?",
                        "SERIAL",
                    )
                    return
                self.kaos_log(
                    "DEBUG",
                    "command='%s'; home/reset and cut filament" % (gcmd.get_commandline(),),
                    "SERIAL",
                )
                # for UIUX dynamic interface
                self.emit_protocol("+P2A6:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                # Vendor note (231201): complete,if,,cannotneedfilament

                if self.G_ToolheadIfHaveFilaFlag == True:
                    # Vendor note (231205): #self.Cmds_MoveToCutFilaAndNotRollback(gcmd)
                    self.kaos_log(
                        "DEBUG", "toolhead has filament, resetXYcut filamentand", "SERIAL"
                    )
                    # self.Cmds_MoveToCutFilaAndNotRollback(gcmd)
                    self.Cmds_MoveToCutFilaAndHomingXY(gcmd)
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("AP")
                        self.kaos_log(
                            "DEBUG",
                            "serial port 1Sending command: AP; all filamentto the park position",
                            "SERIAL",
                        )
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AP")
                        self.kaos_log(
                            "DEBUG",
                            "serial port 2Sending command: AP; all filamentto the park position",
                            "SERIAL",
                        )

                    # self.G_PhrozenFluiddRespondInfo("External macro command-PRZ_WAITINGAREA-waiting area")
                    # command_string = """
                    #     PRZ_WAITINGAREA
                    #     """
                    # self.G_PhrozenGCode.run_script_from_command(command_string)
                    # self.G_PhrozenFluiddRespondInfo("External macro command-PRZ_WAITINGAREA; command_string='%s'" % command_string)

                    # Vendor note (240913): self.G_ProzenToolhead.dwell(6.0)

                    # Vendor note (231201): checknormal,normal
                    # Vendor note (231225): klippercompletehoming,
                    # Vendor note (240224): needchecksuccessful
                    self.Cmds_CutFilaIfNormalCheck()
                else:
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("AP")
                        self.kaos_log(
                            "DEBUG",
                            "serial port 1Sending command: AP; all filamentto the park position",
                            "SERIAL",
                        )
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AP")
                        self.kaos_log(
                            "DEBUG",
                            "serial port 2Sending command: AP; all filamentto the park position",
                            "SERIAL",
                        )

                    # self.G_PhrozenFluiddRespondInfo("External macro command-PRZ_WAITINGAREA-waiting area")
                    # command_string = """
                    #     PRZ_WAITINGAREA
                    #     """
                    # self.G_PhrozenGCode.run_script_from_command(command_string)
                    # self.G_PhrozenFluiddRespondInfo("External macro command-PRZ_WAITINGAREA; command_string='%s'" % command_string)

                self.ManualCmdFlag = True

                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP2]Home all and cut filament")
                # command_string = """
                # PG28
                # """
                # self.G_PhrozenGCode.run_script_from_command(command_string)

                # for UIUX dynamic interface
                self.emit_protocol("+P2A6:1,%d" % self.G_ChangeChannelTimeoutNewChan)

                # time.sleep(0.5)
                self.kaos_log("DEBUG", "when 0.5", "SERIAL")
                self.G_ProzenToolhead.dwell(0.5)

                # Vendor note (250409): AMS
                # self.Cmds_CmdP114(None)

                # time.sleep(0.5)
                self.kaos_log("DEBUG", "when 0.5", "SERIAL")
                self.G_ProzenToolhead.dwell(0.5)

                return

            # P2 A7 filamentfilament,detect,completeAMSfilament
            if action == 7:
                # Vendor note (251014): complete;clearflag;
                self.G_P0M1MCNoneAMS = 0
                # # Vendor note (250515): ,P2A?
                # if self.G_P0M1MCNoneAMS == 1:
                #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP2]standalone multi-material, logical P2A?")
                #     return
                # for UIUX dynamic interface
                self.emit_protocol("+P2A7:0,%d" % self.G_ChangeChannelTimeoutNewChan)

                if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                    self.kaos_log("DEBUG", "[(cmds.python)Cmds_CmdP2]P0 M0Unknown mode", "SERIAL")
                    # return

                # Vendor note (250618): detectexit
                self.G_P0M3Flag = False
                self.G_P0M2MAStartPrintFlag = 0

                # Vendor note (250619): check if AMS reconnected successfully
                self.Cmds_USBConnectErrorCheck()

                if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                    self.kaos_log(
                        "DEBUG", "[(cmds.python)Cmds_CmdP2]P0 M3;single-color mode", "SERIAL"
                    )

                if self.G_SerialPort1OpenFlag == True:
                    try:
                        self.kaos_log("DEBUG", "try;Lo_AMSDeviceStateRspInfo", "SERIAL")
                        # get detailed AMS board state
                        Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort1SendWaitRsp(
                            "SD", sizeof(AMSDetailInfoBytes)
                        )
                        self.kaos_log(
                            "DEBUG",
                            "tty1 serial port receive : %s" % Lo_AMSDeviceStateRspInfo,
                            "SERIAL",
                        )
                        if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                            self.kaos_log(
                                "DEBUG",
                                "AMS1, please checkAMS1='%s'" % (gcmd.get_commandline(),),
                                "SERIAL",
                            )
                            # Vendor note (240412): AMSlabel
                            self.G_AMSDevice1IfNormal = False
                        else:
                            # Vendor note (240412): AMSlabel
                            self.G_AMSDevice1IfNormal = True
                    except:
                        self.kaos_log("DEBUG", "except;Lo_AMSDeviceStateRspInfo", "SERIAL")

                if self.G_SerialPort2OpenFlag == True:
                    try:
                        self.kaos_log("DEBUG", "try;Lo_AMSDeviceStateRspInfo", "SERIAL")
                        # get detailed AMS board state
                        Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort2SendWaitRsp(
                            "SD", sizeof(AMSDetailInfoBytes)
                        )
                        self.kaos_log(
                            "DEBUG",
                            "tty2 serial port receive : %s" % Lo_AMSDeviceStateRspInfo,
                            "SERIAL",
                        )
                        if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                            self.kaos_log(
                                "DEBUG",
                                "AMS2, please checkAMS2='%s'" % (gcmd.get_commandline(),),
                                "SERIAL",
                            )
                            self.G_AMSDevice2IfNormal = False
                        else:
                            self.G_AMSDevice2IfNormal = True
                    except:
                        self.kaos_log("DEBUG", "except;Lo_AMSDeviceStateRspInfo", "SERIAL")

                if self.G_AMSDevice1IfNormal == True or self.G_AMSDevice2IfNormal == True:
                    self.kaos_log("DEBUG", "has AMS multi-material, logical P2 A7", "SERIAL")
                else:
                    self.kaos_log("DEBUG", "has AMS multi-material, logical P2 A7", "SERIAL")
                    # Vendor note (250619): check if AMS reconnected successfully
                    self.Cmds_USBConnectErrorCheck()
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("M0")
                        self.kaos_log("DEBUG", "serial port 1Sending command: M0", "SERIAL")
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("M0")
                        self.kaos_log("DEBUG", "serial port 2Sending command: M0", "SERIAL")

                    # for UIUX dynamic interface
                    self.G_PhrozenFluiddRespondInfo(
                        "+P2A7:1,%d" % self.G_ChangeChannelTimeoutNewChan
                    )
                    self.kaos_log("DEBUG", "return", "SERIAL")
                    return

                self.kaos_log(
                    "DEBUG",
                    "command='%s'; all filamentto park position" % (gcmd.get_commandline(),),
                    "SERIAL",
                )
                # for UIUX dynamic interface
                self.emit_protocol("+P2A7:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                # Vendor note (231201): complete,if,,cannotneedfilament

                if self.G_ToolheadIfHaveFilaFlag == True:
                    # Vendor note (231205): #self.Cmds_MoveToCutFilaAndNotRollback(gcmd)
                    self.kaos_log("DEBUG", "toolhead has filament", "SERIAL")
                    # Vendor note (20231024): ;cannot
                    # Vendor note (240109): filamentallow
                    # if self.G_ToolheadIfHaveFilaFlag==True:
                    # Vendor note (240319): before
                    # self.Cmds_MoveToCutFilaPrepare()
                    self.Cmds_MoveToCutFilaAbsolutePositionNotReset(gcmd)
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("RD")
                        self.kaos_log(
                            "DEBUG",
                            "serial port 1Sending command: RD; all filamentto the park position",
                            "SERIAL",
                        )
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("RD")
                        self.kaos_log(
                            "DEBUG",
                            "serial port 2Sending command: RD; all filamentto the park position",
                            "SERIAL",
                        )

                    self.kaos_log(
                        "DEBUG", "External macro command-PRZ_WAITINGAREA-waiting area", "SERIAL"
                    )
                    command_string = """
                        # PRZ_WAITINGAREA
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.kaos_log(
                        "DEBUG",
                        "External macro command-PRZ_WAITINGAREA; command_string='%s'"
                        % command_string,
                        "SERIAL",
                    )

                    self.kaos_log("DEBUG", "when 16", "SERIAL")
                    # # Vendor note (240913): self.G_ProzenToolhead.dwell(16)
                    self.kaos_log("DEBUG", "when 16", "SERIAL")

                    # Vendor note (250619): check if AMS reconnected successfully
                    self.Cmds_USBConnectErrorCheck()
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("M0")
                        self.kaos_log("DEBUG", "serial port 1Sending command: M0", "SERIAL")
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("M0")
                        self.kaos_log("DEBUG", "serial port 2Sending command: M0", "SERIAL")
                else:
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("RD")
                        self.kaos_log(
                            "DEBUG",
                            "serial port 1Sending command: RD; all filamentto the park position",
                            "SERIAL",
                        )
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("RD")
                        self.kaos_log(
                            "DEBUG",
                            "serial port 2Sending command: RD; all filamentto the park position",
                            "SERIAL",
                        )

                    self.kaos_log(
                        "DEBUG", "External macro command-PRZ_WAITINGAREA-waiting area", "SERIAL"
                    )
                    command_string = """
                        # PRZ_WAITINGAREA
                        """
                    self.G_PhrozenGCode.run_script_from_command(command_string)
                    self.kaos_log(
                        "DEBUG",
                        "External macro command-PRZ_WAITINGAREA; command_string='%s'"
                        % command_string,
                        "SERIAL",
                    )

                    self.kaos_log("DEBUG", "when 12", "SERIAL")
                    # # Vendor note (240913): self.G_ProzenToolhead.dwell(12)
                    self.kaos_log("DEBUG", "when 12", "SERIAL")

                    # Vendor note (250619): check if AMS reconnected successfully
                    self.Cmds_USBConnectErrorCheck()
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("M0")
                        self.kaos_log("DEBUG", "serial port 1Sending command: M0", "SERIAL")
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("M0")
                        self.kaos_log("DEBUG", "serial port 2Sending command: M0", "SERIAL")

                # Vendor note (240113): manual commandflag
                self.ManualCmdFlag = False

                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP2]Home all and cut filament")
                # command_string = """
                # PG28
                # """
                # self.G_PhrozenGCode.run_script_from_command(command_string)

                # time.sleep(0.5)
                self.kaos_log("DEBUG", "when 0.5", "SERIAL")
                self.G_ProzenToolhead.dwell(0.5)

                # Vendor note (250409): AMS
                self.Cmds_CmdP114(None)

                # time.sleep(0.5)
                # time.sleep(0.5)
                self.kaos_log("DEBUG", "when 0.5", "SERIAL")
                self.G_ProzenToolhead.dwell(0.5)

                # for UIUX dynamic interface
                self.emit_protocol("+P2A7:1,%d" % self.G_ChangeChannelTimeoutNewChan)

                return

        # Vendor note (240801): P2 B?
        if "B" in params:
            # self.Cmds_P1CnAutoChangeChannel(int(params["C"]), gcmd)
            # for UIUX dynamic interface
            self.kaos_log("DEBUG", gcmd.get_commandline(), "SERIAL")

        # Vendor note (240516): prevent
        # time.sleep(0.5)
        self.kaos_log("DEBUG", "when 0.5", "SERIAL")
        self.G_ProzenToolhead.dwell(0.5)

    # P1 S0 filament;====="RD";
    # P1 T[n]n:1 ~32(device,1 ~4),();====="T";
    # P1 B[n]n:1 ~32(device,1 ~4)exit Yes;====="B";
    # P1 D[n];n:1~32(device,1~4); Yes;====="P";
    # P1 C[n] n:1~32(device,1~4) (,, , );====="T";
    # Vendor note (231202): # P1 E[n];n:1~32(device,1~4);,need Yes;====="E?";
    # Vendor note (240228): distance,needstm32distance
    # P1 G[n];n:1~32(device,1~4);distance Yes;====="G?";
    # Vendor note (240319): # =====P1 H[n];n:1~32(device,1~4);filament change Yes;====="H?";
    # Vendor note (240329): # =====P1 I[n];stm32need;====="I?";
    # =====P1 J[n];;;
    # =====P1 K[n];
    # =====P1 L[n];
    # =====P1 M[n];
    # =====P1 N[n];
    # =====P1 O[n];
    # =====P1 Q[n];
    # =====P1 U[n];
    # Vendor note (240418): # =====P1 V[n];
    # =====P1 W[n];
    # =====P1 X[n];
    # =====P1 Y[n];
    # =====P1 Z[n];
    # Function: handle P1 command workflow
    # Parameters: gcmd (G-code command context)
    def Cmds_CmdP1(self, gcmd):
        self.kaos_log(
            "DEBUG", "[(cmds.python)Cmds_CmdP1]command='%s'" % (gcmd.get_commandline(),), "SERIAL"
        )

        if self.G_AMSDevice1IfNormal == False and self.G_AMSDevice2IfNormal == False:
            self.kaos_log(
                "DEBUG",
                "has AMS multi-material, all AMS multi-material not connected, logical P1 ??command",
                "SERIAL",
            )

            # # Vendor note (250828): # self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro PRZ_PAUSE_WAITINGAREA")
            # command_string = """
            #     PRZ_PAUSE_WAITINGAREA
            #     """
            # self.G_PhrozenGCode.run_script_from_command(command_string)
            # self.G_PhrozenFluiddRespondInfo("[SERVICE] External macro; command_string='%s'" % command_string)

            # get command parameters
            params = gcmd.get_command_parameters()

            # =====P1 I[n];stm32need;====="I?";
            if "I" in params:
                self.kaos_log(
                    "DEBUG",
                    "AMS multi-material P28not connected, move extrude use use single-color M3 mode",
                    "SERIAL",
                )
                # for UIUX dynamic interface
                self.emit_protocol("+P1In:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                if not int(params["I"]) in range(-1000, 1000):
                    raise gcmd.error(
                        "no parameter number command;cmd '%s', move extrude"
                        % (gcmd.get_commandline())
                    )
                # command_string = """
                #                 M106 S0
                #                 M83
                #                 G92 E0
                #                 G1 E%f F300
                #                 """ %(int(params["I"]),)
                # self.G_PhrozenGCode.run_script_from_command(command_string)
                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]G-code command: %s" % command_string)

                # Vendor note (240705): AMS,gcode,speed
                self.Cmds_P1InExtrudeManualIn(int(params["I"]))

                # for UIUX dynamic interface
                self.emit_protocol("+P1In:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return

        # # Vendor note (240705): ifP114AMS,
        # if self.G_AMSDevice1IfNormal==False:
        #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]self.G_AMSDevice1IfNormal==False")
        #     #get command parameters
        #     params = gcmd.get_command_parameters()
        #     # =====P1 I[n];stm32need;====="I?";
        #     if "I" in params:
        #         self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]AMS multi-material P114 not connected, extrude directly")
        #         # Vendor note (240524): for UIUX dynamic interface
        #         self.emit_protocol("+P1In:0,%d" % self.G_ChangeChannelTimeoutNewChan)
        #         if not int(params["I"]) in range(-1000, 1000):
        #             raise gcmd.error("no parameter number command;cmd '%s', move extrude" % (gcmd.get_commandline()))
        #         # command_string = """
        #         #                 M106 S0
        #         #                 M83
        #         #                 G92 E0
        #         #                 G1 E%f F300
        #         #                 """ %(int(params["I"]),)
        #         # self.G_PhrozenGCode.run_script_from_command(command_string)
        #         # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]G-code command: %s" % command_string)

        #         # Vendor note (240705): encapsulated function
        #         self.Cmds_P1InExtrudeManualIn(int(params["I"]))

        #         # Vendor note (240524): for UIUX dynamic interface
        #         self.emit_protocol("+P1In:1,%d" % self.G_ChangeChannelTimeoutNewChan)

        #         return

        self.kaos_log("DEBUG", "Current mode", "SERIAL")
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            self.emit_mode(0, "unkown")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
            self.emit_mode(1, "MC")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
            self.emit_mode(2, "MA")
        elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
            self.emit_mode(3, "RUNOUT")
        else:
            self.emit_mode(-1, "error")

        # Vendor note (240529): phrozen
        self.G_PhrozenFluiddRespondInfo(
            "V-H%s-I%s-F%s-SN1" % (HW_VERSION, IMAGE_VERSION, FW_VERSION)
        )

        # Vendor note (231228): preventstm32AT
        # time.sleep(0.5)
        # Vendor note (240516): preventtime too close
        # self.G_ProzenToolhead.dwell(0.5)

        # Vendor note (240105): AT
        self.emit_protocol("+P1START:0,%d" % self.G_ChangeChannelTimeoutNewChan)

        # Vendor note (20231019): iffilament detected on toolhead,filamentexit
        # if self.G_ToolheadIfHaveFilaFlag:
        #    gcmd.respond_info("[(cmds.python)Cmds_CmdP1]detectfilament,filament,exit")
        #    gcmd.respond_info("[(cmds.python)Cmds_CmdP1]")
        #    self.Cmds_MoveToCutFilaAction(gcmd)
        #    self.Cmds_AMSSerial1Send("AP")
        #    gcmd.respond_info("Sending command: AP, retract all to park position")

        # get command parameters
        params = gcmd.get_command_parameters()

        # Vendor note (250619): check if AMS reconnected successfully
        self.Cmds_USBConnectErrorCheck()

        # P1 S0;filament
        # P1 S1;distance
        if "S" in params:
            self.kaos_log("DEBUG", "Cmds_CmdP1]P1 S?", "SERIAL")

            # # Vendor note (250515): ,T?
            # if self.G_P0M1MCNoneAMS == 1:
            #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdT0]standalone multi-material, logical T?")
            #     return
            # # Vendor note (250429): # if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_FILA_RUNOUT:
            #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdT0]Single-color mode, skip T0")
            #     return
            # # Vendor note (250514): # if self.G_AMSDeviceWorkMode==AMS_WORK_MODE_MA :
            #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdT0]Single-color refill mode, skip T0")
            #     return

            # Vendor note (240527): ,becausemanual command,defaultSTM32MC
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                self.kaos_log(
                    "DEBUG",
                    "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                    "SERIAL",
                )
                if self.G_SerialPort1OpenFlag == True:
                    self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
                    self.Cmds_AMSSerial1Send("MC")

                if self.G_SerialPort2OpenFlag == True:
                    self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")
                    self.Cmds_AMSSerial2Send("MC")

                self.G_ProzenToolhead.dwell(2)

            if self.G_ToolheadIfHaveFilaFlag:
                # Vendor note (231205): self.Cmds_MoveToCutFilaAndNotRollback(gcmd)
                self.kaos_log("DEBUG", "toolhead has filament, all AMSfirst", "SERIAL")
                # self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutOldChan)
                # self.G_PhrozenFluiddRespondInfo("G%d" % self.G_ChangeChannelTimeoutOldChan)
                # self.G_PhrozenFluiddRespondInfo("AMS old channel retract a distance first: G%d" % self.G_ChangeChannelTimeoutOldChan)
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("AP")
                    self.kaos_log("DEBUG", "Serial port 1 sending command: AP", "SERIAL")
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("AP")
                    self.kaos_log("DEBUG", "Serial port 2 sending command: AP", "SERIAL")

                self.G_ProzenToolhead.dwell(0.5)

                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]External macro command-PG101")
                # command_string = """
                #     PG101
                #     """
                # self.G_PhrozenGCode.run_script_from_command(command_string)
                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]-feed waiting zoneposition;command_string='%s'" % command_string)
                # self.IfDoPG102Flag=True

                # Vendor note (240913): self.G_ProzenToolhead.dwell(6.0)
                # Vendor note (231201): checkfilamentnormal,normal
                self.Cmds_CutFilaIfNormalCheck()
                if self.G_KlipperIfPaused == True:
                    self.kaos_log(
                        "DEBUG",
                        "cut filament 5toolhead detectedfilament, cutter error, please check cutter, pauseklipperprinting",
                        "SERIAL",
                    )
                    # Lo_ChangeChannelIfSuccess = False
                    return

            self.kaos_log("DEBUG", "command='%s';" % (gcmd.get_commandline(),), "SERIAL")
            # for UIUX dynamic interface
            self.emit_protocol("+P1S:0,%d" % self.G_ChangeChannelTimeoutNewChan)

            if self.G_IfInFilaBlockFlag:
                self.kaos_log(
                    "DEBUG",
                    "feed, first move P1 E?from toolhead up filament tube get output and move prz_resumeresume",
                    "SERIAL",
                )
                self.emit_channel_op("P1END", 0, self.G_ChangeChannelTimeoutNewChan)
                # for UIUX dynamic interface
                self.emit_protocol("+P1S:1,%d" % self.G_ChangeChannelTimeoutNewChan)
                return

            if int(params["S"]) == 0:
                #
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("RD")
                    self.kaos_log("DEBUG", "Serial port 1 sending command: RD", "SERIAL")
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("RD")
                    self.kaos_log("DEBUG", "Serial port 2 sending command: RD", "SERIAL")

                self.kaos_log(
                    "DEBUG", "sending command=RD; all filamentfeedto the park position", "SERIAL"
                )
            if int(params["S"]) == 1:
                #
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("RB")
                    self.kaos_log("DEBUG", "Serial port 1 sending command: RB", "SERIAL")
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("RB")
                    self.kaos_log("DEBUG", "Serial port 2 sending command: RB", "SERIAL")
                self.kaos_log("DEBUG", "sending command=RB;", "SERIAL")

            # Vendor note (240113): manual commandflag
            self.ManualCmdFlag = True

            # Vendor note (231201): checknormal,normal
            # Vendor note (240528): detect
            # self.Cmds_CutFilaIfNormalCheck()

            self.emit_channel_op("P1END", 0, self.G_ChangeChannelTimeoutNewChan)
            # for UIUX dynamic interface
            self.emit_protocol("+P1S:1,%d" % self.G_ChangeChannelTimeoutNewChan)
            return

        # Vendor note (20231013): auto filament change
        # P1 C[n] auto filament change
        if "C" in params:
            self.kaos_log("DEBUG", "P1 C?", "SERIAL")
            self.kaos_log(
                "DEBUG",
                "=====self.G_ChangeChannelTimeoutOldChan=%d" % self.G_ChangeChannelTimeoutOldChan,
                "SERIAL",
            )
            self.kaos_log(
                "DEBUG",
                "=====self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan,
                "SERIAL",
            )

            # Vendor note (250515): ,T?
            if self.G_P0M1MCNoneAMS == 1:
                self.kaos_log(
                    "DEBUG",
                    "[(cmds.python)Cmds_CmdT0]standalone multi-material, logical T?",
                    "SERIAL",
                )
                return
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                self.kaos_log(
                    "DEBUG", "[(cmds.python)Cmds_CmdT0]single-color mode, logical T?", "SERIAL"
                )
                return
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                self.kaos_log(
                    "DEBUG",
                    "[(cmds.python)Cmds_CmdT0]single-color refill mode, logical T?",
                    "SERIAL",
                )
                return

            # Vendor note (240527): ,becausemanual command,defaultSTM32MC
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                self.kaos_log(
                    "DEBUG",
                    "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                    "SERIAL",
                )
                self.kaos_log(
                    "DEBUG",
                    "ifreceived T?command, but mode is Unknown mode, switch is MCmulti-material mode",
                    "SERIAL",
                )
                self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MC
                if self.G_SerialPort1OpenFlag == True:
                    self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
                    self.Cmds_AMSSerial1Send("MC")

                if self.G_SerialPort2OpenFlag == True:
                    self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")
                    self.Cmds_AMSSerial2Send("MC")

                self.G_ProzenToolhead.dwell(2)

            # Vendor note (250913): return
            # self.Cmds_CmdOrcaPre()

            self.kaos_log(
                "DEBUG",
                "command='%s'; Automatic filament change" % (gcmd.get_commandline(),),
                "SERIAL",
            )
            self.kaos_log("DEBUG", "Automatic filament change", "SERIAL")
            # for UIUX dynamic interface
            self.emit_protocol("+P1Cn:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            if not int(params["C"]) in range(1, AMS_MAX_CHANNEL + 1):
                raise gcmd.error(
                    "no parameter number command;cmd '%s', that must is C?"
                    % (gcmd.get_commandline())
                )

            # Vendor note (240113): manual commandflag
            self.ManualCmdFlag = False
            # Vendor note (240221): STM32 active report, allow one pause
            self.STM32ReprotPauseFlag = 0
            self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")
            # #cancel
            # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
            Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
            self.kaos_log(
                "DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
            )
            # // current-Lo_PauseStatus='{'is_paused': True}'
            if Lo_PauseStatus["is_paused"] == True:
                self.kaos_log("DEBUG", "Already paused", "SERIAL")
            else:
                self.kaos_log("DEBUG", "Not currently paused", "SERIAL")
            # Vendor note (240113): manual commandflag
            self.ManualCmdFlag = False
            # Vendor note (240221): STM32 active report, allow one pause
            self.STM32ReprotPauseFlag = 0
            self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")

            # Vendor note (241030): P1 C1P1 C32,132
            # 1:1 2 3 4
            # 2:5 6 7 8
            # 3:9 10 11 12
            # 4:13 14 15 16
            # 5:17 18 19 20
            # 6:21 22 23 24
            # 7:25 26 27 28
            # 8:29 30 31 32
            # auto filament change
            # self.Cmds_P1CnAutoChangeChannel(int(params["C"]), gcmd)
            chan = int(params["C"])

            if chan == 1:
                # Vendor note (250515): checkconfigcolor
                if self.G_ChromaKitAccessT0 > 0:
                    self.kaos_log(
                        "DEBUG",
                        "use; self.G_ChromaKitAccessT0=%d" % self.G_ChromaKitAccessT0,
                        "SERIAL",
                    )
                    self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT0, gcmd)
                else:
                    self.kaos_log("DEBUG", "use, default; chan=%d" % chan, "SERIAL")
                    self.Cmds_P1CnAutoChangeChannel(chan, gcmd)
            elif chan == 2:
                # Vendor note (250515): checkconfigcolor
                if self.G_ChromaKitAccessT1 > 0:
                    self.kaos_log(
                        "DEBUG",
                        "use; self.G_ChromaKitAccessT1=%d" % self.G_ChromaKitAccessT1,
                        "SERIAL",
                    )
                    self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT1, gcmd)
                else:
                    self.kaos_log("DEBUG", "use, default; chan=%d" % chan, "SERIAL")
                    self.Cmds_P1CnAutoChangeChannel(chan, gcmd)
            elif chan == 3:
                # Vendor note (250515): checkconfigcolor
                if self.G_ChromaKitAccessT2 > 0:
                    self.kaos_log(
                        "DEBUG",
                        "use; self.G_ChromaKitAccessT2=%d" % self.G_ChromaKitAccessT2,
                        "SERIAL",
                    )
                    self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT2, gcmd)
                else:
                    self.kaos_log("DEBUG", "use, default; chan=%d" % chan, "SERIAL")
                    self.Cmds_P1CnAutoChangeChannel(chan, gcmd)
            elif chan == 4:
                # Vendor note (250515): checkconfigcolor
                if self.G_ChromaKitAccessT3 > 0:
                    self.kaos_log(
                        "DEBUG",
                        "use; self.G_ChromaKitAccessT3=%d" % self.G_ChromaKitAccessT3,
                        "SERIAL",
                    )
                    self.Cmds_P1CnAutoChangeChannel(self.G_ChromaKitAccessT3, gcmd)
                else:
                    self.kaos_log("DEBUG", "use, default; chan=%d" % chan, "SERIAL")
                    self.Cmds_P1CnAutoChangeChannel(chan, gcmd)

            # for UIUX dynamic interface
            self.emit_protocol("+P1Cn:1,%d" % self.G_ChangeChannelTimeoutNewChan)

        # Vendor note (20231013): filament change
        # P1 T[n] filament change
        if "T" in params:
            self.kaos_log("DEBUG", "P1 T?", "SERIAL")
            # Vendor note (240527): ,becausemanual command,defaultSTM32MC
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                self.kaos_log(
                    "DEBUG",
                    "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                    "SERIAL",
                )
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("MC")
                    self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("MC")
                    self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")

                self.G_ProzenToolhead.dwell(2)

            self.kaos_log("DEBUG", "command='%s'; move" % (gcmd.get_commandline(),), "SERIAL")
            self.kaos_log("DEBUG", "move", "SERIAL")
            # for UIUX dynamic interface
            self.emit_protocol("+P1Tn:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            if not int(params["T"]) in range(1, AMS_MAX_CHANNEL + 1):
                raise gcmd.error(
                    "no parameter number command;cmd '%s', that must is T?"
                    % (gcmd.get_commandline())
                )

            self.G_KlipperIfPaused = False
            # Vendor note (240221): STM32 active report, allow one pause
            self.STM32ReprotPauseFlag = 0
            self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")

            # Vendor note (240113): manual commandflag
            self.ManualCmdFlag = True
            # # Vendor note (240529): # self.Cmds_AMSSerial1Send("M0")
            # self.G_PhrozenFluiddRespondInfo("Sending command=M0")
            # self.G_ProzenToolhead.dwell(1)

            if self.G_IfInFilaBlockFlag:
                self.kaos_log(
                    "DEBUG",
                    "feed, first move P1 E?from toolhead up filament tube get output and move prz_resumeresume",
                    "SERIAL",
                )
                self.emit_channel_op("P1END", 0, self.G_ChangeChannelTimeoutNewChan)
                # for UIUX dynamic interface
                self.emit_protocol("+P1Tn:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                return

            # Vendor note (231202): #self.Cmds_MoveToCutFilaAbsolutePositionNotReset(gcmd)
            # Vendor note (240319): before
            # self.G_ChangeChannelTimeoutOldChan=int(params["T"])
            # self.Cmds_MoveToCutFilaPrepare()

            if self.G_ToolheadIfHaveFilaFlag:
                # Vendor note (231205): self.Cmds_MoveToCutFilaAndNotRollback(gcmd)
                self.kaos_log("DEBUG", "toolhead has filament, all AMSfirst", "SERIAL")
                # self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutOldChan)
                # self.G_PhrozenFluiddRespondInfo("G%d" % self.G_ChangeChannelTimeoutOldChan)
                # self.G_PhrozenFluiddRespondInfo("AMS old channel retract a distance first: G%d" % self.G_ChangeChannelTimeoutOldChan)
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("AP")
                    self.kaos_log("DEBUG", "Serial port 1 sending command: AP", "SERIAL")
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("AP")
                    self.kaos_log("DEBUG", "Serial port 2 sending command: AP", "SERIAL")

                self.G_ProzenToolhead.dwell(0.5)

                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]External macro command-PG101")
                # command_string = """
                #     PG101
                #     """
                # self.G_PhrozenGCode.run_script_from_command(command_string)
                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]-feed waiting zoneposition;command_string='%s'" % command_string)
                # self.IfDoPG102Flag=True

                # Vendor note (240913): self.G_ProzenToolhead.dwell(6.0)
                # Vendor note (231201): checkfilamentnormal,normal
                self.Cmds_CutFilaIfNormalCheck()
                if self.G_KlipperIfPaused == True:
                    self.kaos_log(
                        "DEBUG",
                        "cut filament 5toolhead detectedfilament, cutter error, please check cutter, pauseklipperprinting",
                        "SERIAL",
                    )
                    # Lo_ChangeChannelIfSuccess = False
                    return

            # self.G_PhrozenFluiddRespondInfo("External macro command-PG109; after cut, heat and purge residual filament")
            # self.PG102Flag=True
            # self.G_PhrozenFluiddRespondInfo("self.Flag=True")
            # command_string = """
            # PG109
            # """
            # self.G_PhrozenGCode.run_script_from_command(command_string)
            # self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)
            # self.PG102Flag=False
            # self.G_PhrozenFluiddRespondInfo("self.Flag=False")

            self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
            self.G_ChangeChannelTimeoutOldGcmd = self.G_ChangeChannelTimeoutNewGcmd
            self.G_ChangeChannelTimeoutNewChan = int(params["T"])
            self.G_ChangeChannelTimeoutNewGcmd = gcmd

            # Vendor note (241030): P1 C1P1 C32,132
            # 1:1 2 3 4
            # 2:5 6 7 8
            # 3:9 10 11 12
            # 4:13 14 15 16
            # 5:17 18 19 20
            # 6:21 22 23 24
            # 7:25 26 27 28
            # 8:29 30 31 32
            # filament change
            self.Cmds_P1TnManualChangeChannel(int(params["T"]), gcmd)
            # Function name:
            # Input parameters:
            # [Translated vendor note] :
            # [Translated vendor note] Description: -20230830
            ####################################
            # [Translated vendor note] P2 A1 park positionprint Yes; ====="AP";
            # [Translated vendor note] P2 A2; filament Yes; "CL";
            # [Translated vendor note] P2 A3 filament
            # [Translated vendor note] P2 A4 filamentfilament
            # [Translated vendor note] P2 A7 filamentfilament, pause, printAMSfilament

            # for UIUX dynamic interface
            self.emit_protocol("+P1Tn:1,%d" % self.G_ChangeChannelTimeoutNewChan)

        # P1 B[n]
        if "B" in params:
            self.kaos_log("DEBUG", "P1 B?", "SERIAL")
            # Vendor note (240527): ,becausemanual command,defaultSTM32MC
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                self.kaos_log(
                    "DEBUG",
                    "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                    "SERIAL",
                )
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("MC")
                    self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("MC")
                    self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")

                self.G_ProzenToolhead.dwell(2)

            self.kaos_log(
                "DEBUG", "command='%s'; filament full output" % (gcmd.get_commandline(),), "SERIAL"
            )
            # for UIUX dynamic interface
            self.emit_protocol("+P1Bn:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            if not int(params["B"]) in range(1, AMS_MAX_CHANNEL + 1):
                raise gcmd.error(
                    "no parameter number command;cmd '%s', that must is B?"
                    % (gcmd.get_commandline())
                )

            self.G_KlipperIfPaused = False
            # Vendor note (240221): STM32 active report, allow one pause
            self.STM32ReprotPauseFlag = 0
            self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")

            # Vendor note (240113): manual commandflag
            self.ManualCmdFlag = True
            # # Vendor note (240529): # self.Cmds_AMSSerial1Send("M0")
            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]Sending command=M0")
            # self.G_ProzenToolhead.dwell(1)

            # Vendor note (240319): before
            # self.G_ChangeChannelTimeoutNewChan=int(params["B"])
            # self.Cmds_MoveToCutFilaPrepare()

            # # Vendor note (231202): # #self.Cmds_MoveToCutFilaAbsolutePositionNotReset(gcmd)
            # # Vendor note (231205): # # Vendor note (240328): # self.Cmds_MoveToCutFilaAndNotRollback(gcmd)

            if self.G_ToolheadIfHaveFilaFlag == True:
                # Vendor note (231205): self.Cmds_MoveToCutFilaAndNotRollback(gcmd)
                self.kaos_log("DEBUG", "toolhead has filament, all AMSfirst", "SERIAL")
                # self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutOldChan)
                # self.G_PhrozenFluiddRespondInfo("G%d" % self.G_ChangeChannelTimeoutOldChan)
                # self.G_PhrozenFluiddRespondInfo("AMS old channel retract a distance first: G%d" % self.G_ChangeChannelTimeoutOldChan)
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("AP")
                    self.kaos_log("DEBUG", "Serial port 1 sending command: AP", "SERIAL")
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("AP")
                    self.kaos_log("DEBUG", "Serial port 2 sending command: AP", "SERIAL")

                self.G_ProzenToolhead.dwell(0.5)

                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]External macro command-PG101")
                # command_string = """
                #     PG101
                #     """
                # self.G_PhrozenGCode.run_script_from_command(command_string)
                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]-feed waiting zoneposition;command_string='%s'" % command_string)
                # self.IfDoPG102Flag=True

                # Vendor note (240913): self.G_ProzenToolhead.dwell(6.0)
                # Vendor note (231201): checkfilamentnormal,normal
                self.Cmds_CutFilaIfNormalCheck()
                if self.G_KlipperIfPaused == True:
                    self.kaos_log(
                        "DEBUG",
                        "cut filament 5toolhead detectedfilament, cutter error, please check cutter, pauseklipperprinting",
                        "SERIAL",
                    )
                    # Lo_ChangeChannelIfSuccess = False
                    return

            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)]External macro command-PG109; after cut, purge residual filament")
            # self.PG102Flag=True
            # self.G_PhrozenFluiddRespondInfo("[(dev.python)]self.Flag=True")
            # command_string = """
            # PG109
            # """
            # self.G_PhrozenGCode.run_script_from_command(command_string)
            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)]command_string='%s'" % command_string)
            # self.PG102Flag=False
            # self.G_PhrozenFluiddRespondInfo("[(dev.python)]self.Flag=False")

            if self.G_IfInFilaBlockFlag:
                self.kaos_log(
                    "DEBUG",
                    "feed, first move P1 E?from toolhead up filament tube get output and move prz_resumeresume",
                    "SERIAL",
                )
                self.emit_channel_op("P1END", 0, self.G_ChangeChannelTimeoutNewChan)
                # for UIUX dynamic interface
                self.emit_protocol("+P1Bn:1,%d" % self.G_ChangeChannelTimeoutNewChan)

                return

            self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
            self.G_ChangeChannelTimeoutOldGcmd = self.G_ChangeChannelTimeoutNewGcmd
            self.G_ChangeChannelTimeoutNewChan = int(params["B"])
            self.G_ChangeChannelTimeoutNewGcmd = gcmd

            #
            self.Cmds_P1BnWholeRollbackAction(int(params["B"]), gcmd)

            # Vendor note (240115): 1
            time.sleep(1)
            # for UIUX dynamic interface
            self.emit_protocol("+P1Bn:1,%d" % self.G_ChangeChannelTimeoutNewChan)

        # P1 D[n]
        if "D" in params:
            self.kaos_log("DEBUG", "P1 D?", "SERIAL")
            # Vendor note (240527): ,becausemanual command,defaultSTM32MC
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                self.kaos_log(
                    "DEBUG",
                    "Unknown mode, because to action move command, defaultSTM32enterMCmulti-material mode",
                    "SERIAL",
                )
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("MC")
                    self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("MC")
                    self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")

                self.G_ProzenToolhead.dwell(2)

            self.kaos_log(
                "DEBUG", "command='%s'; move to park position" % (gcmd.get_commandline(),), "SERIAL"
            )
            # for UIUX dynamic interface
            self.emit_protocol("+P1Dn:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            if not int(params["D"]) in range(1, AMS_MAX_CHANNEL + 1):
                raise gcmd.error(
                    "no parameter number command;cmd '%s', that must is D?"
                    % (gcmd.get_commandline())
                )

            self.G_KlipperIfPaused = False
            # Vendor note (240221): STM32 active report, allow one pause
            self.STM32ReprotPauseFlag = 0
            self.kaos_log("DEBUG", "self.STM32ReprotPauseFlag=0", "SERIAL")

            # Vendor note (240113): manual commandflag
            self.ManualCmdFlag = True
            # # Vendor note (240529): # self.Cmds_AMSSerial1Send("M0")
            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]Sending command=M0")
            # self.G_ProzenToolhead.dwell(1)

            # Vendor note (240319): before
            # self.G_ChangeChannelTimeoutNewChan=int(params["D"])
            # self.Cmds_MoveToCutFilaPrepare()

            if self.G_ToolheadIfHaveFilaFlag:
                # Vendor note (231205): self.Cmds_MoveToCutFilaAndNotRollback(gcmd)
                self.kaos_log("DEBUG", "toolhead has filament, all AMSfirst", "SERIAL")
                # self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutOldChan)
                # self.G_PhrozenFluiddRespondInfo("G%d" % self.G_ChangeChannelTimeoutOldChan)
                # self.G_PhrozenFluiddRespondInfo("AMS old channel retract a distance first: G%d" % self.G_ChangeChannelTimeoutOldChan)
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("AP")
                    self.kaos_log("DEBUG", "Serial port 1 sending command: AP", "SERIAL")
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("AP")
                    self.kaos_log("DEBUG", "Serial port 2 sending command: AP", "SERIAL")

                self.G_ProzenToolhead.dwell(0.5)

                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]External macro command-PG101")
                # command_string = """
                #     PG101
                #     """
                # self.G_PhrozenGCode.run_script_from_command(command_string)
                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]-feed waiting zoneposition;command_string='%s'" % command_string)
                # self.IfDoPG102Flag=True

                # Vendor note (240913): self.G_ProzenToolhead.dwell(6.0)
                # Vendor note (231201): checkfilamentnormal,normal
                self.Cmds_CutFilaIfNormalCheck()
                if self.G_KlipperIfPaused == True:
                    self.kaos_log(
                        "DEBUG",
                        "cut filament 5toolhead detectedfilament, cutter error, please check cutter, pauseklipperprinting",
                        "SERIAL",
                    )
                    # Lo_ChangeChannelIfSuccess = False
                    return

            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)]External macro command-PG109; after cut, purge residual filament")
            # self.PG102Flag=True
            # self.G_PhrozenFluiddRespondInfo("[(dev.python)]self.Flag=True")
            # command_string = """
            # PG109
            # """
            # self.G_PhrozenGCode.run_script_from_command(command_string)
            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)]command_string='%s'" % command_string)
            # self.PG102Flag=False
            # self.G_PhrozenFluiddRespondInfo("[(dev.python)]self.Flag=False")

            if self.G_IfInFilaBlockFlag:
                self.kaos_log(
                    "DEBUG",
                    "feed, first move P1 E?from toolhead up filament tube get output and move prz_resumeresume",
                    "SERIAL",
                )
                self.emit_channel_op("P1END", 0, self.G_ChangeChannelTimeoutNewChan)
                # for UIUX dynamic interface
                self.emit_protocol("+P1Dn:1,%d" % self.G_ChangeChannelTimeoutNewChan)
                return

            self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
            self.G_ChangeChannelTimeoutOldGcmd = self.G_ChangeChannelTimeoutNewGcmd
            self.G_ChangeChannelTimeoutNewChan = int(params["D"])
            self.G_ChangeChannelTimeoutNewGcmd = gcmd

            #
            self.Cmds_P1DnMoveToParkPositonAction(int(params["D"]), gcmd)

            # Vendor note (240115): 1
            time.sleep(1)

            # for UIUX dynamic interface
            self.emit_protocol("+P1Dn:1,%d" % self.G_ChangeChannelTimeoutNewChan)

        # Vendor note (231202): ,
        # P1 E[n]
        if "E" in params:
            self.kaos_log("DEBUG", "P1 E?", "SERIAL")
            # # Vendor note (240527): ,becausemanual command,defaultSTM32MC
            # if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]Unknown mode, taking manual action, default STM32 enter MC multi-material mode")
            #     self.Cmds_AMSSerial1Send("MC")
            #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]Sending command: MC")

            self.kaos_log(
                "DEBUG",
                "command='%s'; before switch machine, get output toolhead filament tube output"
                % (gcmd.get_commandline(),),
                "SERIAL",
            )
            # for UIUX dynamic interface
            self.emit_protocol("+P1En:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            if not int(params["E"]) in range(1, AMS_MAX_CHANNEL + 1):
                raise gcmd.error(
                    "no parameter number command;cmd '%s', that must is E?"
                    % (gcmd.get_commandline())
                )

            # Vendor note (231202): # Vendor note (231214): filament,,need
            # self.Cmds_MoveToCutFilaAbsolutePositionNotReset(gcmd)

            # Vendor note (240603): prevent
            # time.sleep(2)

            #
            self.Cmds_P1EnForceForward(int(params["E"]), gcmd)

            # for UIUX dynamic interface
            self.emit_protocol("+P1En:1,%d" % self.G_ChangeChannelTimeoutNewChan)

        # Vendor note (240228): distance,needstm32distance
        # P1 G[n];n:1~32(device,1~4);distance Yes;====="G?";
        if "G" in params:
            self.kaos_log("DEBUG", "P1 G?", "SERIAL")
            # # Vendor note (240527): ,becausemanual command,defaultSTM32MC
            # if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]Unknown mode, taking manual action, default STM32 enter MC multi-material mode")
            #     self.Cmds_AMSSerial1Send("MC")
            #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]Sending command: MC")

            self.kaos_log("DEBUG", "command='%s'AMSfirst" % (gcmd.get_commandline(),), "SERIAL")
            # for UIUX dynamic interface
            self.emit_protocol("+P1Gn:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            if not int(params["G"]) in range(1, AMS_MAX_CHANNEL + 1):
                raise gcmd.error(
                    "no parameter number command;cmd '%s', that must is G?"
                    % (gcmd.get_commandline())
                )

            if self.G_ToolheadIfHaveFilaFlag:
                # Vendor note (231205): self.Cmds_MoveToCutFilaAndNotRollback(gcmd)
                self.kaos_log("DEBUG", "toolhead has filament, all AMSfirst", "SERIAL")
                # self.Cmds_AMSSerial1Send("G%d" % self.G_ChangeChannelTimeoutOldChan)
                # self.G_PhrozenFluiddRespondInfo("G%d" % self.G_ChangeChannelTimeoutOldChan)
                # self.G_PhrozenFluiddRespondInfo("AMS old channel retract a distance first: G%d" % self.G_ChangeChannelTimeoutOldChan)
                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("AP")
                    self.kaos_log("DEBUG", "Serial port 1 sending command: AP", "SERIAL")
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("AP")
                    self.kaos_log("DEBUG", "Serial port 2 sending command: AP", "SERIAL")

                self.G_ProzenToolhead.dwell(0.5)

                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]External macro command-PG101")
                # command_string = """
                #     PG101
                #     """
                # self.G_PhrozenGCode.run_script_from_command(command_string)
                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]-feed waiting zoneposition;command_string='%s'" % command_string)
                # self.IfDoPG102Flag=True

                # Vendor note (240913): self.G_ProzenToolhead.dwell(6.0)
                # Vendor note (231201): checkfilamentnormal,normal
                self.Cmds_CutFilaIfNormalCheck()
                if self.G_KlipperIfPaused == True:
                    self.kaos_log(
                        "DEBUG",
                        "cut filament 5toolhead detectedfilament, cutter error, please check cutter, pauseklipperprinting",
                        "SERIAL",
                    )
                    # Lo_ChangeChannelIfSuccess = False
                    return

            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)]External macro command-PG109; after cut, purge residual filament")
            # self.PG102Flag=True
            # self.G_PhrozenFluiddRespondInfo("[(dev.python)]self.Flag=True")
            # command_string = """
            # PG109
            # """
            # self.G_PhrozenGCode.run_script_from_command(command_string)
            # self.G_PhrozenFluiddRespondInfo("[(cmds.python)]command_string='%s'" % command_string)
            # self.PG102Flag=False
            # self.G_PhrozenFluiddRespondInfo("[(dev.python)]self.Flag=False")

            self.G_ChangeChannelTimeoutOldChan = self.G_ChangeChannelTimeoutNewChan
            self.G_ChangeChannelTimeoutOldGcmd = self.G_ChangeChannelTimeoutNewGcmd
            self.G_ChangeChannelTimeoutNewChan = int(params["G"])
            self.G_ChangeChannelTimeoutNewGcmd = gcmd

            self.Cmds_P1GnExtruderBack(int(params["G"]), gcmd)

            # for UIUX dynamic interface
            self.emit_protocol("+P1Gn:1,%d" % self.G_ChangeChannelTimeoutNewChan)

        if "H" in params:
            self.kaos_log("DEBUG", "P1 H?", "SERIAL")
            # # Vendor note (240527): ,becausemanual command,defaultSTM32MC
            # if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]Unknown mode, taking manual action, default STM32 enter MC multi-material mode")
            #     self.Cmds_AMSSerial1Send("MC")
            #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]Sending command: MC")

            self.kaos_log(
                "DEBUG", "command='%s'special refill" % (gcmd.get_commandline(),), "SERIAL"
            )
            # for UIUX dynamic interface
            self.emit_protocol("+P1Hn:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            if not int(params["H"]) in range(1, AMS_MAX_CHANNEL + 1):
                raise gcmd.error(
                    "no parameter number command;cmd '%s', that must is H?"
                    % (gcmd.get_commandline())
                )

            self.Cmds_P1HnSpecialInfila(int(params["H"]), gcmd)

            # for UIUX dynamic interface
            self.emit_protocol("+P1Hn:1,%d" % self.G_ChangeChannelTimeoutNewChan)

        # =====P1 I[n];stm32need;====="I?";
        if "I" in params:
            self.kaos_log("DEBUG", "P1 I?", "SERIAL")
            # # Vendor note (240527): ,becausemanual command,defaultSTM32MC
            # if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
            #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]Unknown mode, taking manual action, default STM32 enter MC multi-material mode")
            #     self.Cmds_AMSSerial1Send("MC")
            #     self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]Sending command: MC")

            self.kaos_log(
                "DEBUG",
                "command='%s' move extrudewhen stm32 need to refill" % (gcmd.get_commandline(),),
                "SERIAL",
            )
            # for UIUX dynamic interface
            self.emit_protocol("+P1In:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            if not int(params["I"]) in range(-1000, 1000):
                raise gcmd.error(
                    "no parameter number command;cmd '%s', move extrude" % (gcmd.get_commandline())
                )

            self.Cmds_P1InExtruderBack(int(params["I"]), gcmd)

            # for UIUX dynamic interface
            self.emit_protocol("+P1In:1,%d" % self.G_ChangeChannelTimeoutNewChan)

        # =====P1 J[n];;;
        if "J" in params:
            self.kaos_log("DEBUG", "P1 J?", "SERIAL")

            self.kaos_log(
                "DEBUG",
                "command='%s' move extrudewhen stm32 need to refill" % (gcmd.get_commandline(),),
                "SERIAL",
            )
            # for UIUX dynamic interface
            self.emit_protocol("+P1Jn:0,%d" % self.G_ChangeChannelTimeoutNewChan)
            if not int(params["J"]) in range(-1000, 1000):
                raise gcmd.error(
                    "no parameter number command;cmd '%s', move extrude parameter number error"
                    % (gcmd.get_commandline())
                )

            self.Cmds_P1JnManualSpitFila(int(params["J"]), gcmd)

            # for UIUX dynamic interface
            self.emit_protocol("+P1Jn:1,%d" % self.G_ChangeChannelTimeoutNewChan)

        # Vendor note (240105): completeAT
        self.emit_channel_op("P1END", 0, self.G_ChangeChannelTimeoutNewChan)

    # P0 M1;(device) Yes;"MC";P0 M1;P28;P2 A1;
    # P0 M2;refill mode(device);"MA";P0 M2;P28;P8;
    # P0 M3; ;P0 M3;
    # Vendor note (240801): # P0 B?
    def Cmds_CmdP0(self, gcmd):
        # P0 traces are protocol — voronFDM pattern-matches them (e.g. LED_SetState)
        # to toggle hardware GPIO via the screen MCU. Bypass the KAOS filter.
        self.emit_protocol("[(cmds.py)Cmds_CmdP0]命令='%s'" % (gcmd.get_commandline(),))
        self.emit_protocol("V-H%s-I%s-F%s" % (HW_VERSION, IMAGE_VERSION, FW_VERSION))

        # get command parametersM?
        params = gcmd.get_command_parameters()

        # if not "M" in params:
        #     return

        # unlock
        self.Base_AMSSerialCmdUnlock()

        # Vendor note (240801): P2 M?
        if "M" in params:
            # KAOS: skip AMS mode-set when ams_attached is false
            if not self.G_AmsAttached:
                self.kaos_log("DEBUG", "P0 M skipped: ams_attached is false", "SERIAL")
                return

            # Vendor note (250522): clearAMS
            self.G_AMSDevice1IfNormal = False
            self.G_AMSDevice2IfNormal = False

            self.G_IfToolheadHaveFilaInitiativePauseFlag = False
            # Vendor note (250526): ,allowgcode,complete
            self.G_KlipperInPausing = False
            # Vendor note (250527): execute
            self.G_KlipperQuickPause = False
            # Vendor note (250607): print status: 1=retracting; 2=feeding; 3=printing; 4=paused
            self.G_KlipperPrintStatus = -1
            self.G_ASM1DisconnectErrorCount = 0
            # Vendor note (250812): single-color runout detection, return to pause zone
            self.G_RetryToPauseAreaFlag = False
            self.G_RetryToPauseAreaCount = 0
            self.G_P10SpitNum = 0
            self.G_IfChangeFilaOngoing = False
            # Vendor note (240223): failed
            self.ToolheadCutFlag = False

            # Vendor note (250618): M3detect
            self.G_P0M3Flag = False
            # Vendor note (250618): M2detect
            self.G_P0M2MAStartPrintFlag = 0
            self.ManualCmdFlag = False
            self.G_CutCheckTest = False

            # Vendor note (250515): clearconfigdata
            self.Cmds_GetUartScreenCfgClear()

            # Vendor note (250514): read json file for single-color refill config and channel-color mapping
            # /home/prz/klipper/klippy/extras/phrozen_dev/serial-screen/test.json
            self.Cmds_GetUartScreenCfg()

            self.kaos_log("DEBUG", "Current mode", "SERIAL")
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                self.emit_mode(0, "unkown")
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
                self.emit_mode(1, "MC")
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                self.emit_mode(2, "MA")
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                self.emit_mode(3, "RUNOUT")
            else:
                self.emit_mode(-1, "error")

            # time.sleep(0.5)
            self.kaos_log("DEBUG", "when 1", "SERIAL")
            self.G_ProzenToolhead.dwell(1)

            # Vendor note (250619): check if AMS reconnected successfully
            self.Cmds_USBConnectErrorCheck()

            # #cancel
            # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
            # Vendor note (250517): #self.G_PhrozenPrinterCancelPauseResume.cmd_CLEAR_PAUSE
            # #cancel
            # self.G_PhrozenPrinterCancelPauseResume.cmd_CANCEL_PRINT(None)
            Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
            self.kaos_log(
                "DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL"
            )
            self.G_PhrozenPrinterCancelPauseResume.cmd_CLEAR_PAUSE(None)
            self.kaos_log("DEBUG", "clearpause state", "SERIAL")
            Lo_PauseStatus = self.G_PhrozenPrinterCancelPauseResume.get_status(None)
            self.kaos_log(
                "DEBUG", "Current pause state-Lo_PauseStatus='%s'" % Lo_PauseStatus, "SERIAL"
            )
            self.kaos_log(
                "DEBUG", "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"], "SERIAL"
            )
            # // current-Lo_PauseStatus='{'is_paused': True}'
            if Lo_PauseStatus["is_paused"] == True:
                self.kaos_log("DEBUG", "Already paused", "SERIAL")
            else:
                self.kaos_log("DEBUG", "Not currently paused", "SERIAL")
            # Vendor note (240511): ,preventAMS
            try:
                self.kaos_log("DEBUG", "Reinitializing serial port 1", "SERIAL")
                self.G_SerialPort1Obj = serial.Serial(
                    self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3
                )
                # serial port opened successfully
                if self.G_SerialPort1Obj is not None:
                    if self.G_SerialPort1Obj.is_open:
                        self.G_SerialPort1OpenFlag = True
                        self.kaos_log("DEBUG", "Reinitializing serial port 1 successful", "SERIAL")
                        # Vendor note (231213): open serial port
                        self.G_SerialPort1Obj.flushInput()  # clean serial write cache
                        self.G_SerialPort1Obj.flush()
                        self.kaos_log("DEBUG", "Serial port 1 buffers cleared", "SERIAL")
                        self.kaos_log("DEBUG", "Re-registering serial port 1 callback", "SERIAL")
                        self.G_SerialPort1RecvTimmer = self.G_PhrozenReactor.register_timer(
                            self.Device_TimmerUart1Recv, self.G_PhrozenReactor.NOW
                        )
            except:
                self.kaos_log(
                    "DEBUG",
                    "Unable to open tty1. Check the USB connection or try rebooting.",
                    "SERIAL",
                )

            try:
                self.kaos_log("DEBUG", "Reinitializing serial port 2", "SERIAL")
                self.G_SerialPort2Obj = serial.Serial(
                    self.G_Serialport2Define, SERIAL_PORT_BAUD, timeout=3
                )
                # serial port opened successfully
                if self.G_SerialPort2Obj is not None:
                    if self.G_SerialPort2Obj.is_open:
                        self.G_SerialPort2OpenFlag = True
                        self.kaos_log("DEBUG", "Reinitializing serial port 2 successful", "SERIAL")
                        self.G_SerialPort2Obj.flushInput()  # clean serial write cache
                        self.G_SerialPort2Obj.flush()
                        self.kaos_log("DEBUG", "Serial port 2 buffers cleared", "SERIAL")
                        self.kaos_log("DEBUG", "Re-registering serial port 2 callback", "SERIAL")
                        self.G_SerialPort2RecvTimmer = self.G_PhrozenReactor.register_timer(
                            self.Device_TimmerUart2Recv, self.G_PhrozenReactor.NOW
                        )
            except:
                self.kaos_log(
                    "DEBUG",
                    "Unable to open tty2. Check the USB connection or try rebooting.",
                    "SERIAL",
                )

            # Vendor note (250323): self.G_PhrozenFluiddRespondInfo("External macro command-PG103- get toolhead; full")
            command_string = """
                # PG103
                """
            self.G_PhrozenGCode.run_script_from_command(command_string)
            self.kaos_log(
                "DEBUG",
                "External macro command-PG103; command_string='%s'" % command_string,
                "SERIAL",
            )

            # parameter
            Lo_AMSWorkMode = int(params["M"])

            if not Lo_AMSWorkMode in [
                # AMS_WORK_MODE_UNKNOW,# 0
                # AMS_WORK_MODE_MC,#MC 1
                # AMS_WORK_MODE_MA,#MA 2
                # AMS_WORK_MODE_FILA_RUNOUT,# 3
            ]:
                raise gcmd.error(
                    "no parameter number command;cmd '%s', that must is M[0/1/2/3]"
                    % (gcmd.get_commandline(),)
                )

            self.G_AMSDeviceWorkMode = Lo_AMSWorkMode
            self.Cmds_PrintMode(self.G_AMSDeviceWorkMode)

            self.kaos_log("DEBUG", "Current mode", "SERIAL")
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                self.emit_mode(0, "unkown")
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
                self.emit_mode(1, "MC")
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                self.emit_mode(2, "MA")
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                self.emit_mode(3, "RUNOUT")
            else:
                self.emit_mode(-1, "error")

            self.G_CancelFlag = False
            self.G_KlipperIfPaused = False
            # Vendor note (240413): STM32 active report, allow one pause
            self.STM32ReprotPauseFlag = 0

            # Function name:
            # Input parameters:
            # [Translated vendor note] :
            # [Translated vendor note] Description: -20230830
            ####################################
            # [Translated vendor note] P1 S0 filamentfeedpark position; ====="RD";
            # [Translated vendor note] P1 T[n]n:1 ~32(,1 ~4)channel,(); ====="T";
            # [Translated vendor note] P1 B[n]n:1 ~32(,1 ~4)channel Yes; ====="B";
            # [Translated vendor note] P1 D[n]; n:1~32(,1~4); channelpark position Yes; ====="P";
            # [Translated vendor note] P1 C[n] n:1~32(,1~4) channel(,, , ); ====="T";
            # Vendor note (231202):
            # [Translated vendor note] P1 E[n]; n:1~32(,1~4); channel, toolhead Yes; ====="E?";
            # [Translated vendor note] Vendor note (240228): toolhead, stm32
            # [Translated vendor note] P1 G[n]; n:1~32(,1~4); channel Yes; ====="G?";
            # Vendor note (240319):
            # [Translated vendor note] =====P1 H[n]; n:1~32(,1~4); filament change Yes; ====="H?";
            # [Translated vendor note] Vendor note (240329):
            # [Translated vendor note] =====P1 I[n]; stm32; ====="I?";
            # [Translated vendor note] =====P1 J[n]; purge; buffer;
            # =====P1 K[n];
            # =====P1 L[n];
            # =====P1 M[n];
            # =====P1 N[n];
            # =====P1 O[n];
            # =====P1 Q[n];
            # =====P1 U[n];
            # Vendor note (240418):
            # [Translated vendor note] =====P1 V[n];
            # =====P1 W[n];
            # =====P1 X[n];
            # =====P1 Y[n];
            # =====P1 Z[n];

            self.G_P0M1MCNoneAMS = 0
            self.kaos_log("DEBUG", "self.G_P0M1MCNoneAMS=0", "SERIAL")

            self.kaos_log("DEBUG", "when 1", "SERIAL")
            self.G_ProzenToolhead.dwell(1)

            # =====M0;
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:  # 0
                self.G_ToolheadFirstInputFila = False
                self.kaos_log("DEBUG", "P0 M0Unknown mode", "SERIAL")

                # Vendor note (240411): ifno P0 M3 command received, skip runout detection
                self.G_P0M3Flag = False

                self.G_P0M1MCNoneAMS = 0
                self.kaos_log("DEBUG", "self.G_P0M1MCNoneAMS=0", "SERIAL")

                # Vendor note (250327): filament changebefore,allowAMS
                self.ManualCmdFlag = True
                self.kaos_log("DEBUG", "self.ManualCmdFlag=True", "SERIAL")

                # Vendor note (250104): P2A3flag
                if self.G_P2A3Flag == 1:
                    self.G_P2A3Flag = 0
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("AT+IDLE")
                        self.kaos_log("DEBUG", "Serial port 1 sending command: AT+IDLE", "SERIAL")
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("AT+IDLE")
                        self.kaos_log("DEBUG", "Serial port 2 sending command: AT+IDLE", "SERIAL")

                else:
                    if self.G_SerialPort1OpenFlag == True:
                        self.Cmds_AMSSerial1Send("M0")
                        self.kaos_log("DEBUG", "Serial port 1 sending command: M0", "SERIAL")
                    if self.G_SerialPort2OpenFlag == True:
                        self.Cmds_AMSSerial2Send("M0")
                        self.kaos_log("DEBUG", "Serial port 2 sending command: M0", "SERIAL")

                self.G_ProzenToolhead.dwell(0.5)

            # =====M1;
            # P0 M1
            # P28
            # P2 A1
            # T?
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:  # 1
                self.kaos_log("DEBUG", "P0 M1multi-material mode: MC", "SERIAL")

                self.ManualCmdFlag = False

                # Vendor note (250304): P0M1,preventlogchannel number
                self.G_ChangeChannelTimeoutOldChan = -1
                self.kaos_log("DEBUG", "self.G_ChangeChannelTimeoutOldChan=-1", "SERIAL")
                self.G_ChangeChannelTimeoutOldGcmd = None
                self.kaos_log("DEBUG", "self.G_ChangeChannelTimeoutOldGcmd=None", "SERIAL")
                self.G_ChangeChannelTimeoutNewChan = -1
                self.kaos_log("DEBUG", "self.G_ChangeChannelTimeoutNewChan=-1", "SERIAL")
                self.G_ChangeChannelTimeoutNewGcmd = None
                self.kaos_log("DEBUG", "self.G_ChangeChannelTimeoutNewGcmd=None", "SERIAL")

                # Vendor note (250102): filament changecalculate
                self.G_PrintCountNum = 0

                # Vendor note (240125): P28can
                # Vendor note (231219): # Vendor note (240319): PG28coordinates
                # self.Cmds_MoveToCutFilaAndNotRollback(gcmd)

                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("MC")
                    self.kaos_log("DEBUG", "Serial port 1 sending command: MC", "SERIAL")

                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("MC")
                    self.kaos_log("DEBUG", "Serial port 2 sending command: MC", "SERIAL")
                # [Translated vendor note] #Vendor note (240705): P114AMS,
                # if self.G_AMSDevice1IfNormal==False:
                # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]self.G_AMSDevice1IfNormal==False")
                # [Translated vendor note] #
                # params = gcmd.get_command_parameters()
                # [Translated vendor note] # =====P1 I[n]; stm32; ====="I?";
                # if "I" in params:
                # [Translated vendor note] self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]AMSP114, ")
                # [Translated vendor note] #Vendor note (240524): UIUX
                # self.emit_protocol("+P1In:0,%d" % self.G_ChangeChannelTimeoutNewChan)
                # if not int(params["I"]) in range(-1000, 1000):
                # [Translated vendor note] raise gcmd.error(";cmd '%s', " % (gcmd.get_commandline()))
                # # command_string = """
                # # M106 S0
                # # M83
                # # G92 E0
                # # G1 E%f F300
                # # """ %(int(params["I"]),)
                # # self.G_PhrozenGCode.run_script_from_command(command_string)
                # [Translated vendor note] # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP1]GCODE: %s" % command_string)

                self.G_ChangeChannelFirstFilaFlag = True

                # time.sleep(0.5)
                self.G_ProzenToolhead.dwell(2.5)

                self.kaos_log("DEBUG", "check is no has AMS", "SERIAL")

                if self.G_SerialPort1OpenFlag == True:
                    try:
                        self.kaos_log("DEBUG", "try;Lo_AMSDeviceStateRspInfo", "SERIAL")
                        # get detailed AMS board state
                        Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort1SendWaitRsp(
                            "SD", sizeof(AMSDetailInfoBytes)
                        )
                        self.kaos_log(
                            "DEBUG",
                            "tty1 serial port receive : %s" % Lo_AMSDeviceStateRspInfo,
                            "SERIAL",
                        )
                        if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                            self.kaos_log(
                                "DEBUG",
                                "AMS1, please checkAMS1='%s'" % (gcmd.get_commandline(),),
                                "SERIAL",
                            )
                            # Vendor note (240412): AMSlabel
                            self.G_AMSDevice1IfNormal = False
                        else:
                            # Vendor note (240412): AMSlabel
                            self.G_AMSDevice1IfNormal = True
                    except:
                        self.kaos_log("DEBUG", "except;Lo_AMSDeviceStateRspInfo", "SERIAL")

                if self.G_SerialPort2OpenFlag == True:
                    try:
                        self.kaos_log("DEBUG", "try;Lo_AMSDeviceStateRspInfo", "SERIAL")
                        # get detailed AMS board state
                        Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort2SendWaitRsp(
                            "SD", sizeof(AMSDetailInfoBytes)
                        )
                        self.kaos_log(
                            "DEBUG",
                            "tty2 serial port receive : %s" % Lo_AMSDeviceStateRspInfo,
                            "SERIAL",
                        )
                        if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                            self.kaos_log(
                                "DEBUG",
                                "AMS2, please checkAMS2='%s'" % (gcmd.get_commandline(),),
                                "SERIAL",
                            )
                            self.G_AMSDevice2IfNormal = False
                        else:
                            self.G_AMSDevice2IfNormal = True
                    except:
                        self.kaos_log("DEBUG", "except;Lo_AMSDeviceStateRspInfo", "SERIAL")

                self.G_ProzenToolhead.dwell(1.0)

                # Vendor note (250515): P0 M1,needAMS
                if self.G_AMSDevice1IfNormal == True or self.G_AMSDevice2IfNormal == True:
                    self.kaos_log(
                        "DEBUG", "=====multi-materialmulti-material type P0 M1, has AMS", "SERIAL"
                    )
                    self.kaos_log(
                        "DEBUG", "=====multi-materialmulti-material type P0 M1, P2 A1", "SERIAL"
                    )
                    self.kaos_log(
                        "DEBUG", "=====multi-materialmulti-material type P0 M1, T?", "SERIAL"
                    )
                    # Vendor note (250722): ;
                    if self.G_AutoReplaceState == 1:
                        self.kaos_log(
                            "DEBUG",
                            "=====multi-materialmulti-material type automatic refill;P0 M1 move switch is P0 M2, refill",
                            "SERIAL",
                        )
                        self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MA

                        # # Vendor note (240416): # if self.G_SerialPort1OpenFlag == True:
                        #     self.Cmds_AMSSerial1Send("MA")
                        #     self.G_PhrozenFluiddRespondInfo("Serial port 1 sending command: MA")
                        # # Vendor note (241030): # if self.G_SerialPort2OpenFlag == True:
                        #     self.Cmds_AMSSerial2Send("MA")
                        #     self.G_PhrozenFluiddRespondInfo("Serial port 2 sending command: MA")

                        self.Cmds_PrintMode(self.G_AMSDeviceWorkMode)
                        self.kaos_log(
                            "DEBUG", "self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MA", "SERIAL"
                        )
                        self.kaos_log("DEBUG", "P8", "SERIAL")
                        # Vendor note (241106): self.Cmds_CmdP8(gcmd)

                        if self.G_ToolheadIfHaveFilaFlag:
                            # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("can resume, STM32printing report error")
                            self.G_KlipperQuickPause = True
                            # # Vendor note (250427): # if self.G_SerialPort1OpenFlag == True:
                            #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                            #     self.G_PhrozenFluiddRespondInfo("serial port 1-AMSfinishtimingbuffer-full time")
                            # if self.G_SerialPort2OpenFlag == True:
                            #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                            #     self.G_PhrozenFluiddRespondInfo("serial port 2-AMSfinishtimingbuffer-full time")
                            # #self.G_ProzenToolhead.dwell(1.5)
                            # Vendor note (251120): entering purge, set flag to prevent PG108 purge Hall-sensor pause at purge zone;
                            self.G_PG108Ingoing = 1
                            command_string = """
                            # PG108
                            """
                            self.G_PhrozenGCode.run_script_from_command(command_string)
                            self.G_PG108Ingoing = 0
                            self.kaos_log("DEBUG", "command_string='%s'" % command_string, "SERIAL")
                            self.kaos_log(
                                "DEBUG",
                                "purge complete, toolhead detectedhas filament, printing",
                                "SERIAL",
                            )

                    else:
                        self.kaos_log("DEBUG", "=====multi-materialmulti-material type", "SERIAL")
                else:
                    self.kaos_log(
                        "DEBUG",
                        "=====single-colormulti-material type P0 M1, has AMS, P0 M3single-color printing, disable disable P2 A1 and T?",
                        "SERIAL",
                    )
                    self.kaos_log("DEBUG", "=====P0 M1 switch is P0 M3", "SERIAL")
                    self.G_AMSDeviceWorkMode = AMS_WORK_MODE_FILA_RUNOUT
                    self.Cmds_PrintMode(self.G_AMSDeviceWorkMode)
                    self.kaos_log(
                        "DEBUG", "self.G_AMSDeviceWorkMode = AMS_WORK_MODE_FILA_RUNOUT", "SERIAL"
                    )
                    self.G_P0M1MCNoneAMS = 1
                    self.kaos_log("DEBUG", "self.G_P0M1MCNoneAMS=1", "SERIAL")
                    # Vendor note (240411): detect
                    self.G_P0M3Flag = True
                    self.kaos_log("DEBUG", "self.G_P0M3Flag = True", "SERIAL")
                    if self.G_AutoReplaceState == 1:
                        self.kaos_log(
                            "DEBUG",
                            "=====single-colormulti-material type automatic refill;",
                            "SERIAL",
                        )
                    else:
                        self.kaos_log("DEBUG", "=====single-colormulti-material type", "SERIAL")

                    self.kaos_log(
                        "DEBUG",
                        "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"],
                        "SERIAL",
                    )
                    # // current-Lo_PauseStatus='{'is_paused': True}'
                    if Lo_PauseStatus["is_paused"] == True:
                        self.kaos_log("DEBUG", "Already paused", "SERIAL")
                    else:
                        self.kaos_log("DEBUG", "Not currently paused", "SERIAL")

                    if self.G_ToolheadIfHaveFilaFlag == True:
                        self.kaos_log("DEBUG", "detectedfilament, startprinting", "SERIAL")
                        # Vendor note (240412): ifAMS,executeMA
                        # Vendor note (241030): AMS,1start,1
                        if self.G_AMSDevice1IfNormal == True:
                            self.kaos_log(
                                "DEBUG", "AMS1/tty1 not connected; send P28 first.", "SERIAL"
                            )
                            self.G_ChangeChannelFirstFilaFlag = True
                        elif self.G_AMSDevice2IfNormal == True:
                            self.kaos_log(
                                "DEBUG",
                                "AMS2/tty2 not available; skipping serial port 2.",
                                "SERIAL",
                            )
                            self.G_ChangeChannelFirstFilaFlag = True
                        else:
                            self.kaos_log(
                                "DEBUG",
                                "detectedfilament, AMS multi-material not connected, please move filament",
                                "SERIAL",
                            )
                            # Vendor note (240411): detect
                            self.G_P0M3Flag = True
                            # Vendor note (240415): filament,
                            self.G_P0M3ToolheadHaveFilaNotSpittingFlag = True
                        # Vendor note (251120): entering purge, set flag to prevent PG108 purge Hall-sensor pause at purge zone;
                        self.G_PG108Ingoing = 1
                        command_string = """
                        # PG108
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.G_PG108Ingoing = 0
                        self.kaos_log("DEBUG", "command_string='%s'" % command_string, "SERIAL")
                        self.kaos_log(
                            "DEBUG",
                            "purge complete, toolhead detectedhas filament, printing",
                            "SERIAL",
                        )

                    else:
                        self.kaos_log("DEBUG", "detectedfilament, pause", "SERIAL")
                        # Vendor note (240407): cannot
                        self.Cmds_PhrozenKlipperPauseM2M3NoneCmdToSTM32(None)
                        # Vendor note (240411): detect
                        self.G_P0M3Flag = True
                        # self.emit_protocol("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                        self.G_PhrozenFluiddRespondInfo(
                            "+PAUSE:b,%d,%d"
                            % (
                                self.G_ChangeChannelTimeoutOldChan,
                                self.G_ChangeChannelTimeoutNewChan,
                            )
                        )

                self.G_ProzenToolhead.dwell(0.5)

            # =====M2;refill mode
            # P0 M2
            # P28
            # P8
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:  # 2
                self.kaos_log("DEBUG", "=====P0 M2single-color refill mode: MA", "SERIAL")

                self.ManualCmdFlag = False

                # Vendor note (250102): filament changecalculate
                self.G_PrintCountNum = 0

                if self.G_SerialPort1OpenFlag == True:
                    self.Cmds_AMSSerial1Send("MA")
                    self.kaos_log("DEBUG", "Serial port 1 sending command: MA", "SERIAL")
                if self.G_SerialPort2OpenFlag == True:
                    self.Cmds_AMSSerial2Send("MA")
                    self.kaos_log("DEBUG", "Serial port 2 sending command: MA", "SERIAL")

                self.G_ChangeChannelFirstFilaFlag = True

                self.G_ProzenToolhead.dwell(0.5)

                # time.sleep(1)
                # Vendor note (240104): M2MArefill modecannot
                # Vendor note (20231219): #self.Cmds_MoveToCutFilaAndNotRollback(gcmd)

            # =====M3;detect
            # Vendor note (250511): ifAMS multi-material present,refill mode;P0 M3convertP0 M2
            elif self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:  # 3
                self.kaos_log("DEBUG", "P0 M3single-colorfilament", "SERIAL")

                self.ManualCmdFlag = False

                # Vendor note (250102): filament changecalculate
                self.G_PrintCountNum = 0

                if self.G_SerialPort1OpenFlag == True:
                    try:
                        self.kaos_log("DEBUG", "try;Lo_AMSDeviceStateRspInfo", "SERIAL")
                        # get detailed AMS board state
                        Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort1SendWaitRsp(
                            "SD", sizeof(AMSDetailInfoBytes)
                        )
                        self.kaos_log(
                            "DEBUG",
                            "tty1 serial port receive : %s" % Lo_AMSDeviceStateRspInfo,
                            "SERIAL",
                        )
                        if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                            self.kaos_log(
                                "DEBUG",
                                "AMS1, please checkAMS1='%s'" % (gcmd.get_commandline(),),
                                "SERIAL",
                            )
                            # Vendor note (240412): AMSlabel
                            self.G_AMSDevice1IfNormal = False
                        else:
                            # Vendor note (240412): AMSlabel
                            self.G_AMSDevice1IfNormal = True
                    except:
                        self.kaos_log("DEBUG", "except;Lo_AMSDeviceStateRspInfo", "SERIAL")

                if self.G_SerialPort2OpenFlag == True:
                    try:
                        self.kaos_log("DEBUG", "try;Lo_AMSDeviceStateRspInfo", "SERIAL")
                        # get detailed AMS board state
                        Lo_AMSDeviceStateRspInfo = self.Cmds_AMSSerialPort2SendWaitRsp(
                            "SD", sizeof(AMSDetailInfoBytes)
                        )
                        self.kaos_log(
                            "DEBUG",
                            "tty2 serial port receive : %s" % Lo_AMSDeviceStateRspInfo,
                            "SERIAL",
                        )
                        if len(Lo_AMSDeviceStateRspInfo) != sizeof(AMSDetailInfoBytes):
                            self.kaos_log(
                                "DEBUG",
                                "AMS2, please checkAMS2='%s'" % (gcmd.get_commandline(),),
                                "SERIAL",
                            )
                            self.G_AMSDevice2IfNormal = False
                        else:
                            self.G_AMSDevice2IfNormal = True
                    except:
                        self.kaos_log("DEBUG", "except;Lo_AMSDeviceStateRspInfo", "SERIAL")

                if self.G_AMSDevice1IfNormal == True or self.G_AMSDevice2IfNormal == True:
                    self.G_P0M2MAStartPrintFlag = 0

                    # Vendor note (250104): prevent
                    # self.Cmds_CmdP28(None)

                    self.kaos_log("DEBUG", "=====multi-material single-color type P0 M3", "SERIAL")
                    self.kaos_log(
                        "DEBUG", "self.G_AutoReplaceState=%d" % self.G_AutoReplaceState, "SERIAL"
                    )

                    # Vendor note (250514): ifenable,P0 M3convertP0 M2
                    if self.G_AutoReplaceState == 1:
                        # Vendor note (250511): ifAMS multi-material present,convertrefill mode
                        # P0 M2
                        # P8
                        self.kaos_log(
                            "DEBUG",
                            "=====multi-material single-color type automatic refill;P0 M3 move switch is P0 M2, refill",
                            "SERIAL",
                        )
                        self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MA
                        self.Cmds_PrintMode(self.G_AMSDeviceWorkMode)
                        self.kaos_log(
                            "DEBUG", "self.G_AMSDeviceWorkMode = AMS_WORK_MODE_MA", "SERIAL"
                        )
                        self.kaos_log("DEBUG", "P8", "SERIAL")
                        # Vendor note (241106): self.Cmds_CmdP8(gcmd)

                        if self.G_ToolheadIfHaveFilaFlag:
                            # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("can resume, STM32printing report error")
                            self.G_KlipperQuickPause = True
                            # # Vendor note (250427): # if self.G_SerialPort1OpenFlag == True:
                            #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                            #     self.G_PhrozenFluiddRespondInfo("serial port 1-AMSfinishtimingbuffer-full time")
                            # if self.G_SerialPort2OpenFlag == True:
                            #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                            #     self.G_PhrozenFluiddRespondInfo("serial port 2-AMSfinishtimingbuffer-full time")
                            # #self.G_ProzenToolhead.dwell(1.5)

                    else:
                        self.kaos_log(
                            "DEBUG",
                            "=====multi-material single-color P0 M3, filament runout",
                            "SERIAL",
                        )
                        self.kaos_log("DEBUG", "P8", "SERIAL")
                        # Vendor note (241106): self.Cmds_CmdP8(gcmd)
                        if self.G_ToolheadIfHaveFilaFlag:
                            # Vendor note (250607): self.G_PhrozenFluiddRespondInfo("can resume, STM32printing report error")
                            self.G_KlipperQuickPause = True
                            # # Vendor note (250427): # if self.G_SerialPort1OpenFlag == True:
                            #     self.Cmds_AMSSerial1Send("AT+EBLOCKEND")
                            #     self.G_PhrozenFluiddRespondInfo("serial port 1-AMSfinishtimingbuffer-full time")
                            # if self.G_SerialPort2OpenFlag == True:
                            #     self.Cmds_AMSSerial2Send("AT+EBLOCKEND")
                            #     self.G_PhrozenFluiddRespondInfo("serial port 2-AMSfinishtimingbuffer-full time")
                            # #self.G_ProzenToolhead.dwell(1.5)
                        # Vendor note (240411): detect
                        self.G_P0M3Flag = True
                        self.kaos_log("DEBUG", "self.G_P0M3Flag = True", "SERIAL")

                    # # Vendor note (250511): ifAMS multi-material present,
                    # #P0 M3
                    # #P8
                    # # Vendor note (250514): # self.G_PhrozenFluiddRespondInfo("P8")
                    # # Vendor note (241106): # self.Cmds_CmdP8(gcmd)

                    # Vendor note (241106): toolhead feed successful
                    if self.G_P0M2MAStartPrintFlag == 1:
                        self.kaos_log("DEBUG", "toolhead has filament", "SERIAL")
                    else:
                        self.kaos_log("DEBUG", "toolhead no filament", "SERIAL")
                else:
                    self.kaos_log("DEBUG", "=====single-colorsingle-color type P0 M3", "SERIAL")
                    self.kaos_log(
                        "DEBUG", "self.G_AutoReplaceState=%d" % self.G_AutoReplaceState, "SERIAL"
                    )

                # Vendor note (231220): filament
                for i in range(10):  #
                    self.G_ProzenToolhead.dwell(1.0)
                    # time.sleep(1)
                    self.kaos_log("DEBUG", "move filament; i=%d" % (i), "SERIAL")

                    if self.G_ToolheadIfHaveFilaFlag == True:
                        self.kaos_log("DEBUG", "detectedfilament, startprinting", "SERIAL")

                        # Vendor note (240412): ifAMS,executeMA
                        # Vendor note (241030): AMS,1start,1
                        if self.G_AMSDevice1IfNormal == True:
                            self.kaos_log(
                                "DEBUG", "AMS1/tty1 not connected; send P28 first.", "SERIAL"
                            )
                            self.G_ChangeChannelFirstFilaFlag = True
                        elif self.G_AMSDevice2IfNormal == True:
                            self.kaos_log(
                                "DEBUG",
                                "AMS2/tty2 not available; skipping serial port 2.",
                                "SERIAL",
                            )
                            self.G_ChangeChannelFirstFilaFlag = True
                        else:
                            self.kaos_log(
                                "DEBUG",
                                "detectedfilament, AMS multi-material not connected, please move filament",
                                "SERIAL",
                            )
                            # Vendor note (240411): detect
                            self.G_P0M3Flag = True
                            # Vendor note (240415): filament,
                            self.G_P0M3ToolheadHaveFilaNotSpittingFlag = True

                        # Vendor note (251120): entering purge, set flag to prevent PG108 purge Hall-sensor pause at purge zone;
                        self.G_PG108Ingoing = 1
                        command_string = """
                        # PG108
                        """
                        self.G_PhrozenGCode.run_script_from_command(command_string)
                        self.kaos_log("DEBUG", "command_string='%s'" % command_string, "SERIAL")
                        self.kaos_log(
                            "DEBUG",
                            "purge complete, toolhead detectedhas filament, printing",
                            "SERIAL",
                        )
                        self.G_PG108Ingoing = 0
                        break
                    if i >= 9:
                        # Vendor note (240412): ifAMS,executeMA
                        if self.G_AMSDevice1IfNormal == True:
                            self.kaos_log(
                                "DEBUG", "AMS1/tty1 not connected; send P28 first.", "SERIAL"
                            )
                            self.G_ChangeChannelFirstFilaFlag = True
                        elif self.G_AMSDevice2IfNormal == True:
                            self.kaos_log(
                                "DEBUG",
                                "AMS2/tty2 not available; skipping serial port 2.",
                                "SERIAL",
                            )
                            self.G_ChangeChannelFirstFilaFlag = True
                        else:
                            self.kaos_log(
                                "DEBUG",
                                "AMS multi-material not connected, please move filament",
                                "SERIAL",
                            )
                            self.kaos_log("DEBUG", "filamentfeedtimeout;pause", "SERIAL")

                            self.kaos_log(
                                "DEBUG",
                                "Lo_PauseStatus['is_paused']='%s'" % Lo_PauseStatus["is_paused"],
                                "SERIAL",
                            )
                            # // current-Lo_PauseStatus='{'is_paused': True}'
                            if Lo_PauseStatus["is_paused"] == True:
                                self.kaos_log("DEBUG", "Already paused", "SERIAL")
                            else:
                                self.kaos_log("DEBUG", "Not currently paused", "SERIAL")

                                # Vendor note (240407): cannot
                                self.Cmds_PhrozenKlipperPauseM2M3NoneCmdToSTM32(None)
                                # Vendor note (240411): detect
                                self.G_P0M3Flag = True

                                # self.emit_protocol("+PAUSE:b,%d" % self.G_ChangeChannelTimeoutNewChan)
                                self.G_PhrozenFluiddRespondInfo(
                                    "+PAUSE:b,%d,%d"
                                    % (
                                        self.G_ChangeChannelTimeoutOldChan,
                                        self.G_ChangeChannelTimeoutNewChan,
                                    )
                                )

                            return

                self.G_ProzenToolhead.dwell(0.5)

            self.kaos_log("DEBUG", "when 0.5", "SERIAL")
            # Vendor note (231228): preventstm32AT
            self.G_ProzenToolhead.dwell(0.5)
            # self.G_ProzenToolhead.dwell(1.0)

        # Vendor note (240801): P0 B?
        if "B" in params:
            # self.Cmds_P1CnAutoChangeChannel(int(params["C"]), gcmd)
            # for UIUX dynamic interface
            self.kaos_log("DEBUG", gcmd.get_commandline(), "SERIAL")

    # Register G-code commands and their handlers for the AMS feature set
    def Cmds_RegisterCmds(self):
        self.kaos_log(
            "DEBUG", "[(cmds.python)Cmds_RegisterCmds]register Phrozen gcode command", "SERIAL"
        )
        # P114 S;;"SB";
        # P114; ;"SD";
        self.G_PhrozenGCode.register_command(
            G_DictPhrozenCmdP114["cmd"],
            self.Cmds_CmdP114,
            desc=G_DictPhrozenCmdP114["desc"],
        )
        # P28 device
        self.G_PhrozenGCode.register_command(
            G_DictPhrozenCmdP28["cmd"],
            self.Cmds_CmdP28,
            desc=G_DictPhrozenCmdP28["desc"],
        )
        # P29 disconnect
        self.G_PhrozenGCode.register_command(
            G_DictPhrozenCmdP29["cmd"],
            self.Cmds_CmdP29,
            desc=G_DictPhrozenCmdP29["desc"],
        )
        # P30 deviceID(device);"I";deviceID
        self.G_PhrozenGCode.register_command(
            G_DictPhrozenCmdP30["cmd"],
            self.Cmds_CmdP30,
            desc=G_DictPhrozenCmdP30["desc"],
        )
        # P0 M1;(device) Yes;"MC";
        # P0 M2;refill mode(device);"MA";
        # P0 M3;
        # Vendor note (240801): # P0 B?; ;
        self.G_PhrozenGCode.register_command(
            G_DictPhrozenP0["cmd"], self.Cmds_CmdP0, desc=G_DictPhrozenP0["desc"]
        )
        # P2 A1 retract to parkposition Yes;====="AP";
        # P2 A2;exitfilament Yes;"CL";
        # P2 A3 filament
        # P2 A4 filamentfilament
        # P2 A5 completefilamentfilament,cannot
        self.G_PhrozenGCode.register_command(
            G_DictPhrozenCmdP2["cmd"], self.Cmds_CmdP2, desc=G_DictPhrozenCmdP2["desc"]
        )
        # P1 S0 , canretract to park;====="RD";
        # P1 T[n]n:1 ~32(device,1 ~4),();====="T";
        # P1 B[n]n:1 ~32(device,1 ~4)exit Yes;====="B";
        # P1 D[n];n:1~32(device,1~4); Yes;====="P";
        # P1 C[n] n:1~32(device,1~4) (,, , );====="T";
        # Vendor note (231202): # P1 E[n];n:1~32(device,1~4);,need Yes;====="E?";
        # Vendor note (240228): distance,needstm32distance
        # P1 G[n];n:1~32(device,1~4);distance Yes;====="G?";
        # Vendor note (240319): phase,
        # =====P1 H[n];n:1~32(device,1~4);phase, Yes;====="H?";
        # Vendor note (240329): # =====P1 I[n];stm32need;====="I?";
        # =====P1 J[n];;;
        # =====P1 K[n];
        # =====P1 L[n];
        # =====P1 M[n];
        # =====P1 N[n];
        # =====P1 O[n];
        # =====P1 Q[n];
        # =====P1 U[n];
        # Vendor note (240418): # =====P1 V[n];
        # =====P1 W[n];
        # =====P1 X[n];
        # =====P1 Y[n];
        # =====P1 Z[n];
        self.G_PhrozenGCode.register_command(
            G_DictPhrozenCmdP1["cmd"], self.Cmds_CmdP1, desc=G_DictPhrozenCmdP1["desc"]
        )
        # P9 X[x_pos]Y[y_pos]W[width]H[height]D[0/1];x_pos:Xcoordinatesy_pos:Ycoordinateswidth:
        # height:D0:XYcount(default)D1:YXcountwaiting area
        # P9 T[expire]A[0/1];expire:time,(default60)A0:,continue(default)A1:
        self.G_PhrozenGCode.register_command(
            G_DictPhrozenCmdP9["cmd"], self.Cmds_CmdP9, desc=G_DictPhrozenCmdP9["desc"]
        )

        # Vendor note (241101): # P10 S?    parameterS[1,5]:control,S1-1,S2-2...,5
        self.G_PhrozenGCode.register_command(
            G_DictPhrozenCmdP10["cmd"],
            self.Cmds_CmdP10,
            desc=G_DictPhrozenCmdP10["desc"],
        )

        # Vendor note (250805): # P11
        self.G_PhrozenGCode.register_command("P11", self.Cmds_CmdP11, desc="P11")
        # Vendor note (250805): # P12 loop
        self.G_PhrozenGCode.register_command("P12", self.Cmds_CmdP12, desc="P12")

        # P8 execute Yes;"FA";
        self.G_PhrozenGCode.register_command(
            G_DictPhrozenCmdP8["cmd"], self.Cmds_CmdP8, desc=G_DictPhrozenCmdP8["desc"]
        )
        # PRZ_ADC
        self.G_PhrozenGCode.register_command(
            G_DictPhrozenCmdToolheadAdc["cmd"],
            self.Cmds_PhrozenAdc,
            desc=G_DictPhrozenCmdToolheadAdc["desc"],
        )
        # PRZ_PAUSE
        self.G_PhrozenGCode.register_command(
            "PRZ_PAUSE", self.Cmds_PhrozenKlipperPauseScreen, desc="PHROZEN_PAUSE"
        )
        # PRZ_RESUME
        self.G_PhrozenGCode.register_command(
            "PRZ_RESUME", self.Cmds_PhrozenKlipperResume, desc="PHROZEN_RESUME"
        )
        # PRZ_CANCEL
        self.G_PhrozenGCode.register_command(
            "PRZ_CANCEL", self.Cmds_PhrozenKlipperCancel, desc="PHROZEN_CANCEL"
        )
        # PRZ_VERSION query
        self.G_PhrozenGCode.register_command(
            "PRZ_VERSION", self.Cmds_PhrozenVersion, desc="PHROZEN_VERSION"
        )
        # P4 device;Stop():"SP";
        self.G_PhrozenGCode.register_command(
            G_DictPhrozenCmdP4["cmd"], self.Cmds_CmdP4, desc=G_DictPhrozenCmdP4["desc"]
        )

        self.G_PhrozenGCode.register_command("PRZ_BM1", self.Cmds_PhrozenBM1, desc="PRZ_BM1")
        self.G_PhrozenGCode.register_command("PRZ_BM0", self.Cmds_PhrozenBM0, desc="PRZ_BM0")

        self.G_PhrozenGCode.register_command(
            "PRZ_PRINT_START", self.Cmds_PrzPrintStart, desc="PRZ_PRINT_START"
        )

        # self.G_PhrozenGCode.register_command("PRINT_END",self.Cmds_PrzPrintEnd,desc="PRINT_END")
        # Vendor note (250514): self.G_PhrozenGCode.register_command("homing_override_end",self.Cmds_HomingOverrideEnd,desc="homing_override_end")

        # Vendor note (250115): self.G_PhrozenGCode.register_command("PRZ_RESTORE",self.Cmds_PrzATRestore,desc="PRZ_RESTORE")
        self.G_PhrozenGCode.register_command("PRZ_IDLE", self.Cmds_PrzATIdle, desc="PRZ_IDLE")

        # Vendor note (250324): orcaT0 T1 T2 T3
        self.G_PhrozenGCode.register_command("T0", self.Cmds_CmdT0, desc="T0")
        self.G_PhrozenGCode.register_command("T1", self.Cmds_CmdT1, desc="T1")
        self.G_PhrozenGCode.register_command("T2", self.Cmds_CmdT2, desc="T2")
        self.G_PhrozenGCode.register_command("T3", self.Cmds_CmdT3, desc="T3")
        self.G_PhrozenGCode.register_command("T4", self.Cmds_CmdT4, desc="T4")
        self.G_PhrozenGCode.register_command("T5", self.Cmds_CmdT5, desc="T5")
        self.G_PhrozenGCode.register_command("T6", self.Cmds_CmdT6, desc="T6")
        self.G_PhrozenGCode.register_command("T7", self.Cmds_CmdT7, desc="T7")
        self.G_PhrozenGCode.register_command("T8", self.Cmds_CmdT8, desc="T8")
        self.G_PhrozenGCode.register_command("T9", self.Cmds_CmdT9, desc="T9")
        self.G_PhrozenGCode.register_command("T10", self.Cmds_CmdT10, desc="T10")
        self.G_PhrozenGCode.register_command("T11", self.Cmds_CmdT11, desc="T11")
        self.G_PhrozenGCode.register_command("T12", self.Cmds_CmdT12, desc="T12")
        self.G_PhrozenGCode.register_command("T13", self.Cmds_CmdT13, desc="T13")
        self.G_PhrozenGCode.register_command("T14", self.Cmds_CmdT14, desc="T14")
        self.G_PhrozenGCode.register_command("T15", self.Cmds_CmdT15, desc="T15")

        # [Translated vendor note] P1 S0 channelpark positionfeedprint, feedpark positionpark position; ====="RD";
        # [Translated vendor note] P1 T[n]n:1 ~32(,1 ~4)channel,(); ====="T";
        # [Translated vendor note] P1 B[n]n:1 ~32(,1 ~4)channel Yes; ====="B";
        # [Translated vendor note] P1 D[n]; n:1~32(,1~4); channelpark position Yes; ====="P";
        # [Translated vendor note] P1 C[n] n:1~32(,1~4) channel(,, , ); ====="T";
        # Vendor note (231202):
        # [Translated vendor note] P1 E[n]; n:1~32(,1~4); channel, toolhead Yes; ====="E?";
        # [Translated vendor note] Vendor note (240228): toolhead, stm32
        # [Translated vendor note] P1 G[n]; n:1~32(,1~4); channel Yes; ====="G?";
        # [Translated vendor note] Vendor note (240319): , buffer
        # [Translated vendor note] =====P1 H[n]; n:1~32(,1~4); , buffer Yes; ====="H?";
        # [Translated vendor note] Vendor note (240329):
        # [Translated vendor note] =====P1 I[n]; stm32; ====="I?";
        # [Translated vendor note] =====P1 J[n]; purge; buffer;
        # =====P1 K[n];
        # =====P1 L[n];
        # =====P1 M[n];
        # =====P1 N[n];
        # =====P1 O[n];
        # =====P1 Q[n];
        # =====P1 U[n];
        # Vendor note (240418):
        # [Translated vendor note] =====P1 V[n];
        # =====P1 W[n];
        # =====P1 X[n];
        # =====P1 Y[n];
        # =====P1 Z[n];

    # Function name:
    # Input parameters:
    # [Translated vendor note] :
    # [Translated vendor note] Description: -20230830
    ####################################
    # [Translated vendor note] P0 M1; modemode() Yes; "MC";P0 M1;P28;P2 A1;
    # [Translated vendor note] P0 M2; mode(); "MA";P0 M2;P28;P8;
    # [Translated vendor note] P0 M3; mode;P0 M3;
    # Vendor note (240801):
    # P0 B?
