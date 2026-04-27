####################################
#Project Name: 
#Chip Type: 
#Function: 
# Developer: Lan Caigang
#Development Date: 20230830
####################################

import binascii
import logging
import time
import struct
import serial
import re

from .cwebsocketapis import *

# Runtime log verbosity levels
LOG_LEVEL_ERROR = 0
LOG_LEVEL_WARN = 1
LOG_LEVEL_INFO = 2
LOG_LEVEL_DEBUG = 3


####################################
#Class Name: 
# Description: Lan Caigang-20230830
####################################
class PhrozenDev(Apis):
    def _prz_parse_log_level(self, msg):
        if msg.startswith("[ERROR]"):
            return "ERROR", msg[7:].strip()
        if msg.startswith("[WARN]"):
            return "WARN", msg[6:].strip()
        if msg.startswith("[DEBUG]"):
            return "DEBUG", msg[7:].strip()
        if msg.startswith("[INFO]"):
            return "INFO", msg[6:].strip()
        return "INFO", msg

    def _prz_should_log(self, level):
        level_map = {
            "ERROR": LOG_LEVEL_ERROR,
            "WARN": LOG_LEVEL_WARN,
            "INFO": LOG_LEVEL_INFO,
            "DEBUG": LOG_LEVEL_DEBUG,
        }
        return level_map.get(level, LOG_LEVEL_INFO) <= self.G_LogVerbosity

    def _prz_detect_log_category(self, msg):
        lowered = msg.lower()
        if any(token in lowered for token in ["serial", "tty", "uart", "usb", "p28", "p29"]):
            return "SERIAL"
        if any(token in lowered for token in ["toolhead", "nozzle", "purge", "spit", "cut", "park", "pause_waitingarea", "movement", "move", "hall sensor"]):
            return "TOOLHEAD"
        if any(token in lowered for token in ["ams", "filament", "channel", "multicolor", "multi-material", "runout", "p0m3", "p114", "mc_state", "ma_state"]):
            return "AMS"
        return "DEV"

    def _prz_render_log(self, level, msg):
        parts = []
        if self.G_LogTimestampEnabled:
            parts.append("[%s]" % time.strftime("%H:%M:%S"))
        parts.append("[%s]" % level)
        if self.G_LogCategoriesEnabled:
            parts.append("[%s]" % self._prz_detect_log_category(msg))
        parts.append(msg)
        return " ".join(parts)

def _prz_filtered_respond_info(self, msg):
    try:
        text = str(msg).strip()

        # Pass through anything that is not explicitly a log message.
        # Phrozen/HMI uses RespondInfo for control messages too.
        if not (
            text.startswith("[INFO]") or
            text.startswith("[WARN]") or
            text.startswith("[ERROR]") or
            text.startswith("[DEBUG]")
        ):
            self._prz_original_respond_info(msg)
            return

        # Pass through log-wrapped protocol messages such as [INFO] +P114:2
        if (
            text.startswith("[INFO] +") or
            text.startswith("[WARN] +") or
            text.startswith("[ERROR] +") or
            text.startswith("[DEBUG] +")
        ):
            self._prz_original_respond_info(msg)
            return

        level, clean_msg = self._prz_parse_log_level(text)

        if self._prz_should_log(level):
            self._prz_original_respond_info(
                self._prz_render_log(level, clean_msg)
            )

    except Exception:
        self._prz_original_respond_info(msg)

    def Device_SetLogLevel(self, gcmd):
        level = gcmd.get_int("LEVEL", self.G_LogVerbosity)
        if level < LOG_LEVEL_ERROR:
            level = LOG_LEVEL_ERROR
        if level > LOG_LEVEL_DEBUG:
            level = LOG_LEVEL_DEBUG
        self.G_LogVerbosity = level
        self._prz_original_respond_info(self._prz_render_log("INFO", "Log verbosity set to %d (0=ERROR, 1=WARN, 2=INFO, 3=DEBUG)" % level))

    def Device_SetLogFlags(self, gcmd):
        self.G_LogCategoriesEnabled = bool(gcmd.get_int("CATEGORIES", 1 if self.G_LogCategoriesEnabled else 0))
        self.G_LogTimestampEnabled = bool(gcmd.get_int("TIMESTAMP", 1 if self.G_LogTimestampEnabled else 0))
        self._prz_original_respond_info(self._prz_render_log(
            "INFO",
            "Log flags updated: CATEGORIES=%d TIMESTAMP=%d" % (
                1 if self.G_LogCategoriesEnabled else 0,
                1 if self.G_LogTimestampEnabled else 0,
            )
        ))

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    #Initialize constructor
    def __init__(self, config):
        super(PhrozenDev, self).__init__(config)
        self.G_LogVerbosity = LOG_LEVEL_INFO
        self.G_LogCategoriesEnabled = True
        self.G_LogTimestampEnabled = False
        self._prz_original_respond_info = self.G_PhrozenFluiddRespondInfo
        self.G_PhrozenFluiddRespondInfo = self._prz_filtered_respond_info
        #Initialize Klipper connect event
        self.G_PhrozenPrinter.register_event_handler("klippy:connect", self.Device_KlipperConnectHandle)
        #Initialize Klipper disconnect event
        self.G_PhrozenPrinter.register_event_handler("klippy:disconnect", self.Device_KlipperDisconnectHandle)

        # dev.py; Reset AMS runtime parameters
        self.Device_ResetParams()

        # cmds.py; Phrozen custom G-code commands
        self.Cmds_RegisterCmds()

        # cwebsocketapis.py; WebSocket API for the web UI
        self.WebsocketAPIs_RegisterAPIs()

        # dev.py; Test commandPRZ_TEST
        self.G_PhrozenGCode.register_command("PRZ_TEST", self.Device_CmdPhrozenTest, desc='PhrozenAMS unitTest command')
        self.G_PhrozenGCode.register_command("PRZ_LOG_LEVEL", self.Device_SetLogLevel, desc="Set log verbosity (0=ERROR,1=WARN,2=INFO,3=DEBUG)")
        self.G_PhrozenGCode.register_command("PRZ_LOG_FLAGS", self.Device_SetLogFlags, desc="Set log flags (CATEGORIES=0/1 TIMESTAMP=0/1)")
    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    #Reset AMS parameters
    def Device_ResetParams(self):
        self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)Device_ResetParams]Reset AMS runtime parameters")
        self.G_FilaRunoutTimmer = None  # runout-handling timer
        self.G_SerialPort1RecvTimmer = None  # serial receive timer
        #lancaigang241030: 
        self.G_SerialPort2RecvTimmer = None  # serial receive timer

        self.AMSRunoutPauseTimeCount = 0  # temporary wait-time counter in the daemon thread
        self.G_ToolheadFirstInputFila = False  # first feed
        self.P0M3FilaRunoutSpittingFinished = False
        self.AMSErrorRetryTimes = 0  # error retry count
        #AMS multi-material state
        self.G_AMS1DeviceState = {}
        #lancaigang241105:
        self.G_AMS2DeviceState = {}

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    #Register runout-monitor timer thread
    def Device_RegisterRunoutErrorThread(self):
        self.G_PhrozenFluiddRespondInfo("[ERROR] [(dev.python)Device_RegisterRunoutErrorThread]")
        # registerfilament runout logical thread
        self.G_FilaRunoutTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerRunoutCheck, self.G_PhrozenReactor.NOW + 0.5)
    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Device_UnregisterDaemonThread(self):
        self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)Device_UnregisterDaemonThread]")
        #Unregister
        self.G_PhrozenReactor.unregister_timer(self.G_FilaRunoutTimmer)
        # self.G_PhrozenReactor.unregister_timer(self.G_SerialPort1RecvTimmer)
    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Device_ConnectAMSDevice(self):
        self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)Device_ConnectAMSDevice]Phrozen extension module connecting to the AMS multi-material unit")
        #Whether to auto-connect the AMS multi-material unit
        #lancaigang240116: Do not auto-connect the AMS
        #if self.G_AMSIfAutoConnectFlag:
            # ttyUSB0serial portconnectAMSmulti-material machine
            #self.Cmds_CmdP28(None)
                # #lancaigang231122: Before using ttyUSB0, stop the background IAP updater process hdl_zigbee_gateway


    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Device_DisconnectAMSDevice(self):
        self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)Device_DisconnectAMSDevice]Phrozen extension module disconnecting from the AMS multi-material unit")
        self.Cmds_CmdP29(None)
    
    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    def Device_CmdPhrozenTest(self, gcmd):
        self.G_PhrozenFluiddRespondInfo("[(dev.python)Device_CmdPhrozenTest]PRZ_TEST command='%s'" % (gcmd.get_commandline(),))
        self.G_PhrozenFluiddRespondInfo("self.prz_test command='%s'" % (gcmd.get_commandline(),))
        # klipperpause
        self.Cmds_PhrozenKlipperPause(None)



    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    #Initial event registration; Phrozen plugin connected to Klipper
    def Device_KlipperConnectHandle(self):
        self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)Device_KlipperConnectHandle]Phrozen extension module connected to Klipper")
        
        #lancaigang250724: Read the system image ID to distinguish product, mainboard, and firmware variants
        #lancaigang250724:Read image ID
        self.Cmds_GetImageId()
        if self.G_ImageId==16:
            self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==16: ARCO300-MKS-RK3328-STM32F407VET6-I16")
            os.system('sh /home/mks/klipper/klippy/extras/phrozen_dev/stop.sh &')
            self.G_PhrozenFluiddRespondInfo("[INFO] sh /home/mks/klipper/klippy/extras/phrozen_dev/stop.sh &")
        elif self.G_ImageId==31:
            self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==31: ARCO300-phrozen-RK3308-STM32F407VET6-I31")
            os.system('sh /home/prz/klipper/klippy/extras/phrozen_dev/stop.sh &')
            self.G_PhrozenFluiddRespondInfo("[INFO] sh /home/prz/klipper/klippy/extras/phrozen_dev/stop.sh &")
        elif self.G_ImageId==-1:
            self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==-1, default: ARCO300-MKS-RK3328-STM32F407VET6-I16")
            os.system('sh /home/mks/klipper/klippy/extras/phrozen_dev/stop.sh &')
            self.G_PhrozenFluiddRespondInfo("[INFO] sh /home/mks/klipper/klippy/extras/phrozen_dev/stop.sh &")
        else:
            self.G_PhrozenFluiddRespondInfo("[INFO] Image IDcould not be read, default: ARCO300-MKS-RK3328-STM32F407VET6-I16")
            os.system('sh /home/mks/klipper/klippy/extras/phrozen_dev/stop.sh &')
            self.G_PhrozenFluiddRespondInfo("[INFO] sh /home/mks/klipper/klippy/extras/phrozen_dev/stop.sh &")

        # get toolhead methods
        self.G_ProzenToolhead = self.G_PhrozenPrinter.lookup_object("toolhead")
        #manual toolhead movement
        self.G_ToolheadManualMovement = self.G_ProzenToolhead.manual_move
        #wait for toolhead movement to finish
        self.G_ToolheadWaitMovementEnd = self.G_ProzenToolhead.wait_moves
        #last toolhead position
        self.G_ToolheadLastPosition = self.G_ProzenToolhead.get_position()
        # registerfilament runoutthread
        self.Device_RegisterRunoutErrorThread()

        # lancaigang240430: becausehaspower-loss recoveryFunction, ifAMSalready keep state, restart after cannot
        # lancaigang240428: ifklipperrestartconnect, then AMSidlestate
        try:
            #Open serial port 1 at 19200 baud
            self.G_SerialPort1Obj = serial.Serial(self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3)
            if self.G_SerialPort1Obj.is_open:
                # lancaigang231213: openserial port
                self.G_SerialPort1Obj.flushInput()
                self.G_SerialPort1Obj.flush()
                #lancaigang250115:multi-material power-loss recovery
                self.G_PhrozenFluiddRespondInfo("[INFO] Sending command: M0")
                self.G_SerialPort1Obj.write("M0".encode())
                self.G_SerialPort1Obj.flush()
                #Close the serial port to prevent later P28 issues
                self.G_SerialPort1Obj.close()
        except:
            self.G_PhrozenFluiddRespondInfo("[ERROR] Unable to open tty1. Check the USB connection or try rebooting.")

        #lancaigang241108: 
        try:
            #Open serial port 2 at 19200 baud
            self.G_SerialPort2Obj = serial.Serial(self.G_Serialport2Define, SERIAL_PORT_BAUD, timeout=3)
            if self.G_SerialPort2Obj.is_open:
                self.G_SerialPort2Obj.flushInput()
                self.G_SerialPort2Obj.flush()
                self.G_PhrozenFluiddRespondInfo("[INFO] Sending command:  M0")
                self.G_SerialPort2Obj.write("M0".encode())
                self.G_SerialPort2Obj.flush()
                #Close the serial port to prevent later P28 issues
                self.G_SerialPort2Obj.close()
        except:
            self.G_PhrozenFluiddRespondInfo("[ERROR] Unable to open tty2. Check the USB connection or try rebooting.")


        # lancaigang240427: AMSerrorrestart, need to
        self.G_AMS1ErrorRestartFlag = False
        self.G_AMS1ErrorRestartCount = 0

        #lancaigang241030:
        self.G_AMS2ErrorRestartFlag = False
        self.G_AMS2ErrorRestartCount = 0


        #lancaigang250514: Read the JSON file for monochrome refill settings and channel/color mappings
        #/home/prz/klipper/klippy/extras/phrozen_dev/serial-screen/test.json






    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    #Initial event registration; Phrozen plugin disconnected from Klipper
    def Device_KlipperDisconnectHandle(self):
        self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)Device_KlipperDisconnectHandle]Phrozen extension module disconnected from Klipper")
        # cancelconnectAMSmulti-material machine
        self.Device_DisconnectAMSDevice()
        self.Device_UnregisterDaemonThread()
        #Reset AMS runtime parameters
        self.Device_ResetParams()
    
    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    #Monochrome M3 filament runout
    #Monochrome MA refill-mode filament runout
    #2s
    def Device_TimmerRunoutCheck(self, eventtime):
        # lancaigang240528:if is P114readstate, not allowed logical smt32report number data
        if self.G_P114RunFlag>=1:
            self.G_P114RunFlag=self.G_P114RunFlag+1
            if self.G_P114RunFlag>=3:
                self.G_PhrozenFluiddRespondInfo("[ERROR] [(dev.python)Device_TimmerRunoutCheck]P114 failed")
                #self.G_PhrozenFluiddRespondInfo("[INFO] +P114:2")
                #self.G_P114RunFlag=0
                # pythonempty
                Lo_AMSDetailState = {"dev_id": -1, "active_dev_id": -1, "dev_mode": -1, "cache_empty": -1, "cache_full": -1, "cache_exist": -1, "mc_state": -1, "ma_state": -1, "entry_state": -1, "park_state": -1}
                # number data json switch
                self.G_PhrozenFluiddRespondInfo(json.dumps(Lo_AMSDetailState))

                #lancaigang250708: 
                self.G_PhrozenFluiddRespondInfo("[ERROR] P114 failed")
                self.G_PhrozenFluiddRespondInfo("[INFO] +P114:2")
                self.G_P114RunFlag=0
            

            #self.G_PhrozenFluiddRespondInfo("[INFO] +P114:1")
            #self.G_P114RunFlag=False
            #return eventtime + AMS_FILA_RUNOUT_TIMER

        #Default to unknown mode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:#M0
            #self.G_PhrozenFluiddRespondInfo("[WARN] [(dev.python)Device_TimmerRunoutCheck]if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:")
            return eventtime + AMS_FILA_RUNOUT_TIMER


        #lancaigang240410: 
        if self.G_CancelFlag==True:
            # self.G_PhrozenFluiddRespondInfo("[WARN] [(dev.python)Device_TimmerRunoutCheck]Print has been canceled")
            return eventtime + AMS_FILA_RUNOUT_TIMER



            


        #lancaigang240105: If the previous print was standalone single-color and this one is monochrome refill mode, the toolhead may keep pausing with errors when no filament is present, so M3 and M2 are handled separately
        # =====M3filament runout logical mode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:#M3 M2
            # self.G_PhrozenFluiddRespondInfo("[WARN] [(dev.python)Device_TimmerRunoutCheck]single-colormodetimer")
            # lancaigang240411: ifnoreceived P0 M3command, use use filament runoutdetect machine
            if self.G_P0M3Flag == False:
                # self.G_PhrozenFluiddRespondInfo("[WARN] [(dev.python)Device_TimmerRunoutCheck]received P0M3command or hasAMSmulti-material, do not executesingle-colorM3modedetect machine, use use AMSmulti-materialprintingsingle-color")
                return eventtime + AMS_FILA_RUNOUT_TIMER
            #else:
                # self.G_PhrozenFluiddRespondInfo("[WARN] [(dev.python)Device_TimmerRunoutCheck]noAMSmulti-materialandreceived P0M3command, single-colorM3modedetect machine ")

            if self.G_ToolheadIfHaveFilaFlag==True:
                self.G_ToolheadFirstInputFila = True
            if self.G_ToolheadFirstInputFila==False:
                self.G_PhrozenFluiddRespondInfo("[WARN] No filament detected during the first feed")
                return eventtime + AMS_FILA_RUNOUT_TIMER
            if self.G_ToolheadIfHaveFilaFlag==True:
                if self.P0M3FilaRunoutSpittingFinished==True:# purgecomplete
                    # self.G_PhrozenFluiddRespondInfo("[WARN] [(dev.python)Device_TimmerRunoutCheck]purgecomplete")
                    return eventtime + AMS_FILA_RUNOUT_TIMER
                self.G_PhrozenFluiddRespondInfo("[INFO] Filament detected. Starting purge")
                self.G_PhrozenFluiddRespondInfo("[WARN] Calling external macro PG108 to start purging in single-color M3 mode; automatic purge is currently disabled")
                # lancaigang240407: calling purgeFunction, in purgebefore, preventtoolheadfeed up output, time command report error
                self.P0M3FilaRunoutSpittingFinished = True# purgecomplete, prevent time calling command
                # command = """
                #     G90
                #     G1 X250 Y10 F10000
                #     G91
                #     G1 Z10 F500
                #     G92 E0
                #     G1 E100 F500
                #     G92 E0
                #     G0
                # """
                # self.G_PhrozenGCode.run_script_from_command(command)
                # self.G_PhrozenFluiddRespondInfo("[(dev.python)Device_TimmerRunoutCheck]command=%s" % command)
                if self.G_P0M3ToolheadHaveFilaNotSpittingFlag==True:
                    self.G_PhrozenFluiddRespondInfo("[INFO] P0M3 entered printing and found filament already in the toolhead; no purge needed")
                    self.G_P0M3ToolheadHaveFilaNotSpittingFlag=False
                else:
                    self.G_PhrozenFluiddRespondInfo("[WARN] Automatic purge is disabled; resume manually and purge afterward")
                    # command_string = """
                    # PG108
                    # """
                    # self.G_PhrozenGCode.run_script_from_command(command_string)
                    # self.G_PhrozenFluiddRespondInfo("command_string='%s'" % command_string)
                    # self.G_PhrozenFluiddRespondInfo("[INFO] purgecomplete, toolheaddetectto hasfilamentresumeprinting")

                self.STM32ReprotPauseFlag=0
                return self.G_PhrozenReactor.NOW + AMS_FILA_RUNOUT_TIMER
            
            # lancaigang240108: ifalready pause, cannotpause
            if self.G_KlipperIfPaused==True:
                # lancaigang251127: mcu report error timer too close;
                # self.G_PhrozenFluiddRespondInfo("[WARN] P0M3standalonemode, already pause")
                if self.G_RetryToPauseAreaFlag==False:
                    self.G_RetryToPauseAreaCount=self.G_RetryToPauseAreaCount+1
                    self.G_PhrozenFluiddRespondInfo("self.G_RetryToPauseAreaCount=%d" % self.G_RetryToPauseAreaCount)
                    # lancaigang251124:pause after send time pause, preventfilament runoutpausehaswhen;
                    if self.G_RetryToPauseAreaCount >= 6:
                        self.G_RetryToPauseAreaCount=0
                        self.G_RetryToPauseAreaFlag=True
                    else:
                        #lancaigang251124:
                        if self.G_PG108Ingoing==1:
                            self.G_PhrozenFluiddRespondInfo("[INFO] PG108 is purging and the toolhead Hall sensor suddenly detected filament depletion")
                            self.G_PhrozenFluiddRespondInfo("[WARN] PG108 is purging. Do not pause immediately, or the current purge position will be saved as the pause position and may cause a collision with the purge box on resume")
                        else:
                            self.G_PhrozenFluiddRespondInfo("[WARN] PG108 is not purging; it is safe to pause for filament runout")

                            if self.G_KlipperInPausing == True:
                                self.G_PhrozenFluiddRespondInfo("[WARN] Pause is already in progress; duplicate pause is not allowed")
                            else:
                                self.G_PhrozenFluiddRespondInfo("[WARN] Pause is not currently in progress; pausing is allowed")
                                #lancaigang250527: Pause in the waiting area
                                self.G_PhrozenFluiddRespondInfo("[WARN] startcalling External macro command-PRZ_PAUSE_WAITINGAREA")
                                command = """
                                PRZ_PAUSE_WAITINGAREA
                                """
                                self.G_PhrozenGCode.run_script_from_command(command)
                                self.G_PhrozenFluiddRespondInfo('calling External macro command:command=%s' % (command))
                
                return eventtime + AMS_FILA_RUNOUT_TIMER
            
            # lancaigang240407: iftoolheadnofilament, pause
            if self.G_ToolheadIfHaveFilaFlag==False:
                self.G_PhrozenFluiddRespondInfo("[WARN] P0M3 standalone mode: no filament detected")
                self.G_PhrozenFluiddRespondInfo("self.G_IfChangeFilaOngoing=%d" % self.G_IfChangeFilaOngoing)
                # lancaigang250522: in AMSmulti-materialfeed down, canfilament runoutdetect
                if self.G_IfChangeFilaOngoing==False:
                    self.AMSRunoutPauseTimeCount = 0
                    self.G_PhrozenFluiddRespondInfo("[WARN] Standalone M3 single-color runout handling: pausing")
                    
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

                        #lancaigang250607:
                        self.G_PhrozenFluiddRespondInfo("[WARN] Quick pause is disabled")
                        self.G_KlipperQuickPause = False

                        if self.G_KlipperInPausing == True:
                            self.G_PhrozenFluiddRespondInfo("[WARN] Pause is already in progress; duplicate pause is not allowed")
                        else:
                            self.G_PhrozenFluiddRespondInfo("[WARN] Pause is not currently in progress; pausing is allowed")
                            #lancaigang251124:
                            if self.G_PG108Ingoing==1:
                                self.G_PhrozenFluiddRespondInfo("[INFO] PG108 is purging and the toolhead Hall sensor suddenly detected filament depletion")
                                self.G_PhrozenFluiddRespondInfo("[WARN] PG108 is purging. Do not pause immediately, or the current purge position will be saved as the pause position and may cause a collision with the purge box on resume")
                            else:
                                self.G_PhrozenFluiddRespondInfo("[WARN] PG108 is not purging; it is safe to pause for filament runout")

                                self.Cmds_PhrozenKlipperPauseM2M3ToSTM32(None)
                                # lancaigang250812:single-colorfilament runoutdetect, to pause
                                self.G_RetryToPauseAreaFlag = False
                                self.G_RetryToPauseAreaCount = 0
                                #lancaigang250527: Pause in the waiting area
                                self.G_PhrozenFluiddRespondInfo("[WARN] startcalling External macro command-PRZ_PAUSE_WAITINGAREA")
                                command = """
                                PRZ_PAUSE_WAITINGAREA
                                """
                                self.G_PhrozenGCode.run_script_from_command(command)
                                self.G_PhrozenFluiddRespondInfo('calling External macro command:command=%s' % (command))
                                # lancaigang250521:hasAMSmulti-material
                                #if self.G_AMSDevice1IfNormal==True or self.G_AMSDevice2IfNormal==True:
                                #    self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                                #else:
                                self.G_PhrozenFluiddRespondInfo("+PAUSE:b,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))

                                #lancaigang250527: Pause in the waiting area
                                self.G_PhrozenFluiddRespondInfo("[WARN] startcalling External macro command-PRZ_PAUSE_WAITINGAREA")
                                command = """
                                PRZ_PAUSE_WAITINGAREA
                                """
                                self.G_PhrozenGCode.run_script_from_command(command)
                                self.G_PhrozenFluiddRespondInfo('calling External macro command:command=%s' % (command))

            self.P0M3FilaRunoutSpittingFinished = False# wait down time purge
            return self.G_PhrozenReactor.NOW + AMS_FILA_RUNOUT_TIMER


        # #=====MAsingle-colorrefill
        # if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:#M2
        # #lancaigang241106: P8feedsuccessful before prompt down, filament runoutdetect and refill
        #     if self.G_P0M2MAStartPrintFlag==1:
        # #self.G_PhrozenFluiddRespondInfo("[WARN] [(dev.python)Device_TimmerRunoutCheck]P8feedsuccessful, filament runoutdetectrefill")
        #         #if self.G_ToolheadIfHaveFilaFlag==True:
        # # self.G_PhrozenFluiddRespondInfo("[INFO] P8feedsuccessful, toolheadhasfilament")
        #         if self.G_ToolheadIfHaveFilaFlag==False:
        # self.G_PhrozenFluiddRespondInfo("[INFO] P8finished printing one channel, toolheadnonefilament, automaticrefillnew channel, move move to waiting areawaittimeout")
        #             #self.Cmds_CmdP8(None)

        # #lancaigang240104: not allowedpause
        #             if self.G_IfChangeFilaOngoing==False:
        # self.G_PhrozenFluiddRespondInfo("[WARN] single-colorrefilltemporary pause, waitstm32feed new filament")
        # #toolheadno filament, klipperpause
        #                 self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
        #                 self.AMSRunoutPauseTimeCount = 0
        #                 self.AMSRunoutPauseTimeoutFlag=0

        #     self.P0M3FilaRunoutSpittingFinished = False
        #     #self.G_PhrozenFluiddRespondInfo("[WARN] [(dev.python)Device_TimmerRunoutCheck]return;self.P0M3FilaRunoutSpittingFinished = False")
        #     return self.G_PhrozenReactor.NOW + AMS_FILA_RUNOUT_TIMER




        # =====M2MAsingle-colorrefill
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:#M2
            # lancaigang250607: state; 1-unload filament; 2-feed; 3-; 4-pause
            if self.G_KlipperPrintStatus == 3:
                # lancaigang250619:ifdetectedserial porterror, then number
                if self.G_SerialPort1OpenFlag==False:
                    self.G_PhrozenFluiddRespondInfo("[DEBUG] single-color refill modeprinting in progress; if self.G_KlipperPrintStatus == 3")
                    self.G_ASM1DisconnectErrorCount=self.G_ASM1DisconnectErrorCount+1
                    self.G_PhrozenFluiddRespondInfo("self.G_ASM1DisconnectErrorCount=%d" % self.G_ASM1DisconnectErrorCount)
                    if self.G_ASM1DisconnectErrorCount >= 2: #4s
                        try:
                            self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 1")
                            self.G_SerialPort1Obj = serial.Serial(self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3)
                            #Serial port opened successfully
                            if self.G_SerialPort1Obj is not None:
                                if self.G_SerialPort1Obj.is_open:
                                    self.G_SerialPort1OpenFlag = True
                                    self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 1successful")
                                    self.G_ASM1DisconnectErrorCount=0
                                    #self.G_PauseToLCDString=""
                                    # lancaigang231213: openserial port
                                    self.G_SerialPort1Obj.flushInput()  # clean serial write cache
                                    self.G_SerialPort1Obj.flush()
                                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 buffers cleared")
                                    self.G_PhrozenFluiddRespondInfo("[INFO] Re-registering serial port 1 callback")
                                    self.G_SerialPort1RecvTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerUart1Recv, self.G_PhrozenReactor.NOW)
                        except:
                            self.G_PhrozenFluiddRespondInfo("[ERROR] Unable to open tty1. Check the USB connection or try rebooting.")
                            self.G_SerialPort1OpenFlag=False
                            self.G_ASM1DisconnectErrorCount=self.G_ASM1DisconnectErrorCount+1
                            self.G_PhrozenFluiddRespondInfo("self.G_ASM1DisconnectErrorCount=%d" % self.G_ASM1DisconnectErrorCount)
                            if self.G_ASM1DisconnectErrorCount >= 5: #10s
                                self.G_ASM1DisconnectErrorCount=0
                                self.G_PhrozenFluiddRespondInfo('[INFO] AMS1connectedfilament runout10s, pause')
                                if self.G_KlipperIfPaused==False:
                                    self.G_KlipperIfPaused = True
                                    #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                                    if self.G_CancelFlag==False:
                                        self.G_PhrozenFluiddRespondInfo("[INFO] AMS1connectederrorpause")
                                        #lancaigang250604:
                                        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:#M0
                                            self.G_PhrozenFluiddRespondInfo("[WARN] Unknown mode, do notpause")
                                        else:
                                            if self.STM32ReprotPauseFlag==0:
                                                self.G_PauseTriggerWhileChangeChannelFlag=True
                                                if self.PG102Flag==True:
                                                    self.G_PhrozenFluiddRespondInfo("[INFO] Purging is in progress. Delay the pause until purging finishes")
                                                    self.PG102DelayPauseFlag=True
                                                    #self.G_PhrozenFluiddRespondInfo("+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                                                    self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                                                else:
                                                    self.G_PhrozenFluiddRespondInfo("[INFO] No purge in progress; can pause immediately")


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
                                                    #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                                                    self.STM32ReprotPauseFlag=1
                                                    # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                                                    self.G_ChangeChannelFirstFilaFlag=True
                                                    self.G_PhrozenFluiddRespondInfo("+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                                                    self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                                            else:
                                                self.G_PauseTriggerWhileChangeChannelFlag=True
                                                self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)


                                #if self.G_KlipperIfPaused==True:
                                else:
                                    self.G_PhrozenFluiddRespondInfo("[INFO] USBerror, currentalready paused")
                                    self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)


            # lancaigang241106: P8feedsuccessful before prompt down, filament runoutdetect and refill
            if self.G_P0M2MAStartPrintFlag==1:
                # self.G_PhrozenFluiddRespondInfo("[WARN] [(dev.python)Device_TimmerRunoutCheck]P8feedsuccessful, filament runoutdetectrefill")
                #if self.G_ToolheadIfHaveFilaFlag==True:
                # self.G_PhrozenFluiddRespondInfo("[INFO] P8feedsuccessful, toolheadhasfilament")
                # if self.G_ToolheadIfHaveFilaFlag==False:
                # self.G_PhrozenFluiddRespondInfo("[INFO] P8finished printing one channel, toolheadnonefilament, automaticrefillnew channel, move move to waiting areawaittimeout")
                    #self.Cmds_CmdP8(None)
                # #1 time detectfilament,, toolhead is no hasfilament
                if self.G_ToolheadIfHaveFilaFlag==True:
                    # toolhead #1 time detectto filament
                    self.G_ToolheadFirstInputFila = True
                # #1 time nomanualfilament,
                if self.G_ToolheadFirstInputFila==False:
                    # filament runout logical;has first time;if not self.G_ToolheadFirstInputFila:")
                    return eventtime + AMS_FILA_RUNOUT_TIMER
                
                if self.G_ToolheadIfHaveFilaFlag==True:
                    #self.G_PhrozenFluiddRespondInfo("[WARN] [(dev.python)Device_TimmerRunoutCheck]if self.G_ToolheadIfHaveFilaFlag==True:")
                    
                    if self.P0M3FilaRunoutSpittingFinished==True:
                        #self.G_PhrozenFluiddRespondInfo("[WARN] [(dev.python)Device_TimmerRunoutCheck]return;if self.P0M3FilaRunoutSpittingFinished==True:")
                        # lancaigang241106: toolheaddetectto has after, if after is hasstate, then down
                        return eventtime + AMS_FILA_RUNOUT_TIMER
                    else:
                        self.P0M3FilaRunoutSpittingFinished = True

                    # lancaigang240123: after nofilamenttimeout after then feed, not allowedautomaticresume
                    if self.AMSRunoutPauseTimeoutFlag==1:
                        # lancaigang240221: toolheadno timeout after, need tomanualresume
                        #self.AMSRunoutPauseTimeoutFlag=0
                        self.G_PhrozenFluiddRespondInfo("[INFO] single-color refill modetimeout, will not auto-resume; manual resume is required")
                        return eventtime + AMS_FILA_RUNOUT_TIMER
                    
                    self.G_PhrozenFluiddRespondInfo("[INFO] single-color refill mode; toolheadfrom noneto has detectedfilamentresume printing")

                    if self.AMSRunoutPauseTimeCount>0:
                        self.G_PhrozenFluiddRespondInfo("AMSRunoutPauseTimeCount=%d" % self.AMSRunoutPauseTimeCount)
                        self.AMSRunoutPauseTimeCount=0
                        self.G_M2MAModeResumeFlag=True
                    # lancaigang241106: count is 0 or timeout is 0
                    else:
                        self.G_PhrozenFluiddRespondInfo("AMSRunoutPauseTimeCount=%d" % self.AMSRunoutPauseTimeCount)
                        if self.G_KlipperIfPaused == True:
                            self.G_PhrozenFluiddRespondInfo('[INFO] already pause, manual resume is required')
                            self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    # lancaigang240108: single-colorrefillfeed, resumefeed normal normal, canresume number data
                    if self.G_M2MAModeResumeFlag==True:
                        #self.Cmds_AMSSerial1Send("FA")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] single-colorrefill; FA; stm32feed new filament")
                        self.G_PhrozenFluiddRespondInfo("[INFO] single-color refill mode; resume printing")
                        #self.G_PhrozenFluiddRespondInfo("[WARN] [(dev.python)Device_TimmerRunoutCheck]if self.G_M2MAModeResumeFlag==True:")
                        #lancaigang250611: 
                        # self.G_PhrozenFluiddRespondInfo("[INFO] External macro command-PG108-heat uppurgewipe nozzle")
                        # command_string = """
                        #     PG108
                        #     """
                        # self.G_PhrozenGCode.run_script_from_command(command_string)
                        # lancaigang250619:checkAMS is no re connectsuccessful
                        self.Cmds_USBConnectErrorCheck()
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
                        
                        # lancaigang251124: ifstm32haspause report error, cannotresume;
                        if self.STM32ReprotPauseFlag == 1:
                            self.G_PhrozenFluiddRespondInfo('[INFO] automatic refillmode, detectedhas filament, but purgeSTM32 up report has pause, cannotauto-resume')
                            # lancaigang240125: number
                            #self.Cmds_PhrozenKlipperResumeCommon()

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
                            #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                            self.STM32ReprotPauseFlag=1
                            # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                            self.G_ChangeChannelFirstFilaFlag=True
                            self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                            #lancaigang241106: 
                            self.G_P0M2MAStartPrintFlag=0
                        else:
                            self.G_PhrozenFluiddRespondInfo("[INFO] toolheadhas filament, resume")
                            self.G_M2MAModeResumeFlag=False
                            # lancaigang240125: number
                            self.Cmds_PhrozenKlipperResumeCommon()
                            self.G_KlipperIfPaused = False
                            # lancaigang240124: stm32 move report, canpause1 time
                            self.STM32ReprotPauseFlag=0

                        #self.G_PhrozenFluiddRespondInfo("[WARN] [(dev.python)Device_TimmerRunoutCheck]self.G_KlipperIfPaused = False")
                        self.G_PhrozenFluiddRespondInfo("+RESUME:2,%d" % self.G_ChangeChannelTimeoutNewChan)

                    #self.G_PhrozenFluiddRespondInfo("[WARN] [(dev.python)Device_TimmerRunoutCheck]return self.G_PhrozenReactor.NOW + AMS_FILA_RUNOUT_TIMER")
                    # lancaigang240109: is eventtime
                    return self.G_PhrozenReactor.NOW + AMS_FILA_RUNOUT_TIMER
                    #return eventtime + AMS_FILA_RUNOUT_TIMER
                


                # lancaigang240108: ifalready pause, cannotpause
                if self.G_KlipperIfPaused==True:
                    self.G_PhrozenFluiddRespondInfo('[WARN] [(dev.python)Device_TimmerRunoutCheck]single-color refill mode; temporary pause, stm32re feed new filament')
                    #lancaigang240224: 
                    if self.AMSRunoutPauseTimeCount==0:
                        # lancaigang240122: pause after, up sending commandstm32new
                        #time.sleep(1)
                        #self.G_ProzenToolhead.dwell(0.5)

                        #self.Cmds_AMSSerial1Send("FA")
                        #lancaigang241106:
                        self.G_PhrozenFluiddRespondInfo('[INFO] single-color refill modetemporary pause after, P8Infila; stm32feed new filament')

                        # lancaigang240511: resumewhen, initialize down serial port, preventAMSserial porterror
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
                            self.G_PhrozenFluiddRespondInfo("[ERROR] Unable to open tty2. Check the USB connection or try rebooting.")
                        # lancaigang250515: re feed
                        self.Cmds_CmdP8Infila()

                    # lancaigang240122: pauseswhen waitnew filament;
                    self.AMSRunoutPauseTimeCount=self.AMSRunoutPauseTimeCount+1
                    self.G_PhrozenFluiddRespondInfo("AMSRunoutPauseTimeCount=%d" % self.AMSRunoutPauseTimeCount)

                    # waitstm32feed, iftoolheaddetectto filament, new already to, canresume
                    if self.G_ToolheadIfHaveFilaFlag==True:
                        self.G_M2MAModeResumeFlag=True
                        self.AMSRunoutPauseTimeCount=0

                        # lancaigang250619:checkAMS is no re connectsuccessful
                        self.Cmds_USBConnectErrorCheck()
                        if self.G_SerialPort1OpenFlag == True:
                            self.Cmds_AMSSerial1Send("AT+EBLOCKCHECK")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 1-AMSstart timingbuffer-full time")
                        if self.G_SerialPort2OpenFlag == True:
                            self.Cmds_AMSSerial2Send("AT+EBLOCKCHECK")
                            self.G_PhrozenFluiddRespondInfo("[INFO] serial port 2-AMSstart timingbuffer-full time")

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


                        # lancaigang251124: ifstm32haspause report error, cannotresume;
                        if self.STM32ReprotPauseFlag == 1:
                            self.G_PhrozenFluiddRespondInfo('[INFO] automatic refillmode, detectedhas filament, but purgeSTM32 up report has pause, cannotauto-resume')
                            # lancaigang240125: number
                            #self.Cmds_PhrozenKlipperResumeCommon()

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
                            #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                            self.STM32ReprotPauseFlag=1
                            # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                            self.G_ChangeChannelFirstFilaFlag=True
                            self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                            #lancaigang241106: 
                            self.G_P0M2MAStartPrintFlag=0
                        else:
                            # self.G_PhrozenFluiddRespondInfo("[INFO] toolheadhasfilament, resume")
                            # self.G_M2MAModeResumeFlag=False
                            # #lancaigang240125: number
                            # self.Cmds_PhrozenKlipperResumeCommon()
                            # self.G_KlipperIfPaused = False
                            # #lancaigang240124: stm32 move report, canpause1 time
                            # self.STM32ReprotPauseFlag=0

                            # lancaigang240125: number
                            self.Cmds_PhrozenKlipperResumeCommon()
                            self.G_PhrozenFluiddRespondInfo('[INFO] MAsingle-color refill mode; toolheaddetectedfilament, move resume printing')
                            self.G_KlipperIfPaused = False
                            # lancaigang240124: stm32 move report, canpause1 time
                            self.STM32ReprotPauseFlag=0

                    if self.AMSRunoutPauseTimeCount>=50:
                        self.AMSRunoutPauseTimeCount=0
                        self.AMSRunoutPauseTimeoutFlag=1
                        #self.Cmds_PhrozenKlipperPauseMAToSTM32(None)
                        self.G_PhrozenFluiddRespondInfo("[INFO] M2MAsingle-color refill mode; stm32feed new filamenttimeout100s")
                        self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        
                    return eventtime + AMS_FILA_RUNOUT_TIMER

                #lancaigang241106:
                if self.G_ToolheadIfHaveFilaFlag==False:
                    # lancaigang240104: not allowedpause
                    if self.G_IfChangeFilaOngoing==False:
                        self.G_PhrozenFluiddRespondInfo('[INFO] M2MAsingle-color refill modetemporary pause, stm32feed new filament')

                        if self.G_KlipperInPausing == False:
                            self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                            #lancaigang250607:
                            # self.G_PhrozenFluiddRespondInfo("[WARN] use pause")
                            #self.G_KlipperQuickPause = True
                            # toolheadno filament, klipperpause
                            self.Cmds_PhrozenKlipperPauseMAToSTM32(None)
                        else:
                            self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")

                        


                        self.AMSRunoutPauseTimeCount = 0
                        self.AMSRunoutPauseTimeoutFlag=0

            self.P0M3FilaRunoutSpittingFinished = False


            



            return eventtime + AMS_FILA_RUNOUT_TIMER
            #return self.G_PhrozenReactor.NOW + AMS_FILA_RUNOUT_TIMER





        # =====M1MCmulti-materialmode
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:#M1
            #self.G_PhrozenFluiddRespondInfo("[WARN] [(dev.python)Device_TimmerRunoutCheck]if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:")
            # self.G_PhrozenFluiddRespondInfo("[WARN] [(dev.python)Device_TimmerRunoutCheck]filament runout logical;multi-materialmode down detect logical ")

            # lancaigang240407: ifmulti-materialmode down detectedtoolheadnofilament, need topause
            # lancaigang240527: is none, can is received P1Cncommand
            # if self.G_ChangeChannelTimeoutOldGcmd is not None:
            #     if self.G_ToolheadIfHaveFilaFlag==False:
            #         if self.G_ToolheadIfHaveFilaFlag==False:
            # self.G_PhrozenFluiddRespondInfo("[WARN] [(cmds.python)Device_TimmerRunoutCheck]MCmulti-materialprinting detectto toolheadnonefilament")

            # lancaigang250607: state; 1-unload filament; 2-feed; 3-; 4-pause
            if self.G_KlipperPrintStatus == 3:
                
                # lancaigang250619:ifdetectedserial porterror, then number
                if self.G_SerialPort1OpenFlag==False:
                    self.G_PhrozenFluiddRespondInfo("[DEBUG] multi-materialprinting in progress; if self.G_KlipperPrintStatus == 3")
                    self.G_ASM1DisconnectErrorCount=self.G_ASM1DisconnectErrorCount+1
                    self.G_PhrozenFluiddRespondInfo('AMSre connect; self.G_ASM1DisconnectErrorCount=%d' % self.G_ASM1DisconnectErrorCount)
                    # lancaigang250619:checkAMS is no re connectsuccessful
                    self.Cmds_USBConnectErrorCheck()

                    if self.G_ASM1DisconnectErrorCount >= 5: #10s
                        try:
                            self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 1")
                            self.G_SerialPort1Obj = serial.Serial(self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3)
                            #Serial port opened successfully
                            if self.G_SerialPort1Obj is not None:
                                if self.G_SerialPort1Obj.is_open:
                                    self.G_SerialPort1OpenFlag = True
                                    self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 1successful")
                                    self.G_ASM1DisconnectErrorCount=0
                                    #self.G_PauseToLCDString=""
                                    # lancaigang231213: openserial port
                                    self.G_SerialPort1Obj.flushInput()  # clean serial write cache
                                    self.G_SerialPort1Obj.flush()
                                    self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 buffers cleared")
                                    self.G_PhrozenFluiddRespondInfo("[INFO] Re-registering serial port 1 callback")
                                    self.G_SerialPort1RecvTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerUart1Recv, self.G_PhrozenReactor.NOW)
                        except:
                            self.G_PhrozenFluiddRespondInfo("[ERROR] Unable to open tty1. Check the USB connection or try rebooting.")
                            self.G_SerialPort1OpenFlag=False
                            self.G_ASM1DisconnectErrorCount=self.G_ASM1DisconnectErrorCount+1
                            self.G_PhrozenFluiddRespondInfo("self.G_ASM1DisconnectErrorCount=%d" % self.G_ASM1DisconnectErrorCount)
                            if self.G_ASM1DisconnectErrorCount >= 20: #40s
                                self.G_ASM1DisconnectErrorCount=0
                                self.G_PhrozenFluiddRespondInfo('[INFO] AMS1connectedfilament runout40s, pause')
                                if self.G_KlipperIfPaused==False:
                                    self.G_KlipperIfPaused = True
                                    #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                                    if self.G_CancelFlag==False:
                                        self.G_PhrozenFluiddRespondInfo("[INFO] AMS1connectederrorpause")
                                        #lancaigang250604:
                                        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:#M0
                                            self.G_PhrozenFluiddRespondInfo("[WARN] Unknown mode, do notpause")
                                        else:
                                            if self.STM32ReprotPauseFlag==0:
                                                self.G_PauseTriggerWhileChangeChannelFlag=True
                                                if self.PG102Flag==True:
                                                    self.G_PhrozenFluiddRespondInfo("[INFO] Purging is in progress. Delay the pause until purging finishes")
                                                    self.PG102DelayPauseFlag=True
                                                    #self.G_PhrozenFluiddRespondInfo("+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                                                    self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                                                else:
                                                    self.G_PhrozenFluiddRespondInfo("[INFO] No purge in progress; can pause immediately")
                                                    if self.PG102Flag==True:
                                                        self.G_PhrozenFluiddRespondInfo("[INFO] Purging is in progress. Delay the pause until purging finishes")
                                                        self.PG102DelayPauseFlag=True
                                                        self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                                                    else:
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
                                            else:
                                                self.G_PauseTriggerWhileChangeChannelFlag=True
                                                self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)


                                #if self.G_KlipperIfPaused==True:
                                else:
                                    self.G_PhrozenFluiddRespondInfo("[INFO] USBerror, currentalready paused")
                                    self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)


            return eventtime + AMS_FILA_RUNOUT_TIMER

        # keep, prevent up,timer stop
        return eventtime + AMS_FILA_RUNOUT_TIMER
    

    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    #100ms
    def Device_TimmerUartRecvHandler(self,AMSNum,SerialRxBytes, SerialRxASCIIStr):
        # lancaigang240603:pausedo not
        if "+PAUSE" in SerialRxASCIIStr:
            self.G_PhrozenFluiddRespondInfo('[(dev.py)Device_TimmerUartRecvHandler]pause; %s' % SerialRxASCIIStr)
        else:
            self.G_PhrozenFluiddRespondInfo("[(dev.py)Device_TimmerUartRecvHandler]%s" % SerialRxASCIIStr)

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

        # self.G_PhrozenFluiddRespondInfo("serial port receive G_PauseToLCDString: %s" % self.G_PauseToLCDString)

        # # // AMSmainboard2firmware-1 1
        # if "V-H1-I1-F?" in SerialRxASCIIStr:
        #     #=====DriveCodeFile.dat
        #     # 1 , 1 , 24053 , 1 , 0
        #     # 2 , 0 , 0 , 0 , 0
        #     # 3 , 0 , 0 , 0 , 0
        #     # 4 , 0 , 0 , 0 , 0
        #     # 5 , 5 , 24046 , 5 , 0
        #     # 6 , 0 , 0 , 0 , 0
        #     # 7 , 7 , 24051 , 7 , 0
        #     # 8 , 0 , 0 , 0 , 0
        #     # 9 , 0 , 0 , 0 , 0
        #     # 10 , 10 , 24054 , 10 , 0
        #     # 11 , 11 , 24047 , 11 , 0
        #     # 12 , 0 , 0 , 0 , 0
        #     # 13 , 0 , 0 , 0 , 0
        #     # 14 , 0 , 0 , 0 , 0
        #     # 15 , 0 , 0 , 0 , 0
        #     # 16 , 0 , 0 , 0 , 0
        # #lancaigang240530: versionwrite todat text file; DriveCodeJson.dat
        #     filename='/home/prz/hdlDat/DriveCodeFile.dat'
        #     self.G_PhrozenFluiddRespondInfo("filename=%s" % filename)
        #     Lo_AllLine=""
        #     #data = [{"DriveCode":16,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":15,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":14,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":13,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":12,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":11,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":10,"DriveImageType":10,"DriveHwVersion":10,"DriveFwVersion":24045,"DriveId":0},{"DriveCode":9,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":8,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":7,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":6,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":5,"DriveImageType":5,"DriveHwVersion":5,"DriveFwVersion":24046,"DriveId":0},{"DriveCode":4,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":3,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":2,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":1,"DriveImageType":1,"DriveHwVersion":1,"DriveFwVersion":24042,"DriveId":0}]
        #     #f = open(filename, 'a')
        # #json.dump(data, f) # to sequence list is byte stream
        #     #f.close()
        #     with open(filename,'r') as file:
        #         #for line in file:
        # # # realine() read internal, "\n"
        #         # self.G_PhrozenFluiddRespondInfo(file.readline().strip())
        #         # #time.sleep(1)
        #         Lo_FileDataList=file.readlines()
        #         for line in Lo_FileDataList:
        #             #split = [i[:-1].split(',') for i in file.readlines()]
        #             #self.G_PhrozenFluiddRespondInfo(type(split))
        #             #self.G_PhrozenFluiddRespondInfo(split[1])
        #             #self.G_PhrozenFluiddRespondInfo(split[2])
        #             #self.G_PhrozenFluiddRespondInfo(split[3])
        #             #line_strip=line.strip()
        #             #self.G_PhrozenFluiddRespondInfo(line)
        #             #self.G_PhrozenFluiddRespondInfo("line.count=%d" % line.count)
        #             split=line.split(',')
        #             #self.G_PhrozenFluiddRespondInfo(type(split))
        #             #self.G_PhrozenFluiddRespondInfo("".join(split))
        #             #self.G_PhrozenFluiddRespondInfo(split[0])
        # split[0]=split[0].strip()# move
        # split[1]=split[1].strip()# file id
        # split[2]=split[2].strip()#firmwareversion
        # split[3]=split[3].strip()#id
        # split[4]=split[4].strip()# is no in
        # #split[4]='0'# is no in, default0
        #             if "SN1" in SerialRxASCIIStr:
        #                 if split[0] == "1":
        #                     self.G_PhrozenFluiddRespondInfo(split[0])
        #                     self.G_PhrozenFluiddRespondInfo(split[1])
        #                     self.G_PhrozenFluiddRespondInfo(split[2])
        #                     self.G_PhrozenFluiddRespondInfo(split[3])
        #                     self.G_PhrozenFluiddRespondInfo(split[4])
        #                     #line=("%d,%d,%d," % (HW_VERSION,,))
        #                     line_modify=split[0]+','+'1'+','+SerialRxASCIIStr[9:14]+','+'1'+','+'1'
        #                     self.G_PhrozenFluiddRespondInfo(line_modify)
        #                     Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
        #                 else:
        #                     Lo_AllLine=Lo_AllLine+line
        #             if "SN2" in SerialRxASCIIStr:
        #                 if split[0] == "2":
        #                     self.G_PhrozenFluiddRespondInfo(split[0])
        #                     self.G_PhrozenFluiddRespondInfo(split[1])
        #                     self.G_PhrozenFluiddRespondInfo(split[2])
        #                     self.G_PhrozenFluiddRespondInfo(split[3])
        #                     self.G_PhrozenFluiddRespondInfo(split[4])
        #                     #line=("%d,%d,%d," % (HW_VERSION,,))
        #                     line_modify=split[0]+','+'1'+','+SerialRxASCIIStr[9:14]+','+'1'+','+'1'
        #                     self.G_PhrozenFluiddRespondInfo(line_modify)
        #                     Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
        #                 else:
        #                     Lo_AllLine=Lo_AllLine+line
        #             if "SN3" in SerialRxASCIIStr:
        #                 if split[0] == "3":
        #                     self.G_PhrozenFluiddRespondInfo(split[0])
        #                     self.G_PhrozenFluiddRespondInfo(split[1])
        #                     self.G_PhrozenFluiddRespondInfo(split[2])
        #                     self.G_PhrozenFluiddRespondInfo(split[3])
        #                     self.G_PhrozenFluiddRespondInfo(split[4])
        #                     #line=("%d,%d,%d," % (HW_VERSION,,))
        #                     line_modify=split[0]+','+'1'+','+SerialRxASCIIStr[9:14]+','+'1'+','+'1'
        #                     self.G_PhrozenFluiddRespondInfo(line_modify)
        #                     Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
        #                 else:
        #                     Lo_AllLine=Lo_AllLine+line
        #             if "SN4" in SerialRxASCIIStr:
        #                 if split[0] == "4":
        #                     self.G_PhrozenFluiddRespondInfo(split[0])
        #                     self.G_PhrozenFluiddRespondInfo(split[1])
        #                     self.G_PhrozenFluiddRespondInfo(split[2])
        #                     self.G_PhrozenFluiddRespondInfo(split[3])
        #                     self.G_PhrozenFluiddRespondInfo(split[4])
        #                     #line=("%d,%d,%d," % (HW_VERSION,,))
        #                     line_modify=split[0]+','+'1'+','+SerialRxASCIIStr[9:14]+','+'1'+','+'1'
        #                     self.G_PhrozenFluiddRespondInfo(line_modify)
        #                     Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
        #                 else:
        #                     Lo_AllLine=Lo_AllLine+line
        #     #self.G_PhrozenFluiddRespondInfo(Lo_AllLine)
        #     with open(filename,"w+") as file_w:
        #         file_w.write(Lo_AllLine)


        # # 16HUBfirmware-7 7
        # if "V-H7-I7-F" in SerialRxASCIIStr:
        # #lancaigang240530: versionwrite todat text file; DriveCodeJson.dat
        #     filename='/home/prz/hdlDat/DriveCodeFile.dat'
        #     self.G_PhrozenFluiddRespondInfo("filename=%s" % filename)
        #     Lo_AllLine=""
        #     #data = [{"DriveCode":16,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":15,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":14,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":13,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":12,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":11,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":10,"DriveImageType":10,"DriveHwVersion":10,"DriveFwVersion":24045,"DriveId":0},{"DriveCode":9,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":8,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":7,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":6,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":5,"DriveImageType":5,"DriveHwVersion":5,"DriveFwVersion":24046,"DriveId":0},{"DriveCode":4,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":3,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":2,"DriveImageType":0,"DriveHwVersion":0,"DriveFwVersion":0,"DriveId":0},{"DriveCode":1,"DriveImageType":1,"DriveHwVersion":1,"DriveFwVersion":24042,"DriveId":0}]
        #     #f = open(filename, 'a')
        # #json.dump(data, f) # to sequence list is byte stream
        #     #f.close()
        #     with open(filename,'r') as file:
        #         #for line in file:
        # # # realine() read internal, "\n"
        #         # self.G_PhrozenFluiddRespondInfo(file.readline().strip())
        #         # #time.sleep(1)
        #         Lo_FileDataList=file.readlines()
        #         for line in Lo_FileDataList:
        #             #split = [i[:-1].split(',') for i in file.readlines()]
        #             #self.G_PhrozenFluiddRespondInfo(type(split))
        #             #self.G_PhrozenFluiddRespondInfo(split[1])
        #             #self.G_PhrozenFluiddRespondInfo(split[2])
        #             #self.G_PhrozenFluiddRespondInfo(split[3])
        #             #line_strip=line.strip()
        #             #self.G_PhrozenFluiddRespondInfo(line)
        #             #self.G_PhrozenFluiddRespondInfo("line.count=%d" % line.count)
        #             split=line.split(',')
        #             #self.G_PhrozenFluiddRespondInfo(type(split))
        #             #self.G_PhrozenFluiddRespondInfo("".join(split))
        #             #self.G_PhrozenFluiddRespondInfo(split[0])
        # split[0]=split[0].strip()# move
        # split[1]=split[1].strip()# file id
        # split[2]=split[2].strip()#firmwareversion
        # split[3]=split[3].strip()#id
        # split[4]=split[4].strip()# is no in
        #             if split[0]== "7":
        #                 self.G_PhrozenFluiddRespondInfo(split[0])
        #                 self.G_PhrozenFluiddRespondInfo(split[1])
        #                 self.G_PhrozenFluiddRespondInfo(split[2])
        #                 self.G_PhrozenFluiddRespondInfo(split[3])
        #                 self.G_PhrozenFluiddRespondInfo(split[4])
        #                 #line=("%d,%d,%d," % (HW_VERSION,,))
        #                 line_modify=split[0]+','+'7'+','+SerialRxASCIIStr[9:14]+','+'7'+','+'1'
        #                 self.G_PhrozenFluiddRespondInfo(line_modify)
        #                 Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
        #             else:
        #                 Lo_AllLine=Lo_AllLine+line
        #     #self.G_PhrozenFluiddRespondInfo(Lo_AllLine)
        #     with open(filename,"w+") as file_w:
        #         file_w.write(Lo_AllLine)


        #lancaigang240326:
        #self.G_PauseToLCDString=SerialRxASCIIStr
        # // ttyUSB0serial port receive : CS00N0M03T04C0
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


        # lancaigang240524: Unknown mode, is M1-MC M2-MA M3mode, do not execute after pause action
        # lancaigang240521: ifnomode, do not executepause
        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:#M1
            self.G_PhrozenFluiddRespondInfo('[WARN] Unknown mode, do not executeserial port number data')
            return
            # lancaigang240524: output callback
            #return self.G_PhrozenReactor.NEVER




        # old AMS
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

        # new AMS
        # // lancaigang231202:+PAUSE:1,oldchannel,newchannel;1-new AMSdo not
        # // lancaigang231202:+PAUSE:2,oldchannel,newchannel;2-pauseACK
        # // lancaigang231204:+PAUSE:3,oldchannel,newchannel;3-
        # // lancaigang231205:+PAUSE:4,oldchannel,newchannel;4-feedtimeout, pause(1、feed timeout60s; 2、feed)
        # // lancaigang231205:+PAUSE:5,oldchannel,newchannel;5-
        # // lancaigang231205:+PAUSE:6,oldchannel,newchannel;6-entryto buffertimeout20s, pause
        # // lancaigang231205:+PAUSE:7,oldchannel,newchannel;7-bufferfullstatetimeout60s, pause(report error because : errorfull or toolheadfull or full)
        # // lancaigang231205:+PAUSE:8,oldchannel,newchannel;8-toolhead cutter or device error, 6stimeout; pause(report error because : old channel use no unload filament or no unload filament or toolhead cutter error)
        # // lancaigang231205:+PAUSE:9,oldchannel,newchannel;9-
        # // lancaigang231202:+PAUSE:a,oldchannel,newchannel;a-
        # // lancaigang231202:+PAUSE:b,oldchannel,newchannel;b-single-colorfilament runoutdetect; detectto filament3spause
        # // lancaigang231202:+PAUSE:c,oldchannel,newchannel;c-purge toolhead head; timeout20s
        # // lancaigang231202:+PAUSE:d,oldchannel,newchannel;d-purge AMSno, filamenthas; timeout20s
        # // lancaigang231202:+PAUSE:e,oldchannel,newchannel;e- move when, AMS in state down, AMS45, pausenot allowed
        # // lancaigang231202:+PAUSE:f,oldchannel,newchannel;f- move when, current AMS in state down, AMS45, pausenot allowed, and stop AMSFunction
        # // lancaigang231202:+PAUSE:g,oldchannel,newchannel;g-AMSmulti-materialUSBerror, timeout10s, then report pause
        # // lancaigang231202:+PAUSE:h,oldchannel,newchannel;h-
        # // lancaigang231202:+PAUSE:i,oldchannel,newchannel;i-
        # // lancaigang231202:+PAUSE:j,oldchannel,newchannel;j-
        # // lancaigang231202:+PAUSE:10,oldchannel,newchannel;10- disable or fluidd network move pause

        # MSG info
         # // lancaigang250516:+MSG:1,0/1,oldchannel,newchannel;0-purge 1-purge


        #lancaigang241128: 
        # lancaigang250712:if is cancel, AMSpausecommand
        if self.G_CancelFlag==True:
            self.G_PhrozenFluiddRespondInfo('[INFO] Print has been canceled, pausecommand')
            return
    



        if "+PAUSE:1" in SerialRxASCIIStr:
            # lancaigang240106: single-colorrefillifalready pause, not allowedthen pause
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[WARN] single-color refill modeMAmode, pauseif self.G_KlipperIfPaused == True:')
                    return
            # lancaigang240413: single-colormode, ifalready pause, not allowedthen pause
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] single-colorM3mode, has AMSmulti-material, pause:')
                    return
            self.G_PhrozenFluiddRespondInfo('[INFO] feed use')
            # lancaigang240103: if is disable move pause, logical stm32 move report
            if self.G_ToolheadIfHaveFilaFlag == True:
                    if self.G_IfToolheadHaveFilaInitiativePauseFlag==True:
                        self.G_PhrozenFluiddRespondInfo('[INFO] disable move pause, logical stm32 move up report')
                        return
            # lancaigang240113: if is manualcommand, stm32pausereport
            if self.ManualCmdFlag==True:
                # lancaigang240611: manualcommandreportserial port disable
                self.G_PhrozenFluiddRespondInfo(SerialRxASCIIStr)
                #self.ManualCmdFlag=False
                self.G_PhrozenFluiddRespondInfo('[DEBUG] move Test command, logical stm32 move pause up report')
                return
            
            # lancaigang240325:During resumeneed todetect is no haspausereport
            self.G_ResumeProcessCheckPauseStatus=True
            self.G_PauseToLCDString=SerialRxASCIIStr
            #if self.G_IfChangeFilaOngoing== False:
            # lancaigang240124: stm32reportpausepause1 time
            if self.STM32ReprotPauseFlag==0:
                self.G_PauseTriggerWhileChangeChannelFlag=True
                if self.PG102Flag==True:
                    # first feed, prevententry device stateerror
                    self.Cmds_AMSSerial1Send("E%d" % self.G_ChangeChannelTimeoutNewChan)
                    self.G_PhrozenFluiddRespondInfo("Sending command: E%d" % self.G_ChangeChannelTimeoutNewChan)

                    self.G_PhrozenFluiddRespondInfo("[INFO] Purging is in progress. Delay the pause until purging finishes")
                    self.PG102DelayPauseFlag=True
                    self.G_PauseToLCDString="+PAUSE:1,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                else:
                    # special refillstate
                    #self.Cmds_AMSSerial1Send("H%d" % self.G_ChangeChannelTimeoutNewChan)
                    #self.G_PhrozenFluiddRespondInfo("[(cmds.python)]Sending command: H%d" % self.G_ChangeChannelTimeoutNewChan)

                    # lancaigang231207:, iffeed use, need tofrom toolhead up filament tube get output, cannotfilament
                    self.G_IfInFilaBlockFlag=True
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

                    self.STM32ReprotPauseFlag=1
                    # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                    self.G_ChangeChannelFirstFilaFlag=True
                    #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)

                    #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                    if "+PAUSE:1,1" in SerialRxASCIIStr:
                        self.G_PhrozenFluiddRespondInfo("[INFO] 1")
                        self.G_PhrozenFluiddRespondInfo("[WARN] +PAUSE:1,1")
                        self.G_PauseToLCDString="+PAUSE:1,1"
                        self.G_Pause1Channel=1
                    elif "+PAUSE:1,2" in SerialRxASCIIStr:
                        self.G_PhrozenFluiddRespondInfo("[INFO] 2")
                        self.G_PhrozenFluiddRespondInfo("[WARN] +PAUSE:1,2")
                        self.G_PauseToLCDString="+PAUSE:1,2"
                        self.G_Pause1Channel=2
                    elif "+PAUSE:1,3" in SerialRxASCIIStr:
                        self.G_PhrozenFluiddRespondInfo("[INFO] 3")
                        self.G_PhrozenFluiddRespondInfo("[WARN] +PAUSE:1,3")
                        self.G_PauseToLCDString="+PAUSE:1,3"
                        self.G_Pause1Channel=3
                    elif "+PAUSE:1,4" in SerialRxASCIIStr:
                        self.G_PhrozenFluiddRespondInfo("[INFO] 4")
                        self.G_PhrozenFluiddRespondInfo("[WARN] +PAUSE:1,4")
                        self.G_PauseToLCDString="+PAUSE:1,3"
                        self.G_Pause1Channel=4
                    elif "+PAUSE:1,5" in SerialRxASCIIStr:
                        self.G_PhrozenFluiddRespondInfo("[INFO] 5")
                        self.G_PhrozenFluiddRespondInfo("[WARN] +PAUSE:1,5")
                        self.G_PauseToLCDString="+PAUSE:1,5"
                        self.G_Pause1Channel=5
                    elif "+PAUSE:1,6" in SerialRxASCIIStr:
                        self.G_PhrozenFluiddRespondInfo("[INFO] 6")
                        self.G_PhrozenFluiddRespondInfo("[WARN] +PAUSE:1,6")
                        self.G_PauseToLCDString="+PAUSE:1,6"
                        self.G_Pause1Channel=6
                    elif "+PAUSE:1,7" in SerialRxASCIIStr:
                        self.G_PhrozenFluiddRespondInfo("[INFO] 7")
                        self.G_PhrozenFluiddRespondInfo("[WARN] +PAUSE:1,7")
                        self.G_PauseToLCDString="+PAUSE:1,7"
                        self.G_Pause1Channel=7
                    elif "+PAUSE:1,8" in SerialRxASCIIStr:
                        self.G_PhrozenFluiddRespondInfo("[INFO] 8")
                        self.G_PhrozenFluiddRespondInfo("[WARN] +PAUSE:1,8")
                        self.G_PauseToLCDString="+PAUSE:1,8"
                        self.G_Pause1Channel=8
                    elif "+PAUSE:1,9" in SerialRxASCIIStr:
                        self.G_PhrozenFluiddRespondInfo("[INFO] 9")
                        self.G_PhrozenFluiddRespondInfo("[WARN] +PAUSE:1,9")
                        self.G_PauseToLCDString="+PAUSE:1,9"
                        self.G_Pause1Channel=9
                    elif "+PAUSE:1,10" in SerialRxASCIIStr:
                        self.G_PhrozenFluiddRespondInfo("[INFO] 10")
                        self.G_PhrozenFluiddRespondInfo("[WARN] +PAUSE:1,10")
                        self.G_PauseToLCDString="+PAUSE:1,10"
                        self.G_Pause1Channel=10
                    elif "+PAUSE:1,11" in SerialRxASCIIStr:
                        self.G_PhrozenFluiddRespondInfo("[INFO] 11")
                        self.G_PhrozenFluiddRespondInfo("[WARN] +PAUSE:1,11")
                        self.G_PauseToLCDString="+PAUSE:1,11"
                        self.G_Pause1Channel=11
                    elif "+PAUSE:1,12" in SerialRxASCIIStr:
                        self.G_PhrozenFluiddRespondInfo("[INFO] 12")
                        self.G_PhrozenFluiddRespondInfo("[WARN] +PAUSE:1,12")
                        self.G_PauseToLCDString="+PAUSE:1,12"
                        self.G_Pause1Channel=12
                    elif "+PAUSE:1,13" in SerialRxASCIIStr:
                        self.G_PhrozenFluiddRespondInfo("[INFO] 13")
                        self.G_PhrozenFluiddRespondInfo("[WARN] +PAUSE:1,13")
                        self.G_PauseToLCDString="+PAUSE:1,13"
                        self.G_Pause1Channel=13
                    elif "+PAUSE:1,14" in SerialRxASCIIStr:
                        self.G_PhrozenFluiddRespondInfo("[INFO] 14")
                        self.G_PhrozenFluiddRespondInfo("[WARN] +PAUSE:1,14")
                        self.G_PauseToLCDString="+PAUSE:1,14"
                        self.G_Pause1Channel=14
                    elif "+PAUSE:1,15" in SerialRxASCIIStr:
                        self.G_PhrozenFluiddRespondInfo("[INFO] 15")
                        self.G_PhrozenFluiddRespondInfo("[WARN] +PAUSE:1,15")
                        self.G_PauseToLCDString="+PAUSE:1,15"
                        self.G_Pause1Channel=15
                    elif "+PAUSE:1,16" in SerialRxASCIIStr:
                        self.G_PhrozenFluiddRespondInfo("[INFO] 16")
                        self.G_PhrozenFluiddRespondInfo("[WARN] +PAUSE:1,16")
                        self.G_PauseToLCDString="+PAUSE:1,16"
                        self.G_Pause1Channel=16
                    else:
                        self.G_PhrozenFluiddRespondInfo("self.G_ChangeChannelTimeoutNewChan=%d" % self.G_ChangeChannelTimeoutNewChan)
                        self.G_PhrozenFluiddRespondInfo("+PAUSE:1,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        self.G_PauseToLCDString="+PAUSE:1,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

            else:
                self.G_PauseTriggerWhileChangeChannelFlag=True
                self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause')
                # lancaigang240325: pause, to reportserial port disable
                #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                self.G_PauseToLCDString=SerialRxASCIIStr
                self.G_PhrozenFluiddRespondInfo("+PAUSE:1,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))

            return


        if "+PAUSE:2" in SerialRxASCIIStr:
            self.G_PhrozenFluiddRespondInfo("[INFO] pauseACK")
            #self.G_PhrozenFluiddRespondInfo("+PAUSE:2,%d" % self.G_ChangeChannelTimeoutNewChan)

            return
        

        if "+PAUSE:3" in SerialRxASCIIStr:
            # lancaigang240106: single-colorrefillifalready pause, not allowedthen pause
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[WARN] single-color refill modeMAmode, pauseif self.G_KlipperIfPaused == True:')
                    return
            # lancaigang240413: single-colormode, ifalready pause, not allowedthen pause
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] single-colorM3mode, has AMSmulti-material, pause:')
                    return

            self.G_PhrozenFluiddRespondInfo('[INFO] new printing refilltimeout10s, pause')

            # lancaigang240103: if is disable move pause, logical stm32 move report
            if self.G_ToolheadIfHaveFilaFlag == True:
                    if self.G_IfToolheadHaveFilaInitiativePauseFlag==True:
                        self.G_PhrozenFluiddRespondInfo('[INFO] disable move pause, logical stm32 move up report')
                        return
            # lancaigang240113: if is manualcommand, stm32pausereport
            if self.ManualCmdFlag==True or self.G_CutCheckTest==True:
                # lancaigang240611: manualcommandreportserial port disable
                self.G_PhrozenFluiddRespondInfo(SerialRxASCIIStr)
                #self.ManualCmdFlag=False
                self.G_PhrozenFluiddRespondInfo('[DEBUG] move Test command, logical stm32 move pause up report')
                return
            

            
            # #if self.G_IfChangeFilaOngoing== False:
            # #lancaigang240124: stm32reportpausepause1 time
            # if self.STM32ReprotPauseFlag==0:
            # self.G_PhrozenFluiddRespondInfo("[WARN] stm32 move pausereport, pause1 time ")
            #     self.STM32ReprotPauseFlag=1
            #     self.G_KlipperIfPaused = True
            #     self.G_ChangeChannelFirstFilaFlag=True
            #     #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
            #     self.G_PhrozenFluiddRespondInfo("+PAUSE:3,%d" % self.G_ChangeChannelTimeoutNewChan)
            # #lancaigang231209: timer logical, will error, after to use thread logical interrupt business logic
            #     self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
            # else:
            # self.G_PhrozenFluiddRespondInfo("[WARN] stm32 move pausereport, pause")
            # lancaigang240325:During resumeneed todetect is no haspausereport
            self.G_ResumeProcessCheckPauseStatus=True
            self.G_PauseToLCDString=SerialRxASCIIStr
            # lancaigang240124: stm32reportpausepause1 time
            if self.STM32ReprotPauseFlag==0:
                self.G_PauseTriggerWhileChangeChannelFlag=True
                if self.PG102Flag==True:
                    # first feed, prevententry device stateerror
                    # lancaigang240323:, temporary disable disable
                    #self.Cmds_AMSSerial1Send("E%d" % self.G_ChangeChannelTimeoutNewChan)
                    #self.G_PhrozenFluiddRespondInfo("[(cmds.python)]Sending command: E%d" % self.G_ChangeChannelTimeoutNewChan)

                    self.G_PhrozenFluiddRespondInfo("[INFO] Purging is in progress. Delay the pause until purging finishes")
                    self.PG102DelayPauseFlag=True
                    self.G_PauseToLCDString="+PAUSE:3,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

                else:
                    #lancaigang250702: 
                    if self.G_KlipperInPausing == False:
                        self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                        # special refillstate
                        #self.Cmds_AMSSerial1Send("H%d" % self.G_ChangeChannelTimeoutNewChan)
                        #self.G_PhrozenFluiddRespondInfo("[(cmds.python)]Sending command: H%d" % self.G_ChangeChannelTimeoutNewChan)

                        

                        # lancaigang231209: timer logical, will error, after to use thread logical interrupt business logic
                        self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause1 time')
                        self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                        self.G_KlipperQuickPause = True
                        self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                        self.G_KlipperIfPaused = True
                        #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)

                        self.STM32ReprotPauseFlag=1
                        # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                        self.G_ChangeChannelFirstFilaFlag=True
                        #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)

                        self.G_PhrozenFluiddRespondInfo("+PAUSE:3,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        self.G_PauseToLCDString="+PAUSE:3,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")

            else:
                self.G_PauseTriggerWhileChangeChannelFlag=True
                self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause')
                # lancaigang240325: pause, to reportserial port disable
                #self.G_PhrozenFluiddRespondInfo("+PAUSE:3,%d" % self.G_ChangeChannelTimeoutNewChan)
                #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                self.G_PhrozenFluiddRespondInfo("+PAUSE:3,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                self.G_PauseToLCDString="+PAUSE:3,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)




            return
        

        if "+PAUSE:5" in SerialRxASCIIStr:
            # lancaigang240106: single-colorrefillifalready pause, not allowedthen pause
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[WARN] single-color refill modeMAmode; pauseif self.G_KlipperIfPaused == True:')
                    return
            # lancaigang240413: single-colormode, ifalready pause, not allowedthen pause
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] single-colorM3mode, has AMSmulti-material, pause:')
                    return
            self.G_PhrozenFluiddRespondInfo('[INFO] new printing refilltimeout10s, pause')
            
            # lancaigang240103: if is disable move pause, logical stm32 move report
            if self.G_ToolheadIfHaveFilaFlag == True:
                    if self.G_IfToolheadHaveFilaInitiativePauseFlag==True:
                        self.G_PhrozenFluiddRespondInfo('[INFO] disable move pause, logical stm32 move up report')
                        return
            # lancaigang240113: if is manualcommand, stm32pausereport
            if self.ManualCmdFlag==True or self.G_CutCheckTest==True:
                # lancaigang240611: manualcommandreportserial port disable
                self.G_PhrozenFluiddRespondInfo(SerialRxASCIIStr)
                #self.ManualCmdFlag=False
                self.G_PhrozenFluiddRespondInfo('[DEBUG] move Test command, logical stm32 move pause up report')
                return
            
            # #if self.G_IfChangeFilaOngoing== False:
            # #lancaigang240124: stm32reportpausepause1 time
            # if self.STM32ReprotPauseFlag==0:
            # self.G_PhrozenFluiddRespondInfo("[WARN] stm32 move pausereport, pause1 time ")
            #     self.STM32ReprotPauseFlag=1
            #     self.G_KlipperIfPaused = True
            #     self.G_ChangeChannelFirstFilaFlag=True
            #     #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
            #     self.G_PhrozenFluiddRespondInfo("+PAUSE:5,%d" % self.G_ChangeChannelTimeoutNewChan)
            # #lancaigang231209: timer logical, will error, after to use thread logical interrupt business logic
            #     self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
            # else:
            # self.G_PhrozenFluiddRespondInfo("[WARN] stm32 move pausereport, pause")
            # lancaigang240325:During resumeneed todetect is no haspausereport
            self.G_ResumeProcessCheckPauseStatus=True
            self.G_PauseToLCDString=SerialRxASCIIStr
            # lancaigang240124: stm32reportpausepause1 time
            if self.STM32ReprotPauseFlag==0:
                self.G_PauseTriggerWhileChangeChannelFlag=True
                if self.PG102Flag==True:
                    # first feed, prevententry device stateerror
                    # lancaigang240323:, temporary disable disable
                    #self.Cmds_AMSSerial1Send("E%d" % self.G_ChangeChannelTimeoutNewChan)
                    #self.G_PhrozenFluiddRespondInfo("[(cmds.python)]Sending command: E%d" % self.G_ChangeChannelTimeoutNewChan)

                    self.G_PhrozenFluiddRespondInfo("[INFO] Purging is in progress. Delay the pause until purging finishes")
                    self.PG102DelayPauseFlag=True
                    self.G_PauseToLCDString="+PAUSE:5,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

                else:
                    #lancaigang250702: 
                    if self.G_KlipperInPausing == False:
                        self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                        # special refillstate
                        #self.Cmds_AMSSerial1Send("H%d" % self.G_ChangeChannelTimeoutNewChan)
                        #self.G_PhrozenFluiddRespondInfo("[(cmds.python)]Sending command: H%d" % self.G_ChangeChannelTimeoutNewChan)

                        # lancaigang231209: timer logical, will error, after to use thread logical interrupt business logic
                        self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause1 time')
                        self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                        self.G_KlipperQuickPause = True
                        self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                        self.G_KlipperIfPaused = True
                        #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)

                        self.STM32ReprotPauseFlag=1
                        # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                        self.G_ChangeChannelFirstFilaFlag=True
                        #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)

                        self.G_PhrozenFluiddRespondInfo("+PAUSE:5,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        self.G_PauseToLCDString="+PAUSE:5,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
            
                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")
    
            else:
                self.G_PauseTriggerWhileChangeChannelFlag=True
                self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause')
                # lancaigang240325: pause, to reportserial port disable
                #self.G_PhrozenFluiddRespondInfo("+PAUSE:5,%d" % self.G_ChangeChannelTimeoutNewChan)
                #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                self.G_PhrozenFluiddRespondInfo("+PAUSE:5,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                self.G_PauseToLCDString="+PAUSE:5,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

            return
        

        if "+PAUSE:4" in SerialRxASCIIStr:
            # lancaigang240106: single-colorrefillifalready pause, not allowedthen pause
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[WARN] single-color refill modeMAmode; pauseif self.G_KlipperIfPaused == True:')
                    return
            # lancaigang240413: single-colormode, ifalready pause, not allowedthen pause
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] single-colorM3mode, has AMSmulti-material, pause:')
                    return
            self.G_PhrozenFluiddRespondInfo('[INFO] new feedtimeout50s, pause')
            
            # lancaigang240103: if is disable move pause, logical stm32 move report
            if self.G_ToolheadIfHaveFilaFlag == True:
                    if self.G_IfToolheadHaveFilaInitiativePauseFlag==True:
                        self.G_PhrozenFluiddRespondInfo('[INFO] disable move pause, logical stm32 move up report')
                        return
            # lancaigang240113: if is manualcommand, stm32pausereport
            if self.ManualCmdFlag==True or self.G_CutCheckTest==True:
                # lancaigang240611: manualcommandreportserial port disable
                self.G_PhrozenFluiddRespondInfo(SerialRxASCIIStr)
                #self.ManualCmdFlag=False
                self.G_PhrozenFluiddRespondInfo('[DEBUG] move Test command, logical stm32 move pause up report')
                return
            
            # #if self.G_IfChangeFilaOngoing== False:
            # #lancaigang240124: stm32reportpausepause1 time
            # if self.STM32ReprotPauseFlag==0:
            # self.G_PhrozenFluiddRespondInfo("[WARN] stm32 move pausereport, pause1 time ")
            #     self.STM32ReprotPauseFlag=1
            #     self.G_KlipperIfPaused = True
            #     self.G_ChangeChannelFirstFilaFlag=True
            #     #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
            #     self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d" % self.G_ChangeChannelTimeoutNewChan)
            # #lancaigang231209: timer logical, will error, after to use thread logical interrupt business logic
            #     self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
            # else:
            # self.G_PhrozenFluiddRespondInfo("[WARN] stm32 move pausereport, pause")
            # lancaigang240325:During resumeneed todetect is no haspausereport
            self.G_ResumeProcessCheckPauseStatus=True
            self.G_PauseToLCDString=SerialRxASCIIStr
            # lancaigang240124: stm32reportpausepause1 time
            if self.STM32ReprotPauseFlag==0:
                self.G_PauseTriggerWhileChangeChannelFlag=True
                if self.PG102Flag==True:
                    # lancaigang240323:, temporary disable disable
                    #self.Cmds_AMSSerial1Send("E%d" % self.G_ChangeChannelTimeoutNewChan)
                    #self.G_PhrozenFluiddRespondInfo("[(cmds.python)]Sending command: E%d" % self.G_ChangeChannelTimeoutNewChan)

                    self.G_PhrozenFluiddRespondInfo("[INFO] Purging is in progress. Delay the pause until purging finishes")
                    self.PG102DelayPauseFlag=True
                    self.G_PauseToLCDString="+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

                else:
                    #lancaigang250702: 
                    if self.G_KlipperInPausing == False:
                        self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                        # special refillstate
                        #self.Cmds_AMSSerial1Send("H%d" % self.G_ChangeChannelTimeoutNewChan)
                        #self.G_PhrozenFluiddRespondInfo("[(cmds.python)]Sending command: H%d" % self.G_ChangeChannelTimeoutNewChan)

                        #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)

                        # lancaigang231209: timer logical, will error, after to use thread logical interrupt business logic
                        self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause1 time')
                        self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                        self.G_KlipperQuickPause = True
                        self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                        self.G_KlipperIfPaused = True
                        #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                        self.STM32ReprotPauseFlag=1
                        # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                        self.G_ChangeChannelFirstFilaFlag=True

                        self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        self.G_PauseToLCDString="+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
            
                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")
    
            else:
                self.G_PauseTriggerWhileChangeChannelFlag=True
                self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause')
                # lancaigang240325: pause, to reportserial port disable
                #self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d" % self.G_ChangeChannelTimeoutNewChan)
                #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                self.G_PauseToLCDString="+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

            return
        

        if "+PAUSE:6" in SerialRxASCIIStr:
            # lancaigang240106: single-colorrefillifalready pause, not allowedthen pause
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[WARN] single-color refill modeMAmode, pauseif self.G_KlipperIfPaused == True:')
                    return
            # lancaigang240413: single-colormode, ifalready pause, not allowedthen pause
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] single-colorM3mode, has AMSmulti-material, pause:')
                    return
            self.G_PhrozenFluiddRespondInfo('[INFO] entryto the park positiontimeout10s, pause')
            
            # lancaigang240103: if is disable move pause, logical stm32 move report
            if self.G_ToolheadIfHaveFilaFlag == True:
                    if self.G_IfToolheadHaveFilaInitiativePauseFlag==True:
                        self.G_PhrozenFluiddRespondInfo('[INFO] disable move pause, logical stm32 move up report')
                        return
            # lancaigang240113: if is manualcommand, stm32pausereport
            if self.ManualCmdFlag==True or self.G_CutCheckTest==True:
                # lancaigang240611: manualcommandreportserial port disable
                self.G_PhrozenFluiddRespondInfo(SerialRxASCIIStr)
                #self.ManualCmdFlag=False
                self.G_PhrozenFluiddRespondInfo('[DEBUG] move Test command, logical stm32 move pause up report')
                return
            
            # #if self.G_IfChangeFilaOngoing== False:
            # #lancaigang240124: stm32reportpausepause1 time
            # if self.STM32ReprotPauseFlag==0:
            # self.G_PhrozenFluiddRespondInfo("[WARN] stm32 move pausereport, pause1 time ")
            #     self.STM32ReprotPauseFlag=1
            #     self.G_KlipperIfPaused = True
            #     self.G_ChangeChannelFirstFilaFlag=True
            #     #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
            #     self.G_PhrozenFluiddRespondInfo("+PAUSE:6,%d" % self.G_ChangeChannelTimeoutNewChan)
            # #lancaigang231209: timer logical, will error, after to use thread logical interrupt business logic
            #     self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
            # else:
            # self.G_PhrozenFluiddRespondInfo("[WARN] stm32 move pausereport, pause")
            # lancaigang240325:During resumeneed todetect is no haspausereport
            self.G_ResumeProcessCheckPauseStatus=True
            self.G_PauseToLCDString=SerialRxASCIIStr
            # lancaigang240124: stm32reportpausepause1 time
            if self.STM32ReprotPauseFlag==0:
                self.G_PauseTriggerWhileChangeChannelFlag=True
                if self.PG102Flag==True:
                    # first feed, prevententry device stateerror
                    # lancaigang240323:, temporary disable disable
                    #self.Cmds_AMSSerial1Send("E%d" % self.G_ChangeChannelTimeoutNewChan)
                    #self.G_PhrozenFluiddRespondInfo("[(cmds.python)]Sending command: E%d" % self.G_ChangeChannelTimeoutNewChan)

                    self.G_PhrozenFluiddRespondInfo("[INFO] Purging is in progress. Delay the pause until purging finishes")
                    self.PG102DelayPauseFlag=True
                    self.G_PauseToLCDString="+PAUSE:6,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

                else:
                    #lancaigang250702: 
                    if self.G_KlipperInPausing == False:
                        self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                        # special refillstate
                        #self.Cmds_AMSSerial1Send("H%d" % self.G_ChangeChannelTimeoutNewChan)
                        #self.G_PhrozenFluiddRespondInfo("[(cmds.python)]Sending command: H%d" % self.G_ChangeChannelTimeoutNewChan)

                        # lancaigang231209: timer logical, will error, after to use thread logical interrupt business logic
                        self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause1 time')
                        self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                        self.G_KlipperQuickPause = True
                        self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                        self.G_KlipperIfPaused = True
                        #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                        
                        self.STM32ReprotPauseFlag=1
                        # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                        self.G_ChangeChannelFirstFilaFlag=True
                        #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)

                        self.G_PhrozenFluiddRespondInfo("+PAUSE:6,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        self.G_PauseToLCDString="+PAUSE:6,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
            
                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")
    
            else:
                self.G_PauseTriggerWhileChangeChannelFlag=True
                self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause')
                # lancaigang240325: pause, to reportserial port disable
                #self.G_PhrozenFluiddRespondInfo("+PAUSE:6,%d" % self.G_ChangeChannelTimeoutNewChan)
                #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                self.G_PhrozenFluiddRespondInfo("+PAUSE:6,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                self.G_PauseToLCDString="+PAUSE:6,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

            return
        

        if "+PAUSE:7" in SerialRxASCIIStr:
            # lancaigang240106: single-colorrefillifalready pause, not allowedthen pause
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[WARN] single-color refill modeMAmode, pauseif self.G_KlipperIfPaused == True:')
                    return
            # lancaigang240413: single-colormode, ifalready pause, not allowedthen pause
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] single-colorM3mode, has AMSmulti-material, pause:')
                    return
                
            self.G_PhrozenFluiddRespondInfo('[INFO] bufferfullstatetimeout30s, pause')
            
            #lancaigang231215:
            self.G_STM32PauseCount+=1
            if self.G_STM32PauseCount==5:
                self.G_PhrozenFluiddRespondInfo("if self.G_STM32PauseCount==5;G_STM32PauseCount=%d" % self.G_STM32PauseCount)
                self.G_STM32PauseCount=0
            else:
                self.G_PhrozenFluiddRespondInfo("else;G_STM32PauseCount=%d" % self.G_STM32PauseCount)
        
            # lancaigang240103: if is disable move pause, logical stm32 move report
            if self.G_ToolheadIfHaveFilaFlag == True:
                    if self.G_IfToolheadHaveFilaInitiativePauseFlag==True:
                        self.G_PhrozenFluiddRespondInfo('[INFO] disable move pause, logical stm32 move up report')
                        # lancaigang240103: resumeafter, sending commandstm32resume up time state machine state
                        # resumestateRS=F,resume up time state
                        # resumestateRS=0,resume up IDLE_STANDBYstate
                        # resumestateRS=X,...
                        # resumestateRS=Y,...
                        # resumestateRS=Z,...
                        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                            self.Cmds_AMSSerial1Send("AT+MARS=F")
                            self.G_PhrozenFluiddRespondInfo('[INFO] pauseMA;AT+MARS=F; STM32resume up time state')

                        if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MC:
                            self.Cmds_AMSSerial1Send("AT+MCRS=F")
                            self.G_PhrozenFluiddRespondInfo('[INFO] pauseMC;AT+MCRS=F; STM32resume up time state')

                        
                        
                        # self.G_ProzenToolhead.dwell(1.0)
                        # self.Cmds_AMSSerial1Send("AT+MARS=F")
                        # self.G_PhrozenFluiddRespondInfo("[INFO] AT+MARS=F")

                        return
            # lancaigang240113: if is manualcommand, stm32pausereport
            if self.ManualCmdFlag==True or self.G_CutCheckTest==True:
                # lancaigang240611: manualcommandreportserial port disable
                self.G_PhrozenFluiddRespondInfo(SerialRxASCIIStr)
                #self.ManualCmdFlag=False
                self.G_PhrozenFluiddRespondInfo('[DEBUG] move Test command, logical stm32 move pause up report')
                return
            
            # #if self.G_IfChangeFilaOngoing== False:
            # #lancaigang240124: stm32reportpausepause1 time
            # if self.STM32ReprotPauseFlag==0:
            # self.G_PhrozenFluiddRespondInfo("[WARN] stm32 move pausereport, pause1 time ")
            #     self.STM32ReprotPauseFlag=1
            #     self.G_KlipperIfPaused = True
            #     self.G_ChangeChannelFirstFilaFlag=True
            #     #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
            #     self.G_PhrozenFluiddRespondInfo("+PAUSE:7,%d" % self.G_ChangeChannelTimeoutNewChan)
            # #lancaigang231209: timer logical, will error, after to use thread logical interrupt business logic
            #     self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
            # else:
            # self.G_PhrozenFluiddRespondInfo("[WARN] stm32 move pausereport, pause")
            # lancaigang240325:During resumeneed todetect is no haspausereport
            self.G_ResumeProcessCheckPauseStatus=True
            self.G_PauseToLCDString=SerialRxASCIIStr
            # lancaigang240124: stm32reportpausepause1 time
            if self.STM32ReprotPauseFlag==0:
                self.G_PauseTriggerWhileChangeChannelFlag=True
                if self.PG102Flag==True:
                    # first feed, prevententry device stateerror
                    #self.Cmds_AMSSerial1Send("E%d" % self.G_ChangeChannelTimeoutNewChan)
                    #self.G_PhrozenFluiddRespondInfo("[(cmds.python)]Sending command: E%d" % self.G_ChangeChannelTimeoutNewChan)

                    self.G_PhrozenFluiddRespondInfo("[INFO] Purging is in progress. Delay the pause until purging finishes")
                    self.PG102DelayPauseFlag=True
                    
                    # lancaigang250725:iftoolheaddetectto filament, is head
                    if self.G_ToolheadIfHaveFilaFlag==True:
                        self.G_PhrozenFluiddRespondInfo('[INFO] purge toolheaddetectedfilament, head')
                        self.G_PauseToLCDString="+PAUSE:c,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                    else:
                        self.G_PhrozenFluiddRespondInfo('[INFO] purge buffererrorfull, toolheaddetectedfilament, class is feedtimeout')
                        self.G_PauseToLCDString="+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

                else:
                    # special refillstate
                    #self.Cmds_AMSSerial1Send("H%d" % self.G_ChangeChannelTimeoutNewChan)
                    #self.G_PhrozenFluiddRespondInfo("[(cmds.python)]Sending command: H%d" % self.G_ChangeChannelTimeoutNewChan)

                    
                    if self.G_KlipperInPausing == False:
                        self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                        # lancaigang231209: timer logical, will error, after to use thread logical interrupt business logic
                        self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause1 time')
                        self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                        self.G_KlipperQuickPause = True
                        self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                        self.G_KlipperIfPaused = True
                        #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)

                        self.STM32ReprotPauseFlag=1
                        # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                        self.G_ChangeChannelFirstFilaFlag=True
                        #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)


                        # lancaigang250725:iftoolheaddetectto filament, is head
                        if self.G_ToolheadIfHaveFilaFlag==True:
                            self.G_PhrozenFluiddRespondInfo('[INFO] printing toolheaddetectedfilament, printing head')
                            self.G_PhrozenFluiddRespondInfo("+PAUSE:7,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                            self.G_PauseToLCDString="+PAUSE:7,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                        else:
                            self.G_PhrozenFluiddRespondInfo('[INFO] buffererrorfull, toolheaddetectedfilament, class is feedtimeout')
                            self.G_PhrozenFluiddRespondInfo("+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                            self.G_PauseToLCDString="+PAUSE:4,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")
            
            else:
                self.G_PauseTriggerWhileChangeChannelFlag=True
                self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause')
                # lancaigang240325: pause, to reportserial port disable
                #self.G_PhrozenFluiddRespondInfo("+PAUSE:7,%d" % self.G_ChangeChannelTimeoutNewChan)
                #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                self.G_PhrozenFluiddRespondInfo("+PAUSE:7,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                self.G_PauseToLCDString="+PAUSE:7,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

            return



        if "+PAUSE:a" in SerialRxASCIIStr:
            # lancaigang240106: single-colorrefillifalready pause, not allowedthen pause
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[WARN] single-color refill modeMAmode; pauseif self.G_KlipperIfPaused == True:')
                    return
            # lancaigang240413: single-colormode, ifalready pause, not allowedthen pause
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] single-colorM3mode, has AMSmulti-material, pause:')
                    return
            self.G_PhrozenFluiddRespondInfo('[INFO] park positionto bufferentrytimeout10s, pause')
            

            # lancaigang240103: if is disable move pause, logical stm32 move report
            if self.G_ToolheadIfHaveFilaFlag == True:
                    if self.G_IfToolheadHaveFilaInitiativePauseFlag==True:
                        self.G_PhrozenFluiddRespondInfo('[INFO] disable move pause, logical stm32 move up report')
                        return
            # lancaigang240113: if is manualcommand, stm32pausereport
            if self.ManualCmdFlag==True or self.G_CutCheckTest==True:
                # lancaigang240611: manualcommandreportserial port disable
                self.G_PhrozenFluiddRespondInfo(SerialRxASCIIStr)
                #self.ManualCmdFlag=False
                self.G_PhrozenFluiddRespondInfo('[DEBUG] move Test command, logical stm32 move pause up report')
                return
            
            # #if self.G_IfChangeFilaOngoing== False:
            # #lancaigang240124: stm32reportpausepause1 time
            # if self.STM32ReprotPauseFlag==0:
            # self.G_PhrozenFluiddRespondInfo("[WARN] stm32 move pausereport, pause1 time ")
            #     self.STM32ReprotPauseFlag=1
            #     self.G_KlipperIfPaused = True
            #     self.G_ChangeChannelFirstFilaFlag=True
            #     #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
            #     self.G_PhrozenFluiddRespondInfo("+PAUSE:a,%d" % self.G_ChangeChannelTimeoutNewChan)
            # #lancaigang231209: timer logical, will error, after to use thread logical interrupt business logic
            #     self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
            # else:
            # self.G_PhrozenFluiddRespondInfo("[WARN] stm32 move pausereport, pause")
            # lancaigang240325:During resumeneed todetect is no haspausereport
            self.G_ResumeProcessCheckPauseStatus=True
            self.G_PauseToLCDString=SerialRxASCIIStr
            # lancaigang240124: stm32reportpausepause1 time
            if self.STM32ReprotPauseFlag==0:
                self.G_PauseTriggerWhileChangeChannelFlag=True
                if self.PG102Flag==True:
                    # first feed, prevententry device stateerror
                    # lancaigang240323:, temporary disable disable
                    #self.Cmds_AMSSerial1Send("E%d" % self.G_ChangeChannelTimeoutNewChan)
                    #self.G_PhrozenFluiddRespondInfo("[(cmds.python)]Sending command: E%d" % self.G_ChangeChannelTimeoutNewChan)

                    self.G_PhrozenFluiddRespondInfo("[INFO] Purging is in progress. Delay the pause until purging finishes")
                    self.PG102DelayPauseFlag=True
                    self.G_PauseToLCDString="+PAUSE:a,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

                else:
                    #lancaigang250702: 
                    if self.G_KlipperInPausing == False:
                        self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                        # special refillstate
                        #self.Cmds_AMSSerial1Send("H%d" % self.G_ChangeChannelTimeoutNewChan)
                        #self.G_PhrozenFluiddRespondInfo("[(cmds.python)]Sending command: H%d" % self.G_ChangeChannelTimeoutNewChan)

                        # lancaigang231209: timer logical, will error, after to use thread logical interrupt business logic
                        self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause1 time')
                        self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                        self.G_KlipperQuickPause = True
                        self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                        self.G_KlipperIfPaused = True
                        #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)

                        self.STM32ReprotPauseFlag=1
                        # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                        self.G_ChangeChannelFirstFilaFlag=True
                        #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)a

                        self.G_PhrozenFluiddRespondInfo("+PAUSE:a,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        self.G_PauseToLCDString="+PAUSE:a,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                
                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")

            else:
                self.G_PauseTriggerWhileChangeChannelFlag=True
                self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause')
                # lancaigang240325: pause, to reportserial port disable
                #self.G_PhrozenFluiddRespondInfo("+PAUSE:a,%d" % self.G_ChangeChannelTimeoutNewChan)
                #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                self.G_PhrozenFluiddRespondInfo("+PAUSE:a,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                self.G_PauseToLCDString="+PAUSE:a,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

            return

        # lancaigang250423: toolhead head detect
        if "+PAUSE:c" in SerialRxASCIIStr:
            # lancaigang240106: single-colorrefillifalready pause, not allowedthen pause
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[WARN] single-color refill modeMAmode; pauseif self.G_KlipperIfPaused == True:')
                    
                    #lancaigang251124: 
                    self.G_PhrozenFluiddRespondInfo('[INFO] single-color refill modeMAmode; toolheaddetectedfilament, but purgestm32pause report error, cannotreturn;')
                    self.STM32ReprotPauseFlag=1
                    # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                    #self.G_ChangeChannelFirstFilaFlag=True
                    #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                    #self.G_PhrozenFluiddRespondInfo("+PAUSE:c,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    self.G_PauseToLCDString="+PAUSE:c,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
        
                    return
            # lancaigang240413: single-colormode, ifalready pause, not allowedthen pause
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] single-colorM3mode, has AMSmulti-material, pause:')
                    return
            self.G_PhrozenFluiddRespondInfo('[INFO] toolhead head, pause')
            

            # lancaigang240103: if is disable move pause, logical stm32 move report
            if self.G_ToolheadIfHaveFilaFlag == True:
                    if self.G_IfToolheadHaveFilaInitiativePauseFlag==True:
                        self.G_PhrozenFluiddRespondInfo('[INFO] disable move pause, logical stm32 move up report')
                        return
            # lancaigang240113: if is manualcommand, stm32pausereport
            if self.ManualCmdFlag==True or self.G_CutCheckTest==True:
                # lancaigang240611: manualcommandreportserial port disable
                self.G_PhrozenFluiddRespondInfo(SerialRxASCIIStr)
                #self.ManualCmdFlag=False
                self.G_PhrozenFluiddRespondInfo('[DEBUG] move Test command, logical stm32 move pause up report')
                return

            self.G_ResumeProcessCheckPauseStatus=True
            self.G_PauseToLCDString=SerialRxASCIIStr
            # lancaigang240124: stm32reportpausepause1 time
            if self.STM32ReprotPauseFlag==0:
                self.G_PauseTriggerWhileChangeChannelFlag=True
                if self.PG102Flag==True:
                    self.G_PhrozenFluiddRespondInfo("[INFO] Purging is in progress. Delay the pause until purging finishes")
                    self.PG102DelayPauseFlag=True
                    self.G_PauseToLCDString="+PAUSE:c,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

                else:
                    #lancaigang250702: 
                    if self.G_KlipperInPausing == False:
                        self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                        #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                        # lancaigang231209: timer logical, will error, after to use thread logical interrupt business logic
                        self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause1 time')
                        self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                        self.G_KlipperQuickPause = True
                        self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                        self.G_KlipperIfPaused = True

                        self.STM32ReprotPauseFlag=1
                        # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                        self.G_ChangeChannelFirstFilaFlag=True
                        #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)a

                        self.G_PhrozenFluiddRespondInfo("+PAUSE:c,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        self.G_PauseToLCDString="+PAUSE:c,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
            
                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")
    
            else:
                self.G_PauseTriggerWhileChangeChannelFlag=True
                self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause')
                # lancaigang240325: pause, to reportserial port disable
                #self.G_PhrozenFluiddRespondInfo("+PAUSE:a,%d" % self.G_ChangeChannelTimeoutNewChan)
                #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                self.G_PhrozenFluiddRespondInfo("+PAUSE:c,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                self.G_PauseToLCDString="+PAUSE:c,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

            return


        # lancaigang250506:purge refillerror, filamenthas
        if "+PAUSE:d" in SerialRxASCIIStr:
            # lancaigang240106: single-colorrefillifalready pause, not allowedthen pause
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[WARN] single-color refill modeMAmode; pauseif self.G_KlipperIfPaused == True:')
                    
                    
                    #lancaigang251124: 
                    self.G_PhrozenFluiddRespondInfo('[INFO] single-color refill modeMAmode; toolheaddetectedfilament, but refillstm32pause report error, cannotreturn;')
                    self.STM32ReprotPauseFlag=1
                    # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                    #self.G_ChangeChannelFirstFilaFlag=True
                    #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                    #self.G_PhrozenFluiddRespondInfo("+PAUSE:c,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    self.G_PauseToLCDString="+PAUSE:d,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
        
                    
                    return
            # lancaigang240413: single-colormode, ifalready pause, not allowedthen pause
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] single-colorM3mode, has AMSmulti-material, pause:')
                    return
            self.G_PhrozenFluiddRespondInfo('[INFO] purgerefillerror, filamenthas, pause')
            

            # lancaigang240103: if is disable move pause, logical stm32 move report
            if self.G_ToolheadIfHaveFilaFlag == True:
                    if self.G_IfToolheadHaveFilaInitiativePauseFlag==True:
                        self.G_PhrozenFluiddRespondInfo('[INFO] disable move pause, logical stm32 move up report')
                        return
            # lancaigang240113: if is manualcommand, stm32pausereport
            if self.ManualCmdFlag==True or self.G_CutCheckTest==True:
                # lancaigang240611: manualcommandreportserial port disable
                self.G_PhrozenFluiddRespondInfo(SerialRxASCIIStr)
                #self.ManualCmdFlag=False
                self.G_PhrozenFluiddRespondInfo('[DEBUG] move Test command, logical stm32 move pause up report')
                return

            self.G_ResumeProcessCheckPauseStatus=True
            self.G_PauseToLCDString=SerialRxASCIIStr
            # lancaigang240124: stm32reportpausepause1 time
            if self.STM32ReprotPauseFlag==0:
                self.G_PauseTriggerWhileChangeChannelFlag=True
                if self.PG102Flag==True:
                    self.G_PhrozenFluiddRespondInfo("[INFO] Purging is in progress. Delay the pause until purging finishes")
                    self.PG102DelayPauseFlag=True
                    self.G_PauseToLCDString="+PAUSE:d,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

                else:
                    #lancaigang250702: 
                    if self.G_KlipperInPausing == False:
                        self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                        #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                        # lancaigang231209: timer logical, will error, after to use thread logical interrupt business logic
                        self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause1 time')
                        self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                        self.G_KlipperQuickPause = True
                        self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                        self.G_KlipperIfPaused = True
                        self.STM32ReprotPauseFlag=1
                        # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                        self.G_ChangeChannelFirstFilaFlag=True
                        #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)a

                        self.G_PhrozenFluiddRespondInfo("+PAUSE:d,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        self.G_PauseToLCDString="+PAUSE:d,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
            
                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")
    
            else:
                self.G_PauseTriggerWhileChangeChannelFlag=True
                self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause')
                # lancaigang240325: pause, to reportserial port disable
                #self.G_PhrozenFluiddRespondInfo("+PAUSE:a,%d" % self.G_ChangeChannelTimeoutNewChan)
                #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                self.G_PhrozenFluiddRespondInfo("+PAUSE:d,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                self.G_PauseToLCDString="+PAUSE:d,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

            return

        # //lancaigang250507:+PAUSE:e,oldchannel,newchannel;e-state down, AMS internal, not allowed
        if "+PAUSE:e" in SerialRxASCIIStr:
            # lancaigang240106: single-colorrefillifalready pause, not allowedthen pause
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[WARN] single-color refill modeMAmode; pauseif self.G_KlipperIfPaused == True:')
                    return
            # lancaigang240413: single-colormode, ifalready pause, not allowedthen pause
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] single-colorM3mode, has AMSmulti-material, pause:')
                    return
            
            # lancaigang250510: in mode, not allowedpauseklipper, but to prompt show serial port disable
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                self.G_PhrozenFluiddRespondInfo('[INFO] in printingmode, not allowedpauseklipper, but to prompt show serial port disable')
                self.G_PhrozenFluiddRespondInfo("+PAUSE:e,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                self.G_PauseToLCDString="+PAUSE:e,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                return

            self.G_PhrozenFluiddRespondInfo('[INFO] state down, AMS internal, not allowedprinting, pause')
            

            # lancaigang240103: if is disable move pause, logical stm32 move report
            if self.G_ToolheadIfHaveFilaFlag == True:
                    if self.G_IfToolheadHaveFilaInitiativePauseFlag==True:
                        self.G_PhrozenFluiddRespondInfo('[INFO] disable move pause, logical stm32 move up report')
                        return
            # lancaigang240113: if is manualcommand, stm32pausereport
            if self.ManualCmdFlag==True or self.G_CutCheckTest==True:
                # lancaigang240611: manualcommandreportserial port disable
                self.G_PhrozenFluiddRespondInfo(SerialRxASCIIStr)
                #self.ManualCmdFlag=False
                self.G_PhrozenFluiddRespondInfo('[DEBUG] move Test command, logical stm32 move pause up report')
                return

            self.G_ResumeProcessCheckPauseStatus=True
            self.G_PauseToLCDString=SerialRxASCIIStr
            # lancaigang240124: stm32reportpausepause1 time
            if self.STM32ReprotPauseFlag==0:
                self.G_PauseTriggerWhileChangeChannelFlag=True
                if self.PG102Flag==True:
                    self.G_PhrozenFluiddRespondInfo("[INFO] Purging is in progress. Delay the pause until purging finishes")
                    self.PG102DelayPauseFlag=True
                    self.G_PauseToLCDString="+PAUSE:e,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

                else:
                    #lancaigang250702: 
                    if self.G_KlipperInPausing == False:
                        self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                        #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                        # lancaigang231209: timer logical, will error, after to use thread logical interrupt business logic
                        self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause1 time')
                        self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                        self.G_KlipperQuickPause = True
                        self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                        self.G_KlipperIfPaused = True
                        self.STM32ReprotPauseFlag=1
                        # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                        self.G_ChangeChannelFirstFilaFlag=True
                        #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)a

                        self.G_PhrozenFluiddRespondInfo("+PAUSE:e,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        self.G_PauseToLCDString="+PAUSE:e,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
            
                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")
    
            else:
                self.G_PauseTriggerWhileChangeChannelFlag=True
                self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause')
                # lancaigang240325: pause, to reportserial port disable
                #self.G_PhrozenFluiddRespondInfo("+PAUSE:e,%d" % self.G_ChangeChannelTimeoutNewChan)
                #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                self.G_PhrozenFluiddRespondInfo("+PAUSE:e,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                self.G_PauseToLCDString="+PAUSE:e,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

            return

        # //lancaigang250507:+PAUSE:f,oldchannel,newchannel;f-state down, not allowed
        if "+PAUSE:f" in SerialRxASCIIStr:
            # lancaigang240106: single-colorrefillifalready pause, not allowedthen pause
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_MA:
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[WARN] single-color refill modeMAmode; pauseif self.G_KlipperIfPaused == True:')
                    return
            # lancaigang240413: single-colormode, ifalready pause, not allowedthen pause
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_FILA_RUNOUT:
                if self.G_KlipperIfPaused == True:
                    self.G_PhrozenFluiddRespondInfo('[INFO] single-colorM3mode, has AMSmulti-material, pause:')
                    return
            self.G_PhrozenFluiddRespondInfo('[INFO] state down, not allowedprinting, pause')
            
            # lancaigang250510: in mode, not allowedpauseklipper, but to prompt show serial port disable
            if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:
                self.G_PhrozenFluiddRespondInfo('[INFO] in printingmode, not allowedpauseklipper, but to prompt show serial port disable')
                self.G_PhrozenFluiddRespondInfo("+PAUSE:f,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                self.G_PauseToLCDString="+PAUSE:f,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                return
            

            # lancaigang240103: if is disable move pause, logical stm32 move report
            if self.G_ToolheadIfHaveFilaFlag == True:
                    if self.G_IfToolheadHaveFilaInitiativePauseFlag==True:
                        self.G_PhrozenFluiddRespondInfo('[INFO] disable move pause, logical stm32 move up report')
                        return
            # lancaigang240113: if is manualcommand, stm32pausereport
            if self.ManualCmdFlag==True or self.G_CutCheckTest==True:
                # lancaigang240611: manualcommandreportserial port disable
                self.G_PhrozenFluiddRespondInfo(SerialRxASCIIStr)
                #self.ManualCmdFlag=False
                self.G_PhrozenFluiddRespondInfo('[DEBUG] move Test command, logical stm32 move pause up report')
                return

            self.G_ResumeProcessCheckPauseStatus=True
            self.G_PauseToLCDString=SerialRxASCIIStr
            # lancaigang240124: stm32reportpausepause1 time
            if self.STM32ReprotPauseFlag==0:
                self.G_PauseTriggerWhileChangeChannelFlag=True
                if self.PG102Flag==True:
                    self.G_PhrozenFluiddRespondInfo("[INFO] Purging is in progress. Delay the pause until purging finishes")
                    self.PG102DelayPauseFlag=True
                    self.G_PauseToLCDString="+PAUSE:f,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

                else:
                    #lancaigang250702: 
                    if self.G_KlipperInPausing == False:
                        self.G_PhrozenFluiddRespondInfo("[INFO] No pause is in progress; a new pause is allowed")
                        #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                        # lancaigang231209: timer logical, will error, after to use thread logical interrupt business logic
                        self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause1 time')
                        self.G_PhrozenFluiddRespondInfo("[INFO] Quick pause enabled")
                        self.G_KlipperQuickPause = True
                        self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                        self.G_KlipperIfPaused = True
                        self.STM32ReprotPauseFlag=1
                        # lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                        self.G_ChangeChannelFirstFilaFlag=True
                        #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)a

                        self.G_PhrozenFluiddRespondInfo("+PAUSE:f,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                        self.G_PauseToLCDString="+PAUSE:f,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
            
                    else:
                        self.G_PhrozenFluiddRespondInfo("[INFO] A pause is already in progress; a new pause is not allowed")
    
            else:
                self.G_PauseTriggerWhileChangeChannelFlag=True
                self.G_PhrozenFluiddRespondInfo('[INFO] stm32 move pause up report, pause')
                # lancaigang240325: pause, to reportserial port disable
                #self.G_PhrozenFluiddRespondInfo("+PAUSE:a,%d" % self.G_ChangeChannelTimeoutNewChan)
                #self.G_PhrozenFluiddRespondInfo(self.G_PauseToLCDString)
                self.G_PhrozenFluiddRespondInfo("+PAUSE:f,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                self.G_PauseToLCDString="+PAUSE:f,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

            return













        if "+FORCEFORWARD:1" in SerialRxASCIIStr:
            self.G_PhrozenFluiddRespondInfo('[INFO] get output toolheadfilament tube, get output')
            return



        # lancaigang231202: ifreceived stm32pause, then pauseklipper




        # // CSid mode before mcstate current mcstate channel
        # // CS00       N0      M09         T09         C5
        # // CS00       N0      M02         T03         C0
        # // CS00       N0      M08         T10         C1
        #    deviceid   mode    pre_state   state       chan
        # normal then table serial port number data
        #CS00N0M03T04C0
        message_obj = re.match(
            AMS_SERIALPORT_RECEIV_PARSE_PATTERN,# normal then table
            SerialRxASCIIStr,
            re.M | re.I,
        )


        # serial port number data error
        if not message_obj:
            return
        

        if int(message_obj.group("mode")) is AMS_MC_MODE:
            self.G_PhrozenFluiddRespondInfo('modemode==AMSmode==%d' % AMS_MC_MODE)
        if int(message_obj.group("mode")) is AMS_MA_MODE:
            self.G_PhrozenFluiddRespondInfo('modemode==refillmode==%d' % AMS_MA_MODE)

        if int(message_obj.group("state")) is MC_STANDBY:
            self.G_PhrozenFluiddRespondInfo('current statestate== machine ==%d' % MC_STANDBY)
        if int(message_obj.group("state")) is MC_PREPARTION:
            self.G_PhrozenFluiddRespondInfo('current statestate====%d' % MC_STANDBY)
        if int(message_obj.group("state")) is MC_CHANGING_P1:
            self.G_PhrozenFluiddRespondInfo('current statestate==1==%d' % MC_CHANGING_P1)
        if int(message_obj.group("state")) is MC_CHANGING_P2:
            self.G_PhrozenFluiddRespondInfo('current statestate==2==%d' % MC_CHANGING_P2)
        if int(message_obj.group("state")) is MC_FORCE_FEED:
            self.G_PhrozenFluiddRespondInfo('current statestate==refill==%d' % MC_FORCE_FEED)
        if int(message_obj.group("state")) is MC_PRINTING:
            self.G_PhrozenFluiddRespondInfo('current statestate==refill==%d' % MC_PRINTING)
        if int(message_obj.group("state")) is MC_ROLLBACK:
            self.G_PhrozenFluiddRespondInfo('current statestate== full unload==%d' % MC_ROLLBACK)
        if int(message_obj.group("state")) is MC_PARKBACK:
            self.G_PhrozenFluiddRespondInfo('current statestate==unloadto park position==%d' % MC_PARKBACK)
        if int(message_obj.group("state")) is MC_PARKALL:
            self.G_PhrozenFluiddRespondInfo('current statestate== full all unloadto park position==%d' % MC_PARKALL)
        if int(message_obj.group("state")) is MC_CLEANING:
            self.G_PhrozenFluiddRespondInfo('current statestate==all filamentclear==%d' % MC_CLEANING)
        if int(message_obj.group("state")) is MC_ERR_TIMEOUT:
            self.G_PhrozenFluiddRespondInfo('current statestate==timeout output error state==%d' % MC_ERR_TIMEOUT)
        if int(message_obj.group("state")) is MC_ERR_RUNOUT:
            self.G_PhrozenFluiddRespondInfo('current statestate==filament runout output error state==%d' % MC_ERR_RUNOUT)
        if int(message_obj.group("state")) is MC_ERR_BLOCKUP:
            self.G_PhrozenFluiddRespondInfo('current statestate== output error state==%d' % MC_ERR_BLOCKUP)
            # raise self.error(" machine output error ")
            #self.Cmds_PhrozenKlipperPause(None)

        self.G_PhrozenFluiddRespondInfo('[INFO] machine chan==%d' % int(message_obj.group("chan")))
        # raise self.error("; channel machine ")

        # lancaigang20231013: logical; re feed
        if int(message_obj.group("mode")) is AMS_MC_MODE:
            # lancaigang20231114: haswhen feed time, first disable disable
            cur_chan = int(message_obj.group("chan")) + 1
            # #lancaigang20231013: 2-->refill
            # if (int(message_obj.group("pre_state")) is MC_CHANGING_P2) and (int(message_obj.group("state")) is MC_FORCE_FEED):
            # #lancaigang20231013: toolhead up nofilament, but bufferfullstate,, re
            #     if not self.G_ToolheadIfHaveFilaFlag:
            #         self.AMSErrorRetryTimes += 1
            #         if self.AMSErrorRetryTimes < 5:
            # #// =====T1~Tncommand; PRZ_T[n] P1 T[n]n:1 ~32(no network, get 1 ~4)manualto channel,(use)
            #             self.Cmds_AMSSerial1Send("T%d" % cur_chan)
            # self.G_PhrozenFluiddRespondInfo("when toolheaddetectto filament, commandT?; cmd T%s at %d times" % (cur_chan, self.AMSErrorRetryTimes))
            #         else:
            # self.G_PhrozenFluiddRespondInfo("[INFO] when toolheaddetectto filament, commandT?retry5 time, commandP?to the park position")
            # #// retractto; // =====P1 D[n]; n:1~32(no network, get 1~4); channelfilamentretractto in park positionstate Yes; ====="P?";
            #             self.Cmds_AMSSerial1Send("P%d" % cur_chan)
            #             self.Cmds_PhrozenKlipperPause(None)
            #             self.AMSErrorRetryTimes = 0
            # #lancaigang20231013: toolhead up detectto filament
            #     else:
            # # state normal normal after, set error retry count
            #         self.AMSErrorRetryTimes = 0

            #     return self.G_PhrozenReactor.NOW + AMS_SERIALPORT_RECV_TIMER

            # lancaigang231103: first disable disable stm32timeoutstate logical, use, error
            # lancaigang20231013: stm32timeoutstate logical
            # if int(message_obj.group("state")) is MC_ERR_TIMEOUT:
            #     # typedef enum Enum_MCStateMachine {
            # # // 00; Idle machine
            #     #     MCSTATEMACHINE_IDLE_STANDBY,
            # # // 01; park positionfeedto machine; // =====P1 S0 allchannel in park positionfeedto machine state, canfeedto park position or retractto park position; ====="RD";
            #     #     MCSTATEMACHINE_PARKPOSITION_ISREADY_INFILA_TO_PRINTER,
            # # // 02; 1; // =====P1 T[n]n:1 ~32(no network, get 1 ~4)manualto channel,(use); ====="T?";
            #     #     MCSTATEMACHINE_CHANGING_FILA_STAGE_P1,
            # # // 03; 2; // =====P1 T[n]n:1 ~32(no network, get 1 ~4)manualto channel,(use); ====="T?";
            #     #     MCSTATEMACHINE_CHANGING_FILA_STAGE_P2,
            # # // 04; refillto head, to P1 T?
            #     #     MCSTATEMACHINE_FORCE_FEED_INFILA_TO_PRINTER,
            # # // 05; (refill)
            #     #     MCSTATEMACHINE_PRINTING_INPROCESS_FEED,
            # # // 06; full unload filament; // =====B1~Bncommand; PRZ_B[n] P1 B[n]n:1 ~32(no network, get 1 ~4)channelfilament full output Yes
            #     #     MCSTATEMACHINE_FULLY_ROLLBACK,
            # # // 07; retractto park position; //"P"; P1 D[n]; n:1~32(no network, get 1~4); channelfilamentretractpark positionstate Yes
            #     #     MCSTATEMACHINE_ROLLBACK_TO_PARKPOSITION,
            # # // 08; retract allto park position; // "AP"; P2 A1 allfilamentto park position Yes
            #     #     MCSTATEMACHINE_ROLLBACK_ALL_TO_PARKPOSITION,
            # # // 09; clearall; //====="CL"; P2 A2; output allfilament Yes
            #     #     MCSTATEMACHINE_CLEAN_ALL_CHANNEL,
            # # // 10; timeout output error state
            #     #     MCSTATEMACHINE_ERROR_TIMEOUT,
            # # // 11; filament runout output error state
            #     #     MCSTATEMACHINE_ERROR_RUNOUT,
            #     # } Enum_MCStateMachine;
            #     self.AMSErrorRetryTimes += 1
            #     if self.AMSErrorRetryTimes < 5:
            # #// =====T1~Tncommand; PRZ_T[n] P1 T[n]n:1 ~32(no network, get 1 ~4)manualto channel,(use)
            # self.G_PhrozenFluiddRespondInfo("[DEBUG] stm32errorstate, commandT?; cmd T%s at %d times" % (message_obj.group("chan"), self.AMSErrorRetryTimes))
            #         self.Cmds_AMSSerial1Send("T%d" % cur_chan)
            #     else:
            # self.G_PhrozenFluiddRespondInfo("[DEBUG] stm32errorstate, commandT?retry5 time, commandP?to the park position")
            # #// retractto; // =====P1 D[n]; n:1~32(no network, get 1~4); channelfilamentretractto in park positionstate Yes; ====="P?";
            #         self.Cmds_AMSSerial1Send("P%d" % cur_chan)
            #         self.Cmds_PhrozenKlipperPause(None)
            #         self.AMSErrorRetryTimes = 0

            #     return self.G_PhrozenReactor.NOW + AMS_SERIALPORT_RECV_TIMER

            

        # lancaigang20231013: refillmode
        if int(message_obj.group("mode")) is AMS_MA_MODE:
            pass



    
    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    # lancaigang20231013: serial port receive logical timer
    #100ms
    def Device_TimmerUart1Recv(self, eventtime):
        #self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)Device_TimmerUart1Recv]")
        #lancaigang240427: try catch
        try:
            # if self.G_SerialPort1OpenFlag==False:
            # self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)Device_TimmerUart1Recv]serial port1already close")
            # if self.G_SerialPort2OpenFlag==False:
            # self.G_PhrozenFluiddRespondInfo("[INFO] [(dev.python)Device_TimmerUart1Recv]serial port2already close")

            # tty1connectfailed
            if self.G_SerialPort1OpenFlag==False:
                self.G_ASM1DisconnectErrorCount=0
                # self.G_PhrozenFluiddRespondInfo("[ERROR] [(dev.python)Device_TimmerUart1Recv]serial port 1connectederror, exitcallback")
                #self.G_PhrozenFluiddRespondInfo("self.G_AMS1ErrorRestartCount=%d" % self.G_AMS1ErrorRestartCount)
                try:
                    if self.G_SerialPort1Obj is not None:
                        if self.G_SerialPort1Obj.is_open:
                            # tty1close
                            self.G_SerialPort1Obj.close()
                            self.G_PhrozenFluiddRespondInfo('[INFO] closeserial port 1successful')
                            self.G_PhrozenFluiddRespondInfo("[INFO] AMS1connectedfailed")
                            #self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                            self.G_PhrozenFluiddRespondInfo('[WARN] pausecommand+PAUSE:g')
                except:
                    self.G_PhrozenFluiddRespondInfo('[INFO] closeserial port 1error')

                self.G_AMS1ErrorRestartCount=self.G_AMS1ErrorRestartCount+1

                # lancaigang241108:when then pause, preventAMSrestartneed towhen
                if self.G_AMS1ErrorRestartCount>=5:
                    #self.G_PhrozenFluiddRespondInfo("[ERROR] if self.G_AMS1ErrorRestartCount>=5:")

                    self.G_AMS1ErrorRestartCount=0
                    # lancaigang250619:ifUSBerror, errorwhen report error
                    self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                    # self.G_PhrozenFluiddRespondInfo("[WARN] pausecommand+PAUSE:g")

                    # if self.G_KlipperIfPaused==False:
                    #     self.G_KlipperIfPaused = True
                    #     #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                    #     if self.G_CancelFlag==False:
                    #         # self.G_PhrozenFluiddRespondInfo("+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    #         # self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                    # self.G_PhrozenFluiddRespondInfo("[ERROR] AMS1connectederrorpause")

                    #         #lancaigang250604:
                    #         if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:#M0
                    # self.G_PhrozenFluiddRespondInfo("[WARN] Unknown mode, do notpause")
                    #         else:
                    #             if self.STM32ReprotPauseFlag==0:
                    #                 self.G_PauseTriggerWhileChangeChannelFlag=True
                    #                 if self.PG102Flag==True:
                    # self.G_PhrozenFluiddRespondInfo("[WARN] purge, temporary pause, purgecompletethen pause")
                    #                     self.PG102DelayPauseFlag=True
                    #                     #self.G_PhrozenFluiddRespondInfo("+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    #                     self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                    #                 else:
                    # self.G_PhrozenFluiddRespondInfo("[WARN] purge, canpause")
                    #                     self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                    #                     self.G_KlipperIfPaused = True
                    #                     #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                    #                     self.STM32ReprotPauseFlag=1
                    # #lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                    #                     self.G_ChangeChannelFirstFilaFlag=True
                    #                     self.G_PhrozenFluiddRespondInfo("+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    #                     self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                    #             else:
                    #                 self.G_PauseTriggerWhileChangeChannelFlag=True
                    #                 self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

                    #         #     self.G_PhrozenFluiddRespondInfo("+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    # #lancaigang20231013: disconnectconnect
                    #         self.Device_DisconnectAMSDevice()

                    # #if self.G_KlipperIfPaused==True:
                    # else:
                    # self.G_PhrozenFluiddRespondInfo("[ERROR] USBerror, currentalready is pausestate")
                    #     self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)


                    # lancaigang240524: output callback
                    return self.G_PhrozenReactor.NEVER


                return eventtime + AMS_SERIALPORT_RECV_TIMER

            # lancaigang250619:USBconnect normal normal, then clear
            #self.G_PauseToLCDString=""
            self.G_AMS1ErrorRestartCount=0






            # #lancaigang240427: AMSerrorrestart, need to
            # if self.G_AMS1ErrorRestartFlag == True:
            # self.G_PhrozenFluiddRespondInfo("AMS1error or restart;self.G_AMSErrorRestartCount=%d" % self.G_AMSErrorRestartCount)
            #     self.G_PhrozenFluiddRespondInfo("+AMSReboot:%d" % self.G_AMSErrorRestartCount)
            #     self.G_AMS1ErrorRestartFlag = False
                
            #     try:
            #         self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 1")
            #         self.G_SerialPort1Obj = serial.Serial(self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3)
            # #serial port1opensuccessful
            #         if self.G_SerialPort1Obj.is_open:
            #             self.G_SerialPort1OpenFlag = True
            # self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 1successful")
            # #lancaigang231213: openserial port1
            #             self.G_SerialPort1Obj.flushInput()  # clean serial write cache
            #             self.G_SerialPort1Obj.flush()
            #             self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 buffers cleared")
            #             self.G_PhrozenFluiddRespondInfo("[INFO] Re-registering serial port 1 callback")
            #             self.G_SerialPort1RecvTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerUart1Recv, self.G_PhrozenReactor.NOW)
            #     except:
            #         self.G_PhrozenFluiddRespondInfo("[ERROR] Unable to open tty1. Check the USB connection or try rebooting.")

            #     return eventtime + AMS_SERIALPORT_RECV_TIMER



            # #lancaigang240410: 
            # if self.G_CancelFlag==True:
            # #self.G_PhrozenFluiddRespondInfo("[INFO] Print has been canceled")
            #     return eventtime + AMS_SERIALPORT_RECV_TIMER



            # lancaigang231103:tty1serial porthas number data
            if self.G_SerialPort1Obj.inWaiting() > 0:
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

                self.G_PhrozenFluiddRespondInfo('[INFO] [(dev.python)Device_TimmerUart1Recv]serial port 1 read get number data')
                Lo_SerialRxLen=self.G_SerialPort1Obj.inWaiting()
                self.G_PhrozenFluiddRespondInfo('byte countLo_SerialRxLen=%d' % Lo_SerialRxLen)
                # self.G_PhrozenFluiddRespondInfo("[INFO] serial porttimer receive ")
                Lo_SerialRxBytes=self.G_SerialPort1Obj.read(Lo_SerialRxLen)
                self.G_PhrozenFluiddRespondInfo('byte streamLo_SerialRxBytes=%s' % Lo_SerialRxBytes)
                self.G_PhrozenFluiddRespondInfo('byte streambinascii.hexlify(Lo_SerialRxBytes)=%s' % binascii.hexlify(Lo_SerialRxBytes))
                #self.G_PhrozenFluiddRespondInfo("%x" % binascii.hexlify(Lo_SerialRxBytes))
                self.G_PhrozenFluiddRespondInfo('byte countlen(Lo_SerialRxBytes)=%d' % len(Lo_SerialRxBytes))
                #self.G_PhrozenFluiddRespondInfo("Lo_SerialRxBytes.count=%d" % Lo_SerialRxBytes.count)
                #for i in Lo_SerialRxBytes:
                    #self.G_PhrozenFluiddRespondInfo("%x" % i)

                self.G_PhrozenFluiddRespondInfo('Lo_SerialRxBytes[0] - hex byte0x%2x' % Lo_SerialRxBytes[0])
                self.G_PhrozenFluiddRespondInfo('Lo_SerialRxBytes[0] - ASCII character%c' % Lo_SerialRxBytes[0])
                # self.G_PhrozenFluiddRespondInfo("Lo_SerialRxBytes[1]-hex byte0x%2x" % Lo_SerialRxBytes[1])
                # self.G_PhrozenFluiddRespondInfo("Lo_SerialRxBytes[1]-ASCII character%c" % Lo_SerialRxBytes[1])



                # lancaigang240705: existsAMSmulti-material
                self.G_AMSDevice1IfNormal=True


                try:
                    # lancaigang250411: AMSstatereport
                    #if "R" in self.G_SerialRxASCIIStr:
                    if Lo_SerialRxBytes[0]==0x52 and Lo_SerialRxLen==16:
                        self.G_PhrozenFluiddRespondInfo('[INFO] AMS unit 1 async response')

                        #self.G_PhrozenFluiddRespondInfo("%s" % SerialRxASCIIStr)
                        Lo_AMSDeviceStateInfo = AMSDetailInfoBytes()
                        Lo_AMSDeviceStateInfo.whole[:] = Lo_SerialRxBytes
                        # pythonempty
                        Lo_AMSDetailState = {}
                        self.G_AMS1DeviceState["dev_id"] = Lo_AMSDetailState["dev_id"] = Lo_AMSDeviceStateInfo.field.dev_id
                        self.G_AMS1DeviceState["active_dev_id"] = Lo_AMSDetailState["active_dev_id"] = Lo_AMSDeviceStateInfo.field.active_dev_id
                        self.G_AMS1DeviceState["dev_mode"] = Lo_AMSDetailState["dev_mode"] = Lo_AMSDeviceStateInfo.field.dev_mode
                        self.G_AMS1DeviceState["cache_empty"] = Lo_AMSDetailState["cache_empty"] = Lo_AMSDeviceStateInfo.field.cache_empty
                        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]buffer device emptystate(bool)==%d" % Lo_AMSDeviceStateInfo.field.cache_empty)
                        self.G_AMS1DeviceState["cache_full"] = Lo_AMSDetailState["cache_full"] = Lo_AMSDeviceStateInfo.field.cache_full
                        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]buffer device fullstate(bool)==%d" % Lo_AMSDeviceStateInfo.field.cache_full)
                        self.G_AMS1DeviceState["cache_exist"] = Lo_AMSDetailState["cache_exist"] = Lo_AMSDeviceStateInfo.field.cache_exist
                        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]buffer device filamentstate(bool)==%d" % Lo_AMSDeviceStateInfo.field.cache_exist)
                        self.G_AMS1DeviceState["mc_state"] = Lo_AMSDetailState["mc_state"] = Lo_AMSDeviceStateInfo.field.mc_state
                        self.G_AMS1DeviceState["ma_state"] = Lo_AMSDetailState["ma_state"] = Lo_AMSDeviceStateInfo.field.ma_state
                        self.G_AMS1DeviceState["entry_state"] = Lo_AMSDetailState["entry_state"] = Lo_AMSDeviceStateInfo.field.entry_state
                        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]entry device state(bit)==%d" % Lo_AMSDeviceStateInfo.field.entry_state)
                        self.G_AMS1DeviceState["park_state"] = Lo_AMSDetailState["park_state"] = Lo_AMSDeviceStateInfo.field.park_state
                        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]park position device state(bit)==%d" % Lo_AMSDeviceStateInfo.field.park_state)
                        
                        # number data json switch
                        self.G_PhrozenFluiddRespondInfo(json.dumps(Lo_AMSDetailState))

                        #lancaigang250708: 
                        self.G_PhrozenFluiddRespondInfo('[INFO] P114 successful')
                        self.G_PhrozenFluiddRespondInfo("[INFO] +P114:1")
                        self.G_P114RunFlag=0

                    else:
                        self.G_PhrozenFluiddRespondInfo('[INFO] AMS file')
                        # lancaigang20231013: readttyUSB0serial portbyte stream switch is ASCII code
                        # lancaigang240530: hex byte switch is ASCII character
                        self.G_SerialRxASCIIStr = Lo_SerialRxBytes.decode("ascii")
                        self.G_PhrozenFluiddRespondInfo('ASCII characterself.G_SerialRxASCIIStr=%s' % self.G_SerialRxASCIIStr)


                        # lancaigang250411: AMSfirmwareversion

                        # // AMSmainboard2firmware-1 1
                        if "V-H18-I18-F" in self.G_SerialRxASCIIStr:
                            self.G_PhrozenFluiddRespondInfo('[INFO] AMS code disable unit 1 file')
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
                            # :c0f535790a90/GetZbGwInfo_Respon
                            # {
                            #     "Data_ID": 95,
                            #     "Data": {
                            #         "GwId": "c0f535790a90",
                            #         "HomeId": "",
                            #         "GWSN": "0000000000000000000",
                            #         "AccountId": "",
                            #         "GwMac": "c0f535790a90",
                            #         "GwIP": "192.168.3.53",
                            #         "GwName": "Name-c0f535790a90",
                            #         "ProductId": "ARCO",
                            #         "MainImage": 15,
                            #         "MainHWVersion": 15,
                            #         "MainFWVersion": 24064,
                            #         "Gw_Ram": 248584,
                            #         "Gw_Rom": 826536,
                            #         "JoinMode": 1,
                            #         "ESSID": "",
                            #         "MqttClientEMQfd": 31,
                            #         "MqttClientId": "c0f535790a90",
                            #         "MqttBrokerUserName": "",
                            #         "MqttBrokerPwd": "",
                            #         "DriveCodeList": [
                            #             {
                            #                 "DriveCode": 1,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 2,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 3,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 4,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 5,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 6,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 7,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 8,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 9,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 10,
                            #                 "DriveImageType": 10,
                            #                 "DriveHwVersion": 10,
                            #                 "DriveFwVersion": 25033,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 11,
                            #                 "DriveImageType": 11,
                            #                 "DriveHwVersion": 11,
                            #                 "DriveFwVersion": 25022,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 12,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 13,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 14,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 15,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 16,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 17,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 18,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 19,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 20,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             }
                            #         ]
                            #     }
                            # }
                            #lancaigang250724: Read the system image ID to distinguish product, mainboard, and firmware variants
                            #lancaigang250724:Read image ID
                            self.Cmds_GetImageId()
                            if self.G_ImageId==16:
                                self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==16: ARCO300-MKS-RK3328-STM32F407VET6-I16")
                                # lancaigang240530: versionwrite todat text file; DriveCodeJson.dat
                                filename='/home/mks/hdlDat/DriveCodeFile.dat'
                            elif self.G_ImageId==31:
                                self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==31: ARCO300-phrozen-RK3308-STM32F407VET6-I31")
                                # lancaigang240530: versionwrite todat text file; DriveCodeJson.dat
                                filename='/home/prz/hdlDat/DriveCodeFile.dat'
                            elif self.G_ImageId==-1:
                                self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==-1, default: ARCO300-MKS-RK3328-STM32F407VET6-I16")
                                # lancaigang240530: versionwrite todat text file; DriveCodeJson.dat
                                filename='/home/mks/hdlDat/DriveCodeFile.dat'
                            else:
                                self.G_PhrozenFluiddRespondInfo("[INFO] Image IDcould not be read, default: ARCO300-MKS-RK3328-STM32F407VET6-I16")
                                # lancaigang240530: versionwrite todat text file; DriveCodeJson.dat
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
                                    # 1 , 18 , 24053 , 18 , 0
                                    split[0]=split[0].strip()# move
                                    split[1]=split[1].strip()# file id
                                    split[2]=split[2].strip()# firmwareversion
                                    split[3]=split[3].strip()# id
                                    split[4]=split[4].strip()# is no in
                                    # split[4]='0'# is no in, default0
                                    #if "SN1" in self.G_SerialRxASCIIStr:
                                    if split[0] == "1":
                                        self.G_PhrozenFluiddRespondInfo('[INFO] AMS code disable unit 1 file')
                                        self.G_PhrozenFluiddRespondInfo(split[0])
                                        self.G_PhrozenFluiddRespondInfo(split[1])
                                        self.G_PhrozenFluiddRespondInfo(split[2])
                                        self.G_PhrozenFluiddRespondInfo(split[3])
                                        self.G_PhrozenFluiddRespondInfo(split[4])
                                        #line=("%d,%d,%d," % (HW_VERSION,,))
                                        line_modify=split[0]+','+'18'+','+self.G_SerialRxASCIIStr[11:16]+','+'18'+','+'1'
                                        self.G_PhrozenFluiddRespondInfo(line_modify)
                                        Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
                                    else:
                                        Lo_AllLine=Lo_AllLine+line
                                    # if "SN2" in self.G_SerialRxASCIIStr:
                                    #     if split[0] == "2":
                                    #         self.G_PhrozenFluiddRespondInfo(split[0])
                                    #         self.G_PhrozenFluiddRespondInfo(split[1])
                                    #         self.G_PhrozenFluiddRespondInfo(split[2])
                                    #         self.G_PhrozenFluiddRespondInfo(split[3])
                                    #         self.G_PhrozenFluiddRespondInfo(split[4])
                                    #         #line=("%d,%d,%d," % (HW_VERSION,,))
                                    #         line_modify=split[0]+','+'1'+','+self.G_SerialRxASCIIStr[9:14]+','+'1'+','+'1'
                                    #         self.G_PhrozenFluiddRespondInfo(line_modify)
                                    #         Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
                                    #     else:
                                    #         Lo_AllLine=Lo_AllLine+line
                                    # if "SN3" in self.G_SerialRxASCIIStr:
                                    #     if split[0] == "3":
                                    #         self.G_PhrozenFluiddRespondInfo(split[0])
                                    #         self.G_PhrozenFluiddRespondInfo(split[1])
                                    #         self.G_PhrozenFluiddRespondInfo(split[2])
                                    #         self.G_PhrozenFluiddRespondInfo(split[3])
                                    #         self.G_PhrozenFluiddRespondInfo(split[4])
                                    #         #line=("%d,%d,%d," % (HW_VERSION,,))
                                    #         line_modify=split[0]+','+'1'+','+self.G_SerialRxASCIIStr[9:14]+','+'1'+','+'1'
                                    #         self.G_PhrozenFluiddRespondInfo(line_modify)
                                    #         Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
                                    #     else:
                                    #         Lo_AllLine=Lo_AllLine+line
                                    # if "SN4" in self.G_SerialRxASCIIStr:
                                    #     if split[0] == "4":
                                    #         self.G_PhrozenFluiddRespondInfo(split[0])
                                    #         self.G_PhrozenFluiddRespondInfo(split[1])
                                    #         self.G_PhrozenFluiddRespondInfo(split[2])
                                    #         self.G_PhrozenFluiddRespondInfo(split[3])
                                    #         self.G_PhrozenFluiddRespondInfo(split[4])
                                    #         #line=("%d,%d,%d," % (HW_VERSION,,))
                                    #         line_modify=split[0]+','+'1'+','+self.G_SerialRxASCIIStr[9:14]+','+'1'+','+'1'
                                    #         self.G_PhrozenFluiddRespondInfo(line_modify)
                                    #         Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
                                    #     else:
                                    #         Lo_AllLine=Lo_AllLine+line
                            #self.G_PhrozenFluiddRespondInfo(Lo_AllLine)
                            with open(filename,"w+") as file_w:
                                file_w.write(Lo_AllLine)


                        self.Device_TimmerUartRecvHandler(1,Lo_SerialRxBytes,self.G_SerialRxASCIIStr)

                except:
                    self.G_PhrozenFluiddRespondInfo('[INFO] serial port number data error, noneAMSstate')


            return eventtime + AMS_SERIALPORT_RECV_TIMER

        except Exception as e:
            self.G_PhrozenFluiddRespondInfo('[INFO] serial port 1 read get error, AMS1error or restart, please checkAMS1 is no normal normal')
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
            # lancaigang0427: ifAMSerrorrestart, need to, and restartsuccessfulsending commandstm32refill
            # lancaigang240427: AMSerrorrestart, need to
            self.G_AMS1ErrorRestartFlag = True
            self.G_AMS1ErrorRestartCount=self.G_AMS1ErrorRestartCount+1

            # lancaigang241011: serial porterror, not allowed send number data
            self.G_SerialPort1OpenFlag=False
            
            # lancaigang240521: resumewhen, ifdetectedAMSerrorrestart, can is is AMS, unload filament
            self.G_ResumeCheckAMS1ErrorRestartFlag = True

            
            return eventtime + AMS_SERIALPORT_RECV_TIMER


    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
    #100ms
    def Device_TimmerUart2Recv(self, eventtime):
        try:
            # tty2connectfailed
            if self.G_SerialPort2OpenFlag==False:
                self.G_ASM1DisconnectErrorCount=0
                # self.G_PhrozenFluiddRespondInfo("[ERROR] [(dev.python)Device_TimmerUart2Recv]serial port 2connectederror, exitcallback")
                #self.G_PhrozenFluiddRespondInfo("self.G_AMS2ErrorRestartCount=%d" % self.G_AMS2ErrorRestartCount)

                self.G_AMS2ErrorRestartCount=self.G_AMS2ErrorRestartCount+1

                try:
                    if self.G_SerialPort2Obj is not None:
                        if self.G_SerialPort2Obj.is_open:
                            # tty2close
                            self.G_SerialPort2Obj.close()
                            self.G_PhrozenFluiddRespondInfo('[INFO] closeserial port 2successful')
                            self.G_PhrozenFluiddRespondInfo("[INFO] AMS2connectedfailed")
                            self.G_PhrozenFluiddRespondInfo('[WARN] pausecommand+PAUSE:g')
                except:
                    self.G_PhrozenFluiddRespondInfo('[INFO] closeserial port 2error')

                # lancaigang241108:when then pause, preventAMSrestartneed towhen
                if self.G_AMS2ErrorRestartCount>=5:
                    #self.G_PhrozenFluiddRespondInfo("[ERROR] if self.G_AMS2ErrorRestartCount>=5:")

                    self.G_AMS2ErrorRestartCount=0
                    # lancaigang250619:ifUSBerror, errorwhen report error
                    self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                    

                    # if self.G_KlipperIfPaused==False:
                    #     self.G_KlipperIfPaused = True
                    #     #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                    #     if self.G_CancelFlag==False:
                    #         # self.G_PhrozenFluiddRespondInfo("+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    #         # self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                    # self.G_PhrozenFluiddRespondInfo("[ERROR] AMS2connectederrorpause")

                    #         #lancaigang250604:
                    #         if self.G_AMSDeviceWorkMode == AMS_WORK_MODE_UNKNOW:#M0
                    # self.G_PhrozenFluiddRespondInfo("[WARN] Unknown mode, do notpause")
                    #         else:
                    #             if self.STM32ReprotPauseFlag==0:
                    #                 self.G_PauseTriggerWhileChangeChannelFlag=True
                    #                 if self.PG102Flag==True:
                    # self.G_PhrozenFluiddRespondInfo("[WARN] purge, temporary pause, purgecompletethen pause")
                    #                     self.PG102DelayPauseFlag=True
                    #                     #self.G_PhrozenFluiddRespondInfo("+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    #                     self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                    #                 else:
                    # self.G_PhrozenFluiddRespondInfo("[WARN] purge, canpause")
                    #                     self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                    #                     self.G_KlipperIfPaused = True
                    #                     #self.Cmds_PhrozenKlipperPauseNoneCmdToSTM32(None)
                    #                     self.STM32ReprotPauseFlag=1
                    # #lancaigang231202: P1 C?Automatic filament changewhen, if #1 time channelfeederrorpause, if to resume, from #1 time channel
                    #                     self.G_ChangeChannelFirstFilaFlag=True
                    #                     self.G_PhrozenFluiddRespondInfo("+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    #                     self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)
                    #             else:
                    #                 self.G_PauseTriggerWhileChangeChannelFlag=True
                    #                 self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)

                    #         #     self.G_PhrozenFluiddRespondInfo("+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan))
                    # #lancaigang20231013: disconnectconnect
                    #         self.Device_DisconnectAMSDevice()

                    # #if self.G_KlipperIfPaused==True:
                    # else:
                    # self.G_PhrozenFluiddRespondInfo("[ERROR] USBerror, currentalready is pausestate")
                    #     self.G_PauseToLCDString="+PAUSE:g,%d,%d" % (self.G_ChangeChannelTimeoutOldChan,self.G_ChangeChannelTimeoutNewChan)


                    # lancaigang240524: output callback
                    return self.G_PhrozenReactor.NEVER

                return eventtime + AMS_SERIALPORT_RECV_TIMER

            # lancaigang250619:USBconnect normal normal, then clear
            #self.G_PauseToLCDString=""
            self.G_AMS2ErrorRestartCount=0


            #lancaigang241128: 
            if self.G_CancelFlag==True:
                # self.G_PhrozenFluiddRespondInfo("[WARN] [(dev.python)Device_TimmerRunoutCheck]Print has been canceled")
                return eventtime + AMS_SERIALPORT_RECV_TIMER
        


            # #lancaigang240427: AMSerrorrestart, need to
            # if self.G_AMS1ErrorRestartFlag == True:
            # self.G_PhrozenFluiddRespondInfo("AMS1error or restart;self.G_AMSErrorRestartCount=%d" % self.G_AMSErrorRestartCount)
            #     self.G_PhrozenFluiddRespondInfo("+AMSReboot:%d" % self.G_AMSErrorRestartCount)
            #     self.G_AMS1ErrorRestartFlag = False
                
            #     try:
            #         self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 1")
            #         self.G_SerialPort1Obj = serial.Serial(self.G_Serialport1Define, SERIAL_PORT_BAUD, timeout=3)
            # #serial port1opensuccessful
            #         if self.G_SerialPort1Obj.is_open:
            #             self.G_SerialPort1OpenFlag = True
            # self.G_PhrozenFluiddRespondInfo("[INFO] Reinitializing serial port 1successful")
            # #lancaigang231213: openserial port1
            #             self.G_SerialPort1Obj.flushInput()  # clean serial write cache
            #             self.G_SerialPort1Obj.flush()
            #             self.G_PhrozenFluiddRespondInfo("[INFO] Serial port 1 buffers cleared")
            #             self.G_PhrozenFluiddRespondInfo("[INFO] Re-registering serial port 1 callback")
            #             self.G_SerialPort1RecvTimmer = self.G_PhrozenReactor.register_timer(self.Device_TimmerUart1Recv, self.G_PhrozenReactor.NOW)
            #     except:
            #         self.G_PhrozenFluiddRespondInfo("[ERROR] Unable to open tty1. Check the USB connection or try rebooting.")

            #     return eventtime + AMS_SERIALPORT_RECV_TIMER



            # #lancaigang240410: 
            # if self.G_CancelFlag==True:
            # #self.G_PhrozenFluiddRespondInfo("[INFO] Print has been canceled")
            #     return eventtime + AMS_SERIALPORT_RECV_TIMER



            # lancaigang231103:tty2serial porthas number data
            if self.G_SerialPort2Obj.inWaiting() > 0:
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

                self.G_PhrozenFluiddRespondInfo('[INFO] [(dev.python)Device_TimmerUart2Recv]serial port 2 read get number data')
                Lo_SerialRxLen=self.G_SerialPort2Obj.inWaiting()
                self.G_PhrozenFluiddRespondInfo('byte countLo_SerialRxLen=%d' % Lo_SerialRxLen)
                # self.G_PhrozenFluiddRespondInfo("[INFO] serial porttimer receive ")
                Lo_SerialRxBytes=self.G_SerialPort2Obj.read(Lo_SerialRxLen)
                self.G_PhrozenFluiddRespondInfo('byte streamLo_SerialRxBytes=%s' % Lo_SerialRxBytes)
                self.G_PhrozenFluiddRespondInfo('byte streambinascii.hexlify(Lo_SerialRxBytes)=%s' % binascii.hexlify(Lo_SerialRxBytes))
                #self.G_PhrozenFluiddRespondInfo("%x" % binascii.hexlify(Lo_SerialRxBytes))
                self.G_PhrozenFluiddRespondInfo('byte countlen(Lo_SerialRxBytes)=%d' % len(Lo_SerialRxBytes))
                #self.G_PhrozenFluiddRespondInfo("Lo_SerialRxBytes.count=%d" % Lo_SerialRxBytes.count)
                #for i in Lo_SerialRxBytes:
                    #self.G_PhrozenFluiddRespondInfo("%x" % i)

                self.G_PhrozenFluiddRespondInfo('Lo_SerialRxBytes[0] - hex byte0x%2x' % Lo_SerialRxBytes[0])
                self.G_PhrozenFluiddRespondInfo('Lo_SerialRxBytes[0] - ASCII character%c' % Lo_SerialRxBytes[0])
                # self.G_PhrozenFluiddRespondInfo("Lo_SerialRxBytes[1]-hex byte0x%2x" % Lo_SerialRxBytes[1])
                # self.G_PhrozenFluiddRespondInfo("Lo_SerialRxBytes[1]-ASCII character%c" % Lo_SerialRxBytes[1])


                # lancaigang20231013: readttyUSB0serial portbyte stream switch is ASCII code
                # lancaigang240530: hex byte switch is ASCII character
                self.G_SerialRxASCIIStr = Lo_SerialRxBytes.decode("ascii")
                # self.G_PhrozenFluiddRespondInfo("ASCII characterself.G_SerialRxASCIIStr=%s" % self.G_SerialRxASCIIStr)



                # lancaigang240705: existsAMSmulti-material
                self.G_AMSDevice2IfNormal=True


                try:
                    # #if "R" in self.G_SerialRxASCIIStr:
                    # if Lo_SerialRxBytes[0]==0x52:
                    # self.G_PhrozenFluiddRespondInfo("[INFO] AMS unit 2 async response")

                    #     #self.G_PhrozenFluiddRespondInfo("%s" % SerialRxASCIIStr)
                    #     Lo_AMSDeviceStateInfo = AMSDetailInfoBytes()
                    #     Lo_AMSDeviceStateInfo.whole[:] = Lo_SerialRxBytes
                    # #pythonempty
                    #     Lo_AMSDetailState = {}
                    #     self.G_AMS2DeviceState["dev_id"] = Lo_AMSDetailState["dev_id"] = Lo_AMSDeviceStateInfo.field.dev_id
                    #     self.G_AMS2DeviceState["active_dev_id"] = Lo_AMSDetailState["active_dev_id"] = Lo_AMSDeviceStateInfo.field.active_dev_id
                    #     self.G_AMS2DeviceState["dev_mode"] = Lo_AMSDetailState["dev_mode"] = Lo_AMSDeviceStateInfo.field.dev_mode
                    #     self.G_AMS2DeviceState["cache_empty"] = Lo_AMSDetailState["cache_empty"] = Lo_AMSDeviceStateInfo.field.cache_empty
                    # #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]buffer device emptystate(bool)==%d" % Lo_AMSDeviceStateInfo.field.cache_empty)
                    #     self.G_AMS2DeviceState["cache_full"] = Lo_AMSDetailState["cache_full"] = Lo_AMSDeviceStateInfo.field.cache_full
                    # #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]buffer device fullstate(bool)==%d" % Lo_AMSDeviceStateInfo.field.cache_full)
                    #     self.G_AMS2DeviceState["cache_exist"] = Lo_AMSDetailState["cache_exist"] = Lo_AMSDeviceStateInfo.field.cache_exist
                    # #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]buffer device filamentstate(bool)==%d" % Lo_AMSDeviceStateInfo.field.cache_exist)
                    #     self.G_AMS2DeviceState["mc_state"] = Lo_AMSDetailState["mc_state"] = Lo_AMSDeviceStateInfo.field.mc_state
                    #     self.G_AMS2DeviceState["ma_state"] = Lo_AMSDetailState["ma_state"] = Lo_AMSDeviceStateInfo.field.ma_state
                    #     self.G_AMS2DeviceState["entry_state"] = Lo_AMSDetailState["entry_state"] = Lo_AMSDeviceStateInfo.field.entry_state
                    # #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]entry device state(bit)==%d" % Lo_AMSDeviceStateInfo.field.entry_state)
                    #     self.G_AMS2DeviceState["park_state"] = Lo_AMSDetailState["park_state"] = Lo_AMSDeviceStateInfo.field.park_state
                    # #self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]park position device state(bit)==%d" % Lo_AMSDeviceStateInfo.field.park_state)
                        
                    # # number data json switch
                    #     self.G_PhrozenFluiddRespondInfo(json.dumps(Lo_AMSDetailState))


                    #if "R" in self.G_SerialRxASCIIStr:
                    if Lo_SerialRxBytes[0]==0x52 and Lo_SerialRxLen==16:
                        self.G_PhrozenFluiddRespondInfo('[INFO] AMS unit 2 async response')

                        #self.G_PhrozenFluiddRespondInfo("%s" % SerialRxASCIIStr)
                        Lo_AMSDeviceStateInfo = AMSDetailInfoBytes()
                        Lo_AMSDeviceStateInfo.whole[:] = Lo_SerialRxBytes
                        # pythonempty
                        Lo_AMSDetailState = {}
                        self.G_AMS1DeviceState["dev_id"] = Lo_AMSDetailState["dev_id"] = Lo_AMSDeviceStateInfo.field.dev_id
                        self.G_AMS1DeviceState["active_dev_id"] = Lo_AMSDetailState["active_dev_id"] = Lo_AMSDeviceStateInfo.field.active_dev_id
                        self.G_AMS1DeviceState["dev_mode"] = Lo_AMSDetailState["dev_mode"] = Lo_AMSDeviceStateInfo.field.dev_mode
                        self.G_AMS1DeviceState["cache_empty"] = Lo_AMSDetailState["cache_empty"] = Lo_AMSDeviceStateInfo.field.cache_empty
                        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]buffer device emptystate(bool)==%d" % Lo_AMSDeviceStateInfo.field.cache_empty)
                        self.G_AMS1DeviceState["cache_full"] = Lo_AMSDetailState["cache_full"] = Lo_AMSDeviceStateInfo.field.cache_full
                        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]buffer device fullstate(bool)==%d" % Lo_AMSDeviceStateInfo.field.cache_full)
                        self.G_AMS1DeviceState["cache_exist"] = Lo_AMSDetailState["cache_exist"] = Lo_AMSDeviceStateInfo.field.cache_exist
                        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]buffer device filamentstate(bool)==%d" % Lo_AMSDeviceStateInfo.field.cache_exist)
                        self.G_AMS1DeviceState["mc_state"] = Lo_AMSDetailState["mc_state"] = Lo_AMSDeviceStateInfo.field.mc_state
                        self.G_AMS1DeviceState["ma_state"] = Lo_AMSDetailState["ma_state"] = Lo_AMSDeviceStateInfo.field.ma_state
                        self.G_AMS1DeviceState["entry_state"] = Lo_AMSDetailState["entry_state"] = Lo_AMSDeviceStateInfo.field.entry_state
                        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]entry device state(bit)==%d" % Lo_AMSDeviceStateInfo.field.entry_state)
                        self.G_AMS1DeviceState["park_state"] = Lo_AMSDetailState["park_state"] = Lo_AMSDeviceStateInfo.field.park_state
                        # self.G_PhrozenFluiddRespondInfo("[(cmds.python)Cmds_CmdP114]park position device state(bit)==%d" % Lo_AMSDeviceStateInfo.field.park_state)
                        
                        # number data json switch
                        self.G_PhrozenFluiddRespondInfo(json.dumps(Lo_AMSDetailState))

                        #lancaigang250708: 
                        self.G_PhrozenFluiddRespondInfo('[INFO] P114 successful')
                        self.G_PhrozenFluiddRespondInfo("[INFO] +P114:1")
                        self.G_P114RunFlag=0

                    else:
                        self.G_PhrozenFluiddRespondInfo('[INFO] AMS file')
                        # lancaigang20231013: readttyUSB0serial portbyte stream switch is ASCII code
                        # lancaigang240530: hex byte switch is ASCII character
                        self.G_SerialRxASCIIStr = Lo_SerialRxBytes.decode("ascii")
                        self.G_PhrozenFluiddRespondInfo('ASCII characterself.G_SerialRxASCIIStr=%s' % self.G_SerialRxASCIIStr)


                        # lancaigang250411: AMSfirmwareversion

                        # // AMSmainboard2firmware-1 1
                        if "V-H18-I18-F" in self.G_SerialRxASCIIStr:
                            self.G_PhrozenFluiddRespondInfo('[INFO] AMS code disable unit 2 file')
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
                            # :c0f535790a90/GetZbGwInfo_Respon
                            # {
                            #     "Data_ID": 95,
                            #     "Data": {
                            #         "GwId": "c0f535790a90",
                            #         "HomeId": "",
                            #         "GWSN": "0000000000000000000",
                            #         "AccountId": "",
                            #         "GwMac": "c0f535790a90",
                            #         "GwIP": "192.168.3.53",
                            #         "GwName": "Name-c0f535790a90",
                            #         "ProductId": "ARCO",
                            #         "MainImage": 15,
                            #         "MainHWVersion": 15,
                            #         "MainFWVersion": 24064,
                            #         "Gw_Ram": 248584,
                            #         "Gw_Rom": 826536,
                            #         "JoinMode": 1,
                            #         "ESSID": "",
                            #         "MqttClientEMQfd": 31,
                            #         "MqttClientId": "c0f535790a90",
                            #         "MqttBrokerUserName": "",
                            #         "MqttBrokerPwd": "",
                            #         "DriveCodeList": [
                            #             {
                            #                 "DriveCode": 1,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 2,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 3,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 4,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 5,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 6,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 7,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 8,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 9,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 10,
                            #                 "DriveImageType": 10,
                            #                 "DriveHwVersion": 10,
                            #                 "DriveFwVersion": 25033,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 11,
                            #                 "DriveImageType": 11,
                            #                 "DriveHwVersion": 11,
                            #                 "DriveFwVersion": 25022,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 12,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 13,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 14,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 15,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 16,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 17,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 18,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 19,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             },
                            #             {
                            #                 "DriveCode": 20,
                            #                 "DriveImageType": 0,
                            #                 "DriveHwVersion": 0,
                            #                 "DriveFwVersion": 0,
                            #                 "DriveId": 0
                            #             }
                            #         ]
                            #     }
                            # }
                            #lancaigang250724: Read the system image ID to distinguish product, mainboard, and firmware variants
                            #lancaigang250724:Read image ID
                            self.Cmds_GetImageId()
                            if self.G_ImageId==16:
                                self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==16: ARCO300-MKS-RK3328-STM32F407VET6-I16")
                                # lancaigang240530: versionwrite todat text file; DriveCodeJson.dat
                                filename='/home/mks/hdlDat/DriveCodeFile.dat'
                            elif self.G_ImageId==31:
                                self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==31: ARCO300-phrozen-RK3308-STM32F407VET6-I31")
                                # lancaigang240530: versionwrite todat text file; DriveCodeJson.dat
                                filename='/home/prz/hdlDat/DriveCodeFile.dat'
                            elif self.G_ImageId==-1:
                                self.G_PhrozenFluiddRespondInfo("[INFO] Image ID==-1, default: ARCO300-MKS-RK3328-STM32F407VET6-I16")
                                # lancaigang240530: versionwrite todat text file; DriveCodeJson.dat
                                filename='/home/mks/hdlDat/DriveCodeFile.dat'
                            else:
                                self.G_PhrozenFluiddRespondInfo("[INFO] Image IDcould not be read, default: ARCO300-MKS-RK3328-STM32F407VET6-I16")
                                # lancaigang240530: versionwrite todat text file; DriveCodeJson.dat
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
                                    # 2 , 18 , 24053 , 18 , 0
                                    split[0]=split[0].strip()# move
                                    split[1]=split[1].strip()# file id
                                    split[2]=split[2].strip()# firmwareversion
                                    split[3]=split[3].strip()# id
                                    split[4]=split[4].strip()# is no in
                                    # split[4]='0'# is no in, default0
                                    # #if "SN1" in self.G_SerialRxASCIIStr:
                                    # if split[0] == "1":
                                    #     self.G_PhrozenFluiddRespondInfo(split[0])
                                    #     self.G_PhrozenFluiddRespondInfo(split[1])
                                    #     self.G_PhrozenFluiddRespondInfo(split[2])
                                    #     self.G_PhrozenFluiddRespondInfo(split[3])
                                    #     self.G_PhrozenFluiddRespondInfo(split[4])
                                    #     #line=("%d,%d,%d," % (HW_VERSION,,))
                                    #     line_modify=split[0]+','+'1'+','+self.G_SerialRxASCIIStr[9:14]+','+'1'+','+'1'
                                    #     self.G_PhrozenFluiddRespondInfo(line_modify)
                                    #     Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
                                    # else:
                                    #     Lo_AllLine=Lo_AllLine+line

                                    #if "SN2" in self.G_SerialRxASCIIStr:
                                    if split[0] == "2":
                                        self.G_PhrozenFluiddRespondInfo('[INFO] AMSunit 2 file')
                                        self.G_PhrozenFluiddRespondInfo(split[0])
                                        self.G_PhrozenFluiddRespondInfo(split[1])
                                        self.G_PhrozenFluiddRespondInfo(split[2])
                                        self.G_PhrozenFluiddRespondInfo(split[3])
                                        self.G_PhrozenFluiddRespondInfo(split[4])
                                        #line=("%d,%d,%d," % (HW_VERSION,,))
                                        line_modify=split[0]+','+'18'+','+self.G_SerialRxASCIIStr[11:16]+','+'18'+','+'1'
                                        self.G_PhrozenFluiddRespondInfo(line_modify)
                                        Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
                                    else:
                                        Lo_AllLine=Lo_AllLine+line

                                    # if "SN3" in self.G_SerialRxASCIIStr:
                                    #     if split[0] == "3":
                                    #         self.G_PhrozenFluiddRespondInfo(split[0])
                                    #         self.G_PhrozenFluiddRespondInfo(split[1])
                                    #         self.G_PhrozenFluiddRespondInfo(split[2])
                                    #         self.G_PhrozenFluiddRespondInfo(split[3])
                                    #         self.G_PhrozenFluiddRespondInfo(split[4])
                                    #         #line=("%d,%d,%d," % (HW_VERSION,,))
                                    #         line_modify=split[0]+','+'1'+','+self.G_SerialRxASCIIStr[9:14]+','+'1'+','+'1'
                                    #         self.G_PhrozenFluiddRespondInfo(line_modify)
                                    #         Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
                                    #     else:
                                    #         Lo_AllLine=Lo_AllLine+line
                                    # if "SN4" in self.G_SerialRxASCIIStr:
                                    #     if split[0] == "4":
                                    #         self.G_PhrozenFluiddRespondInfo(split[0])
                                    #         self.G_PhrozenFluiddRespondInfo(split[1])
                                    #         self.G_PhrozenFluiddRespondInfo(split[2])
                                    #         self.G_PhrozenFluiddRespondInfo(split[3])
                                    #         self.G_PhrozenFluiddRespondInfo(split[4])
                                    #         #line=("%d,%d,%d," % (HW_VERSION,,))
                                    #         line_modify=split[0]+','+'1'+','+self.G_SerialRxASCIIStr[9:14]+','+'1'+','+'1'
                                    #         self.G_PhrozenFluiddRespondInfo(line_modify)
                                    #         Lo_AllLine=Lo_AllLine+line_modify+"\r\n"#0x0D 0x0A
                                    #     else:
                                    #         Lo_AllLine=Lo_AllLine+line
                            #self.G_PhrozenFluiddRespondInfo(Lo_AllLine)
                            with open(filename,"w+") as file_w:
                                file_w.write(Lo_AllLine)


                        self.Device_TimmerUartRecvHandler(2,Lo_SerialRxBytes,self.G_SerialRxASCIIStr)

                except:
                    self.G_PhrozenFluiddRespondInfo('[INFO] serial port number data error, noneAMSstate')

            return eventtime + AMS_SERIALPORT_RECV_TIMER

        except Exception as e:
            self.G_PhrozenFluiddRespondInfo('[INFO] serial port 2 read get error, AMS2error or restart, please checkAMS2 is no normal normal')
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
            # lancaigang0427: ifAMSerrorrestart, need to, and restartsuccessfulsending commandstm32refill
            # lancaigang240427: AMSerrorrestart, need to
            self.G_AMS2ErrorRestartFlag = True
            self.G_AMS2ErrorRestartCount=self.G_AMS2ErrorRestartCount+1

            # lancaigang241011: serial port 2error, not allowed send number data
            self.G_SerialPort2OpenFlag=False
            
            # lancaigang240521: resumewhen, ifdetectedAMSerrorrestart, can is is AMS, unload filament
            self.G_ResumeCheckAMS2ErrorRestartFlag = True

            
            return eventtime + AMS_SERIALPORT_RECV_TIMER


    
    
    ####################################
    #Function Name: 
    #Input Parameters: 
    #Return Value:
    # Description: Lan Caigang-20230830
    ####################################
# lancaigang0914: cannot move position; calling PhrozenDev class
def load_config(config):
    return PhrozenDev(config)