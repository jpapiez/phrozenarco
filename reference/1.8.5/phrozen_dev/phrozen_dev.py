####################################
#项目名称：
#芯片类型: 
#功能: 
#研发人员：蓝才刚
#开发时间: 20230830
####################################

from operator import truediv
import os
import logging
import time
import struct
import serial
import numpy as np




AMS_MAX_CHANNEL = 32
DEFAULT_PORT = "/dev/ttyUSB0"
SERIAL_BAUD_RATE = 19200
FILA_RUNOUT_TIMER = 1.0

TOOLHEAD_ADC_REPORT_TIME = 0.015
TOOLHEAD_ADC_DEBOUNCE_TIME = 0.025
TOOLHEAD_ADC_SAMPLE_TIME = 0.001
TOOLHEAD_ADC_SAMPLE_COUNT = 4

####################################
#类名：
#功能描述：蓝才刚-20230830
####################################
class RunState:
    STANDBY = 0  # 待机阶段
    PREPARTION = 1  # 备料阶段
    CHANGING_P1 = 2  # 换料阶段1
    CHANGING_P2 = 3  # 换料阶段2
    FORCE_FEED = 4  # 强制补料
    PRINTING = 5  # 打印阶段(补料)
    ROLLBACK = 6  # 完全退料
    PARKBACK = 7  # 退料到停泊位
    PARKALL = 8  # 全部退料到停靠位
    CLEANING = 8  # 清空
    TIMEOUT_ERR = 9  # 系统超时出错状态
    FILAMENT_END_ERR = 10  # 系统断料出错状态

####################################
#类名：
#功能描述：蓝才刚-20230830
####################################
class PhrozenDev(object):
    # 接收的命令:
    # T1 ~ T5  --> 切换通道
    # B1 ~ B5  --> 通道退料
    # P1 ~ P5  --> 线料停在停靠位
    # AP       --> 所有线料全部停靠
    # SS       --> 简要状态
    # DS       --> 详细状态
    # RD       --> 进入准备状态
    # SP       --> 紧急停止
    # CL       --> 清空所有通道
    # MC       --> 多色工作模式
    # AC       --> 续料工作模式

    CMD_PARK_ALL = "AP"
    CMD_SIMPLE_STATE = "SB"
    CMD_DETAIL_STATE = "SD"
    CMD_TO_READY_STATE = "RD"
    CMD_TO_STOP_STATE = "SP"
    CMD_CLEAN_ALL_CHANS = "CL"
    CMD_COLOR_WORK_MODE = "MC"
    CMD_FEED_WORK_MODE = "MA"
    CMD_FEED_APPEND = "FA"

    CUT_X_POS = 297  # 切线前的X轴坐标值
    CUT_Y_POS = 150  # 切线前的Y轴坐标值
    ####################################
    #函数名称：
    #输入参数：
    #返 回 值:
    #功能描述：蓝才刚-20230830
    ####################################
    def __init__(self, config):
        #命令锁令牌
        self.G_AMSSerialCmdLock = False
        #默认喷头没有线材
        self.G_ToolheadIfHaveFilaFlag = False

        # 断线处理定时器
        self.G_FilaRunoutTimmer = None
        # 串口接收定时器
        self.G_AMSSerialRecvTimmer = None
        
        #printer.cfg配置文件
        self.G_PhrozenConfig = config
        self.G_PhrozenPrinter = config.get_printer()
        self.G_PhrozenReactor = self.G_PhrozenPrinter.get_reactor()
        #暂停恢复命令
        self.G_PhrozenPrinterCancelPauseResume = self.G_PhrozenPrinter.load_object(config, "pause_resume")
        #gcode命令
        self.G_PhrozenGCode = self.G_PhrozenPrinter.lookup_object("gcode")
        #响应fluidd信息
        self.G_PhrozenFluiddRespondInfo = self.G_PhrozenGCode.respond_info

        #AMS多色状态
        self.G_AMSDeviceState = {}
        #usb转ttl串口
        self.G_PhrozenSerialport = self.G_PhrozenConfig.get("dev_port", None)
        #printer.cfg;是否自动连接
        self.G_AMSIfAutoConnectFlag = self.G_PhrozenConfig.getboolean("auto_connect", None)
        #printer.cfg;切线默认x位置
        self.G_AMSFilaCutXPosition = self.G_PhrozenConfig.getfloat("fila_cut_x_pos", None)
        #printer.cfg;切线默认z抬升位置
        self.G_AMSFilaCutZPositionLiftingUp = self.G_PhrozenConfig.getfloat("fila_cut_x_pos_up", None)
        #printer.cfg;默认喷头线材插入ADC值
        self.G_ToolheadFilaExistAdcValueDefault = self.G_PhrozenConfig.getfloat("fila_exist_value", None)
        #printer.cfg;默认喷头线材空ADC值
        self.G_ToolheadFilaEmptyAdcValueDefault = self.G_PhrozenConfig.getfloat("fila_empty_value", None)
        #printer.cfg;喷头线材ADC阈值
        self.G_ToolheadFilaAdcThresholdValue = (self.G_ToolheadFilaExistAdcValueDefault + self.G_ToolheadFilaEmptyAdcValueDefault) / 2.  # ADC电压基准
        #printer.cfg;喷头ADC检测引脚
        self.G_ToolheadFilaSensorPin = self.G_PhrozenConfig.get("fila_sensor_pin", None)
        
        # printer.cfg;换线等待喷头最大移动速度
        self.G_ChangeChannelWaitMaxMovementSpeed = self.G_PhrozenConfig.getint("wait_max_velocity", None)
        #printer.cfg;换线等待划线宽度
        self.G_ChangeChannelWaitLineWidth = self.G_PhrozenConfig.getfloat("wait_line_width", None)
        #printer.cfg;换线超时时间，可通过printer.cfg文件设置或默认时间
        self.G_ChangeChannelTimeout = self.G_PhrozenConfig.getint("wait_timeout", None)
        #printer.cfg;换线是否由gcode代码抬升z轴高度
        self.G_ChangeChannelIfZLiftingUpByGcode = self.G_PhrozenConfig.getint("switch_fila_zup_by_gcode", None)


        # dev.py；测试命令PRZ_TEST
        self.G_PhrozenGCode.register_command("PRZ_TEST", self.Device_CmdPhrozenTest, desc="Phrozen多色机测试命令")

    ####################################
    #函数名称：
    #输入参数：
    #返 回 值:
    #功能描述：蓝才刚-20230830
    ####################################
    def Device_CmdPhrozenTest(self, gcmd):
        gcmd.respond_info("[(phrozen_dev.python)Device_CmdPhrozenTest]gcmd.prz_test测试='%s'" % (gcmd.get_commandline(),))
        self.G_PhrozenFluiddRespondInfo("[(phrozen_dev.python)Device_CmdPhrozenTest]self.prz_test测试='%s'" % (gcmd.get_commandline(),))

    ####################################
    #函数名称：
    #输入参数：
    #返 回 值: 
    #功能描述：蓝才刚-20230830
    ####################################
def load_config(config):
    return PhrozenDev(config)
