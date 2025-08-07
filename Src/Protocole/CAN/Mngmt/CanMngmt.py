#-------------------------------------------------------------------
# Copyright (C) 2017 - KUHN SA - Electronics
# 
# This document is KUHN SA property.
# It should not be reproduced in any medium or used in any way
# without prior written consent of KUHN SA
#-------------------------------------------------------------------
""" 
 
     
    @file        CanMngmt.py
    @brief       Offer an abtraction between PCAN library
                 and the bench test
    @details     .\n

    @author      AUDMBA
    @date        21/02/2024
    @version     1.0
"""

#-------------------------------------------------------------------
#                     Import
#-------------------------------------------------------------------
import os
from enum import IntEnum
from datetime import datetime
from typing import List
from ..Drivers.Peak.Src.PCANBasic import *
from Library.ModuleLog import MngLogFile, log

import ctypes
from ctypes import Structure
from ctypes import (c_char, 
                    c_uint,
                    c_uint32,
                    c_int8,
                    c_uint16,
                    c_uint8,
                    c_char_p,
                    c_ubyte,
                    POINTER,
                    c_int,
                    create_string_buffer)
import platform
import time
#-------------------------------------------------------------------
#                     Constants
#-------------------------------------------------------------------
# CAUTION : Automatic generated code section: Start #

# CAUTION : Automatic generated code section: End #
DEBUG_MODE = False

SIL_ECU_DLL_PATH = r'Src\Protocole\CAN\Drivers\SIL\Virtual_Ecu.dll'
#-------------------------------------------------------------------
#                     Class
#-------------------------------------------------------------------
class MsgType(IntEnum):
    CAN_MNGMT_MSG_STANDARD = 0
    CAN_MNGMT_MSG_RTR = 1
    CAN_MNGMT_MSG_EXTENDED = 2
    CAN_MNGMT_MSG_FD = 2
    CAN_MNGMT_MSG_BRS = 3
    CAN_MNGMT_MSG_ESI = 4
    CAN_MNGMT_MSG_ECHO = 5
    CAN_MNGMT_MSG_ERRFRAME = 6
    CAN_MNGMT_MSG_STATUS = 7

class StructCANMsg (Structure):
    """
    Represents a PCAN message
    """
    id:int 
    msgType:MsgType
    len:int
    data:List[int] = []


#------------------------
# CanMngmtOptionError
#------------------------
class CanMngmtOptionError(Exception):
    def __init__(self, f_msg_str) -> None:
        super().__init__(f_msg_str)

#------------------------
# CanModuleNotInitError
#------------------------
class CanModuleNotInitError(Exception):
    def __init__(self, f_msg_str) -> None:
        super().__init__(f_msg_str)
#------------------------
# CanMngmtOptionError
#------------------------
class DriverCanUsed(IntEnum):
    DrvPeak = 0
    DrvLibrary32bit = 1
#-------------------------------------------------------------------
#                   functions
#-------------------------------------------------------------------
#------------------------
# CanMngmt
#------------------------    
class CanMngmt():

    MC_DLC_8                = (0x08)
    def __init__(self, 
                 f_can_drv_used:DriverCanUsed,
                 f_can_baudrate:TPCANBaudrate, 
                 f_pcan_device, 
                 f_canlogging:bool = False, 
                 f_dirlog_path = "",
                 f_error_cb = None,
                 f_virtecu_node= None):
        """
            @brief      init of the module 
            @details 

            @param  f_can_baudrate_ui            : CAN baudrate  
            @param  f_canlogging_b         : Whether user xwants to record CAN msg (True) or not (False)
            @param  f_pcan_device          : The hardware device used
            @param  f_dirlog_path          : directory to set the log if needed
        """ 
        if not (isinstance(f_can_baudrate, TPCANBaudrate)
            or isinstance(f_canlogging,bool)):
            raise CanMngmtOptionError("Error, param init invalid")
        
        if f_canlogging == True and not os.path.isdir(f_dirlog_path):
            raise NotADirectoryError(f"{f_dirlog_path} is not a directory")
        # for can
        self._can_usb_channel = f_pcan_device
        
        self._can_baudrate:TPCANBaudrate = f_can_baudrate
        self._can_log_b:bool = f_canlogging
        self.can_drv_use = f_can_drv_used
        # initialize the right instance of hardware driver
        match (self.can_drv_use):

            case DriverCanUsed.DrvPeak:
                self._can_drv_peak = PCANBasic()

            case DriverCanUsed.DrvLibrary32bit:
                self._mdll_virt_ecu = ctypes.windll.LoadLibrary(SIL_ECU_DLL_PATH)
                self._mdll_virt_ecu.VirtEcu_Connect.argtypes = [c_char * 256, POINTER(c_int8), c_uint16]
                self._mdll_virt_ecu.VirtEcu_Connect.restype = c_int  # Assurez-vous que c_int correspond à t_eReturnCode
                appli_name_array = (c_char * 256)(*'Test_1'.encode('utf-8'))
                appli_handle = c_int8()
                dll_version = c_uint16(0x0601)
                
                retcode = self._mdll_virt_ecu.VirtEcu_Connect(appli_name_array, ctypes.byref(appli_handle), dll_version)

                if retcode != 0:
                    raise CanModuleNotInitError(f'Error while initialize VirtEcu -> {retcode}')

        # Flag management
        if DEBUG_MODE:
            self._can_initialized = True
        else:
            self._can_initialized:bool = False

        self._error_cb = f_error_cb
        if(self._can_log_b == True):
            self.make_canlog = MngLogFile(f_dirlog_path, "CanLogging.log",\
                                               log.DEBUG, "Can logging from M500_tester")
    
     #-------------------------
    # initiliaze_drv_peak
    #-------------------------
    def initiliaze_drv(self)->None:
        """
            @brief      Initialize communication
        """
        if not self._can_initialized:
            self._initialize_drv()
        return None
    
    #-------------------------
    # uninitialize_drv
    #-------------------------
    def uninitialize_drv(self)->None:
        """
            @brief      Uninitialize communication
        """
        # update flag
        if self._can_initialized and DEBUG_MODE == False:
            self._can_initialized = False
            

        return None
    
    #-------------------------
    # get_init_module_status
    #-------------------------
    def get_init_module_status(self)->bool:
        """
            @brief      Uninitialize communication with the M500_tester through Peak passerelle
        """
        return self._can_initialized
    #-------------------------
    # send_can
    #-------------------------
    def send_can(self, f_can_struct:StructCANMsg):
        """
            @brief      Send a CAN msg using PCAN_PEAK

            @param   f_can_struct : can structure to send
            @param   f_error_cb   : if problem happened callback function to user for dealing with error
        """
        if DEBUG_MODE:
            print(f"{hex(f_can_struct.ID)} : {tuple(f_can_struct.DATA)}")
            return
        
        if not hasattr(self, '_can_initialized'):
            raise CanModuleNotInitError("This module has not been initialized yet")
        
        if not self._can_initialized:
            self.error_cb_mngmt(PCAN_ERROR_INITIALIZE)
            raise ConnectionAbortedError("Can bus is not operational")
        
        if self._can_log_b:
            self.make_canlog.LCF_SetMsgLog(log.INFO, "Send : 0x%03X %02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X" \
                                            % (f_can_struct.ID , f_can_struct.DATA[0], 
                                                f_can_struct.DATA[1], f_can_struct.DATA[2], 
                                                f_can_struct.DATA[3], f_can_struct.DATA[4], 
                                                f_can_struct.DATA[5], f_can_struct.DATA[6], 
                                                f_can_struct.DATA[7] ))
            
        return self._send_can(f_can_struct)
    
    #-------------------------
    # read_can
    #-------------------------
    def read_can(self)-> StructCANMsg:
        """
            @brief      Send a CAN msg using PCAN_PEAK
            @details    
        """


        if DEBUG_MODE:
            return StructCANMsg()
        if not hasattr(self, '_can_initialized'):
            raise CanModuleNotInitError("This module has not been initialized yet")
        
        if not self._can_initialized:
            self.error_cb_mngmt(PCAN_ERROR_INITIALIZE)
            raise ConnectionAbortedError("Can bus is not operational")

        return self._read_can()

    #-------------------------
    # error_cb_mngmt
    #-------------------------
    def error_cb_mngmt(self, f_status)->None:
        """
            @brief      Uninitialize communication with the M500_tester through Peak passerelle

            @param f_status : status error of can bus
        """
        if self._error_cb is not None:
            self._error_cb(f_status) 

    #-------------------------
    # _send_can
    #-------------------------
    def _send_can(self, f_can_struct:StructCANMsg)->None:
        """
            @brief      Send a message CAN depending on driver/ library used

            @param f_status : status error of can bus
        """
        if not self._can_initialized:
            raise CanModuleNotInitError("Module CAN has not been Initialized yet")
        
        match self.can_drv_use:
            case DriverCanUsed.DrvPeak:
                #---- assemble the message ----#
                peak_can_struct = TPCANMsg()
                peak_can_struct.ID = f_can_struct.id
                peak_can_struct.LEN = f_can_struct.len
                peak_can_struct.MSGTYPE = self._get_peak_msg_type(f_can_struct.msgType)
                peak_can_struct.DATA = [f_can_struct.data[idx_data] for idx_data in range(0, f_can_struct.len)]
                status = self._can_drv_peak.Write(self._can_usb_channel, f_can_struct)

                if(status != PCAN_ERROR_OK):
                    if self._can_log_b:
                        self.make_canlog.LCF_SetMsgLog(log.ERROR, f"[_send_can] -> error occured while sending msg, statusCAN -> {status}, msg ->" \
                                                "0x%03X %02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X" \
                                                    % (f_can_struct.ID , f_can_struct.DATA[0], 
                                                        f_can_struct.DATA[1], f_can_struct.DATA[2], 
                                                        f_can_struct.DATA[3], f_can_struct.DATA[4], 
                                                        f_can_struct.DATA[5], f_can_struct.DATA[6], 
                                                        f_can_struct.DATA[7] ))
                    self.error_cb_mngmt(status)

            case DriverCanUsed.DrvLibrary32bit:
                raise NotImplementedError
            
    #-------------------------
    # _read_can
    #-------------------------
    def _read_can(self)->StructCANMsg:
        """
            @brief      Read one message from buffer CAN depending on driver/ library used

            @param f_status : status error of can bus
        """
        
        can_struct:StructCANMsg = StructCANMsg()
        can_timestamp:TPCANTimestamp
        match self.can_drv_use:
            case DriverCanUsed.DrvPeak:
                can_status:TPCANStatus
                can_timestamp:TPCANTimestamp
                checksum_ui = 0
                peak_can_struct:TPCANMsg
                can_status, peak_can_struct,can_timestamp = self._can_drv_peak.Read(self._can_usb_channel)
                if can_status == PCAN_ERROR_OK:
                    for index in range(0,self.MC_DLC_8):
                        checksum_ui += peak_can_struct.DATA[index]

                    if(checksum_ui != 0 and checksum_ui != 8):
                        # Manage CAN logging
                        if(self._can_log_b == True):
                            self.make_canlog.LCF_SetMsgLog(log.INFO,"Rcv  : 0x%03X %02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X" \
                                                            % (peak_can_struct.ID     , peak_can_struct.DATA[0],             
                                                                peak_can_struct.DATA[1], peak_can_struct.DATA[2], 
                                                                peak_can_struct.DATA[3], peak_can_struct.DATA[4], 
                                                                peak_can_struct.DATA[5], peak_can_struct.DATA[6], 
                                                                peak_can_struct.DATA[7] ))
                    #---- copy data ----#
                    can_struct.ID = peak_can_struct.ID

                    can_struct.id = peak_can_struct.ID
                    can_struct.len = peak_can_struct.LEN
                    can_struct.msgType = peak_can_struct.MSGTYPE
                    can_struct.data = [int(peak_can_struct.DATA[idx_data]) for idx_data in range(0, can_struct.len)]
                elif can_status == PCAN_ERROR_QRCVEMPTY:
                    pass # oK
                else:
                    if self._can_log_b:
                        self.make_canlog.LCF_SetMsgLog(log.WARNING, f"An error occured while reading CAN msg -> {can_status}")

            case DriverCanUsed.DrvLibrary32bit:
                raise NotImplementedError
            
        return can_struct
    
    #-------------------------
    # _initialize_drv
    #-------------------------
    def _initialize_drv(self):
        """
            @brief Initialize the right hardware driver depending on self.can_drv_use
        """
        match (self.can_drv_use):
            case DriverCanUsed.DrvPeak:
                status_can_bus = self._can_drv_peak.Initialize(self._can_usb_channel, self._can_baudrate)
                if(status_can_bus != PCAN_ERROR_OK):
                    raise ConnectionRefusedError(f"Can init failed -> StatusError : {status_can_bus}, Check PCANBasic.py for reference")
                else:
                    self._can_initialized = True
            
            case DriverCanUsed.DrvLibrary32bit:
                retcode = self._mdll_virt_ecu.VirtEcu_CanConfig(c_int8(ecu_handle),c_uint8(f_node_used) , c_uint8(f_driverused))

        return retcode
            
    #-------------------------
    # _uninitialize_drv
    #-------------------------
    def _uninitialize_drv(self):
        """
            @brief Unnitialize the right hardware driver depending on self.can_drv_use
        """
        match (self.can_drv_use):

            case DriverCanUsed.DrvPeak:
                can_status = self._can_drv_peak.Uninitialize(self._can_usb_channel)
                if can_status != PCAN_ERROR_OK:
                    print(f"unable to Uninitilaize pcan driver, status code -> {can_status}")
            
            case DriverCanUsed.DrvLibrary32bit:
                raise NotImplementedError
            
    #-------------------------
    # _get_peak_msg_type
    #-------------------------
    def _get_peak_msg_type(self, f_msg_type:MsgType)->TPCANMessageType:
        """
            @brief Get TPCANMsgType from absqtraction
        """ 
        match f_msg_type.value:
            case MsgType.CAN_MNGMT_MSG_STANDARD.value:
                return PCAN_MESSAGE_STANDARD
            case MsgType.CAN_MNGMT_MSG_RTR.value:
                return PCAN_MESSAGE_RTR
            case MsgType.CAN_MNGMT_MSG_EXTENDED.value:
                return PCAN_MESSAGE_EXTENDED
            case MsgType.CAN_MNGMT_MSG_FD.value:
                return PCAN_MESSAGE_FD
            case MsgType.CAN_MNGMT_MSG_BRS.value:
                return PCAN_MESSAGE_BRS
            case MsgType.CAN_MNGMT_MSG_ESI.value:
                return PCAN_MESSAGE_ESI
            case MsgType.CAN_MNGMT_MSG_ECHO.value:
                return PCAN_MESSAGE_ECHO
            case MsgType.CAN_MNGMT_MSG_ERRFRAME.value:
                return PCAN_MESSAGE_ERRFRAME
            case MsgType.CAN_MNGMT_MSG_STATUS.value:
                return PCAN_MESSAGE_STATUS
            case _: 
                #---- should not arrive here ----#
                return TPCANMessageType(PCAN_MESSAGE_STANDARD)

    #-------------------------
    # _get_peak_msg_type
    #-------------------------
    def get_msg_type_from_peak(self, peak_msg_type: TPCANMessageType) -> MsgType:
        """
        @brief Get MsgType abstraction from TPCANMessageType
        """
        if peak_msg_type == PCAN_MESSAGE_STANDARD:
            return MsgType.CAN_MNGMT_MSG_STANDARD
        elif peak_msg_type == PCAN_MESSAGE_RTR:
            return MsgType.CAN_MNGMT_MSG_RTR
        elif peak_msg_type == PCAN_MESSAGE_EXTENDED:
            return MsgType.CAN_MNGMT_MSG_EXTENDED
        elif peak_msg_type == PCAN_MESSAGE_FD:
            return MsgType.CAN_MNGMT_MSG_FD
        elif peak_msg_type == PCAN_MESSAGE_BRS:
            return MsgType.CAN_MNGMT_MSG_BRS
        elif peak_msg_type == PCAN_MESSAGE_ESI:
            return MsgType.CAN_MNGMT_MSG_ESI
        elif peak_msg_type == PCAN_MESSAGE_ECHO:
            return MsgType.CAN_MNGMT_MSG_ECHO
        elif peak_msg_type == PCAN_MESSAGE_ERRFRAME:
            return MsgType.CAN_MNGMT_MSG_ERRFRAME
        elif peak_msg_type == PCAN_MESSAGE_STATUS:
            return MsgType.CAN_MNGMT_MSG_STATUS
        else:
            return MsgType.CAN_MNGMT_MSG_STANDARD








#-------------------------------------------------------------------
#                          End of file
#-------------------------------------------------------------------

#------------------------
# FuncName
#------------------------

"""
    @brief      
    @details

    @retval    
"""