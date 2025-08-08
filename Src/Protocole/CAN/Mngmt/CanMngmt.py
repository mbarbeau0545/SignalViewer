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
from typing import List, Optional
from ..Drivers.Peak.Src.PCANBasic import *
from Library.ModuleLog import MngLogFile, log
from dataclasses import dataclass, field
from abc import ABC, abstractmethod
from threading import Thread,Event
from queue import Queue, Empty
import ctypes
from ctypes import Structure
from ctypes import (c_char, 
                    c_uint,
                    c_uint32,
                    c_int8,
                    c_int32,
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

SIL_ECU_DLL_PATH = r'Src\Protocole\CAN\Drivers\SIL\VirtualCanBus_NetWrapper.dll'


#-------------------------------------------------------------------
#                     Function
#-------------------------------------------------------------------
def validate_config(config_class, kwargs):
    try:
        return config_class(**kwargs)
    except TypeError as e:
        raise ValueError(f"Arguments invalides pour {config_class.__name__}: {e}")


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

@dataclass
class StructCANMsg:
    id: int = 0
    msgType: MsgType = MsgType.CAN_MNGMT_MSG_STANDARD
    length: int = 0
    data: List[int] = field(default_factory=list)
    timestamp:int = 0



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
# DriverCanUsed
#------------------------
class DriverCanUsed(IntEnum):
    DrvPeak = 0

#------------------------
# CanMngmtOptionError
#------------------------
@dataclass
class PeakCanConfig:
    pcan_usb: TPCANHandle
    pcan_baudrate: TPCANBaudrate

#------------------------
# CanMngmtOptionError
#------------------------
@dataclass
class VirtCanConfig:
    node: int # 4 virtual can bus from 0-3

#------------------------
# CANInterface
#------------------------
class CANInterface(ABC):
    
    _MC_DLC_8 = 8

    def __init__(self, **kwargs) -> None:
        super().__init__()
        self.enable_log = kwargs.get('canlogging', False)
        self.error_cb_mngmt = kwargs.get('error_cb', self.default_error_handler)
        self.is_init = False
        self._receive_queue: Queue = Queue()
        self._rx_thread: Optional[Thread] = None
        self._stop_rx_thread = Event()
        dir_log_path = kwargs.get('dir_log_path', '')
        if self.enable_log == True:
            if not os.path.isdir(dir_log_path):
                raise NotADirectoryError(f"{dir_log_path} is not a directory")

            self.make_log = MngLogFile(dir_log_path, "CanLogging.log",\
                                                log.DEBUG, "Can logging from M500_tester")  
    #------------------------
    # @connect
    #------------------------          
    @abstractmethod
    def connect(self, **kwargs):
        pass
    
    #------------------------
    # @disconnect
    #------------------------     
    @abstractmethod
    def disconnect(self):
        pass
    
    #------------------------
    # @send
    #------------------------     
    @abstractmethod
    def send(self, f_frame:StructCANMsg):
        pass

    #------------------------
    # @receive_poll
    #------------------------     
    @abstractmethod
    def receive_poll(self) ->StructCANMsg:
        pass
    
    #------------------------
    # receive_queue_start
    #------------------------     
    def receive_queue_start(self)->None:
        """
            @brief Get TPCANMsgType from absqtraction
        """ 
        if not self.is_init:
            raise CanModuleNotInitError('Instance Not Init, please use Connect Method first')

        self._stop_rx_thread.clear()
        self._rx_thread = Thread(target= self._can_reader_cyclic, daemon=True)
        self._rx_thread.start()

    #------------------------
    # receive_queue_stop
    #------------------------     
    def receive_queue_stop(self) ->None:
        return self._stop_rx_thread.set()
    
    #------------------------
    # get_can_frame
    #------------------------     
    def get_can_frame(self, f_timeout:float=0.0)->StructCANMsg:
        try:
            msg, timestamp = self._receive_queue.get(timeout=f_timeout)
            return StructCANMsg(msg.ID, MsgType.CAN_MNGMT_MSG_STANDARD, msg.LEN, msg.DATA, timestamp)
        except (Empty):
            return StructCANMsg()
    
    #------------------------
    # @_can_reader_cyclic
    #------------------------     
    @abstractmethod
    def _can_reader_cyclic(self):
        pass
        
    #------------------------
    # @flush
    #------------------------     
    @abstractmethod
    def flush(self)->None:
        pass

    #------------------------
    # default_error_handler
    #------------------------     
    def default_error_handler(self, status):
        print(f'[ERROR] : Can Error code -> {c_int32(status).value}')
    


class t_eReturnCode(IntEnum):
    # Errors
    RC_ERROR_PARAM_INVALID = -32767
    RC_ERROR_PARAM_NOT_SUPPORTED = -32766
    RC_ERROR_WRONG_STATE = -32765
    RC_ERROR_MODULE_NOT_INITIALIZED = -32764
    RC_ERROR_MISSING_CONFIG = -32763
    RC_ERROR_WRONG_CONFIG = -32762
    RC_ERROR_UNDEFINED = -32761
    RC_ERROR_NOT_SUPPORTED = -32760
    RC_ERROR_BUSY = -32759
    RC_ERROR_TIMEOUT = -32758
    RC_ERROR_NOT_ALLOWED = -32757
    RC_ERROR_WRONG_RESULT = -32756
    RC_ERROR_LIMIT_REACHED = -32755
    RC_ERROR_NOT_ENOUGH_MEMORY = -32754
    RC_ERROR_NOT_AVAILABLE = -32753
    RC_ERROR_CHECKSUM = -32752
    RC_ERROR_FAULT_DETECTED = -32751
    # OK
    RC_OK = 0

    # Warnings
    RC_WARNING_NO_OPERATION = 1
    RC_WARNING_BUSY = 2
    RC_WARNING_ALREADY_CONFIGURED = 3
    RC_WARNING_INITIALIZATION_PROBLEM = 4
    RC_WARNING_PENDING = 5
    RC_WARNING_VALUE_TRUNCATED = 6
    RC_WARNING_UNDEFINED = 7

#------------------------
# CanMngmt
#------------------------    
def get_can_interface(  f_can_drv_used:DriverCanUsed,
                        f_canlogging:bool = False, 
                        f_dirlog_path = "",
                        f_error_cb = None) ->CANInterface:
    """
        @brief      init of the module 
        @details 

    """ 
    arg_kwargs = {
        'canlogging' : f_canlogging, 
        'dir_log_path' : f_dirlog_path, 
        'error_cb' : f_error_cb
    }

    match f_can_drv_used:
        case DriverCanUsed.DrvPeak:
            return PeakCanMngmt(kwargs=arg_kwargs)    



#------------------------
# PeakCanMngmt
#------------------------
class PeakCanMngmt(CANInterface):
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.handle = PCANBasic()
        self.usb_bus: TPCANHandle

        

    #------------------------
    # @connect
    #------------------------
    def connect(self, **kwargs)-> None:
        """
            @brief Get TPCANMsgType from absqtraction
        """ 
        if self.handle is None:
            RuntimeError('Class not initized')

        config = validate_config(PeakCanConfig, kwargs)

        status_can_bus = self.handle.Initialize(config.pcan_usb, config.pcan_baudrate)

        if(status_can_bus != PCAN_ERROR_OK):
            raise ConnectionRefusedError(f"Can init failed -> StatusError : {status_can_bus}, Check PCANBasic.py for reference")
        else:
            self.usb_bus = config.pcan_usb
            self.is_init = True

    #------------------------
    # @disconnect
    #------------------------
    def disconnect(self) -> None:
        if self.handle is None:
            RuntimeError('Class not initized')

        if not self.is_init:
            raise CanModuleNotInitError('Instance Not Init, please use Connect Method first')
        
        self.handle.Uninitialize(self.usb_bus)
    #------------------------
    # @send
    #------------------------
    def send(self, f_frame:StructCANMsg):
        """
            @brief Get TPCANMsgType from absqtraction
        """ 
        if not self.is_init:
            raise CanModuleNotInitError('Instance Not Init, please use Connect Method first')
            
        peak_can_struct = TPCANMsg()
        peak_can_struct.ID = f_frame.id
        peak_can_struct.LEN = f_frame.length
        peak_can_struct.MSGTYPE = self._get_peak_msg_type(f_frame.msgType)
        peak_can_struct.DATA = [f_frame.data[idx_data] for idx_data in range(0, f_frame.length)]

        status = self.handle.Write(self.usb_bus, peak_can_struct)

        if(status != PCAN_ERROR_OK):
            if self.enable_log:
                self.make_log.LCF_SetMsgLog(log.ERROR, f"[_send_can] -> error occured while sending msg, statusCAN -> {status}, msg ->" \
                                        "0x%03X %02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X" \
                                            % (f_frame.id , f_frame.data[0], 
                                                f_frame.data[1], f_frame.data[2], 
                                                f_frame.data[3], f_frame.data[4], 
                                                f_frame.data[5], f_frame.data[6], 
                                                f_frame.data[7] ))
            self.error_cb_mngmt(status)

    #------------------------
    # @receive_poll
    #------------------------
    def receive_poll(self) ->StructCANMsg:
        """
            @brief Get TPCANMsgType from absqtraction
        """ 
        can_status:TPCANStatus
        can_struct:StructCANMsg = StructCANMsg()
        peak_can_struct:TPCANMsg
        checksum_ui = 0

        if not self.is_init:
            raise CanModuleNotInitError('Instance Not Init, please use Connect Method first')

        can_status, peak_can_struct, timestamp = self.handle.Read(self.usb_bus)

        if can_status == PCAN_ERROR_OK:
            for index in range(0,self._MC_DLC_8):
                checksum_ui += peak_can_struct.DATA[index]

            if(checksum_ui != 0 and checksum_ui != 8):
                # Manage CAN logging
                if(self.enable_log == True):
                    self.make_log.LCF_SetMsgLog(log.INFO,"Rcv  : 0x%03X %02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X" \
                                                    % (peak_can_struct.ID     , peak_can_struct.DATA[0],             
                                                        peak_can_struct.DATA[1], peak_can_struct.DATA[2], 
                                                        peak_can_struct.DATA[3], peak_can_struct.DATA[4], 
                                                        peak_can_struct.DATA[5], peak_can_struct.DATA[6], 
                                                        peak_can_struct.DATA[7] ))
            #---- copy data ----#
            can_struct = StructCANMsg(peak_can_struct.ID, MsgType.CAN_MNGMT_MSG_STANDARD, peak_can_struct.LEN, peak_can_struct.DATA, int(timestamp.millis))
        
        elif can_status == PCAN_ERROR_QRCVEMPTY:
            pass # OK 

        else:
                print(f"[ERROR] : an error occured in thread reading -> pcan status {self.handle.GetErrorText(can_status)}")
        return can_struct
    




    #------------------------
    # @flush
    #------------------------
    def flush(self) -> None:
        """
            @brief Get TPCANMsgType from absqtraction
        """ 
        if not self.is_init:
            raise CanModuleNotInitError('Instance Not Init, please use Connect Method first')

        self.handle.Reset(self.usb_bus)
    
    #------------------------
    # _can_reader_cyclic
    #------------------------
    def _can_reader_cyclic(self)->None:
        """
            @brief Get TPCANMsgType from absqtraction
        """ 
        while not self._stop_rx_thread.is_set():
            result, msg, timestamp = self.handle.Read(self.usb_bus)

            if result == PCAN_ERROR_OK:
                self._receive_queue.put((msg, timestamp))

            elif result == PCAN_ERROR_QRCVEMPTY:
                time.sleep(0.001) 

            else:                
                print(f"[ERROR] : an error occured in thread reading -> pcan status {self.handle.GetErrorText(result)}")
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