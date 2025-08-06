"""
#  @file        SerialMngmt.py
#  @brief       Manager Serial Communication.
#  @details     Send/Receive send frame.\n
#
#  @author      mba
#  @date        jj/mm/yyyy
#  @version     1.0
"""
#------------------------------------------------------------------------------
#                                       IMPORT
#------------------------------------------------------------------------------
import time
from queue import Queue, Empty
import serial
from typing import Optional
import threading
from enum import IntEnum

#------------------------------------------------------------------------------
#                                       CONSTANT
#------------------------------------------------------------------------------
START_BYTES = [0xAA, 0x55]  
SRL_TIMEOUT = 5
# CAUTION : Automatic generated code section: Start #

# CAUTION : Automatic generated code section: End #
#------------------------------------------------------------------------------
#                                       CLASS
#------------------------------------------------------------------------------
class SerialError(IntEnum):
    SerialErrorTimeout = 0 # no msg received after x ms
    SerialErrorLost = 1 # gate close unexpectedly

class SerialMngmt():
    def __init__(self, f_baudrate:int, f_port_com:str, f_srl_err_cb = None):

        self._baudrate = f_baudrate
        self._port_com:str = f_port_com
        self._serial: Optional[serial.Serial] = None
        self._frame_size : int = 0
        self._receive_queue: Queue = Queue()
        self._rx_thread: Optional[threading.Thread] = None
        self._stop_thread = threading.Event()
        self._buffer = bytearray()
        self.error_callback = f_srl_err_cb
        self._last_received_time = 0


    #--------------------------
    # send_serial
    #--------------------------
    def open_serial_line(self)->None:
        """Configure a serial line with PySerial library

        Args:
            f_baudrate (int): serial line baudrate
            f_prot (str); the COM port 
        """
        try:
            self._serial = serial.Serial(
                port=self._port_com,
                baudrate=self._baudrate,
                timeout=0.1
            )
        except Exception as e:
            raise Exception('Unable to start the serial line, error : %d',e)
    #--------------------------
    # send_serial
    #--------------------------
    def send_serial(self, f_frame:str)-> None:
        """Send a frame to the serial line

        Args:
            f_frame (str): the frame you want to send
        """
        if self._serial and self._serial.is_open:
            self._serial.write(f_frame.encode('utf-8'))
        else:
            raise RuntimeError("Serial port not configured or not open")

    #--------------------------
    # read_serial
    #--------------------------
    def read_serial(self)-> str:
        """Read Hardware buffer directly 

        Returns:
            str: the complete buffer
        """
        if self._serial and self._serial.in_waiting:
            return self._serial.read(self._serial.in_waiting).decode('utf-8')
        return ""

    #--------------------------
    # read_serial
    #--------------------------
    def configure_reception(self, f_nbByte:int)->None:
        """Configure the reception line 
            to make packet of f_nbByte in a queue
            contact 

        Args:
            f_nbByte (int): _description_
        """
        if not self._serial:
            raise RuntimeError("Serial port must be configured before reception")

        self._frame_size  = f_nbByte
        self._stop_thread.clear()
        self._rx_thread = threading.Thread(target=self.__perform_cyclic, daemon=True)
        self._rx_thread.start()
    
    #--------------------------
    # get_frame
    #--------------------------
    def get_frame(self, timeout: float = 0.0)->Optional[bytes]:
        """Get one frame from the Queue

        Returns:
            Optional[bytes]: the frame
        """
        try:
            return self._receive_queue.get(timeout=timeout)
        except Empty:
            return None
        

    #--------------------------
    # __perform_cyclic
    #--------------------------
    def __perform_cyclic(self):
        """Read f_nbByte from serial port and put it in queue continuously
        """  
        self._last_received_time = time.time()
        while not self._stop_thread.is_set():
            try:
                if self._serial.in_waiting:
                    data = self._serial.read(self._serial.in_waiting)
                    if data:
                        self._last_received_time = time.time()  # reset timer
                        self._buffer.extend(data)
                        self._extract_frames()
            except serial.SerialException as e:
                # Erreur liée au port série, probablement déconnexion
                print(f"[ERROR] Serial port error: {e}")
                # try to reconnect 
                self.__try_serial_reco()
            except Exception as e:
                # Autres erreurs inattendues, tu peux aussi les logguer
                print(f"[ERROR] Unexpected error: {e}")
                self._stop_thread.set()
            # petite pause pour ne pas saturer le CPU
            # Vérification du timeout
            if time.time() - self._last_received_time > SRL_TIMEOUT:
                print("[WARNING] No data received for 5 seconds.")
                self.error_callback(SerialError.SerialErrorTimeout)
                self._last_received_time = time.time()  # évite de spammer l'alerte
            time.sleep(0.01)

    #--------------------------
    # _find_start_bytes
    #--------------------------
    def _find_start_bytes(self):
        for i in range(len(self._buffer) - 1):
            if self._buffer[i] == START_BYTES[0] and self._buffer[i + 1] == START_BYTES[1]:
                return i
        return -1
    
    #--------------------------
    # _extract_frames
    #--------------------------
    def _extract_frames(self):
        while len(self._buffer) >= self._frame_size:
            # Cherche le start byte dans la fenêtre glissante
            start_index = self._find_start_bytes()
            if start_index == -1:
                self._buffer.clear()
                return
            elif start_index > 0:
                del self._buffer[:start_index]

            if len(self._buffer) >= self._frame_size:
                frame = self._buffer[:self._frame_size]
                self._receive_queue.put(bytes(frame))
                del self._buffer[:self._frame_size]
            else:
                break
    #--------------------------
    # stop
    #--------------------------
    def stop(self):
        """Stop the reception thread gracefully"""
        self._stop_thread.set()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join()
        if self._serial:
            self._serial.close()

    #--------------------------
    # __try_serial_reco
    #--------------------------
    def __try_serial_reco(self, max_retries=5):
        """When port close from the other side, try to reconnect else
        call user callback
        """
        print(f"[ERROR] Serial port error")
        self._serial.close()  # fermer proprement
        reconnect_delay = 5  # secondes
        print(f"Reconnexion dans {reconnect_delay}s...")
        for _ in range(reconnect_delay * 10):
            if self._stop_thread.is_set():
                return
            time.sleep(0.1)

        attempt = 0
        while not self._stop_thread.is_set() and attempt < max_retries:
            try:
                self._serial.open()  # tente de rouvrir le port
                print("Port série reconnecté avec succès")
                return
            except serial.SerialException:
                attempt += 1
                print(f"Reconnexion échouée (tentative {attempt}/{max_retries}), nouvelle tentative dans 2s...")
                time.sleep(2)

        # Si on arrive ici, c’est que la reconnexion a échoué max_retries fois
        self._stop_thread.set()
        print("Échec de reconnexion après plusieurs tentatives.")
        if self.error_callback:
            self.error_callback(SerialError.SerialErrorLost)


#------------------------------------------------------------------------------
#                             FUNCTION IMPLMENTATION
#------------------------------------------------------------------------------

#------------------------------------------------------------------------------
#			                MAIN
#------------------------------------------------------------------------------

#------------------------------------------------------------------------------
#		                    END OF FILE
#------------------------------------------------------------------------------
#--------------------------
# Function_name
#--------------------------

"""
    @brief
    @details

    @params[in]
    @params[out]
    @retval
"""

