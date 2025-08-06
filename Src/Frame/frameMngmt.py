"""
#  @file        main.py
#  @brief       Template_BriefDescription.
#  @details     TemplateDetailsDescription.\n
#
#  @author      mba
#  @date        jj/mm/yyyy
#  @version     1.0
"""
#------------------------------------------------------------------------------
#                                       IMPORT
#------------------------------------------------------------------------------
import sys, threading
import os, time
import json
import re
from typing import Dict, List, Optional
from queue import Queue, Empty

from Protocole.SerialMngmt import SerialMngmt, SerialError
#------------------------------------------------------------------------------
#                                       CONSTANT
#------------------------------------------------------------------------------
PATTERN_ENUM = r'(\d+)="([^"]+)"'
PATTERN_SIGNAL = re.compile(
    r"Sig=(\w+)\s+unsigned\s+(\d+)"                  # nom et longueur
    r"(?:\s+(-m))?"                                  # encodage
    r"(?:\s+/f:(\d+))?"                              # factor
    r"(?:\s+/o:(\d+))?"                              # offset
    r"(?:\s+/max:(\d+))?"                            # max (non utilisé ici mais capturé)
    r"(?:\s+/e:(\w+))?"                              # enum
)

PATTERN_SYM_ID = re.compile(r'ID=([0-9A-Fa-f]+)h\s*//\s*(\w+)')
PATTERN_SYM_LEN = re.compile(r'Len=(\d+)')
PATTERN_SYM_SIG = re.compile(r'Sig=(\w+)\s+(\d+)')

MSG_TYPE_MAPPING = {
    'RECEIVE' : 'APPSIG_MSG_DIR_RX',
    'SEND' : 'APPSIG_MSG_DIR_TX',
    'SENDRECEIVE' : 'APPSIG_MSG_DIR_RX_TX',
}
# CAUTION : Automatic generated code section: Start #

# CAUTION : Automatic generated code section: End #
#------------------------------------------------------------------------------
#                                       CLASS
#------------------------------------------------------------------------------
class IdxSignal:
    lenght = 0
    enumlink = 1

class FrameMngmt():
    def __init__(self, f_prjcfg_file:str):
        
        if not os.path.isfile(f_prjcfg_file):
            raise FileNotFoundError(f'Signal Config file doest not exits {f_prjcfg_file}')
        
        with open(f_prjcfg_file, "r") as file:
            prj_cfg_data = json.load(file)
        
        try:
            self.sigcfg_file = prj_cfg_data["signal_cfg"]
            baudrate:int = prj_cfg_data["serial_cfg"]["baudrate"]
            protcom:str = prj_cfg_data["serial_cfg"]["port_com"]
            self._is_serial_enable:bool = prj_cfg_data["serial_cfg"]["is_enable"]
            self._srl_frame_len:int = prj_cfg_data["serial_cfg"]["frame_len"]

        except (KeyError, TypeError) as e:
            raise Exception(f'An error occured while extracting config project -> {e}')
        
        # serial managment #
        self._serial_istc = SerialMngmt(baudrate, protcom, self.__error_serial_cb)
        self.sigcfg_file

        # signal maangment
        self.enum:Dict[str, List[List[int]]] = {}
        self.signal:Dict[str, Dict] = {}
        self.sig_value:Dict[str, Queue] = {}
        self.symbol:Dict[str, Dict] = {}
        self.list_id = {
            'SRL' : [],
            'CAN' : []
        }

        # thread maangment 
        self._srl_frame_thread: Optional[threading.Thread] = None
        self._stop_srl_thread = threading.Event()

        #---- extract signals enum and stuff ----#
        self.__extract_signal_cfg()

    #--------------------------
    # get_signal_value
    #--------------------------
    def get_signal_value(self, f_signal: str) -> List[int]:
        """Get the Queue of values for a given signal.
        Args:
            f_signal (str): the name of the signal.
        Returns:
            List: a list of x element containing rawValue and ValueCompute [[1,4, timestamp], [5,20, timestamps]].
        Raises:
            KeyError: if the signal is not found.
        """
        result = [[]]
        sig_queue = self.sig_value.get(f_signal)
        if sig_queue is None:
            raise KeyError(f"{f_signal} does not exist in Signal Configuration")
        
        # Vide la queue
        try:
            while True:
                # get_nowait lève Empty si la queue est vide
                item = sig_queue.get_nowait()
                result.append(item)
        except Empty:
            pass
        
        return result
    #--------------------------
    # get_signal_value
    #--------------------------
    def get_signal_list(self) -> List[str]:
        """Get a signal value

        Args:
            f_signal (str): the signal
        Returns:
            List[str]: the list with all signals
        """
        
        return [str(signal_name) for signal_name in self.signal.keys()]

    def perform_cyclic(self)->None:
        """Start opening serial & can gate (depending on configuration)
        extract raw bufer from can & serial and interpret data to put it 
        into sig_values
        """
        if self._is_serial_enable:
            self._serial_istc.open_serial_line()
            self._serial_istc.configure_reception(f_nbByte=self._srl_frame_len)
            self._stop_srl_thread.clear()
            self._srl_frame_thread = threading.Thread(target=self._cyclic_serial_frame, daemon=True)
            self._srl_frame_thread.start()

        # idem can
    
    #--------------------------
    # unperform_cyclic
    #--------------------------
    def unperform_cyclic(self)->None:
        """Unperform cyclic frame analyzer
        """
        self._serial_istc.stop()
        self._stop_srl_thread.set()
    #--------------------------
    # __interpret_frame
    #--------------------------
    def _cyclic_serial_frame(self)->None:
        """Interpret a frame and put the value into signals

        Args:
            bytes (bytes): frame bytes
            len_frame (int):len of the frame

        Raises:
        """
        while not self._stop_srl_thread.is_set():
            srl_frame = self._serial_istc.get_frame()
            if srl_frame is not None:
                self.__decode_srl_frame(srl_frame)
            time.sleep(0.01)
                

    #--------------------------
    # __decode_srl_frame
    #--------------------------
    def __decode_srl_frame(self, f_srl_frame:bytes)->None:
        """Interpret a serial frame into signals value
        Args: 
            f_srl_frame (bytes); the frame to decode
        Raises:
        """
        current_time = time.time_ns()
        if len(f_srl_frame) < self._srl_frame_len:
            print("[ERROR] : Trame trop courte")
            return

        # 3e octet = id (en hex string pour correspondre à msg_id dans symbol)
        msg_id = f"{f_srl_frame[2]:03X}"  # ex: '010' ou '020'

        # Recherche du symbole correspondant à msg_id
        symbol = None
        for sym_name, sym in self.symbol.items():
            if sym['msg_id'] == msg_id:
                symbol = sym
                break

        if symbol is None:
            print(f"[ERROR] : Symbole inconnu pour msg_id {msg_id}")
            return

        signals:Dict = symbol['signals']  # dict signal_name -> bit position

        for signal_name, start_bit in signals.items():
            sig_conf = self.signal.get(signal_name)
            if not sig_conf:
                print(f"[ERROR] : Signal {signal_name} not configured")
                continue

            length = sig_conf['length']
            encoding = sig_conf['encoding']
            factor = sig_conf.get('factor', 1)
            offset = sig_conf.get('offset', 0)
            enum_name = sig_conf.get('enum')

            # Extraire la valeur brute du signal (bitfield)
            raw_value = self.__extract_bits(f_srl_frame[3:], start_bit, length, encoding)

            # Si enum est défini, traduire la valeur
            if enum_name and enum_name in self.enum:
                enum_map = {entry[0]: entry[1] for entry in self.enum[enum_name]}
                value = enum_map.get(raw_value, raw_value)  # sinon valeur brute

            else:
                # Appliquer facteur et offset
                value = raw_value * factor + offset

            # Stocker la valeur dans la queue associée
            if signal_name not in self.sig_value:
                self.sig_value[signal_name] = Queue()
            self.sig_value[signal_name].put([raw_value, value, current_time])

    #--------------------------
    # __extract_bits
    #--------------------------
    def __extract_bits(self, data: bytes, start_bit: int, length: int, encoding: str) -> int:
        """
        Extrait un champ de bits d'une trame en fonction de son start_bit et de sa longueur.

        Args:
            data (bytes): trame binaire (ex: 8 octets CAN/SRL)
            start_bit (int): position du 1er bit LSB dans le champ (en bits)
            length (int): nombre total de bits à extraire
            encoding (str): 'INTEL' (little endian) ou 'MOTOROLA' (big endian)

        Returns:
            int: valeur entière du champ extrait
        """
        bit_val = 0
        if len(data) != 8:
            raise ValueError(f'Error Expected 8 bytes of data')
        
        for i in range(length):
            if encoding.upper() == "INTEL":
                msg_bit = start_bit + i
            elif encoding.upper() == "MOTOROLA":
                # Algorithme identique à celui de votre code C
                byte = start_bit // 8
                bit = start_bit % 8
                msg_bit = (byte * 8 + bit) - i
                msg_bit = ((7 - (msg_bit // 8)) * 8) + (msg_bit % 8)
            else:
                raise ValueError(f"Encodage non supporté: {encoding}")

            byte_index = msg_bit // 8
            bit_in_byte = msg_bit % 8

            if byte_index >= len(data):
                break  # dépassement de la trame -> ignorer

            bit = (data[byte_index] >> bit_in_byte) & 0x1
            bit_val |= (bit << i)

        return bit_val
    #--------------------------
    # __interpret_frame
    #--------------------------
    def __error_serial_cb(self, f_type_error:SerialError):
        """Management of serial line whenever an error occured
        """
        if f_type_error == SerialError.SerialErrorLost:
            self._stop_srl_thread.set()
            print("[ERROR] : Stopping serial thread in FrameMngmt")
        else:
            print('[WARNING] : Timeout occured in SerialMngmt, did not receive any frame...')
    #--------------------------
    # __extract_signal_cfg
    #--------------------------
    def __extract_signal_cfg(self):
        """
            @brief get enum, signal, symbol from .sym 
        """
        current_read = 'NONE'
        waiting_for_timeout = False
        current_id = None
        current_type = None
        current_len = None
        current_symbol = None

        with open(self.sigcfg_file, 'r') as file:
            file_iter = iter(file)
            for line in file_iter:
                line = line.strip()

                if "ENUMS" in line:
                    current_read = 'ENUMS'
                    continue
                elif "SIGNALS" in line:
                    current_read = 'SIGNALS'
                    continue
                elif "SENDRECEIVE" in line:
                    current_read = 'SENDRECEIVE'
                    continue
                elif "RECEIVE" in line:
                    current_read = 'RECEIVE'
                    continue
                elif "SEND" in line:
                    current_read = 'SEND'
                    continue

                match current_read:
                    case 'ENUMS':
                        if 'Enum=' in line:
                            # Étape 1 : récupérer la ligne complète entre parenthèses
                            full_line = line.strip()
                            while ')' not in full_line:
                                next_line = next(file_iter).strip()  # file_iter = iter(fichier_lignes)
                                full_line += ' ' + next_line

                            # Étape 2 : extraire le nom de l'enum
                            start_index = full_line.index('Enum=') + len('Enum=')
                            end_index = full_line.index('(')
                            enum_name = full_line[start_index:end_index].strip()

                            # Étape 3 : extraire les paires index = "value"
                            resultats = re.findall(PATTERN_ENUM, full_line)
                            pairs = [[int(index), value] for index, value in resultats]

                            # Étape 4 : stocker
                            self.enum[enum_name] = pairs

                    case 'SIGNALS':
                        match = PATTERN_SIGNAL.match(line)
                        if match:
                            nom_signal    = match.group(1)
                            longueur      = int(match.group(2))
                            encoding_flag = match.group(3)
                            factor        = int(match.group(4)) if match.group(4) else 1
                            offset        = int(match.group(5)) if match.group(5) else 0
                            # match.group(6) = max (non utilisé ici)
                            enum_name     = match.group(7) if match.group(7) else None

                            encoding = "MOTOROLA" if encoding_flag else "INTEL"
                            self.sig_value[nom_signal] = Queue()
                            self.signal[nom_signal] = {
                                'length': longueur,
                                'encoding': encoding,
                                'factor': factor,
                                'offset': offset,
                                'enum': enum_name
                            }
                        else:
                            print(f'[INFO] : APPSIG_Codegen : While in SIGNALS, no signal pattern in line: {line.strip()}')

                    case 'SEND' | 'RECEIVE' | 'SENDRECEIVE':
                        if line.startswith('['):  # Ex: [Symbol1]
                            if waiting_for_timeout == True and current_read != 'SEND':
                                raise ValueError(f"Missing Timeout for symbol {current_symbol}")

                            current_symbol = line.strip().strip('[]')
                            self.symbol[current_symbol] = {
                                'msg_id': None,
                                'msg_len': None,
                                'msg_type': None,
                                'msg_direction': current_read,
                                'signals': {},
                                'timeout': 0,
                                'cycle_time': None  # <-- Ajouté ici
                            }
                            waiting_for_timeout = True
                            continue

                        match_id = PATTERN_SYM_ID.match(line)
                        if match_id:
                            current_id = match_id.group(1)
                            current_type = match_id.group(2)

                            if current_type not in self.list_id:
                                self.list_id[current_type] = []

                            if current_id in self.list_id[current_type]:
                                raise ValueError(f'{current_id} already used in msg type {current_type}')
                            else:
                                self.list_id[current_type].append(current_id)

                            if current_symbol:
                                self.symbol[current_symbol]['msg_id'] = current_id
                                self.symbol[current_symbol]['msg_type'] = current_type
                            continue

                        match_len = PATTERN_SYM_LEN.match(line)
                        if match_len:
                            current_len = int(match_len.group(1))
                            if current_symbol:
                                self.symbol[current_symbol]['msg_len'] = current_len
                            continue

                        # Nouveau bloc : Timeout
                        if line.strip().lower().startswith("timeout="):
                            timeout_val = int(line.strip().split("=")[1].strip())
                            if current_symbol:
                                if timeout_val == 0:
                                    raise ValueError(f"Timeout cannot be 0 for symbol '{current_symbol}'")
                                self.symbol[current_symbol]['timeout'] = timeout_val
                                waiting_for_timeout = False
                            continue

                        # Nouveau bloc : CycleTime
                        if line.strip().lower().startswith("cycletime="):
                            cycle_val = int(line.strip().split("=")[1].strip())
                            if current_symbol and (current_read == 'SEND' or current_read == 'SENDRECEIVE') :
                                if cycle_val == 0:
                                    raise ValueError(f"CycleTime cannot be 0 for symbol '{current_symbol}'")
                                self.symbol[current_symbol]['cycle_time'] = cycle_val
                            continue

                        # Ligne signal
                        match_sig = PATTERN_SYM_SIG.match(line)
                        if match_sig:
                            signal_name = match_sig.group(1)
                            position = int(match_sig.group(2))

                            if current_symbol:
                                # Vérifier si le signal est bien défini
                                if signal_name not in self.signal:
                                    raise ValueError(f"Signal '{signal_name}' utilisé par '{current_symbol}' non défini dans SIGNALS")

                                new_start = position
                                new_length = self.signal[signal_name]['length']

                                for existing_signal, existing_start in self.symbol[current_symbol]['signals'].items():
                                    existing_length = self.signal[existing_signal]['length']

                                    new_end = new_start + new_length - 1
                                    existing_end = existing_start + existing_length - 1

                                    if not (new_end < existing_start or existing_end < new_start):
                                        raise ValueError(
                                            f"Conflit dans '{current_symbol}': signal '{signal_name}' (bits {new_start}-{new_end}) "
                                            f"chevauche '{existing_signal}' (bits {existing_start}-{existing_end})"
                                        )

                                # Pas de conflit, on ajoute le signal
                                self.symbol[current_symbol]['signals'][signal_name] = position

                            continue

            if current_symbol:
                sym = self.symbol[current_symbol]
                if sym['msg_direction'] == 'RECEIVE' or sym['msg_direction'] == 'SENDRECEIVE':
                    if sym['timeout'] is None:
                        raise ValueError(f"Missing Timeout for last symbol '{current_symbol}'")
                if sym['msg_direction'] == 'SEND' or sym['msg_direction'] == 'SENDRECEIVE':
                    if sym['cycle_time'] is None:
                        raise ValueError(f"Missing CycleTime for last symbol '{current_symbol}'")
    
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

