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


import importlib
from Protocole.CAN.Mngmt.CanMngmt import get_can_interface, DriverCanUsed
from Protocole.CAN.Mngmt.AbstractCAN import StructCANMsg, CanMngmtError
from Protocole.SERIAL.SerialMngmt import SerialMngmt, SerialError
#------------------------------------------------------------------------------
#                                       CONSTANT
#------------------------------------------------------------------------------
SYM_PATTERN_ENUM = r'(\d+)="([^"]+)"'
PATTERN_SIGNAL = re.compile(
    r"Sig=(\w+)\s+unsigned\s+(\d+)"                  # nom et len
    r"(?:\s+(-m))?"                                  # encodage
    r"(?:\s+/f:(\d+))?"                              # factor
    r"(?:\s+/o:(\d+))?"                              # offset
    r"(?:\s+/max:(\d+))?"                            # max (non utilisé ici mais capturé)
    r"(?:\s+/e:(\w+))?"                              # enum
)

SYM_PATTERN_ID = re.compile(r'ID=([0-9A-Fa-f]+)h\s*//\s*(\w+)')
SYM_PATTERN_LEN = re.compile(r'Len=(\d+)')
SYM_PATTERN_SIG = re.compile(r'Sig=(\w+)\s+(\d+)')

DBC_SYM_PATTERN = re.compile(r'^BO_\s+(\d+)\s+(\w+):\s+(\d+)\s+\S+')
DBC_SIG_PATTERN = re.compile(r'^SG_\s+(\w+)\s+(\w+)\s*:\s*(\d+)\|(\d+)@(\d)([+-])\s+\(([^,]+),([^)]+)\)\s+\[([^|]+)\|([^\]]+)\]\s+"([^"]*)"\s+(\S+)$')
DBC_ENM_PATTERN = re.compile(r'^VAL_TABLE_\s+(\w+)\s+(\d+)\s+(.*);$')
DBC_ENM_VAL_PATTERN = re.compile(r'(\d+)\s+"([^"]+)"')
DBV_ENM_AFECT_PATTERN = re.compile(r'^VAL_\s+\d+\s+(\w+)\s+(\d+)')

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

        pcan_module = importlib.import_module("Protocole.CAN.Drivers.Peak.Src.PCANBasic")

        try:
            self.sigcfg_file = prj_cfg_data["signal_cfg"]
            srl_baudrate:int = prj_cfg_data["serial_cfg"]["baudrate"]
            srl_protcom:str = prj_cfg_data["serial_cfg"]["port_com"]
            self.can_baudrate:int = getattr(pcan_module, prj_cfg_data["can_cfg"]["baudrate"])
            self.can_protcom:int  = getattr(pcan_module, prj_cfg_data["can_cfg"]["usb_bus"])

            self._is_serial_enable:bool = prj_cfg_data["serial_cfg"]["is_enable"]
            self._srl_frame_len:int = prj_cfg_data["serial_cfg"]["frame_len"]

            self._is_can_enable:bool = prj_cfg_data["can_cfg"]["is_enable"]

        except (KeyError, TypeError, AttributeError) as e:
            raise Exception(f'An error occured while extracting config project -> {e}')
        
        # also get the offset master stuff ^^ 
        self._idx_mux_offset = prj_cfg_data["db_mater_cfg"].get("offset_idx_mux", 0)
        # serial managment #
        self._serial_istc = SerialMngmt(srl_baudrate, srl_protcom, self.__error_serial_cb)
        self._can_istc = get_can_interface(DriverCanUsed.DrvPeak, f_error_cb= self.__error_can_cb)
        # signals maangment
        self.enum:Dict[str, List[List[int]]] = {}
        self.signals:Dict[str, Dict] = {}
        self.sig_value:Dict[str, Queue] = {}
        self.symbol:Dict[str, Dict] = {}
        self.list_id = {
            'SRL' : [],
            'CAN' : []
        }

        # thread maangment 
        self._srl_frame_thread: Optional[threading.Thread] = None
        self._can_frame_thread: Optional[threading.Thread] = None
        self._stop_srl_thread = threading.Event()
        self._stop_can_thread = threading.Event()

        #---- extract signals enum and stuff ----#
        self.__extract_signal_cfg()

    #--------------------------
    # get_signal_value
    #--------------------------
    def get_signal_value(self, f_signal: str) -> List[int]:
        """Get the Queue of values for a given signals.
        Args:
            f_signal (str): the name of the signals.
        Returns:
            List: a list of x element containing rawValue and ValueCompute [[1,4, timestamp], [5,20, timestamps]].
        Raises:
            KeyError: if the signals is not found.
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
        """Get a signals value

        Args:
            f_signal (str): the signals
        Returns:
            List[str]: the list with all signals
        """
        
        return [str(signal_name) for signal_name in self.signals.keys()]

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

        if self._is_can_enable:
            self._can_istc.connect(pcan_usb = self.can_protcom, pcan_baudrate= self.can_baudrate) 
            self._can_istc.flush()
            self._can_istc.receive_queue_start()
            self._stop_can_thread.clear()
            self._can_frame_thread = threading.Thread(target=self._cyclic_can_frame, daemon=True)
            self._can_frame_thread.start()

        # idem can
    
    #--------------------------
    # unperform_cyclic
    #--------------------------
    def unperform_cyclic(self)->None:
        """Unperform cyclic frame analyzer
        """

        self._serial_istc.stop()
        self._stop_srl_thread.set()

        self._can_istc.receive_queue_stop()
        self._can_istc.disconnect()
    #--------------------------
    # _cyclic_serial_frame
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
            else:
                time.sleep(0.01)

    
    #--------------------------
    # _cyclic_serial_frame
    #--------------------------
    def _cyclic_can_frame(self)->None:
        """Interpret a frame and put the value into signals

        Args:
            bytes (bytes): frame bytes
            len_frame (int):len of the frame

        Raises:
        """
        cnt_frame = 0
        current_time = 0
        while not self._stop_can_thread.is_set():
            can_frame = self._can_istc.get_can_frame()
            if can_frame.data != [] and can_frame.id != 418381708:
                cnt_frame += 1
                if(time.time() - current_time > 1):
                    print(f'Manage {cnt_frame}')
                    current_time = time.time()
                    cnt_frame = 0

                self.__decode_can_frame(can_frame)
            else:
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
            sig_conf = self.signals.get(signal_name)
            if not sig_conf:
                print(f"[ERROR] : Signal {signal_name} not configured")
                continue

            length = sig_conf['length']
            encoding = sig_conf['encoding']
            factor = sig_conf.get('factor', 1)
            offset = sig_conf.get('offset', 0)
            enum_name = sig_conf.get('enum')

            # Extraire la valeur brute du signals (bitfield)
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
    # __decode_can_frame
    #--------------------------
    def __decode_can_frame(self, f_can_frame:StructCANMsg)->None:
        """Interpret a serial frame into signals value
        Args: 
            f_srl_frame (bytes); the frame to decode
        Raises:
        """
        msg_id = f_can_frame.id  

        # Recherche du symbole correspondant à msg_id
        symbol = None
        for sym_name, sym in self.symbol.items():
            if int(sym['msg_id']& 0x0000FFFF) == int(msg_id & 0x0000FFFF):
                symbol = sym
                break

        if symbol is None:
            pass#print(f"[ERROR] : Symbole inconnu pour msg_id {msg_id}")
            return

        
        raw_data = bytes([int(byte) for byte in list(f_can_frame.data)])


        #---- no mux use ----#
        if symbol['mux_info'] == {}:
            signals:Dict = symbol['signals']['0']
        #---- find value ----#
        else:
            idx_mux:int = self.__extract_bits(bytes(raw_data), 
                                                    symbol['mux_info']['start_bit'], 
                                                    symbol['mux_info']['length'],
                                                    symbol['mux_info']['encoding'])
            if idx_mux >= self._idx_mux_offset:
                idx_mux -= self._idx_mux_offset
            else:
                print('[ERROR] : idx mux offset out of range')

            signals:Dict = symbol['signals'][str(idx_mux)]


        for signal_name, start_bit in signals.items():
            sig_conf = self.signals.get(signal_name)
            if not sig_conf:
                print(f"[ERROR] : Signal {signal_name} not configured")
                continue

            length = sig_conf['length']
            encoding = sig_conf['encoding']
            factor = sig_conf.get('factor', 1)
            offset = sig_conf.get('offset', 0)
            enum_name = sig_conf.get('enum')

            # Extraire la valeur brute du signals (bitfield)
            raw_value = self.__extract_bits(raw_data, start_bit, length, encoding)

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

            self.sig_value[signal_name].put([raw_value, value, f_can_frame.timestamp])


    #--------------------------
    # __extract_bits
    #--------------------------
    def __extract_bits(self, data: bytes, start_bit: int, length: int, encoding: str) -> int:
        """
        Extrait un champ de bits d'une trame en fonction de son start_bit et de sa len.

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
    # __interpret_frame
    #--------------------------
    def __error_can_cb(self, f_type_error:CanMngmtError):
        """Management of serial line whenever an error occured
        """
        if f_type_error == CanMngmtError.ErrorLost:
            self._stop_can_thread.set()
            print("[ERROR] : Stopping serial thread in FrameMngmt")
        else:
            print('[WARNING] : Timeout occured in SerialMngmt, did not receive any frame...')
    #--------------------------
    # __extract_signal_cfg
    #--------------------------
    def __extract_signal_cfg(self):
        """
            @brief get enum, signals, symbol from can signals config 
        """
 
        if not os.path.isfile(self.sigcfg_file):
            raise FileNotFoundError(f'Signal Config file doest not exits {self.sigcfg_file}')
        
        if str(self.sigcfg_file).endswith(".sym"):
            self.__sym_reader()

        elif str(self.sigcfg_file).endswith(".dbc"):
            self.__database_can_reader()

        else:
            raise Exception(f'Cannot found any function that interpret {self.sigcfg_file[str(self.sigcfg_file).index("."):]}')
    #--------------------------
    # __sym_reader
    #--------------------------
    def __sym_reader(self):
        """
            @brief get enum, signals, symbol from .sym file
        """
        current_read = 'NONE'
        waiting_for_timeout = False
        current_id = None
        current_type = None
        current_len = None
        current_symbol = None
        curr_idx_mux = '0'

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
                            resultats = re.findall(SYM_PATTERN_ENUM, full_line)
                            pairs = [[int(index), value] for index, value in resultats]

                            # Étape 4 : stocker
                            self.enum[enum_name] = pairs

                    case 'SIGNALS':
                        match = PATTERN_SIGNAL.match(line)
                        if match:
                            nom_signal    = match.group(1)
                            len_sig      = int(match.group(2))
                            encoding_flag = match.group(3)
                            factor        = int(match.group(4)) if match.group(4) else 1
                            offset        = int(match.group(5)) if match.group(5) else 0
                            # match.group(6) = max (non utilisé ici)
                            enum_name     = match.group(7) if match.group(7) else None

                            encoding = "MOTOROLA" if encoding_flag else "INTEL"
                            self.sig_value[nom_signal] = Queue()
                            self.signals[nom_signal] = {
                                'length': len_sig,
                                'encoding': encoding,
                                'factor': factor,
                                'offset': offset,
                                'enum': enum_name,
                                'unit' : None
                            }
                        else:
                            print(f'[INFO] : APPSIG_Codegen : While in SIGNALS, no signals pattern in line: {line.strip()}')

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
                                'mux_info' : {},
                                'timeout': 0,
                                'cycle_time': None  # <-- Ajouté ici
                            }
                            waiting_for_timeout = True
                            continue

                        match_id = SYM_PATTERN_ID.match(line)
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
                                self.symbol[current_symbol]['msg_id'] = int(current_id)
                                self.symbol[current_symbol]['msg_type'] = int(current_type)
                            continue

                        match_len = SYM_PATTERN_LEN.match(line)
                        if match_len:
                            current_len = int(match_len.group(1))
                            if current_symbol:
                                self.symbol[current_symbol]['msg_len'] = int(current_len)
                            continue

                        # Nouveau bloc : Timeout
                        if line.strip().lower().startswith("timeout="):
                            timeout_val = int(line.strip().split("=")[1].strip())
                            if current_symbol:
                                if timeout_val == 0:
                                    raise ValueError(f"Timeout cannot be 0 for symbol '{current_symbol}'")
                                self.symbol[current_symbol]['timeout'] = int(timeout_val)
                                waiting_for_timeout = False
                            continue

                        # Nouveau bloc : CycleTime
                        if line.strip().lower().startswith("cycletime="):
                            cycle_val = int(line.strip().split("=")[1].strip())
                            if current_symbol and (current_read == 'SEND' or current_read == 'SENDRECEIVE') :
                                if cycle_val == 0:
                                    raise ValueError(f"CycleTime cannot be 0 for symbol '{current_symbol}'")
                                self.symbol[current_symbol]['cycle_time'] = int(cycle_val)
                            continue

                        # Ligne signals
                        match_sig = SYM_PATTERN_SIG.match(line)
                        if match_sig:
                            signal_name = match_sig.group(1)
                            position = int(match_sig.group(2))

                            if current_symbol:
                                # Vérifier si le signals est bien défini
                                if signal_name not in self.signals:
                                    raise ValueError(f"Signal '{signal_name}' utilisé par '{current_symbol}' non défini dans SIGNALS")

                                new_start = position
                                new_length = self.signals[signal_name]['length']

                                for idx_mux, mux_signals in self.symbol[current_symbol].items():
                                    for existing_signal, existing_start in mux_signals:
                                        existing_length = self.signals[existing_signal]['length']

                                        new_end = new_start + new_length - 1
                                        existing_end = existing_start + existing_length - 1

                                        if not (new_end < existing_start or existing_end < new_start):
                                            raise ValueError(
                                                f"Conflit dans '{current_symbol}': signals '{signal_name}' (bits {new_start}-{new_end}) "
                                                f"chevauche '{existing_signal}' (bits {existing_start}-{existing_end})"
                                            )

                                # Pas de conflit, on ajoute le signals
                                if curr_idx_mux not in self.symbol[current_symbol]['signals'].keys():
                                    self.symbol[current_symbol]['signals'][curr_idx_mux] = {}

                                self.symbol[current_symbol]['signals'][curr_idx_mux][signal_name] = int(position)

                            continue

            if current_symbol:
                sym = self.symbol[current_symbol]
                if sym['msg_direction'] == 'RECEIVE' or sym['msg_direction'] == 'SENDRECEIVE':
                    if sym['timeout'] is None:
                        raise ValueError(f"Missing Timeout for last symbol '{current_symbol}'")
                if sym['msg_direction'] == 'SEND' or sym['msg_direction'] == 'SENDRECEIVE':
                    if sym['cycle_time'] is None:
                        raise ValueError(f"Missing CycleTime for last symbol '{current_symbol}'")
    
    #--------------------------
    # __database_can_reader
    #--------------------------
    def __database_can_reader(self):
        """
            @brief get enum, signals, symbol from .sym file
        """
        idxenm_to_string = {}
        current_read:str = ''
        current_symbol:str =  ''
        with open(self.sigcfg_file, 'r') as file:
            lines = file.readlines()

        for line in lines:

            if str(line).upper().startswith('BO_ '):
                match = re.match(DBC_SYM_PATTERN, line.strip())

                if not match:
                    print(f'[INFO] : {line} does not match the symbol pattern')
                else:
                    sym_name = match.group(2)
                    msg_id = match.group(1)
                    msg_len = match.group(3)
                    self.symbol[sym_name] = {
                        "msg_id" : int(msg_id),
                        "msg_len" : int(msg_len),
                        "msg_type" : None,
                        "msg_direction" : None,
                        'signals': {},
                        'mux_info' : {},
                        'timeout': 0,
                        'cycle_time': None
                    }
                    current_symbol = sym_name

                current_read = 'SYMBOLE'
                continue

            elif str(line).upper().startswith('BU_:'):
                current_read = 'ENUM'
                continue

            elif str(line).upper().startswith('VAL_ '):
                current_read = 'AFFECT_ENUM'

            match current_read:
                case 'ENUM':
                    match = re.match(DBC_ENM_PATTERN, line.strip())
                    if not match:
                        continue
                    
                    else:
                        enum_name = match.group(1)
                        enum_idx = match.group(2)
                        table_val = match.group(3)
                        idxenm_to_string[str(enum_idx)] = enum_name
                        results = re.findall(DBC_ENM_VAL_PATTERN, table_val)
                        pairs = [[int(index), value] for index, value in results]
                        self.enum[enum_name] = pairs



                case 'AFFECT_ENUM':
                    match = re.match(DBV_ENM_AFECT_PATTERN, line.strip())
                    if not match:
                        print(f'[INFO] : {line} does not match the signal-enum affectation pattern')

                    else:
                        sig_name = match.group(1)
                        enum_idx = match.group(2)

                        if not sig_name in self.signals.keys():
                            print(f'[INFO] : Found an enum asoo but signal {sig_name} is not register')
                        else:
                            self.signals[sig_name]['enum'] = idxenm_to_string[str(enum_idx)]

                case 'SYMBOLE':
                    if "SG_" in line.upper():
                        match = re.match(DBC_SIG_PATTERN, line.strip())

                        if not match:
                            print(f'[INFO] : {line} does not match the signal pattern')
                        
                        else:
                            sig_name        = match.group(1)
                            idx_multiplexer = match.group(2)
                            start_bit       = match.group(3)
                            len_sig         = int(match.group(4))
                            factor          = int(match.group(7)) if match.group(7) else 1
                            offset          = int(match.group(8)) if match.group(8) else 0
                            encoding        = 'INTEL' if match.group(5) == "1" else 'MOTOROLA'
                            unit            = match.group(11)

                            #--- Info about the mux ----#
                            if str(idx_multiplexer).upper() == 'M':
                                self.symbol[current_symbol]['mux_info'] = {
                                    'length' : int(len_sig),
                                    'start_bit' : int(start_bit),
                                    'encoding': encoding
                                }
                            else:
                                #--- filled signal information ----#
                                if sig_name not in self.signals.keys():
                                    self.signals[sig_name] = {
                                        'length': int(len_sig),
                                        'encoding': encoding,
                                        'factor': int(factor),
                                        'offset': int(offset),
                                        'unit' : unit
                                    }
                                    self.sig_value[sig_name] = Queue()
                                #--- filled symbol information ----#
                                if idx_multiplexer == None:
                                    idx_multiplexer = '0'
                                else:
                                    idx_multiplexer = str(idx_multiplexer)[1:]

                                if idx_multiplexer not in self.symbol[current_symbol]['signals'].keys():

                                    self.symbol[current_symbol]['signals'][idx_multiplexer] = {}
                                
                                self.symbol[current_symbol]['signals'][idx_multiplexer][sig_name] = int(start_bit)


                case _:
                    pass

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

