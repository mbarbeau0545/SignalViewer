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
import sys, json
import os
from collections import deque
import sys, time
import os
from PyQt5.QtWidgets import (
QApplication, QMainWindow, QTableWidget, QTableWidgetItem, QPushButton,
QVBoxLayout, QWidget, QTabWidget, QHBoxLayout, QLabel, QLineEdit, QComboBox,
QScrollArea, QFrame, QAction, QMenu, QToolBar, QHeaderView, QSplitter, QGroupBox,
QFormLayout,QFileDialog
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QColor, QBrush, QFontMetrics

import pyqtgraph as pg


from Frame.frameMngmt import FrameMngmt
from Signal.ActSnsMngmt import ActInfo, SnsInfo
from typing import List, Dict
#------------------------------------------------------------------------------
#                                       CONSTANT
#------------------------------------------------------------------------------
# application constants
REFRESH_IMH_SECONDS = 50  # ms
PLOT_MAX_POINT = 2000
GRAPH_CONFIG_FILENAME = "graph_config.json"
ACT_CTRL_FILENAME = "act_controls.json"
# CAUTION : Automatic generated code section: Start #

# CAUTION : Automatic generated code section: End #
#------------------------------------------------------------------------------
#                                       CLASS
#------------------------------------------------------------------------------





class SignalViewer(QMainWindow):
    def __init__(self, f_prj_cfg: str):
        super().__init__()

        if not os.path.isfile(f_prj_cfg):
            raise FileNotFoundError(f'Signal Config file doest not exits {f_prj_cfg}')
        
        self.prj_cfg = f_prj_cfg
        with open(self.prj_cfg, "r") as file:
            self.prj_cfg_data = json.load(file)

        try:
            excel_path = self.prj_cfg_data["excel_cfg"]
        except (KeyError, TypeError, AttributeError) as e:
            raise Exception(f'An error occured while extracting config project -> {e}')
        
        # --- app paths ---
        self.app_dir = os.path.dirname(os.path.abspath(__file__))
        self.graph_cfg_path = os.path.join(self.app_dir, GRAPH_CONFIG_FILENAME)
        self.act_ctrl_path = os.path.join(self.app_dir, ACT_CTRL_FILENAME)

        # init frame instance
        self.frame_isct = FrameMngmt(self.prj_cfg)
        self.is_ecu_connected = False
        self.last_try_con = 0

        self.setWindowTitle("Signal Viewer")
        self.resize(1200, 800)

        self.signals_name = self.frame_isct.get_signal_list()

        # utilisation de deque pour éviter l'écrasement
        self.signals_values = {
            signal_name: deque(maxlen=PLOT_MAX_POINT)
            for signal_name in self.signals_name
        }
        self.previous_values = {signal_name: -1 for signal_name in self.signals_name}

        self.frame_isct.get_symbol_list()

        

        # ========================
        # UI Setup
        # ========================
        self._create_toolbar()

        # Création du QTabWidget
        self.tab_widget = QTabWidget()

        # --- Signals tab ---
        self._init_signals_tab()       # <-- remplace la création directe de signals_widget

        # --- Message Sender tab ---
        self.msg_sender_widget = QWidget()
        self.msg_sender_layout = QVBoxLayout(self.msg_sender_widget)
        btn_add_msg = QPushButton("Add message")
        btn_add_msg.clicked.connect(self.__add_message_row)
        self.msg_sender_layout.addWidget(btn_add_msg)
        self.msg_scroll = QScrollArea()
        self.msg_scroll.setWidgetResizable(True)
        self.msg_container = QWidget()
        self.msg_rows_container = QVBoxLayout(self.msg_container)
        self.msg_container.setLayout(self.msg_rows_container)
        self.msg_scroll.setWidget(self.msg_container)
        self.msg_sender_layout.addWidget(self.msg_scroll)
        self.tab_widget.addTab(self.msg_sender_widget, "Message Sender")

        # --- Sensors / Actuators ---
        self.sensors = SnsInfo(excel_path)
        self.actuator = ActInfo(excel_path)
        self.act_control_values = self._load_act_controls()

        self._init_sensors_tab()       # <-- crée et remplit l'onglet Sensors
        self._init_actuators_tab()     # <-- crée et remplit l'onglet Actuators
        # load persisted actuator controls
        # === Ajout du tab_widget comme contenu principal ===
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.addWidget(self.tab_widget)
        self.setCentralWidget(container)

        # ========================
        # Timers de mise à jour
        # ========================
        self.__connect_ecu()
        self.timer = QTimer()
        self.timer.timeout.connect(self.__refresh_table)
        self.timer.start(REFRESH_IMH_SECONDS)  # ms

        # timer de garde (évite freeze de l’UI si pas de signal)
        self._timer_interrupt = QTimer()
        self._timer_interrupt.timeout.connect(lambda: None)
        self._timer_interrupt.start(REFRESH_IMH_SECONDS)

        # try to restore saved graphs
        self._load_graphs_state()

    # ------------------------- toolbar -------------------------
    def _create_toolbar(self):
        toolbar = self.addToolBar("Main Toolbar")

        # --- Refresh Connection Button ---
        self.btn_refresh = QPushButton("Refresh Connection")
        self.btn_refresh.clicked.connect(self.__connect_ecu)
        toolbar.addWidget(self.btn_refresh)

        # --- Load Signal CFG ---
        btn_load_signal = QPushButton("Load Signal CFG")
        btn_load_signal.clicked.connect(self.__load_signal_cfg)
        toolbar.addWidget(btn_load_signal)

        # --- Load Excel CFG ---
        btn_load_excel = QPushButton("Load Excel CFG")
        btn_load_excel.clicked.connect(self.__load_excel_cfg)
        toolbar.addWidget(btn_load_excel)

    #--------------------------
    # kill_all_thread
    #--------------------------
    def kill_all_thread(self):
        """Kill all thread currently on going """
        self.frame_isct.unperform_cyclic()

    #--------------------------
    # __connect_ecu
    #--------------------------
    def __connect_ecu(self):
        self.last_try_con = time.time()
        try:
            self.frame_isct.perform_cyclic()
            self.is_ecu_connected = True
            print("[INFO] : Connection succeeded, ECU connected")
        except Exception as e:
            print(f"[INFO] : Connection Failed -> {e}")
            self.is_ecu_connected = False

        self.__update_refresh_btn_color()

    #--------------------------
    # __refresh_table
    #--------------------------
    def __refresh_table(self):
        curr_time = time.time()
        if not self.is_ecu_connected and (curr_time - self.last_try_con) > 5:
            self.__connect_ecu()
            return

        # update signals table and store values for plots
        for row, signal_name in enumerate(self.signals_name):
            sig_val = self.frame_isct.get_signal_value(signal_name)
            if sig_val == [[]]:
                continue

            for sig_info in sig_val:
                if sig_info:
                    self.signals_values[signal_name].append(sig_info)

            if sig_val and sig_val[-1]:
                raw_val = str(sig_val[-1][0])
                calc_val = str(sig_val[-1][1])
                prev_raw = self.previous_values.get(signal_name)

                item_raw = QTableWidgetItem(raw_val)
                item_calc = QTableWidgetItem(calc_val)
                item_raw.setForeground(QBrush(QColor('black')))
                item_calc.setForeground(QBrush(QColor('black')))
                self.table.setItem(row, 1, item_raw)
                self.table.setItem(row, 2, item_calc)

                if prev_raw is not None and prev_raw != raw_val:
                    item_raw.setBackground(QColor("yellow"))
                    item_calc.setBackground(QColor("yellow"))
                else:
                    item_raw.setBackground(QColor("white"))
                    item_calc.setBackground(QColor("white"))

                self.previous_values[signal_name] = raw_val
                if signal_name.upper().startswith("SNS"):
                # update sensors widget values
                    self._refresh_sensors_values(signal_name, raw_val)
                # update actuators interface get_sig values
                elif signal_name.upper().startswith("ACT"):
                    self._refresh_actuators_get_values(signal_name, raw_val)

    #--------------------------
    # Graph tab management (with persistence)
    #--------------------------
    def __open_graph_tab(self, signal_name, saved_signals: List[str] = None):
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # Boutons
        btn_toggle = QPushButton("Stop")
        btn_close = QPushButton("Close Tab")
        layout.addWidget(btn_toggle)
        layout.addWidget(btn_close)

        btn_close.clicked.connect(lambda _, w=widget: self.__close_tab(w))
        btn_toggle.clicked.connect(lambda _, w=widget, b=btn_toggle: self.__toggle_pause(w, b))

        # Plot
        plot_widget = pg.PlotWidget(title="Signals")
        plot_widget.setLabel('bottom', 'Temps', units='s')
        plot_widget.setLabel('left', 'Valeur')
        plot_widget.addLegend()
        plot_widget.setContextMenuPolicy(Qt.CustomContextMenu)
        plot_widget.customContextMenuRequested.connect(
            lambda pos, w=widget: self.__show_graph_context_menu(w, pos)
        )
        layout.addWidget(plot_widget)

        # Ajout de l'onglet
        self.tab_widget.addTab(widget, signal_name)
        self.tab_widget.setCurrentWidget(widget)

        # Stockage
        widget._curves = {}
        widget._plot_widget = plot_widget
        widget._t0 = None
        widget._paused = False

        # Ajouter le premier signal (or use saved list)
        if saved_signals:
            for s in saved_signals:
                self.__add_signal_to_tab(widget, s)
        else:
            self.__add_signal_to_tab(widget, signal_name)

        # Fonction update
        def update_plot():
            if widget._paused:
                return

            # init t0 commun
            if widget._t0 is None:
                min_t0 = None
                for sig_name in widget._curves.keys():
                    if self.signals_values[sig_name]:
                        t0_candidate = self.signals_values[sig_name][0][2]
                        if min_t0 is None or t0_candidate < min_t0:
                            min_t0 = t0_candidate
                widget._t0 = min_t0
                if widget._t0 is None:
                    return

            # mise à jour par signal
            for sig_name, sig_data in list(widget._curves.items()):
                while self.signals_values[sig_name]:
                    raw, calc, ts = self.signals_values[sig_name][0]
                    if ts < widget._t0:
                        self.signals_values[sig_name].popleft()
                        continue

                    t_sec = (ts - widget._t0) / 1e9
                    if not sig_data["times"] or t_sec > sig_data["times"][-1]:
                        sig_data["times"].append(t_sec)
                        sig_data["values"].append(raw)
                    self.signals_values[sig_name].popleft()

                # découpe et affichage
                if len(sig_data["times"]) > PLOT_MAX_POINT:
                    sig_data["times"] = sig_data["times"][-PLOT_MAX_POINT:]
                    sig_data["values"] = sig_data["values"][-PLOT_MAX_POINT:]

                if len(sig_data["times"]) > 1:
                    sig_data["curve"].setData(sig_data["times"], sig_data["values"])

        # Timer Qt
        timer = QTimer(widget)
        timer.timeout.connect(update_plot)
        timer.start(REFRESH_IMH_SECONDS)
        widget._timer = timer

        # attach tab metadata for persistence
        widget._saved_tab_title = signal_name

        # when tab is closed via GUI we must save state
        return widget
    # ------------------------------
    # SIGNALS TAB
    # ------------------------------
    def _init_signals_tab(self):
        """Initialise l'onglet Signals"""
        self.signals_widget = QWidget()
        self.signals_layout = QVBoxLayout(self.signals_widget)

        self.table = QTableWidget(len(self.signals_name), 4)
        self.table.setHorizontalHeaderLabels(["Signal", "Raw Value", "Value", "Graph"])
        self.table.setColumnWidth(0, 150)
        self.table.setColumnWidth(2, 150)
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.signals_layout.addWidget(self.table)
        self.tab_widget.addTab(self.signals_widget, "Signals")

        # Remplissage initial
        for row, signal_name in enumerate(self.signals_name):
            self.table.setItem(row, 0, QTableWidgetItem(signal_name))
            btn = QPushButton("Graph")
            btn.clicked.connect(lambda _, s=signal_name: self.__open_graph_tab(s))
            self.table.setCellWidget(row, 3, btn)


    # ------------------------------
    # SENSORS TAB
    # ------------------------------
    def _init_sensors_tab(self):
        """Initialise l'onglet Sensors"""
        self.sensors_widget = QWidget()
        layout = QVBoxLayout(self.sensors_widget)

        self.sensors_table = QTableWidget()
        self.sensors_table.setColumnCount(3)
        self.sensors_table.setHorizontalHeaderLabels(["Sensor", "Unit", "Value"])
        self.sensors_table.horizontalHeader().setStretchLastSection(True)

        layout.addWidget(self.sensors_table)
        self.tab_widget.addTab(self.sensors_widget, "Sensors")

        # Remplissage initial
        keys = list(self.sensors.info.keys())
        self.sensors_table.setRowCount(len(keys))
        for row, key in enumerate(keys):
            self.sensors_table.setItem(row, 0, QTableWidgetItem(key))
            self.sensors_table.setItem(row, 1, QTableWidgetItem(self.sensors.info[key]["unity"]))

            # Valeur actuelle
            sig_name = self.sensors.info[key]["signal"]
            val = self.frame_isct.get_signal_value(sig_name)
            display_val = str(val[-1][1]) if val and val[-1] else "N/A"
            self.sensors_table.setItem(row, 2, QTableWidgetItem(display_val))


    # ------------------------------
    # ACTUATORS TAB
    # ------------------------------
    def _init_actuators_tab(self):
        """Initialise l'onglet Actuators"""
        self.actuators_widget = QWidget()
        layout = QVBoxLayout(self.actuators_widget)

        self.actuators_tab_widget = QTabWidget()
        layout.addWidget(self.actuators_tab_widget)

        # ---- Interface tab ----
        interface_widget = QWidget()
        interface_layout = QVBoxLayout(interface_widget)
        self.actuators_interface_table = QTableWidget()
        self.actuators_interface_table.setColumnCount(4)
        self.actuators_interface_table.setHorizontalHeaderLabels(
            ["Actuator", "Set Value", "Get Value", "Control"]
        )
        self.actuators_interface_table.horizontalHeader().setStretchLastSection(True)
        interface_layout.addWidget(self.actuators_interface_table)
        self.actuators_tab_widget.addTab(interface_widget, "Interface")

        # Remplissage
        keys = list(self.actuator.info.keys())
        self.actuators_interface_table.setRowCount(len(keys))
        for row, key in enumerate(keys):
            info = self.actuator.info[key]
            self.actuators_interface_table.setItem(row, 0, QTableWidgetItem(key))
            # Valeurs set et get actuelles
            val_set = self.frame_isct.get_signal_value(info["set_sig"])
            val_get = self.frame_isct.get_signal_value(info["get_sig"])
            self.actuators_interface_table.setItem(row, 1, QTableWidgetItem(str(val_set[-1][1]) if val_set and val_set[-1] else "N/A"))
            self.actuators_interface_table.setItem(row, 2, QTableWidgetItem(str(val_get[-1][1]) if val_get and val_get[-1] else "N/A"))
            # Controle
            edit_ctrl = QLineEdit()
            edit_ctrl.setObjectName(key)
            self.actuators_interface_table.setCellWidget(row, 3, edit_ctrl)
            for actitf_dict in self.act_control_values.values():
                if key in actitf_dict.keys():
                    edit_ctrl.setText(str(actitf_dict[key]))
            edit_ctrl.editingFinished.connect(lambda k=key, e=edit_ctrl: self._store_act_control_value(k, e.text()))

        # ---- Device tab ----
        device_widget = QWidget()
        device_layout = QVBoxLayout(device_widget)
        self.actuators_device_table = QTableWidget()
        self.actuators_device_table.setColumnCount(3)
        self.actuators_device_table.setHorizontalHeaderLabels(["Device", "Start", "Stop"])
        self.actuators_device_table.horizontalHeader().setStretchLastSection(True)
        device_layout.addWidget(self.actuators_device_table)
        self.actuators_tab_widget.addTab(device_widget, "Device")

        # Remplissage
        dvc_list = self.actuator.dvc_list
        self.actuators_device_table.setRowCount(len(dvc_list))
        for row, dvc_name in enumerate(dvc_list):
            self.actuators_device_table.setItem(row, 0, QTableWidgetItem(dvc_name))

            btn_start = QPushButton("Start")
            btn_start.clicked.connect(lambda _, d=dvc_name: self.send_act_dvc_values(d))
            self.actuators_device_table.setCellWidget(row, 1, btn_start)

            btn_stop = QPushButton("Stop")
            btn_stop.clicked.connect(lambda _, d=dvc_name: self.stop_act_dvc(d))
            self.actuators_device_table.setCellWidget(row, 2, btn_stop)

        self.tab_widget.addTab(self.actuators_widget, "Actuators")
        
    def __add_signal_to_tab(self, widget, signal_name: str):
        if signal_name in widget._curves:
            return  # déjà présent
        color = pg.intColor(len(widget._curves))  # couleur auto
        curve = widget._plot_widget.plot([], [], pen=color, name=signal_name)
        widget._curves[signal_name] = {
            "times": [],
            "values": [],
            "curve": curve
        }

    def __remove_signal_from_tab(self, widget, signal_name: str):
        if signal_name in widget._curves:
            widget._plot_widget.removeItem(widget._curves[signal_name]["curve"])
            del widget._curves[signal_name]

    def __show_graph_context_menu(self, widget, pos):
        menu = QMenu(widget)

        # Signaux déjà affichés
        if widget._curves:
            submenu_remove = menu.addMenu("Supprimer un signal")
            for sig_name in list(widget._curves.keys()):
                action = QAction(sig_name, self)
                action.triggered.connect(
                    lambda _, s=sig_name: self.__remove_signal_from_tab(widget, s)
                )
                submenu_remove.addAction(action)

        # Signaux disponibles à ajouter
        available = [s for s in self.signals_name if s not in widget._curves]
        if available:
            submenu_add = menu.addMenu("Ajouter un signal")
            for sig_name in available:
                action = QAction(sig_name, self)
                action.triggered.connect(
                    lambda _, s=sig_name: self.__add_signal_to_tab(widget, s)
                )
                submenu_add.addAction(action)

        # Afficher le menu au bon endroit
        menu.exec_(widget._plot_widget.mapToGlobal(pos))

    def __close_tab(self, f_widget):
        index = self.tab_widget.indexOf(f_widget)
        if index != -1:
            tab_title = self.tab_widget.tabText(index)

            self.tab_widget.removeTab(index)

            if hasattr(f_widget, '_timer'):
                f_widget._timer.stop()

            f_widget.deleteLater()

            # mettre à jour le JSON
            self._remove_tab_from_config(tab_title)

    def __toggle_pause(self, f_widget, f_btn_toggle):
        f_widget._paused = not f_widget._paused
        f_btn_toggle.setText("Start" if f_widget._paused else "Stop")

    #--------------------------
    # Message sender UI
    #--------------------------
    def __add_message_row(self):
        """Ajoute une ligne pour configurer un message"""
        row_widget = QFrame()
        row_widget.setFrameShape(QFrame.StyledPanel)
        row_widget.setFrameShadow(QFrame.Raised)
        row_layout = QHBoxLayout(row_widget)
        row_layout.setContentsMargins(10, 5, 10, 5)
        row_layout.setSpacing(15)

        combo = QComboBox()
        combo.addItems(self.frame_isct.get_symbol_list())
        combo.setMinimumWidth(180)
        row_layout.addWidget(combo)

        sig_table = QTableWidget()
        sig_table.setColumnCount(2)
        sig_table.setHorizontalHeaderLabels(["Signal (bits)", "Value"])
        sig_table.horizontalHeader().setStretchLastSection(True)
        sig_table.setAlternatingRowColors(True)
        sig_table.setStyleSheet("""
            QTableWidget::item { padding: 4px; }
            QHeaderView::section {
                background-color: #e0e0e0;
                font-weight: bold;
                padding: 4px;
            }
        """)
        row_layout.addWidget(sig_table, 1)

        if combo.count() > 0:
            self.__populate_signals(sig_table, combo.currentText())

        combo.currentTextChanged.connect(
            lambda sym: self.__populate_signals(sig_table, sym)
        )

        right_layout = QVBoxLayout()
        right_layout.setSpacing(8)

        cyc_layout = QHBoxLayout()
        cyc_layout.addWidget(QLabel("Cyclic (ms):"))
        cyclic_edit = QLineEdit("0")
        cyclic_edit.setFixedWidth(60)
        cyc_layout.addWidget(cyclic_edit)
        right_layout.addLayout(cyc_layout)

        btn_send = QPushButton("Send")
        btn_send.setFixedWidth(70)
        right_layout.addWidget(btn_send)

        btn_send.clicked.connect(
            lambda _, cb=combo, st=sig_table, ce=cyclic_edit, rw=row_widget:
                self.__send_message(cb.currentText(), st, ce.text(), rw)
        )

        btn_delete = QPushButton("Delete")
        btn_delete.setFixedWidth(70)
        btn_delete.setStyleSheet("background-color: #f28b82;")
        right_layout.addWidget(btn_delete)

        btn_delete.clicked.connect(lambda _, rw=row_widget: self.__delete_message_row(rw))

        right_layout.addStretch()
        row_layout.addLayout(right_layout)

        self.msg_rows_container.addWidget(row_widget)

    def __populate_signals(self, table: QTableWidget, sym_name: str):
        table.clearContents()
        table.setRowCount(0)

        signals = self.frame_isct.get_signal_info_from_symbol(sym_name)
        if not signals:
            return

        row_height = 30
        max_visible_rows = 5

        table.setRowCount(len(signals))
        table.setColumnCount(2)
        table.setHorizontalHeaderLabels(["Signal (bits), (factor), (offset)", "Value"])
        table.horizontalHeader().setStretchLastSection(True)

        font = table.font()
        metrics = QFontMetrics(font)
        max_width = 0

        for row, (sig_name, info_sig) in enumerate(signals.items()):
            text = f"{sig_name} ({info_sig['length']}b, {info_sig['factor']}*, {info_sig['offset']}+)"
            item = QTableWidgetItem(text)
            item.setFlags(Qt.ItemIsEnabled)
            table.setItem(row, 0, item)

            edit = QLineEdit()
            edit.setObjectName(sig_name)
            table.setCellWidget(row, 1, edit)

            table.setRowHeight(row, row_height)

            width = metrics.horizontalAdvance(text) + 20
            if width > max_width:
                max_width = width

        table.setColumnWidth(0, max_width)

        header_height = table.horizontalHeader().height()
        visible_rows = min(len(signals), max_visible_rows)
        table.setFixedHeight(header_height + row_height * visible_rows)

        table.setVerticalScrollBarPolicy(
            Qt.ScrollBarAlwaysOff if len(signals) <= max_visible_rows else Qt.ScrollBarAsNeeded
        )

    def __delete_message_row(self, row_widget: QWidget):
        if hasattr(row_widget, "_timer") and row_widget._timer is not None:
            row_widget._timer.stop()
            row_widget._timer.deleteLater()

        self.msg_rows_container.removeWidget(row_widget)
        row_widget.deleteLater()

    def __send_message(self, sym_name: str, table: QTableWidget, cyclic_val: str, row_widget: QWidget):
        signals = {}
        if table and isinstance(table, QTableWidget):
            for row in range(table.rowCount()):
                sig_item = table.item(row, 0)
                sig_name = sig_item.text().split(" ")[0] if sig_item else ""
                edit = table.cellWidget(row, 1)
                try:
                    signals[sig_name] = int(edit.text())
                except (ValueError, AttributeError):
                    signals[sig_name] = 0

        cyclic_val = int(cyclic_val) if str(cyclic_val).isdigit() else 0
        if cyclic_val > 0:
            print(f"[Cyclic] Send {sym_name} every {cyclic_val}ms with {signals}")
            timer = QTimer(row_widget)
            timer.timeout.connect(lambda: self.frame_isct.send_signal_msg(signals, sym_name))
            timer.start(cyclic_val)
            row_widget._timer = timer
        else:
            print(f"[Once] Send {sym_name} with {signals}")
            self.frame_isct.send_signal_msg(signals, sym_name)

    # ----------------------- Sensors tab -----------------------
    def _build_sensors_tab(self):
        # build UI for sensors and add to main tabs
        w = QWidget()
        layout = QVBoxLayout(w)

        self.sensors_table = QTableWidget()
        keys = list(self.sensors.info.keys())
        self.sensors_table.setColumnCount(3)
        self.sensors_table.setHorizontalHeaderLabels(["Sensor", "Unit", "Value"])
        self.sensors_table.setRowCount(len(keys))
        for r, key in enumerate(keys):
            self.sensors_table.setItem(r, 0, QTableWidgetItem(key))
            unit = self.sensors.info[key].get("unity", "")
            self.sensors_table.setItem(r, 1, QTableWidgetItem(str(unit)))
            self.sensors_table.setItem(r, 2, QTableWidgetItem(""))
        layout.addWidget(self.sensors_table)

        self.tab_widget.addTab(w, "Sensors")

    def _refresh_sensors_values(self, f_sig_name, f_sig_val):
        # update values from frame_isct
        for r, key in enumerate(list(self.sensors.info.keys())):
            sig_name = self.sensors.info[key].get("signal")
            if f_sig_name == sig_name:
                self.sensors_table.setItem(r, 2, QTableWidgetItem(f_sig_val))

    def _refresh_actuators_get_values(self, f_sig_name, f_sig_val):
        # update the get_sig column with live values
        for r, key in enumerate(list(self.actuator.info.keys())):
            get_sig = self.actuator.info[key].get("get_sig")
            if get_sig == f_sig_name:
                self.actuators_interface_table.setItem(r, 2, QTableWidgetItem(f_sig_val))
            else:
                set_sig = self.actuator.info[key].get("set_sig")
                if set_sig == f_sig_name:
                    self.actuators_interface_table.setItem(r, 1, QTableWidgetItem(f_sig_val))
    

    def _store_act_control_value(self, actuator_itf_key: str, text_val: str):
        try:
            val = int(text_val)
        except ValueError:
            try:
                val = float(text_val)
            except ValueError:
                val = text_val

        # find the device 
        for key in self.act_control_values.keys():
            if key in str(actuator_itf_key):
                self.act_control_values[key][actuator_itf_key] = val

        print(f"For device {key} interface {actuator_itf_key}, save {val}")


    # ----------------------- actuator device placeholders -----------------------
    def send_act_dvc_values(self, device_name: str):
        """
        Placeholder called when user clicks Start on a device row.
        Implement your actual device-start logic here or override this method in a subclass.
        """
        sig_value = {}
        if self.act_control_values[device_name] != {}:
            for key, value in self.act_control_values[device_name].items():
                signame = f"ACT_ACTITF_CTRL_{key}"
                sig_value[signame] = value

            self.frame_isct.send_signal_msg(sig_value)
            print(f"[Action] Start device {device_name} with signals {sig_value}")
        else:
            print(f"[ERROR] : No signal found for {device_name}")
    def stop_act_dvc(self, device_name: str):
        """
        Placeholder called when user clicks Stop on a device row.
        Implement your actual device-stop logic here or override this method in a subclass.
        """
        sig_value = {}
        if self.act_control_values[device_name] != {}:
            for key, value in self.act_control_values[device_name].items():
                signame = f"ACT_ACTITF_CTRL_{key}"
                sig_value[signame] = 0

            self.frame_isct.send_signal_msg(sig_value)
            print(f"[Action] Stop device {device_name}")

        else:
            print(f"[ERROR] : No signal found for {device_name}")

    # ----------------------- persistence helpers -----------------------
    def _save_graphs_state(self):
        try:
            tabs_state = []
            for i in range(self.tab_widget.count()):
                w = self.tab_widget.widget(i)
                # we only persist graph tabs that we created (they have _curves attr)
                if hasattr(w, '_curves'):
                    tabs_state.append({
                        'title': getattr(w, '_saved_tab_title', f'graph_{i}'),
                        'signals': list(w._curves.keys()),
                        'paused': getattr(w, '_paused', False)
                    })
            with open(self.graph_cfg_path, 'w') as fh:
                json.dump(tabs_state, fh, indent=2)
            print(f"[INFO] Graph state saved to {self.graph_cfg_path}")
        except Exception as e:
            print(f"[WARN] Failed to save graphs state: {e}")

    def _remove_tab_from_config(self, tab_title):
        try:
            # Charger l'existant
            try:
                with open(self.graph_cfg_path, 'r') as fh:
                    tabs_state = json.load(fh)
            except FileNotFoundError:
                tabs_state = []

            # Filtrer en excluant l'onglet supprimé
            new_tabs = [
                t for t in tabs_state
                if t.get("title") != tab_title
            ]

            # Sauvegarder la config nettoyée
            with open(self.graph_cfg_path, 'w') as fh:
                json.dump(new_tabs, fh, indent=2)

            print(f"[INFO] Tab '{tab_title}' removed from JSON")

        except Exception as e:
            print(f"[WARN] Failed to update tab state after removal: {e}")

    def _load_graphs_state(self):
        if not os.path.isfile(self.graph_cfg_path):
            return
        try:
            with open(self.graph_cfg_path, 'r') as fh:
                tabs_state = json.load(fh)
            for tab in tabs_state:
                title = tab.get('title', 'Graph')
                signals = tab.get('signals', [])
                widget = self.__open_graph_tab(title, saved_signals=signals)
                # if paused, flip paused flag
                if tab.get('paused', False):
                    widget._paused = True
            print(f"[INFO] Graph state restored from {self.graph_cfg_path}")
        except Exception as e:
            print(f"[WARN] Failed to restore graph state: {e}")

    def _save_act_controls(self):
        try:
            with open(self.act_ctrl_path, 'w') as fh:
                json.dump(self.act_control_values, fh, indent=2)
        except Exception as e:
            print(f"[WARN] Failed to save actuator controls: {e}")

    def _load_act_controls(self):
        if not os.path.isfile(self.act_ctrl_path):
            return {key : {} for key in self.actuator.dvc_list}
        try:
            with open(self.act_ctrl_path, 'r') as fh:
                return json.load(fh)
        except Exception as e:
            print(f"[WARN] Failed to load actuator controls: {e}")
            return {key : {} for key in self.actuator.dvc_list}

    def __update_refresh_btn_color(self):
        color = "green" if self.is_ecu_connected else "red"
        self.btn_refresh.setStyleSheet(f"background-color: {color};")


    def __load_signal_cfg(self):
        fname, _ = QFileDialog.getOpenFileName(self, "Select Signal CFG", "", "JSON Files (*.json);;All Files (*)")
        if fname:
            self.f_prj_cfg = fname
            self._update_cfg_json("signal_cfg", fname)
            self.__reload_all()

    def __load_excel_cfg(self):
        fname, _ = QFileDialog.getOpenFileName(self, "Select Excel CFG", "", "Excel Files (*.xlsx *.xls);;All Files (*)")
        if fname:
            self.f_excel_cfg = fname
            self._update_cfg_json("excel_cfg", fname)
            self.__reload_all()

    def _update_cfg_json(self, key, value):
        try:
            cfg_path = "config.json"  # votre fichier json de config
            cfg = {}
            if os.path.exists(cfg_path):
                with open(cfg_path, 'r') as f:
                    cfg = json.load(f)
            cfg[key] = value
            with open(cfg_path, 'w') as f:
                json.dump(cfg, f, indent=2)
            print(f"[INFO] Updated {key} in config.json")
        except Exception as e:
            print(f"[WARN] Failed to update config.json: {e}")

    def __reload_all(self):
        """Fonction qui détruit et recrée toutes les tables, onglets et signaux"""
        # On ferme les onglets graphiques
        for i in reversed(range(self.tab_widget.count())):
            w = self.tab_widget.widget(i)
            self.tab_widget.removeTab(i)
            w.deleteLater()

        # Reload frame_isct avec le nouveau f_prj_cfg
        self.frame_isct = FrameMngmt(self.f_prj_cfg)

        # Recréation des onglets Signals, Sensors et Actuators
        # et remplissage de la table Signals
        self.signals_name = self.frame_isct.get_signal_list()
        self.signals_values = {signal_name: deque(maxlen=PLOT_MAX_POINT) for signal_name in self.signals_name}
        self.previous_values = {signal_name: -1 for signal_name in self.signals_name}

        self._init_signals_tab()
        self._init_sensors_tab()
        self._init_actuators_tab()

    # ----------------------- close event -----------------------
    def closeEvent(self, event):
        # save graphs state and act controls on exit
        self._save_graphs_state()
        self._save_act_controls()
        # kill all threads / cyclic
        try:
            self.kill_all_thread()
        except Exception:
            pass
        super().closeEvent(event)


# If run as a script, create an app with dummy config paths (user to adjust)
if __name__ == '__main__':
    app = QApplication(sys.argv)
    # replace with real paths
    project_cfg = 'path/to/prj_cfg'
    excel_cfg = 'path/to/excel_cfg.xlsx'
    w = SignalViewer(project_cfg, excel_cfg)
    w.show()
    sys.exit(app.exec_())

#------------------------------------------------------------------------------
#                             FUNCTION IMPLMENTATION
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

