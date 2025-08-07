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
import sys
import os
"""from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QTableWidget, QTableWidgetItem, QPushButton,
    QVBoxLayout, QWidget, QTabWidget
)
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QColor, QBrush"""

import sys
import os
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QTableWidget, QTableWidgetItem, QPushButton,
    QVBoxLayout, QWidget, QTabWidget
)
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QColor, QBrush

import pyqtgraph as pg


from Frame.frameMngmt import FrameMngmt
from typing import List, Dict
#------------------------------------------------------------------------------
#                                       CONSTANT
#------------------------------------------------------------------------------
REFRESH_IMH_SECONDS = 50
PLOT_MAX_POINT = 1000
# CAUTION : Automatic generated code section: Start #

# CAUTION : Automatic generated code section: End #
#------------------------------------------------------------------------------
#                                       CLASS
#------------------------------------------------------------------------------

class SignalViewer(QMainWindow):
    #--------------------------
    # __init__
    #--------------------------
    def __init__(self, f_prj_cfg:str):
        super().__init__()

        # init frame isntance 
        self.frame_isct = FrameMngmt(f_prj_cfg)

        self.setWindowTitle("Signal Viewer")
        self.resize(1200, 800)

        self.signals_name:List[str] = self.frame_isct.get_signal_list()
        self.signals_values:Dict[str,List[List]] = { 
            signal_name : {} for signal_name in self.signals_name
        }
        self.previous_values:Dict[str, int] = { 
            signal_name : -1 for signal_name in self.signals_name
        }

        self.table = QTableWidget(len(self.signals_name), 4)
        self.table.setHorizontalHeaderLabels(["Signal", "Raw Value", "Value", "Graph"])
        # resize automatically column 'Value' and 'Signal' cause Enum & Signal may take long place
        self.table.setColumnWidth(2, 150)
        self.table.setColumnWidth(0, 150)


        self.tab_widget = QTabWidget()

        layout = QVBoxLayout()
        layout.addWidget(self.table)
        layout.addWidget(self.tab_widget)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        
        
        for row, signal_name in enumerate(self.signals_name):
            self.table.setItem(row, 0, QTableWidgetItem(signal_name))
            # Bouton Graph
            btn = QPushButton("Graph")
            btn.clicked.connect(lambda _, s=signal_name: self.__open_graph_tab(s))
            self.table.setCellWidget(row, 3, btn)

        # start performing frame update 
        self.frame_isct.perform_cyclic()
        # Timer de mise à jour
        self.timer = QTimer()
        self.timer.timeout.connect(self.__refresh_table)
        self.timer.start(REFRESH_IMH_SECONDS)  # toutes les 0.3s
        
        self._timer_interrupt = QTimer()
        self._timer_interrupt.start(500)  # 500 ms
        self._timer_interrupt.timeout.connect(lambda: None)  


    #--------------------------
    # kill_all_thread
    #--------------------------
    def kill_all_thread(self):
        """Kill all thread currently on going 
        """
        self.frame_isct.unperform_cyclic()
    #--------------------------
    # __refresh_table
    #--------------------------
    def __refresh_table(self):
        for row, signal_name in enumerate(self.signals_name):
            sig_val:List[List] = self.frame_isct.get_signal_value(signal_name)
            if sig_val == [[]]:
                continue
            # store if for graph if needed
            self.signals_values[signal_name] = sig_val
            for signal_info in sig_val:
            
                # get the previous value 
                prev_raw = self.previous_values.get(signal_name)

                if signal_info != []:
                    raw_val = str(signal_info[0]) # idx 0 is rawValue
                    calc_val = str(signal_info[1]) # idx 1 is Compute Value
                else: # nothing receive 
                    raw_val = None
                    calc_val = None

                if raw_val is not None and calc_val is not None:
                    # Mise à jour Raw Value
                    item_raw = QTableWidgetItem(raw_val)
                    item_raw.setForeground(QBrush(QColor('black')))
                    self.table.setItem(row, 1, item_raw)

                    # Mise à jour Calculated Value
                    item_calc = QTableWidgetItem(calc_val)
                    item_calc.setForeground(QBrush(QColor('black')))
                    self.table.setItem(row, 2, item_calc)

                    # Coloration si valeur changée
                    
                    if prev_raw is not None and prev_raw != raw_val:
                        item_raw.setBackground(QColor("yellow"))
                        item_calc.setBackground(QColor("yellow"))
                    else:
                        item_raw.setBackground(QColor("white"))
                        item_calc.setBackground(QColor("white"))

                    self.previous_values[signal_name] = raw_val

    #--------------------------
    # __open_graph_tab
    #--------------------------
    def __open_graph_tab(self, signal_name):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        btn_close = QPushButton("Close")
        layout.addWidget(btn_close)
        btn_close.clicked.connect(lambda _, w=widget: self.__close_tab(w))

        plot_widget = pg.PlotWidget(title=signal_name)
        plot_widget.setLabel('bottom', 'Temps', units='s')
        plot_widget.setLabel('left', 'Valeur')
        layout.addWidget(plot_widget)

        # Ajout de l'onglet
        self.tab_widget.addTab(widget, signal_name)
        self.tab_widget.setCurrentWidget(widget)

        # Stocker les valeurs
        widget._times = []
        widget._values = []
        widget._t0 = None

        # Trace (ligne)
        curve = plot_widget.plot(widget._times, widget._values, pen='y')
        widget._curve = curve

        # Timer d'update
        def update_plot():
            sig_val = self.signals_values.get(signal_name, [])
            if not sig_val:
                return

            if widget._t0 is None:
                for sig_info in sig_val:
                    if sig_info:
                        widget._t0 = sig_info[2]
                        break
                if widget._t0 is None:
                    return

            for sig_info in sig_val:
                if sig_info and sig_info[2] >= widget._t0:
                    t_sec = (sig_info[2] - widget._t0) / 1e9
                    if not widget._times or t_sec > widget._times[-1]:
                        widget._times.append(t_sec)
                        widget._values.append(sig_info[1])

            # Limiter les points affichés
            if len(widget._times) > PLOT_MAX_POINT:
                widget._times = widget._times[-PLOT_MAX_POINT:]
                widget._values = widget._values[-PLOT_MAX_POINT:]

            widget._curve.setData(widget._times, widget._values)

        # Timer Qt
        timer = QTimer(widget)
        timer.timeout.connect(update_plot)
        timer.start(50)  # toutes les 50 ms (~20 FPS)
        widget._timer = timer

    #--------------------------
    # __close_tab
    #--------------------------
    def __close_tab(self, f_widget):
        index = self.tab_widget.indexOf(f_widget)
        if index != -1:
            self.tab_widget.removeTab(index)
            f_widget._timer.stop()  # stop timer for this graph
            f_widget.deleteLater()
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

