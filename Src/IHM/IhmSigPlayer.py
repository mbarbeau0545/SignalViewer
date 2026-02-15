"""
#  @file        IhmSigPlayer.py
#  @brief       Signal Player - replay .log files produced by FrameMngmt.
#
#  Expected log line (robust parsing):
#      <time> <signal_name> <raw_value> <value>
#  where <value> can be numeric or an enum string (possibly with spaces).
"""
import os
import re
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QLineEdit,
    QFileDialog, QListWidget, QListWidgetItem, QAbstractItemView, QCheckBox,
    QComboBox, QSpinBox
)
from PyQt5.QtCore import Qt, QTimer

import pyqtgraph as pg


@dataclass
class LogEvent:
    t_sec: float
    sig: str
    raw: float
    val: float
    val_is_raw_fallback: bool


class SignalPlayerWidget(QWidget):
    """
    Widget that loads a .log file and replays selected signals on a plot.

    - Multi-signal plot in a single window (legend).
    - Loop playback.
    - Speed factor.
    - Robust parsing (tolerates prefixes before the 4 fields).
    """
    _RE_NUM = re.compile(r'^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$')

    def __init__(self, parent=None):
        super().__init__(parent)

        self._events: List[LogEvent] = []
        self._sig_set: List[str] = []
        self._selected: Dict[str, bool] = {}

        self._play_idx: int = 0
        self._t0: Optional[float] = None
        self._loop: bool = True
        self._speed: float = 1.0
        self._playing: bool = False

        self._curves: Dict[str, Dict] = {}

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(50)

        self._build_ui()

    # ---------------- UI ----------------
    def _build_ui(self) -> None:
        root = QVBoxLayout(self)

        # --- Top controls ---
        top = QHBoxLayout()

        self.btn_load = QPushButton("Load LOG")
        self.btn_load.clicked.connect(self._on_load_log)
        top.addWidget(self.btn_load)

        self.lbl_file = QLabel("No file")
        self.lbl_file.setTextInteractionFlags(Qt.TextSelectableByMouse)
        top.addWidget(self.lbl_file, 1)

        self.chk_loop = QCheckBox("Loop")
        self.chk_loop.setChecked(True)
        self.chk_loop.stateChanged.connect(lambda _: self._set_loop())
        top.addWidget(self.chk_loop)

        top.addWidget(QLabel("Speed:"))
        self.cmb_speed = QComboBox()
        self.cmb_speed.addItems(["0.25x", "0.5x", "1x", "2x", "4x", "8x"])
        self.cmb_speed.setCurrentText("1x")
        self.cmb_speed.currentTextChanged.connect(self._set_speed)
        top.addWidget(self.cmb_speed)

        self.btn_play = QPushButton("Play")
        self.btn_play.clicked.connect(self._toggle_play)
        top.addWidget(self.btn_play)

        self.btn_rewind = QPushButton("Rewind")
        self.btn_rewind.clicked.connect(self._rewind)
        top.addWidget(self.btn_rewind)

        root.addLayout(top)

        # --- Search + selection + plot ---
        mid = QHBoxLayout()

        left = QVBoxLayout()
        search_row = QHBoxLayout()
        search_row.addWidget(QLabel("Filter:"))
        self.le_filter = QLineEdit()
        self.le_filter.setPlaceholderText("Type to filter signals")
        self.le_filter.textChanged.connect(self._apply_filter)
        search_row.addWidget(self.le_filter, 1)
        left.addLayout(search_row)

        self.list_sig = QListWidget()
        self.list_sig.setSelectionMode(QAbstractItemView.NoSelection)
        self.list_sig.itemChanged.connect(self._on_item_changed)
        left.addWidget(self.list_sig, 1)

        btns = QHBoxLayout()
        self.btn_all = QPushButton("All")
        self.btn_none = QPushButton("None")
        self.btn_all.clicked.connect(lambda: self._select_all(True))
        self.btn_none.clicked.connect(lambda: self._select_all(False))
        btns.addWidget(self.btn_all)
        btns.addWidget(self.btn_none)
        left.addLayout(btns)

        mid.addLayout(left, 0)

        self.plot = pg.PlotWidget(title="Signal Player")
        self.plot.setLabel('bottom', 'Time', units='s')
        self.plot.setLabel('left', 'Value')
        self.plot.addLegend()
        mid.addWidget(self.plot, 1)

        root.addLayout(mid, 1)

    # ---------------- Parsing ----------------
    def _on_load_log(self) -> None:
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Open LOG file",
            "",
            "LOG files (*.log *.txt);;All files (*)"
        )
        if not file_path:
            return

        self.load_log(file_path)

    def load_log(self, file_path: str) -> None:
        self.lbl_file.setText(os.path.basename(file_path))

        events: List[LogEvent] = []
        sigs = set()

        with open(file_path, "r", encoding="utf-8", errors="ignore") as fh:
            for line in fh:
                ev = self._parse_line(line)
                if ev is None:
                    continue
                events.append(ev)
                sigs.add(ev.sig)

        events.sort(key=lambda e: e.t_sec)
        self._events = events
        self._sig_set = sorted(list(sigs), key=lambda s: s.lower())

        self._selected = {s: False for s in self._sig_set}
        self._rebuild_signal_list()

        self._rewind()
        self._clear_plot()

    def _parse_line(self, line: str) -> Optional[LogEvent]:
        """
        Robust parsing:
        - Ignore empty lines.
        - Find the first token that looks like a number => time
        - Next token => signal
        - Next token => raw
        - Remainder => value (numeric or enum string)
        """
        s = line.strip()
        if not s:
            return None

        parts = s.split()
        if len(parts) < 4:
            return None

        # find index of the time token (first numeric)
        idx_t = None
        for i, p in enumerate(parts):
            if self._RE_NUM.match(p):
                idx_t = i
                break
        if idx_t is None:
            return None
        if len(parts) <= idx_t + 2:
            return None

        t_tok = parts[idx_t]
        sig_tok = parts[idx_t + 1]
        raw_tok = parts[idx_t + 2]
        val_tok = " ".join(parts[idx_t + 3:])  # may contain spaces

        if not self._RE_NUM.match(raw_tok):
            return None

        try:
            t = float(t_tok)
            raw = float(raw_tok)
        except ValueError:
            return None

        # value: numeric -> float, else fallback to raw for plotting
        val_is_raw_fallback = False
        if self._RE_NUM.match(val_tok):
            try:
                val = float(val_tok)
            except ValueError:
                val = raw
                val_is_raw_fallback = True
        else:
            val = raw
            val_is_raw_fallback = True

        # normalize time -> seconds relative
        t_sec = self._normalize_time_to_seconds(t)

        return LogEvent(t_sec=t_sec, sig=sig_tok, raw=raw, val=val, val_is_raw_fallback=val_is_raw_fallback)

    def _normalize_time_to_seconds(self, t: float) -> float:
        """
        Heuristic:
        - If values look like ns (very large): use /1e9
        - If values look like us: /1e6
        - If values look like ms: /1e3
        - Else assume already seconds.
        """
        at = abs(t)
        if at >= 1e12:
            return t / 1e9
        if at >= 1e9:
            # ambiguous; most of your timestamps here are ns-like from drivers
            return t / 1e9
        if at >= 1e6:
            return t / 1e3  # likely ms in log (e.g. serial: (ns)/1e6 => ms)
        if at >= 1e3:
            return t / 1e3  # ms
        return t

    # ---------------- Selection ----------------
    def _rebuild_signal_list(self) -> None:
        self.list_sig.blockSignals(True)
        self.list_sig.clear()

        for sig in self._sig_set:
            item = QListWidgetItem(sig)
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Checked if self._selected.get(sig, False) else Qt.Unchecked)
            self.list_sig.addItem(item)

        self.list_sig.blockSignals(False)
        self._apply_filter()

    def _apply_filter(self) -> None:
        q = self.le_filter.text().strip().lower()
        for i in range(self.list_sig.count()):
            it = self.list_sig.item(i)
            name = it.text().lower()
            it.setHidden(bool(q) and (q not in name))

    def _on_item_changed(self, item: QListWidgetItem) -> None:
        sig = item.text()
        self._selected[sig] = (item.checkState() == Qt.Checked)
        self._sync_curves_with_selection()

    def _select_all(self, checked: bool) -> None:
        self.list_sig.blockSignals(True)
        for i in range(self.list_sig.count()):
            it = self.list_sig.item(i)
            it.setCheckState(Qt.Checked if checked else Qt.Unchecked)
            self._selected[it.text()] = checked
        self.list_sig.blockSignals(False)
        self._sync_curves_with_selection()

    # ---------------- Playback ----------------
    def _set_loop(self) -> None:
        self._loop = self.chk_loop.isChecked()

    def _set_speed(self, txt: str) -> None:
        try:
            self._speed = float(txt.replace("x", ""))
        except ValueError:
            self._speed = 1.0

    def _toggle_play(self) -> None:
        self._playing = not self._playing
        self.btn_play.setText("Pause" if self._playing else "Play")

    def _rewind(self) -> None:
        self._play_idx = 0
        self._t0 = None
        for sig in self._curves.keys():
            self._curves[sig]["times"].clear()
            self._curves[sig]["values"].clear()
            self._curves[sig]["curve"].setData([], [])

    def _tick(self) -> None:
        if (not self._playing) or (not self._events):
            return

        # establish t0 once
        if self._t0 is None:
            self._t0 = self._events[0].t_sec

        # advance playhead based on real-time timer interval
        dt = (self._timer.interval() / 1000.0) * self._speed
        playhead = (self._events[self._play_idx].t_sec - self._t0) if self._play_idx < len(self._events) else 0.0
        target = playhead + dt

        # consume events until we reach target time
        while self._play_idx < len(self._events):
            ev = self._events[self._play_idx]
            t_rel = ev.t_sec - self._t0
            if t_rel > target:
                break

            if self._selected.get(ev.sig, False) and ev.sig in self._curves:
                data = self._curves[ev.sig]
                data["times"].append(t_rel)
                data["values"].append(ev.val)

            self._play_idx += 1

        # update curves
        for sig, data in self._curves.items():
            if len(data["times"]) > 1:
                data["curve"].setData(data["times"], data["values"])

        # end reached
        if self._play_idx >= len(self._events):
            if self._loop:
                self._rewind()
            else:
                self._playing = False
                self.btn_play.setText("Play")

    def _sync_curves_with_selection(self) -> None:
        # remove unchecked
        for sig in list(self._curves.keys()):
            if not self._selected.get(sig, False):
                self.plot.removeItem(self._curves[sig]["curve"])
                del self._curves[sig]

        # add checked
        for sig, en in self._selected.items():
            if en and sig not in self._curves:
                curve = self.plot.plot([], [], pen=pg.intColor(len(self._curves)), name=sig)
                self._curves[sig] = {"curve": curve, "times": [], "values": []}

        # on selection change, rewind to avoid mixing old state
        self._rewind()

    def _clear_plot(self) -> None:
        for sig in list(self._curves.keys()):
            self.plot.removeItem(self._curves[sig]["curve"])
        self._curves = {}
