import sys
sys.path.append('../src/accelmon')
sys.path.append('./src/accelmon')

import board
from sinks import SpectrumPlotSink, CsvSampleSink, H5SampleSink, NpH5SampleSink

import threading
import os
import random
import datetime
import serial.tools.list_ports
from PySide6 import QtCore, QtWidgets, QtGui
from PySide6.QtCore import Qt

import numpy as np

from matplotlib.backends.backend_qtagg import FigureCanvas
from matplotlib.figure import Figure

class SpectrumCanvas(FigureCanvas):
    def __init__(self, parent=None, dpi=100, x_lim=[0,100], y_lim=[0,50]):
        self.figure = Figure(dpi=dpi)
        self.axes = self.figure.add_subplot()
        super().__init__(self.figure)

        self.setParent(parent)

        self._plot_ref = None
        self._nx = 0
        self._text_ref = None

        self.axes.set_xlim(x_lim)
        self.axes.set_ylim(y_lim)

        # Graph title text
        self.axes.set_title('Spectrum')

        # Axes labels text
        self.axes.set_ylabel('amplitude (a.u.)')
        self.axes.set_xlabel('freq (Hz)')

        self.draw()

        self.use_blit = True
        self.ax0_background = None
        if self.use_blit:
            self.ax0_background = self.copy_from_bbox(self.axes.bbox)
        
    def update_x_lim(self, x_lim):
        self.axes.set_xlim(x_lim)
        if self._text_ref is not None:
            self._text_ref.set_visible(False)
            self._text_ref = None

    def update_plot_data(self, time_lbl, freq, Z):
        if self._plot_ref is None:
            self._plot_ref,  = self.axes.plot(freq, np.abs(Z))
            self._nx = len(freq)
        elif len(freq) == self._nx:
            self._plot_ref.set_ydata(np.abs(Z))
        else:
            self._plot_ref.set_data(freq, np.abs(Z))
            self._nx = len(freq)

        if self._text_ref is None:
            x_lim = self.axes.get_xlim()
            y_lim = self.axes.get_ylim()
            self._text_ref = self.axes.text(0.98*x_lim[1],0.98*y_lim[1], f't={time_lbl:.2f}s', ha="right", va="top")
        else: 
            self._text_ref.set_text(f't={time_lbl:.2f}s')

        if self.use_blit:
            self.restore_region(self.ax0_background)
            self.axes.draw_artist(self._plot_ref)
            self.axes.draw_artist(self._text_ref)
            self.blit(self.axes.bbox)
        else:
            self.draw()
        

class MonitorSettingsWidget(QtWidgets.QWidget):

    port_changed_signal = QtCore.Signal(str)

    def __init__(self, parent=None):
        super(MonitorSettingsWidget, self).__init__(parent)

        # Grid layout
        # label | field | button/chk | status field

        # select the port to connect to

        self.lbl_port = QtWidgets.QLabel('port',parent=self)
        self.lbl_port.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.cb_port = QtWidgets.QComboBox(self)
        self.btn_port_refresh = QtWidgets.QToolButton(self)
        self.btn_port_refresh.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_BrowserReload))
        self.btn_port_refresh.setToolTip("Refresh")
        self.btn_port_refresh.clicked.connect(self.refresh_port_list)
        self.lbl_board_id_info = QtWidgets.QLabel('board ID: unknown')
        self.lbl_board_id_info.setEnabled(False)    # grey
        self.cb_port.currentIndexChanged.connect(self.on_port_changed)
        self.refresh_port_list()    # load the combobox

        hlport = QtWidgets.QHBoxLayout()
        hlport.addWidget(self.btn_port_refresh)
        hlport.addStretch(1)

        # configure ADC
        self.lbl_sample_rate = QtWidgets.QLabel('sample rate (ksps)',parent=self)
        self.lbl_sample_rate.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.txt_sample_rate = QtWidgets.QLineEdit('20.0', parent=self)
        self.txt_sample_rate.setValidator(QtGui.QDoubleValidator(5,300,2))
        self.btn_sample_rate = QtWidgets.QToolButton(self)
        self.btn_sample_rate.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_DialogApplyButton))
        self.btn_sample_rate.setToolTip("Apply")
        
        hlsamp = QtWidgets.QHBoxLayout()
        hlsamp.addWidget(self.btn_sample_rate)
        hlsamp.addStretch(1)

        self.lbl_ADC_config_details = QtWidgets.QLabel('ADC: unset', parent=self)
        self.lbl_ADC_config_details.setEnabled(False)
        self.sample_rate_editing_finished()
        self.txt_sample_rate.editingFinished.connect(self.sample_rate_editing_finished)

        # set a data file target
        self.lbl_data_file = QtWidgets.QLabel('data file', parent=self)
        self.lbl_data_file.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.edit_data_file = QtWidgets.QLineEdit(self)
        self.chk_data_file_timestamp = QtWidgets.QCheckBox('append timestamp',parent=self)
        self.lbl_data_file_actual = QtWidgets.QLabel('none', parent=self)
        self.lbl_data_file_actual.setEnabled(False)
        self.edit_data_file.editingFinished.connect(self.invalidate_actual_data_file)
        self.chk_data_file_timestamp.clicked.connect(self.invalidate_actual_data_file)

        self.lbl_working_path = QtWidgets.QLabel('working path')
        self.lbl_working_path.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.lbl_cwd = QtWidgets.QLabel(os.getcwd(), parent=self)
        self.lbl_cwd.setEnabled(False)

        layout = QtWidgets.QGridLayout(self)
        layout.addWidget(self.lbl_port, 0, 0)
        layout.addWidget(self.cb_port, 0, 1)
        layout.addLayout(hlport, 0, 2)
        layout.addWidget(self.lbl_board_id_info, 0, 3)

        layout.addWidget(self.lbl_sample_rate, 1, 0)
        layout.addWidget(self.txt_sample_rate, 1, 1)
        layout.addLayout(hlsamp, 1, 2)
        layout.addWidget(self.lbl_ADC_config_details, 1, 3)

        layout.addWidget(self.lbl_data_file, 2, 0)
        layout.addWidget(self.edit_data_file, 2, 1)
        layout.addWidget(self.chk_data_file_timestamp, 2, 2)
        layout.addWidget(self.lbl_data_file_actual, 2, 3)

        layout.addWidget(self.lbl_working_path, 3, 0)
        layout.addWidget(self.lbl_cwd, 3, 1)

    def current_adc_config(self):
        """Return the value of the current sample rate field or None if no board is connected"""
        if self.cb_port.currentIndex() < 0:
            return None
        val = float(self.txt_sample_rate.text())
        return board.ClockSettings(r_sample = int(val*1000), N_min=8, N_max=25)

    def get_data_file_name(self):
        filepath = self.lbl_data_file_actual.text()
        errmsg = self._validate_data_file_name(filepath)
        return filepath if not errmsg else None

    @QtCore.Slot()
    def invalidate_actual_data_file(self):
        path, filename_ext = os.path.split(self.edit_data_file.text())
        filename, ext = os.path.splitext(filename_ext)
        if not filename:
            self.lbl_data_file_actual.clear()
            self.data_file_is_valid_signal.emit(False)
        else:
            if self.chk_data_file_timestamp.isChecked():
                now = datetime.datetime.now()
                filename += now.strftime("-%Y-%m-%d-%H-%M-%S")
            if ext != ".h5" and ext != ".csv":
                filename += ext
                ext = ".h5" 

            new_filename = os.path.join(path,filename+ext)
            self.lbl_data_file_actual.setText(new_filename)
            errmsg = self._validate_data_file_name(new_filename)
            if errmsg:
                red = QtGui.QColor(255,0,0)
                self.lbl_data_file_actual.setStyleSheet("color:{}".format(red.name()))
                self.lbl_data_file_actual.setToolTip(errmsg)
            else:
                self.lbl_data_file_actual.setStyleSheet("")
                self.lbl_data_file_actual.setToolTip("")

    @QtCore.Slot()
    def on_board_id_changed(self, id):
        if id: 
            self.lbl_board_id_info.setText(f"board ID: 0x{id}")
        else:
            self.lbl_board_id_info.setText(f"not connected")

    @QtCore.Slot()
    def on_port_changed(self, index):
        self.port_changed_signal.emit(self.cb_port.currentText())

    @QtCore.Slot()
    def sample_rate_editing_finished(self):
        clk = self.current_adc_config()
        if clk is None:
            self.lbl_ADC_config_details.setText("*ADC: not connected")
        else:
            fconv_ksps = round(clk.f_conversion()/1000.0,2)
            self.txt_sample_rate.setText(f'{fconv_ksps:.2f}')
            Tconv_us = round(clk.T_conversion()*1e6,3)
            self.lbl_ADC_config_details.setText(
                "*ADC: T={}us, N={}".format(Tconv_us, clk.N))

    @QtCore.Slot()
    def on_adc_config_applied(self):
        current_lbl = self.lbl_ADC_config_details.text()
        if current_lbl[0] == "*":
            self.lbl_ADC_config_details.setText(current_lbl[1:])

    @QtCore.Slot()
    def refresh_port_list(self):
        self.cb_port.clear()
        available_ports = serial.tools.list_ports.comports()
        for port in available_ports:
            self.cb_port.addItem(port.device)

    def _validate_data_file_name(self, new_filename):
        path = os.path.dirname(new_filename)
        if path and not os.path.exists(path):
            return "Path does not exist"
        elif os.path.exists(new_filename):
            return "File already exists"
        else:
            return ""
   
class App(QtWidgets.QWidget):

    board_id_changed_signal = QtCore.Signal(str)
    adc_config_applied = QtCore.Signal()

    def __init__(self, parent=None):
        super(App, self).__init__(parent)

        self.mon = board.Controller()   # no port defined yet

        self.resize(800,600)
        self.title = 'Control Monitor'
        self.setWindowTitle(self.title)

        # control settings widget
        self.wgt_settings = MonitorSettingsWidget(self)

        # this is the Canvas Widget that displays the `figure`
        self.canvas = SpectrumCanvas(self)

        # Just some button connected to `plot` method
        self.button = QtWidgets.QPushButton('Run')

        self.lbl_status = QtWidgets.QLabel('status', parent=self)

        # set the layout
        self.layout = QtWidgets.QVBoxLayout(self)
        # self.layout.addWidget(self.toolbar)
        self.layout.addWidget(self.wgt_settings)
        self.layout.addWidget(self.canvas)
        self.layout.addWidget(self.button)
        self.layout.addWidget(self.lbl_status)

        self.board_id_changed_signal.connect(self.wgt_settings.on_board_id_changed)
        self.wgt_settings.port_changed_signal.connect(self.on_port_changed)
        self.wgt_settings.btn_sample_rate.clicked.connect(self.on_apply_ADC_config)
        self.adc_config_applied.connect(self.wgt_settings.on_adc_config_applied)

        self.monitor_thread = None
        self.timer_thread = None
        self.sinks = []

        self.button.clicked.connect(self.on_run)
        

    @QtCore.Slot()
    def on_port_changed(self, portname):
        if portname:
            self.mon.reset_com_port(portname)
            try: 
                id = self.mon.board_id()
                self.board_id_changed_signal.emit(f'{id:08X}')
            except:
                self.board_id_changed_signal.emit('')
        else:
            self.board_id_changed_signal.emit('')
        
    @QtCore.Slot()
    def on_apply_ADC_config(self):
        clk = self.wgt_settings.current_adc_config()
        if clk is None:
            return
        try: 
            self.mon.load_clk_config(clk)  
            self.adc_config_applied.emit()                  
        except:
            pass

    @QtCore.Slot()
    def on_run(self):
        if self.timer_thread is not None:
            self.timer_thread.cancel()
            self.timer_thread = None

        if self.monitor_thread is not None:
            if self.monitor_thread.is_alive():
                self.mon.stop_collection()
                self.monitor_thread.join()
            self.monitor_thread = None

        clk = self.wgt_settings.current_adc_config()
        if clk is None:
            return  #TODO - raise an error
        
        datafile = self.wgt_settings.get_data_file_name()
        if datafile is None:
            return  #TODO - raise an error

        try:
            board_id = self.mon.board_id()        
        except:
            return #TODO - raise an error
        
        base, ext = os.path.splitext(datafile)
        self.sinks = []
        if ext == ".csv":
            csv = CsvSampleSink(datafile)
            csv.open()
            self.sinks = [csv]
        elif ext == ".h5":
            h5s = NpH5SampleSink(
                filename=datafile,
                T=clk.T_conversion(),
                scaling=3.3/4095.0,
                sample_rate=clk.f_conversion(),
                board_id=board_id,
                SAMPLEN=clk.N,
                DIV=clk.D,
                PRESCALER=clk.P)
            h5s.open()
            self.sinks = [h5s]

        spectrum = SpectrumPlotSink(
            chart=self.canvas, 
            dT=clk.T_conversion(), 
            T_window=0.2, 
            T_step=1.0,
            scaling=3.3/4095.0)
        self.canvas.update_x_lim(spectrum.get_x_lim())
        fvec = spectrum.get_freq()
        self.canvas.update_plot_data(
            time_lbl=0, freq=fvec, Z=np.zeros_like(fvec))
        self.sinks.append(spectrum)
        self.mon.set_sinks(self.sinks)

        self.monitor_thread = threading.Thread(target=self.mon.collect_samples, args=(0,))
        self.monitor_thread.start()
    
        self.timer_thread = threading.Timer(10, self.on_halt)
        self.timer_thread.start()
        
        self.button.setEnabled(False)

    @QtCore.Slot()
    def on_halt(self):
        self.mon.stop_collection()
        self.timer_thread.cancel()
        self.monitor_thread.join()
        for s in self.sinks:
            s.close()
            self.lbl_status.setText('sample count: {}'.format(s.sample_count()))
        self.timer_thread = None
        self.monitor_thread = None
        self.button.setEnabled(True)


if __name__ == "__main__":
    app = QtWidgets.QApplication([])

    widget = App()
    widget.show()

    sys.exit(app.exec())