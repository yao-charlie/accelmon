from datetime import datetime, timedelta
import h5py
import pandas as pd
import numpy as np
from scipy.signal import get_window
import threading
import time
import struct

class RawValueConverter:
    """The abstract base class to convert raw values from an ADC to physically
    meaningful numbers for analysis"""
    def conv(self, v, i):
        pass

class NoConversionConverter (RawValueConverter):
    """Do nothing, return the tuple of the value and a zero flag bit"""
    def conv(self, v, i):
        return v, None


class IntervalSignedInt16Converter (RawValueConverter):
    """
    Interpret raw values as signed int 16 and scale to float. 
    The first column is an interval in (us) with the upper bit set if 
    the tach signal is present.
    """

    def __init__(self, cols=4, scaling=1.):
        self.cols = cols
        self.a = scaling

    def conv(self, v, i):
        if i % self.cols == 0:  # first entry in row is interval (us) as uint16_t
            tach_bit = 1 if 0x8000 & int(v) else 0
            return (int(v) & 0x7FFF), tach_bit
        b = int(v).to_bytes(2,'little')
        c = struct.unpack('<h',b)[0]
        return self.a * c, None


class SampleSink:
    """The abstract base class for sinks to record streaming sample data"""

    def write(self, sample):
        """Write sample data to the sink.

        :param sample: The data sample as a iterable container (e.g. list)
        :type sample: list
        :returns: None
        """
        raise NotImplementedError("Abstract method")

    def open(self):
        pass

    def close(self):
        pass

    def sample_count(self):
        return 0

class CsvSampleSink (SampleSink):
    """Write sample data to a text file in comma separated value (CSV) format"""

    def __init__(self, filename, width=1, converter=NoConversionConverter):
        self.filename = filename
        self.hf = None
        self.width = width
        self.n_samples = 0
        self.converter = converter

    def open(self): 
        self.close()
        self.hf = open(self.filename, "w")
        self.n_samples = 0

    def close(self):
        if self.hf is not None:
            self.hf.close()
        self.hf = None

    def write(self, sample):
        for s in sample:
            v, b = self.converter.conv(s,self.n_samples)
            if b is not None:
                self.hf.write(f"{b},")
            self.hf.write(f"{v}")
            self.n_samples += 1
            if self.n_samples % self.width == 0:
                self.hf.write("\n")
            else:
                self.hf.write(",")

    def sample_count(self):
        return self.n_samples

class ListSampleSink (SampleSink):
    """Write sample data to an array"""

    def __init__(self):
        self.data = []

    def write(self, sample):
        self.data.extend(sample)

    def sample_count(self):
        return len(self.data)

class NpArraySampleSink(SampleSink):
    """Write sample data to a Numpy Array"""

    def __init__(self, T, scaling=1, width=1):
        self.timestamp = datetime.now()
        self.delta_ns = round(T*1e9)
        self.resize_count = 0
        self.resize_stride = int(1.0/T)
        self.n_resizes = 1

        self.signal = np.zeros((self.resize_stride, width), dtype=np.float32)
        self.ndx = 0
        self.scaling = scaling
        self.raw_T = T

    def write(self, sample):
        width = self.signal.shape[1]
        if len(sample)+self.resize_count >= self.resize_stride:
            self.n_resizes += 1
            self.signal = np.resize(self.signal, ((self.n_resizes*self.resize_stride,width)))
            self.resize_count -= self.resize_stride
        self.resize_count += len(sample)

        for s in sample:
            r = self.ndx // width
            c = self.ndx % width
            self.signal[r, c] = s * self.scaling
            self.ndx += 1

    def sample_count(self):
        return self.ndx


class DataFrameSampleSink(SampleSink):
    """Write sample data to a Pandas DataFrame"""

    T_NAME = 'timestamp'
    S_NAME = 'signal'

    def __init__(self, T, scaling=1):
        self.timestamp = pd.Timestamp.now()
        self.delta = pd.Timedelta(round(T*1e9), unit="ns")
        self.resize_count = 0
        self.resize_stride = int(1.0/T)

        self.df = pd.DataFrame(columns=[self.T_NAME, self.S_NAME], dtype=np.float32)
        self.df.reindex(index=range(self.resize_stride), columns=self.df.columns)
        self.ndx = 0
        self.scaling = scaling
        self.raw_T = T

    def write(self, sample):
        if len(sample)+self.resize_count >= self.resize_stride:
            self.df.reindex(index=range(len(self.df) + self.resize_stride), columns=self.df.columns)
            self.resize_count -= self.resize_stride
        self.resize_count += len(sample)
        for s in sample:
            self.df.iloc[self.ndx] = [self.timestamp, s * self.scaling]
            self.ndx += 1
            self.timestamp += self.delta

    def sample_count(self):
        return self.ndx

class H5SampleSink(DataFrameSampleSink):
    """Write sample data to an HDF5 file"""

    def __init__(self, filename, T, scaling=1, **kwargs):
        super(H5SampleSink,self).__init__(T=T,scaling=scaling)
        self.filename = filename
        self.extra_attrs = kwargs
        self.extra_attrs.update({
            'timestamp': self.timestamp,
            'T': self.raw_T,
            'scaling' : self.scaling})
        self.store = None

    def open(self):
        self.close()

        self.store = pd.HDFStore(self.filename) 
        metadata = pd.DataFrame([self.extra_attrs])
        self.store.put('/signal/attrs', metadata)

    # def write(self, sample):
    #     istart = len(self.df)
    #     super().write(sample)
    #     # use a new dataframe to efficiently append to the file
    #     df_ext = self.df.iloc[istart:]
    #     self.store.append('/signal/data', df_ext)

    def close(self):
        if self.store is not None:
            self.store.put('/signal/data', self.df)
            self.store.close()
        self.store = None

class NpH5SampleSink(NpArraySampleSink):
    """Write sample data to an HDF5 file"""

    def __init__(self, filename, T, scaling=1, **kwargs):
        super().__init__(T=T,scaling=scaling)
        self.filename = filename
        self.extra_attrs = kwargs
        self.extra_attrs.update({
            'timestamp': self.timestamp.strftime('%Y-%m-%d %H:%M:%S.%f'),
            'T': self.raw_T,
            'scaling' : self.scaling})
        self.store = None

    def open(self):
        pass

    def close(self):
        with h5py.File(self.filename,'w') as f:
            N = self.sample_count()
            grp = f.create_group('/signal')
            dset = grp.create_dataset('data', shape=(N,))
            dset[:] = self.signal[0:N]
            grp.attrs.update(self.extra_attrs)           


class SpectrumPlotSink(NpArraySampleSink):

    def __init__(self, chart, dT, T_window, T_step=None, scaling=1.0, window_type='tukey'):
        super().__init__(T=dT,scaling=scaling)
        self.chart = chart
        self.dT = dT
        self.nperwindow = int(T_window/dT)
        self.nperstride = self.nperwindow // 2 if T_step is None else int(T_step/dT)
        self.stride_count = 0
        #print('dT={}, nperwin={}, nperstr={}'.format(self.dT, self.nperwindow, self.nperstride))
        self.window = get_window(window_type, self.nperwindow)
        self.freq = np.fft.rfftfreq(self.nperwindow, d=self.dT)
        self.itail = 0
        self.ihead = self.nperwindow

        self.t_write = 0
        self.t_fft = 0
        self.t_draw = 0
        self.draw_count = 0
    
    def get_freq(self):
        return self.freq
    
    def get_x_lim(self):
        return [self.freq[0], self.freq[-1]]

    def write(self, sample):
        t_start_write = time.perf_counter_ns()
        super().write(sample)
        t_stop_write = time.perf_counter_ns()
        self.t_write = t_stop_write - t_start_write
        if self.sample_count() > self.ihead:
            t_start_fft = t_stop_write
            zz = np.fft.rfft(self.signal[self.itail:self.ihead])
            t_done_fft = time.perf_counter_ns()
            self.ihead += self.nperstride
            self.itail += self.nperstride
            current_time = (self.nperwindow + self.stride_count*self.nperstride)*self.dT
            #print(f't={current_time:.2f}s')
            self.chart.update_plot_data(current_time, self.freq, zz)
            self.stride_count += 1

            self.t_fft += t_done_fft - t_start_fft
            self.t_draw += time.perf_counter_ns() - t_done_fft
            self.draw_count += 1

    def close(self):
        avg_scale = 1e-6/self.draw_count
        print("t_avg_fft={}ms, t_avg_draw={}ms, t_avg_write={}ms".format(
            self.t_fft*avg_scale, 
            self.t_draw*avg_scale,
            self.t_write*avg_scale))

        


