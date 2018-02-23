import matplotlib.pyplot as plt
import numpy as np

default_col_list = ['n','t_ms','AcY','AcY_offset','AcZ','roll', \
                    'gyroXrate','compX']


class filter_investigation(object):
    def __init__(self, data_array, col_list=None):
        self.data = data_array

        if col_list is None:
            col_list = default_col_list
        self.col_list = col_list

        self.extract_cols()
        self.t = self.t_ms/1000


    def extract_cols(self):
        N = len(self.col_list)

        for i in range(N):
            cur_attr = self.col_list[i]
            cur_col = self.data[:,i]
            setattr(self,cur_attr,cur_col)


    def gyro_int(self):
        """Pure numeric integration of the gyro rate without 
        the complimentary filter."""
        #compAngleX = 0.99 * (compAngleX + gyroXrate * dt_sec)
        N = len(self.gyroXrate)
        theta_X_pure_int = np.zeros(N)

        for i in range(1,N):
            prev_th = theta_X_pure_int[i-1]
            dt_i = self.t[i] - self.t[i-1]
            next_th = prev_th + self.gyroXrate[i]*dt_i
            theta_X_pure_int[i] = next_th

        self.theta_X_pure_int = theta_X_pure_int


    def _plot(self, attr_list):
        if type(attr_list) == str:
            attr_list = [attr_list]
        plt.figure()

        t = self.t

        for attr in attr_list:
            cur_vect = getattr(self, attr)
            plt.plot(t,cur_vect, label=attr)
            plt.xlabel('Time (sec.)')
            plt.ylabel('Signal Amplitude')


    def get_scales(self, attr_list):
        N = len(attr_list)
        scales = np.zeros(N)

        for i, attr in enumerate(attr_list):
            vect = getattr(self, attr)
            cur_scale = np.abs(vect).max()
            scales[i] = cur_scale

        return scales
    

    def plot_scaled(self, attr_list, scales=None):
        """Plot attributes after scaling them so they overlay nicely."""
        if scales is None:
            scales = self.get_scales(attr_list)

        if type(scales) != dict:
            mydict = dict(zip(attr_list, scales))

        plt.figure()
        t = self.t
        
        for attr in attr_list:
            scale = mydict[attr]
            cur_vect = getattr(self, attr)/scale
            plt.plot(t,cur_vect, label=attr)
            plt.xlabel('Time (sec.)')
            plt.ylabel('Signal Amplitude')


    def save(self, filename):
        txt_mixin.dump_delimited(filename, self.data, \
                                 labels=self.col_list)
            #delim='\t', fmt='%s', 
        


class filter_from_saved_data(filter_investigation):
    def get_labels(self):
        f = open(self.filename,'r')
        lines = f.readlines()
        f.close()
        label_str = lines[0].strip()
        labels = label_str.split(self.delim)
        self.col_list = labels
        
    
    def __init__(self, filename, delim='\t'):
        self.filename = filename
        self.delim = delim
        self.get_labels()
        data = np.loadtxt(filename, skiprows=1, delimiter=self.delim)
        filter_investigation.__init__(self, data, col_list=self.col_list)
