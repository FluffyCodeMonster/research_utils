# FT 13/8/26

import numpy as np
from ardupilot_log_reader import Ardupilot
from scipy.interpolate import make_interp_spline


# Load the log file
class LogFileLoader:
    # 'PERF' is a custom type for data from the performance characterisation script
    def __init__(self, path, types=['STAT', 'RCOU', 'XKF1', 'ARSP', 'AOA', 'ATT', 'IMU', 'AETR', 'TECS', 'AHR2', 
                                    'RFND', 'POS', 'BARO', 'GPS', 'LAND', 'PERF', 'DCM', 'MISE', 'MODE', 
                                    'BAT'],
                                    channel_aug_fns={}):
        self.log_file_path = path
        self.name = path.name
        self.types = types
        self.channel_aug_fns = channel_aug_fns

        self.logs = self.get_flight_logs()

        # Get start times and end times
        stat_times = np.array(self.logs.dfs['STAT']['TimeUS']) / 1e6 # ['timestamp'])
        self.start_time_s = stat_times[0]
        self.end_time_s = stat_times[-1]
        self.total_time_s = self.end_time_s - self.start_time_s
    
    def get_flight_logs(self):
        print("Parsing flight logs... ", end="")
        parser = Ardupilot.parse(self.log_file_path, types=self.types)
        print("done")
        
        return parser
    
    # f() is a function, for if it's necessary. E.g. to get altitude, have to multiply PD by -1. Might also
    # want to convert radians to degrees, etc.
    def construct_interpolator(self, log_type, key, k=1): # , f=lambda x: x):
        # Get channel augmentation function, if present.
        f = self.channel_aug_fns.get((log_type, key), lambda x: x)
        
        # unique_times, unique_idx = np.unique(np.array(logs.dfs[log_type]['timestamp']), return_index=True)
        unique_times, unique_idx = np.unique(np.array(self.logs.dfs[log_type]['TimeUS']) / 1e6, return_index=True)
        interp_vals = np.array(self.logs.dfs[log_type][key])[unique_idx]
        # interp = make_interp_spline(unique_times - self.start_time_s, f(interp_vals), k=k)
        interp = make_interp_spline(unique_times - self.start_time_s, f(interp_vals), k=k)
        return interp