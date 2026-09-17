# FT 13/8/26

import numpy as np
from ardupilot_log_reader import Ardupilot
from scipy.interpolate import make_interp_spline


# Load the log file
class LogFileLoader:
    # 'PERF' is a custom type for data from the performance characterisation script
    # 'channel_aug_fns' are applied to the channels to modify them.
    # E.g. to get altitude, have to multiply PD by -1. Might also
    # want to convert radians to degrees, etc.
    def __init__(self,
                 path,
                 types=['STAT', 'RCOU', 'XKF1', 'ARSP', 'AOA', 'ATT', 'IMU', 'AETR', 'TECS', 'AHR2',
                            'RFND', 'POS', 'BARO', 'GPS', 'LAND', 'PERF', 'DCM', 'MISE', 'MODE', 
                                'BAT'],
                channel_aug_fns={}):
        
        self.log_file_path = path
        self.types = types
        self.channel_aug_fns = channel_aug_fns

        self.interps = {}   # Computed interpolators

        # Parse flight logs
        self.logs = self._get_flight_logs()

        # Get start times and end times
        stat_times = np.array(self.logs.dfs['STAT']['TimeUS']) / 1e6 # ['timestamp'])
        self.start_time_s = stat_times[0]
        self.end_time_s = stat_times[-1]
        self.total_time_s = self.end_time_s - self.start_time_s

    @property
    def start_s(self):
        return self.start_time_s

    @property
    def end_s(self):
        return self.end_time_s
    
    # Duration
    @property
    def dur_s(self):
        return self.total_time_s

    def _get_flight_logs(self):
        print("Parsing flight logs... ", end="")
        parser = Ardupilot.parse(self.log_file_path, types=self.types)
        print("done")
        
        return parser
    
    def _construct_interpolator(self, log_type, chan_name, order, filters):
        # Get channel augmentation function, if present.
        aug_fn = self.channel_aug_fns.get((log_type, chan_name), lambda x: x)

        log_df = self.logs.dfs[log_type]

        # Apply filter
        filter_arr = [True]*len(log_df)
        if filters:
            for filter_chan_name, filter_val in filters.items():
                filter_arr = np.bitwise_and(filter_arr, np.array(log_df[filter_chan_name] == filter_val))

        times_s = np.array(log_df['TimeUS'][filter_arr]) / 1e6
        vals = np.array(log_df[chan_name][filter_arr])

        if len(np.unique(times_s)) < len(times_s):
            raise Exception("Time values are not unique")
        
        # Create interpolator
        interp = make_interp_spline(times_s, aug_fn(vals), k=order)

        return interp
    
    # Gets interpolator
    # filter: None / {channel, value} dict. At the moment can only filter by exact values (no </>).
    def get(self, log_type, chan_name, order=1, filters=None):
        key = (log_type, chan_name, order, filter)

        if key in self.interps:
            return self.interps[key]
        else:
            interp = self._construct_interpolator(log_type, chan_name, order, filters)
            # Cache
            self.interps[key] = interp
            return interp
    
    # Gets samples, calculated from interpolator.
    def sample(self, log_type, chan_name, dt=0.01, order=1, filters=None):
        sample_times = np.arange(self.start_time_s, self.end_time_s, dt)
        return self.get(log_type, chan_name, order, filters)(sample_times)


# unique_times, unique_idx = np.unique(np.array(self.logs.dfs[log_type]['TimeUS']) / 1e6, return_index=True)
# interp_vals = np.array(self.logs.dfs[log_type][key])[unique_idx]
# # interp = make_interp_spline(unique_times - self.start_time_s, f(interp_vals), k=k)
# interp = make_interp_spline(unique_times - self.start_time_s, f(interp_vals), k=k)

# self.name = path.name