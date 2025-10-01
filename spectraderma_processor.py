import pandas as pd
import numpy as np
import pywt
from scipy.signal import butter, sosfiltfilt


class SpectraDermaProcessor:
    """Simple data processor for SpectraDerma signals."""
    
    def __init__(self, sample_rate=30):
        self.sample_rate = sample_rate
        self.channels = None
        self.timestamp = None
        
    def load_data(self, csv_path):
        """Load data from CSV file."""
        data = pd.read_csv(csv_path)
        data = data.ffill()
        self.timestamp = data.index.to_series().to_numpy()
        self.channels = data.iloc[:, data.columns.str.startswith('data_')].to_numpy()
        return self
    
    def swt_denoise(self, wavelet="db2", level=3, zero_levels=(1, 2, 3)):
        """Apply stationary wavelet transform denoising."""
        X = np.asarray(self.channels, float)
        T, C = X.shape
        
        # Pad to multiple of 2**level
        block = 2 ** level
        T_pad = int(np.ceil(T / block) * block)
        pad = T_pad - T
        
        Y = np.empty((T, C), dtype=float)
        zl = {l for l in zero_levels if 1 <= l <= level}
        
        for c in range(C):
            xc = X[:, c]
            if pad > 0:
                xc = np.pad(xc, (0, pad), mode="reflect")
            
            # SWT and zero selected detail levels
            coeffs = pywt.swt(xc, wavelet=wavelet, level=level, norm=True)
            new_coeffs = []
            for j, (cA, cD) in enumerate(coeffs, start=1):
                if j in zl:
                    cD = np.zeros_like(cD)
                new_coeffs.append((cA, cD))
            
            # Inverse transform and crop
            xc_hat = pywt.iswt(new_coeffs, wavelet=wavelet, norm=True)
            if pad > 0:
                xc_hat = xc_hat[:T]
            Y[:, c] = xc_hat
        
        self.channels = Y
        return self
    
    def bandpass_filter(self, low_freq=0.5, high_freq=2.0, order=3):
        """Apply bandpass filter."""
        sos = butter(order, (low_freq, high_freq), 'bandpass', fs=self.sample_rate, output='sos')
        self.channels = sosfiltfilt(sos, self.channels, axis=0)
        return self
    
    def normalize(self, *time_limits_list, amplitude_limits=(-25, 25)):
        """Normalize channels based on one or more time regions."""
        # Collect all masks
        combined_mask = None
        for time_limits in time_limits_list:
            mask = (self.timestamp >= time_limits[0]) & (self.timestamp <= time_limits[1])
            if combined_mask is None:
                combined_mask = mask
            else:
                combined_mask = combined_mask | mask
        
        normalized = self.channels.copy()
        
        for i in range(self.channels.shape[1]):
            region_data = self.channels[combined_mask, i]
            if len(region_data) > 0:
                data_min = np.min(region_data)
                data_max = np.max(region_data)
                if data_max != data_min:
                    normalized[:, i] = amplitude_limits[0] + (self.channels[:, i] - data_min) * (amplitude_limits[1] - amplitude_limits[0]) / (data_max - data_min)
                else:
                    normalized[:, i] = amplitude_limits[0]
        
        self.channels = normalized
        return self
    
    def process_pipeline(self, csv_path):
        """Run complete processing pipeline."""
        return (self.load_data(csv_path)
                   .swt_denoise()
                   .bandpass_filter())
    
    def get_data(self):
        """Get processed data."""
        return self.timestamp, self.channels


# Example usage:
if __name__ == "__main__":
    processor = SpectraDermaProcessor()
    
    # Define time limits
    t1, t2, d = 2450, 389100, 300
    time_limits_1 = (t1, t1 + d)
    time_limits_2 = (t2, t2 + d)
    
    # Process data
    processor.process_pipeline('data/spectraderama_data_2025-04-02_sattac555S2.csv')
    
    # Normalize for plotting (separate step) - can pass multiple time regions
    processor.normalize(time_limits_1, time_limits_2)
    
    # Get results
    timestamp, channels = processor.get_data()
    print(f"Processed data shape: {channels.shape}")
