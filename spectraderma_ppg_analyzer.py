import numpy as np
from scipy.signal import find_peaks, peak_widths, resample
import matplotlib.pyplot as plt
import torch

from spectraderma_processor import SpectraDermaProcessor


class SpectraDermaPPGAnalyzer:
    """Simple PPG analyzer using comprehensive pulse detection."""
    
    def __init__(self, sample_rate: float = 50.0):
        self.sample_rate = sample_rate
        self.timestamp = None
        self.channels = None
        self.start_idx = None
        self.end_idx = None
        self.pulse_data = None
        self.peaks = None
        
    def load_data(self, processor: SpectraDermaProcessor):
        """Load data from SpectraDermaProcessor."""
        self.timestamp, self.channels = processor.get_data()
        self.sample_rate = processor.sample_rate
        
    def set_region(self, start_idx: int, end_idx: int):
        """Set the region of interest for analysis."""
        self.start_idx = max(0, start_idx)
        self.end_idx = min(len(self.timestamp), end_idx)
        
    def find_peaks(self, reference_channel=2, tolerance_samples=15):
        """Find peaks using reference channel with cross-channel validation."""
        
        # Extract region data
        region_data = self.channels[self.start_idx:self.end_idx, :]
        n_channels = region_data.shape[1]
        
        # Find peaks in reference channel
        ref_signal = region_data[:, reference_channel]
        ref_peaks, _ = find_peaks(ref_signal, 
                                 height=np.percentile(ref_signal, 70),
                                 distance=int(self.sample_rate * 0.4))
        
        # Find corresponding peaks in all channels
        valid_peaks = []
        
        for ref_peak in ref_peaks:
            peak_group = []
            valid = True
            
            for ch in range(n_channels):
                # Search around reference peak
                start = max(0, ref_peak - tolerance_samples)
                end = min(len(region_data), ref_peak + tolerance_samples + 1)
                search_region = region_data[start:end, ch]
                
                # Find closest peak
                local_peaks, _ = find_peaks(search_region, height=np.percentile(region_data[:, ch], 60))
                if len(local_peaks) == 0:
                    valid = False
                    break
                
                closest_peak = local_peaks[np.argmin(np.abs(local_peaks - (ref_peak - start)))] + start
                peak_group.append(closest_peak)
            
            if valid:
                valid_peaks.append(peak_group)
        
        valid_peaks = np.array(valid_peaks) if valid_peaks else np.array([])
        
        if len(valid_peaks) == 0:
            return np.array([]), np.array([])
            
        # Calculate pulse characteristics
        pulse_data = np.zeros((len(valid_peaks), 3, n_channels))  # [height, interval, width]
        
        for ch in range(n_channels):
            ch_peaks = valid_peaks[:, ch]
            pulse_data[:, 0, ch] = region_data[ch_peaks, ch]
            
            # Intervals 
            if len(ch_peaks) > 1:
                intervals = np.diff(ch_peaks) / self.sample_rate
                pulse_data[1:, 1, ch] = intervals
            
            # Widths
            try:
                widths, _, _, _ = peak_widths(region_data[:, ch], ch_peaks, rel_height=0.5)
                pulse_data[:, 2, ch] = widths / self.sample_rate
            except:
                pulse_data[:, 2, ch] = np.nan
        
        # Store results in class
        self.pulse_data = pulse_data
        self.peaks = valid_peaks
        return pulse_data, valid_peaks
    
    def get_heart_rate(self, channel_idx=2):
        """Get heart rate for specific channel."""
        intervals = self.pulse_data[1:, 1, channel_idx]
        valid_intervals = intervals[~np.isnan(intervals)]
        return 60.0 / np.mean(valid_intervals) if len(valid_intervals) > 0 else 0
    
    def plot_peaks(self, channel_idx=2):
        """Plot signal with detected peaks."""
            
        # Get signal and time
        signal = self.channels[self.start_idx:self.end_idx, channel_idx]
        time = self.timestamp[self.start_idx:self.end_idx]
        ch_peaks = self.peaks[:, channel_idx]
        
        # Calculate heart rate
        heart_rate = self.get_heart_rate(channel_idx)
        
        # Plot
        plt.figure(figsize=(6, 3))
        plt.plot(time, signal, 'b-', linewidth=1.5, label=f'Channel {channel_idx}')
        plt.plot(time[ch_peaks], signal[ch_peaks], 'ro', markersize=8, 
                label=f'Peaks (HR: {heart_rate:.1f} BPM)')
        plt.xlabel('Time')
        plt.ylabel('Amplitude')
        plt.title(f'PPG Analysis - Channel {channel_idx}')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.show()
    
    def segment(self, segment_length=256, channel_idx=2):
        """
        Segment peak waveforms by finding neighboring troughs and resampling to uniform length.
        
        Parameters:
        - segment_length: target length for resampled segments (default: 256)
        - channel_idx: channel to segment (default: 2 for green)
        
        Returns:
        - segments: array of shape (n_peaks, segment_length) with resampled waveforms
        - trough_pairs: array of shape (n_peaks, 2) with [left_trough, right_trough] indices
        """
            
        # Get signal and peaks for the specified channel
        signal = self.channels[self.start_idx:self.end_idx, channel_idx]
        ch_peaks = self.peaks[:, channel_idx]
        
        segments = []
        trough_pairs = []
        
        for peak_idx in ch_peaks:
            # Find left trough (minimum between previous peak and current peak)
            left_search_start = max(0, peak_idx - int(self.sample_rate * 2))  # Search up to 2 seconds back
            if len(segments) > 0:  # If not first peak, start from previous peak
                prev_peak = ch_peaks[len(segments) - 1] if len(segments) < len(ch_peaks) else left_search_start
                left_search_start = max(left_search_start, prev_peak)
            
            left_region = signal[left_search_start:peak_idx]
            left_trough_rel = np.argmin(left_region) if len(left_region) > 0 else 0
            left_trough = left_search_start + left_trough_rel
            
            # Find right trough (minimum between current peak and next peak)
            right_search_end = min(len(signal), peak_idx + int(self.sample_rate * 2))  # Search up to 2 seconds forward
            if len(segments) < len(ch_peaks) - 1:  # If not last peak, end at next peak
                next_peak_idx = np.where(ch_peaks == peak_idx)[0][0] + 1
                if next_peak_idx < len(ch_peaks):
                    next_peak = ch_peaks[next_peak_idx]
                    right_search_end = min(right_search_end, next_peak)
            
            right_region = signal[peak_idx:right_search_end]
            right_trough_rel = np.argmin(right_region) if len(right_region) > 0 else 0
            right_trough = peak_idx + right_trough_rel
            
            # Extract and resample segment
            segment_signal = signal[left_trough:right_trough + 1]
            if len(segment_signal) >= 3:  # Only process valid segments
                resampled_segment = resample(segment_signal, segment_length)
                segments.append(resampled_segment)
                trough_pairs.append([left_trough, right_trough])
        
        return np.array(segments), np.array(trough_pairs)
    
    def plot_segments(self, segments=None, segment_length=256):
        """
        Plot overlaid segments for all channels with averages highlighted.
        
        Parameters:
        - segments: segments array from segment() method. If None, will call segment() automatically
        - segment_length: length of segments for plotting (default: 256)
        """
        # Get segments if not provided
        if segments is None:
            segments, _ = self.segment(segment_length)
        
        if len(segments) == 0:
            print("No segments to plot.")
            return
            
        n_channels = self.channels.shape[1]
        
        # Create 6x1 subplots
        fig, axes = plt.subplots(6, 1, figsize=(6, 18))

        # Ensure we have 6 subplots even if fewer channels
        for ch in range(6):
            ax = axes[ch]
            
            if ch < n_channels:
                # Get segments for this channel by re-running segment method for each channel
                ch_segments, _ = self.segment(segment_length, channel_idx=ch)
                
                if len(ch_segments) > 0:
                    # Plot all individual segments with transparency
                    x = np.arange(segment_length)
                    for i, segment in enumerate(ch_segments):
                        ax.plot(x, segment, 'b-', alpha=0.3, linewidth=0.8)
                    
                    # Calculate and plot average
                    avg_segment = np.mean(ch_segments, axis=0)
                    ax.plot(x, avg_segment, 'r-', linewidth=3, label=f'Average (n={len(ch_segments)})')
                    
                    ax.set_title(f'Channel {ch} - Overlaid Segments')
                    ax.set_ylabel('Amplitude')
                    ax.legend()
                    ax.grid(True, alpha=0.3)
                else:
                    ax.text(0.5, 0.5, f'No segments found for Channel {ch}', 
                           transform=ax.transAxes, ha='center', va='center')
                    ax.set_title(f'Channel {ch} - No Segments')
            else:
                # Empty subplot for channels that don't exist
                ax.text(0.5, 0.5, f'Channel {ch} not available', 
                       transform=ax.transAxes, ha='center', va='center')
                ax.set_title(f'Channel {ch} - Not Available')
            
            if ch == 5:  # Last subplot
                ax.set_xlabel('Sample Index')
        
        plt.tight_layout()
        plt.show()
    
    def compile(self, segment_length=256, save_path="segments.pt"):
        """
        Compile segments from all channels and save as PyTorch tensor with shape (N, T, C).
        
        Parameters:
        - segment_length: target length for resampled segments (default: 256)
        - save_path: path to save the .pt file (default: "segments.pt")
        
        Returns:
        - compiled_segments: tensor of shape (N, T, C) where N=num_segments, T=segment_length, C=num_channels
        """
        if self.peaks is None:
            raise ValueError("No peaks found. Run find_peaks() first.")
        
        n_channels = self.channels.shape[1]
        
        # Get segments for each channel
        all_channel_segments = []
        min_segments = float('inf')
        
        for ch in range(n_channels):
            ch_segments, _ = self.segment(segment_length, channel_idx=ch)
            all_channel_segments.append(ch_segments)
            min_segments = min(min_segments, len(ch_segments))
        
        if min_segments == 0:
            raise ValueError("No valid segments found across channels.")
        
        # Create tensor with shape (N, T, C)
        # N = minimum number of segments across all channels
        # T = segment_length (256)
        # C = number of channels (6)
        compiled_segments = np.zeros((min_segments, segment_length, n_channels))
        
        for ch in range(n_channels):
            # Take only the first min_segments to ensure all channels have same number
            compiled_segments[:, :, ch] = all_channel_segments[ch][:min_segments]
        
        # Convert to PyTorch tensor
        compiled_tensor = torch.from_numpy(compiled_segments).float()
        
        # Save to file
        torch.save(compiled_tensor, save_path)
        
        print(f"Compiled segments saved to '{save_path}'")
        print(f"Shape: {compiled_tensor.shape} (N={min_segments}, T={segment_length}, C={n_channels})")
        print(f"Data type: {compiled_tensor.dtype}")
        
        return compiled_tensor

# Example usage
if __name__ == "__main__":
    # Example with segmentation
    processor = SpectraDermaProcessor()
    processor.process_pipeline('data/spectraderama_data_2025-04-02_sattac555S2.csv')
    
    analyzer = SpectraDermaPPGAnalyzer()
    analyzer.load_data(processor)
    analyzer.set_region(1000, 3000)
    
    # Find peaks
    pulse_data, peaks = analyzer.find_peaks()
    heart_rate = analyzer.get_heart_rate()
    print(f"Found {len(peaks)} peaks, HR: {heart_rate:.1f} BPM")
    
    # Segment waveforms
    segments, trough_pairs = analyzer.segment(segment_length=256)
    print(f"Segmented {len(segments)} waveforms")
    
    # Plot results
    analyzer.plot_peaks()
    
    # Plot overlaid segments
    analyzer.plot_segments(segments)
    
    # Compile and save segments
    compiled_tensor = analyzer.compile(segment_length=256, save_path="example_segments.pt")
    print(f"Compiled tensor shape: {compiled_tensor.shape}")
