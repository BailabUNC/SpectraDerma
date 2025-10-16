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
    
    def _find_reference_peaks(self, reference_channel=2):
        """Find peaks in the reference channel."""
        region_data = self.channels[self.start_idx:self.end_idx, :]
        ref_signal = region_data[:, reference_channel]
        ref_peaks, _ = find_peaks(ref_signal, 
                                 height=np.percentile(ref_signal, 70),
                                 distance=int(self.sample_rate * 0.4))
        return ref_peaks, region_data
    
    def _validate_cross_channel_peaks(self, ref_peaks, region_data, tolerance_samples=15):
        """Find corresponding peaks in all channels for cross-channel validation."""
        n_channels = region_data.shape[1]
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
        
        return np.array(valid_peaks) if valid_peaks else np.array([])
    
    def _calculate_pulse_features(self, valid_peaks, region_data):
        """Calculate pulse characteristics (heights, intervals, widths) from peaks."""
        if len(valid_peaks) == 0:
            return np.array([])
            
        n_channels = region_data.shape[1]
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
        
        return pulse_data
        
    def find_peaks(self, reference_channel=2, tolerance_samples=15):
        """Find peaks using reference channel with cross-channel validation."""
        # Step 1: Find reference peaks
        ref_peaks, region_data = self._find_reference_peaks(reference_channel)
        
        # Step 2: Validate peaks across all channels
        valid_peaks = self._validate_cross_channel_peaks(ref_peaks, region_data, tolerance_samples)
        
        if len(valid_peaks) == 0:
            self.pulse_data = np.array([])
            self.peaks = np.array([])
            return np.array([]), np.array([])
        
        # Step 3: Calculate pulse features
        pulse_data = self._calculate_pulse_features(valid_peaks, region_data)
        
        # Store results in class
        self.pulse_data = pulse_data
        self.peaks = valid_peaks
        return pulse_data, valid_peaks
    
    def get_heart_rate(self, channel_idx=2):
        """Get heart rate for specific channel using raw peak detection."""
        # Get raw peaks from the specified channel without cross-channel validation
        region_data = self.channels[self.start_idx:self.end_idx, :]
        ref_signal = region_data[:, channel_idx]
        raw_peaks, _ = find_peaks(ref_signal, 
                                 height=np.percentile(ref_signal, 70),
                                 distance=int(self.sample_rate * 0.4))
        
        # Calculate intervals between consecutive peaks
        if len(raw_peaks) > 1:
            intervals = np.diff(raw_peaks) / self.sample_rate
            return 60.0 / np.mean(intervals)
        else:
            return 0
    
    def get_meta(self, height_scale=3.0, interval_scale=0.8, width_scale=0.8,
                 height_offset=0.0, interval_offset=0.4, width_offset=0.4):
        """
        Extract normalized pulse metadata with fixed parameters.
        
        Returns:
        - meta_data: normalized metadata array [N, 3, 6]
        """
        if self.pulse_data is None:
            raise ValueError("No pulse data found. Run find_peaks() first.")
        
        # pulse_data shape is already (n_peaks, 3, n_channels) where 3 = [height, interval, width]
        meta_data = self.pulse_data.copy()
        
        # Apply fixed normalization parameters (completely stat-independent)
        # This ensures consistent scaling across all subjects
        
        # Normalize heights: (height - height_offset) / height_scale
        meta_data[:, 0, :] = (meta_data[:, 0, :] - height_offset) / height_scale
        
        # Normalize intervals: (interval - interval_offset) / interval_scale  
        meta_data[:, 1, :] = (meta_data[:, 1, :] - interval_offset) / interval_scale
        
        # Normalize widths: (width - width_offset) / width_scale
        meta_data[:, 2, :] = (meta_data[:, 2, :] - width_offset) / width_scale
        
        return meta_data
    
    def get_meta_info(self):
        """Get information about metadata structure."""
        if self.pulse_data is None:
            return "No pulse data available. Run find_peaks() first."
        
        n_peaks, _, n_channels = self.pulse_data.shape
        return f"Metadata shape: [{n_peaks}, 3, {n_channels}] where 3 = [height, interval, width]"
    
    def get_dc(self, processor: SpectraDermaProcessor, dc_scale=1000.0, dc_offset=0.0):
        """
        Extract DC component at peak regions using low-pass filtered original data with fixed normalization.
        
        Parameters:
        - processor: SpectraDermaProcessor instance used for loading the data
        - dc_scale: scaling factor for DC values (default: 1000.0)
        - dc_offset: offset for DC values (default: 0.0)
        
        Returns:
        - dc_matrix: array of shape (n_peaks, n_channels) with normalized DC values at each peak
        """
        if self.peaks is None:
            raise ValueError("No peaks found. Run find_peaks() first.")
        
        # Reset processor to original data and apply low-pass filter at 0.2 Hz
        processor.reset()
        processor.lowpass_filter(cutoff=0.2, order=3)
        
        # Get the DC-filtered data
        _, dc_channels = processor.get_data()
        
        # Extract DC values at peak locations
        n_peaks, n_channels = self.peaks.shape
        dc_matrix = np.zeros((n_peaks, n_channels))
        
        # Adjust peak indices to account for potential region offset
        region_start = self.start_idx
        
        for peak_idx in range(n_peaks):
            for ch in range(n_channels):
                # Get peak position in original data coordinates
                peak_pos = self.peaks[peak_idx, ch] + region_start
                
                # Ensure we're within bounds of the DC data
                if 0 <= peak_pos < len(dc_channels):
                    dc_matrix[peak_idx, ch] = dc_channels[peak_pos, ch]
                else:
                    dc_matrix[peak_idx, ch] = np.nan
        
        # Apply fixed normalization parameters (completely stat-independent)
        # This ensures consistent scaling across all subjects
        dc_matrix = (dc_matrix - dc_offset) / dc_scale
        
        return dc_matrix
    
    def plot_peaks(self, channel_idx=2):
        """Plot signal with detected peaks."""
            
        # Get signal and time
        signal = self.channels[self.start_idx:self.end_idx, channel_idx]
        time = self.timestamp[self.start_idx:self.end_idx]
        ch_peaks = self.peaks[:, channel_idx]
        
        # Calculate heart rate
        heart_rate = self.get_heart_rate(channel_idx)
        
        # Plot
        plt.figure(figsize=(10, 4))
        plt.plot(time, signal, 'b-', linewidth=1.5, label=f'Channel {channel_idx}')
        plt.plot(time[ch_peaks], signal[ch_peaks], 'ro', markersize=8, 
                label=f'Peaks (HR: {heart_rate:.1f} BPM)')
        plt.xlabel('Time')
        plt.ylabel('Amplitude')
        plt.title(f'PPG Analysis - Channel {channel_idx}')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.show()
    
    def segment(self, segment_length=256, channel_idx=2, normalize=True, outlier_sigma=3):
        """
        Segment peak waveforms by finding neighboring troughs and resampling to uniform length.
        
        Parameters:
        - segment_length: target length for resampled segments (default: 256)
        - channel_idx: channel to segment (default: 2 for green)
        - normalize: whether to normalize segments to prevent overfitting (default: True)
        - outlier_sigma: sigma threshold for outlier removal (default: 3)
        
        Returns:
        - segments: array of shape (n_peaks, segment_length) with resampled waveforms (normalized if requested)
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
        
        segments = np.array(segments)
        
        # Apply normalization to prevent overfitting
        if normalize and len(segments) > 0:
            # Per-segment z-score normalization (most common for time series)
            # Normalize each segment individually to have zero mean and unit variance
            for i in range(len(segments)):
                segment = segments[i]
                
                # Z-score normalization: (x - mean) / std
                segment_mean = np.mean(segment)
                segment_std = np.std(segment)
                
                # Avoid division by zero for constant segments
                if segment_std > 1e-8:
                    segments[i] = (segment - segment_mean) / segment_std
                else:
                    # If std is very small, just center around zero
                    segments[i] = segment - segment_mean

        # Final check: Remove outlier segments based on 3-sigma rule
        if normalize and len(segments) > 0:
            # Calculate average segment for this channel
            avg_segment = np.mean(segments, axis=0)
            std_segment = np.std(segments, axis=0)
            
            # Find outlier segments (any timestamp that deviates >3 sigma from average)
            valid_indices = []
            for i, segment in enumerate(segments):
                # Check if any timestamp in this segment violates 3-sigma rule
                deviations = np.abs(segment - avg_segment)
                threshold = outlier_sigma * std_segment
            
                if np.all(deviations <= threshold):
                    valid_indices.append(i)
                
            # Keep only valid segments and corresponding trough pairs
            if len(valid_indices) > 0:
                segments = segments[valid_indices]
                trough_pairs = np.array(trough_pairs)[valid_indices] if trough_pairs else np.array([])
            else:
                segments = np.array([])
                trough_pairs = np.array([])
        
        return segments, np.array(trough_pairs)
    
    def plot_segments(self, segments=None, segment_length=256, normalize=True):
        """
        Plot overlaid segments for all channels with averages highlighted.
        
        Parameters:
        - segments: segments array from segment() method. If None, will call segment() automatically
        - segment_length: length of segments for plotting (default: 256)
        - normalize: whether to normalize segments when calling segment() automatically (default: True)
        """
        # Get segments if not provided
        if segments is None:
            segments, _ = self.segment(segment_length, normalize=normalize)
        
        if len(segments) == 0:
            print("No segments to plot.")
            return
            
        n_channels = self.channels.shape[1]
        
        # Create 6x1 subplots
        fig, axes = plt.subplots(6, 1, figsize=(4, 15))

        # Ensure we have 6 subplots even if fewer channels
        for ch in range(6):
            ax = axes[ch]
            
            if ch < n_channels:
                # Get segments for this channel by re-running segment method for each channel
                ch_segments, _ = self.segment(segment_length, channel_idx=ch, normalize=normalize)
                
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
        
    def compile(self, segment_length=256, save_path="segments.pt", normalize=True, processor=None, debug=True, num_augmentation=0):
        """
        Compile segments from all channels and save with metadata.
        
        Parameters:
        - segment_length: target length for segments (default: 256)
        - save_path: path to save .pt file (default: "segments.pt") 
        - normalize: normalize segments (default: True)
        - processor: SpectraDermaProcessor for DC extraction (optional)
        - debug: show debug output with data peek (default: True)
        - num_augmentation: number of augmented copies per original sample (default: 0)
        
        Returns:
        - compiled_segments: tensor (N, T, C)
        """
        if self.peaks is None:
            raise ValueError("No peaks found. Run find_peaks() first.")
        
        # Step 1: Get segments from all channels
        compiled_segments, min_segments = self._get_all_segments(segment_length, normalize)
        
        # Step 2: Extract metadata with optional debug output
        metadata = self._extract_metadata_with_debug(min_segments, debug)
        
        # Step 3: Extract DC values with optional debug output
        dc_values = self._extract_dc_with_debug(processor, min_segments, debug)
        
        # Step 4: Apply augmentation if requested
        if num_augmentation > 0:
            print(f"Applying data augmentation with {num_augmentation} copies per sample...")
            original_count = compiled_segments.shape[0]
            
            compiled_segments, metadata, dc_values = self._augment_data(
                compiled_segments, metadata, dc_values, num_augmentation
            )
            
            augmented_count = compiled_segments.shape[0]
            print(f"Augmentation complete: {original_count} -> {augmented_count} samples")
        
        # Step 5: Convert to tensors and save
        compiled_tensor = torch.from_numpy(compiled_segments).float()
        
        data_dict = {
            'segments': compiled_tensor,
            'metadata': torch.from_numpy(metadata).float() if metadata is not None else None,
            'dc_values': torch.from_numpy(dc_values).float() if dc_values is not None else None
        }
        
        torch.save(data_dict, save_path)
        
        # Summary output
        print(f"Saved to '{save_path}': {compiled_tensor.shape} segments")
        print(f"Normalization: {'On' if normalize else 'Off'}")
        print(f"Augmentation: {num_augmentation} copies per sample" if num_augmentation > 0 else "Augmentation: Off")
        print(f"Metadata: {metadata.shape if metadata is not None else 'None'}")
        print(f"DC values: {dc_values.shape if dc_values is not None else 'None'}")
        
        return compiled_tensor

    
    def plot_augmented_samples(self, data_path="segments.pt", max_samples=50, show_original_vs_augmented=True):
        """
        Plot exported augmented samples from a saved .pt file.
        
        Parameters:
        - data_path: path to the saved .pt file containing augmented data (default: "segments.pt")
        - max_samples: maximum number of samples to plot per channel (default: 50)
        - show_original_vs_augmented: if True, highlight original vs augmented samples differently (default: True)
        """
        try:
            # Load the saved data
            data_dict = torch.load(data_path)
            segments = data_dict['segments'].numpy()  # Shape: (N, T, C)
            
            n_samples, segment_length, n_channels = segments.shape
            print(f"Loaded {n_samples} samples with {segment_length} time points and {n_channels} channels")
            
            # Limit samples to plot for performance
            plot_samples = min(max_samples, n_samples)
            segments_to_plot = segments[:plot_samples]
            
            # Create 6x1 subplots (matching plot_segments format)
            fig, axes = plt.subplots(6, 1, figsize=(4, 15))
            
            # Ensure we have 6 subplots even if fewer channels
            for ch in range(6):
                ax = axes[ch]
                
                if ch < n_channels:
                    # Get segments for this channel
                    ch_segments = segments_to_plot[:, :, ch]
                    
                    if len(ch_segments) > 0:
                        x = np.arange(segment_length)
                        
                        if show_original_vs_augmented:
                            # Try to identify original vs augmented samples
                            # Assume augmented samples come after originals in batches
                            # This is a heuristic based on the augmentation logic
                            estimated_original_count = self._estimate_original_count(n_samples)
                            
                            # Plot original samples in blue
                            original_samples = min(estimated_original_count, plot_samples)
                            for i in range(original_samples):
                                ax.plot(x, ch_segments[i], 'b-', alpha=0.4, linewidth=0.8)
                            
                            # Plot augmented samples in green
                            for i in range(original_samples, plot_samples):
                                ax.plot(x, ch_segments[i], 'g-', alpha=0.3, linewidth=0.6)
                            
                            # Calculate and plot averages
                            if original_samples > 0:
                                orig_avg = np.mean(ch_segments[:original_samples], axis=0)
                                ax.plot(x, orig_avg, 'r-', linewidth=3, 
                                       label=f'Original Avg (n={original_samples})')
                            
                            if plot_samples > original_samples:
                                aug_avg = np.mean(ch_segments[original_samples:], axis=0)
                                ax.plot(x, aug_avg, 'm-', linewidth=3, 
                                       label=f'Augmented Avg (n={plot_samples-original_samples})')
                        else:
                            # Plot all samples the same way
                            for i, segment in enumerate(ch_segments):
                                ax.plot(x, segment, 'b-', alpha=0.3, linewidth=0.8)
                            
                            # Calculate and plot average
                            avg_segment = np.mean(ch_segments, axis=0)
                            ax.plot(x, avg_segment, 'r-', linewidth=3, 
                                   label=f'Average (n={len(ch_segments)})')
                        
                        ax.set_title(f'Channel {ch} - Exported Segments ({plot_samples}/{n_samples} shown)')
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
            
        except FileNotFoundError:
            print(f"Error: Could not find file '{data_path}'")
        except Exception as e:
            print(f"Error loading or plotting data: {e}")

    # Internal helper methods

    def _get_all_segments(self, segment_length, normalize):
        """Get segments from all channels and return compiled array."""
        n_channels = self.channels.shape[1]
        all_channel_segments = []
        min_segments = float('inf')
        
        for ch in range(n_channels):
            ch_segments, _ = self.segment(segment_length, channel_idx=ch, normalize=normalize)
            all_channel_segments.append(ch_segments)
            min_segments = min(min_segments, len(ch_segments))
        
        if min_segments == 0:
            raise ValueError("No valid segments found across channels.")
        
        # Create tensor with shape (N, T, C)
        compiled_segments = np.zeros((min_segments, segment_length, n_channels))
        for ch in range(n_channels):
            compiled_segments[:, :, ch] = all_channel_segments[ch][:min_segments]
        
        return compiled_segments, min_segments
    
    def _extract_dc_with_debug(self, processor, min_segments, debug=True):
        """Extract DC values with optional debug output."""
        dc_values = None
        if processor is not None:
            try:
                # Get original DC values for debug peek if requested
                original_dc_matrix = None
                if debug:
                    # Extract original DC values before normalization
                    if self.peaks is None:
                        raise ValueError("No peaks found for DC extraction.")
                    
                    processor.reset()
                    processor.lowpass_filter(cutoff=0.2, order=3)
                    _, dc_channels = processor.get_data()
                    
                    n_peaks, n_channels = self.peaks.shape
                    original_dc_matrix = np.zeros((n_peaks, n_channels))
                    region_start = self.start_idx
                    
                    for peak_idx in range(n_peaks):
                        for ch in range(n_channels):
                            peak_pos = self.peaks[peak_idx, ch] + region_start
                            if 0 <= peak_pos < len(dc_channels):
                                original_dc_matrix[peak_idx, ch] = dc_channels[peak_pos, ch]
                            else:
                                original_dc_matrix[peak_idx, ch] = np.nan
                
                # Get normalized DC values
                dc_matrix = self.get_dc(processor)
                dc_values = dc_matrix[:min_segments, :]
                
                if debug:
                    original_dc_trimmed = original_dc_matrix[:min_segments, :]
                    peek_samples = min(5, min_segments)
                    print(f"Original DC values peek (first {peek_samples} samples, all channels):")
                    for i in range(peek_samples):
                        print(f"  Sample {i}: {original_dc_trimmed[i, :]}")
                    
                    print(f"DC values extracted - Shape: {dc_values.shape}")
                    print(f"Normalized DC values peek (first {peek_samples} samples, all channels):")
                    for i in range(peek_samples):
                        print(f"  Sample {i}: {dc_values[i, :]}")
                        
            except Exception as e:
                print(f"Warning: Could not extract DC values: {e}")
                dc_values = None
        
        return dc_values
    
    def _extract_metadata_with_debug(self, min_segments, debug=True):
        """Extract metadata with optional debug output."""
        try:
            if debug:
                # Show original pulse data before normalization
                original_meta = self.pulse_data[:min_segments, :, :]
                peek_samples = min(5, min_segments)
                print(f"Original metadata peek (first {peek_samples} samples, all channels):")
                for i in range(peek_samples):
                    print(f"  Sample {i}: Heights={original_meta[i, 0, :]}")
                    print(f"            Intervals={original_meta[i, 1, :]}")  
                    print(f"            Widths={original_meta[i, 2, :]}")
            
            # Get normalized metadata
            meta_matrix = self.get_meta()
            metadata = meta_matrix[:min_segments, :, :]
            
            if debug:
                print(f"Metadata extracted - Shape: {metadata.shape}")
                print(f"Normalized metadata peek (first {peek_samples} samples, all channels):")
                for i in range(peek_samples):
                    print(f"  Sample {i}: Heights={metadata[i, :, 0]}")
                    print(f"            Intervals={metadata[i, :, 1]}")  
                    print(f"            Widths={metadata[i, :, 2]}")
                    
            return metadata
        except Exception as e:
            print(f"Warning: No metadata available: {e}")
            return None

    def _augment_data(self, segments, metadata=None, dc_values=None, num_augmentation=5):
        """
        Augment segments, metadata, and DC values by adding noise to increase dataset size.
        
        Parameters:
        - segments: numpy array of segments (N, T, C)
        - metadata: metadata array (N, 3, C) or None
        - dc_values: DC values array (N, C) or None
        - num_augmentation: number of augmented copies to create for each original sample
        
        Returns:
        - augmented_segments: augmented segments array
        - augmented_metadata: augmented metadata array or None
        - augmented_dc_values: augmented DC values array or None
        """
        if len(segments) == 0:
            return segments, metadata, dc_values
        
        n_samples, segment_length, n_channels = segments.shape
        total_samples = n_samples * (num_augmentation + 1)  # original + augmented
        
        # Initialize augmented arrays
        augmented_segments = np.zeros((total_samples, segment_length, n_channels))
        augmented_metadata = None
        augmented_dc_values = None
        
        if metadata is not None:
            augmented_metadata = np.zeros((total_samples, metadata.shape[1], metadata.shape[2]))
        
        if dc_values is not None:
            augmented_dc_values = np.zeros((total_samples, dc_values.shape[1]))
        
        # Copy original data first
        augmented_segments[:n_samples] = segments
        if metadata is not None:
            augmented_metadata[:n_samples] = metadata
        if dc_values is not None:
            augmented_dc_values[:n_samples] = dc_values
        
        # Generate augmented samples
        for aug_idx in range(num_augmentation):
            start_idx = n_samples * (aug_idx + 1)
            end_idx = n_samples * (aug_idx + 2)
            
            # Augment segments with 0.2*sigma noise
            for i in range(n_samples):
                original_segment = segments[i]
                segment_std = np.std(original_segment, axis=0)  # std per channel
                noise = np.random.normal(0, 0.2 * segment_std, original_segment.shape)
                augmented_segments[start_idx + i] = original_segment + noise
            
            # Augment metadata with N(0, 0.1) noise
            if metadata is not None:
                for i in range(n_samples):
                    original_meta = metadata[i]
                    noise = np.random.normal(0, 0.1, original_meta.shape)
                    augmented_metadata[start_idx + i] = original_meta + noise
            
            # Augment DC values with 0.1*sigma noise
            if dc_values is not None:
                for i in range(n_samples):
                    original_dc = dc_values[i]
                    dc_std = np.std(original_dc)
                    noise = np.random.normal(0, 0.1 * dc_std, original_dc.shape)
                    augmented_dc_values[start_idx + i] = original_dc + noise
        
        return augmented_segments, augmented_metadata, augmented_dc_values

        
    def _estimate_original_count(self, total_samples):
        """
        Estimate the number of original (non-augmented) samples based on total count.
        This is a heuristic assuming augmentation creates batches of copies.
        """
        # Common augmentation patterns: 1 original + N copies
        # Try to find a reasonable divisor
        for aug_copies in [0, 1, 2, 3, 4, 5, 9, 19]:  # 0, 1x, 2x, 3x, 4x, 5x, 10x, 20x augmentation
            if (total_samples % (aug_copies + 1)) == 0:
                return total_samples // (aug_copies + 1)
        
        # Fallback: assume 1/6 are original (5x augmentation)
        return max(1, total_samples // 6)

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
    
    # Get metadata, DC values, and segments
    meta = analyzer.get_meta()  # Returns [N, 6, 3] matrix
    dc_vals = analyzer.get_dc(processor)  # Returns [N, 6] matrix
    segments, _ = analyzer.segment(segment_length=256, normalize=True)
    print(f"{meta.shape[0]} pulses, {len(segments)} segments")
    print(f"Metadata: {meta.shape}, DC values: {dc_vals.shape}")
    
    # Compile and save with DC values
    compiled_tensor = analyzer.compile(save_path="example_segments.pt", processor=processor)
    
    # Load example
    data = torch.load("example_segments.pt")
    print(f"Loaded: segments {data['segments'].shape}, metadata {data['metadata'].shape}, DC {data['dc_values'].shape}")
