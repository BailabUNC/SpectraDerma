import pandas as pd
import os
import matplotlib.pyplot as plt
# Load the data from the CSV file
data = pd.read_csv('M:\BlueDataSpectraSamples\BTVIZ_2025-08-11.csv')  # Replace 'your_data_file.csv' with the actual file name
# Change directory
os.chdir('M:\BlueDataSpectraSamples')

# Define limits
timeframe1_limits = [250000, 290000]
timeframe2_limits = [330000, 380000]
timeframe3_limits = [383500, 393000]
timeframe4_limits = [300000, 310000]
timeframe5_limits = [300000, 310000]
timeframe6_limits = [300000, 310000]

ppg_before_limits = [1100, 1600]
ppg_after_limits = [400000, 421000]

ch0 = data['data_0']
ch1 = data['data_1']
ch2 = data['data_2']
ch3 = data['data_3']
ch4 = data['data_4']
ch5 = data['data_5']

# Create a time axis using the DataFrame index
time = data.index

def plot_channels(time, ch0, ch1, ch2, ch3, ch4, ch5):
    # Plot all channels with time as the x-axis
    plt.figure(figsize=(10, 6))
    plt.plot(time, ch0, label='Channel 0')
    plt.plot(time, ch1, label='Channel 1')
    plt.plot(time, ch2, label='Channel 2')
    plt.plot(time, ch3, label='Channel 3')
    plt.plot(time, ch4, label='Channel 4')
    plt.plot(time, ch5, label='Channel 5')

    # Add labels, title, and legend
    plt.xlabel('Time (Data Point Index)')
    plt.ylabel('Light Intensity')
    plt.title('Light Intensity vs Time for All Channels')
    plt.legend()
    plt.grid(True)
    plt.show()

# Call the function to plot the original channels
plot_channels(time, ch0, ch1, ch2, ch3, ch4, ch5)

# Assuming ch0, ch1, ch2, ch3, ch4, ch5 are pandas Series
ch0_new = ch0.fillna(method='ffill')
ch1_new = ch1.fillna(method='ffill')
ch2_new = ch2.fillna(method='ffill')
ch3_new = ch3.fillna(method='ffill')
ch4_new = ch4.fillna(method='ffill')
ch5_new = ch5.fillna(method='ffill')
print(ch0_new)
print(ch1_new)
print(ch2_new)
print(ch3_new)
print(ch4_new)
print(ch5_new)

# BEGIN: new_channel_plot_function
def plot_new_channels(time, ch0_new, ch1_new, ch2_new, ch3_new, ch4_new, ch5_new):
    # Plot all new channels with time as the x-axis
    plt.figure(figsize=(10, 6))
    plt.plot(time, ch0_new, label='New Channel 0')
    plt.plot(time, ch1_new, label='New Channel 1')
    plt.plot(time, ch2_new, label='New Channel 2')
    plt.plot(time, ch3_new, label='New Channel 3')
    plt.plot(time, ch4_new, label='New Channel 4')
    plt.plot(time, ch5_new, label='New Channel 5')

    # Add labels, title, and legend
    plt.xlabel('Time (Data Point Index)')
    plt.ylabel('Light Intensity')
    plt.title('Light Intensity vs Time for New Channels')
    plt.legend()
    plt.grid(True)
    plt.show()

# Call the function to plot the new channels
plot_new_channels(time, ch0_new, ch1_new, ch2_new, ch3_new, ch4_new, ch5_new)
# END: new_channel_plot_function

# Assuming `data` is a pandas DataFrame
data_tf1 = pd.DataFrame({
    'ch0_new': slice_pos(ch0_new, timeframe1_limits),
    'ch1_new': slice_pos(ch1_new, timeframe1_limits),
    'ch2_new': slice_pos(ch2_new, timeframe1_limits),
    'ch3_new': slice_pos(ch3_new, timeframe1_limits),
    'ch4_new': slice_pos(ch4_new, timeframe1_limits),
    'ch5_new': slice_pos(ch5_new, timeframe1_limits),

})
data_tf2 = pd.DataFrame({
    'ch0_new': ch0_new[timeframe2_limits[0]:timeframe2_limits[1]],
    'ch1_new': ch1_new[timeframe2_limits[0]:timeframe2_limits[1]],
    'ch2_new': ch2_new[timeframe2_limits[0]:timeframe2_limits[1]],
    'ch3_new': ch3_new[timeframe2_limits[0]:timeframe2_limits[1]],
    'ch4_new': ch4_new[timeframe2_limits[0]:timeframe2_limits[1]],
    'ch5_new': ch5_new[timeframe2_limits[0]:timeframe2_limits[1]],
})
data_tf3 = pd.DataFrame({
    'ch0_new': ch0_new[timeframe3_limits[0]:timeframe3_limits[1]],
    'ch1_new': ch1_new[timeframe3_limits[0]:timeframe3_limits[1]],
    'ch2_new': ch2_new[timeframe3_limits[0]:timeframe3_limits[1]],
    'ch3_new': ch3_new[timeframe3_limits[0]:timeframe3_limits[1]],
    'ch4_new': ch4_new[timeframe3_limits[0]:timeframe3_limits[1]],
    'ch5_new': ch5_new[timeframe3_limits[0]:timeframe3_limits[1]],
})
data_tf4 = pd.DataFrame({
    'ch0_new': ch0_new[timeframe4_limits[0]:timeframe4_limits[1]],
    'ch1_new': ch1_new[timeframe4_limits[0]:timeframe4_limits[1]],
    'ch2_new': ch2_new[timeframe4_limits[0]:timeframe4_limits[1]],
    'ch3_new': ch3_new[timeframe4_limits[0]:timeframe4_limits[1]],
    'ch4_new': ch4_new[timeframe4_limits[0]:timeframe4_limits[1]],
    'ch5_new': ch5_new[timeframe4_limits[0]:timeframe4_limits[1]],
})
data_tf5 = pd.DataFrame({
    'ch0_new': ch0_new[timeframe5_limits[0]:timeframe5_limits[1]],
    'ch1_new': ch1_new[timeframe5_limits[0]:timeframe5_limits[1]],
    'ch2_new': ch2_new[timeframe5_limits[0]:timeframe5_limits[1]],
    'ch3_new': ch3_new[timeframe5_limits[0]:timeframe5_limits[1]],
    'ch4_new': ch4_new[timeframe5_limits[0]:timeframe5_limits[1]],
    'ch5_new': ch5_new[timeframe5_limits[0]:timeframe5_limits[1]],
})
data_tf6 = pd.DataFrame({
    'ch0_new': ch0_new[timeframe6_limits[0]:timeframe6_limits[1]],
    'ch1_new': ch1_new[timeframe6_limits[0]:timeframe6_limits[1]],
    'ch2_new': ch2_new[timeframe6_limits[0]:timeframe6_limits[1]],
    'ch3_new': ch3_new[timeframe6_limits[0]:timeframe6_limits[1]],
    'ch4_new': ch4_new[timeframe6_limits[0]:timeframe6_limits[1]],
    'ch5_new': ch5_new[timeframe6_limits[0]:timeframe6_limits[1]],
})
data_ppg_before = pd.DataFrame({
    'ch0_new': ch0_new[ppg_before_limits[0]:ppg_before_limits[1]],
    'ch1_new': ch1_new[ppg_before_limits[0]:ppg_before_limits[1]],
    'ch2_new': ch2_new[ppg_before_limits[0]:ppg_before_limits[1]],
    'ch3_new': ch3_new[ppg_before_limits[0]:ppg_before_limits[1]],
    'ch4_new': ch4_new[ppg_before_limits[0]:ppg_before_limits[1]],
    'ch5_new': ch5_new[ppg_before_limits[0]:ppg_before_limits[1]],
})
data_ppg_after = pd.DataFrame({
    'ch0_new': ch0_new[ppg_after_limits[0]:ppg_after_limits[1]],
    'ch1_new': ch1_new[ppg_after_limits[0]:ppg_after_limits[1]],
    'ch2_new': ch2_new[ppg_after_limits[0]:ppg_after_limits[1]],
    'ch3_new': ch3_new[ppg_after_limits[0]:ppg_after_limits[1]],
    'ch4_new': ch4_new[ppg_after_limits[0]:ppg_after_limits[1]],
    'ch5_new': ch5_new[ppg_after_limits[0]:ppg_after_limits[1]],
})



    """
    'ch0_new': ch0_new[timeframe1_limits[0]:timeframe1_limits[1]],
    'ch1_new': ch1_new[timeframe1_limits[0]:timeframe1_limits[1]],
    'ch2_new': ch2_new[timeframe1_limits[0]:timeframe1_limits[1]],
    'ch3_new': ch3_new[timeframe1_limits[0]:timeframe1_limits[1]],
    'ch4_new': ch4_new[timeframe1_limits[0]:timeframe1_limits[1]],
    'ch5_new': ch5_new[timeframe1_limits[0]:timeframe1_limits[1]],

    """


print(data_tf1)

print(ch0_new[timeframe1_limits[0]:timeframe1_limits[1]])

plt.figure(figsize=(10, 6))
plt.plot(data_tf1.index, ch0, label='Channel 0')
plt.plot(data_tf1.index, ch1, label='Channel 1')
plt.plot(data_tf1.index, ch2, label='Channel 2')
plt.plot(data_tf1.index, ch3, label='Channel 3')
plt.plot(data_tf1.index, ch4, label='Channel 4')
plt.plot(data_tf1.index, ch5, label='Channel 5')
plt.xlabel('Time (Data Point Index)')
plt.ylabel('Light Intensity')
plt.title('Timeframe 1: Light Intensity vs Time')
plt.legend()
plt.grid(True)
plt.show()

# Display the data within the specified timeframes
print("Timeframe 1 Data:")
print(data_tf1)

"""
all_channels_raw = [ch0, ch1, ch2, ch3, ch4, ch5]
plt.figure()
plt.plot(all_channels_raw)
plt.show()

print("Data Shape:", data.shape)
print("Timeframe 1 data:", data.iloc[timeframe1_limits[0]:timeframe1_limits[1], :])
print("Data indices:", data.index.min(), data.index.max())
print("Data shape:", data.shape)
print("Timeframe1 limits:", timeframe1_limits)
print("Data index range:", data.index.min(), data.index.max())
print(data.head())
"""


# Assuming `data` is a pandas DataFrame
data_tf1 = data.iloc[timeframe1_limits[0]:timeframe1_limits[1], :]
data_tf2 = data.iloc[timeframe2_limits[0]:timeframe2_limits[1], :]
data_tf3 = data.iloc[timeframe3_limits[0]:timeframe3_limits[1], :]
data_tf4 = data.iloc[timeframe4_limits[0]:timeframe4_limits[1], :]
data_tf5 = data.iloc[timeframe5_limits[0]:timeframe5_limits[1], :]
data_tf6 = data.iloc[timeframe6_limits[0]:timeframe6_limits[1], :]
data_ppg_before = data.iloc[ppg_before_limits[0]:ppg_before_limits[1], :]
data_ppg_after = data.iloc[ppg_after_limits[0]:ppg_after_limits[1], :]

# Display the data within the specified timeframes
print("Timeframe 1 Data:")
print(data_tf1)

print("Timeframe 2 Data:")
print(data_tf2)

print("Timeframe 3 Data:")
print(data_tf3)

print("Timeframe 4 Data:")
print(data_tf4)

print("Timeframe 5 Data:")
print(data_tf5)

print("Timeframe 6 Data:")
print(data_tf6)

print("PPG Before Data:")
print(data_ppg_before)

print("PPG After Data:")
print(data_ppg_after)






%filling in any NaN values in the data columns
ch0_new = fillmissing(ch0, 'previous');
ch1_new = fillmissing(ch1, 'previous');
ch2_new = fillmissing(ch2, 'previous');
ch3_new = fillmissing(ch3, 'previous');
ch4_new = fillmissing(ch4, 'previous');
ch5_new = fillmissing(ch5, 'previous');


%active_regionx = [datetime(2025, 4, 7, 14, 23, 0) datetime(2025, 4, 7, 16, 35, 0)];
%active_regiony = [0 200];


purple = [128, 0, 128] / 255;


all_channels = [ch0_new, ch1_new ch2_new ch3_new ch4_new ch5_new];

colors = ['m', 'b', 'g', 'y', 'r', 'k'];


figure;
plot(all_channels)


Raw Blue Sample Light Values
figure('Position', [100, 100, 1200, 800]);
hold on;
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

xlim([0 5993])
title('testing', 'FontSize', 30)
set(gca, 'Fontsize', 30)

Raw Timeframes 1-18 Plots
timeframe1_limits      = [5994 9270]; %[5994 10785];
timeframe2a_limits     = [10786 14310]; %[10786 15712];
timeframe2b_limits     = [15713 19190]; %[15713 19800];
timeframe3_limits      = [19801 23420]; %[19801 23978];
timeframe4a_limits     = [23979 27300]; %[23979 28564];
timeframe4b_limits     = [28565 31800]; %[28565 33675];
timeframe5a_limits     = [33676 37620]; %[33676 38457];
timeframe5b_limits     = [38458 41820]; %[38458 43271];
timeframe6a_limits     = [43272 46510]; %[43272 48356];  
timeframe6b_limits     = [49700 53170]; %[48357 54698]; 
timeframe7a_limits     = [54699 57860]; %[54699 59117];
timeframe7b_limits     = [59118 62460]; %[59118 64920]; %[59118 65325];
timeframe8a_limits     = [65326 69530]; %[65326 70265];
timeframe8b_limits     = [70266 73470]; %[70266 74108];
timeframe9_limits      = [74109 77800]; %[74109 79316];  
timeframe10_limits     = [79317 83320]; %[79317 83890]; 
timeframe11_limits     = [83891 87950]; %[83891 88728];
timeframe1again_limits = [88729 92820]; %[88729 94000];

timeframe1_data       = all_channels(5994:10785, :);
timeframe2a_data      = all_channels(10786:15712, :);
timeframe2b_data      = all_channels(15713:19800, :);
timeframe3_data       = all_channels(19801:23978, :);
timeframe4a_data      = all_channels(23979:28564, :);
timeframe4b_data      = all_channels(28565:33675, :);
timeframe5a_data      = all_channels(33676:38457, :);
timeframe5b_data      = all_channels(38458:43271, :);
timeframe6a_data      = all_channels(43272:48356, :);
timeframe6b_data      = all_channels(48357:54698, :);
timeframe7a_data      = all_channels(54699:59117, :);
timeframe7b_data      = all_channels(59118:65325, :);
timeframe8a_data      = all_channels(65326:70265, :);
timeframe8b_data      = all_channels(70266:74108, :);
timeframe9_data       = all_channels(74109:79316, :);
timeframe10_data      = all_channels(79317:83890, :);
timeframe11_data      = all_channels(83891:88728, :);
timeframe1again_data  = all_channels(88729:93984, :);


size_font = 11; %this is the font size for timeframes 1-6 plots, set to 30 when plotting individually

subplot(6,3,1)
%figure('Position', [100, 100, 1200, 800]);
hold on;
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

xlim(timeframe1_limits)
title('Blue 1', 'FontSize', size_font)
%30Hz sampling frequency but not consistent
set(gca, 'Fontsize', size_font)




subplot(6,3,2)
%figure('Position', [100, 100, 1200, 800]);
hold on;
%subplot(2,2,2)
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

xlim(timeframe2a_limits)
title('Blue 2a', 'FontSize', size_font)
%30Hz sampling frequency but not consistent
set(gca, 'Fontsize', size_font)




subplot(6,3,3)
%figure('Position', [100, 100, 1200, 800]);
hold on;

for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

xlim(timeframe2b_limits)
title('Blue 2b', 'FontSize', size_font)
%30Hz sampling frequency but not consistent
set(gca, 'Fontsize', size_font)




subplot(6,3,4)
%figure('Position', [100, 100, 1200, 800]);
hold on;
%subplot(2,2,2)
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

xlim(timeframe3_limits)
title('Blue 3', 'FontSize', size_font)
%30Hz sampling frequency but not consistent
%xlim([2.542e5 2.545e5])  slightly different pattern in this region
set(gca, 'Fontsize', size_font)




subplot(6,3,5)
%figure('Position', [100, 100, 1200, 800]);
hold on;
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

title('Blue 4a', 'FontSize', size_font)
xlim(timeframe4a_limits) % slightly different pattern in this region
%30Hz sampling frequency but not consistent
set(gca, 'Fontsize', size_font)




subplot(6,3,6)
%figure('Position', [100, 100, 1200, 800]);
hold on;
%subplot(2,2,2)
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

title('Blue 4b', 'FontSize', size_font)
xlim(timeframe4b_limits) % slightly different pattern in this region
%30Hz sampling frequency but not consistent
set(gca, 'Fontsize', size_font)



subplot(6,3,7)
%figure('Position', [100, 100, 1200, 800]);
hold on;
%subplot(2,2,2)
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

title('Blue 5a', 'FontSize', size_font)
xlim(timeframe5a_limits) % slightly different pattern in this region
%30Hz sampling frequency but not consistent
set(gca, 'Fontsize', size_font)



subplot(6,3,8)
%figure('Position', [100, 100, 1200, 800]);
hold on;
%subplot(2,2,2)
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

title('Blue 5b', 'FontSize', size_font)
xlim(timeframe5b_limits) % slightly different pattern in this region
%30Hz sampling frequency but not consistent
set(gca, 'Fontsize', size_font)



subplot(6,3,9)
%figure('Position', [100, 100, 1200, 800]);
hold on;
%subplot(2,2,2)
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

title('Blue 6a', 'FontSize', size_font)
xlim(timeframe6a_limits) % slightly different pattern in this region
%30Hz sampling frequency but not consistent
set(gca, 'Fontsize', size_font)



subplot(6,3,10)
%figure('Position', [100, 100, 1200, 800]);
hold on;
%subplot(2,2,2)
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

title('Blue 6b', 'FontSize', size_font)
xlim(timeframe6b_limits) % slightly different pattern in this region
%30Hz sampling frequency but not consistent
set(gca, 'Fontsize', size_font)



subplot(6,3,11)
%figure('Position', [100, 100, 1200, 800]);
hold on;
%subplot(2,2,2)
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

title('Blue 7a', 'FontSize', size_font)
xlim(timeframe7a_limits) % slightly different pattern in this region
%30Hz sampling frequency but not consistent
set(gca, 'Fontsize', size_font)



subplot(6,3,12)
%figure('Position', [100, 100, 1200, 800]);
hold on;
%subplot(2,2,2)
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

title('Blue 7b', 'FontSize', size_font)
xlim(timeframe7b_limits) % slightly different pattern in this region
%30Hz sampling frequency but not consistent
set(gca, 'Fontsize', size_font)



subplot(6,3,13)
%figure('Position', [100, 100, 1200, 800]);
hold on;
%subplot(2,2,2)
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

title('Blue 8a', 'FontSize', size_font)
xlim(timeframe8a_limits) % slightly different pattern in this region
%30Hz sampling frequency but not consistent
set(gca, 'Fontsize', size_font)



subplot(6,3,14)
%figure('Position', [100, 100, 1200, 800]);
hold on;
%subplot(2,2,2)
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

title('Blue 8b', 'FontSize', size_font)
xlim(timeframe8b_limits) % slightly different pattern in this region
%30Hz sampling frequency but not consistent
set(gca, 'Fontsize', size_font)



subplot(6,3,15)
%figure('Position', [100, 100, 1200, 800]);
hold on;
%subplot(2,2,2)
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

title('Blue 9', 'FontSize', size_font)
xlim(timeframe9_limits) % slightly different pattern in this region
%30Hz sampling frequency but not consistent
set(gca, 'Fontsize', size_font)



subplot(6,3,16)
%figure('Position', [100, 100, 1200, 800]);
hold on;
%subplot(2,2,2)
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

title('Blue 10', 'FontSize', size_font)
xlim(timeframe10_limits) % slightly different pattern in this region
%30Hz sampling frequency but not consistent
set(gca, 'Fontsize', size_font)



subplot(6,3,17)
%figure('Position', [100, 100, 1200, 800]);
hold on;
%subplot(2,2,2)
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

title('Blue 11', 'FontSize', size_font)
xlim(timeframe11_limits) % slightly different pattern in this region
%30Hz sampling frequency but not consistent
set(gca, 'Fontsize', size_font)



subplot(6,3,18)
%figure('Position', [100, 100, 1200, 800]);
hold on;
%subplot(2,2,2)
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

title('Blue 1 again', 'FontSize', size_font)
xlim(timeframe1again_limits) % slightly different pattern in this region
%30Hz sampling frequency but not consistent
set(gca, 'Fontsize', size_font)



Testing

figure('Position', [100, 100, 1200, 800]);
hold on;
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

xlim([59118 62460])
title('testing', 'FontSize', 30)
set(gca, 'Fontsize', 30)

% span = 0.1;                        % fraction of data length (1% window)
% xs = smooth(timeframe1_data, span, 'rlowess');    % or 'loess'
% plot(xs)
% --- Smoothing parameters (tweak these) ---
Fs_est   = 30;          % ~30 Hz (your estimate)
win_sec  = 0.75;        % smooth ~0.75 s of data
W        = max(5, 2*floor(Fs_est*win_sec/2)+1);  % odd window length

smoothed_1 = all_channels;                     % preallocate
for c = 1:size(all_channels,2)
    x = all_channels(:,c);
    x = medfilt1(x, 5);                      % kill spikes/outliers
    smoothed_1(:,c) = sgolayfilt(x, 2, W);     % gentle smoothing (order 2)
end

plot(smoothed_1)
xlim(timeframe1_limits)

smoothed1 = smoothdata(all_channels, 'movmean', W);   % column-wise
plot(smoothed1)
xlim(timeframe1_limits)

figure;
subplot(2,1,1);
boxplot(smoothed_1, "Orientation", "vertical");
title('smoothed_1 Blue 1');

subplot(2,1,2);
boxplot(smoothed1, "Orientation", "vertical");
title('smoothed1 Blue 1');

% --- choose smoother (keep one) ---
Fs_est   = 30;
win_sec  = 0.75;
W        = max(5, 2*floor(Fs_est*win_sec/2)+1);

% Savitzky–Golay with spike removal (robust)
smoothed = all_channels;
for c = 1:size(all_channels,2)
    x = medfilt1(all_channels(:,c), 5);
    smoothed(:,c) = sgolayfilt(x, 2, W);
end
% OR a simple moving average in one line:
% smoothed = smoothdata(all_channels, 1, 'movmean', W);
plot(smoothed)
xlim(timeframe1_limits)

idx1  = timeframe1_limits(1):timeframe1_limits(2);
idx2a = timeframe2a_limits(1):timeframe2a_limits(2);
% ... (etc.)

figure;
subplot(2,1,1)
boxplot(smoothed(idx1, :), "Orientation","vertical");
title('Blue 1 (smoothed)')

subplot(2,1,2)
boxplot(smoothed(idx2a, :), "Orientation","vertical");
title('Blue 2a (smoothed)')
% ... etc.

%% Indices for all timeframes
idx1          = timeframe1_limits(1)     : timeframe1_limits(2);
idx2a         = timeframe2a_limits(1)    : timeframe2a_limits(2);
idx2b         = timeframe2b_limits(1)    : timeframe2b_limits(2);
idx3          = timeframe3_limits(1)     : timeframe3_limits(2);
idx4a         = timeframe4a_limits(1)    : timeframe4a_limits(2);
idx4b         = timeframe4b_limits(1)    : timeframe4b_limits(2);
idx5a         = timeframe5a_limits(1)    : timeframe5a_limits(2);
idx5b         = timeframe5b_limits(1)    : timeframe5b_limits(2);
idx6a         = timeframe6a_limits(1)    : timeframe6a_limits(2);
idx6b         = timeframe6b_limits(1)    : timeframe6b_limits(2);
idx7a         = timeframe7a_limits(1)    : timeframe7a_limits(2);
idx7b         = timeframe7b_limits(1)    : timeframe7b_limits(2);
idx8a         = timeframe8a_limits(1)    : timeframe8a_limits(2);
idx8b         = timeframe8b_limits(1)    : timeframe8b_limits(2);
idx9          = timeframe9_limits(1)     : timeframe9_limits(2);
idx10         = timeframe10_limits(1)    : timeframe10_limits(2);
idx11         = timeframe11_limits(1)    : timeframe11_limits(2);
idx1again     = timeframe1again_limits(1): timeframe1again_limits(2);

%% Slice the smoothed matrix per timeframe (keeps names consistent)
smoothed_1        = smoothed(idx1,       :);
smoothed_2a       = smoothed(idx2a,      :);
smoothed_2b       = smoothed(idx2b,      :);
smoothed_3        = smoothed(idx3,       :);
smoothed_4a       = smoothed(idx4a,      :);
smoothed_4b       = smoothed(idx4b,      :);
smoothed_5a       = smoothed(idx5a,      :);
smoothed_5b       = smoothed(idx5b,      :);
smoothed_6a       = smoothed(idx6a,      :);
smoothed_6b       = smoothed(idx6b,      :);
smoothed_7a       = smoothed(idx7a,      :);
smoothed_7b       = smoothed(idx7b,      :);
smoothed_8a       = smoothed(idx8a,      :);
smoothed_8b       = smoothed(idx8b,      :);
smoothed_9        = smoothed(idx9,       :);
smoothed_10       = smoothed(idx10,      :);
smoothed_11       = smoothed(idx11,      :);
smoothed_1again   = smoothed(idx1again,  :);

%% Figure 1 (1 .. 6a)
figure; 
subplot(3,3,1); boxplot(smoothed_1,  "Orientation","vertical");  title('Blue 1 (smoothed)');
hold on;
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
medians_1 = median(timeframe1_data);
p_1 = polyfit((1:6), medians_1, 4);
x_fit_1 = linspace(1, 6, 100);
y_fit_1 = polyval(p_1, x_fit_1);
plot(x_fit_1, y_fit_1, 'm-')
hold off;

subplot(3,3,2); boxplot(smoothed_2a, "Orientation","vertical");  title('Blue 2a (smoothed)');
hold on;
title('Blue 2a', 'FontSize', 10);
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
medians_2a = median(timeframe2a_data);
p_2a = polyfit((1:6), medians_2a, 4);
x_fit_2a = linspace(1, 6, 100);
y_fit_2a = polyval(p_2a, x_fit_2a);
plot(x_fit_2a, y_fit_2a, 'm-')
hold off;

subplot(3,3,3); boxplot(smoothed_2b, "Orientation","vertical");  title('Blue 2b (smoothed)');
hold on;
set(gca, 'xtick', 1:6, "XTickLabels", x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts (Light Intensity), Unitless')
medians_t2b = median(timeframe2b_data);
[x_fit2b, y_fit2b] = prepareCurveData(1:6, medians_t2b);
ft = fittype('smoothingspline');
opts = fitoptions('Method','SmoothingSpline');
opts.SmoothingParam = 1;
[fitresult, ~] = fit(x_fit2b, y_fit2b, ft, opts);
fittedline_2b = plot(fitresult, x_fit2b, y_fit2b);
lgd = findobj('type','legend'); delete(lgd);
set(fittedline_2b, 'Color', 'magenta')
hold off;
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')

subplot(3,3,4); boxplot(smoothed_3,  "Orientation","vertical");  title('Blue 3 (smoothed)');
hold on;
set(gca, 'xtick', 1:6, "XTickLabels", x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts (Light Intensity), Unitless')
medians_t3 = median(timeframe3_data);
[x_fit3, y_fit3] = prepareCurveData(1:6, medians_t3);
ft = fittype('smoothingspline');
opts = fitoptions('Method','SmoothingSpline');
opts.SmoothingParam = 1;
[fitresult, ~] = fit(x_fit3, y_fit3, ft, opts);
fittedline_3 = plot(fitresult, x_fit3, y_fit3);
lgd = findobj('type','legend'); delete(lgd);
set(fittedline_3, 'Color', 'magenta')
hold off;
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')

subplot(3,3,5); boxplot(smoothed_4a, "Orientation","vertical");  title('Blue 4a (smoothed)');
subplot(3,3,6); boxplot(smoothed_4b, "Orientation","vertical");  title('Blue 4b (smoothed)');
subplot(3,3,7); boxplot(smoothed_5a, "Orientation","vertical");  title('Blue 5a (smoothed)');
subplot(3,3,8); boxplot(smoothed_5b, "Orientation","vertical");  title('Blue 5b (smoothed)');
subplot(3,3,9); boxplot(smoothed_6a, "Orientation","vertical");  title('Blue 6a (smoothed)');

%% Figure 2 (6b .. 1again)
figure;
subplot(3,3,1); boxplot(smoothed_6b,     "Orientation","vertical"); title('Blue 6b (smoothed)');
subplot(3,3,2); boxplot(smoothed_7a,     "Orientation","vertical"); title('Blue 7a (smoothed)');
subplot(3,3,3); boxplot(smoothed_7b,     "Orientation","vertical"); title('Blue 7b (smoothed)');
subplot(3,3,4); boxplot(smoothed_8a,     "Orientation","vertical"); title('Blue 8a (smoothed)');
subplot(3,3,5); boxplot(smoothed_8b,     "Orientation","vertical"); title('Blue 8b (smoothed)');
subplot(3,3,6); boxplot(smoothed_9,      "Orientation","vertical"); title('Blue 9 (smoothed)');
subplot(3,3,7); boxplot(smoothed_10,     "Orientation","vertical"); title('Blue 10 (smoothed)');
subplot(3,3,8); boxplot(smoothed_11,     "Orientation","vertical"); title('Blue 11 (smoothed)');
subplot(3,3,9); boxplot(smoothed_1again, "Orientation","vertical"); title('Blue 1again (smoothed)');


Boxplots for each blue tint sample
figure('Name','All Timeframes - Boxplots','Color','w');

% Plot 1
subplot(3,3,1);
boxplot(timeframe1_data, "Orientation", "vertical");
title('Blue 1 boxplot');

% Plot 2
subplot(3,3,2);
boxplot(timeframe2a_data, "Orientation", "vertical");
title('Blue 2a boxplot');

% Plot 3
subplot(3,3,3);
boxplot(timeframe2b_data, "Orientation", "vertical");
title('Blue 2b boxplot');

% Plot 4
subplot(3,3,4);
boxplot(timeframe3_data, "Orientation", "vertical");
title('Blue 3 boxplot');

% Plot 5
subplot(3,3,5);
boxplot(timeframe4a_data, "Orientation", "vertical");
title('Blue 4a boxplot');

% Plot 6
subplot(3,3,6);
boxplot(timeframe4b_data, "Orientation", "vertical");
title('Blue 4b boxplot');


% Plot 7
subplot(3,3,7);
boxplot(timeframe5a_data, "Orientation", "vertical");
title('Blue 5a boxplot');

% Plot 8
subplot(3,3,8);
boxplot(timeframe5b_data, "Orientation", "vertical");
title('Blue 5b boxplot');

% Plot 9
subplot(3,3,9);
boxplot(timeframe6a_data, "Orientation", "vertical");
title('Blue 6a boxplot');

figure;
% Plot 10
subplot(3,3,1);
boxplot(timeframe6b_data, "Orientation", "vertical");
title('Blue 6b boxplot');

% Plot 11
subplot(3,3,2);
boxplot(timeframe7a_data, "Orientation", "vertical");
title('Blue 7a boxplot');

% Plot 12
subplot(3,3,3);
boxplot(timeframe7b_data, "Orientation", "vertical");
title('Blue 7b boxplot');

% Plot 13
subplot(3,3,4);
boxplot(timeframe8a_data, "Orientation", "vertical");
title('Blue 8a boxplot');

% Plot 14
subplot(3,3,5);
boxplot(timeframe8b_data, "Orientation", "vertical");
title('Blue 8b boxplot');

% Plot 15
subplot(3,3,6);
boxplot(timeframe9_data, "Orientation", "vertical");
title('Blue 9 boxplot');

% Plot 16
subplot(3,3,7);
boxplot(timeframe10_data, "Orientation", "vertical");
title('Blue 10 boxplot');

% Plot 17
subplot(3,3,8);
boxplot(timeframe11_data, "Orientation", "vertical");
title('Blue 11 boxplot');

% Plot 18
subplot(3,3,9);
boxplot(timeframe1again_data, "Orientation", "vertical");
title('Blue 1again boxplot');

%% --- Smoothing parameters (edit as needed) ---
Fs_est  = 30;           % ~30 Hz
win_sec = 0.75;         % smooth ~0.75 s of data
W       = max(5, 2*floor(Fs_est*win_sec/2)+1);   % odd
make_odd = @(n) n - (mod(n,2)==0);               % helper to make odd

%% --- Figure 1: Blue 1 .. Blue 6a ---
figure('Name','All Timeframes - Boxplots','Color','w');

% Plot 1
subplot(3,3,1);
L = make_odd(min(W, size(timeframe1_data,1)));
smoothed_1 = timeframe1_data;
for c = 1:size(smoothed_1,2)
    x = medfilt1(smoothed_1(:,c), 5);
    smoothed_1(:,c) = sgolayfilt(x, 2, L);
end
boxplot(smoothed_1, "Orientation","vertical");
title('Blue 1 boxplot');

% Plot 2
subplot(3,3,2);
L = make_odd(min(W, size(timeframe2a_data,1)));
smoothed_2a = timeframe2a_data;
for c = 1:size(smoothed_2a,2)
    x = medfilt1(smoothed_2a(:,c), 5);
    smoothed_2a(:,c) = sgolayfilt(x, 2, L);
end
boxplot(smoothed_2a, "Orientation","vertical");
title('Blue 2a boxplot');

% Plot 3
subplot(3,3,3);
L = make_odd(min(W, size(timeframe2b_data,1)));
smoothed_2b = timeframe2b_data;
for c = 1:size(smoothed_2b,2)
    x = medfilt1(smoothed_2b(:,c), 5);
    smoothed_2b(:,c) = sgolayfilt(x, 2, L);
end
boxplot(smoothed_2b, "Orientation","vertical");
title('Blue 2b boxplot');

% Plot 4
subplot(3,3,4);
L = make_odd(min(W, size(timeframe3_data,1)));
smoothed_3 = timeframe3_data;
for c = 1:size(smoothed_3,2)
    x = medfilt1(smoothed_3(:,c), 5);
    smoothed_3(:,c) = sgolayfilt(x, 2, L);
end
boxplot(smoothed_3, "Orientation","vertical");
title('Blue 3 boxplot');

% Plot 5
subplot(3,3,5);
L = make_odd(min(W, size(timeframe4a_data,1)));
smoothed_4a = timeframe4a_data;
for c = 1:size(smoothed_4a,2)
    x = medfilt1(smoothed_4a(:,c), 5);
    smoothed_4a(:,c) = sgolayfilt(x, 2, L);
end
boxplot(smoothed_4a, "Orientation","vertical");
title('Blue 4a boxplot');

% Plot 6
subplot(3,3,6);
L = make_odd(min(W, size(timeframe4b_data,1)));
smoothed_4b = timeframe4b_data;
for c = 1:size(smoothed_4b,2)
    x = medfilt1(smoothed_4b(:,c), 5);
    smoothed_4b(:,c) = sgolayfilt(x, 2, L);
end
boxplot(smoothed_4b, "Orientation","vertical");
title('Blue 4b boxplot');

% Plot 7
subplot(3,3,7);
L = make_odd(min(W, size(timeframe5a_data,1)));
smoothed_5a = timeframe5a_data;
for c = 1:size(smoothed_5a,2)
    x = medfilt1(smoothed_5a(:,c), 5);
    smoothed_5a(:,c) = sgolayfilt(x, 2, L);
end
boxplot(smoothed_5a, "Orientation","vertical");
title('Blue 5a boxplot');

% Plot 8
subplot(3,3,8);
L = make_odd(min(W, size(timeframe5b_data,1)));
smoothed_5b = timeframe5b_data;
for c = 1:size(smoothed_5b,2)
    x = medfilt1(smoothed_5b(:,c), 5);
    smoothed_5b(:,c) = sgolayfilt(x, 2, L);
end
boxplot(smoothed_5b, "Orientation","vertical");
title('Blue 5b boxplot');

% Plot 9
subplot(3,3,9);
L = make_odd(min(W, size(timeframe6a_data,1)));
smoothed_6a = timeframe6a_data;
for c = 1:size(smoothed_6a,2)
    x = medfilt1(smoothed_6a(:,c), 5);
    smoothed_6a(:,c) = sgolayfilt(x, 2, L);
end
boxplot(smoothed_6a, "Orientation","vertical");
title('Blue 6a boxplot');

%% --- Figure 2: Blue 6b .. Blue 1again ---
figure('Name','All Timeframes - Boxplots (cont.)','Color','w');

% Plot 10
subplot(3,3,1);
L = make_odd(min(W, size(timeframe6b_data,1)));
smoothed_6b = timeframe6b_data;
for c = 1:size(smoothed_6b,2)
    x = medfilt1(smoothed_6b(:,c), 5);
    smoothed_6b(:,c) = sgolayfilt(x, 2, L);
end
boxplot(smoothed_6b, "Orientation","vertical");
title('Blue 6b boxplot');

% Plot 11
subplot(3,3,2);
L = make_odd(min(W, size(timeframe7a_data,1)));
smoothed_7a = timeframe7a_data;
for c = 1:size(smoothed_7a,2)
    x = medfilt1(smoothed_7a(:,c), 5);
    smoothed_7a(:,c) = sgolayfilt(x, 2, L);
end
boxplot(smoothed_7a, "Orientation","vertical");
title('Blue 7a boxplot');

% Plot 12
subplot(3,3,3);
L = make_odd(min(W, size(timeframe7b_data,1)));
smoothed_7b = timeframe7b_data;
for c = 1:size(smoothed_7b,2)
    x = medfilt1(smoothed_7b(:,c), 5);
    smoothed_7b(:,c) = sgolayfilt(x, 2, L);
end
boxplot(smoothed_7b, "Orientation","vertical");
title('Blue 7b boxplot');

% Plot 13
subplot(3,3,4);
L = make_odd(min(W, size(timeframe8a_data,1)));
smoothed_8a = timeframe8a_data;
for c = 1:size(smoothed_8a,2)
    x = medfilt1(smoothed_8a(:,c), 5);
    smoothed_8a(:,c) = sgolayfilt(x, 2, L);
end
boxplot(smoothed_8a, "Orientation","vertical");
title('Blue 8a boxplot');

% Plot 14
subplot(3,3,5);
L = make_odd(min(W, size(timeframe8b_data,1)));
smoothed_8b = timeframe8b_data;
for c = 1:size(smoothed_8b,2)
    x = medfilt1(smoothed_8b(:,c), 5);
    smoothed_8b(:,c) = sgolayfilt(x, 2, L);
end
boxplot(smoothed_8b, "Orientation","vertical");
title('Blue 8b boxplot');

% Plot 15
subplot(3,3,6);
L = make_odd(min(W, size(timeframe9_data,1)));
smoothed_9 = timeframe9_data;
for c = 1:size(smoothed_9,2)
    x = medfilt1(smoothed_9(:,c), 5);
    smoothed_9(:,c) = sgolayfilt(x, 2, L);
end
boxplot(smoothed_9, "Orientation","vertical");
title('Blue 9 boxplot');

% Plot 16
subplot(3,3,7);
L = make_odd(min(W, size(timeframe10_data,1)));
smoothed_10 = timeframe10_data;
for c = 1:size(smoothed_10,2)
    x = medfilt1(smoothed_10(:,c), 5);
    smoothed_10(:,c) = sgolayfilt(x, 2, L);
end
boxplot(smoothed_10, "Orientation","vertical");
title('Blue 10 boxplot');

% Plot 17
subplot(3,3,8);
L = make_odd(min(W, size(timeframe11_data,1)));
smoothed_11 = timeframe11_data;
for c = 1:size(smoothed_11,2)
    x = medfilt1(smoothed_11(:,c), 5);
    smoothed_11(:,c) = sgolayfilt(x, 2, L);
end
boxplot(smoothed_11, "Orientation","vertical");
title('Blue 11 boxplot');

% Plot 18
subplot(3,3,9);
L = make_odd(min(W, size(timeframe1again_data,1)));
smoothed_1again = timeframe1again_data;
for c = 1:size(smoothed_1again,2)
    x = medfilt1(smoothed_1again(:,c), 5);
    smoothed_1again(:,c) = sgolayfilt(x, 2, L);
end
boxplot(smoothed_1again, "Orientation","vertical");
title('Blue 1again boxplot');



Boxplots with smooth line between medians
x_axis_labels = [415, 480, 555, 590, 680, 780]; %630 445 515



figure;

% 1
subplot(6,3,1);
boxplot(timeframe1_data);
hold on;
title('Blue 1', 'FontSize', 10);
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
medians_1 = median(timeframe1_data);
p_1 = polyfit((1:6), medians_1, 4);
x_fit_1 = linspace(1, 6, 100);
y_fit_1 = polyval(p_1, x_fit_1);
plot(x_fit_1, y_fit_1, 'm-')
hold off;

% 2a
subplot(6,3,2);
boxplot(timeframe2a_data);
hold on;
title('Blue 2a', 'FontSize', 10);
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
medians_2a = median(timeframe2a_data);
p_2a = polyfit((1:6), medians_2a, 4);
x_fit_2a = linspace(1, 6, 100);
y_fit_2a = polyval(p_2a, x_fit_2a);
plot(x_fit_2a, y_fit_2a, 'm-')
hold off;

% 2b
subplot(6,3,3);
boxplot(timeframe2b_data);
hold on;
title('Blue 2b', 'FontSize', 10);
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
medians_2b = median(timeframe2b_data);
p_2b = polyfit((1:6), medians_2b, 4);
x_fit_2b = linspace(1, 6, 100);
y_fit_2b = polyval(p_2b, x_fit_2b);
plot(x_fit_2b, y_fit_2b, 'm-')
hold off;

% 3
subplot(6,3,4);
boxplot(timeframe3_data);
hold on;
title('Blue 3', 'FontSize', 10);
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
medians_3 = median(timeframe3_data);
p_3 = polyfit((1:6), medians_3, 4);
x_fit_3 = linspace(1, 6, 100);
y_fit_3 = polyval(p_3, x_fit_3);
plot(x_fit_3, y_fit_3, 'm-')
hold off;

% 4a
subplot(6,3,5);
boxplot(timeframe4a_data);
hold on;
title('Blue 4a', 'FontSize', 10);
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
medians_4a = median(timeframe4a_data);
p_4a = polyfit((1:6), medians_4a, 4);
x_fit_4a = linspace(1, 6, 100);
y_fit_4a = polyval(p_4a, x_fit_4a);
plot(x_fit_4a, y_fit_4a, 'm-')
hold off;

% 4b
subplot(6,3,6);
boxplot(timeframe4b_data);
hold on;
title('Blue 4b', 'FontSize', 10);
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
medians_4b = median(timeframe4b_data);
p_4b = polyfit((1:6), medians_4b, 4);
x_fit_4b = linspace(1, 6, 100);
y_fit_4b = polyval(p_4b, x_fit_4b);
plot(x_fit_4b, y_fit_4b, 'm-')
hold off;

% 5a
subplot(6,3,7);
boxplot(timeframe5a_data);
hold on;
title('Blue 5a', 'FontSize', 10);
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
medians_5a = median(timeframe5a_data);
p_5a = polyfit((1:6), medians_5a, 4);
x_fit_5a = linspace(1, 6, 100);
y_fit_5a = polyval(p_5a, x_fit_5a);
plot(x_fit_5a, y_fit_5a, 'm-')
hold off;

% 5b
subplot(6,3,8);
boxplot(timeframe5b_data);
hold on;
title('Blue 5b', 'FontSize', 10);
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
medians_5b = median(timeframe5b_data);
p_5b = polyfit((1:6), medians_5b, 4);
x_fit_5b = linspace(1, 6, 100);
y_fit_5b = polyval(p_5b, x_fit_5b);
plot(x_fit_5b, y_fit_5b, 'm-')
hold off;

% 6a
subplot(6,3,9);
boxplot(timeframe6a_data);
hold on;
title('Blue 6a', 'FontSize', 10);
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
medians_6a = median(timeframe6a_data);
p_6a = polyfit((1:6), medians_6a, 4);
x_fit_6a = linspace(1, 6, 100);
y_fit_6a = polyval(p_6a, x_fit_6a);
plot(x_fit_6a, y_fit_6a, 'm-')
hold off;

% 6b
subplot(6,3,10);
boxplot(timeframe6b_data);
hold on;
title('Blue 6b', 'FontSize', 10);
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
medians_6b = median(timeframe6b_data);
p_6b = polyfit((1:6), medians_6b, 4);
x_fit_6b = linspace(1, 6, 100);
y_fit_6b = polyval(p_6b, x_fit_6b);
plot(x_fit_6b, y_fit_6b, 'm-')
hold off;

% 7a
subplot(6,3,11);
boxplot(timeframe7a_data);
hold on;
title('Blue 7a', 'FontSize', 10);
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
medians_7a = median(timeframe7a_data);
p_7a = polyfit((1:6), medians_7a, 4);
x_fit_7a = linspace(1, 6, 100);
y_fit_7a = polyval(p_7a, x_fit_7a);
plot(x_fit_7a, y_fit_7a, 'm-')
hold off;

% 7b
subplot(6,3,12);
boxplot(timeframe7b_data);
hold on;
title('Blue 7b', 'FontSize', 10);
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
medians_7b = median(timeframe7b_data);
p_7b = polyfit((1:6), medians_7b, 4);
x_fit_7b = linspace(1, 6, 100);
y_fit_7b = polyval(p_7b, x_fit_7b);
plot(x_fit_7b, y_fit_7b, 'm-')
hold off;

% 8a
subplot(6,3,13);
boxplot(timeframe8a_data);
hold on;
title('Blue 8a', 'FontSize', 10);
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
medians_8a = median(timeframe8a_data);
p_8a = polyfit((1:6), medians_8a, 4);
x_fit_8a = linspace(1, 6, 100);
y_fit_8a = polyval(p_8a, x_fit_8a);
plot(x_fit_8a, y_fit_8a, 'm-')
hold off;

% 8b
subplot(6,3,14);
boxplot(timeframe8b_data);
hold on;
title('Blue 8b', 'FontSize', 10);
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
medians_8b = median(timeframe8b_data);
p_8b = polyfit((1:6), medians_8b, 4);
x_fit_8b = linspace(1, 6, 100);
y_fit_8b = polyval(p_8b, x_fit_8b);
plot(x_fit_8b, y_fit_8b, 'm-')
hold off;

% 9
subplot(6,3,15);
boxplot(timeframe9_data);
hold on;
title('Blue 9', 'FontSize', 10);
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
medians_9 = median(timeframe9_data);
p_9 = polyfit((1:6), medians_9, 4);
x_fit_9 = linspace(1, 6, 100);
y_fit_9 = polyval(p_9, x_fit_9);
plot(x_fit_9, y_fit_9, 'm-')
hold off;

% 10
subplot(6,3,16);
boxplot(timeframe10_data);
hold on;
title('Blue 10', 'FontSize', 10);
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
medians_10 = median(timeframe10_data);
p_10 = polyfit((1:6), medians_10, 4);
x_fit_10 = linspace(1, 6, 100);
y_fit_10 = polyval(p_10, x_fit_10);
plot(x_fit_10, y_fit_10, 'm-')
hold off;

% 11
subplot(6,3,17);
boxplot(timeframe11_data);
hold on;
title('Blue 11', 'FontSize', 10);
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
medians_11 = median(timeframe11_data);
p_11 = polyfit((1:6), medians_11, 4);
x_fit_11 = linspace(1, 6, 100);
y_fit_11 = polyval(p_11, x_fit_11);
plot(x_fit_11, y_fit_11, 'm-')
hold off;

% 1again
subplot(6,3,18);
%boxplot(timeframe1again_data);
means = mean(timeframe1_data);
sems = std(timeframe1_data) ./ sqrt(size(timeframe1_data,1));
errorbar(1:6, means, sems, 'o-');
hold on;
title('Blue 1again', 'FontSize', 10);
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
medians_1again = median(timeframe1again_data);
p_1again = polyfit((1:6), medians_1again, 4);
x_fit_1again = linspace(1, 6, 100);
y_fit_1again = polyval(p_1again, x_fit_1again);
plot(x_fit_1again, y_fit_1again, 'm-')
hold off;


pos = get(gcf, 'Position');
set(gcf, 'Position',pos+[0 500 0 500])


figure;

boxplot(timeframe1_data);
hold on;
title('Blue 1', 'FontSize', 10); 
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
plot(x_fit_1, y_fit_1, 'm-');
ylim([-inf 140])
hold off;

figure;
boxplot(timeframe2a_data);
hold on;
title('Blue 2a', 'FontSize', 10); 
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
plot(x_fit_2a, y_fit_2a, 'm-');
ylim([-inf 140])

hold off;

figure;

boxplot(timeframe2b_data);
hold on;
title('Blue 2b', 'FontSize', 10); 
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
plot(x_fit_2b, y_fit_2b, 'm-');
hold off;

figure;

boxplot(timeframe3_data);
hold on;
title('Blue 3', 'FontSize', 10); 
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
plot(x_fit_3, y_fit_3, 'm-');
hold off;

figure;

boxplot(timeframe4a_data);
hold on;
title('Blue 4a', 'FontSize', 10); 
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
plot(x_fit_4a, y_fit_4a, 'm-');
hold off;

figure;

boxplot(timeframe4b_data);
hold on;
title('Blue 4b', 'FontSize', 10); 
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
plot(x_fit_4b, y_fit_4b, 'm-');
ylim([-inf 140])

hold off;

figure;

boxplot(timeframe5a_data);
hold on;
title('Blue 5a', 'FontSize', 10); 
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
plot(x_fit_5a, y_fit_5a, 'm-');
hold off;

figure;

boxplot(timeframe5b_data);
hold on;
title('Blue 5b', 'FontSize', 10); 
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
plot(x_fit_5b, y_fit_5b, 'm-');
hold off;

figure;

boxplot(timeframe6a_data);
hold on;
title('Blue 6a', 'FontSize', 10); 
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
plot(x_fit_6a, y_fit_6a, 'm-');
hold off;

figure;

boxplot(timeframe6b_data);
hold on;
title('Blue 6b', 'FontSize', 10); 
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
plot(x_fit_6b, y_fit_6b, 'm-');
hold off;

figure;

boxplot(timeframe7a_data);
hold on;
title('Blue 7a', 'FontSize', 10); 
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
plot(x_fit_7a, y_fit_7a, 'm-');
hold off;

figure;

boxplot(timeframe7b_data);
hold on;
title('Blue 7b', 'FontSize', 10); 
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
plot(x_fit_7b, y_fit_7b, 'm-');
hold off;

figure;

boxplot(timeframe8a_data);
hold on;
title('Blue 8a', 'FontSize', 10); 
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
plot(x_fit_8a, y_fit_8a, 'm-');
hold off;

figure;

boxplot(timeframe8b_data);
hold on;
title('Blue 8b', 'FontSize', 10); 
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
plot(x_fit_8b, y_fit_8b, 'm-');
hold off;

figure;

boxplot(timeframe9_data);
hold on;
title('Blue 9', 'FontSize', 10); 
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
plot(x_fit_9, y_fit_9, 'm-');
hold off;

figure;

boxplot(timeframe10_data);
hold on;
title('Blue 10', 'FontSize', 10); 
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
plot(x_fit_10, y_fit_10, 'm-');
hold off;

figure;

boxplot(timeframe11_data);
hold on;
title('Blue 11', 'FontSize', 10); 
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
plot(x_fit_11, y_fit_11, 'm-');
hold off;

figure;

%boxplot(timeframe1again_data);
means = mean(timeframe1_data);
sems = std(timeframe1_data) ./ sqrt(size(timeframe1_data,1));
errorbar(1:6, means, sems, 'o-');
hold on;
title('Blue 1again', 'FontSize', 10); 
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
plot(x_fit_1again, y_fit_1again, 'm-');
hold off;

%Alternate graphing:
% plot(1:6, medians, 'm-', 'LineWidth', 0.7)
% hold on;
% [x_fitaa, y_fitaa] = prepareCurveData(1:6, medians_aa );
% ft = fittype( 'smoothingspline' );
% opts = fitoptions( 'Method', 'SmoothingSpline' );
% opts.SmoothingParam = 1;
% [fitresult, gof] = fit( x_fitaa, y_fitaa, ft, opts );
% fittedline_aa = plot( fitresult, x_fitaa, y_fitaa);
% lgd = findobj('type', 'legend');
% delete(lgd);
% set(fittedline_aa, 'Color', 'magenta')

Timeframes 1-18 figures with boxplots and smoothed line
% 1
subplot(6,3,1);
boxplot(timeframe1_data);
hold on;
title('Blue 1', 'FontSize', 12);
set(gca, 'xtick', 1:6, "XTickLabels", x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts (Light Intensity), Unitless')
medians_t1 = median(timeframe1_data);

[x_fit1, y_fit1] = prepareCurveData(1:6, medians_t1);
ft = fittype('smoothingspline');
opts = fitoptions('Method','SmoothingSpline');
opts.SmoothingParam = 1;
[fitresult, ~] = fit(x_fit1, y_fit1, ft, opts);
fittedline_1 = plot(fitresult, x_fit1, y_fit1);
lgd = findobj('type','legend'); delete(lgd);
set(fittedline_1, 'Color', 'magenta')
hold off;
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')

% 2a
subplot(6,3,2);
boxplot(timeframe2a_data);
hold on;
title('Blue 2a', 'FontSize', 12);
set(gca, 'xtick', 1:6, "XTickLabels", x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts (Light Intensity), Unitless')
medians_t2a = median(timeframe2a_data);

[x_fit2a, y_fit2a] = prepareCurveData(1:6, medians_t2a);
ft = fittype('smoothingspline');
opts = fitoptions('Method','SmoothingSpline');
opts.SmoothingParam = 1;
[fitresult, ~] = fit(x_fit2a, y_fit2a, ft, opts);
fittedline_2a = plot(fitresult, x_fit2a, y_fit2a);
lgd = findobj('type','legend'); delete(lgd);
set(fittedline_2a, 'Color', 'magenta')
hold off;
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')

% 2b
subplot(6,3,3);
boxplot(timeframe2b_data);
hold on;
title('Blue 2b', 'FontSize', 12);
set(gca, 'xtick', 1:6, "XTickLabels", x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts (Light Intensity), Unitless')
medians_t2b = median(timeframe2b_data);

[x_fit2b, y_fit2b] = prepareCurveData(1:6, medians_t2b);
ft = fittype('smoothingspline');
opts = fitoptions('Method','SmoothingSpline');
opts.SmoothingParam = 1;
[fitresult, ~] = fit(x_fit2b, y_fit2b, ft, opts);
fittedline_2b = plot(fitresult, x_fit2b, y_fit2b);
lgd = findobj('type','legend'); delete(lgd);
set(fittedline_2b, 'Color', 'magenta')
hold off;
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')

% 3
subplot(6,3,4);
boxplot(timeframe3_data);
hold on;
title('Blue 3', 'FontSize', 12);
set(gca, 'xtick', 1:6, "XTickLabels", x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts (Light Intensity), Unitless')
medians_t3 = median(timeframe3_data);

[x_fit3, y_fit3] = prepareCurveData(1:6, medians_t3);
ft = fittype('smoothingspline');
opts = fitoptions('Method','SmoothingSpline');
opts.SmoothingParam = 1;
[fitresult, ~] = fit(x_fit3, y_fit3, ft, opts);
fittedline_3 = plot(fitresult, x_fit3, y_fit3);
lgd = findobj('type','legend'); delete(lgd);
set(fittedline_3, 'Color', 'magenta')
hold off;
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')

% 4a
subplot(6,3,5);
boxplot(timeframe4a_data);
hold on;
title('Blue 4a', 'FontSize', 12);
set(gca, 'xtick', 1:6, "XTickLabels", x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts (Light Intensity), Unitless')
medians_t4a = median(timeframe4a_data);

[x_fit4a, y_fit4a] = prepareCurveData(1:6, medians_t4a);
ft = fittype('smoothingspline');
opts = fitoptions('Method','SmoothingSpline');
opts.SmoothingParam = 1;
[fitresult, ~] = fit(x_fit4a, y_fit4a, ft, opts);
fittedline_4a = plot(fitresult, x_fit4a, y_fit4a);
lgd = findobj('type','legend'); delete(lgd);
set(fittedline_4a, 'Color', 'magenta')
hold off;
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')

% 4b
subplot(6,3,6);
boxplot(timeframe4b_data);
hold on;
title('Blue 4b', 'FontSize', 12);
set(gca, 'xtick', 1:6, "XTickLabels", x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts (Light Intensity), Unitless')
medians_t4b = median(timeframe4b_data);

[x_fit4b, y_fit4b] = prepareCurveData(1:6, medians_t4b);
ft = fittype('smoothingspline');
opts = fitoptions('Method','SmoothingSpline');
opts.SmoothingParam = 1;
[fitresult, ~] = fit(x_fit4b, y_fit4b, ft, opts);
fittedline_4b = plot(fitresult, x_fit4b, y_fit4b);
lgd = findobj('type','legend'); delete(lgd);
set(fittedline_4b, 'Color', 'magenta')
hold off;
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')

% 5a
subplot(6,3,7);
boxplot(timeframe5a_data);
hold on;
title('Blue 5a', 'FontSize', 12);
set(gca, 'xtick', 1:6, "XTickLabels", x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts (Light Intensity), Unitless')
medians_t5a = median(timeframe5a_data);

[x_fit5a, y_fit5a] = prepareCurveData(1:6, medians_t5a);
ft = fittype('smoothingspline');
opts = fitoptions('Method','SmoothingSpline');
opts.SmoothingParam = 1;
[fitresult, ~] = fit(x_fit5a, y_fit5a, ft, opts);
fittedline_5a = plot(fitresult, x_fit5a, y_fit5a);
lgd = findobj('type','legend'); delete(lgd);
set(fittedline_5a, 'Color', 'magenta')
hold off;
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')

% 5b
subplot(6,3,8);
boxplot(timeframe5b_data);
hold on;
title('Blue 5b', 'FontSize', 12);
set(gca, 'xtick', 1:6, "XTickLabels", x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts (Light Intensity), Unitless')
medians_t5b = median(timeframe5b_data);

[x_fit5b, y_fit5b] = prepareCurveData(1:6, medians_t5b);
ft = fittype('smoothingspline');
opts = fitoptions('Method','SmoothingSpline');
opts.SmoothingParam = 1;
[fitresult, ~] = fit(x_fit5b, y_fit5b, ft, opts);
fittedline_5b = plot(fitresult, x_fit5b, y_fit5b);
lgd = findobj('type','legend'); delete(lgd);
set(fittedline_5b, 'Color', 'magenta')
hold off;
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')

% 6a
subplot(6,3,9);
boxplot(timeframe6a_data);
hold on;
title('Blue 6a', 'FontSize', 12);
set(gca, 'xtick', 1:6, "XTickLabels", x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts (Light Intensity), Unitless')
medians_t6a = median(timeframe6a_data);

[x_fit6a, y_fit6a] = prepareCurveData(1:6, medians_t6a);
ft = fittype('smoothingspline');
opts = fitoptions('Method','SmoothingSpline');
opts.SmoothingParam = 1;
[fitresult, ~] = fit(x_fit6a, y_fit6a, ft, opts);
fittedline_6a = plot(fitresult, x_fit6a, y_fit6a);
lgd = findobj('type','legend'); delete(lgd);
set(fittedline_6a, 'Color', 'magenta')
hold off;
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')

% 6b
subplot(6,3,10);
boxplot(timeframe6b_data);
hold on;
title('Blue 6b', 'FontSize', 12);
set(gca, 'xtick', 1:6, "XTickLabels", x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts (Light Intensity), Unitless')
medians_t6b = median(timeframe6b_data);

[x_fit6b, y_fit6b] = prepareCurveData(1:6, medians_t6b);
ft = fittype('smoothingspline');
opts = fitoptions('Method','SmoothingSpline');
opts.SmoothingParam = 1;
[fitresult, ~] = fit(x_fit6b, y_fit6b, ft, opts);
fittedline_6b = plot(fitresult, x_fit6b, y_fit6b);
lgd = findobj('type','legend'); delete(lgd);
set(fittedline_6b, 'Color', 'magenta')
hold off;
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')

% 7a
subplot(6,3,11);
boxplot(timeframe7a_data);
hold on;
title('Blue 7a', 'FontSize', 12);
set(gca, 'xtick', 1:6, "XTickLabels", x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts (Light Intensity), Unitless')
medians_t7a = median(timeframe7a_data);

[x_fit7a, y_fit7a] = prepareCurveData(1:6, medians_t7a);
ft = fittype('smoothingspline');
opts = fitoptions('Method','SmoothingSpline');
opts.SmoothingParam = 1;
[fitresult, ~] = fit(x_fit7a, y_fit7a, ft, opts);
fittedline_7a = plot(fitresult, x_fit7a, y_fit7a);
lgd = findobj('type','legend'); delete(lgd);
set(fittedline_7a, 'Color', 'magenta')
hold off;
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')

% 7b
subplot(6,3,12);
boxplot(timeframe7b_data);
hold on;
title('Blue 7b', 'FontSize', 12);
set(gca, 'xtick', 1:6, "XTickLabels", x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts (Light Intensity), Unitless')
medians_t7b = median(timeframe7b_data);

[x_fit7b, y_fit7b] = prepareCurveData(1:6, medians_t7b);
ft = fittype('smoothingspline');
opts = fitoptions('Method','SmoothingSpline');
opts.SmoothingParam = 1;
[fitresult, ~] = fit(x_fit7b, y_fit7b, ft, opts);
fittedline_7b = plot(fitresult, x_fit7b, y_fit7b);
lgd = findobj('type','legend'); delete(lgd);
set(fittedline_7b, 'Color', 'magenta')
hold off;
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')

% 8a
subplot(6,3,13);
boxplot(timeframe8a_data);
hold on;
title('Blue 8a', 'FontSize', 12);
set(gca, 'xtick', 1:6, "XTickLabels", x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts (Light Intensity), Unitless')
medians_t8a = median(timeframe8a_data);

[x_fit8a, y_fit8a] = prepareCurveData(1:6, medians_t8a);
ft = fittype('smoothingspline');
opts = fitoptions('Method','SmoothingSpline');
opts.SmoothingParam = 1;
[fitresult, ~] = fit(x_fit8a, y_fit8a, ft, opts);
fittedline_8a = plot(fitresult, x_fit8a, y_fit8a);
lgd = findobj('type','legend'); delete(lgd);
set(fittedline_8a, 'Color', 'magenta')
hold off;
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')

% 8b
subplot(6,3,14);
boxplot(timeframe8b_data);
hold on;
title('Blue 8b', 'FontSize', 12);
set(gca, 'xtick', 1:6, "XTickLabels", x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts (Light Intensity), Unitless')
medians_t8b = median(timeframe8b_data);

[x_fit8b, y_fit8b] = prepareCurveData(1:6, medians_t8b);
ft = fittype('smoothingspline');
opts = fitoptions('Method','SmoothingSpline');
opts.SmoothingParam = 1;
[fitresult, ~] = fit(x_fit8b, y_fit8b, ft, opts);
fittedline_8b = plot(fitresult, x_fit8b, y_fit8b);
lgd = findobj('type','legend'); delete(lgd);
set(fittedline_8b, 'Color', 'magenta')
hold off;
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')

% 9
subplot(6,3,15);
boxplot(timeframe9_data);
hold on;
title('Blue 9', 'FontSize', 12);
set(gca, 'xtick', 1:6, "XTickLabels", x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts (Light Intensity), Unitless')
medians_t9 = median(timeframe9_data);

[x_fit9, y_fit9] = prepareCurveData(1:6, medians_t9);
ft = fittype('smoothingspline');
opts = fitoptions('Method','SmoothingSpline');
opts.SmoothingParam = 1;
[fitresult, ~] = fit(x_fit9, y_fit9, ft, opts);
fittedline_9 = plot(fitresult, x_fit9, y_fit9);
lgd = findobj('type','legend'); delete(lgd);
set(fittedline_9, 'Color', 'magenta')
hold off;
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')

% 10
subplot(6,3,16);
boxplot(timeframe10_data);
hold on;
title('Blue 10', 'FontSize', 12);
set(gca, 'xtick', 1:6, "XTickLabels", x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts (Light Intensity), Unitless')
medians_t10 = median(timeframe10_data);

[x_fit10, y_fit10] = prepareCurveData(1:6, medians_t10);
ft = fittype('smoothingspline');
opts = fitoptions('Method','SmoothingSpline');
opts.SmoothingParam = 1;
[fitresult, ~] = fit(x_fit10, y_fit10, ft, opts);
fittedline_10 = plot(fitresult, x_fit10, y_fit10);
lgd = findobj('type','legend'); delete(lgd);
set(fittedline_10, 'Color', 'magenta')
hold off;
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')

% 11
subplot(6,3,17);
boxplot(timeframe11_data);
hold on;
title('Blue 11', 'FontSize', 12);
set(gca, 'xtick', 1:6, "XTickLabels", x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts (Light Intensity), Unitless')
medians_t11 = median(timeframe11_data);

[x_fit11, y_fit11] = prepareCurveData(1:6, medians_t11);
ft = fittype('smoothingspline');
opts = fitoptions('Method','SmoothingSpline');
opts.SmoothingParam = 1;
[fitresult, ~] = fit(x_fit11, y_fit11, ft, opts);
fittedline_11 = plot(fitresult, x_fit11, y_fit11);
lgd = findobj('type','legend'); delete(lgd);
set(fittedline_11, 'Color', 'magenta')
hold off;
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')

% 1again
subplot(6,3,18);
boxplot(timeframe1again_data);
hold on;
title('Blue 1again', 'FontSize', 12);
set(gca, 'xtick', 1:6, "XTickLabels", x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts (Light Intensity), Unitless')
medians_t1again = median(timeframe1again_data);

[x_fit1again, y_fit1again] = prepareCurveData(1:6, medians_t1again);
ft = fittype('smoothingspline');
opts = fitoptions('Method','SmoothingSpline');
opts.SmoothingParam = 1;
[fitresult, ~] = fit(x_fit1again, y_fit1again, ft, opts);
fittedline_1again = plot(fitresult, x_fit1again, y_fit1again);
lgd = findobj('type','legend'); delete(lgd);
set(fittedline_1again, 'Color', 'magenta')
hold off;
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')


pos = get(gcf, 'Position');
set(gcf, 'Position',pos+[0 500 0 500])


Normalized Data (Min-Max) as a smoothed line, no box plots, Timeframes Samples 1-11
% 1
subplot(6,3,1);
%first, min max normalization
norm_t1 = normalize(medians_t1, 'range');

%then smoothing that line
[x_fit_t1, y_fit_t1] = prepareCurveData(1:6, norm_t1 );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t1, y_fit_t1, ft, opts );
fittedline_t1 = plot( fitresult, x_fit_t1, y_fit_t1);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t1, 'Color', 'magenta')
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')
title('Blue 1 Norm.')

% 2a
subplot(6,3,2);
norm_t2a = normalize(medians_t2a, 'range');
[x_fit_t2a, y_fit_t2a] = prepareCurveData(1:6, norm_t2a );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t2a, y_fit_t2a, ft, opts );
fittedline_t2a = plot( fitresult, x_fit_t2a, y_fit_t2a);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t2a, 'Color', 'magenta')
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')
title('Blue 2a Norm.')

% 2b
subplot(6,3,3);
norm_t2b = normalize(medians_t2b, 'range');
[x_fit_t2b, y_fit_t2b] = prepareCurveData(1:6, norm_t2b );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t2b, y_fit_t2b, ft, opts );
fittedline_t2b = plot( fitresult, x_fit_t2b, y_fit_t2b);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t2b, 'Color', 'magenta')
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')
title('Blue 2b Norm.')

% 3
subplot(6,3,4);
norm_t3 = normalize(medians_t3, 'range');
[x_fit_t3, y_fit_t3] = prepareCurveData(1:6, norm_t3 );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t3, y_fit_t3, ft, opts );
fittedline_t3 = plot( fitresult, x_fit_t3, y_fit_t3);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t3, 'Color', 'magenta')
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')
title('Blue 3 Norm.')

% 4a
subplot(6,3,5);
norm_t4a = normalize(medians_t4a, 'range');
[x_fit_t4a, y_fit_t4a] = prepareCurveData(1:6, norm_t4a );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t4a, y_fit_t4a, ft, opts );
fittedline_t4a = plot( fitresult, x_fit_t4a, y_fit_t4a);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t4a, 'Color', 'magenta')
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')
title('Blue 4a Norm.')

% 4b
subplot(6,3,6);
norm_t4b = normalize(medians_t4b, 'range');
[x_fit_t4b, y_fit_t4b] = prepareCurveData(1:6, norm_t4b );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t4b, y_fit_t4b, ft, opts );
fittedline_t4b = plot( fitresult, x_fit_t4b, y_fit_t4b);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t4b, 'Color', 'magenta')
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')
title('Blue 4b Norm.')

% 5a
subplot(6,3,7);
norm_t5a = normalize(medians_t5a, 'range');
[x_fit_t5a, y_fit_t5a] = prepareCurveData(1:6, norm_t5a );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t5a, y_fit_t5a, ft, opts );
fittedline_t5a = plot( fitresult, x_fit_t5a, y_fit_t5a);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t5a, 'Color', 'magenta')
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')
title('Blue 5a Norm.')

% 5b
subplot(6,3,8);
norm_t5b = normalize(medians_t5b, 'range');
[x_fit_t5b, y_fit_t5b] = prepareCurveData(1:6, norm_t5b );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t5b, y_fit_t5b, ft, opts );
fittedline_t5b = plot( fitresult, x_fit_t5b, y_fit_t5b);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t5b, 'Color', 'magenta')
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')
title('Blue 5b Norm.')

% 6a
subplot(6,3,9);
norm_t6a = normalize(medians_t6a, 'range');
[x_fit_t6a, y_fit_t6a] = prepareCurveData(1:6, norm_t6a );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t6a, y_fit_t6a, ft, opts );
fittedline_t6a = plot( fitresult, x_fit_t6a, y_fit_t6a);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t6a, 'Color', 'magenta')
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')
title('Blue 6a Norm.')

% 6b
subplot(6,3,10);
norm_t6b = normalize(medians_t6b, 'range');
[x_fit_t6b, y_fit_t6b] = prepareCurveData(1:6, norm_t6b );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t6b, y_fit_t6b, ft, opts );
fittedline_t6b = plot( fitresult, x_fit_t6b, y_fit_t6b);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t6b, 'Color', 'magenta')
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')
title('Blue 6b Norm.')

% 7a
subplot(6,3,11);
norm_t7a = normalize(medians_t7a, 'range');
[x_fit_t7a, y_fit_t7a] = prepareCurveData(1:6, norm_t7a );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t7a, y_fit_t7a, ft, opts );
fittedline_t7a = plot( fitresult, x_fit_t7a, y_fit_t7a);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t7a, 'Color', 'magenta')
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')
title('Blue 7a Norm.')

% 7b
subplot(6,3,12);
norm_t7b = normalize(medians_t7b, 'range');
[x_fit_t7b, y_fit_t7b] = prepareCurveData(1:6, norm_t7b );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t7b, y_fit_t7b, ft, opts );
fittedline_t7b = plot( fitresult, x_fit_t7b, y_fit_t7b);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t7b, 'Color', 'magenta')
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')
title('Blue 7b Norm.')

% 8a
subplot(6,3,13);
norm_t8a = normalize(medians_t8a, 'range');
[x_fit_t8a, y_fit_t8a] = prepareCurveData(1:6, norm_t8a );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t8a, y_fit_t8a, ft, opts );
fittedline_t8a = plot( fitresult, x_fit_t8a, y_fit_t8a);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t8a, 'Color', 'magenta')
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')
title('Blue 8a Norm.')

% 8b
subplot(6,3,14);
norm_t8b = normalize(medians_t8b, 'range');
[x_fit_t8b, y_fit_t8b] = prepareCurveData(1:6, norm_t8b );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t8b, y_fit_t8b, ft, opts );
fittedline_t8b = plot( fitresult, x_fit_t8b, y_fit_t8b);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t8b, 'Color', 'magenta')
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')
title('Blue 8b Norm.')

% 9
subplot(6,3,15);
norm_t9 = normalize(medians_t9, 'range');
[x_fit_t9, y_fit_t9] = prepareCurveData(1:6, norm_t9 );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t9, y_fit_t9, ft, opts );
fittedline_t9 = plot( fitresult, x_fit_t9, y_fit_t9);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t9, 'Color', 'magenta')
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')
title('Blue 9 Norm.')

% 10
subplot(6,3,16);
norm_t10 = normalize(medians_t10, 'range');
[x_fit_t10, y_fit_t10] = prepareCurveData(1:6, norm_t10 );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t10, y_fit_t10, ft, opts );
fittedline_t10 = plot( fitresult, x_fit_t10, y_fit_t10);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t10, 'Color', 'magenta')
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')
title('Blue 10 Norm.')

% 11
subplot(6,3,17);
norm_t11 = normalize(medians_t11, 'range');
[x_fit_t11, y_fit_t11] = prepareCurveData(1:6, norm_t11 );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t11, y_fit_t11, ft, opts );
fittedline_t11 = plot( fitresult, x_fit_t11, y_fit_t11);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t11, 'Color', 'magenta')
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')
title('Blue 11 Norm.')

% 1again
subplot(6,3,18);
norm_t1again = normalize(medians_t1again, 'range');
[x_fit_t1again, y_fit_t1again] = prepareCurveData(1:6, norm_t1again );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t1again, y_fit_t1again, ft, opts );
fittedline_t1again = plot( fitresult, x_fit_t1again, y_fit_t1again);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t1again, 'Color', 'magenta')
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts')
title('Blue 1again Norm.')


pos = get(gcf, 'Position');
set(gcf, 'Position',pos+[0 500 0 500])

Timeframes 1-6 Min-Max Normalized Lines of Best Fit, Superimposed on Same Plot
[x_fit_g1, y_fit_g1] = prepareCurveData(1:6, norm_t1 );
[x_fit_g2a, y_fit_g2a] = prepareCurveData(1:6, norm_t2a );
[x_fit_g2b, y_fit_g2b] = prepareCurveData(1:6, norm_t2b );
[x_fit_g3, y_fit_g3] = prepareCurveData(1:6, norm_t3 );

[x_fit_g4a, y_fit_g4a] = prepareCurveData(1:6, norm_t4a );
[x_fit_g4b, y_fit_g4b] = prepareCurveData(1:6, norm_t4b );
[x_fit_g5a, y_fit_g5a] = prepareCurveData(1:6, norm_t5a );
[x_fit_g5b, y_fit_g5b] = prepareCurveData(1:6, norm_t5b );

[x_fit_g6a, y_fit_g6a] = prepareCurveData(1:6, norm_t6a );
[x_fit_g6b, y_fit_g6b] = prepareCurveData(1:6, norm_t6b );
[x_fit_g7a, y_fit_g7a] = prepareCurveData(1:6, norm_t7a );
[x_fit_g7b, y_fit_g7b] = prepareCurveData(1:6, norm_t7b );

[x_fit_g8a, y_fit_g8a] = prepareCurveData(1:6, norm_t8a );
[x_fit_g8b, y_fit_g8b] = prepareCurveData(1:6, norm_t8b );
[x_fit_g9, y_fit_g9] = prepareCurveData(1:6, norm_t9 );
[x_fit_g10, y_fit_g10] = prepareCurveData(1:6, norm_t10 );

[x_fit_g11, y_fit_g11] = prepareCurveData(1:6, norm_t11 );
[x_fit_g1again, y_fit_g1again] = prepareCurveData(1:6, norm_t1again );



figure;

% t1
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t1, y_fit_t1, ft, opts );
fittedline_t1 = plot( fitresult, x_fit_t1, y_fit_t1);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t1, 'Color', 'red')
hold on;

% t2a
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t2a, y_fit_t2a, ft, opts );
fittedline_t2a = plot( fitresult, x_fit_t2a, y_fit_t2a);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t2a, 'Color', [1 0.5333 0]) % #f80
hold on;

% t2b
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t2b, y_fit_t2b, ft, opts );
fittedline_t2b = plot( fitresult, x_fit_t2b, y_fit_t2b);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t2b, 'Color', 'yellow')
hold on;

% t3
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t3, y_fit_t3, ft, opts );
fittedline_t3 = plot( fitresult, x_fit_t3, y_fit_t3);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t3, 'Color', 'green')
hold on;

% t4a
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t4a, y_fit_t4a, ft, opts );
fittedline_t4a = plot( fitresult, x_fit_t4a, y_fit_t4a);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t4a, 'Color', 'blue')
hold on;

% t4b
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t4b, y_fit_t4b, ft, opts );
fittedline_t4b = plot( fitresult, x_fit_t4b, y_fit_t4b);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t4b, 'Color', [0.2941 0 0.5098]) % #4b0082
hold on;

% t5a
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t5a, y_fit_t5a, ft, opts );
fittedline_t5a = plot( fitresult, x_fit_t5a, y_fit_t5a);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t5a, 'Color', 'magenta')
hold on;

% t5b
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t5b, y_fit_t5b, ft, opts );
fittedline_t5b = plot( fitresult, x_fit_t5b, y_fit_t5b);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t5b, 'Color', 'cyan')
hold on;

% t6a
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t6a, y_fit_t6a, ft, opts );
fittedline_t6a = plot( fitresult, x_fit_t6a, y_fit_t6a);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t6a, 'Color', [0 0 0])
hold on;

% t6b
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t6b, y_fit_t6b, ft, opts );
fittedline_t6b = plot( fitresult, x_fit_t6b, y_fit_t6b);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t6b, 'Color', [0.8500 0.3250 0.0980])
hold on;

% t7a
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t7a, y_fit_t7a, ft, opts );
fittedline_t7a = plot( fitresult, x_fit_t7a, y_fit_t7a);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t7a, 'Color', [0.4940 0.1840 0.5560])
hold on;

% t7b
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t7b, y_fit_t7b, ft, opts );
fittedline_t7b = plot( fitresult, x_fit_t7b, y_fit_t7b);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t7b, 'Color', [0.4660 0.6740 0.1880])
hold on;

% t8a
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t8a, y_fit_t8a, ft, opts );
fittedline_t8a = plot( fitresult, x_fit_t8a, y_fit_t8a);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t8a, 'Color', [0.3010 0.7450 0.9330])
hold on;

% t8b
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t8b, y_fit_t8b, ft, opts );
fittedline_t8b = plot( fitresult, x_fit_t8b, y_fit_t8b);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t8b, 'Color', [0.6350 0.0780 0.1840])
hold on;

% t9
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t9, y_fit_t9, ft, opts );
fittedline_t9 = plot( fitresult, x_fit_t9, y_fit_t9);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t9, 'Color', [0.25 0.25 0.25])
hold on;

% t10
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t10, y_fit_t10, ft, opts );
fittedline_t10 = plot( fitresult, x_fit_t10, y_fit_t10);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t10, 'Color', [0 0.4470 0.7410])
hold on;

% t11
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t11, y_fit_t11, ft, opts );
fittedline_t11 = plot( fitresult, x_fit_t11, y_fit_t11);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t11, 'Color', [0.5 0.5 0])
hold on;

% t1again
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t1again, y_fit_t1again, ft, opts );
fittedline_t1again = plot( fitresult, x_fit_t1again, y_fit_t1again);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t1again, 'Color', [0 0.5 0.5])

set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)

hold off;
xlabel('Wavelength in nm');
ylabel('Min-Max Normalized Medians')
title('PD Readings of Blue Samples, Min-Max Norm. Lines of Best Fit')

figure;

% g1
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_g1, y_fit_g1, ft, opts );
fittedline_g1 = plot( fitresult, x_fit_g1, y_fit_g1);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_g1, 'Color', 'red')
hold on;

% g1again

[fitresult, ~] = fit( x_fit_g1again, y_fit_g1again, ft, opts );
fittedline_g1again = plot( fitresult, x_fit_g1again, y_fit_g1again);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_g1again, 'Color', [0 0.5 0.5])
xlabel('Wavelength in nm');
ylabel('Min-Max Normalized Medians')
title('Raw PD Readings of Blue 1a, 1b')
hold off;

set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
figure;
%g2a

[fitresult, ~] = fit( x_fit_g2a, y_fit_g2a, ft, opts );
fittedline_g2a = plot( fitresult, x_fit_g2a, y_fit_g2a);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_g2a, 'Color', [1 0.5333 0]) % #f80
hold on;

% t2b

[fitresult, ~] = fit( x_fit_g2b, y_fit_g2b, ft, opts );
fittedline_g2b = plot( fitresult, x_fit_g2b, y_fit_g2b);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_g2b, 'Color', 'yellow')
xlabel('Wavelength in nm');
ylabel('Min-Max Normalized Medians')
title('Raw PD Readings of Blue 2a, 2b')
hold off;

figure;
% g3

[fitresult, ~] = fit( x_fit_t3, y_fit_t3, ft, opts );
fittedline_t3 = plot( fitresult, x_fit_t3, y_fit_t3);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t3, 'Color', 'green')
xlabel('Wavelength in nm');
ylabel('Min-Max Normalized Medians')
title('Raw PD Readings of Blue 3')
hold off;
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
figure;
% g4a

[fitresult, ~] = fit( x_fit_g4a, y_fit_g4a, ft, opts );
fittedline_g4a = plot( fitresult, x_fit_g4a, y_fit_g4a);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_g4a, 'Color', 'blue')
hold on;

% g4b

[fitresult, ~] = fit( x_fit_g4b, y_fit_g4b, ft, opts );
fittedline_g4b = plot( fitresult, x_fit_g4b, y_fit_g4b);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_g4b, 'Color', [0.2941 0 0.5098]) % #4b0082
xlabel('Wavelength in nm');
ylabel('Min-Max Normalized Medians')
title('Raw PD Readings of Blue 4a, 4b')
hold off;
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
figure;
% g5a

[fitresult, ~] = fit( x_fit_g5a, y_fit_g5a, ft, opts );
fittedline_g5a = plot( fitresult, x_fit_g5a, y_fit_g5a);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_g5a, 'Color', 'magenta')
hold on;

% g5b

[fitresult, ~] = fit( x_fit_g5b, y_fit_g5b, ft, opts );
fittedline_g5b = plot( fitresult, x_fit_g5b, y_fit_g5b);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_g5b, 'Color', 'cyan')
xlabel('Wavelength in nm');
ylabel('Min-Max Normalized Medians')
title('Raw PD Readings of Blue 5a, 5b')
hold off;
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
figure;
% g6a

[fitresult, ~] = fit( x_fit_g6a, y_fit_g6a, ft, opts );
fittedline_g6a = plot( fitresult, x_fit_g6a, y_fit_g6a);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_g6a, 'Color', [0 0 0])
hold on;

% g6b

[fitresult, ~] = fit( x_fit_g6b, y_fit_g6b, ft, opts );
fittedline_g6b = plot( fitresult, x_fit_g6b, y_fit_g6b);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_t6b, 'Color', [0.8500 0.3250 0.0980])
xlabel('Wavelength in nm');
ylabel('Min-Max Normalized Medians')
title('Raw PD Readings of Blue 6a, 6b')
hold off;
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
figure;
% g7a

[fitresult, ~] = fit( x_fit_g7a, y_fit_g7a, ft, opts );
fittedline_g7a = plot( fitresult, x_fit_g7a, y_fit_g7a);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_g7a, 'Color', [0.4940 0.1840 0.5560])
hold on;

% g7b

[fitresult, ~] = fit( x_fit_g7b, y_fit_g7b, ft, opts );
fittedline_g7b = plot( fitresult, x_fit_g7b, y_fit_g7b);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_g7b, 'Color', [0.4660 0.6740 0.1880])
xlabel('Wavelength in nm');
ylabel('Min-Max Normalized Medians')
title('Raw PD Readings of Blue 7a, 7b')
hold off;
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
figure;
% g8a

[fitresult, ~] = fit( x_fit_g8a, y_fit_g8a, ft, opts );
fittedline_g8a = plot( fitresult, x_fit_g8a, y_fit_g8a);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_g8a, 'Color', [0.3010 0.7450 0.9330])
hold on;

% g8b

[fitresult, ~] = fit( x_fit_g8b, y_fit_g8b, ft, opts );
fittedline_g8b = plot( fitresult, x_fit_g8b, y_fit_g8b);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_g8b, 'Color', [0.6350 0.0780 0.1840])
xlabel('Wavelength in nm');
ylabel('Min-Max Normalized Medians')
title('Raw PD Readings of Blue 8a, 8b')
hold off;
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
figure;
% g9

[fitresult, ~] = fit( x_fit_g9, y_fit_g9, ft, opts );
fittedline_g9 = plot( fitresult, x_fit_g9, y_fit_g9);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_g9, 'Color', [0.25 0.25 0.25])
xlabel('Wavelength in nm');
ylabel('Min-Max Normalized Medians')
title('Raw PD Readings of Blue 9')
hold off;
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
figure;
% g10

[fitresult, ~] = fit( x_fit_g10, y_fit_g10, ft, opts );
fittedline_g10 = plot( fitresult, x_fit_g10, y_fit_g10);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_g10, 'Color', [0 0.4470 0.7410])
xlabel('Wavelength in nm');
ylabel('Min-Max Normalized Medians')
title('Raw PD Readings of Blue 10')
hold off;
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
figure;
% g11

[fitresult, ~] = fit( x_fit_g11, y_fit_g11, ft, opts );
fittedline_g11 = plot( fitresult, x_fit_g11, y_fit_g11);
lgd = findobj('type', 'legend'); delete(lgd);
set(fittedline_g11, 'Color', [0.5 0.5 0])
xlabel('Wavelength in nm');
ylabel('Min-Max Normalized Medians')
title('Raw PD Readings of Blue 11')
hold off;

set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)


figure;

ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t1, y_fit_t1, ft, opts );
fittedline_t1 = plot( fitresult, x_fit_t1, y_fit_t1);
lgd = findobj('type', 'legend');
delete(lgd);
set(fittedline_t1, 'Color', 'red')

hold on;

ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t2, y_fit_t2, ft, opts );
fittedline_t2 = plot( fitresult, x_fit_t2, y_fit_t2);
lgd = findobj('type', 'legend');
delete(lgd);
set(fittedline_t2, 'Color', '#f80')

hold on;

ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t3, y_fit_t3, ft, opts );
fittedline_t3 = plot( fitresult, x_fit_t3, y_fit_t3);
lgd = findobj('type', 'legend');
delete(lgd);
set(fittedline_t3, 'Color', 'yellow')

hold on;

ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t4, y_fit_t4, ft, opts );
fittedline_t4 = plot( fitresult, x_fit_t4, y_fit_t4);
lgd = findobj('type', 'legend');
delete(lgd);
set(fittedline_t4, 'Color', 'green')

hold on;

ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t5, y_fit_t5, ft, opts );
fittedline_t5 = plot( fitresult, x_fit_t5, y_fit_t5);
lgd = findobj('type', 'legend');
delete(lgd);
set(fittedline_t5, 'Color', 'blue')

hold on;

ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_t6, y_fit_t6, ft, opts );
fittedline_t6 = plot( fitresult, x_fit_t6, y_fit_t6);
lgd = findobj('type', 'legend');
delete(lgd);
set(fittedline_t6, 'Color', '#4b0082')
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)

hold off;
xlabel('Wavelength in nm');
ylabel('Min-Max Normalized Medians')
title('Timeframes 1-6 Min-Max Normalized Lines of Best Fit')

Mode A PPG Normalized Data for All 6 Channels, Superimposed on Same Plot
figure;

%first, min max normalization
norm_ba = normalize(medians_ba, 'range');

%then smoothing that line
[x_fit_ba, y_fit_ba] = prepareCurveData(1:6, norm_ba );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, ~] = fit( x_fit_ba, y_fit_ba, ft, opts );
fittedline_ba = plot( fitresult, x_fit_ba, y_fit_ba);
lgd = findobj('type', 'legend');
delete(lgd);
set(fittedline_ba, 'Color', 'magenta')
hold on;

%first, min max normalization
norm_aa = normalize(medians_aa, 'range');

%then smoothing that line
[x_fit_aa, y_fit_aa] = prepareCurveData(1:6, norm_aa );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, gof] = fit( x_fit_aa, y_fit_aa, ft, opts );
fittedline_aa = plot( fitresult, x_fit_aa, y_fit_aa);
lgd = findobj('type', 'legend');
delete(lgd);
set(fittedline_aa, 'Color', 'blue')
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)

hold off;

xlabel('Wavelength in nm');
ylabel('Min-Max Normalized Medians')
title('Mode A PPG Normalized Data for All 6 Channels')


%Note: for the third set of data analyzed, spectraderama_data_2025-03-25_sattac520S2.csv

% timeframe1_limits = [36800 38000];
% timeframe2_limits = [8900 9300];
% timeframe3_limits = [14300 14600];  %13400 15000
% timeframe4_limits = [15000 15350]; %15000 21000
% timeframe5_limits = [19750 20000];
% timeframe6_limits = [222500 232000];
% 
% timeframe1_data = all_channels(36800:38000, :);
% timeframe2_data = all_channels(8900:9300, :);
% timeframe3_data = all_channels(14300:14600, :);
% timeframe4_data = all_channels(15000:15350, :);
% timeframe5_data = all_channels(19750:20000, :);
% timeframe6_data = all_channels(222500:232000, :);

To export data segments as zip files, see below:
cd('M:\Daniela''s things\Github\SpectraDerma\BTViz\new_data\final_data_segments')
pwd

% Convert limits to integers
timeframe1_limits = round([250000 290000]); %89200 92900, 222500 232000, 240000 250000, 255000 265000
timeframe2_limits = round([330000 380000]);
timeframe3_limits = round([383500 393000]);
timeframe4_limits = round([300000 310000]); %270000 290000 not useful
timeframe5_limits = round([300000 310000]); %300000 330000 not useful
timeframe6_limits = round([300000 310000]); %              not useful

ppg_before_limits = [1100 1600];
ppg_after_limits  = [400000 421000];

% Extract data segments from the original table
data_tf1 = data(timeframe1_limits(1):timeframe1_limits(2), :);
data_tf2 = data(timeframe2_limits(1):timeframe2_limits(2), :);
data_tf3 = data(timeframe3_limits(1):timeframe3_limits(2), :);
data_tf4 = data(timeframe4_limits(1):timeframe4_limits(2), :);
data_tf5 = data(timeframe5_limits(1):timeframe5_limits(2), :);
data_tf6 = data(timeframe6_limits(1):timeframe6_limits(2), :);
data_ppg_before = data(ppg_before_limits(1):ppg_before_limits(2), :);
data_ppg_after  = data(ppg_after_limits(1):ppg_after_limits(2), :);

% Export to CSV
writetable(data_tf1, 'timeframe1_raw_3.csv');
writetable(data_tf2, 'timeframe2_raw_3.csv');
writetable(data_tf3, 'timeframe3_raw_3.csv');
writetable(data_tf4, 'timeframe4_raw_3.csv');
writetable(data_tf5, 'timeframe5_raw_3.csv');
writetable(data_tf6, 'timeframe6_raw_3.csv');
writetable(data_ppg_before, 'ppg_before_raw_3.csv');
writetable(data_ppg_after,  'ppg_after_raw_3.csv');