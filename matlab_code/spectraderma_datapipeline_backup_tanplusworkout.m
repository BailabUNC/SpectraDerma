clc; clear; close all;

data = readtable("M:\Daniela's things\Github\SpectraDerma\BTViz\data_tanning\BTVIZ_2025-07-17 (1).csv");

data

%time = data.id;
%time = cell2mat(time);
%time = string(time);

%newtime = cellfun(@(x) extractBetween(x, 12, 19), time, 'UniformOutput', true);

ch0 = data.data_0;
ch1 = data.data_1;
ch2 = data.data_2;
ch3 = data.data_3;
ch4 = data.data_4;
ch5 = data.data_5;

all_channels_raw = [ch0, ch1, ch2, ch3, ch4, ch5];
plot(all_channels_raw)

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


Inner Arm/Wrist (lighter, non-tanned)- all
figure('Position', [100, 100, 1200, 800]);
hold on;
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

xlim([7318 9208])
title('Inner Arm/Wrist (non-tanned)- all', 'FontSize', 30)
%30Hz sampling frequency but not consistent
set(gca, 'Fontsize', 30)

Elbow (tanned)- all
figure('Position', [100, 100, 1200, 800]);
hold on;
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;
%plot(all_channels)
xlim([9209 12792])
title('Elbow (tanned)- all', 'FontSize', 30)
set(gca, 'Fontsize', 30)

Inner Arm/Wrist (lighter, non-tanned)- segment
figure('Position', [100, 100, 1200, 800]);
hold on;
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

xlim([7550 8740])
title('Inner Arm/Wrist (non-tanned)- all', 'FontSize', 30)
%30Hz sampling frequency but not consistent
set(gca, 'Fontsize', 30)

Elbow (tanned)- segment
figure('Position', [100, 100, 1200, 800]);
hold on;
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;
%plot(all_channels)
xlim([11100 12750])
title('Elbow (tanned)- all', 'FontSize', 30)
set(gca, 'Fontsize', 30)

size_font = 11; %this is the font size for timeframes 1-6 plots, set to 30 when plotting individually

figure('Position', [100, 100, 1200, 800]);
hold on;
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

xlim([0 421000]) 
title('Timeframe 1', 'FontSize', size_font)
%30Hz sampling frequency but not consistent
set(gca, 'Fontsize', size_font)

Raw Timeframes 1-6 Plots
timeframe1_limits = [7550 8740];
timeframe2_limits = [11100 12750];

timeframe1_data = all_channels(7550:8740, :);
timeframe2_data = all_channels(11100:12750, :);

size_font = 11; %this is the font size for timeframes 1-6 plots, set to 30 when plotting individually

subplot(2,1,1)
%figure('Position', [100, 100, 1200, 800]);
hold on;
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

xlim(timeframe1_limits)
title('Timeframe 1', 'FontSize', size_font)
%30Hz sampling frequency but not consistent
set(gca, 'Fontsize', size_font)

subplot(2,1,2)
%figure('Position', [100, 100, 1200, 800]);
hold on;
%subplot(2,2,2)
for i = 1:size(all_channels, 2)
    plot(all_channels(:, i), colors(i));
end
hold off;

xlim(timeframe2_limits)
title('Timeframe 2', 'FontSize', size_font)
%30Hz sampling frequency but not consistent
set(gca, 'Fontsize', size_font)

Defining the segments that I'll plot later
figure;
subplot(2,2,1);
segment_1 = all_channels(7550:8740, 1:2);
%plot(segment);
boxplot(segment_1, "Orientation", "horizontal");
title('wrist');

subplot(2,2,2);
segment_2 = all_channels(7550:8740, 3:4);
%plot(segment);
boxplot(segment_2, "Orientation", "horizontal");
title('wrist');

subplot(2,2,3);
segment_3 = all_channels(7550:8740, 5:6);
%plot(segment);
boxplot(segment_3, "Orientation", "horizontal");
title('wrist');

subplot(2,2,4);
segment = all_channels(7550:8740, :);
%plot(segment);
boxplot(segment, "Orientation", "horizontal");
title('wrist-all');

figure;
subplot(2,2,1);
segment_1a = all_channels(11100:12750, 1:2);
boxplot(segment_1a, "Orientation", "horizontal");
title('elbow');

subplot(2,2,2);
segment_2a = all_channels(11100:12750, 3:4);
boxplot(segment_2a, "Orientation", "horizontal");
title('elbow');

subplot(2,2,3);
segment_3a = all_channels(11100:12750, 5:6);
boxplot(segment_3a, "Orientation", "horizontal");
title('elbow');

subplot(2,2,4);
segmenta = all_channels(11100:12750, :);
boxplot(segmenta, "Orientation", "horizontal");
title('elbow', 'FontSize', 16);

Boxplots with smooth line between medians of COLOR SIGNALS
x_axis_labels = [415, 480, 555, 590, 680, 780]; %630 445 515

%PPG BEFORE ALCOHOL, MODE A , polyfit looks better for some reason
figure;
boxplot(segment);
hold on;
title('Wrist (Inner Arm)', 'FontSize', 16);
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts (Light Intensity), Unitless')
medians_ba = median(segment);
p = polyfit((1:6), medians_ba, 4);
x_fit = linspace(1, 6, 100);
y_fit = polyval(p, x_fit);
plot(x_fit, y_fit, 'm-')

hold off;


%PPG AFTER ALCOHOL, MODE A, the polyfit looks better for some reason
figure;
boxplot(segmenta);
hold on;
title('Elbow', 'FontSize', 16);
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
ylabel('Raw Digital Counts (Light Intensity), Unitless')
medians_aa = median(segmenta);
p = polyfit((1:6), medians_aa, 4);
x_fit = linspace(1, 6, 100);
y_fit = polyval(p, x_fit);
plot(x_fit, y_fit, 'm-')

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

hold off;


Timeframes 1-6 figures with boxplots and smoothed line
%TIMEFRAME 1 MODE B
subplot(1,2,1);
segment_t1 = timeframe1_data;
boxplot(segment_t1);
hold on;
title('wrist', 'FontSize', 12);
set(gca, 'xtick', 1:6, "XTickLabels", x_axis_labels)
xlabel('Wavelength in nm')
medians_t1 = median(segment_t1);

[x_fit1, y_fit1] = prepareCurveData(1:6, medians_t1 );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, gof] = fit( x_fit1, y_fit1, ft, opts );
fittedline_1 = plot( fitresult, x_fit1, y_fit1);
lgd = findobj('type', 'legend');
delete(lgd);
set(fittedline_1, 'Color', 'magenta')
hold off;
xlabel('Wavelength in nm')

%TIMEFRAME 2 MODE B
subplot(1,2,2);
segment_t2 = timeframe2_data;
boxplot(segment_t2);
hold on;
title('elbow', 'FontSize', 12);
set(gca, 'xtick', 1:6, "XTickLabels", x_axis_labels)
xlabel('Wavelength in nm')
medians_t2 = median(segment_t2);

[x_fit2, y_fit2] = prepareCurveData(1:6, medians_t2 );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, gof] = fit( x_fit2, y_fit2, ft, opts );
fittedline_2 = plot( fitresult, x_fit2, y_fit2);
lgd = findobj('type', 'legend');
delete(lgd);
set(fittedline_2, 'Color', 'magenta')
hold off;
xlabel('Wavelength in nm')

% pos = get(gcf, 'Position');
% set(gcf, 'Position',pos+[0 -500 0 -500])


Normalized Data (Min-Max) as a smoothed line, no box plots, Timeframes 1-6
subplot(1,2,1);
%first, min max normalization
norm_t1 = normalize(medians_t1, 'range');

%then smoothing that line
[x_fit_t1, y_fit_t1] = prepareCurveData(1:6, norm_t1 );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, gof] = fit( x_fit_t1, y_fit_t1, ft, opts );
fittedline_t1 = plot( fitresult, x_fit_t1, y_fit_t1);
lgd = findobj('type', 'legend');
delete(lgd);
set(fittedline_t1, 'Color', 'magenta')
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
title('wrist Normalized Data')



subplot(1,2,2);
%first, min max normalization
norm_t2 = normalize(medians_t2, 'range');

%then smoothing that line
[x_fit_t2, y_fit_t2] = prepareCurveData(1:6, norm_t2 );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, gof] = fit( x_fit_t2, y_fit_t2, ft, opts );
fittedline_t2 = plot( fitresult, x_fit_t2, y_fit_t2);
lgd = findobj('type', 'legend');
delete(lgd);
set(fittedline_t2, 'Color', 'magenta')
set(gca, 'xtick',1:6,'XTickLabels', x_axis_labels)
xlabel('Wavelength in nm')
title('elbow Normalized Data')

% pos = get(gcf, 'Position');
% set(gcf, 'Position',pos+[0 200 0 200])

Timeframes 1-6 Min-Max Normalized Lines of Best Fit, Superimposed on Same Plot
figure;

ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, gof] = fit( x_fit_t1, y_fit_t1, ft, opts );
fittedline_t1 = plot( fitresult, x_fit_t1, y_fit_t1);
lgd = findobj('type', 'legend');
delete(lgd);
set(fittedline_t1, 'Color', 'red')

hold on;

ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, gof] = fit( x_fit_t2, y_fit_t2, ft, opts );
fittedline_t2 = plot( fitresult, x_fit_t2, y_fit_t2);
lgd = findobj('type', 'legend');
delete(lgd);
set(fittedline_t2, 'Color', '#f80')

hold off;
xlabel('Wavelength in nm');
ylabel('Min-Max Normalized Medians')
title('wrist and elbow Min-Max Normalized Lines of Best Fit')

Mode A PPG Normalized Data for All 6 Channels, Superimposed on Same Plot
figure;

%first, min max normalization
norm_ba = normalize(medians_ba, 'range');

%then smoothing that line
[x_fit_ba, y_fit_ba] = prepareCurveData(1:6, norm_ba );
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 1;
[fitresult, gof] = fit( x_fit_ba, y_fit_ba, ft, opts );
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
title('Wrist and Elbow Normalized Data for All 6 Channels')
legend('', 'Wrist', '', 'Elbow');


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


Fs = 1000; % sampling frequency, adjust to your data rate
t = 0:1/Fs:1; % duration 1s
hychirp = chirp(t,0,1,200);

class(ch0_new);
ch0_new_chirp = ch0_new(7550:8740);
ch0_new_chirp;

class(ch1_new);
ch1_new_chirp = ch1_new(7550:8740);
ch1_new_chirp;

class(ch2_new);
ch2_new_chirp = ch2_new(7550:8740);
ch2_new_chirp;

class(ch3_new);
ch3_new_chirp = ch3_new(7550:8740);
ch3_new_chirp;

class(ch4_new);
ch4_new_chirp = ch4_new(7550:8740);
ch4_new_chirp;

class(ch5_new);
ch5_new_chirp = ch5_new(7550:8740);
ch5_new_chirp;



[corr, lags] = xcorr(ch0_new_chirp, hychirp);
lags_time = lags / Fs;

figure;
sgtitle('Wrist Hychirp Data');
subplot(3,2,1);
plot(lags_time, corr)
xlabel('Time Lag (s)')
ylabel('Cross-correlation')
title('Channel 0')
grid on

[corr, lags] = xcorr(ch1_new_chirp, hychirp);
lags_time = lags / Fs;

%figure;
subplot(3,2,2);
plot(lags_time, corr)
xlabel('Time Lag (s)')
ylabel('Cross-correlation')
title('Channel 1')
grid on

[corr, lags] = xcorr(ch2_new_chirp, hychirp);
lags_time = lags / Fs;

%figure;
subplot(3,2,3);
plot(lags_time, corr)
xlabel('Time Lag (s)')
ylabel('Cross-correlation')
title('Channel 2')
grid on

[corr, lags] = xcorr(ch3_new_chirp, hychirp);
lags_time = lags / Fs;

% figure;
subplot(3,2,4);
plot(lags_time, corr)
xlabel('Time Lag (s)')
ylabel('Cross-correlation')
title('Channel 3')
grid on

[corr, lags] = xcorr(ch4_new_chirp, hychirp);
lags_time = lags / Fs;

% figure;
subplot(3,2,5);
plot(lags_time, corr)
xlabel('Time Lag (s)')
ylabel('Cross-correlation')
title('Channel 4')
grid on

[corr, lags] = xcorr(ch5_new_chirp, hychirp);
lags_time = lags / Fs;

% figure;
subplot(3,2,6);
plot(lags_time, corr)
xlabel('Time Lag (s)')
ylabel('Cross-correlation')
title('Channel 5')
grid on

class(ch0_new);
ch0_new_chirp2 = ch0_new(11100:12750);
ch0_new_chirp2;

class(ch1_new);
ch1_new_chirp2 = ch1_new(11100:12750);
ch1_new_chirp2;

class(ch2_new);
ch2_new_chirp2 = ch2_new(11100:12750);
ch2_new_chirp2;

class(ch3_new);
ch3_new_chirp2 = ch3_new(11100:12750);
ch3_new_chirp2;

class(ch4_new);
ch4_new_chirp2 = ch4_new(11100:12750);
ch4_new_chirp2;

class(ch5_new);
ch5_new_chirp2 = ch5_new(11100:12750);
ch5_new_chirp2;


[corr, lags] = xcorr(ch0_new_chirp2, hychirp);
lags_time = lags / Fs;

figure;
sgtitle('Elbow Hychirp Data');
subplot(3,2,1)
plot(lags_time, corr)
xlabel('Time Lag (s)')
ylabel('Cross-correlation')
title('Channel 0')
grid on

[corr, lags] = xcorr(ch1_new_chirp2, hychirp);
lags_time = lags / Fs;

% figure;
subplot(3,2,2)
plot(lags_time, corr)
xlabel('Time Lag (s)')
ylabel('Cross-correlation')
title('Channel 1')
grid on

[corr, lags] = xcorr(ch2_new_chirp2, hychirp);
lags_time = lags / Fs;

% figure;
subplot(3,2,3)
plot(lags_time, corr)
xlabel('Time Lag (s)')
ylabel('Cross-correlation')
title('Channel 2')
grid on

[corr, lags] = xcorr(ch3_new_chirp2, hychirp);
lags_time = lags / Fs;

% figure;
subplot(3,2,4)
plot(lags_time, corr)
xlabel('Time Lag (s)')
ylabel('Cross-correlation')
title('Channel 3')
grid on

[corr, lags] = xcorr(ch4_new_chirp2, hychirp);
lags_time = lags / Fs;

% figure;
subplot(3,2,5)
plot(lags_time, corr)
xlabel('Time Lag (s)')
ylabel('Cross-correlation')
title('Channel 4')
grid on

[corr, lags] = xcorr(ch5_new_chirp2, hychirp);
lags_time = lags / Fs;

% figure;
subplot(3,2,6)
plot(lags_time, corr)
xlabel('Time Lag (s)')
ylabel('Cross-correlation')
title('Channel 5')
grid on
