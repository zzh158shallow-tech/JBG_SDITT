function FZ_Measure = TIrr_WideFreq

% clc
% clear

global N_ConPatch TIrr d_TIrr Vlc

% addpath('C:\Users\Kayan Chan\Desktop\2022-07-09 WideFreq Impact')
addpath('C:\Users\jiayi\Desktop\2022-07-09 WideFreq Impact')


Choose_Plot = 0;

%% Load Data
% Vlc = 302/3.6;
filename = 'Inspection Data.xlsx';
Data = readtable(filename, 'Sheet', 'TIrr_FZ', 'ReadVariableNames', true);
temp_Data = table2array(Data);
TIrr_Z_inp(:,1) = temp_Data(:,1);
TIrr_Z_inp(:,2) = temp_Data(:,5);
FZ_Measure_inp = [TIrr_Z_inp(:,1), temp_Data(:,end)];

% É¾³ýÖØ¸´Ïî£º7384-7426, 15394-15398, 23394-23411
bools_del = [7384:1:7426, 15394:1:15398, 23394:1:23411];
TIrr_Z_inp(bools_del,:) = [];
FZ_Measure_inp(bools_del,:) = [];

if Choose_Plot == 1
    figure(21); clf
    subplot(2,1,1)
    plot(TIrr_Z_inp(:,1), TIrr_Z_inp(:,2)); grid on
    subplot(2,1,2)
    plot(TIrr_Z_inp(1:end-1,1), diff(TIrr_Z_inp(:,1))); grid on
end

T = (TIrr_Z_inp(:,1)-TIrr_Z_inp(1,1))*1000/Vlc;
Fs = 1/mean(diff(T));
Fs = 2000;

% Wavelength - PSD
yy = TIrr_Z_inp(:,2);
nfft = 2^nextpow2(length(yy));
win = {'[]', 'rectwin(64)', 'rectwin(256)', 'rectwin(512)', 'rectwin(1028)', 'rectwin(2048)', 'rectwin(4096)', 'rectwin(8192)'};
[Pyy, Freq] = cpsd(yy, yy, rectwin(2048), [], nfft, Fs);
PSD_yy = Pyy / (Fs/nfft);
Wavelength = 1./Freq*Vlc;

% ZP_Dyn.Dis_TIrr.FF.R1
% yy = ZP_Dyn.Dis_TIrr.FF.R1(:,4)*1000;
% nfft = 2^nextpow2(length(yy));
% Fs = 1/drtaT;
% win = {'[]', 'rectwin(64)', 'rectwin(256)', 'rectwin(512)', 'rectwin(1028)', 'rectwin(2048)', 'rectwin(4096)', 'rectwin(8192)'};
% [Pyy, Freq] = cpsd(yy, yy, rectwin(2048), [], nfft, Fs);
% PSD_yy = Pyy / (Fs/nfft);
% Wavelength = 1./Freq*Vlc;

if Choose_Plot == 1
    figure(22); clf
    plot(Wavelength, PSD_yy); grid on
    set(gca,'YMinorTick','on','YScale','log');
    set(gca,'XMinorTick','on','XScale','log');
    set(gca,'xdir','reverse')
    xlabel('Wavelength (m)')
    ylabel('PSD (mm^2/Hz)')
    xlim([8e-2, 100])
%     xlim([2e-3, 100])
end

%% Target_Range: 842.850~842.900km
% ²¨Ä¥¶Î£º842.880~843.086km
clear TIrr_Z
dX_TIrr_Spline = 4;
X_TIrr_Start = 30;
bools = TIrr_Z_inp(:,1)>=842.850 & TIrr_Z_inp(:,1)<=842.850+0.1;
TIrr_Z = TIrr_Z_inp(bools,:);
TIrr_Z(:,1) = (TIrr_Z(:,1)-TIrr_Z(1,1))*1000+X_TIrr_Start;
TIrr_Z(:,2) = TIrr_Z(:,2) / 1000;
FZ_Measure = FZ_Measure_inp(bools,:);
FZ_Measure(:,1) = (FZ_Measure(:,1)-FZ_Measure(1,1))*1000+X_TIrr_Start;

% TIrr_Z, TIrr_Z_d1
x0 = X_TIrr_Start-dX_TIrr_Spline;
xx_1 = [(0:5:x0-0.5), x0]';
TIrr_Z = [xx_1, zeros(length(xx_1),1);
                TIrr_Z];
            
xx_2 = (x0+0.1 : 0.1 : X_TIrr_Start-0.1)';
yy_2 = interp1(TIrr_Z(:,1), TIrr_Z(:,2), xx_2, 'spline');
TIrr_Z = [TIrr_Z; xx_2, yy_2];
TIrr_Z = sortrows(TIrr_Z, 1);

TIrr_Z_d1 = diff(TIrr_Z(:,2)) ./ (diff(TIrr_Z(:,1))/Vlc);

% TIrr
len = size(TIrr_Z, 1);
TIrr = zeros(len, N_ConPatch*2+1);
d_TIrr = zeros(len-1, N_ConPatch*2+1);
TIrr(:,1) = TIrr_Z(:,1);
d_TIrr(:,1) = TIrr_Z(1:end-1,1);
for i2 = 1:1:2
    TIrr(:,2*i2) = TIrr_Z(:,2);
    d_TIrr(:,2*i2) = TIrr_Z_d1;    
end

if Choose_Plot == 1
    figure(23); clf
    subplot(3,1,1)
    plot(TIrr(:,1), TIrr(:,2)); grid on
    xlabel('Mileage (m)'); ylabel('TIrr_Z (m)')
    xlim([0,140]);
    
    subplot(3,1,2)
    plot(d_TIrr(:,1), d_TIrr(:,2)); grid on
    xlabel('Mileage (m)'); ylabel('TIrr_Z_d1 (m/s)')
    xlim([0,140]);
    
    subplot(3,1,3)
    plot(FZ_Measure(:,1), FZ_Measure(:,2)); grid on
    xlim([0,140]);
    xlabel('Mileage (m)'); ylabel('FZ (kN)')
end

