%% 导入轨道不平顺
function [TIrr, d_TIrr] = Load_TIrr(Vlc, j0, drtaT)

% clc
% clear

% Vlc = 160/3.6;
% j0 = -t_preload*Vlc;
% drtaT = 1e-4/4;

Choose_Plot = 0;

%% Chirp: Swept-frequency cosine
% drtaT = drtaT/4;
% t_start = 0;
% t_preload = 0.025;
% t_end = 1+t_preload;
% t_sum = (t_start:drtaT:t_end)';
% Fs = 1/drtaT;
% 
% % Single = chirp(t_sum, 0, t_end, 2200, 'linear')/1e5;
% % Excitation = [t_sum Single(:,1)];
% 
% % Single = [chirp(t_sum, 0, t_end, 2200, 'quadratic', 0,'convex')/1e5, (1:1:length(t_sum))'];
% % Single = sortrows(Single,-2);
% % Excitation = [t_sum Single(:,1)];
% 
% Single = chirp(t_sum(t_sum>=0 & t_sum<=t_end-t_preload), 0, t_end-t_preload, 2200, 'linear', 90)/2e5;
% Excitation = [t_sum zeros(length(t_sum),1)];
% Excitation(t_sum>=t_preload & t_sum<=t_end,2) = Single;
% 
% Excitation_d1 = [t_sum(1), 0; t_sum(2:end), diff(Excitation(:,2))/drtaT];

%% White Noise
Excitation = load('TrackIrr_WhiteNoise_5To2000Hz_160km_0.01mm.txt');
Excitation(:,1) = Excitation(:,1) / Vlc;
Excitation(:,2) = Excitation(:,2) / 2;
drtaT = min(diff(Excitation(:,1)));
Fs = 1/drtaT;

t_start = Excitation(1,1);
t_end = Excitation(end,1);
t_sum = Excitation(:,1);

Excitation_d1 = [Excitation(1,1), 0; Excitation(2:end,1), diff(Excitation(:,2))/drtaT];

if Choose_Plot == 1
    figure(11); clf
    subplot(2,1,1)
    plot(Excitation(:,1),Excitation(:,2));  grid on;
    xlabel('Time (s)');
    ylabel('Dis-Excitation (m)');
    subplot(2,1,2)
    plot(Excitation_d1(:,1), Excitation_d1(:,2));  grid on;
    xlabel('Time (s)');
    ylabel('Vel-Excitation (m/s)');
    
    figure(12); clf
    nfft = 2^nextpow2(length(Excitation));
    spectrogram(Excitation(:,2),128,128/2,nfft,Fs,'yaxis');
    set(gca,'Fontsize',11, 'fontname','Times');
end


%% TIRR: [Time*Vlc, L1-Ver, L1-Lat, L2-Ver, L2-Lat, R1-Ver, R1-Lat, R2-Ver, R2-Lat];
% temp = load(filename);

TIrr = zeros(length(Excitation)+4,9);
d_TIrr = zeros(length(Excitation)+4,9);
TIrr(:,1) = [(t_start-1)*Vlc; ...
             (t_start-drtaT)*Vlc; ...
              t_sum*Vlc; ...
             (t_end+drtaT)*Vlc; ...
             (t_end+1)*Vlc];
% TIrr(:,1) = [(t_start-1)*Vlc+j0; ...
%              (t_start-drtaT)*Vlc+j0; ...
%               t_sum*Vlc+j0; ...
%              (t_end+drtaT)*Vlc+j0; ...
%              (t_end+1)*Vlc+j0];
d_TIrr(:,1) = TIrr(:,1);

Range_col = [1,7]+1;
for k = 1:1:length(Range_col)
    col = Range_col(k);
    TIrr(3:end-2,col) = Excitation(:,2);
    d_TIrr(3:end-2,col) = Excitation_d1(:,2);
end

if Choose_Plot == 1
    figure(13); clf
    subplot(2,1,1); plot(TIrr(:,1),TIrr(:,2)); grid on
    xlabel('Mileage (m)'); ylabel('Dis-Excitation (m)');
    subplot(2,1,2); plot(d_TIrr(:,1),d_TIrr(:,2)); grid on
    xlabel('Mileage (m)'); ylabel('Vel-Excitation (m/s)');
    
    figure(14); clf
    [~, f, s_fft] = fun_FFT(Fs, Excitation(:,2));
    plot(f,s_fft); grid on
    
end
