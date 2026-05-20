function [TIrr,d_TIrr] = Track_Irr

% filename = 'TrackIrr_WhiteNoise_5To2000Hz_160km_0.01mm.txt';
% Fs_TIrr = 5000;
% pos_start = j0 + 2.5;
% [TIrr, d_TIrr] = Load_TIrr(filename,Fs_TIrr,pos_start,Vlc);
% figure(30); clf
% subplot(2,1,1); plot(TIrr(:,1),TIrr(:,2)); grid on
% subplot(2,1,2); plot(d_TIrr(:,1),d_TIrr(:,2)); grid on

% 不考虑轨道不平顺
TIrr = zeros(2,9);
d_TIrr = zeros(2,9);
TIrr(:,1) = [-1000,1000];
d_TIrr(:,1) = [-1000,1000];