%% 快速傅里叶变换

function [Fs, f, s_fft] = fun_FFT(Fs, s)

% Fs = 1/min(diff(t));
N=length(s);
NFFT = 2^nextpow2(N);
% NFFT = N;
Y = fft(s,NFFT)/N;
f = Fs/2*linspace(0,1,NFFT/2+1);
s_fft = 2*abs(Y(1:NFFT/2+1));