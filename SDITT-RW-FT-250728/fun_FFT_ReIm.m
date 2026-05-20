%% 快速傅里叶变换，补零，保留输出结果的实部和虚部

function [Fs, f, oup_fft, s_fft, p_fft, Re_fft, Im_fft] = fun_FFT_ReIm(Fs, s, L_zero)

% % Fs = 1/min(diff(t));
% N=length(s);
% % NFFT = 2^nextpow2(N);
% % NFFT = 2^(nextpow2(N)+2);
% NFFT = N;
% Y = fft(s,NFFT)/N;
% f = Fs/2*linspace(0,1,NFFT/2+1);
% s_fft = 2*abs(Y(1:NFFT/2+1));

L0 = length(s);           % 信号长度
if mod(L0+L_zero, 2)~=0
    L_zero = L_zero+1;
end
L = L0+L_zero;          % 数据长度

y = zeros(1,L);
y(1:L0) = s;                 % 补零后的信号

Y = fft(y);                   % 快速傅里叶变换
% 计算双侧频谱 P2；然后基于 P2 和偶数信号长度 L 计算单侧频谱 P1。

P2 = abs(Y/L0);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;     % Rfft
s_fft = P1;

P2_p = angle(Y/L0);
p_fft = P2_p(1:L/2+1);

Re_fft = s_fft .* cos(p_fft);
Im_fft = s_fft .* sin(p_fft);

oup_fft = Re_fft+1i*Im_fft;

% f = Fs/2*linspace(0,1,NFFT/2+1);


%% FFT 补零 https://zhuanlan.zhihu.com/p/85863024
% clear;clc
% close all
% %% FFT
% Fs = 100e6;		    % 采样频率 Hz
% T = 1/Fs;		    % 采样周期 s
% L0 = 1000;                  % 信号长度
% L = 7000;                   % 数据长度
% t0 = (0:L0-1)*T;            % 信号时间序列
% x = cos(2*pi*1e6*t0) + cos(2*pi*1.05e6*t0); % 信号函数
% t = (0:L-1)*T;              % 数据时间序列
% y = zeros(1,L);
% y(1:L0) = x;
% %% Plot
% figure(1)
% plot(t*1e6,y,'b-','linewidth',1.5)
% title('\fontsize{10}\fontname{Times New Roman}Time domain signal with Zero Padding')
% xlabel('\fontsize{10}\fontname{Times New Roman}\it t /\rm \mus')
% ylabel('\fontsize{10}\fontname{Times New Roman}\it y\rm(\itt\rm)')
% grid on;
% axis([0 70 -2 2])
% set(gca,'FontSize', 10 ,'FontName', 'Times New Roman')
% set(gcf,'unit','centimeters','position',[15 10 13.53 9.03],'color','white')
% %% FFT
% Y = fft(y);                   % 快速傅里叶变换
% % 计算双侧频谱 P2。然后基于 P2 和偶数信号长度 L 计算单侧频谱 P1。
% P2 = abs(Y/L0);
% P1 = P2(1:L/2+1);
% P1(2:end-1) = 2*P1(2:end-1);
% f = Fs*(0:(L/2))/L;% Rfft
% figure(2)
% plot(f, P1,'r-','Marker','.','markersize',10,'linewidth',1.5)
% axis([0.5e6 1.5e6 0 1.5])
% title('\fontsize{10}\fontname{Times New Roman}Power Spectrum')
% xlabel('\fontsize{10}\fontname{Times New Roman}\it f /\rm Hz')
% ylabel('\fontsize{10}\fontname{Times New Roman}\it y\rm(\itf\rm)')
% grid on;
% set(gca,'FontSize', 10 ,'FontName', 'Times New Roman')
% set(gcf,'unit','centimeters','position',[15 10 13.53 9.03],'color','white')