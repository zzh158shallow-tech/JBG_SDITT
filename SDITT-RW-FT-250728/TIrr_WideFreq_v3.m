function InpPar = TIrr_WideFreq_v3(InpPar, dX_TIrr_Spline, X_TIrr_Simulation_Start, dX_TIrr, filename)

% addpath('G:\Rigid-Flex Interaction\# 道岔区实测数据\1a-京哈高铁道岔区轮轨力和不平顺数据')
% addpath('D:\# Projects\# Rigid-Flexible Interaction\# 道岔区实测数据\1a-京哈高铁道岔区轮轨力和不平顺数据')
% addpath('G:\Rigid-Flex Interaction\# 道岔区实测数据\4a-沈大高铁鲅鱼圈北站18号道岔')
addpath('D:\# Projects\# Rigid-Flexible Interaction\# 道岔区实测数据\4a-沈大高铁鲅鱼圈北站18号道岔')

Choose_Plot = 0;
FontSize = 11;

%% A. Load Data
% A1. TIrr, FZ
% Mileage, Y_L, Y_R, Z_L, Z_R
% filename = 'TIrr_GSDS-Dalianbei-Shenyangbei-07112020-095710-1(280-18)_pic.txt';
inp_TIrr_ori = load(filename);

if Choose_Plot == 1

    figure(22); clf
    subplot(2,2,1)
    plot(inp_TIrr_ori(:,1), inp_TIrr_ori(:,2)*1000); grid on
    title('左轨向')
    subplot(2,2,2)
    plot(inp_TIrr_ori(:,1), inp_TIrr_ori(:,3)*1000); grid on
    title('右轨向')
    subplot(2,2,3)
    plot(inp_TIrr_ori(:,1), inp_TIrr_ori(:,4)*1000); grid on
    title('左高低')
    set(gca, 'ydir', 'reverse');
    subplot(2,2,4)
    plot(inp_TIrr_ori(:,1), inp_TIrr_ori(:,5)*1000); grid on
    title('右高低')
    set(gca, 'ydir', 'reverse');
    set(gcf, 'Position', [185, 250, 1230, 535]);

    figure(21); clf
    subplot(2,1,1)
    plot(inp_TIrr_ori(:,1), inp_TIrr_ori(:,4)); hold on
    plot(inp_TIrr_ori(:,1), inp_TIrr_ori(:,5)); hold on
    grid on
    xlabel('Mileage (m)'); ylabel('Ver. TIrr (mm)')
    set(gca, 'ydir', 'reverse');
    set(gca, 'FontName', 'Times', 'FontSize', 11);

    subplot(2,1,2)
    plot(inp_TIrr_ori(1:end-1,1), diff(inp_TIrr_ori(:,4))); hold on
    plot(inp_TIrr_ori(1:end-1,1), diff(inp_TIrr_ori(:,5))); hold on
    grid on
    xlabel('Mileage (m)'); ylabel('diff TIrr_X (m)')
    set(gca, 'FontName', 'Times', 'FontSize', 11);

end

% A3. TIrr: Wavelength - PSD
Choose_PSD = 0;
if Choose_PSD == 1
    T = (inp_TIrr_ori(:,1)-inp_TIrr_ori(1,1))/InpPar.Vlc;
    Fs = 1/mean(diff(T));
%     Fs = 2000;
    
    yy = inp_TIrr_ori(:,5)*1000;
    nfft = 2^nextpow2(length(yy));
    win = {'[]', 'rectwin(64)', 'rectwin(256)', 'rectwin(512)', 'rectwin(1028)', 'rectwin(2048)', 'rectwin(4096)', 'rectwin(8192)'};
    [Pyy, Freq] = cpsd(yy, yy, rectwin(2^(nextpow2(length(yy))-1)), [], nfft, Fs);
    PSD_yy = Pyy / (Fs/nfft);
    Wavelength = 1./Freq*InpPar.Vlc;
    
    figure(22); clf
    plot(Wavelength, PSD_yy); grid on
    set(gca,'YMinorTick','on','YScale','log');
    set(gca,'XMinorTick','on','XScale','log');
    set(gca,'xdir','reverse')
    xlabel('Wavelength (m)')
    ylabel('PSD (mm^2/Hz)')
    xlim([1, 100])
    set(gca, 'FontName', 'Times', 'FontSize', 11);
%     xlim([2e-3, 100])
end

%% B. Target_Range: 842.850~842.900km
% B1a. inp_TIrr
clear TIrr inp_TIrr_d1

inp_TIrr = inp_TIrr_ori;
inp_TIrr(:,1) = inp_TIrr(:,1)+dX_TIrr;

% TIrr_Z 前移1.25-1.75m，TIrr_Y 不变
% n = 1.75/0.25;
% % n = 1.50/0.25;
% % n = 1.25/0.25;
% inp_TIrr(1:end-n,4:5) = inp_TIrr(n+1:end,4:5);
% inp_TIrr(end-n+1:end,:) = [];


bools_1 = inp_TIrr(:,1)>=X_TIrr_Simulation_Start;
inp_TIrr = inp_TIrr(bools_1,:);

x0 = X_TIrr_Simulation_Start-dX_TIrr_Spline;
xx_1 = [(0:5:x0-0.5), x0]';
inp_TIrr = [xx_1, zeros(length(xx_1),size(inp_TIrr, 2)-1); inp_TIrr];

xx_2 = (x0+0.1 : 0.1 : X_TIrr_Simulation_Start-0.1)';
yy_2 = interp1(inp_TIrr(:,1), inp_TIrr(:,2:5), xx_2, 'spline');
inp_TIrr = [inp_TIrr; xx_2, yy_2];
inp_TIrr = sortrows(inp_TIrr, 1);

xx_3 = max(inp_TIrr(:,1))+0.5 : 5 : 200;
inp_TIrr = [inp_TIrr; xx_3', zeros(length(xx_3),size(inp_TIrr, 2)-1)];

% B1b. 尖轨尖端和辙叉实际咽喉的轨距标记点处，消除因横向结构不平顺引起的峰值
% 20190806
% Range_x = [49.5, 50.75; 102.5, 103.75];
% 20191118
% Range_x = [49.75, 51.25; 103, 104.25]+(dX_TIrr-50);
% bools = (inp_TIrr(:,1)>=Range_x(1,1)-0.05 & inp_TIrr(:,1)<=Range_x(1,2)+0.05) | ...
%               (inp_TIrr(:,1)>=Range_x(2,1)-0.05 & inp_TIrr(:,1)<=Range_x(2,2)+0.05);
% x = inp_TIrr(bools,1);
% inp_TIrr(bools,3) = interp1(inp_TIrr(~bools,1), inp_TIrr(~bools,3), x, 'spline');


% B1c. 长心轨降低值范围内 103.25 (cxg_12.5) ~ 104.75 (cxg_61)，消除因垂向结构不平顺引起的峰值
% % TIrr_Z-1.75m
% % Range_x = [103.25, 104.75];
% % Range_x = [104.00, 104.75]
% 
% % Range_x = [103.25, 104.50];
% % Range_x = [103.25, 104.25];
% Range_x = [103.25, 104.25]+(dX_TIrr-50);
% % Range_x = [103.25, 106.0]+(dX_TIrr-50);           % Test
% 
% % TIrr_Z-1.50m
% % Range_x = [103.25, 104.25]+0.25+(dX_TIrr-50);
% 
% bools = (inp_TIrr(:,1)>=Range_x(1,1)-0.05 & inp_TIrr(:,1)<=Range_x(1,2)+0.05);
% x = inp_TIrr(bools,1);
% % inp_TIrr(bools,5) = interp1(inp_TIrr(~bools,1), inp_TIrr(~bools,5), x, 'spline');
% inp_TIrr(bools,5) = interp1(inp_TIrr(~bools,1), inp_TIrr(~bools,5), x, 'pchip');
% % inp_TIrr(bools,5) = interp1(inp_TIrr(~bools,1), inp_TIrr(~bools,5), x, 'makima');


% B1d. 保证尖轨尖端和辙叉实际咽喉处轨向突变量为正（向外侧）
% inp_TIrr(:,3) = inp_TIrr(:,3)*-1;
inp_TIrr(:,2:3) = inp_TIrr(:,2:3)*-1;           % -Y
% 辙叉垂向结构不平顺原始右高低为负，修改为正向下
% inp_TIrr(:,4:5) = inp_TIrr(:,4:5)*-1;

% inp_TIrr_d1
inp_TIrr_d1(:,1) = inp_TIrr(:,1);
inp_TIrr_d1(2:end,2:5) = diff(inp_TIrr(:,2:5)) ./ repmat((diff(inp_TIrr(:,1))/InpPar.Vlc),1,4);
% inp_TIrr_d1(2:end,2:5) = diff(inp_TIrr(:,2:5)) ./ (diff(inp_TIrr(:,1))/InpPar.Vlc);

% B2. TIrr, d_TIrr
% inp_TIrr, inp_TIrr_d1: [Mileage, Y_L, Y_R, Z_L, Z_R]
% TIrr: [Mileage, Z_L1, Y_L1, Z_R1, Y_R1, Z_R2, Y_R2, Z_R3, Y_R3]
len = size(inp_TIrr_d1, 1);
TIrr = zeros(len, InpPar.N_ConPatch*2+1);
TIrr(:,1) = inp_TIrr(:,1);
d_TIrr = TIrr;

% Left
TIrr(:,[2,3]) = inp_TIrr(:,[4,2]);
d_TIrr(:,[2,3]) = inp_TIrr_d1(:,[4,2]);

% Right
for i2 = 2:1:InpPar.N_ConPatch
    TIrr(:,2*i2) = inp_TIrr(:,5);
    TIrr(:,2*i2+1) = inp_TIrr(:,3);
    d_TIrr(:,2*i2) = inp_TIrr_d1(:,5);
    d_TIrr(:,2*i2+1) = inp_TIrr_d1(:,3);
end

Choose_Plot = 1;
if Choose_Plot == 1
    figure(23); clf
    subplot(2,2,1)
    plot(TIrr(:,1), TIrr(:,2)); hold on
    plot(TIrr(:,1), TIrr(:,4)); hold on
%     plot(inp_TIrr_ori(1:end-n+0,1)+dX_TIrr, inp_TIrr_ori(n+1-0:end,5)*-1); hold on
    xlabel('Mileage (m)'); ylabel('TIrr-Z (m)')
    xlim([0,140]);
    
    subplot(2,2,3)
    plot(d_TIrr(:,1), d_TIrr(:,2)); hold on
    plot(d_TIrr(:,1), d_TIrr(:,4)); hold on
    xlabel('Mileage (m)'); ylabel('TIrr-Z-d1 (m/s)')
    xlim([0,140]);

    subplot(2,2,2)
    plot(TIrr(:,1), TIrr(:,3)); hold on
    plot(TIrr(:,1), TIrr(:,5)); hold on
    xlabel('Mileage (m)'); ylabel('TIrr-Y (m)')
    xlim([0,140]);
    
    subplot(2,2,4)
    plot(d_TIrr(:,1), d_TIrr(:,3)); hold on
    plot(d_TIrr(:,1), d_TIrr(:,5)); hold on
    xlabel('Mileage (m)'); ylabel('TIrr-Y-d1 (m/s)')
    xlim([0,140]);
    
    for kk = 1:1:4
        subplot(2,2,kk)
        grid on
        set(gca, 'ydir', 'reverse');
        set(gca, 'FontName', 'Times', 'FontSize', 11);
        xlim([X_TIrr_Simulation_Start-dX_TIrr_Spline, 130]);
    end
end
Range_X = [103, 108];
subplot(2,2,1)
xlim(Range_X)
subplot(2,2,3)
xlim(Range_X)


InpPar.TIrr = TIrr;
InpPar.d_TIrr = d_TIrr;

%% FZ: Test
temp = load('CitData_201107104010_GSDS_167.74_167.88_v2.mat');
k = 1;
k_Range = 1;
Result.Mileage_interp{k, k_Range} = (temp.mile_seg - 167.815) * -1000 + (54.315-32.880);
bools = (Result.Mileage_interp{k, k_Range}>=-10 & Result.Mileage_interp{k, k_Range}<=70);
Result.Mileage_interp{k, k_Range} = Result.Mileage_interp{k, k_Range}(bools);
Result.FZ.L{k,k_Range} = temp.vt_r_seg(bools);
Result.FY.L{k,k_Range} = temp.lt_r_seg(bools)*-1;
Result.FZ.R{k,k_Range} = temp.vt_l_seg(bools);
Result.FY.R{k,k_Range} = temp.lt_l_seg(bools)*-1;

temp = load('GSDS-Dalianbei-Shenyangbei-07112020-095710-1(280-18)_pic.mat');
s = (temp.ww_mile - 167.815) * -1000 + 50;
y = temp.ww_lacc;
Result.CB_Acc_Y = [s, y];

Choose_Plot = 0;
if Choose_Plot == 1
    figure(10); clf
    FontSize = 12;
    x = Result.Mileage_interp{k, k_Range};
    subplot(2,1,1);
    plot(x, Result.FZ.L{k,k_Range}, x, Result.FZ.R{k,k_Range}); hold on
    xlabel('Mileage (m)');
    ylabel('Ver. Force (kN)');
    grid on;
    set(gca, 'FontName', 'Times', 'FontSize', FontSize);
    subplot(2,1,2);
    plot(x, Result.FY.L{k,k_Range}, x, Result.FY.R{k,k_Range}); hold on
    xlabel('Mileage (m)');
    ylabel('Lat. Force (kN)');
    grid on;
    set(gca, 'FontName', 'Times', 'FontSize', FontSize);
end

InpPar.Test = Result;


% ========================= A. 导入数据
% 里程，速度，左垂力，左横力，右垂力，右横力，脱轨系数，减载率 
% Type_Range = {'ChangTuXi', 'SiPingDong', 'FuYuBei'};

% clear data
% fid = fopen('京哈8月份道岔区轮轨力.txt');
% data{1,1} = textscan(fid, '%f %f %f %f %f %f %f %f','HeaderLines',1,'Delimiter',',');
% fclose(fid);
% fid = fopen('京哈11月份道岔区轮轨力.txt');
% data{2,1} = textscan(fid, '%f %f %f %f %f %f %f %f','HeaderLines',1,'Delimiter',',');
% fclose(fid);
% fid = fopen('京哈8月份道岔区geo.txt');
% data{3,1} = textscan(fid, '%f %f %f %f %f %f','HeaderLines',1,'Delimiter',',');
% fclose(fid);
% 
% Choose_Plot = 0;
% if Choose_Plot == 1
%     figure(21); clf
%     plot(data{1, 1}{1, 1}, data{1, 1}{1, 2}); hold on
%     plot(data{2, 1}{1, 1}, data{2, 1}{1, 2}); hold on
% end
%   
% % ========================= B. 轮轨力时域TD和时频域SWT分析
% clear Result
% SIP = [830, 880, 1140];
% Range = [800, 850; 850, 900; 1100, 1150];
% % 四平东下行前半段
% % Range = [800, 850; 850, 883.5; 1100, 1150];
% % 四平东下行后半段
% % Range = [800, 850; 883.5, 900; 1100, 1150];
% 
% k_min = 1;
% k_max = 2;              % 8月WRForce / 11月WRForce / 8月Geo
% k_Range = 1;          % Range = [800, 850; 850, 900; 1100, 1150];
% k_turnout = 1;        % 直接选取第 k_turnout 组道岔数据
% % k_Range = 2;          % Range = [800, 850; 850, 900; 1100, 1150];
% % k_turnout = 6;        % 直接选取第 k_turnout 组道岔数据
% Type_Dir = {'L', 'R'};
% LineStyle = {'-','--'};
% Choose_PlotLatForce = 1;
% 
% % ChangTuXi-1#, 5#
% Mileage_Correct{1,1} =[-13.356, -12.721-0.05]+0.07;
% Mileage_Correct{2,1} =[ -12.694, -12.609-0.05]+0.07;
% % SiPingDong-1#, 7#, 9#, 10#, 6#, 4#
% Mileage_Correct{1,2} =[-89.368, 0, -104.353, 0, 0, -113.476];
% Mileage_Correct{2,2} =[-18.573, 0, -110.653, 0, 0, -111.936];
% % FuYuBei-1#, 5#
% Mileage_Correct{1,3} =[-9.311, -14.311] + 0.123;
% Mileage_Correct{2,3} =[-9.311-19, -14.311-13] + 0.123;
% 
% if k_Range==1
%     Mileage_turnout = load('道岔起止里程-昌图西-下行.txt');
% elseif k_Range==2
%     Mileage_turnout = load('道岔起止里程-四平东-下行.txt');
% elseif k_Range==3
%     Mileage_turnout = load('道岔起止里程-扶余北-下行.txt');
% end
%     
% dx = 10e-3;
% for k = k_min:1:k_max
%     % 仿真计算：直逆向过岔时，辙叉区轮轨垂向力最大值位置，距尖轨尖端距离为 53.148+1.129=54.277m    
%     % 仿真计算：直逆向过岔时，辙叉区轮轨垂向力最大值位置，距尖轨尖端前焊缝距离为 54.277+1.955=56.232m
%     % 仿真计算：用最新的翼轨廓形，辙叉区轮轨垂向力最大值位置从54.277m后移至54.400m，后移0.123m
%     if k_Range==0
%         bools_1 = true(length(data{k,1}{1}),1);
%     else
%         % 里程处理原则-2：对首尾两端里程线性插值
%         bools_0 = data{k,1}{1}>Range(k_Range,1) & data{k,1}{1}<Range(k_Range,2);
%         temp = data{k,1}{1}(bools_0,1);
%         if k_turnout==0
%             bools_1 = bools_0;
%             Mileage_interp_temp = ( linspace(temp(1), temp(end), length(temp)) )' + Mileage_Correct{k,k_Range}(1)*1e-3;
%             Result.Mileage_interp{k, k_Range} = (Mileage_interp_temp-SIP(k_Range))*1000;
%         else
%             % M1-里程插值针对车站全长范围
% %             temp = data{k,1}{1};
% %             temp(bools_0) = Mileage_interp_temp;
% %             bools_1 = temp > min(Mileage_turnout(k_turnout,1:2))-10e-3 & temp < max(Mileage_turnout(k_turnout,1:2))+10e-3;
% %             Result.Mileage_interp{k, k_Range} = (temp(bools_1,:)-Mileage_turnout(k_turnout,1))*1000-1.955;
%             % M2-里程插值只针对道岔全长范围
%             bools_1 = data{k,1}{1}+Mileage_Correct{k,k_Range}(k_turnout)*1e-3 > min(Mileage_turnout(k_turnout,1:2))-dx & ...
%                              data{k,1}{1}+Mileage_Correct{k,k_Range}(k_turnout)*1e-3 < max(Mileage_turnout(k_turnout,1:2))+dx;
%             temp = data{k,1}{1}(bools_1);
%             
%             if k_Range==2 && k_turnout == 6
%                 p = find(diff(temp)<0);
%                 if ~isempty(p)
%                     val_limit = temp(min(p));
%                     bools_del = (1:1:length(temp))'>min(p) & temp<=val_limit;
%                     temp(bools_del) = [];
%                     pos = find(bools_1);
%                     pos(bools_del) = [];
%                     bools_1 = false(length(data{k,1}{1}),1);
%                     bools_1(pos) = true;
% %                     figure(61); clf
% %                     plot(1:1:length(temp)-1, diff(temp)); grid on
% %                     plot(temp(1:end-1,1), diff(temp)); grid on
%                 end
%             end
%             Mileage_interp_temp = ( linspace(temp(1), temp(end), length(temp)) )'+Mileage_Correct{k,k_Range}(k_turnout)*1e-3;
%             Result.Mileage_interp{k, k_Range} = (Mileage_interp_temp-Mileage_turnout(k_turnout,1))*1000-1.955;
%         end
%     end
%         
%     %%% ========================= 检查里程重复情况
%     Choose_Plot = 0;
%     if Choose_Plot == 1
%         figure(1); clf;
%         temp = [Result.Mileage_interp{k,k_Range}(1:end-1) diff(Result.Mileage_interp{k,k_Range}(:,1))*1];
%         plot(temp(:,1),temp(:,2));
%         ylabel('[m]');  grid on
%     end
%     
%     %% B1 Time domain, Result
%     Result.Velocity{k,k_Range} = data{k,1}{2}(bools_1,:);
%     Result.FZ.L{k,k_Range} = data{k,1}{3}(bools_1,:);
%     Result.FY.L{k,k_Range} = data{k,1}{4}(bools_1,:);
%     Result.FZ.R{k,k_Range} = data{k,1}{5}(bools_1,:);
%     Result.FY.R{k,k_Range} = data{k,1}{6}(bools_1,:);
%     Result.Derailment{k,k_Range} = data{k,1}{7}(bools_1,:);
%     Result.LoadReduction{k,k_Range} = data{k,1}{8}(bools_1,:);    
%     
%     Choose_Plot = 0;
%     if Choose_Plot == 1
%         figure(10);
%         if k==k_min
%             clf
%         end
%         x = Result.Mileage_interp{k, k_Range};
%         for kk = 1:1:length(Type_Dir)
%             Dir = Type_Dir{kk};
%             subplot(2,1,kk); hold on
%             plot(x, Result.FZ.(Dir){k,k_Range}, LineStyle{k}); hold on
%             ylim([40, 100])
%             if Choose_PlotLatForce==1
%                 plot(x, Result.FY.(Dir){k,k_Range}, LineStyle{k}); hold on
%                 ylim([-20, 100])
%             end
%             xlabel('Mileage (m)');
%             ylabel('Ver, / Lat. Force (kN)');
%             grid on;
%             xlim([x(1), x(end)]);
%             title([Dir, ' Side']);
%             set(gca, 'FontName', 'Times', 'FontSize', FontSize);
%             if k == k_max
%                 Plot_Handle = gca;
%                 Plot_TurnoutBoundary(Mileage_turnout, SIP(k_Range), Plot_Handle, '--', '2D')
%                 temp = get(gca, 'Position');
%                 set(gca, 'Position', [0.0865, temp(2), 0.8782, temp(4)]);
%             end
%         end
%         set(gcf, 'Position', [51   491   821   480]);
%     end
% end

% InpPar.Test = Result;


