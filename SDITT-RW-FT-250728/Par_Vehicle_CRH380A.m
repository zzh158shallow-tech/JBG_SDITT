%% CRH380A车辆参数
function Par_Vehicle = Par_Vehicle_CRH380A(InpPar)

% global  InpPar.Vlc

% 质量/转动惯量
Par_Vehicle.Mw = 1850;                   % 轮对质量     kg
Par_Vehicle.Jwx = 967;                   % 轮对侧滚惯量 kg.m^2
Par_Vehicle.Jwy = 123;                   % 轮对点头惯量 kg.m^2
Par_Vehicle.Jwz = 967;                   % 轮对摇头惯量 kg.m^2
Par_Vehicle.Mb = 2400;                   % 构架质量     kg
Par_Vehicle.Jbx = 1944;                  % 构架侧滚惯量 kg.m^2
Par_Vehicle.Jby = 1314;                  % 构架点头惯量 kg.m^2
Par_Vehicle.Jbz = 2400;                  % 构架摇头惯量 kg.m^2
Par_Vehicle.Mc = 43862.5;                  % 车体质量     kg
Par_Vehicle.Jcx = 1.094e5;               % 车体侧滚惯量 kg.m^2
Par_Vehicle.Jcy = 1.654e6;               % 车体点头惯量 kg.m^2
Par_Vehicle.Jcz = 1.561e6;               % 车体摇头惯量 kg.m^2

% 车辆基本参数
Par_Vehicle.R0 = 0.430;                 % 车轮名义半径  m
Par_Vehicle.Omiga = InpPar.Vlc/Par_Vehicle.R0; % 名义滚动角速度
Par_Vehicle.Drc = 0.07;                 % 车轮名义半径测量点至轮背距离  m
Par_Vehicle.Dlb = 1.353/2;              % 车轮轮背距之半  m
Par_Vehicle.Lb1 = 2.00/2;               % 一系悬挂横向距之半  m
Par_Vehicle.Lb2 = 2.46/2;               % 二系悬挂横向距之半  m
Par_Vehicle.Ll1 = 2.50/2;               % 构架固定轴距之半  m
Par_Vehicle.Ll2 = 17.50/2;              % 车辆定距之半  m
Par_Vehicle.H2 = 0.54;                  % 车体质心至二系悬挂点上作用点的高度  m
Par_Vehicle.H3 = 0.29;                  % 二系悬挂点下作用点至构架质心的高度  m
Par_Vehicle.H4 = 0.08;                  % 构架质心至一系悬挂点的高度  m

Par_Vehicle.Hs_b = 0.68-0.48;           % 横向止挡至构架质心高差绝对值
Par_Vehicle.Hs_c = 1.51-0.68;           % 横向止挡至车体质心高差绝对值

% 一系悬挂
Par_Vehicle.K1x = 14.680e6;             % 一系纵向悬挂刚度（每轴箱）  N/m
Par_Vehicle.K1y = 6.470e6;               % 一系横向悬挂刚度（每轴箱）  N/m
Par_Vehicle.K1z = 1.176e6;               % 一系垂向悬挂刚度（每轴箱）  N/m

Par_Vehicle.C1x = 0;                           % 一系纵向悬挂阻尼（每轴箱）  N.s/m
Par_Vehicle.C1y = 0;                           % 一系横向悬挂阻尼（每轴箱）  N.s/m
Par_Vehicle.C1z = 0;                           % 一系垂向悬挂非线性阻尼（每轴箱）  N.s/m（Non-Linear）
% Non-linear primary suspension damping - 220920
Par_Vehicle.C1z_Table = ...
    [-1,	-(0.01*13e3+(1-0.01)*6.5e3)
     -0.01,	-0.01*13e3
       0,	0
       0.01,	0.01*13e3
       1,	0.01*13e3+(1-0.01)*6.5e3];

% syms x
% fun(x) = 13e3 + (6.5-13)*1e3/(0.15-0.01)*(x-0.01);
% Par_Vehicle.C1z_Table(:,1) = [0, 0.01/2, 0.01:0.0005:0.15, 0.2, 1];
% y1 = 13e3*0.01;
% y2 = int(fun(x), x, 0.01, 0.15)+y1;
% for i = 1:1:length(Par_Vehicle.C1z_Table)
%     if Par_Vehicle.C1z_Table(i,1)<0.01
%         Par_Vehicle.C1z_Table(i,2) = Par_Vehicle.C1z_Table(i,1)*13e3;
%     elseif Par_Vehicle.C1z_Table(i,1)>=0.01 && Par_Vehicle.C1z_Table(i,1)<0.15
%         Par_Vehicle.C1z_Table(i,2) =  int(fun(x), x, 0.01, Par_Vehicle.C1z_Table(i,1)) + y1;
%     else
%         Par_Vehicle.C1z_Table(i,2) = (Par_Vehicle.C1z_Table(i,1)-0.15)*6.5e3 + y2;
%     end
% end
% Par_Vehicle.C1z_Table = [sortrows(Par_Vehicle.C1z_Table(2:end,:), -1)*-1; Par_Vehicle.C1z_Table];
        
% figure(2); clf
% plot(Par_Vehicle.C1z_Table(:,1), Par_Vehicle.C1z_Table(:,2)); hold on
% plot(Par_Vehicle.C1z_Table(:,1), Par_Vehicle.C1z_Table(:,1)*6e3); hold on
% grid on    

% for i1 = 1:1:2          % Bogie
%     for i2 = 1:1:2      % Wheelset
%         for i3 = 1:1:2  % Left / Right
%             pos = 4*(i1-1)+2*(i2-1)+i3;
%             pos_BG = N_track+20+5*(i1-1);
%             pos_WS = N_track+5*(2*(i1-1)+i2-1);            
%             dz_ps(pos,1) = Zsd(pos_BG+1,4) - Zsd(pos_WS+1,4) + (-1)^i2*Par_Vehicle.Ll1*Zsd(pos_BG+4,4) + ...
%                                         (-1)^(i3+1)*Par_Vehicle.Lb1*Zsd(pos_WS+3,4) + (-1)^(i3)*Par_Vehicle.Lb1*Zsd(pos_BG+3,4); 
%         end
%     end
% end
% Par_Vehicle.C1z = interp1(Par_Vehicle.C1z_Table(:,1), Par_Vehicle.C1z_Table(:,2), abs(dz_ps), 'linear');
   
% 二系悬挂
Par_Vehicle.K2x = 0.160e6;               %二系纵向悬挂刚度（转向架一侧）  N/m
Par_Vehicle.K2y = 0.160e6;               %二系横向悬挂刚度（转向架一侧）  N/m
Par_Vehicle.K2z = 0.190e6;                %二系垂向悬挂刚度（转向架一侧）  N/m

% for i1 = 1:1:2          % Bogie
%     for i3 = 1:1:2      % Left / Right
%         pos = 2*(i1-1)+i3;
%         pos_CB = N_track+30;
%         pos_BG = N_track+20+5*(i1-1);
%         row_bogie = i1 + 4;
%         row_carbody = 7;
%         
%         dx_ss(pos,1) = Par_Vehicle.H2*Zsd(pos_CB+4,4) + Par_Vehicle.H3*Zsd(pos_BG+4,4) + ...
%                                    (-1)^(i3+1)*Par_Vehicle.Lb2*Zsd(pos_CB+5,4) + (-1)^(i3)*Par_Vehicle.Lb2*Zsd(pos_BG+5,4) + ...
%                                    (-1)^(i3+1)* Par_Vehicle.Lb2* (vel_yaw_track(row_carbody,1)-vel_yaw_track(row_bogie,1));
%                                
% %                                (-1)^(j-1) * Par_Vehicle.C2x * Par_Vehicle.Lb2* (vel_yaw_track(row_carbody,1)-vel_yaw_track(row_bogie,1));
%         
%         dy_ss(pos,1) = Zsd(pos_BG+2,4) - Zsd(pos_CB+2,4) + Par_Vehicle.H3*Zsd(pos_BG+3,4) + Par_Vehicle.H2*Zsd(pos_CB+3,4) + ...
%                                    (-1)^(i1)*Par_Vehicle.Ll2*Zsd(pos_CB+5,4) + vel_BG_CarbodyCS(i1,2);     
%                                
% %         dz_ss(pos,1) = Zsd(pos_CB+1,4) - Zsd(pos_BG+1,4) + (-1)^i1*Par_Vehicle.Ll2*Zsd(pos_CB+4,4) + ...
% %                                     (-1)^(i3+1)*Par_Vehicle.Lb2*Zsd(pos_BG+3,4) + (-1)^(i3)*Par_Vehicle.Lb2*Zsd(pos_CB+3,4);
%     end
% end

% Par_Vehicle.C2x =zeros(4,1);
% bools_xx = abs(dx_ss)<0.015;                   % 二系纵向悬挂阻尼（转向架一侧）  N.s/m
% Par_Vehicle.C2x(bools_xx) = 4.9e6;
% Par_Vehicle.C2x(~bools_xx) = 0.01e6;
% 
% Par_Vehicle.C2y =zeros(4,1);
% bools_yy = abs(dy_ss)<0.15;                     % 二系横向悬挂阻尼（转向架一侧）  N.s/m
% Par_Vehicle.C2y(bools_yy) = 58.8e3;
% Par_Vehicle.C2y(~bools_yy) = 6.10e3;

Par_Vehicle.C2x = 0;                                % 二系纵向悬挂阻尼（转向架一侧）  N.s/m（Non-Linear）
Par_Vehicle.C2y = 0;                                % 二系横向悬挂阻尼（转向架一侧）  N.s/m（Non-Linear）
Par_Vehicle.C2z = 40e3;                          % 二系垂向悬挂阻尼（转向架一侧）  N.s/m

% Par_Vehicle.C2x_Table = ...
% [-100, -19600
% -0.2, -19600
% -0.004, -9810
% 0, 0
% 0.004, 9810
% 0.2, 19600
% 100, 19600];
% 
% Par_Vehicle.C2y_Table = ...
% [-100, -10800
% -0.3, -10800
% -0.1, -5.88E+03
% 0, 0
% 0.1, 5.88E+03
% 0.3, 10800
% 100, 10800];

% 220920
Par_Vehicle.C2x_Table = ...
[-1	-(4.9e6*0.015+0.01e6*(1-0.015))
-0.015	-4.9e6*0.015
0	0
0.015	4.9e6*0.015
1	4.9e6*0.015+0.01e6*(1-0.015)];

Par_Vehicle.C2y_Table = ...
[-1	-(58.8e3*0.15+6.10e3*(1-0.15))
-0.15	-58.8e3*0.15
0	0
0.15	58.8e3*0.15
1	58.8e3*0.15+6.10e3*(1-0.15)];

Par_Vehicle.K2b = 0;                                   % 二系抗弯刚度    （转向架一侧）  Nm/rad
Par_Vehicle.Kr = 0;                                      % 抗侧滚扭杆刚度


% if strcmp(Type_Vehicle, 'HSR')
%     % 质量/转动惯量
%     Par_Vehicle.Mw = 1400;                   % 轮对质量     kg
%     Par_Vehicle.Jwx = 915;                   % 轮对侧滚惯量 kg.m^2
%     Par_Vehicle.Jwy = 140;                   % 轮对点头惯量 kg.m^2
%     Par_Vehicle.Jwz = 915;                   % 轮对摇头惯量 kg.m^2
%     Par_Vehicle.Mb = 3000;                   % 构架质量     kg
%     Par_Vehicle.Jbx = 2260;                  % 构架侧滚惯量 kg.m^2
%     Par_Vehicle.Jby = 2710;                  % 构架点头惯量 kg.m^2
%     Par_Vehicle.Jbz = 3160;                  % 构架摇头惯量 kg.m^2
% %     Par_Vehicle.Mc = 34000;                  % 车体质量     kg
%     Par_Vehicle.Mc = 34000+2100*4;                  % 车体质量     kg
%     Par_Vehicle.Jcx = 7.506e4;               % 车体侧滚惯量 kg.m^2
%     Par_Vehicle.Jcy = 2.277e6;               % 车体点头惯量 kg.m^2
%     Par_Vehicle.Jcz = 2.086e6;               % 车体摇头惯量 kg.m^2    
%     
%     % 一系悬挂
%     Par_Vehicle.K1z = 0.55e6;               %一系垂向悬挂刚度（每轴箱）  N/m
%     Par_Vehicle.C1z = 6e3;                  %一系垂向悬挂阻尼（每轴箱）  N.s/m
%     Par_Vehicle.K1y = 5e6;                  %一系横向悬挂刚度（每轴箱）  N/m
%     Par_Vehicle.C1y = 0;                    %一系横向悬挂阻尼（每轴箱）  N.s/m
%     Par_Vehicle.K1x = 10e6;                 %一系纵向悬挂刚度（每轴箱）  N/m
%     Par_Vehicle.C1x = 0;                    %一系纵向悬挂阻尼（每轴箱）  N.s/m
%     
%     % 二系悬挂
%     Par_Vehicle.K2z = 0.4e6;                %二系垂向悬挂刚度（转向架一侧）  N/m
%     Par_Vehicle.C2z = 80e3;                 %二系垂向悬挂阻尼（转向架一侧）  N.s/m
%     Par_Vehicle.K2b = 0;                    %二系抗弯刚度    （转向架一侧）  Nm/rad
%     Par_Vehicle.K2y = 0.15e6;               %二系横向悬挂刚度（转向架一侧）  N/m
%     Par_Vehicle.C2y = 60e3;                 %二系横向悬挂阻尼（转向架一侧）  N.s/m
%     Par_Vehicle.K2x = 0.15e6;               %二系纵向悬挂刚度（转向架一侧）  N/m
%     Par_Vehicle.C2x = 0;                    %二系纵向悬挂阻尼（转向架一侧）  N.s/m    
%     Par_Vehicle.Kr = 0e3;                   %抗侧滚扭杆刚度
%     
%     % 车辆基本参数
%     Par_Vehicle.R0 = 0.430;                 % 车轮名义半径  m
%     Par_Vehicle.Omiga = InpPar.Vlc/Par_Vehicle.R0; % 名义滚动角速度
%     Par_Vehicle.Drc = 0.07;                 % 车轮名义半径测量点至轮背距离  m
%     Par_Vehicle.Dlb = 1.353/2;              % 车轮轮背距之半  m
%     Par_Vehicle.Lb1 = 2.00/2;               % 一系悬挂横向距之半  m
%     Par_Vehicle.Lb2 = 2.46/2;               % 二系悬挂横向距之半  m
%     Par_Vehicle.Ll1 = 2.50/2;               % 构架固定轴距之半  m
%     Par_Vehicle.Ll2 = 17.50/2;              % 车辆定距之半  m
%     % Par_Vehicle.H2 = 1.8-1.13;            % 车体质心至二系悬挂点上作用点的高度  m
%     % Par_Vehicle.H3 = -(0.6-0.525);        % 二系悬挂点下作用点至构架质心的高度  m
%     % % Par_Vehicle.H3 = 0.6-0.525;         % 构架质心至二系悬挂点的高度  m
%     % Par_Vehicle.H4 = 0.6-0.46;            % 构架质心至一系悬挂点的高度  m
%     Par_Vehicle.H2 = 1.51 - (1+0.8)/2;      % 车体质心至二系悬挂点上作用点的高度  m
%     Par_Vehicle.H3 = (1+0.8)/2 - 0.48;      % 二系悬挂点下作用点至构架质心的高度  m
%     Par_Vehicle.H4 = 0.48 - 0.43;           % 构架质心至一系悬挂点的高度  m
%     Par_Vehicle.Hs_b = 0.68-0.48;           % 横向止挡至构架质心高差绝对值
%     Par_Vehicle.Hs_c = 1.51-0.68;           % 横向止挡至车体质心高差绝对值
%     
%     % 横向止挡
%     Par_Vehicle.Bump_stop = [-45e-3	-40e-3	-35e-3	-30e-3	-25e-3	-20e-3	0	20e-3	25e-3	30e-3	35e-3	40e-3   45e-3
%                              -30096	-12603	-5596	-2107	-696	0		0	0		696     2107    5596    12603   30096];
%     Par_Vehicle.Bump_stop = Par_Vehicle.Bump_stop';
% 
% elseif strcmp(Type_Vehicle, 'Manchester Benchmark')
%     
%     Par_Vehicle.Mw = 1813;                   % 轮对质量  kg
%     % Par_Vehicle.Mw = 1476.17;                % 柔性轮对有限元模型质量 kg
%     Par_Vehicle.Jwx = 1120;                  % 轮对侧滚惯量  kg.m^2
%     Par_Vehicle.Jwy = 112;                   % 轮对点头惯量  kg.m^2
%     Par_Vehicle.Jwz = 1120;                  % 轮对摇头惯量  kg.m^2
%     Par_Vehicle.Mb = 2615;                   % 构架质量  kg
%     Par_Vehicle.Jbx = 1722;                  % 构架侧滚惯量  kg.m^2
%     Par_Vehicle.Jby = 1476;                  % 构架点头惯量  kg.m^2
%     Par_Vehicle.Jbz = 3067;                  % 构架摇头惯量  kg.m^2
%     Par_Vehicle.Mc = 32000;                  % 车体质量  kg
%     Par_Vehicle.Jcx = 56800;                 % 车体侧滚惯量  kg.m^2
%     Par_Vehicle.Jcy = 1970000;               % 车体点头惯量  kg.m^2
%     Par_Vehicle.Jcz = 1970000;               % 车体摇头惯量  kg.m^2
%     
%     Par_Vehicle.K1z = 1220e+3;              %一系垂向悬挂刚度（每轴箱）  N/m
%     Par_Vehicle.C1z = 4e3;                  %一系垂向悬挂阻尼（每轴箱）  N.s/m
%     Par_Vehicle.K1y = 3884e+3;              %一系横向悬挂刚度（每轴箱）  N/m
%     Par_Vehicle.C1y = 2e3;                  %一系横向悬挂阻尼（每轴箱）  N.s/m
%     Par_Vehicle.K1x = 31391e+3;             %一系纵向悬挂刚度（每轴箱）  N/m
%     Par_Vehicle.C1x = 15e3;                 %一系纵向悬挂阻尼（每轴箱）  N.s/m
%     
%     % 并联
%     Par_Vehicle.K2z = 430e+3;               %二系垂向悬挂刚度（转向架一侧）  N/m
%     Par_Vehicle.C2z = 20e+3;                %二系垂向悬挂阻尼（转向架一侧）  N.s/m
%     Par_Vehicle.K2b = 10.5e3;               %二系抗弯刚度    （转向架一侧）  Nm/rad
%     Par_Vehicle.K2y = 1*160e+3;             %二系横向悬挂刚度（转向架一侧）  N/m
%     Par_Vehicle.C2y = 32e+3;                %二系横向悬挂阻尼（转向架一侧）  N.s/m
%     Par_Vehicle.K2x = 160e+3;               %二系纵向悬挂刚度（转向架一侧）  N/m
%     Par_Vehicle.C2x = 0e+6;                 %二系纵向悬挂阻尼（转向架一侧）  N.s/m
%     
%     Par_Vehicle.Kr = 940e3;                 %抗侧滚扭杆刚度
%     
%     Par_Vehicle.R0 = 0.460;                 % 车轮名义半径  m
%     Par_Vehicle.Omiga = InpPar.Vlc/Par_Vehicle.R0; % 名义滚动角速度
%     Par_Vehicle.Drc = 0.07;                 % 车轮名义半径测量点至轮背距离  m
%     Par_Vehicle.Dlb = 1.360/2;              % 车轮轮背距之半  m
%     Par_Vehicle.Lb1 = 1.0;                  % 一系悬挂横向距之半  m
%     Par_Vehicle.Lb2 = 1.0;                  % 二系悬挂横向距之半  m
%     Par_Vehicle.Ll1 = 1.28;                 % 构架固定轴距之半  m
%     Par_Vehicle.Ll2 = 9.50;                 % 车辆定距之半  m
%     % Par_Vehicle.H2 = 1.8-1.13;              % 车体质心至二系悬挂点上作用点的高度  m
%     % Par_Vehicle.H3 = -(0.6-0.525);          % 二系悬挂点下作用点至构架质心的高度  m
%     % % Par_Vehicle.H3 = 0.6-0.525;            % 构架质心至二系悬挂点的高度  m
%     % Par_Vehicle.H4 = 0.6-0.46;              % 构架质心至一系悬挂点的高度  m
%     Par_Vehicle.H2 = 1.8 - 0.8275;            % 车体质心至二系悬挂点上作用点的高度  m
%     Par_Vehicle.H3 = 0.8275 - 0.6;            % 二系悬挂点下作用点至构架质心的高度  m
%     Par_Vehicle.H4 = 0.6 - 0.46;              % 构架质心至一系悬挂点的高度  m
%     Par_Vehicle.Hs_b = 0.65-0.6;            % 横向止挡至车体质心abs距离
%     Par_Vehicle.Hs_c = 1.8-0.65;            % 横向止挡至构架质心abs距离
%     
%     %%% 横向止挡
%     Par_Vehicle.Bump_stop = [-65e-3	-60e-3	-55e-3	-50e-3	-45e-3	-40e-3	-35e-3	-30e-3	-25e-3	0	25e-3	30e-3	35e-3	40e-3	45e-3	50e-3	55e-3	60e-3	65e-3
%                              -230000	-29200	-17170	-11580	-6870	-3730	-1760	-600	0	0	0	600	1760	3730	6870	11580	17170	29200	230000];
%     Par_Vehicle.Bump_stop = Par_Vehicle.Bump_stop';
%     
% end