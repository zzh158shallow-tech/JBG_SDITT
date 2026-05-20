%% CRH380A车辆参数
% 230403修改，参考SIMPACK模型，补充一系钢弹簧悬挂垂向阻尼，补充一系垂向串联减振器参数
function Par_Vehicle = Par_Vehicle_CRH380A_v2(InpPar)

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
% Par_Vehicle.Mc = 43862.5;                  % 车体质量     kg
Par_Vehicle.Mc = 43862.5-5e3*8/9.81;                  % 车体质量     kg
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
Par_Vehicle.C1z = 10000;                  % 一系垂向悬挂非线性阻尼（每轴箱）  N.s/m（Non-Linear）

Par_Vehicle.K1z_dp = 2450000;               % 一系垂向减振器悬挂刚度（每轴箱）  N/m
Par_Vehicle.C1z_dp = 20000;                    % 一系垂向减振器悬挂阻尼（每轴箱）  N.s/m


% Non-linear primary suspension damping - 220920
% Par_Vehicle.C1z_Table = ...
%     [-1,	-(0.01*13e3+(1-0.01)*6.5e3)
%     -0.01,	-0.01*13e3
%     0,	0
%     0.01,	0.01*13e3
%     1,	0.01*13e3+(1-0.01)*6.5e3];

   
% 二系悬挂
Par_Vehicle.K2x = 0.160e6;               %二系纵向悬挂刚度（转向架一侧）  N/m
Par_Vehicle.K2y = 0.160e6;               %二系横向悬挂刚度（转向架一侧）  N/m
Par_Vehicle.K2z = 0.190e6;                %二系垂向悬挂刚度（转向架一侧）  N/m

Par_Vehicle.C2x = 0;                                % 二系纵向悬挂阻尼（转向架一侧）  N.s/m（Non-Linear）
Par_Vehicle.C2y = 0;                                % 二系横向悬挂阻尼（转向架一侧）  N.s/m（Non-Linear）
Par_Vehicle.C2z = 40e3;                          % 二系垂向悬挂阻尼（转向架一侧）  N.s/m

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
