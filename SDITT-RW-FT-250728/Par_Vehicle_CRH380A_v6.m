%% CRH380A车辆参数
% 230403修改，参考SIMPACK模型，补充一系钢弹簧悬挂垂向阻尼，补充一系垂向串联减振器参数
% 230404修改，参考CRH2-380.pdf的参数，修正一系悬挂垂向减振器参数 和 二系抗蛇形减振器参数
% v5: 一系垂向减振、二系横向减振的串联刚度/2，添加一系悬挂阻尼1e4
% 230619-v6修改: 将柔性轮对FEM自重作为Mw，而非刚性轮对自重，取消一系垂向减振、二系横向减振的串联刚度/2

function Par_Vehicle = Par_Vehicle_CRH380A_v6(InpPar)


%% 质量/转动惯量
Par_Vehicle.Mw = 1.9018e3;                   % 轮对质量     kg
% Par_Vehicle.Mw = 1010.39;                      % 轮对质量     kg
Par_Vehicle.Jwx = 685;                             % 轮对侧滚惯量 kg.m^2
Par_Vehicle.Jwy = 76;                               % 轮对点头惯量 kg.m^2
Par_Vehicle.Jwz = 685;                             % 轮对摇头惯量 kg.m^2

Par_Vehicle.Mb = 2280;                           % 构架质量     kg
Par_Vehicle.Jbx = 1847;                           % 构架侧滚惯量 kg.m^2
Par_Vehicle.Jby = 1249;                           % 构架点头惯量 kg.m^2
Par_Vehicle.Jbz = 2280;                           % 构架摇头惯量 kg.m^2

Par_Vehicle.Mc = 33.766e3+7.074e3;  % 车体质量     kg
% Par_Vehicle.Mc = 33.766e3+7.074e3+(1.9018e3-1010.39)*4;  % 车体质量     kg
Par_Vehicle.Jcx = 110.2e3;                      % 车体侧滚惯量 kg.m^2
Par_Vehicle.Jcy = 1666e3;                       % 车体点头惯量 kg.m^2
Par_Vehicle.Jcz = 1572.2e3;                    % 车体摇头惯量 kg.m^2

%% 轮轨参数
Par_Vehicle.R0 = 0.430;                                    % 车轮名义半径  m
Par_Vehicle.Omiga = InpPar.Vlc/Par_Vehicle.R0;     % 名义滚动角速度
Par_Vehicle.Drc = 0.07;                                     % 车轮名义半径测量点至轮背距离  m
Par_Vehicle.Dlb = 1.353/2;                               % 车轮轮背距之半  m

%% 车辆结构几何参数
% Ll
Par_Vehicle.Ll1 = 2.50/2;                 % 构架固定轴距之半  m
Par_Vehicle.Ll2 = 17.50/2;               % 车辆定距之半  m
Par_Vehicle.Ll_Jt = 0.75;                  % 轴箱转臂节点距转向架中心纵向距离 m
Par_Vehicle.Ll_Jw = 0.75-1.25;        % 轴箱转臂节点距轮对中心的纵向距离 m
Par_Vehicle.Ll_DPzt = 1.48;             % 一系垂向减振器距转向架中心的纵向距离 m
Par_Vehicle.Ll_DPzw = 1.48-1.25;   % 一系垂向减振器距轮对中心的纵向距离 m
Par_Vehicle.Ll_DPyt = 0.2;               % 二系横向减振器与转向架中心的纵向距离 m
% Par_Vehicle.Ll_DPyc = 0.2;               % 二系横向减振器与车体中心的纵向距离 m      !!! ??？
Par_Vehicle.Ll_ST = 17.50/2;            % 横向止挡距车体中心的纵向距离 m

% Lb
Par_Vehicle.Lb1 = 2.00/2;               % 一系悬挂横向距之半  m
Par_Vehicle.Lb2 = 2.46/2;               % 二系空气弹簧横向跨距之半  m
Par_Vehicle.Lb_J = 2.00/2;              % 轴箱转臂节点横向跨距之半  m
Par_Vehicle.Lb_DPz = 2.00/2;         % 一系垂向减振器横向跨距之半  m
Par_Vehicle.Lb_S = 2.70/2;             % 抗蛇行减振器横向跨距之半  m
Par_Vehicle.L0_ST = 0.02;               % 横向止挡自由间隙  m

% H
Par_Vehicle.H_tw = 0.51-0.43;           % 转向架质心与轮对中心线的垂向距离  m
Par_Vehicle.H_Bt = 0.8-0.51;              % 二系悬挂下平面至构架质心的垂向距离  m
Par_Vehicle.H_cB = 1.52-1.0;              % 车体质心至二系悬挂上平面的垂向距离  m
Par_Vehicle.H_tJ = 0.51-0.446;           % 构架质心至轴箱转臂节点的垂向距离  m
Par_Vehicle.H_Jw = 0.446-0.43;          % 轴箱转臂节点至轮对质心的垂向距离  m
Par_Vehicle.H_St = 0.415-0.51;           % 抗蛇行减振器与构架连接点至转向架质心的垂向距离  m
Par_Vehicle.H_cS = 1.52-0.415;           % 车体质心至抗蛇行减振器与车体连接点的垂向距离  m
Par_Vehicle.H_Tt = 0.38-0.51;              % 牵引拉杆与构架连接点至构架质心的垂向距离  m
Par_Vehicle.H_cT = 1.52-0.3975;         % 车体质心至牵引拉杆与车体连接点的垂向距离  m
Par_Vehicle.H_DPyt = 0.797-0.51;       % 二系横向减振器与构架连接点至构架质心的垂向距离  m
Par_Vehicle.H_cDPy = 1.52-0.785;       % 车体质心至二系横向减振器与车体连接点的垂向距离  m
Par_Vehicle.H_STt = 0.68-0.51;            % 横向止挡至构架质心的垂向距离  m
Par_Vehicle.H_cST = 1.52-0.68;            % 车体质心至横向止挡的垂向距离  m

%% 一系悬挂
Par_Vehicle.K1x = 980000;             % 一系纵向悬挂刚度（每轴箱）  N/m
Par_Vehicle.K1y = 980000;             % 一系横向悬挂刚度（每轴箱）  N/m
Par_Vehicle.K1z = 1176000;           % 一系垂向悬挂刚度（每轴箱）  N/m
Par_Vehicle.C1x = 0;                        % 一系纵向悬挂阻尼（每轴箱）  N.s/m
Par_Vehicle.C1y = 0;                        % 一系横向悬挂阻尼（每轴箱）  N.s/m
% Par_Vehicle.C1z = 0;                        % 一系垂向悬挂非线性阻尼（每轴箱）  N.s/m -- PVv6z
Par_Vehicle.C1z = 1e4;                    % 一系垂向悬挂非线性阻尼（每轴箱）  N.s/m__CRH2A SIMPACK 模型

Par_Vehicle.K_Jx = 13720000;         % 轴箱转臂节点纵向刚度（每轴箱）  N/m
Par_Vehicle.K_Jy = 5520000;           % 轴箱转臂节点横向刚度（每轴箱）  N/m

Par_Vehicle.K_DPz = 4.9e6;               % 一系垂向减振器接头刚度（每轴箱）  N/m___v5
% Par_Vehicle.K_DPz = 4.9e6/2;               % 一系垂向减振器接头刚度（每轴箱）  N/m___v5b
Par_Vehicle.C_DPz(1,1) = 19.6e3;     % 一系垂向减振器阻尼（每轴箱）  N.s/m（Non-Linear）
Par_Vehicle.C_DPz(2,1) = 14.7e3;     % 一系垂向减振器阻尼（每轴箱）  N.s/m（Non-Linear）
Par_Vehicle.C_DPz_V0 =   0.1;            % 一系垂向减振器阻尼（每轴箱）  N.s/m（Non-Linear）

% Non-linear primary suspension damping - 230404
Par_Vehicle.C_DPz_Table = ...
    [-1000,     -(0.1*19.6e3+(1000-0.1)*14.7e3)	
    -0.1,    -0.1*19.6e3
      0,        0
      0.1,     0.1*19.6e3
      1000,        0.1*19.6e3+(1000-0.1)*14.7e3];

%% 二系悬挂
Par_Vehicle.K2x = 168900;               % 二系纵向悬挂刚度（转向架一侧）  N/m
Par_Vehicle.K2y = 168900;               % 二系横向悬挂刚度（转向架一侧）  N/m
Par_Vehicle.K2z = 260000;                % 二系垂向悬挂刚度（转向架一侧）  N/m
Par_Vehicle.C2x = 0;                           % 二系纵向悬挂阻尼（转向架一侧）  N.s/m
Par_Vehicle.C2y = 0;                           % 二系横向悬挂阻尼（转向架一侧）  N.s/m（Non-Linear）
Par_Vehicle.C2z = 40000;                  % 二系垂向悬挂阻尼（转向架一侧）  N.s/m

Par_Vehicle.K_Sx = 8.82e6;               % 抗蛇行减振器节点刚度（两端串联后）  N/m
Par_Vehicle.C_Sx(1,1) = 2.4525e6;   % 抗蛇行减振器阻尼（每个）  N.s/m（Non-Linear）
Par_Vehicle.C_Sx(2,1) = 49.95e3;     % 抗蛇行减振器阻尼（每个）  N.s/m（Non-Linear）
Par_Vehicle.C_Sx_V0 =   0.004;         % 抗蛇行减振器临界速度（每个）  N.s/m（Non-Linear）

Par_Vehicle.K_Tx = 7882e3;              % 牵引拉杆纵向刚度  N/m
Par_Vehicle.K_ROTx = 1e6;                % 抗侧滚扭杆角刚度 N.m/rad

Par_Vehicle.K_DPy = 17150e3;         % 二系横向减振器节点刚度  N/m___v5
% Par_Vehicle.K_DPy = 17150e3/2;         % 二系横向减振器节点刚度  N/m___v5b
Par_Vehicle.C_DPy(1,1) = 58.8e3;     % 二系横向减振器阻尼  N.s/m（Non-Linear）
Par_Vehicle.C_DPy(2,1) = 24.6e3;     % 二系横向减振器阻尼  N.s/m（Non-Linear）
Par_Vehicle.C_DPy_V0 =  0.1;            % 二系横向减振器临界速度  N.s/m（Non-Linear）

% Lateral Stop
Par_Vehicle.K_STy_Table(:,1) = [0, 0.02, 0.026, 0.032, 0.038, 0.044, 0.048];
Par_Vehicle.K_STy_Table(:,2) = [0, 0, 2e3, 5e3, 11e3, 25.5e3, 49e3];
Par_Vehicle.K_STy_Table = [Par_Vehicle.K_STy_Table; Par_Vehicle.K_STy_Table(2:end,:)*-1];
Par_Vehicle.K_STy_Table = sortrows(Par_Vehicle.K_STy_Table, 1);

% 230404
Par_Vehicle.C_Sx_Table = ...
    [-1,        -(0.004*2.4525e6+(1-0.004)*49.949e3)
    -0.004,   -0.004*2.4525e6
      0,            0
      0.004,    0.004*2.4525e6
      1,            0.004*2.4525e6+(1-0.004)*49.949e3];

Par_Vehicle.C_DPy_Table =  ...
    [-1,     -(0.1*58.8e3+(1-0.1)*24.6e3)	
    -0.1,    -0.1*58.8e3
      0,        0
      0.1,     0.1*58.8e3
      1,        0.1*58.8e3+(1-0.1)*24.6e3];

% Zhai
% Par_Vehicle.C2y_Table = ...
%     [-1	-(58.8e3*0.15+6.10e3*(1-0.15))
%     -0.15	-58.8e3*0.15
%     0	0
%     0.15	58.8e3*0.15
%     1	58.8e3*0.15+6.10e3*(1-0.15)];

% % 220920
% Par_Vehicle.C2x_Table = ...
%     [-1	-(4.9e6*0.015+0.01e6*(1-0.015))
%     -0.015	-4.9e6*0.015
%     0	0
%     0.015	4.9e6*0.015
%     1	4.9e6*0.015+0.01e6*(1-0.015)];
% 
% Par_Vehicle.C2y_Table = ...
%     [-1	-(58.8e3*0.15+6.10e3*(1-0.15))
%     -0.15	-58.8e3*0.15
%     0	0
%     0.15	58.8e3*0.15
%     1	58.8e3*0.15+6.10e3*(1-0.15)];

