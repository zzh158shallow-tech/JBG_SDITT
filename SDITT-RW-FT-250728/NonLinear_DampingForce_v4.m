function [Pxt_test, DamperNL] = NonLinear_DampingForce_v4(InpPar, Par_Vehicle, xlcs, Pxt_test, DamperNL)

% 220717：为了考虑一系垂向阻尼、二系纵向及横向阻尼的非线性，相关力元的阻尼力在此不进行计算
% NonLinear_DampingForce_v2: 对于串联弹簧，若将非线性阻尼力项全部置于外荷载向量中，则预平衡计算不收敛
% NonLinear_DampingForce_v3: 对于串联弹簧，阻尼矩阵中含阻尼项；若非线性阻尼力对应速度小于V0，则外荷载向量中无常数项；反之亦然
% NonLinear_DampingForce_v4: 补充横向减振器阻尼力 (串联弹簧), 横向止挡力

% Pxt_test = zeros(InpPar.N_track+NM*InpPar.Nw+35,1);
% Pxt_test = Pxt;
% pos_Radius_Vehicle,7*5; 车辆刚体对应里程位置(xlcs)、曲率半径(xlcs)、线路摇头角(xlcs)、曲率半径(xlcs-1)、线路摇头角(xlcs-1)
% pos_Body_global,7*6; 车辆刚体 Pos_XYZ(xlcs)、车辆刚体 Pos_XYZ(xlcs-1)
% pos_yaw_track,7*2; 线路摇头角(xlcs)、线路摇头角(xlcs-1)
% pos_WS_BogieCS = zeros(4,6);
% pos_BG_CarbodyCS = zeros(2,6);
% R1, R2, L_trans, Curve_start, 

% global Par_Vehicle InpPar.N_track InpPar.Nw InpPar.NM_FW Par_FW ModeShape xlcs

pos_DOF_RV = InpPar.N_track+InpPar.NM_FW*InpPar.Nw;

if xlcs<4
    tt = xlcs;
else
    tt = 4;
end

%% 1. 一系悬挂垂向非线性阻尼力 C_DPz

FZ_DPz_Damp = zeros(8,1);
bools = DamperNL.DPz.Mark;
FZ_DPz_Damp(bools,1) = sign(DamperNL.DPz.DampVel(bools))*(Par_Vehicle.C_DPz(1)-Par_Vehicle.C_DPz(2))*Par_Vehicle.C_DPz_V0;

pos_BG = [(pos_DOF_RV+20)*ones(4,1); (pos_DOF_RV+25)*ones(4,1)];
pos_Pz = pos_DOF_RV+35+(1:1:8)';
Range_i3 = repmat([1, 2]', 4, 1);
Range_i2 = [1, 1, 2, 2, 1, 1, 2, 2]';

Pxt_test(pos_BG+1,1) = Pxt_test(pos_BG+1,1) - FZ_DPz_Damp;
Pxt_test(pos_Pz,1) = Pxt_test(pos_Pz,1) + FZ_DPz_Damp;
Pxt_test(pos_BG+3,1) = Pxt_test(pos_BG+3,1) + (-1).^(Range_i3+1).*Par_Vehicle.Lb_DPz.*FZ_DPz_Damp;
Pxt_test(pos_BG+4,1) = Pxt_test(pos_BG+4,1) + (-1).^(Range_i2+1).*Par_Vehicle.Ll_DPzt.*FZ_DPz_Damp;

%% 2. 抗蛇行减振器纵向非线性阻尼力 C_Sx
FX_Sx_Damp = zeros(4,1);
bools = DamperNL.Sx.Mark;
FX_Sx_Damp(bools,1) = sign(DamperNL.Sx.DampVel(bools))*(Par_Vehicle.C_Sx(1)-Par_Vehicle.C_Sx(2))*Par_Vehicle.C_Sx_V0;

pos_CB = (pos_DOF_RV+30)*ones(4,1);
pos_Sn = pos_DOF_RV+35+8+(1:1:4)';
Range_i1 = [1, 1, 2, 2]';
Range_i3 = repmat([1, 2]', 2, 1);

Pxt_test(pos_Sn,1) = Pxt_test(pos_Sn,1) + FX_Sx_Damp;
Pxt_test(pos_CB+4) = Pxt_test(pos_CB+4) - Par_Vehicle.H_cS.*FX_Sx_Damp;
Pxt_test(pos_CB+5) = Pxt_test(pos_CB+5) + (-1).^(Range_i3) .*Par_Vehicle.Lb_S.*FX_Sx_Damp;

% % TEST
% FX_Sx_Damp = repmat(10,4,1);
% Pxt_test = zeros(150, 1);
% Pxt_test(pos_CB+5) = Pxt_test(pos_CB+5)+FX_Sx_Damp;

%% 3. 二系悬挂横向非线性阻尼力 C_DPy
% pos_BG = pos_DOF_RV+[20; 20; 25; 25];
% dy_SS(:,1) = Zsd(pos_BG+2,tt) - Zsd(pos_CB+2,tt) + Par_Vehicle.H_Bt.*Zsd(pos_BG+3,tt) + Par_Vehicle.H_cB.*Zsd(pos_CB+3,tt) + ...
%                        (-1).^Range_i1.*Par_Vehicle.Ll2.*Zsd(pos_CB+5,tt) + vel_BG_CarbodyCS(Range_i1,2);
% FY_DPy_Damp(:,1) = interp1(Par_Vehicle.C_DPy_Table(:,1), Par_Vehicle.C_DPy_Table(:,2), dy_SS(:,1), 'linear');
% 
% Pxt_test(pos_BG+2,1) = Pxt_test(pos_BG+2,1) - FY_DPy_Damp(:,1);
% Pxt_test(pos_CB+2,1) = Pxt_test(pos_CB+2,1) + FY_DPy_Damp(:,1);
% Pxt_test(pos_BG+3,1) = Pxt_test(pos_BG+3,1) - FY_DPy_Damp(:,1).*Par_Vehicle.H_Bt;
% Pxt_test(pos_CB+3,1) = Pxt_test(pos_CB+3,1) - FY_DPy_Damp(:,1).*Par_Vehicle.H_cB;
% Pxt_test(pos_CB+5,1) = Pxt_test(pos_CB+5,1) + (-1).^(Range_i1+1).*FY_DPy_Damp(:,1).*Par_Vehicle.Ll2;

FY_DPy_Damp = zeros(4,1);
bools = DamperNL.DPy.Mark;
FY_DPy_Damp(bools,1) = sign(DamperNL.DPy.DampVel(bools))*(Par_Vehicle.C_DPy(1)-Par_Vehicle.C_DPy(2))*Par_Vehicle.C_DPy_V0;

pos_Py = pos_DOF_RV+35+12+(1:1:4)';

Pxt_test(pos_CB+2,1) = Pxt_test(pos_CB+2,1) - FY_DPy_Damp(:,1);
Pxt_test(pos_Py,1) = Pxt_test(pos_Py,1) + FY_DPy_Damp(:,1);
Pxt_test(pos_CB+3,1) = Pxt_test(pos_CB+3,1) + Par_Vehicle.H_cDPy.*FY_DPy_Damp(:,1);
Pxt_test(pos_CB+5,1) = Pxt_test(pos_CB+5,1) - ((-1).^(Range_i1+1)).*Par_Vehicle.Ll_DPyc.*FY_DPy_Damp(:,1);

%% 4. 横向止挡力 K_STy_Table
FY_STy_Stiff = interp1(Par_Vehicle.K_STy_Table(:,1), Par_Vehicle.K_STy_Table(:,2), DamperNL.STy.SpringLength, 'linear');

Range_i1 = [1,2]';
pos_CB = (pos_DOF_RV+30)*ones(2,1);
pos_BG = pos_DOF_RV+20+5.*(Range_i1-1);

Pxt_test(pos_CB+2,1) = Pxt_test(pos_CB+2,1) - FY_STy_Stiff;
Pxt_test(pos_BG+2,1) = Pxt_test(pos_BG+2,1) + FY_STy_Stiff;
Pxt_test(pos_CB+3,1) = Pxt_test(pos_CB+3,1) + Par_Vehicle.H_cST.*FY_STy_Stiff;
Pxt_test(pos_BG+3,1) = Pxt_test(pos_BG+3,1) + Par_Vehicle.H_STt.*FY_STy_Stiff;
Pxt_test(pos_CB+5,1) = Pxt_test(pos_CB+5,1) - ((-1).^(Range_i1+1)).*Par_Vehicle.Ll_ST.*FY_STy_Stiff;

DamperNL.FZ_DPz_Damp = FZ_DPz_Damp;
DamperNL.FX_Sx_Damp = FX_Sx_Damp;
DamperNL.FY_DPy_Damp = FY_DPy_Damp;
DamperNL.FY_STy_Stiff = FY_STy_Stiff;

