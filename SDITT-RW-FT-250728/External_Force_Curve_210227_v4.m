function [pos_Radius_Vehicle, pos_Body_global, pos_WS_BogieCS, pos_BG_CarbodyCS, vel_WS_BogieCS, vel_BG_CarbodyCS, ...
                 pos_yaw_track, vel_Body_global, vel_yaw_track, Pxt_test] = ...
                External_Force_Curve_210227_v4(InpPar, Par_Vehicle, j1, Par_Layout, pos_Radius_Vehicle, pos_Body_global, pos_WS_BogieCS, pos_BG_CarbodyCS, pos_yaw_track, Zsd, Pxt_test)

% 220717：为了考虑一系垂向阻尼、二系纵向及横向阻尼的非线性，相关力元的阻尼力在此不进行计算
            
% Pxt_test = zeros(InpPar.N_track+NM*InpPar.Nw+35,1);
% Pxt_test = Pxt;
% pos_Radius_Vehicle,7*5; 车辆刚体对应里程位置(xlcs)、曲率半径(xlcs)、线路摇头角(xlcs)、曲率半径(xlcs-1)、线路摇头角(xlcs-1)
% pos_Body_global,7*6; 车辆刚体 Pos_XYZ(xlcs)、车辆刚体 Pos_XYZ(xlcs-1)
% pos_yaw_track,7*2; 线路摇头角(xlcs)、线路摇头角(xlcs-1)
% pos_WS_BogieCS = zeros(4,6);
% pos_BG_CarbodyCS = zeros(2,6);

% R1, R2, L_trans, Curve_start, 

% global j1 Par_Layout Par_Vehicle InpPar.N_track InpPar.Nw Vlc 
% global InpPar.NM_FW

syms Alg_Vlc dt

Vlc = InpPar.Vlc;

%% 0. 确定各体对应的曲率半径
k1 = 1/Par_Layout.R1;
k2 = 1/Par_Layout.R2;
k = 1/Par_Layout.R;
vehicle_loc = [0; 2*Par_Vehicle.Ll1; 2*Par_Vehicle.Ll2; 2*(Par_Vehicle.Ll1+Par_Vehicle.Ll2); Par_Vehicle.Ll1; 2*Par_Vehicle.Ll2+Par_Vehicle.Ll1; Par_Vehicle.Ll1+Par_Vehicle.Ll2];
vel_Body_global = zeros(7,3);
vel_yaw_track = zeros(7,1);
pos_Radius_Vehicle(:,1) = j1 - vehicle_loc;

for i = 1:1:size(pos_Radius_Vehicle,1)
    
    % STR-1
    if pos_Radius_Vehicle(i,1) <= -Par_Layout.L_trans_1/2+Par_Layout.Curve_start_1
        pos_yaw_track(i,1) = 0;
        pos_Radius_Vehicle(i,2:3) = [Par_Layout.R1 pos_yaw_track(i,1)];
        pos_Body_global(i,1:3) = [pos_Radius_Vehicle(i,1), 0, 0];
        vel_Body_global(i,1:3) = [Vlc,0,0];
        vel_yaw_track(i,1) = 0;
        k_curvature_d1(i,1) = 0;
        
    % Trans-1a
    elseif pos_Radius_Vehicle(i,1) > -Par_Layout.L_trans_1/2+Par_Layout.Curve_start_1 && pos_Radius_Vehicle(i,1) <= Par_Layout.Curve_start_1
        ds = pos_Radius_Vehicle(i,1)-(-Par_Layout.L_trans_1/2+Par_Layout.Curve_start_1);
        k_curvature = double(subs(Par_Layout.fun_k_trans1a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        pos_yaw_track(i,1) = double(subs(Par_Layout.fun_yaw_trans1a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        pos_Body_global(i,1:3) = [double(subs(Par_Layout.fun_x_trans1a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ...
                                  double(subs(Par_Layout.fun_y_trans1a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
        pos_Radius_Vehicle(i,2:3) = [1/k_curvature, pos_yaw_track(i,1)];
        vel_Body_global(i,1:3) = [double(subs(Par_Layout.fun_x_trans1a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ...
                                  double(subs(Par_Layout.fun_y_trans1a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
        vel_yaw_track(i,1) = double(subs(Par_Layout.fun_yaw_trans1a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        k_curvature_d1(i,1) = double(subs(Par_Layout.fun_k_trans1a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        
    % Trans-1b
    elseif pos_Radius_Vehicle(i,1) > Par_Layout.Curve_start_1 && pos_Radius_Vehicle(i,1) <= Par_Layout.L_trans_1/2+Par_Layout.Curve_start_1
        ds = pos_Radius_Vehicle(i,1)-(-Par_Layout.L_trans_1/2+Par_Layout.Curve_start_1);
        k_curvature = double(subs(Par_Layout.fun_k_trans1b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        pos_yaw_track(i,1) = double(subs(Par_Layout.fun_yaw_trans1b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        pos_Body_global(i,1:3) = [double(subs(Par_Layout.fun_x_trans1b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ...
                                  double(subs(Par_Layout.fun_y_trans1b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
        pos_Radius_Vehicle(i,2:3) = [1/k_curvature, pos_yaw_track(i,1)];
        vel_Body_global(i,1:3) = [double(subs(Par_Layout.fun_x_trans1b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ...
                                  double(subs(Par_Layout.fun_y_trans1b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
        vel_yaw_track(i,1) = double(subs(Par_Layout.fun_yaw_trans1b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        k_curvature_d1(i,1) = double(subs(Par_Layout.fun_k_trans1b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        
    % Trans-2a
    elseif pos_Radius_Vehicle(i,1) > -Par_Layout.L_trans_2/2+Par_Layout.Curve_start_2 && pos_Radius_Vehicle(i,1) <= Par_Layout.Curve_start_2
        ds = pos_Radius_Vehicle(i,1)-(-Par_Layout.L_trans_2/2+Par_Layout.Curve_start_2);
        k_curvature = double(subs(Par_Layout.fun_k_trans2a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        pos_yaw_track(i,1) = double(subs(Par_Layout.fun_yaw_trans2a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        pos_Body_global(i,1:3) = [double(subs(Par_Layout.fun_x_trans2a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ...
                                  double(subs(Par_Layout.fun_y_trans2a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
        pos_Radius_Vehicle(i,2:3) = [1/k_curvature, pos_yaw_track(i,1)];
        vel_Body_global(i,1:3) = [double(subs(Par_Layout.fun_x_trans2a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ...
                                  double(subs(Par_Layout.fun_y_trans2a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
        vel_yaw_track(i,1) = double(subs(Par_Layout.fun_yaw_trans2a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        k_curvature_d1(i,1) = double(subs(Par_Layout.fun_k_trans2a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        
    % Trans-2b
    elseif pos_Radius_Vehicle(i,1) > Par_Layout.Curve_start_2 && pos_Radius_Vehicle(i,1) <= Par_Layout.L_trans_2/2+Par_Layout.Curve_start_2
        ds = pos_Radius_Vehicle(i,1)-(-Par_Layout.L_trans_2/2+Par_Layout.Curve_start_2);
        k_curvature = double(subs(Par_Layout.fun_k_trans2b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        pos_yaw_track(i,1) = double(subs(Par_Layout.fun_yaw_trans2b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        pos_Body_global(i,1:3) = [double(subs(Par_Layout.fun_x_trans2b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ...
                                  double(subs(Par_Layout.fun_y_trans2b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
        pos_Radius_Vehicle(i,2:3) = [1/k_curvature, pos_yaw_track(i,1)];
        vel_Body_global(i,1:3) = [double(subs(Par_Layout.fun_x_trans2b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ...
                                  double(subs(Par_Layout.fun_y_trans2b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
        vel_yaw_track(i,1) = double(subs(Par_Layout.fun_yaw_trans2b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        k_curvature_d1(i,1) = double(subs(Par_Layout.fun_k_trans2b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        
    % STR-2
    elseif pos_Radius_Vehicle(i,1) > Par_Layout.L_trans_2/2+Par_Layout.Curve_start_2 && pos_Radius_Vehicle(i,1) <= Par_Layout.Curve_start_3-Par_Layout.L_trans_3/2
        ds = pos_Radius_Vehicle(i,1)-(Par_Layout.L_trans_2/2+Par_Layout.Curve_start_2);
        k_curvature = double(subs(Par_Layout.fun_k_str(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        pos_yaw_track(i,1) = double(subs(Par_Layout.fun_yaw_str(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        pos_Body_global(i,1:3) = [double(subs(Par_Layout.fun_x_str(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ...
            double(subs(Par_Layout.fun_y_str(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
        pos_Radius_Vehicle(i,2:3) = [1/k_curvature, pos_yaw_track(i,1)];
        vel_Body_global(i,1:3) = [double(subs(Par_Layout.fun_x_str_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ...
            double(subs(Par_Layout.fun_y_str_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
        vel_yaw_track(i,1) = double(subs(Par_Layout.fun_yaw_str_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        k_curvature_d1(i,1) = double(subs(Par_Layout.fun_k_str_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        
    % Trans-3a
    elseif pos_Radius_Vehicle(i,1) > Par_Layout.Curve_start_3-Par_Layout.L_trans_3/2 && pos_Radius_Vehicle(i,1) <= Par_Layout.Curve_start_3
        ds = pos_Radius_Vehicle(i,1)-(Par_Layout.Curve_start_3-Par_Layout.L_trans_3/2);
        k_curvature = double(subs(Par_Layout.fun_k_trans3a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        pos_yaw_track(i,1) = double(subs(Par_Layout.fun_yaw_trans3a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        pos_Body_global(i,1:3) = [double(subs(Par_Layout.fun_x_trans3a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ...
            double(subs(Par_Layout.fun_y_trans3a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
        pos_Radius_Vehicle(i,2:3) = [1/k_curvature, pos_yaw_track(i,1)];
        vel_Body_global(i,1:3) = [double(subs(Par_Layout.fun_x_trans3a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ...
            double(subs(Par_Layout.fun_y_trans3a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
        vel_yaw_track(i,1) = double(subs(Par_Layout.fun_yaw_trans3a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        k_curvature_d1(i,1) = double(subs(Par_Layout.fun_k_trans3a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        
    % Trans-3b
    elseif pos_Radius_Vehicle(i,1) > Par_Layout.Curve_start_3 && pos_Radius_Vehicle(i,1) <= Par_Layout.Curve_start_3+Par_Layout.L_trans_3/2
        ds = pos_Radius_Vehicle(i,1)-(Par_Layout.Curve_start_3-Par_Layout.L_trans_3/2);
        k_curvature = double(subs(Par_Layout.fun_k_trans3b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        pos_yaw_track(i,1) = double(subs(Par_Layout.fun_yaw_trans3b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        pos_Body_global(i,1:3) = [double(subs(Par_Layout.fun_x_trans3b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ...
            double(subs(Par_Layout.fun_y_trans3b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
        pos_Radius_Vehicle(i,2:3) = [1/k_curvature, pos_yaw_track(i,1)];
        vel_Body_global(i,1:3) = [double(subs(Par_Layout.fun_x_trans3b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ...
            double(subs(Par_Layout.fun_y_trans3b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
        vel_yaw_track(i,1) = double(subs(Par_Layout.fun_yaw_trans3b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        k_curvature_d1(i,1) = double(subs(Par_Layout.fun_k_trans3b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        
    % Curve
    elseif pos_Radius_Vehicle(i,1) > Par_Layout.Curve_start_3+Par_Layout.L_trans_3/2
        ds = pos_Radius_Vehicle(i,1)-(Par_Layout.Curve_start_3+Par_Layout.L_trans_3/2);
        k_curvature = double(subs(Par_Layout.fun_k_curve(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        pos_yaw_track(i,1) = double(subs(Par_Layout.fun_yaw_curve(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        pos_Body_global(i,1:3) = [double(subs(Par_Layout.fun_x_curve(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ...
            double(subs(Par_Layout.fun_y_curve(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
        pos_Radius_Vehicle(i,2:3) = [1/k_curvature, pos_yaw_track(i,1)];
        vel_Body_global(i,1:3) = [double(subs(Par_Layout.fun_x_curve_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ...
            double(subs(Par_Layout.fun_y_curve_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
        vel_yaw_track(i,1) = double(subs(Par_Layout.fun_yaw_curve_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        k_curvature_d1(i,1) = double(subs(Par_Layout.fun_k_curve_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        
    end
    
end

%% 1. 离心力做功
for kk = 1:1:4
    Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(kk-1)+2,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(kk-1)+2,1)-Par_Vehicle.Mw*Vlc^2 / pos_Radius_Vehicle(kk,2);
end
% 前转向架
Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+2,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+2,1)-Par_Vehicle.Mb*Vlc^2 / pos_Radius_Vehicle(5,2);
% 后转向架
Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+25+2,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+25+2,1)-Par_Vehicle.Mb*Vlc^2 / pos_Radius_Vehicle(6,2);
% 车体
Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30+2,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30+2,1)-Par_Vehicle.Mc*Vlc^2 / pos_Radius_Vehicle(end,2);

%% 2. 一系横向悬挂
for i = 1:1:4
    if i <= 2
        num_bogie = 1;
        row_bogie = num_bogie+4;
    else
        num_bogie = 2;
        row_bogie = num_bogie+4;
    end
    T = [cos(pos_yaw_track(row_bogie,1))   -sin(pos_yaw_track(row_bogie,1))    0
            sin(pos_yaw_track(row_bogie,1))      cos(pos_yaw_track(row_bogie,1))    0
             0                                  0                                  1]; 
    T_d1_temp = [-sin(pos_yaw_track(row_bogie,1))   -cos(pos_yaw_track(row_bogie,1))    0
                                cos(pos_yaw_track(row_bogie,1))	 -sin(pos_yaw_track(row_bogie,1))     0
                                0                                  0                                  0]; 
    T_d1 = T_d1_temp * vel_yaw_track(row_bogie,1);
    pos_WS_BogieCS(i,1:3) = (pos_Body_global(i,1:3)-pos_Body_global(row_bogie,1:3)) * T;
    vel_WS_BogieCS(i,1:3) = (vel_Body_global(i,1:3)-vel_Body_global(row_bogie,1:3)) * T + ...
                            (pos_Body_global(i,1:3)-pos_Body_global(row_bogie,1:3)) * T_d1;
    F_PriSusp_Lat  = Par_Vehicle.K1y*pos_WS_BogieCS(i,2) + Par_Vehicle.C1y*vel_WS_BogieCS(i,2);
    
    Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i-1)+2,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i-1)+2,1) - 2*F_PriSusp_Lat;
    Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(num_bogie-1)+2,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(num_bogie-1)+2,1) + 2*F_PriSusp_Lat;
    Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(num_bogie-1)+3,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(num_bogie-1)+3,1) - 2*F_PriSusp_Lat*Par_Vehicle.H4;
    Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(num_bogie-1)+5,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(num_bogie-1)+5,1) + (-1)^(i-1)*2*F_PriSusp_Lat*Par_Vehicle.Ll1;    
end

%% 3. 二系横向悬挂（二系横向阻尼为非线性，此处阻尼为0）
for i = 1:1:2
    row_bogie = i + 4;
    row_carbody = 7;
    T = [cos(pos_yaw_track(row_carbody,1))	-sin(pos_yaw_track(row_carbody,1))    0
            sin(pos_yaw_track(row_carbody,1))	 cos(pos_yaw_track(row_carbody,1))    0
            0                                                                  0                                                                 1];
    T_d1_temp = [-sin(pos_yaw_track(row_carbody,1))   -cos(pos_yaw_track(row_carbody,1))  0
                                cos(pos_yaw_track(row_carbody,1))  -sin(pos_yaw_track(row_carbody,1))   0
                                0                                                                   0                                                               0];
    T_d1 = T_d1_temp * vel_yaw_track(row_carbody,1);
    pos_BG_CarbodyCS(i,1:3) = (pos_Body_global(row_bogie,1:3)-pos_Body_global(row_carbody,1:3)) * T;
    vel_BG_CarbodyCS(i,1:3) = (vel_Body_global(row_bogie,1:3)-vel_Body_global(row_carbody,1:3)) * T + ...
                                                     (pos_Body_global(row_bogie,1:3)-pos_Body_global(row_carbody,1:3)) * T_d1;
    F_SecSusp_Lat  = Par_Vehicle.K2y*pos_BG_CarbodyCS(i,2) + Par_Vehicle.C2y*vel_BG_CarbodyCS(i,2);
    
    Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(i-1)+2,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(i-1)+2,1) - 2*F_SecSusp_Lat;
    Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(i-1)+3,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(i-1)+3,1) - 2*F_SecSusp_Lat*Par_Vehicle.H3;
    Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30+2,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30+2,1) + 2*F_SecSusp_Lat;
    Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30+3,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30+3,1) - 2*F_SecSusp_Lat*Par_Vehicle.H2;
    Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30+5,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30+5,1) + (-1)^(i-1)*2*F_SecSusp_Lat*Par_Vehicle.Ll2;
end

%% 4. 一系纵向悬挂
for i = 1:1:4
    if i <= 2
        num_bogie = 1;
        row_bogie = num_bogie+4;
    else
        num_bogie = 2;
        row_bogie = num_bogie+4;
    end
    for j = 1:1:2
        F_PriSusp_Lon = (-1)^(j-1)* Par_Vehicle.K1x * Par_Vehicle.Lb1* (pos_yaw_track(row_bogie,1)-pos_yaw_track(i,1)) + ...
                                       (-1)^(j-1)* Par_Vehicle.C1x * Par_Vehicle.Lb1* (vel_yaw_track(row_bogie,1)-vel_yaw_track(i,1));
        Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i-1)+5) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i-1)+5) + Par_Vehicle.Lb1 * (-1)^(j-1) * F_PriSusp_Lon;
        Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(num_bogie-1)+5) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(num_bogie-1)+5) + Par_Vehicle.Lb1 * (-1)^j * F_PriSusp_Lon;
        Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(num_bogie-1)+4) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(num_bogie-1)+4) - Par_Vehicle.H4 * F_PriSusp_Lon;
    end
end

%% 5. 二系纵向悬挂（二系纵向阻尼为非线性，此处阻尼为0）
for i = 1:1:2
    num_bogie = i;
    row_bogie = i + 4;
    row_carbody = 7;
    for j = 1:1:2
        F_SecSusp_Lon = (-1)^(j-1)* Par_Vehicle.K2x * Par_Vehicle.Lb2* (pos_yaw_track(row_carbody,1)-pos_yaw_track(row_bogie,1)) + ...
                                        (-1)^(j-1)* Par_Vehicle.C2x * Par_Vehicle.Lb2* (vel_yaw_track(row_carbody,1)-vel_yaw_track(row_bogie,1));
        Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(num_bogie-1)+5) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(num_bogie-1)+5) + Par_Vehicle.Lb2 * (-1)^(j-1) * F_SecSusp_Lon;        
        Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(num_bogie-1)+4) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(num_bogie-1)+4) - Par_Vehicle.H3 * F_SecSusp_Lon;
        Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30+5) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30+5) + Par_Vehicle.Lb2 * (-1)^j * F_SecSusp_Lon; 
        Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30+4) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30+4) - Par_Vehicle.H2 * F_SecSusp_Lon;
    end
end

%% 6. 曲线上其他力对车辆系统做功
for i1 = 1:1:InpPar.Nw
    Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i1-1)+3,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i1-1)+3,1)+Par_Vehicle.Jwy*(Zsd(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i1-1)+4,4)-Vlc/R0)*(Zsd(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i1-1)+5,4)+Vlc/pos_Radius_Vehicle(i1,2));  % Eq. (2.124)
    Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i1-1)+5,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i1-1)+5,1)+Par_Vehicle.Jwy*(Zsd(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i1-1)+4,4)-Vlc/R0)* Zsd(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i1-1)+3,4);  % Eq. (2.125)
    Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i1-1)+5,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i1-1)+5,1)-Par_Vehicle.Jwz*Vlc*k_curvature_d1(i1,1);   % Eq. (2.125)
end
for i2 = 1:1:2
    Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(i2-1)+5,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(i2-1)+5,1)-Par_Vehicle.Jbz*Vlc*k_curvature_d1(4+i2,1);
end
Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+35,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+35,1)-Par_Vehicle.Jcz*Vlc*k_curvature_d1(end,1);

% for i1 = 1:1:InpPar.Nw
%     Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i1-1)+3,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i1-1)+3,1)+Par_Vehicle.Jwy*(Zsd(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i1-1)+4,3)-Par_Vehicle.Omiga)*(Zsd(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i1-1)+5,3)+Vlc/pos_Radius_Vehicle(i1,2));
%     Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i1-1)+5,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i1-1)+5,1)+Par_Vehicle.Jwy*(Zsd(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i1-1)+4,3)-Par_Vehicle.Omiga)* Zsd(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i1-1)+3,3);
%     Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i1-1)+5,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i1-1)+5,1)-Par_Vehicle.Jwz*Vlc*k_curvature_d1(i1,1);
% end
% for i2 = 1:1:2
%     Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(i2-1)+5,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(i2-1)+5,1)-Par_Vehicle.Jbz*Vlc*k_curvature_d1(4+i2,1);
% end
% Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+35,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+35,1)-Par_Vehicle.Jcz*Vlc*k_curvature_d1(end,1);
