%% 考虑轮对弹性，计算初始状态下轮轨最小垂向间隙，两侧间隙应该对称
function [d0, Dis_min_L, Dis_min_R] = Cal_d0_FW(InpPar, Par_Vehicle, Par_Track, Par_FW, WheelPro_ProCS, Zwy, RailPro_ProCS)

% clc
% clear

% global InpPar.N_track InpPar.NM_FW InpPar.Nw InpPar.ModeShape InpPar.Pos_Node
% global Par_Track Par_Vehicle Par_FW 
% global WheelPro_ProCS

R0 = Par_Vehicle.R0;
Dlb = Par_Vehicle.Dlb;
Drc = Par_Vehicle.Drc;
Ori_prr = Par_Track.Ori_prr;

if strcmp(InpPar.VehicleDir, 'Face')
    i11 = 1;
elseif strcmp(InpPar.VehicleDir, 'Trail')
    i11 = 4;
end
Zw = Zwy(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+(5*(i11-1)+1),4);    %%% 轮对垂向位移
Yw = Zwy(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+(5*(i11-1)+2),4);    %%% 轮对横向位移
Yaw = Zwy(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+(5*(i11-1)+5),4);   %%% 轮对摇头角
Roll = Zwy(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+(5*(i11-1)+3),4);  %%% 轮对侧滚角

profile_w_R = WheelPro_ProCS.profile_w_R;
profile_w_L = WheelPro_ProCS.profile_w_L;
Con_ang_R = WheelPro_ProCS.Con_ang_R;
Con_ang_L = WheelPro_ProCS.Con_ang_L;
profile_w_R_Radius = WheelPro_ProCS.profile_w_R_Radius;
profile_w_L_Radius = WheelPro_ProCS.profile_w_L_Radius;

%%% 定义转换矩阵
syms Inp_Yaw Inp_Roll Inp_Ang
T_WS_Track = [ cos(Inp_Yaw)                          sin(Inp_Yaw)                           0;
                           -cos(Inp_Roll)*sin(Inp_Yaw)  cos(Inp_Roll)*cos(Inp_Yaw) sin(Inp_Roll);
                             sin(Inp_Roll)*sin(Inp_Yaw) -sin(Inp_Roll)*cos(Inp_Yaw)  cos(Inp_Roll)];
Choose_Plot = 0;

%% 1. 导入截面
if strcmp(InpPar.VehicleDir, 'Face')
    profile_r_R = RailPro_ProCS.R1.FF_Profile;
elseif strcmp(InpPar.VehicleDir, 'Trail')
    profile_r_R = RailPro_ProCS.R3.FF_Profile;
end
profile_r_L = RailPro_ProCS.L1.FF_Profile;

%
% profile_r_L = sortrows(profile_r_L,-1);
% profile_r_R(:,2)-profile_r_L(:,2)

profile_r_R(:,1) = profile_r_R(:,1)+1.435/2+Ori_prr;
profile_r_R(:,2) = profile_r_R(:,2)+0.6;
profile_r_R = sortrows(profile_r_R,1);

profile_r_L(:,1) = -(profile_r_L(:,1)+1.435/2+Ori_prr);
profile_r_L(:,2) = profile_r_L(:,2)+0.6;
profile_r_L = sortrows(profile_r_L,1);

%% 2. 空间接触几何计算
A_WS = double(subs(T_WS_Track, [Inp_Yaw, Inp_Roll], [Yaw, Roll]));
discrete_len_tread = 2e-5;
discrete_len_flange = 0.5e-5;
%%% 2.1 计算半轮对刚体空间位置
if InpPar.NM_FW > 0    
    % a.考虑轮对弹性，修正侧滚角, Pos_FW
    clear Pos_FW
    pos_NM_FW = InpPar.N_track+InpPar.NM_FW*(i11-1)+(1:1:InpPar.NM_FW);
    Type_Node = {'NRC', 'WBack', 'WOut'};
    for i1 = 1:1:length(Type_Node)
        T1 = Type_Node{i1};
        for i2 = 1:1:2
            T2 = InpPar.Type_Side{i2};
            T = [T1, '_', T2];
            DOF_pos = Par_FW.DOF_pos.(T);
            pos_ori = InpPar.Pos_Node.(T1).(T2);
            Pos_FW.(T) = [0 Yw Zw] + (pos_ori+(InpPar.ModeShape.FW(DOF_pos,:)*Zwy(pos_NM_FW,4))') * A_WS;
        end
    end
         
    % b.考虑轮对弹性，修正侧滚角和摇头角
    Roll_DW.R = (Pos_FW.WOut_R(3)-Pos_FW.WBack_R(3))/(Pos_FW.WOut_R(2)-Pos_FW.WBack_R(2)) ...
                         -(InpPar.Pos_Node.WOut.R(3)-InpPar.Pos_Node.WBack.R(3))/(InpPar.Pos_Node.WOut.R(2)-InpPar.Pos_Node.WBack.R(2));
                     
    Roll_DW.L = (Pos_FW.WBack_L(3)-Pos_FW.WOut_L(3))/(Pos_FW.WBack_L(2)-Pos_FW.WOut_L(2)) ...
                         -(InpPar.Pos_Node.WBack.L(3)-InpPar.Pos_Node.WOut.L(3))/(InpPar.Pos_Node.WBack.L(2)-InpPar.Pos_Node.WOut.L(2));
                     
    Yaw_DW.R = -(Pos_FW.WOut_R(1)-Pos_FW.WBack_R(1))/(Pos_FW.WOut_R(2)-Pos_FW.WBack_R(2));    
    Yaw_DW.L = -(Pos_FW.WBack_L(1)-Pos_FW.WOut_L(1))/(Pos_FW.WBack_L(2)-Pos_FW.WOut_L(2));
    
    % c.确定DWR和DWL半轮对轮轴中心的位置
    for i1 = 1:1:2
        T1 = InpPar.Type_Side{i1};
        A_DW.(T1) = [ cos(Yaw_DW.(T1))                                    sin(Yaw_DW.(T1))                                     0;
                                 -cos(Roll_DW.(T1))*sin(Yaw_DW.(T1))   cos(Roll_DW.(T1))*cos(Yaw_DW.(T1))  sin(Roll_DW.(T1));
                                   sin(Roll_DW.(T1))*sin(Yaw_DW.(T1))  -sin(Roll_DW.(T1))*cos(Yaw_DW.(T1))   cos(Roll_DW.(T1))];
        ADL1_ODL1 = [0, (Dlb+Drc)*(-1+2*sign(i1-1)), R0]*A_DW.(T1);
        O1_ODL1 = Pos_FW.(['NRC_', T1]) - ADL1_ODL1;
        Yw_DW.(T1) = O1_ODL1(2);
        Zw_DW.(T1) = O1_ODL1(3);
    end
    
    Roll_DWR = Roll_DW.R;        Roll_DWL = Roll_DW.L;
    Yaw_DWR = Yaw_DW.R;        Yaw_DWL = Yaw_DW.L;
    Yw_DWR = Yw_DW.R;            Yw_DWL = Yw_DW.L;
    Zw_DWR = Zw_DW.R;            Zw_DWL = Zw_DW.L;
%     A_DWR = A_DW.R;         A_DWL = A_DW.L;
    
else    
    Roll_DWR = Roll;        Roll_DWL = Roll;
    Yaw_DWR = Yaw;        Yaw_DWL = Yaw;
    Yw_DWR = Yw;            Yw_DWL = Yw;
    Zw_DWR = Zw;            Zw_DWL = Zw;
%     A_DWR = A_WS;         A_DWL = A_WS;    
end

%%% 2.2 计算空间迹线
[Con_ang_R_temp, traceline_w_R, Con_ang_L_temp, traceline_w_L] = TracePrinciple(WheelPro_ProCS, Dlb, discrete_len_flange, discrete_len_tread,...
 profile_w_R, Roll_DWR, Yaw_DWR, Yw_DWR, Zw_DWR, profile_w_L, Roll_DWL, Yaw_DWL, Yw_DWL, Zw_DWL);

%%% 2.3 计算轮轨垂向间隙
temp = 0.15e-3;
% 左侧
bools_L = traceline_w_L(:,2) > max(min(traceline_w_L(:,2),min(profile_r_L(:,1))))+temp & ...
                   traceline_w_L(:,2) < min(max(traceline_w_L(:,2),max(profile_r_L(:,1))))-temp;
range_y_L = traceline_w_L(bools_L,2)';
wheel_interp_L = traceline_w_L(bools_L,:);
rail_interp_L = [range_y_L' interp1(profile_r_L(:,1),profile_r_L(:,2),range_y_L,'spline')'];
bools_linear = (rail_interp_L(:,2)>0.6+8e-3);
range_y_L_lienar = (range_y_L(bools_linear))';
rail_interp_L(bools_linear,:) = [range_y_L_lienar interp1(profile_r_L(:,1),profile_r_L(:,2),range_y_L_lienar,'linear')];
rail_interp_L = sortrows(rail_interp_L,1);
Ver_Dis_L = [range_y_L' rail_interp_L(:,2)-wheel_interp_L(:,3)];

% 右侧
bools_R = (traceline_w_R(:,2) > max(min(traceline_w_R(:,2),min(profile_r_R(:,1))))+temp &...
                    traceline_w_R(:,2) < min(max(traceline_w_R(:,2),max(profile_r_R(:,1))))-temp);
range_y_R = traceline_w_R(bools_R,2)';
wheel_interp_R = traceline_w_R(bools_R,:);
rail_interp_R = [range_y_R' interp1(profile_r_R(:,1),profile_r_R(:,2),range_y_R,'spline')'];
bools_linear = (rail_interp_R(:,2)>0.6+8e-3);
range_y_R_lienar = (range_y_R(bools_linear))';
rail_interp_R(bools_linear,:) = [range_y_R_lienar interp1(profile_r_R(:,1),profile_r_R(:,2),range_y_R_lienar,'linear')];
rail_interp_R = sortrows(rail_interp_R,1);
Ver_Dis_R = [range_y_R' rail_interp_R(:,2)-wheel_interp_R(:,3)];

[~,p] = min(Ver_Dis_L(:,2));
[~,q] = min(Ver_Dis_R(:,2));

Dis_min_L = Ver_Dis_L(p,2);
Dis_min_R = Ver_Dis_R(q,2);
d0 = 1/2*(Dis_min_L + Dis_min_R);

%%% 2.4 绘图对比
if Choose_Plot == 1
    % 迹线与钢轨廓形接触状态
    figure(20); clf;
    subplot(1,2,1); plot(wheel_interp_L(:,2), wheel_interp_L(:,3)+min(Ver_Dis_L(:,2)), rail_interp_L(:,1), rail_interp_L(:,2)); set(gca,'ydir','reverse'); grid on
    subplot(1,2,2); plot(wheel_interp_R(:,2), wheel_interp_R(:,3)+min(Ver_Dis_R(:,2)), rail_interp_R(:,1), rail_interp_R(:,2)); set(gca,'ydir','reverse'); grid on

    % 拟合后、拟合前两侧轮轨垂向间隙
    figure(21); clf;
    subplot(1,2,1); plot(Ver_Dis_L(:,1),Ver_Dis_L(:,2),Ver_Dis_L_ori(:,1),Ver_Dis_L_ori(:,2),'--'); grid on; title('Ver-Dis-L');
    subplot(1,2,2); plot(Ver_Dis_R(:,1),Ver_Dis_R(:,2),Ver_Dis_R_ori(:,1),Ver_Dis_R_ori(:,2),'--'); grid on; title('Ver-Dis-R');
end
