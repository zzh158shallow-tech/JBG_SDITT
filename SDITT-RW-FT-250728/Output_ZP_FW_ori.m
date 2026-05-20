%% 结果输出函数
function Output_ZP_FW(Con_FF, Con_FR, Con_RF, Con_RR, Zwy, Pjcc, Pjch, Prhxf, Dis_Rail, Vel_Rail, Acc_Rail, Profile_ori_L1, Profile_ori_R1, Profile_ori_R2, Profile_ori_R3)

global Nw N_ConPatch xlcs Par_Track NM_FW N_track Expression_WS Expression_DummyRail
global ZP_Yw ZP_Mileage ZP_Vzd ZP_WR_Geometry_FW
global ZP_Con_Rail_1_L ZP_Con_Wheel_1_L ZP_Con_Wheel_2_L ZP_Normal_Force_L ZP_Prhx_L ZP_Prhxf_L ZP_a2_L ZP_b2_L ZP_RHXS_L ZP_RHLv_L
global ZP_Con_Rail_1_R ZP_Con_Wheel_1_R ZP_Con_Wheel_2_R ZP_Normal_Force_R ZP_Prhx_R ZP_Prhxf_R ZP_a2_R ZP_b2_R ZP_RHXS_R ZP_RHLv_R
global ZP_FX_FF_L1 ZP_FX_FF_R1 ZP_FX_FF_R2 ZP_FX_FF_R3 ZP_FY_FF_L1 ZP_FY_FF_R1 ZP_FY_FF_R2 ZP_FY_FF_R3 ZP_FZ_FF_L1 ZP_FZ_FF_R1 ZP_FZ_FF_R2 ZP_FZ_FF_R3
% global ZP_FX_FR_L1 ZP_FX_FR_R1 ZP_FX_FR_R2 ZP_FX_FR_R3 ZP_FY_FR_L1 ZP_FY_FR_R1 ZP_FY_FR_R2 ZP_FY_FR_R3 ZP_FZ_FR_L1 ZP_FZ_FR_R1 ZP_FZ_FR_R2 ZP_FZ_FR_R3
% global ZP_FX_RF_L1 ZP_FX_RF_R1 ZP_FX_RF_R2 ZP_FX_RF_R3 ZP_FY_RF_L1 ZP_FY_RF_R1 ZP_FY_RF_R2 ZP_FY_RF_R3 ZP_FZ_RF_L1 ZP_FZ_RF_R1 ZP_FZ_RF_R2 ZP_FZ_RF_R3
% global ZP_FX_RR_L1 ZP_FX_RR_R1 ZP_FX_RR_R2 ZP_FX_RR_R3 ZP_FY_RR_L1 ZP_FY_RR_R1 ZP_FY_RR_R2 ZP_FY_RR_R3 ZP_FZ_RR_L1 ZP_FZ_RR_R1 ZP_FZ_RR_R2 ZP_FZ_RR_R3
global ZP_Dis_Rail ZP_Vel_Rail ZP_Acc_Rail
global ZP_Con_Rail_L1 ZP_Con_Area_L1 ZP_Con_Angle_L1 ZP_Con_CX_L1 ZP_Con_CY_L1 ZP_Con_CSpin_L1 ZP_Con_CFX_L1 ZP_Con_CFY_L1 ZP_Con_CFSpin_L1 ZP_Con_NF_L1
global ZP_Con_Rail_L2 ZP_Con_Area_L2 ZP_Con_Angle_L2 ZP_Con_CX_L2 ZP_Con_CY_L2 ZP_Con_CSpin_L2 ZP_Con_CFX_L2 ZP_Con_CFY_L2 ZP_Con_CFSpin_L2 ZP_Con_NF_L2
global ZP_Con_Rail_R1 ZP_Con_Area_R1 ZP_Con_Angle_R1 ZP_Con_CX_R1 ZP_Con_CY_R1 ZP_Con_CSpin_R1 ZP_Con_CFX_R1 ZP_Con_CFY_R1 ZP_Con_CFSpin_R1 ZP_Con_NF_R1
global ZP_Con_Rail_R2 ZP_Con_Area_R2 ZP_Con_Angle_R2 ZP_Con_CX_R2 ZP_Con_CY_R2 ZP_Con_CSpin_R2 ZP_Con_CFX_R2 ZP_Con_CFY_R2 ZP_Con_CFSpin_R2 ZP_Con_NF_R2
global ZP_Con_Rail_R3 ZP_Con_Area_R3 ZP_Con_Angle_R3 ZP_Con_CX_R3 ZP_Con_CY_R3 ZP_Con_CSpin_R3 ZP_Con_CFX_R3 ZP_Con_CFY_R3 ZP_Con_CFSpin_R3 ZP_Con_NF_R3
global ZP_Con_Vjd_L1  ZP_Con_Vsdc_L1  ZP_Con_Vjsdc_L1  ZP_Con_Vgd_L1 ZP_Con_R_yy_w_L1 ZP_Con_R_xx_w_L1 ZP_Con_R_xx_r_L1 ZP_Con_rou_L1
global ZP_Con_Vjd_L2 ZP_Con_Vsdc_L2 ZP_Con_Vjsdc_L2 ZP_Con_Vgd_L2 ZP_Con_R_yy_w_L2 ZP_Con_R_xx_w_L2 ZP_Con_R_xx_r_L2 ZP_Con_rou_L2
global ZP_Con_Vjd_R1 ZP_Con_Vsdc_R1 ZP_Con_Vjsdc_R1 ZP_Con_Vgd_R1 ZP_Con_R_yy_w_R1 ZP_Con_R_xx_w_R1 ZP_Con_R_xx_r_R1 ZP_Con_rou_R1
global ZP_Con_Vjd_R2  ZP_Con_Vsdc_R2  ZP_Con_Vjsdc_R2  ZP_Con_Vgd_R2 ZP_Con_R_yy_w_R2 ZP_Con_R_xx_w_R2 ZP_Con_R_xx_r_R2 ZP_Con_rou_R2
global ZP_Con_Vjd_R3  ZP_Con_Vsdc_R3  ZP_Con_Vjsdc_R3  ZP_Con_Vgd_R3 ZP_Con_R_yy_w_R3 ZP_Con_R_xx_w_R3 ZP_Con_R_xx_r_R3 ZP_Con_rou_R3

Ori_prr = Par_Track.Ori_prr;

%% ======所有轮对
for i1 = 1:1:Nw
    eval(['Con_str = Con_',Expression_WS{i1},';']);
    ZP_WR_Geometry_FW{i1,1}(1+xlcs, :) = [Con_str.Mileage Con_str.WR_Geometry];
    for k = 1:1:size(Con_str.Normal_Force_L,1)
        ZP_Con_Rail_1_L{i1,k}(1+xlcs,:) = [Con_str.Mileage Con_str.Con_rail_L_1(k,:)];
        ZP_Con_Wheel_1_L{i1,k}(1+xlcs,:) = [Con_str.Mileage Con_str.Con_wheel_L_1(k,:)];
        ZP_Con_Wheel_2_L{i1,k}(1+xlcs,:) = [Con_str.Mileage Con_str.Con_wheel_L_2(k,:) Con_str.elastic_permeability_Unit_L(k)];
        ZP_Normal_Force_L{i1,k}(1+xlcs,:) = [Con_str.Mileage Con_str.Normal_Force_L(k,:)];
        ZP_Prhx_L{i1,k}(1+xlcs,:) = [Con_str.Mileage Con_str.Prhx_L(k,:)];
        ZP_Prhxf_L{i1,k}(1+xlcs,:) = [Con_str.Mileage Con_str.Prhxf_L(k,:)];
        ZP_a2_L{i1,k}(1+xlcs,:) = [Con_str.Mileage Con_str.a2_L(k,:)];
        ZP_b2_L{i1,k}(1+xlcs,:) = [Con_str.Mileage Con_str.b2_L(k,:)];
        ZP_RHXS_L{i1,k}(1+xlcs,:) = [Con_str.Mileage Con_str.RHXS_L(k,:)];
        ZP_RHLv_L{i1,k}(1+xlcs,:) = [Con_str.Mileage Con_str.RHLv_L(k,:)];
    end
    for k = 1:1:size(Con_str.Normal_Force_R,1)
        ZP_Con_Rail_1_R{i1,k}(1+xlcs,:) = [Con_str.Mileage Con_str.Con_rail_R_1(k,:)];
        ZP_Con_Wheel_1_R{i1,k}(1+xlcs,:) = [Con_str.Mileage Con_str.Con_wheel_R_1(k,:)];
        ZP_Con_Wheel_2_R{i1,k}(1+xlcs,:) = [Con_str.Mileage Con_str.Con_wheel_R_2(k,:) Con_str.elastic_permeability_Unit_R(k)];
        ZP_Normal_Force_R{i1,k}(1+xlcs,:) = [Con_str.Mileage Con_str.Normal_Force_R(k,:)];
        ZP_Prhx_R{i1,k}(1+xlcs,:) = [Con_str.Mileage Con_str.Prhx_R(k,:)];
        ZP_Prhxf_R{i1,k}(1+xlcs,:) = [Con_str.Mileage Con_str.Prhxf_R(k,:)];
        ZP_a2_R{i1,k}(1+xlcs,:) = [Con_str.Mileage Con_str.a2_R(k,:)];
        ZP_b2_R{i1,k}(1+xlcs,:) = [Con_str.Mileage Con_str.b2_R(k,:)];
        ZP_RHXS_R{i1,k}(1+xlcs,:) = [Con_str.Mileage Con_str.RHXS_R(k,:)];
        ZP_RHLv_R{i1,k}(1+xlcs,:) = [Con_str.Mileage Con_str.RHLv_R(k,:)];
    end
end

%% ====== 轨下位移、速度和加速度,纵向、横向、垂向轮轨力
for i1 = 1:1:Nw
    for i2 = 1:1:N_ConPatch
        i_contact = N_ConPatch*(i1-1)+i2;
        eval(['ZP_Dis_Rail.', Expression_WS{i1}, '_', Expression_DummyRail{i2}, '(1+xlcs,:) = [Con_', Expression_WS{i1}, '.Mileage Dis_Rail(i_contact,:)];']);
        eval(['ZP_Vel_Rail.', Expression_WS{i1}, '_', Expression_DummyRail{i2}, '(1+xlcs,:) = [Con_', Expression_WS{i1}, '.Mileage Vel_Rail(i_contact,:)];']);
        eval(['ZP_Acc_Rail.', Expression_WS{i1}, '_', Expression_DummyRail{i2}, '(1+xlcs,:) = [Con_', Expression_WS{i1}, '.Mileage Acc_Rail(i_contact,:)];']);
    end
end

for i1 = 1:1:1
    for i2 = 1:1:N_ConPatch
        i_contact = N_ConPatch*(i1-1)+i2;
        eval(['ZP_FX_', Expression_WS{i1}, '_', Expression_DummyRail{i2}, '(1+xlcs,:) = [Con_', Expression_WS{i1}, '.Mileage, -Prhxf(i_contact,1)];']);
        eval(['ZP_FY_', Expression_WS{i1}, '_', Expression_DummyRail{i2}, '(1+xlcs,:) = [Con_', Expression_WS{i1}, '.Mileage, -Prhxf(i_contact,2)-Pjch(i_contact,1)];']);
        eval(['ZP_FZ_', Expression_WS{i1}, '_', Expression_DummyRail{i2}, '(1+xlcs,:) = [Con_', Expression_WS{i1}, '.Mileage, -Prhxf(i_contact,3)-Pjcc(i_contact,1)];']);
%         ZP_FX_FF_L1(1+xlcs,:) = [Con_FF.Mileage, -Prhxf(i_contact,1)];
%         ZP_FY_FF_L1(1+xlcs,:) = [Con_FF.Mileage, -Prhxf(i_contact,2)-Pjch(i_contact,1)];
%         ZP_FZ_FF_L1(1+xlcs,:) = [Con_FF.Mileage, -Prhxf(i_contact,3)-Pjcc(i_contact,1)];
    end
end

%% ======导向轮对
i1 = 1;
eval(['Con_str = Con_',Expression_WS{i1},';']);
ZP_Yw(1+xlcs,:) = [Con_FF.Mileage, Zwy(N_track+NM_FW*Nw+5*(i1-1)+2,4)];
ZP_Mileage(1+xlcs,:) = {Con_FF.Mileage  Profile_ori_L1.FF_Profile_num Profile_ori_R1.FF_Profile_num Profile_ori_R2.FF_Profile_num Profile_ori_R3.FF_Profile_num};
% ZP_Mileage(1+xlcs,:) = [Con_FF.Mileage  Profile_ori_L1.FF_Profile_num Profile_ori_R1.FF_Profile_num Profile_ori_R2.FF_Profile_num Profile_ori_R3.FF_Profile_num];
ZP_Vzd(1+xlcs,:)     = [Con_FF.Mileage  Con_FF.Vzdli, Con_FF.Vzdlj, Con_FF.Vzdlk];

%% 导向轮对左侧接触信息
t_L1 = 1;
for k = 1:1:size(Con_str.Normal_Force_L,1)
    i2 =  Con_str.Normal_Force_L(k,4);    
    eval(['ZP_Con_Rail_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, (Con_FF.Con_rail_L_1(k,1)+0.7175+Ori_prr-Dis_Rail(4*(i1-1)+i2,2))*1e3, (Con_FF.Con_rail_L_1(k,2)-0.6)*1e3];']);
    eval(['ZP_Con_Area_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, (pi*Con_FF.a2_L(k,:)*Con_FF.b2_L(k,:))*1e6];']);
    eval(['ZP_Con_Angle_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Con_wheel_L_2(k,6)];']);
    eval(['ZP_Con_CX_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.RHLv_L(k,1)];']);
    eval(['ZP_Con_CY_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.RHLv_L(k,2)];']);
    eval(['ZP_Con_CSpin_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.RHLv_L(k,3)];']);
    eval(['ZP_Con_CFX_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Prhx_L(k,1)];']);
    eval(['ZP_Con_CFY_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Prhx_L(k,2)];']);
    eval(['ZP_Con_CFSpin_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Prhx_L(k,3)];']);
    eval(['ZP_Con_NF_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Normal_Force_L(k,:)];']);
    eval(['ZP_Con_Vjd_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Vjd_L(k,:)];']);
    eval(['ZP_Con_Vsdc_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Vsdc_L(k,:)];']);
    eval(['ZP_Con_Vjsdc_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Vjsdc_L(k,:)];']);
    eval(['ZP_Con_Vgd_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Vgd_L(k,:)];']);
    eval(['ZP_Con_R_yy_w_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.R_yy_w_L(k,1)];']);
    eval(['ZP_Con_R_xx_w_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.R_xx_w_L(k,1)];']);
    eval(['ZP_Con_R_xx_r_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.R_xx_r_L(k,1)];']);
    eval(['ZP_Con_rou_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.rou_L(k,1)];']);
    eval(['t_', Expression_DummyRail{i2}, ' = t_', Expression_DummyRail{i2}, '+1;']);
end

t_R1 = 1;
t_R2 = 1;
t_R3 = 1;
for k = 1:1:size(Con_str.Normal_Force_R,1)
    i2 =  Con_str.Normal_Force_R(k,4);    
    eval(['ZP_Con_Rail_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, (Con_FF.Con_rail_R_1(k,1)-0.7175-Ori_prr-Dis_Rail(4*(i1-1)+i2,2))*1e3, (Con_FF.Con_rail_R_1(k,2)-0.6)*1e3];']);
    eval(['ZP_Con_Area_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, (pi*Con_FF.a2_R(k,:)*Con_FF.b2_R(k,:))*1e6];']);
    eval(['ZP_Con_Angle_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Con_wheel_R_2(k,6)];']);
    eval(['ZP_Con_CX_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.RHLv_R(k,1)];']);
    eval(['ZP_Con_CY_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.RHLv_R(k,2)];']);
    eval(['ZP_Con_CSpin_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.RHLv_R(k,3)];']);
    eval(['ZP_Con_CFX_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Prhx_R(k,1)];']);
    eval(['ZP_Con_CFY_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Prhx_R(k,2)];']);
    eval(['ZP_Con_CFSpin_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Prhx_R(k,3)];']);
    eval(['ZP_Con_NF_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Normal_Force_R(k,:)];']);
    eval(['ZP_Con_Vjd_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Vjd_R(k,:)];']);
    eval(['ZP_Con_Vsdc_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Vsdc_R(k,:)];']);
    eval(['ZP_Con_Vjsdc_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Vjsdc_R(k,:)];']);
    eval(['ZP_Con_Vgd_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Vgd_R(k,:)];']);
    eval(['ZP_Con_R_yy_w_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.R_yy_w_R(k,1)];']);
    eval(['ZP_Con_R_xx_w_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.R_xx_w_R(k,1)];']);
    eval(['ZP_Con_R_xx_r_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.R_xx_r_R(k,1)];']);
    eval(['ZP_Con_rou_', Expression_DummyRail{i2}, '{1,t_', Expression_DummyRail{i2}, '}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.rou_R(k,1)];']);
    eval(['t_', Expression_DummyRail{i2}, ' = t_', Expression_DummyRail{i2}, '+1;']);
end

% %% 左侧
% t_L1 = 1;
% for k = 1:1:size(Con_str.Normal_Force_L,1)
%     num_Patch =  Con_str.Normal_Force_L(i,4);
%     ZP_Con_Rail_L1{1,t_L1}(1+xlcs,:) = [Con_FF.Mileage (Con_FF.Con_rail_L_1(k,1)+0.7175+Ori_prr-Dis_Rail(4*(i1-1)+1,2))*1e3 ...
%                                        (Con_FF.Con_rail_L_1(k,2)-0.6)*1e3];
%     ZP_Con_Area_L1{1,t_L1}(1+xlcs,:) = [Con_FF.Mileage (pi*Con_FF.a2_L(k,:)*Con_FF.b2_L(k,:))*1e6];
%     ZP_Con_Angle_L1{1,t_L1}(1+xlcs,:) = [Con_FF.Mileage Con_FF.Con_wheel_L_2(k,6)];
%     ZP_Con_CX_L1{1,t_L1}(1+xlcs,:) = [Con_FF.Mileage Con_FF.RHLv_L(k,1)];
%     ZP_Con_CY_L1{1,t_L1}(1+xlcs,:) = [Con_FF.Mileage Con_FF.RHLv_L(k,2)];
%     ZP_Con_CSpin_L1{1,t_L1}(1+xlcs,:) = [Con_FF.Mileage Con_FF.RHLv_L(k,3)];
%     ZP_Con_CFX_L1{1,t_L1}(1+xlcs,:) = [Con_FF.Mileage Con_FF.Prhx_L(k,1)];
%     ZP_Con_CFY_L1{1,t_L1}(1+xlcs,:) = [Con_FF.Mileage Con_FF.Prhx_L(k,2)];
%     ZP_Con_CFSpin_L1{1,t_L1}(1+xlcs,:) = [Con_FF.Mileage Con_FF.Prhx_L(k,3)];
%     ZP_Con_NF_L1{1,t_L1}(1+xlcs,:) = [Con_FF.Mileage Con_FF.Normal_Force_L(k,1)];
%     ZP_Con_Vjd_L1{1,t_L1}(1+xlcs,:) = [Con_FF.Mileage Con_FF.Vjd_L(k,:)];
%     ZP_Con_Vsdc_L1{1,t_L1}(1+xlcs,:) = [Con_FF.Mileage Con_FF.Vsdc_L(k,:)];
%     ZP_Con_Vjsdc_L1{1,t_L1}(1+xlcs,:) = [Con_FF.Mileage Con_FF.Vjsdc_L(k,:)];
%     ZP_Con_Vgd_L1{1,t_L1}(1+xlcs,:) = [Con_FF.Mileage Con_FF.Vgd_L(k,:)];
%     ZP_Con_R_yy_w_L1{1,t_L1}(1+xlcs,:) = [Con_FF.Mileage Con_FF.R_yy_w_L(k,1)];
%     ZP_Con_R_xx_w_L1{1,t_L1}(1+xlcs,:) = [Con_FF.Mileage Con_FF.R_xx_w_L(k,1)];
%     ZP_Con_R_xx_r_L1{1,t_L1}(1+xlcs,:) = [Con_FF.Mileage Con_FF.R_xx_r_L(k,1)];
%     ZP_Con_rou_L1{1,t_L1}(1+xlcs,:) = [Con_FF.Mileage Con_FF.rou_L(k,1)];
%     t_L1 = t_L1+1;
% end

% %% 右侧
% for k = 1:1:size(Con_str.Normal_Force_R,1)
%     if ~isempty(Con_FF.profile_r_R1) && (Con_FF.Con_rail_R_1(k,1) <= max(Con_FF.profile_r_R1(:,1)))
%         % R1
%         ZP_Con_Rail_R1{t_R1}(1+xlcs,:) = [Con_FF.Mileage, (Con_FF.Con_rail_R_1(k,1)-0.7175-Ori_prr-Dis_Rail(4*(i1-1)+3,2))*1e3 ...
%                                                          (Con_FF.Con_rail_R_1(k,2)-0.6)*1e3];
%         ZP_Con_Area_R1{t_R1}(1+xlcs,:) = [Con_FF.Mileage, (pi*Con_FF.a2_R(k,:)*Con_FF.b2_R(k,:))*1e6];
%         ZP_Con_Angle_R1{t_R1}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Con_wheel_R_2(k,6)];
%         ZP_Con_CX_R1{t_R1}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.RHLv_R(k,1)];
%         ZP_Con_CY_R1{t_R1}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.RHLv_R(k,2)];
%         ZP_Con_CSpin_R1{t_R1}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.RHLv_R(k,3)];
%         ZP_Con_CFX_R1{t_R1}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Prhx_R(k,1)];
%         ZP_Con_CFY_R1{t_R1}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Prhx_R(k,2)];
%         ZP_Con_CFSpin_R1{t_R1}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Prhx_R(k,3)];
%         ZP_Con_NF_R1{t_R1}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Normal_Force_R(k,1)];
%         ZP_Con_Vjd_R1{t_R1}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Vjd_R(k,:)];
%         ZP_Con_Vsdc_R1{t_R1}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Vsdc_R(k,:)];
%         ZP_Con_Vjsdc_R1{t_R1}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Vjsdc_R(k,:)];
%         ZP_Con_Vgd_R1{t_R1}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Vgd_R(k,:)];
%         ZP_Con_R_yy_w_R1{t_R1}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.R_yy_w_R(k,1)];
%         ZP_Con_R_xx_w_R1{t_R1}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.R_xx_w_R(k,1)];
%         ZP_Con_R_xx_r_R1{t_R1}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.R_xx_r_R(k,1)];
%         ZP_Con_rou_R1{t_R1}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.rou_R(k,1)];
%         t_R1 = t_R1+1;
%     else
%         % R2
%         ZP_Con_Rail_R2{t_R2}(1+xlcs,:) = [Con_FF.Mileage, (Con_FF.Con_rail_R_1(k,1)-0.7175-Ori_prr-Dis_Rail(4*(i1-1)+4,2))*1e3 ...
%                                                          (Con_FF.Con_rail_R_1(k,2)-0.6)*1e3];
%         ZP_Con_Area_R2{t_R2}(1+xlcs,:) = [Con_FF.Mileage, (pi*Con_FF.a2_R(k,:)*Con_FF.b2_R(k,:))*1e6];
%         ZP_Con_Angle_R2{t_R2}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Con_wheel_R_2(k,6)];
%         ZP_Con_CX_R2{t_R2}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.RHLv_R(k,1)];
%         ZP_Con_CY_R2{t_R2}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.RHLv_R(k,2)];
%         ZP_Con_CSpin_R2{t_R2}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.RHLv_R(k,3)];
%         ZP_Con_CFX_R2{t_R2}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Prhx_R(k,1)];
%         ZP_Con_CFY_R2{t_R2}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Prhx_R(k,2)];
%         ZP_Con_CFSpin_R2{t_R2}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Prhx_R(k,3)];
%         ZP_Con_NF_R2{t_R2}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Normal_Force_R(k,1)];
%         ZP_Con_Vjd_R2{t_R2}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Vjd_R(k,:)];
%         ZP_Con_Vsdc_R2{t_R2}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Vsdc_R(k,:)];
%         ZP_Con_Vjsdc_R2{t_R2}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Vjsdc_R(k,:)];
%         ZP_Con_Vgd_R2{t_R2}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.Vgd_R(k,:)];
%         ZP_Con_R_yy_w_R2{t_R2}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.R_yy_w_R(k,1)];
%         ZP_Con_R_xx_w_R2{t_R2}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.R_xx_w_R(k,1)];
%         ZP_Con_R_xx_r_R2{t_R2}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.R_xx_r_R(k,1)];
%         ZP_Con_rou_R2{t_R2}(1+xlcs,:) = [Con_FF.Mileage, Con_FF.rou_R(k,1)];
%         t_R2 = t_R2+1;
%     end
% end