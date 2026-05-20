 %% 存储迭代不平衡项
 %%% ZP_Zwy_int：积分步、导向轮对里程、位移、速度、加速度、荷载、法向力迭代误差、法向力切向力合力迭代误差

 function Storage_Iteration_P2(xcs_t, Zwy_int, Zsd_int, Zjsd_int, Pxt_int, ...
           Con_Wheel_L_1_Ia_int, Con_Wheel_L_1_Ib_int, Con_Ver_Pen_L_postive_start_int, Con_Ver_Pen_L_postive_end_int, Con_Wheel_2_L_int, Con_Rail_1_L_int,...
           Con_NF_L_int, Con_Prh_L_int, Con_Prhx_L_int, Con_Prhxf_L_int, Con_RHXS_L_int, Con_RHLv_L_int, Con_Vjd_L_int, Con_Vsdc_L_int, Con_RelVel_L_int,...
           Con_Wheel_R_1_Ia_int, Con_Wheel_R_1_Ib_int, Con_Ver_Pen_R_postive_start_int, Con_Ver_Pen_R_postive_end_int, Con_Wheel_2_R_int, Con_Rail_1_R_int,...
           Con_NF_R_int, Con_Prh_R_int, Con_Prhx_R_int, Con_Prhxf_R_int, Con_RHXS_R_int, Con_RHLv_R_int, Con_Vjd_R_int, Con_Vsdc_R_int, Con_RelVel_R_int,...
           Dis_Rail_int, Vel_Rail_int, Con_WR_Geometry_int, Q_temp,...
           Con_Coor_Target_R_local_XOY_int, Con_pos_Con_Around_R_Deformed_WS_XOY_int, Con_pos_Con_Around_R_Deformed_track_XOY_int, Con_vel_Con_Around_R_Deformed_track_XOY_int,...
           Con_Coor_Target_R_local_YOZ_int, Con_pos_Con_Around_R_Deformed_WS_YOZ_int, Con_pos_Con_Around_R_Deformed_track_YOZ_int, Con_vel_Con_Around_R_Deformed_track_YOZ_int,...
           Con_Coor_Target_L_local_XOY_int, Con_pos_Con_Around_L_Deformed_WS_XOY_int, Con_pos_Con_Around_L_Deformed_track_XOY_int, Con_vel_Con_Around_L_Deformed_track_XOY_int,...
           Con_Coor_Target_L_local_YOZ_int, Con_pos_Con_Around_L_Deformed_WS_YOZ_int, Con_pos_Con_Around_L_Deformed_track_YOZ_int, Con_vel_Con_Around_L_Deformed_track_YOZ_int)

global NM_FW ZP_Zwy_int ZP_Con_int Type_Side Expression_WS Expression_DummyRail xlcs j1 ZP_IntError_Nor ZP_IntError_NorTan
       
ZP_Zwy_int{xcs_t,1} = xlcs;
ZP_Zwy_int{xcs_t,2} = j1;
ZP_Zwy_int{xcs_t,3} = Zwy_int;
ZP_Zwy_int{xcs_t,4} = Zsd_int;
ZP_Zwy_int{xcs_t,5} = Zjsd_int;
ZP_Zwy_int{xcs_t,6} = Pxt_int;
ZP_Zwy_int{xcs_t,7} = ZP_IntError_Nor(:,end);
ZP_Zwy_int{xcs_t,8} = ZP_IntError_NorTan(:,end);
ZP_Zwy_int{xcs_t,9} = Q_temp;

for kk = 1:1:2
    eval(['ZP_Con_int.Con_WR_Geometry_',Expression_WS{kk},'{xcs_t,1} = Con_WR_Geometry_int.',Expression_WS{kk},';']);
    for k = 1:1:2        
        eval(['len = length(Con_Wheel_2_',Type_Side{k},'_int.',Expression_WS{kk},');']);
        eval(['ZP_Con_int.Wheel_',Expression_WS{kk},'_',Type_Side{k},'(xcs_t,1:len) = Con_Wheel_2_',Type_Side{k},'_int.',Expression_WS{kk},';']);
        eval(['ZP_Con_int.Con_Wheel_1_Ia_',Expression_WS{kk},'_',Type_Side{k},'(xcs_t,1:len) = Con_Wheel_',Type_Side{k},'_1_Ia_int.',Expression_WS{kk},';']);
        eval(['ZP_Con_int.Con_Wheel_1_Ib_',Expression_WS{kk},'_',Type_Side{k},'(xcs_t,1:len) = Con_Wheel_',Type_Side{k},'_1_Ib_int.',Expression_WS{kk},';']);
        eval(['ZP_Con_int.Con_Ver_Pen_postive_start_',Expression_WS{kk},'_',Type_Side{k},'(xcs_t,1:len) = Con_Ver_Pen_',Type_Side{k},'_postive_start_int.',Expression_WS{kk},';']);
        eval(['ZP_Con_int.Con_Ver_Pen_postive_end_',Expression_WS{kk},'_',Type_Side{k},'(xcs_t,1:len) = Con_Ver_Pen_',Type_Side{k},'_postive_end_int.',Expression_WS{kk},';']);
        eval(['ZP_Con_int.Rail_',Expression_WS{kk},'_',Type_Side{k},'(xcs_t,1:len) = Con_Rail_1_',Type_Side{k},'_int.',Expression_WS{kk},';']);
        eval(['ZP_Con_int.Con_NF_',Expression_WS{kk},'_',Type_Side{k},'_int(xcs_t,1:len) = Con_NF_',Type_Side{k},'_int.',Expression_WS{kk},';']);
        eval(['ZP_Con_int.Con_Vjd_',Expression_WS{kk},'_',Type_Side{k},'_int(xcs_t,1:len) = Con_Vjd_',Type_Side{k},'_int.',Expression_WS{kk},';']);
        eval(['ZP_Con_int.Con_Vsdc_',Expression_WS{kk},'_',Type_Side{k},'_int(xcs_t,1:len) = Con_Vsdc_',Type_Side{k},'_int.',Expression_WS{kk},';']);
        eval(['ZP_Con_int.Con_RelVel_',Expression_WS{kk},'_',Type_Side{k},'_int(xcs_t,1:len) = Con_RelVel_',Type_Side{k},'_int.',Expression_WS{kk},';']);
        eval(['ZP_Con_int.Con_RHLv_',Expression_WS{kk},'_',Type_Side{k},'_int(xcs_t,1:len) = Con_RHLv_',Type_Side{k},'_int.',Expression_WS{kk},';']);
        eval(['ZP_Con_int.Con_RHXS_',Expression_WS{kk},'_',Type_Side{k},'_int(xcs_t,1:len) = Con_RHXS_',Type_Side{k},'_int.',Expression_WS{kk},';']);
        eval(['ZP_Con_int.Con_Prh_',Expression_WS{kk},'_',Type_Side{k},'_int(xcs_t,1:len) = Con_Prh_',Type_Side{k},'_int.',Expression_WS{kk},';']);
        eval(['ZP_Con_int.Con_Prhx_',Expression_WS{kk},'_',Type_Side{k},'_int(xcs_t,1:len) = Con_Prhx_',Type_Side{k},'_int.',Expression_WS{kk},';']);
        eval(['ZP_Con_int.Con_Prhxf_',Expression_WS{kk},'_',Type_Side{k},'_int(xcs_t,1:len) = Con_Prhxf_',Type_Side{k},'_int.',Expression_WS{kk},';']);
        
        if NM_FW > 0
            eval(['ZP_Con_int.Con_Coor_Target_',Expression_WS{kk},'_',Type_Side{k},'_local_XOY_int(xcs_t,1:len) = Con_Coor_Target_',Type_Side{k},'_local_XOY_int.',Expression_WS{kk},';']);
            eval(['ZP_Con_int.Con_pos_Con_Around_',Expression_WS{kk},'_',Type_Side{k},'_Deformed_WS_XOY_int(xcs_t,1:len) = Con_pos_Con_Around_',Type_Side{k},'_Deformed_WS_XOY_int.',Expression_WS{kk},';']);
            eval(['ZP_Con_int.Con_pos_Con_Around_',Expression_WS{kk},'_',Type_Side{k},'_Deformed_track_XOY_int(xcs_t,1:len) = Con_pos_Con_Around_',Type_Side{k},'_Deformed_track_XOY_int.',Expression_WS{kk},';']);
            eval(['ZP_Con_int.Con_vel_Con_Around_',Expression_WS{kk},'_',Type_Side{k},'_Deformed_track_XOY_int(xcs_t,1:len) = Con_vel_Con_Around_',Type_Side{k},'_Deformed_track_XOY_int.',Expression_WS{kk},';']);
            eval(['ZP_Con_int.Con_Coor_Target_',Expression_WS{kk},'_',Type_Side{k},'_local_YOZ_int(xcs_t,1:len) = Con_Coor_Target_',Type_Side{k},'_local_YOZ_int.',Expression_WS{kk},';']);
            eval(['ZP_Con_int.Con_pos_Con_Around_',Expression_WS{kk},'_',Type_Side{k},'_Deformed_WS_YOZ_int(xcs_t,1:len) = Con_pos_Con_Around_',Type_Side{k},'_Deformed_WS_YOZ_int.',Expression_WS{kk},';']);
            eval(['ZP_Con_int.Con_pos_Con_Around_',Expression_WS{kk},'_',Type_Side{k},'_Deformed_track_YOZ_int(xcs_t,1:len) = Con_pos_Con_Around_',Type_Side{k},'_Deformed_track_YOZ_int.',Expression_WS{kk},';']);
            eval(['ZP_Con_int.Con_vel_Con_Around_',Expression_WS{kk},'_',Type_Side{k},'_Deformed_track_YOZ_int(xcs_t,1:len) = Con_vel_Con_Around_',Type_Side{k},'_Deformed_track_YOZ_int.',Expression_WS{kk},';']);
        end
    end
end

for kk = 1:1:4
    eval(['ZP_Con_int.Rail_Dis_FF_',Expression_DummyRail{kk},'{xcs_t,:} = Dis_Rail_int.FF_',Expression_DummyRail{kk},';']);
    eval(['ZP_Con_int.Rail_Vel_FF_',Expression_DummyRail{kk},'{xcs_t,:} = Vel_Rail_int.FF_',Expression_DummyRail{kk},';']);
    eval(['ZP_Con_int.Rail_Dis_FR_',Expression_DummyRail{kk},'{xcs_t,:} = Dis_Rail_int.FR_',Expression_DummyRail{kk},';']);
    eval(['ZP_Con_int.Rail_Vel_FR_',Expression_DummyRail{kk},'{xcs_t,:} = Vel_Rail_int.FR_',Expression_DummyRail{kk},';']);
end
