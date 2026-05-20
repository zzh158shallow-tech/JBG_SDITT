%% 在该时间步迭代过程中，创建结构体暂时保存

function [Con_WR_Geometry_int, Con_Wheel_L_1_Ia_int, Con_Wheel_L_1_Ib_int, Con_Ver_Pen_L_postive_start_int, Con_Ver_Pen_L_postive_end_int, Con_Wheel_2_L_int, Con_Rail_1_L_int,...
 Con_NF_L_int, Con_Prh_L_int, Con_Prhx_L_int, Con_Prhxf_L_int, Con_RHXS_L_int, Con_RHLv_L_int, Con_Vjd_L_int, Con_Vsdc_L_int, Con_RelVel_L_int,...
 Con_Wheel_R_1_Ia_int, Con_Wheel_R_1_Ib_int, Con_Ver_Pen_R_postive_start_int, Con_Ver_Pen_R_postive_end_int, Con_Wheel_2_R_int, Con_Rail_1_R_int,...
 Con_NF_R_int, Con_Prh_R_int, Con_Prhx_R_int, Con_Prhxf_R_int, Con_RHXS_R_int, Con_RHLv_R_int, Con_Vjd_R_int, Con_Vsdc_R_int, Con_RelVel_R_int,...
 Con_Coor_Target_R_local_XOY_int, Con_pos_Con_Around_R_Deformed_WS_XOY_int, Con_pos_Con_Around_R_Deformed_track_XOY_int, Con_vel_Con_Around_R_Deformed_track_XOY_int,...
 Con_Coor_Target_R_local_YOZ_int, Con_pos_Con_Around_R_Deformed_WS_YOZ_int, Con_pos_Con_Around_R_Deformed_track_YOZ_int, Con_vel_Con_Around_R_Deformed_track_YOZ_int,...
 Con_Coor_Target_L_local_XOY_int, Con_pos_Con_Around_L_Deformed_WS_XOY_int, Con_pos_Con_Around_L_Deformed_track_XOY_int, Con_vel_Con_Around_L_Deformed_track_XOY_int,...
 Con_Coor_Target_L_local_YOZ_int, Con_pos_Con_Around_L_Deformed_WS_YOZ_int, Con_pos_Con_Around_L_Deformed_track_YOZ_int, Con_vel_Con_Around_L_Deformed_track_YOZ_int] = ...
Storage_Iteration_P1(Con_str, i11, xcs, Mileage, Con_WR_Geometry_int,...
 Con_Wheel_L_1_Ia_int, Con_Wheel_L_1_Ib_int, Con_Ver_Pen_L_postive_start_int, Con_Ver_Pen_L_postive_end_int, Con_Wheel_2_L_int, Con_Rail_1_L_int,...
 Con_NF_L_int, Con_Prh_L_int, Con_Prhx_L_int, Con_Prhxf_L_int, Con_RHXS_L_int, Con_RHLv_L_int, Con_Vjd_L_int, Con_Vsdc_L_int, Con_RelVel_L_int,...
 Con_Wheel_R_1_Ia_int, Con_Wheel_R_1_Ib_int, Con_Ver_Pen_R_postive_start_int, Con_Ver_Pen_R_postive_end_int, Con_Wheel_2_R_int, Con_Rail_1_R_int,...
 Con_NF_R_int, Con_Prh_R_int, Con_Prhx_R_int, Con_Prhxf_R_int, Con_RHXS_R_int, Con_RHLv_R_int, Con_Vjd_R_int, Con_Vsdc_R_int, Con_RelVel_R_int,...
 Con_Coor_Target_R_local_XOY_int, Con_pos_Con_Around_R_Deformed_WS_XOY_int, Con_pos_Con_Around_R_Deformed_track_XOY_int, Con_vel_Con_Around_R_Deformed_track_XOY_int,...
 Con_Coor_Target_R_local_YOZ_int, Con_pos_Con_Around_R_Deformed_WS_YOZ_int, Con_pos_Con_Around_R_Deformed_track_YOZ_int, Con_vel_Con_Around_R_Deformed_track_YOZ_int,...
 Con_Coor_Target_L_local_XOY_int, Con_pos_Con_Around_L_Deformed_WS_XOY_int, Con_pos_Con_Around_L_Deformed_track_XOY_int, Con_vel_Con_Around_L_Deformed_track_XOY_int,...
 Con_Coor_Target_L_local_YOZ_int, Con_pos_Con_Around_L_Deformed_WS_YOZ_int, Con_pos_Con_Around_L_Deformed_track_YOZ_int, Con_vel_Con_Around_L_Deformed_track_YOZ_int)

global Type_Side Expression_WS NM_FW

eval(['Con_WR_Geometry_int.', Expression_WS{i11},'(xcs,:)= Con_str.WR_Geometry;']);
for kk = 1:1:2
    eval(['len = size(Con_str.Normal_Force_',Type_Side{kk},',1);']);
    for k = 1:1:len
        if NM_FW > 0
            eval(['Con_Coor_Target_',Type_Side{kk},'_local_XOY_int.', Expression_WS{i11},'{1,k}(xcs,:)= Con_str.Coor_Target_',Type_Side{kk},'_local_XOY(k,:);']);
            eval(['Con_pos_Con_Around_',Type_Side{kk},'_Deformed_WS_XOY_int.', Expression_WS{i11},'{1,k}(5*(xcs-1)+1:5*(xcs-1)+4,:)= Con_str.pos_Con_Around_',Type_Side{kk},'_Deformed_WS_XOY{k,1};']);
            eval(['Con_pos_Con_Around_',Type_Side{kk},'_Deformed_track_XOY_int.', Expression_WS{i11},'{1,k}(5*(xcs-1)+1:5*(xcs-1)+4,:)= Con_str.pos_Con_Around_',Type_Side{kk},'_Deformed_track_XOY{k,1};']);
            eval(['Con_vel_Con_Around_',Type_Side{kk},'_Deformed_track_XOY_int.', Expression_WS{i11},'{1,k}(5*(xcs-1)+1:5*(xcs-1)+4,:)= Con_str.vel_Con_Around_',Type_Side{kk},'_Deformed_track_XOY{k,1};']);
            eval(['Con_Coor_Target_',Type_Side{kk},'_local_YOZ_int.', Expression_WS{i11},'{1,k}(xcs,:)= Con_str.Coor_Target_',Type_Side{kk},'_local_YOZ(k,:);']);
            eval(['Con_pos_Con_Around_',Type_Side{kk},'_Deformed_WS_YOZ_int.', Expression_WS{i11},'{1,k}(5*(xcs-1)+1:5*(xcs-1)+4,:)= Con_str.pos_Con_Around_',Type_Side{kk},'_Deformed_WS_YOZ{k,1};']);
            eval(['Con_pos_Con_Around_',Type_Side{kk},'_Deformed_track_YOZ_int.', Expression_WS{i11},'{1,k}(5*(xcs-1)+1:5*(xcs-1)+4,:)= Con_str.pos_Con_Around_',Type_Side{kk},'_Deformed_track_YOZ{k,1};']);
            eval(['Con_vel_Con_Around_',Type_Side{kk},'_Deformed_track_YOZ_int.', Expression_WS{i11},'{1,k}(5*(xcs-1)+1:5*(xcs-1)+4,:)= Con_str.vel_Con_Around_',Type_Side{kk},'_Deformed_track_YOZ{k,1};']);
        end
        eval(['Con_Wheel_',Type_Side{kk},'_1_Ia_int.', Expression_WS{i11},'{1,k}(xcs,:)= Con_str.Con_wheel_',Type_Side{kk},'_1_Ia(k,:);']);
        eval(['Con_Wheel_',Type_Side{kk},'_1_Ib_int.', Expression_WS{i11},'{1,k}(xcs,:)= Con_str.Con_wheel_',Type_Side{kk},'_1_Ib(k,:);']);
        eval(['temp = Con_str.Ver_Pen_',Type_Side{kk},'_postive_start;']);
        if ~isempty(temp)
            eval(['Con_Ver_Pen_',Type_Side{kk},'_postive_start_int.', Expression_WS{i11},'{1,k}(xcs,:)= Con_str.Ver_Pen_',Type_Side{kk},'_postive_start(k,:);']);
            eval(['Con_Ver_Pen_',Type_Side{kk},'_postive_end_int.', Expression_WS{i11},'{1,k}(xcs,:)= Con_str.Ver_Pen_',Type_Side{kk},'_postive_end(k,:);']);
        end
        eval(['Con_Wheel_2_',Type_Side{kk},'_int.',Expression_WS{i11},'{1,k}(xcs,:)= [Mileage Con_str.Con_wheel_',Type_Side{kk},'_2(k,:) Con_str.elastic_permeability_Unit_',Type_Side{kk},'(k)];']);
        eval(['Con_Rail_1_',Type_Side{kk},'_int.', Expression_WS{i11},'{1,k}(xcs,:)= [Mileage Con_str.Con_rail_',Type_Side{kk},'_1(k,:)];']);
        eval(['Con_NF_',Type_Side{kk},'_int.', Expression_WS{i11},'{1,k}(xcs,:)= Con_str.Normal_Force_',Type_Side{kk},'(k,:);']);
        eval(['Con_Prh_',Type_Side{kk},'_int.', Expression_WS{i11},'{1,k}(xcs,:)= Con_str.Prh_',Type_Side{kk},'(k,:);']);
        eval(['Con_Prhx_',Type_Side{kk},'_int.', Expression_WS{i11},'{1,k}(xcs,:)= Con_str.Prhx_',Type_Side{kk},'(k,:);']);
        eval(['Con_Prhxf_',Type_Side{kk},'_int.', Expression_WS{i11},'{1,k}(xcs,:)= Con_str.Prhxf_',Type_Side{kk},'(k,:);']);
        eval(['Con_RHXS_',Type_Side{kk},'_int.', Expression_WS{i11},'{1,k}(xcs,:)= Con_str.RHXS_',Type_Side{kk},'(k,:);']);
        eval(['Con_RHLv_',Type_Side{kk},'_int.', Expression_WS{i11},'{1,k}(xcs,:)= Con_str.RHLv_',Type_Side{kk},'(k,:);']);
        eval(['Con_Vjd_',Type_Side{kk},'_int.', Expression_WS{i11},'{1,k}(xcs,:)= Con_str.Vjd_',Type_Side{kk},'(k,:);']);
        eval(['Con_Vsdc_',Type_Side{kk},'_int.', Expression_WS{i11},'{1,k}(xcs,:)= Con_str.Vsdc_',Type_Side{kk},'(k,:);']);
        eval(['Con_RelVel_',Type_Side{kk},'_int.', Expression_WS{i11},'{1,k}(xcs,:)= Con_str.Con_RelVel_',Type_Side{kk},'(k,:);']);
    end
end
