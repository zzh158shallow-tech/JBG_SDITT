function [ZP_Dyn, ZP_Con, ZP_Int] = DefineStruct_ZP(InpPar, xlzcs)

% global InpPar.Nw InpPar.N_ConPatch xlzcs
% global InpPar.Exp_WS InpPar.Exp_DummyRail InpPar.Type_Side InpPar.NM_FW

%% ZP_Dyn
ZP_Dyn.WR_Geometry_FW = cell(4,1);
ZP_Dyn.Mileage = zeros(xlzcs, InpPar.N_ConPatch+1);
ZP_Dyn.drtaT = NaN(xlzcs,4);
ZP_Dyn.Yw = NaN(xlzcs,2);
ZP_Dyn.Vzd = NaN(xlzcs,4);
ZP_Dyn.VF = NaN(xlzcs,8);  % FF_L, FF_R, FR_L, FR_R, ...
ZP_Dyn.LF = NaN(xlzcs,8);  % 对应8个车轮
ZP_Dyn.Pjcc = NaN(xlzcs,16+1);
ZP_Dyn.Pjch = NaN(xlzcs,16+1);

for i1 = 1:1:InpPar.Nw
    for i2 = 1:1:InpPar.N_ConPatch
        ZP_Dyn.Dis_Rail.(InpPar.Exp_WS{i1}).(InpPar.Exp_DummyRail{i2}) = NaN(xlzcs,4+3);
        ZP_Dyn.Vel_Rail.(InpPar.Exp_WS{i1}).(InpPar.Exp_DummyRail{i2}) = NaN(xlzcs,4+3);
        ZP_Dyn.Acc_Rail.(InpPar.Exp_WS{i1}).(InpPar.Exp_DummyRail{i2}) = NaN(xlzcs,4+3);
        ZP_Dyn.FX.(InpPar.Exp_WS{i1}).(InpPar.Exp_DummyRail{i2}) = NaN(xlzcs,2);
        ZP_Dyn.FY.(InpPar.Exp_WS{i1}).(InpPar.Exp_DummyRail{i2}) = NaN(xlzcs,2);
        ZP_Dyn.FZ.(InpPar.Exp_WS{i1}).(InpPar.Exp_DummyRail{i2}) = NaN(xlzcs,2);
    end
end

for i1 = 1:1:InpPar.Nw
    for i3 = 1:1:length(InpPar.Type_Side)
        ZP_Dyn.Con_Rail_1.(InpPar.Type_Side{i3}){i1,1} = NaN(xlzcs,2+1);
        ZP_Dyn.Con_Rail_2.(InpPar.Type_Side{i3}){i1,1} = NaN(xlzcs,2+1);
        ZP_Dyn.Con_Wheel_1.(InpPar.Type_Side{i3}){i1,1} = NaN(xlzcs,6+1);
        ZP_Dyn.Con_Wheel_2.(InpPar.Type_Side{i3}){i1,1} = NaN(xlzcs,7+1);
        ZP_Dyn.Normal_Force.(InpPar.Type_Side{i3}){i1,1} = NaN(xlzcs,6+1);
        ZP_Dyn.Prhx.(InpPar.Type_Side{i3}){i1,1} = NaN(xlzcs,3+1);
        ZP_Dyn.Prhxf.(InpPar.Type_Side{i3}){i1,1} = NaN(xlzcs,6+1);
        ZP_Dyn.a2.(InpPar.Type_Side{i3}){i1,1} = NaN(xlzcs,1+1);
        ZP_Dyn.b2.(InpPar.Type_Side{i3}){i1,1} = NaN(xlzcs,1+1);
        ZP_Dyn.RHXS.(InpPar.Type_Side{i3}){i1,1} = NaN(xlzcs,4+1);
        ZP_Dyn.RHLv.(InpPar.Type_Side{i3}){i1,1} = NaN(xlzcs,3+1);
    end
end

%% ZP_Con
% for i1 = 1:1:1
for i1 = 1:1:InpPar.Nw
    for i2 = 1:1:InpPar.N_ConPatch
        ZP_Con.Rail_1.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,3);
        ZP_Con.Rail_2.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,3);
        ZP_Con.Wheel_2.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,7);
        ZP_Con.Area.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,2);
        ZP_Con.Angle.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,2);
        ZP_Con.CX.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,2);
        ZP_Con.CY.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,2);
        ZP_Con.CSpin.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,2);
        ZP_Con.CFX.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,2);
        ZP_Con.CFY.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,2);
        ZP_Con.CFSpin.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,2);
        ZP_Con.NF.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,7);
        ZP_Con.Vjd.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,4);
        ZP_Con.Vjd_r.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,4);
        ZP_Con.Vsdc.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,4);
        ZP_Con.Vjsdc.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,4);
        ZP_Con.Vgd.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,2);
        ZP_Con.R_yy_w.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,2);
        ZP_Con.R_xx_w.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,2);
        ZP_Con.R_xx_r.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,2);
        ZP_Con.rou.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,2);
        ZP_Con.RelVel.(InpPar.Exp_DummyRail{i2}) =cell(0,0);
        ZP_Con.RelVel_max.(InpPar.Exp_DummyRail{i2}) =cell(0,0);
%         ZP_Con.RelVel.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,3);
%         ZP_Con.RelVel_max.(InpPar.Exp_DummyRail{i2}){i1,1} = NaN(xlzcs,2);
    end
end

%% ZP_Int
ZP_Int.Zwy = cell(1,9);

% for i1 = 1:1:InpPar.Nw
%     for i2 = 1:1:InpPar.N_ConPatch
%         ZP_Int.Dis_Rail.(InpPar.Exp_WS{i1}).(InpPar.Exp_DummyRail{i2}) = cell(1,1);
%         ZP_Int.Vel_Rail.(InpPar.Exp_WS{i1}).(InpPar.Exp_DummyRail{i2}) = cell(1,1);
%         ZP_Int.Acc_Rail.(InpPar.Exp_WS{i1}).(InpPar.Exp_DummyRail{i2}) = cell(1,1);
%     end
% end
% 
% for i1 = 1:1:InpPar.Nw
%     ZP_Int.Con_WR_Geometry.(InpPar.Exp_WS{i1}) = cell(1,1);
%     for i3 = 1:1:length(InpPar.Type_Side)
%         ZP_Int.Con_Wheel_2.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
%         ZP_Int.Con_Wheel_1_Ia.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
%         ZP_Int.Con_Wheel_1_Ib.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
%         ZP_Int.Con_Ver_Pen_postive_start.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
%         ZP_Int.Con_Ver_Pen_postive_end.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
%         ZP_Int.Con_Rail_1.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
%         ZP_Int.Con_NF.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
%         ZP_Int.Con_Vjd.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
%         ZP_Int.Con_Vsdc.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
%         ZP_Int.Con_RelVel.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
%         ZP_Int.Con_RHLv.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
%         ZP_Int.Con_RHXS.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
%         ZP_Int.Con_Prh.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
%         ZP_Int.Con_Prhx.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
%         ZP_Int.Con_Prhxf.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
%         if InpPar.NM_FW > 0
%             ZP_Int.FW_Con_ShapeFun_XOY.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
%             ZP_Int.FW_Con_ShapeFun_Z.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
% %             ZP_Int.FW_Con_Pos_FW_global.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
% %             ZP_Int.FW_Con_Vel_FW_global.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
% %             ZP_Int.FW_Con_pos_Con_Around_Deformed_WS_XOY.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
% %             ZP_Int.FW_Con_pos_Con_Around_Deformed_track_XOY.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
% %             ZP_Int.FW_Con_vel_Con_Around_Deformed_track_XOY.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
% %             ZP_Int.FW_Con_Coor_Target_local_YOZ.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
% %             ZP_Int.FW_Con_pos_Con_Around_Deformed_WS_YOZ.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
% %             ZP_Int.FW_Con_pos_Con_Around_Deformed_track_YOZ.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
% %             ZP_Int.FW_Con_vel_Con_Around_Deformed_track_YOZ.(InpPar.Type_Side{i3}).(InpPar.Exp_WS{i1}) = cell(1,1);
%         end
%     end
% end


