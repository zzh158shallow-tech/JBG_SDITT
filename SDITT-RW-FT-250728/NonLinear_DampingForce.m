function [Pxt_test, FZ_PS_Damp, FX_SS_Damp, FY_SS_Damp] = NonLinear_DampingForce(InpPar, Par_Vehicle, vel_yaw_track, vel_BG_CarbodyCS, Zsd, Pxt_test)

% 220717：为了考虑一系垂向阻尼、二系纵向及横向阻尼的非线性，相关力元的阻尼力在此不进行计算
            
% Pxt_test = zeros(InpPar.N_track+NM*InpPar.Nw+35,1);
% Pxt_test = Pxt;
% pos_Radius_Vehicle,7*5; 车辆刚体对应里程位置(xlcs)、曲率半径(xlcs)、线路摇头角(xlcs)、曲率半径(xlcs-1)、线路摇头角(xlcs-1)
% pos_Body_global,7*6; 车辆刚体 Pos_XYZ(xlcs)、车辆刚体 Pos_XYZ(xlcs-1)
% pos_yaw_track,7*2; 线路摇头角(xlcs)、线路摇头角(xlcs-1)
% pos_WS_BogieCS = zeros(4,6);
% pos_BG_CarbodyCS = zeros(2,6);

% R1, R2, L_trans, Curve_start, 

% global Par_Vehicle InpPar.N_track InpPar.Nw InpPar.NM_FW Par_FW ModeShape

%% 1. 一系悬挂垂向非线性阻尼力
% dz_PS = zeros(8,1);
FZ_PS_Damp = zeros(8,1);
% for i1 = 1:1:2          % Bogie
%     for i2 = 1:1:2      % WS
%         for i3 = 1:1:2  % Left / Right
%             pos = 4*(i1-1)+2*(i2-1)+i3;
%             pos_BG = InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(i1-1);
%             pos_WS = InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(2*(i1-1)+i2-1);
%             dz_PS(pos,1) = Zsd(pos_BG+1,4) - Zsd(pos_WS+1,4) + (-1)^i2*Par_Vehicle.Ll1*Zsd(pos_BG+4,4) + ...
%                                       (-1)^(i3+1)*Par_Vehicle.Lb1*Zsd(pos_WS+3,4) + (-1)^(i3)*Par_Vehicle.Lb1*Zsd(pos_BG+3,4);
%                                   
%             if InpPar.NM_FW>0
%                 if i3 == 1
%                     DOF_pos_AB = Par_FW.DOF_pos.AxleBox_L;
%                 else
%                     DOF_pos_AB = Par_FW.DOF_pos.AxleBox_R;
%                 end
%                 i11 = 2*(i1-1)+i2;
%                 dz_PS(pos,1) = dz_PS(pos,1) - ModeShape.FW(DOF_pos_AB(3),:) * Zsd(InpPar.N_track+InpPar.NM_FW*(i11-1)+1:InpPar.N_track+InpPar.NM_FW*i11,4);
%             end                                       
%                                     
%             FZ_PS_Damp(pos,1) = interp1(Par_Vehicle.C1z_Table(:,1), Par_Vehicle.C1z_Table(:,2), dz_PS(pos,1), 'linear');
%             
%             Pxt_test(pos_WS+1,1) = Pxt_test(pos_WS+1,1) + FZ_PS_Damp(pos,1);
%             Pxt_test(pos_BG+1,1) = Pxt_test(pos_BG+1,1) - FZ_PS_Damp(pos,1);
%             Pxt_test(pos_BG+4,1) = Pxt_test(pos_BG+4,1) + (-1)^(i2+1)*Par_Vehicle.Ll1*FZ_PS_Damp(pos,1);
%             Pxt_test(pos_WS+3,1) = Pxt_test(pos_WS+3,1) + (-1)^(i3)*Par_Vehicle.Lb1*FZ_PS_Damp(pos,1);
%             Pxt_test(pos_BG+3,1) = Pxt_test(pos_BG+3,1) + (-1)^(i3+1)*Par_Vehicle.Lb1*FZ_PS_Damp(pos,1);
%             
%         end
%     end
% end

% Check
% for i = 3:1:xlcs
% for i1 = 1:1:1          % Bogie
%     for i2 = 1:1:1      % WS
%         for i3 = 2:1:2  % Left / Right
%             pos = 4*(i1-1)+2*(i2-1)+i3;
%             pos_BG = InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(i1-1);
%             pos_WS = InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(2*(i1-1)+i2-1);
%             dz_PS(i-2,1) = ZP_Vel(i,pos_BG+1) - ZP_Vel(i,pos_WS+1) + (-1)^i2*Par_Vehicle.Ll1*ZP_Vel(i,pos_BG+4) + ...
%                                         (-1)^(i3+1)*Par_Vehicle.Lb1*ZP_Vel(i,pos_WS+3) + (-1)^(i3)*Par_Vehicle.Lb1*ZP_Vel(i,pos_BG+3);
%             FZ_PS_Damp(i-2,1) = interp1(Par_Vehicle.C1z_Table(:,1), Par_Vehicle.C1z_Table(:,2), dz_PS(i-2,1), 'linear');            
%         end
%     end
% end
% end
% 
% figure(21); clf
% xx = cell2mat(ZP_Dyn.Mileage(:,1));
% plot(xx, dz_PS(3:end-1,1));
% plot(xx, FZ_PS_Damp(3:end-1,1));

%% 2. 二系悬挂横向非线性阻尼力
% for i = 1:1:2
%     row_bogie = i + 4;
%     row_carbody = 7;
%     T = [cos(pos_yaw_track(row_carbody,1))	-sin(pos_yaw_track(row_carbody,1))    0
%             sin(pos_yaw_track(row_carbody,1))	 cos(pos_yaw_track(row_carbody,1))    0
%             0                                   0                                    1];    
%     T_d1_temp = [-sin(pos_yaw_track(row_carbody,1)) -cos(pos_yaw_track(row_carbody,1))    0
%                   cos(pos_yaw_track(row_carbody,1))	-sin(pos_yaw_track(row_carbody,1))    0
%                   0                                   0                                   0];
%     T_d1 = T_d1_temp * vel_yaw_track(row_carbody,1);
%     pos_BG_CarbodyCS(i,1:3) = (pos_Body_global(row_bogie,1:3)-pos_Body_global(row_carbody,1:3)) * T;
%     vel_BG_CarbodyCS(i,1:3) = (vel_Body_global(row_bogie,1:3)-vel_Body_global(row_carbody,1:3)) * T + ...
%                                                     (pos_Body_global(row_bogie,1:3)-pos_Body_global(row_carbody,1:3)) * T_d1;
%     F_SecSusp_Lat  = Par_Vehicle.K2y*pos_BG_CarbodyCS(i,2) + Par_Vehicle.C2y*vel_BG_CarbodyCS(i,2);
%     
%     Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(i-1)+2,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(i-1)+2,1) - 2*F_SecSusp_Lat;
%     Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(i-1)+3,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(i-1)+3,1) - 2*F_SecSusp_Lat*Par_Vehicle.H3;
%     Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30+2,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30+2,1) + 2*F_SecSusp_Lat;
%     Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30+3,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30+3,1) - 2*F_SecSusp_Lat*Par_Vehicle.H2;
%     Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30+5,1) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30+5,1) + (-1)^(i-1)*2*F_SecSusp_Lat*Par_Vehicle.Ll2;
% end

%% 3. 二系悬挂纵向非线性阻尼力
% for i = 1:1:2
%     num_bogie = i;
%     row_bogie = i + 4;
%     row_carbody = 7;
%     for j = 1:1:2
%         F_SecSusp_Lon = (-1)^(j-1)* Par_Vehicle.K2x * Par_Vehicle.Lb2* (pos_yaw_track(row_carbody,1)-pos_yaw_track(row_bogie,1)) + ...
%                                         (-1)^(j-1)* Par_Vehicle.C2x * Par_Vehicle.Lb2* (vel_yaw_track(row_carbody,1)-vel_yaw_track(row_bogie,1));
%         Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(i-1)+5) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(i-1)+5) + Par_Vehicle.Lb2 * (-1)^(j-1) * F_SecSusp_Lon;        
%         Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(i-1)+4) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(i-1)+4) - Par_Vehicle.H3 * F_SecSusp_Lon;
%         Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30+5) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30+5) + Par_Vehicle.Lb2 * (-1)^j * F_SecSusp_Lon; 
%         Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30+4) = Pxt_test(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30+4) - Par_Vehicle.H2 * F_SecSusp_Lon;
%     end
% end

dx_SS = zeros(4,1);
dy_SS = zeros(4,1);
FX_SS_Damp = zeros(4,1);
FY_SS_Damp = zeros(4,1);
for i1 = 1:1:2          % Bogie
    for i3 = 1:1:2      % Left / Right
        pos = 2*(i1-1)+i3;
        pos_CB = InpPar.N_track+InpPar.NM_FW*InpPar.Nw+30;
        pos_BG = InpPar.N_track+InpPar.NM_FW*InpPar.Nw+20+5*(i1-1);
        row_bogie = i1 + 4;
        row_carbody = 7;
        
        dx_SS(pos,1) = Par_Vehicle.H2*Zsd(pos_CB+4,4) + Par_Vehicle.H3*Zsd(pos_BG+4,4) + ...
                                   (-1)^(i3+1)*Par_Vehicle.Lb2*Zsd(pos_CB+5,4) + (-1)^(i3)*Par_Vehicle.Lb2*Zsd(pos_BG+5,4) + ...
                                   (-1)^(i3+1)* Par_Vehicle.Lb2* (vel_yaw_track(row_carbody,1)-vel_yaw_track(row_bogie,1));
                                       
        dy_SS(pos,1) = Zsd(pos_BG+2,4) - Zsd(pos_CB+2,4) + Par_Vehicle.H3*Zsd(pos_BG+3,4) + Par_Vehicle.H2*Zsd(pos_CB+3,4) + ...
                                   (-1)^(i1)*Par_Vehicle.Ll2*Zsd(pos_CB+5,4) + vel_BG_CarbodyCS(i1,2);     
                               
%         dz_SS(pos,1) = Zsd(pos_CB+1,4) - Zsd(pos_BG+1,4) + (-1)^i1*Par_Vehicle.Ll2*Zsd(pos_CB+4,4) + ...
%                                     (-1)^(i3+1)*Par_Vehicle.Lb2*Zsd(pos_BG+3,4) + (-1)^(i3)*Par_Vehicle.Lb2*Zsd(pos_CB+3,4);
        
        FX_SS_Damp(pos,1) = interp1(Par_Vehicle.C2x_Table(:,1), Par_Vehicle.C2x_Table(:,2), dx_SS(pos,1), 'linear');
        FY_SS_Damp(pos,1) = interp1(Par_Vehicle.C2y_Table(:,1), Par_Vehicle.C2y_Table(:,2), dy_SS(pos,1), 'linear');
        
        Pxt_test(pos_BG+2,1) = Pxt_test(pos_BG+2,1) - FY_SS_Damp(pos,1);
        Pxt_test(pos_BG+3,1) = Pxt_test(pos_BG+3,1) - FY_SS_Damp(pos,1)*Par_Vehicle.H3;
        Pxt_test(pos_CB+2,1) = Pxt_test(pos_CB+2,1) + FY_SS_Damp(pos,1);
        Pxt_test(pos_CB+3,1) = Pxt_test(pos_CB+3,1) - FY_SS_Damp(pos,1)*Par_Vehicle.H2;
        Pxt_test(pos_CB+5,1) = Pxt_test(pos_CB+5,1) + (-1)^(i1-1)*FY_SS_Damp(pos,1)*Par_Vehicle.Ll2;
        
        Pxt_test(pos_BG+5) = Pxt_test(pos_BG+5) + Par_Vehicle.Lb2 * (-1)^(i3-1) * FX_SS_Damp(pos,1);        
        Pxt_test(pos_BG+4) = Pxt_test(pos_BG+4) - Par_Vehicle.H3 * FX_SS_Damp(pos,1);
        Pxt_test(pos_CB+5) = Pxt_test(pos_CB+5) + Par_Vehicle.Lb2 * (-1)^i3 * FX_SS_Damp(pos,1); 
        Pxt_test(pos_CB+4) = Pxt_test(pos_CB+4) - Par_Vehicle.H2 * FX_SS_Damp(pos,1);
        
    end
end


