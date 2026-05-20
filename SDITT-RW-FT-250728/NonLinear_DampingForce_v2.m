function [Pxt_test, FZ_DPz_Damp, FX_Sx_Damp, FY_DPy_Damp, dz_PS, dx_SS, dy_SS] = NonLinear_DampingForce_v2(vel_yaw_track, vel_BG_CarbodyCS, Zsd, Pxt_test)

% 220717：为了考虑一系垂向阻尼、二系纵向及横向阻尼的非线性，相关力元的阻尼力在此不进行计算
            
% Pxt_test = zeros(N_track+NM*Nw+35,1);
% Pxt_test = Pxt;
% pos_Radius_Vehicle,7*5; 车辆刚体对应里程位置(xlcs)、曲率半径(xlcs)、线路摇头角(xlcs)、曲率半径(xlcs-1)、线路摇头角(xlcs-1)
% pos_Body_global,7*6; 车辆刚体 Pos_XYZ(xlcs)、车辆刚体 Pos_XYZ(xlcs-1)
% pos_yaw_track,7*2; 线路摇头角(xlcs)、线路摇头角(xlcs-1)
% pos_WS_BogieCS = zeros(4,6);
% pos_BG_CarbodyCS = zeros(2,6);

% R1, R2, L_trans, Curve_start, 

global Par_Vehicle N_track Nw NM_FW Par_FW ModeShape

%% 1. 一系悬挂垂向非线性阻尼力
dz_PS = zeros(8,1);
FZ_DPz_Damp = zeros(8,1);
for i1 = 1:1:2          % Bogie
    for i2 = 1:1:2      % WS
        for i3 = 1:1:2  % Left / Right
            pos = 4*(i1-1)+2*(i2-1)+i3;
            pos_BG = N_track+NM_FW*Nw+20+5*(i1-1);
            pos_P = N_track+NM_FW*Nw+35+pos;
%             pos_WS = N_track+NM_FW*Nw+10*(i1-1)+5*(i2-1);
            dz_PS(pos,1) = Zsd(pos_BG+1,4) - Zsd(pos_P,4) + (-1)^(i3)*Par_Vehicle.Lb_DPz*Zsd(pos_BG+3,4) + (-1)^i2*Par_Vehicle.Ll_DPz*Zsd(pos_BG+4,4);
                           
            % 待修改！
%             if NM_FW>0
%                 if i3 == 1
%                     DOF_pos_AB = Par_FW.DOF_pos.AxleBox_L;
%                 else
%                     DOF_pos_AB = Par_FW.DOF_pos.AxleBox_R;
%                 end
%                 i11 = 2*(i1-1)+i2;
%                 dz_PS(pos,1) = dz_PS(pos,1) - ModeShape.FW(DOF_pos_AB(3),:) * Zsd(N_track+NM_FW*(i11-1)+1:N_track+NM_FW*i11,4);
%             end                                    
            FZ_DPz_Damp(pos,1) = interp1(Par_Vehicle.C_DPz_Table(:,1), Par_Vehicle.C_DPz_Table(:,2), dz_PS(pos,1), 'linear');
            
            % 对构架，FZ_DPz_Damp(pos,1)向上为正
%             Pxt_test(pos_WS+1,1) = Pxt_test(pos_WS+1,1) + FZ_DPz_Damp(pos,1);
%             Pxt_test(pos_WS+3,1) = Pxt_test(pos_WS+3,1) + (-1)^(i3)*Par_Vehicle.Lb1*FZ_DPz_Damp(pos,1);
            Pxt_test(pos_BG+1,1) = Pxt_test(pos_BG+1,1) - FZ_DPz_Damp(pos,1);
            Pxt_test(pos_BG+4,1) = Pxt_test(pos_BG+4,1) + (-1)^(i2+1)*Par_Vehicle.Ll_DPz*FZ_DPz_Damp(pos,1);
            Pxt_test(pos_BG+3,1) = Pxt_test(pos_BG+3,1) + (-1)^(i3+1)*Par_Vehicle.Lb_DPz*FZ_DPz_Damp(pos,1);
            Pxt_test(pos_P,1) = Pxt_test(pos_P,1) + FZ_DPz_Damp(pos,1);
        end
    end
end


%% 2. 抗蛇行减振器纵向非线性阻尼力

%% 3. 二系悬挂纵向非线性阻尼力
dx_SS = zeros(4,1);
dy_SS = zeros(4,1);
FX_Sx_Damp = zeros(4,1);
FY_DPy_Damp = zeros(4,1);
for i1 = 1:1:2          % Bogie
    for i3 = 1:1:2      % Left / Right
        pos = 2*(i1-1)+i3;
        pos_CB = N_track+NM_FW*Nw+30;
        pos_Sn = N_track+NM_FW*Nw+35+8+pos;
        pos_BG = N_track+NM_FW*Nw+20+5*(i1-1);
        row_bogie = i1 + 4;
        row_carbody = 7;
        
%         dx_SS(pos,1) = Par_Vehicle.H2*Zsd(pos_CB+4,4) + Par_Vehicle.H3*Zsd(pos_BG+4,4) + ...
%                                    (-1)^(i3+1)*Par_Vehicle.Lb2*Zsd(pos_CB+5,4) + (-1)^(i3)*Par_Vehicle.Lb2*Zsd(pos_BG+5,4) + ...
%                                    (-1)^(i3+1)* Par_Vehicle.Lb2* (vel_yaw_track(row_carbody,1)-vel_yaw_track(row_bogie,1));
                               
        dx_SS(pos,1) = Par_Vehicle.H_cS*Zsd(pos_CB+4,4) - Zsd(pos_Sn,4) + ...
                                   (-1)^(i3+1)*Par_Vehicle.Lb_S*Zsd(pos_CB+5,4) + ...
                                   (-1)^(i3+1)* Par_Vehicle.Lb_S* (vel_yaw_track(row_carbody,1)-vel_yaw_track(row_bogie,1));
                                       
        dy_SS(pos,1) = Zsd(pos_BG+2,4) - Zsd(pos_CB+2,4) + Par_Vehicle.H_Bt*Zsd(pos_BG+3,4) + Par_Vehicle.H_cB*Zsd(pos_CB+3,4) + ...
                                   (-1)^(i1)*Par_Vehicle.Ll2*Zsd(pos_CB+5,4) + vel_BG_CarbodyCS(i1,2);     
                               
%         dz_SS(pos,1) = Zsd(pos_CB+1,4) - Zsd(pos_BG+1,4) + (-1)^i1*Par_Vehicle.Ll2*Zsd(pos_CB+4,4) + ...
%                                     (-1)^(i3+1)*Par_Vehicle.Lb2*Zsd(pos_BG+3,4) + (-1)^(i3)*Par_Vehicle.Lb2*Zsd(pos_CB+3,4);
        
        FX_Sx_Damp(pos,1) = interp1(Par_Vehicle.C_Sx_Table(:,1), Par_Vehicle.C_Sx_Table(:,2), dx_SS(pos,1), 'linear');
        FY_DPy_Damp(pos,1) = interp1(Par_Vehicle.C_DPy_Table(:,1), Par_Vehicle.C_DPy_Table(:,2), dy_SS(pos,1), 'linear');

%         Pxt_test(pos_BG+5) = Pxt_test(pos_BG+5) + Par_Vehicle.Lb2 * (-1)^(i3-1) * FX_SS_Damp(pos,1);        
%         Pxt_test(pos_BG+4) = Pxt_test(pos_BG+4) - Par_Vehicle.H3 * FX_SS_Damp(pos,1);
%         Pxt_test(pos_CB+5) = Pxt_test(pos_CB+5) + Par_Vehicle.Lb2 * (-1)^i3 * FX_SS_Damp(pos,1); 
%         Pxt_test(pos_CB+4) = Pxt_test(pos_CB+4) - Par_Vehicle.H2 * FX_SS_Damp(pos,1);

        Pxt_test(pos_CB+5) = Pxt_test(pos_CB+5) - (-1)^(i1+i3) *Par_Vehicle.Lb_S * FX_Sx_Damp(pos,1);
        Pxt_test(pos_CB+4) = Pxt_test(pos_CB+4) + (-1)^(i1)*Par_Vehicle.H_cS * FX_Sx_Damp(pos,1);
        Pxt_test(pos_P,1) = Pxt_test(pos_P,1) + FX_Sx_Damp(pos,1);
                
        Pxt_test(pos_BG+2,1) = Pxt_test(pos_BG+2,1) - FY_DPy_Damp(pos,1);
        Pxt_test(pos_BG+3,1) = Pxt_test(pos_BG+3,1) - FY_DPy_Damp(pos,1)*Par_Vehicle.H_Bt;
        Pxt_test(pos_CB+2,1) = Pxt_test(pos_CB+2,1) + FY_DPy_Damp(pos,1);
        Pxt_test(pos_CB+3,1) = Pxt_test(pos_CB+3,1) - FY_DPy_Damp(pos,1)*Par_Vehicle.H_cB;
        Pxt_test(pos_CB+5,1) = Pxt_test(pos_CB+5,1) + (-1)^(i1+1)*FY_DPy_Damp(pos,1)*Par_Vehicle.Ll2;
        
    end
end


