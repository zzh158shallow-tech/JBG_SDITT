function [DamperNL, Cxt, Newmark, Park, Houbolt] = Judge_DamperNL(InpPar, xcs, xlcs, Par_Vehicle, Zwy, Zsd, ...
                 vel_yaw_track, Mxt, Kxt, Cxt, drtaT, i_drtaT, Newmark, Park, Houbolt, C_vehicle_v0, Con_Int, Par_FW)

% global InpPar.N_track InpPar.NM_FW InpPar.Nw Par_Vehicle InpPar.Type_simulation xlcs InpPar.Int_Method

if xlcs<4
    tt = xlcs;
else
    tt = 4;
end

% Mark_DamperNL
DamperNL = struct;
pos_DOF_RV = InpPar.N_track+InpPar.NM_FW*InpPar.Nw;

%% A1. Ò»Ïµ´¹Ïò¼õÕñÆ÷
% DamperNL.DPz.DampVel = zeros(8,1);
% kk = 1;
% for i1 = 1:1:2          % Bogie
%     for i2 = 1:1:2      % WS
%         for i3 = 1:1:2  % Left / Right
%             pos = 4*(i1-1)+2*(i2-1)+i3;
%             pos_BG(kk) = pos_DOF_RV+20+5*(i1-1);
%             pos_P(kk) = pos_DOF_RV+35+pos;
%             DamperNL.DPz(pos,1) = Zsd(pos_BG+1,tt) - Zsd(pos_P,tt) + (-1)^(i3)*Par_Vehicle.Lb_DPz*Zsd(pos_BG+3,tt) + (-1)^i2*Par_Vehicle.Ll_DPz*Zsd(pos_BG+4,tt);
%             kk = kk+1;
%         end
%     end
% end

pos_BG = [(pos_DOF_RV+20)*ones(4,1); (pos_DOF_RV+25)*ones(4,1)];
pos_P = pos_DOF_RV+35+(1:1:8)';
pos_WS = pos_DOF_RV+[0; 0; 5; 5; 10; 10; 15; 15];
Range_i3 = repmat([1, 2]', 4, 1);
Range_i2 = [1, 1, 2, 2, 1, 1, 2, 2]';
DamperNL.DPz.DampVel(:,1) = Zsd(pos_BG+1,tt) - Zsd(pos_P,tt) + (-1).^Range_i3.*Par_Vehicle.Lb_DPz.*Zsd(pos_BG+3,tt) + (-1).^Range_i2.*Par_Vehicle.Ll_DPzt.*Zsd(pos_BG+4,tt);

if InpPar.NM_FW==0
    DamperNL.DPz.SpringLength(:,1) = Zwy(pos_P,tt) - Zwy(pos_WS+1,tt) - (-1).^Range_i3.*Par_Vehicle.Lb_DPz.*Zwy(pos_WS+3,tt) - (-1).^Range_i2.*Par_Vehicle.Ll_DPzw.*Zwy(pos_WS+4,tt);
else
    for i1 = 1:1:2
        for i2 = 1:1:2
            i11 = 2*(i1-1)+i2;
            pos_FW = InpPar.N_track+InpPar.NM_FW*(i11-1)+(1:1:InpPar.NM_FW);
            for i3 = 1:1:2
                i = 4*(i1-1)+2*(i2-1)+i3;
                if i3==1
                    DOFpos = Par_FW.DOF_pos.AxleBox_L(1,3);
                else
                    DOFpos = Par_FW.DOF_pos.AxleBox_R(1,3);
                end
                DamperNL.DPz.SpringLength(i,1) = Zwy(pos_P(i),tt) - Zwy(pos_WS(i)+1,tt) - (-1).^Range_i3(i).*Par_Vehicle.Lb_DPz.*Zwy(pos_WS(i)+3,tt) - ...
                    (-1).^Range_i2(i).*Par_Vehicle.Ll_DPzw.*Zwy(pos_WS(i)+4,tt) - InpPar.ModeShape.FW(DOFpos,:)*Zwy(pos_FW,tt);
            end
        end
    end
end
DamperNL.DPz.Mark = abs(DamperNL.DPz.DampVel)>Par_Vehicle.C_DPz_V0;

%% A2. ¿¹ÉßÐÐ¼õÕñÆ÷
% dx_SS = zeros(4,1);
% dy_SS = zeros(4,1);
% for i1 = 1:1:2          % Bogie
%     for i3 = 1:1:2      % Left / Right
%         pos = 2*(i1-1)+i3;
%         pos_CB = pos_DOF_RV+30;
%         pos_Sn = pos_DOF_RV+35+8+pos;
%         pos_BG = pos_DOF_RV+20+5*(i1-1);
%         row_bogie = i1 + 4;
%         row_carbody = 7;
%         dx_SS(pos,1) = Par_Vehicle.H_cS*Zsd(pos_CB+4,tt) - Zsd(pos_Sn,tt) + ...
%                                    (-1)^(i3+1)*Par_Vehicle.Lb_S*Zsd(pos_CB+5,tt) + ...
%                                    (-1)^(i3+1)* Par_Vehicle.Lb_S* (vel_yaw_track(row_carbody,1)-vel_yaw_track(row_bogie,1));                                       
%         dy_SS(pos,1) = Zsd(pos_BG+2,tt) - Zsd(pos_CB+2,tt) + Par_Vehicle.H_Bt*Zsd(pos_BG+3,tt) + Par_Vehicle.H_cB*Zsd(pos_CB+3,tt) + ...
%                                    (-1)^(i1)*Par_Vehicle.Ll2*Zsd(pos_CB+5,tt) + vel_BG_CarbodyCS(i1,2);     
%     end
% end

pos_CB = (pos_DOF_RV+30)*ones(4,1);
pos_Sn = pos_DOF_RV+35+8+(1:1:4)';
pos_BG = pos_DOF_RV+[20; 20; 25; 25];
row_carbody = 7*ones(4,1);
row_Sn = 7+(1:1:4)';
Range_i3 = repmat([1, 2]', 2, 1);
% DamperNL.Sx.DampVel(:,1) = Par_Vehicle.H_cS.*Zsd(pos_CB+4,tt) - Zsd(pos_Sn,tt) + ...
%                                                     (-1).^(Range_i3+1).*Par_Vehicle.Lb_S.*Zsd(pos_CB+5,tt) + ...
%                                                     (-1).^(Range_i3+1).* Par_Vehicle.Lb_S.* (vel_yaw_track(row_carbody,1)-vel_yaw_track(row_Sn,1));
DamperNL.Sx.DampVel(:,1) = Par_Vehicle.H_cS.*Zsd(pos_CB+4,tt) - Zsd(pos_Sn,tt) + ...
                                                (-1).^(Range_i3+1).*Par_Vehicle.Lb_S.*Zsd(pos_CB+5,tt);
DamperNL.Sx.SpringLength(:,1) = Zwy(pos_Sn,tt) + Par_Vehicle.H_St.*Zwy(pos_BG+4,tt) - ...
                                                       (-1).^(Range_i3+1).*Par_Vehicle.Lb_S.*Zwy(pos_BG+5,tt);
DamperNL.Sx.Mark = abs(DamperNL.Sx.DampVel)>Par_Vehicle.C_Sx_V0;

%% A3. ¶þÏµºáÏò¼õÕñÆ÷
Range_i1 = [1,1,2,2]';
Range_i2 = [1,2,1,2]';
pos_CB = (pos_DOF_RV+30)*ones(4,1);
pos_P = pos_DOF_RV+35+12+(1:1:4)';
pos_BG = pos_DOF_RV+20+5.*(Range_i1-1);
row_carbody = 7*ones(4,1);
row_P = 7+4+(1:1:4)';
row_bogie = 4+Range_i1;

% T = [cos(pos_yaw_track(row_P,1))	-sin(pos_yaw_track(row_P,1))    0
%         sin(pos_yaw_track(row_P,1))       cos(pos_yaw_track(row_P,1))   0
%          0                                                      0                                                     1];
% T_d1_temp = [-sin(pos_yaw_track(row_P,1))   -cos(pos_yaw_track(row_P,1))  0
%                             cos(pos_yaw_track(row_P,1))  -sin(pos_yaw_track(row_P,1))   0
%                             0                                                     0                                                   0];
% T_d1 = T_d1_temp * vel_yaw_track(row_P,1);
% pos_CB_PCS(:,1:3) = (pos_Body_global(row_carbody,1:3)-pos_Body_global(row_P,1:3)) * T;
% vel_CB_PCS(:,1:3) = (vel_Body_global(row_carbody,1:3)-vel_Body_global(row_P,1:3)) * T + ...
%                                    (pos_Body_global(row_carbody,1:3)-pos_Body_global(row_P,1:3)) * T_d1;
% DamperNL.DPy.DampVel(:,1) = Zsd(pos_CB+2,tt) - Zsd(pos_P,tt) - Par_Vehicle.H_cDPy.*Zsd(pos_CB+3,tt) + ...
%                                                          ((-1).^(Range_i1+1)).*Par_Vehicle.Ll_DPyc.*Zsd(pos_CB+5,tt) +vel_CB_PCS(:,2);


% DamperNL.DPy.DampVel(:,1) = Zsd(pos_CB+2,tt) - Zsd(pos_P,tt) - Par_Vehicle.H_cDPy.*Zsd(pos_CB+3,tt) + ...
%                                                    ((-1).^(Range_i1+1)).*Par_Vehicle.Ll_DPyc.*Zsd(pos_CB+5,tt);

for kk = 1:1:4
    if kk==1 || kk==4
        Ll_DPyc = Par_Vehicle.Ll2+Par_Vehicle.Ll_DPyt;
    else
        Ll_DPyc = Par_Vehicle.Ll2-Par_Vehicle.Ll_DPyt;
    end
    DamperNL.DPy.DampVel(kk,1) = Zsd(pos_CB(kk)+2,tt) - Zsd(pos_P(kk),tt) - Par_Vehicle.H_cDPy*Zsd(pos_CB(kk)+3,tt) + ...
                                                       ((-1)^(Range_i1(kk)+1)) * Ll_DPyc * Zsd(pos_CB(kk)+5,tt);
end

DamperNL.DPy.SpringLength(:,1) = Zwy(pos_P,tt) - Zwy(pos_BG+2,tt) - Par_Vehicle.H_DPyt.*Zwy(pos_BG+3,tt) - ...
                                                          (-1).^(Range_i2+1).*Par_Vehicle.Ll_DPyt.*Zwy(pos_BG+5,tt);
DamperNL.DPy.Mark = abs(DamperNL.DPy.DampVel)>Par_Vehicle.C_DPy_V0;

%% ºáÏòÖ¹µ²
Range_i1 = [1,2]';
pos_CB = (pos_DOF_RV+30)*ones(2,1);
pos_BG = pos_DOF_RV+20+5.*(Range_i1-1);

DamperNL.STy.SpringLength(:,1) = Zwy(pos_CB+2,tt) - Zwy(pos_BG+2,tt) - Par_Vehicle.H_cST.*Zwy(pos_CB+3,tt) - Par_Vehicle.H_STt.*Zwy(pos_BG+3,tt) + ...
                                                         (-1).^(Range_i1+1).*Par_Vehicle.Ll_ST.*Zwy(pos_CB+5,tt);
DamperNL.STy.Mark = abs(DamperNL.STy.SpringLength)>Par_Vehicle.L0_ST;

%% È·¶¨ Cxt, Newmark.Kyx, Park.Ajz
if ~( xcs>1 && isequal(Con_Int.DamperNL.DPz.Mark(xcs-1,:)',DamperNL.DPz.Mark) && isequal(Con_Int.DamperNL.Sx.Mark(xcs-1,:)',DamperNL.Sx.Mark) ...
        && isequal(Con_Int.DamperNL.DPy.Mark(xcs-1,:)',DamperNL.DPy.Mark) )
    
    if isempty(find(DamperNL.DPz.Mark,1)) && isempty(find(DamperNL.Sx.Mark,1)) && isempty(find(DamperNL.DPy.Mark,1))
        Cxt{2,1} = Cxt{1,1};
        if strcmp(InpPar.Int_Method, 'Park')
            Park.Ajz{2,i_drtaT} = Park.Ajz{1,i_drtaT};
        elseif strcmp(InpPar.Int_Method, 'Houbolt')
            Houbolt.Kyx{2,i_drtaT} = Houbolt.Kyx{1,i_drtaT};
        end
        if strcmp(InpPar.Type_simulation, 'Preload') && xlcs<=3
            Newmark.Kyx{2,i_drtaT} = Newmark.Kyx{1,i_drtaT};
        end
    else
        tt = InpPar.N_track+InpPar.Nw*InpPar.NM_FW+InpPar.N_RV;
        Cxt{2,1} = zeros(tt, tt);
        Cxt{2,1}(1:InpPar.N_track,1:InpPar.N_track) = Cxt{1,1}(1:InpPar.N_track,1:InpPar.N_track);
        Cxt{2,1}(InpPar.N_track+1:end, InpPar.N_track+1:end) = Create_Cxt_NL(InpPar, Par_Vehicle, DamperNL, C_vehicle_v0);
        temp = false(8,3);
        temp(:,1) = DamperNL.DPz.Mark;
        temp(1:4,2) = DamperNL.Sx.Mark;
        temp(1:4,3) = DamperNL.DPy.Mark;
        
        if strcmp(InpPar.Int_Method, 'Park')
            if isequal(temp, Park.Ajz{4,i_drtaT})
                Park.Ajz{2,i_drtaT} = Park.Ajz{3,i_drtaT};
            else
                Park.Ajz{2,i_drtaT} = inv( (10/(6*drtaT))*(10/(6*drtaT))*Mxt+(10/(6*drtaT))*Cxt{2,1}+Kxt );
                Park.Ajz{3,i_drtaT} = Park.Ajz{2,i_drtaT};
                Park.Ajz{4,i_drtaT} = temp;
            end
        elseif strcmp(InpPar.Int_Method, 'Houbolt')
            if isequal(temp, Houbolt.Kyx{4,i_drtaT})
                Houbolt.Kyx{2,i_drtaT} = Houbolt.Kyx{3,i_drtaT};
            else
                Houbolt.Kyx{2,i_drtaT} = inv( Kxt + Houbolt.c0{1,i_drtaT}*Mxt + Houbolt.c1{1,i_drtaT}*Cxt{2,1} );
                Houbolt.Kyx{3,i_drtaT} = Houbolt.Kyx{2,i_drtaT};
                Houbolt.Kyx{4,i_drtaT} = temp;
            end
        end
        if strcmp(InpPar.Type_simulation, 'Preload') && xlcs<=3
            if isequal(temp, Newmark.Kyx{4,i_drtaT})
                Newmark.Kyx{2,i_drtaT} = Newmark.Kyx{3,i_drtaT};
            else
                Newmark.Kyx{2,i_drtaT} = inv( Kxt+Newmark.A1{1,i_drtaT}*Mxt+Newmark.A2{1,i_drtaT}*Cxt{2,1} );
                Newmark.Kyx{3,i_drtaT} = Newmark.Kyx{2,i_drtaT};
                Newmark.Kyx{4,i_drtaT} = temp;
            end
        end
    end
    
end
