%% 轮轨力对车辆系统做功
function [Pxt, Q_temp] = WR_Force_VehicleSys(Pxt, Zwy, xlcs, N_track, NM, Nw, Node_FW_num, num_NRC_L, num_NRC_R, ModeShape, Pjcc, Pjch, Prhxf, Br, R0, ...
               Expression_WS, Type_Side, Con_FF,  Con_FR,  Con_RF,  Con_RR, Mileage_sum_Check)

%%% 轮轨力对车辆系统刚性轮对做功
% if xlcs <= 3
if isempty(Con_FF)
    for i1 = 1:1:Nw           %%% 轮对数 1-4
        if NM > 0
            Q_temp = zeros(Node_FW_num*3,1);
        else
            Q_temp = [];
        end
        for i2 = 1:1:4        %%% 集总质量块数目
            wheelside = fix((i2+1)/2);  %%% Left->1, Right->2
            % 法向力做功
            i_contact_normal = 4*(i1-1)+i2;
            Pxt(N_track+NM*Nw+5*(i1-1)+1,1) = Pxt(N_track+NM*Nw+5*(i1-1)+1,1)-Pjcc(i_contact_normal,1);                        % Pjcc=zeros(8,1) 法向接触力垂向分量 N
            Pxt(N_track+NM*Nw+5*(i1-1)+2,1) = Pxt(N_track+NM*Nw+5*(i1-1)+2,1)-Pjch(i_contact_normal,1)*((-1)^(wheelside));     % Pjch=zeros(8,1) 法向接触力横向分量 N
            Pxt(N_track+NM*Nw+5*(i1-1)+3,1) = Pxt(N_track+NM*Nw+5*(i1-1)+3,1)-Pjcc(i_contact_normal,1)*((-1)^(wheelside))*Br;  % Br=0.7175m
            Pxt(N_track+NM*Nw+5*(i1-1)+3,1) = Pxt(N_track+NM*Nw+5*(i1-1)+3,1)+Pjch(i_contact_normal,1)*((-1)^(wheelside))*R0;  % R0 车轮名义半径  m
            Pxt(N_track+NM*Nw+5*(i1-1)+5,1) = Pxt(N_track+NM*Nw+5*(i1-1)+5,1)+Pjch(i_contact_normal,1)*Br*Zwy(N_track+NM*Nw+5*(i1-1)+5,3);
            % 蠕滑力做功 （蠕滑力坐标系作用在车轮上）
            i_contact_creep = 16*(i1-1)+4*(i2-1);
            Pxt(N_track+NM*Nw+5*(i1-1)+1,1) = Pxt(N_track+NM*Nw+5*(i1-1)+1,1)+Prhxf((i_contact_creep+4),1);                     % Prhxf 左右车轮编号及X、Y、Z三方向轮轨间蠕滑力（行）修正值分量，列代表蠕滑力和蠕滑力矩
            Pxt(N_track+NM*Nw+5*(i1-1)+2,1) = Pxt(N_track+NM*Nw+5*(i1-1)+2,1)+Prhxf((i_contact_creep+3),1);
            Pxt(N_track+NM*Nw+5*(i1-1)+3,1) = Pxt(N_track+NM*Nw+5*(i1-1)+3,1)+Prhxf((i_contact_creep+4),1)*((-1)^(wheelside))*Br;
            Pxt(N_track+NM*Nw+5*(i1-1)+3,1) = Pxt(N_track+NM*Nw+5*(i1-1)+3,1)-Prhxf((i_contact_creep+3),1)*R0;
            Pxt(N_track+NM*Nw+5*(i1-1)+4,1) = Pxt(N_track+NM*Nw+5*(i1-1)+4,1)+Prhxf((i_contact_creep+2),1)*R0;
            Pxt(N_track+NM*Nw+5*(i1-1)+4,1) = Pxt(N_track+NM*Nw+5*(i1-1)+4,1)+Prhxf((i_contact_creep+3),2);
            Pxt(N_track+NM*Nw+5*(i1-1)+5,1) = Pxt(N_track+NM*Nw+5*(i1-1)+5,1)-Prhxf((i_contact_creep+2),1)*((-1)^(wheelside))*Br;
            Pxt(N_track+NM*Nw+5*(i1-1)+5,1) = Pxt(N_track+NM*Nw+5*(i1-1)+5,1)-Prhxf((i_contact_creep+3),1)*((-1)^(wheelside))*Br*Zwy(N_track+NM*Nw+5*(i1-1)+5,3);
            Pxt(N_track+NM*Nw+5*(i1-1)+5,1) = Pxt(N_track+NM*Nw+5*(i1-1)+5,1)+Prhxf((i_contact_creep+4),2);            
            if NM > 0
                if wheelside == 1
                    num_NRC = num_NRC_L;
                else
                    num_NRC = num_NRC_R;
                end
                i_contact_normal = 4*(i1-1)+i2;
                i_contact_creep = 16*(i1-1)+4*(i2-1);
                % 纵向蠕滑力
                Q_temp(Node_FW_num*0+num_NRC,1) = Q_temp(Node_FW_num*0+num_NRC,1) + Prhxf(i_contact_creep+2,1);
                % 横向蠕滑力+法向力横向分量
                Q_temp(Node_FW_num*1+num_NRC,1) = Q_temp(Node_FW_num*1+num_NRC,1) + ...
                                               Prhxf(i_contact_creep+3,1) - Pjch(i_contact_normal,1)*(-1)^(wheelside);
                % 垂向蠕滑力+法向力垂向分量
                Q_temp(Node_FW_num*2+num_NRC,1) = Q_temp(Node_FW_num*2+num_NRC,1) + ...
                                               Prhxf(i_contact_creep+4,1) - Pjcc(i_contact_normal,1);                
            end            
        end
        if NM > 0
            Pxt(N_track+NM*(i1-1)+1:N_track+NM*i1,1) = Pxt(N_track+NM*(i1-1)+1:N_track+NM*i1,1) + ModeShape'*Q_temp;
        end
    end
    
else
    for i1 = 1:1:Nw           %%% 轮对数 1-4
        eval(['Con_str = Con_',Expression_WS{i1},';']);
        if NM > 0
            Q_temp = zeros(Node_FW_num*3,1);
        else
            Q_temp = [];
        end
        for wheelside = 1:1:2
            eval(['Normal_Force = Con_str.Normal_Force_',Type_Side{wheelside},';']);
            eval(['Con_wheel_2 = Con_str.Con_wheel_',Type_Side{wheelside},'_2;']);
            eval(['Prhxf_temp = Con_str.Prhxf_',Type_Side{wheelside},';']);
            if NM > 0
                eval(['Coor_Target_local_XOY = Con_str.Coor_Target_',Type_Side{wheelside},'_local_XOY;']);
                eval(['pos_Con_Around_Deformed_WS_XOY = Con_str.pos_Con_Around_',Type_Side{wheelside},'_Deformed_WS_XOY;']);
                eval(['Coor_Target_local_YOZ = Con_str.Coor_Target_',Type_Side{wheelside},'_local_YOZ;']);
                eval(['pos_Con_Around_Deformed_WS_YOZ = Con_str.pos_Con_Around_',Type_Side{wheelside},'_Deformed_WS_YOZ;']);
            end            
            for k = 1:1:length(Normal_Force)
                % 法向力做功
                % Through Route
                if wheelside == 1
                    if ~isempty(Con_str.profile_r_L2) && Con_str.Con_rail_L_1(k,1) > min(Con_str.profile_r_L2(:,1))
                        kk = -1;
                    else
                        kk = 1;
                    end
                else
                    kk = 1;
                end
%                 % Diverging Route
%                 if wheelside == 2
%                     if ~isempty(Con_str.profile_r_R1) && Con_str.Con_rail_R_1(k,1) < max(Con_str.profile_r_R1(:,1))
%                         kk = -1;
%                     else
%                         kk = 1;
%                     end
%                 else
%                     kk = 1;
%                 end
                Pjcc_temp = Normal_Force(k).*cos(Con_wheel_2(k,6)+kk*(-1)^(wheelside+1)*Zwy(N_track+NM*Nw+5*(i1-1)+3,4));
                Pjch_temp = kk * Normal_Force(k).*sin(Con_wheel_2(k,6)+kk*(-1)^(wheelside+1)*Zwy(N_track+NM*Nw+5*(i1-1)+3,4));
                Br_temp = abs(Con_wheel_2(k,2));
                R_temp = Con_wheel_2(k,3);
                % 法向力做功
                Pxt(N_track+NM*Nw+5*(i1-1)+1,1) = Pxt(N_track+NM*Nw+5*(i1-1)+1,1)-Pjcc_temp;
                Pxt(N_track+NM*Nw+5*(i1-1)+2,1) = Pxt(N_track+NM*Nw+5*(i1-1)+2,1)-Pjch_temp*((-1)^(wheelside));
                Pxt(N_track+NM*Nw+5*(i1-1)+3,1) = Pxt(N_track+NM*Nw+5*(i1-1)+3,1)-Pjcc_temp*((-1)^(wheelside))*Br_temp;
                Pxt(N_track+NM*Nw+5*(i1-1)+3,1) = Pxt(N_track+NM*Nw+5*(i1-1)+3,1)+Pjch_temp*((-1)^(wheelside))*R_temp;
                Pxt(N_track+NM*Nw+5*(i1-1)+5,1) = Pxt(N_track+NM*Nw+5*(i1-1)+5,1)+Pjch_temp*Br_temp*Zwy(N_track+NM*Nw+5*(i1-1)+5,3);
                % 蠕滑力做功
                Pxt(N_track+NM*Nw+5*(i1-1)+1,1) = Pxt(N_track+NM*Nw+5*(i1-1)+1,1)+Prhxf_temp(k,3);
                Pxt(N_track+NM*Nw+5*(i1-1)+2,1) = Pxt(N_track+NM*Nw+5*(i1-1)+2,1)+Prhxf_temp(k,2);
                Pxt(N_track+NM*Nw+5*(i1-1)+3,1) = Pxt(N_track+NM*Nw+5*(i1-1)+3,1)+Prhxf_temp(k,3)*(-1)^(wheelside)*Br_temp;
                Pxt(N_track+NM*Nw+5*(i1-1)+3,1) = Pxt(N_track+NM*Nw+5*(i1-1)+3,1)-Prhxf_temp(k,2)*R_temp;
                Pxt(N_track+NM*Nw+5*(i1-1)+4,1) = Pxt(N_track+NM*Nw+5*(i1-1)+4,1)+Prhxf_temp(k,1)*R_temp;
                Pxt(N_track+NM*Nw+5*(i1-1)+4,1) = Pxt(N_track+NM*Nw+5*(i1-1)+4,1)+Prhxf_temp(k,5);
                Pxt(N_track+NM*Nw+5*(i1-1)+5,1) = Pxt(N_track+NM*Nw+5*(i1-1)+5,1)-Prhxf_temp(k,1)*(-1)^(wheelside)*Br_temp;
                Pxt(N_track+NM*Nw+5*(i1-1)+5,1) = Pxt(N_track+NM*Nw+5*(i1-1)+5,1)-Prhxf_temp(k,2)*(-1)^(wheelside)*Br_temp*Zwy(N_track+NM*Nw+5*(i1-1)+5,3);
                Pxt(N_track+NM*Nw+5*(i1-1)+5,1) = Pxt(N_track+NM*Nw+5*(i1-1)+5,1)+Prhxf_temp(k,6);
                
                if NM > 0                    
                    % input_Con_Target_temp: Nz, Ny, Tx, Ty, Tz
                    input_Load_Target_temp = [Pjcc_temp, Pjch_temp, Prhxf_temp(k,:)];
                    output_Load_Around  = Shape_Isoparametric_P(Coor_Target_local_XOY(k,:), Coor_Target_local_YOZ(k,:), input_Load_Target_temp);
                    
                    % 纵向蠕滑力
                    Q_temp(Node_FW_num*0+pos_Con_Around_Deformed_WS_XOY{k,1}(:,1),1) = Q_temp(Node_FW_num*0+pos_Con_Around_Deformed_WS_XOY{k,1}(:,1),1) + ...
                                                                                       output_Load_Around(:,3);
                    % 横向蠕滑力+法向力横向分量
                    Q_temp(Node_FW_num*1+pos_Con_Around_Deformed_WS_XOY{k,1}(:,1),1) = Q_temp(Node_FW_num*1+pos_Con_Around_Deformed_WS_XOY{k,1}(:,1),1) + ...
                                                                                       output_Load_Around(:,4) - output_Load_Around(:,2).*((-1)^(wheelside));
                    % 垂向蠕滑力+法向力垂向分量
                    Q_temp(Node_FW_num*2+pos_Con_Around_Deformed_WS_YOZ{k,1}(:,1),1) = Q_temp(Node_FW_num*2+pos_Con_Around_Deformed_WS_YOZ{k,1}(:,1),1) + ...
                                                                                       output_Load_Around(:,5) - output_Load_Around(:,1);                    
                end                
            end
        end
        if NM > 0
            Pxt(N_track+NM*(i1-1)+1:N_track+NM*i1,1) = Pxt(N_track+NM*(i1-1)+1:N_track+NM*i1,1) + ModeShape'*Q_temp;
        end
    end
end


% if wheelside == 1
%     num_NRC = num_NRC_L;
% else
%     num_NRC = num_NRC_R;
% end
% % 纵向蠕滑力
% Q_temp(Node_num*0+num_NRC,1) = Q_temp(Node_num*0+num_NRC,1) + Prhxf_temp(k,1);
% % 横向蠕滑力+法向力横向分量
% Q_temp(Node_num*1+num_NRC,1) = Q_temp(Node_num*1+num_NRC,1) + Prhxf_temp(k,2) - Pjch_temp*((-1)^(wheelside));
% % 垂向蠕滑力+法向力垂向分量
% Q_temp(Node_num*2+num_NRC,1) = Q_temp(Node_num*2+num_NRC,1) + Prhxf_temp(k,3) - Pjcc_temp;

% %%% 轮轨力对车辆系统柔性轮对做功
% if NM > 0
%     for i1 = 1:1:Nw             %%% 轮对数 1-4
%         Q_temp = zeros(Node_num*3,1);
%         for i2 = 1:1:4           %%% 集总质量块数目
%             wheelside = fix((i2+1)/2);  %%% Left->1, Right->2
%             if wheelside == 1
%                 num_NRC = num_NRCL;
%             else
%                 num_NRC = num_NRCR;
%             end
%             i_contact_normal = 4*(i1-1)+i2;
%             i_contact_creep = 16*(i1-1)+4*(i2-1);
%             % 纵向蠕滑力
%             Q_temp(Node_num*0+num_NRC,1) = Q_temp(Node_num*0+num_NRC,1) + Prhxf(i_contact_creep+2,1);
%             % 横向蠕滑力+法向力横向分量
%             Q_temp(Node_num*1+num_NRC,1) = Q_temp(Node_num*1+num_NRC,1) + ...
%                                            Prhxf(i_contact_creep+3,1) - Pjch(i_contact_normal,1)*(-1)^(wheelside);
%             % 垂向蠕滑力+法向力垂向分量
%             Q_temp(Node_num*2+num_NRC,1) = Q_temp(Node_num*2+num_NRC,1) + ...
%                                            Prhxf(i_contact_creep+4,1) - Pjcc(i_contact_normal,1);
%         end
%         Pxt(N_track+NM*(i1-1)+1:N_track+NM*(i1-1)+NM,1) = ModeShape'*Q_temp;
%     end
% end