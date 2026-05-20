%% 轮轨力对车辆系统做功
function [Pxt, Q_temp] = WR_Force_VehicleSys_Rotation(InpPar, Par_Vehicle, Par_Track, Par_FW, Pxt, Zwy, Pjcc, Pjch, Prhxf, Con_WS)

Br = Par_Track.Br;
R0 = Par_Vehicle.R0;
pos_RV = InpPar.N_track+InpPar.NM_FW*InpPar.Nw;
Q_temp = zeros(InpPar.Nw*InpPar.N_ConPatch, 3);

%%% 轮轨力对车辆系统刚性轮对做功
if ~isfield(Con_WS.FF, 'Normal_Force')
    
    for i1 = 1:1:InpPar.Nw           %%% 轮对数 1-4
%         if InpPar.NM_FW > 0
%             len = size(InpPar.ModeShape.Tread_Mid,1);
%             Q_temp = zeros(len,1);
%         end
        
        for i2 = 1:1:InpPar.N_ConPatch        %%% 集总质量块数目
            if strcmp(InpPar.Exp_DummyRail_WheelSide{1,i2},'L')
                wheelside = 1;
                T2 = 'L';
            elseif strcmp(InpPar.Exp_DummyRail_WheelSide{1,i2},'R')
                wheelside = 2;
                T2 = 'R';
            end            
            % 法向力做功
            i_contact = InpPar.N_ConPatch*(i1-1)+i2;
            Pxt(pos_RV+5*(i1-1)+1,1) = Pxt(pos_RV+5*(i1-1)+1,1)+Pjcc(i_contact,1);	% Pjcc=zeros(8,1) 法向接触力垂向分量 N
            Pxt(pos_RV+5*(i1-1)+2,1) = Pxt(pos_RV+5*(i1-1)+2,1)+Pjch(i_contact,1);	% Pjch=zeros(8,1) 法向接触力横向分量 N
            Pxt(pos_RV+5*(i1-1)+3,1) = Pxt(pos_RV+5*(i1-1)+3,1)+Pjcc(i_contact,1)*((-1)^wheelside)*Br;  % Br=0.7175m
            Pxt(pos_RV+5*(i1-1)+3,1) = Pxt(pos_RV+5*(i1-1)+3,1)-Pjch(i_contact,1)*R0;  % R0 车轮名义半径  m
            Pxt(pos_RV+5*(i1-1)+5,1) = Pxt(pos_RV+5*(i1-1)+5,1)-Pjch(i_contact,1)*Br*Zwy(pos_RV+5*(i1-1)+5,4)*((-1)^wheelside); 
            % 蠕滑力做功 （蠕滑力坐标系作用在车轮上）
            Pxt(pos_RV+5*(i1-1)+1,1) = Pxt(pos_RV+5*(i1-1)+1,1)+Prhxf(i_contact,3);                     % Prhxf 左右车轮编号及X、Y、Z三方向轮轨间蠕滑力（行）修正值分量，列代表蠕滑力和蠕滑力矩
            Pxt(pos_RV+5*(i1-1)+2,1) = Pxt(pos_RV+5*(i1-1)+2,1)+Prhxf(i_contact,2);
            Pxt(pos_RV+5*(i1-1)+3,1) = Pxt(pos_RV+5*(i1-1)+3,1)+Prhxf(i_contact,3)*((-1)^wheelside)*Br;
            Pxt(pos_RV+5*(i1-1)+3,1) = Pxt(pos_RV+5*(i1-1)+3,1)-Prhxf(i_contact,2)*R0;
            Pxt(pos_RV+5*(i1-1)+4,1) = Pxt(pos_RV+5*(i1-1)+4,1)+Prhxf(i_contact,1)*R0;
            Pxt(pos_RV+5*(i1-1)+4,1) = Pxt(pos_RV+5*(i1-1)+4,1)+Prhxf(i_contact,5);
            Pxt(pos_RV+5*(i1-1)+5,1) = Pxt(pos_RV+5*(i1-1)+5,1)-Prhxf(i_contact,1)*Br*((-1)^wheelside);
            Pxt(pos_RV+5*(i1-1)+5,1) = Pxt(pos_RV+5*(i1-1)+5,1)-Prhxf(i_contact,2)*Br*Zwy(pos_RV+5*(i1-1)+5,4)*((-1)^wheelside);
            Pxt(pos_RV+5*(i1-1)+5,1) = Pxt(pos_RV+5*(i1-1)+5,1)+Prhxf(i_contact,6);
            % FW
            if InpPar.NM_FW > 0
%                 Target_Node = Par_FW.Tread_Mid;
%                 if strcmp(InpPar.Exp_DummyRail_WheelSide{1,i2},'L')
%                     Target_Row = find(Target_Node(:,2+1) == -0.7465);
%                 else
%                     Target_Row = find(Target_Node(:,2+1) == 0.7465);
%                 end
%                 % 纵向蠕滑力
%                 Q_temp(3*(Target_Row-1)+1,1) = Q_temp(3*(Target_Row-1)+1,1) + Prhxf(i_contact,1);
%                 % 横向蠕滑力+法向力横向分量
%                 Q_temp(3*(Target_Row-1)+2,1) = Q_temp(3*(Target_Row-1)+2,1) + Prhxf(i_contact,2) + Pjch(i_contact,1);
%                 % 垂向蠕滑力+法向力垂向分量
%                 Q_temp(3*(Target_Row-1)+3,1) = Q_temp(3*(Target_Row-1)+3,1) + Prhxf(i_contact,3) + Pjcc(i_contact,1);
                T3 = 'Mid';
                if strcmp(InpPar.Exp_DummyRail_WheelSide{1,i2},'L')
                    ConPos_Y = -0.7465;
                    % For example, replace code such as min(find(A)) with find(A,1). Similarly, replace max(find(A)) with find(A, 1, 'last').
                    m = find(ConPos_Y-InpPar.Pos_Node.Tread.(T2).(T3)(:,3)>=0, 1, 'last');
                else
                    ConPos_Y = 0.7465;
                    % For example, replace code such as min(find(A)) with find(A,1). Similarly, replace max(find(A)) with find(A, 1, 'last').
                    m = find(ConPos_Y-InpPar.Pos_Node.Tread.(T2).(T3)(:,3)>0, 1, 'last');
                end
                SubMat_F = Par_FW.Matrix_F_SumAll.(T2).Front.Matrix_F{m,1};
                Q_temp(i_contact,:) = [Prhxf(i_contact,1), Prhxf(i_contact,2)+Pjch(i_contact,1), Prhxf(i_contact,3)+Pjcc(i_contact,1)];
                pos_NM_FW = InpPar.N_track+InpPar.NM_FW*(i1-1)+(1:1:InpPar.NM_FW);
                Pxt(pos_NM_FW,1) = Pxt(pos_NM_FW,1) + SubMat_F*Q_temp(i_contact,:)';
            end
        end
        
        if InpPar.NM_FW > 0
            pos_NM_FW = InpPar.N_track+InpPar.NM_FW*(i1-1)+(1:1:InpPar.NM_FW);
            Pxt(pos_NM_FW,1) = Pxt(pos_NM_FW,1) + (InpPar.Vlc/Par_Vehicle.R0)^2*Par_FW.Matrix_L;
%             Pxt(pos_NM_FW,1) = Pxt(pos_NM_FW,1) + InpPar.ModeShape.Tread_Mid'*Q_temp;
        end
    end            % End of WS
    
else
        
    for i1 = 1:1:InpPar.Nw           %%% 轮对数 1-4
        Con_str = Con_WS.(InpPar.Exp_WS{i1});
%         if InpPar.NM_FW > 0
%             len = size(InpPar.ModeShape.Tread_Mid,1);
%             Q_temp = zeros(len,1);
%         else
%             Q_temp = [];
%         end
        
        for wheelside = 1:1:2       % Left / Right
            T2 = InpPar.Type_Side{wheelside};
            Normal_Force = Con_str.Normal_Force.(T2);
            Con_wheel_2 = Con_str.Con_wheel_2.(T2);
            Prhxf_temp = Con_str.Prhxf_T.(T2);
%             if InpPar.NM_FW > 0
%                 Coor_Target_local_XOY = Con_str.(['Coor_Target_',InpPar.Type_Side{wheelside},'_local_XOY']);
%                 pos_Con_Around_Deformed_WS_XOY = Con_str.(['pos_Con_Around_',InpPar.Type_Side{wheelside},'_Deformed_WS_XOY']);
%                 Coor_Target_local_YOZ = Con_str.(['Coor_Target_',InpPar.Type_Side{wheelside},'_local_YOZ']);
%                 pos_Con_Around_Deformed_WS_YOZ = Con_str.(['pos_Con_Around_',InpPar.Type_Side{wheelside},'_Deformed_WS_YOZ']);
%             end
            for k = 1:1:size(Normal_Force,1)        % ConPatch
                Br_temp = abs(Con_wheel_2(k,2));
                R_temp = Con_wheel_2(k,3);
                % 法向力做功
                Pxt(pos_RV+5*(i1-1)+1,1) = Pxt(pos_RV+5*(i1-1)+1,1)+Normal_Force(k,3);
                Pxt(pos_RV+5*(i1-1)+2,1) = Pxt(pos_RV+5*(i1-1)+2,1)+Normal_Force(k,2);
                Pxt(pos_RV+5*(i1-1)+3,1) = Pxt(pos_RV+5*(i1-1)+3,1)+Normal_Force(k,3)*((-1)^wheelside)*Br_temp;
                Pxt(pos_RV+5*(i1-1)+3,1) = Pxt(pos_RV+5*(i1-1)+3,1)-Normal_Force(k,2)*R_temp;
                Pxt(pos_RV+5*(i1-1)+5,1) = Pxt(pos_RV+5*(i1-1)+5,1)-Normal_Force(k,2)*Br_temp*Zwy(pos_RV+5*(i1-1)+5,4)*((-1)^wheelside);
                % 蠕滑力做功
                Pxt(pos_RV+5*(i1-1)+1,1) = Pxt(pos_RV+5*(i1-1)+1,1)+Prhxf_temp(k,3);
                Pxt(pos_RV+5*(i1-1)+2,1) = Pxt(pos_RV+5*(i1-1)+2,1)+Prhxf_temp(k,2);
                Pxt(pos_RV+5*(i1-1)+3,1) = Pxt(pos_RV+5*(i1-1)+3,1)+Prhxf_temp(k,3)*((-1)^wheelside)*Br_temp;
                Pxt(pos_RV+5*(i1-1)+3,1) = Pxt(pos_RV+5*(i1-1)+3,1)-Prhxf_temp(k,2)*R_temp;
                Pxt(pos_RV+5*(i1-1)+4,1) = Pxt(pos_RV+5*(i1-1)+4,1)+Prhxf_temp(k,1)*R_temp;
                Pxt(pos_RV+5*(i1-1)+4,1) = Pxt(pos_RV+5*(i1-1)+4,1)+Prhxf_temp(k,5);
                Pxt(pos_RV+5*(i1-1)+5,1) = Pxt(pos_RV+5*(i1-1)+5,1)-Prhxf_temp(k,1)*Br_temp*((-1)^wheelside);
                Pxt(pos_RV+5*(i1-1)+5,1) = Pxt(pos_RV+5*(i1-1)+5,1)-Prhxf_temp(k,2)*Br_temp*Zwy(pos_RV+5*(i1-1)+5,4)*((-1)^wheelside);
                Pxt(pos_RV+5*(i1-1)+5,1) = Pxt(pos_RV+5*(i1-1)+5,1)+Prhxf_temp(k,6);
                % FW
                if InpPar.NM_FW > 0
                    ConPos_Y = Con_wheel_2(k,2);
                    if Con_wheel_2(k,1)>=0
                        T3 = 'Front';
                        m = find(ConPos_Y-InpPar.Pos_Node.Tread.(T2).(T3)(:,3)>=0, 1, 'last');
                    else
                        T3 = 'Rear';
                        m = find(ConPos_Y-InpPar.Pos_Node.Tread.(T2).(T3)(:,3)>0, 1, 'last');
                    end
%                     m = find(ConPos_Y-InpPar.Pos_Node.Tread.(T2).(T3)(:,3)>=0, 1, 'last');
                    SubMat_F = Par_FW.Matrix_F_SumAll.(T2).(T3).Matrix_F{m,1};
                    Q_temp_k = [Prhxf_temp(k,1), Prhxf_temp(k,2)+Normal_Force(k,2), Prhxf_temp(k,3)+Normal_Force(k,3)];
                    pos_NM_FW = InpPar.N_track+InpPar.NM_FW*(i1-1)+(1:1:InpPar.NM_FW);
                    Pxt(pos_NM_FW,1) = Pxt(pos_NM_FW,1) + SubMat_F*Q_temp_k';
                    i_contact = InpPar.N_ConPatch*(i1-1)+Normal_Force(k,4);
                    Q_temp(i_contact,:) = Q_temp(i_contact,:)+Q_temp_k;
%                     % 纵向蠕滑力
%                     FX = Prhxf_temp(k,1) * Con_str.ShapeFun_XOY.(InpPar.Type_Side{wheelside}){k,1};
%                     % 横向蠕滑力+法向力横向分量
%                     FY = (Prhxf_temp(k,2) + Normal_Force(k,2)) * Con_str.ShapeFun_XOY.(InpPar.Type_Side{wheelside}){k,1};
%                     % 垂向蠕滑力+法向力垂向分量
%                     FZ = (Prhxf_temp(k,3) + Normal_Force(k,3)) * Con_str.ShapeFun_Z.(InpPar.Type_Side{wheelside}){k,1};  
%                     Q_temp = [FX'; FY'; FZ'];
%                     DOF_pos = [Con_str.DOF_pos_FW.(InpPar.Type_Side{wheelside}).Node_Around_XOY{k,1}(:,1); 
%                                           Con_str.DOF_pos_FW.(InpPar.Type_Side{wheelside}).Node_Around_XOY{k,1}(:,2);
%                                           Con_str.DOF_pos_FW.(InpPar.Type_Side{wheelside}).Node_Around_Z{k,1}(:,3)];                                          
%                     Pxt(InpPar.N_track+InpPar.NM_FW*(i1-1)+1:InpPar.N_track+InpPar.NM_FW*i1,1) = Pxt(InpPar.N_track+InpPar.NM_FW*(i1-1)+1:InpPar.N_track+InpPar.NM_FW*i1,1) + InpPar.ModeShape.FW(DOF_pos,:)' * Q_temp;                    
                end
            end    % End of ConPatch
            
        end        % End of wheelside
        
        if InpPar.NM_FW > 0
            pos_NM_FW = InpPar.N_track+InpPar.NM_FW*(i1-1)+(1:1:InpPar.NM_FW);
            Pxt(pos_NM_FW,1) = Pxt(pos_NM_FW,1) + (InpPar.Vlc/Par_Vehicle.R0)^2*Par_FW.Matrix_L;
        end
    end            % End of WS
    
end
