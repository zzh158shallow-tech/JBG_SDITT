%% 轮轨力对车辆系统做功
function [Pxt, Q_temp] = WR_Force_VehicleSys_RotationIII(InpPar, Par_Vehicle, Par_Track, Par_FW, Pxt, Zwy, Pjcc, Pjch, Prhxf, Con_WS)

% WR_Force_VehicleSys_RotationII: 根据厄米特形函数，三向轮轨力只分配到踏面平面内的两个节点处
% WR_Force_VehicleSys_RotationIII: 根据二维形函数，纵横向轮轨力分配到单元表面四个节点；根据一维形函数，垂向轮轨力分配到踏面两个节点

Br = Par_Track.Br;
R0 = Par_Vehicle.R0;
pos_RV = InpPar.N_track+InpPar.NM_FW*InpPar.Nw;
Q_temp = struct;

%%% 轮轨力对车辆系统刚性轮对做功
if ~isfield(Con_WS.FF, 'Normal_Force')
    
    for i1 = 1:1:InpPar.Nw           %%% 轮对数 1-4 
        pos_NM_FW = InpPar.N_track+InpPar.NM_FW*(i1-1)+(1:1:InpPar.NM_FW);        
        if InpPar.NM_FW > 0
            Pxt(pos_NM_FW,1) = Pxt(pos_NM_FW,1) + (InpPar.Vlc/Par_Vehicle.R0)^2*Par_FW.Matrix_L;
        end        
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
                % Rotation-III
                ConPos_Y = 0.753-0.753*2*(strcmp(InpPar.Exp_DummyRail_WheelSide{1,i2},'L'));
                m = find(ConPos_Y-InpPar.Pos_Node.Tread.(T2).Mid(:,3)>=0, 1, 'last');
                Length_All = diff(InpPar.Pos_Node.Tread.(T2).Mid(:,3));
                x = ConPos_Y - InpPar.Pos_Node.Tread.(T2).Mid(m,3);
                a = Length_All(m,1);
                sigma = (x/a);
                ShapeFunction_temp = [(1-3*sigma^2+2*sigma^3), sigma^2*(3-2*sigma)];
                T = ['Tread_', T2, '_Mid'];
                pos_DOF_Load = [Par_FW.DOF_pos.(T)(m,:)'; Par_FW.DOF_pos.(T)(m+1,:)'];
                Q_temp_k = [Prhxf(i_contact,1); Prhxf(i_contact,2)+Pjch(i_contact,1); Prhxf(i_contact,3)+Pjcc(i_contact,1)];
                Q_temp.(T2) = [Q_temp_k*ShapeFunction_temp(1); Q_temp_k*ShapeFunction_temp(2)];
                Pxt(pos_NM_FW,1) = Pxt(pos_NM_FW,1) + InpPar.ModeShape.FW(pos_DOF_Load,:)'*Q_temp.(T2);

                % Rotation-III
%                 if strcmp(InpPar.Exp_DummyRail_WheelSide{1,i2},'L')
%                     ConPos_Y = -0.7465;
%                 else
%                     ConPos_Y = 0.7465;
%                 end
%                 m = find(ConPos_Y-InpPar.Pos_Node.Tread.(T2).Mid(:,3)>=0, 1, 'last');
%                 Length_All = diff(InpPar.Pos_Node.Tread.(T2).Mid(:,3));
%                 x = ConPos_Y - InpPar.Pos_Node.Tread.(T2).Mid(m,3);
%                 a = Length_All(m,1);
%                 sigma = (x/a);
%                 ShapeFunction_temp = [(1-3*sigma^2+2*sigma^3), sigma^2*(3-2*sigma)];
%                 T = ['Tread_', T2, '_Mid'];
%                 pos_DOF_Load = [Par_FW.DOF_pos.(T)(m,:)'; Par_FW.DOF_pos.(T)(m+1,:)'];
%                 Q_temp_k = [Prhxf(i_contact,1); Prhxf(i_contact,2)+Pjch(i_contact,1); Prhxf(i_contact,3)+Pjcc(i_contact,1)];
%                 Q_temp_Node = [Q_temp_k*ShapeFunction_temp(1); Q_temp_k*ShapeFunction_temp(2)];
%                 Pxt(pos_NM_FW,1) = Pxt(pos_NM_FW,1) + InpPar.ModeShape.FW(pos_DOF_Load,:)'*Q_temp_Node;
%                 Q_temp(i_contact,:) = Q_temp(i_contact,:)+Q_temp_k';

                % Rotation-I
%                 T3 = 'Mid';
%                 if strcmp(InpPar.Exp_DummyRail_WheelSide{1,i2},'L')
%                     ConPos_Y = -0.7465;
%                     % For example, replace code such as min(find(A)) with find(A,1). Similarly, replace max(find(A)) with find(A, 1, 'last').
%                     m = find(ConPos_Y-InpPar.Pos_Node.Tread.(T2).(T3)(:,3)>=0, 1, 'last');
%                 else
%                     ConPos_Y = 0.7465;
%                     % For example, replace code such as min(find(A)) with find(A,1). Similarly, replace max(find(A)) with find(A, 1, 'last').
%                     m = find(ConPos_Y-InpPar.Pos_Node.Tread.(T2).(T3)(:,3)>0, 1, 'last');
%                 end
%                 SubMat_F = Par_FW.Matrix_F_SumAll.(T2).Front.Matrix_F{m,1};
%                 Q_temp(i_contact,:) = [Prhxf(i_contact,1), Prhxf(i_contact,2)+Pjch(i_contact,1), Prhxf(i_contact,3)+Pjcc(i_contact,1)];
%                 pos_NM_FW = InpPar.N_track+InpPar.NM_FW*(i1-1)+(1:1:InpPar.NM_FW);
%                 Pxt(pos_NM_FW,1) = Pxt(pos_NM_FW,1) + SubMat_F*Q_temp(i_contact,:)';

                % VSD
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
            end
        end

    end            % End of WS
    
else
        
    for i1 = 1:1:InpPar.Nw           %%% 轮对数 1-4
        Con_str = Con_WS.(InpPar.Exp_WS{i1});
        pos_NM_FW = InpPar.N_track+InpPar.NM_FW*(i1-1)+(1:1:InpPar.NM_FW);        
        if InpPar.NM_FW > 0
            Pxt(pos_NM_FW,1) = Pxt(pos_NM_FW,1) + (InpPar.Vlc/Par_Vehicle.R0)^2*Par_FW.Matrix_L;
        end
        
        for wheelside = 1:1:2       % Left / Right
            T2 = InpPar.Type_Side{wheelside};
            Normal_Force = Con_str.Normal_Force.(T2);
            Con_wheel_2 = Con_str.Con_wheel_2.(T2);
            Prhxf_temp = Con_str.Prhxf_T.(T2);
            % ConPatch
            for k = 1:1:size(Normal_Force,1)        
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
                Pxt(pos_RV+5*(i1-1)+5,1) = Pxt(pos_RV+5*(i1-1)+5,1)-Prhxf_temp(k,1)*Br_temp*((-1)^wheelside);   % !
                Pxt(pos_RV+5*(i1-1)+5,1) = Pxt(pos_RV+5*(i1-1)+5,1)-Prhxf_temp(k,2)*Br_temp*Zwy(pos_RV+5*(i1-1)+5,4)*((-1)^wheelside);
                Pxt(pos_RV+5*(i1-1)+5,1) = Pxt(pos_RV+5*(i1-1)+5,1)+Prhxf_temp(k,6);                
                % FW
                if InpPar.NM_FW > 0
                    % Rotation-III
                    Q_temp_k = [Prhxf_temp(k,1); Prhxf_temp(k,2)+Normal_Force(k,2); Prhxf_temp(k,3)+Normal_Force(k,3)];
                    % 纵向蠕滑力
                    FX = Q_temp_k(1) * Con_str.ShapeFun_FW.(T2).XOY{k,1};
                    % 横向蠕滑力+法向力横向分量
                    FY = Q_temp_k(2) * Con_str.ShapeFun_FW.(T2).XOY{k,1};
                    % 垂向蠕滑力+法向力垂向分量
                    FZ = Q_temp_k(3) * Con_str.ShapeFun_FW.(T2).Z{k,1};
                    Q_temp.(T2)(:,k) = [FX'; FY'; FZ'];
                    DOF_pos = [Con_str.DOF_pos_FW.(T2).Node_Around_XOY{k,1}(:,1); 
                                        Con_str.DOF_pos_FW.(T2).Node_Around_XOY{k,1}(:,2);
                                        Con_str.DOF_pos_FW.(T2).Node_Around_Z{k,1}(:,3)];
                    Pxt(pos_NM_FW,1) = Pxt(pos_NM_FW,1) + InpPar.ModeShape.FW(DOF_pos,:)' * Q_temp.(T2)(:,k);      
                end
            end    % End of ConPatch            
        end        % End of wheelside

    end            % End of WS
    
end
