%% 轮轨力对车辆系统做功
function [Pxt, Q_temp] = WR_Force_VehicleSys(InpPar, Par_Vehicle, Par_Track, Par_FW, Pxt, Zwy, Pjcc, Pjch, Prhxf, Con_WS)

% global InpPar.N_track InpPar.Nw InpPar.N_ConPatch Par_Track Par_Vehicle
% global InpPar.NM_FW Par_FW InpPar.ModeShape 
% global InpPar.Exp_WS InpPar.Exp_DummyRail_WheelSide InpPar.Type_Side

Br = Par_Track.Br;
R0 = Par_Vehicle.R0;

Q_temp = [];

%%% 轮轨力对车辆系统刚性轮对做功
if ~isfield(Con_WS.FF, 'Normal_Force')
    
    for i1 = 1:1:InpPar.Nw           %%% 轮对数 1-4
        if InpPar.NM_FW > 0
            len = size(InpPar.ModeShape.Tread_Mid,1);
            Q_temp = zeros(len,1);
        end
        
        for i2 = 1:1:InpPar.N_ConPatch        %%% 集总质量块数目
            if strcmp(InpPar.Exp_DummyRail_WheelSide{1,i2},'L')
                wheelside = 1;
            elseif strcmp(InpPar.Exp_DummyRail_WheelSide{1,i2},'R')
                wheelside = 2;
            end
            
            % 法向力做功
            pos = InpPar.N_track+InpPar.NM_FW*InpPar.Nw;
            i_contact = InpPar.N_ConPatch*(i1-1)+i2;
            Pxt(pos+5*(i1-1)+1,1) = Pxt(pos+5*(i1-1)+1,1)+Pjcc(i_contact,1);	% Pjcc=zeros(8,1) 法向接触力垂向分量 N
            Pxt(pos+5*(i1-1)+2,1) = Pxt(pos+5*(i1-1)+2,1)+Pjch(i_contact,1);	% Pjch=zeros(8,1) 法向接触力横向分量 N
            Pxt(pos+5*(i1-1)+3,1) = Pxt(pos+5*(i1-1)+3,1)+Pjcc(i_contact,1)*((-1)^wheelside)*Br;  % Br=0.7175m
            Pxt(pos+5*(i1-1)+3,1) = Pxt(pos+5*(i1-1)+3,1)-Pjch(i_contact,1)*R0;  % R0 车轮名义半径  m
%             Pxt(pos+5*(i1-1)+5,1) = Pxt(pos+5*(i1-1)+5,1)-Pjch(i_contact,1)*Br*Zwy(pos+5*(i1-1)+5,4)*((-1)^wheelside); 
            Pxt(pos+5*(i1-1)+5,1) = Pxt(pos+5*(i1-1)+5,1)-Pjch(i_contact,1)*Br*Zwy(pos+5*(i1-1)+5,4)*((-1)^wheelside); 
            % 蠕滑力做功 （蠕滑力坐标系作用在车轮上）
            Pxt(pos+5*(i1-1)+1,1) = Pxt(pos+5*(i1-1)+1,1)+Prhxf(i_contact,3);                     % Prhxf 左右车轮编号及X、Y、Z三方向轮轨间蠕滑力（行）修正值分量，列代表蠕滑力和蠕滑力矩
            Pxt(pos+5*(i1-1)+2,1) = Pxt(pos+5*(i1-1)+2,1)+Prhxf(i_contact,2);
            Pxt(pos+5*(i1-1)+3,1) = Pxt(pos+5*(i1-1)+3,1)+Prhxf(i_contact,3)*((-1)^wheelside)*Br;
            Pxt(pos+5*(i1-1)+3,1) = Pxt(pos+5*(i1-1)+3,1)-Prhxf(i_contact,2)*R0;
            Pxt(pos+5*(i1-1)+4,1) = Pxt(pos+5*(i1-1)+4,1)+Prhxf(i_contact,1)*R0;
            Pxt(pos+5*(i1-1)+4,1) = Pxt(pos+5*(i1-1)+4,1)+Prhxf(i_contact,5);
            Pxt(pos+5*(i1-1)+5,1) = Pxt(pos+5*(i1-1)+5,1)-Prhxf(i_contact,1)*Br*((-1)^wheelside);
%             Pxt(pos+5*(i1-1)+5,1) = Pxt(pos+5*(i1-1)+5,1)-Prhxf(i_contact,2)*Br*Zwy(pos+5*(i1-1)+5,4)*((-1)^wheelside);
            Pxt(pos+5*(i1-1)+5,1) = Pxt(pos+5*(i1-1)+5,1)-Prhxf(i_contact,2)*Br*Zwy(pos+5*(i1-1)+5,4)*((-1)^wheelside);
            Pxt(pos+5*(i1-1)+5,1) = Pxt(pos+5*(i1-1)+5,1)+Prhxf(i_contact,6);
            
            if InpPar.NM_FW > 0
                Target_Node = Par_FW.Tread_Mid;
                if strcmp(InpPar.Exp_DummyRail_WheelSide{1,i2},'L')
                    Target_Row = find(Target_Node(:,2+1) == -0.7465);
                else
                    Target_Row = find(Target_Node(:,2+1) == 0.7465);
                end
                % 纵向蠕滑力
                Q_temp(3*(Target_Row-1)+1,1) = Q_temp(3*(Target_Row-1)+1,1) + Prhxf(i_contact,1);
                % 横向蠕滑力+法向力横向分量
                Q_temp(3*(Target_Row-1)+2,1) = Q_temp(3*(Target_Row-1)+2,1) + Prhxf(i_contact,2) + Pjch(i_contact,1);
                % 垂向蠕滑力+法向力垂向分量
                Q_temp(3*(Target_Row-1)+3,1) = Q_temp(3*(Target_Row-1)+3,1) + Prhxf(i_contact,3) + Pjcc(i_contact,1);
            end
        end
        
        if InpPar.NM_FW > 0
            Pxt(InpPar.N_track+InpPar.NM_FW*(i1-1)+1:InpPar.N_track+InpPar.NM_FW*i1,1) = Pxt(InpPar.N_track+InpPar.NM_FW*(i1-1)+1:InpPar.N_track+InpPar.NM_FW*i1,1) + InpPar.ModeShape.Tread_Mid'*Q_temp;
        end
        
    end
    
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
            Normal_Force = Con_str.Normal_Force.(InpPar.Type_Side{wheelside});
            Con_wheel_2 = Con_str.Con_wheel_2.(InpPar.Type_Side{wheelside});
            Prhxf_temp = Con_str.Prhxf_T.(InpPar.Type_Side{wheelside});
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
                Pxt(pos+5*(i1-1)+1,1) = Pxt(pos+5*(i1-1)+1,1)+Normal_Force(k,3);
                Pxt(pos+5*(i1-1)+2,1) = Pxt(pos+5*(i1-1)+2,1)+Normal_Force(k,2);
                Pxt(pos+5*(i1-1)+3,1) = Pxt(pos+5*(i1-1)+3,1)+Normal_Force(k,3)*((-1)^wheelside)*Br_temp;
                Pxt(pos+5*(i1-1)+3,1) = Pxt(pos+5*(i1-1)+3,1)-Normal_Force(k,2)*R_temp;
%                 Pxt(pos+5*(i1-1)+5,1) = Pxt(pos+5*(i1-1)+5,1)-Normal_Force(k,2)*Br_temp*Zwy(pos+5*(i1-1)+5,4)*((-1)^wheelside);
                Pxt(pos+5*(i1-1)+5,1) = Pxt(pos+5*(i1-1)+5,1)-Normal_Force(k,2)*Br_temp*Zwy(pos+5*(i1-1)+5,4)*((-1)^wheelside);
                % 蠕滑力做功
                Pxt(pos+5*(i1-1)+1,1) = Pxt(pos+5*(i1-1)+1,1)+Prhxf_temp(k,3);
                Pxt(pos+5*(i1-1)+2,1) = Pxt(pos+5*(i1-1)+2,1)+Prhxf_temp(k,2);
                Pxt(pos+5*(i1-1)+3,1) = Pxt(pos+5*(i1-1)+3,1)+Prhxf_temp(k,3)*((-1)^wheelside)*Br_temp;
                Pxt(pos+5*(i1-1)+3,1) = Pxt(pos+5*(i1-1)+3,1)-Prhxf_temp(k,2)*R_temp;
                Pxt(pos+5*(i1-1)+4,1) = Pxt(pos+5*(i1-1)+4,1)+Prhxf_temp(k,1)*R_temp;
                Pxt(pos+5*(i1-1)+4,1) = Pxt(pos+5*(i1-1)+4,1)+Prhxf_temp(k,5);
                Pxt(pos+5*(i1-1)+5,1) = Pxt(pos+5*(i1-1)+5,1)-Prhxf_temp(k,1)*Br_temp*((-1)^wheelside);
%                 Pxt(pos+5*(i1-1)+5,1) = Pxt(pos+5*(i1-1)+5,1)-Prhxf_temp(k,2)*Br_temp*Zwy(pos+5*(i1-1)+5,4)*((-1)^wheelside);
                Pxt(pos+5*(i1-1)+5,1) = Pxt(pos+5*(i1-1)+5,1)-Prhxf_temp(k,2)*Br_temp*Zwy(pos+5*(i1-1)+5,4)*((-1)^wheelside);
                Pxt(pos+5*(i1-1)+5,1) = Pxt(pos+5*(i1-1)+5,1)+Prhxf_temp(k,6);
                
                if InpPar.NM_FW > 0
                    % 纵向蠕滑力
                    FX = Prhxf_temp(k,1) * Con_str.ShapeFun_XOY.(InpPar.Type_Side{wheelside}){k,1};
                    % 横向蠕滑力+法向力横向分量
                    FY = (Prhxf_temp(k,2) + Normal_Force(k,2)) * Con_str.ShapeFun_XOY.(InpPar.Type_Side{wheelside}){k,1};
                    % 垂向蠕滑力+法向力垂向分量
                    FZ = (Prhxf_temp(k,3) + Normal_Force(k,3)) * Con_str.ShapeFun_Z.(InpPar.Type_Side{wheelside}){k,1};  
                    
                    Q_temp = [FX'; FY'; FZ'];
                    DOF_pos = [Con_str.DOF_pos_FW.(InpPar.Type_Side{wheelside}).Node_Around_XOY{k,1}(:,1); 
                                          Con_str.DOF_pos_FW.(InpPar.Type_Side{wheelside}).Node_Around_XOY{k,1}(:,2);
                                          Con_str.DOF_pos_FW.(InpPar.Type_Side{wheelside}).Node_Around_Z{k,1}(:,3)];                                          
                    Pxt(InpPar.N_track+InpPar.NM_FW*(i1-1)+1:InpPar.N_track+InpPar.NM_FW*i1,1) = Pxt(InpPar.N_track+InpPar.NM_FW*(i1-1)+1:InpPar.N_track+InpPar.NM_FW*i1,1) + InpPar.ModeShape.FW(DOF_pos,:)' * Q_temp;                    
                end
                
            end    % End of ConPatch
            
        end        % End of wheelside

    end            % End of WS
    
end
