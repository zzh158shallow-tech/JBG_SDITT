%% 计算车轮下钢轨的位移、速度、加速度
function [Dis_Rail, Vel_Rail, DynStatus_Rail] = RailDyn_ModalFT_P1(Zwy, Zsd, Zjsd, NM_FT_start, ShapeFunction)

global Nw N_ConPatch N_Node Pos_Node ModeShape Expression_WS Type_Rail j1 Distance_Vehicle

Dis_Rail = zeros(Nw*N_ConPatch,3);      % 钢轨接触点处 X、Y、Z 方向位移（列） m
Vel_Rail = zeros(Nw*N_ConPatch,3);      % 钢轨接触点处 X、Y、Z 方向（列）速度 m/s
% Acc_Rail = zeros(Nw*N_ConPatch,3);      % 钢轨接触点处 X、Y、Z 方向（列）加速度 m/s^2

%% 获取各钢轨的动力学参数 DynStatus_Rail N_Node
col = 1;
for i = 1:1:N_ConPatch
    if i == 1
        wheelside = 1;
    elseif i >= 2
        wheelside = 2;
    end
    m_NM = NM_FT_start(wheelside)+1;
    n_NM = NM_FT_start(wheelside+1);
    ModeShape_Rail_temp = eval(['ModeShape.', Type_Rail{i,col}]);
    Dis_temp = ModeShape_Rail_temp * Zwy(m_NM:n_NM,4);
    Vel_temp = ModeShape_Rail_temp * Zsd(m_NM:n_NM,4);
%     Acc_temp = ModeShape_Rail_temp * Zjsd(m_NM:n_NM,4);
    
    j_max = eval(['N_Node.', Type_Rail{i,col}]);
    eval(['DynStatus_Rail.',Type_Rail{i,col},'_Dis = zeros(j_max,6);']);
    eval(['DynStatus_Rail.',Type_Rail{i,col},'_Vel = zeros(j_max,6);']);
%     eval(['DynStatus_Rail.',Type_Rail{i,col},'_Acc = zeros(j_max,6);']);
    for j = 1:1:j_max
        eval(['DynStatus_Rail.',Type_Rail{i,col},'_Dis(j,:) = Dis_temp((j-1)*6+1:j*6,1)'';']);
        eval(['DynStatus_Rail.',Type_Rail{i,col},'_Vel(j,:) = Vel_temp((j-1)*6+1:j*6,1)'';']);
%         eval(['DynStatus_Rail.',Type_Rail{i,col},'_Acc(j,:) = Acc_temp((j-1)*6+1:j*6,1)'';']);
    end
end

col = 2;
for i = 1:1:3
    if i == 1
        wheelside = 1;
    elseif i >= 2
        wheelside = 2;
    end
    m_NM = NM_FT_start(wheelside)+1;
    n_NM = NM_FT_start(wheelside+1);
    ModeShape_Rail_temp = eval(['ModeShape.', Type_Rail{i,col}]);
    Dis_temp = ModeShape_Rail_temp * Zwy(m_NM:n_NM,4);
    Vel_temp = ModeShape_Rail_temp * Zsd(m_NM:n_NM,4);
%     Acc_temp = ModeShape_Rail_temp * Zjsd(m_NM:n_NM,4);
    
    j_max = eval(['N_Node.', Type_Rail{i,col}]);
    eval(['DynStatus_Rail.',Type_Rail{i,col},'_Dis = zeros(j_max,6);']);
    eval(['DynStatus_Rail.',Type_Rail{i,col},'_Vel = zeros(j_max,6);']);
%     eval(['DynStatus_Rail.',Type_Rail{i,col},'_Acc = zeros(j_max,6);']);
    eval(['DynStatus_Rail.',Type_Rail{i,col},'_Dis(:,3) = Dis_temp'';']);
    eval(['DynStatus_Rail.',Type_Rail{i,col},'_Vel(:,3) = Vel_temp'';']);
%     eval(['DynStatus_Rail.',Type_Rail{i,col},'_Acc(:,3) = Acc_temp'';']);
end

%% 获取轮下钢轨的动力学参数 Dis_Rail Vel_Rail Acc_Rail
for i1 = 1:1:Nw         %%% 轮对数
    for i2 = 1:1:N_ConPatch      %%% 集总质量块数目
        i_ConPatch = 4*(i1-1)+i2;
        DynStatus_RailDis_temp = eval(['DynStatus_Rail.',Type_Rail{i2,1},'_Dis']);
        DynStatus_RailVel_temp = eval(['DynStatus_Rail.',Type_Rail{i2,1},'_Vel']);
%         DynStatus_RailAcc_temp = eval(['DynStatus_Rail.',Type_Rail{i2,1},'_Acc']);
        
        Target_ShapeFunction_Y = eval(['ShapeFunction.',Expression_WS{i1},'_',Type_Rail{i2,1},'_Y']);
        Target_ShapeFunction_Z = eval(['ShapeFunction.',Expression_WS{i1},'_',Type_Rail{i2,1},'_Z']);
        Mapping_Y = eval(['ShapeFunction.',Expression_WS{i1},'_',Type_Rail{i2,1},'_Mapping_DynStatus_Y']);
        Mapping_Z = eval(['ShapeFunction.',Expression_WS{i1},'_',Type_Rail{i2,1},'_Mapping_DynStatus_Z']);
        
%         Mileage  = j1-Distance_Vehicle(i1);        
%         Pos_Node_temp = eval(['Pos_Node.',Type_Rail{i2,1}]);
%         Pos_Node_Length_temp = eval(['Pos_Node.',Type_Rail{i2,1},'_Length']);
%         m = find(Mileage-Pos_Node_temp(:,1+1)>=0, 1, 'last');
        if ~isempty(Target_ShapeFunction_Y)
%             x = Mileage - Pos_Node_temp(m,2);
%             a = Pos_Node_Length_temp(m,1);
%             ShapeFunction_Y = [1-3*(x/a)^2+2*(x/a)^3, x*(1-2*x/a+(x/a)^2), (x/a)^2*(3-2*x/a), x*((x/a)^2-x/a)];
%             ShapeFunction_Z = [1-3*(x/a)^2+2*(x/a)^3,-x*(1-2*x/a+(x/a)^2), (x/a)^2*(3-2*x/a),-x*((x/a)^2-x/a)];
%             
%             len = eval(['N_Node.', Type_Rail{i2,1}]);
% %             pos_DynStatus_Lon = [len*(1-1)+m; len*(4-1)+m; len*(1-1)+m+1; len*(4-1)+m+1];
%             pos_DynStatus_Lat = [len*(2-1)+m; len*(6-1)+m; len*(2-1)+m+1; len*(6-1)+m+1];
%             pos_DynStatus_Ver = [len*(3-1)+m; len*(5-1)+m; len*(3-1)+m+1; len*(5-1)+m+1];
            
%             Dis_Rail(i_ConPatch,2) = interp1(Pos_Node_temp(:,2),DynStatus_RailDis_temp(:,2),Mileage,'spline');
%             Dis_Rail(i_ConPatch,3) = interp1(Pos_Node_temp(:,2),DynStatus_RailDis_temp(:,3),Mileage,'spline');
%             Vel_Rail(i_ConPatch,2) = interp1(Pos_Node_temp(:,2),DynStatus_RailVel_temp(:,2),Mileage,'spline');
%             Vel_Rail(i_ConPatch,3) = interp1(Pos_Node_temp(:,2),DynStatus_RailVel_temp(:,3),Mileage,'spline');

            Dis_Rail(i_ConPatch,2) = Target_ShapeFunction_Y * DynStatus_RailDis_temp(Mapping_Y);     % 钢轨横向位移
            Dis_Rail(i_ConPatch,3) = Target_ShapeFunction_Z * DynStatus_RailDis_temp(Mapping_Z);     % 钢轨垂向位移
            
            Vel_Rail(i_ConPatch,2) = Target_ShapeFunction_Y * DynStatus_RailVel_temp(Mapping_Y);     % 钢轨横向速度
            Vel_Rail(i_ConPatch,3) = Target_ShapeFunction_Z * DynStatus_RailVel_temp(Mapping_Z);     % 钢轨垂向速度
            
%             Acc_Rail(i_ConPatch,2) = Target_ShapeFunction_Y * DynStatus_RailAcc_temp(Mapping_Y);     % 钢轨横向加速度
%             Acc_Rail(i_ConPatch,3) = Target_ShapeFunction_Z * DynStatus_RailAcc_temp(Mapping_Z);     % 钢轨垂向加速度
            
%             Dis_Rail(i_ConPatch,1) = 0;     % 钢轨纵向位移
%             Dis_Rail(i_ConPatch,2) = ShapeFunction_Y * DynStatus_RailDis_temp(pos_DynStatus_Lat);     % 钢轨横向位移
%             Dis_Rail(i_ConPatch,3) = ShapeFunction_Z * DynStatus_RailDis_temp(pos_DynStatus_Ver);     % 钢轨垂向位移
%             
%             Vel_Rail(i_ConPatch,1) = 0;     % 钢轨纵向速度
%             Vel_Rail(i_ConPatch,2) = ShapeFunction_Y * DynStatus_RailVel_temp(pos_DynStatus_Lat);     % 钢轨横向速度
%             Vel_Rail(i_ConPatch,3) = ShapeFunction_Z * DynStatus_RailVel_temp(pos_DynStatus_Ver);     % 钢轨垂向速度
%             
%             Acc_Rail(i_ConPatch,1) = 0;     % 钢轨纵向加速度
%             Acc_Rail(i_ConPatch,2) = ShapeFunction_Y * DynStatus_RailAcc_temp(pos_DynStatus_Lat);     % 钢轨横向加速度
%             Acc_Rail(i_ConPatch,3) = ShapeFunction_Z * DynStatus_RailAcc_temp(pos_DynStatus_Ver);     % 钢轨垂向加速度
            
        end
    end
end
