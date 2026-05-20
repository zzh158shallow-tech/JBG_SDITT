%% 计算车轮下钢轨的位移、速度、加速度
function [Dis_Rail, Vel_Rail, Acc_Rail, DynStatus_Rail] = RailDyn_FT_FEM(Zwy, Zsd, Zjsd, ShapeFunction)

global Nw N_ConPatch N_Node Type_Rail Expression_WS DOF_Rail
      
Dis_Rail = zeros(Nw*N_ConPatch,3);      % 钢轨接触点处 X、Y、Z 方向位移（列） m
Vel_Rail = zeros(Nw*N_ConPatch,3);      % 钢轨接触点处 X、Y、Z 方向（列）速度 m/s
Acc_Rail = zeros(Nw*N_ConPatch,3);      % 钢轨接触点处 X、Y、Z 方向（列）加速度 m/s^2

%% 获取各钢轨、铁垫板的动力学参数 DynStatus_Rail N_Node
col = 1;
for i = 1:1:N_ConPatch
    if i == 1
        m = 1;
    else
        m = sum(DOF_Rail.Sort(1:i-1))+1;
    end
    n = sum(DOF_Rail.Sort(1:i));
    Dis_temp = Zwy(m:n,4);
    Vel_temp = Zsd(m:n,4);
    Acc_temp = Zjsd(m:n,4);
    
    j_max = N_Node.(Type_Rail{i,col});
    DynStatus_Rail.([Type_Rail{i,col},'_Dis']) = zeros(j_max,6);
    DynStatus_Rail.([Type_Rail{i,col},'_Vel']) = zeros(j_max,6);
    DynStatus_Rail.([Type_Rail{i,col},'_Acc']) = zeros(j_max,6);
    for j = 1:1:j_max
        DynStatus_Rail.([Type_Rail{i,col},'_Dis'])(j,[2,3,5,6]) = Dis_temp((j-1)*4+1:j*4,1)';
        DynStatus_Rail.([Type_Rail{i,col},'_Vel'])(j,[2,3,5,6]) = Vel_temp((j-1)*4+1:j*4,1)';
        DynStatus_Rail.([Type_Rail{i,col},'_Acc'])(j,[2,3,5,6]) = Acc_temp((j-1)*4+1:j*4,1)';
    end
end

col = 2;
for i = 1:1:2
    m = sum(DOF_Rail.Sort(1:N_ConPatch+i-1))+1;
    n = sum(DOF_Rail.Sort(1:N_ConPatch+i));
    Dis_temp = Zwy(m:n,4);
    Vel_temp = Zsd(m:n,4);
    Acc_temp = Zjsd(m:n,4);
    
    j_max = N_Node.(Type_Rail{i,col});
    DynStatus_Rail.([Type_Rail{i,col},'_Dis']) = zeros(j_max,6);
    DynStatus_Rail.([Type_Rail{i,col},'_Vel']) = zeros(j_max,6);
    DynStatus_Rail.([Type_Rail{i,col},'_Acc']) = zeros(j_max,6);
    for j = 1:1:j_max
        DynStatus_Rail.([Type_Rail{i,col},'_Dis'])(j,[3,4]) = Dis_temp((j-1)*2+1:j*2,1)';
        DynStatus_Rail.([Type_Rail{i,col},'_Vel'])(j,[3,4]) = Vel_temp((j-1)*2+1:j*2,1)';
        DynStatus_Rail.([Type_Rail{i,col},'_Acc'])(j,[3,4]) = Acc_temp((j-1)*2+1:j*2,1)';
    end
end

%% 获取轮下钢轨的动力学参数 Dis_Rail Vel_Rail Acc_Rail
for i1 = 1:1:Nw         %%% 轮对数
    for i2 = 1:1:N_ConPatch      %%% 集总质量块数目
        i_ConPatch = N_ConPatch*(i1-1)+i2;
        Target_ShapeFunction_Y = ShapeFunction.([Expression_WS{i1},'_',Type_Rail{i2,1},'_Y']);
        Target_ShapeFunction_Z = ShapeFunction.([Expression_WS{i1},'_',Type_Rail{i2,1},'_Z']);
        Mapping_Y = ShapeFunction.([Expression_WS{i1},'_',Type_Rail{i2,1},'_Mapping_DynStatus_Y']);
        Mapping_Z = ShapeFunction.([Expression_WS{i1},'_',Type_Rail{i2,1},'_Mapping_DynStatus_Z']);
        
        if ~isempty(Target_ShapeFunction_Y)
            Dis_Rail(i_ConPatch,2) = Target_ShapeFunction_Y * Zwy(Mapping_Y,4);     % 钢轨横向位移
            Dis_Rail(i_ConPatch,3) = Target_ShapeFunction_Z * Zwy(Mapping_Z,4);     % 钢轨垂向位移
            
            Vel_Rail(i_ConPatch,2) = Target_ShapeFunction_Y * Zsd(Mapping_Y,4);     % 钢轨横向速度
            Vel_Rail(i_ConPatch,3) = Target_ShapeFunction_Z * Zsd(Mapping_Z,4);     % 钢轨垂向速度
            
            Acc_Rail(i_ConPatch,2) = Target_ShapeFunction_Y * Zjsd(Mapping_Y,4);     % 钢轨横向加速度
            Acc_Rail(i_ConPatch,3) = Target_ShapeFunction_Z * Zjsd(Mapping_Z,4);     % 钢轨垂向加速度                      
        end
    end
end

