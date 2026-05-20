function  [Pos_FW, DOF_pos_FW, Vel_FW_global, ShapeFun, Oup_Vjd] = ...
                Cal_Vjd_FW_230515(InpPar, Par_FW, T1, Tar_Con_wheel_1_II, R0, Yw, i11, Zwy, Zsd, A_WS, pos_WS_track, vel_WS_global, vel_WS_track)

% Cal_Vjd_FW_230515(InpPar, Par_FW, T1, Con_wheel_1_II.b.(T1), R0, Yw, i11, Zwy, Zsd, A_WS, pos_WS_track, vel_WS_global, vel_WS_track);
% TEST        
% T1 = 'L';
% Tar_Con_wheel_1_II = Con_wheel_1_II .b.(T1);

clear Pos_FW DOF_pos_FW Vel_FW_global ShapeFun_XOY ShapeFun_Z Oup_Vjd Pos_FW_T
clear DOF_pos_FW_Around_WS pos_XOY

%% 1. Global C.S.中，考虑车轮节点的弹性变形, Pos_FW
Type_T2 = {'Mid', 'Front', 'Rear'};
Type_Result = {['Tread_', T1, '_Mid'], ['Tread_', T1, '_Front'], ['Tread_', T1, '_Rear']};
pos_NM_FW = InpPar.N_track+InpPar.NM_FW*(i11-1)+(1:1:InpPar.NM_FW);
N_patch = size(Tar_Con_wheel_1_II,1);

for i2 = 1:1:length(Type_T2)
    T2 = Type_T2{i2};
    T = ['Tread_', T1, '_', T2];
    Tar_pos_Tread = InpPar.Pos_Node.Tread.(T1).(T2);
    Tar_DOF_pos_Tread = Par_FW.DOF_pos.(T);

    Pos_FW.WS.(T2)(:,1) = Tar_pos_Tread(:,1);
    Pos_FW.global.(T2)(:,1) = Tar_pos_Tread(:,1);
    len = size(Tar_DOF_pos_Tread,1);
    temp_1 = reshape(Tar_DOF_pos_Tread, len*3, 1);
    temp_2 = (InpPar.ModeShape.FW(temp_1,:)*Zwy(pos_NM_FW,4));
    Pos_FW.WS.(T2)(:,2:4) = Tar_pos_Tread(:,2:4) + reshape(temp_2,len,3);
    Pos_FW.global.(T2)(:,2:4) = repmat([0,Yw,0], len, 1) + Pos_FW.WS.(T2)(:,2:4)* A_WS;
end

%% 2. 距离接触点最近的踏面节点的指针位置, pos_XOY
pos_XOY = zeros(N_patch,1);
Pos_Tread_Y_global = Pos_FW.global.Mid(:,3);
for i1 = 1:1:N_patch
    [~, pos_XOY(i1,1)] = min(abs(Tar_Con_wheel_1_II(i1,2)-Pos_Tread_Y_global));
end

%% 3. 接触点周围四个网格的变形后位置：Pos_FW, DOF_pos_FW_Around_WS
DOF_pos_FW_Around_WS = cell(N_patch,4);
for i1 = 1:1:N_patch  
    % FL, FR, RL, RR
    % Global
    Pos_FW.global.Around{i1,1} = [Pos_FW.global.(Type_T2{1})(pos_XOY(i1,1)-1,:);  Pos_FW.global.(Type_T2{1})(pos_XOY(i1,1),:);
                                                      Pos_FW.global.(Type_T2{2})(pos_XOY(i1,1)-1,:);  Pos_FW.global.(Type_T2{2})(pos_XOY(i1,1),:)];
                                                          
    Pos_FW.global.Around{i1,2} = [Pos_FW.global.(Type_T2{1})(pos_XOY(i1,1),:);  Pos_FW.global.(Type_T2{1})(pos_XOY(i1,1)+1,:);
                                                      Pos_FW.global.(Type_T2{2})(pos_XOY(i1,1),:);  Pos_FW.global.(Type_T2{2})(pos_XOY(i1,1)+1,:)];
                                                          
    Pos_FW.global.Around{i1,3} = [Pos_FW.global.(Type_T2{3})(pos_XOY(i1,1)-1,:);  Pos_FW.global.(Type_T2{3})(pos_XOY(i1,1),:);
                                                      Pos_FW.global.(Type_T2{1})(pos_XOY(i1,1)-1,:);  Pos_FW.global.(Type_T2{1})(pos_XOY(i1,1),:)];
                                                          
    Pos_FW.global.Around{i1,4} = [Pos_FW.global.(Type_T2{3})(pos_XOY(i1,1),:);  Pos_FW.global.(Type_T2{3})(pos_XOY(i1,1)+1,:);
                                                      Pos_FW.global.(Type_T2{1})(pos_XOY(i1,1),:);  Pos_FW.global.(Type_T2{1})(pos_XOY(i1,1)+1,:)];
                                                          
    % WS
    Pos_FW.WS.Around{i1,1} = [Pos_FW.WS.(Type_T2{1})(pos_XOY(i1,1)-1,:);  Pos_FW.WS.(Type_T2{1})(pos_XOY(i1,1),:);
                                                 Pos_FW.WS.(Type_T2{2})(pos_XOY(i1,1)-1,:);  Pos_FW.WS.(Type_T2{2})(pos_XOY(i1,1),:)];
                                                          
    Pos_FW.WS.Around{i1,2} = [Pos_FW.WS.(Type_T2{1})(pos_XOY(i1,1),:);  Pos_FW.WS.(Type_T2{1})(pos_XOY(i1,1)+1,:);
                                                 Pos_FW.WS.(Type_T2{2})(pos_XOY(i1,1),:);  Pos_FW.WS.(Type_T2{2})(pos_XOY(i1,1)+1,:)];
                                                          
    Pos_FW.WS.Around{i1,3} = [Pos_FW.WS.(Type_T2{3})(pos_XOY(i1,1)-1,:);  Pos_FW.WS.(Type_T2{3})(pos_XOY(i1,1),:);
                                                 Pos_FW.WS.(Type_T2{1})(pos_XOY(i1,1)-1,:);  Pos_FW.WS.(Type_T2{1})(pos_XOY(i1,1),:)];
                                                          
    Pos_FW.WS.Around{i1,4} = [Pos_FW.WS.(Type_T2{3})(pos_XOY(i1,1),:);  Pos_FW.WS.(Type_T2{3})(pos_XOY(i1,1)+1,:);
                                                 Pos_FW.WS.(Type_T2{1})(pos_XOY(i1,1),:);  Pos_FW.WS.(Type_T2{1})(pos_XOY(i1,1)+1,:)];  

    % DOF_pos
    DOF_pos_FW_Around_WS{i1,1} = [Par_FW.DOF_pos.(Type_Result{1})(pos_XOY(i1,1)-1,:);  Par_FW.DOF_pos.(Type_Result{1})(pos_XOY(i1,1),:);
                                                           Par_FW.DOF_pos.(Type_Result{2})(pos_XOY(i1,1)-1,:);  Par_FW.DOF_pos.(Type_Result{2})(pos_XOY(i1,1),:)];
                                                          
    DOF_pos_FW_Around_WS{i1,2} = [Par_FW.DOF_pos.(Type_Result{1})(pos_XOY(i1,1),:);  Par_FW.DOF_pos.(Type_Result{1})(pos_XOY(i1,1)+1,:);
                                                           Par_FW.DOF_pos.(Type_Result{2})(pos_XOY(i1,1),:);  Par_FW.DOF_pos.(Type_Result{2})(pos_XOY(i1,1)+1,:)];
                                                          
    DOF_pos_FW_Around_WS{i1,3} = [Par_FW.DOF_pos.(Type_Result{3})(pos_XOY(i1,1)-1,:);  Par_FW.DOF_pos.(Type_Result{3})(pos_XOY(i1,1),:);
                                                           Par_FW.DOF_pos.(Type_Result{1})(pos_XOY(i1,1)-1,:);  Par_FW.DOF_pos.(Type_Result{1})(pos_XOY(i1,1),:)];
                                                          
    DOF_pos_FW_Around_WS{i1,4} = [Par_FW.DOF_pos.(Type_Result{3})(pos_XOY(i1,1),:);  Par_FW.DOF_pos.(Type_Result{3})(pos_XOY(i1,1)+1,:);
                                                           Par_FW.DOF_pos.(Type_Result{1})(pos_XOY(i1,1),:);  Par_FW.DOF_pos.(Type_Result{1})(pos_XOY(i1,1)+1,:)];
end

%% 4. 投影至 XOY 平面内，判断接触点位于哪个网格中：Pos_FW, DOF_pos_FW
Coor_XOY = cell(N_patch,1);
Tar_Around = zeros(N_patch,1);
Tar_AroundPos = zeros(N_patch,2);
for i1 = 1:1:N_patch
    xx = Tar_Con_wheel_1_II(i1,2);
    yy = Tar_Con_wheel_1_II(i1,1);
    for j = 1:1:4
        x = Pos_FW.global.Around{i1,j}(:,3);
        y = Pos_FW.global.Around{i1,j}(:,2);        
        func_XOY = @(Coor)[x(1)/4*(1-Coor(1))*(1-Coor(2)) + x(2)/4*(1+Coor(1))*(1-Coor(2)) + x(3)/4*(1-Coor(1))*(1+Coor(2)) + x(4)/4*(1+Coor(1))*(1+Coor(2)) - xx;
                                           y(1)/4*(1-Coor(1))*(1-Coor(2)) + y(2)/4*(1+Coor(1))*(1-Coor(2)) + y(3)/4*(1-Coor(1))*(1+Coor(2)) + y(4)/4*(1+Coor(1))*(1+Coor(2)) - yy];
        options = optimset('Display','off');
        Coor_XOY{i1,1}(j,:) = fsolve(func_XOY, [0,0], options);
    end
    Thread = 2e-6;      % Normal
%     Thread = 2.5e-6;
%     Thread = 3e-6;
    Thread = 1.5e-3;    % WearRailProfile

%     bools = (Coor_XOY{i1,1}(:,1)>=-1-2e-6 & Coor_XOY{i1,1}(:,1)<=1+2e-6) & (Coor_XOY{i1,1}(:,2)>=-1-2e-6 & Coor_XOY{i1,1}(:,2)<=1+2e-6);
    bools = (Coor_XOY{i1,1}(:,1)>=-1-Thread & Coor_XOY{i1,1}(:,1)<=1+Thread) & (Coor_XOY{i1,1}(:,2)>=-1-Thread & Coor_XOY{i1,1}(:,2)<=1+Thread);
    
    Tar_Around(i1,1) = find(bools,1);
    Pos_FW.WS.Node_Around_XOY{i1,1} = Pos_FW.WS.Around{i1,find(bools,1)};
    Pos_FW.global.Node_Around_XOY{i1,1} = Pos_FW.global.Around{i1,find(bools,1)};
    DOF_pos_FW.Node_Around_XOY{i1,1} = DOF_pos_FW_Around_WS{i1,find(bools,1)};
    Coor_XOY{i1,1} = Coor_XOY{i1,1}(find(bools,1),:);

    if Tar_Around(i1,1)<=2
        Tar_AroundPos(i1,1:2) = [1,2];
    else
        Tar_AroundPos(i1,1:2) = [3,4];
    end
    Pos_FW.WS.Node_Around_Z{i1,1} = Pos_FW.WS.Node_Around_XOY{i1,1}(Tar_AroundPos(i1,1:2),:);
    Pos_FW.global.Node_Around_Z{i1,1} = Pos_FW.global.Node_Around_XOY{i1,1}(Tar_AroundPos(i1,1:2),:);
    DOF_pos_FW.Node_Around_Z{i1,1} = DOF_pos_FW.Node_Around_XOY{i1,1}(Tar_AroundPos(i1,1:2),:);    
end

%% 5. 计算接触点所在网格周边节点的运动速度 Vel_FW_global
% 旋转顺序 3-1-2（翟院士专著计算车轮上接触点速度）：旋转矩阵 G' 的各列为沿 3 个卡尔丹转动轴的单位矢量在随体坐标系中的坐标列阵
G_Cardan_WSBody = [1, 0, 0; 0, 1, sin(pos_WS_track(4)); 0, 0, cos(pos_WS_track(4))];
Angvel_WS_Body = G_Cardan_WSBody * [vel_WS_track(4); (-InpPar.Vlc/R0+vel_WS_track(5)); vel_WS_track(6)];
vel_WS = vel_WS_global(1:3);

for i1 = 1:1:N_patch
    Vel_FW_global.Node_Around_XOY{i1,1}(:,1) = Pos_FW.WS.Node_Around_XOY{i1,1}(:,1);
    for j = 1:1:4
        u = Pos_FW.WS.Node_Around_XOY{i1,1}(j,2:4);
        u_Antisym = [0, -u(3), u(2); u(3), 0, -u(1); -u(2), u(1), 0];
        bools = DOF_pos_FW.Node_Around_XOY{i1,1}(j,:);
        Vel_FW_global.Node_Around_XOY{i1,1}(j,2:4) = vel_WS' - A_WS'*u_Antisym*Angvel_WS_Body + A_WS'*(InpPar.ModeShape.FW(bools,:)*Zsd(pos_NM_FW,4));
    end
    Vel_FW_global.Node_Around_Z{i1,1} = Vel_FW_global.Node_Around_XOY{i1,1}(Tar_AroundPos(i1,1:2),:);
end

%% 6. 插值获取接触点处的速度：ShapeFun_XOY, ShapeFun_Z, Vjd
Oup_Vjd = zeros(N_patch,3);
ShapeFun = struct;
for i1 = 1:1:N_patch    
    % X、Y 方向
    ShapeFun.XOY{i1,1} = [(1-Coor_XOY{i1,1}(1))*(1-Coor_XOY{i1,1}(2)),  (1+Coor_XOY{i1,1}(1))*(1-Coor_XOY{i1,1}(2)), ...
                                          (1-Coor_XOY{i1,1}(1))*(1+Coor_XOY{i1,1}(2)), (1+Coor_XOY{i1,1}(1))*(1+Coor_XOY{i1,1}(2))]*1/4;
                        
    % Z 方向
    Coor_Z = (Tar_Con_wheel_1_II(i1,2)-Pos_FW.global.Node_Around_Z{i1,1}(1,3)) / (Pos_FW.global.Node_Around_Z{i1,1}(2,3)-Pos_FW.global.Node_Around_Z{i1,1}(1,3));
    ShapeFun.Z{i1,1} = [1-Coor_Z, Coor_Z];
    
    % Vjd
    Oup_Vjd(i1,1:3) = [ShapeFun.XOY{i1,1} * Vel_FW_global.Node_Around_XOY{i1,1}(:,2), ...
                                  ShapeFun.XOY{i1,1} * Vel_FW_global.Node_Around_XOY{i1,1}(:,3), ...
                                  ShapeFun.Z{i1,1} * Vel_FW_global.Node_Around_Z{i1,1}(:,4)];    
end
    
%% Draft
% 1. Global C.S.中，考虑车轮节点的弹性变形, Pos_FW
% for i2 = 1:1:length(Type_T2)
%     T2 = Type_T2{i2};
%     T = ['Tread_', T1, '_', T2];
%     Tar_pos_Tread = InpPar.Pos_Node.Tread.(T1).(T2);
%     Tar_DOF_pos_Tread = Par_FW.DOF_pos.(T);
% 
%     Pos_FW.WS.(T2)(:,1) = Tar_pos_Tread(:,1);
%     Pos_FW.global.(T2)(:,1) = Tar_pos_Tread(:,1);
%     len = size(Tar_DOF_pos_Tread,1);
%     temp_1 = reshape(Tar_DOF_pos_Tread, len*3, 1);
%     temp_2 = (InpPar.ModeShape.FW(temp_1,:)*Zwy(pos_NM_FW,4));
%     Pos_FW.WS.(T2)(:,2:4) = Tar_pos_Tread(:,2:4) + reshape(temp_2,len,3);
%     Pos_FW.global.(T2)(:,2:4) = repmat([0,Yw,0], len, 1) + Pos_FW.WS.(T2)(:,2:4)* A_WS;
% 
%     for j = 1:1:size(Tar_pos_Tread,1)
%         DOF_pos = Tar_DOF_pos_Tread(j,:);
%         Pos_FW_T.WS.(T2)(j,1) = Tar_pos_Tread(j,1);
%         Pos_FW_T.WS.(T2)(j,2:4) = Tar_pos_Tread(j,2:4) +(InpPar.ModeShape.FW(DOF_pos,:)*Zwy(pos_NM_FW,4))';
%         Pos_FW_T.global.(T2)(j,1) = Tar_pos_Tread(j,1);
%         Pos_FW_T.global.(T2)(j,2:4) = [0 Yw 0] + (Pos_FW_T.WS.(T2)(j,2:4)) * A_WS;
%     end    
% end
% A = rand(10,3);
% B = reshape(reshape(A, 30, 1), 10, 3);
% A-B
% Pos_FW.WS.(T2) - Pos_FW_T.WS.(T2)
% Pos_FW.global.(T2) - Pos_FW_T.global.(T2)
