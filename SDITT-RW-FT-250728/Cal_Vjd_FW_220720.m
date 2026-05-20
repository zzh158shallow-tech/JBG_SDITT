function  [Pos_FW, DOF_pos_FW, Vel_FW_global, ShapeFun_XOY, ShapeFun_Z, Vjd] = ...
                 Cal_Vjd_FW_220720(Opt_Side, Con_wheel_1_II, R0, Yw, i11, Zwy, Zsd, A_WS, pos_WS_track, vel_WS_global, vel_WS_track)

global Par_FW Vlc N_track NM_FW ModeShape

%%% 测试用
% Opt_Side = 'L';
% % Opt_Side = 'R';
% if strcmp(Opt_Side,'R')
%     Con_wheel_1_II = Con_wheel_R_1_II;
% elseif strcmp(Opt_Side,'L')
%     Con_wheel_1_II = Con_wheel_L_1_II;
% end

clear Pos_FW DOF_pos_FW Vel_FW_global ShapeFun_XOY ShapeFun_Z Vjd
clear DOF_pos_FW_Around_WS pos_XOY

%% 1. Global C.S.中，考虑车轮节点的弹性变形, Pos_FW
Type_Result = {['Tread_', Opt_Side, '_Mid'], ['Tread_', Opt_Side, '_Front'], ['Tread_', Opt_Side, '_Rear']};
for i2 = 1:1:length(Type_Result)
    Target_pos_Tread = Par_FW.(Type_Result{i2});
    Target_DOF_pos_Tread = Par_FW.DOF_pos.(Type_Result{i2});
    for j = 1:1:size(Target_pos_Tread,1)
        DOF_pos = Target_DOF_pos_Tread(j,:);
        Pos_FW.WS.(Type_Result{i2})(j,1) = Target_pos_Tread(j,1);
        Pos_FW.WS.(Type_Result{i2})(j,2:4) = Target_pos_Tread(j,2:4) +(ModeShape.FW(DOF_pos,:)*Zwy(N_track+NM_FW*(i11-1)+1:N_track+NM_FW*i11,4))';
        Pos_FW.global.(Type_Result{i2})(j,1) = Target_pos_Tread(j,1);
        Pos_FW.global.(Type_Result{i2})(j,2:4) = [0 Yw 0] + (Pos_FW.WS.(Type_Result{i2})(j,2:4)) * A_WS;
    end
end

%% 2. 距离接触点最近的踏面节点的指针位置, pos_XOY
clear pos
Pos_Tread_Y_global = Pos_FW.global.(['Tread_', Opt_Side, '_Mid'])(:,3);
for i1 = 1:1:size(Con_wheel_1_II,1)
    [~, pos_XOY(i1,1)] = min(abs(Con_wheel_1_II(i1,2)-Pos_Tread_Y_global));
%     pos_Z(i1,1:2) = [max(find(Con_wheel_1_II(i1,2)-Pos_Tread_Y_global>=0)), min(find(Con_wheel_1_II(i1,2)-Pos_Tread_Y_global<0))];
end

%% 3. 接触点周围四个网格的变形后位置：Pos_FW, DOF_pos_FW_Around_WS
Type_Result = {['Tread_', Opt_Side, '_Mid'], ['Tread_', Opt_Side, '_Front'], ['Tread_', Opt_Side, '_Rear']};
for i1 = 1:1:size(Con_wheel_1_II,1)    
    % FL, FR, RL, RR
    % Global
    Pos_FW.global.Around{i1,1} = [Pos_FW.global.(Type_Result{1})(pos_XOY(i1,1)-1,:);  Pos_FW.global.(Type_Result{1})(pos_XOY(i1,1),:);
                                                            Pos_FW.global.(Type_Result{2})(pos_XOY(i1,1)-1,:);  Pos_FW.global.(Type_Result{2})(pos_XOY(i1,1),:)];
                                                          
    Pos_FW.global.Around{i1,2} = [Pos_FW.global.(Type_Result{1})(pos_XOY(i1,1),:);  Pos_FW.global.(Type_Result{1})(pos_XOY(i1,1)+1,:);
                                                            Pos_FW.global.(Type_Result{2})(pos_XOY(i1,1),:);  Pos_FW.global.(Type_Result{2})(pos_XOY(i1,1)+1,:)];
                                                          
    Pos_FW.global.Around{i1,3} = [Pos_FW.global.(Type_Result{3})(pos_XOY(i1,1)-1,:);  Pos_FW.global.(Type_Result{3})(pos_XOY(i1,1),:);
                                                            Pos_FW.global.(Type_Result{1})(pos_XOY(i1,1)-1,:);  Pos_FW.global.(Type_Result{1})(pos_XOY(i1,1),:)];
                                                          
    Pos_FW.global.Around{i1,4} = [Pos_FW.global.(Type_Result{3})(pos_XOY(i1,1),:);  Pos_FW.global.(Type_Result{3})(pos_XOY(i1,1)+1,:);
                                                            Pos_FW.global.(Type_Result{1})(pos_XOY(i1,1),:);  Pos_FW.global.(Type_Result{1})(pos_XOY(i1,1)+1,:)];
                                                          
    % WS
    Pos_FW.WS.Around{i1,1} = [Pos_FW.WS.(Type_Result{1})(pos_XOY(i1,1)-1,:);  Pos_FW.WS.(Type_Result{1})(pos_XOY(i1,1),:);
                                                       Pos_FW.WS.(Type_Result{2})(pos_XOY(i1,1)-1,:);  Pos_FW.WS.(Type_Result{2})(pos_XOY(i1,1),:)];
                                                          
    Pos_FW.WS.Around{i1,2} = [Pos_FW.WS.(Type_Result{1})(pos_XOY(i1,1),:);  Pos_FW.WS.(Type_Result{1})(pos_XOY(i1,1)+1,:);
                                                       Pos_FW.WS.(Type_Result{2})(pos_XOY(i1,1),:);  Pos_FW.WS.(Type_Result{2})(pos_XOY(i1,1)+1,:)];
                                                          
    Pos_FW.WS.Around{i1,3} = [Pos_FW.WS.(Type_Result{3})(pos_XOY(i1,1)-1,:);  Pos_FW.WS.(Type_Result{3})(pos_XOY(i1,1),:);
                                                       Pos_FW.WS.(Type_Result{1})(pos_XOY(i1,1)-1,:);  Pos_FW.WS.(Type_Result{1})(pos_XOY(i1,1),:)];
                                                          
    Pos_FW.WS.Around{i1,4} = [Pos_FW.WS.(Type_Result{3})(pos_XOY(i1,1),:);  Pos_FW.WS.(Type_Result{3})(pos_XOY(i1,1)+1,:);
                                                       Pos_FW.WS.(Type_Result{1})(pos_XOY(i1,1),:);  Pos_FW.WS.(Type_Result{1})(pos_XOY(i1,1)+1,:)];  

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
clear Coor_XOY
for i1 = 1:1:size(Con_wheel_1_II,1)
    xx = Con_wheel_1_II(i1,2);
    yy = Con_wheel_1_II(i1,1);
    for j = 1:1:4
        x = Pos_FW.global.Around{i1,j}(:,3);
        y = Pos_FW.global.Around{i1,j}(:,2);
        
        func_XOY = @(Coor)[x(1)/4*(1-Coor(1))*(1-Coor(2)) + x(2)/4*(1+Coor(1))*(1-Coor(2)) + x(3)/4*(1-Coor(1))*(1+Coor(2)) + x(4)/4*(1+Coor(1))*(1+Coor(2)) - xx;
                                               y(1)/4*(1-Coor(1))*(1-Coor(2)) + y(2)/4*(1+Coor(1))*(1-Coor(2)) + y(3)/4*(1-Coor(1))*(1+Coor(2)) + y(4)/4*(1+Coor(1))*(1+Coor(2)) - yy];
        Coor_0 = [0,0];
        options = optimset('Display','off');
        Coor_XOY{i1,1}(j,:) = fsolve(func_XOY, Coor_0, options);
    end
    bools = (Coor_XOY{i1,1}(:,1)>=-1-1e-6 & Coor_XOY{i1,1}(:,1)<=1+1e-6) & (Coor_XOY{i1,1}(:,2)>=-1-1e-6 & Coor_XOY{i1,1}(:,2)<=1+1e-6);
    
    Target_Around(i1,1) = min(find(bools));
    Pos_FW.WS.Node_Around_XOY{i1,1} = Pos_FW.WS.Around{i1,Target_Around(i1,1)};
    Pos_FW.global.Node_Around_XOY{i1,1} = Pos_FW.global.Around{i1,Target_Around(i1,1)};
    DOF_pos_FW.Node_Around_XOY{i1,1} = DOF_pos_FW_Around_WS{i1,Target_Around(i1,1)};
    Coor_XOY{i1,1} = Coor_XOY{i1,1}(Target_Around(i1,1),:);
    if Target_Around(i1,1)<=2
        Pos_FW.global.Node_Around_Z{i1,1} = Pos_FW.global.Node_Around_XOY{i1,1}(1:2,:);
        DOF_pos_FW.Node_Around_Z{i1,1} = DOF_pos_FW.Node_Around_XOY{i1,1}(1:2,:);
    else
        Pos_FW.global.Node_Around_Z{i1,1} = Pos_FW.global.Node_Around_XOY{i1,1}(3:4,:);
        DOF_pos_FW.Node_Around_Z{i1,1} = DOF_pos_FW.Node_Around_XOY{i1,1}(3:4,:);
    end
    
end

%% 5. 计算接触点所在网格周边节点的运动速度 Vel_FW_global
vel_WS = vel_WS_global(1:3);
% 旋转顺序 3-1-2（翟院士专著计算车轮上接触点速度）：旋转矩阵 G' 的各列为沿 3 个卡尔丹转动轴的单位矢量在随体坐标系中的坐标列阵
G_Cardan_WS_Body = [1   0   0;...
                                          0   1   sin(pos_WS_track(4));...
                                          0   0   cos(pos_WS_track(4))];
Angvel_WS_Body = G_Cardan_WS_Body * [vel_WS_track(4); (-Vlc/R0 + vel_WS_track(5)); vel_WS_track(6)];

for i1 = 1:1:size(Con_wheel_1_II,1)
    Vel_FW_global.Node_Around_XOY{i1,1}(:,1) = Pos_FW.WS.Node_Around_XOY{i1,1}(:,1);
    for j = 1:1:4
        u = Pos_FW.WS.Node_Around_XOY{i1,1}(j,2:4);
        u_Antisym = [     0, -u(3),  u(2); ...
                                 u(3),       0, -u(1); ...
                                -u(2),  u(1),  0];  
        bools = DOF_pos_FW.Node_Around_XOY{i1,1}(j,:);
        Vel_FW_global.Node_Around_XOY{i1,1}(j,2:4) = (vel_WS' - A_WS'*u_Antisym*Angvel_WS_Body + ...
                                                                                                A_WS'*(ModeShape.FW(bools,:)*Zsd(N_track+NM_FW*(i11-1)+1:N_track+NM_FW*i11,4)))';
    end
    if Target_Around(i1,1)<=2
        Vel_FW_global.Node_Around_Z{i1,1} = Vel_FW_global.Node_Around_XOY{i1,1}(1:2,:);
    else
        Vel_FW_global.Node_Around_Z{i1,1} = Vel_FW_global.Node_Around_XOY{i1,1}(3:4,:);
    end
end

%% 6. 插值获取接触点处的速度：ShapeFun_XOY, ShapeFun_Z, Vjd
Vjd = zeros(size(Con_wheel_1_II,1),3);

for i1 = 1:1:size(Con_wheel_1_II,1)
    
    % X、Y 方向
    ShapeFun_XOY{i1,1} = [(1-Coor_XOY{i1,1}(1))*(1-Coor_XOY{i1,1}(2)), (1+Coor_XOY{i1,1}(1))*(1-Coor_XOY{i1,1}(2)), ...
                                              (1-Coor_XOY{i1,1}(1))*(1+Coor_XOY{i1,1}(2)), (1+Coor_XOY{i1,1}(1))*(1+Coor_XOY{i1,1}(2))]*1/4;
                        
    % Z 方向
    Coor_Z = (Con_wheel_1_II(i1,2)-Pos_FW.global.Node_Around_Z{i1,1}(1,3)) / (Pos_FW.global.Node_Around_Z{i1,1}(2,3)-Pos_FW.global.Node_Around_Z{i1,1}(1,3));
    ShapeFun_Z{i1,1} = [1-Coor_Z, Coor_Z];
    
    % Vjd
    Vjd(i1,1:3) = [ShapeFun_XOY{i1,1}*Vel_FW_global.Node_Around_XOY{i1,1}(:,2), ...
                            ShapeFun_XOY{i1,1}*Vel_FW_global.Node_Around_XOY{i1,1}(:,3), ...
                            ShapeFun_Z{i1,1}*Vel_FW_global.Node_Around_Z{i1,1}(:,4)];
    
end
    
