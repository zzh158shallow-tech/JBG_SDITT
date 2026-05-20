%% 计算柔性轮对在接触点处的速度

function [Vjd, Coor_Target_local_XOY, pos_Con_Around_Deformed_track_XOY, pos_Con_Around_Deformed_WS_XOY, vel_Con_Around_Deformed_track_XOY,...
               Coor_Target_local_YOZ, pos_Con_Around_Deformed_track_YOZ, pos_Con_Around_Deformed_WS_YOZ, vel_Con_Around_Deformed_track_YOZ] = ...
         Cal_Vjd_FW(Opt, Con_wheel_1_II, R0, Yw, i11, Zwy, Zsd, A_WS, pos_WS_track, vel_WS_global, vel_WS_track)

global Par_FW N_Node Vlc N_track NM_FW ModeShape
                
Choose_Plot = 0;
% %%% 测试用
% Opt = 'L';
% Opt = 'R';
% if strcmp(Opt,'R')
%     pos_tread_Cartesian_temp = Par_FW.pos_tread_Cartesian_R;
%     Con_wheel_1_II = Con_wheel_R_1_II;
% elseif strcmp(Opt,'L')
%     pos_tread_Cartesian_temp = Par_FW.pos_tread_Cartesian_L;
%     Con_wheel_1_II = Con_wheel_L_1_II;
% end

clear Vjd pos_Con_Around_Deformed_track pos_Con_Around_Deformed_WS vel_Con_Around_Deformed_track

if strcmp(Opt,'L')
    pos_tread_Cartesian_temp = Par_FW.pos_tread_Cartesian_L;
elseif strcmp(Opt,'R')
    pos_tread_Cartesian_temp = Par_FW.pos_tread_Cartesian_R;    
end

%% 2. Global C.S.中，考虑车轮节点的弹性变形
num_tread = pos_tread_Cartesian_temp(:,1);
for i = 1:1:length(num_tread)
    bools = [N_Node.FW*0, N_Node.FW*1, N_Node.FW*2] + num_tread(i);
    pos_tread_Cartesian_Deformed_track(i,:) = ...
    [num_tread(i), [0 Yw 0] + (pos_tread_Cartesian_temp(i,2:4)+(ModeShape.FW(bools,:)*Zwy(N_track+NM_FW*(i11-1)+1:N_track+NM_FW*i11,4))')*A_WS];
    pos_tread_Cartesian_Deformed_WS(i,:) = ...
    [num_tread(i), pos_tread_Cartesian_temp(i,2:4)+(ModeShape.FW(bools,:)*Zwy(N_track+NM_FW*(i11-1)+1:N_track+NM_FW*i11,4))'];
end

%%% 输出：pos_tread_Cartesian_Deformed：len*4 考虑弹性变形后的踏面节点

%% 3. 选取接触点周围4个节点
len = length(Par_FW.pos_tread_Lat);
pos_tread_Cartesian_1_Deformed_track = pos_tread_Cartesian_Deformed_track(len*0+1:len*1,:);
pos_tread_Cartesian_2_Deformed_track = pos_tread_Cartesian_Deformed_track(len*1+1:len*2,:);
pos_tread_Cartesian_3_Deformed_track = pos_tread_Cartesian_Deformed_track(len*2+1:len*3,:);
pos_tread_Cartesian_1_Deformed_WS = pos_tread_Cartesian_Deformed_WS(len*0+1:len*1,:);
pos_tread_Cartesian_2_Deformed_WS = pos_tread_Cartesian_Deformed_WS(len*1+1:len*2,:);
pos_tread_Cartesian_3_Deformed_WS = pos_tread_Cartesian_Deformed_WS(len*2+1:len*3,:);

for i = 1:1:size(Con_wheel_1_II,1)
    [~,pos] = min(abs(Con_wheel_1_II(i,2)-pos_tread_Cartesian_1_Deformed_track(:,3)));
    Polygon_track{1,1} = [pos_tread_Cartesian_1_Deformed_track(pos,:);...
                          pos_tread_Cartesian_1_Deformed_track(pos+1,:);...
                          pos_tread_Cartesian_2_Deformed_track(pos+1,:);...
                          pos_tread_Cartesian_2_Deformed_track(pos,:);...
                          pos_tread_Cartesian_1_Deformed_track(pos,:)];
    Polygon_track{2,1} = [pos_tread_Cartesian_1_Deformed_track(pos,:);...
                          pos_tread_Cartesian_1_Deformed_track(pos+1,:);...
                          pos_tread_Cartesian_3_Deformed_track(pos+1,:);...
                          pos_tread_Cartesian_3_Deformed_track(pos,:);...
                          pos_tread_Cartesian_1_Deformed_track(pos,:)];
    Polygon_track{3,1} = [pos_tread_Cartesian_1_Deformed_track(pos,:);...
                          pos_tread_Cartesian_1_Deformed_track(pos-1,:);...
                          pos_tread_Cartesian_3_Deformed_track(pos-1,:);...
                          pos_tread_Cartesian_3_Deformed_track(pos,:);...
                          pos_tread_Cartesian_1_Deformed_track(pos,:)];
    Polygon_track{4,1} = [pos_tread_Cartesian_1_Deformed_track(pos,:);...
                          pos_tread_Cartesian_1_Deformed_track(pos-1,:);...
                          pos_tread_Cartesian_2_Deformed_track(pos-1,:);...
                          pos_tread_Cartesian_2_Deformed_track(pos,:);...
                          pos_tread_Cartesian_1_Deformed_track(pos,:)];
                
    Polygon_WS{1,1} = [pos_tread_Cartesian_1_Deformed_WS(pos,:);...
                       pos_tread_Cartesian_1_Deformed_WS(pos+1,:);...
                       pos_tread_Cartesian_2_Deformed_WS(pos+1,:);...
                       pos_tread_Cartesian_2_Deformed_WS(pos,:);...
                       pos_tread_Cartesian_1_Deformed_WS(pos,:)];
    Polygon_WS{2,1} = [pos_tread_Cartesian_1_Deformed_WS(pos,:);...
                       pos_tread_Cartesian_1_Deformed_WS(pos+1,:);...
                       pos_tread_Cartesian_3_Deformed_WS(pos+1,:);...
                       pos_tread_Cartesian_3_Deformed_WS(pos,:);...
                       pos_tread_Cartesian_1_Deformed_WS(pos,:)];
    Polygon_WS{3,1} = [pos_tread_Cartesian_1_Deformed_WS(pos,:);...
                       pos_tread_Cartesian_1_Deformed_WS(pos-1,:);...
                       pos_tread_Cartesian_3_Deformed_WS(pos-1,:);...
                       pos_tread_Cartesian_3_Deformed_WS(pos,:);...
                       pos_tread_Cartesian_1_Deformed_WS(pos,:)];
    Polygon_WS{4,1} = [pos_tread_Cartesian_1_Deformed_WS(pos,:);...
                       pos_tread_Cartesian_1_Deformed_WS(pos-1,:);...
                       pos_tread_Cartesian_2_Deformed_WS(pos-1,:);...
                       pos_tread_Cartesian_2_Deformed_WS(pos,:);...
                       pos_tread_Cartesian_1_Deformed_WS(pos,:)];
    
    bools_in = zeros(4,1);
    bools_in(1:4,1) = [inpolygon(Con_wheel_1_II(i,1),Con_wheel_1_II(i,2),Polygon_track{1,1}(:,2),Polygon_track{1,1}(:,3));...
                       inpolygon(Con_wheel_1_II(i,1),Con_wheel_1_II(i,2),Polygon_track{2,1}(:,2),Polygon_track{2,1}(:,3));...
                       inpolygon(Con_wheel_1_II(i,1),Con_wheel_1_II(i,2),Polygon_track{3,1}(:,2),Polygon_track{3,1}(:,3));...
                       inpolygon(Con_wheel_1_II(i,1),Con_wheel_1_II(i,2),Polygon_track{4,1}(:,2),Polygon_track{4,1}(:,3))];
    pos_Con_Around_Deformed_track{i,1} = Polygon_track{find(bools_in,1),1}; % find(A,n) Find n elements that meet the A condition.
    pos_Con_Around_Deformed_WS{i,1} = Polygon_WS{find(bools_in,1),1};
    
    if Choose_Plot == 1
       figure(35); clf
       for mm = 1:1:4
           hold on
           plot(Polygon_track{mm,1}(:,2),Polygon_track{mm,1}(:,3));
       end
       hold on; plot(Con_wheel_1_II(i,1),Con_wheel_1_II(i,2),'bo');
       grid on
    end
    
    pos_Con_Around_Deformed_track{i,1}(end,:) = [];
    pos_Con_Around_Deformed_WS{i,1}(end,:) = [];
    
end

%%% 输出
% pos_Con_Around_Deformed_track：{num_Con*1}, 4*4, 在 Track C.S.下，每个接触点周围4个单元节点的位置
% pos_Con_Around_Deformed_WS：   {num_Con*1}, 4*4, 在 WS C.S.下，每个接触点周围4个单元节点的位置

%% 4. 计算接触点周围节点的绝对速度
vel_WS = vel_WS_global(1:3);
% 旋转顺序 3-1-2（翟院士专著计算车轮上接触点速度）：旋转矩阵 G' 的各列为沿 3 个卡尔丹转动轴的单位矢量在随体坐标系中的坐标列阵
G_Cardan_WS_Body = [1   0   0;...
                                          0   1   sin(pos_WS_track(4));...
                                          0   0   cos(pos_WS_track(4))];
Angvel_WS_track_Body = G_Cardan_WS_Body * [vel_WS_track(4); (-Vlc/R0 + vel_WS_track(5)); vel_WS_track(6)];

for i = 1:1:size(Con_wheel_1_II,1)
    vel_Con_Around_Deformed_track{i,1}(:,1) = pos_Con_Around_Deformed_track{i,1}(:,1);
    for j = 1:1:4        
        u = pos_Con_Around_Deformed_WS{i,1}(j,2:4);        
        u_Antisym = [0, -u(3), u(2);...
                     u(3), 0, -u(1);...
                    -u(2), u(1), 0];        
        bools = [N_Node.FW*0, N_Node.FW*1, N_Node.FW*2] + pos_Con_Around_Deformed_WS{i,1}(j,1);
        vel_Con_Around_Deformed_track{i,1}(j,2:4) = (vel_WS' - A_WS'*u_Antisym*Angvel_WS_track_Body + ...
                                                                                           A_WS'*(ModeShape.FW(bools,:)*Zsd(N_track+NM_FW*(i11-1)+1:N_track+NM_FW*i11,4)))';
    end
end

%%% 输出
% vel_Con_Around_Deformed_track：{num_Con*1}, 4*4, 在 Track C.S.下，每个接触点周围4个单元节点的速度

%% 5. 利用形函数，计算轮对上接触点的绝对速度
Coor_Target_local_XOY = zeros(size(Con_wheel_1_II,1),2);
pos_Con_Around_Deformed_track_XOY = cell(size(Con_wheel_1_II,1),1);
pos_Con_Around_Deformed_WS_XOY = cell(size(Con_wheel_1_II,1),1);
vel_Con_Around_Deformed_track_XOY = cell(size(Con_wheel_1_II,1),1);

Coor_Target_local_YOZ = zeros(size(Con_wheel_1_II,1),2);
pos_Con_Around_Deformed_track_YOZ = cell(size(Con_wheel_1_II,1),1);
pos_Con_Around_Deformed_WS_YOZ = cell(size(Con_wheel_1_II,1),1);
vel_Con_Around_Deformed_track_YOZ = cell(size(Con_wheel_1_II,1),1);

Vjd = zeros(size(Con_wheel_1_II,1),3);

for i = 1:1:size(Con_wheel_1_II,1)
    
    % X、Y 方向
    Coor_Around_track_XOY_ori = pos_Con_Around_Deformed_track{i,1}(:,2:3);
    Coor_Target_track_XOY     = Con_wheel_1_II(i,1:2);

    [Coor_Target_local_XOY(i,1:2), pos_Con_Around_Deformed_track_XOY{i,1}, pos_Con_Around_Deformed_WS_XOY{i,1}, vel_Con_Around_Deformed_track_XOY{i,1}] = ...
    Coor_Isoparametric('XOY', Coor_Around_track_XOY_ori, Coor_Target_track_XOY, pos_Con_Around_Deformed_track{i,1},...
                       pos_Con_Around_Deformed_WS{i,1}, vel_Con_Around_Deformed_track{i,1}, Par_FW);
    Vjd(i,1) = Shape_Isoparametric('X', Coor_Target_local_XOY(i,1:2), vel_Con_Around_Deformed_track_XOY{i,1});
    Vjd(i,2) = Shape_Isoparametric('Y', Coor_Target_local_XOY(i,1:2), vel_Con_Around_Deformed_track_XOY{i,1});
    
    % Y、Z 方向（假设Z方向均施加在轮对主平面内）（略有疑问 210630）
    Coor_Around_track_YOZ_ori = pos_Con_Around_Deformed_track{i,1}(:,3:4);
    Coor_Target_track_YOZ  = Con_wheel_1_II(i,2:3); 
    temp = sortrows(pos_Con_Around_Deformed_track{i,1},1);
    if Coor_Target_track_YOZ(1) > max(temp(1:2,3)) || Coor_Target_track_YOZ(1) < min(temp(1:2,3))
        [~,pos] = min(Coor_Target_track_YOZ(1)-temp(1:2,3));
        Coor_Target_track_YOZ(1) = temp(pos,3);
    end
    
    [Coor_Target_local_YOZ(i,1:2), pos_Con_Around_Deformed_track_YOZ{i,1}, pos_Con_Around_Deformed_WS_YOZ{i,1}, vel_Con_Around_Deformed_track_YOZ{i,1}] = ...
    Coor_Isoparametric('YOZ', Coor_Around_track_YOZ_ori, Coor_Target_track_YOZ, pos_Con_Around_Deformed_track{i,1},...
                       pos_Con_Around_Deformed_WS{i,1}, vel_Con_Around_Deformed_track{i,1}, Par_FW);
    Vjd(i,3) = Shape_Isoparametric('Z', Coor_Target_local_YOZ(i,1:2), vel_Con_Around_Deformed_track_YOZ{i,1});
    
end


