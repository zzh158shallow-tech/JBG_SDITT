function [SleeperNodes_pos, SleeperNodes_pos_x, SleeperNodes_pos_y, SleeperNodes_count, Sleeper_Length] = ...
          Get_FEM_Sleeper_v2(RailNodes_LSR, RailNodes_pos_v2, Area_s, Density_s)

clear SleeperNodes_pos SleeperNodes_pos_x SleeperNodes_pos_y ...
      Sleeper_Length Sleeper_Mass SleeperNodes_All_num SleeperNodes_All_pos

load AbaqusInput
load AbaqusInputTrackExtension
Choose_plot = 0;

%% 整理 Sleeper 节点
for i = 1:1:size(SleeperNodes,1)
   for j = 1:1:size(SleeperNodes,2)
       pos = SleeperNodes(i,j).location;
       if ~isempty(pos)
           pos(1) = pos(1) + sleeperPos(i);
           pos(2) = pos(2) - 0.75;
           SleeperNodes_All_num(i,j) = SleeperNodes(i,j).number;
           SleeperNodes_All_pos{i,j} = [pos(1) pos(2) 0];
       end
   end
end

k = 1;
for i = size(SleeperNodesTrackExtension,1):-1:1
   for j = 1:1:size(SleeperNodesTrackExtension,2)
       pos = SleeperNodesTrackExtension(i,j).location;
       if ~isempty(pos)
           pos(1) = pos(1) - sleeperPosTrackExtension(i);
           pos(2) = pos(2) - 0.75;
           SleeperNodesTrackExtension_num(k,j) = SleeperNodesTrackExtension(i,j).number;
           SleeperNodesTrackExtension_pos{k,j} = [pos(1) pos(2) 0];
       end
   end
   k = k+1;
end

SleeperNodes_All_num = [SleeperNodesTrackExtension_num zeros(length(SleeperNodesTrackExtension_num),31-9); SleeperNodes_All_num];
SleeperNodes_All_pos = [SleeperNodesTrackExtension_pos cell(length(SleeperNodesTrackExtension_pos),31-9); SleeperNodes_All_pos];

Ns_x = length(RailNodes_LSR)/2;
Ns_y = size(SleeperNodes_All_num,2);
SleeperNodes_pos = cell(Ns_x,Ns_y);
SleeperNodes_pos_x = zeros(Ns_x,Ns_y);
SleeperNodes_pos_y = zeros(Ns_x,Ns_y);
SleeperNodes_count = zeros(Ns_x,1);
Sleeper_Length = zeros(Ns_x,Ns_y);

%% 按照横坐标顺序整理 Sleeper 节点（所有节点均考虑）
SleeperNodes_pos = SleeperNodes_All_pos;
for i = 1:1:size(SleeperNodes_All_pos,1)
    for j = 1:1:size(SleeperNodes_All_pos,2)
        if ~isempty(SleeperNodes_All_pos{i,j})
            SleeperNodes_pos_x(i,j) = SleeperNodes_All_pos{i,j}(1);
            SleeperNodes_pos_y(i,j) = SleeperNodes_All_pos{i,j}(2);
            SleeperNodes_count(i,1) = SleeperNodes_count(i,1)+1;
        end
    end  
    Sleeper_Length(i,1:SleeperNodes_count(i)-1) = diff(SleeperNodes_pos_y(i,1:1:SleeperNodes_count(i)));
end

%% 绘图对比
if Choose_plot == 1
    figure(11); clf
    for i = 1:1:6
        eval(['temp = RailNodes_',Type_Rail{i},';']);
        for j = 1:1:length(temp)
            hold on
            plot(temp(j).location(1),temp(j).location(2),'*');
        end
    end
    grid on; set(gca, 'ydir', 'reverse');
end

% %% 按照横坐标顺序整理 Sleeper 节点（只考虑岔枕两端节点和扣件约束节点）
% for i_rail = 1:2:length(RailNodes_LSR)-1
%     i_sleeper = (i_rail+1)/2;   
%     % 临时存放岔枕节点的矩阵
%     SleeperNodes_pos_temp = [];
%     % 左侧端点
%     SleeperNodes_pos_temp(1,:) = SleeperNodes_All_pos{i_sleeper,1};
%     % 与钢轨扣件约束点，LSR第 i_rail 行，对应 RailNodes_pos_v2 矩阵第 2*i_rail-1
%     for j = 1:1:4
%         if ~isempty(RailNodes_pos_v2{2*i_rail-1,j})
%             if abs(RailNodes_pos_v2{2*i_rail-1,j}(2)-SleeperNodes_pos_temp(end,2))>1e-6
%                 SleeperNodes_pos_temp = [SleeperNodes_pos_temp; RailNodes_pos_v2{2*i_rail-1,j}];
%             end
%         end
%     end
%     % 末端节点
%     % Replace max(find(A)) with find(A, 1, 'last').
%     q = find(SleeperNodes_All_num(i_sleeper,:)>0, 1, 'last');
%     SleeperNodes_pos_temp = [SleeperNodes_pos_temp; SleeperNodes_All_pos{i_sleeper,q}];
%     
%     SleeperNodes_pos_temp = sortrows(SleeperNodes_pos_temp,2);    
%     % 将岔枕节点赋予至 SleeperNodes_pos
%     for i_temp = 1:1:size(SleeperNodes_pos_temp,1)
%         SleeperNodes_pos{i_sleeper,i_temp} = SleeperNodes_pos_temp(i_temp,:);
%         SleeperNodes_pos_x(i_sleeper,i_temp) = SleeperNodes_pos_temp(i_temp,1);
%         SleeperNodes_pos_y(i_sleeper,i_temp) = SleeperNodes_pos_temp(i_temp,2);
%     end
%     SleeperNodes_count(i_sleeper,1) = size(SleeperNodes_pos_temp,1);
%     Sleeper_Length(i_sleeper,1:size(SleeperNodes_pos_temp,1)-1) = diff(SleeperNodes_pos_y(i_sleeper,1:size(SleeperNodes_pos_temp,1)));
% end
