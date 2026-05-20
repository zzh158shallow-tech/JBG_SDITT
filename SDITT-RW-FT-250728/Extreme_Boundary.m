%% 判断一阶导数极值点、可能的多个接触点
function [Ver_Dis_1_Extreme,Ver_Dis_2_postive, Ver_Dis_start, Ver_Dis_end] = Extreme_Boundary(Ver_Dis, Opt_1)

% Ver_Dis = Elastic_pen_R;
% Opt_1 = 'max';

Ver_Dis_1 = [Ver_Dis(:,1) gradient(Ver_Dis(:,2))./gradient(Ver_Dis(:,1))];
Ver_Dis_2 = [Ver_Dis(:,1) gradient(Ver_Dis_1(:,2))./gradient(Ver_Dis_1(:,1))];

bools = (Ver_Dis(1:end-1,2) >= 0) & ((Ver_Dis_1(1:end-1,2)<0 & Ver_Dis_1(2:end,2)>=0) | (Ver_Dis_1(1:end-1,2)>=0 & Ver_Dis_1(2:end,2)<0));
Ver_Dis_1_Extreme = [find(bools) Ver_Dis(bools,:) Ver_Dis_1(bools,2)];

% 计算垂向渗透量大于0的起止范围
bools_start = (Ver_Dis(1:end-1,2)<0 & Ver_Dis(2:end,2)>=0);
bools_end = (Ver_Dis(1:end-1,2)>=0 & Ver_Dis(2:end,2)<0);

% if isempty(find(bools_start, 1))
%     bools_start(1) = true;
% end
% if isempty(find(bools_end, 1))
%     bools_end(end) = true;
% end

% Ver_Dis_start = [find(bools_start) Ver_Dis(bools_start,:) Ver_Dis_1(bools_start,2)];
Ver_Dis_start = [find(bools_start)+1 Ver_Dis(find(bools_start)+1,:) Ver_Dis_1(find(bools_start)+1,2)];
Ver_Dis_end = [find(bools_end) Ver_Dis(bools_end,:) Ver_Dis_1(bools_end,2)];

if isempty(Ver_Dis_start)
    Ver_Dis_2_postive = [];
else    
    if strcmp(Opt_1,'min')
        bools_min = (Ver_Dis(1:end-1,2) >= 0) & (Ver_Dis_1(1:end-1,2)<0 & Ver_Dis_1(2:end,2)>=0);
        Ver_Dis_2_postive = [find(bools_min) Ver_Dis(bools_min,:) Ver_Dis_1(bools_min,2)];
    elseif strcmp(Opt_1,'max')
        for i = 1:1:size(Ver_Dis_start,1)
            p = Ver_Dis_start(i,1);
            q = Ver_Dis_end(i,1);
            [~,pos] = max(Ver_Dis(p:q,2));
            Ver_Dis_2_postive(i,:) = [p+pos-1, Ver_Dis(p+pos-1,:), Ver_Dis_1(p+pos-1,2)];
        end
    end
end