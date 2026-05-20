%% 判断一阶导数极值点、可能的多个接触点
function [Ver_Dis_1 Ver_Dis_2 Ver_Dis_1_Extreme Ver_Dis_2_postive] = Extreme_point(Ver_Dis, Opt_1)

% if ~exist('Opt_1')
%     Opt_1 = 'min';
% end

Ver_Dis_1 = [Ver_Dis(:,1) gradient(Ver_Dis(:,2))./gradient(Ver_Dis(:,1))];
Ver_Dis_2 = [Ver_Dis(:,1) gradient(Ver_Dis_1(:,2))./gradient(Ver_Dis_1(:,1))];

bools = (Ver_Dis_1(1:end-1,2)<0 & Ver_Dis_1(2:end,2)>=0) | (Ver_Dis_1(1:end-1,2)>=0 & Ver_Dis_1(2:end,2)<0);
Ver_Dis_1_Extreme = [find(bools) Ver_Dis(bools,:) Ver_Dis_1(bools,2)];

if exist('Opt_1')
    if strcmp(Opt_1,'min')
        bools_min = (Ver_Dis_1(1:end-1,2)<0 & Ver_Dis_1(2:end,2)>=0);
        Ver_Dis_2_postive = [find(bools_min) Ver_Dis(bools_min,:) Ver_Dis_1(bools_min,2)];
    elseif strcmp(Opt_1,'max')
        bools_max = (Ver_Dis_1(1:end-1,2)>=0 & Ver_Dis_1(2:end,2)<0);
        Ver_Dis_2_postive = [find(bools_max) Ver_Dis(bools_max,:) Ver_Dis_1(bools_max,2)];
    end
else
    Ver_Dis_2_postive = [];
end
    

% for i = 1:1:length(Ver_Dis)-1
%    if (Ver_Dis_1(i,2)<0 && Ver_Dis_1(i+1,2)>=0) || (Ver_Dis_1(i,2)>=0 && Ver_Dis_1(i+1,2)<0)
%        Ver_Dis_1_Extreme = [Ver_Dis_1_Extreme; i Ver_Dis(i,:) Ver_Dis_1(i,2)];
%    end
%    if strcmp(Opt_1,'min')
%        if (Ver_Dis_1(i,2)<0 && Ver_Dis_1(i+1,2)>=0)
%            Ver_Dis_2_postive = [Ver_Dis_2_postive; i Ver_Dis(i,:) Ver_Dis_1(i,2)];
%        end
%    elseif strcmp(Opt_1,'max')
%        if (Ver_Dis_1(i,2)>=0 && Ver_Dis_1(i+1,2)<0)
%            Ver_Dis_2_postive = [Ver_Dis_2_postive; i Ver_Dis(i,:) Ver_Dis_1(i,2)];
%        end
%    end
% end

% figure(2)
% if min(Ver_Dis(:,1))<=0
%     subplot(2,2,1)
%     hold on
%     plot(Ver_Dis(:,1),Ver_Dis(:,2));grid on
%     subplot(2,2,3)
%     hold on
%     plot(Ver_Dis_1(:,1),Ver_Dis_1(:,2));grid on
% else
%     subplot(2,2,2)
%     hold on
%     plot(Ver_Dis(:,1),Ver_Dis(:,2));grid on
%     subplot(2,2,4)
%     hold on
%     plot(Ver_Dis_1(:,1),Ver_Dis_1(:,2));grid on
% end
    
