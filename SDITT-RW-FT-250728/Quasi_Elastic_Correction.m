%% 考虑准弹性接触参数修正后的潜在接触点在 Track C.S. 中的位置信息

function [Con_wheel_1_Ia, Con_rail_1_Ia, Ver_Pen_b, Con_wheel_1_Ib, Con_rail_1_Ib] = Quasi_Elastic_Correction...
         (Elastic_pen, Ver_Pen_a, Ver_Pen_postive_start, Ver_Pen_postive_end, Yw, Roll, Con_ang, A_track, bools, wheel_interp, rail_interp, Con_ang_temp)

% Elastic_pen = Elastic_pen_L;
% Ver_Pen_a = Ver_Pen_L_a;
% Ver_Pen_postive_start = Ver_Pen_L_postive_start;
% Ver_Pen_postive_end = Ver_Pen_L_postive_end;
% traceline_w = traceline_w_L;
% Yw = Yw_DWL;
% roll_dyn = Roll_DWL;
% Con_ang = Con_ang_L;
% A_track = A_DWL;
% Opt_side = 'L';
% bools = bools_L;
% wheel_interp = wheel_interp_L;
% rail_interp = rail_interp_L;
% Con_ang_temp = Con_ang_L_temp;

if size(Ver_Pen_a,1) ~= 0
    % 计算修正后接触点信息
    theta = 2e-5;
    for i = 1:1:size(Ver_Pen_a,1)
        pen_max = Ver_Pen_a(i,3);
        p = Ver_Pen_postive_start(i,1);
        q = Ver_Pen_postive_end(i,1);
        fun_weight = exp((Elastic_pen(p:q,2)-pen_max)/theta);
        dis_y = diff(Elastic_pen(p:q+1,1));
        Ver_Pen_b(i,1) = sum(Elastic_pen(p:q,1).*fun_weight.*dis_y) ./ sum(fun_weight.*dis_y);
% 	    Ver_Pen_b(i,1) = Ver_Pen_a(i,2);		% TEST AND COMP!!!!!!
        Ver_Pen_b(i,2) = interp1(Elastic_pen(:,1), Elastic_pen(:,2), Ver_Pen_b(i,1), 'linear');
    end
    
    % Con_wheel_1_I N*6 绝对坐标系下车轮上接触点位置 轮轨垂向渗透量 轮轨法向间隙 接触角
    Con_wheel_1_Ib = [interp1(wheel_interp(:,2), wheel_interp(:,1), Ver_Pen_b(:,1), 'linear'), Ver_Pen_b(:,1),...
                                   interp1(wheel_interp(:,2), wheel_interp(:,3), Ver_Pen_b(:,1), 'linear')];
%     temp = (Con_wheel_1_Ib - repmat([0, Yw, 0],size(Con_wheel_1_Ib,1),1)) * inv(A_track);
    temp = (Con_wheel_1_Ib - repmat([0, Yw, 0],size(Con_wheel_1_Ib,1),1)) / A_track;
    Con_ang_Ver_Pen = interp1(Con_ang(:,1), Con_ang(:,2), temp(:,2), 'linear');

%     Nor_Dis(:,1) = Ver_Pen_b(:,2).*cos(Con_ang_Ver_Pen(:,1)+Roll);
    Nor_Dis(:,1) = Ver_Pen_b(:,2)./cos(Con_ang_Ver_Pen(:,1)+Roll);
    Con_wheel_1_Ib = [Con_wheel_1_Ib, Ver_Pen_b(:,2), Nor_Dis, Con_ang_Ver_Pen];
    Con_rail_1_Ib = [Con_wheel_1_Ib(:,2), interp1(rail_interp(:,1), rail_interp(:,2), Con_wheel_1_Ib(:,2),'linear')];
    
    % 计算修正前接触点信息
    p = Ver_Pen_a(:,1);
    Con_ang_Ver_Pen = Con_ang_temp(bools);
    Con_ang_Ver_Pen = Con_ang_Ver_Pen(p);
%     Nor_Dis(:,1) = Ver_Pen_a(:,3).*cos(Con_ang_Ver_Pen(:,1)+Roll);
    Nor_Dis(:,1) = Ver_Pen_a(:,3)./cos(Con_ang_Ver_Pen(:,1)+Roll);
    Con_wheel_1_Ia = [wheel_interp(p,:), Ver_Pen_a(:,3), Nor_Dis, Con_ang_Ver_Pen];
    Con_rail_1_Ia = rail_interp(p,:);
    
else
    % 无接触点
    Con_ang_inter = Con_ang_temp(bools);
    [~,p] = max(Elastic_pen(:,2));
%     Con_wheel_1_Ib = [wheel_interp(p,:), Elastic_pen(p,2), Elastic_pen(p,2).*cos(Con_ang_inter(p,1)+Roll), Con_ang_inter(p,1)];
    Con_wheel_1_Ib = [wheel_interp(p,:), Elastic_pen(p,2), Elastic_pen(p,2)./cos(Con_ang_inter(p,1)+Roll), Con_ang_inter(p,1)];
    Con_wheel_1_Ia = Con_wheel_1_Ib;
    Con_rail_1_Ib = rail_interp(p,:);
    Con_rail_1_Ia = Con_rail_1_Ib;
    Ver_Pen_b = [];
end
