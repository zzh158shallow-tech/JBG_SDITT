%% 等参元坐标变化

function [Coor_Target_local, pos_Con_Around_Deformed_track, pos_Con_Around_Deformed_WS, vel_Con_Around_Deformed_track] = ...
         Coor_Isoparametric(Opt, Coor_Around_global_ori, Coor_Target_global, pos_Con_Around_Deformed_track_ori,...
          pos_Con_Around_Deformed_WS_ori, vel_Con_Around_Deformed_track_ori, Par_FW)
      
% clc
% clear

% Opt = 'YOZ';
% Coor_Around_global_ori = Coor_Around_track_YOZ_ori;
% Coor_Target_global = Coor_Target_track_YOZ;
% 
% Opt = 'XOY';
% Coor_Around_global_ori = Coor_Around_track_XOY_ori;
% Coor_Target_global = Coor_Target_track_XOY;
% 
% pos_Con_Around_Deformed_track_ori = pos_Con_Around_Deformed_track{i,1};
% pos_Con_Around_Deformed_WS_ori = pos_Con_Around_Deformed_WS{i,1};
% vel_Con_Around_Deformed_track_ori = vel_Con_Around_Deformed_track{i,1};

%% 全局坐标系坐标
%%% 输入数据
Choose_Plot = 0;
x = Coor_Target_global(1);
y = Coor_Target_global(2);

%%% 坐标排序
% bools_1 = Coor_Around_global_ori(:,1) <= x & Coor_Around_global_ori(:,2) <= y;
% bools_2 = Coor_Around_global_ori(:,1) <= x & Coor_Around_global_ori(:,2) >= y;
% bools_3 = Coor_Around_global_ori(:,1) >= x & Coor_Around_global_ori(:,2) >= y;
% bools_4 = Coor_Around_global_ori(:,1) >= x & Coor_Around_global_ori(:,2) <= y;

% Coor_Around_global = [Coor_Around_global_ori(bools_1,:); Coor_Around_global_ori(bools_2,:); ...
%                       Coor_Around_global_ori(bools_3,:); Coor_Around_global_ori(bools_4,:)];
% pos_Con_Around_Deformed_track = [pos_Con_Around_Deformed_track_ori(bools_1,:); pos_Con_Around_Deformed_track_ori(bools_2,:); ...
%                                  pos_Con_Around_Deformed_track_ori(bools_3,:); pos_Con_Around_Deformed_track_ori(bools_4,:)];
% pos_Con_Around_Deformed_WS = [pos_Con_Around_Deformed_WS_ori(bools_1,:); pos_Con_Around_Deformed_WS_ori(bools_2,:); ...
%                               pos_Con_Around_Deformed_WS_ori(bools_3,:); pos_Con_Around_Deformed_WS_ori(bools_4,:)];
% vel_Con_Around_Deformed_track = [vel_Con_Around_Deformed_track_ori(bools_1,:); vel_Con_Around_Deformed_track_ori(bools_2,:); ...
%                                  vel_Con_Around_Deformed_track_ori(bools_3,:); vel_Con_Around_Deformed_track_ori(bools_4,:)];

Sortrows_x = sortrows([Coor_Around_global_ori,(1:4)'],1);
temp = Sortrows_x(1:2,:);
[~,pos] = min(temp(:,2));
pos_1 = temp(pos,3);
[~,pos] = max(temp(:,2));
pos_2 = temp(pos,3);
temp = Sortrows_x(3:4,:);
[~,pos] = min(temp(:,2));
pos_4 = temp(pos,3);
[~,pos] = max(temp(:,2));
pos_3 = temp(pos,3);

Coor_Around_global = [Coor_Around_global_ori(pos_1,:); Coor_Around_global_ori(pos_2,:); ...
                      Coor_Around_global_ori(pos_3,:); Coor_Around_global_ori(pos_4,:)];
                  
pos_Con_Around_Deformed_track = [pos_Con_Around_Deformed_track_ori(pos_1,:); pos_Con_Around_Deformed_track_ori(pos_2,:); ...
                                 pos_Con_Around_Deformed_track_ori(pos_3,:); pos_Con_Around_Deformed_track_ori(pos_4,:)];
pos_Con_Around_Deformed_WS = [pos_Con_Around_Deformed_WS_ori(pos_1,:); pos_Con_Around_Deformed_WS_ori(pos_2,:); ...
                              pos_Con_Around_Deformed_WS_ori(pos_3,:); pos_Con_Around_Deformed_WS_ori(pos_4,:)];
vel_Con_Around_Deformed_track = [vel_Con_Around_Deformed_track_ori(pos_1,:); vel_Con_Around_Deformed_track_ori(pos_2,:); ...
                                 vel_Con_Around_Deformed_track_ori(pos_3,:); vel_Con_Around_Deformed_track_ori(pos_4,:)];
    
if Choose_Plot == 1
    figure(25); clf
    Type_plot ={'r*','y*','b*','g*'};
    plot(Coor_Around_global(:,1),Coor_Around_global(:,2));
%     plot(Coor_Around_global(:,1),Coor_Around_global(:,2)+Zw);
    for i = 1:1:4
        hold on; plot(Coor_Around_global(i,1),Coor_Around_global(i,2),Type_plot{i});
%         hold on; plot(Coor_Around_global(i,1),Coor_Around_global(i,2)+Zw,Type_plot{i});
    end
    hold on; plot(x,y,'ko');
%     hold on; plot(x,y+Zw_DWR,'ko');
    grid on
end

Ay = (Coor_Around_global(3,2)+Coor_Around_global(4,2)) - (Coor_Around_global(1,2)+Coor_Around_global(2,2));
By = (Coor_Around_global(3,2)-Coor_Around_global(4,2)) - (Coor_Around_global(1,2)-Coor_Around_global(2,2));
Cy = (Coor_Around_global(1,2)-Coor_Around_global(2,2)) + (Coor_Around_global(3,2)-Coor_Around_global(4,2));
Dy = 4*y - sum(Coor_Around_global(:,2));

Ax = (Coor_Around_global(3,1)+Coor_Around_global(4,1)) - (Coor_Around_global(1,1)+Coor_Around_global(2,1));
Bx = (Coor_Around_global(3,1)-Coor_Around_global(4,1)) - (Coor_Around_global(1,1)-Coor_Around_global(2,1));
Cx = (Coor_Around_global(1,1)-Coor_Around_global(2,1)) + (Coor_Around_global(3,1)-Coor_Around_global(4,1));
Dx = 4*x - sum(Coor_Around_global(:,1));

% temp = [Ay, By, Cy, Dy; Ax, Bx, Cx, Dx];

% Ay=1;
% By=2;
% Cy=3;
% Dy=4;
% Ax=5;
% Bx=6;
% Cx=7;
% Dx=8;

%% 等参变换至边长为2的正方形等参元
if strcmp(Opt,'XOY')
    if (Ay == 0 && Cy == 0) || (By == 0 && Cy == 0) || (Ax == 0 && Cx == 0) || (Bx == 0 && Cx == 0)
        syms zeta_v2 eta_v2
        Eq_1 = Ay*zeta_v2 + By*eta_v2 + Cy*zeta_v2*eta_v2 - Dy;
        Eq_2 = Ax*zeta_v2 + Bx*eta_v2 + Cx*zeta_v2*eta_v2 - Dx;
        output = solve(Eq_1, Eq_2, zeta_v2, eta_v2);
        Coor_Target_local = [double(output.zeta_v2)'; double(output.eta_v2)'];
    else
%         clear zeta eta
%         syms Algebra_zeta Algebra_eta Algebra_Ay Algebra_By Algebra_Cy Algebra_Dy Algebra_Ax Algebra_Bx Algebra_Cx Algebra_Dx
%         Eq_1 = Algebra_Ay*Algebra_zeta + Algebra_By*Algebra_eta + Algebra_Cy*Algebra_zeta*Algebra_eta - Algebra_Dy;
%         Eq_2 = Algebra_Ax*Algebra_zeta + Algebra_Bx*Algebra_eta + Algebra_Cx*Algebra_zeta*Algebra_eta - Algebra_Dx;
%         [zeta, eta] = solve(Eq_1, Eq_2, Algebra_zeta, Algebra_eta);
        Coor_Target_local = [double(subs(Par_FW.zeta, [Par_FW.Algebra_Ay, Par_FW.Algebra_By, Par_FW.Algebra_Cy, Par_FW.Algebra_Dy, Par_FW.Algebra_Ax, Par_FW.Algebra_Bx, Par_FW.Algebra_Cx, Par_FW.Algebra_Dx], [Ay,By,Cy,Dy,Ax,Bx,Cx,Dx]))';...
                             double(subs(Par_FW.eta,  [Par_FW.Algebra_Ay, Par_FW.Algebra_By, Par_FW.Algebra_Cy, Par_FW.Algebra_Dy, Par_FW.Algebra_Ax, Par_FW.Algebra_Bx, Par_FW.Algebra_Cx, Par_FW.Algebra_Dx], [Ay,By,Cy,Dy,Ax,Bx,Cx,Dx]))'];
    end
elseif strcmp(Opt,'YOZ')
    Coor_Target_local = [(Dx-Bx)/(Ax+Cx); 1];    
end

% if ~(Ay == 0 && Cy == 0)
%     clear a b c Coor_Target_local
%     a = Bx*Cy - Cx*By;
%     b = -Ax*By + Bx*Ay + Cx*Dy - Dx*Cy;
%     c = Ax*Dy - Dx*Ay;
%     if a ~= 0
%         Coor_Target_local(2,:) = [(-b + sqrt(b^2-4*a*c))/(2*a), (-b - sqrt(b^2-4*a*c))/(2*a)];
%     else
%         Coor_Target_local(2,1) = -c/b;
%     end
%     (Dy-By*Coor_Target_local(2,2))./(Ay+Cy*Coor_Target_local(2,2));
%     Coor_Target_local(1,:) = (Dy-By*Coor_Target_local(2,:))./(Ay+Cy*Coor_Target_local(2,:));
% elseif ~(By == 0 && Cy == 0)
%     clear a b c Coor_Target_local
%     a = Ax*Cy - Cx*Ay;
%     b = Ax*By - Bx*Ay + Cx*Dy - Dx*Cy;
%     c = Bx*Dy - Dx*By;
%     if a ~= 0
%         Coor_Target_local(1,:) = [(-b + sqrt(b^2-4*a*c))/(2*a), (-b - sqrt(b^2-4*a*c))/(2*a)];
%     else
%         Coor_Target_local(1,1) = -c/b;
%     end
%     Coor_Target_local(2,:) = (Dy-Ay*Coor_Target_local(1,:))./(By+Cy*Coor_Target_local(1,:));
% end

bools_del = zeros(1,size(Coor_Target_local,2));
for j = 1:1:size(Coor_Target_local,2)
    bools_del(1,j) = ~(min(Coor_Target_local(:,j))>=-1-1e-6 & max(Coor_Target_local(:,j))<=1+1e-6);
end
Coor_Target_local(:,find(bools_del)) = [];
Coor_Target_local = Coor_Target_local';

if isempty(Coor_Target_local) || size(Coor_Target_local,1)>1
    temp = input('Error: Coor_Isoparametric.');
end

% Coor_Target_local
