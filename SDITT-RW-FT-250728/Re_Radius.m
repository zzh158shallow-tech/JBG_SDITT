%% 后处理接触位置曲率半径
function [R_yy_w, R_xx_w, R_xx_r, rou] = Re_Radius(profile_w_Radius, profile_r_Radius, Con_wheel_2, Con_rail_1)

% Con_wheel_2 = Con_wheel_L_2_I;
% Con_rail_1 = Con_rail_L_1_I;

%% R_yy_w
R_yy_w = Con_wheel_2(:,3);

%% R_xx_w
R_xx_w = interp1(profile_w_Radius(:,1),profile_w_Radius(:,2),Con_wheel_2(:,2),'linear');

%% R_xx_r
R_xx_r = interp1(profile_r_Radius(:,1),profile_r_Radius(:,2),Con_rail_1(:,1),'linear');

% 修正用
bools_r_3 = (R_xx_w<0 & abs(R_xx_w)*0.9<=abs(R_xx_r));
R_xx_r(bools_r_3,:) = abs(R_xx_w(bools_r_3,:))*0.9;

bools_r_4 = abs(R_xx_r)>1;
R_xx_r(bools_r_4,:) = 1;

%% rou
rou = 4./(1./R_yy_w+1./R_xx_w+1./R_xx_r);
