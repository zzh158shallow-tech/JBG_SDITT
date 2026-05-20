%% 计算轮对坐标系下的接触角、车轮廓形曲率半径
function WheelPro_ProCS = Radius_wheel(InpPar, Par_Vehicle)

% global WheelPro_ProCS

clear profile_w_R profile_w_L
Choose_Plot = 0;

Drc = Par_Vehicle.Drc;
Dlb = Par_Vehicle.Dlb;
R0  = Par_Vehicle.R0;

%% 1. 接触角计算
% profile_w = load('LMA.txt');
% profile_w_R_0(:,1) = profile_w(:,1)+Drc+Dlb;
% profile_w_R_0(:,2) = profile_w(:,2)+R0;
% profile_w_R_0 = sortrows(profile_w_R_0,1);
% discrete_len = 0.2e-3;
% profile_w_R(:,1) = profile_w_R_0(1,1):discrete_len:profile_w_R_0(end,1);
% profile_w_R(:,2) = interp1(profile_w_R_0(:,1),profile_w_R_0(:,2),profile_w_R(:,1),'spline');
% profile_w_L(:,1) = -profile_w_R(:,1);
% profile_w_L(:,2) =  profile_w_R(:,2);
% profile_w_L = sortrows(profile_w_L,1);

if strcmp(InpPar.Type_Vehicle, 'CR400BF')
    profile_w = load('LMB10.txt');
    profile_w_R_0(:,1) = profile_w(:,1)+Drc+Dlb;
    profile_w_R_0(:,2) = profile_w(:,2)+R0;
else
    % profile_w = load('LMB10N.txt');
    profile_w = load('LMA_UnitMM.txt');
    profile_w_R_0(:,1) = profile_w(:,1)/1000+Drc+Dlb;
    profile_w_R_0(:,2) = profile_w(:,2)/1000+R0;
end

profile_w_R_0 = sortrows(profile_w_R_0,1);
profile_w_R = profile_w_R_0;
profile_w_L(:,1) = -profile_w_R(:,1);
profile_w_L(:,2) =  profile_w_R(:,2);
profile_w_L = sortrows(profile_w_L,1);

% %%% 接触角计算方法-1
% Con_ang_R_1 = [profile_w_R(:,1) abs(atan(gradient(profile_w_R(:,2))./gradient(profile_w_R(:,1))))];
% Con_ang_L_1 = [profile_w_L(:,1) abs(atan(gradient(profile_w_L(:,2))./gradient(profile_w_L(:,1))))];
Con_ang_R_1 = [profile_w_R(:,1) atan(gradient(profile_w_R(:,2))./gradient(profile_w_R(:,1)))];
Con_ang_L_1 = [profile_w_L(:,1) atan(gradient(profile_w_L(:,2))./gradient(profile_w_L(:,1)))];
%%% 接触角计算方法-2
% temp_R = abs(atan(diff(profile_w_R(:,2))./diff(profile_w_R(:,1))));
% temp_L = abs(atan(diff(profile_w_L(:,2))./diff(profile_w_L(:,1))));
temp_R = atan(diff(profile_w_R(:,2))./diff(profile_w_R(:,1)));
temp = profile_w_R(end,1)-profile_w_R(end-1,1);
Con_ang_R = [profile_w_R(1,1) temp_R(1,1); profile_w_R(2:end,1) temp_R; profile_w_R(end,1)+temp temp_R(end,1)];
Con_ang_L = Con_ang_R;
Con_ang_L(:,1) = -Con_ang_L(:,1);
Con_ang_L(:,2) = -Con_ang_L(:,2);
Con_ang_L = sortrows(Con_ang_L,1);

%%% 接触角对比绘图
if Choose_Plot == 1
    figure(7); clf
    subplot(2,1,1)
    plot(profile_w_R(:,1), profile_w_R(:,2));grid on; set(gca,'ydir','reverse');
    title('Right Wheel')
    subplot(2,1,2)
    plot(Con_ang_R_1(:,1), Con_ang_R_1(:,2), Con_ang_R(:,1), Con_ang_R(:,2));grid on
    
    figure(8); clf
    subplot(2,1,1)
    plot(profile_w_L(:,1), profile_w_L(:,2));grid on; set(gca,'ydir','reverse');
    title('Left Wheel')
    subplot(2,1,2)
    plot(Con_ang_L_1(:,1), Con_ang_L_1(:,2), Con_ang_L(:,1), Con_ang_L(:,2));grid on
end

%% 2. 曲率半径计算
%%% 右侧车轮曲率半径
if strcmp(InpPar.Type_Vehicle, 'CR400BF')
    % LMB10
%     profile_w_R_Radius = Radius_profile_v3(profile_w_R,1-5e-12,1,1);
    profile_w_R_Radius = Radius_profile_v3(profile_w_R,1-5e-10,1,1);
    profile_w_R_Radius(:,2) = -profile_w_R_Radius(:,2);

    xx = min(profile_w_R_Radius(:,1)) : 0.0001 : max(profile_w_R_Radius(:,1));
    yy = interp1(profile_w_R_Radius(:,1), profile_w_R_Radius(:,2), xx, 'linear');
    profile_w_R_Radius = [xx' yy'];

    %%% 右侧车轮曲率半径修正
%     bools_R_1 = profile_w_R_Radius(:,1) > 0.76 | abs(profile_w_R_Radius(:,2)) > 2;
    bools_R_1 = profile_w_R_Radius(:,1) > 0.76 | abs(profile_w_R_Radius(:,2)) > 0.3;
    profile_w_R_Radius(bools_R_1,2) = -inf;

else
    % LMA
    profile_w_R_Radius = Radius_profile_v3(profile_w_R,1-5e-12,1,1);
    profile_w_R_Radius(:,2) = -profile_w_R_Radius(:,2);

    %%% 右侧车轮曲率半径修正
    % bools_R_1 = profile_w_R_Radius(:,1) > 746.5e-3+4.2535e-3;
    bools_R_1 = profile_w_R_Radius(:,1) > 0.7512;
    profile_w_R_Radius(bools_R_1,2) = -inf;
    % bools_R_2 = profile_w_R_Radius(:,1) >= 746.5e-3 & profile_w_R_Radius(:,1) <= 746.5e-3+4.2535e-3;
    % profile_w_R_Radius(bools_R_1,2) = -0.45;
    % profile_w_R_Radius(bools_R_1,2) = interp1(profile_w_R_Radius(:,1),profile_w_R_Radius(:,2),profile_w_R_Radius(bools_R_2,1),'spline');

    % bools_R_1 = profile_w_R_Radius(:,1)>0.778 | (profile_w_R_Radius(:,1)>0.7612 & profile_w_R_Radius(:,1)<0.7641);
    % profile_w_R_Radius(bools_R_1,2) = inf;
    % bools_R_2 = profile_w_R_Radius(:,1)>0.705 & profile_w_R_Radius(:,1)<=0.7125;
    % profile_w_R_Radius(bools_R_2,2) = 0.02;
    % bools_R_3 = profile_w_R_Radius(:,1)>0.7125 & profile_w_R_Radius(:,1)<0.720;
    % temp = profile_w_R_Radius(bools_R_3,:);
    % [~,m] = min(abs(temp(:,2)-(-0.015)));
    % bools_R_4 = profile_w_R_Radius(:,1)>0.7125 & profile_w_R_Radius(:,1)<temp(m,1);
    % profile_w_R_Radius(bools_R_4,2) = -0.015;

end

profile_w_R_Radius = sortrows(profile_w_R_Radius,1);

if Choose_Plot == 1
    figure(7)
    subplot(2,1,1)
    plot(profile_w_R(:,1), profile_w_R(:,2)); grid on; set(gca,'ydir','reverse');
    title('Radius of Right Wheels');
    xlabel('Y [m]'); ylabel('Z [m]'); xlim([0.68, 0.82]);
    subplot(2,1,2)
    plot(profile_w_R_Radius(:,1), profile_w_R_Radius(:,2)); grid on;
    xlabel('Y [m]'); ylabel('Radius [m]'); xlim([0.68, 0.82]); ylim([-1,1])
end

% 左侧车轮
profile_w_L_Radius = profile_w_R_Radius;
profile_w_L_Radius(:,1) = -profile_w_L_Radius(:,1);
profile_w_L_Radius = sortrows(profile_w_L_Radius,1);

% profile_w_R, profile_w_L, Con_ang_R, Con_ang_L, profile_w_R_Radius, profile_w_L_Radius
WheelPro_ProCS.profile_w_R = profile_w_R;
WheelPro_ProCS.profile_w_L = profile_w_L;
WheelPro_ProCS.Con_ang_R = Con_ang_R;
WheelPro_ProCS.Con_ang_L = Con_ang_L;
WheelPro_ProCS.profile_w_R_Radius = profile_w_R_Radius;
WheelPro_ProCS.profile_w_L_Radius = profile_w_L_Radius;

