function Profile_TrackCS = Get_Profile_P2_v2(InpPar, Par_Track, i11, Dis_Rail, Mileage, RailPro_ProCS, Dis_TIrr_temp)

% global InpPar.Exp_DummyRail_WheelSide InpPar.N_ConPatch 
% global Par_Track InpPar.N_ConPatch_L InpPar.N_ConPatch_R
% global InpPar.Exp_DummyRail InpPar.Exp_DummyRail_L InpPar.Exp_DummyRail_R

clear Profile_TrackCS

Choose_Plot = 0;

%% 将钢轨廓形从 Profile CS 转化至 Track CS
for i2 = 1:1:InpPar.N_ConPatch
    Target_DummyRail = InpPar.Exp_DummyRail{i2};
    Profile_ProCS_inp = RailPro_ProCS.(Target_DummyRail);
    
    [profile_Radius, profile_r, profile_Front, profile_Rear, profile_Front_d1, profile_Rear_d1, dY, dZ] = ...
    Offset_profile_r(InpPar, InpPar.Exp_DummyRail_WheelSide{1,i2}, i11, i2, Par_Track.Ori_prr, Mileage, Dis_Rail, Profile_ProCS_inp, Dis_TIrr_temp);

    Profile_TrackCS.profile_r.(Target_DummyRail) = profile_r;
    Profile_TrackCS.profile_r_Radius.(Target_DummyRail) = profile_Radius;
    Profile_TrackCS.profile_Front.(Target_DummyRail) = profile_Front;
    Profile_TrackCS.profile_Rear.(Target_DummyRail) = profile_Rear;
    Profile_TrackCS.profile_Front.([Target_DummyRail, '_d1']) = profile_Front_d1;
    Profile_TrackCS.profile_Rear.([Target_DummyRail, '_d1']) = profile_Rear_d1;
    Profile_TrackCS.profile_r_dY.(Target_DummyRail) = dY;
    Profile_TrackCS.profile_r_dZ.(Target_DummyRail) = dZ;

end

%% 合并廓形和曲率矩阵
% Left: profile_r_L, profile_r_L1, profile_r_L2
if InpPar.N_ConPatch_L == 1
    T = InpPar.Exp_DummyRail_L{1};
    Profile_TrackCS.profile_r.L = Profile_TrackCS.profile_r.(T);
    Profile_TrackCS.profile_r.L = sortrows(Profile_TrackCS.profile_r.L,1);
    Profile_TrackCS.profile_r_Radius.L = Profile_TrackCS.profile_r_Radius.(T);
    Profile_TrackCS.profile_r_Radius.L = sortrows(Profile_TrackCS.profile_r_Radius.L,1);
else
    % 判断有多少组实际参与运算的钢轨廓形
    bools_r_nonemtpy = false(InpPar.N_ConPatch_L,1);
    for i = 1:1:InpPar.N_ConPatch_L
        T = InpPar.Exp_DummyRail_L{i};
        Target_r = Profile_TrackCS.profile_r.(T);
        bools_r_nonemtpy(i,1) = ~isempty(Target_r);
        if ~isempty(Target_r)
            bools_r.(T) = true(size(Target_r,1),1);
        end
    end
    % 非空集钢轨廓形对应的编号
    num_r_nonempty = find(bools_r_nonemtpy);
    % 合并赋值
    if length(num_r_nonempty) == 1
        T = InpPar.Exp_DummyRail_L{num_r_nonempty};
        Profile_TrackCS.profile_r.L = Profile_TrackCS.profile_r.(T);
        Profile_TrackCS.profile_r_Radius.L = Profile_TrackCS.profile_r_Radius.(T);
    else
        Profile_TrackCS.profile_r.L = [];
        Profile_TrackCS.profile_r_Radius.L = [];
        % 判断每组钢轨廓形实际赋予计算的横坐标范围
        for j = 1:1:length(num_r_nonempty)-1
            T1 = InpPar.Exp_DummyRail_L{num_r_nonempty(j)};
            T2 = InpPar.Exp_DummyRail_L{num_r_nonempty(j+1)};
            Target_r_outside = Profile_TrackCS.profile_r.(T1);
            Target_r_inside = Profile_TrackCS.profile_r.(T2);
            bools_r.(T1) = bools_r.(T1) & Target_r_outside(:,1) < min(max(Target_r_outside(:,1)),min(Target_r_inside(:,1)));
            bools_r.(T2) = bools_r.(T2) & Target_r_inside(:,1)    > max(max(Target_r_outside(:,1)),min(Target_r_inside(:,1)));
        end
        % Merge into the new profile data
        for j = 1:1:length(num_r_nonempty)
            T = InpPar.Exp_DummyRail_L{num_r_nonempty(j)};
            Profile_TrackCS.profile_r.L = [Profile_TrackCS.profile_r.L; Profile_TrackCS.profile_r.(T)(bools_r.(T),:)];
            Profile_TrackCS.profile_r_Radius.L = [Profile_TrackCS.profile_r_Radius.L; Profile_TrackCS.profile_r_Radius.(T)(bools_r.(T),:)];
        end
        Profile_TrackCS.profile_r.L = sortrows(Profile_TrackCS.profile_r.L,1);
        Profile_TrackCS.profile_r_Radius.L = sortrows(Profile_TrackCS.profile_r_Radius.L,1);
    end
end

if InpPar.N_ConPatch_R == 1
    T = InpPar.Exp_DummyRail_R{1};
    Profile_TrackCS.profile_r.R = Profile_TrackCS.profile_r.(T);
    Profile_TrackCS.profile_r.R = sortrows(Profile_TrackCS.profile_r.R,1);
    Profile_TrackCS.profile_r_Radius.R = Profile_TrackCS.profile_r_Radius.(T);
    Profile_TrackCS.profile_r_Radius.R = sortrows(Profile_TrackCS.profile_r_Radius.R,1);
else
    % 判断有多少组实际参与运算的钢轨廓形
    bools_r_nonemtpy = false(InpPar.N_ConPatch_R,1);
    for i = 1:1:InpPar.N_ConPatch_R
        T = InpPar.Exp_DummyRail_R{i};
        Target_r = Profile_TrackCS.profile_r.(T);
        bools_r_nonemtpy(i,1) = ~isempty(Target_r);
        if ~isempty(Target_r)
            bools_r.(T) = true(size(Target_r,1),1);
        end
    end
    % 非空集钢轨廓形对应的编号
    num_r_nonempty = find(bools_r_nonemtpy);
    % 合并赋值
    if length(num_r_nonempty) == 1
        T = InpPar.Exp_DummyRail_R{num_r_nonempty};
        Profile_TrackCS.profile_r.R = Profile_TrackCS.profile_r.(T);
        Profile_TrackCS.profile_r_Radius.R = Profile_TrackCS.profile_r_Radius.(T);
    else
        Profile_TrackCS.profile_r.R = [];
        Profile_TrackCS.profile_r_Radius.R = [];
        % 判断每组钢轨廓形实际赋予计算的横坐标范围
        for j = 1:1:length(num_r_nonempty)-1
            T1 = InpPar.Exp_DummyRail_R{num_r_nonempty(j)};
            T2 = InpPar.Exp_DummyRail_R{num_r_nonempty(j+1)};
            Target_r_outside = Profile_TrackCS.profile_r.(T1);
            Target_r_inside = Profile_TrackCS.profile_r.(T2);
            bools_r.(T1) = bools_r.(T1) & Target_r_outside(:,1) > max(min(Target_r_outside(:,1)),max(Target_r_inside(:,1)));
            bools_r.(T2) = bools_r.(T2) & Target_r_inside(:,1)    < min(min(Target_r_outside(:,1)),max(Target_r_inside(:,1)));
        end
        % Merge into the new profile data
        for j = 1:1:length(num_r_nonempty)
            T = InpPar.Exp_DummyRail_R{num_r_nonempty(j)};
            Profile_TrackCS.profile_r.R = [Profile_TrackCS.profile_r.R; Profile_TrackCS.profile_r.(T)(bools_r.(T),:)];
            Profile_TrackCS.profile_r_Radius.R = [Profile_TrackCS.profile_r_Radius.R; Profile_TrackCS.profile_r_Radius.(T)(bools_r.(T),:)];
        end
        Profile_TrackCS.profile_r.R = sortrows(Profile_TrackCS.profile_r.R,1);
        Profile_TrackCS.profile_r_Radius.R = sortrows(Profile_TrackCS.profile_r_Radius.R,1);
    end
end

%% 人工调整计算所得曲率半径
% bools_Re = (profile_r_L_Radius(:,2)>11e-3 & profile_r_L_Radius(:,2)<15e-3) | (profile_r_L_Radius(:,1)>-0.726);
% profile_r_L_Radius(bools_Re,2) = 13e-3;

%% 画图检查廓形
if Choose_Plot == 1
    % Left
    figure(10); clf
    plot(Profile_TrackCS.profile_r.L(:,1),Profile_TrackCS.profile_r.L(:,2));
    if ~isempty(Profile_TrackCS.profile_Front.L1)
        hold on
        plot(Profile_TrackCS.profile_Front.L1(:,1),Profile_TrackCS.profile_Front.L1(:,2),...
             Profile_TrackCS.profile_Rear.L1(:,1), Profile_TrackCS.profile_Rear.L1(:,2),'--');
    end
    set(gca,'ydir','reverse');  grid on;
    
    % Right
    figure(10); clf
    plot(Profile_TrackCS.profile_r.R(:,1),Profile_TrackCS.profile_r.R(:,2));
    if ~isempty(Profile_TrackCS.profile_Front.R1)
        hold on
        plot(Profile_TrackCS.profile_Front.R1(:,1),Profile_TrackCS.profile_Front.R1(:,2),'--',...
             Profile_TrackCS.profile_Rear.R1(:,1),Profile_TrackCS.profile_Rear.R1(:,2),'--');
    end
    if ~isempty(Profile_TrackCS.profile_Front.R2)
        hold on
        plot(Profile_TrackCS.profile_Front.R2(:,1),Profile_TrackCS.profile_Front.R2(:,2),'--',...
             Profile_TrackCS.profile_Rear.R2(:,1),Profile_TrackCS.profile_Rear.R2(:,2),'--');
    end
%     if ~isempty(Profile_TrackCS.profile_Front.R3)
%         hold on
%         plot(Profile_TrackCS.profile_Front.R3(:,1),Profile_TrackCS.profile_Front.R3(:,2),'--',...
%              Profile_TrackCS.profile_Rear.R3(:,1),Profile_TrackCS.profile_Rear.R3(:,2),'--');
%     end
    set(gca,'ydir','reverse');  grid on;
end

