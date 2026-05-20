%% 利用 Bezier 曲线沿纵向插值截面
function [Profile_ori_L1, Profile_ori_L2, Profile_ori_R1, Profile_ori_R2] = ...
    Get_Profile_P1_Curve(Mileage_sum_Stock, Mileage_sum_Switch, Mileage_sum_Crossing, Mileage_sum_Check,...
    Profile_Crossing_Bezier,MileageInterp_Crossing_div,Profile_Switch_Bezier,MileageInterp_Switch_div,...
    Expression_WS, j1, Distance_Vehicle, Expression_DummyRail)

%% 参数设置
Choose_Plot = 0;
num_interp_Bezier = 1000;
Profile_ori_L1 = struct('FF_Profile_num', [],'FF_Profile', [], 'FF_FrontProfile', [], 'FF_RearProfile', [], 'FF_FrontProfile_d1', [], 'FF_RearProfile_d1', [], 'FF_Radius', [], ...
                        'FR_Profile_num', [],'FR_Profile', [], 'FR_FrontProfile', [], 'FR_RearProfile', [], 'FR_FrontProfile_d1', [], 'FR_RearProfile_d1', [], 'FR_Radius', [], ...
                        'RF_Profile_num', [],'RF_Profile', [], 'RF_FrontProfile', [], 'RF_RearProfile', [], 'RF_FrontProfile_d1', [], 'RF_RearProfile_d1', [], 'RF_Radius', [], ...
                        'RR_Profile_num', [],'RR_Profile', [], 'RR_FrontProfile', [], 'RR_RearProfile', [], 'RR_FrontProfile_d1', [], 'RR_RearProfile_d1', [], 'RR_Radius', []);
Profile_ori_L2 = struct('FF_Profile_num', [],'FF_Profile', [], 'FF_FrontProfile', [], 'FF_RearProfile', [], 'FF_FrontProfile_d1', [], 'FF_RearProfile_d1', [], 'FF_Radius', [], ...
                        'FR_Profile_num', [],'FR_Profile', [], 'FR_FrontProfile', [], 'FR_RearProfile', [], 'FR_FrontProfile_d1', [], 'FR_RearProfile_d1', [], 'FR_Radius', [], ...
                        'RF_Profile_num', [],'RF_Profile', [], 'RF_FrontProfile', [], 'RF_RearProfile', [], 'RF_FrontProfile_d1', [], 'RF_RearProfile_d1', [], 'RF_Radius', [], ...
                        'RR_Profile_num', [],'RR_Profile', [], 'RR_FrontProfile', [], 'RR_RearProfile', [], 'RR_FrontProfile_d1', [], 'RR_RearProfile_d1', [], 'RR_Radius', []);
Profile_ori_R1 = struct('FF_Profile_num', [],'FF_Profile', [], 'FF_FrontProfile', [], 'FF_RearProfile', [], 'FF_FrontProfile_d1', [], 'FF_RearProfile_d1', [], 'FF_Radius', [], ...
                        'FR_Profile_num', [],'FR_Profile', [], 'FR_FrontProfile', [], 'FR_RearProfile', [], 'FR_FrontProfile_d1', [], 'FR_RearProfile_d1', [], 'FR_Radius', [], ...
                        'RF_Profile_num', [],'RF_Profile', [], 'RF_FrontProfile', [], 'RF_RearProfile', [], 'RF_FrontProfile_d1', [], 'RF_RearProfile_d1', [], 'RF_Radius', [], ...
                        'RR_Profile_num', [],'RR_Profile', [], 'RR_FrontProfile', [], 'RR_RearProfile', [], 'RR_FrontProfile_d1', [], 'RR_RearProfile_d1', [], 'RR_Radius', []);
Profile_ori_R2 = struct('FF_Profile_num', [],'FF_Profile', [], 'FF_FrontProfile', [], 'FF_RearProfile', [], 'FF_FrontProfile_d1', [], 'FF_RearProfile_d1', [], 'FF_Radius', [], ...
                        'FR_Profile_num', [],'FR_Profile', [], 'FR_FrontProfile', [], 'FR_RearProfile', [], 'FR_FrontProfile_d1', [], 'FR_RearProfile_d1', [], 'FR_Radius', [], ...
                        'RF_Profile_num', [],'RF_Profile', [], 'RF_FrontProfile', [], 'RF_RearProfile', [], 'RF_FrontProfile_d1', [], 'RF_RearProfile_d1', [], 'RF_Radius', [], ...
                        'RR_Profile_num', [],'RR_Profile', [], 'RR_FrontProfile', [], 'RR_RearProfile', [], 'RR_FrontProfile_d1', [], 'RR_RearProfile_d1', [], 'RR_Radius', []);

%% 模拟区间线路
% for i11 = 1:1:4
%     for kk = 1:3:length(Expression_DummyRail)
%         eval(['profile_', Expression_DummyRail{kk},'_num = 1;']);
%         filename_profile = 'Crossing_Section1.txt';
%         eval(['profile_r_ori_', Expression_DummyRail{kk},'  = load(filename_profile);']);
%         eval(['profile_r_ori_Front_', Expression_DummyRail{kk},' = profile_r_ori_', Expression_DummyRail{kk},';']);
%         eval(['profile_r_ori_Rear_', Expression_DummyRail{kk},' = profile_r_ori_', Expression_DummyRail{kk},';']);
%         eval(['profile_r_ori_', Expression_DummyRail{kk},'_Radius = Radius_profile_v3(profile_r_ori_', Expression_DummyRail{kk},',1-5e-11,1,1);']);
%         eval(['profile_r_ori_', Expression_DummyRail{kk},'_Radius(:,2) = abs(profile_r_ori_', Expression_DummyRail{kk},'_Radius(:,2));']);
%         eval(['[~,~,profile_r_ori_Front_', Expression_DummyRail{kk},'_d1,~] = Extreme_point(profile_r_ori_Front_', Expression_DummyRail{kk},');']);
%         eval(['[~,~,profile_r_ori_Rear_', Expression_DummyRail{kk},'_d1,~] = Extreme_point(profile_r_ori_Rear_', Expression_DummyRail{kk},');']);
%         
%         eval(['Profile_ori_', Expression_DummyRail{kk},'.',Expression_WS{i11},'_Profile_num = profile_', Expression_DummyRail{kk},'_num;']);
%         eval(['Profile_ori_', Expression_DummyRail{kk},'.',Expression_WS{i11},'_Profile = profile_r_ori_', Expression_DummyRail{kk},';']);
%         eval(['Profile_ori_', Expression_DummyRail{kk},'.',Expression_WS{i11},'_FrontProfile = profile_r_ori_Front_', Expression_DummyRail{kk},';']);
%         eval(['Profile_ori_', Expression_DummyRail{kk},'.',Expression_WS{i11},'_RearProfile  = profile_r_ori_Rear_', Expression_DummyRail{kk},';']);
%         eval(['Profile_ori_', Expression_DummyRail{kk},'.',Expression_WS{i11},'_FrontProfile_d1 = profile_r_ori_Front_', Expression_DummyRail{kk},'_d1;']);
%         eval(['Profile_ori_', Expression_DummyRail{kk},'.',Expression_WS{i11},'_RearProfile_d1  = profile_r_ori_Rear_', Expression_DummyRail{kk},'_d1;']);
%         eval(['Profile_ori_', Expression_DummyRail{kk},'.',Expression_WS{i11},'_Radius = profile_r_ori_', Expression_DummyRail{kk},'_Radius;']);
%     end
% end

%% 右轮下曲基本轨——R2
for i11 = 1:1:4
    profile_R2_num = 1;
    filename_profile = ['Crossing_Section',num2str(profile_R2_num),'.txt'];
    profile_r_ori_R2  = load(filename_profile);
    profile_r_ori_Front_R2 = profile_r_ori_R2;
    profile_r_ori_Rear_R2 = profile_r_ori_R2;
    profile_r_ori_R2_Radius = Radius_profile_v3(profile_r_ori_R2,1-1e-11,1,1);
    profile_r_ori_R2_Radius(:,2) = abs(profile_r_ori_R2_Radius(:,2));
    [~,~,profile_r_ori_Front_R2_d1,~] = Extreme_point(profile_r_ori_Front_R2);
    [~,~,profile_r_ori_Rear_R2_d1,~] = Extreme_point(profile_r_ori_Rear_R2);
    
    eval(['Profile_ori_R2.',Expression_WS{i11},'_Profile_num = profile_R2_num;']);
    eval(['Profile_ori_R2.',Expression_WS{i11},'_Profile = profile_r_ori_R2;']);
    eval(['Profile_ori_R2.',Expression_WS{i11},'_FrontProfile = profile_r_ori_Front_R2;']);
    eval(['Profile_ori_R2.',Expression_WS{i11},'_RearProfile  = profile_r_ori_Rear_R2;']);
    eval(['Profile_ori_R2.',Expression_WS{i11},'_FrontProfile_d1 = profile_r_ori_Front_R2_d1;']);
    eval(['Profile_ori_R2.',Expression_WS{i11},'_RearProfile_d1  = profile_r_ori_Rear_R2_d1;']);
    eval(['Profile_ori_R2.',Expression_WS{i11},'_Radius = profile_r_ori_R2_Radius;']);    
end


%% 不同轮对里程下廓形插值
for i11 = 1:1:4
    Mileage  = j1 - Distance_Vehicle(i11);
    if Mileage <= 15
       %% 转辙器
        %%% 1.1.1 尖轨廓形插值——L2
        Fit_p0 = 1;
        if Mileage < Mileage_sum_Switch(1,1)
            profile_L2_num = 1;
            profile_r_ori_L2  = [];
            profile_r_ori_Front_L2 = profile_r_ori_L2;
            profile_r_ori_Rear_L2  = profile_r_ori_L2;
            profile_r_ori_Front_L2_d1 = [];
            profile_r_ori_Rear_L2_d1 = [];
        elseif Mileage > Mileage_sum_Switch(end,1)
            profile_L2_num = length(Mileage_sum_Switch);
            filename_profile = ['SwitchRail_Section',num2str(profile_L2_num),'.txt'];            
            profile_r_ori_L2_v0  = load(filename_profile);
            if Fit_p0 < 1
                [output,~] = csaps(profile_r_ori_L2_v0(:,1),profile_r_ori_L2_v0(:,2),Fit_p0,profile_r_ori_L2_v0(:,1));
                profile_r_ori_L2 = [profile_r_ori_L2_v0(:,1) output];
            else
                profile_r_ori_L2 = load(filename_profile);
            end
            profile_r_ori_Front_L2 = profile_r_ori_L2;
            profile_r_ori_Rear_L2 = profile_r_ori_L2;
            [~,~,profile_r_ori_Front_L2_d1,~] = Extreme_point(profile_r_ori_Front_L2);
            [~,~,profile_r_ori_Rear_L2_d1,~] = Extreme_point(profile_r_ori_Rear_L2);
        else
            if Mileage < Mileage_sum_Switch(82,1)
%             if Mileage < Mileage_sum_Switch(84,1)
%             if Mileage < 0
                %%% （1）传统插值方法
                profile_L2_num = min(find(Mileage-Mileage_sum_Switch(:,1)<=0));
                filename_Profile_1 = ['SwitchRail_Section',num2str(profile_L2_num-1),'.txt'];
                filename_Profile_2 = ['SwitchRail_Section',num2str(profile_L2_num),'.txt'];
                profile_r_ori_Front_L2 = load(filename_Profile_1);
                profile_r_ori_Rear_L2 = load(filename_Profile_2);
                
                profile_r_ori_L2_v0 = [];
                % 分段方法：根据极值点分段
                [~,~,profile_r_ori_Front_L2_d1,~] = Extreme_point(profile_r_ori_Front_L2);
                [~,~,profile_r_ori_Rear_L2_d1,~] = Extreme_point(profile_r_ori_Rear_L2);
                if length(profile_r_ori_Front_L2_d1(:,1)) > 1 || length(profile_r_ori_Rear_L2_d1(:,1)) >1
                    limit_x = 2e-3;
                    limit_y = 1e-3;
                    %%% Profile_1
                    bools_11a = [1; abs(profile_r_ori_Front_L2_d1(2:end,3)-profile_r_ori_Front_L2_d1(1:end-1,3))>limit_y] + ...
                        [abs(profile_r_ori_Front_L2_d1(1:end-1,3)-profile_r_ori_Front_L2_d1(2:end,3))>limit_y; 1];
                    bools_11b = find(bools_11a);
                    bools_11c = [];
                    for k = 2:1:length(bools_11b)
                        bools_11c = [bools_11c; (bools_11b(k-1,1)+1:1:bools_11b(k,1)-1)'];
                    end
                    profile_r_ori_Front_L2_d1(bools_11c,:) = [];
                    %%% Profile_2
                    bools_21a = [1; abs(profile_r_ori_Rear_L2_d1(2:end,3)-profile_r_ori_Rear_L2_d1(1:end-1,3))>limit_y] + ...
                        [abs(profile_r_ori_Rear_L2_d1(1:end-1,3)-profile_r_ori_Rear_L2_d1(2:end,3))>limit_y; 1];
                    bools_21b = find(bools_21a);
                    bools_21c = [];
                    for k = 2:1:length(bools_21b)
                        bools_21c = [bools_21c; (bools_21b(k-1,1)+1:1:bools_21b(k,1)-1)'];
                    end
                    profile_r_ori_Rear_L2_d1(bools_21c,:) = [];
                end
                
                % 选择尖轨部分插值方式
                if Mileage < 8.5
                    Type_interp_switch = 'linear';
                else
                    Type_interp_switch = 'spline';
                end
                % 分段线性插值
                for k = 1:1:length(profile_r_ori_Front_L2_d1(:,1))+1
                    if k == 1
                        xx_start_1 = 1;
                        xx_start_2 = 1;
                        xx_end_1 = profile_r_ori_Front_L2_d1(k,1);
                        xx_end_2 = profile_r_ori_Rear_L2_d1(k,1);
                    elseif k == length(profile_r_ori_Front_L2_d1(:,1))+1
                        xx_start_1 = profile_r_ori_Front_L2_d1(end,1)+1;
                        xx_start_2 = profile_r_ori_Rear_L2_d1(end,1)+1;
                        xx_end_1 = length(profile_r_ori_Front_L2);
                        xx_end_2 = length(profile_r_ori_Rear_L2);
                    else
                        xx_start_1 = profile_r_ori_Front_L2_d1(k-1,1)+1;
                        xx_start_2 = profile_r_ori_Rear_L2_d1(k-1,1)+1;
                        xx_end_1 = profile_r_ori_Front_L2_d1(k,1);
                        xx_end_2 = profile_r_ori_Rear_L2_d1(k,1);
                    end
                    seg_y = min(profile_r_ori_Front_L2_d1(1,2),profile_r_ori_Rear_L2_d1(1,2))-2e-3;
                    ratio =  (Mileage-Mileage_sum_Switch(profile_L2_num-1,1))/(Mileage_sum_Switch(profile_L2_num,1)-Mileage_sum_Switch(profile_L2_num-1,1));
                    if k==1
                        num_interp_L2 = 300;
                        temp_min = max(profile_r_ori_Front_L2(xx_start_1,1),profile_r_ori_Rear_L2(xx_start_2,1));
                        xx = linspace(temp_min,seg_y,num_interp_L2)';
                        yy_1 = interp1(profile_r_ori_Front_L2(xx_start_1:xx_end_1,1),profile_r_ori_Front_L2(xx_start_1:xx_end_1,2),xx,Type_interp_switch);
                        yy_2 = interp1(profile_r_ori_Rear_L2(xx_start_2:xx_end_2,1),profile_r_ori_Rear_L2(xx_start_2:xx_end_2,2),xx,Type_interp_switch);
                        profile_r_ori_L2_v0 = [profile_r_ori_L2_v0; xx yy_1+(yy_2-yy_1).*ratio];
                        
                        num_interp_L2 = 100;
                        xx_1 = linspace(seg_y,profile_r_ori_Front_L2(xx_end_1,1),num_interp_L2)';
                        yy_1 = interp1(profile_r_ori_Front_L2(xx_start_1:xx_end_1,1),profile_r_ori_Front_L2(xx_start_1:xx_end_1,2),xx_1,Type_interp_switch);
                        xx_2 = linspace(seg_y,profile_r_ori_Rear_L2(xx_end_2,1),num_interp_L2)';
                        yy_2 = interp1(profile_r_ori_Rear_L2(xx_start_2:xx_end_2,1),profile_r_ori_Rear_L2(xx_start_2:xx_end_2,2),xx_2,Type_interp_switch);
                        profile_r_ori_L2_v0 = [profile_r_ori_L2_v0; xx_1+(xx_2-xx_1).*ratio  yy_1+(yy_2-yy_1).*ratio];
                    else
                        num_interp_L2 = 150;
                        xx_1 = linspace(profile_r_ori_Front_L2(xx_start_1,1),profile_r_ori_Front_L2(xx_end_1,1),num_interp_L2)';
                        yy_1 = interp1(profile_r_ori_Front_L2(xx_start_1:xx_end_1,1),profile_r_ori_Front_L2(xx_start_1:xx_end_1,2),xx_1,Type_interp_switch);
                        xx_2 = linspace(profile_r_ori_Rear_L2(xx_start_2,1),profile_r_ori_Rear_L2(xx_end_2,1),num_interp_L2)';
                        yy_2 = interp1(profile_r_ori_Rear_L2(xx_start_2:xx_end_2,1),profile_r_ori_Rear_L2(xx_start_2:xx_end_2,2),xx_2,Type_interp_switch);
                        ratio =  (Mileage-Mileage_sum_Switch(profile_L2_num-1,1))/(Mileage_sum_Switch(profile_L2_num,1)-Mileage_sum_Switch(profile_L2_num-1,1));
                        profile_r_ori_L2_v0 = [profile_r_ori_L2_v0; xx_1+(xx_2-xx_1).*ratio  yy_1+(yy_2-yy_1).*ratio];
                    end
                end
                profile_r_ori_L2_v0 = sortrows(profile_r_ori_L2_v0,1);
                profile_r_ori_L2_v0(diff(profile_r_ori_L2_v0(:,1))==0,:) = [];
                % 拟合廓形-Fit
                if Fit_p0 < 1
                    [output,~] = csaps(profile_r_ori_L2_v0(:,1),profile_r_ori_L2_v0(:,2),Fit_p0,profile_r_ori_L2_v0(:,1));
                    profile_r_ori_L2 = [profile_r_ori_L2_v0(:,1) output];
                else
                    profile_r_ori_L2 = profile_r_ori_L2_v0;
                end
            else
                %%% （2）Bezier 曲线插值
                p = find(Mileage>=MileageInterp_Switch_div(:,1) & Mileage<MileageInterp_Switch_div(:,2));
                profile_L2_num = min(find(Mileage-Mileage_sum_Switch(:,1)<=0));
                filename_profile_1 = ['SwitchRail_Section',num2str(profile_L2_num-1),'.txt'];
                filename_profile_2 = ['SwitchRail_Section',num2str(profile_L2_num),'.txt'];
                profile_r_ori_Front_L2 = load(filename_profile_1);
                profile_r_ori_Rear_L2  = load(filename_profile_2);
                profile_r_ori_L2 = [];
                if ~isempty(p)
                    t = (Mileage-MileageInterp_Switch_div(p,1))/(MileageInterp_Switch_div(p,2)-MileageInterp_Switch_div(p,1));
                    for j = 1:1:num_interp_Bezier
                        output = Bezier_Curve(Profile_Switch_Bezier.xx(j,sum(MileageInterp_Switch_div(1:p-1,3))+1:sum(MileageInterp_Switch_div(1:p,3))), ...
                            Profile_Switch_Bezier.yy(j,sum(MileageInterp_Switch_div(1:p-1,3))+1:sum(MileageInterp_Switch_div(1:p,3))), t);
                        profile_r_ori_L2 = [profile_r_ori_L2; output];
                    end
                else
                    xx_min = max(min(profile_r_ori_Front_L2(:,1)),min(profile_r_ori_Rear_L2(:,1)));
                    xx_max = min(max(profile_r_ori_Front_L2(:,1)),max(profile_r_ori_Rear_L2(:,1)));
                    xx  = linspace(xx_min,xx_max,num_interp_Bezier)';
                    yy_1 = interp1(profile_r_ori_Front_L2(:,1),profile_r_ori_Front_L2(:,2),xx,'linear');
                    yy_2 = interp1(profile_r_ori_Rear_L2(:,1),profile_r_ori_Rear_L2(:,2),xx,'linear');
                    ratio = (Mileage-Mileage_sum_Switch(profile_L2_num-1,1))...
                        /(Mileage_sum_Switch(profile_L2_num,1)-Mileage_sum_Switch(profile_L2_num-1,1));
                    profile_r_ori_L2 = [xx  yy_1+(yy_2-yy_1).*ratio];
                end
                
                [~,~,profile_r_ori_Front_L2_d1,~] = Extreme_point(profile_r_ori_Front_L2);
                [~,~,profile_r_ori_Rear_L2_d1,~] = Extreme_point(profile_r_ori_Rear_L2);
            end
        end
       
       %%% 1.1.2 尖轨廓形曲率半径计算       
       if ~isempty(profile_r_ori_L2)
           if Mileage < 8.5
               profile_r_ori_L2_Radius = Radius_profile_v3(profile_r_ori_L2,1-1e-12,1,1);
           else
               profile_r_ori_L2_Radius = Radius_profile_v3(profile_r_ori_L2,1-1e-12,1,1);
%                profile_r_ori_L2_Radius = Radius_profile_v3(profile_r_ori_L2,1-5e-11,1,1);
           end
           profile_r_ori_L2_Radius(:,2) = abs(profile_r_ori_L2_Radius(:,2));
       else
           profile_r_ori_L2_Radius = [];
       end
              
       %%% 1.2.1 直基本轨廓形插值——L1
       if Mileage < Mileage_sum_Stock(1,1)
           profile_L1_num = 1;
           filename_profile = ['MainStockRail_Section',num2str(profile_L1_num),'.txt'];
           profile_r_ori_L1  = load(filename_profile);
           profile_r_ori_Front_L1 = profile_r_ori_L1;
           profile_r_ori_Rear_L1  = profile_r_ori_L1;
           [~,~,profile_r_ori_Front_L1_d1,~] = Extreme_point(profile_r_ori_Front_L1);
           [~,~,profile_r_ori_Rear_L1_d1,~] = Extreme_point(profile_r_ori_Rear_L1);
       elseif Mileage > Mileage_sum_Stock(end,1)
           profile_L1_num = NaN;
           profile_r_ori_L1  = [];
           profile_r_ori_Front_L1 = [];
           profile_r_ori_Rear_L1 = [];
           profile_r_ori_Front_L1_d1 = [];
           profile_r_ori_Rear_L1_d1 = [];
       else
           profile_L1_num = min(find(Mileage-Mileage_sum_Stock(:,1)<=0));
           filename_Profile_1 = ['MainStockRail_Section',num2str(profile_L1_num-1),'.txt'];
           filename_Profile_2 = ['MainStockRail_Section',num2str(profile_L1_num),'.txt'];
           profile_r_ori_Front_L1 = load(filename_Profile_1);
           profile_r_ori_Rear_L1 = load(filename_Profile_2);
           num_interp_L1 = 150;
           xx_1 = linspace(profile_r_ori_Front_L1(1,1),profile_r_ori_Front_L1(end,1),num_interp_L1)';
           yy_1 = interp1(profile_r_ori_Front_L1(:,1),profile_r_ori_Front_L1(:,2),xx_1,'spline');
           xx_2 = linspace(profile_r_ori_Rear_L1(1,1),profile_r_ori_Rear_L1(end,1),num_interp_L1)';
           yy_2 = interp1(profile_r_ori_Rear_L1(:,1),profile_r_ori_Rear_L1(:,2),xx_2,'spline');
           ratio =  (Mileage-Mileage_sum_Stock(profile_L1_num-1,1))/(Mileage_sum_Stock(profile_L1_num,1)-Mileage_sum_Stock(profile_L1_num-1,1));
           profile_r_ori_L1 = [xx_1+(xx_2-xx_1).*ratio  yy_1+(yy_2-yy_1).*ratio];
           [~,~,profile_r_ori_Front_L1_d1,~] = Extreme_point(profile_r_ori_Front_L1);
           [~,~,profile_r_ori_Rear_L1_d1,~] = Extreme_point(profile_r_ori_Rear_L1);
       end
       
        % 1.2.2 直基本轨廓形曲率半径计算
        if ~isempty(profile_r_ori_L1)
            profile_r_ori_L1 = sortrows(profile_r_ori_L1,1);
            profile_r_ori_L1_Radius = Radius_profile_v3(profile_r_ori_L1,1-1e-10,1,1);
            profile_r_ori_L1_Radius(:,2) = abs(profile_r_ori_L1_Radius(:,2));
        else
            profile_r_ori_L1_Radius = [];
        end
        
        profile_R1_num = NaN;
        profile_r_ori_R1 = [];       
        profile_r_ori_Front_R1 = [];
        profile_r_ori_Rear_R1 = [];
        profile_r_ori_Front_R1_d1 = [];
        profile_r_ori_Rear_R1_d1 = [];
        profile_r_ori_R1_Radius = [];
       
    else
       %% 辙叉
        % 2.1.1 固定辙叉廓形插值——L2
        if Mileage < Mileage_sum_Crossing(1,1)
            profile_L2_num = 1;
            filename_profile = ['Crossing_Section',num2str(profile_L2_num),'.txt'];
            profile_r_ori_L2  = load(filename_profile);
            profile_r_ori_Front_L2 = profile_r_ori_L2;
            profile_r_ori_Rear_L2 = profile_r_ori_L2;
        elseif Mileage > Mileage_sum_Crossing(end,1)
            profile_L2_num = length(Mileage_sum_Crossing);
            filename_profile = ['Crossing_Section',num2str(profile_L2_num),'.txt'];
            profile_r_ori_L2  = load(filename_profile);
            profile_r_ori_Front_L2 = profile_r_ori_L2;
            profile_r_ori_Rear_L2 = profile_r_ori_L2;
        else
            p = find(Mileage>=MileageInterp_Crossing_div(:,1) & Mileage<MileageInterp_Crossing_div(:,2));
            profile_L2_num = min(find(Mileage-Mileage_sum_Crossing(:,1)<=0));
            filename_profile_1 = ['Crossing_Section',num2str(profile_L2_num-1),'.txt'];
            filename_profile_2 = ['Crossing_Section',num2str(profile_L2_num),'.txt'];
            profile_r_ori_Front_L2 = load(filename_profile_1);
            profile_r_ori_Rear_L2 = load(filename_profile_2);
            profile_r_ori_L2 = [];
            if ~isempty(p)
                t = (Mileage-MileageInterp_Crossing_div(p,1))/(MileageInterp_Crossing_div(p,2)-MileageInterp_Crossing_div(p,1));
                for j = 1:1:num_interp_Bezier
                    output = Bezier_Curve(Profile_Crossing_Bezier.xx(j,sum(MileageInterp_Crossing_div(1:p-1,3))+1:sum(MileageInterp_Crossing_div(1:p,3))), ...
                                          Profile_Crossing_Bezier.yy(j,sum(MileageInterp_Crossing_div(1:p-1,3))+1:sum(MileageInterp_Crossing_div(1:p,3))), t);
                    profile_r_ori_L2 = [profile_r_ori_L2; output];
                end
            else
                xx_min = max(min(profile_r_ori_Front_L2(:,1)),min(profile_r_ori_Rear_L2(:,1)));
                xx_max = min(max(profile_r_ori_Front_L2(:,1)),max(profile_r_ori_Rear_L2(:,1)));
                xx  = linspace(xx_min,xx_max,num_interp_Bezier)';
                yy_1 = interp1(profile_r_ori_Front_L2(:,1),profile_r_ori_Front_L2(:,2),xx,'spline');
                yy_2 = interp1(profile_r_ori_Rear_L2(:,1),profile_r_ori_Rear_L2(:,2),xx,'spline');
                ratio = (Mileage-Mileage_sum_Crossing(profile_L2_num-1,1))...
                    /(Mileage_sum_Crossing(profile_L2_num,1)-Mileage_sum_Crossing(profile_L2_num-1,1));
                profile_r_ori_L2 = [xx  yy_1+(yy_2-yy_1).*ratio];
            end
            [~,~,profile_r_ori_Front_L2_d1,~] = Extreme_point(profile_r_ori_Front_L2);
            [~,~,profile_r_ori_Rear_L2_d1,~] = Extreme_point(profile_r_ori_Rear_L2);
        end
        % 2.1.2 固定辙叉廓形曲率半径计算
        profile_r_ori_L2 = sortrows(profile_r_ori_L2,1);
%         profile_r_ori_L2_Radius = Radius_profile_v3(profile_r_ori_L2,1-5e-11,1,1);
        profile_r_ori_L2_Radius = Radius_profile_v3(profile_r_ori_L2,1-1e-11,1,1);
%         profile_r_ori_L2_Radius = Radius_profile_v3(profile_r_ori_L2,1-1e-9,1,1);
        profile_r_ori_L2_Radius(:,2) = abs(profile_r_ori_L2_Radius(:,2));
        
        % 2.2.1 护轨截面插值——R1
        if Mileage<Mileage_sum_Check(1,1) || Mileage>Mileage_sum_Check(end,1)
            profile_R1_num = 1;
            profile_r_ori_R1 = [];
            profile_r_ori_Front_R1 = [];
            profile_r_ori_Rear_R1 = [];
            profile_r_ori_Front_R1_d1 = [];
            profile_r_ori_Rear_R1_d1 = [];
        else
            num_interp_R1 = 200;
            profile_R1_num_pos = min(find(Mileage-Mileage_sum_Check(:,1)<=0));
            profile_R1_num = Mileage_sum_Check(profile_R1_num_pos,2);
            filename_R1_Profile_1 = ['CheckRail_Section',num2str(profile_R1_num-1),'.txt'];
            filename_R1_Profile_2 = ['CheckRail_Section',num2str(profile_R1_num),'.txt'];
            profile_r_ori_Front_R1 = load(filename_R1_Profile_1);
            profile_r_ori_Rear_R1 = load(filename_R1_Profile_2);
            xx_1 = linspace(profile_r_ori_Front_R1(1,1),profile_r_ori_Front_R1(end,1),num_interp_R1)';
            yy_1 = interp1(profile_r_ori_Front_R1(:,1),profile_r_ori_Front_R1(:,2),xx_1,'spline');
            xx_2 = linspace(profile_r_ori_Rear_R1(1,1),profile_r_ori_Rear_R1(end,1),num_interp_R1)';
            yy_2 = interp1(profile_r_ori_Rear_R1(:,1),profile_r_ori_Rear_R1(:,2),xx_2,'spline');
            ratio = (Mileage-Mileage_sum_Check(profile_R1_num_pos-1,1))/(Mileage_sum_Check(profile_R1_num_pos,1)-Mileage_sum_Check(profile_R1_num_pos-1,1));
            profile_r_ori_R1 = [xx_1+(xx_2-xx_1).*ratio  yy_1+(yy_2-yy_1).*ratio];            
            [~,~,profile_r_ori_Front_R1_d1,~] = Extreme_point(profile_r_ori_Front_R1);
            [~,~,profile_r_ori_Rear_R1_d1,~] = Extreme_point(profile_r_ori_Rear_R1);
        end
        
        % 2.2.2 护轨廓形曲率半径计算
        if ~isempty(profile_r_ori_R1)
            profile_r_ori_R1 = sortrows(profile_r_ori_R1,1);
            profile_r_ori_R1_Radius = Radius_profile_v3(profile_r_ori_R1,1-1e-11,1,1);
            profile_r_ori_R1_Radius(:,2) = abs(profile_r_ori_R1_Radius(:,2));
        else
            profile_r_ori_R1_Radius = [];
        end
        
        profile_L1_num = NaN;
        profile_r_ori_L1 = [];       
        profile_r_ori_Front_L1 = [];
        profile_r_ori_Rear_L1 = [];
        profile_r_ori_L1_Radius = [];
        profile_r_ori_Front_L1_d1 = [];
        profile_r_ori_Rear_L1_d1 = [];
        
    end
    
    % 3. 保存截面至 struct
    eval(['Profile_ori_L1.',Expression_WS{i11},'_Profile_num = profile_L1_num;']);
    eval(['Profile_ori_L1.',Expression_WS{i11},'_Profile = profile_r_ori_L1;']);
    eval(['Profile_ori_L1.',Expression_WS{i11},'_FrontProfile = profile_r_ori_Front_L1;']);
    eval(['Profile_ori_L1.',Expression_WS{i11},'_RearProfile  = profile_r_ori_Rear_L1;']);
    eval(['Profile_ori_L1.',Expression_WS{i11},'_FrontProfile_d1 = profile_r_ori_Front_L1_d1;']);
    eval(['Profile_ori_L1.',Expression_WS{i11},'_RearProfile_d1  = profile_r_ori_Rear_L1_d1;']);
    eval(['Profile_ori_L1.',Expression_WS{i11},'_Radius = profile_r_ori_L1_Radius;']);
    
    eval(['Profile_ori_L2.',Expression_WS{i11},'_Profile_num = profile_L2_num;']);
    eval(['Profile_ori_L2.',Expression_WS{i11},'_Profile = profile_r_ori_L2;']);
    eval(['Profile_ori_L2.',Expression_WS{i11},'_FrontProfile = profile_r_ori_Front_L2;']);
    eval(['Profile_ori_L2.',Expression_WS{i11},'_RearProfile  = profile_r_ori_Rear_L2;']);
    eval(['Profile_ori_L2.',Expression_WS{i11},'_FrontProfile_d1 = profile_r_ori_Front_L2_d1;']);
    eval(['Profile_ori_L2.',Expression_WS{i11},'_RearProfile_d1 = profile_r_ori_Rear_L2_d1;']);
    eval(['Profile_ori_L2.',Expression_WS{i11},'_Radius = profile_r_ori_L2_Radius;']);
        
    eval(['Profile_ori_R1.',Expression_WS{i11},'_Profile_num = profile_R1_num;']);
    eval(['Profile_ori_R1.',Expression_WS{i11},'_Profile = profile_r_ori_R1;']);
    eval(['Profile_ori_R1.',Expression_WS{i11},'_FrontProfile = profile_r_ori_Front_R1;']);
    eval(['Profile_ori_R1.',Expression_WS{i11},'_RearProfile  = profile_r_ori_Rear_R1;']);
    eval(['Profile_ori_R1.',Expression_WS{i11},'_FrontProfile_d1 = profile_r_ori_Front_R1_d1;']);
    eval(['Profile_ori_R1.',Expression_WS{i11},'_RearProfile_d1  = profile_r_ori_Rear_R1_d1;']);
    eval(['Profile_ori_R1.',Expression_WS{i11},'_Radius = profile_r_ori_R1_Radius;']);
       
end

%% 绘图对比
if Choose_Plot == 1
    if ~isempty(Profile_ori_R1.FF_Profile)
        figure(32); clf
        plot(Profile_ori_R1.FF_FrontProfile(:,1), Profile_ori_R1.FF_FrontProfile(:,2), '--',...
            Profile_ori_R1.FF_RearProfile(:,1),  Profile_ori_R1.FF_RearProfile(:,2), '--',...
            Profile_ori_R1.FF_Profile(:,1), Profile_ori_R1.FF_Profile(:,2))
        set(gca,'ydir','reverse'); grid on
    end
    
    if ~isempty(Profile_ori_L2.FF_Profile)
        figure(33); clf
        plot(Profile_ori_L2.FF_FrontProfile(:,1), Profile_ori_L2.FF_FrontProfile(:,2), '--',...
            Profile_ori_L2.FF_RearProfile(:,1),  Profile_ori_L2.FF_RearProfile(:,2), '--',...
            Profile_ori_L2.FF_Profile(:,1), Profile_ori_L2.FF_Profile(:,2))
        set(gca,'xdir','reverse'); set(gca,'ydir','reverse'); grid on
    end
end

