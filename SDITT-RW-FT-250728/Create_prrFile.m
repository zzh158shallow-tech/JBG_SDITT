%% 通过贝塞尔曲线拟合或线性插值，获取某类接触对下，四个车轮下的钢轨廓形
function Profile_ori = Create_prrFile(j1, InpPar, Par_Vehicle, IF_SameFrontProf, IF_SameRearProf, Mileage_sum_Target, MileageInterp_Div_Target, Profile_Bezier_Target, Par_RailPro_Fit)

% IF_SameFrontProf = 0;
% IF_SameRearProf = 0;
% Mileage_sum_Target = InpPar.Mileage_sum.R2;
% MileageInterp_Div_Target = InpPar.MileageInterp_Div.R2;
% Profile_Bezier_Target = InpPar.Profile_Bezier .R2;
% Par_RailPro_Fit = [1-1e-12,1,1];

% global j1 InpPar.Distance_Vehicle InpPar.Exp_WS InpPar.num_interp_Bezier

for i11 = 1:1:InpPar.Nw
    
    Mileage  = j1 - Par_Vehicle.Distance_Vehicle(i11);

    if Mileage < Mileage_sum_Target{1,1}
        if IF_SameFrontProf == 1
            profile_num = 1;
            filename_profile = Mileage_sum_Target{1,2};
            profile_r_ori  = load(filename_profile);
            profile_r_ori_Front = profile_r_ori;
            profile_r_ori_Rear = profile_r_ori;
            [~,~,profile_r_ori_Front_d1,~] = Extreme_point(profile_r_ori_Front);
            profile_r_ori_Rear_d1 = profile_r_ori_Front_d1;
        elseif IF_SameFrontProf == 0
            profile_num = [];
            profile_r_ori  = [];
            profile_r_ori_Front = [];
            profile_r_ori_Rear = [];
            profile_r_ori_Front_d1 = [];
            profile_r_ori_Rear_d1 = [];
            profile_r_ori_Radius = [];
        end

    elseif Mileage > Mileage_sum_Target{end,1}
        if IF_SameRearProf == 1
            profile_num = size(Mileage_sum_Target,1);
            filename_profile = Mileage_sum_Target{end,2};
            profile_r_ori  = load(filename_profile);
            profile_r_ori_Front = profile_r_ori;
            profile_r_ori_Rear = profile_r_ori;
            [~,~,profile_r_ori_Front_d1,~] = Extreme_point(profile_r_ori_Front);
            profile_r_ori_Rear_d1 = profile_r_ori_Front_d1;
        elseif IF_SameRearProf == 0
            profile_num = [];
            profile_r_ori  = [];
            profile_r_ori_Front = [];
            profile_r_ori_Rear = [];
            profile_r_ori_Front_d1 = [];
            profile_r_ori_Rear_d1 = [];
            profile_r_ori_Radius = [];
        end

    elseif (Mileage >= Mileage_sum_Target{1,1}) && (Mileage <= Mileage_sum_Target{end,1})
%         profile_r_ori = [];
        p = min(find(Mileage>=MileageInterp_Div_Target(:,1) & Mileage<=MileageInterp_Div_Target(:,2)));
        % Replace code such as min(find(A)) with find(A,1). Replace max(find(A)) with find(A, 1, 'last').
        profile_num = find(Mileage-cell2mat(Mileage_sum_Target(:,1))<=0,1);
        filename_profile_1 = Mileage_sum_Target{profile_num-1,2};
        filename_profile_2 = Mileage_sum_Target{profile_num,2};
        profile_r_ori_Front = load(filename_profile_1);
        profile_r_ori_Rear = load(filename_profile_2);
        if ~isempty(p)
%             tic
            source = Profile_Bezier_Target.(['x2t_Div',num2str(p)]);
            t = interp1(source(:,1),source(:,2),Mileage,'linear');
            profile_r_ori = zeros(InpPar.num_interp_Bezier,2);
            for j = 1:1:InpPar.num_interp_Bezier
                p1 = sum(MileageInterp_Div_Target(1:p-1,3))+1;
                p2 = sum(MileageInterp_Div_Target(1:p,3));
                y = Profile_Bezier_Target.yy(j, p1:p2);
                z = Profile_Bezier_Target.zz(j, p1:p2);
                % Bezier_Curve
                NumPoint = length(y)-1;
                kk = (0:1:NumPoint)';
                w = factorial(NumPoint)./(factorial(kk).*factorial(NumPoint-kk)) .* (1-t).^(NumPoint-kk) .* t.^(kk);
                yy = w.*y';
                zz = w.*z';
                output = [sum(yy), sum(zz)];
%                 output = Bezier_Curve([], y, z, t, '2D');
                profile_r_ori(j,:) = output;
            end
%             toc
        else
            xx_min = max(min(profile_r_ori_Front(:,1)),min(profile_r_ori_Rear(:,1)));
            xx_max = min(max(profile_r_ori_Front(:,1)),max(profile_r_ori_Rear(:,1)));
            xx  = linspace(xx_min,xx_max,InpPar.num_interp_Bezier)';
            yy_1 = interp1(profile_r_ori_Front(:,1),profile_r_ori_Front(:,2),xx,'spline');
            yy_2 = interp1(profile_r_ori_Rear(:,1),profile_r_ori_Rear(:,2),xx,'spline');
            ratio = (Mileage-Mileage_sum_Target{profile_num-1,1})/(Mileage_sum_Target{profile_num,1}-Mileage_sum_Target{profile_num-1,1});
            profile_r_ori = [xx  yy_1+(yy_2-yy_1).*ratio];
        end
        [~,~,profile_r_ori_Front_d1,~] = Extreme_point(profile_r_ori_Front);
        [~,~,profile_r_ori_Rear_d1,~] = Extreme_point(profile_r_ori_Rear);
        
    end
    
    if ~isempty(profile_r_ori)
        % 不同里程对应不同的廓形拟合系数
        profile_r_ori = sortrows(profile_r_ori,1);
        if size(Par_RailPro_Fit,1)==1
            profile_r_ori_Radius = Radius_profile_v3(profile_r_ori, Par_RailPro_Fit(1), Par_RailPro_Fit(2), Par_RailPro_Fit(3));
        else
            bools = (Mileage>=Par_RailPro_Fit(:,4) & Mileage<=Par_RailPro_Fit(:,5));
            profile_r_ori_Radius = Radius_profile_v3(profile_r_ori, Par_RailPro_Fit(bools,1), Par_RailPro_Fit(bools,2), Par_RailPro_Fit(bools,3));
        end
%         profile_r_ori_Radius = Radius_profile_v3(profile_r_ori,Par_RailPro_Fit(1),Par_RailPro_Fit(2),Par_RailPro_Fit(3));
        profile_r_ori_Radius(:,2) = abs(profile_r_ori_Radius(:,2));
    end
    
    Profile_ori.([InpPar.Exp_WS{i11},'_Profile_num']) = profile_num;
    Profile_ori.([InpPar.Exp_WS{i11},'_Profile']) = profile_r_ori;
    Profile_ori.([InpPar.Exp_WS{i11},'_FrontProfile']) = profile_r_ori_Front;
    Profile_ori.([InpPar.Exp_WS{i11},'_RearProfile'])  = profile_r_ori_Rear;
    Profile_ori.([InpPar.Exp_WS{i11},'_FrontProfile_d1']) = profile_r_ori_Front_d1;
    Profile_ori.([InpPar.Exp_WS{i11},'_RearProfile_d1'])  = profile_r_ori_Rear_d1;
    Profile_ori.([InpPar.Exp_WS{i11},'_Radius']) = profile_r_ori_Radius;
    
end
