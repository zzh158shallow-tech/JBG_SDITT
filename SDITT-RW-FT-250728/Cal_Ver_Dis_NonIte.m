%% 计算接触点位置和轮轨间隙
% 无需通过迭代计算侧滚角

function [Elastic_pen_L, Elastic_pen_R, Ver_Dis_L, Ver_Dis_R, rail_interp_L, rail_interp_R, wheel_interp_L, wheel_interp_R, Yw_DWL, Yw_DWR, A_DWL, A_DWR, Zw_DWL, Zw_DWR] = ...
                Cal_Ver_Dis_NonIte(InpPar, WheelPro_ProCS, Zw, Yw, Yaw, Roll, Profile_TrackCS, profile_w_L, profile_w_R, Dlb, d0)

% global N_ConPatch_L N_ConPatch_R Expression_DummyRail_L Expression_DummyRail_R
N_ConPatch_L = InpPar.N_ConPatch_L;
N_ConPatch_R = InpPar.N_ConPatch_R;
Expression_DummyRail_L = InpPar.Exp_DummyRail_L;
Expression_DummyRail_R = InpPar.Exp_DummyRail_R;

%% A. Profile
% j1 = SIP(2)+Range_X(i1);
% RailPro_ProCS = struct;
% RailPro_ProCS = Get_Profile_P1_Through_210624(RailPro_ProCS, ['L1', 'R1', 'R2', 'R3']);
% Choose_Plot = 0;
% if Choose_Plot == 1
%     Plot_RailPro_ProCS(RailPro_ProCS)
% end

% Zw = 0;         % 轮对垂向位移
% Yw = 0;         % 轮对横向位移
% Yaw = 0;        % 轮对摇头角
% Roll = 0;        % 轮对侧滚角
% Mileage  = j1;
Choose_Plot = 0;

% for i2 = 1:1:N_ConPatch
%     % TIrr: Mileage, Z-L1, Y-L1, Z-R1, Y-R1, Z-R2, Y-R2, Z-R3, Y-R3
%     %         Dis_TIrr_temp{1,i2} = [0, interp1(TIrr(:,1),TIrr(:,2*i2+1), Mileage,'spline'), interp1(TIrr(:,1),TIrr(:,2*i2), Mileage,'spline')];
%     % NonTIrr
%     Dis_TIrr_temp{1,i2} = zeros(1,3);
% end

% 导入钢轨变截面廓形
% Dis_Rail = zeros(Nw*N_ConPatch,3);
% Profile_TrackCS = Get_Profile_P2(i11, Dis_Rail, Mileage, RailPro_ProCS, Dis_TIrr_temp);

%% B. 基于迹线法计算侧滚角
% i2 = 0;
% diff_VerDis_Z = 1;
% clear Oup_Int
% while abs(diff_VerDis_Z)>=1e-8
%     i2 = i2+1;

%     A_WS = double(subs(T_WS_Track, [Inp_Yaw, Inp_Roll], [Yaw, Roll]));
    A_WS = [ cos(Yaw)                 sin(Yaw)                   0;
                  -cos(Roll)*sin(Yaw)  cos(Roll)*cos(Yaw)   sin(Roll);
                    sin(Roll)*sin(Yaw)  -sin(Roll)*cos(Yaw)   cos(Roll)];
    discrete_len_tread = 2.5e-5;
    discrete_len_flange = 0.5e-5;
    Roll_DWR = Roll;	Roll_DWL = Roll;
    Yaw_DWR = Yaw;      Yaw_DWL = Yaw;
    Yw_DWR = Yw;        Yw_DWL = Yw;
    Zw_DWR = Zw;        Zw_DWL = Zw;
    A_DWR = A_WS;       A_DWL = A_WS;
    
    [Con_ang_R_temp, traceline_w_R, Con_ang_L_temp, traceline_w_L] = TracePrinciple(WheelPro_ProCS, Dlb, discrete_len_flange, discrete_len_tread,...
     profile_w_R, Roll_DWR, Yaw_DWR, Yw_DWR, Zw_DWR, profile_w_L, Roll_DWL, Yaw_DWL, Yw_DWL, Zw_DWL);
  
    %% B1. 计算轮轨垂向间隙
    temp = 0.1e-3;
    % 左侧
    bools_L = traceline_w_L(:,2) > max( min(traceline_w_L(:,2)), min(Profile_TrackCS.profile_r.L(:,1)) ) +temp & ...
                     traceline_w_L(:,2) < min( max(traceline_w_L(:,2)), max(Profile_TrackCS.profile_r.L(:,1)) ) -temp;

    %%% 避免因基本轨与护轨廓形之间的空档，导致插值不良
    if N_ConPatch_L > 1
        for kk = 1:1:N_ConPatch_L-1
            Profile_outside = Profile_TrackCS.profile_r.(InpPar.Exp_DummyRail_L{kk});
            Profile_inside = Profile_TrackCS.profile_r.(InpPar.Exp_DummyRail_L{kk+1});
            if ~isempty(Profile_outside) && ~isempty(Profile_inside)
                bools_L = bools_L & ( traceline_w_L(:,2) >= min(Profile_inside(:,1)) | traceline_w_L(:,2) <= max(Profile_outside(:,1)) );
            end
        end
    end
    
    range_y_L = traceline_w_L(bools_L,2)';
    wheel_interp_L = traceline_w_L(bools_L,:);
    rail_interp_L = [range_y_L' interp1(Profile_TrackCS.profile_r.L(:,1),Profile_TrackCS.profile_r.L(:,2), range_y_L,'spline')'];
    bools_linear = (rail_interp_L(:,2)>0.6+8e-3);
    range_y_L_lienar = (range_y_L(bools_linear))';
    rail_interp_L(bools_linear,:) = [range_y_L_lienar interp1(Profile_TrackCS.profile_r.L(:,1),Profile_TrackCS.profile_r.L(:,2),range_y_L_lienar,'linear')];
    rail_interp_L = sortrows(rail_interp_L,1);
    Ver_Dis_L = [range_y_L' rail_interp_L(:,2)-wheel_interp_L(:,3)];
    
    % 右侧
    bools_R = traceline_w_R(:,2) > max( min(traceline_w_R(:,2)), min(Profile_TrackCS.profile_r.R(:,1)) ) +temp &...
                     traceline_w_R(:,2) < min( max(traceline_w_R(:,2)), max(Profile_TrackCS.profile_r.R(:,1)) ) -temp;
    %%% 避免因基本轨与护轨廓形之间的空档，导致插值不良
    if N_ConPatch_R > 1
        for kk = 1:1:N_ConPatch_R-1            
            Profile_outside = Profile_TrackCS .profile_r.(InpPar.Exp_DummyRail_R{kk});
            Profile_inside = Profile_TrackCS.profile_r.(InpPar.Exp_DummyRail_R{kk+1});
            if ~isempty(Profile_outside) && ~isempty(Profile_inside)
                bools_R = bools_R & ( traceline_w_R(:,2) <= max(Profile_inside(:,1)) | traceline_w_R(:,2) >= min(Profile_outside(:,1)) );
            end
        end
    end
    
    range_y_R = traceline_w_R(bools_R,2)';
    wheel_interp_R = traceline_w_R(bools_R,:);
    rail_interp_R = [range_y_R' interp1(Profile_TrackCS.profile_r.R(:,1),Profile_TrackCS.profile_r.R(:,2), range_y_R,'spline')'];
    bools_linear = (rail_interp_R(:,2)>0.6+8e-3);
    range_y_R_lienar = (range_y_R(bools_linear))';
    rail_interp_R(bools_linear,:) = [range_y_R_lienar interp1(Profile_TrackCS.profile_r.R(:,1),Profile_TrackCS.profile_r.R(:,2),range_y_R_lienar,'linear')];
    rail_interp_R = sortrows(rail_interp_R,1);
    Ver_Dis_R = [range_y_R' rail_interp_R(:,2)-wheel_interp_R(:,3)];
    
    % 计算垂向渗透量
    Elastic_pen_L = [Ver_Dis_L(:,1), Zw_DWL-Ver_Dis_L(:,2)+d0];
    Elastic_pen_R = [Ver_Dis_R(:,1), Zw_DWR-Ver_Dis_R(:,2)+d0];
    
    %% B2. 绘图对比
    if Choose_Plot == 1
        % 迹线状态
        figure(20); clf;
        subplot(1,2,1); plot(wheel_interp_L(:,2), wheel_interp_L(:,3)); set(gca,'ydir','reverse'); grid on
        subplot(1,2,2); plot(wheel_interp_R(:,2), wheel_interp_R(:,3)); set(gca,'ydir','reverse'); grid on
        
        % 迹线与钢轨廓形接触状态
        figure(20); clf;
        subplot(1,2,1); plot(wheel_interp_L(:,2), wheel_interp_L(:,3)+min(Ver_Dis_L(:,2)), rail_interp_L(:,1), rail_interp_L(:,2)); set(gca,'ydir','reverse'); grid on
        subplot(1,2,2); plot(wheel_interp_R(:,2), wheel_interp_R(:,3)+min(Ver_Dis_R(:,2)), rail_interp_R(:,1), rail_interp_R(:,2)); set(gca,'ydir','reverse'); grid on
        
        % 两侧轮轨垂向间隙
        figure(21); clf;
        subplot(1,2,1); plot(Ver_Dis_L(:,1),Ver_Dis_L(:,2)); grid on; title('Ver-Dis-L');
        subplot(1,2,2); plot(Ver_Dis_R(:,1),Ver_Dis_R(:,2)); grid on; title('Ver-Dis-R');
    end
    
%     %% B3. 计算迭代修正量，并修正
%     [~, p] = min(Ver_Dis_L(:,2));
%     [~, q] = min(Ver_Dis_R(:,2));
%     diff_VerDis_Z = Ver_Dis_R(q,2) - Ver_Dis_L(p,2);
%     diff_Roll = diff_VerDis_Z / (Ver_Dis_R(q,1) - Ver_Dis_L(p,1));
%     Oup_Int(i2, :) = [i2, Roll, diff_Roll, diff_VerDis_Z];
%     Roll = Roll+diff_Roll;
%     clc
%     disp(['No. = ', num2str(i1)]);
%     disp(['Mileage = ', num2str(fix(j1*1000)/1000), 'm']);
%     disp(['Mileage-SIP = ', num2str(fix((j1-SIP(2))*1000)/1000), 'm']);
%     disp(['Times = ', num2str(i2)]);
%     
% end



