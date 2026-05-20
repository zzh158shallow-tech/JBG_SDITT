%%  轮轨多点接触算法、非线性Hertz接触算法、Kalker线性蠕滑理论+沈氏理论修正
%  210123：先通过垂向间隙确定接触点，再求法向间隙
%  210204：优化迹线法，采用准弹性接触参数修正
%  210212：计算蠕化率全局坐标系下运动学参数
%  210215：加入道岔廓形
%  210217：法向接触力采用 STRIPES 算法
%  210227：考虑轮对柔性变形对轮轨接触几何的影响
%  210629：使用全局变量，法向力的垂向和横向分量直接计算表示
%  210717：在Hertz接触算法的基础上，考虑接触阻尼
%  230331：针对 STRIPES 接触算法，接触阻尼在各条带分别计算

function [Mileage, Con_str, Pjc, Pjch, Pjcc, Prhx, Prhxf] = Multi_Con_230412_FW_IVb3_CoRunning(InpPar, Par_Vehicle, Par_Track, Par_FW, WheelPro_ProCS, ZP_Con, ZP_Dyn, j1, xlcs, ...
                i11, Zwy, Zsd, Dis_Rail, Vel_Rail, d0, Pjc, Pjch, Pjcc, Prhx, Prhxf, RailPro_ProCS, ...
                pos_Radius_Vehicle, pos_Body_global, vel_Body_global, vel_yaw_track, drtaT, RailBeam_Motion, dX_TIrr_Spline, X_TIrr_Simulation_Start, Tar_SIP)

% i11 = 2;
% i11 = 1;

clear Con_wheel_1_I Con_wheel_1_II Con_wheel_2_I Con_wheel_2_II Con_wheel_3_II Con_rail_1_II Con_rail_2_II
clear Normal_Force elastic_permeability_Unit Con_str Con_RelVel Con_RelVel_max Prhx_T Prhxf_T RHXS RHLv profile_r traceline_w Mileage

%% 1. 初始化
% R0 为轮对对中时车轮滚动圆半径-Test
% R0 = 0.429837250475761;
R0 = Par_Vehicle.R0;
Dlb = Par_Vehicle.Dlb;
Drc = Par_Vehicle.Drc;
Vr = Par_Track.Vr;
Er = Par_Track.Er;
fr = Par_Track.fr;

Choose_Plot = 0;
Mileage  = j1 - Par_Vehicle.Distance_Vehicle(i11);
pos_WSDof = InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i11-1);
Zw = Zwy(pos_WSDof+1,4);    % 轮对垂向位移
Yw = Zwy(pos_WSDof+2,4);    % 轮对横向位移
Yaw = Zwy(pos_WSDof+5,4);   % 轮对摇头角
Roll = Zwy(pos_WSDof+3,4);   % 轮对侧滚角

% profile_w, profile_w_Radius, Con_ang
profile_w.R = WheelPro_ProCS.profile_w_R;
profile_w.L = WheelPro_ProCS.profile_w_L;
Con_ang.R = WheelPro_ProCS.Con_ang_R;
Con_ang.L = WheelPro_ProCS.Con_ang_L;
profile_w_Radius.R = WheelPro_ProCS.profile_w_R_Radius;
profile_w_Radius.L = WheelPro_ProCS.profile_w_L_Radius;
BGmn = WheelPro_ProCS.BGmn;
BGC1 = WheelPro_ProCS.BGC1;
BGC2 = WheelPro_ProCS.BGC2;
                                                                                     
% InpPar.TIrr
Dis_TIrr_temp = cell(1,InpPar.N_ConPatch);
Vel_TIrr_temp = cell(1,InpPar.N_ConPatch);

% if (Mileage< X_TIrr_Simulation_Start-dX_TIrr_Spline) || Mileage>103
if (Mileage< X_TIrr_Simulation_Start-dX_TIrr_Spline)
    Med = 'linear';
else
    Med = 'spline';
end
T1 = InpPar.Exp_WS{i11};
for i2 = 1:1:InpPar.N_ConPatch
    T2 = InpPar.Exp_DummyRail{i2};
    Dis_TIrr_temp{1,i2} = [0, interp1(InpPar.TIrr(:,1),InpPar.TIrr(:,2*i2+1),Mileage,Med), interp1(InpPar.TIrr(:,1),InpPar.TIrr(:,2*i2),Mileage,Med)];
%     if isfield(ZP_Dyn, 'Dis_TIrr')
%         Vel_TIrr_temp{1,i2} = (Dis_TIrr_temp{1,i2} - ZP_Dyn.Dis_TIrr.(T1).(T2)(xlcs,2:4)) ./ drtaT;
%     else
%         Vel_TIrr_temp{1,i2} = [0,0,0];
%     end
    Vel_TIrr_temp{1,i2} = [0, interp1(InpPar.d_TIrr(:,1),InpPar.d_TIrr(:,2*i2+1),Mileage,Med), interp1(InpPar.d_TIrr(:,1),InpPar.d_TIrr(:,2*i2),Mileage,Med)];
end

pos_NM_FW = InpPar.N_track+InpPar.NM_FW*(i11-1)+(1:1:InpPar.NM_FW);

% 导入钢轨变截面廓形
Profile_TrackCS = Get_Profile_P2_v2(InpPar, Par_Track, i11, Dis_Rail, Mileage, RailPro_ProCS, Dis_TIrr_temp);

if Choose_Plot == 1
    % 轮对接触角示意图
    figure(10)
    subplot(2,1,1)
    plot(profile_w.R(:,1)*1000,profile_w.R(:,2)*1000); grid on
    subplot(2,1,2)
    plot(Con_ang.R(:,1)*1000,Con_ang.R(:,2)); grid on
    
    % 车轮廓形曲率半径图
    figure(11)
    for i2 = 1:1:2
        subplot(2,2,i2)
        plot(profile_w.(InpPar.Type_Side{i2})(:,1)*1000,profile_w.(InpPar.Type_Side{i2})(:,2)*1000);
        set(gca,'ydir','reverse');  grid on
        subplot(2,2,i2+2)
        plot(profile_w_Radius.(InpPar.Type_Side{i2})(:,1)*1000, profile_w_Radius.(InpPar.Type_Side{i2})(:,2)*1000);
        ylim([-1000,500]);  grid on;
    end
    
    % 钢轨廓形曲率半径图
    figure(12)
    for i2 = 1:1:2
        Range_X = sortrows(([0.7 0.8]*1000*(-1+(i2>1)*2))', 1);
        subplot(2,2,i2)
        plot(Profile_TrackCS.profile_r.(InpPar.Type_Side{i2})(:,1)*1000, Profile_TrackCS.profile_r.(InpPar.Type_Side{i2})(:,2)*1000);
        set(gca,'ydir','reverse');  grid on; xlim(Range_X);
        subplot(2,2,i2+2)
        plot(Profile_TrackCS.profile_r_Radius.(InpPar.Type_Side{i2})(:,1)*1000, Profile_TrackCS.profile_r_Radius.(InpPar.Type_Side{i2})(:,2)*1000);
        ylim([0,500]);  grid on; xlim(Range_X);
    end
end

%% 2. 计算空间迹线
A_WS = [ cos(Yaw)                 sin(Yaw)                   0;
              -cos(Roll)*sin(Yaw)  cos(Roll)*cos(Yaw)   sin(Roll);
                sin(Roll)*sin(Yaw)  -sin(Roll)*cos(Yaw)   cos(Roll)];
discrete_len_tread = 2.5e-5;
discrete_len_flange = 0.5e-5;

% 2.1 计算半轮对刚体空间位置 InpPar, Par_FW
if InpPar.NM_FW > 0
    % a.考虑轮对弹性，修正侧滚角, Pos_FW
    clear Pos_FW Defor_FW
%     pos_NM_FW = InpPar.N_track+InpPar.NM_FW*(i11-1)+(1:1:InpPar.NM_FW);
    Type_Node = {'NRC', 'WBack', 'WOut'};
    for i1 = 1:1:length(Type_Node)
        T1 = Type_Node{i1};
        for i2 = 1:1:2
            T2 = InpPar.Type_Side{i2};
            T = [T1, '_', T2];
            DOF_pos = Par_FW.DOF_pos.(T);
            pos_ori = InpPar.Pos_Node.(T1).(T2);
            Defor_FW.(T) = InpPar.ModeShape.FW(DOF_pos,:)*Zwy(pos_NM_FW,4);
%             Pos_FW.(T) = [0 Yw Zw] + (pos_ori) * A_WS;
%             Pos_FW.(T) = pos_ori+(Defor_FW.(T))';
            Pos_FW.(T) = [0 Yw Zw] + (pos_ori+(Defor_FW.(T))') * A_WS;
        end
    end
         
    % b.考虑轮对弹性，修正侧滚角和摇头角
    Roll_DW.R = (Pos_FW.WOut_R(3)-Pos_FW.WBack_R(3))/(Pos_FW.WOut_R(2)-Pos_FW.WBack_R(2)) ...
                         -(InpPar.Pos_Node.WOut.R(3)-InpPar.Pos_Node.WBack.R(3))/(InpPar.Pos_Node.WOut.R(2)-InpPar.Pos_Node.WBack.R(2));
                     
    Roll_DW.L = (Pos_FW.WBack_L(3)-Pos_FW.WOut_L(3))/(Pos_FW.WBack_L(2)-Pos_FW.WOut_L(2)) ...
                         -(InpPar.Pos_Node.WBack.L(3)-InpPar.Pos_Node.WOut.L(3))/(InpPar.Pos_Node.WBack.L(2)-InpPar.Pos_Node.WOut.L(2));
                     
    Yaw_DW.R = -(Pos_FW.WOut_R(1)-Pos_FW.WBack_R(1))/(Pos_FW.WOut_R(2)-Pos_FW.WBack_R(2));    
    Yaw_DW.L = -(Pos_FW.WBack_L(1)-Pos_FW.WOut_L(1))/(Pos_FW.WBack_L(2)-Pos_FW.WOut_L(2));

    % c.确定DWR和DWL半轮对轮轴中心的位置   
    for i1 = 1:1:2
        T1 = InpPar.Type_Side{i1};
        A_DW.(T1) = [ cos(Yaw_DW.(T1))                                sin(Yaw_DW.(T1))                                 0;
                              -cos(Roll_DW.(T1))*sin(Yaw_DW.(T1))   cos(Roll_DW.(T1))*cos(Yaw_DW.(T1))  sin(Roll_DW.(T1));
                                sin(Roll_DW.(T1))*sin(Yaw_DW.(T1))  -sin(Roll_DW.(T1))*cos(Yaw_DW.(T1))   cos(Roll_DW.(T1))];
        AD1_OD1 = [0, (Dlb+Drc)*(-1+2*sign(i1-1)), R0]*A_DW.(T1);
        O1_OD1 = Pos_FW.(['NRC_', T1]) - AD1_OD1;
        Yw_DW.(T1) = O1_OD1(2);
        Zw_DW.(T1) = O1_OD1(3);
    end

    % Roll_DW, Yaw_DW, Yw_DW, Zw_DW, A_DW
    for i1 = 1:1:2
        T1 = [InpPar.Type_Side{i1}, '_RW'];
        Roll_DW.(T1) = Roll;
        Yaw_DW.(T1) = Yaw;
        Yw_DW.(T1) = Yw;
        Zw_DW.(T1) = Zw;
        A_DW.(T1) = A_WS;
    end

else
    Roll_DW.R = Roll;	Roll_DW.L = Roll;
    Yaw_DW.R = Yaw;   Yaw_DW.L = Yaw;
    Yw_DW.R = Yw;       Yw_DW.L = Yw;
    Zw_DW.R = Zw;       Zw_DW.L = Zw;
    A_DW.R = A_WS;    A_DW.L = A_WS;
end

% Oup_temp_R = [Roll_DW.R, Yaw_DW.R, Yw_DW.R, Zw_DW.R];
% Oup_temp_L = [Roll_DW.L, Yaw_DW.L, Yw_DW.L, Zw_DW.L];
% Oup_temp_R_RW = [Roll_DW.R_RW, Yaw_DW.R_RW, Yw_DW.R_RW, Zw_DW.R_RW];
% Oup_temp_L_RW = [Roll_DW.L_RW, Yaw_DW.L_RW, Yw_DW.L_RW, Zw_DW.L_RW];
% Roll_DW.R-Roll
% Yaw_DW.R-Yaw
% Yw_DW.R-Yw
% Zw_DW.R-Zw
% A_DW.R-A_WS

clear bools
for i0 = 1:1:1
    % 2.2a 计算空间迹线, Con_ang, traceline_w
    if i0==1
        [Con_ang.R_temp, traceline_w.R, Con_ang.L_temp, traceline_w.L] = TracePrinciple(WheelPro_ProCS, Dlb, discrete_len_flange, discrete_len_tread,...
         profile_w.R, Roll_DW.R, Yaw_DW.R, Yw_DW.R, Zw_DW.R, profile_w.L, Roll_DW.L, Yaw_DW.L, Yw_DW.L, Zw_DW.L);
    else
        [Con_ang.R_RW_temp, traceline_w.R_RW, Con_ang.L_RW_temp, traceline_w.L_RW] = TracePrinciple(WheelPro_ProCS, Dlb, discrete_len_flange, discrete_len_tread,...
         profile_w.R, Roll, Yaw, Yw, Zw, profile_w.L, Roll, Yaw, Yw, Zw);
    end

    for i1 = 1:1:length(InpPar.Type_Side)
        T_Side = InpPar.Type_Side{i1};
        if i0==1
            T1 = T_Side;
        else
            T1 = [T_Side, '_RW'];
        end

        % 2.3a 计算轮轨垂向间隙, bools
        temp = 0.1e-3;
        bools.(T1) = traceline_w.(T1)(:,2) > max( min(traceline_w.(T1)(:,2)), min(Profile_TrackCS.profile_r.(T_Side)(:,1)) ) + temp & ...
                             traceline_w.(T1)(:,2) < min( max(traceline_w.(T1)(:,2)), max(Profile_TrackCS.profile_r.(T_Side)(:,1)) ) - temp;
                   
        % 避免因基本轨与护轨廓形之间的空档，导致插值不良
        Tar = ['N_ConPatch_', T_Side];
        N_Target = InpPar.(Tar);
        if N_Target > 1
            for i = 1:1:N_Target-1
                Profile_outside = Profile_TrackCS.profile_r.([T_Side, num2str(i)]);
                Profile_inside = Profile_TrackCS.profile_r.([T_Side, num2str(i+1)]);
                if ~isempty(Profile_outside) && ~isempty(Profile_inside)
                    if i1==1
                        bools.(T1) = bools.(T1) & ( traceline_w.(T1)(:,2) >= min(Profile_inside(:,1)) | traceline_w.(T1)(:,2) <= max(Profile_outside(:,1)) );
                    elseif i1==2
                        bools.(T1) = bools.(T1) & ( traceline_w.(T1)(:,2) <= max(Profile_inside(:,1)) | traceline_w.(T1)(:,2) >= min(Profile_outside(:,1)) );                    
                    end
                end
            end
        end
        
        range_y.(T1) = traceline_w.(T1)(bools.(T1),2)';
        wheel_interp.(T1) = traceline_w.(T1)(bools.(T1),:);
        rail_interp.(T1) = [range_y.(T1)', interp1(Profile_TrackCS.profile_r.(T_Side)(:,1), Profile_TrackCS.profile_r.(T_Side)(:,2), range_y.(T1), 'spline')'];
        bools_linear = (rail_interp.(T1)(:,2)>0.6+8e-3);
        range_y_lienar.(T1) = (range_y.(T1)(bools_linear))';
        rail_interp.(T1)(bools_linear,:) = [range_y_lienar.(T1), interp1(Profile_TrackCS.profile_r.(T_Side)(:,1), Profile_TrackCS.profile_r.(T_Side)(:,2), range_y_lienar.(T1), 'linear')];
        rail_interp.(T1) = sortrows(rail_interp.(T1), 1);
        Ver_Dis.(T1) = [range_y.(T1)' rail_interp.(T1)(:,2)-wheel_interp.(T1)(:,3)];    
        
        % 2.4a 计算垂向渗透量
        Elastic_pen.(T1) = [Ver_Dis.(T1)(:,1), Zw_DW.(T1)-Ver_Dis.(T1)(:,2)+d0];
        
        % 2.5 绘图对比
        if Choose_Plot == 1
            for i1=1:1:length(InpPar.Type_Side)
                T1 = InpPar.Type_Side{i1};
                % 迹线状态
                figure(20)
                subplot(1,2,i1); plot(wheel_interp.(T1)(:,2), wheel_interp.(T1)(:,3)); set(gca,'ydir','reverse'); grid on
                
                % 迹线与钢轨廓形接触状态
                figure(21)
                subplot(1,2,i1); plot(wheel_interp.(T1)(:,2), wheel_interp.(T1)(:,3)+min(Ver_Dis.(T1)(:,2)), rail_interp.(T1)(:,1), rail_interp.(T1)(:,2));
                set(gca,'ydir','reverse'); grid on
                
                % 两侧轮轨垂向间隙
                figure(22)
                subplot(1,2,i1); plot(Ver_Dis.(T1)(:,1),Ver_Dis.(T1)(:,2)); grid on; title(['Ver-Dis-', InpPar.Type_Side{i1}]);
                
                % 垂向渗透量
                figure(23)
                subplot(1,2,i1); plot(Elastic_pen.(T1)(:,1), Elastic_pen.(T1)(:,2)*1000); set(gca,'ydir','reverse'); grid on; title(['Elastic permeability-', InpPar.Type_Side{i1}]);
                ylabel('(mm)')
                
                % 拟合后、拟合前两侧轮轨法向间隙
        %         figure(24)
        %         subplot(1,2,i1); plot(Nor_Dis.(T1)(:,1), Nor_Dis.(T1)(:,2)); set(gca,'ydir','reverse');  grid on; title(['Nor-Dis-', InpPar.Type_Side{i1}]);
            end
        end
        
        %% 3. 准弹性接触参数修正        
        % Ver_Pen_a 可能接触点在迹线矩阵中的顺序、在绝对坐标系中的横向位置、垂向渗透量、一阶导数
        % Ver_Pen_b 修正后接触点在绝对坐标系中的横向位置、垂向渗透量
        [~,Ver_Pen_a.(T1), Ver_Pen_postive_start.(T1), Ver_Pen_postive_end.(T1)] = Extreme_Boundary(Elastic_pen.(T1),'max');
        
        % Con_wheel_R_1_Ia N*6 准弹性修正前 绝对坐标系下车轮上接触点位置、轮轨垂向渗透量、轮轨法向间隙、接触角 Con_rail_1_I
        % Con_wheel_1_I, Con_rail_1_I, Ver_Pen_b
        [Con_wheel_1_I.a.(T1), Con_rail_1_I.a.(T1), Ver_Pen_b.(T1), Con_wheel_1_I.b.(T1), Con_rail_1_I.b.(T1)] = Quasi_Elastic_Correction...
        (Elastic_pen.(T1), Ver_Pen_a.(T1), Ver_Pen_postive_start.(T1), Ver_Pen_postive_end.(T1), Yw_DW.(T1), Roll_DW.(T1), Con_ang.(T_Side), A_DW.(T1), bools.(T1), wheel_interp.(T1), rail_interp.(T1), Con_ang.([T1 '_temp']));
    end

end

%% 4. 判断是否存在多点接触
y_lim.L = 2e-3;
y_lim.R = 2e-3;
Type_T0 = {'a', 'b'};
% b/A for b*INV(A)
for kk = 1:1:1

    for i1 = 1:1:length(InpPar.Type_Side)
        T_Side = InpPar.Type_Side{i1};
        if kk==1
            T1 = T_Side;
        else
            T1 = [T_Side, '_RW'];
        end
        
        % 4.1a 通过曲率半径筛选左侧轮轨接触点
        for i0 = 1:1:2
            T0 = Type_T0{i0};
            len = length(Con_wheel_1_I.(T0).(T1)(:,1));
            Con_wheel_1_I.(T0).(T1) = sortrows(Con_wheel_1_I.(T0).(T1),2);
            Con_wheel_2_I.(T0).(T1)(:,1:3) = (Con_wheel_1_I.(T0).(T1)(:,1:3)-repmat([0,Yw_DW.(T1),0],len,1)) / A_DW.(T1);
            Con_wheel_2_I.(T0).(T1)(:,4:6) =  Con_wheel_1_I.(T0).(T1)(:,4:6);
            
            % R_yy_w, R_xx_w, R_xx_r, rou
            [R_yy_w.(T0).(T1), R_xx_w.(T0).(T1), R_xx_r.(T0).(T1), rou.(T0).(T1)] = Re_Radius(profile_w_Radius.(T_Side), Profile_TrackCS.profile_r_Radius.(T_Side), Con_wheel_2_I.(T0).(T1), Con_rail_1_I.(T0).(T1));
            if i0==1
                tt = (rou.(T0).(T1)<=0 | R_xx_w.(T0).(T1)==-R_xx_r.(T0).(T1) | (R_xx_w.(T0).(T1)<0 & abs(R_xx_w.(T0).(T1))<=abs(R_xx_r.(T0).(T1))));
                Ver_Pen_postive_start.(T1)(tt,:) = [];
                Ver_Pen_postive_end.(T1)(tt,:) = [];
    %             if ~isempty(find(tt))
    %                 temp = input(['~isempty(find(bools_del_', T1, '))']);
    %             end
            end
            Con_wheel_1_I.(T0).(T1)(tt,:) = [];
            Con_rail_1_I.(T0).(T1)(tt,:) = [];
            R_yy_w.(T0).(T1)(tt,:) = [];
            R_xx_w.(T0).(T1)(tt,:) = [];
            R_xx_r.(T0).(T1)(tt,:) = [];
            rou.(T0).(T1)(tt,:) = [];
            eval(['Ver_Pen_', T0, '.(T1)(tt,:) = [];']);
        end
        
        % 4.1b 通过横向间隔筛选接触点
        len = length(Con_wheel_1_I.b.(T1)(:,1));
        bools_II.(T1) = true(len,1);
        if len > 1
            p = 1;
            for q = 2:1:len
                if abs(Con_wheel_1_I.b.(T1)(p,2)-Con_wheel_1_I.b.(T1)(q,2)) > y_lim.(T1)
                    p = q;
                else
                    if Con_wheel_1_I.b.(T1)(p,5) < Con_wheel_1_I.b.(T1)(q,5)
                        bools_II.(T1)(p,1) = false; p = q;
                    else
                        bools_II.(T1)(q,1) = false;
                    end
                end
            end
        end
        tt = bools_II.(T1);
    
        for i0 = 1:1:2
            T0 = Type_T0{i0};
            Con_wheel_1_II.(T0).(T1) = sortrows(Con_wheel_1_I.(T0).(T1)(tt,:),2);
            Con_rail_1_II.(T0).(T1) = Con_rail_1_I.(T0).(T1)(tt,:);
            R_yy_w.(T0).(T1) = R_yy_w.(T0).(T1)(tt,:);
            R_xx_w.(T0).(T1) = R_xx_w.(T0).(T1)(tt,:);
            R_xx_r.(T0).(T1) = R_xx_r.(T0).(T1)(tt,:);
            rou.(T0).(T1) = rou.(T0).(T1)(tt,:);
            % 4.1c 将接触点位置转换至轮对坐标系
            len = length(Con_wheel_1_II.(T0).(T1)(:,1));
            Con_wheel_2_II.(T0).(T1)(:,1:3) = (Con_wheel_1_II.(T0).(T1)(:,1:3) - repmat([0,Yw_DW.(T1),0], len, 1)) / A_DW.(T1);
            Con_wheel_2_II.(T0).(T1)(:,4:6) = Con_wheel_1_II.(T0).(T1)(:,4:6);
        end
        if ~isempty(Ver_Pen_postive_start.(T1))
            Ver_Pen_postive_start.(T1) = Ver_Pen_postive_start.(T1)(tt,:);
            Ver_Pen_postive_end.(T1) = Ver_Pen_postive_end.(T1)(tt,:);
            Ver_Pen_a.(T1) = Ver_Pen_a.(T1)(tt,:);
            Ver_Pen_b.(T1) = Ver_Pen_b.(T1)(tt,:);
        end
        
    end         % END of i1

end             % END of kk

% Con_wheel_1_II .(T0).(T1) N*6 绝对坐标系下车轮上接触点位置 轮轨垂向间隙 弹性渗透量 接触角
% Con_wheel_1_I .a, 修正前 Track C.S. 坐标
% Con_wheel_1_I .b, 修正后 Track C.S. 坐标
% Con_wheel_1_II .a, 修正前、筛选后 Track C.S. 坐标
% Con_wheel_1_II .b, 修正后、筛选后 Track C.S. 坐标
% Con_wheel_2_I .a, 修正前 WS C.S. 坐标
% Con_wheel_2_I .b, 修正后 WS C.S. 坐标
% Con_wheel_2_II .a, 修正前、筛选后 WS C.S. 坐标
% Con_wheel_2_II .b, 修正后、筛选后 WS C.S. 坐标
% Con_rail_1_II .a, 修正前、筛选后 Track C.S. 坐标
% Con_rail_1_II .b, 修正后、筛选后 Track C.S. 坐标

%% 5. 轮轨法向力计算
clear m n
for i1=1:1:length(InpPar.Type_Side)
    T = InpPar.Type_Side{i1};

    % 5.1 接触坐标系-->绝对坐标系转换矩阵: B_track
    Con_Ang.(T) = Con_wheel_2_II.b.(T)(:,6);
    len = size(Con_wheel_2_II.b.(T)(:,6), 1);
    B_track.(T) = cell(len,1);
    B_global.(T) = cell(len,1);
    for i2 = 1:1:len
        B_track.(T){i2,1} = [ cos(Yaw_DW.(T))                                                             sin(Yaw_DW.(T))                                                               0
                                      -cos(Con_Ang.(T)(i2,1)+Roll_DW.(T))*sin(Yaw_DW.(T))   cos(Con_Ang.(T)(i2,1)+Roll_DW.(T))*cos(Yaw_DW.(T))    sin(Con_Ang.(T)(i2,1)+Roll_DW.(T))
                                        sin(Con_Ang.(T)(i2,1)+Roll_DW.(T))*sin(Yaw_DW.(T))   -sin(Con_Ang.(T)(i2,1)+Roll_DW.(T))*cos(Yaw_DW.(T))    cos(Con_Ang.(T)(i2,1)+Roll_DW.(T))];
    end

    Con_Ang_a.(T) = Con_wheel_2_II.a.(T)(:,6);
    B_track_a.(T) = cell(len,1);
    for i2 = 1:1:len
        B_track_a.(T){i2,1} = [ cos(Yaw_DW.(T))                                                             sin(Yaw_DW.(T))                                                               0
                                         -cos(Con_Ang_a.(T)(i2,1)+Roll_DW.(T))*sin(Yaw_DW.(T))   cos(Con_Ang_a.(T)(i2,1)+Roll_DW.(T))*cos(Yaw_DW.(T))    sin(Con_Ang_a.(T)(i2,1)+Roll_DW.(T))
                                           sin(Con_Ang_a.(T)(i2,1)+Roll_DW.(T))*sin(Yaw_DW.(T))   -sin(Con_Ang_a.(T)(i2,1)+Roll_DW.(T))*cos(Yaw_DW.(T))    cos(Con_Ang_a.(T)(i2,1)+Roll_DW.(T))];
    end
    
    % 5.2 单位法向力作用下,计算接触点处弹性渗透量: a1, b1, m, n, elastic_permeability_Unit
    [~, ~, m.a.(T), n.a.(T), elastic_permeability_Unit.a.(T), Con_A.a.(T), Con_B.a.(T), Con_r.a.(T)] = Par_Hertz(R_yy_w.a.(T), R_xx_w.a.(T), R_xx_r.a.(T), rou.a.(T), BGmn, Vr, Er, InpPar.Type_Normal);
    [~, ~, m.b.(T), n.b.(T), elastic_permeability_Unit.b.(T), Con_A.b.(T), Con_B.b.(T), Con_r.b.(T)] = Par_Hertz(R_yy_w.b.(T), R_xx_w.b.(T), R_xx_r.b.(T), rou.b.(T), BGmn, Vr, Er, InpPar.Type_Normal);
    
    % 5.3a 各个接触斑的法向力——由接触刚度引起的力
    if strcmp(InpPar.Type_Normal, 'Hertz') || strcmp(InpPar.Type_Normal, 'Hertz&ConDamp')
        Normal_Force.(T)(:,5) = (Con_wheel_2_II.b.(T)(:,5)./elastic_permeability_Unit.b.(T)(:,1)).^(3/2);
    end
    
    % 5.3b 翟院士专著-简化算法——由接触刚度引起的力
    if strcmp(InpPar.Type_Normal, 'SIMHertz&ConDamp')
        GG.(T) = 3.86e-8.*(R_yy_w.b.(T)).^(-0.115);
        Normal_Force.(T)(:,5) = (Con_wheel_2_II.b.(T)(:,5)./GG.(T)).^(3/2);
    end
    
    % 5.3c STRIPES法向力算法
    if strcmp(InpPar.Type_Normal, 'STRIPES&ConDamp')
        if ~isempty(Ver_Pen_postive_start.(T))
            N_Stripes.(T) = 51;
            Med = 'AB';
%             Med = 'A';
%             [~, Normal_Force.(T), Area_STRIPES.(T), Con_STRIPES.(T), N_Stripes.(T), Epsilon.(T)] = NF_STRIPES_211229(profile_w_Radius.(T), Profile_TrackCS.profile_r_Radius.(T), ...
%              Con_wheel_1_II.a.(T), Con_wheel_2_II.a.(T), Con_rail_1_II.a.(T), wheel_interp.(T), rail_interp.(T), ...
%              Yw_DW.(T), A_DW.(T), B_track.(T), Ver_Pen_a.(T), m.a.(T), n.a.(T), Con_A.a.(T), Con_B.a.(T), Con_r.a.(T), BGmn, Er, Vr, N_Stripes.(T), Med);
            [~, Normal_Force.(T), Area_STRIPES.(T), Con_STRIPES.(T), N_Stripes.(T), Epsilon.(T)] = NF_STRIPES_230623(profile_w_Radius.(T), Profile_TrackCS.profile_r_Radius.(T), ...
             Con_wheel_1_II.a.(T), Con_wheel_2_II.a.(T), Con_rail_1_II.a.(T), wheel_interp.(T), rail_interp.(T), ...
             Yw_DW.(T), A_DW.(T), B_track_a.(T), Ver_Pen_a.(T), m.a.(T), n.a.(T), Con_A.a.(T), Con_B.a.(T), Con_r.a.(T), BGmn, Er, Vr, N_Stripes.(T), Med);
        else
            Normal_Force.(T)(1,5) = 1e-3;
            Area_STRIPES.(T) = 0;
            Con_STRIPES.(T) = [];
            Epsilon.(T) = [];
        end
    else
        Area_STRIPES.(T) = zeros(size(Normal_Force.(T),1),1);
    end
    
    bools_NF.(T) = Normal_Force.(T)(:,5)<=0;
    Normal_Force.(T)(bools_NF.(T),5) = 1e-3;
end
% ========================== 输出
% Con_wheel_2_II .b.(T) % N*6 轮对坐标系下车轮上接触点空间位置 轮轨垂向间隙 弹性渗透量 接触角 Con_wheel_2_II
% Con_rail_1_IIb.(T)  % N*2 绝对坐标系下钢轨上接触点平面位置
% Normal_Force.(T)   % N*6 不同接触点处的法向力【法向力标量，法向力横向分矢量，法向力垂向分矢量，接触斑编号, 分别由接触刚度和阻尼引起的法向接触力】


%% 6. 蠕滑力求解
% 6.1 初始化参数 =======================
% 6.1a Track CS: pos_WS_track, vel_WS_track
pos_WS_track = [0, Yw, Zw, Roll, 0, Yaw];
vel_WS_track  = [0, Zsd(pos_WSDof+[2,1,3,4,5]',4)'];

% 6.1b Global CS: pos_WS_global, vel_WS_global
% 直接用公式含义计算 pos_Trans/Rot_global
pos_Yaw_trackCS = pos_Radius_Vehicle(i11,3);
if pos_Yaw_trackCS==0
    pos_WS_global = pos_WS_track;
else
    A_track = [ cos(pos_Yaw_trackCS)             sin(pos_Yaw_trackCS)              0;
                     -cos(0)*sin(pos_Yaw_trackCS)   cos(0)*cos(pos_Yaw_trackCS)  sin(0);
                       sin(0)*sin(pos_Yaw_trackCS)  -sin(0)*cos(pos_Yaw_trackCS)   cos(0)];
    pos_temp = pos_Body_global(i11,1:3) + pos_WS_track(1:3)*A_track;
    pos_WS_global = [pos_temp, pos_WS_track(4), pos_WS_track(5), pos_Yaw_trackCS+pos_WS_track(6)];
end

% 直接用公式含义计算 vel_Trans/Rot_global
R_track_d1 = vel_Body_global(i11,:);
vel_Yaw_trackCS = vel_yaw_track(i11,1);
if pos_Yaw_trackCS==0&&vel_Yaw_trackCS==0
    vel_WS_global(1:3) = R_track_d1 + vel_WS_track(1:3);
    vel_WS_global(4:6) = vel_WS_track(4:6)';    
else
    % Trans
    A_track_d1 = double(subs(T_Track_d1_Yaw, [Inp_Pos_Yaw, Inp_Vel_Yaw], [pos_Yaw_trackCS, vel_Yaw_trackCS]));
    vel_WS_global(1:3) = R_track_d1 + vel_WS_track(1:3)*A_track + pos_WS_track(1:3)*A_track_d1;    
    % Rot: 旋转矩阵 G 的各列为沿 3 个卡尔丹转动轴的单位矢量在全局坐标系中的坐标列阵
    % 旋转顺序 3-1-2
    TG_WS_Track = [cos(Inp_Pos_Yaw)  -sin(Inp_Pos_Yaw)*cos(Inp_Pos_Roll)    0
                               sin(Inp_Pos_Yaw)    cos(Inp_Pos_Yaw)*cos(Inp_Pos_Roll)    0
                               0                              sin(Inp_Pos_Roll)                                  1];
    G_Cardan_WS = double(subs(TG_WS_Track, [Inp_Pos_Yaw,Inp_Pos_Roll], [pos_WS_track(6), pos_WS_track(4)]));
    G_Cardan_track = double(subs(TG_WS_Track, [Inp_Pos_Yaw,Inp_Pos_Roll], [pos_Yaw_trackCS, 0]));
    Angvel_temp = [0; 0; vel_Yaw_trackCS] + G_Cardan_WS * vel_WS_track(4:6)';
    vel_WS_global(4:6) = (G_Cardan_track * Angvel_temp)';
end

% 6.1c 接触坐标系-->绝对坐标系转换矩阵: B_track, B_global
for i1=1:1:length(InpPar.Type_Side)
    T = InpPar.Type_Side{i1};
    Con_Ang.(T) = Con_wheel_2_II.b.(T)(:,6);
    Vgd.(T) = InpPar.Vlc/2.*(1+Con_wheel_2_II.b.(T)(:,3)./R0.*cos(Yaw_DW.(T)));    % 名义速度  m/s
    len = size(Normal_Force.(T), 1);
    if pos_Yaw_trackCS==0
        A_global = A_WS;
        B_global.(T) = B_track.(T);
    else
        A_global = [ cos(pos_WS_global(6))                                        sin(pos_WS_global(6))                                         0;
                           -cos(pos_WS_global(4))*sin(pos_WS_global(6))   cos(pos_WS_global(4))*cos(pos_WS_global(6))  sin(pos_WS_global(4));
                             sin(pos_WS_global(4))*sin(pos_WS_global(6))  -sin(pos_WS_global(4))*cos(pos_WS_global(6))   cos(pos_WS_global(4))];
        for i2 = 1:1:len
            Yaw_temp = Yaw_DW.(T)+pos_Yaw_trackCS;
            B_global.(T){i2,1} = [ cos(Yaw_temp)                                                             sin(Yaw_temp)                                                               0
                                            -cos(Con_Ang.(T)(i2,1)+Roll_DW.(T))*sin(Yaw_temp)   cos(Con_Ang.(T)(i2,1)+Roll_DW.(T))*cos(Yaw_temp)    sin(Con_Ang.(T)(i2,1)+Roll_DW.(T))
                                              sin(Con_Ang.(T)(i2,1)+Roll_DW.(T))*sin(Yaw_temp)   -sin(Con_Ang.(T)(i2,1)+Roll_DW.(T))*cos(Yaw_temp)    cos(Con_Ang.(T)(i2,1)+Roll_DW.(T))];
        end
    end
end

% 6.1d 判断该时间步与上一时间步之间，接触点的连续关系: ConPosInt, Con_rail_1_II
clear ConPosInt
for i1=1:1:length(InpPar.Type_Side)
    T1 = InpPar.Type_Side{i1};    
    for i2 = 1:1:size(Normal_Force.(T1),1)
        % i_Patch
        i_Patch = Judge_i2_ConRail(i1, Con_rail_1_II.b.(T1)(i2,1), Profile_TrackCS);
        Normal_Force.(T1)(i2,4) = i_Patch;      
        T2 = InpPar.Exp_DummyRail{i_Patch};
        
        % Con_rail_2_II, R_Con_r
        dy = (0.7175+Par_Track.Ori_prr)*(-1+2*sign(i1-1)) + Profile_TrackCS.profile_r_dY.(T2);
        dz = Profile_TrackCS.profile_r_dZ.(T2);
        Con_rail_2_II.(T1)(i2,1:2) = Con_rail_1_II.b.(T1)(i2,1:2) - [dy, dz];
%         if InpPar.NM_FW>0 && size(Con_rail_1_II.b.([T1, '_RW']),1)>=i2
%             Con_rail_2_II.([T1, '_RW'])(i2,1:2) = Con_rail_1_II.b.([T1, '_RW'])(i2,1:2) - [dy, dz];
%         end
        
        % Dis_temp, ZP_Con, ConPosInt
        Target_PrevConPos_rail = ZP_Con.Rail_2.(T2);
        Dis_temp = repmat(Con_rail_2_II.(T1)(i2,1), length(Target_PrevConPos_rail(i11,:)), 1);
        for i3 = 1:1:length(Target_PrevConPos_rail(i11,:))
            if ~isempty(Target_PrevConPos_rail{i11,i3})
                bools_NaN = (Target_PrevConPos_rail{i11,i3}(:,1)==0);
                Target_PrevConPos_rail{i11,i3}(bools_NaN,:) = NaN(length(find(bools_NaN)),3);
                if size(Target_PrevConPos_rail{i11,i3},1)<xlcs
                    Dis_temp(i3,2) = NaN;
                else
                    Dis_temp(i3,2) = Target_PrevConPos_rail{i11,i3}(xlcs,2);
                end
            else
                Dis_temp(i3,2) = NaN;
            end
        end
        Dis_temp(:,3) = Dis_temp(:,1)-Dis_temp(:,2);
        [value, pos] = min(abs(Dis_temp(:,3)));
        if isnan(value) || value>2e-3
            ConPosInt.(T1)(i2,:) = [NaN, i_Patch];
        else
            ConPosInt.(T1)(i2,:) = [pos, i_Patch];
        end
    end    
end

% 6.2a-I 轮对在接触点的绝对速度 Vjd =======================  M6
% 绝对坐标下，轮对绕X、Y和Z轴转动速度  rad/s
Vzdli = vel_WS_track(4) * cos(pos_WS_global(6)) - (-InpPar.Vlc/R0 + vel_WS_track(5)) * cos(pos_WS_global(4)) * sin(pos_WS_global(6));
Vzdlj = vel_WS_track(4) * sin(pos_WS_global(6)) + (-InpPar.Vlc/R0 + vel_WS_track(5)) * cos(pos_WS_global(4)) * cos(pos_WS_global(6));
Vzdlk = (-InpPar.Vlc/R0 + vel_WS_track(5)) * sin(pos_WS_global(4)) + vel_WS_global(6);
AngVel_Track = [Vzdli, Vzdlj, Vzdlk];

clear Vjd Vjd_r Vjd_Stripes R_Con R_Con_Stripes
clear Tar_u_rigid_WS Tar_u_flex_WS Tar_DOFpos Tar_deformation_WS Tar_ShapeFun ConPon_FlexDefor
for i1=1:1:length(InpPar.Type_Side)
    T1 = InpPar.Type_Side{i1};
    len = size(Normal_Force.(T1),1);

    if InpPar.NM_FW > 0
        % FW
        % A. Pos_FW, DOF_pos_FW, Vel_FW_global, ShapeFun_FW, Vjd
        [Pos_FW.Tread.(T1), DOF_pos_FW.(T1), Vel_FW_global.(T1), ShapeFun_FW.(T1), Vjd.(T1)] = ...
        Cal_Vjd_FW_230515(InpPar, Par_FW, T1, Con_wheel_1_II.b.(T1), R0, Yw, i11, Zwy, Zsd, A_WS, pos_WS_track, vel_WS_global, vel_WS_track);
        
        % B. 刚性接触点位置 Con_wheel_2_II
        Con_wheel_2_II.b.([T1, '_RW']) = Con_wheel_2_II.b.(T1);
        % B1. 通过柔性接触点减去 WS 坐标系内变形插值量而计算
        ConPon_FlexDefor = zeros(len,3);
        for i2= 1:1:len
            Tar_u_flex_WS.XOY{i2,1} = Pos_FW.Tread.(T1).WS.Node_Around_XOY{i2,1};
            Tar_DOFpos.XOY{i2,1} = DOF_pos_FW.(T1).Node_Around_XOY{i2,1};
            temp = reshape(Tar_DOFpos.XOY{i2,1}, 12, 1);
            Tar_deformation_WS.XOY{i2,1}(:,1) = Tar_u_flex_WS.XOY{i2,1}(:,1);
            Tar_deformation_WS.XOY{i2,1}(:,2:4) = reshape(InpPar.ModeShape.FW(temp,:)*Zwy(pos_NM_FW,4), 4, 3);
            Tar_ShapeFun.XOY{i2,1} = ShapeFun_FW.(T1).XOY{i2,1};

            Tar_u_flex_WS.Z{i2,1} = Pos_FW.Tread.(T1).WS.Node_Around_Z{i2,1};
            Tar_DOFpos.Z{i2,1} = DOF_pos_FW.(T1).Node_Around_Z{i2,1};
            temp = reshape(Tar_DOFpos.Z{i2,1}, 6, 1);
            Tar_deformation_WS.Z{i2,1}(:,1) = Tar_u_flex_WS.Z{i2,1}(:,1);
            Tar_deformation_WS.Z{i2,1}(:,2:4) = reshape(InpPar.ModeShape.FW(temp,:)*Zwy(pos_NM_FW,4), 2, 3);
            Tar_ShapeFun.Z{i2,1} = ShapeFun_FW.(T1).Z{i2,1};

            ConPon_FlexDefor(i2,1:3) = [Tar_ShapeFun.XOY{i2,1}*Tar_deformation_WS.XOY{i2,1}(:,2),...
                                                          Tar_ShapeFun.XOY{i2,1}*Tar_deformation_WS.XOY{i2,1}(:,3),...
                                                          Tar_ShapeFun.Z{i2,1}*Tar_deformation_WS.Z{i2,1}(:,4)];
            Con_wheel_2_II.b.([T1, '_RW'])(i2,1:3) = Con_wheel_2_II.b.([T1, '_RW'])(i2,1:3)-ConPon_FlexDefor(i2,:);
            Con_wheel_3_II.b.([T1, '_RW'])(i2,1:3) = Con_wheel_2_II.b.([T1, '_RW'])(i2,1:3)*A_DW.(T1);
        end
        % B2. 通过形函数插值出刚性接触点
%         for i2 = 1:1:len
%             Tar_u_rigid_WS.XOY{i2,1} = InpPar.Pos_Node.FW(Pos_FW.Tread.(T1).WS.Node_Around_XOY{i2,1}(:,1),:);
%             Tar_ShapeFun.XOY{i2,1} = ShapeFun_FW.(T1).XOY{i2,1};
% 
%             Tar_u_rigid_WS.Z{i2,1} = InpPar.Pos_Node.FW(Pos_FW.Tread.(T1).WS.Node_Around_Z{i2,1}(:,1),:);
%             Tar_ShapeFun.Z{i2,1} = ShapeFun_FW.(T1).Z{i2,1};
% 
%             Con_wheel_2_II.b.([T1, '_RW'])(i2,1:3) = [Tar_ShapeFun.XOY{i2,1}*Tar_u_rigid_WS.XOY{i2,1}(:,2),...
%                                                                             Tar_ShapeFun.XOY{i2,1}*Tar_u_rigid_WS.XOY{i2,1}(:,3),...
%                                                                             Tar_ShapeFun.Z{i2,1}*Tar_u_rigid_WS.Z{i2,1}(:,4)];
%             Con_wheel_2_II.b.([T1, '_RW_TrackCS'])(i2,1:3) = Con_wheel_2_II.b.([T1, '_RW'])(i2,1:3)*A_WS;
% %             Con_wheel_2_II.b.([T1, '_RW_TrackCS'])(i2,1:3) = Con_wheel_2_II.b.([T1, '_RW'])(i2,1:3)*A_DW.(T1);
%         end
        % B3. 通过柔性接触点减去质心变形求解
%         for i2 = 1:1:len
%             Con_wheel_3_II.b.([T1, '_RW'])(i2,1:3) = Con_wheel_1_II.b.(T1)(i2,1:3) - [0, Yw_DW.(T1), 0];
%         end
    else
        % RW
        for i2 = 1:1:len
            % 接触点相对于 随体C.S. 原点的矢径在 Global C.S. 的投影 R
            R_Con.(T1)(i2,:) = Con_wheel_2_II.b.(T1)(i2,1:3) * A_global;
            Con_wheel_3_II.b.(T1)(i2,1:3) = Con_wheel_2_II.b.(T1)(i2,1:3)*A_DW.(T1);
            Vjd.(T1)(i2,:) = zeros(3,1);
        end
        Vjd.(T1)(:,1) = Vjd.(T1)(:,1) + vel_WS_global(1) + Vzdlj.*R_Con.(T1)(:,3) - Vzdlk.*R_Con.(T1)(:,2);
        Vjd.(T1)(:,2) = Vjd.(T1)(:,2) + vel_WS_global(2) + Vzdlk.*R_Con.(T1)(:,1) - Vzdli.*R_Con.(T1)(:,3);
        Vjd.(T1)(:,3) = Vjd.(T1)(:,3) + vel_WS_global(3) + Vzdli.*R_Con.(T1)(:,2) - Vzdlj.*R_Con.(T1)(:,1);
    end
    
    % C. 接触点迁移速度-Wheel
    for i2 = 1:1:len
        T2 = InpPar.Exp_DummyRail{ConPosInt.(T1)(i2,2)};
        num_Adv = ConPosInt.(T1)(i2,1);
        if ~isnan(num_Adv) && ~isnan(ZP_Con.Wheel_2.(T2){i11,num_Adv}(xlcs,2)) && RailBeam_Motion.Vel_Y.(T2)(i11,1)~=0
            t1_range = 150;
            t1 = (xlcs-t1_range) - (xlcs-(t1_range+1))*(xlcs<(t1_range+1));
            drta_T_All = [ZP_Dyn.drtaT(t1+1:xlcs,2); drtaT];
            Time = [ZP_Dyn.T(t1:xlcs,1); ZP_Dyn.T(xlcs,1)+drtaT];

            % C1. Lat
            % Vel_ConPos_w_2I
            temp = ZP_Con.Wheel_3_RW.(T2){i11,num_Adv}(t1:xlcs,:);
            bools_NaN = (temp(:,1)==0);
            temp(bools_NaN,:) = NaN(length(find(bools_NaN)),4);
            if InpPar.NM_FW>0
                ConPos_w_2_Lat = [temp(:,2:4); Con_wheel_3_II.b.([T1, '_RW'])(i2,1:3)];
            else
                ConPos_w_2_Lat = [temp(:,2:4); Con_wheel_3_II.b.(T1)(i2,1:3)];
            end
            % Fit: ConPos_w_2II
            ConPos_w_2II_Lat = ConPos_w_2_Lat;
            bools_fit = find(~isnan(ConPos_w_2II_Lat(:,1)));
            if length(bools_fit)>3 && ~(strcmp(T2, 'R1')&&Mileage>50+1e-4&&Mileage<50.85) && ~(strcmp(T2, 'R2')&&Mileage>103.148&&Mileage<103.40)
                if Mileage<70
%                     p = 1-5e-7;      % SW
                    p = 1-5e-9;      % SW
                else
                    p = 1-5e-13;    % CR
                end
                [ConPos_w_2II_Lat(bools_fit,2), ~] = csaps(Time(bools_fit), ConPos_w_2II_Lat(bools_fit,2), p, Time(bools_fit));
            end
            % Consider Ver
%             [ConPos_w_2II(bools_fit,3), ~] = csaps(Time(bools_fit), ConPos_w_2II(bools_fit,3), 1-5e-13, Time(bools_fit));
            % Vjd-Lat
            Vel_ConPos_w_2I_Lat = diff(ConPos_w_2_Lat) ./ repmat(drta_T_All,1,3);
            Vel_ConPos_w_2II_Lat= diff(ConPos_w_2II_Lat) ./ repmat(drta_T_All,1,3);
            temp = Vel_ConPos_w_2II_Lat(end,:) * RailBeam_Motion.Win.(T2)(i11,1);
            Vjd.(T1)(i2,2) = Vjd.(T1)(i2,2) + temp(2);
%             Vjd.(T1)(i2,3) = Vjd.(T1)(i2,3) + temp(3);

            % C2. Ver
%             temp = ZP_Con.Wheel_2_RW.(T2){i11,num_Adv}(t1:xlcs,1:4);
%             bools_NaN = (temp(:,1)==0);
%             temp(bools_NaN,:) = NaN(length(find(bools_NaN)),4);
%             if InpPar.NM_FW>0
%                 ConPos_w_2_Ver = [temp(:,2:4); Con_wheel_2_II.b.([T1, '_RW'])(i2,1:3)];
%             else
%                 ConPos_w_2_Ver = [temp(:,2:4); Con_wheel_2_II.b.(T1)(i2,1:3)];
%             end
%             % Fit: ConPos_w_2II
%             ConPos_w_2II_Ver = ConPos_w_2_Ver;
%             bools_fit = find(~isnan(ConPos_w_2II_Ver(:,1)));
%             if length(bools_fit)>3 && ~(strcmp(T2, 'R1')&&Mileage>50+1e-4&&Mileage<50.85) && ~(strcmp(T2, 'R2')&&Mileage>103.148&&Mileage<103.40)
%                 p = 1-5e-13;
%                 [ConPos_w_2II_Ver(bools_fit,3), ~] = csaps(Time(bools_fit), ConPos_w_2II_Ver(bools_fit,3), p, Time(bools_fit));
%             end
%             % Vjd-Ver
%             Vel_ConPos_w_2I_Ver = diff(ConPos_w_2_Ver) ./ repmat(drta_T_All,1,3);
%             Vel_ConPos_w_2II_Ver= diff(ConPos_w_2II_Ver) ./ repmat(drta_T_All,1,3);
%             temp = Vel_ConPos_w_2II_Ver(end,:) * A_DW.(T1) * RailBeam_Motion.Win.(T2)(i11,1);
%             Vjd.(T1)(i2,3) = Vjd.(T1)(i2,3) + temp(3);

            % C3. Plot and Comparison
            if Choose_Plot==1
                figure(98); clf
                tt = 1:1:length(Time)-2;
%                 plot(Time(tt), Vel_ConPos_w_2I_Lat(:,2)); hold on
%                 plot(Time(tt), Vel_ConPos_w_2II_Lat(:,2)); hold on
                plot(Time(tt), Vel_ConPos_w_2I_Ver(:,3)); hold on
                plot(Time(tt), Vel_ConPos_w_2II_Ver(:,3)); hold on
                grid on
            end
        end
    end

end

% 6.2a-II 轮对在接触点的绝对速度 Vjd-RW ======================= M7, M7b
% 绝对坐标下，轮对绕X、Y和Z轴转动速度  rad/s
% G_Cardan_WSBody = [1, 0, 0; 0, 1, sin(pos_WS_track(4)); 0, 0, cos(pos_WS_track(4))];
% AngVel_WSBody = G_Cardan_WSBody * [vel_WS_track(1,4), -InpPar.Vlc/R0+vel_WS_track(1,5), vel_WS_track(1,6)]';
% AngVel_Track = AngVel_WSBody'*A_WS;
% Vzdli = AngVel_Track(1);
% Vzdlj = AngVel_Track(2);
% Vzdlk = AngVel_Track(3);
% 
% A_WS_d1, ZP_Con
% R11_d1 = -sin(Yaw)*vel_WS_track(1,6);
% R12_d1 = cos(Yaw)*vel_WS_track(1,6);
% R13_d1 = 0;
% R21_d1 = sin(Roll)*sin(Yaw)*vel_WS_track(1,4)-cos(Roll)*cos(Yaw)*vel_WS_track(1,6);
% R22_d1 =-sin(Roll)*cos(Yaw)*vel_WS_track(1,4)-cos(Roll)*sin(Yaw)*vel_WS_track(1,6);
% R23_d1 =  cos(Roll)*vel_WS_track(1,4);
% R31_d1 =  cos(Roll)*sin(Yaw)*vel_WS_track(1,4)+sin(Roll)*cos(Yaw)*vel_WS_track(1,6);
% R32_d1 = -cos(Roll)*cos(Yaw)*vel_WS_track(1,4)+sin(Roll)*sin(Yaw)*vel_WS_track(1,6);
% R33_d1 = -sin(Roll)*vel_WS_track(1,4);
% A_WS_d1 = [R11_d1, R12_d1, R13_d1;	R21_d1, R22_d1, R23_d1;	R31_d1, R32_d1, R33_d1];
% 
% clear Vjd Vjd_r Vjd_Stripes R_Con R_Con_Stripes
% for i1=1:1:length(InpPar.Type_Side)
%     T = InpPar.Type_Side{i1};
%     len = size(Normal_Force.(T),1);
%     for i2 = 1:1:len
%         % 接触点迁移速度-Wheel
%         T2 = InpPar.Exp_DummyRail{ConPosInt.(T)(i2,2)};
%         num_Adv = ConPosInt.(T)(i2,1);
%         if ~isnan(num_Adv) && ~isnan(ZP_Con.Wheel_2.(T2){i11,num_Adv}(xlcs,2)) && RailBeam_Motion.Vel_Y.(T2)(i11,1)~=0    % M7bIII, M7bIV, M7bVI
%             % M7b-Fliter
%             if xlcs<101
%                 t1 = 1;
%             else
%                 t1 = xlcs-100;
%             end
%             ConPos_w_2 = [ZP_Con.Wheel_2.(T2){i11,num_Adv}(t1:xlcs,2:4); Con_wheel_2_II.b.(T)(i2,1:3)];
%             drta_T_All = [ZP_Dyn.drtaT(t1+1:xlcs,2); drtaT];
%             Vel_ConPos_w_2I = diff(ConPos_w_2) ./ repmat(drta_T_All,1,3);
%             % Fit
%             p = 1-5e-11;
%             Vel_ConPos_w_2II = Vel_ConPos_w_2I;
%             bools_fit = find(~isnan(Vel_ConPos_w_2II(:,1)));
%             if length(bools_fit)>1 && ~(strcmp(T2, 'R1')&&Mileage>50&&Mileage<50.85) && ~(strcmp(T2, 'R2')&&Mileage<103.40)
%                 Time = [ZP_Dyn.T(t1+1:xlcs,1); ZP_Dyn.T(xlcs,1)+drtaT];
%                 [Vel_ConPos_w_2II(bools_fit,2), ~] = csaps(Time(bools_fit), Vel_ConPos_w_2II(bools_fit,2), p, Time(bools_fit));
%             end
%             % Vel_ConPos_TrackCS
%             Vjd.(T)(i2,:) = Vel_ConPos_w_2II(end,:) * A_WS;
%             % 不考虑纵向和垂向的迁移速度
%             Vjd.(T)(i2,[1,3]) = 0;
%         else
%             Vjd.(T)(i2,:) = zeros(1,3);
%         end
%     end
%     % Vw+Ucon*A_d1
%     Vjd.(T) = Vjd.(T) + repmat([R_track_d1(1),vel_WS_track(2:3)],len,1) + Con_wheel_2_II.b.(T)(:,1:3)*A_WS_d1;
%     % 叠加牵连点速度
%     Vjd.(T) = Vjd.(T) + [(-InpPar.Vlc/R0+vel_WS_track(1,5)).*Con_wheel_2_II.b.(T)(:,3), zeros(len,2)]*A_WS;
% end

% 6.2b, 6.3 =======================
for i1=1:1:length(InpPar.Type_Side)
    T1 = InpPar.Type_Side{i1};
    len = size(Normal_Force.(T1), 1);
    for i2 = 1:1:len
        i_Patch = Normal_Force.(T1)(i2,4);
        T2 = InpPar.Exp_DummyRail{i_Patch};
        pos = InpPar.N_ConPatch*(i11-1)+i_Patch;
        
        % 6.2b 计算钢轨上接触点速度 Vjd_r =======================
        % M1: Zhai's Monograph
        Vjd_r.(T1)(i2,1:3) = Vel_Rail(pos,1:3)+Vel_TIrr_temp{1,i_Patch};

        num_Adv = ConPosInt.(T1)(i2,1);
        % 尖轨处降低值抬升速度
        if strcmp(T2, 'R2')
            % M7b: RailBeam_Motion
            Vjd_r.(T1)(i2,3) = Vjd_r.(T1)(i2,3) + RailBeam_Motion.Vel_Z.(T2)(i11,1);
        end

        if ~isnan(num_Adv)&&~(strcmp(T2, 'R1')&&Mileage>50+1e-4&&Mileage<50.85) && ~(strcmp(T2, 'R2')&&Mileage>103.148&&Mileage<103.40)             % FWVa
            % M7b: RailBeam_Motion
            Vjd_r.(T1)(i2,2) = Vjd_r.(T1)(i2,2) + RailBeam_Motion.Vel_Y.(T2)(i11,1)*RailBeam_Motion.Win.(T2)(i11,1);      % FWVc
        else
            % M6: 接触点迁移速度-Rail
            num_Adv = ConPosInt.(T1)(i2,1);
            if ~isnan(num_Adv) && ~isnan(ZP_Con.Rail_2.(T2){i11,num_Adv}(xlcs,2))
                % 只考虑横向迁移速度
                Vjd_r.(T1)(i2,2) = Vjd_r.(T1)(i2,2) + (Con_rail_2_II.(T1)(i2,1)-ZP_Con.Rail_2.(T2){i11,num_Adv}(xlcs,2))/drtaT*RailBeam_Motion.Win.(T2)(i11,1);      % FWVc
            end
        end
        
        % 6.3 速度差 Vsdc, Vsdc_Stripes（绝对坐标系-->接触点坐标系） =======================
        Vsdc.(T1)(i2,1:3) = (Vjd.(T1)(i2,:)-Vjd_r.(T1)(i2,:)) / B_global.(T1){i2};
        Vjsdc.(T1)(i2,1:3) = [Vzdli Vzdlj Vzdlk] / B_global.(T1){i2};
    end
end

% 6.4 计算法向接触阻尼力 Normal_Force =======================
clear NF_Damp_Stripes_temp Con_RelVel_Stripes
for i1=1:1:length(InpPar.Type_Side)
    T = InpPar.Type_Side{i1};
    
    % 6.4a Con_RelVel, Con_RelVel_max
    for i2 = 1:1:size(Normal_Force.(T),1)
        i_Patch = Normal_Force.(T)(i2,4);
        T2 = InpPar.Exp_DummyRail{i_Patch};
        Target_Con_RelVel_max = ZP_Con.RelVel_max.(T2);       
        % Con_RelVel_max_ori
        num_Adv = ConPosInt.(T)(i2,1);
        if isnan(num_Adv)
            Con_RelVel_max_ori = 0;
        else
            Con_RelVel_max_ori = Target_Con_RelVel_max{i11,num_Adv}(xlcs,2);
        end
        % Con_RelVel
        Con_RelVel.(T)(i2,1) = Vsdc.(T)(i2,3);
        if Con_RelVel_max_ori<=0 && Con_RelVel.(T)(i2,1)<=0
            Con_RelVel_max.(T)(i2,1) = Con_RelVel.(T)(i2,1);
            Con_RelVel.(T)(i2,2) = 0;
        elseif Con_RelVel.(T)(i2,1) >= Con_RelVel_max_ori
            Con_RelVel_max.(T)(i2,1) = Con_RelVel.(T)(i2,1);
            Con_RelVel.(T)(i2,2) = 1;
        else
            Con_RelVel_max.(T)(i2,1) = Con_RelVel_max_ori;
            Con_RelVel.(T)(i2,2) = Con_RelVel.(T)(i2,1)/Con_RelVel_max_ori;
        end        
        if strcmp(InpPar.Type_Normal, 'STRIPES&ConDamp') && ~isempty(Ver_Pen_postive_start.(T))
%             len = size(Vsdc_Stripes.(T){i2,1},1);
%             Con_RelVel_Stripes.(T){i2,1}(:,1) = Vsdc_Stripes.(T){i2,1}(:,3);
%             if Con_RelVel_max_ori<=0 && Con_RelVel.(T)(i2,1)<=0
%                 Con_RelVel_Stripes.(T){i2,1}(:,2) = zeros(len,1);
%             elseif Con_RelVel.(T)(i2,1) >= Con_RelVel_max_ori
%                 Con_RelVel_Stripes.(T){i2,1}(:,2) = ones(len,1);
%             else
%                 Con_RelVel_Stripes.(T){i2,1}(:,2) = Con_RelVel_Stripes.(T){i2,1}(:,1)./Con_RelVel_max_ori;                
%             end
            Con_RelVel_Stripes.(T){i2,1} = repmat(Con_RelVel.(T)(i2,:), N_Stripes.(T), 1);
        end        
    end
    
    % 6.4b Normal_Force
    bools.(T) = Con_wheel_2_II.b.(T)(:,5) > 0;
    if strcmp(InpPar.Type_Normal, 'STRIPES&ConDamp') && ~isempty(Ver_Pen_postive_start.(T))
        NF_Damp_Stripes_temp = cell(size(Normal_Force.(T),1),1);
        Range_i2 = find(bools.(T));
        for kk = 1:1:length(Range_i2)
            i2 = Range_i2(kk);
            i_Patch = Normal_Force.(T)(i2,4);
            if Tar_SIP==1
                Win_xy = [28.5, 0; 40, 1; 1000, 1];
            elseif Tar_SIP==2
                Win_xy = [87, 0; 97, 1; 1000, 1];
            end
            Win = interp1(Win_xy(:,1), Win_xy(:,2), j1, 'linear');
%             if i_Patch~=3
%             if i_Patch<=2
%             if i_Patch~=4
                % Hu-Guo
                NF_Damp_Stripes_temp{i2,1} = Con_STRIPES.(T){i2,4}(:,1) .* (3*(1-0.83)/2/0.83.*Con_RelVel_Stripes.(T){i2,1}(:,2)) .* Con_STRIPES.(T){i2,4}(:,2);
%             else
                % Lankarani–Nikravesh
%                 NF_Damp_Stripes_temp{i2,1} = Con_STRIPES.(T){i2,4}(:,1) .* (3/4*(1-0.83^2).*Con_RelVel_Stripes.(T){i2,1}(:,2)) .* Con_STRIPES.(T){i2,4}(:,2);
%             else
                % SIMPACK / UM
%                 ConStiff_Ref = 5e8;
% %                 ConDamp_Ref = 3e4;
%                 ConDamp_Ref = 1e4;          % 为减小KV阻尼的影响，尝试消除 zgyg 上的残余接触点
%                 ConDamp_Coff = ConDamp_Ref * (Con_STRIPES.(T){i2,4}(:,1)./ConStiff_Ref).^(1/2);
%                 NF_Damp_Stripes_temp{i2,1} = ConDamp_Coff.*Con_RelVel_Stripes.(T){i2,1}(:,1);
%             end
            %%%%%%
            Normal_Force.(T)(i2,6) = sum(NF_Damp_Stripes_temp{i2,1})*Win;
%             Normal_Force.(T)(i2,6) = sum(NF_Damp_Stripes_temp{i2,1});
        end
    elseif strcmp(InpPar.Type_Normal, 'Hertz&ConDamp')
        Normal_Force.(T)(bools.(T),6) = (1./elastic_permeability_Unit.b.(T)(bools.(T),1)).^(3/2) .* (3/4*(1-0.83^2).*Con_RelVel.(T)(bools.(T),2)) .* Con_wheel_2_II.b.(T)(bools.(T),5).^(3/2);
    elseif strcmp(InpPar.Type_Normal, 'SIMHertz&ConDamp')
        GG.(T) = 3.86e-8.*(R_yy_w.b.(T)).^(-0.115);
        Normal_Force.(T)(bools.(T),6) = (1./GG.(T)(bools.(T),1)).^(3/2) .* (3/4*(1-0.83^2).*Con_RelVel.(T)(bools.(T),2)) .* Con_wheel_2_II.b.(T)(bools.(T),5).^(3/2);
    else
        % d.Non ConDamping
        Normal_Force.(T)(:,6) = zeros(size(Normal_Force.(T),1),1);
    end    
    Normal_Force.(T)(:,1) = Normal_Force.(T)(:,5) + Normal_Force.(T)(:,6);
    bools_NF.(T) = Normal_Force.(T)(:,1)<=0;
    Normal_Force.(T)(bools_NF.(T),1) = 1e-3;
end

% 6.5~6.8 RHLv, RHXS, Prhxf_T, Pjc, Pjcc, Pjch, Prhxf =======================
for i1=1:1:length(InpPar.Type_Side)
    T = InpPar.Type_Side{i1};
    
    % 6.5 接触斑坐标系下蠕滑率 =======================
    RHLv.(T)(:,1:3) = [Vsdc.(T)(:,1)./Vgd.(T)(:,1), Vsdc.(T)(:,2)./Vgd.(T)(:,1), Vjsdc.(T)(:,3)./Vgd.(T)(:,1)];
    
    % 6.6 蠕滑系数 =======================
    a2.(T) = zeros(size(Normal_Force.(T),1),1);
    b2.(T) = zeros(size(Normal_Force.(T),1),1);
    for i2 = 1:1:size(Normal_Force.(T),1)
        if rou.b.(T)(i2)/R_yy_w.b.(T)(i2) <= 2  % m>=n，a>=b，b/a<=1
            a2.(T)(i2) = 0.1506e-3 * m.b.(T)(i2) * (rou.b.(T)(i2)*Normal_Force.(T)(i2,1))^(1/3);
            b2.(T)(i2) = 0.1506e-3 * n.b.(T)(i2) * (rou.b.(T)(i2)*Normal_Force.(T)(i2,1))^(1/3);
            gg = b2.(T)(i2)/a2.(T)(i2);
            gg_A = log(16./(gg.^2));
            BGC2_11 = 2*pi./(gg_A-2*Vr)./gg.*(1+(3-log(4))./(gg_A-2*Vr));
            BGC2_22 = 2*pi.*(1+(1-Vr).*(3-log(4))./((1-Vr).*gg_A+2*Vr))./((1-Vr).*gg_A+2*Vr)./gg;
            BGC2_23 = 2*pi./(3.*gg.*sqrt(gg))./((1-Vr).*gg_A-2+4*Vr);
            BGC2_33 = pi/4*(1-(Vr*gg_A-2)/((1-Vr)*gg_A-2+4*Vr));
            BGC2_v2 = [0 BGC2_11 BGC2_22 BGC2_23 BGC2_33; BGC2];
            Czc11.(T)(i2,1) = interp1(BGC2_v2(:,1),BGC2_v2(:,2),gg,'linear');
            Czc22.(T)(i2,1) = interp1(BGC2_v2(:,1),BGC2_v2(:,3),gg,'linear');
            Czc23.(T)(i2,1) = interp1(BGC2_v2(:,1),BGC2_v2(:,4),gg,'linear');
            Czc33.(T)(i2,1) = interp1(BGC2_v2(:,1),BGC2_v2(:,5),gg,'linear');
        else                          % m>=n，b>=a，a/b<=1
            a2.(T)(i2) = 0.1506e-3 * n.b.(T)(i2) * (rou.b.(T)(i2)*Normal_Force.(T)(i2,1))^(1/3);
            b2.(T)(i2) = 0.1506e-3 * m.b.(T)(i2) * (rou.b.(T)(i2)*Normal_Force.(T)(i2,1))^(1/3);
            gg = a2.(T)(i2)/b2.(T)(i2);
            BGC1_11 = pi^2/4/(1-Vr);
            BGC1_22 = pi^2/4;
            BGC1_23 = pi*sqrt(gg)/3/(1-Vr) * (1+Vr*(0.5*log(16/(gg^2))+log(4)-5));
            BGC1_33 = pi^2/16/(1-Vr)/gg;
            BGC1_v2 = [0 BGC1_11 BGC1_22 BGC1_23 BGC1_33; BGC1];
            Czc11.(T)(i2,1) = interp1(BGC1_v2(:,1),BGC1_v2(:,2),gg,'linear');
            Czc22.(T)(i2,1) = interp1(BGC1_v2(:,1),BGC1_v2(:,3),gg,'linear');
            Czc23.(T)(i2,1) = interp1(BGC1_v2(:,1),BGC1_v2(:,4),gg,'linear');
            Czc33.(T)(i2,1) = interp1(BGC1_v2(:,1),BGC1_v2(:,5),gg,'linear');
        end
    end
    
    % RHXS 左侧不同接触斑处（行）四个（列）蠕滑系数 = Gwr*(ab)^n*C = E/(2(1+v))*(ab)^n*C
    RHXS.(T)(:,1) = Er/(2*(1+Vr))*(a2.(T).*b2.(T)).*Czc11.(T);
    RHXS.(T)(:,2) = Er/(2*(1+Vr))*(a2.(T).*b2.(T)).*Czc22.(T);
    RHXS.(T)(:,3) = Er/(2*(1+Vr))*((a2.(T).*b2.(T)).^(3/2)).*Czc23.(T);
    RHXS.(T)(:,4) = Er/(2*(1+Vr))*((a2.(T).*b2.(T)).^2).*Czc33.(T);

    % 6.7a 线性蠕滑理论计算蠕滑力，开展非线性蠕滑力修正 =======================
    Prh.(T)(:,1) = -RHXS.(T)(:,1).*RHLv.(T)(:,1);
    Prh.(T)(:,2) = -RHXS.(T)(:,2).*RHLv.(T)(:,2) - RHXS.(T)(:,3).*RHLv.(T)(:,3);
    Prh.(T)(:,3) =  RHXS.(T)(:,3).*RHLv.(T)(:,2) - RHXS.(T)(:,4).*RHLv.(T)(:,3);
    
%     Pz.(T) = (Prh.(T)(:,1).^2 + Prh.(T)(:,2).^2).^(1/2);
%     for i2 = 1:1:size(Normal_Force.(T),1)
%         if Pz.(T)(i2,1) <= 3*fr*Normal_Force.(T)(i2,1)
%             temp = Pz.(T)(i2)/(fr*Normal_Force.(T)(i2,1));
%             Pzz.(T)(i2,1) = fr*Normal_Force.(T)(i2,1) * (temp-1/3*temp^2+1/27*temp^3);
%         else
%             Pzz.(T)(i2,1) = fr*Normal_Force.(T)(i2,1);
%         end
%         epxl.(T)(i2,1) = Pzz.(T)(i2,1)/Pz.(T)(i2,1);
%         Prhx.(T)(i2,:) = Prh.(T)(i2,:)*epxl.(T)(i2,1);
%     end

    % 6.7b FASTSIM计算蠕滑力 =======================
    len = size(Normal_Force.(T),1);
    Prhx_T.(T) = zeros(len,3);
    Prhxf_T.(T) = zeros(len,6);
    for i2 = 1:1:size(Normal_Force.(T),1)
        [Fx, Fy, MU] = FASTSIM(Normal_Force.(T)(i2,1), fr, a2.(T)(i2), b2.(T)(i2), RHLv.(T)(i2,1), RHLv.(T)(i2,2), RHLv.(T)(i2,3), Czc11.(T)(i2,1), Czc22.(T)(i2,1), Czc23.(T)(i2,1));
        Prhx_T.(T)(i2,:) = [Fx, Fy, MU];
    end
    
    % 6.8 蠕滑力及力矩各方向分量，将法向力蠕滑力从接触坐标系-->轨道坐标系
    % N*6，不同接触斑处蠕滑力在X、Y和Z三个方向的分量、蠕滑力矩在X、Y和Z三个方向的分量
    % Prh = zeros(InpPar.N_ConPatch*InpPar.Nw*4,1);     % 左右车轮编号及三项轮轨间蠕滑力（行）  N
    % Prhx = zeros(InpPar.N_ConPatch*InpPar.Nw*4,2);    % 左右车轮编号及三项轮轨间蠕滑力（行）修正值，列代表两个相邻计算次数  N
    % Prhxf = zeros(InpPar.N_ConPatch*InpPar.Nw*4,2);   % 左右车轮编号及X、Y、Z三方向轮轨间蠕滑力（行）修正值分量，列代表蠕滑力和蠕滑力矩
    % Normal_Force_L = zeros(N_Patch,6)   % 接触点数*6：法向力标量，法向力横向分矢量，法向力垂向分矢量，接触斑编号, 分别由接触刚度和阻尼引起的法向接触力    
    for i2 = 1:1:size(Normal_Force.(T),1)
        Prhxf_T.(T)(i2,1:3) = [Prhx_T.(T)(i2,1:2) 0] * B_track.(T){i2,1};
        Prhxf_T.(T)(i2,4:6) = [0 0 Prhx_T.(T)(i2,3)] * B_track.(T){i2,1};
%         T_trans.(T) = double(subs(T_Con_Track, [Inp_Pos_Yaw, Inp_Pos_Roll, Inp_Ang], [0, Roll_DW.(T), Con_Ang.(T)(i2)]));
        T_trans.(T) = [1    0                                                       0
                              0   cos(Con_Ang.(T)(i2)+Roll_DW.(T))    sin(Con_Ang.(T)(i2)+Roll_DW.(T))
                              0   -sin(Con_Ang.(T)(i2)+Roll_DW.(T))    cos(Con_Ang.(T)(i2)+Roll_DW.(T))];
        temp_Normal = [0,0,-Normal_Force.(T)(i2,1)]*T_trans.(T);
        Normal_Force.(T)(i2,2:3) = temp_Normal(2:3);
        
        i_contact = InpPar.N_ConPatch*(i11-1)+Normal_Force.(T)(i2,4);
        Pjc(i_contact,2)  = Pjc(i_contact,2) + Normal_Force.(T)(i2,1);
        Pjch(i_contact,1) = Pjch(i_contact,1) + Normal_Force.(T)(i2,2);
        Pjcc(i_contact,1) = Pjcc(i_contact,1) + Normal_Force.(T)(i2,3);
        Prhx(i_contact,:) = Prhx(i_contact,:) + Prhx_T.(T)(i2,:);
        Prhxf(i_contact,:) = Prhxf(i_contact,:) + Prhxf_T.(T)(i2,:);
    end

end


%% 7.创建结构体
WR_Geometry = [Roll_DW.R, Yaw_DW.R, Yw_DW.R, Zw_DW.R, Roll_DW.L, Yaw_DW.L, Yw_DW.L, Zw_DW.L];
Con_str = struct('WR_Geometry', WR_Geometry,...
                           'pos_WS_track', pos_WS_track,...
                           'vel_WS_track', vel_WS_track,...
                           'pos_WS_global', pos_WS_global,...
                           'vel_WS_global', vel_WS_global,...
                           'profile_w', profile_w,...
                           'profile_r', Profile_TrackCS.profile_r,...
                           'traceline_w', traceline_w,...
                           'Con_wheel_1_I', Con_wheel_1_I,...
                           'Con_wheel_1', Con_wheel_1_II.b,...
                           'Con_wheel_2', Con_wheel_2_II.b,...
                           'Con_rail_1', Con_rail_1_II.b,...
                           'Con_rail_2', Con_rail_2_II,...
                           'Con_RelVel', Con_RelVel,...
                           'Con_RelVel_max', Con_RelVel_max,...
                           'Normal_Force', Normal_Force,...
                           'Prh', Prh,...
                           'Prhx_T', Prhx_T,...
                           'Prhxf_T', Prhxf_T,...
                           'RHXS', RHXS,...
                           'RHLv', RHLv,...
                           'a2', a2,...
                           'b2', b2,...
                           'Vjd',Vjd,...
                           'Vjd_r',Vjd_r,...
                           'Vsdc',Vsdc,...
                           'Vjsdc',Vjsdc,...
                           'Vgd',Vgd,...
                           'Ver_Pen_postive_start', Ver_Pen_postive_start,...
                           'Ver_Pen_postive_end', Ver_Pen_postive_end,...
                           'Area_STRIPES', Area_STRIPES,...
                           'Vzdli',Vzdli,...
                           'Vzdlj',Vzdlj,...
                           'Vzdlk',Vzdlk,...
                           'ConPosInt',ConPosInt,...
                           'Mileage', Mileage);

if strcmp(InpPar.Type_Normal, 'STRIPES&ConDamp')
    Con_str.Con_STRIPES = Con_STRIPES;
    Con_str.Epsilon = Epsilon;
    T0 = 'a';
else
    T0 = 'b';
end
Con_str.elastic_permeability_Unit = elastic_permeability_Unit.(T0);
Con_str.R_yy_w = R_yy_w.(T0);
Con_str.R_xx_w = R_xx_w.(T0);
Con_str.R_xx_r = R_xx_r.(T0);
Con_str.rou = rou.(T0);

% Profile_TrackCS, Con_str
for i2 = 1:1:InpPar.N_ConPatch    
    Con_str.profile_r.(InpPar.Exp_DummyRail{i2}) = Profile_TrackCS.profile_r.(InpPar.Exp_DummyRail{i2});
    Con_str.profile_r_dY.(InpPar.Exp_DummyRail{i2}) = Profile_TrackCS.profile_r_dY.(InpPar.Exp_DummyRail{i2});
    Con_str.profile_r_dZ.(InpPar.Exp_DummyRail{i2}) = Profile_TrackCS.profile_r_dZ.(InpPar.Exp_DummyRail{i2});
end

Con_str.Dis_TIrr = Dis_TIrr_temp;
Con_str.Vel_TIrr = Vel_TIrr_temp;

if InpPar.NM_FW > 0
    % Pos_FW, DOF_pos_FW, Vel_FW_global, ShapeFun_FW, Vjd
    Con_str.DOF_pos_FW = DOF_pos_FW;
    Con_str.ShapeFun_FW = ShapeFun_FW;
    Con_str.Pos_FW_global.L = Pos_FW.Tread.L.global.Node_Around_XOY;
    Con_str.Pos_FW_global.R = Pos_FW.Tread.R.global.Node_Around_XOY;
    Con_str.Vel_FW_global.L = Vel_FW_global.L.Node_Around_XOY;    
    Con_str.Vel_FW_global.R = Vel_FW_global.R.Node_Around_XOY;
    Con_str.Con_wheel_2_RW.L = Con_wheel_2_II.b.L_RW;
    Con_str.Con_wheel_2_RW.R = Con_wheel_2_II.b.R_RW;
    Con_str.Con_wheel_3_RW.L = Con_wheel_3_II.b.L_RW;
    Con_str.Con_wheel_3_RW.R = Con_wheel_3_II.b.R_RW;
else
    Con_str.Con_wheel_2_RW = Con_wheel_2_II.b;
    Con_str.Con_wheel_3_RW = Con_wheel_3_II.b;
end


%% Draft
% % Sliding smooth
% span = 10;
% Vel_ConPos_w_2II = [smooth(temp(:,1),span), smooth(temp(:,2),span), smooth(temp(:,3),span)];
% % Check
% if Choose_Plot==1
%     figure(115);
%     clf
%     x = ZP_Con.Wheel_2.(T2){i11,num_Adv}(t1:xlcs,1);
%     plot(x, Vel_ConPos_w_2I(:,2)); hold on
%     plot(x, Vel_ConPos_w_2II(:,2), '--'); hold on
%     grid on
% end

