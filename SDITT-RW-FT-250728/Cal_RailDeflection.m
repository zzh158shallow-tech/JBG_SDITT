function RailBeam = Cal_RailDeflection(HR_r)

global Pos_Node Vlc Type_Rail

clear RailBeam
%% RailBeam
for i1 = 1:1:length(Type_Rail)
    if i1==1
        kk = 1;
    else
        kk = -1;
    end
    Target = Pos_Node.(Type_Rail{i1});
    RailBeam.Pos_Y.(Type_Rail{i1}) = [Target(:,2), Target(:,3)+0.753*kk];
    RailBeam.Vel_Y.(Type_Rail{i1}) = [Target(2:end,2), diff(RailBeam.Pos_Y.(Type_Rail{i1})(:,2))./diff(Target(:,2))*Vlc];
    RailBeam.Pos_Yaw.(Type_Rail{i1}) = [Target(2:end,2), diff(Target(:,3))./diff(Target(:,2))];
    RailBeam.Vel_Yaw.(Type_Rail{i1}) = [Target(3:end,2), diff(RailBeam.Pos_Yaw.(Type_Rail{i1})(:,2))./diff(Target(2:end,2))*Vlc];
    
    xx = Pos_Node.(Type_Rail{i1})(1,2):0.1:Pos_Node.(Type_Rail{i1})(end,2);
    if isfield(HR_r, Type_Rail{i1})
        xx_yy = [xx', interp1(HR_r.(Type_Rail{i1})(:,1)+50, HR_r.(Type_Rail{i1})(:,2), xx', 'linear')];
        RailBeam.Pos_Z.(Type_Rail{i1}) = xx_yy;
        RailBeam.Vel_Z.(Type_Rail{i1}) = [xx_yy(2:end,1), diff(xx_yy(:,2))./diff(xx_yy(:,1))*Vlc];
        RailBeam.Pos_Pitch.(Type_Rail{i1}) = [xx_yy(2:end,1), diff(xx_yy(:,2))./diff(xx_yy(:,1))];
        RailBeam.Vel_Pitch.(Type_Rail{i1}) = [xx_yy(3:end,1), diff(RailBeam.Pos_Pitch.(Type_Rail{i1})(:,2))./diff(xx_yy(2:end,1))*Vlc];
    else
        RailBeam.Pos_Z.(Type_Rail{i1}) = [xx', zeros(length(xx),1)];
        RailBeam.Vel_Z.(Type_Rail{i1}) = [xx(2:end)', zeros(length(xx)-1,1)];
        RailBeam.Pos_Pitch.(Type_Rail{i1}) = [xx(2:end)', zeros(length(xx)-1,1)];
        RailBeam.Vel_Pitch.(Type_Rail{i1}) = [xx(3:end)', zeros(length(xx)-2,1)];
    end
end

%% Transition Curve
syms Alg_Vlc dt L_0 k1 k2 kc y_TransEnd yaw_TransEnd
Type_Layout = {'trans1', 'trans2', 'curve'};

%%% 缓和曲线-1a（以缓和曲线1起点为原点）
fun_k.trans1(Alg_Vlc, dt, L_0, k1, k2) = k1 + 2*(Alg_Vlc*dt)^2/L_0^2*(k2-2*k1);
fun_y.trans1(Alg_Vlc, dt, L_0, k1, k2) = 1/2*k1*(Alg_Vlc*dt)^2 + (Alg_Vlc*dt)^4*(k2-2*k1)/(6*L_0^2);
fun_yaw.trans1(Alg_Vlc, dt, L_0, k1, k2) = k1*(Alg_Vlc*dt) + 2*(Alg_Vlc*dt)^3/(3*L_0^2)*(k2-2*k1);
fun_y_d1.trans1(Alg_Vlc, dt, L_0, k1, k2) = diff(fun_y.trans1, dt);
fun_yaw_d1.trans1(Alg_Vlc, dt, L_0, k1, k2) = diff(fun_yaw.trans1, dt);

%%% 缓和曲线-1b（以缓和曲线1起点为原点）
fun_k.trans2(Alg_Vlc, dt, L_0, k1, k2) = k1 + 2/L_0^2*(2*L_0*(Alg_Vlc*dt)-(Alg_Vlc*dt)^2-L_0^2/2)*(k2-2*k1);
fun_y.trans2(Alg_Vlc, dt, L_0, k1, k2) = 1/2*k1*(Alg_Vlc*dt)^2 + 2/(L_0^2)*(1/3*L_0*(Alg_Vlc*dt)^3-1/12*(Alg_Vlc*dt)^4-L_0^2/4*(Alg_Vlc*dt)^2)*(k2-2*k1) + ...
                                                                       L_0/6*(Alg_Vlc*dt)*(k2-2*k1) - 1/48*(L_0^2)*(k2-2*k1);
fun_yaw.trans2(Alg_Vlc, dt, L_0, k1, k2) = k1*(Alg_Vlc*dt)+2/(L_0^2)*(L_0*(Alg_Vlc*dt)^2-1/3*(Alg_Vlc*dt)^3-(L_0^2)/2*(Alg_Vlc*dt))*(k2-2*k1)+L_0/6*(k2-2*k1);
fun_y_d1.trans2(Alg_Vlc, dt, L_0, k1, k2) = diff(fun_y.trans2, dt);
fun_yaw_d1.trans2(Alg_Vlc, dt, L_0, k1, k2) = diff(fun_yaw.trans2, dt);

%%% 圆曲线 (Alg_Vlc*dt 为距离圆曲线起点的距离)
fun_k.curve(kc) = kc;
fun_yaw.curve(Alg_Vlc, dt, kc, yaw_TransEnd) = (Alg_Vlc*dt)*kc + yaw_TransEnd;
fun_y.curve(Alg_Vlc, dt, kc, yaw_TransEnd, y_TransEnd) = (1-cos(fun_yaw.curve))/kc - (1-cos(yaw_TransEnd))/kc + y_TransEnd;
fun_yaw_d1.curve(Alg_Vlc, dt, kc, yaw_TransEnd) = diff(fun_yaw.curve, dt);
fun_y_d1.curve(Alg_Vlc, dt, kc, yaw_TransEnd, y_TransEnd) = diff(fun_y.curve, dt);


%% 计算缓和曲线终点曲率
% Par_RailDef
SIP = [50, 50+53.148];
Par_RailDef.L_trans.qjbg = 5.161;
Par_RailDef.L_trans.zjg_zgyg = 1.912;
Par_RailDef.W_tangency.qjbg = 26.8e-3;
Par_RailDef.W_tangency.zjg_zgyg = 71.3e-3;
Par_RailDef.R1_trans.qjbg = inf;
Par_RailDef.R1_trans.zjg_zgyg = inf;
Par_RailDef.R_curve.qjbg = 1100-1.435/2;

Type_Rail_Target = {'qjbg', 'zjg_zgyg'};
for i1 = 1:1:length(Type_Rail_Target)
    T1 = (Type_Rail_Target{i1});
    Eq_1(k2) = subs(fun_y.trans2, [Alg_Vlc, dt, L_0, k1], [Vlc, Par_RailDef.L_trans.(T1)/Vlc, Par_RailDef.L_trans.(T1), 1/Par_RailDef.R1_trans.(T1)]) ...
                    - Par_RailDef.W_tangency.((T1));
    Par_RailDef.R2_trans.(T1) =1/double(solve(Eq_1(k2)));
    Par_RailDef.yaw_TransEnd.(T1) = Par_RailDef.W_tangency.(T1) / Par_RailDef.L_trans.(T1);
    Par_RailDef.y_TransEnd.(T1) = double( subs(fun_y.trans2, [Alg_Vlc, dt, L_0, k1, k2], ...
                                                     [Vlc, Par_RailDef.L_trans.(T1)/Vlc, Par_RailDef.L_trans.(T1), 1/Par_RailDef.R1_trans.(T1), 1/Par_RailDef.R2_trans.(T1)]) );
end

% qjbg, bools
clear xx
xx.qjbg = [-2:0.2:-0.2, 0:0.02:6, 6.1:0.1:20]+SIP(1);
xx.zjg_zgyg = [-2:0.2:-0.2, 0:0.01:Par_RailDef.L_trans.zjg_zgyg]+SIP(2);

Type_RailDef = {'Pos_Y', 'Vel_Y', 'Pos_Yaw', 'Vel_Yaw'};
% Pos_Y, Vel_Y, Pos_Yaw, Vel_Yaw
clear RailBeam_Cal
for i1 = 1:1:length(Type_Rail_Target)
    T1 = (Type_Rail_Target{i1});
    for i2 = 1:1:length(xx.(T1))
        for kk = 1:1:length(Type_RailDef)
            RailBeam_Cal.(Type_RailDef{kk}).(T1)(i2,1) = xx.(T1)(i2);
        end
        if xx.(T1)(i2)<SIP(i1)
            kk = 0;
        elseif xx.(T1)(i2)>=SIP(i1) && xx.(T1)(i2)<Par_RailDef.L_trans.(T1)/2+SIP(i1)
            kk = 1;
        elseif xx.(T1)(i2)>=Par_RailDef.L_trans.(T1)/2+SIP(i1) && xx.(T1)(i2)<Par_RailDef.L_trans.(T1)+SIP(i1)
            kk = 2;
        elseif xx.(T1)(i2)>=Par_RailDef.L_trans.(T1)+SIP(i1)
            kk = 3;
        end
        if kk==0
            for kk = 1:1:length(Type_RailDef)
                RailBeam_Cal.(Type_RailDef{kk}).(T1)(i2,2) = 0;
            end
        elseif kk<3
            dx = xx.(T1)(i2)-SIP(i1);
            RailBeam_Cal.Pos_Y.(T1)(i2,2) = double( subs(fun_y.(Type_Layout{kk}), [Alg_Vlc, dt, L_0, k1, k2], [Vlc, dx/Vlc, Par_RailDef.L_trans.(T1), 1/Par_RailDef.R1_trans.(T1), 1/Par_RailDef.R2_trans.(T1)]) );
            RailBeam_Cal.Vel_Y.(T1)(i2,2) = double( subs(fun_y_d1.(Type_Layout{kk}), [Alg_Vlc, dt, L_0, k1, k2], [Vlc, dx/Vlc, Par_RailDef.L_trans.(T1), 1/Par_RailDef.R1_trans.(T1), 1/Par_RailDef.R2_trans.(T1)]) );
            RailBeam_Cal.Pos_Yaw.(T1)(i2,2) = double( subs(fun_yaw.(Type_Layout{kk}), [Alg_Vlc, dt, L_0, k1, k2], [Vlc, dx/Vlc, Par_RailDef.L_trans.(T1), 1/Par_RailDef.R1_trans.(T1), 1/Par_RailDef.R2_trans.(T1)]) );
            RailBeam_Cal.Vel_Yaw.(T1)(i2,2) = double( subs(fun_yaw_d1.(Type_Layout{kk}), [Alg_Vlc, dt, L_0, k1, k2], [Vlc, dx/Vlc, Par_RailDef.L_trans.(T1), 1/Par_RailDef.R1_trans.(T1), 1/Par_RailDef.R2_trans.(T1)]) );
        else
            dx =xx.(T1)(i2)-SIP(i1)-Par_RailDef.L_trans.(T1);
            RailBeam_Cal.Pos_Y.(T1)(i2,2) = double( subs(fun_y.(Type_Layout{kk}), [Alg_Vlc, dt, kc, yaw_TransEnd, y_TransEnd], [Vlc, dx/Vlc, 1/Par_RailDef.R_curve.(T1), Par_RailDef.yaw_TransEnd.(T1), Par_RailDef.y_TransEnd.(T1)]) );
            RailBeam_Cal.Vel_Y.(T1)(i2,2) = double( subs(fun_y_d1.(Type_Layout{kk}), [Alg_Vlc, dt, kc, yaw_TransEnd, y_TransEnd], [Vlc, dx/Vlc, 1/Par_RailDef.R_curve.(T1), Par_RailDef.yaw_TransEnd.(T1), Par_RailDef.y_TransEnd.(T1)]) );
            RailBeam_Cal.Pos_Yaw.(T1)(i2,2) = double( subs(fun_yaw.(Type_Layout{kk}), [Alg_Vlc, dt, kc, yaw_TransEnd], [Vlc, dx/Vlc, 1/Par_RailDef.R_curve.(T1), Par_RailDef.yaw_TransEnd.(T1)]) );
            RailBeam_Cal.Vel_Yaw.(T1)(i2,2) = double( subs(fun_yaw_d1.(Type_Layout{kk}), [Alg_Vlc, dt, kc, yaw_TransEnd], [Vlc, dx/Vlc, 1/Par_RailDef.R_curve.(T1), Par_RailDef.yaw_TransEnd.(T1)]) );
        end
    end
end

% Replace, RailBeam_Cal, RailBeam
for i1 = 1:1:length(Type_Rail_Target)
    T1 = (Type_Rail_Target{i1});
    for i2 = 1:1:length(Type_RailDef)
        bools_R = RailBeam.(Type_RailDef{i2}).(T1)(:,1)>=min(xx.(T1)) & RailBeam.(Type_RailDef{i2}).(T1)(:,1)<=max(xx.(T1));
        RailBeam.(Type_RailDef{i2}).(T1)(bools_R,:) = [];
        RailBeam.(Type_RailDef{i2}).(T1) = sortrows( [RailBeam.(Type_RailDef{i2}).(T1); RailBeam_Cal.(Type_RailDef{i2}).(T1)], 1 );
    end
end


%% Plot, RailBeam
Choose_Plot = 0;
if Choose_Plot==1
    figure(40); clf
    Target_1 = RailBeam.Pos_Y;         Target_2 = RailBeam.Vel_Y;
    Target_RelVel_Z = RailBeam.Pos_Yaw;    Target_4 = RailBeam.Vel_Yaw;
%     Target_1 = RailBeam.Pos_Z;           Target_2 = RailBeam.Vel_Z;
%     Target_3 = RailBeam.Pos_Pitch;    Target_4 = RailBeam.Vel_Pitch;
    for i1 = 1:1:length(Type_Rail)
        T1 = Type_Rail{i1};
        subplot(2,2,1)
        plot(Target_1.(T1)(:,1)-50, Target_1.(T1)(:,2)); hold on; ylabel('Pos Y / Z (m)')
        subplot(2,2,2)
        plot(Target_2.(T1)(:,1)-50, Target_2.(T1)(:,2)); hold on; ylabel('Vel Y / Z (m/s)')
        subplot(2,2,3)
        plot(Target_RelVel_Z.(T1)(:,1)-50, Target_RelVel_Z.(T1)(:,2)); hold on; ylabel('Pos Yaw / Pitch (m)')
        subplot(2,2,4)
        plot(Target_4.(T1)(:,1)-50, Target_4.(T1)(:,2)); hold on; ylabel('Vel Yaw / Pitch (m/s)')
    end
    for kk = 1:1:4
        subplot(2,2, kk)
        if kk==1
            legend(Type_Rail, 'Interpreter', 'none', 'Position',[0.2649 0.9433 0.4643 0.0440], 'Orientation','horizontal', 'FontName', 'Times');
        end
        set(gca, 'FontName', 'Times');  grid on
        xlim([-5, 65]);
%         xlim([-5, 25]);
    end
end
