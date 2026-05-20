function [ElaPen_Unit, ConPos_Wheel_WSCS, ConRadius_Wheel_WSCS, ConRadius_Rail_TrackCS, ConAng, ConPar] = ...
                Cal_ElePen_Unit(WheelPro_ProCS, wheel_interp, profile_w_Radius_WSCS, profile_r_Radius_TrackCS, ConPos_Rail_TrackCS_Y, Zw_DW, Yw_DW, A_DW, BGmn, Vr, Er)

% Test
% Zw_DW = Zw_DWL;
% Yw_DW = Yw_DWL;
% A_DW = A_DWL;
% wheel_interp = wheel_interp_L;
% profile_w_Radius_WSCS = WheelPro_ProCS.profile_w_L_Radius;
% profile_r_Radius_TrackCS = Profile_TrackCS.profile_r_L_Radius;
% ConPos_Rail_TrackCS_Y = rail_interp_L(p,1);
% BGmn = WheelPro_ProCS.BGmn;

ConRadius_Rail_TrackCS = [ConPos_Rail_TrackCS_Y, interp1(profile_r_Radius_TrackCS(:,1), profile_r_Radius_TrackCS(:,2), ConPos_Rail_TrackCS_Y, 'linear')];
[~, p1] = min(abs(wheel_interp(:,2)-ConPos_Rail_TrackCS_Y));
ConPos_Wheel_WSCS = (wheel_interp(p1,:)-[0,Yw_DW,Zw_DW])*inv(A_DW);
ConRadius_Wheel_WSCS(1) = ConPos_Wheel_WSCS(2);
ConRadius_Wheel_WSCS(2) = interp1(profile_w_Radius_WSCS(:,1), profile_w_Radius_WSCS(:,2), ConRadius_Wheel_WSCS(1), 'linear');

if ConPos_Wheel_WSCS(2)<0
    xx_yy = WheelPro_ProCS.Con_ang_L;
else
    xx_yy = WheelPro_ProCS.Con_ang_R;
end
ConAng = [ConPos_Wheel_WSCS(2), interp1(xx_yy(:,1), xx_yy(:,2), ConPos_Wheel_WSCS(2), 'linear')];

% ElaPen_Unit = [];
R_yy_w = ConPos_Wheel_WSCS(:,3);
R_xx_w = ConRadius_Wheel_WSCS(:,2);
R_xx_r = ConRadius_Rail_TrackCS(:,2);
if R_xx_w<0 && (abs(R_xx_w)<abs(R_xx_r))
    R_xx_r = abs(R_xx_w)*0.95;
end

%%% 测试用-230218，为保证 Roll 迭代收敛
if R_xx_r < 4e-3
    R_xx_r = 4e-3;
end

rou = 4./(1./R_yy_w+1./R_xx_w+1./R_xx_r);
beta = acos(rou./4.*abs(1./R_yy_w-1./R_xx_w-1./R_xx_r));
[a1, b1, m, n, ElaPen_Unit, Con_A, Con_B, Con_r] = Par_Hertz(R_yy_w, R_xx_w, R_xx_r, rou, BGmn, Vr, Er, 'Hertz');

ConPar.A = Con_A;
ConPar.B = Con_B;
ConPar.r = Con_r;
ConPar.Lambda = Con_A./Con_B;
ConPar.a1 = a1;
ConPar.b1 = b1;
ConPar.m = m;
ConPar.n = n;
ConPar.R_yy_w = R_yy_w;
ConPar.R_xx_w = R_xx_w;
ConPar.R_xx_r = R_xx_r;
ConPar.rou = rou;
ConPar.beta = beta;



% [Con_a1, Con_b1, Con_m, Con_n, ElaPen_Unit] = Par_Hertz_ConStiff(R_yy_w, R_xx_w, R_xx_r, rou, BGmn, Vr, Er);
% A = 1/2 * 1./R_yy_w;
% B = 1/2 * (1./R_xx_w+1./R_xx_r);
% Con_Lambda = A./B;

% ConPar.Lambda = Con_Lambda;
% ConPar.b1 = Con_b1;
% ConPar.m = Con_m;
% ConPar.n = Con_n;
% ConPar.R_yy_w = R_yy_w;
% ConPar.R_xx_w = R_xx_w;
% ConPar.R_xx_r = R_xx_r;
% ConPar.rou = rou;
% ConPar.beta = beta;

