%% 迹线法获取车轮空间迹线

function [Con_ang_R_temp, traceline_w_R, Con_ang_L_temp, traceline_w_L] = TracePrinciple(WheelPro_ProCS, Dlb, discrete_len_flange, discrete_len_tread,...
          profile_w_R, Roll_DWR, Yaw_DWR, Yw_DWR, Zw_DWR, profile_w_L, Roll_DWL, Yaw_DWL, Yw_DWL, Zw_DWL)

Con_ang_R = WheelPro_ProCS.Con_ang_R;
Con_ang_L = WheelPro_ProCS.Con_ang_L;
      
%%% Right
bools_flange_R = profile_w_R(:,1) <= Dlb + 38e-3;
xx_R = [min(profile_w_R(bools_flange_R,1)) : discrete_len_flange : max(profile_w_R(bools_flange_R,1)),...
        min(profile_w_R(~bools_flange_R,1)): discrete_len_tread : max(profile_w_R(~bools_flange_R,1))]';
xx_R = sortrows(xx_R,1);
% xx_R = (profile_w_R(1,1):discrete_len_1:profile_w_R(end,1))';
Rw_R = interp1(profile_w_R(:,1),profile_w_R(:,2),xx_R,'spline');
Con_ang_R_temp = interp1(Con_ang_R(:,1),Con_ang_R(:,2),xx_R,'spline');
lx_R = -cos(Roll_DWR)*sin(Yaw_DWR);
ly_R =  cos(Roll_DWR)*cos(Yaw_DWR);
lz_R =  sin(Roll_DWR);
m_R  =  sqrt(1-lx_R^2*(1.+(tan(Con_ang_R_temp)).^2));
xb_R = xx_R.*lx_R;
yb_R = xx_R.*ly_R+Yw_DWR;
zb_R = xx_R.*lz_R;
% zb_R = xx_R.*lz_R+Zw_DWR;
traceline_w_R(:,1) = xb_R + lx_R.*Rw_R.*tan(Con_ang_R_temp);
traceline_w_R(:,2) = yb_R - Rw_R.*(lx_R^2*ly_R.*tan(Con_ang_R_temp)+lz_R.*m_R)/(1-lx_R^2);
traceline_w_R(:,3) = zb_R - Rw_R.*(lx_R^2*lz_R.*tan(Con_ang_R_temp)-ly_R.*m_R)/(1-lx_R^2);

%%% Left
bools_flange_L = profile_w_L(:,1) >= -(Dlb + 38e-3);
xx_L = [min(profile_w_L(bools_flange_L,1)) : discrete_len_flange : max(profile_w_L(bools_flange_L,1)),...
        min(profile_w_L(~bools_flange_L,1)): discrete_len_tread : max(profile_w_L(~bools_flange_L,1))]';
xx_L = sortrows(xx_L,1);
% xx_L = (profile_w_L(1,1):discrete_len_1:profile_w_L(end,1))';
Rw_L = interp1(profile_w_L(:,1),profile_w_L(:,2),xx_L,'spline');
Con_ang_L_temp = interp1(Con_ang_L(:,1),Con_ang_L(:,2),xx_L,'spline');
lx_L = -cos(Roll_DWL)*sin(Yaw_DWL);
ly_L =  cos(Roll_DWL)*cos(Yaw_DWL);
lz_L =  sin(Roll_DWL);
m_L  =  sqrt(1-lx_L^2*(1.+(tan(Con_ang_L_temp)).^2));
xb_L = xx_L*lx_L;
yb_L = xx_L*ly_L+Yw_DWL;
zb_L = xx_L*lz_L;
% zb_L = xx_L*lz_L+Zw_DWL;
traceline_w_L(:,1) = xb_L + lx_L.*Rw_L.*tan(Con_ang_L_temp);
traceline_w_L(:,2) = yb_L - Rw_L.*(lx_L^2*ly_L.*tan(Con_ang_L_temp)+lz_L.*m_L)/(1-lx_L^2);
traceline_w_L(:,3) = zb_L - Rw_L.*(lx_L^2*lz_L.*tan(Con_ang_L_temp)-ly_L.*m_L)/(1-lx_L^2);
