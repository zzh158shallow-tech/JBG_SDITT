%% 210210: 计算缓和曲线、平滑段曲线的函数表达式

function [fun_k_trans1, fun_k_trans1_d1, fun_x_trans1, fun_x_trans1_d1, fun_x_trans1_d2, fun_y_trans1, fun_y_trans1_d1, fun_y_trans1_d2, fun_yaw_trans1, fun_yaw_trans1_d1, fun_yaw_trans1_d2, ...
          fun_k_trans2, fun_k_trans2_d1, fun_x_trans2, fun_x_trans2_d1, fun_x_trans2_d2, fun_y_trans2, fun_y_trans2_d1, fun_y_trans2_d2, fun_yaw_trans2, fun_yaw_trans2_d1, fun_yaw_trans2_d2, ...
          fun_k_curve,  fun_k_curve_d1,  fun_x_curve, fun_x_curve_d1, fun_x_curve_d2, fun_y_curve, fun_y_curve_d1, fun_y_curve_d2, fun_yaw_curve, fun_yaw_curve_d1, fun_yaw_curve_d2] = ...
         Cal_fun_smooth_trans_210212(R1, R2, L_trans, L_smooth, Vlc)

     k1 = 1/R1; k2 = 1/R2;
     
     %%% 缓和曲线-1
     syms fun_k_trans1(dt) fun_k_trans1_d1(dt) fun_x_trans1(dt) fun_x_trans1_d1(dt) fun_x_trans1_d2(dt) fun_y_trans1(dt) fun_y_trans1_d1(dt) fun_y_trans1_d2(dt) ...
          fun_yaw_trans1(dt) fun_yaw_trans1_d1(dt) fun_yaw_trans1_d2(dt)
     fun_k_trans1(dt) = k1 + 2*(Vlc*dt)^2/L_trans^2*(k2-2*k1);
     fun_k_trans1_d1(dt) = diff(fun_k_trans1,dt);
     
     fun_x_trans1(dt) = (Vlc*dt);
     fun_x_trans1_d1(dt) = diff(fun_x_trans1,dt);
     fun_x_trans1_d2(dt) = diff(fun_x_trans1_d1,dt);
     
     fun_y_trans1(dt) = 1/2*k1*(Vlc*dt)^2 + (Vlc*dt)^4*(k2-2*k1)/(6*L_trans^2);
     fun_y_trans1_d1(dt) = diff(fun_y_trans1,dt);
     fun_y_trans1_d2(dt) = diff(fun_y_trans1_d1,dt);
     
     fun_yaw_trans1(dt) = k1*(Vlc*dt) + 2*(Vlc*dt)^3/(3*L_trans^2)*(k2-2*k1);
     fun_yaw_trans1_d1(dt) = diff(fun_yaw_trans1,dt);
     fun_yaw_trans1_d2(dt) = diff(fun_yaw_trans1_d1,dt);
     
     %%% 缓和曲线-2
     syms fun_k_trans2(dt) fun_k_trans2_d1(dt) fun_x_trans2(dt) fun_x_trans2_d1(dt) fun_x_trans2_d2(dt) fun_y_trans2(dt) fun_y_trans2_d1(dt) fun_y_trans2_d2(dt) ...
          fun_yaw_trans2(dt) fun_yaw_trans2_d1(dt) fun_yaw_trans2_d2(dt)
     fun_k_trans2(dt) = k1 + 2/L_trans^2*(2*L_trans*(Vlc*dt)-(Vlc*dt)^2-L_trans^2/2)*(k2-2*k1);
     fun_k_trans2_d1(dt) = diff(fun_k_trans2,dt);
     
     fun_x_trans2(dt) = fun_x_trans1(dt);
     fun_x_trans2_d1(dt) = fun_x_trans1_d1(dt);
     fun_x_trans2_d2(dt) = fun_x_trans1_d2(dt);
     
     fun_y_trans2(dt) = 1/2*k1*(Vlc*dt)^2 + 2/(L_trans^2)*(1/3*L_trans*(Vlc*dt)^3-1/12*(Vlc*dt)^4-L_trans^2/4*(Vlc*dt)^2)*(k2-2*k1) + ...
                        L_trans/6*(Vlc*dt)*(k2-2*k1) - 1/48*(L_trans^2)*(k2-2*k1);
     fun_y_trans2_d1(dt) = diff(fun_y_trans2,dt);
     fun_y_trans2_d2(dt) = diff(fun_y_trans2_d1,dt);

     fun_yaw_trans2(dt) = k1*(Vlc*dt)+2/(L_trans^2)*(L_trans*(Vlc*dt)^2-1/3*(Vlc*dt)^3-(L_trans^2)/2*(Vlc*dt))*(k2-2*k1)+L_trans/6*(k2-2*k1);
     fun_yaw_trans2_d1(dt) = diff(fun_yaw_trans2,dt);
     fun_yaw_trans2_d2(dt) = diff(fun_yaw_trans2_d1,dt);     
    
     %%% 圆曲线
     syms fun_k_curve(dt) fun_k_curve_d1(dt) fun_x_curve(dt) fun_x_curve_d1(dt) fun_x_curve_d2(dt) fun_y_curve(dt) fun_y_curve_d1(dt) fun_y_curve_d2(dt) ...
          fun_yaw_curve(dt) fun_yaw_curve_d1(dt) fun_yaw_curve_d2(dt)
     fun_k_curve(dt) = 1/R2;
     fun_k_curve_d1(dt) = diff(fun_k_curve,dt);
     
     phi_transition = k1*L_trans + 1/2*L_trans*(k2-2*k1);
     pos_Body_global_y_transition = 1/2*k1*L_trans^2 + 7/48*L_trans^2*(k2-2*k1);
     
     fun_yaw_curve(dt) = ((Vlc*dt)-L_trans/2)/R2 + phi_transition;
     fun_yaw_curve_d1(dt) = diff(fun_yaw_curve,dt);
     fun_yaw_curve_d2(dt) = diff(fun_yaw_curve_d1,dt);
     
     fun_x_curve(dt) = R2*sin(((Vlc*dt)-L_trans/2)/R2 + phi_transition) - R2*sin(phi_transition) + L_trans/2;
     fun_x_curve_d1(dt) = diff(fun_x_curve,dt);
     fun_x_curve_d2(dt) = diff(fun_x_curve_d1,dt);
     
     fun_y_curve(dt) = R2*(1-cos(((Vlc*dt)-L_trans/2)/R2 + phi_transition)) - R2*(1-cos(phi_transition)) + pos_Body_global_y_transition;
     fun_y_curve_d1(dt) = diff(fun_y_curve,dt);
     fun_y_curve_d2(dt) = diff(fun_y_curve_d1,dt);
     
%      Track_fun = struct('fun_k_curve', fun_k_curve, ...
%                         'fun_k_curve_d1', fun_k_curve_d1, ...
%                         'fun_yaw_curve', fun_yaw_curve, ...
%                         'fun_yaw_curve_d1', fun_yaw_curve_d1, ...
%                         'fun_yaw_curve_d2', fun_yaw_curve_d2, ...
%                         'fun_x_curve', fun_x_curve, ...
%                         'fun_x_curve_d1', fun_x_curve_d1, ...
%                         'fun_x_curve_d2', fun_x_curve_d2, ...
%                         'fun_y_curve', fun_y_curve, ...
%                         'fun_y_curve_d1', fun_y_curve_d1, ...
%                         'fun_y_curve_d2', fun_y_curve_d2);
     
%      %%% 平滑段-1
%      syms fun_s1(ds) fun_smooth1(ds) fun_smooth1_d1(ds) fun_smooth1_d2(ds)
%      fun_s1(ds) = 1/2*k1*ds^2 + ds^4*(k2-2*k1)/(6*L_trans^2);
%      fun_s1_d1(ds) = diff(fun_s1,ds);
%      fun_s1_d2(ds) = diff(fun_s1_d1,ds);
%      fun_s1_d3(ds) = diff(fun_s1_d2,ds);
%      fun_s1_d4(ds) = diff(fun_s1_d3,ds);
%      fun_s1_d5(ds) = diff(fun_s1_d4,ds);
%      s_L = 0;            s_R = L_smooth;
%      par_s1_L = 0;       par_s1_R =  L_smooth/2;
%      Coff_A1 = Layout_Smooth(fun_s1, fun_s1_d1, fun_s1_d2, fun_s1_d3, fun_s1_d4, fun_s1_d5, ...
%                              fun_s1, fun_s1_d1, fun_s1_d2, fun_s1_d3, fun_s1_d4, fun_s1_d5, ...
%                              s_L, s_R, par_s1_L, par_s1_R);
%      fun_smooth1(ds) = Coff_A1(12)*ds^11 + Coff_A1(11)*ds^10 + Coff_A1(10)*ds^9 +Coff_A1(9)*ds^8 + ...
%                        Coff_A1(8) *ds^7  + Coff_A1(7) *ds^6  + Coff_A1(6) *ds^5 +Coff_A1(5)*ds^4 + ...
%                        Coff_A1(4) *ds^3  + Coff_A1(3) *ds^2  + Coff_A1(2) *ds^1 +Coff_A1(1)*ds^0;
%      fun_smooth1_d1(ds) = diff(fun_smooth1,ds);
%      fun_smooth1_d2(ds) = diff(fun_smooth1_d1,ds);
%      
%      %%% 平滑段-2
%      syms fun_sL2(ds) fun_sR2(ds) fun_smooth2(ds) fun_smooth2_d1(ds) fun_smooth2_d2(ds)
%      fun_sL2(ds) = 1/2*k1*ds^2 + 2/(L_trans^2)*(1/3*L_trans*ds^3-1/12*ds^4-L_trans^2/4*ds^2)*(k2-2*k1) + ...
%                    L_trans/6*ds*(k2-2*k1) - 1/48*(L_trans^2)*(k2-2*k1);
%      fun_sL2_d1(ds) = diff(fun_sL2,ds);
%      fun_sL2_d2(ds) = diff(fun_sL2_d1,ds);
%      fun_sL2_d3(ds) = diff(fun_sL2_d2,ds);
%      fun_sL2_d4(ds) = diff(fun_sL2_d3,ds);
%      fun_sL2_d5(ds) = diff(fun_sL2_d4,ds);
%      
%      phi_transition = atan(double(fun_trans2_d1(L_trans)));
%      pos_Body_global_y_transition = double(fun_trans2(L_trans));
%      
%      phi_track(i,1) = (ds-L_trans/2)/R2 + phi_transition;
%      k_curvature(i,1) = 1/R2;
%      fun_sR2(ds) = R2*(1-cos(phi_track(i,1))) - R2*(1-cos(phi_transition)) + pos_Body_global_y_transition;
%      
%      
%      s_L = 0;            s_R = L_smooth;
%      par_s2_L = L_trans-L_smooth/2;       par_s2_R = L_trans+L_smooth/2;
%      Coff_A2 = Layout_Smooth(fun_sL2, fun_sL2_d1, fun_sL2_d2, fun_sL2_d3, fun_sL2_d4, fun_sL2_d5, ...
%                              fun_sR2, fun_sR2_d1, fun_sR2_d2, fun_sR2_d3, fun_sR2_d4, fun_sR2_d5, ...
%                              s_L, s_R, par_s2_L, par_s2_R);
%      fun_smooth2(ds) = Coff_A2(12)*ds^11 + Coff_A2(11)*ds^10 + Coff_A2(10)*ds^9 +Coff_A2(9)*ds^8 + ...
%                        Coff_A2(8) *ds^7  + Coff_A2(7) *ds^6  + Coff_A2(6) *ds^5 +Coff_A2(5)*ds^4 + ...
%                        Coff_A2(4) *ds^3  + Coff_A2(3) *ds^2  + Coff_A2(2) *ds^1 +Coff_A2(1)*ds^0;
%      fun_smooth2_d1(ds) = diff(fun_smooth2,ds);
%      fun_smooth2_d2(ds) = diff(fun_smooth2_d1,ds);
     
%      double(fun_smooth1(0))
%      double(fun_smooth1_d1(0))
%      double(fun_smooth1_d2(0))
%      double(fun_smooth1(L_smooth)) - double(fun_trans1(L_smooth/2))
%      double(fun_smooth1_d1(L_smooth)) - double(fun_trans1_d1(L_smooth/2))
%      double(fun_smooth1_d2(L_smooth)) - double(fun_trans1_d2(L_smooth/2))
% 
%      double(fun_smooth2(0)) - double(fun_trans2(L_trans-L_smooth/2))
%      double(fun_smooth2_d1(0)) - double(fun_trans2_d1(L_trans-L_smooth/2))
%      double(fun_smooth2_d2(0)) - double(fun_trans2_d2(L_trans-L_smooth/2))
%      double(fun_smooth2(L_smooth)) - double(fun_trans2(L_trans))
%      double(fun_smooth2_d1(L_smooth)) - double(fun_trans2_d1(L_trans))
%      double(fun_smooth2_d2(L_smooth)) - double(fun_trans2_d2(L_trans))
     
     
     