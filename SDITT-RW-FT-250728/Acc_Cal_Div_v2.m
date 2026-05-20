%%  轮对上点加速度计算(考虑从 Track C.S. 转化至 Global C.S.)
function [Acc_Target_Global, Acc_Target_Track] = Acc_Cal_Div_v2(Type_WS, Range_Mileage, ZP_Mileage, ZP_Dis, ZP_Vel, ZP_Acc, i11, N_track, NM, Nw, ...
          ModeShape, DOFpos_Target, pos_Target)

%%% Track Par
Type_Layout = 'Curve';
R1 = inf;	R2 = 760;	L_trans = 0.2;	L_smooth = 0;	Curve_start = 0;  Vlc = 80/3.6;
Par_Track = Cal_fun_smooth_trans_210212(R1, R2, L_trans, L_smooth, Curve_start, Vlc);
clear R1 R2 L_trans L_smooth Curve_start
     
%%% Target_oup
bools = (ZP_Mileage(:,1)>=Range_Mileage(1) & ZP_Mileage(:,1)<=Range_Mileage(2));
p = min(find(bools));
q = max(find(bools));

% pos_Yaw_trackCS, vel_Yaw_trackCS, acc_Yaw_trackCS, R_track_d2
Mileage_temp = ZP_Mileage(p:q,1);
bools_1 = (Mileage_temp < -Par_Track.L_trans/2+Par_Track.Curve_start);
pos_Yaw_trackCS(bools_1,1) = 0;
vel_Yaw_trackCS(bools_1,1) = 0;
acc_Yaw_trackCS(bools_1,1) = 0;
R_track_d2(bools_1,:) = zeros(length(find(bools_1)),3);

bools_2 = (Mileage_temp >= -Par_Track.L_trans/2+Par_Track.Curve_start) & (Mileage_temp < 0+Par_Track.Curve_start);
s = Mileage_temp(bools_2,1)-Par_Track.Curve_start+Par_Track.L_trans/2;
pos_Yaw_trackCS(bools_2,1) = double(Par_Track.fun_yaw_trans1(s/Vlc));
vel_Yaw_trackCS(bools_2,1) = double(Par_Track.fun_yaw_trans1_d1(s/Vlc));
acc_Yaw_trackCS(bools_2,1) = double(Par_Track.fun_yaw_trans1_d2(s/Vlc));
R_track_d2(bools_2,:) = [double(Par_Track.fun_x_trans1_d2(s/Vlc)) double(Par_Track.fun_y_trans1_d2(s/Vlc)) zeros(length(find(bools_2)),1)];

bools_3 = (Mileage_temp >= 0+Par_Track.Curve_start) & (Mileage_temp < Par_Track.L_trans/2+Par_Track.Curve_start);
s = Mileage_temp(bools_3,1)-Par_Track.Curve_start+Par_Track.L_trans/2;
pos_Yaw_trackCS(bools_3,1) = double(Par_Track.fun_yaw_trans2(s/Vlc));
vel_Yaw_trackCS(bools_3,1) = double(Par_Track.fun_yaw_trans2_d1(s/Vlc));
acc_Yaw_trackCS(bools_3,1) = double(Par_Track.fun_yaw_trans2_d2(s/Vlc));
R_track_d2(bools_3,:) = [double(Par_Track.fun_x_trans2_d2(s/Vlc)) double(Par_Track.fun_y_trans2_d2(s/Vlc)) zeros(length(find(bools_3)),1)];

bools_4 = (Mileage_temp >= Par_Track.L_trans/2+Par_Track.Curve_start);
s = Mileage_temp(bools_4,1)-Par_Track.Curve_start;
pos_Yaw_trackCS(bools_4,1) = double(Par_Track.fun_yaw_curve(s/Vlc));
vel_Yaw_trackCS(bools_4,1) = double(Par_Track.fun_yaw_curve_d1(s/Vlc));
acc_Yaw_trackCS(bools_4,1) = double(Par_Track.fun_yaw_curve_d2(s/Vlc));
R_track_d2(bools_4,:) = [double(Par_Track.fun_x_curve_d2(s/Vlc)) double(Par_Track.fun_y_curve_d2(s/Vlc)) zeros(length(find(bools_4)),1)];

for i = p:1:q
    
    clc
    disp([num2str((i-p)/(q-p)*100),'%']);
    
    %%% A_track, A_track_d1, A_track_d2
%     % pos_Yaw_trackCS, vel_Yaw_trackCS, acc_Yaw_trackCS, R_track_d2
%     Mileage = ZP_Mileage(i,1);
%     if Mileage < -Par_Track.L_trans/2 + Par_Track.Curve_start
%         pos_Yaw_trackCS = 0;
%         vel_Yaw_trackCS = 0;
%         acc_Yaw_trackCS = 0;
% %         R_track    = [Mileage, 0, 0];
% %         R_track_d1 = [Vlc, 0, 0];
%         R_track_d2 = [0, 0, 0];
%     elseif Mileage >= -Par_Track.L_trans/2+Par_Track.Curve_start && Mileage < 0+Par_Track.Curve_start
%         s = Mileage-Par_Track.Curve_start+Par_Track.L_trans/2;
%         pos_Yaw_trackCS = double(Par_Track.fun_yaw_trans1(s/Vlc));
%         vel_Yaw_trackCS = double(Par_Track.fun_yaw_trans1_d1(s/Vlc));
%         acc_Yaw_trackCS = double(Par_Track.fun_yaw_trans1_d2(s/Vlc));
% %         R_track    = [double(Par_Track.fun_x_trans1(s/Vlc)) double(Par_Track.fun_y_trans1(s/Vlc)) 0];
% %         R_track_d1 = [double(Par_Track.fun_x_trans1_d1(s/Vlc)) double(Par_Track.fun_y_trans1_d1(s/Vlc)) 0];
%         R_track_d2 = [double(Par_Track.fun_x_trans1_d2(s/Vlc)) double(Par_Track.fun_y_trans1_d2(s/Vlc)) 0];
%     elseif Mileage >= 0+Par_Track.Curve_start && Mileage < Par_Track.L_trans/2+Par_Track.Curve_start
%         s = Mileage-Par_Track.Curve_start+Par_Track.L_trans/2;
%         pos_Yaw_trackCS = double(Par_Track.fun_yaw_trans2(s/Vlc));
%         vel_Yaw_trackCS = double(Par_Track.fun_yaw_trans2_d1(s/Vlc));
%         acc_Yaw_trackCS = double(Par_Track.fun_yaw_trans2_d2(s/Vlc));
% %         R_track    = [double(Par_Track.fun_x_trans2(s/Vlc)) double(Par_Track.fun_y_trans2(s/Vlc)) 0];
% %         R_track_d1 = [double(Par_Track.fun_x_trans2_d1(s/Vlc)) double(Par_Track.fun_y_trans2_d1(s/Vlc)) 0];
%         R_track_d2 = [double(Par_Track.fun_x_trans2_d2(s/Vlc)) double(Par_Track.fun_y_trans2_d2(s/Vlc)) 0];
%     else
%         s = Mileage-Par_Track.Curve_start;
%         pos_Yaw_trackCS = double(Par_Track.fun_yaw_curve(s/Vlc));
%         vel_Yaw_trackCS = double(Par_Track.fun_yaw_curve_d1(s/Vlc));
%         acc_Yaw_trackCS = double(Par_Track.fun_yaw_curve_d2(s/Vlc));
% %         R_track    = [double(Par_Track.fun_x_curve(s/Vlc)) double(Par_Track.fun_y_curve(s/Vlc)) 0];
% %         R_track_d1 = [double(Par_Track.fun_x_curve_d1(s/Vlc)) double(Par_Track.fun_y_curve_d1(s/Vlc)) 0];
%         R_track_d2 = [double(Par_Track.fun_x_curve_d2(s/Vlc)) double(Par_Track.fun_y_curve_d2(s/Vlc)) 0];
%     end
    
    % A_Track
    A_track = [cos(pos_Yaw_trackCS(i-p+1,1))  sin(pos_Yaw_trackCS(i-p+1,1))  0
              -sin(pos_Yaw_trackCS(i-p+1,1))  cos(pos_Yaw_trackCS(i-p+1,1))  0
               0                     0                     1];
    % A_Track_d1
    A_track_d1_temp = [-sin(pos_Yaw_trackCS(i-p+1,1)),  cos(pos_Yaw_trackCS(i-p+1,1)), 0
                       -cos(pos_Yaw_trackCS(i-p+1,1)), -sin(pos_Yaw_trackCS(i-p+1,1)), 0
                        0                   ,  0                   , 0];
    A_track_d1 = A_track_d1_temp * vel_Yaw_trackCS(i-p+1,1);
    % A_Track_d2
    A_track_d2a = A_track_d1_temp * acc_Yaw_trackCS(i-p+1,1);
    A_track_d2b_temp = [-cos(pos_Yaw_trackCS(i-p+1,1)), -sin(pos_Yaw_trackCS(i-p+1,1)), 0
                         sin(pos_Yaw_trackCS(i-p+1,1)), -cos(pos_Yaw_trackCS(i-p+1,1)), 0
                         0                   ,  0                   , 0];
    A_track_d2b = A_track_d2b_temp * (vel_Yaw_trackCS(i-p+1,1))^2;
    A_track_d2  = A_track_d2a + A_track_d2b;

    %%% FW
    if strcmp(Type_WS, 'FW')
        % Track C.S.
        Acc_WS = [0; ZP_Acc(i,N_track+NM*Nw+5*(i11-1)+2); ZP_Acc(i,N_track+NM*Nw+5*(i11-1)+1)];        
        pos_Roll_track = ZP_Dis(i,N_track+NM*Nw+5*(i11-1)+3);
        pos_Pitch_track = ZP_Dis(i,N_track+NM*Nw+5*(i11-1)+4);
        pos_Yaw_track = ZP_Dis(i,N_track+NM*Nw+5*(i11-1)+5);
        vel_Roll_track = ZP_Vel(i,N_track+NM*Nw+5*(i11-1)+3);
        vel_Pitch_track = ZP_Vel(i,N_track+NM*Nw+5*(i11-1)+4);
        vel_Yaw_track = ZP_Vel(i,N_track+NM*Nw+5*(i11-1)+5);
        
        G_Carden = [cos(pos_Yaw_track), -sin(pos_Yaw_track)*cos(pos_Roll_track), 0;...
                    sin(pos_Yaw_track),  cos(pos_Yaw_track)*cos(pos_Roll_track), 0;...
                    0,                   sin(pos_Roll_track),                    1];
        temp_G_Carden_d1_12 = -cos(pos_Yaw_track)*cos(pos_Roll_track)*vel_Yaw_track + sin(pos_Yaw_track)*sin(pos_Roll_track)*vel_Roll_track;
        temp_G_Carden_d1_22 = -sin(pos_Yaw_track)*cos(pos_Roll_track)*vel_Yaw_track - cos(pos_Yaw_track)*sin(pos_Roll_track)*vel_Roll_track;
        G_Carden_d1 = [-sin(pos_Yaw_track)*vel_Yaw_track, temp_G_Carden_d1_12,                  0;...
                        cos(pos_Yaw_track)*vel_Yaw_track, temp_G_Carden_d1_22,                  0;...
                        0,                                cos(pos_Roll_track)*vel_Roll_track,	0];
        
        AngVel_WS = G_Carden * (ZP_Vel(i,N_track+NM*Nw+5*(i11-1)+3:N_track+NM*Nw+5*(i11-1)+5)');
        AngAcc_WS = G_Carden_d1 * (ZP_Vel(i,N_track+NM*Nw+5*(i11-1)+3:N_track+NM*Nw+5*(i11-1)+5)') + G_Carden * ZP_Acc(i,N_track+NM*Nw+5*(i11-1)+3:N_track+NM*Nw+5*(i11-1)+5)';
  
        AngVel_WS_Antisym = [0, -AngVel_WS(3), AngVel_WS(2);...
                             AngVel_WS(3), 0, -AngVel_WS(1);...
                            -AngVel_WS(2), AngVel_WS(1), 0];        
        AngAcc_WS_Antisym = [0, -AngAcc_WS(3), AngAcc_WS(2);...
                             AngAcc_WS(3), 0, -AngAcc_WS(1);...
                            -AngAcc_WS(2), AngAcc_WS(1), 0];
        
        A_WS = [cos(pos_Yaw_track)                      sin(pos_Yaw_track)                      0;
               -cos(pos_Roll_track)*sin(pos_Yaw_track)	cos(pos_Roll_track)*cos(pos_Yaw_track)	sin(pos_Roll_track);
                sin(pos_Roll_track)*sin(pos_Yaw_track) -sin(pos_Roll_track)*cos(pos_Yaw_track)	cos(pos_Roll_track)];
        A_WS = A_WS';
        
        bools = DOFpos_Target;
        u = A_WS*(pos_Target + ModeShape(bools,:)*ZP_Dis(i,N_track+NM*(i11-1)+1:N_track+NM*i11)');
        
        Acc_Target_1 = Acc_WS;
        Acc_Target_2 = AngVel_WS_Antisym*AngVel_WS_Antisym*u;
        Acc_Target_3 = AngAcc_WS_Antisym*u;
        Acc_Target_4 = 2*AngVel_WS_Antisym*A_WS*ModeShape(bools,:)*ZP_Vel(i,N_track+NM*(i11-1)+1:N_track+NM*i11)';  % 科氏加速度
        Acc_Target_5 = A_WS*(ModeShape(bools,:)*ZP_Acc(i,N_track+NM*(i11-1)+1:N_track+NM*i11)');                    % 相对加速度
        
        Acc_Target_Track(i-p+1,1:4) = [ZP_Mileage(i,1) (Acc_Target_1+Acc_Target_2+Acc_Target_3+Acc_Target_4+Acc_Target_5)'];
%         Acc_Target_Track(i-p+1,1:4) = [ZP_Mileage(i,1) (Acc_Target_1+Acc_Target_2+Acc_Target_3)'];
        
        % Global C.S.
        Rp    = [0, ZP_Dis(i,N_track+NM*Nw+5*(i11-1)+2), ZP_Dis(i,N_track+NM*Nw+5*(i11-1)+1)] + u';
        Rp_d1 = [0, ZP_Vel(i,N_track+NM*Nw+5*(i11-1)+2), ZP_Vel(i,N_track+NM*Nw+5*(i11-1)+1)] + (AngVel_WS_Antisym*u)' + ...
                (A_WS*ModeShape(bools,:)*ZP_Vel(i,N_track+NM*(i11-1)+1:N_track+NM*i11)')';
        Rp_d2 = Acc_Target_Track(i-p+1,2:4);
        Acc_Target_Global(i-p+1,1:4) = [ZP_Mileage(i,1) R_track_d2(i-p+1,:) + Rp*A_track_d2 + 2*Rp_d1*A_track_d1 + Rp_d2*A_track];        

    %%% RW
    elseif strcmp(Type_WS, 'RW')    
        % Track C.S.
        Acc_WS = [0; ZP_Acc(i,N_track+NM*Nw+5*(i11-1)+2); ZP_Acc(i,N_track+NM*Nw+5*(i11-1)+1)];
        pos_Roll_track = ZP_Dis(i,N_track+NM*Nw+5*(i11-1)+3);
        pos_Pitch_track = ZP_Dis(i,N_track+NM*Nw+5*(i11-1)+4);
        pos_Yaw_track = ZP_Dis(i,N_track+NM*Nw+5*(i11-1)+5);
        vel_Roll_track = ZP_Vel(i,N_track+NM*Nw+5*(i11-1)+3);
        vel_Pitch_track = ZP_Vel(i,N_track+NM*Nw+5*(i11-1)+4);
        vel_Yaw_track = ZP_Vel(i,N_track+NM*Nw+5*(i11-1)+5);
        
        G_Carden    = [cos(pos_Yaw_track), -sin(pos_Yaw_track)*cos(pos_Roll_track), 0;...
                       sin(pos_Yaw_track),  cos(pos_Yaw_track)*cos(pos_Roll_track), 0;...
                       0,                   sin(pos_Roll_track),                    1];
        temp_G_Carden_d1_12 = -cos(pos_Yaw_track)*cos(pos_Roll_track)*vel_Yaw_track + sin(pos_Yaw_track)*sin(pos_Roll_track)*vel_Roll_track;
        temp_G_Carden_d1_22 = -sin(pos_Yaw_track)*cos(pos_Roll_track)*vel_Yaw_track - cos(pos_Yaw_track)*sin(pos_Roll_track)*vel_Roll_track;
        G_Carden_d1 = [-sin(pos_Yaw_track)*vel_Yaw_track, temp_G_Carden_d1_12,                  0;...
                        cos(pos_Yaw_track)*vel_Yaw_track, temp_G_Carden_d1_22,                  0;...
                        0,                                cos(pos_Roll_track)*vel_Roll_track,	0];
        
        AngVel_WS = G_Carden * (ZP_Vel(i,N_track+NM*Nw+5*(i11-1)+3:N_track+NM*Nw+5*(i11-1)+5)');
        AngAcc_WS = G_Carden_d1 * (ZP_Vel(i,N_track+NM*Nw+5*(i11-1)+3:N_track+NM*Nw+5*(i11-1)+5)') + G_Carden * ZP_Acc(i,N_track+NM*Nw+5*(i11-1)+3:N_track+NM*Nw+5*(i11-1)+5)';
        
        A_WS = [cos(pos_Yaw_track)                      sin(pos_Yaw_track)                      0;
               -cos(pos_Roll_track)*sin(pos_Yaw_track)	cos(pos_Roll_track)*cos(pos_Yaw_track)	sin(pos_Roll_track);
                sin(pos_Roll_track)*sin(pos_Yaw_track) -sin(pos_Roll_track)*cos(pos_Yaw_track)	cos(pos_Roll_track)];
        A_WS = A_WS';
        
        u = A_WS*pos_Target;
        
        Acc_Target_1 = Acc_WS;
        Acc_Target_2 = [AngAcc_WS(2).*u(3)-AngAcc_WS(3).*u(2);...
                        AngAcc_WS(3).*u(1)-AngAcc_WS(1).*u(3);...
                        AngAcc_WS(1).*u(2)-AngAcc_WS(2).*u(1)];
        Acc_Target_3_temp1 = [AngVel_WS(2).*u(3)-AngVel_WS(3).*u(2);...
                              AngVel_WS(3).*u(1)-AngVel_WS(1).*u(3);...
                              AngVel_WS(1).*u(2)-AngVel_WS(2).*u(1)];
        Acc_Target_3 = [AngVel_WS(2).*Acc_Target_3_temp1(3)-AngVel_WS(3).*Acc_Target_3_temp1(2);...
                        AngVel_WS(3).*Acc_Target_3_temp1(1)-AngVel_WS(1).*Acc_Target_3_temp1(3);...
                        AngVel_WS(1).*Acc_Target_3_temp1(2)-AngVel_WS(2).*Acc_Target_3_temp1(1)];
        Acc_Target_Track(i-p+1,1:4) = [ZP_Mileage(i,1) (Acc_Target_1+Acc_Target_2+Acc_Target_3)'];
        
        % Global C.S.
        Rp    = [0, ZP_Dis(i,N_track+NM*Nw+5*(i11-1)+2), ZP_Dis(i,N_track+NM*Nw+5*(i11-1)+1)] + u';
        Rp_d1 = [0, ZP_Vel(i,N_track+NM*Nw+5*(i11-1)+2), ZP_Vel(i,N_track+NM*Nw+5*(i11-1)+1)] + Acc_Target_3_temp1';
        Rp_d2 = Acc_Target_Track(i-p+1,2:4);        
        Acc_Target_Global(i-p+1,1:4) = [ZP_Mileage(i,1) R_track_d2(i-p+1,:) + Rp*A_track_d2 + 2*Rp_d1*A_track_d1 + Rp_d2*A_track];
    
    end
    
end
    
% Acc_WS = ZP_Acc(i,N_track+NM*Nw+5*(i11-1)+1:N_track+NM*Nw+5*(i11-1)+3)';
% pos_Roll_track = ZP_Dis(i,N_track+NM*Nw+5*(i11-1)+4);
% pos_Pitch_track = ZP_Dis(i,N_track+NM*Nw+5*(i11-1)+5);
% pos_Yaw_track = ZP_Dis(i,N_track+NM*Nw+5*(i11-1)+6);
% vel_Roll_track = ZP_Vel(i,N_track+NM*Nw+5*(i11-1)+4);
% vel_Pitch_track = ZP_Vel(i,N_track+NM*Nw+5*(i11-1)+5);
% vel_Yaw_track = ZP_Vel(i,N_track+NM*Nw+5*(i11-1)+6);
% 
% Vjd_R(:,1) = vel_Xw_global + Vzdlj.*R_R(:,3) - Vzdlk.*R_R(:,2);
% Vjd_R(:,2) = vel_Yw_global + Vzdlk.*R_R(:,1) - Vzdli.*R_R(:,3);
% Vjd_R(:,3) = vel_Zw_global + Vzdli.*R_R(:,2) - Vzdlj.*R_R(:,1);
