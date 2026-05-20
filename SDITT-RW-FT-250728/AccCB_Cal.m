%%  轮对上点加速度计算
function Acc_Target = AccCB_Cal(Range_Mileage, ZP_Mileage, ZP_Dis_Target, ZP_Vel_Target, ZP_Acc_Target, N_track, NM_FW, Nw, pos_Target)

% Test, Target_oup
Choose_Test = 0;
if Choose_Test==1
    Range_Mileage = [40, 115];
    ZP_Mileage = cell2mat(ZP_Dyn.Mileage);
    ZP_Dis_Target = ZP_Dis.Correction;
    ZP_Vel_Target = ZP_Vel.Correction;
    ZP_Acc_Target = ZP_Acc.Correction;
    pos_Target = [8.75, 1, 0.51];   % HSR
end

% Target_oup
bools = (ZP_Mileage(:,1)>=Range_Mileage(1) & ZP_Mileage(:,1)<=Range_Mileage(2));
p = min(find(bools));
q = max(find(bools));

for i = p:1:q
    pos_CB = N_track+NM_FW*Nw+30;
    Acc_CB = [0; ZP_Acc_Target(i,pos_CB+2); ZP_Acc_Target(i,pos_CB+1)];
    pos_Roll_track = ZP_Dis_Target(i,pos_CB+3);
    pos_Pitch_track = ZP_Dis_Target(i,pos_CB+4);
    pos_Yaw_track = ZP_Dis_Target(i,pos_CB+5);
    vel_Roll_track = ZP_Vel_Target(i,pos_CB+3);
    vel_Pitch_track = ZP_Vel_Target(i,pos_CB+4);
    vel_Yaw_track = ZP_Vel_Target(i,pos_CB+5);

    % 旋转顺序 3-1-2
    G_Carden    = [cos(pos_Yaw_track), -sin(pos_Yaw_track)*cos(pos_Roll_track), 0;
                            sin(pos_Yaw_track),  cos(pos_Yaw_track)*cos(pos_Roll_track), 0;
                            0,                              sin(pos_Roll_track),                                  1];

    temp_G_Carden_d1_12 = -cos(pos_Yaw_track)*cos(pos_Roll_track)*vel_Yaw_track + sin(pos_Yaw_track)*sin(pos_Roll_track)*vel_Roll_track;
    temp_G_Carden_d1_22 = -sin(pos_Yaw_track)*cos(pos_Roll_track)*vel_Yaw_track - cos(pos_Yaw_track)*sin(pos_Roll_track)*vel_Roll_track;
    G_Carden_d1 = [-sin(pos_Yaw_track)*vel_Yaw_track,  temp_G_Carden_d1_12,                 0;
                                cos(pos_Yaw_track)*vel_Yaw_track,  temp_G_Carden_d1_22,                 0;
                                0,                                 cos(pos_Roll_track)*vel_Roll_track,	                    0];

    AngVel_CB = G_Carden * ZP_Vel_Target(i,pos_CB+3:pos_CB+5)';
    AngAcc_CB = G_Carden_d1 * ZP_Vel_Target(i,pos_CB+3:pos_CB+5)' + G_Carden * ZP_Acc_Target(i,pos_CB+3:pos_CB+5)';

    A1 = [ cos(pos_Yaw_track)      sin(pos_Yaw_track)    0;
              -sin(pos_Yaw_track)       cos(pos_Yaw_track)   0;
                0                         0                      1];
    A2 = [1      0                        0;
              0      cos(pos_Roll_track)     sin(pos_Roll_track);
              0     -sin(pos_Roll_track)      cos(pos_Roll_track)];
    A3 = [ cos(pos_Pitch_track)    0  -sin(pos_Pitch_track);
               0                    1    0
               sin(pos_Pitch_track)    0    cos(pos_Pitch_track)];
%     A3 = [ cos(pos_Pitch_track)    0   sin(pos_Pitch_track);
%                0                    1    0
%               -sin(pos_Pitch_track)    0    cos(pos_Pitch_track)];
    A_Body = A3*A2*A1;
%     A_Body = A1*A2*A3;
    A_Body = A_Body';

    u = A_Body*pos_Target';

    Acc_Target_1 = Acc_CB;
    Acc_Target_2 = [AngAcc_CB(2).*u(3)-AngAcc_CB(3).*u(2);...
                               AngAcc_CB(3).*u(1)-AngAcc_CB(1).*u(3);...
                               AngAcc_CB(1).*u(2)-AngAcc_CB(2).*u(1)];
    Acc_Target_3_temp1 = [AngVel_CB(2).*u(3)-AngVel_CB(3).*u(2);...
                                           AngVel_CB(3).*u(1)-AngVel_CB(1).*u(3);...
                                           AngVel_CB(1).*u(2)-AngVel_CB(2).*u(1)];
    Acc_Target_3 = [AngVel_CB(2).*Acc_Target_3_temp1(3)-AngVel_CB(3).*Acc_Target_3_temp1(2);...
                               AngVel_CB(3).*Acc_Target_3_temp1(1)-AngVel_CB(1).*Acc_Target_3_temp1(3);...
                               AngVel_CB(1).*Acc_Target_3_temp1(2)-AngVel_CB(2).*Acc_Target_3_temp1(1)];
    Acc_Target(i-p+1,1:4) = [ZP_Mileage(i,1) (Acc_Target_1+Acc_Target_2+Acc_Target_3)'];

    clc
    disp(['Acc_CB Cal. Progress = ', num2str(i-p+1), '/', num2str(q-p+1)]);

end
