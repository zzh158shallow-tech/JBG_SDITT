%%  轮对上点加速度计算
function Acc_Target = Acc_Cal_Thr(Type_WS, Range_Mileage, ZP_Mileage, ZP_Dis, ZP_Vel, ZP_Acc, i11, N_track, NM, Nw, ...
                                                         ModeShape, DOFpos_Target, pos_Target)
                                                            
% Test, Target_oup
Choose_Test = 0;
if Choose_Test==1    
    Type_WS = 'FW';
    ZP_Mileage = Target_oup.ZP_Dyn.Mileage;
    ZP_Dis = Target_oup.ZP_Dis;
    ZP_Vel = Target_oup.ZP_Vel;
    ZP_Acc = Target_oup.ZP_Acc;
    ModeShape = Target_oup.ModeShape.FW;
    NM = Target_oup.NM_FW;
    
    Type_WS = 'RW';
    ZP_Mileage = cell2mat(Target_oup.ZP_Dyn.Mileage);
    ZP_Dis = Target_oup.ZP_Dis;
    ZP_Vel = Target_oup.ZP_Vel;
    ZP_Acc = Target_oup.ZP_Acc;
    ModeShape = [];
    NM = 0;
end
% Test, Target_oup

% ZP_Mileage 从 cell 转为 mat 后，在前面补充3个零
% ZP_Mileage = [zeros(3,3); ZP_Mileage];

% Target_oup
bools = (ZP_Mileage(:,1)>=Range_Mileage(1) & ZP_Mileage(:,1)<=Range_Mileage(2));
p = find(bools, 1 );
q = find(bools, 1, 'last' );
Acc_Target = zeros(q-p+1,4);
    
for i = p:1:q
    pos_WS = N_track+NM*Nw+5*(i11-1);
    Acc_WS = [0; ZP_Acc(i,pos_WS+2); ZP_Acc(i,pos_WS+1)];
    pos_Roll_track = ZP_Dis(i,pos_WS+3);
%     pos_Pitch_track = ZP_Dis(i,pos_WS+4);
    pos_Yaw_track = ZP_Dis(i,pos_WS+5);
    vel_Roll_track = ZP_Vel(i,pos_WS+3);
%     vel_Pitch_track = ZP_Vel(i,pos_WS+4);
    vel_Yaw_track = ZP_Vel(i,pos_WS+5);

    % 旋转顺序 3-1-2
    G_Carden = [cos(pos_Yaw_track), -sin(pos_Yaw_track)*cos(pos_Roll_track), 0;...
                         sin(pos_Yaw_track),   cos(pos_Yaw_track)*cos(pos_Roll_track), 0;...
                         0,                                sin(pos_Roll_track),                                  1];
    temp_G_Carden_d1_12 = -cos(pos_Yaw_track)*cos(pos_Roll_track)*vel_Yaw_track + sin(pos_Yaw_track)*sin(pos_Roll_track)*vel_Roll_track;
    temp_G_Carden_d1_22 = -sin(pos_Yaw_track)*cos(pos_Roll_track)*vel_Yaw_track - cos(pos_Yaw_track)*sin(pos_Roll_track)*vel_Roll_track;
    G_Carden_d1 = [-sin(pos_Yaw_track)*vel_Yaw_track, temp_G_Carden_d1_12,                   0; ...
                                cos(pos_Yaw_track)*vel_Yaw_track, temp_G_Carden_d1_22,                   0; ...
                                0,                                                      cos(pos_Roll_track)*vel_Roll_track,  0];

    AngVel_WS = G_Carden * (ZP_Vel(i,pos_WS+3:pos_WS+5)');
    AngAcc_WS = G_Carden_d1 * (ZP_Vel(i,pos_WS+3:pos_WS+5)') + G_Carden * ZP_Acc(i,pos_WS+3:pos_WS+5)';

    AngVel_WS_Antisym = [0, -AngVel_WS(3), AngVel_WS(2); AngVel_WS(3), 0, -AngVel_WS(1); -AngVel_WS(2), AngVel_WS(1), 0];
    AngAcc_WS_Antisym = [0, -AngAcc_WS(3), AngAcc_WS(2); AngAcc_WS(3), 0, -AngAcc_WS(1); -AngAcc_WS(2), AngAcc_WS(1), 0];

    A_WS = [cos(pos_Yaw_track)                                  sin(pos_Yaw_track)                                    0;
                  -cos(pos_Roll_track)*sin(pos_Yaw_track)	cos(pos_Roll_track)*cos(pos_Yaw_track)   sin(pos_Roll_track);
                    sin(pos_Roll_track)*sin(pos_Yaw_track)   -sin(pos_Roll_track)*cos(pos_Yaw_track)   cos(pos_Roll_track)];
    A_WS = A_WS';

    if strcmp(Type_WS, 'FW')
        bools = DOFpos_Target;
        u = A_WS*( pos_Target + ModeShape(bools,:)*ZP_Dis(i,N_track+NM*(i11-1)+(1:1:NM))' );
        Acc_Target_1 = Acc_WS;
        Acc_Target_2 = AngVel_WS_Antisym*AngVel_WS_Antisym*u;
        Acc_Target_3 = AngAcc_WS_Antisym*u;
        Acc_Target_4 = 2*AngVel_WS_Antisym*A_WS*ModeShape(bools,:)*ZP_Vel(i,N_track+NM*(i11-1)+(1:1:NM))';    % 科氏加速度
        Acc_Target_5 = A_WS*(ModeShape(bools,:)*ZP_Acc(i,N_track+NM*(i11-1)+(1:1:NM))');                                        % 相对加速度
        Acc_Target(i-p+1,1:4) = [ZP_Mileage(i,1) (Acc_Target_1+Acc_Target_2+Acc_Target_3+Acc_Target_4+Acc_Target_5)'];
    elseif strcmp(Type_WS, 'RW')
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
        Acc_Target(i-p+1,1:4) = [ZP_Mileage(i,1) (Acc_Target_1+Acc_Target_2+Acc_Target_3)'];
    end

end

