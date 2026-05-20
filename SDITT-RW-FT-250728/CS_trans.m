%% 计算接触点相对于轮对质心的相对速度的转换矩阵

function A_trans = CS_trans(Pos_Yaw_track, Pos_Roll_track, Pos_Yaw_WS, Pos_Roll_WS, Vel_Yaw_track, Vel_Roll_track, Vel_Yaw_WS, Vel_Roll_WS)

A_track_d0 = Matrix_A_trans_d0(Pos_Yaw_track, Pos_Roll_track);
A_WS_d0    = Matrix_A_trans_d0(Pos_Yaw_WS, Pos_Roll_WS);
A_track_d1 = Matrix_A_trans_d1(Pos_Yaw_track, Pos_Roll_track, Vel_Yaw_track, Vel_Roll_track);
A_WS_d1    = Matrix_A_trans_d1(Pos_Yaw_WS, Pos_Roll_WS, Vel_Yaw_WS, Vel_Roll_WS);

A_trans = A_WS_d1 * A_track_d0 + A_WS_d0 * A_track_d1;

function A_d0 = Matrix_A_trans_d0(Pos_Yaw, Pos_Roll)
A_d0 =[cos(Pos_Yaw)                sin(Pos_Yaw)                 0;
      -cos(Pos_Roll)*sin(Pos_Yaw)  cos(Pos_Roll)*cos(Pos_Yaw)	sin(Pos_Roll);
       sin(Pos_Roll)*sin(Pos_Yaw) -sin(Pos_Roll)*cos(Pos_Yaw)	cos(Pos_Roll)];

function A_d1 = Matrix_A_trans_d1(Pos_Yaw, Pos_Roll, Vel_Yaw, Vel_Roll)
A_d_Yaw =[-sin(Pos_Yaw)                cos(Pos_Yaw)                 0;
          -cos(Pos_Roll)*cos(Pos_Yaw) -cos(Pos_Roll)*sin(Pos_Yaw)	0;
           sin(Pos_Roll)*cos(Pos_Yaw)  sin(Pos_Roll)*sin(Pos_Yaw)	0];
A_d_Roll =[0                            0                           0;
           sin(Pos_Roll)*sin(Pos_Yaw)  -sin(Pos_Roll)*cos(Pos_Yaw)	cos(Pos_Roll);
           cos(Pos_Roll)*sin(Pos_Yaw)  -cos(Pos_Roll)*cos(Pos_Yaw) -sin(Pos_Roll)];
A_d1 = A_d_Yaw * Vel_Yaw + A_d_Roll * Vel_Roll;
