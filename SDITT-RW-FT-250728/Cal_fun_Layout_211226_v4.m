%% 07(009)道岔定义线型函数：尖轨尖端位置定义两段S型缓和曲线，导曲线实际起点处定义S型缓和曲线

function Par_Layout = Cal_fun_Layout_211226_v4(Par_Layout)

Choose_Plot = 1;

k1 = 1/Par_Layout.R1;
k2 = 1/Par_Layout.R2;
k3 = 1/Par_Layout.R3;
k = 1/Par_Layout.R;

syms Alg_Vlc
%%% 缓和曲线-1a（以缓和曲线1起点为原点）
syms fun_k_trans1(Alg_Vlc,dt) fun_k_trans1_d1(Alg_Vlc,dt) fun_x_trans1(Alg_Vlc,dt) fun_x_trans1_d1(Alg_Vlc,dt) fun_x_trans1_d2(Alg_Vlc,dt) fun_y_trans1(Alg_Vlc,dt) fun_y_trans1_d1(Alg_Vlc,dt) fun_y_trans1_d2(Alg_Vlc,dt) ...
     fun_yaw_trans1(Alg_Vlc,dt) fun_yaw_trans1_d1(Alg_Vlc,dt) fun_yaw_trans1_d2(Alg_Vlc,dt)
fun_k_trans1a(Alg_Vlc,dt) = k1 + 2*(Alg_Vlc*dt)^2/Par_Layout.L_trans_1^2*(k2-2*k1);
fun_k_trans1a_d1(Alg_Vlc,dt) = diff(fun_k_trans1a,dt);

fun_x_trans1a(Alg_Vlc,dt) = (Alg_Vlc*dt)+(-Par_Layout.L_trans_1/2+Par_Layout.Curve_start_1);
fun_x_trans1a_d1(Alg_Vlc,dt) = diff(fun_x_trans1a,dt);
fun_x_trans1a_d2(Alg_Vlc,dt) = diff(fun_x_trans1a_d1,dt);

fun_y_trans1a(Alg_Vlc,dt) = 1/2*k1*(Alg_Vlc*dt)^2 + (Alg_Vlc*dt)^4*(k2-2*k1)/(6*Par_Layout.L_trans_1^2);
fun_y_trans1a_d1(Alg_Vlc,dt) = diff(fun_y_trans1a,dt);
fun_y_trans1a_d2(Alg_Vlc,dt) = diff(fun_y_trans1a_d1,dt);

fun_yaw_trans1a(Alg_Vlc,dt) = k1*(Alg_Vlc*dt) + 2*(Alg_Vlc*dt)^3/(3*Par_Layout.L_trans_1^2)*(k2-2*k1);
fun_yaw_trans1a_d1(Alg_Vlc,dt) = diff(fun_yaw_trans1a,dt);
fun_yaw_trans1a_d2(Alg_Vlc,dt) = diff(fun_yaw_trans1a_d1,dt);

% double(subs(fun_k_trans1,  [Alg_Vlc,dt], [80/3.6, 0.2/80*3.6]))';

%%% 缓和曲线-1b（以缓和曲线1起点为原点）
syms fun_k_trans2(Alg_Vlc,dt) fun_k_trans2_d1(Alg_Vlc,dt) fun_x_trans2(Alg_Vlc,dt) fun_x_trans2_d1(Alg_Vlc,dt) fun_x_trans2_d2(Alg_Vlc,dt) fun_y_trans2(Alg_Vlc,dt) fun_y_trans2_d1(Alg_Vlc,dt) fun_y_trans2_d2(Alg_Vlc,dt) ...
     fun_yaw_trans2(Alg_Vlc,dt) fun_yaw_trans2_d1(Alg_Vlc,dt) fun_yaw_trans2_d2(Alg_Vlc,dt)
fun_k_trans1b(Alg_Vlc,dt) = k1 + 2/Par_Layout.L_trans_1^2*(2*Par_Layout.L_trans_1*(Alg_Vlc*dt)-(Alg_Vlc*dt)^2-Par_Layout.L_trans_1^2/2)*(k2-2*k1);
fun_k_trans1b_d1(Alg_Vlc,dt) = diff(fun_k_trans1b,dt);

fun_x_trans1b(Alg_Vlc,dt) = fun_x_trans1a(Alg_Vlc,dt);
fun_x_trans1b_d1(Alg_Vlc,dt) = fun_x_trans1a_d1(Alg_Vlc,dt);
fun_x_trans1b_d2(Alg_Vlc,dt) = fun_x_trans1a_d2(Alg_Vlc,dt);

fun_y_trans1b(Alg_Vlc,dt) = 1/2*k1*(Alg_Vlc*dt)^2 + 2/(Par_Layout.L_trans_1^2)*(1/3*Par_Layout.L_trans_1*(Alg_Vlc*dt)^3-1/12*(Alg_Vlc*dt)^4-Par_Layout.L_trans_1^2/4*(Alg_Vlc*dt)^2)*(k2-2*k1) + ...
                            Par_Layout.L_trans_1/6*(Alg_Vlc*dt)*(k2-2*k1) - 1/48*(Par_Layout.L_trans_1^2)*(k2-2*k1);
fun_y_trans1b_d1(Alg_Vlc,dt) = diff(fun_y_trans1b,dt);
fun_y_trans1b_d2(Alg_Vlc,dt) = diff(fun_y_trans1b_d1,dt);

fun_yaw_trans1b(Alg_Vlc,dt) = k1*(Alg_Vlc*dt)+2/(Par_Layout.L_trans_1^2)*(Par_Layout.L_trans_1*(Alg_Vlc*dt)^2-1/3*(Alg_Vlc*dt)^3-(Par_Layout.L_trans_1^2)/2*(Alg_Vlc*dt))*(k2-2*k1)+Par_Layout.L_trans_1/6*(k2-2*k1);
fun_yaw_trans1b_d1(Alg_Vlc,dt) = diff(fun_yaw_trans1b,dt);
fun_yaw_trans1b_d2(Alg_Vlc,dt) = diff(fun_yaw_trans1b_d1,dt);

Par_Layout.Yaw_trans1b_End = double(subs(fun_yaw_trans1b(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_1/80]));
Par_Layout.X_trans1b_End = double(subs(fun_x_trans1b(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_1/80]));
Par_Layout.Y_trans1b_End = double(subs(fun_y_trans1b(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_1/80]));

%%% 缓和曲线-2a（以缓和曲线2起点为原点）
syms fun_k_trans2(Alg_Vlc,dt) fun_k_trans2_d1(Alg_Vlc,dt) fun_x_trans2(Alg_Vlc,dt) fun_x_trans2_d1(Alg_Vlc,dt) fun_x_trans2_d2(Alg_Vlc,dt) fun_y_trans2(Alg_Vlc,dt) fun_y_trans2_d1(Alg_Vlc,dt) fun_y_trans2_d2(Alg_Vlc,dt) ...
     fun_yaw_trans2(Alg_Vlc,dt) fun_yaw_trans2_d1(Alg_Vlc,dt) fun_yaw_trans2_d2(Alg_Vlc,dt)
fun_k_trans2a(Alg_Vlc,dt) = k3 + (1-2/Par_Layout.L_trans_2^2*(Alg_Vlc*dt)^2)*(k2-2*k3);
fun_k_trans2a_d1(Alg_Vlc,dt) = diff(fun_k_trans2a,dt);

fun_x_trans2a(Alg_Vlc,dt) = (Alg_Vlc*dt)+Par_Layout.X_trans1b_End;
fun_x_trans2a_d1(Alg_Vlc,dt) = diff(fun_x_trans2a,dt);
fun_x_trans2a_d2(Alg_Vlc,dt) = diff(fun_x_trans2a_d1,dt);

C1 = k1*Par_Layout.L_trans_2 + Par_Layout.L_trans_2/2*(k2-2*k1);
C2 = 1/2*k1*Par_Layout.L_trans_2^2 + 7/48*Par_Layout.L_trans_2^2*(k2-2*k1);

% fun_y_trans2a(Alg_Vlc,dt) = 1/2*(k2-k3)*(Alg_Vlc*dt)^2 + (4*k3-2*k2)/(12*Par_Layout.L_trans_2^2)*(Alg_Vlc*dt)^4 + ...
%                             C1*(Alg_Vlc*dt) + C2 + Par_Layout.Y_trans1b_End;
fun_y_trans2a(Alg_Vlc,dt) = 1/2*(k2-k3)*(Alg_Vlc*dt)^2 + (4*k3-2*k2)/(12*Par_Layout.L_trans_2^2)*(Alg_Vlc*dt)^4 + ...
                            C1*(Alg_Vlc*dt) + C2;
fun_y_trans2a_d1(Alg_Vlc,dt) = diff(fun_y_trans2a,dt);
fun_y_trans2a_d2(Alg_Vlc,dt) = diff(fun_y_trans2a_d1,dt);

% fun_yaw_trans2a(Alg_Vlc,dt) = (k2-k3)*(Alg_Vlc*dt) + (4*k3-2*k2)/(3*Par_Layout.L_trans_2^2)*(Alg_Vlc*dt)^3 + C1 + Par_Layout.Yaw_trans1b_End;
fun_yaw_trans2a(Alg_Vlc,dt) = (k2-k3)*(Alg_Vlc*dt) + (4*k3-2*k2)/(3*Par_Layout.L_trans_2^2)*(Alg_Vlc*dt)^3 + C1;
fun_yaw_trans2a_d1(Alg_Vlc,dt) = diff(fun_yaw_trans2a,dt);
fun_yaw_trans2a_d2(Alg_Vlc,dt) = diff(fun_yaw_trans2a_d1,dt);

% % Check
% double(subs(fun_k_trans1b(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_2/80]))
% double(subs(fun_k_trans2a(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_2/80*0]))
% double(subs(fun_y_trans1b(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_2/80]))
% double(subs(fun_y_trans2a(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_2/80*0]))
% double(subs(fun_yaw_trans1b(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_2/80]))
% double(subs(fun_yaw_trans2a(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_2/80*0]))

%%% 缓和曲线-2b（以缓和曲线2起点为原点）
syms fun_k_trans2(Alg_Vlc,dt) fun_k_trans2_d1(Alg_Vlc,dt) fun_x_trans2(Alg_Vlc,dt) fun_x_trans2_d1(Alg_Vlc,dt) fun_x_trans2_d2(Alg_Vlc,dt) fun_y_trans2(Alg_Vlc,dt) fun_y_trans2_d1(Alg_Vlc,dt) fun_y_trans2_d2(Alg_Vlc,dt) ...
     fun_yaw_trans2(Alg_Vlc,dt) fun_yaw_trans2_d1(Alg_Vlc,dt) fun_yaw_trans2_d2(Alg_Vlc,dt)
fun_k_trans2b(Alg_Vlc,dt) = k3 + 2/Par_Layout.L_trans_2^2*(Par_Layout.L_trans_2^2+(Alg_Vlc*dt)^2-2*Par_Layout.L_trans_2*(Alg_Vlc*dt))*(k2-2*k3);
fun_k_trans2b_d1(Alg_Vlc,dt) = diff(fun_k_trans2b,dt);

fun_x_trans2b(Alg_Vlc,dt) = fun_x_trans2a(Alg_Vlc,dt);
fun_x_trans2b_d1(Alg_Vlc,dt) = fun_x_trans2a_d1(Alg_Vlc,dt);
fun_x_trans2b_d2(Alg_Vlc,dt) = fun_x_trans2a_d2(Alg_Vlc,dt);

C1 = 1/3*Par_Layout.L_trans_2*(k2+k3);
C2 = (5/24*k1+1/6*k2-1/24*k3)*Par_Layout.L_trans_2^2;

fun_y_trans2b(Alg_Vlc,dt) = (2*k2-3*k3)/2*(Alg_Vlc*dt)^2 + 2/(12*Par_Layout.L_trans_2^2)*(k2-2*k3)*(Alg_Vlc*dt)^4 - ...
                             2/(3*Par_Layout.L_trans_2)*(k2-2*k3)*(Alg_Vlc*dt)^3 + C1*(Alg_Vlc*dt) + C2;
fun_y_trans2b_d1(Alg_Vlc,dt) = diff(fun_y_trans2b,dt);
fun_y_trans2b_d2(Alg_Vlc,dt) = diff(fun_y_trans2b_d1,dt);

fun_yaw_trans2b(Alg_Vlc,dt) = (2*k2-3*k3)*(Alg_Vlc*dt) + 2/(3*Par_Layout.L_trans_2^2)*(k2-2*k3)*(Alg_Vlc*dt)^3 - ...
                              2/Par_Layout.L_trans_2*(k2-2*k3)*(Alg_Vlc*dt)^2 + C1;
fun_yaw_trans2b_d1(Alg_Vlc,dt) = diff(fun_yaw_trans2b,dt);
fun_yaw_trans2b_d2(Alg_Vlc,dt) = diff(fun_yaw_trans2b_d1,dt);

Par_Layout.Yaw_trans2b_End = double(subs(fun_yaw_trans2b(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_2/80]));
Par_Layout.X_trans2b_End = double(subs(fun_x_trans2b(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_2/80]));
Par_Layout.Y_trans2b_End = double(subs(fun_y_trans2b(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_2/80]));

% % Check
% double(subs(fun_k_trans2a(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_2/2/80]))
% double(subs(fun_k_trans2b(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_2/2/80]))
% double(subs(fun_y_trans2a(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_2/2/80]))-...
% double(subs(fun_y_trans2b(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_2/2/80]))
% double(subs(fun_yaw_trans2a(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_2/2/80]))-...
% double(subs(fun_yaw_trans2b(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_2/2/80]))

%%% 夹直线（以缓和曲线的末端为原点）
syms fun_k_str(Alg_Vlc,dt) fun_k_str_d1(Alg_Vlc,dt) fun_x_str(Alg_Vlc,dt) fun_x_str_d1(Alg_Vlc,dt) fun_x_str_d2(Alg_Vlc,dt) fun_y_str(Alg_Vlc,dt) fun_y_str_d1(Alg_Vlc,dt) fun_y_str_d2(Alg_Vlc,dt) ...
     fun_yaw_str(Alg_Vlc,dt) fun_yaw_str_d1(Alg_Vlc,dt) fun_yaw_str_d2(Alg_Vlc,dt)
fun_k_str(Alg_Vlc,dt) = 0;
fun_k_str_d1(Alg_Vlc,dt) = 0;

fun_yaw_str(Alg_Vlc,dt) = Par_Layout.Yaw_trans2b_End;
fun_yaw_str_d1(Alg_Vlc,dt) = 0;
fun_yaw_str_d2(Alg_Vlc,dt) = 0;

fun_x_str(Alg_Vlc,dt) = (Alg_Vlc*dt) * cos(Par_Layout.Yaw_trans2b_End) + Par_Layout.X_trans2b_End;
fun_x_str_d1(Alg_Vlc,dt) = diff(fun_x_str,dt);
fun_x_str_d2(Alg_Vlc,dt) = diff(fun_x_str_d1,dt);

fun_y_str(Alg_Vlc,dt) = (Alg_Vlc*dt) * sin(Par_Layout.Yaw_trans2b_End) + Par_Layout.Y_trans2b_End;
fun_y_str_d1(Alg_Vlc,dt) = diff(fun_y_str,dt);
fun_y_str_d2(Alg_Vlc,dt) = diff(fun_y_str_d1,dt);

Par_Layout.Yaw_Str_End = double(subs(fun_yaw_str(Alg_Vlc,dt), [Alg_Vlc,dt], [80/3.6, (Par_Layout.L_Str)/(80/3.6)]));
Par_Layout.X_Str_End = double(subs(fun_x_str(Alg_Vlc,dt), [Alg_Vlc,dt], [80/3.6, (Par_Layout.L_Str)/(80/3.6)]));
Par_Layout.Y_Str_End = double(subs(fun_y_str(Alg_Vlc,dt), [Alg_Vlc,dt], [80/3.6, (Par_Layout.L_Str)/(80/3.6)]));

%%% 缓和曲线-3a（以缓和曲线1起点为原点） Par_Layout
syms fun_k_trans3(Alg_Vlc,dt) fun_k_trans3_d1(Alg_Vlc,dt) fun_x_trans3(Alg_Vlc,dt) fun_x_trans3_d1(Alg_Vlc,dt) fun_x_trans3_d2(Alg_Vlc,dt) fun_y_trans3(Alg_Vlc,dt) fun_y_trans3_d1(Alg_Vlc,dt) fun_y_trans3_d2(Alg_Vlc,dt) ...
     fun_yaw_trans3(Alg_Vlc,dt) fun_yaw_trans3_d1(Alg_Vlc,dt) fun_yaw_trans3_d2(Alg_Vlc,dt)
fun_k_trans3a(Alg_Vlc,dt) = k3 + 2*(Alg_Vlc*dt)^2/Par_Layout.L_trans_3^2*(k-2*k3);
fun_k_trans3a_d1(Alg_Vlc,dt) = diff(fun_k_trans3a,dt);

fun_x_trans3a(Alg_Vlc,dt) = (Alg_Vlc*dt)+Par_Layout.X_Str_End;
fun_x_trans3a_d1(Alg_Vlc,dt) = diff(fun_x_trans3a,dt);
fun_x_trans3a_d2(Alg_Vlc,dt) = diff(fun_x_trans3a_d1,dt);

syms C1_trans3a C2_trans3a
fun_yaw_trans3a(Alg_Vlc,dt,C1_trans3a) = k3*(Alg_Vlc*dt) + 2*(Alg_Vlc*dt)^3/(3*Par_Layout.L_trans_3^2)*(k-2*k3)+C1_trans3a;
Equa(Alg_Vlc,dt,C1_trans3a) = fun_yaw_trans3a(Alg_Vlc,dt,C1_trans3a)-Par_Layout.Yaw_Str_End;
C1_trans3a = double(solve(subs(Equa(Alg_Vlc,dt,C1_trans3a), [Alg_Vlc,dt], [80, 0/80])));

fun_yaw_trans3a(Alg_Vlc,dt) = k3*(Alg_Vlc*dt) + 2*(Alg_Vlc*dt)^3/(3*Par_Layout.L_trans_3^2)*(k-2*k3)+C1_trans3a;
fun_yaw_trans3a_d1(Alg_Vlc,dt) = diff(fun_yaw_trans3a,dt);
fun_yaw_trans3a_d2(Alg_Vlc,dt) = diff(fun_yaw_trans3a_d1,dt);

fun_y_trans3a(Alg_Vlc,dt,C2_trans3a) = 1/2*k3*(Alg_Vlc*dt)^2 + (Alg_Vlc*dt)^4*(k-2*k3)/(6*Par_Layout.L_trans_3^2)+...
                                       C1_trans3a*(Alg_Vlc*dt)+C2_trans3a;
Equa(Alg_Vlc,dt,C2_trans3a) = fun_y_trans3a(Alg_Vlc,dt,C2_trans3a)-Par_Layout.Y_Str_End;
C2_trans3a = double(solve(subs(Equa(Alg_Vlc,dt,C2_trans3a), [Alg_Vlc,dt], [80, 0/80])));

fun_y_trans3a(Alg_Vlc,dt) = 1/2*k3*(Alg_Vlc*dt)^2 + (Alg_Vlc*dt)^4*(k-2*k3)/(6*Par_Layout.L_trans_3^2)+...
                            C1_trans3a*(Alg_Vlc*dt)+C2_trans3a;
fun_y_trans3a_d1(Alg_Vlc,dt) = diff(fun_y_trans3a,dt);
fun_y_trans3a_d2(Alg_Vlc,dt) = diff(fun_y_trans3a_d1,dt);

Par_Layout.Yaw_trans3a_End = double(subs(fun_yaw_trans3a(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_3/2/80]));
Par_Layout.X_trans3a_End = double(subs(fun_x_trans3a(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_3/2/80]));
Par_Layout.Y_trans3a_End = double(subs(fun_y_trans3a(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_3/2/80]));

% double(subs(fun_k_trans3,  [Alg_Vlc,dt], [80/3.6, 0.2/80*3.6]))';

%%% 缓和曲线-3b（以缓和曲线1起点为原点）
syms fun_k_trans3(Alg_Vlc,dt) fun_k_trans3_d1(Alg_Vlc,dt) fun_x_trans3(Alg_Vlc,dt) fun_x_trans3_d1(Alg_Vlc,dt) fun_x_trans3_d2(Alg_Vlc,dt) fun_y_trans3(Alg_Vlc,dt) fun_y_trans3_d1(Alg_Vlc,dt) fun_y_trans3_d2(Alg_Vlc,dt) ...
     fun_yaw_trans3(Alg_Vlc,dt) fun_yaw_trans3_d1(Alg_Vlc,dt) fun_yaw_trans3_d2(Alg_Vlc,dt)
fun_k_trans3b(Alg_Vlc,dt) = k3 + 2/Par_Layout.L_trans_3^2*(2*Par_Layout.L_trans_3*(Alg_Vlc*dt)-(Alg_Vlc*dt)^2-Par_Layout.L_trans_3^2/2)*(k-2*k3);
fun_k_trans3b_d1(Alg_Vlc,dt) = diff(fun_k_trans3b,dt);

fun_x_trans3b(Alg_Vlc,dt) = fun_x_trans3a(Alg_Vlc,dt);
fun_x_trans3b_d1(Alg_Vlc,dt) = fun_x_trans3a_d1(Alg_Vlc,dt);
fun_x_trans3b_d2(Alg_Vlc,dt) = fun_x_trans3a_d2(Alg_Vlc,dt);

syms C1_trans3b C2_trans3b
fun_yaw_trans3b(Alg_Vlc,dt,C1_trans3b) = k3*(Alg_Vlc*dt)+2/(Par_Layout.L_trans_3^2)*(Par_Layout.L_trans_3*(Alg_Vlc*dt)^2-1/3*(Alg_Vlc*dt)^3-(Par_Layout.L_trans_3^2)/2*(Alg_Vlc*dt))*(k-2*k3)+C1_trans3b;
Equa(Alg_Vlc,dt,C1_trans3b) = fun_yaw_trans3b(Alg_Vlc,dt,C1_trans3b) - Par_Layout.Yaw_trans3a_End;
C1_trans3b = double(solve(subs(Equa(Alg_Vlc,dt,C1_trans3b), [Alg_Vlc,dt], [80, Par_Layout.L_trans_3/2/80])));

fun_yaw_trans3b(Alg_Vlc,dt) = k3*(Alg_Vlc*dt)+2/(Par_Layout.L_trans_3^2)*(Par_Layout.L_trans_3*(Alg_Vlc*dt)^2-1/3*(Alg_Vlc*dt)^3-(Par_Layout.L_trans_3^2)/2*(Alg_Vlc*dt))*(k-2*k3)+C1_trans3b;
fun_yaw_trans3b_d1(Alg_Vlc,dt) = diff(fun_yaw_trans3b,dt);
fun_yaw_trans3b_d2(Alg_Vlc,dt) = diff(fun_yaw_trans3b_d1,dt);

fun_y_trans3b(Alg_Vlc,dt,C2_trans3b) = 1/2*k3*(Alg_Vlc*dt)^2 + 2/(Par_Layout.L_trans_3^2)*(1/3*Par_Layout.L_trans_3*(Alg_Vlc*dt)^3-1/12*(Alg_Vlc*dt)^4-Par_Layout.L_trans_3^2/4*(Alg_Vlc*dt)^2)*(k-2*k3) + ...
                                       C1_trans3b*(Alg_Vlc*dt) + C2_trans3b;
Equa(Alg_Vlc,dt,C2_trans3b) = fun_y_trans3b(Alg_Vlc,dt,C2_trans3b) - Par_Layout.Y_trans3a_End;
C2_trans3b = double(solve(subs(Equa(Alg_Vlc,dt,C2_trans3b), [Alg_Vlc,dt], [80, Par_Layout.L_trans_3/2/80])));
                                   
fun_y_trans3b(Alg_Vlc,dt) = 1/2*k3*(Alg_Vlc*dt)^2 + 2/(Par_Layout.L_trans_3^2)*(1/3*Par_Layout.L_trans_3*(Alg_Vlc*dt)^3-1/12*(Alg_Vlc*dt)^4-Par_Layout.L_trans_3^2/4*(Alg_Vlc*dt)^2)*(k-2*k3) + ...
                            C1_trans3b*(Alg_Vlc*dt) + C2_trans3b;
fun_y_trans3b_d1(Alg_Vlc,dt) = diff(fun_y_trans3b,dt);
fun_y_trans3b_d2(Alg_Vlc,dt) = diff(fun_y_trans3b_d1,dt);

Par_Layout.Yaw_trans3b_End = double(subs(fun_yaw_trans3b(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_3/80]));
Par_Layout.X_trans3b_End = double(subs(fun_x_trans3b(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_3/80]));
Par_Layout.Y_trans3b_End = double(subs(fun_y_trans3b(Alg_Vlc,dt), [Alg_Vlc,dt], [80, Par_Layout.L_trans_3/80]));

%%% 圆曲线（以圆曲线的起点为原点）
syms fun_k_curve(Alg_Vlc,dt) fun_k_curve_d1(Alg_Vlc,dt) fun_x_curve(Alg_Vlc,dt) fun_x_curve_d1(Alg_Vlc,dt) fun_x_curve_d2(Alg_Vlc,dt) fun_y_curve(Alg_Vlc,dt) fun_y_curve_d1(Alg_Vlc,dt) fun_y_curve_d2(Alg_Vlc,dt) ...
     fun_yaw_curve(Alg_Vlc,dt) fun_yaw_curve_d1(Alg_Vlc,dt) fun_yaw_curve_d2(Alg_Vlc,dt)
fun_k_curve(Alg_Vlc,dt) = k;
fun_k_curve_d1(Alg_Vlc,dt) = 0;

fun_yaw_curve(Alg_Vlc,dt) = (Alg_Vlc*dt)/Par_Layout.R + Par_Layout.Yaw_trans3b_End;
fun_yaw_curve_d1(Alg_Vlc,dt) = diff(fun_yaw_curve,dt);
fun_yaw_curve_d2(Alg_Vlc,dt) = diff(fun_yaw_curve_d1,dt);

fun_x_curve(Alg_Vlc,dt) = Par_Layout.R*sin((Alg_Vlc*dt)/Par_Layout.R + Par_Layout.Yaw_trans3b_End) - ...
                          Par_Layout.R*sin(Par_Layout.Yaw_trans3b_End) + Par_Layout.X_trans3b_End;
fun_x_curve_d1(Alg_Vlc,dt) = diff(fun_x_curve,dt);
fun_x_curve_d2(Alg_Vlc,dt) = diff(fun_x_curve_d1,dt);

fun_y_curve(Alg_Vlc,dt) = Par_Layout.R*(1-cos((Alg_Vlc*dt)/Par_Layout.R + Par_Layout.Yaw_trans3b_End)) - ...
                          Par_Layout.R*(1-cos(Par_Layout.Yaw_trans3b_End)) + Par_Layout.Y_trans3b_End;
fun_y_curve_d1(Alg_Vlc,dt) = diff(fun_y_curve,dt);
fun_y_curve_d2(Alg_Vlc,dt) = diff(fun_y_curve_d1,dt);

%%% 存储
Par_Layout.fun_k_trans1a = fun_k_trans1a;
Par_Layout.fun_k_trans1a_d1 = fun_k_trans1a_d1;
Par_Layout.fun_x_trans1a = fun_x_trans1a;
Par_Layout.fun_x_trans1a_d1 = fun_x_trans1a_d1;
Par_Layout.fun_x_trans1a_d2 = fun_x_trans1a_d2;
Par_Layout.fun_y_trans1a = fun_y_trans1a;
Par_Layout.fun_y_trans1a_d1 = fun_y_trans1a_d1;
Par_Layout.fun_y_trans1a_d2 = fun_y_trans1a_d2;
Par_Layout.fun_yaw_trans1a = fun_yaw_trans1a;
Par_Layout.fun_yaw_trans1a_d1 = fun_yaw_trans1a_d1;
Par_Layout.fun_yaw_trans1a_d2 = fun_yaw_trans1a_d2;

Par_Layout.fun_k_trans1b = fun_k_trans1b;
Par_Layout.fun_k_trans1b_d1 = fun_k_trans1b_d1;
Par_Layout.fun_x_trans1b = fun_x_trans1b;
Par_Layout.fun_x_trans1b_d1 = fun_x_trans1b_d1;
Par_Layout.fun_x_trans1b_d2 = fun_x_trans1b_d2;
Par_Layout.fun_y_trans1b = fun_y_trans1b;
Par_Layout.fun_y_trans1b_d1 = fun_y_trans1b_d1;
Par_Layout.fun_y_trans1b_d2 = fun_y_trans1b_d2;
Par_Layout.fun_yaw_trans1b = fun_yaw_trans1b;
Par_Layout.fun_yaw_trans1b_d1 = fun_yaw_trans1b_d1;
Par_Layout.fun_yaw_trans1b_d2 = fun_yaw_trans1b_d2;

Par_Layout.fun_k_trans2a = fun_k_trans2a;
Par_Layout.fun_k_trans2a_d1 = fun_k_trans2a_d1;
Par_Layout.fun_x_trans2a = fun_x_trans2a;
Par_Layout.fun_x_trans2a_d1 = fun_x_trans2a_d1;
Par_Layout.fun_x_trans2a_d2 = fun_x_trans2a_d2;
Par_Layout.fun_y_trans2a = fun_y_trans2a;
Par_Layout.fun_y_trans2a_d1 = fun_y_trans2a_d1;
Par_Layout.fun_y_trans2a_d2 = fun_y_trans2a_d2;
Par_Layout.fun_yaw_trans2a = fun_yaw_trans2a;
Par_Layout.fun_yaw_trans2a_d1 = fun_yaw_trans2a_d1;
Par_Layout.fun_yaw_trans2a_d2 = fun_yaw_trans2a_d2;

Par_Layout.fun_k_trans2b = fun_k_trans2b;
Par_Layout.fun_k_trans2b_d1 = fun_k_trans2b_d1;
Par_Layout.fun_x_trans2b = fun_x_trans2b;
Par_Layout.fun_x_trans2b_d1 = fun_x_trans2b_d1;
Par_Layout.fun_x_trans2b_d2 = fun_x_trans2b_d2;
Par_Layout.fun_y_trans2b = fun_y_trans2b;
Par_Layout.fun_y_trans2b_d1 = fun_y_trans2b_d1;
Par_Layout.fun_y_trans2b_d2 = fun_y_trans2b_d2;
Par_Layout.fun_yaw_trans2b = fun_yaw_trans2b;
Par_Layout.fun_yaw_trans2b_d1 = fun_yaw_trans2b_d1;
Par_Layout.fun_yaw_trans2b_d2 = fun_yaw_trans2b_d2;

Par_Layout.fun_k_trans3a = fun_k_trans3a;
Par_Layout.fun_k_trans3a_d1 = fun_k_trans3a_d1;
Par_Layout.fun_x_trans3a = fun_x_trans3a;
Par_Layout.fun_x_trans3a_d1 = fun_x_trans3a_d1;
Par_Layout.fun_x_trans3a_d2 = fun_x_trans3a_d2;
Par_Layout.fun_y_trans3a = fun_y_trans3a;
Par_Layout.fun_y_trans3a_d1 = fun_y_trans3a_d1;
Par_Layout.fun_y_trans3a_d2 = fun_y_trans3a_d2;
Par_Layout.fun_yaw_trans3a = fun_yaw_trans3a;
Par_Layout.fun_yaw_trans3a_d1 = fun_yaw_trans3a_d1;
Par_Layout.fun_yaw_trans3a_d2 = fun_yaw_trans3a_d2;

Par_Layout.fun_k_trans3b = fun_k_trans3b;
Par_Layout.fun_k_trans3b_d1 = fun_k_trans3b_d1;
Par_Layout.fun_x_trans3b = fun_x_trans3b;
Par_Layout.fun_x_trans3b_d1 = fun_x_trans3b_d1;
Par_Layout.fun_x_trans3b_d2 = fun_x_trans3b_d2;
Par_Layout.fun_y_trans3b = fun_y_trans3b;
Par_Layout.fun_y_trans3b_d1 = fun_y_trans3b_d1;
Par_Layout.fun_y_trans3b_d2 = fun_y_trans3b_d2;
Par_Layout.fun_yaw_trans3b = fun_yaw_trans3b;
Par_Layout.fun_yaw_trans3b_d1 = fun_yaw_trans3b_d1;
Par_Layout.fun_yaw_trans3b_d2 = fun_yaw_trans3b_d2;

Par_Layout.fun_k_str = fun_k_str;
Par_Layout.fun_k_str_d1 = fun_k_str_d1;
Par_Layout.fun_x_str = fun_x_str;
Par_Layout.fun_x_str_d1 = fun_x_str_d1;
Par_Layout.fun_x_str_d2 = fun_x_str_d2;
Par_Layout.fun_y_str = fun_y_str;
Par_Layout.fun_y_str_d1 = fun_y_str_d1;
Par_Layout.fun_y_str_d2 = fun_y_str_d2;
Par_Layout.fun_yaw_str = fun_yaw_str;
Par_Layout.fun_yaw_str_d1 = fun_yaw_str_d1;
Par_Layout.fun_yaw_str_d2 = fun_yaw_str_d2;

Par_Layout.fun_k_curve = fun_k_curve;
Par_Layout.fun_k_curve_d1 = fun_k_curve_d1;
Par_Layout.fun_x_curve = fun_x_curve;
Par_Layout.fun_x_curve_d1 = fun_x_curve_d1;
Par_Layout.fun_x_curve_d2 = fun_x_curve_d2;
Par_Layout.fun_y_curve = fun_y_curve;
Par_Layout.fun_y_curve_d1 = fun_y_curve_d1;
Par_Layout.fun_y_curve_d2 = fun_y_curve_d2;
Par_Layout.fun_yaw_curve = fun_yaw_curve;
Par_Layout.fun_yaw_curve_d1 = fun_yaw_curve_d1;
Par_Layout.fun_yaw_curve_d2 = fun_yaw_curve_d2;

%%% 绘图
clear pos_Radius_Vehicle pos_yaw_track pos_Body_global vel_Body_global vel_yaw_track k_curvature_d1
if Choose_Plot == 1
    Mileage = (54.5:0.01:55.5)';
    Vlc = 80/3.6;
    for i = 1:1:length(Mileage)
        pos_Radius_Vehicle(i,1) = Mileage(i,1);
        % STR-1
        if Mileage(i,1) <= -Par_Layout.L_trans_1/2+Par_Layout.Curve_start_1
            pos_yaw_track(i,1) = 0;
            pos_Radius_Vehicle(i,2:3) = [Par_Layout.R1 pos_yaw_track(i,1)];
            pos_Body_global(i,1:3) = [pos_Radius_Vehicle(i,1), 0, 0];
            vel_Body_global(i,1:3) = [Vlc,0,0];
            vel_yaw_track(i,1) = 0;
            k_curvature_d1(i,1) = 0;
            
        % Trans-1a
        elseif Mileage(i,1) > -Par_Layout.L_trans_1/2+Par_Layout.Curve_start_1 && Mileage(i,1) <= Par_Layout.Curve_start_1
            ds = Mileage(i,1)-(-Par_Layout.L_trans_1/2+Par_Layout.Curve_start_1);
            k_curvature = double(subs(fun_k_trans1a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            pos_yaw_track(i,1) = double(subs(fun_yaw_trans1a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            pos_Body_global(i,1:3) = [double(subs(fun_x_trans1a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ... 
                                      double(subs(fun_y_trans1a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
            pos_Radius_Vehicle(i,2:3) = [1/k_curvature, pos_yaw_track(i,1)];
            vel_Body_global(i,1:3) = [double(subs(fun_x_trans1a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ... 
                                      double(subs(fun_y_trans1a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
            vel_yaw_track(i,1) = double(subs(fun_yaw_trans1a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            k_curvature_d1(i,1) = double(subs(fun_k_trans1a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));

        % Trans-1b
        elseif Mileage(i,1) > Par_Layout.Curve_start_1 && Mileage(i,1) <= Par_Layout.L_trans_1/2+Par_Layout.Curve_start_1
            ds = Mileage(i,1)-(-Par_Layout.L_trans_1/2+Par_Layout.Curve_start_1);
            k_curvature = double(subs(fun_k_trans1b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            pos_yaw_track(i,1) = double(subs(fun_yaw_trans1b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            pos_Body_global(i,1:3) = [double(subs(fun_x_trans1b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ... 
                                      double(subs(fun_y_trans1b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
            pos_Radius_Vehicle(i,2:3) = [1/k_curvature, pos_yaw_track(i,1)];
            vel_Body_global(i,1:3) = [double(subs(fun_x_trans1b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ... 
                                      double(subs(fun_y_trans1b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
            vel_yaw_track(i,1) = double(subs(fun_yaw_trans1b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            k_curvature_d1(i,1) = double(subs(fun_k_trans1b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
        
        % Trans-2a
        elseif Mileage(i,1) > -Par_Layout.L_trans_2/2+Par_Layout.Curve_start_2 && Mileage(i,1) <= Par_Layout.Curve_start_2
            ds = Mileage(i,1)-(-Par_Layout.L_trans_2/2+Par_Layout.Curve_start_2);
            k_curvature = double(subs(fun_k_trans2a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            pos_yaw_track(i,1) = double(subs(fun_yaw_trans2a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            pos_Body_global(i,1:3) = [double(subs(fun_x_trans2a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ... 
                                      double(subs(fun_y_trans2a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
            pos_Radius_Vehicle(i,2:3) = [1/k_curvature, pos_yaw_track(i,1)];
            vel_Body_global(i,1:3) = [double(subs(fun_x_trans2a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ... 
                                      double(subs(fun_y_trans2a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
            vel_yaw_track(i,1) = double(subs(fun_yaw_trans2a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            k_curvature_d1(i,1) = double(subs(fun_k_trans2a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));

        % Trans-2b
        elseif Mileage(i,1) > Par_Layout.Curve_start_2 && Mileage(i,1) <= Par_Layout.L_trans_2/2+Par_Layout.Curve_start_2
            ds = Mileage(i,1)-(-Par_Layout.L_trans_2/2+Par_Layout.Curve_start_2);
            k_curvature = double(subs(fun_k_trans2b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            pos_yaw_track(i,1) = double(subs(fun_yaw_trans2b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            pos_Body_global(i,1:3) = [double(subs(fun_x_trans2b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ... 
                                      double(subs(fun_y_trans2b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
            pos_Radius_Vehicle(i,2:3) = [1/k_curvature, pos_yaw_track(i,1)];
            vel_Body_global(i,1:3) = [double(subs(fun_x_trans2b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ... 
                                      double(subs(fun_y_trans2b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
            vel_yaw_track(i,1) = double(subs(fun_yaw_trans2b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            k_curvature_d1(i,1) = double(subs(fun_k_trans2b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            
        % STR-2
        elseif Mileage(i,1) > Par_Layout.L_trans_2/2+Par_Layout.Curve_start_2 && Mileage(i,1) <= Par_Layout.Curve_start_3-Par_Layout.L_trans_3/2
            ds = Mileage(i,1)-(Par_Layout.L_trans_2/2+Par_Layout.Curve_start_2);
            k_curvature = double(subs(fun_k_str(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            pos_yaw_track(i,1) = double(subs(fun_yaw_str(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            pos_Body_global(i,1:3) = [double(subs(fun_x_str(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ... 
                                      double(subs(fun_y_str(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
            pos_Radius_Vehicle(i,2:3) = [1/k_curvature, pos_yaw_track(i,1)];
            vel_Body_global(i,1:3) = [double(subs(fun_x_str_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ... 
                                      double(subs(fun_y_str_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
            vel_yaw_track(i,1) = double(subs(fun_yaw_str_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            k_curvature_d1(i,1) = double(subs(fun_k_str_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
         
        % Trans-3a
        elseif Mileage(i,1) > Par_Layout.Curve_start_3-Par_Layout.L_trans_3/2 && Mileage(i,1) <= Par_Layout.Curve_start_3
            ds = Mileage(i,1)-(Par_Layout.Curve_start_3-Par_Layout.L_trans_3/2);
            k_curvature = double(subs(fun_k_trans3a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            pos_yaw_track(i,1) = double(subs(fun_yaw_trans3a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            pos_Body_global(i,1:3) = [double(subs(fun_x_trans3a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ... 
                                      double(subs(fun_y_trans3a(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
            pos_Radius_Vehicle(i,2:3) = [1/k_curvature, pos_yaw_track(i,1)];
            vel_Body_global(i,1:3) = [double(subs(fun_x_trans3a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ... 
                                      double(subs(fun_y_trans3a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
            vel_yaw_track(i,1) = double(subs(fun_yaw_trans3a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            k_curvature_d1(i,1) = double(subs(fun_k_trans3a_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            
        % Trans-3b
        elseif Mileage(i,1) > Par_Layout.Curve_start_3 && Mileage(i,1) <= Par_Layout.Curve_start_3+Par_Layout.L_trans_3/2
            ds = Mileage(i,1)-(Par_Layout.Curve_start_3-Par_Layout.L_trans_3/2);
            k_curvature = double(subs(fun_k_trans3b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            pos_yaw_track(i,1) = double(subs(fun_yaw_trans3b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            pos_Body_global(i,1:3) = [double(subs(fun_x_trans3b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ... 
                                      double(subs(fun_y_trans3b(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
            pos_Radius_Vehicle(i,2:3) = [1/k_curvature, pos_yaw_track(i,1)];
            vel_Body_global(i,1:3) = [double(subs(fun_x_trans3b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ... 
                                      double(subs(fun_y_trans3b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
            vel_yaw_track(i,1) = double(subs(fun_yaw_trans3b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            k_curvature_d1(i,1) = double(subs(fun_k_trans3b_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            
        % Curve
        elseif Mileage(i,1) > Par_Layout.Curve_start_3+Par_Layout.L_trans_3/2
            ds = Mileage(i,1)-(Par_Layout.Curve_start_3+Par_Layout.L_trans_3/2);
            k_curvature = double(subs(fun_k_curve(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            pos_yaw_track(i,1) = double(subs(fun_yaw_curve(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            pos_Body_global(i,1:3) = [double(subs(fun_x_curve(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ... 
                                      double(subs(fun_y_curve(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
            pos_Radius_Vehicle(i,2:3) = [1/k_curvature, pos_yaw_track(i,1)];
            vel_Body_global(i,1:3) = [double(subs(fun_x_curve_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])) ... 
                                      double(subs(fun_y_curve_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc])), 0];
            vel_yaw_track(i,1) = double(subs(fun_yaw_curve_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            k_curvature_d1(i,1) = double(subs(fun_k_curve_d1(Alg_Vlc,dt), [Alg_Vlc,dt], [Vlc,ds/Vlc]));
            
        end
    end

    figure(21); clf
    subplot(3,2,1)
    plot(pos_Body_global(:,1), pos_Body_global(:,2)); grid on
    set(gca,'ydir','reverse'); xlabel('X(m)'); ylabel('Y(m)');
    subplot(3,2,3)
    plot(Mileage(:,1), 1./pos_Radius_Vehicle(:,2)); grid on
    xlabel('s(m)'); ylabel('Curvature(1/m)');
    % xlabel('s(m)'); ylabel('Radius(m)');
    subplot(3,2,5)
    plot(Mileage(:,1), pos_Radius_Vehicle(:,3)); grid on
    xlabel('s(m)'); ylabel('Yaw(rad)');
    
    subplot(3,2,2)
    plot(Mileage(:,1), vel_Body_global(:,2)); grid on
    set(gca,'ydir','reverse'); xlabel('s(m)'); ylabel('Vel-Y(m/s)');
    subplot(3,2,4)
    plot(Mileage(:,1), k_curvature_d1(:,1)); grid on
    xlabel('s(m)'); ylabel('Vel-Curvature(1/(ms))');
    subplot(3,2,6)
    plot(Mileage(:,1), vel_yaw_track(:,1)); grid on
    xlabel('s(m)'); ylabel('Vel-Yaw(rad/s)');
    
end
