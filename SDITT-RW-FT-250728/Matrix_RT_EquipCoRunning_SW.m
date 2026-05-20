function [M_track, K_track, C_track, InpPar] = Matrix_RT_EquipCoRunning_SW(InpPar, Rail, Baseplate)

%% ============ 轨道质量、刚度、阻尼矩阵
% Dof 1-8:  z_01, y_01, z_02, y_02, z_03, y_03, z_04, y_04
% Dof 9-16: z_r1, y_r1, z_r2, y_r2, z_r3, y_r3, z_r4, y_r4
% Dof 17-18: z_bL, z_bR
N_RT_OneWS = 4*2+4*2+2*1;
Nw = InpPar.Nw;

clear Mgd Kgd Cgd

%% 轨道质量（位移变分为行，加速度为列）
Mgd = zeros(N_RT_OneWS,N_RT_OneWS);
Mass_DOF = [repmat(2.5,1,8), ...
                      repmat(Rail.Mass.zjbg,1,2), repmat(Rail.Mass.zjg_zgyg,1,2), repmat(Rail.Mass.zjg_zgyg,1,2), repmat(Rail.Mass.qjbg,1,2), ...
                      Baseplate.Mass.Switch_L, Baseplate.Mass.Switch_R];
for i = 1:1:N_RT_OneWS
   Mgd(i,i) = Mass_DOF(i);
end

%% 轨道刚度
Kgd = zeros(N_RT_OneWS,N_RT_OneWS);
Type_i1 = {'zjbg', 'zjg_zgyg', 'zjg_zgyg', 'qjbg'};
for i1 = 1:1:4
    T1 = Type_i1{i1};
    Kgd(2*(i1-1)+1,2*(i1-1)+1) = Kgd(2*(i1-1)+1,2*(i1-1)+1) + Rail.krz_co.(T1);
    Kgd(2*(i1-1)+1,2*(i1-1)+1+8) = Kgd(2*(i1-1)+1,2*(i1-1)+1+8) - Rail.krz_co.(T1);
    Kgd(2*(i1-1)+1+8,2*(i1-1)+1) = Kgd(2*(i1-1)+1+8,2*(i1-1)+1) - Rail.krz_co.(T1);
    Kgd(2*(i1-1)+1+8,2*(i1-1)+1+8) = Kgd(2*(i1-1)+1+8,2*(i1-1)+1+8) + Rail.krz_co.(T1);

    Kgd(2*(i1-1)+2,2*(i1-1)+2) = Kgd(2*(i1-1)+2,2*(i1-1)+2) + Rail.kry_co.(T1);
    Kgd(2*(i1-1)+2,2*(i1-1)+2+8) = Kgd(2*(i1-1)+2,2*(i1-1)+2+8) - Rail.kry_co.(T1);
    Kgd(2*(i1-1)+2+8,2*(i1-1)+2) = Kgd(2*(i1-1)+2+8,2*(i1-1)+2) - Rail.kry_co.(T1);
    Kgd(2*(i1-1)+2+8,2*(i1-1)+2+8) = Kgd(2*(i1-1)+2+8,2*(i1-1)+2+8) + Rail.kry_co.(T1);

    if i1<=2
        pos_bz = N_RT_OneWS-1;
    else
        pos_bz = N_RT_OneWS;
    end
    Kgd(2*(i1-1)+1+8,2*(i1-1)+1+8) = Kgd(2*(i1-1)+1+8,2*(i1-1)+1+8) + Rail.krz_co.(T1);
    Kgd(2*(i1-1)+1+8,pos_bz) = Kgd(2*(i1-1)+1+8,pos_bz) - Rail.krz_co.(T1);
    Kgd(pos_bz,2*(i1-1)+1+8) = Kgd(pos_bz,2*(i1-1)+1+8) - Rail.krz_co.(T1);
    Kgd(pos_bz,pos_bz) = Kgd(pos_bz,pos_bz) + Rail.krz_co.(T1);

    Kgd(2*(i1-1)+2+8,2*(i1-1)+2+8) = Kgd(2*(i1-1)+2+8,2*(i1-1)+2+8) + Rail.kry_co.(T1);
end
Kgd(N_RT_OneWS-1, N_RT_OneWS-1) = Kgd(N_RT_OneWS-1, N_RT_OneWS-1) + Baseplate.kbz_co.Switch_L;
Kgd(N_RT_OneWS, N_RT_OneWS) = Kgd(N_RT_OneWS, N_RT_OneWS) + Baseplate.kbz_co.Switch_R;

%% 轨道阻尼
Cgd = zeros(N_RT_OneWS,N_RT_OneWS);
Type_i1 = {'zjbg', 'zjg_zgyg', 'zjg_zgyg', 'qjbg'};
for i1 = 1:1:4
    T1 = Type_i1{i1};
    Cgd(2*(i1-1)+1,2*(i1-1)+1) = Cgd(2*(i1-1)+1,2*(i1-1)+1) + Rail.C0z.(T1);
    Cgd(2*(i1-1)+1,2*(i1-1)+1+8) = Cgd(2*(i1-1)+1,2*(i1-1)+1+8) - Rail.C0z.(T1);
    Cgd(2*(i1-1)+1+8,2*(i1-1)+1) = Cgd(2*(i1-1)+1+8,2*(i1-1)+1) - Rail.C0z.(T1);
    Cgd(2*(i1-1)+1+8,2*(i1-1)+1+8) = Cgd(2*(i1-1)+1+8,2*(i1-1)+1+8) + Rail.C0z.(T1);

    Cgd(2*(i1-1)+2,2*(i1-1)+2) = Cgd(2*(i1-1)+2,2*(i1-1)+2) + Rail.C0y.(T1);
    Cgd(2*(i1-1)+2,2*(i1-1)+2+8) = Cgd(2*(i1-1)+2,2*(i1-1)+2+8) - Rail.C0y.(T1);
    Cgd(2*(i1-1)+2+8,2*(i1-1)+2) = Cgd(2*(i1-1)+2+8,2*(i1-1)+2) - Rail.C0y.(T1);
    Cgd(2*(i1-1)+2+8,2*(i1-1)+2+8) = Cgd(2*(i1-1)+2+8,2*(i1-1)+2+8) + Rail.C0y.(T1);

    if i1<=2
        pos_bz = N_RT_OneWS-1;
    else
        pos_bz = N_RT_OneWS;
    end
    Cgd(2*(i1-1)+1+8,2*(i1-1)+1+8) = Cgd(2*(i1-1)+1+8,2*(i1-1)+1+8) + Rail.crz_co.(T1);
    Cgd(2*(i1-1)+1+8,pos_bz) = Cgd(2*(i1-1)+1+8,pos_bz) - Rail.crz_co.(T1);
    Cgd(pos_bz,2*(i1-1)+1+8) = Cgd(pos_bz,2*(i1-1)+1+8) - Rail.crz_co.(T1);
    Cgd(pos_bz,pos_bz) = Cgd(pos_bz,pos_bz) + Rail.crz_co.(T1);

    Cgd(2*(i1-1)+2+8,2*(i1-1)+2+8) = Cgd(2*(i1-1)+2+8,2*(i1-1)+2+8) + Rail.cry_co.(T1);
end
Cgd(N_RT_OneWS-1, N_RT_OneWS-1) = Cgd(N_RT_OneWS-1, N_RT_OneWS-1) + Baseplate.cbz_co.Switch_L;
Cgd(N_RT_OneWS, N_RT_OneWS) = Cgd(N_RT_OneWS, N_RT_OneWS) + Baseplate.cbz_co.Switch_R;

%% 合并为轨道矩阵
N_track = N_RT_OneWS*Nw;
M_track = zeros(N_track,N_track);
K_track = zeros(N_track,N_track);
C_track = zeros(N_track,N_track);
for kk = 1:1:Nw
    p1 = N_RT_OneWS*(kk-1)+1;
    p2 = N_RT_OneWS*kk;
    M_track(p1:p2,p1:p2) = Mgd;
    K_track(p1:p2,p1:p2) = Kgd;
    C_track(p1:p2,p1:p2) = Cgd;
end
InpPar.N_track = N_track;

