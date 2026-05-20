function [M_track,K_track,C_track] = Matrix_RT(Mr1,Mr2,Mr3,Mr4,Ms,Jt,Krz1,Crz1,Krz2,Crz2,Krz3,Crz3,Krz4,Crz4,...
Kry1,Cry1,Kry2,Cry2,Kry3,Cry3,Kry4,Cry4,Lt,Kzs,Czs,Kys,Cys,Nw)

%% ============ 轨道质量、刚度、阻尼矩阵
% 轨道质量（位移变分为行，加速度为列）
Mgd = zeros(11,11);
for i = 1:1:11
    if i == 1 || i == 2
        temp_Mass = Mr1;
    elseif i == 3 || i == 4
        temp_Mass = Mr2;
    elseif i == 5 || i == 6
        temp_Mass = Mr3;
    elseif i == 7 || i == 8
        temp_Mass = Mr4;
    elseif i == 9 || i == 10
        temp_Mass = Ms;
    else
        temp_Mass = Jt;
    end
   Mgd(i,i) = temp_Mass;
end

% 轨道刚度（位移变分为行，位移为列）
Kgd = zeros(11,11);
for i = 1:2
   if i == 1
       temp_K = Krz1;
   elseif i ==2
       temp_K = Krz2;
   end
   Kgd(2*i-1,2*i-1) = Kgd(2*i-1,2*i-1) + temp_K;
   Kgd(2*i-1,9) = Kgd(2*i-1,9) - temp_K;
   Kgd(2*i-1,11) = Kgd(2*i-1,11) + Lt*temp_K;
   Kgd(9,2*i-1) = Kgd(9,2*i-1) - temp_K;
   Kgd(9,9) = Kgd(9,9) + temp_K;
   Kgd(9,11) = Kgd(9,11) - Lt*temp_K;
   Kgd(11,2*i-1) = Kgd(11,2*i-1) + Lt*temp_K;
   Kgd(11,9) = Kgd(11,9) - Lt*temp_K;
   Kgd(11,11) = Kgd(11,11) + Lt*Lt*temp_K;   
end
for i = 3:4
   if i == 3
       temp_K = Krz3;
   elseif i == 4
       temp_K = Krz4;
   end
   Kgd(2*i-1,2*i-1) = Kgd(2*i-1,2*i-1) + temp_K;
   Kgd(2*i-1,9) = Kgd(2*i-1,9) - temp_K;
   Kgd(2*i-1,11) = Kgd(2*i-1,11) - Lt*temp_K;
   Kgd(9,2*i-1) = Kgd(9,2*i-1) - temp_K;
   Kgd(9,9) = Kgd(9,9) + temp_K;
   Kgd(9,11) = Kgd(9,11) + Lt*temp_K;
   Kgd(11,2*i-1) = Kgd(11,2*i-1) - Lt*temp_K;
   Kgd(11,9) = Kgd(11,9) + Lt*temp_K;
   Kgd(11,11) = Kgd(11,11) + Lt*Lt*temp_K;   
end
for i = 1:4
    if i == 1
        temp_K = Kry1;
    elseif i ==2
        temp_K = Kry2;
    elseif i == 3
        temp_K = Kry3;
    else
        temp_K = Kry4;
    end
    Kgd(2*i,2*i) = Kgd(2*i,2*i) + temp_K;
    Kgd(2*i,10) = Kgd(2*i,10) - temp_K;
    Kgd(10,2*i) = Kgd(10,2*i) - temp_K;
    Kgd(10,10) = Kgd(10,10) + temp_K;    
end
for i = 1:1:2
   Kgd(9,9) = Kgd(9,9) + Kzs;
   Kgd(9,11) = Kgd(9,11) + Lt*Kzs*(-1)^i;
   Kgd(11,9) = Kgd(11,9) + Lt*Kzs*(-1)^i;
   Kgd(11,11) = Kgd(11,11) + Lt*Lt*Kzs;
end
Kgd(10,10) = Kgd(10,10) + Kys;

%轨道阻尼（位移变分为行，速度为列）
Cgd = zeros(11,11);
for i = 1:2
   if i == 1
       temp_C = Crz1;
   elseif i ==2
       temp_C = Crz2;
   end
   Cgd(2*i-1,2*i-1) = Cgd(2*i-1,2*i-1) + temp_C;
   Cgd(2*i-1,9) = Cgd(2*i-1,9) - temp_C;
   Cgd(2*i-1,11) = Cgd(2*i-1,11) + Lt*temp_C;
   Cgd(9,2*i-1) = Cgd(9,2*i-1) - temp_C;
   Cgd(9,9) = Cgd(9,9) + temp_C;
   Cgd(9,11) = Cgd(9,11) - Lt*temp_C;
   Cgd(11,2*i-1) = Cgd(11,2*i-1) + Lt*temp_C;
   Cgd(11,9) = Cgd(11,9) - Lt*temp_C;
   Cgd(11,11) = Cgd(11,11) + Lt*Lt*temp_C;   
end
for i = 3:4
   if i == 3
       temp_C = Crz3;
   elseif i == 4
       temp_C = Crz4;
   end
   Cgd(2*i-1,2*i-1) = Cgd(2*i-1,2*i-1) + temp_C;
   Cgd(2*i-1,9) = Cgd(2*i-1,9) - temp_C;
   Cgd(2*i-1,11) = Cgd(2*i-1,11) - Lt*temp_C;
   Cgd(9,2*i-1) = Cgd(9,2*i-1) - temp_C;
   Cgd(9,9) = Cgd(9,9) + temp_C;
   Cgd(9,11) = Cgd(9,11) + Lt*temp_C;
   Cgd(11,2*i-1) = Cgd(11,2*i-1) - Lt*temp_C;
   Cgd(11,9) = Cgd(11,9) + Lt*temp_C;
   Cgd(11,11) = Cgd(11,11) + Lt*Lt*temp_C;   
end
for i = 1:4
    if i == 1
        temp_C = Cry1;
    elseif i ==2
        temp_C = Cry2;
    elseif i == 3
        temp_C = Cry3;
    else
        temp_C = Cry4;
    end
    Cgd(2*i,2*i) = Cgd(2*i,2*i) + temp_C;
    Cgd(2*i,10) = Cgd(2*i,10) - temp_C;
    Cgd(10,2*i) = Cgd(10,2*i) - temp_C;
    Cgd(10,10) = Cgd(10,10) + temp_C;    
end
for i = 1:1:2
   Cgd(9,9) = Cgd(9,9) + Czs;
   Cgd(9,11) = Cgd(9,11) + Lt*Czs*(-1)^i;
   Cgd(11,9) = Cgd(11,9) + Lt*Czs*(-1)^i;
   Cgd(11,11) = Cgd(11,11) + Lt*Lt*Czs;
end
Cgd(10,10) = Cgd(10,10) + Cys;

%%% 合并为轨道矩阵
N_track = 11*Nw;
M_track = zeros(N_track,N_track);
K_track = zeros(N_track,N_track);
C_track = zeros(N_track,N_track);
for kk = 1:1:4
    M_track(11*(kk-1)+1:11*kk,11*(kk-1)+1:11*kk) = Mgd;
    K_track(11*(kk-1)+1:11*kk,11*(kk-1)+1:11*kk) = Kgd;
    C_track(11*(kk-1)+1:11*kk,11*(kk-1)+1:11*kk) = Cgd;
end


