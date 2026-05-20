%% 构建多刚体车辆系统 M、C、K 矩阵
% 230403: 补充了一系垂向减振器（串联弹簧）
% 230404: 修正了一系垂向减振器（串联弹簧），补充了抗蛇形减振器（串联弹簧），对应参数 Par_Vehicle_CRH380A_v3
% 230409: 简化了一系和二系悬挂的 Clc 矩阵表示方式；
% 230409: 修正了 K_Jx, K_Jy, K_DPz 对应刚度矩阵，补充了 K_DPy (串联弹簧), K_Tx, K_ROTx 对应刚度/阻尼矩阵，对应参数 Par_Vehicle_CRH380A_v4

function [Mlc, Klc, Clc, Clc_0] = Matrix_Vehicle_RW_230409(InpPar, Par_Vehicle)

% global Par_Vehicle InpPar.N_RV

Mw = Par_Vehicle.Mw;
Jwx = Par_Vehicle.Jwx;
Jwy = Par_Vehicle.Jwy;
Jwz = Par_Vehicle.Jwz;
Mb = Par_Vehicle.Mb;
Jbx = Par_Vehicle.Jbx;
Jby = Par_Vehicle.Jby;
Jbz = Par_Vehicle.Jbz;
Mc = Par_Vehicle.Mc;
Jcx = Par_Vehicle.Jcx;
Jcy = Par_Vehicle.Jcy;
Jcz = Par_Vehicle.Jcz;

K1x = Par_Vehicle.K1x;      C1x = Par_Vehicle.C1x;
K1y = Par_Vehicle.K1y;      C1y = Par_Vehicle.C1y;
K1z = Par_Vehicle.K1z;      C1z = Par_Vehicle.C1z;
K2x = Par_Vehicle.K2x;      C2x = Par_Vehicle.C2x;
K2y = Par_Vehicle.K2y;      C2y = Par_Vehicle.C2y;
K2z = Par_Vehicle.K2z;      C2z = Par_Vehicle.C2z;

K_Jx = Par_Vehicle.K_Jx;
K_Jy = Par_Vehicle.K_Jy;

K_DPz = Par_Vehicle.K_DPz;
C_DPz = Par_Vehicle.C_DPz(1);

K_Sx = Par_Vehicle.K_Sx;
C_Sx = Par_Vehicle.C_Sx(1);

K_DPy = Par_Vehicle.K_DPy;
C_DPy = Par_Vehicle.C_DPy(1);

K_Tx = Par_Vehicle.K_Tx;
K_ROTx = Par_Vehicle.K_ROTx;

Ll1 = Par_Vehicle.Ll1;
Ll2 = Par_Vehicle.Ll2;
Ll_Jt = Par_Vehicle.Ll_Jt;
Ll_Jw = Par_Vehicle.Ll_Jw;
Ll_DPzt = Par_Vehicle.Ll_DPzt;
Ll_DPzw = Par_Vehicle.Ll_DPzw;
Ll_DPyt = Par_Vehicle.Ll_DPyt;
% Ll_DPyc = Par_Vehicle.Ll_DPyc;     % 250717发现错误后修改
Ll_ST = Par_Vehicle.Ll_ST;

Lb1 = Par_Vehicle.Lb1;
Lb2 = Par_Vehicle.Lb2;
Lb_J = Par_Vehicle.Lb_J;
Lb_DPz = Par_Vehicle.Lb_DPz;
Lb_S = Par_Vehicle.Lb_S;

H_tw = Par_Vehicle.H_tw;
H_Bt = Par_Vehicle.H_Bt;
H_cB = Par_Vehicle.H_cB;
H_tJ = Par_Vehicle.H_tJ;
H_Jw = Par_Vehicle.H_Jw;
H_St = Par_Vehicle.H_St;
H_cS = Par_Vehicle.H_cS;
H_Tt = Par_Vehicle.H_Tt;
H_cT = Par_Vehicle.H_cT;
H_DPyt = Par_Vehicle.H_DPyt;
H_cDPy = Par_Vehicle.H_cDPy;
H_STt = Par_Vehicle.H_STt;
H_cST = Par_Vehicle.H_cST;

%% 列车质量（位移变分为行，加速度为列）
Mlc = zeros(InpPar.N_RV, InpPar.N_RV);
i1=0;
while i1<4
    Mlc(5*i1+1, 5*i1+1)=Mw;
    Mlc(5*i1+2, 5*i1+2)=Mw;
    Mlc(5*i1+3, 5*i1+3)=Jwx;         %轮对侧滚惯量 Jwf  kg.m^2
    Mlc(5*i1+4, 5*i1+4)=Jwy;         %轮对点头惯量 Jwf  kg.m^2        
    Mlc(5*i1+5, 5*i1+5)=Jwz;         %轮对摇头惯量 Jwc  kg.m^2
    i1=i1+1;
end

i1=0;
while i1<2
    Mlc(20+5*i1+1, 20+5*i1+1)=Mb;
    Mlc(20+5*i1+2, 20+5*i1+2)=Mb;
    Mlc(20+5*i1+3, 20+5*i1+3)=Jbx;
    Mlc(20+5*i1+4, 20+5*i1+4)=Jby;
    Mlc(20+5*i1+5, 20+5*i1+5)=Jbz;
    i1=i1+1;
end

Mlc(30+1, 30+1)=Mc;
Mlc(30+2, 30+2)=Mc;
Mlc(30+3, 30+3)=Jcx;
Mlc(30+4, 30+4)=Jcy;
Mlc(30+5, 30+5)=Jcz;

for kk = 36:1:InpPar.N_RV
    Mlc(kk, kk)=1e-8;
end

%% 列车刚度（位移变分为行，位移为列）
Klc = zeros(InpPar.N_RV, InpPar.N_RV);

%%% 一系悬挂 K1z、K1y、K1x
i1=0;   % i1 = 0:1:7;
while i1<8
    i1g1=fix((i1+1+3)/4);       %%% 转向架序号 i=1,2
    i1g2=fix((i1+1+1)/2);       %%% 轮对序号 m=1,2,3,4
    i1g3=fix((3*(i1+1)+1)/2);   %%% 车轮位置序号 ...............j+m=2,3,5,6,8,9,11,12 Left/Right
    i1g4=fix((i1+1-1)/2);       %%% m±1=[0,0,1,1,2,2,3,3;]
    
    pos_B = 20+5*(i1g1-1);
    pos_W = 5*(i1g2-1);
    
    %%% 21、26列：zti(i=1,2)
    Klc(pos_B+1,pos_B+1) = Klc(pos_B+1,pos_B+1)+K1z;
    Klc(pos_B+3,pos_B+1) = Klc(pos_B+3,pos_B+1)+((-1)^(i1+1))*Lb1*K1z;
    Klc(pos_B+4,pos_B+1) = Klc(pos_B+4,pos_B+1)+((-1)^i1g2)*Ll1*K1z;
    Klc(pos_W+1,pos_B+1) = Klc(pos_W+1,pos_B+1)-K1z;
    Klc(pos_W+3,pos_B+1) = Klc(pos_W+3,pos_B+1)-((-1)^(i1+1))*Lb1*K1z;
    %%% 1、6、11、16列：zwm(m = 1,2,3,4)
    Klc(pos_B+1,pos_W+1) = Klc(pos_B+1,pos_W+1)-K1z;
    Klc(pos_B+3,pos_W+1) = Klc(pos_B+3,pos_W+1)-((-1)^(i1+1))*Lb1*K1z;
    Klc(pos_B+4,pos_W+1) = Klc(pos_B+4,pos_W+1)-((-1)^i1g2)*Ll1*K1z;
    Klc(pos_W+1,pos_W+1) = Klc(pos_W+1,pos_W+1)+K1z;
    Klc(pos_W+3,pos_W+1) = Klc(pos_W+3,pos_W+1)+((-1)^(i1+1))*Lb1*K1z;
    %%% 23、28列：φti(i = 1,2)   
    Klc(pos_B+1,pos_B+3) = Klc(pos_B+1,pos_B+3)+((-1)^(i1+1))*Lb1*K1z;
    Klc(pos_B+3,pos_B+3) = Klc(pos_B+3,pos_B+3)+Lb1*Lb1*K1z;
    Klc(pos_B+4,pos_B+3) = Klc(pos_B+4,pos_B+3)+((-1)^i1g3)*Lb1*Ll1*K1z;
    Klc(pos_W+1,pos_B+3) = Klc(pos_W+1,pos_B+3)-((-1)^(i1+1))*Lb1*K1z;
    Klc(pos_W+3,pos_B+3) = Klc(pos_W+3,pos_B+3)-Lb1*Lb1*K1z;
    %%% 24、29列：βti(i = 1,2)      
    Klc(pos_B+1,pos_B+4) = Klc(pos_B+1,pos_B+4)+((-1)^i1g2)*Ll1*K1z;
    Klc(pos_B+3,pos_B+4) = Klc(pos_B+3,pos_B+4)+((-1)^i1g3)*Lb1*Ll1*K1z;
    Klc(pos_B+4,pos_B+4) = Klc(pos_B+4,pos_B+4)+Ll1*Ll1*K1z;
    Klc(pos_W+1,pos_B+4) = Klc(pos_W+1,pos_B+4)-((-1)^i1g2)*Ll1*K1z;
    Klc(pos_W+3,pos_B+4) = Klc(pos_W+3,pos_B+4)-((-1)^i1g3)*Lb1*Ll1*K1z;
    %%% 3、8、13、18列：φwm(m=1,2,3,4)    
    Klc(pos_B+1,pos_W+3)=Klc(pos_B+1,pos_W+3)-((-1)^(i1+1))*Lb1*K1z;
    Klc(pos_B+3,pos_W+3)=Klc(pos_B+3,pos_W+3)-Lb1*Lb1*K1z;
    Klc(pos_B+4,pos_W+3)=Klc(pos_B+4,pos_W+3)-((-1)^i1g3)*Lb1*Ll1*K1z;
    Klc(pos_W+1,pos_W+3)=Klc(pos_W+1,pos_W+3)+((-1)^(i1+1))*Lb1*K1z;
    Klc(pos_W+3,pos_W+3)=Klc(pos_W+3,pos_W+3)+Lb1*Lb1*K1z;    
    
    %%% 22、27行：δyti(i = 1,2)
    Klc(pos_B+2,pos_B+2) = Klc(pos_B+2,pos_B+2)+K1y;
    Klc(pos_B+2,pos_B+3) = Klc(pos_B+2,pos_B+3)-H_tw*K1y;
    Klc(pos_B+2,pos_B+5) = Klc(pos_B+2,pos_B+5)+((-1)^i1g4)*Ll1*K1y;
    Klc(pos_B+2,pos_W+2) = Klc(pos_B+2,pos_W+2)-K1y;
    %%% 23、28行：δφti(i = 1,2)    
    Klc(pos_B+3,pos_B+2) = Klc(pos_B+3,pos_B+2)-H_tw*K1y;
    Klc(pos_B+3,pos_B+3) = Klc(pos_B+3,pos_B+3)+H_tw*H_tw*K1y;
    Klc(pos_B+3,pos_B+5) = Klc(pos_B+3,pos_B+5)-((-1)^i1g4)*Ll1*H_tw*K1y;
    Klc(pos_B+3,pos_W+2) = Klc(pos_B+3,pos_W+2)+H_tw*K1y;
    %%% 25、30行：δψti(i = 1,2)     
    Klc(pos_B+5,pos_B+2) = Klc(pos_B+5,pos_B+2)+((-1)^i1g4)*Ll1*K1y;
    Klc(pos_B+5,pos_B+3) = Klc(pos_B+5,pos_B+3)-((-1)^i1g4)*Ll1*H_tw*K1y;
    Klc(pos_B+5,pos_B+5) = Klc(pos_B+5,pos_B+5)+Ll1*Ll1*K1y;
    Klc(pos_B+5,pos_W+2) = Klc(pos_B+5,pos_W+2)-((-1)^i1g4)*Ll1*K1y;
    %%% 2、7、12、17行：δywm(m = 1,2,3,4)      
    Klc(pos_W+2,pos_B+2) = Klc(pos_W+2,pos_B+2)-K1y;
    Klc(pos_W+2,pos_B+3) = Klc(pos_W+2,pos_B+3)+H_tw*K1y;
    Klc(pos_W+2,pos_B+5) = Klc(pos_W+2,pos_B+5)-((-1)^i1g4)*Ll1*K1y;
    Klc(pos_W+2,pos_W+2) = Klc(pos_W+2,pos_W+2)+K1y;
    
    %%% 24、29行：δβti(i = 1,2)     
    Klc(pos_B+4,pos_B+4) = Klc(pos_B+4,pos_B+4)+H_tw*H_tw*K1x;
    Klc(pos_B+4,pos_B+5) = Klc(pos_B+4,pos_B+5)+((-1)^(i1+1+1))*Lb1*H_tw*K1x;
    Klc(pos_B+4,pos_W+5) = Klc(pos_B+4,pos_W+5)-((-1)^(i1+1+1))*Lb1*H_tw*K1x;
    %%% 25、30行：δψti(i = 1,2)      
    Klc(pos_B+5,pos_B+4) = Klc(pos_B+5,pos_B+4)+((-1)^(i1+1+1))*Lb1*H_tw*K1x;
    Klc(pos_B+5,pos_B+5) = Klc(pos_B+5,pos_B+5)+Lb1*Lb1*K1x;
    Klc(pos_B+5,pos_W+5) = Klc(pos_B+5,pos_W+5)-Lb1*Lb1*K1x;
    %%% 5、10、15、20行：δψwm(m = 1,2,3,4)     
    Klc(pos_W+5,pos_B+4) = Klc(pos_W+5,pos_B+4)-((-1)^(i1+1+1))*Lb1*H_tw*K1x;
    Klc(pos_W+5,pos_B+5) = Klc(pos_W+5,pos_B+5)-Lb1*Lb1*K1x;
    Klc(pos_W+5,pos_W+5) = Klc(pos_W+5,pos_W+5)+Lb1*Lb1*K1x;
    
    i1=i1+1;
end

%%% 二系悬挂 K2z、K2y、K2x
i1=0;   % i1 = 0:1:3;
while i1<4
    i1g2=fix((i1+1+1)/2);          %%% 构架序号 i=[1,1,2,2]
    i1g3=fix((3*(i1+1)+1)/2);      %%% 车轮位置序号 j+i=2,3,5,6 Left/Right
    i1g4=fix((i1+1-1)/2);          %%% i±1=[0,0,1,1]
    
    pos_BG = 20+5*(i1g2-1);
    pos_CB = 30;
    
    %%% 31列：zc
    Klc(pos_CB+1, pos_CB+1) = Klc(pos_CB+1, pos_CB+1)+K2z;
    Klc(pos_CB+3, pos_CB+1) = Klc(pos_CB+3, pos_CB+1)+((-1)^(i1+1))*Lb2*K2z;
    Klc(pos_CB+4, pos_CB+1) = Klc(pos_CB+4, pos_CB+1)+((-1)^i1g2)*Ll2*K2z;
    Klc(pos_BG+1, pos_CB+1) = Klc(pos_BG+1, pos_CB+1)-K2z;
    Klc(pos_BG+3, pos_CB+1) = Klc(pos_BG+3, pos_CB+1)-((-1)^(i1+1))*Lb2*K2z;
    %%% 33列：φc    
    Klc(pos_CB+1, pos_CB+3) = Klc(pos_CB+1, pos_CB+3)+((-1)^(i1+1))*Lb2*K2z;
    Klc(pos_CB+3, pos_CB+3) = Klc(pos_CB+3, pos_CB+3)+Lb2*Lb2*K2z;
    Klc(pos_CB+4, pos_CB+3) = Klc(pos_CB+4, pos_CB+3)+((-1)^i1g3)*Lb2*Ll2*K2z;
    Klc(pos_BG+1, pos_CB+3) = Klc(pos_BG+1, pos_CB+3)-((-1)^(i1+1))*Lb2*K2z;
    Klc(pos_BG+3, pos_CB+3) = Klc(pos_BG+3, pos_CB+3)-Lb2*Lb2*K2z;
    %%% 34列：βc     
    Klc(pos_CB+1, pos_CB+4) = Klc(pos_CB+1, pos_CB+4)+((-1)^i1g2)*Ll2*K2z;
    Klc(pos_CB+3, pos_CB+4) = Klc(pos_CB+3, pos_CB+4)+((-1)^i1g3)*Lb2*Ll2*K2z;
    Klc(pos_CB+4, pos_CB+4) = Klc(pos_CB+4, pos_CB+4)+Ll2*Ll2*K2z;
    Klc(pos_BG+1, pos_CB+4) = Klc(pos_BG+1, pos_CB+4)-((-1)^i1g2)*Ll2*K2z;
    Klc(pos_BG+3, pos_CB+4) = Klc(pos_BG+3, pos_CB+4)-((-1)^i1g3)*Lb2*Ll2*K2z;
    %%% 21、26列：zti(i = 1,2)      
    Klc(pos_CB+1, pos_BG+1) = Klc(pos_CB+1, pos_BG+1)-K2z;
    Klc(pos_CB+3, pos_BG+1) = Klc(pos_CB+3, pos_BG+1)-((-1)^(i1+1))*Lb2*K2z;
    Klc(pos_CB+4, pos_BG+1) = Klc(pos_CB+4, pos_BG+1)-((-1)^i1g2)*Ll2*K2z;
    Klc(pos_BG+1, pos_BG+1) = Klc(pos_BG+1, pos_BG+1)+K2z;
    Klc(pos_BG+3, pos_BG+1) = Klc(pos_BG+3, pos_BG+1)+((-1)^(i1+1))*Lb2*K2z;
    %%% 23、27列：φti(i = 1,2)    
    Klc(pos_CB+1, pos_BG+3) = Klc(pos_CB+1, pos_BG+3)-((-1)^(i1+1))*Lb2*K2z;
    Klc(pos_CB+3, pos_BG+3) = Klc(pos_CB+3, pos_BG+3)-Lb2*Lb2*K2z;
    Klc(pos_CB+4, pos_BG+3) = Klc(pos_CB+4, pos_BG+3)-((-1)^i1g3)*Lb2*Ll2*K2z;
    Klc(pos_BG+1, pos_BG+3) = Klc(pos_BG+1, pos_BG+3)+((-1)^(i1+1))*Lb2*K2z;
    Klc(pos_BG+3, pos_BG+3) = Klc(pos_BG+3, pos_BG+3)+Lb2*Lb2*K2z;    
    
    %%% 32行：δyc    
    Klc(pos_CB+2, pos_CB+2) = Klc(pos_CB+2, pos_CB+2)+K2y;
    Klc(pos_CB+2, pos_CB+3) = Klc(pos_CB+2, pos_CB+3)-H_cB*K2y;
    Klc(pos_CB+2, pos_CB+5) = Klc(pos_CB+2, pos_CB+5)+((-1)^i1g4)*Ll2*K2y;
    Klc(pos_CB+2, pos_BG+2) = Klc(pos_CB+2, pos_BG+2)-K2y;
    Klc(pos_CB+2, pos_BG+3) = Klc(pos_CB+2, pos_BG+3)-H_Bt*K2y;
    %%% 33行：δφc      
    Klc(pos_CB+3, pos_CB+2) = Klc(pos_CB+3, pos_CB+2)-H_cB*K2y;
    Klc(pos_CB+3, pos_CB+3) = Klc(pos_CB+3, pos_CB+3)+H_cB*H_cB*K2y;
    Klc(pos_CB+3, pos_CB+5) = Klc(pos_CB+3, pos_CB+5)-((-1)^i1g4)*Ll2*H_cB*K2y;
    Klc(pos_CB+3, pos_BG+2) = Klc(pos_CB+3, pos_BG+2)+H_cB*K2y;
    Klc(pos_CB+3, pos_BG+3) = Klc(pos_CB+3, pos_BG+3)+H_cB*H_Bt*K2y;
    %%% 35行：δψc       
    Klc(pos_CB+5, pos_CB+2) = Klc(pos_CB+5, pos_CB+2)+((-1)^i1g4)*Ll2*K2y;
    Klc(pos_CB+5, pos_CB+3) = Klc(pos_CB+5, pos_CB+3)-((-1)^i1g4)*Ll2*H_cB*K2y;
    Klc(pos_CB+5, pos_CB+5) = Klc(pos_CB+5, pos_CB+5)+Ll2*Ll2*K2y;
    Klc(pos_CB+5, pos_BG+2) = Klc(pos_CB+5, pos_BG+2)-((-1)^i1g4)*Ll2*K2y;
    Klc(pos_CB+5, pos_BG+3) = Klc(pos_CB+5, pos_BG+3)-((-1)^i1g4)*Ll2*H_Bt*K2y;
    %%% 22、27行：δyti(i = 1,2)     
    Klc(pos_BG+2, pos_CB+2) = Klc(pos_BG+2, pos_CB+2)-K2y;
    Klc(pos_BG+2, pos_CB+3) = Klc(pos_BG+2, pos_CB+3)+H_cB*K2y;
    Klc(pos_BG+2, pos_CB+5) = Klc(pos_BG+2, pos_CB+5)-((-1)^i1g4)*Ll2*K2y;
    Klc(pos_BG+2, pos_BG+2) = Klc(pos_BG+2, pos_BG+2)+K2y;
    Klc(pos_BG+2, pos_BG+3) = Klc(pos_BG+2, pos_BG+3)+H_Bt*K2y;
    %%% 23、28行：δφti(i = 1,2)       
    Klc(pos_BG+3, pos_CB+2) = Klc(pos_BG+3, pos_CB+2)-H_Bt*K2y;
    Klc(pos_BG+3, pos_CB+3) = Klc(pos_BG+3, pos_CB+3)+H_cB*H_Bt*K2y;
    Klc(pos_BG+3, pos_CB+5) = Klc(pos_BG+3, pos_CB+5)-((-1)^i1g4)*Ll2*H_Bt*K2y;
    Klc(pos_BG+3, pos_BG+2) = Klc(pos_BG+3, pos_BG+2)+H_Bt*K2y;
    Klc(pos_BG+3, pos_BG+3) = Klc(pos_BG+3, pos_BG+3)+H_Bt*H_Bt*K2y;
    
    %%% 34行：δβc   
    Klc(pos_CB+4, pos_CB+4) = Klc(pos_CB+4, pos_CB+4)+H_cB*H_cB*K2x;
    Klc(pos_CB+4, pos_CB+5) = Klc(pos_CB+4, pos_CB+5)+((-1)^(i1+1+1))*Lb2*H_cB*K2x;
    Klc(pos_CB+4, pos_BG+5) = Klc(pos_CB+4, pos_BG+5)-((-1)^(i1+1+1))*Lb2*H_cB*K2x;
    Klc(pos_CB+4, pos_BG+4) = Klc(pos_CB+4, pos_BG+4)+H_cB*H_Bt*K2x;
    %%% 35行：δψc      
    Klc(pos_CB+5, pos_CB+4) = Klc(pos_CB+5, pos_CB+4)+((-1)^(i1+1+1))*Lb2*H_cB*K2x;
    Klc(pos_CB+5, pos_CB+5) = Klc(pos_CB+5, pos_CB+5)+Lb2*Lb2*K2x;
    Klc(pos_CB+5, pos_BG+5) = Klc(pos_CB+5, pos_BG+5)-Lb2*Lb2*K2x;
    Klc(pos_CB+5, pos_BG+4) = Klc(pos_CB+5, pos_BG+4)+((-1)^(i1+1+1))*Lb2*H_Bt*K2x;
    %%% 25、30行：δψti(i=1,2)       
    Klc(pos_BG+5, pos_CB+4) = Klc(pos_BG+5, pos_CB+4)-((-1)^(i1+1+1))*Lb2*H_cB*K2x;
    Klc(pos_BG+5, pos_CB+5) = Klc(pos_BG+5, pos_CB+5)-Lb2*Lb2*K2x;
    Klc(pos_BG+5, pos_BG+5) = Klc(pos_BG+5, pos_BG+5)+Lb2*Lb2*K2x;
    Klc(pos_BG+5, pos_BG+4) = Klc(pos_BG+5, pos_BG+4)-((-1)^(i1+1+1))*Lb2*H_Bt*K2x;
    %%% 24、29行：δβti(i=1,2)       
    Klc(pos_BG+4, pos_CB+4) = Klc(pos_BG+4, pos_CB+4)+H_cB*H_Bt*K2x;
    Klc(pos_BG+4, pos_CB+5) = Klc(pos_BG+4, pos_CB+5)+((-1)^(i1+1+1))*Lb2*H_Bt*K2x;
    Klc(pos_BG+4, pos_BG+5) = Klc(pos_BG+4, pos_BG+5)-((-1)^(i1+1+1))*Lb2*H_Bt*K2x;
    Klc(pos_BG+4, pos_BG+4) = Klc(pos_BG+4, pos_BG+4)+H_Bt*H_Bt*K2x;
    
    i1=i1+1;
end

%% 列车阻尼（位移变分为行，速度为列）
Clc = zeros(InpPar.N_RV, InpPar.N_RV);

%%% 一系悬挂 C1z、C1y、C1x
i1=0;   % i1 = 0:1:7;
while i1<8
    i1g1=fix((i1+1+3)/4);       %%% 转向架序号 i=1,2
    i1g2=fix((i1+1+1)/2);       %%% 轮对序号 m=1,2,3,4
    i1g3=fix((3*(i1+1)+1)/2);   %%% 车轮位置序号 ...............j+m=2,3,5,6,8,9,11,12 Left/Right
    i1g4=fix((i1+1-1)/2);       %%% m±1=[0,0,1,1,2,2,3,3;]
    
    pos_B = 20+5*(i1g1-1);
    pos_W = 5*(i1g2-1);
    
    %%% 21、26列：zti(i=1,2)
    Clc(pos_B+1,pos_B+1) = Clc(pos_B+1,pos_B+1)+C1z;
    Clc(pos_B+3,pos_B+1) = Clc(pos_B+3,pos_B+1)+((-1)^(i1+1))*Lb1*C1z;
    Clc(pos_B+4,pos_B+1) = Clc(pos_B+4,pos_B+1)+((-1)^i1g2)*Ll1*C1z;
    Clc(pos_W+1,pos_B+1) = Clc(pos_W+1,pos_B+1)-C1z;
    Clc(pos_W+3,pos_B+1) = Clc(pos_W+3,pos_B+1)-((-1)^(i1+1))*Lb1*C1z;
    %%% 1、6、11、16列：zwm(m = 1,2,3,4)
    Clc(pos_B+1,pos_W+1) = Clc(pos_B+1,pos_W+1)-C1z;
    Clc(pos_B+3,pos_W+1) = Clc(pos_B+3,pos_W+1)-((-1)^(i1+1))*Lb1*C1z;
    Clc(pos_B+4,pos_W+1) = Clc(pos_B+4,pos_W+1)-((-1)^i1g2)*Ll1*C1z;
    Clc(pos_W+1,pos_W+1) = Clc(pos_W+1,pos_W+1)+C1z;
    Clc(pos_W+3,pos_W+1) = Clc(pos_W+3,pos_W+1)+((-1)^(i1+1))*Lb1*C1z;
    %%% 23、28列：φti(i = 1,2)   
    Clc(pos_B+1,pos_B+3) = Clc(pos_B+1,pos_B+3)+((-1)^(i1+1))*Lb1*C1z;
    Clc(pos_B+3,pos_B+3) = Clc(pos_B+3,pos_B+3)+Lb1*Lb1*C1z;
    Clc(pos_B+4,pos_B+3) = Clc(pos_B+4,pos_B+3)+((-1)^i1g3)*Lb1*Ll1*C1z;
    Clc(pos_W+1,pos_B+3) = Clc(pos_W+1,pos_B+3)-((-1)^(i1+1))*Lb1*C1z;
    Clc(pos_W+3,pos_B+3) = Clc(pos_W+3,pos_B+3)-Lb1*Lb1*C1z;
    %%% 24、29列：βti(i = 1,2)      
    Clc(pos_B+1,pos_B+4) = Clc(pos_B+1,pos_B+4)+((-1)^i1g2)*Ll1*C1z;
    Clc(pos_B+3,pos_B+4) = Clc(pos_B+3,pos_B+4)+((-1)^i1g3)*Lb1*Ll1*C1z;
    Clc(pos_B+4,pos_B+4) = Clc(pos_B+4,pos_B+4)+Ll1*Ll1*C1z;
    Clc(pos_W+1,pos_B+4) = Clc(pos_W+1,pos_B+4)-((-1)^i1g2)*Ll1*C1z;
    Clc(pos_W+3,pos_B+4) = Clc(pos_W+3,pos_B+4)-((-1)^i1g3)*Lb1*Ll1*C1z;
    %%% 3、8、13、18列：φwm(m=1,2,3,4)    
    Clc(pos_B+1,pos_W+3)=Clc(pos_B+1,pos_W+3)-((-1)^(i1+1))*Lb1*C1z;
    Clc(pos_B+3,pos_W+3)=Clc(pos_B+3,pos_W+3)-Lb1*Lb1*C1z;
    Clc(pos_B+4,pos_W+3)=Clc(pos_B+4,pos_W+3)-((-1)^i1g3)*Lb1*Ll1*C1z;
    Clc(pos_W+1,pos_W+3)=Clc(pos_W+1,pos_W+3)+((-1)^(i1+1))*Lb1*C1z;
    Clc(pos_W+3,pos_W+3)=Clc(pos_W+3,pos_W+3)+Lb1*Lb1*C1z;    
    
    %%% 22、27行：δyti(i = 1,2)
    Clc(pos_B+2,pos_B+2) = Clc(pos_B+2,pos_B+2)+C1y;
    Clc(pos_B+2,pos_B+3) = Clc(pos_B+2,pos_B+3)-H_tw*C1y;
    Clc(pos_B+2,pos_B+5) = Clc(pos_B+2,pos_B+5)+((-1)^i1g4)*Ll1*C1y;
    Clc(pos_B+2,pos_W+2) = Clc(pos_B+2,pos_W+2)-C1y;
    %%% 23、28行：δφti(i = 1,2)    
    Clc(pos_B+3,pos_B+2) = Clc(pos_B+3,pos_B+2)-H_tw*C1y;
    Clc(pos_B+3,pos_B+3) = Clc(pos_B+3,pos_B+3)+H_tw*H_tw*C1y;
    Clc(pos_B+3,pos_B+5) = Clc(pos_B+3,pos_B+5)-((-1)^i1g4)*Ll1*H_tw*C1y;
    Clc(pos_B+3,pos_W+2) = Clc(pos_B+3,pos_W+2)+H_tw*C1y;
    %%% 25、30行：δψti(i = 1,2)     
    Clc(pos_B+5,pos_B+2) = Clc(pos_B+5,pos_B+2)+((-1)^i1g4)*Ll1*C1y;
    Clc(pos_B+5,pos_B+3) = Clc(pos_B+5,pos_B+3)-((-1)^i1g4)*Ll1*H_tw*C1y;
    Clc(pos_B+5,pos_B+5) = Clc(pos_B+5,pos_B+5)+Ll1*Ll1*C1y;
    Clc(pos_B+5,pos_W+2) = Clc(pos_B+5,pos_W+2)-((-1)^i1g4)*Ll1*C1y;
    %%% 2、7、12、17行：δywm(m = 1,2,3,4)      
    Clc(pos_W+2,pos_B+2) = Clc(pos_W+2,pos_B+2)-C1y;
    Clc(pos_W+2,pos_B+3) = Clc(pos_W+2,pos_B+3)+H_tw*C1y;
    Clc(pos_W+2,pos_B+5) = Clc(pos_W+2,pos_B+5)-((-1)^i1g4)*Ll1*C1y;
    Clc(pos_W+2,pos_W+2) = Clc(pos_W+2,pos_W+2)+C1y;
    
    %%% 24、29行：δβti(i = 1,2)     
    Clc(pos_B+4,pos_B+4) = Clc(pos_B+4,pos_B+4)+H_tw*H_tw*C1x;
    Clc(pos_B+4,pos_B+5) = Clc(pos_B+4,pos_B+5)+((-1)^(i1+1+1))*Lb1*H_tw*C1x;
    Clc(pos_B+4,pos_W+5) = Clc(pos_B+4,pos_W+5)-((-1)^(i1+1+1))*Lb1*H_tw*C1x;
    %%% 25、30行：δψti(i = 1,2)      
    Clc(pos_B+5,pos_B+4) = Clc(pos_B+5,pos_B+4)+((-1)^(i1+1+1))*Lb1*H_tw*C1x;
    Clc(pos_B+5,pos_B+5) = Clc(pos_B+5,pos_B+5)+Lb1*Lb1*C1x;
    Clc(pos_B+5,pos_W+5) = Clc(pos_B+5,pos_W+5)-Lb1*Lb1*C1x;
    %%% 5、10、15、20行：δψwm(m = 1,2,3,4)     
    Clc(pos_W+5,pos_B+4) = Clc(pos_W+5,pos_B+4)-((-1)^(i1+1+1))*Lb1*H_tw*C1x;
    Clc(pos_W+5,pos_B+5) = Clc(pos_W+5,pos_B+5)-Lb1*Lb1*C1x;
    Clc(pos_W+5,pos_W+5) = Clc(pos_W+5,pos_W+5)+Lb1*Lb1*C1x;
    
    i1=i1+1;
end

%%% 二系悬挂 C2z、C2y、C2x
i1=0;   % i1 = 0:1:3;
while i1<4
    i1g2=fix((i1+1+1)/2);          %%% 构架序号 i=[1,1,2,2]
    i1g3=fix((3*(i1+1)+1)/2);      %%% 车轮位置序号 j+i=2,3,5,6 Left/Right
    i1g4=fix((i1+1-1)/2);          %%% i±1=[0,0,1,1]
    
    pos_BG = 20+5*(i1g2-1);
    pos_CB = 30;
    
    %%% 31列：zc
    Clc(pos_CB+1, pos_CB+1) = Clc(pos_CB+1, pos_CB+1)+C2z;
    Clc(pos_CB+3, pos_CB+1) = Clc(pos_CB+3, pos_CB+1)+((-1)^(i1+1))*Lb2*C2z;
    Clc(pos_CB+4, pos_CB+1) = Clc(pos_CB+4, pos_CB+1)+((-1)^i1g2)*Ll2*C2z;
    Clc(pos_BG+1, pos_CB+1) = Clc(pos_BG+1, pos_CB+1)-C2z;
    Clc(pos_BG+3, pos_CB+1) = Clc(pos_BG+3, pos_CB+1)-((-1)^(i1+1))*Lb2*C2z;
    %%% 33列：φc    
    Clc(pos_CB+1, pos_CB+3) = Clc(pos_CB+1, pos_CB+3)+((-1)^(i1+1))*Lb2*C2z;
    Clc(pos_CB+3, pos_CB+3) = Clc(pos_CB+3, pos_CB+3)+Lb2*Lb2*C2z;
    Clc(pos_CB+4, pos_CB+3) = Clc(pos_CB+4, pos_CB+3)+((-1)^i1g3)*Lb2*Ll2*C2z;
    Clc(pos_BG+1, pos_CB+3) = Clc(pos_BG+1, pos_CB+3)-((-1)^(i1+1))*Lb2*C2z;
    Clc(pos_BG+3, pos_CB+3) = Clc(pos_BG+3, pos_CB+3)-Lb2*Lb2*C2z;
    %%% 34列：βc     
    Clc(pos_CB+1, pos_CB+4) = Clc(pos_CB+1, pos_CB+4)+((-1)^i1g2)*Ll2*C2z;
    Clc(pos_CB+3, pos_CB+4) = Clc(pos_CB+3, pos_CB+4)+((-1)^i1g3)*Lb2*Ll2*C2z;
    Clc(pos_CB+4, pos_CB+4) = Clc(pos_CB+4, pos_CB+4)+Ll2*Ll2*C2z;
    Clc(pos_BG+1, pos_CB+4) = Clc(pos_BG+1, pos_CB+4)-((-1)^i1g2)*Ll2*C2z;
    Clc(pos_BG+3, pos_CB+4) = Clc(pos_BG+3, pos_CB+4)-((-1)^i1g3)*Lb2*Ll2*C2z;
    %%% 21、26列：zti(i = 1,2)      
    Clc(pos_CB+1, pos_BG+1) = Clc(pos_CB+1, pos_BG+1)-C2z;
    Clc(pos_CB+3, pos_BG+1) = Clc(pos_CB+3, pos_BG+1)-((-1)^(i1+1))*Lb2*C2z;
    Clc(pos_CB+4, pos_BG+1) = Clc(pos_CB+4, pos_BG+1)-((-1)^i1g2)*Ll2*C2z;
    Clc(pos_BG+1, pos_BG+1) = Clc(pos_BG+1, pos_BG+1)+C2z;
    Clc(pos_BG+3, pos_BG+1) = Clc(pos_BG+3, pos_BG+1)+((-1)^(i1+1))*Lb2*C2z;
    %%% 23、27列：φti(i = 1,2)    
    Clc(pos_CB+1, pos_BG+3) = Clc(pos_CB+1, pos_BG+3)-((-1)^(i1+1))*Lb2*C2z;
    Clc(pos_CB+3, pos_BG+3) = Clc(pos_CB+3, pos_BG+3)-Lb2*Lb2*C2z;
    Clc(pos_CB+4, pos_BG+3) = Clc(pos_CB+4, pos_BG+3)-((-1)^i1g3)*Lb2*Ll2*C2z;
    Clc(pos_BG+1, pos_BG+3) = Clc(pos_BG+1, pos_BG+3)+((-1)^(i1+1))*Lb2*C2z;
    Clc(pos_BG+3, pos_BG+3) = Clc(pos_BG+3, pos_BG+3)+Lb2*Lb2*C2z;    
    
    %%% 32行：δyc    
    Clc(pos_CB+2, pos_CB+2) = Clc(pos_CB+2, pos_CB+2)+C2y;
    Clc(pos_CB+2, pos_CB+3) = Clc(pos_CB+2, pos_CB+3)-H_cB*C2y;
    Clc(pos_CB+2, pos_CB+5) = Clc(pos_CB+2, pos_CB+5)+((-1)^i1g4)*Ll2*C2y;
    Clc(pos_CB+2, pos_BG+2) = Clc(pos_CB+2, pos_BG+2)-C2y;
    Clc(pos_CB+2, pos_BG+3) = Clc(pos_CB+2, pos_BG+3)-H_Bt*C2y;
    %%% 33行：δφc      
    Clc(pos_CB+3, pos_CB+2) = Clc(pos_CB+3, pos_CB+2)-H_cB*C2y;
    Clc(pos_CB+3, pos_CB+3) = Clc(pos_CB+3, pos_CB+3)+H_cB*H_cB*C2y;
    Clc(pos_CB+3, pos_CB+5) = Clc(pos_CB+3, pos_CB+5)-((-1)^i1g4)*Ll2*H_cB*C2y;
    Clc(pos_CB+3, pos_BG+2) = Clc(pos_CB+3, pos_BG+2)+H_cB*C2y;
    Clc(pos_CB+3, pos_BG+3) = Clc(pos_CB+3, pos_BG+3)+H_cB*H_Bt*C2y;
    %%% 35行：δψc       
    Clc(pos_CB+5, pos_CB+2) = Clc(pos_CB+5, pos_CB+2)+((-1)^i1g4)*Ll2*C2y;
    Clc(pos_CB+5, pos_CB+3) = Clc(pos_CB+5, pos_CB+3)-((-1)^i1g4)*Ll2*H_cB*C2y;
    Clc(pos_CB+5, pos_CB+5) = Clc(pos_CB+5, pos_CB+5)+Ll2*Ll2*C2y;
    Clc(pos_CB+5, pos_BG+2) = Clc(pos_CB+5, pos_BG+2)-((-1)^i1g4)*Ll2*C2y;
    Clc(pos_CB+5, pos_BG+3) = Clc(pos_CB+5, pos_BG+3)-((-1)^i1g4)*Ll2*H_Bt*C2y;
    %%% 22、27行：δyti(i = 1,2)     
    Clc(pos_BG+2, pos_CB+2) = Clc(pos_BG+2, pos_CB+2)-C2y;
    Clc(pos_BG+2, pos_CB+3) = Clc(pos_BG+2, pos_CB+3)+H_cB*C2y;
    Clc(pos_BG+2, pos_CB+5) = Clc(pos_BG+2, pos_CB+5)-((-1)^i1g4)*Ll2*C2y;
    Clc(pos_BG+2, pos_BG+2) = Clc(pos_BG+2, pos_BG+2)+C2y;
    Clc(pos_BG+2, pos_BG+3) = Clc(pos_BG+2, pos_BG+3)+H_Bt*C2y;
    %%% 23、28行：δφti(i = 1,2)       
    Clc(pos_BG+3, pos_CB+2) = Clc(pos_BG+3, pos_CB+2)-H_Bt*C2y;
    Clc(pos_BG+3, pos_CB+3) = Clc(pos_BG+3, pos_CB+3)+H_cB*H_Bt*C2y;
    Clc(pos_BG+3, pos_CB+5) = Clc(pos_BG+3, pos_CB+5)-((-1)^i1g4)*Ll2*H_Bt*C2y;
    Clc(pos_BG+3, pos_BG+2) = Clc(pos_BG+3, pos_BG+2)+H_Bt*C2y;
    Clc(pos_BG+3, pos_BG+3) = Clc(pos_BG+3, pos_BG+3)+H_Bt*H_Bt*C2y;
    
    %%% 34行：δβc   
    Clc(pos_CB+4, pos_CB+4) = Clc(pos_CB+4, pos_CB+4)+H_cB*H_cB*C2x;
    Clc(pos_CB+4, pos_CB+5) = Clc(pos_CB+4, pos_CB+5)+((-1)^(i1+1+1))*Lb2*H_cB*C2x;
    Clc(pos_CB+4, pos_BG+5) = Clc(pos_CB+4, pos_BG+5)-((-1)^(i1+1+1))*Lb2*H_cB*C2x;
    Clc(pos_CB+4, pos_BG+4) = Clc(pos_CB+4, pos_BG+4)+H_cB*H_Bt*C2x;
    %%% 35行：δψc      
    Clc(pos_CB+5, pos_CB+4) = Clc(pos_CB+5, pos_CB+4)+((-1)^(i1+1+1))*Lb2*H_cB*C2x;
    Clc(pos_CB+5, pos_CB+5) = Clc(pos_CB+5, pos_CB+5)+Lb2*Lb2*C2x;
    Clc(pos_CB+5, pos_BG+5) = Clc(pos_CB+5, pos_BG+5)-Lb2*Lb2*C2x;
    Clc(pos_CB+5, pos_BG+4) = Clc(pos_CB+5, pos_BG+4)+((-1)^(i1+1+1))*Lb2*H_Bt*C2x;
    %%% 25、30行：δψti(i=1,2)       
    Clc(pos_BG+5, pos_CB+4) = Clc(pos_BG+5, pos_CB+4)-((-1)^(i1+1+1))*Lb2*H_cB*C2x;
    Clc(pos_BG+5, pos_CB+5) = Clc(pos_BG+5, pos_CB+5)-Lb2*Lb2*C2x;
    Clc(pos_BG+5, pos_BG+5) = Clc(pos_BG+5, pos_BG+5)+Lb2*Lb2*C2x;
    Clc(pos_BG+5, pos_BG+4) = Clc(pos_BG+5, pos_BG+4)-((-1)^(i1+1+1))*Lb2*H_Bt*C2x;
    %%% 24、29行：δβti(i=1,2)       
    Clc(pos_BG+4, pos_CB+4) = Clc(pos_BG+4, pos_CB+4)+H_cB*H_Bt*C2x;
    Clc(pos_BG+4, pos_CB+5) = Clc(pos_BG+4, pos_CB+5)+((-1)^(i1+1+1))*Lb2*H_Bt*C2x;
    Clc(pos_BG+4, pos_BG+5) = Clc(pos_BG+4, pos_BG+5)-((-1)^(i1+1+1))*Lb2*H_Bt*C2x;
    Clc(pos_BG+4, pos_BG+4) = Clc(pos_BG+4, pos_BG+4)+H_Bt*H_Bt*C2x;
    
    i1=i1+1;
end

Clc_0 = Clc;

%% 一系轴箱转臂节点刚度 K_Jx, K_Jy
for i1 = 1:1:2
    for i2 = 1:1:2
        for i3 = 1:1:2
            pos_W = 10*(i1-1)+5*(i2-1);
            pos_B = 20+5*(i1-1);            
            
            % K_Jx
            % Col 24, 29: βt
            Klc(pos_B+4, pos_B+4) = Klc(pos_B+4, pos_B+4) + H_tJ*H_tJ*K_Jx;
            Klc(pos_B+5, pos_B+4) = Klc(pos_B+5, pos_B+4) + H_tJ*((-1)^(i3+1))*Lb_J*K_Jx;
            Klc(pos_W+4, pos_B+4) = Klc(pos_W+4, pos_B+4) + H_tJ*H_Jw*K_Jx;
            Klc(pos_W+5, pos_B+4) = Klc(pos_W+5, pos_B+4) - H_tJ*((-1)^(i3+1))*Lb_J*K_Jx;            
            % Col 25, 30: ψt
            Klc(pos_B+4, pos_B+5) = Klc(pos_B+4, pos_B+5) + ((-1)^(i3+1))*Lb_J*H_tJ*K_Jx;
            Klc(pos_B+5, pos_B+5) = Klc(pos_B+5, pos_B+5) + Lb_J*Lb_J*K_Jx;
            Klc(pos_W+4, pos_B+5) = Klc(pos_W+4, pos_B+5) + ((-1)^(i3+1))*Lb_J*H_Jw*K_Jx;
            Klc(pos_W+5, pos_B+5) = Klc(pos_W+5, pos_B+5) - Lb_J*Lb_J*K_Jx;            
            % Col 4, 9, 14, 19: βw
            Klc(pos_B+4, pos_W+4) = Klc(pos_B+4, pos_W+4) + H_Jw*H_tJ*K_Jx;
            Klc(pos_B+5, pos_W+4) = Klc(pos_B+5, pos_W+4) + H_Jw*((-1)^(i3+1))*Lb_J*K_Jx;
            Klc(pos_W+4, pos_W+4) = Klc(pos_W+4, pos_W+4) + H_Jw*H_Jw*K_Jx;
            Klc(pos_W+5, pos_W+4) = Klc(pos_W+5, pos_W+4) - H_Jw*((-1)^(i3+1))*Lb_J*K_Jx;
            % Col 5, 10, 15, 20: ψw
            Klc(pos_B+4, pos_W+5) = Klc(pos_B+4, pos_W+5) - ((-1)^(i3+1))*Lb_J*H_tJ*K_Jx;
            Klc(pos_B+5, pos_W+5) = Klc(pos_B+5, pos_W+5) - Lb_J*Lb_J*K_Jx;
            Klc(pos_W+4, pos_W+5) = Klc(pos_W+4, pos_W+5) - ((-1)^(i3+1))*Lb_J*H_Jw*K_Jx;            
            Klc(pos_W+5, pos_W+5) = Klc(pos_W+5, pos_W+5) + Lb_J*Lb_J*K_Jx;    
            
            % K_Jy
            % Col 22, 27: yt
            Klc(pos_B+2, pos_B+2) = Klc(pos_B+2, pos_B+2) + K_Jy;
            Klc(pos_B+3,pos_B+2) = Klc(pos_B+3,pos_B+2) - H_tJ*K_Jy;
            Klc(pos_B+5,pos_B+2) = Klc(pos_B+5,pos_B+2) + ((-1)^(i2+1))*Ll_Jt*K_Jy;
            Klc(pos_W+2, pos_B+2) = Klc(pos_W+2, pos_B+2) - K_Jy;
            Klc(pos_W+3, pos_B+2) = Klc(pos_W+3, pos_B+2) - H_Jw*K_Jy;
            Klc(pos_W+5, pos_B+2) = Klc(pos_W+5, pos_B+2) - ((-1)^(i2+1))*Ll_Jw*K_Jy;            
            % Col 23, 28: φt
            Klc(pos_B+2, pos_B+3) = Klc(pos_B+2, pos_B+3) - H_tJ*K_Jy;
            Klc(pos_B+3,pos_B+3) = Klc(pos_B+3,pos_B+3) + H_tJ*H_tJ*K_Jy;
            Klc(pos_B+5,pos_B+3) = Klc(pos_B+5,pos_B+3) - H_tJ*((-1)^(i2+1))*Ll_Jt*K_Jy;
            Klc(pos_W+2, pos_B+3) = Klc(pos_W+2, pos_B+3) + H_tJ*K_Jy;
            Klc(pos_W+3, pos_B+3) = Klc(pos_W+3, pos_B+3) + H_tJ*H_Jw*K_Jy;
            Klc(pos_W+5, pos_B+3) = Klc(pos_W+5, pos_B+3) + H_tJ*((-1)^(i2+1))*Ll_Jw*K_Jy;            
            % Col 25, 30: ψt
            Klc(pos_B+2, pos_B+5) = Klc(pos_B+2, pos_B+5) + ((-1)^(i2+1))*Ll_Jt*K_Jy;
            Klc(pos_B+3,pos_B+5) = Klc(pos_B+3,pos_B+5) - ((-1)^(i2+1))*Ll_Jt*H_tJ*K_Jy;
            Klc(pos_B+5,pos_B+5) = Klc(pos_B+5,pos_B+5) + Ll_Jt*Ll_Jt*K_Jy;
            Klc(pos_W+2, pos_B+5) = Klc(pos_W+2, pos_B+5) - ((-1)^(i2+1))*Ll_Jt*K_Jy;
            Klc(pos_W+3, pos_B+5) = Klc(pos_W+3, pos_B+5) - ((-1)^(i2+1))*Ll_Jt*H_Jw*K_Jy;
            Klc(pos_W+5, pos_B+5) = Klc(pos_W+5, pos_B+5) - ((-1)^(i2+1))*Ll_Jt*((-1)^(i2+1))*Ll_Jw*K_Jy;            
            % Col 2, 7, 12, 17: yw
            Klc(pos_B+2, pos_W+2) = Klc(pos_B+2, pos_W+2) - K_Jy;
            Klc(pos_B+3,pos_W+2) = Klc(pos_B+3,pos_W+2) + H_tJ*K_Jy;
            Klc(pos_B+5,pos_W+2) = Klc(pos_B+5,pos_W+2) - ((-1)^(i2+1))*Ll_Jt*K_Jy;
            Klc(pos_W+2, pos_W+2) = Klc(pos_W+2, pos_W+2) + K_Jy;
            Klc(pos_W+3, pos_W+2) = Klc(pos_W+3, pos_W+2) + H_Jw*K_Jy;
            Klc(pos_W+5, pos_W+2) = Klc(pos_W+5, pos_W+2) + ((-1)^(i2+1))*Ll_Jw*K_Jy;            
            % Col 3, 8, 13, 18: φw
            Klc(pos_B+2, pos_W+3) = Klc(pos_B+2, pos_W+3) - H_Jw*K_Jy;
            Klc(pos_B+3, pos_W+3) = Klc(pos_B+3, pos_W+3) + H_Jw*H_tJ*K_Jy;
            Klc(pos_B+5, pos_W+3) = Klc(pos_B+5, pos_W+3) - H_Jw*((-1)^(i2+1))*Ll_Jt*K_Jy;
            Klc(pos_W+2, pos_W+3) = Klc(pos_W+2, pos_W+3) + H_Jw*K_Jy;
            Klc(pos_W+3, pos_W+3) = Klc(pos_W+3, pos_W+3) + H_Jw*H_Jw*K_Jy;
            Klc(pos_W+5, pos_W+3) = Klc(pos_W+5, pos_W+3) + H_Jw*((-1)^(i2+1))*Ll_Jw*K_Jy;            
            % Col 5, 10, 15, 20: ψw
            Klc(pos_B+2, pos_W+5) = Klc(pos_B+2, pos_W+5) - ((-1)^(i2+1))*Ll_Jw*K_Jy;
            Klc(pos_B+3, pos_W+5) = Klc(pos_B+3, pos_W+5) + ((-1)^(i2+1))*Ll_Jw*H_tJ*K_Jy;
            Klc(pos_B+5, pos_W+5) = Klc(pos_B+5, pos_W+5) - Ll_Jw*Ll_Jt*K_Jy;
            Klc(pos_W+2, pos_W+5) = Klc(pos_W+2, pos_W+5) + ((-1)^(i2+1))*Ll_Jw*K_Jy;
            Klc(pos_W+3, pos_W+5) = Klc(pos_W+3, pos_W+5) + ((-1)^(i2+1))*Ll_Jw*H_Jw*K_Jy;
            Klc(pos_W+5, pos_W+5) = Klc(pos_W+5, pos_W+5) + Ll_Jw*Ll_Jw*K_Jy;        
        end
    end
end

%% NL-1: 一系垂向减振器 K_DPz, C_DPz
%  K_DPz
for i1 = 1:1:2
    for i2 = 1:1:2
        for i3 = 1:1:2
            pos_P = 35+4*(i1-1)+2*(i2-1)+i3;
            pos_W = 10*(i1-1)+5*(i2-1);
            
            % 36~43列：zp (i=1~8)
            Klc(pos_P, pos_P) = Klc(pos_P, pos_P) + K_DPz;
            Klc(pos_W+1, pos_P) = Klc(pos_W+1, pos_P) - K_DPz;
            Klc(pos_W+3, pos_P) = Klc(pos_W+3, pos_P) - ((-1)^i3)*Lb_DPz*K_DPz;
            Klc(pos_W+4, pos_P) = Klc(pos_W+4, pos_P) - ((-1)^i2)*Ll_DPzw*K_DPz;
            
            % 1, 6, 11, 16列：zw (i=1~4)
            Klc(pos_P, pos_W+1) = Klc(pos_P, pos_W+1) - K_DPz;
            Klc(pos_W+1, pos_W+1) = Klc(pos_W+1, pos_W+1) + K_DPz;
            Klc(pos_W+3, pos_W+1) = Klc(pos_W+3, pos_W+1) + ((-1)^i3)*Lb_DPz*K_DPz;
            Klc(pos_W+4, pos_W+1) = Klc(pos_W+4, pos_W+1) + ((-1)^i2)*Ll_DPzw*K_DPz;
            
            % 3, 8, 13, 18列：φw (i=1~4)
            Klc(pos_P, pos_W+3) = Klc(pos_P, pos_W+3) - ((-1)^i3)*Lb_DPz*K_DPz;
            Klc(pos_W+1, pos_W+3) = Klc(pos_W+1, pos_W+3) + ((-1)^i3)*Lb_DPz*K_DPz;
            Klc(pos_W+3, pos_W+3) = Klc(pos_W+3, pos_W+3) + Lb_DPz*Lb_DPz*K_DPz;
            Klc(pos_W+4, pos_W+3) = Klc(pos_W+4, pos_W+3) + ((-1)^i3)*Lb_DPz*((-1)^i2)*Ll_DPzw*K_DPz;
            
            % 4, 9, 14, 19列：βw (i=1~4)
            Klc(pos_P, pos_W+4) = Klc(pos_P, pos_W+4) - ((-1)^i2)*Ll_DPzw*K_DPz;
            Klc(pos_W+1, pos_W+4) = Klc(pos_W+1, pos_W+4) + ((-1)^i2)*Ll_DPzw*K_DPz;
            Klc(pos_W+3, pos_W+4) = Klc(pos_W+3, pos_W+4) + ((-1)^i2)*Ll_DPzw*((-1)^i3)*Lb_DPz*K_DPz;
            Klc(pos_W+4, pos_W+4) = Klc(pos_W+4, pos_W+4) + Ll_DPzw*Ll_DPzw*K_DPz;
        end
    end
end

% C_DPz
for i1 = 1:1:2
    for i2 = 1:1:2
        for i3 = 1:1:2
            pos_P = 35+4*(i1-1)+2*(i2-1)+i3;
            pos_B = 20+5*(i1-1);
            
            % 21, 26列：zt (i=1~2)
            Clc(pos_B+1, pos_B+1) = Clc(pos_B+1, pos_B+1) + C_DPz;
            Clc(pos_P, pos_B+1) = Clc(pos_P, pos_B+1) - C_DPz;
            Clc(pos_B+3, pos_B+1) = Clc(pos_B+3, pos_B+1) + ((-1)^i3)*Lb_DPz*C_DPz;
            Clc(pos_B+4, pos_B+1) = Clc(pos_B+4, pos_B+1) + ((-1)^i2)*Ll_DPzt* C_DPz;
            
            % 36~43列：zp (i=1~8)
            Clc(pos_B+1, pos_P) = Clc(pos_B+1, pos_P) - C_DPz;
            Clc(pos_P, pos_P) = Clc(pos_P, pos_P) + C_DPz;
            Clc(pos_B+3, pos_P) = Clc(pos_B+3, pos_P) - ((-1)^i3)*Lb_DPz*C_DPz;
            Clc(pos_B+4, pos_P) = Clc(pos_B+4, pos_P) - ((-1)^i2)*Ll_DPzt* C_DPz;
            
            % 23, 28列：φt (i=1~2)
            Clc(pos_B+1, pos_B+3) = Clc(pos_B+1, pos_B+3) + ((-1)^i3)*Lb_DPz*C_DPz;
            Clc(pos_P, pos_B+3) = Clc(pos_P, pos_B+3) - ((-1)^i3)*Lb_DPz*C_DPz;
            Clc(pos_B+3, pos_B+3) = Clc(pos_B+3, pos_B+3) + Lb_DPz*Lb_DPz*C_DPz;
            Clc(pos_B+4, pos_B+3) = Clc(pos_B+4, pos_B+3) + ((-1)^i2)*Ll_DPzt*((-1)^i3)*Lb_DPz*C_DPz;
                       
            % 24, 29列：βt (i=1~2)
            Clc(pos_B+1, pos_B+4) = Clc(pos_B+1, pos_B+4) + ((-1)^i2)*Ll_DPzt* C_DPz;
            Clc(pos_P, pos_B+4) = Clc(pos_P, pos_B+4) - ((-1)^i2)*Ll_DPzt* C_DPz;
            Clc(pos_B+3, pos_B+4) = Clc(pos_B+3, pos_B+4) + ((-1)^i3)*Lb_DPz*((-1)^i2)*Ll_DPzt*C_DPz;
            Clc(pos_B+4, pos_B+4) = Clc(pos_B+4, pos_B+4) + Ll_DPzt*Ll_DPzt*C_DPz;            
        end
    end
end

%% NL-2: 抗蛇行减振器 K_Sx, C_Sx
%  K_Sx
for i1 = 1:1:2
    for i3 = 1:1:2
        pos_P = 35+8+2*(i1-1)+i3;
        pos_B = 20+5*(i1-1);
        
        % Col 44~47: x_Sn (n=1~4)
        Klc(pos_P, pos_P) = Klc(pos_P, pos_P) + K_Sx;
        Klc(pos_B+4, pos_P) = Klc(pos_B+4, pos_P) + H_St*K_Sx;
        Klc(pos_B+5, pos_P) = Klc(pos_B+5, pos_P) - ((-1)^(i3+1))*Lb_S*K_Sx;
        
        % Col 24, 29列: βt (i=1~2)
        Klc(pos_P, pos_B+4) = Klc(pos_P, pos_B+4) + H_St*K_Sx;
        Klc(pos_B+4, pos_B+4) = Klc(pos_B+4, pos_B+4) + H_St*H_St*K_Sx;
        Klc(pos_B+5, pos_B+4) = Klc(pos_B+5, pos_B+4) - H_St*((-1)^(i3+1))*Lb_S*K_Sx;
        
        % Col 25, 30: ψt (i=1~2)
        Klc(pos_P, pos_B+5) = Klc(pos_P, pos_B+5) - ((-1)^(i3+1))*Lb_S*K_Sx;
        Klc(pos_B+4, pos_B+5) = Klc(pos_B+4, pos_B+5) - ((-1)^(i3+1))*Lb_S*H_St*K_Sx;
        Klc(pos_B+5, pos_B+5) = Klc(pos_B+5, pos_B+5) + Lb_S*Lb_S*K_Sx;
    end
end

% C_Sx
for i1 = 1:1:2
    for i3 = 1:1:2
        pos_P = 35+8+2*(i1-1)+i3;
        pos_C = 30;
        
        % Col 34: βc
        Clc(pos_C+4, pos_C+4) = Clc(pos_C+4, pos_C+4) + H_cS*H_cS*C_Sx;
        Clc(pos_C+5, pos_C+4) = Clc(pos_C+5, pos_C+4) + H_cS*((-1)^(i3+1))*Lb_S*C_Sx;
        Clc(pos_P, pos_C+4) = Clc(pos_P, pos_C+4) - H_cS*C_Sx;
        
        % Col 35: ψc
        Clc(pos_C+4, pos_C+5) = Clc(pos_C+4, pos_C+5) + ((-1)^(i3+1))*Lb_S*H_cS*C_Sx;
        Clc(pos_C+5, pos_C+5) = Clc(pos_C+5, pos_C+5) + Lb_S*Lb_S*C_Sx;
        Clc(pos_P, pos_C+5) = Clc(pos_P, pos_C+5) - ((-1)^(i3+1))*Lb_S*C_Sx;
        
        % Col 44~47: x_Sn (n=1~4)
        Clc(pos_C+4, pos_P) = Clc(pos_C+4, pos_P) - H_cS*C_Sx;
        Clc(pos_C+5, pos_P) = Clc(pos_C+5, pos_P) - ((-1)^(i3+1))*Lb_S*C_Sx;
        Clc(pos_P, pos_P) = Clc(pos_P, pos_P) + C_Sx;
    end
end

%% NL-3: 二系横向减振器 K_DPy, C_DPy
%  K_DPy
for i1 = 1:1:2
    for i2 = 1:1:2
        pos_P = 35+8+4+2*(i1-1)+i2;
        pos_B = 20+5*(i1-1);
        
        % pos_P: y_DPy
        Klc(pos_P, pos_P) = Klc(pos_P, pos_P) + K_DPy;
        Klc(pos_B+2, pos_P) = Klc(pos_B+2, pos_P) - K_DPy;
        Klc(pos_B+3, pos_P) = Klc(pos_B+3, pos_P) - H_DPyt*K_DPy;
        Klc(pos_B+5, pos_P) = Klc(pos_B+5, pos_P) - ((-1)^(i2+1))*Ll_DPyt*K_DPy;
        
        % pos_B+2: y_t
        Klc(pos_P, pos_B+2) = Klc(pos_P, pos_B+2) - K_DPy;
        Klc(pos_B+2, pos_B+2) = Klc(pos_B+2, pos_B+2) + K_DPy;
        Klc(pos_B+3, pos_B+2) = Klc(pos_B+3, pos_B+2) + H_DPyt*K_DPy;
        Klc(pos_B+5, pos_B+2) = Klc(pos_B+5, pos_B+2) + ((-1)^(i2+1))*Ll_DPyt*K_DPy;        
        
        % pos_B+3: φ_t
        Klc(pos_P, pos_B+3) = Klc(pos_P, pos_B+3) - H_DPyt*K_DPy;
        Klc(pos_B+2, pos_B+3) = Klc(pos_B+2, pos_B+3) + H_DPyt*K_DPy;
        Klc(pos_B+3, pos_B+3) = Klc(pos_B+3, pos_B+3) + H_DPyt*H_DPyt*K_DPy;
        Klc(pos_B+5, pos_B+3) = Klc(pos_B+5, pos_B+3) + H_DPyt*((-1)^(i2+1))*Ll_DPyt*K_DPy;
        
        % pos_B+5: ψ_t
        Klc(pos_P, pos_B+5) = Klc(pos_P, pos_B+5) - ((-1)^(i2+1))*Ll_DPyt*K_DPy;
        Klc(pos_B+2, pos_B+5) = Klc(pos_B+2, pos_B+5) + ((-1)^(i2+1))*Ll_DPyt*K_DPy;
        Klc(pos_B+3, pos_B+5) = Klc(pos_B+3, pos_B+5) + ((-1)^(i2+1))*Ll_DPyt*H_DPyt*K_DPy;
        Klc(pos_B+5, pos_B+5) = Klc(pos_B+5, pos_B+5) + Ll_DPyt*Ll_DPyt*K_DPy;
    end
end

% C_DPy
for i1 = 1:1:2
    for i2 = 1:1:2
        pos_P = 35+8+4+2*(i1-1)+i2;
        pos_C = 30;

        % 250717发现错误后修改
        if (i1==1&&i2==1) || (i1==2&&i2==2)
            Ll_DPyc = Par_Vehicle.Ll2+Par_Vehicle.Ll_DPyt;
        else
            Ll_DPyc = Par_Vehicle.Ll2-Par_Vehicle.Ll_DPyt;
        end
        
        % pos_C+2: y_c
        Clc(pos_C+2, pos_C+2) = Clc(pos_C+2, pos_C+2) + C_DPy;
        Clc(pos_P, pos_C+2) = Clc(pos_P, pos_C+2) - C_DPy;
        Clc(pos_C+3, pos_C+2) = Clc(pos_C+3, pos_C+2) - H_cDPy*C_DPy;
        Clc(pos_C+5, pos_C+2) = Clc(pos_C+5, pos_C+2) + ((-1)^(i1+1))*Ll_DPyc*C_DPy;
        
        % pos_P: y_DPy
        Clc(pos_C+2, pos_P) = Clc(pos_C+2, pos_P) - C_DPy;
        Clc(pos_P, pos_P) = Clc(pos_P, pos_P) + C_DPy;
        Clc(pos_C+3, pos_P) = Clc(pos_C+3, pos_P) + H_cDPy*C_DPy;
        Clc(pos_C+5, pos_P) = Clc(pos_C+5, pos_P) - ((-1)^(i1+1))*Ll_DPyc*C_DPy;
        
        % pos_C+3: φ_c
        Clc(pos_C+2, pos_C+3) = Clc(pos_C+2, pos_C+3) - H_cDPy*C_DPy;
        Clc(pos_P, pos_C+3) = Clc(pos_P, pos_C+3) + H_cDPy*C_DPy;
        Clc(pos_C+3, pos_C+3) = Clc(pos_C+3, pos_C+3) + H_cDPy*H_cDPy*C_DPy;
        Clc(pos_C+5, pos_C+3) = Clc(pos_C+5, pos_C+3) - H_cDPy*((-1)^(i1+1))*Ll_DPyc*C_DPy;        
        
        % pos_C+5: ψ_c
        Clc(pos_C+2, pos_C+5) = Clc(pos_C+2, pos_C+5) + ((-1)^(i1+1))*Ll_DPyc*C_DPy;
        Clc(pos_P, pos_C+5) = Clc(pos_P, pos_C+5) - ((-1)^(i1+1))*Ll_DPyc*C_DPy;
        Clc(pos_C+3, pos_C+5) = Clc(pos_C+3, pos_C+5) - ((-1)^(i1+1))*Ll_DPyc*H_cDPy*C_DPy;
        Clc(pos_C+5, pos_C+5) = Clc(pos_C+5, pos_C+5) + Ll_DPyc*Ll_DPyc*C_DPy;
    end   
end

%% 牵引拉杆 K_Tx
for i1 = 1:1:2
    pos_B = 20+5*(i1-1);
    pos_C = 30;
    
    % Col 34: βc
    Klc(pos_C+4, pos_C+4) = Klc(pos_C+4, pos_C+4) + H_cT*H_cT*K_Tx;
    Klc(pos_B+4, pos_C+4) = Klc(pos_B+4, pos_C+4) + H_cT*H_Tt*K_Tx;
    
    % Col 24, 29: βt
    Klc(pos_C+4, pos_B+4) = Klc(pos_C+4, pos_B+4) + H_Tt*H_cT*K_Tx;
    Klc(pos_B+4, pos_B+4) = Klc(pos_B+4, pos_B+4) + H_Tt*H_Tt*K_Tx;
end

%% 抗侧滚扭杆 K_ROTx
for i1 = 1:1:2
    pos_B = 20+5*(i1-1);
    pos_C = 30;
    
    % Col 33: βc
    Klc(pos_C+3, pos_C+3) = Klc(pos_C+3, pos_C+3) + K_ROTx;
    Klc(pos_B+3, pos_C+3) = Klc(pos_B+3, pos_C+3) - K_ROTx;
    
    % Col 23, 28: φt
    Klc(pos_C+3, pos_B+3) = Klc(pos_C+3, pos_B+3) - K_ROTx;
    Klc(pos_B+3, pos_B+3) = Klc(pos_B+3, pos_B+3) + K_ROTx;
end

% %%% 抗侧滚扭杆
% for i = 1:1:2
%     Klc((30+3),(30+3)) = Klc((30+3),(30+3)) + Kr;
%     Klc((30+3),(20+5*(i-1)+3)) = Klc((30+3),(20+5*(i-1)+3)) - Kr;
%     Klc((20+5*(i-1)+3),(30+3)) = Klc((20+5*(i-1)+3),(30+3)) - Kr;
%     Klc((20+5*(i-1)+3),(20+5*(i-1)+3)) = Klc((20+5*(i-1)+3),(20+5*(i-1)+3)) + Kr;
% end

% %%% 二系悬挂抗弯曲刚度
% for i = 1:1:2
%     Klc((30+3),(30+3)) = Klc((30+3),(30+3)) + 2*K2b;
%     Klc((30+3),(20+5*(i-1)+3)) = Klc((30+3),(20+5*(i-1)+3)) - 2*K2b;
%     Klc((20+5*(i-1)+3),(30+3)) = Klc((20+5*(i-1)+3),(30+3)) - 2*K2b;
%     Klc((20+5*(i-1)+3),(20+5*(i-1)+3)) = Klc((20+5*(i-1)+3),(20+5*(i-1)+3)) + 2*K2b;
% end
% for i = 1:1:2
%     Klc((30+4),(30+4)) = Klc((30+4),(30+4)) + 2*K2b;
%     Klc((30+4),(20+5*(i-1)+4)) = Klc((30+4),(20+5*(i-1)+4)) - 2*K2b;
%     Klc((20+5*(i-1)+4),(30+4)) = Klc((20+5*(i-1)+4),(30+4)) - 2*K2b;
%     Klc((20+5*(i-1)+4),(20+5*(i-1)+4)) = Klc((20+5*(i-1)+4),(20+5*(i-1)+4)) + 2*K2b;
% end
