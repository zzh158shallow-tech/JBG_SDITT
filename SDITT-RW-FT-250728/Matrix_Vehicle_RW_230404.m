%% 构建多刚体车辆系统 M、C、K 矩阵
% 230403: 补充了一系垂向减振器（串联弹簧）
% 230404: 修正了一系垂向减振器（串联弹簧），补充了抗蛇形减振器（串联弹簧）

function [Mlc, Klc, Clc, Clc_0] = Matrix_Vehicle_RW_230404

global Par_Vehicle N_RV

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

K_Jx = Par_Vehicle.K_Jx;        K_Jy = Par_Vehicle.K_Jy;
K_Sx = Par_Vehicle.K_Sx;
K_DPz = Par_Vehicle.K_DPz;

% C_Sx = Par_Vehicle.C_Sx;
% C_DPz = Par_Vehicle.C_DPz;
C_Sx = Par_Vehicle.C_Sx(1);
C_DPz = Par_Vehicle.C_DPz(1);

Lb1 = Par_Vehicle.Lb1;      Lb2 = Par_Vehicle.Lb2;      Lb_J = Par_Vehicle.Lb_J;    Lb_DPz = Par_Vehicle.Lb_DPz;      Lb_S = Par_Vehicle.Lb_S;
Ll1 = Par_Vehicle.Ll1;         Ll2 = Par_Vehicle.Ll2;         Ll_J = Par_Vehicle.Ll_J;       Ll_DPz = Par_Vehicle.Ll_DPz;

H_cB = Par_Vehicle.H_cB;       H_Bt = Par_Vehicle.H_Bt;          H_tw = Par_Vehicle.H_tw;
H_tJ = Par_Vehicle.H_tJ;          H_St = Par_Vehicle.H_St;          H_cS = Par_Vehicle.H_cS;

%% 列车质量（位移变分为行，加速度为列）
Mlc = zeros(N_RV, N_RV);
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

for kk = 36:1:N_RV
    Mlc(kk, kk)=1e-8;
end

%% 列车刚度（位移变分为行，位移为列）
Klc = zeros(N_RV, N_RV);

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
    
    %%% 31列：zc
    Klc((30+1),(30+1)) = Klc((30+1),(30+1))+K2z;
    Klc((30+3),(30+1)) = Klc((30+3),(30+1))+((-1)^(i1+1))*Lb2*K2z;
    Klc((30+4),(30+1)) = Klc((30+4),(30+1))+((-1)^i1g2)*Ll2*K2z;
    Klc((20+5*(i1g2-1)+1),(30+1)) = Klc((20+5*(i1g2-1)+1),(30+1))-K2z;
    Klc((20+5*(i1g2-1)+3),(30+1)) = Klc((20+5*(i1g2-1)+3),(30+1))-((-1)^(i1+1))*Lb2*K2z;
    %%% 33列：φc    
    Klc((30+1),(30+3)) = Klc((30+1),(30+3))+((-1)^(i1+1))*Lb2*K2z;
    Klc((30+3),(30+3)) = Klc((30+3),(30+3))+Lb2*Lb2*K2z;
    Klc((30+4),(30+3)) = Klc((30+4),(30+3))+((-1)^i1g3)*Lb2*Ll2*K2z;
    Klc((20+5*(i1g2-1)+1),(30+3)) = Klc((20+5*(i1g2-1)+1),(30+3))-((-1)^(i1+1))*Lb2*K2z;
    Klc((20+5*(i1g2-1)+3),(30+3)) = Klc((20+5*(i1g2-1)+3),(30+3))-Lb2*Lb2*K2z;
    %%% 34列：βc     
    Klc((30+1),(30+4)) = Klc((30+1),(30+4))+((-1)^i1g2)*Ll2*K2z;
    Klc((30+3),(30+4)) = Klc((30+3),(30+4))+((-1)^i1g3)*Lb2*Ll2*K2z;
    Klc((30+4),(30+4)) = Klc((30+4),(30+4))+Ll2*Ll2*K2z;
    Klc((20+5*(i1g2-1)+1),(30+4)) = Klc((20+5*(i1g2-1)+1),(30+4))-((-1)^i1g2)*Ll2*K2z;
    Klc((20+5*(i1g2-1)+3),(30+4)) = Klc((20+5*(i1g2-1)+3),(30+4))-((-1)^i1g3)*Lb2*Ll2*K2z;
    %%% 21、26列：zti(i = 1,2)      
    Klc((30+1),(20+5*(i1g2-1)+1)) = Klc((30+1),(20+5*(i1g2-1)+1))-K2z;
    Klc((30+3),(20+5*(i1g2-1)+1)) = Klc((30+3),(20+5*(i1g2-1)+1))-((-1)^(i1+1))*Lb2*K2z;
    Klc((30+4),(20+5*(i1g2-1)+1)) = Klc((30+4),(20+5*(i1g2-1)+1))-((-1)^i1g2)*Ll2*K2z;
    Klc((20+5*(i1g2-1)+1),(20+5*(i1g2-1)+1)) = Klc((20+5*(i1g2-1)+1),(20+5*(i1g2-1)+1))+K2z;
    Klc((20+5*(i1g2-1)+3),(20+5*(i1g2-1)+1)) = Klc((20+5*(i1g2-1)+3),(20+5*(i1g2-1)+1))+((-1)^(i1+1))*Lb2*K2z;
    %%% 23、27列：φti(i = 1,2)    
    Klc((30+1),(20+5*(i1g2-1)+3)) = Klc((30+1),(20+5*(i1g2-1)+3))-((-1)^(i1+1))*Lb2*K2z;
    Klc((30+3),(20+5*(i1g2-1)+3)) = Klc((30+3),(20+5*(i1g2-1)+3))-Lb2*Lb2*K2z;
    Klc((30+4),(20+5*(i1g2-1)+3)) = Klc((30+4),(20+5*(i1g2-1)+3))-((-1)^i1g3)*Lb2*Ll2*K2z;
    Klc((20+5*(i1g2-1)+1),(20+5*(i1g2-1)+3)) = Klc((20+5*(i1g2-1)+1),(20+5*(i1g2-1)+3))+((-1)^(i1+1))*Lb2*K2z;
    Klc((20+5*(i1g2-1)+3),(20+5*(i1g2-1)+3)) = Klc((20+5*(i1g2-1)+3),(20+5*(i1g2-1)+3))+Lb2*Lb2*K2z;    
    
    %%% 32行：δyc    
    Klc((30+2),(30+2)) = Klc((30+2),(30+2))+K2y;
    Klc((30+2),(30+3)) = Klc((30+2),(30+3))-H_cB*K2y;
    Klc((30+2),(30+5)) = Klc((30+2),(30+5))+((-1)^i1g4)*Ll2*K2y;
    Klc((30+2),(20+5*(i1g2-1)+2)) = Klc((30+2),(20+5*(i1g2-1)+2))-K2y;
    Klc((30+2),(20+5*(i1g2-1)+3)) = Klc((30+2),(20+5*(i1g2-1)+3))-H_Bt*K2y;
    %%% 33行：δφc      
    Klc((30+3),(30+2)) = Klc((30+3),(30+2))-H_cB*K2y;
    Klc((30+3),(30+3)) = Klc((30+3),(30+3))+H_cB*H_cB*K2y;
    Klc((30+3),(30+5)) = Klc((30+3),(30+5))-((-1)^i1g4)*Ll2*H_cB*K2y;
    Klc((30+3),(20+5*(i1g2-1)+2)) = Klc((30+3),(20+5*(i1g2-1)+2))+H_cB*K2y;
    Klc((30+3),(20+5*(i1g2-1)+3)) = Klc((30+3),(20+5*(i1g2-1)+3))+H_cB*H_Bt*K2y;
    %%% 35行：δψc       
    Klc((30+5),(30+2)) = Klc((30+5),(30+2))+((-1)^i1g4)*Ll2*K2y;
    Klc((30+5),(30+3)) = Klc((30+5),(30+3))-((-1)^i1g4)*Ll2*H_cB*K2y;
    Klc((30+5),(30+5)) = Klc((30+5),(30+5))+Ll2*Ll2*K2y;
    Klc((30+5),(20+5*(i1g2-1)+2)) = Klc((30+5),(20+5*(i1g2-1)+2))-((-1)^i1g4)*Ll2*K2y;
    Klc((30+5),(20+5*(i1g2-1)+3)) = Klc((30+5),(20+5*(i1g2-1)+3))-((-1)^i1g4)*Ll2*H_Bt*K2y;
    %%% 22、27行：δyti(i = 1,2)     
    Klc((20+5*(i1g2-1)+2),(30+2)) = Klc((20+5*(i1g2-1)+2),(30+2))-K2y;
    Klc((20+5*(i1g2-1)+2),(30+3)) = Klc((20+5*(i1g2-1)+2),(30+3))+H_cB*K2y;
    Klc((20+5*(i1g2-1)+2),(30+5)) = Klc((20+5*(i1g2-1)+2),(30+5))-((-1)^i1g4)*Ll2*K2y;
    Klc((20+5*(i1g2-1)+2),(20+5*(i1g2-1)+2)) = Klc((20+5*(i1g2-1)+2),(20+5*(i1g2-1)+2))+K2y;
    Klc((20+5*(i1g2-1)+2),(20+5*(i1g2-1)+3)) = Klc((20+5*(i1g2-1)+2),(20+5*(i1g2-1)+3))+H_Bt*K2y;
    %%% 23、28行：δφti(i = 1,2)       
    Klc((20+5*(i1g2-1)+3),(30+2)) = Klc((20+5*(i1g2-1)+3),(30+2))-H_Bt*K2y;
    Klc((20+5*(i1g2-1)+3),(30+3)) = Klc((20+5*(i1g2-1)+3),(30+3))+H_cB*H_Bt*K2y;
    Klc((20+5*(i1g2-1)+3),(30+5)) = Klc((20+5*(i1g2-1)+3),(30+5))-((-1)^i1g4)*Ll2*H_Bt*K2y;
    Klc((20+5*(i1g2-1)+3),(20+5*(i1g2-1)+2)) = Klc((20+5*(i1g2-1)+3),(20+5*(i1g2-1)+2))+H_Bt*K2y;
    Klc((20+5*(i1g2-1)+3),(20+5*(i1g2-1)+3)) = Klc((20+5*(i1g2-1)+3),(20+5*(i1g2-1)+3))+H_Bt*H_Bt*K2y;
    
    %%% 34行：δβc   
    Klc((30+4),(30+4))=Klc((30+4),(30+4))+H_cB*H_cB*K2x;
    Klc((30+4),(30+5))=Klc((30+4),(30+5))+((-1)^(i1+1+1))*Lb2*H_cB*K2x;
    Klc((30+4),(20+5*(i1g2-1)+5))=Klc((30+4),(20+5*(i1g2-1)+5))-((-1)^(i1+1+1))*Lb2*H_cB*K2x;
    Klc((30+4),(20+5*(i1g2-1)+4))=Klc((30+4),(20+5*(i1g2-1)+4))+H_cB*H_Bt*K2x;
    %%% 35行：δψc      
    Klc((30+5),(30+4))=Klc((30+5),(30+4))+((-1)^(i1+1+1))*Lb2*H_cB*K2x;
    Klc((30+5),(30+5))=Klc((30+5),(30+5))+Lb2*Lb2*K2x;
    Klc((30+5),(20+5*(i1g2-1)+5))=Klc((30+5),(20+5*(i1g2-1)+5))-Lb2*Lb2*K2x;
    Klc((30+5),(20+5*(i1g2-1)+4))=Klc((30+5),(20+5*(i1g2-1)+4))+((-1)^(i1+1+1))*Lb2*H_Bt*K2x;
    %%% 25、30行：δψti(i=1,2)       
    Klc((20+5*(i1g2-1)+5),(30+4))=Klc((20+5*(i1g2-1)+5),(30+4))-((-1)^(i1+1+1))*Lb2*H_cB*K2x;
    Klc((20+5*(i1g2-1)+5),(30+5))=Klc((20+5*(i1g2-1)+5),(30+5))-Lb2*Lb2*K2x;
    Klc((20+5*(i1g2-1)+5),(20+5*(i1g2-1)+5))=Klc((20+5*(i1g2-1)+5),(20+5*(i1g2-1)+5))+Lb2*Lb2*K2x;
    Klc((20+5*(i1g2-1)+5),(20+5*(i1g2-1)+4))=Klc((20+5*(i1g2-1)+5),(20+5*(i1g2-1)+4))-((-1)^(i1+1+1))*Lb2*H_Bt*K2x;
    %%% 24、29行：δβti(i=1,2)       
    Klc((20+5*(i1g2-1)+4),(30+4))=Klc((20+5*(i1g2-1)+4),(30+4))+H_cB*H_Bt*K2x;
    Klc((20+5*(i1g2-1)+4),(30+5))=Klc((20+5*(i1g2-1)+4),(30+5))+((-1)^(i1+1+1))*Lb2*H_Bt*K2x;
    Klc((20+5*(i1g2-1)+4),(20+5*(i1g2-1)+5))=Klc((20+5*(i1g2-1)+4),(20+5*(i1g2-1)+5))-((-1)^(i1+1+1))*Lb2*H_Bt*K2x;
    Klc((20+5*(i1g2-1)+4),(20+5*(i1g2-1)+4))=Klc((20+5*(i1g2-1)+4),(20+5*(i1g2-1)+4))+H_Bt*H_Bt*K2x;
    
    i1=i1+1;
end

%% 列车阻尼（位移变分为行，速度为列）
Clc=zeros(N_RV, N_RV);
%%% 一系悬挂 C1z、C1y、C1x
i1=0;   % i1 = 0:1:7;
while i1<8
    i1g1=fix((i1+1+3)/4);       %%% 转向架序号 i=1,2
    i1g2=fix((i1+1+1)/2);       %%% 轮对序号 m=1,2,3,4
    i1g3=fix((3*(i1+1)+1)/2);   %%% 车轮位置序号 ...............j+m=2,3,5,6,8,9,11,12 Left/Right
    i1g4=fix((i1+1-1)/2);       %%% m±1=[0,0,1,1,2,2,3,3;]
    
    %%% 21、26列：zti(i=1,2)
    Clc((20+5*(i1g1-1)+1),(20+5*(i1g1-1)+1)) = Clc((20+5*(i1g1-1)+1),(20+5*(i1g1-1)+1))+C1z;
    Clc((20+5*(i1g1-1)+3),(20+5*(i1g1-1)+1)) = Clc((20+5*(i1g1-1)+3),(20+5*(i1g1-1)+1))+((-1)^(i1+1))*Lb1*C1z;
    Clc((20+5*(i1g1-1)+4),(20+5*(i1g1-1)+1)) = Clc((20+5*(i1g1-1)+4),(20+5*(i1g1-1)+1))+((-1)^i1g2)*Ll1*C1z;
    Clc((5*(i1g2-1)+1),(20+5*(i1g1-1)+1)) = Clc((5*(i1g2-1)+1),(20+5*(i1g1-1)+1))-C1z;
    Clc((5*(i1g2-1)+3),(20+5*(i1g1-1)+1)) = Clc((5*(i1g2-1)+3),(20+5*(i1g1-1)+1))-((-1)^(i1+1))*Lb1*C1z;
    %%% 1、6、11、16列：zwm(m = 1,2,3,4)
    Clc((20+5*(i1g1-1)+1),(5*(i1g2-1)+1)) = Clc((20+5*(i1g1-1)+1),(5*(i1g2-1)+1))-C1z;
    Clc((20+5*(i1g1-1)+3),(5*(i1g2-1)+1)) = Clc((20+5*(i1g1-1)+3),(5*(i1g2-1)+1))-((-1)^(i1+1))*Lb1*C1z;
    Clc((20+5*(i1g1-1)+4),(5*(i1g2-1)+1)) = Clc((20+5*(i1g1-1)+4),(5*(i1g2-1)+1))-((-1)^i1g2)*Ll1*C1z;
    Clc((5*(i1g2-1)+1),(5*(i1g2-1)+1)) = Clc((5*(i1g2-1)+1),(5*(i1g2-1)+1))+C1z;
    Clc((5*(i1g2-1)+3),(5*(i1g2-1)+1)) = Clc((5*(i1g2-1)+3),(5*(i1g2-1)+1))+((-1)^(i1+1))*Lb1*C1z;
    %%% 23、28列：φti(i = 1,2)   
    Clc((20+5*(i1g1-1)+1),(20+5*(i1g1-1)+3)) = Clc((20+5*(i1g1-1)+1),(20+5*(i1g1-1)+3))+((-1)^(i1+1))*Lb1*C1z;
    Clc((20+5*(i1g1-1)+3),(20+5*(i1g1-1)+3)) = Clc((20+5*(i1g1-1)+3),(20+5*(i1g1-1)+3))+Lb1*Lb1*C1z;
    Clc((20+5*(i1g1-1)+4),(20+5*(i1g1-1)+3)) = Clc((20+5*(i1g1-1)+4),(20+5*(i1g1-1)+3))+((-1)^i1g3)*Lb1*Ll1*C1z;
    Clc((5*(i1g2-1)+1),(20+5*(i1g1-1)+3)) = Clc((5*(i1g2-1)+1),(20+5*(i1g1-1)+3))-((-1)^(i1+1))*Lb1*C1z;
    Clc((5*(i1g2-1)+3),(20+5*(i1g1-1)+3)) = Clc((5*(i1g2-1)+3),(20+5*(i1g1-1)+3))-Lb1*Lb1*C1z;
    %%% 24、29列：βti(i = 1,2)      
    Clc((20+5*(i1g1-1)+1),(20+5*(i1g1-1)+4)) = Clc((20+5*(i1g1-1)+1),(20+5*(i1g1-1)+4))+((-1)^i1g2)*Ll1*C1z;
    Clc((20+5*(i1g1-1)+3),(20+5*(i1g1-1)+4)) = Clc((20+5*(i1g1-1)+3),(20+5*(i1g1-1)+4))+((-1)^i1g3)*Lb1*Ll1*C1z;
    Clc((20+5*(i1g1-1)+4),(20+5*(i1g1-1)+4)) = Clc((20+5*(i1g1-1)+4),(20+5*(i1g1-1)+4))+Ll1*Ll1*C1z;
    Clc((5*(i1g2-1)+1),(20+5*(i1g1-1)+4)) = Clc((5*(i1g2-1)+1),(20+5*(i1g1-1)+4))-((-1)^i1g2)*Ll1*C1z;
    Clc((5*(i1g2-1)+3),(20+5*(i1g1-1)+4)) = Clc((5*(i1g2-1)+3),(20+5*(i1g1-1)+4))-((-1)^i1g3)*Lb1*Ll1*C1z;
    %%% 3、8、13、18列：φwm(m=1,2,3,4)    
    Clc((20+5*(i1g1-1)+1),(5*(i1g2-1)+3))=Clc((20+5*(i1g1-1)+1),(5*(i1g2-1)+3))-((-1)^(i1+1))*Lb1*C1z;
    Clc((20+5*(i1g1-1)+3),(5*(i1g2-1)+3))=Clc((20+5*(i1g1-1)+3),(5*(i1g2-1)+3))-Lb1*Lb1*C1z;
    Clc((20+5*(i1g1-1)+4),(5*(i1g2-1)+3))=Clc((20+5*(i1g1-1)+4),(5*(i1g2-1)+3))-((-1)^i1g3)*Lb1*Ll1*C1z;
    Clc((5*(i1g2-1)+1),(5*(i1g2-1)+3))=Clc((5*(i1g2-1)+1),(5*(i1g2-1)+3))+((-1)^(i1+1))*Lb1*C1z;
    Clc((5*(i1g2-1)+3),(5*(i1g2-1)+3))=Clc((5*(i1g2-1)+3),(5*(i1g2-1)+3))+Lb1*Lb1*C1z;    
    
    %%% 22、27行：δyti(i = 1,2)
    Clc((20+5*(i1g1-1)+2),(20+5*(i1g1-1)+2)) = Clc((20+5*(i1g1-1)+2),(20+5*(i1g1-1)+2))+C1y;
    Clc((20+5*(i1g1-1)+2),(20+5*(i1g1-1)+3)) = Clc((20+5*(i1g1-1)+2),(20+5*(i1g1-1)+3))-H_tw*C1y;
    Clc((20+5*(i1g1-1)+2),(20+5*(i1g1-1)+5)) = Clc((20+5*(i1g1-1)+2),(20+5*(i1g1-1)+5))+((-1)^i1g4)*Ll1*C1y;
    Clc((20+5*(i1g1-1)+2),(5*(i1g2-1)+2)) = Clc((20+5*(i1g1-1)+2),(5*(i1g2-1)+2))-C1y;
    %%% 23、28行：δφti(i = 1,2)    
    Clc((20+5*(i1g1-1)+3),(20+5*(i1g1-1)+2)) = Clc((20+5*(i1g1-1)+3),(20+5*(i1g1-1)+2))-H_tw*C1y;
    Clc((20+5*(i1g1-1)+3),(20+5*(i1g1-1)+3)) = Clc((20+5*(i1g1-1)+3),(20+5*(i1g1-1)+3))+H_tw*H_tw*C1y;
    Clc((20+5*(i1g1-1)+3),(20+5*(i1g1-1)+5)) = Clc((20+5*(i1g1-1)+3),(20+5*(i1g1-1)+5))-((-1)^i1g4)*Ll1*H_tw*C1y;
    Clc((20+5*(i1g1-1)+3),(5*(i1g2-1)+2)) = Clc((20+5*(i1g1-1)+3),(5*(i1g2-1)+2))+H_tw*C1y;
    %%% 25、30行：δψti(i = 1,2)     
    Clc((20+5*(i1g1-1)+5),(20+5*(i1g1-1)+2)) = Clc((20+5*(i1g1-1)+5),(20+5*(i1g1-1)+2))+((-1)^i1g4)*Ll1*C1y;
    Clc((20+5*(i1g1-1)+5),(20+5*(i1g1-1)+3)) = Clc((20+5*(i1g1-1)+5),(20+5*(i1g1-1)+3))-((-1)^i1g4)*Ll1*H_tw*C1y;
    Clc((20+5*(i1g1-1)+5),(20+5*(i1g1-1)+5)) = Clc((20+5*(i1g1-1)+5),(20+5*(i1g1-1)+5))+Ll1*Ll1*C1y;
    Clc((20+5*(i1g1-1)+5),(5*(i1g2-1)+2)) = Clc((20+5*(i1g1-1)+5),(5*(i1g2-1)+2))-((-1)^i1g4)*Ll1*C1y;
    %%% 2、7、12、17行：δywm(m = 1,2,3,4)      
    Clc((5*(i1g2-1)+2),(20+5*(i1g1-1)+2)) = Clc((5*(i1g2-1)+2),(20+5*(i1g1-1)+2))-C1y;
    Clc((5*(i1g2-1)+2),(20+5*(i1g1-1)+3)) = Clc((5*(i1g2-1)+2),(20+5*(i1g1-1)+3))+H_tw*C1y;
    Clc((5*(i1g2-1)+2),(20+5*(i1g1-1)+5)) = Clc((5*(i1g2-1)+2),(20+5*(i1g1-1)+5))-((-1)^i1g4)*Ll1*C1y;
    Clc((5*(i1g2-1)+2),(5*(i1g2-1)+2)) = Clc((5*(i1g2-1)+2),(5*(i1g2-1)+2))+C1y;
    
    %%% 24、29行：δβti(i = 1,2)     
    Clc((20+5*(i1g1-1)+4),(20+5*(i1g1-1)+4)) = Clc((20+5*(i1g1-1)+4),(20+5*(i1g1-1)+4))+H_tw*H_tw*C1x;
    Clc((20+5*(i1g1-1)+4),(20+5*(i1g1-1)+5)) = Clc((20+5*(i1g1-1)+4),(20+5*(i1g1-1)+5))+((-1)^(i1+1+1))*Lb1*H_tw*C1x;
    Clc((20+5*(i1g1-1)+4),(5*(i1g2-1)+5)) = Clc((20+5*(i1g1-1)+4),(5*(i1g2-1)+5))-((-1)^(i1+1+1))*Lb1*H_tw*C1x;
    %%% 25、30行：δψti(i = 1,2)      
    Clc((20+5*(i1g1-1)+5),(20+5*(i1g1-1)+4)) = Clc((20+5*(i1g1-1)+5),(20+5*(i1g1-1)+4))+((-1)^(i1+1+1))*Lb1*H_tw*C1x;
    Clc((20+5*(i1g1-1)+5),(20+5*(i1g1-1)+5)) = Clc((20+5*(i1g1-1)+5),(20+5*(i1g1-1)+5))+Lb1*Lb1*C1x;
    Clc((20+5*(i1g1-1)+5),(5*(i1g2-1)+5)) = Clc((20+5*(i1g1-1)+5),(5*(i1g2-1)+5))-Lb1*Lb1*C1x;
    %%% 5、10、15、20行：δψwm(m = 1,2,3,4)     
    Clc((5*(i1g2-1)+5),(20+5*(i1g1-1)+4)) = Clc((5*(i1g2-1)+5),(20+5*(i1g1-1)+4))-((-1)^(i1+1+1))*Lb1*H_tw*C1x;
    Clc((5*(i1g2-1)+5),(20+5*(i1g1-1)+5)) = Clc((5*(i1g2-1)+5),(20+5*(i1g1-1)+5))-Lb1*Lb1*C1x;
    Clc((5*(i1g2-1)+5),(5*(i1g2-1)+5)) = Clc((5*(i1g2-1)+5),(5*(i1g2-1)+5))+Lb1*Lb1*C1x;
    
    i1=i1+1;
end

%%% 二系悬挂 C2z、C2y、C2x
i1=0;   % i1 = 0:1:3;
while i1<4
    i1g2=fix((i1+1+1)/2);          %%% 构架序号 i=[1,1,2,2]
    i1g3=fix((3*(i1+1)+1)/2);      %%% 车轮位置序号 j+i=2,3,5,6 Left/Right
    i1g4=fix((i1+1-1)/2);          %%% i±1=[0,0,1,1]
    
    %%% 31列：zc
    Clc((30+1),(30+1)) = Clc((30+1),(30+1))+C2z;
    Clc((30+3),(30+1)) = Clc((30+3),(30+1))+((-1)^(i1+1))*Lb2*C2z;
    Clc((30+4),(30+1)) = Clc((30+4),(30+1))+((-1)^i1g2)*Ll2*C2z;
    Clc((20+5*(i1g2-1)+1),(30+1)) = Clc((20+5*(i1g2-1)+1),(30+1))-C2z;
    Clc((20+5*(i1g2-1)+3),(30+1)) = Clc((20+5*(i1g2-1)+3),(30+1))-((-1)^(i1+1))*Lb2*C2z;
    %%% 33列：φc    
    Clc((30+1),(30+3)) = Clc((30+1),(30+3))+((-1)^(i1+1))*Lb2*C2z;
    Clc((30+3),(30+3)) = Clc((30+3),(30+3))+Lb2*Lb2*C2z;
    Clc((30+4),(30+3)) = Clc((30+4),(30+3))+((-1)^i1g3)*Lb2*Ll2*C2z;
    Clc((20+5*(i1g2-1)+1),(30+3)) = Clc((20+5*(i1g2-1)+1),(30+3))-((-1)^(i1+1))*Lb2*C2z;
    Clc((20+5*(i1g2-1)+3),(30+3)) = Clc((20+5*(i1g2-1)+3),(30+3))-Lb2*Lb2*C2z;
    %%% 34列：βc     
    Clc((30+1),(30+4)) = Clc((30+1),(30+4))+((-1)^i1g2)*Ll2*C2z;
    Clc((30+3),(30+4)) = Clc((30+3),(30+4))+((-1)^i1g3)*Lb2*Ll2*C2z;
    Clc((30+4),(30+4)) = Clc((30+4),(30+4))+Ll2*Ll2*C2z;
    Clc((20+5*(i1g2-1)+1),(30+4)) = Clc((20+5*(i1g2-1)+1),(30+4))-((-1)^i1g2)*Ll2*C2z;
    Clc((20+5*(i1g2-1)+3),(30+4)) = Clc((20+5*(i1g2-1)+3),(30+4))-((-1)^i1g3)*Lb2*Ll2*C2z;
    %%% 21、26列：zti(i = 1,2)      
    Clc((30+1),(20+5*(i1g2-1)+1)) = Clc((30+1),(20+5*(i1g2-1)+1))-C2z;
    Clc((30+3),(20+5*(i1g2-1)+1)) = Clc((30+3),(20+5*(i1g2-1)+1))-((-1)^(i1+1))*Lb2*C2z;
    Clc((30+4),(20+5*(i1g2-1)+1)) = Clc((30+4),(20+5*(i1g2-1)+1))-((-1)^i1g2)*Ll2*C2z;
    Clc((20+5*(i1g2-1)+1),(20+5*(i1g2-1)+1)) = Clc((20+5*(i1g2-1)+1),(20+5*(i1g2-1)+1))+C2z;
    Clc((20+5*(i1g2-1)+3),(20+5*(i1g2-1)+1)) = Clc((20+5*(i1g2-1)+3),(20+5*(i1g2-1)+1))+((-1)^(i1+1))*Lb2*C2z;
    %%% 23、27列：φti(i = 1,2)    
    Clc((30+1),(20+5*(i1g2-1)+3)) = Clc((30+1),(20+5*(i1g2-1)+3))-((-1)^(i1+1))*Lb2*C2z;
    Clc((30+3),(20+5*(i1g2-1)+3)) = Clc((30+3),(20+5*(i1g2-1)+3))-Lb2*Lb2*C2z;
    Clc((30+4),(20+5*(i1g2-1)+3)) = Clc((30+4),(20+5*(i1g2-1)+3))-((-1)^i1g3)*Lb2*Ll2*C2z;
    Clc((20+5*(i1g2-1)+1),(20+5*(i1g2-1)+3)) = Clc((20+5*(i1g2-1)+1),(20+5*(i1g2-1)+3))+((-1)^(i1+1))*Lb2*C2z;
    Clc((20+5*(i1g2-1)+3),(20+5*(i1g2-1)+3)) = Clc((20+5*(i1g2-1)+3),(20+5*(i1g2-1)+3))+Lb2*Lb2*C2z;    
    
    %%% 32行：δyc    
    Clc((30+2),(30+2)) = Clc((30+2),(30+2))+C2y;
    Clc((30+2),(30+3)) = Clc((30+2),(30+3))-H_cB*C2y;
    Clc((30+2),(30+5)) = Clc((30+2),(30+5))+((-1)^i1g4)*Ll2*C2y;
    Clc((30+2),(20+5*(i1g2-1)+2)) = Clc((30+2),(20+5*(i1g2-1)+2))-C2y;
    Clc((30+2),(20+5*(i1g2-1)+3)) = Clc((30+2),(20+5*(i1g2-1)+3))-H_Bt*C2y;
    %%% 33行：δφc      
    Clc((30+3),(30+2)) = Clc((30+3),(30+2))-H_cB*C2y;
    Clc((30+3),(30+3)) = Clc((30+3),(30+3))+H_cB*H_cB*C2y;
    Clc((30+3),(30+5)) = Clc((30+3),(30+5))-((-1)^i1g4)*Ll2*H_cB*C2y;
    Clc((30+3),(20+5*(i1g2-1)+2)) = Clc((30+3),(20+5*(i1g2-1)+2))+H_cB*C2y;
    Clc((30+3),(20+5*(i1g2-1)+3)) = Clc((30+3),(20+5*(i1g2-1)+3))+H_cB*H_Bt*C2y;
    %%% 35行：δψc       
    Clc((30+5),(30+2)) = Clc((30+5),(30+2))+((-1)^i1g4)*Ll2*C2y;
    Clc((30+5),(30+3)) = Clc((30+5),(30+3))-((-1)^i1g4)*Ll2*H_cB*C2y;
    Clc((30+5),(30+5)) = Clc((30+5),(30+5))+Ll2*Ll2*C2y;
    Clc((30+5),(20+5*(i1g2-1)+2)) = Clc((30+5),(20+5*(i1g2-1)+2))-((-1)^i1g4)*Ll2*C2y;
    Clc((30+5),(20+5*(i1g2-1)+3)) = Clc((30+5),(20+5*(i1g2-1)+3))-((-1)^i1g4)*Ll2*H_Bt*C2y;
    %%% 22、27行：δyti(i = 1,2)     
    Clc((20+5*(i1g2-1)+2),(30+2)) = Clc((20+5*(i1g2-1)+2),(30+2))-C2y;
    Clc((20+5*(i1g2-1)+2),(30+3)) = Clc((20+5*(i1g2-1)+2),(30+3))+H_cB*C2y;
    Clc((20+5*(i1g2-1)+2),(30+5)) = Clc((20+5*(i1g2-1)+2),(30+5))-((-1)^i1g4)*Ll2*C2y;
    Clc((20+5*(i1g2-1)+2),(20+5*(i1g2-1)+2)) = Clc((20+5*(i1g2-1)+2),(20+5*(i1g2-1)+2))+C2y;
    Clc((20+5*(i1g2-1)+2),(20+5*(i1g2-1)+3)) = Clc((20+5*(i1g2-1)+2),(20+5*(i1g2-1)+3))+H_Bt*C2y;
    %%% 23、28行：δφti(i = 1,2)       
    Clc((20+5*(i1g2-1)+3),(30+2)) = Clc((20+5*(i1g2-1)+3),(30+2))-H_Bt*C2y;
    Clc((20+5*(i1g2-1)+3),(30+3)) = Clc((20+5*(i1g2-1)+3),(30+3))+H_cB*H_Bt*C2y;
    Clc((20+5*(i1g2-1)+3),(30+5)) = Clc((20+5*(i1g2-1)+3),(30+5))-((-1)^i1g4)*Ll2*H_Bt*C2y;
    Clc((20+5*(i1g2-1)+3),(20+5*(i1g2-1)+2)) = Clc((20+5*(i1g2-1)+3),(20+5*(i1g2-1)+2))+H_Bt*C2y;
    Clc((20+5*(i1g2-1)+3),(20+5*(i1g2-1)+3)) = Clc((20+5*(i1g2-1)+3),(20+5*(i1g2-1)+3))+H_Bt*H_Bt*C2y;
    
    %%% 34行：δβc   
    Clc((30+4),(30+4))=Clc((30+4),(30+4))+H_cB*H_cB*C2x;
    Clc((30+4),(30+5))=Clc((30+4),(30+5))+((-1)^(i1+1+1))*Lb2*H_cB*C2x;
    Clc((30+4),(20+5*(i1g2-1)+5))=Clc((30+4),(20+5*(i1g2-1)+5))-((-1)^(i1+1+1))*Lb2*H_cB*C2x;
    Clc((30+4),(20+5*(i1g2-1)+4))=Clc((30+4),(20+5*(i1g2-1)+4))+H_cB*H_Bt*C2x;
    %%% 35行：δψc      
    Clc((30+5),(30+4))=Clc((30+5),(30+4))+((-1)^(i1+1+1))*Lb2*H_cB*C2x;
    Clc((30+5),(30+5))=Clc((30+5),(30+5))+Lb2*Lb2*C2x;
    Clc((30+5),(20+5*(i1g2-1)+5))=Clc((30+5),(20+5*(i1g2-1)+5))-Lb2*Lb2*C2x;
    Clc((30+5),(20+5*(i1g2-1)+4))=Clc((30+5),(20+5*(i1g2-1)+4))+((-1)^(i1+1+1))*Lb2*H_Bt*C2x;
    %%% 25、30行：δψti(i=1,2)       
    Clc((20+5*(i1g2-1)+5),(30+4))=Clc((20+5*(i1g2-1)+5),(30+4))-((-1)^(i1+1+1))*Lb2*H_cB*C2x;
    Clc((20+5*(i1g2-1)+5),(30+5))=Clc((20+5*(i1g2-1)+5),(30+5))-Lb2*Lb2*C2x;
    Clc((20+5*(i1g2-1)+5),(20+5*(i1g2-1)+5))=Clc((20+5*(i1g2-1)+5),(20+5*(i1g2-1)+5))+Lb2*Lb2*C2x;
    Clc((20+5*(i1g2-1)+5),(20+5*(i1g2-1)+4))=Clc((20+5*(i1g2-1)+5),(20+5*(i1g2-1)+4))-((-1)^(i1+1+1))*Lb2*H_Bt*C2x;
    %%% 24、29行：δβti(i=1,2)       
    Clc((20+5*(i1g2-1)+4),(30+4))=Clc((20+5*(i1g2-1)+4),(30+4))+H_cB*H_Bt*C2x;
    Clc((20+5*(i1g2-1)+4),(30+5))=Clc((20+5*(i1g2-1)+4),(30+5))+((-1)^(i1+1+1))*Lb2*H_Bt*C2x;
    Clc((20+5*(i1g2-1)+4),(20+5*(i1g2-1)+5))=Clc((20+5*(i1g2-1)+4),(20+5*(i1g2-1)+5))-((-1)^(i1+1+1))*Lb2*H_Bt*C2x;
    Clc((20+5*(i1g2-1)+4),(20+5*(i1g2-1)+4))=Clc((20+5*(i1g2-1)+4),(20+5*(i1g2-1)+4))+H_Bt*H_Bt*C2x;
    
    i1=i1+1;
end

Clc_0 = Clc;

%% 一系轴箱转臂节点刚度 K_Jx
for i1 = 1:1:2
    for i2 = 1:1:2
        for i3 = 1:1:2
            pos_W = 10*(i1-1)+5*(i2-1);
            pos_B = 20+5*(i1-1);
            
            % Col 22, 27: yt
            Klc(pos_B+2, pos_B+2) = Klc(pos_B+2, pos_B+2) + K_Jy;
            Klc(pos_B+3,pos_B+2) = Klc(pos_B+3,pos_B+2) - H_tJ*K_Jy;
            Klc(pos_B+5,pos_B+2) = Klc(pos_B+5,pos_B+2) + ((-1)^(i2+1))*Ll_J*K_Jy;
            Klc(pos_W+2, pos_B+2) = Klc(pos_W+2, pos_B+2) - K_Jy;            
            % Col 23, 28: φt
            Klc(pos_B+2, pos_B+3) = Klc(pos_B+2, pos_B+3) - H_tJ*K_Jy;
            Klc(pos_B+3,pos_B+3) = Klc(pos_B+3,pos_B+3) + H_tJ*H_tJ*K_Jy;
            Klc(pos_B+5,pos_B+3) = Klc(pos_B+5,pos_B+3) - H_tJ*((-1)^(i2+1))*Ll_J*K_Jy;
            Klc(pos_W+2, pos_B+3) = Klc(pos_W+2, pos_B+3) + H_tJ*K_Jy;            
            % Col 25, 30: ψt
            Klc(pos_B+2, pos_B+5) = Klc(pos_B+2, pos_B+5) + ((-1)^(i2+1))*Ll_J*K_Jy;
            Klc(pos_B+3,pos_B+5) = Klc(pos_B+3,pos_B+5) - ((-1)^(i2+1))*Ll_J*H_tJ*K_Jy;
            Klc(pos_B+5,pos_B+5) = Klc(pos_B+5,pos_B+5) + Ll_J*Ll_J*K_Jy;
            Klc(pos_W+2, pos_B+5) = Klc(pos_W+2, pos_B+5) - ((-1)^(i2+1))*Ll_J*K_Jy;            
            % Col 2, 7, 12, 17: yw
            Klc(pos_B+2, pos_W+2) = Klc(pos_B+2, pos_W+2) - K_Jy;
            Klc(pos_B+3,pos_W+2) = Klc(pos_B+3,pos_W+2) + H_tJ*K_Jy;
            Klc(pos_B+5,pos_W+2) = Klc(pos_B+5,pos_W+2) - ((-1)^(i2+1))*Ll_J*K_Jy;
            Klc(pos_W+2, pos_W+2) = Klc(pos_W+2, pos_W+2) + K_Jy;
            
            % Col 24, 29: βt
            Klc(pos_B+4, pos_B+4) = Klc(pos_B+4, pos_B+4) + H_tJ*H_tJ*K_Jx;
            Klc(pos_B+5, pos_B+4) = Klc(pos_B+5, pos_B+4) + H_tJ*((-1)^(i3+1))*Lb_J*K_Jx;
            Klc(pos_W+5, pos_B+4) = Klc(pos_W+5, pos_B+4) - H_tJ*((-1)^(i3+1))*Lb_J*K_Jx;            
            % Col 25, 30: ψt
            Klc(pos_B+4, pos_B+5) = Klc(pos_B+4, pos_B+5) + ((-1)^(i3+1))*Lb_J*H_tJ*K_Jx;
            Klc(pos_B+5, pos_B+5) = Klc(pos_B+5, pos_B+5) + Lb_J*Lb_J*K_Jx;
            Klc(pos_W+5, pos_B+5) = Klc(pos_W+5, pos_B+5) - Lb_J*Lb_J*K_Jx;            
            % Col 5, 10, 15, 20: ψw
            Klc(pos_B+4, pos_W+5) = Klc(pos_B+4, pos_W+5) - ((-1)^(i3+1))*Lb_J*H_tJ*K_Jx;
            Klc(pos_B+5, pos_W+5) = Klc(pos_B+5, pos_W+5) - Lb_J*Lb_J*K_Jx;
            Klc(pos_W+5, pos_W+5) = Klc(pos_W+5, pos_W+5) + Lb_J*Lb_J*K_Jx;            
        end
    end
end

%% 一系垂向减振器 K_DPz, C_DPz
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
            
            % 1, 6, 11, 16列：zw (i=1~4)
            Klc(pos_P, pos_W+1) = Klc(pos_P, pos_W+1) - K_DPz;
            Klc(pos_W+1, pos_W+1) = Klc(pos_W+1, pos_W+1) + K_DPz;
            Klc(pos_W+3, pos_W+1) = Klc(pos_W+3, pos_W+1) + ((-1)^i3)*Lb_DPz*K_DPz;
            
            % 3, 8, 13, 18列：φw (i=1~4)
            Klc(pos_P, pos_W+3) = Klc(pos_P, pos_W+3) - ((-1)^i3)*Lb_DPz*K_DPz;
            Klc(pos_W+1, pos_W+3) = Klc(pos_W+1, pos_W+3) + ((-1)^i3)*Lb_DPz*K_DPz;
            Klc(pos_W+3, pos_W+3) = Klc(pos_W+3, pos_W+3) + Lb_DPz*Lb_DPz*K_DPz;
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
            Clc(pos_B+4, pos_B+1) = Clc(pos_B+4, pos_B+1) + ((-1)^i2)*Ll_DPz* C_DPz;
            
            % 36~43列：zp (i=1~8)
            Clc(pos_B+1, pos_P) = Clc(pos_B+1, pos_P) - C_DPz;
            Clc(pos_P, pos_P) = Clc(pos_P, pos_P) + C_DPz;
            Clc(pos_B+3, pos_P) = Clc(pos_B+3, pos_P) - ((-1)^i3)*Lb_DPz*C_DPz;
            Clc(pos_B+4, pos_P) = Clc(pos_B+4, pos_P) - ((-1)^i2)*Ll_DPz* C_DPz;
            
            % 23, 28列：φt (i=1~2)
            Clc(pos_B+1, pos_B+3) = Clc(pos_B+1, pos_B+3) + ((-1)^i3)*Lb_DPz*C_DPz;
            Clc(pos_P, pos_B+3) = Clc(pos_P, pos_B+3) - ((-1)^i3)*Lb_DPz*C_DPz;
            Clc(pos_B+3, pos_B+3) = Clc(pos_B+3, pos_B+3) + Lb_DPz*Lb_DPz*C_DPz;
            Clc(pos_B+4, pos_B+3) = Clc(pos_B+4, pos_B+3) + ((-1)^i2)*Ll_DPz*((-1)^i3)*Lb_DPz*C_DPz;
                       
            % 24, 29列：βt (i=1~2)
            Clc(pos_B+1, pos_B+4) = Clc(pos_B+1, pos_B+4) + ((-1)^i2)*Ll_DPz* C_DPz;
            Clc(pos_P, pos_B+4) = Clc(pos_P, pos_B+4) - ((-1)^i2)*Ll_DPz* C_DPz;
            Clc(pos_B+3, pos_B+4) = Clc(pos_B+3, pos_B+4) + ((-1)^i3)*Lb_DPz*((-1)^i2)*Ll_DPz*C_DPz;
            Clc(pos_B+4, pos_B+4) = Clc(pos_B+4, pos_B+4) + Ll_DPz*Ll_DPz*C_DPz;            
        end
    end
end

%% 抗蛇行减振器 K_Sx, C_Sx
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
