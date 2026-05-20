%% 形成车辆系统（考虑柔性轮对） M、C、K 矩阵
function [Mlc,Klc,Clc] = Matrix_Vehicle_FW(DampingRatio)

global Par_Vehicle Par_FW NM_FW Nw ModeFreq ModeShape

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
K1z = Par_Vehicle.K1z; C1z = Par_Vehicle.C1z;
K1y = Par_Vehicle.K1y; C1y = Par_Vehicle.C1y;
K1x = Par_Vehicle.K1x; C1x = Par_Vehicle.C1x;
K2z = Par_Vehicle.K2z; C2z = Par_Vehicle.C2z;
K2y = Par_Vehicle.K2y; C2y = Par_Vehicle.C2y;
K2x = Par_Vehicle.K2x; C2x = Par_Vehicle.C2x;
Lb1 = Par_Vehicle.Lb1; Lb2 = Par_Vehicle.Lb2;
Ll1 = Par_Vehicle.Ll1; Ll2 = Par_Vehicle.Ll2;
H2 = Par_Vehicle.H2; H3 = Par_Vehicle.H3; H4 = Par_Vehicle.H4;
Kr = Par_Vehicle.Kr; K2b = Par_Vehicle.K2b;
      
%%% ===================== 列车质量（位移变分为行，加速度为列）
Mlc = zeros(NM_FW*Nw+35,NM_FW*Nw+35);
Mlc(1:NM_FW*Nw,1:NM_FW*Nw) = diag(ones(NM_FW*Nw,1),0);
for i1 = 1:1:4
    pos_WS = NM_FW*Nw+5*(i1-1);
    Mlc(pos_WS+1,pos_WS+1) = Mw;
    Mlc(pos_WS+2,pos_WS+2) = Mw;
    Mlc(pos_WS+3,pos_WS+3) = Jwx;         %轮对侧滚惯量 Jwf  kg.m^2
    Mlc(pos_WS+4,pos_WS+4) = Jwy;         %轮对点头惯量 Jwf  kg.m^2
    Mlc(pos_WS+5,pos_WS+5) = Jwz;         %轮对摇头惯量 Jwc  kg.m^2
end
for i1 = 1:1:2
    pos_BG = NM_FW*Nw+5*(i1-1)+20;
    Mlc(pos_BG+1,pos_BG+1) = Mb;
    Mlc(pos_BG+2,pos_BG+2) = Mb;
    Mlc(pos_BG+3,pos_BG+3) = Jbx;
    Mlc(pos_BG+4,pos_BG+4) = Jby;
    Mlc(pos_BG+5,pos_BG+5) = Jbz;
end
pos_CB = NM_FW*Nw+30;
Mlc(pos_CB+1,pos_CB+1) = Mc;
Mlc(pos_CB+2,pos_CB+2) = Mc;
Mlc(pos_CB+3,pos_CB+3) = Jcx;
Mlc(pos_CB+4,pos_CB+4) = Jcy;
Mlc(pos_CB+5,pos_CB+5) = Jcz;

%%% ===================== 列车刚度（位移变分为行，位移为列）
Klc = zeros(NM_FW*Nw+35,NM_FW*Nw+35);
%%% 一系悬挂 K1z、K1y、K1x
i1=0;   % i1 = 0:1:7;
while i1<8
    i1g1=fix((i1+1+3)/4);       %%% 构架序号 i=1,2
    i1g2=fix((i1+1+1)/2);       %%% 轮对序号 m=1,2,3,4
    i1g3=fix((3*(i1+1)+1)/2);   %%% 车轮位置序号 ...............j+m=2,3,5,6,8,9,11,12 Left/Right
    i1g4=fix((i1+1-1)/2);       %%% m±1=[0,0,1,1,2,2,3,3;]
    
    if mod(i1+1,2)==0           %%% 车轮编号 j = i1+1 位于右侧
        DOF_pos_AB = Par_FW.DOF_pos.AxleBox_R;
    else                                     %%% 车轮编号 j = i1+1 位于左侧
        DOF_pos_AB = Par_FW.DOF_pos.AxleBox_L;
    end
    
    pos_BG = NM_FW*Nw+5*(i1g1-1)+20;
    pos_WS = NM_FW*Nw+5*(i1g2-1);
    
    %%% pos_BG+4行：δβti (i = 1,2)——K1x
    Klc(pos_BG+4,pos_BG+4) = Klc(pos_BG+4,pos_BG+4) + H4*H4*K1x;
    Klc(pos_BG+4,pos_BG+5) = Klc(pos_BG+4,pos_BG+5) + (-1)^(i1+1+1)*Lb1*H4*K1x;
    Klc(pos_BG+4,pos_WS+5) = Klc(pos_BG+4,pos_WS+5)  - (-1)^(i1+1+1)*Lb1*H4*K1x;
    for k = 1:1:NM_FW
        Klc(pos_BG+4,(i1g2-1)*NM_FW+k) = Klc(pos_BG+4,(i1g2-1)*NM_FW+k) ...
                                                                          -ModeShape.FW(DOF_pos_AB(1),k)*H4*K1x;
    end
    %%% pos_BG+5行：δψti (i = 1,2)——K1x
    Klc(pos_BG+5,pos_BG+4) = Klc(pos_BG+5,pos_BG+4) + (-1)^(i1+1+1)*Lb1*H4*K1x;
    Klc(pos_BG+5,pos_BG+5) = Klc(pos_BG+5,pos_BG+5) + Lb1*Lb1*K1x;
    Klc(pos_BG+5,pos_WS+5)  = Klc(pos_BG+5,pos_WS+5)  - Lb1*Lb1*K1x;
    for k = 1:1:NM_FW
        Klc(pos_BG+5,(i1g2-1)*NM_FW+k) = Klc(pos_BG+5,(i1g2-1)*NM_FW+k) ...
                                                                          -ModeShape.FW(DOF_pos_AB(1),k)*(-1)^(i1+1+1)*Lb1*K1x;
    end
    %%% pos_WS+5行：δψwm (m = 1,2,3,4)——K1x
    Klc(pos_WS+5,pos_BG+4) = Klc(pos_WS+5,pos_BG+4) - (-1)^(i1+1+1)*Lb1*H4*K1x;
    Klc(pos_WS+5,pos_BG+5) = Klc(pos_WS+5,pos_BG+5) - Lb1*Lb1*K1x;
    Klc(pos_WS+5,pos_WS+5)  = Klc(pos_WS+5,pos_WS+5)  + Lb1*Lb1*K1x;
    for k = 1:1:NM_FW
        Klc(pos_WS+5,(i1g2-1)*NM_FW+k) = Klc(pos_WS+5,(i1g2-1)*NM_FW+k) ...
                                                                         +ModeShape.FW(DOF_pos_AB(1),k)*(-1)^(i1+1+1)*Lb1*K1x;
    end
    %%% (i1g2-1)*NM+k行：δqk (k = 1,2,...,NM)——K1x
    for k = 1:1:NM_FW
        Klc((i1g2-1)*NM_FW+k,pos_BG+4) = Klc((i1g2-1)*NM_FW+k,pos_BG+4) ...
                                                                          -ModeShape.FW(DOF_pos_AB(1),k)*H4*K1x;
        Klc((i1g2-1)*NM_FW+k,pos_BG+5) = Klc((i1g2-1)*NM_FW+k,pos_BG+5) ...
                                                                          -ModeShape.FW(DOF_pos_AB(1),k)*(-1)^(i1+1+1)*Lb1*K1x;
        Klc((i1g2-1)*NM_FW+k,pos_WS+5) = Klc((i1g2-1)*NM_FW+k,pos_WS+5) ...
                                                                         +ModeShape.FW(DOF_pos_AB(1),k)*(-1)^(i1+1+1)*Lb1*K1x;
        for kk = 1:1:NM_FW
            Klc((i1g2-1)*NM_FW+k,(i1g2-1)*NM_FW+kk) = Klc((i1g2-1)*NM_FW+k,(i1g2-1)*NM_FW+kk) ...
                                                                                                +ModeShape.FW(DOF_pos_AB(1),k)*ModeShape.FW(DOF_pos_AB(1),kk)*K1x;
        end
    end
    
    %%% pos_BG+2行：δyti(i = 1,2)——K1y
    Klc(pos_BG+2,pos_BG+2) = Klc(pos_BG+2,pos_BG+2) + K1y;
    Klc(pos_BG+2,pos_WS+2) = Klc(pos_BG+2,pos_WS+2) - K1y;
    Klc(pos_BG+2,pos_BG+3) = Klc(pos_BG+2,pos_BG+3) - H4*K1y;
    Klc(pos_BG+2,pos_BG+5) = Klc(pos_BG+2,pos_BG+5) + (-1)^i1g4*Ll1*K1y;
    for k = 1:1:NM_FW
        Klc(pos_BG+2,(i1g2-1)*NM_FW+k) = Klc(pos_BG+2,(i1g2-1)*NM_FW+k) ...
                                                                          -ModeShape.FW(DOF_pos_AB(2),k)*K1y;
    end
    %%% pos_WS+2行：δywm(m = 1,2,3,4)——K1y
    Klc(pos_WS+2,pos_BG+2) = Klc(pos_WS+2,pos_BG+2) - K1y;
    Klc(pos_WS+2,pos_WS+2)  = Klc(pos_WS+2,pos_WS+2)  + K1y;
    Klc(pos_WS+2,pos_BG+3) = Klc(pos_WS+2,pos_BG+3) + H4*K1y;
    Klc(pos_WS+2,pos_BG+5) = Klc(pos_WS+2,pos_BG+5) - (-1)^i1g4*Ll1*K1y;
    for k = 1:1:NM_FW
        Klc(pos_WS+2,(i1g2-1)*NM_FW+k) = Klc(pos_WS+2,(i1g2-1)*NM_FW+k) ...
                                                                          +ModeShape.FW(DOF_pos_AB(2),k)*K1y;
    end
    %%% pos_BG+3行：δφti(i = 1,2)——K1y
    Klc(pos_BG+3,pos_BG+2) = Klc(pos_BG+3,pos_BG+2) - H4*K1y;
    Klc(pos_BG+3,pos_WS+2)  = Klc(pos_BG+3,pos_WS+2)  + H4*K1y;
    Klc(pos_BG+3,pos_BG+3) = Klc(pos_BG+3,pos_BG+3) + H4*H4*K1y;
    Klc(pos_BG+3,pos_BG+5) = Klc(pos_BG+3,pos_BG+5) - H4*(-1)^i1g4*Ll1*K1y;
    for k = 1:1:NM_FW
        Klc(pos_BG+3,(i1g2-1)*NM_FW+k) = Klc(pos_BG+3,(i1g2-1)*NM_FW+k) ...
                                                                          +ModeShape.FW(DOF_pos_AB(2),k)*H4*K1y;
    end
    %%% pos_BG+5行：δψti(i = 1,2)——K1y
    Klc(pos_BG+5,pos_BG+2) = Klc(pos_BG+5,pos_BG+2) + (-1)^i1g4*Ll1*K1y;
    Klc(pos_BG+5,pos_WS+2)  = Klc(pos_BG+5,pos_WS+2)  - (-1)^i1g4*Ll1*K1y;
    Klc(pos_BG+5,pos_BG+3) = Klc(pos_BG+5,pos_BG+3) - (-1)^i1g4*Ll1*H4*K1y;
    Klc(pos_BG+5,pos_BG+5) = Klc(pos_BG+5,pos_BG+5) + Ll1*Ll1*K1y;
    for k = 1:1:NM_FW
        Klc(pos_BG+5,(i1g2-1)*NM_FW+k) = Klc(pos_BG+5,(i1g2-1)*NM_FW+k) ...
                                                                          -ModeShape.FW(DOF_pos_AB(2),k)*(-1)^i1g4*Ll1*K1y;
    end
    %%% (i1g2-1)*NM+k行：δqk (k = 1,2,...,NM)——K1y
    for k = 1:1:NM_FW
        Klc((i1g2-1)*NM_FW+k,pos_BG+2) = Klc((i1g2-1)*NM_FW+k,pos_BG+2) ...
                                                                          -ModeShape.FW(DOF_pos_AB(2),k)*K1y;
        Klc((i1g2-1)*NM_FW+k,pos_WS+2)  = Klc((i1g2-1)*NM_FW+k,pos_WS+2) ...
                                                                          +ModeShape.FW(DOF_pos_AB(2),k)*K1y;                                          
        Klc((i1g2-1)*NM_FW+k,pos_BG+3) = Klc((i1g2-1)*NM_FW+k,pos_BG+3) ...
                                                                          +ModeShape.FW(DOF_pos_AB(2),k)*H4*K1y;
        Klc((i1g2-1)*NM_FW+k,pos_BG+5) = Klc((i1g2-1)*NM_FW+k,pos_BG+5) ...
                                                                          -ModeShape.FW(DOF_pos_AB(2),k)*(-1)^i1g4*Ll1*K1y;
        for kk = 1:1:NM_FW
            Klc((i1g2-1)*NM_FW+k,(i1g2-1)*NM_FW+kk) = Klc((i1g2-1)*NM_FW+k,(i1g2-1)*NM_FW+kk) ...
                                                                                               +ModeShape.FW(DOF_pos_AB(2),k)*ModeShape.FW(DOF_pos_AB(2),kk)*K1y;
        end
    end
    
    %%% pos_BG+1行：δzti(i = 1,2)——K1z
    Klc(pos_BG+1,pos_BG+1) = Klc(pos_BG+1,pos_BG+1) + K1z;
    Klc(pos_BG+1,pos_WS+1)  = Klc(pos_BG+1,pos_WS+1)  - K1z;
    Klc(pos_BG+1,pos_BG+3) = Klc(pos_BG+1,pos_BG+3) + (-1)^(i1+1)*Lb1*K1z;
    Klc(pos_BG+1,pos_WS+3)  = Klc(pos_BG+1,pos_WS+3)  - (-1)^(i1+1)*Lb1*K1z;
    Klc(pos_BG+1,pos_BG+4) = Klc(pos_BG+1,pos_BG+4) + (-1)^i1g2*Ll1*K1z;
    for k = 1:1:NM_FW
        Klc(pos_BG+1,(i1g2-1)*NM_FW+k) = Klc(pos_BG+1,(i1g2-1)*NM_FW+k) ...
                                                                          -ModeShape.FW(DOF_pos_AB(3),k)*K1z;
    end
    %%% pos_WS+1行：δzwm(m = 1,2,3,4)——K1z
    Klc(pos_WS+1,pos_BG+1) = Klc(pos_WS+1,pos_BG+1) - K1z;
    Klc(pos_WS+1,pos_WS+1) = Klc(pos_WS+1,pos_WS+1)  + K1z;
    Klc(pos_WS+1,pos_BG+3) = Klc(pos_WS+1,pos_BG+3) - (-1)^(i1+1)*Lb1*K1z;
    Klc(pos_WS+1,pos_WS+3) = Klc(pos_WS+1,pos_WS+3)  + (-1)^(i1+1)*Lb1*K1z;
    Klc(pos_WS+1,pos_BG+4) = Klc(pos_WS+1,pos_BG+4) - (-1)^i1g2*Ll1*K1z;
    for k = 1:1:NM_FW
        Klc(pos_WS+1,(i1g2-1)*NM_FW+k) = Klc(pos_WS+1,(i1g2-1)*NM_FW+k) ...
                                                                         +ModeShape.FW(DOF_pos_AB(3),k)*K1z;
    end
    %%% pos_BG+3行：δφti(i = 1,2)——K1z
    Klc(pos_BG+3,pos_BG+1) = Klc(pos_BG+3,pos_BG+1) + (-1)^(i1+1)*Lb1*K1z;
    Klc(pos_BG+3,pos_WS+1)  = Klc(pos_BG+3,pos_WS+1)  - (-1)^(i1+1)*Lb1*K1z;
    Klc(pos_BG+3,pos_BG+3) = Klc(pos_BG+3,pos_BG+3) + Lb1*Lb1*K1z;
    Klc(pos_BG+3,pos_WS+3)  = Klc(pos_BG+3,pos_WS+3)  - Lb1*Lb1*K1z;
    Klc(pos_BG+3,pos_BG+4) = Klc(pos_BG+3,pos_BG+4) + (-1)^i1g3*Lb1*Ll1*K1z;
    for k = 1:1:NM_FW
        Klc(pos_BG+3,(i1g2-1)*NM_FW+k) = Klc(pos_BG+3,(i1g2-1)*NM_FW+k) ...
                                                                          -ModeShape.FW(DOF_pos_AB(3),k)*(-1)^(i1+1)*Lb1*K1z;
    end
    %%% pos_WS+3行：δφwm(m = 1,2,3,4)——K1z
    Klc(pos_WS+3,pos_BG+1) = Klc(pos_WS+3,pos_BG+1) - (-1)^(i1+1)*Lb1*K1z;
    Klc(pos_WS+3,pos_WS+1)  = Klc(pos_WS+3,pos_WS+1)  + (-1)^(i1+1)*Lb1*K1z;
    Klc(pos_WS+3,pos_BG+3) = Klc(pos_WS+3,pos_BG+3) - Lb1*Lb1*K1z;
    Klc(pos_WS+3,pos_WS+3)  = Klc(pos_WS+3,pos_WS+3)  + Lb1*Lb1*K1z;
    Klc(pos_WS+3,pos_BG+4) = Klc(pos_WS+3,pos_BG+4) - (-1)^i1g3*Lb1*Ll1*K1z;
    for k = 1:1:NM_FW
        Klc(pos_WS+3,(i1g2-1)*NM_FW+k) = Klc(pos_WS+3,(i1g2-1)*NM_FW+k) ...
                                                                         +ModeShape.FW(DOF_pos_AB(3),k)*(-1)^(i1+1)*Lb1*K1z;
    end
    %%% pos_BG+4行：δβti(i = 1,2)——K1z
    Klc(pos_BG+4,pos_BG+1) = Klc(pos_BG+4,pos_BG+1) + (-1)^i1g2*Ll1*K1z;
    Klc(pos_BG+4,pos_WS+1)  = Klc(pos_BG+4,pos_WS+1)  - (-1)^i1g2*Ll1*K1z;
    Klc(pos_BG+4,pos_BG+3) = Klc(pos_BG+4,pos_BG+3) + (-1)^i1g3*Lb1*Ll1*K1z;
    Klc(pos_BG+4,pos_WS+3)  = Klc(pos_BG+4,pos_WS+3)  - (-1)^i1g3*Lb1*Ll1*K1z;
    Klc(pos_BG+4,pos_BG+4) = Klc(pos_BG+4,pos_BG+4) + Ll1*Ll1*K1z;
    for k = 1:1:NM_FW
        Klc(pos_BG+4,(i1g2-1)*NM_FW+k) = Klc(pos_BG+4,(i1g2-1)*NM_FW+k) ...
                                                                          -ModeShape.FW(DOF_pos_AB(3),k)*(-1)^i1g2*Ll1*K1z;
    end
    %%% (i1g2-1)*NM+k行：δqk (k = 1,2,...,NM)——K1z
    for k = 1:1:NM_FW
        Klc((i1g2-1)*NM_FW+k,pos_BG+1) = Klc((i1g2-1)*NM_FW+k,pos_BG+1) ...
                                                                         -ModeShape.FW(DOF_pos_AB(3),k)*K1z;
        Klc((i1g2-1)*NM_FW+k,pos_WS+1) = Klc((i1g2-1)*NM_FW+k,pos_WS+1) ...
                                                                         +ModeShape.FW(DOF_pos_AB(3),k)*K1z;
        Klc((i1g2-1)*NM_FW+k,pos_BG+3) = Klc((i1g2-1)*NM_FW+k,pos_BG+3) ...
                                                                         -ModeShape.FW(DOF_pos_AB(3),k)*(-1)^(i1+1)*Lb1*K1z;
        Klc((i1g2-1)*NM_FW+k,pos_WS+3) = Klc((i1g2-1)*NM_FW+k,pos_WS+3) ...
                                                                         +ModeShape.FW(DOF_pos_AB(3),k)*(-1)^(i1+1)*Lb1*K1z;
        Klc((i1g2-1)*NM_FW+k,pos_BG+4) = Klc((i1g2-1)*NM_FW+k,pos_BG+4) ...
                                                                         -ModeShape.FW(DOF_pos_AB(3),k)*(-1)^i1g2*Ll1*K1z;
        for kk = 1:1:NM_FW
            Klc((i1g2-1)*NM_FW+k,(i1g2-1)*NM_FW+kk) = Klc((i1g2-1)*NM_FW+k,(i1g2-1)*NM_FW+kk) ...
                                                                                                +ModeShape.FW(DOF_pos_AB(3),k)*ModeShape.FW(DOF_pos_AB(3),kk)*K1z;
        end
    end    
    
    i1=i1+1;
end

%%% 圆频率矩阵 Ω=2πf
for i = 1:1:Nw
    for k = 1:1:NM_FW
        pos = (i-1)*NM_FW+k;
        Klc(pos,pos) = Klc(pos,pos) + (2*pi*ModeFreq.FW(k,2))^2;        
    end
end

%%% 二系悬挂 K2z、K2y、K2x
i1=0;   % i1 = 0:1:3;
while i1<4
    i1g2=fix((i1+1+1)/2);          %%% 构架序号 i=[1,1,2,2]
    i1g3=fix((3*(i1+1)+1)/2);      %%% 车轮位置序号 j+i=2,3,5,6 Left/Right
    i1g4=fix((i1+1-1)/2);          %%% i±1=[0,0,1,1]（其实就是 i+1=[2,2,3,3]）
    
    pos_CB = NM_FW*Nw+30;
    pos_BG = NM_FW*Nw+5*(i1g2-1)+20;
    
    %%% pos_CB+1列：zc——K2z
    Klc(pos_CB+1,pos_CB+1) = Klc(pos_CB+1,pos_CB+1)+K2z;
    Klc(pos_CB+3,pos_CB+1) = Klc(pos_CB+3,pos_CB+1)+((-1)^(i1+1))*Lb2*K2z;
    Klc(pos_CB+4,pos_CB+1) = Klc(pos_CB+4,pos_CB+1)+((-1)^i1g2)*Ll2*K2z;
    Klc(pos_BG+1,pos_CB+1) = Klc(pos_BG+1,pos_CB+1)-K2z;
    Klc(pos_BG+3,pos_CB+1) = Klc(pos_BG+3,pos_CB+1)-((-1)^(i1+1))*Lb2*K2z;
    %%% pos_CB+3列：φc——K2z
    Klc((pos_CB+1),(pos_CB+3)) = Klc((pos_CB+1),(pos_CB+3))+((-1)^(i1+1))*Lb2*K2z;
    Klc((pos_CB+3),(pos_CB+3)) = Klc((pos_CB+3),(pos_CB+3))+Lb2*Lb2*K2z;
    Klc((pos_CB+4),(pos_CB+3)) = Klc((pos_CB+4),(pos_CB+3))+((-1)^i1g3)*Lb2*Ll2*K2z;
    Klc((pos_BG+1),(pos_CB+3)) = Klc((pos_BG+1),(pos_CB+3))-((-1)^(i1+1))*Lb2*K2z;
    Klc((pos_BG+3),(pos_CB+3)) = Klc((pos_BG+3),(pos_CB+3))-Lb2*Lb2*K2z;
    %%% pos_CB+4列：βc——K2z
    Klc((pos_CB+1),(pos_CB+4)) = Klc((pos_CB+1),(pos_CB+4))+((-1)^i1g2)*Ll2*K2z;
    Klc((pos_CB+3),(pos_CB+4)) = Klc((pos_CB+3),(pos_CB+4))+((-1)^i1g3)*Lb2*Ll2*K2z;
    Klc((pos_CB+4),(pos_CB+4)) = Klc((pos_CB+4),(pos_CB+4))+Ll2*Ll2*K2z;
    Klc((pos_BG+1),(pos_CB+4)) = Klc((pos_BG+1),(pos_CB+4))-((-1)^i1g2)*Ll2*K2z;
    Klc((pos_BG+3),(pos_CB+4)) = Klc((pos_BG+3),(pos_CB+4))-((-1)^i1g3)*Lb2*Ll2*K2z;
    %%% pos_BG+1列：zti(i = 1,2)——K2z      
    Klc((pos_CB+1),(pos_BG+1)) = Klc((pos_CB+1),(pos_BG+1))-K2z;
    Klc((pos_CB+3),(pos_BG+1)) = Klc((pos_CB+3),(pos_BG+1))-((-1)^(i1+1))*Lb2*K2z;
    Klc((pos_CB+4),(pos_BG+1)) = Klc((pos_CB+4),(pos_BG+1))-((-1)^i1g2)*Ll2*K2z;
    Klc((pos_BG+1),(pos_BG+1)) = Klc((pos_BG+1),(pos_BG+1))+K2z;
    Klc((pos_BG+3),(pos_BG+1)) = Klc((pos_BG+3),(pos_BG+1))+((-1)^(i1+1))*Lb2*K2z;
    %%% pos_BG+3列：φti(i = 1,2)——K2z
    Klc((pos_CB+1),(pos_BG+3)) = Klc((pos_CB+1),(pos_BG+3))-((-1)^(i1+1))*Lb2*K2z;
    Klc((pos_CB+3),(pos_BG+3)) = Klc((pos_CB+3),(pos_BG+3))-Lb2*Lb2*K2z;
    Klc((pos_CB+4),(pos_BG+3)) = Klc((pos_CB+4),(pos_BG+3))-((-1)^i1g3)*Lb2*Ll2*K2z;
    Klc((pos_BG+1),(pos_BG+3)) = Klc((pos_BG+1),(pos_BG+3))+((-1)^(i1+1))*Lb2*K2z;
    Klc((pos_BG+3),(pos_BG+3)) = Klc((pos_BG+3),(pos_BG+3))+Lb2*Lb2*K2z;
    
    %%% pos_CB+2行：δyc——K2y
    Klc((pos_CB+2),(pos_CB+2)) = Klc((pos_CB+2),(pos_CB+2))+K2y;
    Klc((pos_CB+2),(pos_CB+3)) = Klc((pos_CB+2),(pos_CB+3))-H2*K2y;
    Klc((pos_CB+2),(pos_CB+5)) = Klc((pos_CB+2),(pos_CB+5))+((-1)^i1g4)*Ll2*K2y;
    Klc((pos_CB+2),(pos_BG+2)) = Klc((pos_CB+2),(pos_BG+2))-K2y;
    Klc((pos_CB+2),(pos_BG+3)) = Klc((pos_CB+2),(pos_BG+3))-H3*K2y;
    %%% pos_CB+3行：δφc——K2y
    Klc((pos_CB+3),(pos_CB+2)) = Klc((pos_CB+3),(pos_CB+2))-H2*K2y;
    Klc((pos_CB+3),(pos_CB+3)) = Klc((pos_CB+3),(pos_CB+3))+H2*H2*K2y;
    Klc((pos_CB+3),(pos_CB+5)) = Klc((pos_CB+3),(pos_CB+5))-((-1)^i1g4)*Ll2*H2*K2y;
    Klc((pos_CB+3),(pos_BG+2)) = Klc((pos_CB+3),(pos_BG+2))+H2*K2y;
    Klc((pos_CB+3),(pos_BG+3)) = Klc((pos_CB+3),(pos_BG+3))+H2*H3*K2y;
    %%% pos_CB+5行：δψc——K2y
    Klc((pos_CB+5),(pos_CB+2)) = Klc((pos_CB+5),(pos_CB+2))+((-1)^i1g4)*Ll2*K2y;
    Klc((pos_CB+5),(pos_CB+3)) = Klc((pos_CB+5),(pos_CB+3))-((-1)^i1g4)*Ll2*H2*K2y;
    Klc((pos_CB+5),(pos_CB+5)) = Klc((pos_CB+5),(pos_CB+5))+Ll2*Ll2*K2y;
    Klc((pos_CB+5),(pos_BG+2)) = Klc((pos_CB+5),(pos_BG+2))-((-1)^i1g4)*Ll2*K2y;
    Klc((pos_CB+5),(pos_BG+3)) = Klc((pos_CB+5),(pos_BG+3))-((-1)^i1g4)*Ll2*H3*K2y;
    %%% pos_BG+2行：δyti(i = 1,2)——K2y
    Klc((pos_BG+2),(pos_CB+2)) = Klc((pos_BG+2),(pos_CB+2))-K2y;
    Klc((pos_BG+2),(pos_CB+3)) = Klc((pos_BG+2),(pos_CB+3))+H2*K2y;
    Klc((pos_BG+2),(pos_CB+5)) = Klc((pos_BG+2),(pos_CB+5))-((-1)^i1g4)*Ll2*K2y;
    Klc((pos_BG+2),(pos_BG+2)) = Klc((pos_BG+2),(pos_BG+2))+K2y;
    Klc((pos_BG+2),(pos_BG+3)) = Klc((pos_BG+2),(pos_BG+3))+H3*K2y;       
    %%% pos_BG+3行：δφti(i = 1,2)——K2y
    Klc((pos_BG+3),(pos_CB+2)) = Klc((pos_BG+3),(pos_CB+2))-H3*K2y;
    Klc((pos_BG+3),(pos_CB+3)) = Klc((pos_BG+3),(pos_CB+3))+H2*H3*K2y;
    Klc((pos_BG+3),(pos_CB+5)) = Klc((pos_BG+3),(pos_CB+5))-((-1)^i1g4)*Ll2*H3*K2y;
    Klc((pos_BG+3),(pos_BG+2)) = Klc((pos_BG+3),(pos_BG+2))+H3*K2y;
    Klc((pos_BG+3),(pos_BG+3)) = Klc((pos_BG+3),(pos_BG+3))+H3*H3*K2y;
    
    %%% pos_CB+4行：δβc——K2x
    Klc((pos_CB+4),(pos_CB+4))=Klc((pos_CB+4),(pos_CB+4))+H2*H2*K2x;
    Klc((pos_CB+4),(pos_CB+5))=Klc((pos_CB+4),(pos_CB+5))+((-1)^(i1+1+1))*Lb2*H2*K2x;
    Klc((pos_CB+4),(pos_BG+5))=Klc((pos_CB+4),(pos_BG+5))-((-1)^(i1+1+1))*Lb2*H2*K2x;
    Klc((pos_CB+4),(pos_BG+4))=Klc((pos_CB+4),(pos_BG+4))+H2*H3*K2x;
    %%% pos_CB+5行：δψc——K2x
    Klc((pos_CB+5),(pos_CB+4))=Klc((pos_CB+5),(pos_CB+4))+((-1)^(i1+1+1))*Lb2*H2*K2x;
    Klc((pos_CB+5),(pos_CB+5))=Klc((pos_CB+5),(pos_CB+5))+Lb2*Lb2*K2x;
    Klc((pos_CB+5),(pos_BG+5))=Klc((pos_CB+5),(pos_BG+5))-Lb2*Lb2*K2x;
    Klc((pos_CB+5),(pos_BG+4))=Klc((pos_CB+5),(pos_BG+4))+((-1)^(i1+1+1))*Lb2*H3*K2x;
    %%% pos_BG+5行：δψti(i=1,2)——K2x
    Klc((pos_BG+5),(pos_CB+4))=Klc((pos_BG+5),(pos_CB+4))-((-1)^(i1+1+1))*Lb2*H2*K2x;
    Klc((pos_BG+5),(pos_CB+5))=Klc((pos_BG+5),(pos_CB+5))-Lb2*Lb2*K2x;
    Klc((pos_BG+5),(pos_BG+5))=Klc((pos_BG+5),(pos_BG+5))+Lb2*Lb2*K2x;
    Klc((pos_BG+5),(pos_BG+4))=Klc((pos_BG+5),(pos_BG+4))-((-1)^(i1+1+1))*Lb2*H3*K2x;
    %%% pos_BG+4行：δβti(i=1,2)——K2x
    Klc((pos_BG+4),(pos_CB+4))=Klc((pos_BG+4),(pos_CB+4))+H2*H3*K2x;
    Klc((pos_BG+4),(pos_CB+5))=Klc((pos_BG+4),(pos_CB+5))+((-1)^(i1+1+1))*Lb2*H3*K2x;
    Klc((pos_BG+4),(pos_BG+5))=Klc((pos_BG+4),(pos_BG+5))-((-1)^(i1+1+1))*Lb2*H3*K2x;
    Klc((pos_BG+4),(pos_BG+4))=Klc((pos_BG+4),(pos_BG+4))+H3*H3*K2x;
    
    i1=i1+1;
end

%%% ===================== 列车阻尼（位移变分为行，位移为列）
Clc = zeros(NM_FW*Nw+35,NM_FW*Nw+35);
%%% 一系悬挂 C1z、C1y、C1x
i1=0;   % i1 = 0:1:7;
while i1<8
    i1g1=fix((i1+1+3)/4);       %%% 构架序号 i=1,2
    i1g2=fix((i1+1+1)/2);       %%% 轮对序号 m=1,2,3,4
    i1g3=fix((3*(i1+1)+1)/2);   %%% 车轮位置序号 ...............j+m=2,3,5,6,8,9,11,12 Left/Right
    i1g4=fix((i1+1-1)/2);       %%% m±1=[0,0,1,1,2,2,3,3;]
    
    if mod(i1+1,2)==0           %%% 车轮编号 j = i1+1 位于右侧
        DOF_pos_AB = Par_FW.DOF_pos.AxleBox_R;
    else                                     %%% 车轮编号 j = i1+1 位于左侧
        DOF_pos_AB = Par_FW.DOF_pos.AxleBox_L;
    end
        
    pos_BG = NM_FW*Nw+5*(i1g1-1)+20;
    pos_WS = NM_FW*Nw+5*(i1g2-1);
    
    %%% pos_BG+4行：δβti (i = 1,2)——C1x
    Clc(pos_BG+4,pos_BG+4) = Clc(pos_BG+4,pos_BG+4) + H4*H4*C1x;
    Clc(pos_BG+4,pos_BG+5) = Clc(pos_BG+4,pos_BG+5) + (-1)^(i1+1+1)*Lb1*H4*C1x;
    Clc(pos_BG+4,pos_WS+5)  = Clc(pos_BG+4,pos_WS+5)  - (-1)^(i1+1+1)*Lb1*H4*C1x;
    for k = 1:1:NM_FW
        Clc(pos_BG+4,(i1g2-1)*NM_FW+k) = Clc(pos_BG+4,(i1g2-1)*NM_FW+k) ...
                                                                          -ModeShape.FW(DOF_pos_AB(1),k)*H4*C1x;
    end
    %%% pos_BG+5行：δψti (i = 1,2)——C1x
    Clc(pos_BG+5,pos_BG+4) = Clc(pos_BG+5,pos_BG+4) + (-1)^(i1+1+1)*Lb1*H4*C1x;
    Clc(pos_BG+5,pos_BG+5) = Clc(pos_BG+5,pos_BG+5) + Lb1*Lb1*C1x;
    Clc(pos_BG+5,pos_WS+5)  = Clc(pos_BG+5,pos_WS+5)  - Lb1*Lb1*C1x;
    for k = 1:1:NM_FW
        Clc(pos_BG+5,(i1g2-1)*NM_FW+k) = Clc(pos_BG+5,(i1g2-1)*NM_FW+k) ...
                                                                          -ModeShape.FW(DOF_pos_AB(1),k)*(-1)^(i1+1+1)*Lb1*C1x;
    end
    %%% pos_WS+5行：δψwm (m = 1,2,3,4)——C1x
    Clc(pos_WS+5,pos_BG+4) = Clc(pos_WS+5,pos_BG+4) - (-1)^(i1+1+1)*Lb1*H4*C1x;
    Clc(pos_WS+5,pos_BG+5) = Clc(pos_WS+5,pos_BG+5) - Lb1*Lb1*C1x;
    Clc(pos_WS+5,pos_WS+5)  = Clc(pos_WS+5,pos_WS+5)  + Lb1*Lb1*C1x;
    for k = 1:1:NM_FW
        Clc(pos_WS+5,(i1g2-1)*NM_FW+k) = Clc(pos_WS+5,(i1g2-1)*NM_FW+k) ...
                                                                         +ModeShape.FW(DOF_pos_AB(1),k)*(-1)^(i1+1+1)*Lb1*C1x;
    end
    %%% (i1g2-1)*NM+k行：δqk (k = 1,2,...,NM)——C1x
    for k = 1:1:NM_FW
        Clc((i1g2-1)*NM_FW+k,pos_BG+4) = Clc((i1g2-1)*NM_FW+k,pos_BG+4) ...
                                                                         -ModeShape.FW(DOF_pos_AB(1),k)*H4*C1x;
        Clc((i1g2-1)*NM_FW+k,pos_BG+5) = Clc((i1g2-1)*NM_FW+k,pos_BG+5) ...
                                                                         -ModeShape.FW(DOF_pos_AB(1),k)*(-1)^(i1+1+1)*Lb1*C1x;
        Clc((i1g2-1)*NM_FW+k,pos_WS+5) = Clc((i1g2-1)*NM_FW+k,pos_WS+5) ...
                                                                         +ModeShape.FW(DOF_pos_AB(1),k)*(-1)^(i1+1+1)*Lb1*C1x;
       for kk = 1:1:NM_FW
           Clc((i1g2-1)*NM_FW+k,(i1g2-1)*NM_FW+kk) = Clc((i1g2-1)*NM_FW+k,(i1g2-1)*NM_FW+kk) ...
                                                                         +ModeShape.FW(DOF_pos_AB(1),k)*ModeShape.FW(DOF_pos_AB(1),kk)*C1x;
       end       
    end
    
    %%% pos_BG+2行：δyti(i = 1,2)——C1y
    Clc(pos_BG+2,pos_BG+2) = Clc(pos_BG+2,pos_BG+2) + C1y;
    Clc(pos_BG+2,pos_WS+2)  = Clc(pos_BG+2,pos_WS+2)  - C1y;
    Clc(pos_BG+2,pos_BG+3) = Clc(pos_BG+2,pos_BG+3) - H4*C1y;
    Clc(pos_BG+2,pos_BG+5) = Clc(pos_BG+2,pos_BG+5) + (-1)^i1g4*Ll1*C1y;
    for k = 1:1:NM_FW
        Clc(pos_BG+2,(i1g2-1)*NM_FW+k) = Clc(pos_BG+2,(i1g2-1)*NM_FW+k) ...
                                                                          -ModeShape.FW(DOF_pos_AB(2),k)*C1y;
    end
    %%% pos_WS+2行：δywm(m = 1,2,3,4)——C1y
    Clc(pos_WS+2,pos_BG+2) = Clc(pos_WS+2,pos_BG+2) - C1y;
    Clc(pos_WS+2,pos_WS+2)  = Clc(pos_WS+2,pos_WS+2)  + C1y;
    Clc(pos_WS+2,pos_BG+3) = Clc(pos_WS+2,pos_BG+3) + H4*C1y;
    Clc(pos_WS+2,pos_BG+5) = Clc(pos_WS+2,pos_BG+5) - (-1)^i1g4*Ll1*C1y;
    for k = 1:1:NM_FW
        Clc(pos_WS+2,(i1g2-1)*NM_FW+k) = Clc(pos_WS+2,(i1g2-1)*NM_FW+k) ...
                                                                          +ModeShape.FW(DOF_pos_AB(2),k)*C1y;
    end
    %%% pos_BG+3行：δφti(i = 1,2)——C1y
    Clc(pos_BG+3,pos_BG+2) = Clc(pos_BG+3,pos_BG+2) - H4*C1y;
    Clc(pos_BG+3,pos_WS+2)  = Clc(pos_BG+3,pos_WS+2)  + H4*C1y;
    Clc(pos_BG+3,pos_BG+3) = Clc(pos_BG+3,pos_BG+3) + H4*H4*C1y;
    Clc(pos_BG+3,pos_BG+5) = Clc(pos_BG+3,pos_BG+5) - H4*(-1)^i1g4*Ll1*C1y;
    for k = 1:1:NM_FW
        Clc(pos_BG+3,(i1g2-1)*NM_FW+k) = Clc(pos_BG+3,(i1g2-1)*NM_FW+k) ...
                                                                          +ModeShape.FW(DOF_pos_AB(2),k)*H4*C1y;
    end
    %%% pos_BG+5行：δψti(i = 1,2)——C1y
    Clc(pos_BG+5,pos_BG+2) = Clc(pos_BG+5,pos_BG+2) + (-1)^i1g4*Ll1*C1y;
    Clc(pos_BG+5,pos_WS+2)  = Clc(pos_BG+5,pos_WS+2)  - (-1)^i1g4*Ll1*C1y;
    Clc(pos_BG+5,pos_BG+3) = Clc(pos_BG+5,pos_BG+3) - (-1)^i1g4*Ll1*H4*C1y;
    Clc(pos_BG+5,pos_BG+5) = Clc(pos_BG+5,pos_BG+5) + Ll1*Ll1*C1y;
    for k = 1:1:NM_FW
        Clc(pos_BG+5,(i1g2-1)*NM_FW+k) = Clc(pos_BG+5,(i1g2-1)*NM_FW+k) ...
                                                                          -ModeShape.FW(DOF_pos_AB(2),k)*(-1)^i1g4*Ll1*C1y;
    end
    %%% (i1g2-1)*NM+k行：δqk (k = 1,2,...,NM)——C1y
    for k = 1:1:NM_FW
        Clc((i1g2-1)*NM_FW+k,pos_BG+2) = Clc((i1g2-1)*NM_FW+k,pos_BG+2) ...
                                                                          -ModeShape.FW(DOF_pos_AB(2),k)*C1y;
        Clc((i1g2-1)*NM_FW+k,pos_WS+2)  = Clc((i1g2-1)*NM_FW+k,pos_WS+2) ...
                                                                          +ModeShape.FW(DOF_pos_AB(2),k)*C1y;                                                                     
        Clc((i1g2-1)*NM_FW+k,pos_BG+3) = Clc((i1g2-1)*NM_FW+k,pos_BG+3) ...
                                                                          +ModeShape.FW(DOF_pos_AB(2),k)*H4*C1y;
        Clc((i1g2-1)*NM_FW+k,pos_BG+5) = Clc((i1g2-1)*NM_FW+k,pos_BG+5) ...
                                                                          -ModeShape.FW(DOF_pos_AB(2),k)*(-1)^i1g4*Ll1*C1y;
        for kk = 1:1:NM_FW
            Clc((i1g2-1)*NM_FW+k,(i1g2-1)*NM_FW+kk) = Clc((i1g2-1)*NM_FW+k,(i1g2-1)*NM_FW+kk) ...
                                                                          +ModeShape.FW(DOF_pos_AB(2),k)*ModeShape.FW(DOF_pos_AB(2),kk)*C1y;
        end
    end
    
    %%% pos_BG+1行：δzti(i = 1,2)——C1z
    Clc(pos_BG+1,pos_BG+1) = Clc(pos_BG+1,pos_BG+1) + C1z;
    Clc(pos_BG+1,pos_WS+1)  = Clc(pos_BG+1,pos_WS+1)  - C1z;
    Clc(pos_BG+1,pos_BG+3) = Clc(pos_BG+1,pos_BG+3) + (-1)^(i1+1)*Lb1*C1z;
    Clc(pos_BG+1,pos_WS+3)  = Clc(pos_BG+1,pos_WS+3)  - (-1)^(i1+1)*Lb1*C1z;
    Clc(pos_BG+1,pos_BG+4) = Clc(pos_BG+1,pos_BG+4) + (-1)^i1g2*Ll1*C1z;
    for k = 1:1:NM_FW
        Clc(pos_BG+1,(i1g2-1)*NM_FW+k) = Clc(pos_BG+1,(i1g2-1)*NM_FW+k) ...
                                                                          -ModeShape.FW(DOF_pos_AB(3),k)*C1z;
    end
    %%% pos_WS+1行：δzwm(m = 1,2,3,4)——C1z
    Clc(pos_WS+1,pos_BG+1) = Clc(pos_WS+1,pos_BG+1) - C1z;
    Clc(pos_WS+1,pos_WS+1)  = Clc(pos_WS+1,pos_WS+1)  + C1z;
    Clc(pos_WS+1,pos_BG+3) = Clc(pos_WS+1,pos_BG+3) - (-1)^(i1+1)*Lb1*C1z;
    Clc(pos_WS+1,pos_WS+3)  = Clc(pos_WS+1,pos_WS+3)  + (-1)^(i1+1)*Lb1*C1z;
    Clc(pos_WS+1,pos_BG+4) = Clc(pos_WS+1,pos_BG+4) - (-1)^i1g2*Ll1*C1z;
    for k = 1:1:NM_FW
        Clc(pos_WS+1,(i1g2-1)*NM_FW+k) = Clc(pos_WS+1,(i1g2-1)*NM_FW+k) ...
                                                                         +ModeShape.FW(DOF_pos_AB(3),k)*C1z;
    end
    %%% pos_BG+3行：δφti(i = 1,2)——C1z
    Clc(pos_BG+3,pos_BG+1) = Clc(pos_BG+3,pos_BG+1) + (-1)^(i1+1)*Lb1*C1z;
    Clc(pos_BG+3,pos_WS+1)  = Clc(pos_BG+3,pos_WS+1)  - (-1)^(i1+1)*Lb1*C1z;
    Clc(pos_BG+3,pos_BG+3) = Clc(pos_BG+3,pos_BG+3) + Lb1*Lb1*C1z;
    Clc(pos_BG+3,pos_WS+3)  = Clc(pos_BG+3,pos_WS+3)  - Lb1*Lb1*C1z;
    Clc(pos_BG+3,pos_BG+4) = Clc(pos_BG+3,pos_BG+4) + (-1)^i1g3*Lb1*Ll1*C1z;
    for k = 1:1:NM_FW
        Clc(pos_BG+3,(i1g2-1)*NM_FW+k) = Clc(pos_BG+3,(i1g2-1)*NM_FW+k) ...
                                                                          -ModeShape.FW(DOF_pos_AB(3),k)*(-1)^(i1+1)*Lb1*C1z;
    end
    %%% pos_WS+3行：δφwm(m = 1,2,3,4)——C1z
    Clc(pos_WS+3,pos_BG+1) = Clc(pos_WS+3,pos_BG+1) - (-1)^(i1+1)*Lb1*C1z;
    Clc(pos_WS+3,pos_WS+1)  = Clc(pos_WS+3,pos_WS+1)  + (-1)^(i1+1)*Lb1*C1z;
    Clc(pos_WS+3,pos_BG+3) = Clc(pos_WS+3,pos_BG+3) - Lb1*Lb1*C1z;
    Clc(pos_WS+3,pos_WS+3)  = Clc(pos_WS+3,pos_WS+3)  + Lb1*Lb1*C1z;
    Clc(pos_WS+3,pos_BG+4) = Clc(pos_WS+3,pos_BG+4) - (-1)^i1g3*Lb1*Ll1*C1z;
    for k = 1:1:NM_FW
        Clc(pos_WS+3,(i1g2-1)*NM_FW+k) = Clc(pos_WS+3,(i1g2-1)*NM_FW+k) ...
                                                                         +ModeShape.FW(DOF_pos_AB(3),k)*(-1)^(i1+1)*Lb1*C1z;
    end
    %%% pos_BG+4行：δβti(i = 1,2)——C1z
    Clc(pos_BG+4,pos_BG+1) = Clc(pos_BG+4,pos_BG+1) + (-1)^i1g2*Ll1*C1z;
    Clc(pos_BG+4,pos_WS+1)  = Clc(pos_BG+4,pos_WS+1)  - (-1)^i1g2*Ll1*C1z;
    Clc(pos_BG+4,pos_BG+3) = Clc(pos_BG+4,pos_BG+3) + (-1)^i1g3*Lb1*Ll1*C1z;
    Clc(pos_BG+4,pos_WS+3)  = Clc(pos_BG+4,pos_WS+3)  - (-1)^i1g3*Lb1*Ll1*C1z;
    Clc(pos_BG+4,pos_BG+4) = Clc(pos_BG+4,pos_BG+4) + Ll1*Ll1*C1z;
    for k = 1:1:NM_FW
        Clc(pos_BG+4,(i1g2-1)*NM_FW+k) = Clc(pos_BG+4,(i1g2-1)*NM_FW+k) ...
                                                                          -ModeShape.FW(DOF_pos_AB(3),k)*(-1)^i1g2*Ll1*C1z;
    end
    %%% (i1g2-1)*NM+k行：δqk (k = 1,2,...,NM)——C1z
    for k = 1:1:NM_FW
        Clc((i1g2-1)*NM_FW+k,pos_BG+1) = Clc((i1g2-1)*NM_FW+k,pos_BG+1) ...
                                                                         -ModeShape.FW(DOF_pos_AB(3),k)*C1z;
        Clc((i1g2-1)*NM_FW+k,pos_WS+1) = Clc((i1g2-1)*NM_FW+k,pos_WS+1) ...
                                                                         +ModeShape.FW(DOF_pos_AB(3),k)*C1z;
        Clc((i1g2-1)*NM_FW+k,pos_BG+3) = Clc((i1g2-1)*NM_FW+k,pos_BG+3) ...
                                                                         -ModeShape.FW(DOF_pos_AB(3),k)*(-1)^(i1+1)*Lb1*C1z;
        Clc((i1g2-1)*NM_FW+k,pos_WS+3) = Clc((i1g2-1)*NM_FW+k,pos_WS+3) ...
                                                                         +ModeShape.FW(DOF_pos_AB(3),k)*(-1)^(i1+1)*Lb1*C1z;
        Clc((i1g2-1)*NM_FW+k,pos_BG+4) = Clc((i1g2-1)*NM_FW+k,pos_BG+4) ...
                                                                         -ModeShape.FW(DOF_pos_AB(3),k)*(-1)^i1g2*Ll1*C1z;
        for kk = 1:1:NM_FW
            Clc((i1g2-1)*NM_FW+k,(i1g2-1)*NM_FW+kk) = Clc((i1g2-1)*NM_FW+k,(i1g2-1)*NM_FW+kk) ...
                                                                          +ModeShape.FW(DOF_pos_AB(3),k)*ModeShape.FW(DOF_pos_AB(3),kk)*C1z;
        end
    end    
    
    i1=i1+1;
end

%%% 广义阻尼矩阵
for i1 = 1:1:Nw
    for k = 1:1:size(ModeFreq.FW,1)
        pos = (i1-1)*NM_FW+k;
        Clc(pos,pos) = Clc(pos,pos) + 2*(2*pi*ModeFreq.FW(k,2))*DampingRatio;
    end
end

%%% 二系悬挂 C2z、C2y、C2x
i1=0;   % i1 = 0:1:3;
while i1<4
    i1g2=fix((i1+1+1)/2);          %%% 构架序号 i=[1,1,2,2]
    i1g3=fix((3*(i1+1)+1)/2);      %%% 车轮位置序号 j+i=2,3,5,6 Left/Right
    i1g4=fix((i1+1-1)/2);          %%% i±1=[0,0,1,1]（其实就是 i+1=[2,2,3,3]）
        
    pos_CB = NM_FW*Nw+30;
    pos_BG = NM_FW*Nw+5*(i1g2-1)+20;
    
    %%% NM*4+10+1列：zc——C2z
    Clc(pos_CB+1,pos_CB+1) = Clc(pos_CB+1,pos_CB+1)+C2z;
    Clc(pos_CB+3,pos_CB+1) = Clc(pos_CB+3,pos_CB+1)+((-1)^(i1+1))*Lb2*C2z;
    Clc(pos_CB+4,pos_CB+1) = Clc(pos_CB+4,pos_CB+1)+((-1)^i1g2)*Ll2*C2z;
    Clc(pos_BG+1,pos_CB+1) = Clc(pos_BG+1,pos_CB+1)-C2z;
    Clc(pos_BG+3,pos_CB+1) = Clc(pos_BG+3,pos_CB+1)-((-1)^(i1+1))*Lb2*C2z;
    %%% pos_CB+3列：φc——C2z
    Clc((pos_CB+1),(pos_CB+3)) = Clc((pos_CB+1),(pos_CB+3))+((-1)^(i1+1))*Lb2*C2z;
    Clc((pos_CB+3),(pos_CB+3)) = Clc((pos_CB+3),(pos_CB+3))+Lb2*Lb2*C2z;
    Clc((pos_CB+4),(pos_CB+3)) = Clc((pos_CB+4),(pos_CB+3))+((-1)^i1g3)*Lb2*Ll2*C2z;
    Clc((pos_BG+1),(pos_CB+3)) = Clc((pos_BG+1),(pos_CB+3))-((-1)^(i1+1))*Lb2*C2z;
    Clc((pos_BG+3),(pos_CB+3)) = Clc((pos_BG+3),(pos_CB+3))-Lb2*Lb2*C2z;
    %%% pos_CB+4列：βc——C2z
    Clc((pos_CB+1),(pos_CB+4)) = Clc((pos_CB+1),(pos_CB+4))+((-1)^i1g2)*Ll2*C2z;
    Clc((pos_CB+3),(pos_CB+4)) = Clc((pos_CB+3),(pos_CB+4))+((-1)^i1g3)*Lb2*Ll2*C2z;
    Clc((pos_CB+4),(pos_CB+4)) = Clc((pos_CB+4),(pos_CB+4))+Ll2*Ll2*C2z;
    Clc((pos_BG+1),(pos_CB+4)) = Clc((pos_BG+1),(pos_CB+4))-((-1)^i1g2)*Ll2*C2z;
    Clc((pos_BG+3),(pos_CB+4)) = Clc((pos_BG+3),(pos_CB+4))-((-1)^i1g3)*Lb2*Ll2*C2z;
    %%% pos_BG+1列：zti(i = 1,2)——C2z      
    Clc((pos_CB+1),(pos_BG+1)) = Clc((pos_CB+1),(pos_BG+1))-C2z;
    Clc((pos_CB+3),(pos_BG+1)) = Clc((pos_CB+3),(pos_BG+1))-((-1)^(i1+1))*Lb2*C2z;
    Clc((pos_CB+4),(pos_BG+1)) = Clc((pos_CB+4),(pos_BG+1))-((-1)^i1g2)*Ll2*C2z;
    Clc((pos_BG+1),(pos_BG+1)) = Clc((pos_BG+1),(pos_BG+1))+C2z;
    Clc((pos_BG+3),(pos_BG+1)) = Clc((pos_BG+3),(pos_BG+1))+((-1)^(i1+1))*Lb2*C2z;
    %%% pos_BG+3列：φti(i = 1,2)——C2z
    Clc((pos_CB+1),(pos_BG+3)) = Clc((pos_CB+1),(pos_BG+3))-((-1)^(i1+1))*Lb2*C2z;
    Clc((pos_CB+3),(pos_BG+3)) = Clc((pos_CB+3),(pos_BG+3))-Lb2*Lb2*C2z;
    Clc((pos_CB+4),(pos_BG+3)) = Clc((pos_CB+4),(pos_BG+3))-((-1)^i1g3)*Lb2*Ll2*C2z;
    Clc((pos_BG+1),(pos_BG+3)) = Clc((pos_BG+1),(pos_BG+3))+((-1)^(i1+1))*Lb2*C2z;
    Clc((pos_BG+3),(pos_BG+3)) = Clc((pos_BG+3),(pos_BG+3))+Lb2*Lb2*C2z;
    
    %%% pos_CB+2行：δyc——C2y
    Clc((pos_CB+2),(pos_CB+2)) = Clc((pos_CB+2),(pos_CB+2))+C2y;
    Clc((pos_CB+2),(pos_CB+3)) = Clc((pos_CB+2),(pos_CB+3))-H2*C2y;
    Clc((pos_CB+2),(pos_CB+5)) = Clc((pos_CB+2),(pos_CB+5))+((-1)^i1g4)*Ll2*C2y;
    Clc((pos_CB+2),(pos_BG+2)) = Clc((pos_CB+2),(pos_BG+2))-C2y;
    Clc((pos_CB+2),(pos_BG+3)) = Clc((pos_CB+2),(pos_BG+3))-H3*C2y;
    %%% pos_CB+3行：δφc——C2y
    Clc((pos_CB+3),(pos_CB+2)) = Clc((pos_CB+3),(pos_CB+2))-H2*C2y;
    Clc((pos_CB+3),(pos_CB+3)) = Clc((pos_CB+3),(pos_CB+3))+H2*H2*C2y;
    Clc((pos_CB+3),(pos_CB+5)) = Clc((pos_CB+3),(pos_CB+5))-((-1)^i1g4)*Ll2*H2*C2y;
    Clc((pos_CB+3),(pos_BG+2)) = Clc((pos_CB+3),(pos_BG+2))+H2*C2y;
    Clc((pos_CB+3),(pos_BG+3)) = Clc((pos_CB+3),(pos_BG+3))+H2*H3*C2y;
    %%% pos_CB+5行：δψc——C2y
    Clc((pos_CB+5),(pos_CB+2)) = Clc((pos_CB+5),(pos_CB+2))+((-1)^i1g4)*Ll2*C2y;
    Clc((pos_CB+5),(pos_CB+3)) = Clc((pos_CB+5),(pos_CB+3))-((-1)^i1g4)*Ll2*H2*C2y;
    Clc((pos_CB+5),(pos_CB+5)) = Clc((pos_CB+5),(pos_CB+5))+Ll2*Ll2*C2y;
    Clc((pos_CB+5),(pos_BG+2)) = Clc((pos_CB+5),(pos_BG+2))-((-1)^i1g4)*Ll2*C2y;
    Clc((pos_CB+5),(pos_BG+3)) = Clc((pos_CB+5),(pos_BG+3))-((-1)^i1g4)*Ll2*H3*C2y;
    %%% pos_BG+2行：δyti(i = 1,2)——C2y
    Clc((pos_BG+2),(pos_CB+2)) = Clc((pos_BG+2),(pos_CB+2))-C2y;
    Clc((pos_BG+2),(pos_CB+3)) = Clc((pos_BG+2),(pos_CB+3))+H2*C2y;
    Clc((pos_BG+2),(pos_CB+5)) = Clc((pos_BG+2),(pos_CB+5))-((-1)^i1g4)*Ll2*C2y;
    Clc((pos_BG+2),(pos_BG+2)) = Clc((pos_BG+2),(pos_BG+2))+C2y;
    Clc((pos_BG+2),(pos_BG+3)) = Clc((pos_BG+2),(pos_BG+3))+H3*C2y;       
    %%% pos_BG+3行：δφti(i = 1,2)——C2y
    Clc((pos_BG+3),(pos_CB+2)) = Clc((pos_BG+3),(pos_CB+2))-H3*C2y;
    Clc((pos_BG+3),(pos_CB+3)) = Clc((pos_BG+3),(pos_CB+3))+H2*H3*C2y;
    Clc((pos_BG+3),(pos_CB+5)) = Clc((pos_BG+3),(pos_CB+5))-((-1)^i1g4)*Ll2*H3*C2y;
    Clc((pos_BG+3),(pos_BG+2)) = Clc((pos_BG+3),(pos_BG+2))+H3*C2y;
    Clc((pos_BG+3),(pos_BG+3)) = Clc((pos_BG+3),(pos_BG+3))+H3*H3*C2y;
    
    %%% pos_CB+4行：δβc——C2x
    Clc((pos_CB+4),(pos_CB+4))=Clc((pos_CB+4),(pos_CB+4))+H2*H2*C2x;
    Clc((pos_CB+4),(pos_CB+5))=Clc((pos_CB+4),(pos_CB+5))+((-1)^(i1+1+1))*Lb2*H2*C2x;
    Clc((pos_CB+4),(pos_BG+5))=Clc((pos_CB+4),(pos_BG+5))-((-1)^(i1+1+1))*Lb2*H2*C2x;
    Clc((pos_CB+4),(pos_BG+4))=Clc((pos_CB+4),(pos_BG+4))+H2*H3*C2x;
    %%% pos_CB+5行：δψc——C2x
    Clc((pos_CB+5),(pos_CB+4))=Clc((pos_CB+5),(pos_CB+4))+((-1)^(i1+1+1))*Lb2*H2*C2x;
    Clc((pos_CB+5),(pos_CB+5))=Clc((pos_CB+5),(pos_CB+5))+Lb2*Lb2*C2x;
    Clc((pos_CB+5),(pos_BG+5))=Clc((pos_CB+5),(pos_BG+5))-Lb2*Lb2*C2x;
    Clc((pos_CB+5),(pos_BG+4))=Clc((pos_CB+5),(pos_BG+4))+((-1)^(i1+1+1))*Lb2*H3*C2x;
    %%% pos_BG+5行：δψti(i=1,2)——C2x
    Clc((pos_BG+5),(pos_CB+4))=Clc((pos_BG+5),(pos_CB+4))-((-1)^(i1+1+1))*Lb2*H2*C2x;
    Clc((pos_BG+5),(pos_CB+5))=Clc((pos_BG+5),(pos_CB+5))-Lb2*Lb2*C2x;
    Clc((pos_BG+5),(pos_BG+5))=Clc((pos_BG+5),(pos_BG+5))+Lb2*Lb2*C2x;
    Clc((pos_BG+5),(pos_BG+4))=Clc((pos_BG+5),(pos_BG+4))-((-1)^(i1+1+1))*Lb2*H3*C2x;
    %%% pos_BG+4行：δβti(i=1,2)——C2x
    Clc((pos_BG+4),(pos_CB+4))=Clc((pos_BG+4),(pos_CB+4))+H2*H3*C2x;
    Clc((pos_BG+4),(pos_CB+5))=Clc((pos_BG+4),(pos_CB+5))+((-1)^(i1+1+1))*Lb2*H3*C2x;
    Clc((pos_BG+4),(pos_BG+5))=Clc((pos_BG+4),(pos_BG+5))-((-1)^(i1+1+1))*Lb2*H3*C2x;
    Clc((pos_BG+4),(pos_BG+4))=Clc((pos_BG+4),(pos_BG+4))+H3*H3*C2x;
    
    i1=i1+1;
end

% %%% 抗侧滚扭杆
% for i = 1:1:2
%     Klc((30+3),(30+3)) = Klc((30+3),(30+3)) + Kr;
%     Klc((30+3),(20+5*(i-1)+3)) = Klc((30+3),(20+5*(i-1)+3)) - Kr;
%     Klc((20+5*(i-1)+3),(30+3)) = Klc((20+5*(i-1)+3),(30+3)) - Kr;
%     Klc((20+5*(i-1)+3),(20+5*(i-1)+3)) = Klc((20+5*(i-1)+3),(20+5*(i-1)+3)) + Kr;
% end
% 
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