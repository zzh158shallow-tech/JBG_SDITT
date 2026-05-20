%% 构建多刚体车辆系统 M、C、K 矩阵
% 230403: 补充了一系垂向减振器（串联弹簧）
% 230404: 修正了一系垂向减振器（串联弹簧），补充了抗蛇形减振器（串联弹簧）

function Clc = Create_Cxt_NL(InpPar, Par_Vehicle, DamperNL, C_vehicle_v0)

% global Par_Vehicle

Lb_DPz = Par_Vehicle.Lb_DPz;
Ll_DPzt = Par_Vehicle.Ll_DPzt;

Lb_S = Par_Vehicle.Lb_S;
H_cS = Par_Vehicle.H_cS;

% Ll_DPyc = Par_Vehicle.Ll_DPyc;
H_cDPy = Par_Vehicle.H_cDPy;

%% 列车阻尼（位移变分为行，速度为列）
Clc = C_vehicle_v0;
DOF_FW = InpPar.Nw*InpPar.NM_FW;

%% NL-1: 一系垂向减振器 K_DPz, C_DPz
% C_DPz
for i1 = 1:1:2
    for i2 = 1:1:2
        for i3 = 1:1:2
            pos = 4*(i1-1)+2*(i2-1)+i3;
            pos_P = DOF_FW+35+pos;
            pos_B = DOF_FW+20+5*(i1-1);
            
            if ~DamperNL.DPz.Mark(pos,1)                
                C_DPz = Par_Vehicle.C_DPz(1,1);
            else
                C_DPz = Par_Vehicle.C_DPz(2,1);
            end
            
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
% C_Sx
for i1 = 1:1:2
    for i3 = 1:1:2
        pos = 2*(i1-1)+i3;
        pos_P = DOF_FW+35+8+pos;
        pos_C = DOF_FW+30;
            
        if ~DamperNL.Sx.Mark(pos,1)
            C_Sx = Par_Vehicle.C_Sx(1,1);
        else
            C_Sx = Par_Vehicle.C_Sx(2,1);
        end
        
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
% C_DPy
for i1 = 1:1:2
    for i2 = 1:1:2
        pos = 2*(i1-1)+i2;
        pos_P = DOF_FW+35+8+4+pos;
        pos_C = DOF_FW+30;      
            
        if ~DamperNL.DPy.Mark(pos,1)
            C_DPy = Par_Vehicle.C_DPy(1,1);
        else
            C_DPy = Par_Vehicle.C_DPy(2,1);
        end

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
 
