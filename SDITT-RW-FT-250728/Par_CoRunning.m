%% 连续支撑转换为离散支撑, Rail, Baseplate, Dis_Baseplate

function [Rail, Baseplate] = Par_CoRunning(Tar_SIP, Range_X)

% clc
% clear

% Tar_SIP = 1;
% Tar_SIP = 2;
% Range_X = [6.53, 54.309]';
SIP = [0, 53.148]+50;


Path_1 = 'E:\# Flex-Rigid-20200418\# Abaqus Turnout Model\TurnoutScript_All_230730_S8a';
addpath(Path_1)

% RailPro BaseplatePro ConsPar
load('RailPro.mat') 
load('BaseplatePro.mat')
load('ConsPar.mat')

Choose_Plot = 0;
if Choose_Plot==1
    figure(89); clf
    x = ZP_Dyn.Mileage(:,1)-SIP(1);
    x(1:3) = NaN;
    y = ZP_Dyn.FZ.FF.R1(:,2)+ZP_Dyn.FZ.FF.R2(:,2);
    plot(x, y/1000); grid on
end


%%% Rail
Er = 2.14e11;
Gr = Er/(2*(1+0.3));
Ls = 0.6;
Density = 7850;
% Elastic Cons.
Rail.kpz.zjbg = 275e6;
Rail.kpz.qjbg = 275e6;
Rail.kpz.zjg_zgyg = 275e6;
Rail.kpz.cxg = 275e6;

Rail.dpz.zjbg = 10e3;
Rail.dpz.qjbg = 10e3;
Rail.dpz.zjg_zgyg = 10e3;
Rail.dpz.cxg = 10e3;

if Tar_SIP==1
    Rail.kpy.zjbg = 15e6/2;
    Rail.kpy.qjbg = 15e6/2;
    Rail.kpy.zjg_zgyg = 15e6/2;            % zjg
    Rail.dpy.zjbg = 15e3/2;
    Rail.dpy.qjbg = 15e3/2;
    Rail.dpy.zjg_zgyg = 15e3/2;            % zjg
    Type_Rail = {'zjbg', 'zjg_zgyg', 'qjbg'};
    Type_Baseplate = {'Switch_L', 'Switch_R'};
elseif Tar_SIP==2
    Rail.kpy.zjbg = 15e6;
    Rail.kpy.zjg_zgyg = 15e6*0.8;     % zgyg
    Rail.kpy.cxg = 15e6*0.8;
    Rail.dpy.zjbg = 15e3;
    Rail.dpy.zjg_zgyg = 15e3*0.8;     % zgyg
    Rail.dpy.cxg = 15e3*0.8;
    Type_Rail = {'zjbg', 'zjg_zgyg', 'cxg'};
    Type_Baseplate = {'Crossing_L', 'Crossing_Center'};
end
TBaseplate_L = Type_Baseplate{1};
TBaseplate_R = Type_Baseplate{2};

% for i1 = 1:1:length(Range_X)
for i1 = Tar_SIP:1:Tar_SIP
    Mileage = Range_X(i1);
    Mileage_Baseplate = BaseplatePro.Mileage_Sleeper_Thr(:,2)+50-SIP(1);
    [~, p1] = min(abs(Mileage-Mileage_Baseplate));
    bools_L =  abs(Mileage_Baseplate(p1)-(cell2mat(ConsPar.PlatePad.(TBaseplate_L)(:,5))+50-SIP(1)))<1e-4 ;
    bools_R =  abs(Mileage_Baseplate(p1)-(cell2mat(ConsPar.PlatePad.(TBaseplate_R)(:,5))+50-SIP(1)))<1e-4 ;
    Baseplate.No(i1,1) = BaseplatePro.Mileage_Sleeper_Thr(p1,1);
    
    for i2 = 1:1:length(Type_Rail)
        T2 = Type_Rail{i2};
        if i2==1
            if Tar_SIP==1
                pos = p1;               % 转辙器区左侧垫板尺寸随计算的矩阵位置
            elseif Tar_SIP==2
                pos = 1;                  % 辙叉区左侧垫板尺寸以首位计算
            end
            temp_stiff = sum(cell2mat(ConsPar.PlatePad.(TBaseplate_L)(bools_L,4)));
        else
            pos = p1;
            temp_stiff = sum(cell2mat(ConsPar.PlatePad.(TBaseplate_R)(bools_R,4)));
        end
        % Baseplate
        Baseplate.Lb.(T2)(i1,1) = BaseplatePro .Baseplate_Size(pos,2);     % Lb
        Baseplate.Iry.(T2)(i1,1) = BaseplatePro.Baseplate_Size(pos,3);     % b
        Baseplate.Iry.(T2)(i1,2) = BaseplatePro.Baseplate_Size(pos,4);     % h
        Baseplate.Iry.(T2)(i1,3) = Baseplate.Iry.(T2)(i1,1)*Baseplate.Iry.(T2)(i1,2)^3/12;     % (b*h^3)/12
        Baseplate.Stiff_Sleeper.(T2)(i1,1) = temp_stiff(2);
        Baseplate.Sb.(T2)(i1,1) = Baseplate.Stiff_Sleeper.(T2)(i1,1) / Baseplate.Lb.(T2)(i1,1);
        Baseplate.Damp_Sleeper.(T2)(i1,1) = temp_stiff(3);
        Baseplate.Db.(T2)(i1,1) = Baseplate.Damp_Sleeper.(T2)(i1,1) / Baseplate.Lb.(T2)(i1,1);
        Baseplate.Leff.(T2)(i1,1) = 2*sqrt(2)*(Er*Baseplate.Iry.(T2)(i1,3))^(1/4)*(Baseplate.Sb.(T2)(i1,1))^(-1/4);         % Kayan Chan
%         Baseplate.Leff.(T2)(i1,1) = 2*sqrt(2)*(Er*Baseplate.Iry.(T2)(i1,3))^(1/4)*(Baseplate.Sb.(T2)(i1,1))^(-1/4) / Baseplate.Lb.(T2)(i1,1);        % JoyiShhi
        if Baseplate.Leff.(T2)(i1,1)>Baseplate.Lb.(T2)(i1,1)
            Baseplate.Leff.(T2)(i1,1)=Baseplate.Lb.(T2)(i1,1);
        end
        Baseplate.kbz.(T2)(i1,1) = Baseplate.Sb.(T2)(i1,1)*Baseplate.Leff.(T2)(i1,1);
        Baseplate.cbz.(T2)(i1,1) = Baseplate.Db.(T2)(i1,1)*Baseplate.Leff.(T2)(i1,1);

        if Mileage >= min(RailPro.(T2)(:,2)+50-SIP(1)) && Mileage <= max(RailPro.(T2)(:,2)+50-SIP(1))
            Rail.Ar.(T2)(i1,1) = interp1(RailPro.(T2)(:,2)+50-SIP(1), RailPro.(T2)(:,5), Mileage, 'linear');
            Rail.Iry.(T2)(i1,1) = interp1(RailPro.(T2)(:,2)+50-SIP(1), RailPro.(T2)(:,8), Mileage, 'linear');
            Rail.Irz.(T2)(i1,1) = interp1(RailPro.(T2)(:,2)+50-SIP(1), RailPro.(T2)(:,6), Mileage, 'linear');
            Rail.k_shear_z.(T2)(i1,1) = interp1(RailPro .(T2)(:,2)+50-SIP(1), RailPro.(T2)(:,10), Mileage, 'linear');
            Rail.k_shear_y.(T2)(i1,1) = interp1(RailPro.(T2)(:,2)+50-SIP(1), RailPro.(T2)(:,11), Mileage, 'linear');

            % Sz, Kst, Leff, krz_co
            % No baseplate
%             Rail.Sz.(T2)(i1,1) = Rail.kpz.(T2) / Ls;
            % With baseplate
            Rail.Sz.(T2)(i1,1) = Rail.kpz.(T2)*Baseplate.kbz.(T2)(i1,1) / Ls / (Rail.kpz.(T2)+Baseplate.kbz.(T2)(i1,1));
            Rail.Kstz.(T2)(i1,1) = 2*sqrt(2) * (Er*Rail.Iry.(T2)(i1,1))^(1/4) * Rail.Sz.(T2)(i1,1)^(3/4);
            Rail.Leff_z.(T2)(i1,1) = Rail.Kstz.(T2)(i1,1) / Rail.Sz.(T2)(i1,1);
            Rail.krz_co.(T2)(i1,1) = Rail.Leff_z.(T2)(i1,1) / Ls * Rail.kpz.(T2);
            Rail.crz_co.(T2)(i1,1) = Rail.Leff_z.(T2)(i1,1) / Ls * Rail.dpz.(T2);

            Rail.Sy.(T2)(i1,1) = Rail.kpy.(T2) / Ls;
            Rail.Ksty.(T2)(i1,1) = 2*sqrt(2) * (Er*Rail.Irz.(T2)(i1,1))^(1/4) * Rail.Sy.(T2)(i1,1)^(3/4);
            Rail.Leff_y.(T2)(i1,1) = Rail.Ksty.(T2)(i1,1) / Rail.Sy.(T2)(i1,1);
            Rail.kry_co.(T2)(i1,1) = Rail.Leff_y.(T2)(i1,1) / Ls * Rail.kpy.(T2);
            Rail.cry_co.(T2)(i1,1) = Rail.Leff_y.(T2)(i1,1) / Ls * Rail.dpy.(T2);

            Rail.Mass.(T2)(i1,1) = Density*Rail.Ar.(T2)(i1,1)*Rail.Leff_z.(T2)(i1,1);

            Rail.C0z.(T2)(i1,1) = 2*Rail.Ar.(T2)(i1,1)*Rail.k_shear_z.(T2)(i1,1) * sqrt(0.4*Gr*Density);
            Rail.C0y.(T2)(i1,1) = 2*Rail.Ar.(T2)(i1,1)*Rail.k_shear_y.(T2)(i1,1) * sqrt(0.4*Gr*Density);

        else
            Rail.Ar.(T2)(i1,1) = NaN;
            Rail.Iry.(T2)(i1,1) = NaN;
            Rail.Sz.(T2)(i1,1) = NaN;
            Rail.Kst.(T2)(i1,1) = NaN;
            Rail.Leff_z.(T2)(i1,1) = NaN;
            Rail.krz_co.(T2)(i1,1) = NaN;
            Rail.crz_co.(T2)(i1,1) = NaN;
            Rail.kry_co.(T2)(i1,1) = NaN;
            Rail.cry_co.(T2)(i1,1) = NaN;
            Rail.Mass.(T2)(i1,1) = NaN;
        end
    end
        
    for i2 = 1:1:length(Type_Baseplate)
        T2 = Type_Rail{i2};
        if i2==1
            Baseplate.kbz_co.(Type_Baseplate{i2})(i1,1) = Baseplate.kbz.(T2)(i1,1) * Rail.Leff_z.(T2)(i1,1) / Ls;
            Baseplate.cbz_co.(Type_Baseplate{i2})(i1,1) = Baseplate.cbz.(T2)(i1,1) * Rail.Leff_z.(T2)(i1,1) / Ls;
            Baseplate.Mass.(Type_Baseplate{i2})(i1,1) = Density*Baseplate.Iry.(T2)(i1,1)*Baseplate.Iry.(T2)(i1,2)*Baseplate.Leff.(T2)(i1,1) * Rail.Leff_z.(T2)(i1,1) / Ls;
%             Dis_Baseplate(i1,i2) = Pst.L(i1,1)/Baseplate.kbz_co.(T2)(i1,1);
        else
            if ~isnan(Rail.Leff_z.(Type_Rail{3})(i1,1))
                Coff = (Rail.Leff_z.(Type_Rail{2})(i1,1)+Rail.Leff_z.(Type_Rail{3})(i1,1)) / 2;
            else
                Coff = Rail.Leff_z.(Type_Rail{2})(i1,1);
            end
            Baseplate.kbz_co.(Type_Baseplate{i2})(i1,1) = Baseplate.kbz.(T2)(i1,1) * Coff / Ls;
            Baseplate.cbz_co.(Type_Baseplate{i2})(i1,1) = Baseplate.cbz.(T2)(i1,1) * Coff / Ls;
            Baseplate.Mass.(Type_Baseplate{i2})(i1,1) = Density*Baseplate.Iry.(T2)(i1,1)*Baseplate.Iry.(T2)(i1,2)*Baseplate.Leff.(T2)(i1,1) * Coff / Ls;
%             Dis_Baseplate(i1,i2) = Pst.R(i1,1)/Baseplate.kbz_co.(T2)(i1,1);
        end
    end
    if Tar_SIP==1
        % 转辙器两侧垫板参数均相同，左侧也需要考虑曲尖轨的作用
        Baseplate.kbz_co.(Type_Baseplate{1})(i1,1) = Baseplate.kbz_co.(Type_Baseplate{2})(i1,1);
        Baseplate.cbz_co.(Type_Baseplate{1})(i1,1) = Baseplate.cbz_co.(Type_Baseplate{2})(i1,1);
        Baseplate.Mass.(Type_Baseplate{1})(i1,1) = Baseplate.Mass.(Type_Baseplate{2})(i1,1);
    end
    
end


%% Baseplate, Rail
if Choose_Plot==1
    % Baseplate Plot
    figure(6); clf
    for i2 = 1:1:3
        T2 = Type_Rail{i2};
        if i2==3
            LineType = '--';
        else
            LineType = '-';
        end
        subplot(2,1,1)
%         plot(Range_X, Baseplate.Lb.(T2), LineType); hold on
%         plot(Range_X, Baseplate.Iry.(T2)(:,3), LineType); hold on
%         plot(Range_X, Baseplate.Stiff_Sleeper.(T2), LineType); hold on
%         plot(Range_X, Baseplate.No, LineType); hold on
%         plot(Baseplate.No, Baseplate.Stiff_Sleeper.(T2), LineType); hold on
%         plot(Range_X, Baseplate.Leff.(T2), LineType); hold on
%         plot(Range_X, Baseplate.kbz.(T2)/1e6, LineType); hold on
%         xlabel('Mileage (m)'); ylabel('Kbz (kN/mm)'); grid on
        plot(Range_X, Dis_Baseplate(:,i2), LineType); hold on
        xlabel('Mileage (m)'); ylabel('Dis-Baseplate (m)'); grid on
        set(gca,'FontName', 'Times', 'FontSize', FontSize);
        grid on
        subplot(2,1,2)
        plot(Range_X, Baseplate.kbz_co.(T2)./Baseplate.kbz.(T2), LineType); hold on
%         plot(Range_X, Baseplate.kbz.(T2)/1e6, LineType); hold on
%         plot(Range_X, Baseplate.Leff.(T2)/Baseplate.Lb.(T2), LineType); hold on
        xlabel('Mileage (m)'); ylabel('Kbz-Co-Coff. (-)'); grid on
        set(gca,'FontName', 'Times', 'FontSize', FontSize);
        grid on
    end

    % Rail Plot
    figure(7); clf
    for i2 = 1:1:3
        T2 = Type_Rail{i2};
        if i2==3
            LineType = '--';
        else
            LineType = '-';
        end
        subplot(2,1,1)
        plot(Range_X, Rail.Iry.(T2), LineType); hold on
        xlabel('Mileage (m)'); ylabel('Iry (m^4)'); grid on
        set(gca,'FontName', 'Times', 'FontSize', FontSize);
        grid on
        subplot(2,1,2)
        plot(Range_X, Rail.Leff.(T2)/Ls, LineType); hold on
        % plot(Range_X, Rail.krz_co.(T2), LineType); hold on
        xlabel('Mileage (m)'); ylabel('Kpz-Co-Coff. (-)'); grid on
        set(gca,'FontName', 'Times', 'FontSize', FontSize);
    end
end

