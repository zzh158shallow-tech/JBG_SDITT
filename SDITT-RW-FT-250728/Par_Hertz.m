%% 根据 Hertz 理论，判断单位法向力下接触点处的接触参数

function [a1, b1, m, n, elastic_permeability_Unit, Con_A, Con_B, Con_r] = Par_Hertz(R_yy_w, R_xx_w, R_xx_r, rou, BGmn, Vr, Er, Type_Normal)

beta = acos(rou./4.*abs(1./R_yy_w-1./R_xx_w-1./R_xx_r));
m = interp1(BGmn(:,1),BGmn(:,2),beta,'linear');
n = interp1(BGmn(:,1),BGmn(:,3),beta,'linear');
a1 = zeros(length(rou),1);
b1 = zeros(length(rou),1);
for i = 1:1:length(rou)
   if rou(i)/R_yy_w(i) <= 2
       a1(i) = 0.1506e-3 * m(i) * (rou(i)*1)^(1/3);
       b1(i) = 0.1506e-3 * n(i) * (rou(i)*1)^(1/3);
   else
       a1(i) = 0.1506e-3 * n(i) * (rou(i)*1)^(1/3);
       b1(i) = 0.1506e-3 * m(i) * (rou(i)*1)^(1/3);
   end
end

Con_A = 1/2 *  1./R_yy_w;
Con_B = 1/2 * (1./R_xx_w+1./R_xx_r);

% cos_beta_v2, m, n, r
BGmnr = [0	0.1711	0.3329	0.4781	0.6022	0.7036	0.7836	0.8446	0.89	0.9231	0.9467	0.9634	0.975	0.9831	0.9886	0.9923	0.9949	0.9966	0.9977	0.9985	0.999
1	1.1257	1.2754	1.4536	1.6652	1.916	2.2121	2.5609	2.9708	3.4514	4.0141	4.6721	5.441	6.3387	7.3864	8.6088	10.0346	11.6976	13.637	15.8984	18.5353
1	0.8942	0.8047	0.7285	0.6629	0.6059	0.5557	0.511	0.4708	0.4345	0.4014	0.3711	0.3433	0.3177	0.2941	0.2722	0.2521	0.2334	0.2161	0.2001	0.1854
1	0.9934	0.9741	0.9436	0.9036	0.8566	0.8048	0.7503	0.6949	0.6398	0.5861	0.5346	0.4859	0.4401	0.3974	0.358	0.3217	0.2885	0.2582	0.2307	0.2058]';
% cos_beta_v2 = -1*(1./R_yy_w-1./R_xx_w-1./R_xx_r) ./ (1./R_yy_w+1./R_xx_w+1./R_xx_r);        % Thompson 书籍公式
cos_beta_v2 = abs(1./R_yy_w-1./R_xx_w-1./R_xx_r) ./ (1./R_yy_w+1./R_xx_w+1./R_xx_r);
Con_r = interp1(BGmnr(:,1), BGmnr(:,4), cos_beta_v2, 'linear');

elastic_permeability_Unit = zeros(length(rou),1);
if strcmp(Type_Normal, 'Hertz&ConDamp') || strcmp(Type_Normal, 'Hertz') || strcmp(Type_Normal, 'STRIPES&ConDamp')
    for i = 1:1:length(rou)
        if a1(i) > b1(i)
            psi(i) = b1(i)./a1(i);
            fun = @(phi) 1./sqrt(1-(1-psi(i).^2).*sin(phi).^2);
            temp = integral(fun,0,pi/2);
            elastic_permeability_Unit(i,1) = 3*(1-Vr^2)/(pi*Er*a1(i))*temp;
        else
            psi(i) = a1(i)./b1(i);
            fun = @(phi) 1./sqrt(1-(1-psi(i).^2).*sin(phi).^2);
            temp = integral(fun,0,pi/2);
            elastic_permeability_Unit(i,1) = 3*(1-Vr^2)/(pi*Er*b1(i))*temp;
        end
    end    
end