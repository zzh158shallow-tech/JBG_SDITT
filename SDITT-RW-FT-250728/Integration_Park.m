%% Newmark积分、Park积分算法

function [Zwy, Zsd, Zjsd] = Integration_Park(InpPar, xlcs, m1, Mxt, Kxt, Cxt, Pxt, Zwy, Zsd, Zjsd, drtaT, i_drtaT, Con_WS, Newmark, Park, Houbolt)

% global xlcs InpPar.Type_simulation InpPar.Int_Method

if strcmp(InpPar.Type_simulation,'Preload') && xlcs <= 3 && ~exist('Pre')   % (Preload)初始位置求初始加速度
    
    if m1==1
        Zjsd(:,1) = Mxt\(Pxt-Cxt*Zsd(:,1)-Kxt*Zwy(:,1));
    end
    % Newmark法起步,求2、3、4列
    Pyx = Pxt+Mxt*(Newmark.A1{i_drtaT}*Zwy(:,xlcs)+Newmark.A3{i_drtaT}*Zsd(:,xlcs)+Newmark.A4{i_drtaT}*Zjsd(:,xlcs))+Cxt*(Newmark.A2{i_drtaT}*Zwy(:,xlcs)+...
               Newmark.A6{i_drtaT}*Zsd(:,xlcs)+Newmark.A5{i_drtaT}*Zjsd(:,xlcs));
    % 起步位移
%     Zwy(:,1+xlcs) = Newmark.Kyx{i_drtaT}*Pyx;        % Kyx = (Kxt+A1*Mxt+A2*Cxt)\eye(11*Nw+35);
    Zwy(:,1+xlcs) = Newmark.Kyx{2,i_drtaT}*Pyx;        % Kyx = (Kxt+A1*Mxt+A2*Cxt)\eye(11*Nw+35);
    % 起步加速度
    Zjsd(:,1+xlcs) = Newmark.A1{i_drtaT}*(Zwy(:,1+xlcs)-Zwy(:,xlcs))-Newmark.A3{i_drtaT}*Zsd(:,xlcs)-Newmark.A4{i_drtaT}*Zjsd(:,xlcs);
    % 起步速度
    Zsd(:,1+xlcs) = Zsd(:,xlcs)+(1-Newmark.alpha)*drtaT*Zjsd(:,xlcs)+Newmark.alpha*drtaT*Zjsd(:,1+xlcs);
    if xlcs>=1 && xlcs<=2
        Zwy(:,4) = Zwy(:,1+xlcs);
        Zsd(:,4) = Zsd(:,1+xlcs);
        Zjsd(:,4)= Zjsd(:,1+xlcs);
    end
    
elseif strcmp(InpPar.Int_Method, 'Park')    
    % (Cal)Park法计算
    Bw = (-15/(6*drtaT)*Zwy(:,3)+1/drtaT*Zwy(:,2)-1/(6*drtaT)*Zwy(:,1));
    Bs = (-15/(6*drtaT)*Zsd(:,3)+1/drtaT*Zsd(:,2)-1/(6*drtaT)*Zsd(:,1));
    Bjz= Pxt-(10/(6*drtaT))*Mxt*Bw-Mxt*Bs-Cxt*Bw;
%     Zwy(:,4) = Park.Ajz{i_drtaT}*Bjz;
    Zwy(:,4) = Park.Ajz{2,i_drtaT}*Bjz;
    Zsd(:,4) = (10/(6*drtaT))*Zwy(:,4)+Bw;
    Zjsd(:,4)= (10/(6*drtaT))*Zsd(:,4)+Bs;
    
elseif strcmp(InpPar.Int_Method, 'Houbolt')
    % (Cal) Houbolt
    Pyx = Pxt + Mxt*(Houbolt.c2{1,i_drtaT}*Zwy(:,3)+Houbolt.c4{1,i_drtaT}*Zwy(:,2)+Houbolt.c6{1,i_drtaT}*Zwy(:,1)) + ...
                          Cxt*(Houbolt.c3{1,i_drtaT}*Zwy(:,3)+Houbolt.c5{1,i_drtaT}*Zwy(:,2)+Houbolt.c7{1,i_drtaT}*Zwy(:,1));
%     Zwy(:,4) = Houbolt.Kyx * Pyx;
    Zwy(:,4) = Houbolt.Kyx{2,i_drtaT} * Pyx;
    Zjsd(:,4) = Houbolt.c0{1,i_drtaT}*Zwy(:,4) - Houbolt.c2{1,i_drtaT}*Zwy(:,3) - Houbolt.c4{1,i_drtaT}*Zwy(:,2) - Houbolt.c6{1,i_drtaT}*Zwy(:,1);
    Zsd(:,4)  = Houbolt.c1{1,i_drtaT}*Zwy(:,4) - Houbolt.c3{1,i_drtaT}*Zwy(:,3) - Houbolt.c5{1,i_drtaT}*Zwy(:,2) - Houbolt.c7{1,i_drtaT}*Zwy(:,1);
    
% elseif strcmp(InpPar.Int_Method, 'Newmark')
    % Newmark法计算
%     Pyx=Pxt+Mxt*(Newmark.A1*Zwy(:,3)+Newmark.A3*Zsd(:,3)+Newmark.A4*Zjsd(:,3))+Cxt*(Newmark.A2*Zwy(:,3)+...
%             Newmark.A6*Zsd(:,3)+Newmark.A5*Zjsd(:,3));
%     % Kyx=(Kxt+A1*Mxt+A2*Cxt)\eye(11*Nw+NM*4+35);
%     Zwy(:,4) = Newmark.Kyx*Pyx;
%     Zjsd(:,4) = Newmark.A1*(Zwy(:,4)-Zwy(:,3))-Newmark.A3*Zsd(:,3)-Newmark.A4*Zjsd(:,3);
%     Zsd(:,4) = Zsd(:,3)+(1-Newmark.alpha)*drtaT*Zjsd(:,3)+Newmark.alpha*drtaT*Zjsd(:,4);

end