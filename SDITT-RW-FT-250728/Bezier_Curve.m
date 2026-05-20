%% BEZIER ªÊ÷∆Bezier«˙œﬂ
function output = Bezier_Curve(x, y, z, t, Opt)

% Profile_R2_Bezier
% x = Profile_R2_Bezier.xx(1,sum(MileageInterp_R2_div(1:i-1,3))+1:sum(MileageInterp_R2_div(1:i,3)));
% y = Profile_R2_Bezier.yy(j,sum(MileageInterp_R2_div(1:i-1,3))+1:sum(MileageInterp_R2_div(1:i,3)));
% z = Profile_R2_Bezier.zz(j,sum(MileageInterp_R2_div(1:i-1,3))+1:sum(MileageInterp_R2_div(1:i,3)));
% t = (0:0.001:1)';

if strcmp(Opt, '3D')
    format long
    NumPoint = length(y)-1;
    xx = (1-t).^(NumPoint)*x(1);
    yy = (1-t).^(NumPoint)*y(1);
    zz = (1-t).^(NumPoint)*z(1);
    for j = 1:1:NumPoint
        w = factorial(NumPoint)/(factorial(j)*factorial(NumPoint-j)).*(1-t).^(NumPoint-j).*t.^(j);
%         w = nchoosek(NumPoint,j).*(1-t).^(NumPoint-j).*t.^(j);
        xx = xx + w*x(j+1);
        yy = yy + w*y(j+1);
        zz = zz + w*z(j+1);
    end
    output = [xx,yy,zz];
    format short
%     format("default")     % Matlab 2022a
    
elseif strcmp(Opt, '2D')
% %     format long
%     NumPoint = length(y)-1;
%     yy = (1-t).^(NumPoint)*y(1);
%     zz = (1-t).^(NumPoint)*z(1);
%     for j = 1:1:NumPoint
%         w = factorial(NumPoint)/(factorial(j)*factorial(NumPoint-j)).*(1-t).^(NumPoint-j).*t.^(j);
%         yy = yy + w*y(j+1);
%         zz = zz + w*z(j+1);
%     end
%     output = [yy,zz];
% %     format("default")
% %     format short

    NumPoint = length(y)-1;
%     yy = zeros(length(y),1);
%     zz = zeros(length(y),1);
%     yy(1) = (1-t).^(NumPoint)*y(1);
%     zz(1) = (1-t).^(NumPoint)*z(1);
%     j = 0;
    j = (0:1:NumPoint)';
    w = factorial(NumPoint)./(factorial(j).*factorial(NumPoint-j)) .* (1-t).^(NumPoint-j) .* t.^(j);
    yy = w.*y';
    zz = w.*z';
    output = [sum(yy), sum(zz)];

    
end

% figure(2); clf
% plot(x, y, xx, yy); grid on
