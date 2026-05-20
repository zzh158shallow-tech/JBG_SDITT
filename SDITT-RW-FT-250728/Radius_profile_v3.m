%% 计算曲率半径（参考高浩博士论文）
function Radius_curvature = Radius_profile_v3(profile_tar, p, Choose_1, Choose_2)

if Choose_1 == 1
    %%% s(x)
    [scs_0,p_0] = csaps(profile_tar(:,1),profile_tar(:,2),p,profile_tar(:,1));
    d0 = [profile_tar(:,1) scs_0];
    
    %%% s'(x)
    profile_1 = [d0(:,1) gradient(d0(:,2))./gradient(d0(:,1))];
    [scs_1,p_1] = csaps(profile_1(:,1),profile_1(:,2),p,profile_1(:,1));
    d1 = [profile_1(:,1) scs_1];
    
    %%% s''(x)
    profile_2 = [d1(:,1) gradient(d1(:,2))./gradient(d1(:,1))];
    [yy,p_2] = csaps(profile_2(:,1),profile_2(:,2),p,profile_2(:,1));
    d2 = [profile_2(:,1) yy];
     
    %%% Radius
    Radius_curvature_ori = [d2(:,1) d2(:,2)./((1+d1(:,2).^2).^(3/2))];
    if Choose_2 ~= 1
        Radius_curvature = [Radius_curvature_ori(:,1) 1./Radius_curvature_ori(:,2)];
    else
        Radius_curvature = [Radius_curvature_ori(:,1) 1./(csaps(Radius_curvature_ori(:,1),Radius_curvature_ori(:,2),p,Radius_curvature_ori(:,1)))];
    end
%     Radius_curvature_ori = [d2(:,1) ((1+d1(:,2).^2).^(3/2))./d2(:,2)];
%     if Choose_2 ~= 1
%         Radius_curvature = Radius_curvature_ori;
%     else
%         Radius_curvature = [Radius_curvature_ori(:,1) csaps(Radius_curvature_ori(:,1),Radius_curvature_ori(:,2),p,Radius_curvature_ori(:,1))];
%     end
elseif Choose_1 ~= 1
    %%% 既有程序
    profile_1 = [profile_tar(:,1) gradient(profile_tar(:,2))./gradient(profile_tar(:,1))];
    profile_2 = [profile_tar(:,1) gradient(profile_1(:,2))./gradient(profile_1(:,1))];
    Radius_curvature = [profile_tar(:,1) ((1+profile_1(:,2).^2).^(3/2))./profile_2(:,2)];
    if Choose_2 == 1
        Radius_curvature = [Radius_curvature(:,1) csaps(Radius_curvature(:,1),Radius_curvature(:,2),p,Radius_curvature(:,1))];
    end
    % Radius_curvature(:,2) = smooth(Radius_curvature(:,2),'moving'); %%% 5点平滑
    % Radius_curvature(:,2) = smooth(Radius_curvature(:,2),50,'moving'); %%% 20点平滑
end

% % 绘图对比
% figure(10)
% subplot(2,1,1)
% plot(profile_tar(:,1)*1000,profile_tar(:,2)*1000,d0(:,1)*1000,d0(:,2)*1000,'r--');
% set(gca,'ydir','reverse');grid on
% subplot(2,1,2);
% hold on
% plot(Radius_curvature(:,1)*1000,Radius_curvature(:,2)*1000,'b',...
%     Radius_curvature_ori(:,1)*1000,Radius_curvature_ori(:,2)*1000,'r--');
% % plot(Radius_curvature(:,1)*1000,Radius_curvature(:,2)*1000,'r--');
% grid on
% ylim([-1000,1000]);

