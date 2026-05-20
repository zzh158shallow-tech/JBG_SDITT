function [Profile_Bezier, MileageInterp_Div] = ...
    Create_BezierIntData(Mileage_sum, MileageInterp_Div, num_interp_Bezier, Choose_Plot, SIP_Plot, Coff_YZ, Choose_zgygRaise, Choose_zjgUnEven)

% num_interp_Bezier = 1000;
% Mileage_sum = Mileage_sum_R2;
% MileageInterp_Div = MileageInterp_Div_R2;

% TEST
% Mileage_sum = InpPar.Mileage_sum.R3;
% MileageInterp_Div = MileageInterp_Div_R3;
% num_interp_Bezier = InpPar.num_interp_Bezier;
% Choose_Plot = 0;
% SIP_Plot = 50;
% Coff_YZ= 1000;
% Choose_zgygRaise = 1;
% TEST

if nargin == 6
    Choose_zgygRaise = 0;
    Choose_zjgUnEven = 0;
end

Profile_Bezier = struct('xx', [], 'yy', [], 'zz', []);
for i = 1:1:size(MileageInterp_Div,1)
    bools = cell2mat(Mileage_sum(:,1))>=MileageInterp_Div(i,1) & cell2mat(Mileage_sum(:,1))<=MileageInterp_Div(i,2);
    profile_interp_sum = Mileage_sum(bools,:);

    pos = find(diff(cell2mat(profile_interp_sum(:,1)))==0);
    if pos == size(profile_interp_sum,1)-1
        profile_interp_sum(end,:) = [];
    elseif pos==1
        profile_interp_sum(1,:) = [];
    end

    MileageInterp_Div(i,3) = size(profile_interp_sum,1);
    
    bools_mark = [1:75:num_interp_Bezier, num_interp_Bezier];
    for j = 1:1:size(profile_interp_sum,1)
        filename_profile = profile_interp_sum{j,2};
        profile_r_ori  = load(filename_profile);
        
        bools = profile_r_ori(:,2)>23e-3;
        profile_r_ori(bools,:) = [];

        if Choose_zgygRaise==1
            % v7: 控制 45 mm 顶宽断面实际降低值1.8 mm
            xy = [0 0
                    53.200 0.0 % 实际尖端
                    54.274 1.1 % 45 mm (设计降低值 0.70 mm)
                    54.425 1.25 % 50 mm (设计降低值 0 mm)
                    55.051 0.61 % 71 mm
                    1000 0.61];

            % v7b: 控制45 mm顶宽左右实际降低值1.5 mm
%             xy = [0 0
% 	            53.200 0.0      % 实际尖端
% 	            54.274 0.8      % 45 mm (设计降低值 0.70 mm)
% 	            54.425 0.912  % 50 mm (设计降低值 0 mm)
% 	            55.051 0.45    % 71 mm
%                 1000    0.45];

            % v7c: 控制45 mm顶宽左右实际降低值1.2 mm
%             xy = [0 0
% 	            53.200 0.0      % 实际尖端
% 	            54.274 0.5      % 45 mm (设计降低值 0.70 mm)
% 	            54.425 0.57  % 50 mm (设计降低值 0 mm)
% 	            55.051 0.28    % 71 mm
%                 1000    0.28];

            xy(:,1) = xy(:,1)+50;
            xx = profile_interp_sum{j,1};
            yy_Raise = interp1(xy(:,1), xy(:,2), xx, 'linear');
            profile_r_ori(:,2) = profile_r_ori(:,2) - yy_Raise/1000;
        end

        if Choose_zjgUnEven==1
            % 实测尖轨降低值超差
            xy = [0        0.9      % 尖轨尖端
                    0.578 -0.4      % 3 mm
                    2.889  1.9       % 15 mm
                    7.304  1.7       % 40 mm
                    10.962   0       % 71 mm
                    1000      0];
            xy(:,1) = xy(:,1)+50;
            xx = profile_interp_sum{j,1};
            yy_UnEven = interp1(xy(:,1), xy(:,2), xx, 'linear');
            profile_r_ori(:,2) = profile_r_ori(:,2) + yy_UnEven/1000;
        end
        
        yy = linspace(profile_r_ori(1,1),profile_r_ori(end,1),num_interp_Bezier)';
        zz = interp1(profile_r_ori(:,1),profile_r_ori(:,2),yy,'linear');
        Profile_Bezier.xx = [Profile_Bezier.xx profile_interp_sum{j,1}];
        Profile_Bezier.yy = [Profile_Bezier.yy yy];
        Profile_Bezier.zz = [Profile_Bezier.zz zz];
        
        if Choose_Plot == 1
            if i==1 && j==1
%                 clf;
            end
            hold on
            plot3(ones(num_interp_Bezier,1)*profile_interp_sum{j,1}-SIP_Plot, yy*Coff_YZ, zz*Coff_YZ, 'b--', 'Linewidth', 0.5);
            plot3(ones(length((bools_mark)),1)*profile_interp_sum{j,1}-SIP_Plot, yy(bools_mark)*Coff_YZ, zz(bools_mark)*Coff_YZ, 'c*', 'MarkerSize', 3);
        end
    end
    
    t = (0:0.0001:1)';
    for j = bools_mark
        interp_xyz = Bezier_Curve(Profile_Bezier.xx(1,sum(MileageInterp_Div(1:i-1,3))+1:sum(MileageInterp_Div(1:i,3))),...
                                                   Profile_Bezier.yy(j,sum(MileageInterp_Div(1:i-1,3))+1:sum(MileageInterp_Div(1:i,3))),...
                                                   Profile_Bezier.zz(j,sum(MileageInterp_Div(1:i-1,3))+1:sum(MileageInterp_Div(1:i,3))), t, '3D');
        if Choose_Plot == 1
            hold on
            plot3(interp_xyz(:,1)-SIP_Plot, interp_xyz(:,2)*Coff_YZ, interp_xyz(:,3)*Coff_YZ, 'm', 'Linewidth', 1.5);
        end
    end
    eval(['Profile_Bezier.x2t_Div',num2str(i),' = [interp_xyz(:,1), t];']);
%     Profile_Bezier.x2t_Div1 = [interp_xyz(:,1), t];
    
end

if Choose_Plot == 1
    set(gca,'ydir','reverse');
    set(gca,'zdir','reverse');
    xlabel('Longitudinal direction [m]')
    ylabel('Lateral direction [m]')
    grid on
    view([-90,90]);
    kk_start = 1;
    kk_end = size(MileageInterp_Div,1);
    xlim([MileageInterp_Div(kk_start,1),MileageInterp_Div(kk_end,2)])
end
