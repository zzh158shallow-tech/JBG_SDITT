%% 利用 Bezier 曲线沿纵向插值截面
function InpPar = Bezier_InterpProfile_210620_TrailTest(InpPar, Choose_Plot, Choose_zgygRaise, Choose_zjgUnEven)

% clc
% clear

% global InpPar.Mileage_sum InpPar.MileageInterp_Div InpPar.Profile_Bezier

Choose_Plot = 0;
% Choose_Plot = 1;

InpPar.SIP = 0;
InpPar.num_interp_Bezier = 1000;

%% 1. 导入里程文件
fid = fopen('07(009)-Mileage-qjbg.txt');
Mileage_sum_R1_ori = textscan(fid, '%f%s', 'HeaderLines', 0, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
fclose(fid);
InpPar.Mileage_sum.R1 = cell(size(Mileage_sum_R1_ori{1,1},1)-3,2);
Range_i = 3:1:size(Mileage_sum_R1_ori{1,1},1)-1;
for i = 1:1:length(Range_i)
    k = Range_i(i);
    InpPar.Mileage_sum.R1{i,1} = Mileage_sum_R1_ori{1}(k,1);
    InpPar.Mileage_sum.R1(i,2) = Mileage_sum_R1_ori{2}(k,1);
end

fid = fopen('07(009)-Mileage-zjg_zgyg_250722.txt');
Mileage_sum_R2_ori = textscan(fid, '%f%s', 'HeaderLines', 0, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
fclose(fid);
InpPar .Mileage_sum.R2 = cell(size(Mileage_sum_R2_ori{1,1},1),2);
Range_i = 1:1:size(Mileage_sum_R2_ori{1,1},1);
for i = 1:1:length(Range_i)
    k = Range_i(i);
    InpPar.Mileage_sum.R2{i,1} = Mileage_sum_R2_ori{1}(k,1);
    InpPar.Mileage_sum.R2(i,2) = Mileage_sum_R2_ori{2}(k,1);
end

fid = fopen('07(009)-Mileage-cxg.txt');
% fid = fopen('07(009)-Mileage-cxg-250801.txt');
Mileage_sum_R3_ori = textscan(fid, '%f%s', 'HeaderLines', 0, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
fclose(fid);
InpPar.Mileage_sum.R3 = cell(size(Mileage_sum_R3_ori{1,1},1)-1,2);
Range_i = 1:1:size(Mileage_sum_R3_ori{1,1},1)-1;
for i = 1:1:length(Range_i)
    k = Range_i(i);
    InpPar.Mileage_sum.R3{i,1} = Mileage_sum_R3_ori{1}(k,1);
    InpPar.Mileage_sum.R3(i,2) = Mileage_sum_R3_ori{2}(k,1);
end

if strcmp(InpPar.VehicleDir, 'Trail')
    % Pf_1: Trail profile reduction
%     bools = true(size(InpPar.Mileage_sum.R3,1),1);
%     bools([16:19, 21:24, 26:29, 31:34, 36:39, 41:44, 46:49, 51:54, 56:62]) = false;
%     % 25, 30, 35, 40, 45, 50, 55, 60, 65, 71.3 mm
%     InpPar.Mileage_sum.R3 = InpPar .Mileage_sum.R3(bools,:);

    % Pf_2: Trail profile reduction
    bools = true(size(InpPar.Mileage_sum.R3,1),1);
    % 30, 35, 40, 45, 50, 55, 60mm
    bools([21:24, 26:29, 31:34, 36:39, 41:44, 46:49]) = false;
    InpPar.Mileage_sum.R3 = InpPar .Mileage_sum.R3(bools,:);

    % Pf_3: Trail profile reduction
%     bools = true(size(InpPar.Mileage_sum.R3,1),1);
%     % 30, 35, 40, 45, 50, 55, 60, 65, 71.3 mm
%     bools([21:24, 26:29, 31:34, 36:39, 41:44, 46:49, 51:54, 56:62]) = false;
%     InpPar.Mileage_sum.R3 = InpPar .Mileage_sum.R3(bools,:);

    % Pf_4: Trail profile reduction
%     bools = true(size(InpPar.Mileage_sum.R3,1),1);
%     % 40, 45, 50, 55, 60mm
%     bools([31:34, 36:39, 41:44, 46:49]) = false;
%     InpPar.Mileage_sum.R3 = InpPar .Mileage_sum.R3(bools,:);

    % Pf_5: Trail profile reduction
%     bools = true(size(InpPar.Mileage_sum.R3,1),1);
%     % 30, 35, 45, 50, 55, 60mm
%     bools([21:24, 26:34, 36:39, 41:44, 46:49]) = false;
%     InpPar.Mileage_sum.R3 = InpPar .Mileage_sum.R3(bools,:);

    % Pf_6: Trail profile reduction (CR400BF, Trail)
%     bools = true(size(InpPar.Mileage_sum.R3,1),1);
%     % 30, 35, 50, 55, 60mm
%     bools([21:24, 26:39, 41:44, 46:49]) = false;
%     InpPar.Mileage_sum.R3 = InpPar .Mileage_sum.R3(bools,:);

    % Pf_7: Trail profile reduction
%     bools = true(size(InpPar.Mileage_sum.R3,1),1);
%     % 30, 35, 40, 45, 50mm
%     bools([21:24, 26:29, 31:34, 36:39]) = false;
%     InpPar.Mileage_sum.R3 = InpPar .Mileage_sum.R3(bools,:);
end

%% 变截面廓形沿纵向插值 Profile_R2_Bezier
SIP_Plot = 50;
% SIP_Plot = 50+53.148;
Coff_YZ = 1000;

%%% R1: qjbg
% figure(15); clf
% plot(cell2mat(InpPar.Mileage_sum_R1(1:end-1,1)), diff(cell2mat(InpPar.Mileage_sum_R1(:,1))))
% grid on
if Choose_Plot == 1
    figure(11); clf;
end
MileageInterp_Div_R1 = [InpPar.Mileage_sum.R1(1,1), InpPar.Mileage_sum.R1(end,1)];
MileageInterp_Div_R1 = cell2mat(MileageInterp_Div_R1);
[InpPar.Profile_Bezier.R1, InpPar.MileageInterp_Div.R1] = ...
Create_BezierIntData(InpPar.Mileage_sum.R1, MileageInterp_Div_R1, InpPar.num_interp_Bezier, Choose_Plot, SIP_Plot, Coff_YZ);

%%% R2: zjg_zgyg
% figure(15); clf
% plot(cell2mat(InpPar.Mileage_sum_R2(1:end-1,1)), diff(cell2mat(InpPar.Mileage_sum_R2(:,1))))
% grid on
if Choose_Plot == 1
    figure(11);
%     figure(12); clf;
end
% MileageInterp_Div_R2 = [InpPar.Mileage_sum.R2(1,1),    InpPar.Mileage_sum.R2(73,1)
%                                          InpPar.Mileage_sum.R2(73,1),  InpPar.Mileage_sum.R2(78,1)
%                                          InpPar.Mileage_sum.R2(78,1),  InpPar.Mileage_sum.R2(119,1)
%                                          InpPar.Mileage_sum.R2(119,1),InpPar.Mileage_sum.R2(end,1)];
% MileageInterp_Div_R2 = [InpPar.Mileage_sum.R2(1,1),    InpPar.Mileage_sum.R2(73,1)
%                                          InpPar.Mileage_sum.R2(73,1),  InpPar.Mileage_sum.R2(78,1)
%                                          InpPar.Mileage_sum.R2(79,1),  InpPar.Mileage_sum.R2(119,1)
%                                          InpPar.Mileage_sum.R2(120,1),InpPar.Mileage_sum.R2(end,1)];

% Pf_2: Face profile reduction-zgyg
MileageInterp_Div_R2 = [InpPar.Mileage_sum.R2(1,1),    InpPar.Mileage_sum.R2(73,1)
                                         InpPar.Mileage_sum.R2(73,1),  InpPar.Mileage_sum.R2(78,1)
                                         InpPar.Mileage_sum.R2(79,1),  InpPar.Mileage_sum.R2(end,1)];

MileageInterp_Div_R2 = cell2mat(MileageInterp_Div_R2);
[InpPar.Profile_Bezier.R2, InpPar.MileageInterp_Div.R2] = ...
Create_BezierIntData(InpPar.Mileage_sum.R2, MileageInterp_Div_R2, InpPar.num_interp_Bezier, Choose_Plot, SIP_Plot, Coff_YZ, Choose_zgygRaise, Choose_zjgUnEven);


if Choose_Plot == 1
    %% Ph.D. Thesis: Curved stock rail + Straight switch rail
    Fontsize = 12;
    figure(11);
    xlim([50, 61]-SIP_Plot);
    set(gca, 'XTick', 0:1:20)
    ylim([-0.040, 0.110]*1000)
    zlim([-0.005, 0.023]*1000)
    set(gca, 'fontname', 'Times', 'fontsize', Fontsize);
    set(gca, 'Position', [0.1300    0.1100    0.7548    0.8150])
    % get(gcf, 'Position')
    view([-90 65])
    view([90 70])
    set(gcf, 'Position', [30   160   600   800])
    xlabel('\fontname{宋体}距尖轨尖端距离\fontname{Times} (m)', 'Rotation', 90)
    ylabel('\fontname{宋体}廓形坐标系横坐标\fontname{Times} (mm)', 'Rotation', 0)
    zlabel('\fontname{宋体}廓形坐标系竖坐标\fontname{Times} (mm)', 'Rotation', 90)

    view([-130 40])
    set(gcf, 'Position', [30   420   1000   535])
    xlabel('\fontname{宋体}距尖轨尖端距离\fontname{Times} (m)', 'Rotation', -20)
    ylabel('\fontname{宋体}廓形坐标系横坐标\fontname{Times} (m)', 'Rotation', 14.5)
    zlabel('\fontname{宋体}廓形坐标系竖坐标\fontname{Times} (m)')

    % get(gca, 'Position')
end


%%% R3: cxg
SIP_Plot = 50+53.148;
% figure(15); clf
% plot(cell2mat(InpPar.Mileage_sum_R3(1:end-1,1)), diff(cell2mat(InpPar.Mileage_sum_R3(:,1))))
% grid on
if Choose_Plot == 1
    figure(12); clf;
%     figure(12); 
end
% MileageInterp_Div_R3 = [InpPar.Mileage_sum.R3(1,1),          InpPar.Mileage_sum.R3(end-4,1)
%                                          InpPar.Mileage_sum.R3(end-4,1),  InpPar.Mileage_sum.R3(end,1)];

MileageInterp_Div_R3 = [InpPar.Mileage_sum.R3(1,1), InpPar.Mileage_sum.R3(end,1)];

% MileageInterp_Div_R3 = [InpPar.Mileage_sum.R3(1,1),          InpPar.Mileage_sum.R3(12,1)
%                                          InpPar.Mileage_sum.R3(12,1),  InpPar.Mileage_sum.R3(end,1)];

MileageInterp_Div_R3 = cell2mat(MileageInterp_Div_R3);
[InpPar.Profile_Bezier.R3, InpPar.MileageInterp_Div.R3] = ...
Create_BezierIntData(InpPar.Mileage_sum.R3, MileageInterp_Div_R3, InpPar.num_interp_Bezier, Choose_Plot, SIP_Plot, Coff_YZ);


if Choose_Plot == 1
    %%% Ph.D. Thesis: Wing rail in the through route+Long point rail
    Fontsize = 12;
    figure(12);
    xlim([102.5, 105]-SIP_Plot);
    xlim([-1, 2]);
    zlim([-0.005, 0.023]*Coff_YZ)
    ylim([-0.040, 0.145]*Coff_YZ)
    set(gca, 'fontname', 'Times', 'fontsize', Fontsize);

    % get(gcf, 'Position')
    view([90 77])
    set(gcf, 'Position', [30   160   600   800])
    xlabel('\fontname{宋体}距实际咽喉距离\fontname{Times} (m)')
    ylabel('\fontname{宋体}廓形坐标系横坐标\fontname{Times} (mm)')
    zlabel('\fontname{宋体}廓形坐标系竖坐标\fontname{Times} (mm)', 'Rotation', 90)

    view([-90 65])
    set(gcf, 'Position', [30   160   600   800])
    xlabel('\fontname{宋体}距实际咽喉距离\fontname{Times} (m)')
    ylabel('\fontname{宋体}廓形坐标系横坐标\fontname{Times} (m)')
    zlabel('\fontname{宋体}廓形坐标系竖坐标\fontname{Times} (m)')
    view([-130 40])
    set(gcf, 'Position', [30   420   1000   535])
    view([-120 55])
    set(gcf, 'Position', [30   212   1070   740])
    set(gca, 'XTick', -20:0.25:20)
    set(gca, 'ZTick', -5:5:25)
    xlabel('\fontname{宋体}距实际咽喉距离\fontname{Times} (m)', 'Rotation', -40)
    ylabel('\fontname{宋体}廓形坐标系横坐标\fontname{Times} (m)', 'Rotation', 14.5)
    zlabel('\fontname{宋体}廓形坐标系竖坐标\fontname{Times} (m)')
    view([-180 0])
end

% if Choose_Plot == 1
%     kk_start = 1; kk_end = 1;
%     xlim([MileageInterp_Div_R3(kk_start,1),MileageInterp_Div_R3(kk_end,2)])
%     xlabel('X [m]')
% end

%% Matlab 道岔廓形三维绘图
% %%% InpPar.Profile_Bezier_R3
% profile_mesh_1=[];   %%% Point Rail
% p = sum(MileageInterp_Div_R3(1:0,3))+1;
% q = sum(MileageInterp_Div_R3(1:1,3));
% xx_R3 = InpPar.Profile_Bezier_R3.xx(1,p:q);
% yy_R3 = [min(min(InpPar.Profile_Bezier_R3.yy(:,p:q))):0.0005:min(min(InpPar.Profile_Bezier_R3.yy(:,p:q)))+0.005 ...
%          min(min(InpPar.Profile_Bezier_R3.yy(:,p:q)))+0.0055:0.001:max(max(InpPar.Profile_Bezier_R3.yy(:,p:q)))+0.001];
% %%% 导入廓形并沿横向插值
% for i = p:1:q
%     profile_mesh_1 = [profile_mesh_1 (interp1(InpPar.Profile_Bezier_R3.yy(:,i),InpPar.Profile_Bezier_R3.zz(:,i), yy_R3,'linear'))'];
% end
% 
% %%% InpPar.Profile_Bezier_R2
% profile_mesh_2=[];   %%% Wing Rail
% p = sum(MileageInterp_Div_R2(1:2,3))+1;
% q = sum(MileageInterp_Div_R2(1:4,3));
% xx_R2 = InpPar.Profile_Bezier_R2.xx(1,p:q);
% kk = min(find((xx_R2-102)>=0));
% xx_R2 = xx_R2(1,kk:end);
% p = p+kk-1;
% yy_R2 = [min(min(InpPar.Profile_Bezier_R2.yy(:,p:q))):0.00025:min(min(InpPar.Profile_Bezier_R2.yy(:,p:q)))+0.005 ...
%          min(min(InpPar.Profile_Bezier_R2.yy(:,p:q)))+0.0055:0.00075:max(max(InpPar.Profile_Bezier_R2.yy(:,p:q)))+0.001];       %%% 沿横向插值坐标间隔
% [~,index] = unique(xx_R2(1,:));
% xx_R2 = xx_R2(:,index);
% Range_i = p:1:q;
% Range_i = Range_i(index);
% %%% 导入廓形并沿横向插值
% for i = Range_i
%     profile_mesh_2 = [profile_mesh_2 (interp1(InpPar.Profile_Bezier_R2.yy(:,i),InpPar.Profile_Bezier_R2.zz(:,i),yy_R2,'linear'))'];
% end
% 
% %%% Figure
% figure(1)
% [X_R3,Y_R3] = meshgrid(yy_R3,xx_R3);
% Z_R3 = profile_mesh_1';
% mesh(X_R3,Y_R3,Z_R3,Y_R3)
% hold on
% [X_R2,Y_R2] = meshgrid(yy_R2,xx_R2);
% Z_R2 = profile_mesh_2';
% mesh(X_R2,Y_R2,Z_R2,Y_R2)
% 
% % set(gcf,'Position',[150 150 1300 800]);
% view([-13 48]);
% set(gca,'zdir','reverse');
% set(gca,'fontsize',12,'fontname','Times New Roman');
% xlabel('Y [m]');
% ylabel('Distance [m]');
% zlabel('Z [m]');
% zlim([-0.0005,0.016]);

