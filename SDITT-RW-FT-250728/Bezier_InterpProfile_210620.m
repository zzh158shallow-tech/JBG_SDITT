%% 利用 Bezier 曲线沿纵向插值截面
function InpPar = Bezier_InterpProfile_210620(InpPar, Choose_Plot, Choose_zgygRaise, Choose_zjgUnEven)

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

% fid = fopen('07(009)-Mileage-zjg_zgyg.txt');
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

    % Pf_2: Trail profile reduction
%     bools = true(size(InpPar.Mileage_sum.R3,1),1);
%     % 30, 35, 40, 45, 50, 55, 60mm
%     bools([21:24, 26:29, 31:34, 36:39, 41:44, 46:49]) = false;

    % Pf_3: Trail profile reduction
%     bools = true(size(InpPar.Mileage_sum.R3,1),1);
%     % 30, 35, 40, 45, 50, 55, 60, 65, 71.3 mm
%     bools([21:24, 26:29, 31:34, 36:39, 41:44, 46:49, 51:54, 56:62]) = false;

    % Pf_3b=8: Trail profile reduction
%     bools = true(size(InpPar.Mileage_sum.R3,1),1);
%     % 30, 35, 40, 45, 50, 55, 60, 65mm
%     bools([21:24, 26:29, 31:34, 36:39, 41:44, 46:49, 51:54]) = false;

    % Pf_4: Trail profile reduction
%     bools = true(size(InpPar.Mileage_sum.R3,1),1);
%     % 40, 45, 50, 55, 60mm
%     bools([31:34, 36:39, 41:44, 46:49]) = false;

    % Pf_4b=9: Trail profile reduction
%     bools = true(size(InpPar.Mileage_sum.R3,1),1);
%     % 50, 55, 60mm
%     bools([41:44, 46:49]) = false;

    % Pf_4c=10: Trail profile reduction (CR400AF, Trail)
%     bools = true(size(InpPar.Mileage_sum.R3,1),1);
%     % 45, 50, 55, 60mm
%     bools([36:39, 41:44, 46:49]) = false;

    % Pf_5: Trail profile reduction
%     bools = true(size(InpPar.Mileage_sum.R3,1),1);
%     % 30, 35, 45, 50, 55, 60mm
%     bools([21:24, 26:34, 36:39, 41:44, 46:49]) = false;

    % Pf_6: Trail profile reduction (CR400BF, Trail)
    bools = true(size(InpPar.Mileage_sum.R3,1),1);
    % 30, 35, 50, 55, 60mm
    bools([21:24, 26:39, 41:44, 46:49]) = false;

    % Pf_7: Trail profile reduction
%     bools = true(size(InpPar.Mileage_sum.R3,1),1);
%     % 30, 35, 40, 45, 50mm
%     bools([21:24, 26:29, 31:34, 36:39]) = false;

    InpPar.Mileage_sum.R3 = InpPar .Mileage_sum.R3(bools,:);
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
end


%%% R3: cxg
SIP_Plot = 50+53.148;
SIP_Plot = 0;
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

pos_1 = find(strcmp(InpPar.Mileage_sum.R3(:,2), '07(009)-cxg-20200422-22.5.txt'));
pos_2 = find(strcmp(InpPar.Mileage_sum.R3(:,2), '07(009)-cxg-20200422-50.txt'));
% MileageInterp_Div_R3 = [InpPar.Mileage_sum.R3(1,1),           InpPar.Mileage_sum.R3(pos_1,1)
%                                          InpPar.Mileage_sum.R3(pos_1,1),   InpPar.Mileage_sum.R3(pos_2,1)
%                                          InpPar.Mileage_sum.R3(pos_2,1),   InpPar.Mileage_sum.R3(end,1)];

MileageInterp_Div_R3 = cell2mat(MileageInterp_Div_R3);
[InpPar.Profile_Bezier.R3, InpPar.MileageInterp_Div.R3] = ...
Create_BezierIntData(InpPar.Mileage_sum.R3, MileageInterp_Div_R3, InpPar.num_interp_Bezier, Choose_Plot, SIP_Plot, Coff_YZ);


if Choose_Plot == 1
    %%% Ph.D. Thesis: Wing rail in the through route+Long point rail
    Fontsize = 12;
    figure(12);
    xlim([102.5, 105]-SIP_Plot);
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
    xlim([103, 105]-SIP_Plot);
    set(gcf, 'Position', [30   160   600   800])
    xlabel('\fontname{宋体}距实际咽喉距离\fontname{Times} (m)')
    ylabel('\fontname{宋体}廓形坐标系横坐标\fontname{Times} (m)')
    zlabel('\fontname{宋体}廓形坐标系竖坐标\fontname{Times} (m)')
    view([-130 40])
    set(gcf, 'Position', [30   420   1000   535])
    view([-120 55])
    set(gcf, 'Position', [30   212   1070   740])
    set(gca, 'XTick', -20:0.25:200)
    set(gca, 'ZTick', -5:5:25)
    xlabel('\fontname{宋体}距实际咽喉距离\fontname{Times} (m)', 'Rotation', 0)
    ylabel('\fontname{宋体}廓形坐标系横坐标\fontname{Times} (m)', 'Rotation', 14.5)
    zlabel('\fontname{宋体}廓形坐标系竖坐标\fontname{Times} (m)')
    view([-180 0])
end


