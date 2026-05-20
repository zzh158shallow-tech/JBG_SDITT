%% 利用 Bezier 曲线沿纵向插值截面
function InpPar = Bezier_InterpProfile_CN18_250813(InpPar, Choose_Plot, Choose_zgygRaise, Choose_zjgUnEven)

Choose_Plot_Switch = 0;
Choose_Plot_Crossing = 0;
Choose_Plot_zjbg = 0;
Choose_Plot_cxg = 0;
% Choose_Plot_Switch = 1;
% Choose_Plot_Crossing = 1;
% Choose_Plot_zjbg = 1;
% Choose_Plot_cxg = 1;

InpPar.num_interp_Bezier = 1000;

%% 1. 导入里程文件
fid = fopen('CN18-Mileage-zjbg.txt');
Mileage_sum_L1_ori = textscan(fid, '%f%s', 'HeaderLines', 0, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
fclose(fid);
InpPar.Mileage_sum.L1 = cell(size(Mileage_sum_L1_ori{1,1},1)-1,2);
Range_i = 2:1:size(Mileage_sum_L1_ori{1,1},1);
for i = 1:1:length(Range_i)
    k = Range_i(i);
    InpPar.Mileage_sum.L1{i,1} = Mileage_sum_L1_ori{1}(k,1);
    InpPar.Mileage_sum.L1(i,2) = Mileage_sum_L1_ori{2}(k,1);
end

fid = fopen('CN18-Mileage-qjbg.txt');
Mileage_sum_R1_ori = textscan(fid, '%f%s', 'HeaderLines', 0, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
fclose(fid);
InpPar.Mileage_sum.R1 = cell(size(Mileage_sum_R1_ori{1,1},1)-1,2);
Range_i = 2:1:size(Mileage_sum_R1_ori{1,1},1);
for i = 1:1:length(Range_i)
    k = Range_i(i);
    InpPar.Mileage_sum.R1{i,1} = Mileage_sum_R1_ori{1}(k,1);
    InpPar.Mileage_sum.R1(i,2) = Mileage_sum_R1_ori{2}(k,1);
end

fid = fopen('CN18-Mileage-zjg_zgyg.txt');
% fid = fopen('CN18-Mileage-zjg_zgyg-Add-0.025m.txt');
% fid = fopen('CN18-Mileage-zjg_zgyg-Add-0.0125m.txt');
% fid = fopen('CN18-Mileage-zjg_zgyg-Add-0.03m.txt');
Mileage_sum_R2_ori = textscan(fid, '%f%s', 'HeaderLines', 0, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
fclose(fid);
InpPar .Mileage_sum.R2 = cell(size(Mileage_sum_R2_ori{1,1},1),2);
Range_i = 1:1:size(Mileage_sum_R2_ori{1,1},1);
for i = 1:1:length(Range_i)
    k = Range_i(i);
    InpPar.Mileage_sum.R2{i,1} = Mileage_sum_R2_ori{1}(k,1);
    InpPar.Mileage_sum.R2(i,2) = Mileage_sum_R2_ori{2}(k,1);
end

fid = fopen('CN18-Mileage-cx.txt');
Mileage_sum_R3_ori = textscan(fid, '%f%s', 'HeaderLines', 0, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
fclose(fid);
InpPar.Mileage_sum.R3 = cell(size(Mileage_sum_R3_ori{1,1},1),2);
Range_i = 1:1:size(Mileage_sum_R3_ori{1,1},1);
for i = 1:1:length(Range_i)
    k = Range_i(i);
    InpPar.Mileage_sum.R3{i,1} = Mileage_sum_R3_ori{1}(k,1);
    InpPar.Mileage_sum.R3(i,2) = Mileage_sum_R3_ori{2}(k,1);
end
if strcmp(InpPar.VehicleDir, 'Trail')
    % 43-'CN18-cx-44.txt'，降低值（理论/实际）=2.0/3.8mm
    % 54-'CN18-cx-55.txt'，降低值（理论/实际）=0.4/2.7mm
    % 70-'CN18-cx-71.txt'，降低值（理论/实际）=0.1/1.5mm
    bools = false(size(InpPar.Mileage_sum.R3,1),1);
    % Pf_1: Trail profile reduction
%     bools([1:43, 70:end, 47:4:67]) = true;
    % Pf_2: Trail profile reduction
%     bools([1:43, 70:end, 45:2:67]) = true;
    % Pf_3: Trail profile reduction
    bools([1:44, 70:end, 46:2:54, 56:3:68]) = true;
    InpPar.Mileage_sum.R3 = InpPar .Mileage_sum.R3(bools,:);
end

%% 变截面廓形沿纵向插值 Profile_R2_Bezier
Coff_YZ = 1000;
if Choose_Plot_Switch ==1 || Choose_Plot_zjbg ==1
    SIP_Plot = 50;
elseif Choose_Plot_Crossing ==1 || Choose_Plot_cxg ==1
    SIP_Plot = 50+53.253;
end
SIP_Plot = 0;

%% L1: zjbg
if Choose_Plot_zjbg == 1
    figure(11); clf;
end
MileageInterp_Div_L1 = [InpPar.Mileage_sum.L1(1,1), InpPar.Mileage_sum.L1(end,1)];
MileageInterp_Div_L1 = cell2mat(MileageInterp_Div_L1);
[InpPar.Profile_Bezier.L1, InpPar.MileageInterp_Div.L1] = ...
Create_BezierIntData(InpPar.Mileage_sum.L1, MileageInterp_Div_L1, InpPar.num_interp_Bezier, Choose_Plot_zjbg, SIP_Plot, Coff_YZ);

%% R1: qjbg
if Choose_Plot_Switch == 1
    figure(11); clf;
end
MileageInterp_Div_R1 = [InpPar.Mileage_sum.R1(1,1), InpPar.Mileage_sum.R1(end,1)];
MileageInterp_Div_R1 = cell2mat(MileageInterp_Div_R1);
[InpPar.Profile_Bezier.R1, InpPar.MileageInterp_Div.R1] = ...
Create_BezierIntData(InpPar.Mileage_sum.R1, MileageInterp_Div_R1, InpPar.num_interp_Bezier, Choose_Plot_Switch, SIP_Plot, Coff_YZ);

%% R2: zjg_zgyg
if Choose_Plot_Switch == 1
    figure(11);
end
if Choose_Plot_Crossing == 1
    figure(12); clf;
end
% 114, 104.697m, 'CN18_zgyg_54.697m.txt'
% MileageInterp_Div_R2 = [InpPar.Mileage_sum.R2(1,1),    InpPar.Mileage_sum.R2(end,1)];
MileageInterp_Div_R2 = [InpPar.Mileage_sum.R2(1,1),    InpPar.Mileage_sum.R2(73,1)
                                         InpPar.Mileage_sum.R2(73,1),  InpPar.Mileage_sum.R2(77,1)
                                         InpPar.Mileage_sum.R2(77,1),  InpPar.Mileage_sum.R2(114,1)
                                         InpPar.Mileage_sum.R2(114,1),InpPar.Mileage_sum.R2(end,1)];
MileageInterp_Div_R2 = cell2mat(MileageInterp_Div_R2);
[InpPar.Profile_Bezier.R2, InpPar.MileageInterp_Div.R2] = ...
Create_BezierIntData(InpPar.Mileage_sum.R2, MileageInterp_Div_R2, InpPar.num_interp_Bezier, max(Choose_Plot_Switch, Choose_Plot_Crossing), SIP_Plot, Coff_YZ, Choose_zgygRaise, Choose_zjgUnEven);

if Choose_Plot_Switch == 1 || Choose_Plot_zjbg == 1
    Fontsize = 12;
    figure(11);
    xlim([50, 61]-SIP_Plot);
    set(gca, 'XTick', 0:1:20)
    ylim([-0.040, 0.110]*1000)
    zlim([-0.005, 0.023]*1000)
    set(gca, 'XTick', -20:1:1000)
    set(gca, 'ZTick', -5:5:25)
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
    set(gcf, 'Position', [30   100   1070*0.75   740*0.75])
    xlabel('\fontname{宋体}距尖轨尖端距离\fontname{Times} (m)', 'Rotation', -20)
    ylabel('\fontname{宋体}廓形坐标系横坐标\fontname{Times} (m)', 'Rotation', 14.5)
    zlabel('\fontname{宋体}廓形坐标系竖坐标\fontname{Times} (m)')
end


%% R3: cxg
% figure(15); clf
% plot(cell2mat(InpPar.Mileage_sum_R3(1:end-1,1)), diff(cell2mat(InpPar.Mileage_sum_R3(:,1))))
% grid on
if Choose_Plot_Crossing == 1 || Choose_Plot_cxg == 1
    figure(12);
    if Choose_Plot_cxg == 1
        clf
    end
end
pos = find(strcmp(InpPar.Mileage_sum.R3(:,2), 'CN18-cx-179.6.txt'));
if strcmp(InpPar.VehicleDir, 'Face')
    MileageInterp_Div_R3 = [InpPar.Mileage_sum.R3(1,1), InpPar.Mileage_sum.R3(19,1);
                                             InpPar.Mileage_sum.R3(19,1), InpPar.Mileage_sum.R3(54,1);
                                             InpPar.Mileage_sum.R3(54,1), InpPar.Mileage_sum.R3(pos,1);
                                             InpPar.Mileage_sum.R3(pos,1), InpPar.Mileage_sum.R3(end,1)];
elseif strcmp(InpPar.VehicleDir, 'Trail')
    MileageInterp_Div_R3 = [InpPar.Mileage_sum.R3(1,1), InpPar.Mileage_sum.R3(pos,1);
                                             InpPar.Mileage_sum.R3(pos,1), InpPar.Mileage_sum.R3(end,1)];
%     MileageInterp_Div_R3 = [InpPar.Mileage_sum.R3(1,1), InpPar.Mileage_sum.R3(19,1);
%                                              InpPar.Mileage_sum.R3(19,1), InpPar.Mileage_sum.R3(pos,1);
%                                              InpPar.Mileage_sum.R3(pos,1), InpPar.Mileage_sum.R3(end,1)];
end
MileageInterp_Div_R3 = cell2mat(MileageInterp_Div_R3);
[InpPar.Profile_Bezier.R3, InpPar.MileageInterp_Div.R3] = ...
Create_BezierIntData(InpPar.Mileage_sum.R3, MileageInterp_Div_R3, InpPar.num_interp_Bezier, max(Choose_Plot_Crossing, Choose_Plot_cxg), SIP_Plot, Coff_YZ);

if Choose_Plot_Crossing == 1 || Choose_Plot_cxg == 1
    %%% Ph.D. Thesis: Wing rail in the through route+Long point rail
    Fontsize = 12;
    figure(12);
%     xlim([-1, 2]);
    zlim([-0.005, 0.023]*Coff_YZ)
    ylim([-0.040, 0.145]*Coff_YZ)
    set(gca, 'fontname', 'Times', 'fontsize', Fontsize);

    % get(gcf, 'Position')
    view([90 77])
    set(gcf, 'Position', [30   160   600   800])
    xlabel('\fontname{宋体}距实际咽喉距离\fontname{Times} (m)')
    ylabel('\fontname{宋体}廓形坐标系横坐标\fontname{Times} (mm)')
    zlabel('\fontname{宋体}廓形坐标系竖坐标\fontname{Times} (mm)', 'Rotation', 90)

    xlim([100,106]);
    view([-90 65])
    set(gcf, 'Position', [30   160   600   800])
    xlabel('\fontname{宋体}距实际咽喉距离\fontname{Times} (m)')
    ylabel('\fontname{宋体}廓形坐标系横坐标\fontname{Times} (m)')
    zlabel('\fontname{宋体}廓形坐标系竖坐标\fontname{Times} (m)')

    view([-130 40])
    set(gcf, 'Position', [30   100   1070*0.75   740*0.75])
    set(gca, 'XTick', -20:0.25:1000)
    set(gca, 'ZTick', -5:5:25)

    if strcmp(InpPar.VehicleDir, 'Face')
        view([0 0])
    elseif strcmp(InpPar.VehicleDir, 'Trail')
        view([-180 0])
    end
    xlabel('\fontname{宋体}距实际咽喉距离\fontname{Times} (m)', 'Rotation', 0)
    zlabel('\fontname{宋体}廓形坐标系竖坐标\fontname{Times} (m)')
    xlim([103.25, 105.75]-SIP_Plot);
end

