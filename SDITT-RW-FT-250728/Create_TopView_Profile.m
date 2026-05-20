%% 绘制钢轨廓形俯视图

function Create_TopView_Profile(Fig_num, FontSize, Choose_Panel, Col_yy, Range_colorbar, dx_plot, dy_plot)

Type_Panel = {'Switch', 'Crossing'};

%% B 构造道岔三维廓形矩阵
% Path = 'D:\2021-10-10 Braking in S&C\SDITT_FT_FEM_211224_Beam189_Thr\';
% Path = 'F:\2021-10-10 Braking in S&C\SDITT_FT_FEM_211224_Beam189_Thr\';
% addpath([Path, 'WRProfile-07(009)-1_18\07(009)-Mileage'])
% addpath([Path, 'WRProfile-07(009)-1_18\07(009)-qjbg-20200418'])
% addpath([Path, 'WRProfile-07(009)-1_18\07(009)-zjg-20200418'])
% addpath([Path, 'WRProfile-07(009)-1_18\07(009)-zgyg-20200424'])
% addpath([Path, 'WRProfile-07(009)-1_18\07(009)-zgyg-20200509'])
% addpath([Path, 'WRProfile-07(009)-1_18\07(009)-cxg-20200422'])
addpath('E:\# Flex-Rigid-20200418\# Linear_FlexTurnout Model\Post_DP')

%% B1 qjbg_zjg
% dx_plot = 0.05;                          %%% 设置纵向插值间隔
% dy_plot = 0.05;                             %%% 设置横向插值间隔
yy = 0:dy_plot:145;                   %%% 设置横向插值坐标
xx = 0.58:dx_plot:14;                  %%% 从尖轨顶宽3mm开始绘图
Rail_inter_sum_Switch = [];
Mileage_zjg = load('Mileage-zjg.txt');
Mileage_qjbg = load('Mileage-qjbg.txt');
Mileage_qjbg(:,2) = Mileage_qjbg(:,2) - 50;
%%% 沿纵向xx提取廓形
for i = 1:1:length(xx)
    Rail_zjg = []; 
    Rail_qjbg = [];     
    Mileage = xx(i);
    % 直尖轨插值廓形
    if Mileage <= Mileage_zjg(end,2)
        Width = interp1(Mileage_zjg(:,2), Mileage_zjg(:,1), Mileage, 'linear');
        filename_prr_front = ['07(009)-zjg-20200418-',num2str(floor(Width)),'.txt'];
        if floor(Width)+1 > 72
            filename_prr_row = '07(009)-zjg-20200418-72.2.txt';
            w2 = 72.2;
        else
            filename_prr_row = ['07(009)-zjg-20200418-',num2str(floor(Width)+1),'.txt'];
            w2 = floor(Width)+1;
        end
        prr_front = load(filename_prr_front);
        prr_row = load(filename_prr_row);
        Rail_zjg(:,1) = prr_row(:,1) - (w2-Width) .* (prr_row(:,1) - prr_front(:,1));
        Rail_zjg(:,2) = prr_row(:,2) - (w2-Width) .* (prr_row(:,2) - prr_front(:,2));
    else
        Rail_zjg = load('07(009)-zjg-20200418-72.2.txt');
    end
    bools_1 = (Rail_zjg(:,2)>16e-3) & (Rail_zjg(:,1)>-32.5e-3);
    Rail_zjg(bools_1,:) = [];

    % 曲基本轨插值廓形
    if Mileage <= Mileage_qjbg(end,2)
        Width = interp1(Mileage_qjbg(:,2), Mileage_qjbg(:,1), Mileage, 'linear');
        filename_prr_front = ['07(009)-qjbg-20200418-',num2str(floor(Width)),'.txt'];
        filename_prr_row = ['07(009)-qjbg-20200418-',num2str(floor(Width)+1),'.txt'];
        prr_front = load(filename_prr_front);
        prr_row = load(filename_prr_row);
        Rail_qjbg(:,1) = prr_row(:,1) - ((floor(Width)+1)-Width) .* (prr_row(:,1) - prr_front(:,1));
        Rail_qjbg(:,2) = prr_row(:,2) - ((floor(Width)+1)-Width) .* (prr_row(:,2) - prr_front(:,2));
    else
        R = 1100-1.435/2;
        Beta = asin((Mileage+542.0151e-3)/R);
        dy = R*(1-cos(Beta))+0.012;
        Rail_qjbg = load('07(009)-qjbg-20200418-0.txt');
        Rail_qjbg(:,1) = Rail_qjbg(:,1)+dy;
    end
    bools_2 = Rail_qjbg(:,1) < Rail_zjg(end,1);
    Rail_qjbg(bools_2,:) = [];

    Rail_zjg(end,2) = NaN;
    Rail_qjbg(1,2) = NaN;
   
    Rail = [Rail_zjg; Rail_qjbg];
    Rail(:,1) = Rail(:,1) *  1000 + 35.5;     %%% 原数值为35.5
    Rail(:,2) = Rail(:,2) *  1000;
    
    %%% 廓形沿横向插值
    Rail_inter = [yy', interp1(Rail(:,1), Rail(:,2), yy', 'linear')];
    
    %%% 廓形汇总
    Rail_inter_sum_Switch = [Rail_inter_sum_Switch Rail_inter(:,2)];   
end

%% B2 zgyg_cxg
% dx = 0.05;                                  %%% 设置纵向插值间隔
yy = 0:dy_plot:145;                           %%% 设置横向插值坐标
xx = 53.148-1:dx_plot:53.148+7;
Rail_inter_sum_Crossing = [];
Mileage_cxg = load('Mileage-cxg.txt');

fid = fopen('07(009)-Mileage-zjg_zgyg.txt');
Mileage_sum_R2_ori = textscan(fid, '%f%s', 'HeaderLines', 0, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
fclose(fid);
Mileage_sum_R2 = cell(size(Mileage_sum_R2_ori{1,1},1)-76,2);
Range_i = 1:1:size(Mileage_sum_R2_ori{1,1},1);
for i = 77:1:length(Range_i)
    k = Range_i(i);
    Mileage_sum_R2{i-76,1} = Mileage_sum_R2_ori{1}(k,1)-50;
    Mileage_sum_R2(i-76,2) = Mileage_sum_R2_ori{2}(k,1);
end
Mileage_zgyg = Mileage_sum_R2;

%%% 沿纵向xx提取廓形
for i = 1:1:length(xx)
    Rail_cxg = [];
    Rail_zgyg = []; 
    Mileage = xx(i);

    % 长心轨插值廓形
    if Mileage >= Mileage_cxg(1,2)
        if Mileage <= Mileage_cxg(end,2)
            Width = interp1(Mileage_cxg(:,2), Mileage_cxg(:,1), Mileage, 'linear');
            filename_prr_front = ['07(009)-cxg-20200422-',num2str(floor(Width)),'.txt'];
            if floor(Width)+1 > 71
                filename_prr_row = '07(009)-cxg-20200422-71.3.txt';
                d_Width = 0.3;
            else
                filename_prr_row = ['07(009)-cxg-20200422-',num2str(floor(Width)+1),'.txt'];
                d_Width = 1;
            end
            prr_front = load(filename_prr_front);
            prr_row = load(filename_prr_row);
            Rail_cxg(:,1) = prr_row(:,1) - ((floor(Width)+1)-Width) .* (prr_row(:,1) - prr_front(:,1)) / d_Width;
            Rail_cxg(:,2) = prr_row(:,2) - ((floor(Width)+1)-Width) .* (prr_row(:,2) - prr_front(:,2)) / d_Width;
        else
            Rail_cxg = load('07(009)-cxg-20200422-71.3.txt');
        end
        bools_1 = Rail_cxg(:,2) > 16e-3 & Rail_cxg(:,1) > -32.5e-3;
        Rail_cxg(bools_1,:) = [];
        Rail_cxg(end,2) = NaN;
    end

    % 直股翼轨插值廓形
    if Mileage < Mileage_zgyg{end,1}
        [~, pos_F] = max(find(Mileage-cell2mat(Mileage_zgyg(:,1))>=0));
        filename_prr_front = Mileage_zgyg{pos_F,2};
        filename_prr_row = Mileage_zgyg{pos_F+1,2};
        prr_front = load(filename_prr_front);
        prr_row = load(filename_prr_row);
        Mileage_Front = Mileage_zgyg{pos_F,1};
        Mileage_Rear = Mileage_zgyg{pos_F+1,1};
        Rail_zgyg(:,1) = prr_row(:,1) - (Mileage_Rear-Mileage) .* (prr_row(:,1) - prr_front(:,1)) / (Mileage_Rear-Mileage_Front);
        Rail_zgyg(:,2) = prr_row(:,2) - (Mileage_Rear-Mileage) .* (prr_row(:,2) - prr_front(:,2)) / (Mileage_Rear-Mileage_Front);
        if ~isempty(Rail_cxg)
            bools_2 = Rail_zgyg(:,1) < Rail_cxg(end,1);
            Rail_zgyg(bools_2,:) = [];
        end
        Rail_zgyg(1,2) = NaN;
    else
        Rail_zgyg = [];
%         R = 1100-1.435/2;
%         Beta = asin((Mileage+542.0151e-3)/R);
%         dy = R*(1-cos(Beta))+0.012;
%         Rail_zgyg = load('07(009)-qjbg-20200418-0.txt');
%         Rail_zgyg(:,1) = Rail_zgyg(:,1)+dy;
    end
   
    Rail = [Rail_cxg; Rail_zgyg];
    Rail(:,1) = Rail(:,1) *  1000 + 35.5;     %%% 原数值为35.5
    Rail(:,2) = Rail(:,2) *  1000;
    
    %%% 廓形沿横向插值
    Rail_inter = [yy', interp1(Rail(:,1), Rail(:,2), yy', 'linear')];
    
    %%% 廓形汇总
    Rail_inter_sum_Crossing = [Rail_inter_sum_Crossing Rail_inter(:,2)];   
end

% Choose_Panel = 2;
if strcmp(Type_Panel{Choose_Panel}, 'Switch')
    % 尖轨实际尖端
    SIP = 50;
    Rail_inter_sum = Rail_inter_sum_Switch;
    xx = 0.58:dx_plot:14;                  %%% 从尖轨顶宽3mm开始绘图
elseif strcmp(Type_Panel{Choose_Panel}, 'Crossing')
    % 辙叉实际咽喉
    SIP = 50+ 53.148;
    Rail_inter_sum = Rail_inter_sum_Crossing;
    xx = 53.148-1:dx_plot:53.148+7;
    xx = xx-SIP+50;
end

%% C1 绘制道岔钢轨截面
% FontSize = 12;
figure(Fig_num); clf
% surf(xx, yy, Rail_inter_sum, Rail_inter_sum, 'EdgeColor','none');
h = pcolor(xx, yy, Rail_inter_sum);
set(h, 'LineStyle','none');
colorbar
% Col_x = 0:1:1;
% % Col_y = [0.5, 0.5, 0.5; 1.0, 1.0, 1.0];
% Col_y = [1.0, 1.0, 1.0; 0.5, 0.5, 0.5; ];
% Col_xx = linspace(0,1,64);
% Col_yy = [interp1(Col_x,Col_y(:,1),Col_xx,'linear')' interp1(Col_x,Col_y(:,2),Col_xx,'linear')' interp1(Col_x,Col_y(:,3),Col_xx,'linear')'];
colormap(Col_yy);
% caxis([-5 0.1]);
caxis(Range_colorbar);
view([0, -90])
