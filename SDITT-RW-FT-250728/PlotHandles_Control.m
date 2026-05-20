function PlotHandles = PlotHandles_Control(InpPar, j0, Fig_num_max, Fig_pos_times, Choose_Screen)

% global InpPar.Type_simulation InpPar.Exp_DummyRail

%% 1. ÉèÖÃ²ÎÊý
% if strcmp(Choose_Screen, 'Lab')
    height_screen = 9.5-4.75*0;   weight_figure = 12;   height_figure = 16;
% elseif strcmp(Choose_Screen, 'Laptop')
%     height_screen = 9.5-4.75*1;   weight_figure = 12;   height_figure = 16;
% end
Fontsize = 11;

xlim_start = j0;
plot_initial_x = [xlim_start j0];
plot_initial_y = [0 0];
LineStyle = {'-', '--', '-.', ':', '*'};
LineStyle_v2 = {'-', '-', '--', '-.', ':'};
% Color_line = {[0, 113, 188]/256, [216, 82, 24]/256, [125, 46, 141]/256, [119, 172, 48]/256, [95, 95, 95]/256};
% Color_line = {[0, 113, 188]/256, [216, 82, 24]/256, [125, 46, 141]/256, [119, 172, 48]/256, [222, 125, 0]/256};
Color_line = ...
    {[0.00, 0.45, 0.74],     [0.85, 0.33, 0.10], ...
    [0.93, 0.69, 0.13],     [0.49, 0.18, 0.56], ...
    [0.47, 0.67, 0.19],     [0.30, 0.75, 0.93], ...
    [0.64, 0.08, 0.18],     [0.07, 0.62, 1.00], ...
    [1.00, 0.41, 0.16],     [0.39, 0.83, 0.07], ...
    [0.72, 0.27, 1.00],     [0.06, 1.00, 1.00], ...
    [1.00, 0.07, 0.65],     [1.00, 1.00, 0.07]};

Label_X = 'Distance (m)';

%% Fig.1
figure(1); clf
color = {'-','r*','g*','b*','c*','r+','g+','b+','c+'};
set(gcf,'Units','centimeters','Position',[21.3+weight_figure*(Fig_pos_times-1*(Fig_pos_times==-2.1)), 1, 15, 6]);
% set(gcf,'Units','centimeters','Position',[15.32, 1.21, 15, 6]);
for i = 1:1:2
    subplot(1,2,i)
    for j = 1:1:10
        hold on
        PlotHandles.Fig_1(i,j) = plot(plot_initial_x, plot_initial_y, color{ceil(j/2)});
        set(PlotHandles.Fig_1(i,j), 'xdata', [], 'ydata', []);
        xlabel('Y (m)'); ylabel('Z (m)');
        set(gca,'ydir','reverse');grid on;
        set(gca, 'Fontname', 'Times', 'Fontsize', Fontsize);
    end
end

%% Fig.2
figure(2); clf
% if strcmp(InpPar.Type_simulation, 'Preload')
    Sub_Row = 3;
% else
%     Sub_Row = 2;
% end
set(gcf,'Units','centimeters','Position',[0.1+weight_figure*(0+Fig_pos_times), height_screen, weight_figure, height_figure]);
for i = 1:1:Sub_Row
    subplot(Sub_Row,1,i)
    for j = 1:1:8        
        hold on
        if j==InpPar.N_ConPatch+1
            PlotHandles.Fig_2(i,j) = plot(plot_initial_x, plot_initial_y, 'Color', [0.65, 0.65, 0.65]);
        else
            PlotHandles.Fig_2(i,j) = plot(plot_initial_x, plot_initial_y, 'Color', Color_line{j});
        end
        set(PlotHandles.Fig_2(i,j), 'xdata', [], 'ydata', []);
        xlabel(Label_X)
        xlim([xlim_start,inf]);
        grid on
        set(gca, 'Fontname', 'Times', 'Fontsize', Fontsize);
    end
end
subplot(Sub_Row,1,1)
if isfield(InpPar, 'Test')
    legend([InpPar.Exp_DummyRail, 'Test']);
else
    legend(InpPar.Exp_DummyRail);
end
% if strcmp (InpPar.Type_simulation, 'Preload')
%     legend('L1', 'R1');
% else
%     legend(InpPar.Exp_DummyRail);
% end
set(legend, 'Position', [0.1385 0.9360 0.7717 0.0387], 'Orientation', 'horizontal');
ylabel('FZ (kN)');
% if strcmp(InpPar.Type_simulation, 'Preload')
    subplot(Sub_Row,1,2)
    ylabel('FY (kN)');
    subplot(Sub_Row,1,3)
    ylabel('FX (kN)');
% else
%     subplot(Sub_Row,1,2)
%     ylabel('TIrr_Z (mm)');
%     set(gca, 'ydir', 'reverse')
% end

%% Fig.3
if Fig_num_max >= 3
    figure(3); clf
    set(gcf,'Units','centimeters','Position',[0.2+weight_figure*(1+Fig_pos_times), height_screen, weight_figure, height_figure]);
%     set(gcf,'Units','centimeters','Position',[0.2+weight_figure*2.5, height_screen, weight_figure, height_figure]);
    for i = 1:1:3
        subplot(3,1,i)
        for j0 = 1:1:4
            for j = 1:1:5
                hold on
%                 if i<3
                if i<2
                    PlotHandles.Fig_3(i,j+5*(j0-1)) = plot(plot_initial_x, plot_initial_y, LineStyle{j0}, 'Color', Color_line{j});
                else
                    PlotHandles.Fig_3(i,j+5*(j0-1)) = plot(plot_initial_x, plot_initial_y, LineStyle{j}, 'Color', Color_line{j0});
                end
                set(PlotHandles.Fig_3(i,j+5*(j0-1)), 'xdata', [], 'ydata', []);
            end
        end
        xlabel(Label_X)
        xlim([xlim_start,inf]);
        grid on
        set(gca, 'Fontname', 'Times', 'Fontsize', Fontsize);
    end
    subplot(3,1,1)
%     ylabel('Lat. (mm)');
%     legend('FF','FR'); set(legend,'Location','southeast');
%     set(legend, 'Position', [0.1385 0.9360 0.7717 0.0387], 'Orientation', 'horizontal');
    ylabel('Lat. (mm) / Yaw (mrad)');
    legend('Lat.','Yaw'); set(legend,'Location','southeast');
    set(legend, 'Position', [0.1385 0.9360 0.7717 0.0387], 'Orientation', 'horizontal');
    subplot(3,1,2)
%     ylabel('Yaw (mrad)');
%     ylabel('Con. Ang. (mrad)');
    set(gca,'ydir','reverse');
    ylabel('Lat. Con pos-Wheel (mm)');
    subplot(3,1,3)
    set(gca,'ydir','reverse');
    ylabel('Lat. Con pos-Rail (mm)');
else
    PlotHandles.Fig_3 = [];
end

%% Fig.4
if Fig_num_max >= 4
    figure(4); clf
    set(gcf,'Units','centimeters','Position',[0.3+weight_figure*(2+Fig_pos_times), height_screen, weight_figure, height_figure]);
    
    for i = 1:1:3
        subplot(3,1,i)
        if i<3
            for j0 = 1:1:4+1
                for j = 1:1:5
                    hold on
                    PlotHandles.Fig_4(i,j+5*(j0-1)) = plot(plot_initial_x, plot_initial_y, LineStyle_v2{j}, 'Color', Color_line{j0});
                    set(PlotHandles.Fig_4(i,j+5*(j0-1)), 'xdata', [], 'ydata', []);
                end
            end
        else
            for j = 1:1:8
                hold on
                PlotHandles.Fig_4(i,j) = plot(plot_initial_x, plot_initial_y, LineStyle_v2{1}, 'Color', Color_line{j});
                set(PlotHandles.Fig_4(i,j), 'xdata', [], 'ydata', []);
            end
        end
        xlabel(Label_X)
        xlim([xlim_start,inf]);
        grid on
        set(gca, 'Fontname', 'Times', 'Fontsize', Fontsize);
    end
    subplot(3,1,1)
    ylabel('Creepage-X');   set(gca,'ydir','reverse');
    subplot(3,1,2)
    ylabel('Creepage-Y');   set(gca,'ydir','reverse');
    subplot(3,1,3)
    ylabel('Acc-CB (m/s^2)');
    legend('Center', 'Front Bogie');
%     ylabel('Creepage-Spin (1/m)');
end

%% Fig.5
if Fig_num_max >= 5
    figure(5); clf
    set(gcf,'Units','centimeters','Position',[0.4+weight_figure*(3+Fig_pos_times), height_screen, weight_figure, height_figure]);
        
    for i = 1:1:2
        subplot(2,1,i)
        for j = 1:1:5
            hold on
            PlotHandles.Fig_5(i,j) = plot(plot_initial_x, plot_initial_y, 'Color', Color_line{j});
            set(PlotHandles.Fig_5(i,j), 'xdata', [], 'ydata', []);
        end
        for j = 1:1:5
            hold on
            PlotHandles.Fig_5(i,5+j) = plot(plot_initial_x, plot_initial_y, '--', 'Color', Color_line{j});
            set(PlotHandles.Fig_5(i,5+j), 'xdata', [], 'ydata', []);
        end
        xlabel('X (m)')
        xlim([-50, 100]);
        grid on
        set(gca, 'Fontname', 'Times', 'Fontsize', Fontsize);
    end
    subplot(2,1,1)
    ylabel('Z (mm)');   set(gca,'ydir','reverse');
    subplot(2,1,2)
    ylabel('Y (mm)');
    
    legend(InpPar.Exp_DummyRail);
    set(legend, 'Position', [0.1385 0.9360 0.7717 0.0387], 'Orientation', 'horizontal');
    
end

