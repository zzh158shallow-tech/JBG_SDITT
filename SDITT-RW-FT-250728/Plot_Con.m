%% ½Ó´¥×´Ì¬»æÍ¼

function Plot_Con(PlotHandles_Fig_1, Con_FF, R0)

Type_Side = {'L', 'R'};

% R0 = Par_Vehicle.R0;
figure(1)
for i1=1:1:length(Type_Side)
    T = (Type_Side{i1});    
    set(PlotHandles_Fig_1(i1,1), 'xdata', Con_FF.traceline_w.(T)(:,2), 'ydata', Con_FF.traceline_w.(T)(:,3)-R0-0.01);
    set(PlotHandles_Fig_1(i1,2), 'xdata', Con_FF.profile_r.(T)(:,1), 'ydata', Con_FF.profile_r.(T)(:,2)-0.6);
    for j = 1:1:4
        if j <= size(Con_FF.Normal_Force.(T),1)
            set(PlotHandles_Fig_1(i1,2*j+1), 'xdata', Con_FF.Con_wheel_1.(T)(j,2), 'ydata', Con_FF.Con_wheel_1.(T)(j,3)-R0-0.01);
            set(PlotHandles_Fig_1(i1,2*j+2), 'xdata', Con_FF.Con_rail_1.(T)(j,1), 'ydata', Con_FF.Con_rail_1.(T)(j,2)-0.6);
        else
            set(PlotHandles_Fig_1(i1,2*j+1), 'xdata', [], 'ydata', []);
            set(PlotHandles_Fig_1(i1,2*j+2), 'xdata', [], 'ydata', []);
        end
    end
end
