function PlotPost(InpPar, ZP_Dyn, ZP_Con, xlcs, DynStatus_Rail, ZP_Dis, ZP_Vel, ZP_Acc, PlotHandles, Fig_num_max, Par_Vehicle, j1, j0, Choose_WS)

% Choose_WS = 'RR';

if strcmp(Choose_WS, 'FF')
    tt = 1;
    dx = 0;
elseif strcmp(Choose_WS, 'FR')
    tt = 2;
    dx = Par_Vehicle.Ll1*2*-1;
elseif strcmp(Choose_WS, 'RF')
    tt = 3;
    dx = Par_Vehicle.Ll2*2*-1;
elseif strcmp(Choose_WS, 'RR')
    tt = 4;
    dx = (Par_Vehicle.Ll1+Par_Vehicle.Ll2)*2*-1;
end


% if j1>48
%     x_start = max([j0, 47.5])+dx;
% else
%     x_start = j0;
% end
x_start = min([j1+dx, j0+dx]);
x_end = max([j1+dx, j0+dx]);

k_month = 1;
% k_month = 2;

%% Cal
figure(2)
N_Sub = 3;
for i2 = 1:1:InpPar.N_ConPatch
    Target_FZ = ZP_Dyn.FZ.(Choose_WS).(InpPar.Exp_DummyRail{i2});
    Target_FY = ZP_Dyn.FY.(Choose_WS).(InpPar.Exp_DummyRail{i2});
    Target_FX = ZP_Dyn.FX.(Choose_WS).(InpPar.Exp_DummyRail{i2});
    Target_TIrr = ZP_Dyn.Dis_TIrr.FF.(InpPar.Exp_DummyRail{i2});
    bools = Target_FZ(:,2)~=0;
%     Target_FZ = Target_FZ(bools,:);
%     Target_FZ = sortrows(Target_FZ,1);
%     set(PlotHandles.Fig_2(1,i2), 'xdata', Target_FZ(:,1), 'ydata', Target_FZ(:,2)/1000,  'LineWidth', 1);
    set(PlotHandles.Fig_2(1,i2), 'xdata', Target_FZ(bools,1), 'ydata', Target_FZ(bools,2)/1000,  'LineWidth', 1);
    set(PlotHandles.Fig_2(2,i2), 'xdata', Target_FY(bools,1), 'ydata', Target_FY(bools,2)/1000,  'LineWidth', 1);
    set(PlotHandles.Fig_2(3,i2), 'xdata', Target_FX(bools,1), 'ydata', Target_FX(bools,2)/1000,  'LineWidth', 1);
end
% Target_FZ_L = ZP_Dyn.FZ.(Choose_WS).L1;
% Target_FZ_R = [ZP_Dyn.FZ.(Choose_WS).R1(:,1), ZP_Dyn.FZ.(Choose_WS).R1(:,2)+ZP_Dyn.FZ.(Choose_WS).R2(:,2)+ZP_Dyn.FZ.(Choose_WS).R3(:,2)];
% set(PlotHandles.Fig_2(1,1), 'xdata', Target_FZ_L(4:end,1), 'ydata', Target_FZ_L(4:end,2)/1000,  'LineWidth', 1);
% set(PlotHandles.Fig_2(1,2), 'xdata', Target_FZ_R(4:end,1), 'ydata', Target_FZ_R(4:end,2)/1000,  'LineWidth', 1);

if isfield(InpPar, 'Test') && ~isempty(InpPar.Test)
    bools = true(length(InpPar .Test.Mileage_interp{k_month,1}),1);
    xx = InpPar.Test.Mileage_interp{k_month,1}(bools,1)+50;
    set(PlotHandles.Fig_2(1,7), 'xdata', xx, 'ydata', InpPar.Test.FZ.R{k_month,1}(bools,1), 'LineStyle', '-', 'Color', [0.65, 0.65, 0.65], 'LineWidth', 0.5);
    set(PlotHandles.Fig_2(2,7), 'xdata', xx, 'ydata', InpPar.Test.FY.R{k_month,1}(bools,1), 'LineStyle', '-', 'Color', [0.65, 0.65, 0.65], 'LineWidth', 0.5);
end
for i3 = 1:1:N_Sub
    subplot(N_Sub,1,i3)
     xlim([x_start, x_end]);
%      xlim([x_start, j1+dx]);
end

% ZP_Dyn, ZP_Con
if Fig_num_max>=3
    figure(3)
    pos = InpPar.N_track+InpPar.NM_FW*InpPar.Nw;
    if isfield(ZP_Dis, 'Correction')
        set(PlotHandles.Fig_3(1,1), 'xdata', ZP_Dyn.Yw(4:end,1)+dx, 'ydata', ZP_Dis.Correction(4:end,pos+5*(tt-1)+2)*1000);
        set(PlotHandles.Fig_3(1,2), 'xdata', ZP_Dyn.Yw(4:end,1)+dx, 'ydata', ZP_Dis.Correction(4:end,pos+5*(tt-1)+5)*1000);
    else
        set(PlotHandles.Fig_3(1,1), 'xdata', ZP_Dyn.Yw(4:end,1)+dx, 'ydata', ZP_Dis(4:end,InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(tt-1)+2)*1000);
        set(PlotHandles.Fig_3(1,2), 'xdata', ZP_Dyn.Yw(4:end,1)+dx, 'ydata', ZP_Dis(4:end,InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(tt-1)+5)*1000);
%         set(PlotHandles.Fig_3(1,3), 'xdata', ZP_Dyn.Yw(4:end,1)+dx-2.5, 'ydata', ZP_Dis(4:end,InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(tt-1)+7)*1000);
    end
    % ConPos-Rail
    i3_min = 2; i3_max = InpPar.N_ConPatch;
    for i3 = i3_min:1:i3_max
        T3 = InpPar.Exp_DummyRail{i3};
        for j = 1:1:length(ZP_Con.Rail_2.(T3)(tt,:))
            if ~isempty(ZP_Con.Rail_2.(T3){tt,j})
                bools = (ZP_Con.Rail_2.(T3){tt,j}(:,2)==0);
                temp_ZP_Con.Angle.(T3) = ZP_Con.Angle.(T3){tt,j};
                temp_ZP_Con.Angle.(T3)(bools,:) = NaN;
                temp_ZP_Con.Wheel_2.(T3) = ZP_Con.Wheel_2.(T3){tt,j};
                temp_ZP_Con.Wheel_2.(T3)(bools,:) = NaN;
                temp_ZP_Con.Rail_2.(T3) = ZP_Con.Rail_2.(T3){tt,j};
                temp_ZP_Con.Rail_2.(T3)(bools,:) = NaN;
%                 set(PlotHandles.Fig_3(2,5*(i3-i3_min)+j), 'xdata', temp_ZP_Con.Angle.(T3)(:,1), 'ydata', temp_ZP_Con.Angle.(T3)(:,2)*1000);
                set(PlotHandles.Fig_3(2,5*(i3-i3_min)+j), 'xdata', temp_ZP_Con.Wheel_2.(T3)(:,1), 'ydata', temp_ZP_Con.Wheel_2.(T3)(:,3)*1000-746.5);
                set(PlotHandles.Fig_3(3,5*(i3-i3_min)+j), 'xdata', temp_ZP_Con.Rail_2.(T3)(:,1), 'ydata', temp_ZP_Con.Rail_2.(T3)(:,2)*1000);
            end
        end
    end
end
for i3 = 1:1:N_Sub
    subplot(N_Sub,1,i3)
     xlim([x_start, x_end]);
%      xlim([x_start, j1+dx]);
end

if Fig_num_max>=4
    figure(4)
    for i3 = 1:1:InpPar.N_ConPatch
        T3 = InpPar.Exp_DummyRail{i3};
        for j = 1:1:length(ZP_Con.CX.(T3)(tt,:))
            if ~isempty(ZP_Con.CX.(T3){tt,j})
                bools = (ZP_Con.CX.(T3){tt,j}(:,2)==0);
                temp_CX = ZP_Con.CX.(T3){tt,j};
                temp_CY = ZP_Con.CY.(T3){tt,j};
                temp_CSpin = ZP_Con.CSpin.(T3){tt,j};
                temp_CX(bools,:) = NaN;
                temp_CY(bools,:) = NaN;
                temp_CSpin(bools,:) = NaN;
                set(PlotHandles.Fig_4(1,5*(i3-1)+j), 'xdata', temp_CX(:,1), 'ydata', temp_CX(:,2));
                set(PlotHandles.Fig_4(2,5*(i3-1)+j), 'xdata', temp_CY(:,1), 'ydata', temp_CY(:,2));
%                 set(PlotHandles.Fig_4(3,5*(i3-1)+j), 'xdata', temp_CSpin(:,1), 'ydata', temp_CSpin(:,2));
            end
        end
    end
    x = ZP_Dyn.Mileage(4:end,1)-(Par_Vehicle.Ll1+Par_Vehicle.Ll2);
%     pos = InpPar.N_track+InpPar.Nw*InpPar.NM_FW+32;   % Y
    pos = InpPar.N_track+InpPar.Nw*InpPar.NM_FW+31;   % Z
    y = ZP_Acc(4:end, pos);
    set(PlotHandles.Fig_4(3,1), 'xdata', x, 'ydata', y);

    Range_Mileage = [20, 200];
%     pos_Target = [8.75, 1, 0.52];   % HSR
%     Acc_CB_FrontBogie = AccCB_Cal(Range_Mileage, ZP_Dyn.Mileage, ZP_Dis, ZP_Vel, ZP_Acc, InpPar.N_track, InpPar.NM_FW, InpPar.Nw, pos_Target);
%     set(PlotHandles.Fig_4(3,2), 'xdata', Acc_CB_FrontBogie(:,1), 'ydata', Acc_CB_FrontBogie(:,3));

%     pos_Target = [8.9*(-1)^sign(tt>2), 1, 0.67];   % HSR-CR400AF
    pos_Target = [8.75*(-1)^sign(tt>2), 1, 0.52];   % HSR
    Tar_Mileage = ZP_Dyn.Mileage(:,1);
    Tar_Mileage(4:end,1) = Tar_Mileage(4:end,1)+dx;
    Acc_CB_RearBogie = AccCB_Cal(Range_Mileage, Tar_Mileage, ZP_Dis, ZP_Vel, ZP_Acc, InpPar.N_track, InpPar.NM_FW, InpPar.Nw, pos_Target);
    set(PlotHandles.Fig_4(3,2), 'xdata', Acc_CB_RearBogie(:,1), 'ydata', Acc_CB_RearBogie(:,4));
%     set(PlotHandles.Fig_4(3,2), 'xdata', Acc_CB_RearBogie(:,1), 'ydata', Acc_CB_RearBogie(:,3));
%     ylim([-1.5 1.5]);

    if isfield(InpPar.Test, 'CB_Acc_Y')
        set(PlotHandles.Fig_4(3,3), 'xdata', InpPar.Test.CB_Acc_Y(:,1), 'ydata', InpPar.Test.CB_Acc_Y(:,2)*9.81, 'LineStyle', '-', 'Color', [0.65, 0.65, 0.65], 'LineWidth', 0.5);
        legend('Center', 'Front Bogie', 'Test')
        set(legend, 'orientation', 'horizontal', 'Position', [0.3113 0.3206 0.6079 0.0331]);
    end

end
for i3 = 1:1:N_Sub
    subplot(N_Sub,1,i3)
     xlim([x_start, x_end]);
%      xlim([x_start, j1+dx]);
end

if Fig_num_max>=5
    figure(5)
    Range_kk = [1, 2, 3, 4];
    for kk = Range_kk
        OutputTarget = DynStatus_Rail.([InpPar.Type_Rail{kk},'_Dis']);
        PosTarget = InpPar.Pos_Node.(InpPar.Type_Rail{kk});
        PosTarget(:,2) = PosTarget(:,2)-50;
        set(PlotHandles.Fig_5(1,kk), 'xdata', PosTarget(:,2), 'ydata', OutputTarget(:,3)*1000);     % UZ
        set(PlotHandles.Fig_5(2,kk), 'xdata', PosTarget(:,2), 'ydata', OutputTarget(:,2)*1000);     % UY
    end
end

