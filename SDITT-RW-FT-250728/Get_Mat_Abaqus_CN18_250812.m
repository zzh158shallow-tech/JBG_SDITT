%% 从有限元ANSYS中提取钢轨梁模型的质量、刚度矩阵
function [ModeFreq, ModeShape_Mapping, ModeShape, Pos_Node, N_Node, DOF_Node, Type_SpaceIron] = Get_Mat_Abaqus_CN18_250812(InpPar)

clear ModeFreq ModeShape_Mapping ModeShape Pos_Node N_Node DOF_Node Node_sum

% RailPro BaseplatePro SpacerIronPro
load('RailPro.mat')
load('BaseplatePro.mat')
load('SpacerIronPro.mat')

%% 1. Load data: Data, Output_MS
% Node_sum = 3617+5682+11;
Node_sum.Rail = 3617;
Node_sum.Baseplate = 5682;
Node_sum.SpacerIron = 11;
Mode_sum = 6078;

% Rails
% 1021+411+411+1021+31+354+31+338-2+1 = 3617
% Baseplates
% 1033*2+168*4+224*2+368+532*4 = 5682
% SpacerIron
% (10-6)+(3-2)+(9-6)+(9-6) = 11
% 10+3+9+9 = 31

% Data, Output_MS
Type_T0 = {'Rail', 'Baseplate', 'SpacerIron'};
Ragee_kk_All = {[2,3,5,6]; [3,4]; (1:1:6)};
for i0 = 1:1:3
    T0 = Type_T0{i0};
    Range_kk = Ragee_kk_All{i0};

    % UX, UY, UZ, ROTX, ROTY, ROTZ
    for kk = Range_kk
        Tdof = InpPar.Type_DOF{kk};
        if i0==1
            fid = fopen(['AllRails_CN18_T4_', Tdof, '.rpt']);
        elseif i0==2
            fid = fopen(['AllBaseplates_CN18_T4_', Tdof, '.rpt']);
        elseif i0==3
            fid = fopen(['AllSpacerIron_CN18_T4_', Tdof, '.rpt']);
        end
        if kk<=3
            Data.(T0).(Tdof).Title = textscan(fid, ['%s', repmat('%s%f',1,Node_sum.(T0))], 1, 'HeaderLines', 2, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
        else
            Data.(T0).(Tdof).Title = textscan(fid, ['%s', repmat('%s%s%f',1,Node_sum.(T0))], 1, 'HeaderLines', 2, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
        end
        for i = 1:1:Node_sum.(T0)
            if kk<=3
                Data.(T0).(Tdof).NodeNum(i,1) = Data.(T0).(Tdof).Title{1, 1+2*i};
            else
                Data.(T0).(Tdof).NodeNum(i,1) = Data.(T0).(Tdof).Title{1, 1+3*i};
            end
        end
        Data.(T0).(Tdof).ModeShape = textscan(fid, repmat('%f',1,Node_sum.(T0)+1), Mode_sum, 'HeaderLines', 1, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
        fclose(fid);
    end
    
    % Output_MS
    DofDir = InpPar.Type_DOF{Range_kk(1)};
    Output_MS.(T0).Mapping = Data.(T0).(DofDir).NodeNum;
    for kk = Range_kk
        Tdof = InpPar.Type_DOF{kk};
        Output_MS.(T0).(Tdof) = zeros(Node_sum.(T0), Mode_sum);      % N*NM
        for i = 1:1:Node_sum.(T0)
            Output_MS.(T0).(Tdof)(i,:) = Data.(T0).(Tdof).ModeShape{1,i+1}';     % 1*NM
        end
    end

end

% ROTX, ROTY, ROTZ
% for kk = 5:1:length(InpPar.Type_DOF)
%     Tdof = InpPar.Type_DOF{kk};
%     fid = fopen(['AllRails_CN18_T4_', Tdof, '.rpt']);
%     Data.Rail.(Tdof).Title = textscan(fid, ['%s', repmat('%s%s%f',1,Node_sum)], 1, 'HeaderLines', 2, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
%     for i = 1:1:Node_sum
%         Data.Rail.(Tdof).NodeNum(i,1) = Data.Rail.(Tdof).Title{1, 1+3*i};
%     end
%     Data.Rail.(Tdof).ModeShape = textscan(fid, repmat('%f',1,Node_sum+1), Mode_sum, 'HeaderLines', 1, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
%     fclose(fid);
% end


%% 2. ModeFreq
ModeFreq.FT_All(:,1) = Data.Rail.UY.ModeShape{1,1};
fid = fopen('EigenFrequency_T4_250812.txt');
Data.Freq = textscan(fid, repmat('%f',1,6), Mode_sum, 'HeaderLines', 4, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
fclose(fid);
ModeFreq.FT_All(:,2) = Data.Freq{1,4};

%% 3. Pos_Node, N_Node, DOF_Node, Pos_Yaw_RailBeam
DOF_Node_Rail = 4;
DOF_Node_Baseplate = 2;
DOF_Node_SpaceIron = 6;
% DOF_Node_Slab = 3;

clear Pos_Node N_Node DOF_Node
% RailPro
N_Node.Rail = 0;
for i1 = 1:1:length(InpPar.Type_Rail_All)
    T1 = InpPar.Type_Rail_All{i1};
%     if i1==6 || i1==7
    if i1==6
        i_min = 2;
    else
        i_min = 1;
    end
    Pos_Node.(T1) = RailPro.(T1)(i_min:end,1:4);
    Pos_Node.(T1)(:,2) = Pos_Node.(T1)(:,2)+50;
    Pos_Node.([T1, '_Length']) = sqrt(diff(Pos_Node.(T1)(:,2)).^2+diff(Pos_Node.(T1)(:,3)).^2);
    N_Node.(T1) = size(Pos_Node.(T1),1);
    N_Node.Rail = N_Node.Rail+N_Node.(T1);
    DOF_Node.(T1) = N_Node.(T1)*DOF_Node_Rail;
end
% 合并 cx 与 zghjg
Pos_Node.cx = [Pos_Node.cx; Pos_Node.zghjg];
Pos_Node.cx_Length = sqrt(diff(Pos_Node.cx(:,2)).^2+diff(Pos_Node.cx(:,3)).^2);
Pos_Node = rmfield(Pos_Node, 'zghjg');
Pos_Node = rmfield(Pos_Node, 'zghjg_Length');

N_Node.cx = N_Node.cx + N_Node.zghjg;
DOF_Node.cx = DOF_Node.cx + DOF_Node.zghjg;
N_Node = rmfield(N_Node, 'zghjg');
DOF_Node = rmfield(DOF_Node, 'zghjg');


% Baseplate
N_Node.Baseplate = 0;
for i1 = 1:1:length(InpPar.Type_Baseplate)
    T1 = InpPar.Type_Baseplate{i1};
    Pos_Node.(T1) = BaseplatePro.ND.(T1)(:,1:4);
    Pos_Node.([T1, '_cell']) = BaseplatePro.ND_cell.(T1);
    Pos_Node.(T1)(:,2) = Pos_Node.(T1)(:,2)+50;
    for i2 = 1:1:size(Pos_Node.([T1, '_cell']),1)
        Pos_Node.([T1, '_cell']){i2, 2} = Pos_Node.([T1, '_cell']){i2, 2}+50;
    end
    Pos_Node.([T1, '_Length']) = sqrt(diff(Pos_Node.(T1)(:,2)).^2+diff(Pos_Node.(T1)(:,3)).^2);
    N_Node.(T1) = size(Pos_Node.(T1),1);
    N_Node.Baseplate = N_Node.Baseplate+N_Node.(T1);
    DOF_Node.(T1) = N_Node.(T1)*DOF_Node_Baseplate;
end

% N_Node.All = N_Node.Rail+N_Node.Baseplate;

% Pos_Node.Sleeper =BaseplatePro.Mileage_Sleeper_Thr;
Pos_Node.Sleeper = [cell2mat(Pos_Node.Switch_L_cell(:,1:2)); cell2mat(Pos_Node.Closure_zjbg_cell(:,1:2)); 
                                   cell2mat(Pos_Node.Crossing_L_cell(:,1:2)); cell2mat(Pos_Node.Plain_Thr_L_cell(:,1:2))];

for i1 = 1:1:length(InpPar.Type_Baseplate)
    T1 = InpPar.Type_Baseplate{i1};
    i2_max = size(Pos_Node.([T1, '_cell']),1);
    t_2 = 1;
    for i2 = 1:1:i2_max
        i3_max = cell2mat(Pos_Node.([T1, '_cell'])(i2,4));
        num_baseplate = cell2mat(Pos_Node.([T1, '_cell'])(i2,1));
        if num_baseplate<0
            T2 = ['B_m', num2str(abs(num_baseplate))];
        else
            T2 = ['B_', num2str(num_baseplate)];
        end
        for i3 = 1:1:i3_max
            Pos_Node.([T1, '_Div']).(T2)(i3,:) = Pos_Node.(T1)(t_2,:);
            t_2 = t_2+1;
        end
        Pos_Node.([T1, '_Div_Length']).(T2) = sqrt( diff(Pos_Node.([T1, '_Div']).(T2)(:,2)).^2+diff(Pos_Node.([T1, '_Div']).(T2)(:,3)).^2 );
    end
end

% SpacerIronPro
for i1 = 1:1:size(SpacerIronPro,1)
    p1 = SpacerIronPro{i1,1}(1);
    p2 = SpacerIronPro{i1,1}(2);
    T1 = ['SI_', InpPar.Type_Rail_All{p1}, '_', InpPar.Type_Rail_All{p2}];
    i2_max = size(SpacerIronPro{i1,5},1);
    Type_SpaceIron{i1,1} = T1;
    
    for i2 = 1:1:i2_max
        T2 = ['S_', num2str(i2)];
        % 编号、XYZ坐标
        Pos_Node.(T1).(T2)(:,1:4) = SpacerIronPro{i1,4}{i2,1};
        % Height, X_Length
        % {k,3}: 间隔铁质量、横向长度、横截面高度、横截面长度
        N_Node.(T1).(T2) = size(SpacerIronPro{i1,4}{i2,1},1);
        DOF_Node.(T1).(T2) = DOF_Node_SpaceIron * (N_Node.(T1).(T2)-2);
        Pos_Node.(T1).(T2)(:,5:6) = repmat(SpacerIronPro{i1,3}(i2,3:4), N_Node.(T1).(T2),1);
    end
end


%% 4. ModeShape_Mapping, ModeShape
% ModeShape_Mapping, ModeShape, Pos_Node, N_Node
DOF_Rail = [2, 3, 5, 6];
% for i1 = 1:1:length(InpPar.Type_Rail_All)
for i1 = [1:1:5, 7:1:8]
    T1 = InpPar.Type_Rail_All{i1};
    for i2 = 1:1:N_Node.(T1)
        tt = DOF_Node_Rail*(i2-1);
        ModeShape_Mapping.(T1)(tt+1:tt+DOF_Node_Rail,1) = repmat(Pos_Node.(T1)(i2,1), DOF_Node_Rail, 1);
        ModeShape_Mapping.(T1)(tt+1:tt+DOF_Node_Rail,2) = DOF_Rail;
        pos = Output_MS.Rail.Mapping==Pos_Node.(T1)(i2,1);
        for j1 = 1:1:DOF_Node_Rail
            T_DOF = InpPar.Type_DOF{DOF_Rail(j1)};
            ModeShape.(T1)(tt+j1,:) = Output_MS.Rail.(T_DOF)(pos,:);
        end
    end
end

DOF_Baseplate = [3, 4];
for i1 = 1:1:length(InpPar.Type_Baseplate)
    T1 = InpPar.Type_Baseplate{i1};
    i2_max = size(Pos_Node.([T1, '_cell']),1);
    t_2 = 1;
    for i2 = 1:1:i2_max
        i3_max = cell2mat(Pos_Node .([T1, '_cell'])(i2,4));
        num_baseplate = cell2mat(Pos_Node.([T1, '_cell'])(i2,1));
        if num_baseplate<0
            T2 = ['B_m', num2str(abs(num_baseplate))];
        else
            T2 = ['B_', num2str(num_baseplate)];
        end
        for i3 = 1:1:i3_max
            tt = DOF_Node_Baseplate*(i3-1);
            ModeShape_Mapping.(T1).(T2)(tt+1:tt+DOF_Node_Baseplate,1) = repmat(Pos_Node.(T1)(t_2,1), DOF_Node_Baseplate, 1);
            ModeShape_Mapping.(T1).(T2)(tt+1:tt+DOF_Node_Baseplate,2) = DOF_Baseplate;
            pos = Output_MS.Baseplate.Mapping==Pos_Node.(T1)(t_2,1);
            for j1 = 1:1:DOF_Node_Baseplate
                T_DOF = InpPar.Type_DOF{DOF_Baseplate(j1)};
                ModeShape.(T1).(T2)(tt+j1,:) = Output_MS.Baseplate.(T_DOF)(pos,:);
            end            
            t_2 = t_2+1;
        end
    end
end

DOF_SpaceIron = [1, 2, 3, 4, 5, 6];
for i1 = 1:1:size(SpacerIronPro,1)
    T1 = Type_SpaceIron{i1};
    i2_max = size(SpacerIronPro{i1,5},1);    
    for i2 = 1:1:i2_max
        T2 = ['S_', num2str(i2)];
        tt = 0;
        for i3 = 1:1:N_Node.(T1).(T2)
            if i3==1 || i3==N_Node.(T1).(T2)
                p2 = DOF_Rail;
                T3 = 'Rail';
            else
                p2 = DOF_SpaceIron;
                T3 = 'SpacerIron';
            end
            p1 = length(p2);
            ModeShape_Mapping.(T1).(T2)(tt+1:tt+p1,1) = repmat(Pos_Node.(T1).(T2)(i3,1), p1, 1);
            ModeShape_Mapping.(T1).(T2)(tt+1:tt+p1,2) = p2;
            pos = Output_MS.(T3).Mapping==Pos_Node.(T1).(T2)(i3,1);
            for j1 = 1:1:p1
                T_DOF = InpPar.Type_DOF{p2(j1)};
                ModeShape.(T1).(T2)(tt+j1,:) = Output_MS.(T3).(T_DOF)(pos,:);
            end
            tt = tt+p1;
        end        
    end    
end

