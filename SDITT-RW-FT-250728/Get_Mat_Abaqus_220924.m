%% 从有限元ANSYS中提取钢轨梁模型的质量、刚度矩阵
function [ModeFreq, ModeShape_Mapping, ModeShape, Pos_Node, N_Node, DOF_Node, Type_SpaceIron] = Get_Mat_Abaqus_220924(InpPar)

% global InpPar.Type_DOF InpPar.Type_Rail_All InpPar.Type_Baseplate InpPar.Vlc InpPar.Type_Rail Type_SpaceIron

clear ModeFreq ModeShape_Mapping ModeShape Pos_Node N_Node DOF_Node

load('RailPro.mat')
load('BaseplatePro.mat')
load('SpacerIronPro.mat')

%% 1. Load data: Data, Output_MS
Node_sum = 3751+3222+28;
Mode_sum = 6236;

% UX, UY, UZ
for kk = 1:1:3
    Tdof = InpPar.Type_DOF{kk};
    fid = fopen(['AllRailsPlates_S1_', Tdof, '.rpt']);
    Data.(Tdof).Title = textscan(fid, ['%s', repmat('%s%f',1,Node_sum)], 1, 'HeaderLines', 2, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
    for i = 1:1:Node_sum
        Data.(Tdof).NodeNum(i,1) = Data.(Tdof).Title{1, 1+2*i};
    end
    Data.(Tdof).ModeShape = textscan(fid, repmat('%f',1,Node_sum+1), Mode_sum, 'HeaderLines', 1, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
    fclose(fid);
end

% ROTX, ROTY, ROTZ
% for kk = 4:1:length(InpPar.Type_DOF)
%     Tdof = InpPar.Type_DOF{kk};
%     fid = fopen(['AllRailsPlates_S1_', Tdof, '.rpt']);
%     Data.(Tdof).Title = textscan(fid, ['%s', repmat('%s%s%f',1,Node_sum)], 1, 'HeaderLines', 2, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
%     for i = 1:1:Node_sum
%         Data.(Tdof).NodeNum(i,1) = Data.(Tdof).Title{1, 1+3*i};
%     end
%     Data.(Tdof).ModeShape = textscan(fid, repmat('%f',1,Node_sum+1), Mode_sum, 'HeaderLines', 1, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
%     fclose(fid);
% end

% max(abs(Data.UX.NodeNum-Data.ROTZ.NodeNum))

% Output_MS
Output_MS.Mapping = Data.UX.NodeNum;
for kk = 1:1:length(InpPar.Type_DOF)
    Tdof = InpPar.Type_DOF{kk};
    Output_MS.(Tdof) = zeros(Node_sum, Mode_sum);      % N*NM
    for i = 1:1:Node_sum
        Output_MS.(Tdof)(i,:) = Data.(Tdof).ModeShape{1,i+1}';     % 1*NM
    end
end

%% 2. ModeFreq
ModeFreq.FT_All(:,1) = Data.UX.ModeShape{1,1};
fid = fopen('EigenFrequency_S1.txt');
Data.Freq = textscan(fid, repmat('%f',1,6), Mode_sum, 'HeaderLines', 4, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
fclose(fid);
ModeFreq.FT_All(:,2) = Data.Freq{1,4};

%% 3. Pos_Node, N_Node, DOF_Node, Pos_Yaw_RailBeam
DOF_Node_Rail = 4;
DOF_Node_Baseplate = 2;
DOF_Node_SpaceIron = 6;
DOF_Node_Slab = 3;

clear Pos_Node N_Node DOF_Node
% RailPro
N_Node.Rail = 0;
for i1 = 1:1:length(InpPar.Type_Rail_All)
    T1 = InpPar.Type_Rail_All{i1};
    Pos_Node.(T1) = RailPro.(T1)(:,1:4);
    Pos_Node.(T1)(:,2) = Pos_Node.(T1)(:,2)+50;
    Pos_Node.([T1, '_Length']) = sqrt(diff(Pos_Node.(T1)(:,2)).^2+diff(Pos_Node.(T1)(:,3)).^2);
    N_Node.(T1) = size(Pos_Node.(T1),1);
    N_Node.Rail = N_Node.Rail+N_Node.(T1);
    DOF_Node.(T1) = N_Node.(T1)*DOF_Node_Rail;
end

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

N_Node.All = N_Node.Rail+N_Node.Baseplate;
Pos_Node.Sleeper =BaseplatePro.Mileage_Sleeper_Thr;

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
% ModeShape_Mapping, ModeShape, Pos_Node
DOF_Rail = [2, 3, 5, 6];
for i1 = 1:1:length(InpPar.Type_Rail_All)
    T1 = InpPar.Type_Rail_All{i1};
    for i2 = 1:1:N_Node.(T1)
        tt = DOF_Node_Rail*(i2-1);
        ModeShape_Mapping.(T1)(tt+1:tt+DOF_Node_Rail,1) = repmat(Pos_Node.(T1)(i2,1), DOF_Node_Rail, 1);
        ModeShape_Mapping.(T1)(tt+1:tt+DOF_Node_Rail,2) = DOF_Rail;
        pos = find(Output_MS.Mapping==Pos_Node.(T1)(i2,1));
        for j1 = 1:1:DOF_Node_Rail
            ModeShape.(T1)(tt+j1,:) = Output_MS.(InpPar.Type_DOF{DOF_Rail(j1)})(pos,:);
        end
    end
end

DOF_Baseplate = [3, 4];
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
            tt = DOF_Node_Baseplate*(i3-1);
            ModeShape_Mapping.(T1).(T2)(tt+1:tt+DOF_Node_Baseplate,1) = repmat(Pos_Node.(T1)(t_2,1), DOF_Node_Baseplate, 1);
            ModeShape_Mapping.(T1).(T2)(tt+1:tt+DOF_Node_Baseplate,2) = DOF_Baseplate;
            pos = find(Output_MS.Mapping==Pos_Node.(T1)(t_2,1));
            for j1 = 1:1:DOF_Node_Baseplate
                ModeShape.(T1).(T2)(tt+j1,:) = Output_MS.(InpPar.Type_DOF{DOF_Baseplate(j1)})(pos,:);
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
            else
                p2 = DOF_SpaceIron;
            end
            p1 = length(p2);
            ModeShape_Mapping.(T1).(T2)(tt+1:tt+p1,1) = repmat(Pos_Node.(T1).(T2)(i3,1), p1, 1);
            ModeShape_Mapping.(T1).(T2)(tt+1:tt+p1,2) = p2;
            pos = find(Output_MS.Mapping==Pos_Node.(T1).(T2)(i3,1));
            for j1 = 1:1:p1
                ModeShape.(T1).(T2)(tt+j1,:) = Output_MS.(InpPar.Type_DOF{p2(j1)})(pos,:);
            end
            tt = tt+p1;
        end        
    end
    
end



