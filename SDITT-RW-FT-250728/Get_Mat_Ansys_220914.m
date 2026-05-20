%% 从有限元ANSYS中提取钢轨梁模型的质量、刚度矩阵
%% 各节点考虑6自由度

function [Mass_Rail, Stiff_Rail, Damp_Rail, CompDiff, ...
                 ModeFreq, ModeShape, Pos_Node, N_Node, NM_Rail, DOF_Rail, DOF_Rail_start] = ...
                Get_Mat_Ansys_220914

Choose_Check = 0;

global Par_Track

%% 1. Mass_Rail, Stiff_Rail, Damp_Rail, DOF_Rail
[Mass_Rail.Free, Mass_Rail.Free_Mapping, DOF_Rail.Free] = Load_HBFILE('HBFILE_PT-220709_Free_MASS');
[Stiff_Rail.Free, Stiff_Rail.Free_Mapping, DOF_Rail.Free] = Load_HBFILE('HBFILE_PT-220709_Free_STIFF');
[Mass_Rail.Cons, Mass_Rail.Cons_Mapping, DOF_Rail.Cons] = Load_HBFILE('HBFILE_PT-220709_Cons_6DOF_MASS');
[Stiff_Rail.Cons, Stiff_Rail.Cons_Mapping, DOF_Rail.Cons] = Load_HBFILE('HBFILE_PT-220709_Cons_6DOF_STIFF');

DOF_Node_Rail = 6;
% DOF_Node_Rail = 4;
% DOF_Node_Baseplate = 2;

%% 2. Pos_Node, N_Node, DOF_Rail, DOF_Rail_start
fid = fopen('NodePos_PT-220709.txt');
format = '%f %f %f %f %f %f %f';
pos_Node_load = textscan(fid, format);
fclose(fid);
pos_Node_ori = [pos_Node_load{1,1} pos_Node_load{1,2} pos_Node_load{1,3} pos_Node_load{1,4} pos_Node_load{1,5} pos_Node_load{1,6} pos_Node_load{1,7}];

Pos_Node.zjbg = pos_Node_ori(1:1441,:);
Pos_Node.zjbg = sortrows(Pos_Node.zjbg,2);
% Pos_Node.qjg_cgyg = pos_Node_ori(473+1:473+235,:);
% Pos_Node.qjg_cgyg = sortrows(Pos_Node.qjg_cgyg,2);
% Pos_Node.Baseplate_L_Switch = pos_Node_ori(473+235+1:473+235+703,:);
% Pos_Node.Baseplate_L_Switch = sortrows(Pos_Node.Baseplate_L_Switch,2);
% Pos_Node.zjg_zgyg = pos_Node_ori(473+235+703+1:473+235+703+231,:);
% Pos_Node.zjg_zgyg = sortrows(Pos_Node.zjg_zgyg,2);
Pos_Node.qjbg = pos_Node_ori(1441+1:end,:);
Pos_Node.qjbg = sortrows(Pos_Node.qjbg,2);
% Pos_Node.Baseplate_R_Switch = pos_Node_ori(473*2+235+703+231+1:473*2+235+703*2+231,:);
% Pos_Node.Baseplate_R_Switch = sortrows(Pos_Node.Baseplate_R_Switch,2);

% Beam_189
% bools = 1:2:length(Pos_Node.zjbg);
% Pos_Node.zjbg_ShapeFun = Pos_Node.zjbg(bools,:);
% Pos_Node.qjbg_ShapeFun = Pos_Node.qjbg(bools,:);
% bools = 1:2:length(Pos_Node.qjg_cgyg);
% Pos_Node.qjg_cgyg_ShapeFun = Pos_Node.qjg_cgyg(bools,:);
% bools = 1:2:length(Pos_Node.zjg_zgyg);
% Pos_Node.zjg_zgyg_ShapeFun = Pos_Node.zjg_zgyg(bools,:);

Pos_Node.zjbg_Length = sqrt(diff(Pos_Node.zjbg(:,2)).^2+diff(Pos_Node.zjbg(:,3)).^2);
% Pos_Node.qjg_cgyg_Length = sqrt(diff(Pos_Node.qjg_cgyg(:,2)).^2+diff(Pos_Node.qjg_cgyg(:,3)).^2);
% Pos_Node.zjg_zgyg_Length = sqrt(diff(Pos_Node.zjg_zgyg(:,2)).^2+diff(Pos_Node.zjg_zgyg(:,3)).^2);
Pos_Node.qjbg_Length = sqrt(diff(Pos_Node.qjbg(:,2)).^2+diff(Pos_Node.qjbg(:,3)).^2);

% Beam_189
% Pos_Node.zjbg_ShapeFun_Length = sqrt(diff(Pos_Node.zjbg_ShapeFun(:,2)).^2+diff(Pos_Node.zjbg_ShapeFun(:,3)).^2);
% Pos_Node.qjg_cgyg_ShapeFun_Length = sqrt(diff(Pos_Node.qjg_cgyg_ShapeFun(:,2)).^2+diff(Pos_Node.qjg_cgyg_ShapeFun(:,3)).^2);
% Pos_Node.zjg_zgyg_ShapeFun_Length = sqrt(diff(Pos_Node.zjg_zgyg_ShapeFun(:,2)).^2+diff(Pos_Node.zjg_zgyg_ShapeFun(:,3)).^2);
% Pos_Node.qjbg_ShapeFun_Length = sqrt(diff(Pos_Node.qjbg_ShapeFun(:,2)).^2+diff(Pos_Node.qjbg_ShapeFun(:,3)).^2);

N_Node.zjbg = size(Pos_Node.zjbg,1);
% N_Node.qjg_cgyg = size(Pos_Node.qjg_cgyg,1);
% N_Node.zjg_zgyg = size(Pos_Node.zjg_zgyg,1);
N_Node.qjbg = size(Pos_Node.qjbg,1);
% N_Node.Baseplate_L_Switch = size(Pos_Node.Baseplate_L_Switch,1);
% N_Node.Baseplate_R_Switch = size(Pos_Node.Baseplate_R_Switch,1);
% N_Node.zjbg_ShapeFun = size(Pos_Node.zjbg_ShapeFun,1);
% N_Node.qjg_cgyg_ShapeFun = size(Pos_Node.qjg_cgyg_ShapeFun,1);
% N_Node.zjg_zgyg_ShapeFun = size(Pos_Node.zjg_zgyg_ShapeFun,1);
% N_Node.qjbg_ShapeFun = size(Pos_Node.qjbg_ShapeFun,1);

N_Node.Rail = N_Node.zjbg+N_Node.qjbg;
N_Node.Baseplate = 0;
% N_Node.Rail = N_Node.zjbg+N_Node.qjg_cgyg+N_Node.zjg_zgyg+N_Node.qjbg;
% N_Node.Baseplate = N_Node.Baseplate_L_Switch+N_Node.Baseplate_R_Switch;
N_Node.All = N_Node.Rail+N_Node.Baseplate;
N_Node.Sort = [N_Node.zjbg; N_Node.qjbg];
% N_Node.Sort = [N_Node.zjbg; N_Node.qjg_cgyg; N_Node.zjg_zgyg; N_Node.qjbg; N_Node.Baseplate_L_Switch; N_Node.Baseplate_R_Switch];

bools = 1:8:N_Node.zjbg;
Pos_Node.Sleeper = [(0:1:180)' Pos_Node.zjbg(bools,2)];
% bools = 1:4:N_Node.zjbg;
% Pos_Node.Sleeper = [(-60:1:58)' Pos_Node.zjbg(bools,2)];

DOF_Rail.zjbg = N_Node.zjbg*DOF_Node_Rail;
% DOF_Rail.qjg_cgyg = N_Node.qjg_cgyg*DOF_Node_Rail;
% DOF_Rail.zjg_zgyg = N_Node.zjg_zgyg*DOF_Node_Rail;
DOF_Rail.qjbg = N_Node.qjbg*DOF_Node_Rail;
% DOF_Rail.Baseplate_L_Switch = N_Node.Baseplate_L_Switch*DOF_Node_Baseplate;
% DOF_Rail.Baseplate_R_Switch = N_Node.Baseplate_R_Switch*DOF_Node_Baseplate;
% DOF_Rail.Sort = [DOF_Rail.zjbg; DOF_Rail.qjg_cgyg; DOF_Rail.zjg_zgyg; DOF_Rail.qjbg; DOF_Rail.Baseplate_L_Switch; DOF_Rail.Baseplate_R_Switch];
DOF_Rail.Sort = [DOF_Rail.zjbg; DOF_Rail.qjbg];

% DOF_Rail_start(:,1) = [0;
%                                        DOF_Rail.zjbg;
%                                        DOF_Rail.zjbg + DOF_Rail.qjg_cgyg;
%                                        DOF_Rail.zjbg + DOF_Rail.qjg_cgyg + DOF_Rail.zjg_zgyg;
%                                        DOF_Rail.zjbg + DOF_Rail.qjg_cgyg + DOF_Rail.zjg_zgyg + DOF_Rail.qjbg];
% DOF_Rail_start(1:3,2) = [0;
%                                             DOF_Rail.Baseplate_L_Switch;
%                                             DOF_Rail.Baseplate_L_Switch + DOF_Rail.Baseplate_R_Switch];
% DOF_Rail_start(1:2,3) = [0;
%                                            DOF_Rail.Cons];
DOF_Rail_start(:,1) = [0; DOF_Rail.zjbg; DOF_Rail.zjbg+DOF_Rail.qjbg];
DOF_Rail_start(:,2) = zeros(3,1);
DOF_Rail_start(:,3) =DOF_Rail_start(:,1);

%% 3. Transfer the Constrainted Matrices based on the Sepcified DOF Order
%%% Pos_Node, N_Node, DOF_Rail, Mapping_temp
clear Mapping_temp
Mapping_temp = cell(DOF_Rail.Cons,3);
Mapping_temp(:,1) = num2cell(1:1:DOF_Rail.Cons);
for i = 1:1:N_Node.All
%     if i <= sum(N_Node.Sort(1:1,1))
%         Mapping_temp(DOF_Node_Rail*(i-1)+1:DOF_Node_Rail*i,2) = num2cell(Pos_Node.zjbg(i,1));
%         Mapping_temp(DOF_Node_Rail*(i-1)+1:DOF_Node_Rail*i,3) = {'UY'; 'UZ'; 'ROTY'; 'ROTZ'};
%     elseif i <= sum(N_Node.Sort(1:2,1))
%         Mapping_temp(DOF_Node_Rail*(i-1)+1:DOF_Node_Rail*i,2) = num2cell(Pos_Node.qjg_cgyg(i-sum(N_Node.Sort(1:1,1)),1));
%         Mapping_temp(DOF_Node_Rail*(i-1)+1:DOF_Node_Rail*i,3) = {'UY'; 'UZ'; 'ROTY'; 'ROTZ'};
%     elseif i <= sum(N_Node.Sort(1:3,1))
%         Mapping_temp(DOF_Node_Rail*(i-1)+1:DOF_Node_Rail*i,2) = num2cell(Pos_Node.zjg_zgyg(i-sum(N_Node.Sort(1:2,1)),1));
%         Mapping_temp(DOF_Node_Rail*(i-1)+1:DOF_Node_Rail*i,3) = {'UY'; 'UZ'; 'ROTY'; 'ROTZ'};
%     elseif i <= sum(N_Node.Sort(1:4,1))
%         Mapping_temp(DOF_Node_Rail*(i-1)+1:DOF_Node_Rail*i,2) = num2cell(Pos_Node.qjbg(i-sum(N_Node.Sort(1:3,1)),1));
%         Mapping_temp(DOF_Node_Rail*(i-1)+1:DOF_Node_Rail*i,3) = {'UY'; 'UZ'; 'ROTY'; 'ROTZ'};
%     elseif i <= sum(N_Node.Sort(1:5,1))
%         pos = DOF_Node_Rail*sum(N_Node.Sort(1:4,1))+DOF_Node_Baseplate*(i-sum(N_Node.Sort(1:4,1))-1)+1:1:...
%               DOF_Node_Rail*sum(N_Node.Sort(1:4,1))+DOF_Node_Baseplate*(i-sum(N_Node.Sort(1:4,1)));
%         Mapping_temp(pos,2) = num2cell(Pos_Node.Baseplate_L_Switch(i-sum(N_Node.Sort(1:4,1)),1));
%         Mapping_temp(pos,3) = {'UZ','ROTX'};
%     else
%         pos = DOF_Node_Rail*sum(N_Node.Sort(1:4,1))+DOF_Node_Baseplate*(i-sum(N_Node.Sort(1:4,1))-1)+1:1:...
%               DOF_Node_Rail*sum(N_Node.Sort(1:4,1))+DOF_Node_Baseplate*(i-sum(N_Node.Sort(1:4,1)));
%         Mapping_temp(pos,2) = num2cell(Pos_Node.Baseplate_R_Switch(i-sum(N_Node.Sort(1:5,1)),1));
%         Mapping_temp(pos,3) = {'UZ','ROTX'};
%     end
    if i <= sum(N_Node.Sort(1:1,1))
        Mapping_temp(DOF_Node_Rail*(i-1)+1:DOF_Node_Rail*i,2) = num2cell(Pos_Node.zjbg(i,1));
%         Mapping_temp(DOF_Node_Rail*(i-1)+1:DOF_Node_Rail*i,3) = {'UY'; 'UZ'; 'ROTY'; 'ROTZ'};
        Mapping_temp(DOF_Node_Rail*(i-1)+1:DOF_Node_Rail*i,3) = {'UX'; 'UY'; 'UZ'; 'ROTX'; 'ROTY'; 'ROTZ'};
    else
        Mapping_temp(DOF_Node_Rail*(i-1)+1:DOF_Node_Rail*i,2) = num2cell(Pos_Node.qjbg(i-sum(N_Node.Sort(1:1,1)),1));
%         Mapping_temp(DOF_Node_Rail*(i-1)+1:DOF_Node_Rail*i,3) = {'UY'; 'UZ'; 'ROTY'; 'ROTZ'};
        Mapping_temp(DOF_Node_Rail*(i-1)+1:DOF_Node_Rail*i,3) = {'UX'; 'UY'; 'UZ'; 'ROTX'; 'ROTY'; 'ROTZ'};
    end
end

% Mass_Rail, Stiff_Rail
% [Target_Mat] = Matrix_PosTrans(Source_Mat, Source_Mapping, Target_Mapping, N_DOF_Rail)
Mass_Rail.Cons_Trans = Matrix_PosTrans(Mass_Rail.Cons, Mass_Rail.Cons_Mapping, Mapping_temp, DOF_Rail.Cons);
Stiff_Rail.Cons_Trans = Matrix_PosTrans(Stiff_Rail.Cons, Stiff_Rail.Cons_Mapping, Mapping_temp, DOF_Rail.Cons);
Mass_Rail.Cons_Trans_Mapping = Mapping_temp;
Stiff_Rail.Cons_Trans_Mapping = Mapping_temp;

fields = {'Cons','Cons_Mapping'};
Mass_Rail = rmfield(Mass_Rail,fields);
Stiff_Rail = rmfield(Stiff_Rail,fields);

% Delete the Constrained DOF of Free Matrices, Mass_Rail, Stiff_Rail, Pos_Node
% bools_1 = (cell2mat(Mass_Rail.Free_Mapping(:,2))>=min(Pos_Node.zjbg(:,1))) & (cell2mat(Mass_Rail.Free_Mapping(:,2))<=max(Pos_Node.qjg_cgyg(:,1))) & ...
%                    (strcmp(Mass_Rail.Free_Mapping(:,3),'UX')|strcmp(Mass_Rail.Free_Mapping(:,3),'ROTX'));
% bools_2 = (cell2mat(Mass_Rail.Free_Mapping(:,2))>=min(Pos_Node.zjg_zgyg(:,1))) & (cell2mat(Mass_Rail.Free_Mapping(:,2))<=max(Pos_Node.qjbg(:,1))) & ...
%                    (strcmp(Mass_Rail.Free_Mapping(:,3),'UX')|strcmp(Mass_Rail.Free_Mapping(:,3),'ROTX'));
% bools_3 = (cell2mat(Mass_Rail.Free_Mapping(:,2))>=min(Pos_Node.Baseplate_L_Switch(:,1))) & (cell2mat(Mass_Rail.Free_Mapping(:,2))<=max(Pos_Node.Baseplate_L_Switch(:,1))) & ...
%                    (~(strcmp(Mass_Rail.Free_Mapping(:,3),'UZ')|strcmp(Mass_Rail.Free_Mapping(:,3),'ROTX')));
% bools_4 = (cell2mat(Mass_Rail.Free_Mapping(:,2))>=min(Pos_Node.Baseplate_R_Switch(:,1))) & (cell2mat(Mass_Rail.Free_Mapping(:,2))<=max(Pos_Node.Baseplate_R_Switch(:,1))) & ...
%                    (~(strcmp(Mass_Rail.Free_Mapping(:,3),'UZ')|strcmp(Mass_Rail.Free_Mapping(:,3),'ROTX')));
% bools = bools_1+bools_2+bools_3+bools_4;

% bools = strcmp(Mass_Rail.Free_Mapping(:,3),'UX') | strcmp(Mass_Rail.Free_Mapping(:,3),'ROTX');
     
% Mass_Rail.Free_Trans1 = Mass_Rail.Free;
% Mass_Rail.Free_Trans1(find(bools),:) = [];
% Mass_Rail.Free_Trans1(:,find(bools)) = [];
% Mass_Rail.Free_Trans1_Mapping = Mass_Rail.Free_Mapping;
% Mass_Rail.Free_Trans1_Mapping(find(bools),:) = [];
% Stiff_Rail.Free_Trans1 = Stiff_Rail.Free;
% Stiff_Rail.Free_Trans1(find(bools),:) = [];
% Stiff_Rail.Free_Trans1(:,find(bools)) = [];
% Stiff_Rail.Free_Trans1_Mapping = Stiff_Rail.Free_Mapping;
% Stiff_Rail.Free_Trans1_Mapping(find(bools),:) = [];

% Check
Target_Mapping_1 = Mapping_temp;
Target_Mapping_2 = Mass_Rail.Free_Mapping;
Check = zeros(size(Target_Mapping_1,1), 2);
for i = 1:1:size(Target_Mapping_1,1)
    if Target_Mapping_2{i,2} == Target_Mapping_1{i,2}
        Check(i,1) = 1;
    end  
    if strcmp(Target_Mapping_2{i,3}, Target_Mapping_1{i,3})
        Check(i,2) = 1;
    end    
end

%% 4. NM_Rail, ModeFreq, ModeShape
%%% ModeFreq
NM_del = 2;
NM_Rail.All = 2730-NM_del;
fid = fopen('Modefile_PT-220709_6DOF_FreqModal.txt');
ModeFreq_temp = textscan(fid, '%f %f %f', NM_Rail.All, 'HeaderLines', NM_del);
fclose(fid);
ModeFreq.FT_All = [ModeFreq_temp{1,1} ModeFreq_temp{1,2} ModeFreq_temp{1,3}];
ModeFreq.FT_All(:,1) = ModeFreq.FT_All(:,1)-NM_del;

%%% 确定第k阶 Modeshape 矩阵(N*1)元素位置与 Modefile 文件中振型结果位置的映射关系
fid = fopen('Modefile_PT-220709_Cons_6DOF_Mode120.txt');
temp = textscan(fid, repmat(' %f',1,7), 'HeaderLines',1);
fclose(fid);
Modefile_temp = [temp{1},temp{2},temp{3},temp{4},temp{5},temp{6},temp{7}];

Target = Mass_Rail.Cons_Trans_Mapping;
for i = 1:1:DOF_Rail.Cons
    pos = find(Modefile_temp(:,1)==Target{i,2});
    if strcmp(Target{i,3}, 'UX')
        ModeShape.Modefile_Mapping(i,1) = pos+size(Modefile_temp,1)*1;
    elseif strcmp(Target{i,3}, 'UY')
        ModeShape.Modefile_Mapping(i,1) = pos+size(Modefile_temp,1)*2;
    elseif strcmp(Target{i,3}, 'UZ')
        ModeShape.Modefile_Mapping(i,1) = pos+size(Modefile_temp,1)*3;
    elseif strcmp(Target{i,3}, 'ROTX')
        ModeShape.Modefile_Mapping(i,1) = pos+size(Modefile_temp,1)*4;
    elseif strcmp(Target{i,3}, 'ROTY')
        ModeShape.Modefile_Mapping(i,1) = pos+size(Modefile_temp,1)*5;
    else
        ModeShape.Modefile_Mapping(i,1) = pos+size(Modefile_temp,1)*6;
    end
end

%%% 振型文件 ModeShape
ModeShape.FT_Cons_All = zeros(DOF_Rail.Cons, NM_Rail.All);
for j = 1:1:NM_Rail.All
    fid = fopen(['Modefile_PT-220709_Cons_6DOF_Mode',num2str(j+NM_del),'.txt']);
    temp = textscan(fid, repmat(' %f',1,7), 'HeaderLines',1);
    fclose(fid);
    Modefile_temp = [temp{1},temp{2},temp{3},temp{4},temp{5},temp{6},temp{7}];    
    for i = 1:1:DOF_Rail.Cons
        pos = ModeShape.Modefile_Mapping(i,1);
        ModeShape.FT_Cons_All(i,j) = Modefile_temp(pos);
    end
end

p = DOF_Rail_start(1,1)+1;
q = DOF_Rail_start(2,1);
ModeShape.zjbg = ModeShape.FT_Cons_All(p:q,:);
p = DOF_Rail_start(2,1)+1;
q = DOF_Rail_start(3,1);
ModeShape.qjbg = ModeShape.FT_Cons_All(p:q,:);

%% 0.1 Generalized Mass and Stiffness, ModeShape, ModeFreq
%%% Mass_Rail, Stiff_Rail, Damp_Rail, Pos_Node, N_Node, DOF_Rail, NM_Rail, ModeFreq, ModeShape
if Choose_Check == 1
    Target = 'FT_Cons_All';
    clear output_Nor
    for p = 1:1:NM_Rail.All
        output_Nor(p,1) = (ModeShape.(Target)(:,p))' * Mass_Rail.([Target, '_Trans']) * ModeShape.(Target)(:,p);
        output_Nor(p,2) = (ModeShape.(Target)(:,p))' * Stiff_Rail.([Target, '_Trans']) * ModeShape.(Target)(:,p);
        output_Nor(p,3) = (2*pi*ModeFreq.FT_All(p,2))^2*output_Nor(p,1)/output_Nor(p,2);
    end
    
    len = 1:1:NM_Rail.All;
    Mass_Generalized = (ModeShape.(Target))' * Mass_Rail.([Target, '_Trans']) * ModeShape.(Target);
    for i = 1:1:length(Mass_Generalized)
        Mass_Generalized(i,i) = 0;
    end
    min(min(Mass_Generalized))
    max(max(Mass_Generalized))
    
    figure(1); clf
    plot(len, output_Nor(:,1), len, output_Nor(:,3));
    grid on
end

%% 0.2 Compare matrices between that in the free and constrainted state: Mass_Rail, Stiff_Rail, Pos_Node
%%% Mass_Rail, Stiff_Rail, Damp_Rail, Pos_Node, N_Node, DOF_Rail， NM_Rail, ModeFreq, ModeShape
% [Target_Mat] = Matrix_PosTrans(Source_Mat, Source_Mapping, Target_Mapping, N_DOF_Rail)
% Mass_Rail.Free_Trans2 = Matrix_PosTrans(Mass_Rail.Free_Trans1, Mass_Rail.Free_Trans1_Mapping, Mass_Rail.Cons_Trans_Mapping, DOF_Rail.Cons);
% Mass_Rail.Free_Trans2_Mapping = Mass_Rail.Cons_Trans_Mapping;
% Stiff_Rail.Free_Trans2 = Matrix_PosTrans(Stiff_Rail.Free_Trans1, Stiff_Rail.Free_Trans1_Mapping, Stiff_Rail.Cons_Trans_Mapping, DOF_Rail.Cons);
% Stiff_Rail.Free_Trans2_Mapping = Stiff_Rail.Cons_Trans_Mapping;
% 
% fields = {'Free','Free_Mapping','Free_Trans1','Free_Trans1_Mapping'};
% Mass_Rail = rmfield(Mass_Rail,fields);
% Stiff_Rail = rmfield(Stiff_Rail,fields);

%%% Input: Stiff_AddCons, Damp_AddCons
ConsPar_RailPad = Par_Track.ConsPar_RailPad;        % 轨下胶垫
% ConsPar_PlatePad = Par_Track.ConsPar_PlatePad;      % 板下胶垫, Par_Track.ConsPar_PlatePad.Baseplate_L_Switch_Stiff_Z
% ConsPar_Baseplate = Par_Track.ConsPar_Baseplate;    % 滑床台板刚性支撑
% ConsPar_LatCon = Par_Track.ConsPar_LatCon;          % 密贴和顶铁横向支撑刚度

Target_Mapping = Mass_Rail.Cons_Trans_Mapping;
Stiff_AddCons = zeros(size(Stiff_Rail.Cons_Trans));
Damp_AddCons = zeros(size(Stiff_Rail.Cons_Trans));
% pos_PlatePad_UZ_L={};	pos_PlatePad_UZ_R={}; 
% pos_Rail_UX=[]; pos_Rail_UY=[]; pos_Rail_UZ=[]; pos_Baseplate_UZ=[]; pos_RailBaseplate_UZ = []; pos_BaseplateRail_UZ = [];

%%% Plain Track
for i = 1:1:size(Target_Mapping,1)
    num_Node = Target_Mapping{i,2};
    if num_Node<=max(Pos_Node.zjbg(:,1))
        pos = find(Pos_Node.zjbg(:,1)==num_Node);
        Coor = Pos_Node.zjbg(pos,2:4);
    else
        pos = find(Pos_Node.qjbg(:,1)==num_Node);
        Coor = Pos_Node.qjbg(pos,2:4);
    end
    if find(abs(Pos_Node.Sleeper(:,2)-Coor(1))<=1e-10)
        if strcmp(Target_Mapping{i,3}, 'UX')
            Stiff_AddCons(i,i) = Stiff_AddCons(i,i)+ConsPar_RailPad.Stiff_X;
            Damp_AddCons(i,i) = Damp_AddCons(i,i)+ConsPar_RailPad.Damp_X;
        elseif strcmp(Target_Mapping{i,3}, 'UY')
            Stiff_AddCons(i,i) = Stiff_AddCons(i,i)+ConsPar_RailPad.Stiff_Y;
            Damp_AddCons(i,i) = Damp_AddCons(i,i)+ConsPar_RailPad.Damp_Y;
        elseif strcmp(Target_Mapping{i,3}, 'UZ')
            Stiff_AddCons(i,i) = Stiff_AddCons(i,i)+ConsPar_RailPad.Stiff_Z;
            Damp_AddCons(i,i) = Damp_AddCons(i,i)+ConsPar_RailPad.Damp_Z;
        end
    end
end

% %%% A. Baseplate PlatePad
% kk = 1;
% for i = 1:1:length(Pos_Node.Sleeper)
%     [pos_PlatePad_UZ_L, kk] = AddCons_I_BaseplateSearchPos(DOF_Rail.Baseplate_L_Switch, Target_Mapping, Pos_Node.Sleeper, Pos_Node.Baseplate_L_Switch, i, kk, pos_PlatePad_UZ_L);
% end
% kk = 1;
% for i = 1:1:length(Pos_Node.Sleeper)
%     [pos_PlatePad_UZ_R, kk] = AddCons_I_BaseplateSearchPos(DOF_Rail.Baseplate_R_Switch, Target_Mapping, Pos_Node.Sleeper, Pos_Node.Baseplate_R_Switch, i, kk, pos_PlatePad_UZ_R);
% end
% 
% % [Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat(Type_Cons, Stiff_AddCons, Damp_AddCons, ConsPar_RailPad, ConsPar_PlatePad, Coff_ConsPar, ...
% %  pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, i_PlatePad);
% [Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Baseplate', Stiff_AddCons, Damp_AddCons, [], ConsPar_PlatePad.Baseplate_L_Switch, [],...
%  [], [], [], pos_PlatePad_UZ_L, [], [], DOF_Rail.Cons);
% [Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Baseplate', Stiff_AddCons, Damp_AddCons, [], ConsPar_PlatePad.Baseplate_R_Switch, [],...
%  [], [], [], pos_PlatePad_UZ_R, [], [], DOF_Rail.Cons);
% 
% %%% B. Stock Rail
% %%% B1, 基本轨单侧扣压，尖轨尖端之后，04~34岔枕，50.120~68.145m
% % Left: zjbg
% bools = Pos_Node.Sleeper(:,2)>=50.120 & Pos_Node.Sleeper(:,2)<=68.145;
% kk = 1;
% for i = 1:1:length(find(bools))
%     [pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, kk] = ...
%     AddCons_I_RailSearchPos('Accurate', DOF_Rail.Cons, Target_Mapping, Pos_Node.Sleeper(bools,:), Pos_Node.zjbg, Pos_Node.Baseplate_L_Switch, i, kk,...
%      pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ);
% end
% [Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Rail', Stiff_AddCons, Damp_AddCons, ConsPar_RailPad, [], [1,1/2,1], ...
%  pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, DOF_Rail.Cons);
% % Right: qjbg
% kk = 1;
% for i = 1:1:length(find(bools))
%     [pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, kk] = ...
%     AddCons_I_RailSearchPos('Accurate', DOF_Rail.Cons, Target_Mapping, Pos_Node.Sleeper(bools,:), Pos_Node.qjbg, Pos_Node.Baseplate_R_Switch, i, kk,...
%      pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ);
% end
% [Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Rail', Stiff_AddCons, Damp_AddCons, ConsPar_RailPad, [], [1,1/2,1], ...
%  pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, DOF_Rail.Cons);
% 
% %%% B2. 基本轨双侧扣压
% % Left: zjbg
% kk = 1;
% for i = 1:1:length(find(~bools))
%     [pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, kk] = ...
%     AddCons_I_RailSearchPos('Accurate', DOF_Rail.Cons, Target_Mapping, Pos_Node.Sleeper(~bools,:), Pos_Node.zjbg, Pos_Node.Baseplate_L_Switch, i, kk,...
%      pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ);
% end
% [Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Rail', Stiff_AddCons, Damp_AddCons, ConsPar_RailPad, [], [1,1,1], ...
%  pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, DOF_Rail.Cons);
% % Right: qjbg
% kk = 1;
% for i = 1:1:length(find(~bools))
%     [pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, kk] = ...
%     AddCons_I_RailSearchPos('Accurate', DOF_Rail.Cons, Target_Mapping, Pos_Node.Sleeper(~bools,:), Pos_Node.qjbg, Pos_Node.Baseplate_R_Switch, i, kk,...
%      pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ);
% end
% [Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Rail', Stiff_AddCons, Damp_AddCons, ConsPar_RailPad, [], [1,1,1], ...
%  pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, DOF_Rail.Cons);
% 
% %%% C. Switch Rail
% %%% C1, 尖轨前端，岔枕号4，滑床台板i=65
% kk = 1;
% Relation_C1 = load('SwitchBlade_C1_211224.txt');
% for i = 1:1:size(Relation_C1,1)
%     [pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, kk] = ...
%     AddCons_I_RailSearchPos_v2(Relation_C1(i,1), Relation_C1(i,5), Target_Mapping, kk,...
%      pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ);
% end
% [Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Rail', Stiff_AddCons, Damp_AddCons, ConsPar_Baseplate, [], [1,0,1]/2, ...
%  pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, DOF_Rail.Cons);
% 
% %%% C2, 滑床台板支撑，岔枕号5-34，滑床台板i=66-95，50.770-68.145m
% % Left: qjg_cgyg
% bools = Pos_Node.Sleeper(:,2)>=50.770 & Pos_Node.Sleeper(:,2)<=68.145;
% kk = 1;
% for i = 1:1:length(find(bools))
%     [pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, kk] = ...
%     AddCons_I_RailSearchPos('Accurate', DOF_Rail.Cons, Target_Mapping, Pos_Node.Sleeper(bools,:), Pos_Node.qjg_cgyg, Pos_Node.Baseplate_L_Switch, i, kk,...
%      pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ);
% end
% [Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Rail', Stiff_AddCons, Damp_AddCons, ConsPar_Baseplate, [], [1,0,1], ...
%  pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, DOF_Rail.Cons);
% % Right: zjg_zgyg
% kk = 1;
% for i = 1:1:length(find(bools))
%     [pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, kk] = ...
%     AddCons_I_RailSearchPos('Accurate', DOF_Rail.Cons, Target_Mapping, Pos_Node.Sleeper(bools,:), Pos_Node.zjg_zgyg, Pos_Node.Baseplate_R_Switch, i, kk,...
%      pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ);
% end
% [Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Rail', Stiff_AddCons, Damp_AddCons, ConsPar_Baseplate, [], [1,0,1], ...
%  pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, DOF_Rail.Cons);
% 
% %%% C3, 扣件垫板支撑，岔枕号35-58，滑床台板i=96-end，68.745-82.545m
% % Left: qjg_cgyg
% bools = Pos_Node.Sleeper(:,2)>=68.745 & Pos_Node.Sleeper(:,2)<=82.545;
% kk = 1;
% for i = 1:1:length(find(bools))
%     [pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, kk] = ...
%     AddCons_I_RailSearchPos('Accurate', DOF_Rail.Cons, Target_Mapping, Pos_Node.Sleeper(bools,:), Pos_Node.qjg_cgyg, Pos_Node.Baseplate_L_Switch, i, kk,...
%      pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ);
% end
% [Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Rail', Stiff_AddCons, Damp_AddCons, ConsPar_RailPad, [], [1,1,1], ...
%  pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, DOF_Rail.Cons);
% % Right: zjg_zgyg
% kk = 1;
% for i = 1:1:length(find(bools))
%     [pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, kk] = ...
%     AddCons_I_RailSearchPos('Accurate', DOF_Rail.Cons, Target_Mapping, Pos_Node.Sleeper(bools,:), Pos_Node.zjg_zgyg, Pos_Node.Baseplate_R_Switch, i, kk,...
%      pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ);
% end
% [Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Rail', Stiff_AddCons, Damp_AddCons, ConsPar_RailPad, [], [1,1,1], ...
%  pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, DOF_Rail.Cons);
% 
% %%% D. 密贴力和顶铁力，LatContact_Switch_jg_jbg
% pos_Rail_11_UY = [];    pos_Rail_22_UY = [];    pos_Rail_12_UY = [];    pos_Rail_21_UY = [];
% Relation_C1 = load('CloseForce_jg_jbg_D1_211224.txt');
% Relation_C2 = load('CloseForce_jg_jbg_D2_211224.txt');
% Relation_C3 = load('CloseForce_jg_jbg_D3_211224.txt');
% %%% D1, 尖轨尖端与基本轨密贴，岔枕号3.75-4, i=1-3, 50.000m
% kk = 1;
% for i = 1:1:size(Relation_C1,1)
%     [pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY, kk] = ...
%      AddCons_I_LatConSearchPos_v2(Relation_C1(i,1), Relation_C1(i,5), Target_Mapping, kk,...
%      pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY);
% end
% [Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat_LatCon(Stiff_AddCons, Damp_AddCons, ConsPar_LatCon, [1,1,1]/4,...
%  pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY, DOF_Rail.Cons);
% 
% %%% D2, 尖轨与基本轨密贴，岔枕号4.5-22, i=4-83, 50.770-60.970m
% kk = 1;
% for i = 1:1:size(Relation_C2,1)
%     [pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY, kk] = ...
%      AddCons_I_LatConSearchPos_v2(Relation_C2(i,1), Relation_C2(i,5), Target_Mapping, kk,...
%      pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY);
% end
% [Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat_LatCon(Stiff_AddCons, Damp_AddCons, ConsPar_LatCon, [1,1,1]/2,...
%  pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY, DOF_Rail.Cons);
% 
% %%% D3, 尖轨与基本轨顶铁约束, ET6/R61 (岔枕号23.5-33.5, i=81-121, 61.792-67.845m)
% kk = 1;
% for i = 1:1:size(Relation_C3,1)
%     [pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY, kk] = ...
%      AddCons_I_LatConSearchPos_v2(Relation_C3(i,1), Relation_C3(i,5), Target_Mapping, kk,...
%      pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY);
% end
% [Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat_LatCon(Stiff_AddCons, Damp_AddCons, ConsPar_LatCon, [1,1,1]/1,...
%  pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY, DOF_Rail.Cons);

%%% E. Check, Stiff_Rail
% Stiff_Diff = Stiff_Rail.Cons_Trans - Stiff_Rail.Free_Trans2;
% Comp_Stiff_Diff(1,1) = min(min(Stiff_Diff));
% Comp_Stiff_Diff(1,2) = max(max(Stiff_Diff));
Comp = (Stiff_Rail.Cons_Trans - Stiff_Rail.Free) - Stiff_AddCons;
CompDiff(1,1) = min(min(Comp));
CompDiff(1,2) = max(max(Comp));
% [~,pos] = min(min(Comp))
% [~,pos] = max(max(Comp))

%%% F. Damp_Rail
Damp_Rail.Cons_Trans = Damp_AddCons;
Damp_Rail.Cons_Trans_Mapping = Mass_Rail.Cons_Trans_Mapping;
clear Stiff_AddCons Damp_AddCons
% !!!!!!    测试用
% Stiff_Rail.Cons_Trans = Stiff_Rail.Free_Trans2 + Stiff_AddCons;
% !!!!!!

