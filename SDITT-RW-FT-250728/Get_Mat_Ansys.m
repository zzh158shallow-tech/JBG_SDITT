%% ¥””–œﬁ‘™ANSYS÷–Ã·»°∏÷πÏ¡∫ƒ£–Õµƒ÷ ¡ø°¢∏’∂»æÿ’Û
function [Mass_Rail, Stiff_Rail, Damp_Rail, CompDiff, ...
          ModeFreq, ModeShape, Pos_Node, N_Node, NM_Rail, DOF_Rail, DOF_Rail_start] = ...
         Get_Mat_Ansys

Choose_Check = 0;

global Par_Track

%% 1. Mass_Rail, Stiff_Rail, Damp_Rail, DOF_Rail
[Mass_Rail.Free, Mass_Rail.Free_Mapping, DOF_Rail.Free] = Load_HBFILE('HBFILE_Switch-211105_Free_MASS');
[Stiff_Rail.Free, Stiff_Rail.Free_Mapping, DOF_Rail.Free] = Load_HBFILE('HBFILE_Switch-211105_Free_STIFF');
[Mass_Rail.Cons, Mass_Rail.Cons_Mapping, DOF_Rail.Cons] = Load_HBFILE('HBFILE_Switch-211105_Constraints_MASS');
[Stiff_Rail.Cons, Stiff_Rail.Cons_Mapping, DOF_Rail.Cons] = Load_HBFILE('HBFILE_Switch-211105_Constraints_STIFF');

DOF_Node_Rail = 4;
DOF_Node_Baseplate = 2;

%% 2. Pos_Node, N_Node, DOF_Rail
fid = fopen('NodePos_Switch-211105.txt');
format = '%f %f %f %f %f %f %f';
pos_Node_load = textscan(fid, format);
fclose(fid);
pos_Node_ori = [pos_Node_load{1,1} pos_Node_load{1,2} pos_Node_load{1,3} pos_Node_load{1,4} pos_Node_load{1,5} pos_Node_load{1,6} pos_Node_load{1,7}];

Pos_Node.zjbg = pos_Node_ori(1:285,:);
Pos_Node.zjbg = sortrows(Pos_Node.zjbg,2);
Pos_Node.qjg_cgyg = pos_Node_ori(285+1:285+183,:);
Pos_Node.qjg_cgyg = sortrows(Pos_Node.qjg_cgyg,2);
Pos_Node.Baseplate_L_Switch = pos_Node_ori(285+183+1:285+183+703,:);
Pos_Node.Baseplate_L_Switch = sortrows(Pos_Node.Baseplate_L_Switch,2);
Pos_Node.zjg_zgyg = pos_Node_ori(285+183+703+1:285+183*2+703,:);
Pos_Node.zjg_zgyg = sortrows(Pos_Node.zjg_zgyg,2);
Pos_Node.qjbg = pos_Node_ori(285+183*2+703+1:285*2+183*2+703,:);
Pos_Node.qjbg = sortrows(Pos_Node.qjbg,2);
Pos_Node.Baseplate_R_Switch = pos_Node_ori(285*2+183*2+703+1:(285+183+703)*2,:);
Pos_Node.Baseplate_R_Switch = sortrows(Pos_Node.Baseplate_R_Switch,2);

N_Node.zjbg = size(Pos_Node.zjbg,1);
N_Node.qjg_cgyg = size(Pos_Node.qjg_cgyg,1);
N_Node.zjg_zgyg = size(Pos_Node.zjg_zgyg,1);
N_Node.qjbg = size(Pos_Node.qjbg,1);
N_Node.Baseplate_L_Switch = size(Pos_Node.Baseplate_L_Switch,1);
N_Node.Baseplate_R_Switch = size(Pos_Node.Baseplate_R_Switch,1);

N_Node.Rail = N_Node.zjbg+N_Node.qjg_cgyg+N_Node.zjg_zgyg+N_Node.qjbg;
N_Node.Baseplate = N_Node.Baseplate_L_Switch+N_Node.Baseplate_R_Switch;
N_Node.All = N_Node.Rail+N_Node.Baseplate;
N_Node.Sort = [N_Node.zjbg; N_Node.qjg_cgyg; N_Node.zjg_zgyg; N_Node.qjbg; N_Node.Baseplate_L_Switch; N_Node.Baseplate_R_Switch];

DOF_Rail.zjbg = N_Node.zjbg*DOF_Node_Rail;
DOF_Rail.qjg_cgyg = N_Node.qjg_cgyg*DOF_Node_Rail;
DOF_Rail.zjg_zgyg = N_Node.zjg_zgyg*DOF_Node_Rail;
DOF_Rail.qjbg = N_Node.qjbg*DOF_Node_Rail;
DOF_Rail.Baseplate_L_Switch = N_Node.Baseplate_L_Switch*DOF_Node_Baseplate;
DOF_Rail.Baseplate_R_Switch = N_Node.Baseplate_R_Switch*DOF_Node_Baseplate;
DOF_Rail.Sort = [DOF_Rail.zjbg; DOF_Rail.qjg_cgyg; DOF_Rail.zjg_zgyg; DOF_Rail.qjbg; DOF_Rail.Baseplate_L_Switch; DOF_Rail.Baseplate_R_Switch];

Pos_Node.Sleeper = Pos_Node.Baseplate_L_Switch;
bools_save = diff(Pos_Node.Sleeper(:,2))~=0;
Pos_Node.Sleeper = Pos_Node.Sleeper([find(bools_save);length(Pos_Node.Sleeper)],:);
Pos_Node.Sleeper = [(-60:1:58)' Pos_Node.Sleeper(:,2)];

%% 3. Transfer the Constrainted Matrices based on the Sepcified DOF Order
%%% Pos_Node, N_Node, DOF_Rail, Mapping_temp
clear Mapping_temp
Mapping_temp = cell(DOF_Rail.Cons,3);
Mapping_temp(:,1) = num2cell(1:1:DOF_Rail.Cons);
for i = 1:1:N_Node.All
    if i <= sum(N_Node.Sort(1:1,1))
        Mapping_temp(DOF_Node_Rail*(i-1)+1:DOF_Node_Rail*i,2) = num2cell(Pos_Node.zjbg(i,1));
        Mapping_temp(DOF_Node_Rail*(i-1)+1:DOF_Node_Rail*i,3) = {'UY'; 'UZ'; 'ROTY'; 'ROTZ'};
    elseif i <= sum(N_Node.Sort(1:2,1))
        Mapping_temp(DOF_Node_Rail*(i-1)+1:DOF_Node_Rail*i,2) = num2cell(Pos_Node.qjg_cgyg(i-sum(N_Node.Sort(1:1,1)),1));
        Mapping_temp(DOF_Node_Rail*(i-1)+1:DOF_Node_Rail*i,3) = {'UY'; 'UZ'; 'ROTY'; 'ROTZ'};
    elseif i <= sum(N_Node.Sort(1:3,1))
        Mapping_temp(DOF_Node_Rail*(i-1)+1:DOF_Node_Rail*i,2) = num2cell(Pos_Node.zjg_zgyg(i-sum(N_Node.Sort(1:2,1)),1));
        Mapping_temp(DOF_Node_Rail*(i-1)+1:DOF_Node_Rail*i,3) = {'UY'; 'UZ'; 'ROTY'; 'ROTZ'};
    elseif i <= sum(N_Node.Sort(1:4,1))
        Mapping_temp(DOF_Node_Rail*(i-1)+1:DOF_Node_Rail*i,2) = num2cell(Pos_Node.qjbg(i-sum(N_Node.Sort(1:3,1)),1));
        Mapping_temp(DOF_Node_Rail*(i-1)+1:DOF_Node_Rail*i,3) = {'UY'; 'UZ'; 'ROTY'; 'ROTZ'};
    elseif i <= sum(N_Node.Sort(1:5,1))
        pos = DOF_Node_Rail*sum(N_Node.Sort(1:4,1))+DOF_Node_Baseplate*(i-sum(N_Node.Sort(1:4,1))-1)+1:1:...
              DOF_Node_Rail*sum(N_Node.Sort(1:4,1))+DOF_Node_Baseplate*(i-sum(N_Node.Sort(1:4,1)));
        Mapping_temp(pos,2) = num2cell(Pos_Node.Baseplate_L_Switch(i-sum(N_Node.Sort(1:4,1)),1));
        Mapping_temp(pos,3) = {'UZ','ROTX'};
    else
        pos = DOF_Node_Rail*sum(N_Node.Sort(1:4,1))+DOF_Node_Baseplate*(i-sum(N_Node.Sort(1:4,1))-1)+1:1:...
              DOF_Node_Rail*sum(N_Node.Sort(1:4,1))+DOF_Node_Baseplate*(i-sum(N_Node.Sort(1:4,1)));
        Mapping_temp(pos,2) = num2cell(Pos_Node.Baseplate_R_Switch(i-sum(N_Node.Sort(1:5,1)),1));
        Mapping_temp(pos,3) = {'UZ','ROTX'};
    end
end

% [Target_Mat] = Matrix_PosTrans(Source_Mat, Source_Mapping, Target_Mapping, N_DOF_Rail)
Mass_Rail.Cons_Trans = Matrix_PosTrans(Mass_Rail.Cons, Mass_Rail.Cons_Mapping, Mapping_temp, DOF_Rail.Cons);
Stiff_Rail.Cons_Trans = Matrix_PosTrans(Stiff_Rail.Cons, Stiff_Rail.Cons_Mapping, Mapping_temp, DOF_Rail.Cons);
Mass_Rail.Cons_Trans_Mapping = Mapping_temp;
Stiff_Rail.Cons_Trans_Mapping = Mapping_temp;

fields = {'Cons','Cons_Mapping'};
Mass_Rail = rmfield(Mass_Rail,fields);
Stiff_Rail = rmfield(Stiff_Rail,fields);

% Delete the Constrained DOF of Free Matrices, Mass_Rail, Stiff_Rail, Pos_Node
bools_1 = (cell2mat(Mass_Rail.Free_Mapping(:,2))>=min(Pos_Node.zjbg(:,1))) & (cell2mat(Mass_Rail.Free_Mapping(:,2))<=max(Pos_Node.qjg_cgyg(:,1))) & ...
          (strcmp(Mass_Rail.Free_Mapping(:,3),'UX')|strcmp(Mass_Rail.Free_Mapping(:,3),'ROTX'));
bools_2 = (cell2mat(Mass_Rail.Free_Mapping(:,2))>=min(Pos_Node.zjg_zgyg(:,1))) & (cell2mat(Mass_Rail.Free_Mapping(:,2))<=max(Pos_Node.qjbg(:,1))) & ...
          (strcmp(Mass_Rail.Free_Mapping(:,3),'UX')|strcmp(Mass_Rail.Free_Mapping(:,3),'ROTX'));
bools_3 = (cell2mat(Mass_Rail.Free_Mapping(:,2))>=min(Pos_Node.Baseplate_L_Switch(:,1))) & (cell2mat(Mass_Rail.Free_Mapping(:,2))<=max(Pos_Node.Baseplate_L_Switch(:,1))) & ...
          (~(strcmp(Mass_Rail.Free_Mapping(:,3),'UZ')|strcmp(Mass_Rail.Free_Mapping(:,3),'ROTX')));
bools_4 = (cell2mat(Mass_Rail.Free_Mapping(:,2))>=min(Pos_Node.Baseplate_R_Switch(:,1))) & (cell2mat(Mass_Rail.Free_Mapping(:,2))<=max(Pos_Node.Baseplate_R_Switch(:,1))) & ...
          (~(strcmp(Mass_Rail.Free_Mapping(:,3),'UZ')|strcmp(Mass_Rail.Free_Mapping(:,3),'ROTX')));
bools = bools_1+bools_2+bools_3+bools_4;
     
Mass_Rail.Free_Trans1 = Mass_Rail.Free;
Mass_Rail.Free_Trans1(find(bools),:) = [];
Mass_Rail.Free_Trans1(:,find(bools)) = [];
Mass_Rail.Free_Trans1_Mapping = Mass_Rail.Free_Mapping;
Mass_Rail.Free_Trans1_Mapping(find(bools),:) = [];
Stiff_Rail.Free_Trans1 = Stiff_Rail.Free;
Stiff_Rail.Free_Trans1(find(bools),:) = [];
Stiff_Rail.Free_Trans1(:,find(bools)) = [];
Stiff_Rail.Free_Trans1_Mapping = Stiff_Rail.Free_Mapping;
Stiff_Rail.Free_Trans1_Mapping(find(bools),:) = [];

% Check
% temp = Stiff_Rail.Cons_Trans-Stiff_Rail.Free_Trans1;

%% 4. NM_Rail, ModeFreq, ModeShape
%%% ModeFreq
NM_del = 0;
NM_Rail.All = 300-NM_del;
fid = fopen('Modefile_Switch-211105_FreqModal.txt');
ModeFreq_temp = textscan(fid, '%f %f %f', NM_Rail.All, 'HeaderLines', NM_del);
fclose(fid);
ModeFreq.All = [ModeFreq_temp{1,1} ModeFreq_temp{1,2} ModeFreq_temp{1,3}];
ModeFreq.All(:,1) = ModeFreq.All(:,1)-NM_del;

%%% »∑∂®µ⁄kΩ◊ Modeshape æÿ’Û(N*1)‘™ÀÿŒª÷√”Î Modefile Œƒº˛÷–’Ò–ÕΩ·π˚Œª÷√µƒ”≥…‰πÿœµ
fid = fopen('Modefile_Switch-211105_Constraints_Mode125.txt');
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

%%% ’Ò–ÕŒƒº˛ ModeShape
ModeShape.Cons = zeros(DOF_Rail.Cons, NM_Rail.All);
for j = 1:1:NM_Rail.All
    fid = fopen(['Modefile_Switch-211105_Constraints_Mode',num2str(j+NM_del),'.txt']);
    temp = textscan(fid, repmat(' %f',1,7), 'HeaderLines',1);
    fclose(fid);
    Modefile_temp = [temp{1},temp{2},temp{3},temp{4},temp{5},temp{6},temp{7}];    
    for i = 1:1:DOF_Rail.Cons
        pos = ModeShape.Modefile_Mapping(i,1);
        ModeShape.Cons(i,j) = Modefile_temp(pos);
    end
end

%% 0.1 Generalized Mass and Stiffness, ModeShape, ModeFreq
%%% Mass_Rail, Stiff_Rail, Damp_Rail, Pos_Node, N_Node, DOF_Rail£¨ NM_Rail, ModeFreq, ModeShape
if Choose_Check == 1
%     Target = 'L';
%     Target = 'R';
    Target = 'Cons';
    clear output_Nor
    for p = 1:1:300
        eval(['output_Nor(p,1) = (ModeShape.',Target,'(:,p))'' * Mass_Rail.',Target,'_Trans * ModeShape.',Target,'(:,p);']);
        eval(['output_Nor(p,2) = (ModeShape.',Target,'(:,p))'' * Stiff_Rail.',Target,'_Trans * ModeShape.',Target,'(:,p);']);
        output_Nor(p,3) = (2*pi*ModeFreq.All(p,2))^2*output_Nor(p,1)/output_Nor(p,2);
%         eval(['output_Nor(p,3) = (2*pi*ModeFreq.',Target,'_All(p,2))^2*output_Nor(p,1)/output_Nor(p,2);']);
    end
    
    len = 1:1:300;
    eval(['Mass_Generalized = (ModeShape.',Target,')'' * Mass_Rail.',Target,'_Trans * ModeShape.',Target,';']);
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
%%% Mass_Rail, Stiff_Rail, Damp_Rail, Pos_Node, N_Node, DOF_Rail£¨ NM_Rail, ModeFreq, ModeShape
% [Target_Mat] = Matrix_PosTrans(Source_Mat, Source_Mapping, Target_Mapping, N_DOF_Rail)
Mass_Rail.Free_Trans2 = Matrix_PosTrans(Mass_Rail.Free_Trans1, Mass_Rail.Free_Trans1_Mapping, Mass_Rail.Cons_Trans_Mapping, DOF_Rail.Cons);
Mass_Rail.Free_Trans2_Mapping = Mass_Rail.Cons_Trans_Mapping;
Stiff_Rail.Free_Trans2 = Matrix_PosTrans(Stiff_Rail.Free_Trans1, Stiff_Rail.Free_Trans1_Mapping, Stiff_Rail.Cons_Trans_Mapping, DOF_Rail.Cons);
Stiff_Rail.Free_Trans2_Mapping = Stiff_Rail.Cons_Trans_Mapping;

fields = {'Free','Free_Mapping','Free_Trans1','Free_Trans1_Mapping'};
Mass_Rail = rmfield(Mass_Rail,fields);
Stiff_Rail = rmfield(Stiff_Rail,fields);

%%% Input: Stiff_AddCons, Damp_AddCons
ConsPar_RailPad = Par_Track.ConsPar_RailPad;        % πÏœ¬Ω∫µÊ
ConsPar_PlatePad = Par_Track.ConsPar_PlatePad;      % ∞Âœ¬Ω∫µÊ, Par_Track.ConsPar_PlatePad.Baseplate_L_Switch_Stiff_Z
ConsPar_Baseplate = Par_Track.ConsPar_Baseplate;    % ª¨¥≤Ã®∞Â∏’–‘÷ß≥≈
ConsPar_LatCon =Par_Track.ConsPar_LatCon;   % √‹Ã˘∫Õ∂•Ã˙∫·œÚ÷ß≥≈∏’∂»

Target_Mapping = Mass_Rail.Cons_Trans_Mapping;
Stiff_AddCons = zeros(size(Stiff_Rail.Cons_Trans));
Damp_AddCons = zeros(size(Stiff_Rail.Cons_Trans));
pos_PlatePad_UZ_L={};	pos_PlatePad_UZ_R={}; 
pos_Rail_UX=[]; pos_Rail_UY=[]; pos_Rail_UZ=[]; pos_Baseplate_UZ=[]; pos_RailBaseplate_UZ = []; pos_BaseplateRail_UZ = [];

%%% A. Baseplate PlatePad
kk = 1;
for i = 1:1:length(Pos_Node.Sleeper)
    [pos_PlatePad_UZ_L, kk] = AddCons_I_BaseplateSearchPos(DOF_Rail.Baseplate_L_Switch, Target_Mapping, Pos_Node.Sleeper, Pos_Node.Baseplate_L_Switch, i, kk, pos_PlatePad_UZ_L);
end
kk = 1;
for i = 1:1:length(Pos_Node.Sleeper)
    [pos_PlatePad_UZ_R, kk] = AddCons_I_BaseplateSearchPos(DOF_Rail.Baseplate_R_Switch, Target_Mapping, Pos_Node.Sleeper, Pos_Node.Baseplate_R_Switch, i, kk, pos_PlatePad_UZ_R);
end

% [Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat(Type_Cons, Stiff_AddCons, Damp_AddCons, ConsPar_RailPad, ConsPar_PlatePad, Coff_ConsPar, ...
%  pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, i_PlatePad);
[Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Baseplate', Stiff_AddCons, Damp_AddCons, [], ConsPar_PlatePad.Baseplate_L_Switch, [],...
 [], [], [], pos_PlatePad_UZ_L, [], [], DOF_Rail.Cons);
[Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Baseplate', Stiff_AddCons, Damp_AddCons, [], ConsPar_PlatePad.Baseplate_R_Switch, [],...
 [], [], [], pos_PlatePad_UZ_R, [], [], DOF_Rail.Cons);

%%% B. Stock Rail
% [pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, kk] = ...
% AddCons_I_RailSearchPos(Type_Search_BaseplateNum, Target_DOF_Rail, Target_Mapping, Inp_Pos_Sleeper, Inp_Pos_Node_Rail, Inp_Pos_Node_Baseplate, i, kk,...
%  pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ);
% [Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat(Type_Cons, Stiff_AddCons, Damp_AddCons, ConsPar_RailPad, ConsPar_PlatePad, Coff_Cos,...
%  pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, Len);
%%% B1, ª˘±æπÏµ•≤‡ø€—π£¨º‚πÏº‚∂À÷Æ∫Û£¨04~34≤Ì’Ì£¨50.120~68.145m
% Left: zjbg
bools = Pos_Node.Sleeper(:,2)>=50.120 & Pos_Node.Sleeper(:,2)<=68.145;
kk = 1;
for i = 1:1:length(find(bools))
    [pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, kk] = ...
    AddCons_I_RailSearchPos('Accurate', DOF_Rail.Cons, Target_Mapping, Pos_Node.Sleeper(bools,:), Pos_Node.zjbg, Pos_Node.Baseplate_L_Switch, i, kk,...
     pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ);
end
[Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Rail', Stiff_AddCons, Damp_AddCons, ConsPar_RailPad, [], [1,1/2,1], ...
 pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, DOF_Rail.Cons);
% Right: qjbg
kk = 1;
for i = 1:1:length(find(bools))
    [pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, kk] = ...
    AddCons_I_RailSearchPos('Accurate', DOF_Rail.Cons, Target_Mapping, Pos_Node.Sleeper(bools,:), Pos_Node.qjbg, Pos_Node.Baseplate_R_Switch, i, kk,...
     pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ);
end
[Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Rail', Stiff_AddCons, Damp_AddCons, ConsPar_RailPad, [], [1,1/2,1], ...
 pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, DOF_Rail.Cons);

%%% B2. ª˘±æπÏÀ´≤‡ø€—π
% Left: zjbg
kk = 1;
for i = 1:1:length(find(~bools))
    [pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, kk] = ...
    AddCons_I_RailSearchPos('Accurate', DOF_Rail.Cons, Target_Mapping, Pos_Node.Sleeper(~bools,:), Pos_Node.zjbg, Pos_Node.Baseplate_L_Switch, i, kk,...
     pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ);
end
[Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Rail', Stiff_AddCons, Damp_AddCons, ConsPar_RailPad, [], [1,1,1], ...
 pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, DOF_Rail.Cons);
% Right: qjbg
kk = 1;
for i = 1:1:length(find(~bools))
    [pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, kk] = ...
    AddCons_I_RailSearchPos('Accurate', DOF_Rail.Cons, Target_Mapping, Pos_Node.Sleeper(~bools,:), Pos_Node.qjbg, Pos_Node.Baseplate_R_Switch, i, kk,...
     pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ);
end
[Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Rail', Stiff_AddCons, Damp_AddCons, ConsPar_RailPad, [], [1,1,1], ...
 pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, DOF_Rail.Cons);

%%% C. Switch Rail
%%% C1, º‚πÏ«∞∂À£¨≤Ì’Ì∫≈4£¨ª¨¥≤Ã®∞Âi=65
% Left: qjg_cgyg, Pos_Node.Baseplate_L_Switch: 1516, 50.120, -0.752, 0.100 (Row 324)
kk = 1;
num_Baseplate_SpecificRow = 324;
for i = 1:1:2
    [pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, kk] = ...
    AddCons_I_RailSearchPos('MinDistance', DOF_Rail.Cons, Target_Mapping, Pos_Node.Sleeper, Pos_Node.qjg_cgyg, Pos_Node.Baseplate_L_Switch(num_Baseplate_SpecificRow,:), i, kk,...
     pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ);
end
[Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Rail', Stiff_AddCons, Damp_AddCons, ConsPar_Baseplate, [], [1,0,1]/2, ...
 pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, DOF_Rail.Cons);
% Right: zjg_zgyg, Pos_Node.Baseplate_R_Switch: 3733,50.120,0.753,0.100 (Row 322)
kk = 1;
num_Baseplate_SpecificRow = 322;
for i = 1:1:2
    [pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, kk] = ...
    AddCons_I_RailSearchPos('MinDistance', DOF_Rail.Cons, Target_Mapping, Pos_Node.Sleeper, Pos_Node.zjg_zgyg, Pos_Node.Baseplate_R_Switch(num_Baseplate_SpecificRow,:), i, kk,...
     pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ);
end
[Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Rail', Stiff_AddCons, Damp_AddCons, ConsPar_Baseplate, [], [1,0,1]/2, ...
 pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, DOF_Rail.Cons);

%%% C2, ª¨¥≤Ã®∞Â÷ß≥≈£¨≤Ì’Ì∫≈5-34£¨ª¨¥≤Ã®∞Âi=66-95£¨50.770-68.145m
% Left: qjg_cgyg
bools = Pos_Node.Sleeper(:,2)>=50.770 & Pos_Node.Sleeper(:,2)<=68.145;
kk = 1;
for i = 1:1:length(find(bools))
    [pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, kk] = ...
    AddCons_I_RailSearchPos('Accurate', DOF_Rail.Cons, Target_Mapping, Pos_Node.Sleeper(bools,:), Pos_Node.qjg_cgyg, Pos_Node.Baseplate_L_Switch, i, kk,...
     pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ);
end
[Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Rail', Stiff_AddCons, Damp_AddCons, ConsPar_Baseplate, [], [1,0,1], ...
 pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, DOF_Rail.Cons);
% Right: zjg_zgyg
kk = 1;
for i = 1:1:length(find(bools))
    [pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, kk] = ...
    AddCons_I_RailSearchPos('Accurate', DOF_Rail.Cons, Target_Mapping, Pos_Node.Sleeper(bools,:), Pos_Node.zjg_zgyg, Pos_Node.Baseplate_R_Switch, i, kk,...
     pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ);
end
[Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Rail', Stiff_AddCons, Damp_AddCons, ConsPar_Baseplate, [], [1,0,1], ...
 pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, DOF_Rail.Cons);

%%% C3, ø€º˛µÊ∞Â÷ß≥≈£¨≤Ì’Ì∫≈35-58£¨ª¨¥≤Ã®∞Âi=96-end£¨68.745-82.545m
% Left: qjg_cgyg
bools = Pos_Node.Sleeper(:,2)>=68.745 & Pos_Node.Sleeper(:,2)<=82.545;
kk = 1;
for i = 1:1:length(find(bools))
    [pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, kk] = ...
    AddCons_I_RailSearchPos('Accurate', DOF_Rail.Cons, Target_Mapping, Pos_Node.Sleeper(bools,:), Pos_Node.qjg_cgyg, Pos_Node.Baseplate_L_Switch, i, kk,...
     pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ);
end
[Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Rail', Stiff_AddCons, Damp_AddCons, ConsPar_RailPad, [], [1,1,1], ...
 pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, DOF_Rail.Cons);
% Right: zjg_zgyg
kk = 1;
for i = 1:1:length(find(bools))
    [pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, kk] = ...
    AddCons_I_RailSearchPos('Accurate', DOF_Rail.Cons, Target_Mapping, Pos_Node.Sleeper(bools,:), Pos_Node.zjg_zgyg, Pos_Node.Baseplate_R_Switch, i, kk,...
     pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ);
end
[Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat('Rail', Stiff_AddCons, Damp_AddCons, ConsPar_RailPad, [], [1,1,1], ...
 pos_Rail_UX, pos_Rail_UY, pos_Rail_UZ, pos_Baseplate_UZ, pos_RailBaseplate_UZ, pos_BaseplateRail_UZ, DOF_Rail.Cons);

%%% D. √‹Ã˘¡¶∫Õ∂•Ã˙¡¶£¨LatContact_Switch_jg_jbg
% [pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY, kk] = ...
% AddCons_I_LatConSearchPos(Type_Search_BaseplateNum, Target_Mapping, Inp_Pos_Node_Rail_1, Inp_Pos_Node_Rail_2, i, kk,...
%  pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY);
% [Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat_LatCon(Stiff_AddCons, Damp_AddCons, ConsPar_LatCon, Coff_Cos,...
%  pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY, Len);

pos_Rail_11_UY = [];    pos_Rail_22_UY = [];    pos_Rail_12_UY = [];    pos_Rail_21_UY = [];

%%% D1, º‚πÏº‚∂À”Îª˘±æπÏ√‹Ã˘£¨≤Ì’Ì∫≈3.75, i=1-74, 50.000m
% Left: qjg_cgyg, zjbg
i = 1;  kk = 1;
[pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY, ~] = ...
AddCons_I_LatConSearchPos('MinDistance', Target_Mapping, Pos_Node.qjg_cgyg, Pos_Node.zjbg, i, kk,...
 pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY);
[Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat_LatCon(Stiff_AddCons, Damp_AddCons, ConsPar_LatCon, [1,1,1],...
 pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY, DOF_Rail.Cons);
% Right: zjg_zgyg, qjbg
i = 1;  kk = 1;
[pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY, ~] = ...
AddCons_I_LatConSearchPos('MinDistance', Target_Mapping, Pos_Node.zjg_zgyg, Pos_Node.qjbg, i, kk,...
 pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY);
[Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat_LatCon(Stiff_AddCons, Damp_AddCons, ConsPar_LatCon, [1,1,1],...
 pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY, DOF_Rail.Cons);

%%% D2, º‚πÏ”Îª˘±æπÏ√‹Ã˘£¨≤Ì’Ì∫≈4-22, i=2-74, 50.120-60.970m
% Left: qjg_cgyg, zjbg
bools = Pos_Node.qjg_cgyg(:,2)>=50.120 & Pos_Node.qjg_cgyg(:,2)<=60.970;
Range_i = find(bools);
kk = 1;
for i = Range_i(1):1:Range_i(end)
    [pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY, kk] = ...
    AddCons_I_LatConSearchPos('Search', Target_Mapping, Pos_Node.qjg_cgyg, Pos_Node.zjbg, i, kk,...
     pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY);
end
[Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat_LatCon(Stiff_AddCons, Damp_AddCons, ConsPar_LatCon, [1,1,1],...
 pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY, DOF_Rail.Cons);

% Right: zjg_zgyg, qjbg
bools = Pos_Node.zjg_zgyg(:,2)>=50.120 & Pos_Node.zjg_zgyg(:,2)<=60.970;
Range_i = find(bools);
kk = 1;
for i = Range_i(1):1:Range_i(end)
    [pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY, kk] = ...
    AddCons_I_LatConSearchPos('Search', Target_Mapping, Pos_Node.zjg_zgyg, Pos_Node.qjbg, i, kk,...
     pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY);
end
[Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat_LatCon(Stiff_AddCons, Damp_AddCons, ConsPar_LatCon, [1,1,1],...
 pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY, DOF_Rail.Cons);

%%% D3, º‚πÏ”Îª˘±æπÏ∂•Ã˙‘º ¯, ET6/R61 (≤Ì’Ì∫≈23.5-33.5, i=80-120, 61.845-67.845m)
% 727, 61.792, -0.67190, 0
% 807, 67.845, -0.58742, 0
% Left: qjg_cgyg, zjbg
bools = Pos_Node.qjg_cgyg(:,2)>=61.792 & Pos_Node.qjg_cgyg(:,2)<=67.845;
Range_i = find(bools);
kk = 1;
for i = Range_i(1):4:Range_i(end)
    [pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY, kk] = ...
    AddCons_I_LatConSearchPos('Search', Target_Mapping, Pos_Node.qjg_cgyg, Pos_Node.zjbg, i, kk,...
     pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY);
end
[Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat_LatCon(Stiff_AddCons, Damp_AddCons, ConsPar_LatCon, [1,1,1],...
 pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY, DOF_Rail.Cons);
% Right: zjg_zgyg, qjbg
bools = Pos_Node.zjg_zgyg(:,2)>=61.792 & Pos_Node.zjg_zgyg(:,2)<=67.845;
Range_i = find(bools);
kk = 1;
for i = Range_i(1):4:Range_i(end)
    [pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY, kk] = ...
    AddCons_I_LatConSearchPos('Search', Target_Mapping, Pos_Node.zjg_zgyg, Pos_Node.qjbg, i, kk,...
     pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY);
end
[Stiff_AddCons, Damp_AddCons] = AddCons_II_NumberSeat_LatCon(Stiff_AddCons, Damp_AddCons, ConsPar_LatCon, [1,1,1],...
 pos_Rail_11_UY, pos_Rail_22_UY, pos_Rail_12_UY, pos_Rail_21_UY, DOF_Rail.Cons);

%%% E. Check, Stiff_Rail
% Stiff_Diff = Stiff_Rail.Cons_Trans - Stiff_Rail.Free_Trans2;
% Comp_Stiff_Diff(1,1) = min(min(Stiff_Diff));
% Comp_Stiff_Diff(1,2) = max(max(Stiff_Diff));
Comp = (Stiff_Rail.Cons_Trans - Stiff_Rail.Free_Trans2) - Stiff_AddCons;
CompDiff(1,1) = min(min(Comp));
CompDiff(1,2) = max(max(Comp));
% [~,pos] = min(min(Comp))
% [~,pos] = max(max(Comp))

%%% F. Damp_Rail
%%% £°£°£°£°£° ≤‚ ‘”√
% Stiff_Rail.Cons_Trans = Stiff_Rail.Free_Trans2 + Stiff_AddCons;
%%% £°£°£°£°£°
Damp_Rail.Cons_Trans = Damp_AddCons;
Damp_Rail.Cons_Trans_Mapping = Mass_Rail.Cons_Trans_Mapping;
clear Stiff_AddCons Damp_AddCons

%% 3. DOF_Rail_start, DOF_Rail, Pos_Node, Type_Rail
DOF_Rail_start(:,1) = [0;
                       DOF_Rail.zjbg;
                       DOF_Rail.zjbg + DOF_Rail.qjg_cgyg;
                       DOF_Rail.zjbg + DOF_Rail.qjg_cgyg + DOF_Rail.zjg_zgyg;
                       DOF_Rail.zjbg + DOF_Rail.qjg_cgyg + DOF_Rail.zjg_zgyg + DOF_Rail.qjbg];
DOF_Rail_start(1:3,2) = [0;
                       DOF_Rail.Baseplate_L_Switch;
                       DOF_Rail.Baseplate_L_Switch + DOF_Rail.Baseplate_R_Switch];
DOF_Rail_start(1:2,3) = [0;
                       DOF_Rail.Cons];

Pos_Node.zjbg_Length = sqrt(diff(Pos_Node.zjbg(:,2)).^2+diff(Pos_Node.zjbg(:,3)).^2);
Pos_Node.qjbg_Length = sqrt(diff(Pos_Node.qjbg(:,2)).^2+diff(Pos_Node.qjbg(:,3)).^2);
Pos_Node.zjg_zgyg_Length = sqrt(diff(Pos_Node.zjg_zgyg(:,2)).^2+diff(Pos_Node.zjg_zgyg(:,3)).^2);
Pos_Node.qjg_cgyg_Length = sqrt(diff(Pos_Node.qjg_cgyg(:,2)).^2+diff(Pos_Node.qjg_cgyg(:,3)).^2);
