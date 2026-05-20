%% 从有限元ANSYS中提取钢轨梁模型的质量、刚度矩阵
function [Mass_Rail, Stiff_Rail, Damp_Rail,...
          ModeFreq_Rail, ModeShape_Rail, Pos_Node, N_Node_Rail, NM_Rail, DOF_Rail, DOF_Rail_start] = ...
         Get_Mass_Stiff_Ansys(Par_Track)

Choose_Check = 0;

%% 1.Mass_Rail, Stiff_Rail, Damp_Rail, DOF_Rail
[Mass_Rail.zjbg_Free, Mass_Rail.zjbg_Free_Mapping, DOF_Rail.zjbg_Free] = Load_HBFILE('HBFILE_zjbg_L_Free_MASS');
[Stiff_Rail.zjbg_Free, Stiff_Rail.zjbg_Free_Mapping, DOF_Rail.zjbg_Free] = Load_HBFILE('HBFILE_zjbg_L_Free_STIFF');
[Mass_Rail.zjbg, Mass_Rail.zjbg_Mapping, DOF_Rail.zjbg] = Load_HBFILE('HBFILE_zjbg_L_Constraints_MASS');
[Stiff_Rail.zjbg, Stiff_Rail.zjbg_Mapping, DOF_Rail.zjbg] = Load_HBFILE('HBFILE_zjbg_L_Constraints_STIFF');
[Damp_Rail.zjbg, Damp_Rail.zjbg_Mapping, DOF_Rail.zjbg] = Load_HBFILE_Sub('HBFILE_SuperEle_zjbg_L_Constraints_DAMP');

%% 2. N_Node_Rail, Pos_Node
fid = fopen('NodePos_zjbg_L.txt');
format = '%f %f %f %f %f %f %f';
pos_Node_load = textscan(fid, format);
fclose(fid);
pos_Node_zjbg_ori = [pos_Node_load{1,1} pos_Node_load{1,2} pos_Node_load{1,3} pos_Node_load{1,4} pos_Node_load{1,5} pos_Node_load{1,6} pos_Node_load{1,7}];
Pos_Node.zjbg = pos_Node_zjbg_ori(pos_Node_load{1,4}==0,:);
Pos_Node.zjbg = sortrows(Pos_Node.zjbg,2);
Pos_Node.Baseplate_L = pos_Node_zjbg_ori(pos_Node_load{1,4}~=0,:);
Pos_Node.Baseplate_L = sortrows(Pos_Node.Baseplate_L,[2,3]);
N_Node_Rail.zjbg = size(Pos_Node.zjbg,1);
N_Node_Rail.Baseplate_L = size(Pos_Node.Baseplate_L,1);

%% 3. Transfer the Constrainted Matrices based on the Sepcified DOF Order





%% 4. NM_Rail, ModeFreq_Rail, ModeShape_Rail
%%% ModeFreq_Rail
NM_del = 6;
NM_Rail.zjbg = 2100-NM_del;
fid = fopen('Modefile_zjbg_L_FreqModal.txt');
ModeFreq_temp = textscan(fid, '%f %f %f', NM_Rail.zjbg, 'HeaderLines',NM_del);
fclose(fid);
ModeFreq_Rail.zjbg = [ModeFreq_temp{1,1} ModeFreq_temp{1,2} ModeFreq_temp{1,3}];
ModeFreq_Rail.zjbg(:,1) = ModeFreq_Rail.zjbg(:,1)-NM_del;

%%% 确定第k阶 Modeshape 矩阵(N*1)元素位置与 Modefile 文件中振型结果位置的映射关系
fid = fopen('Modefile_zjbg_L_Constraints_Mode100.txt');
temp = textscan(fid, repmat(' %f',1,7), 'HeaderLines',1);
fclose(fid);
Modefile_temp = [temp{1},temp{2},temp{3},temp{4},temp{5},temp{6},temp{7}];

Target = Mass_Rail.zjbg_Mapping;
for i = 1:1:size(Target,1)
    pos = find(Modefile_temp(:,1)==Target{i,2});
    if strcmp(Target{i,3}, 'UX')
        ModeShape_Rail.Modefile_mapping_zjbg(i,1) = pos+size(Modefile_temp,1)*1;
    elseif strcmp(Target{i,3}, 'UY')
        ModeShape_Rail.Modefile_mapping_zjbg(i,1) = pos+size(Modefile_temp,1)*2;
    elseif strcmp(Target{i,3}, 'UZ')
        ModeShape_Rail.Modefile_mapping_zjbg(i,1) = pos+size(Modefile_temp,1)*3;
    elseif strcmp(Target{i,3}, 'ROTX')
        ModeShape_Rail.Modefile_mapping_zjbg(i,1) = pos+size(Modefile_temp,1)*4;
    elseif strcmp(Target{i,3}, 'ROTY')
        ModeShape_Rail.Modefile_mapping_zjbg(i,1) = pos+size(Modefile_temp,1)*5;
    else
        ModeShape_Rail.Modefile_mapping_zjbg(i,1) = pos+size(Modefile_temp,1)*6;
    end
end

%%% 振型文件 ModeShape_Rail
ModeShape_Rail.zjbg = zeros(DOF_Rail.zjbg, NM_Rail.zjbg);
for j = 1:1:NM_Rail.zjbg
    fid = fopen(['Modefile_zjbg_L_Constraints_Mode',num2str(j+NM_del),'.txt']);
    temp = textscan(fid, repmat(' %f',1,7), 'HeaderLines',1);
    fclose(fid);
    Modefile_temp = [temp{1},temp{2},temp{3},temp{4},temp{5},temp{6},temp{7}];    
    for i = 1:1:DOF_Rail.zjbg
        pos = ModeShape_Rail.Modefile_mapping_zjbg(i,1);
        ModeShape_Rail.zjbg(i,j) = Modefile_temp(pos);
    end
end

%% 0.1 Generalized Mass and Stiffness, ModeShape_Rail, ModeFreq_Rail
if Choose_Check == 1
    clear output_Nor
    for p = 1:1:NM_Rail.zjbg
        output_Nor(p,1) = (ModeShape_Rail.zjbg(:,p))' * Mass_Rail.zjbg * ModeShape_Rail.zjbg(:,p);
        output_Nor(p,2) = (ModeShape_Rail.zjbg(:,p))' * Stiff_Rail.zjbg * ModeShape_Rail.zjbg(:,p);
        output_Nor(p,3) = (2*pi*ModeFreq_Rail.zjbg(p,2))^2*output_Nor(p,1)/output_Nor(p,2);
    end
    
    Mass_Generalized = (ModeShape_Rail.zjbg)' * Mass_Rail.zjbg * ModeShape_Rail.zjbg;
    
    figure(1); clf
    len = 1:1:NM_Rail.zjbg;
    plot(len, output_Nor(:,1), len, output_Nor(:,3));
    grid on
end

%% 0.2 Compare matrices between that in the free and constrainted state: Mass_Rail, Stiff_Rail, Pos_Node
if Choose_Check == 1
    %%% Delete the Constrained DOF of Free Matrices
    p = min(Pos_Node.Baseplate_L(:,1));
    q = max(Pos_Node.Baseplate_L(:,1));
    bools = (cell2mat(Mass_Rail.zjbg_Free_Mapping(:,2)) >= p & cell2mat(Mass_Rail.zjbg_Free_Mapping(:,2)) <= q) & ...
        (~strcmp(Mass_Rail.zjbg_Free_Mapping(:,3),'UZ'));
    Mass_Rail.zjbg_Free_Trans1 = Mass_Rail.zjbg_Free;
    Mass_Rail.zjbg_Free_Trans1(bools,:) = [];
    Mass_Rail.zjbg_Free_Trans1(:,bools) = [];
    Mass_Rail.zjbg_Free_Trans1_Mapping = Mass_Rail.zjbg_Free_Mapping;
    Mass_Rail.zjbg_Free_Trans1_Mapping(bools,:) = [];
    Stiff_Rail.zjbg_Free_Trans1 = Stiff_Rail.zjbg_Free;
    Stiff_Rail.zjbg_Free_Trans1(bools,:) = [];
    Stiff_Rail.zjbg_Free_Trans1(:,bools) = [];
    Stiff_Rail.zjbg_Free_Trans1_Mapping = Stiff_Rail.zjbg_Free_Mapping;
    Stiff_Rail.zjbg_Free_Trans1_Mapping(bools,:) = [];
    
    %%% Mass Matrix, Mass_Rail, Pos_Node
    % [Target_Mat] = Matrix_PosTrans(Source_Mat, Source_Mapping, Target_Mapping, N_DOF_Rail)
    Mass_Rail.zjbg_Free_Trans2 = Matrix_PosTrans(Mass_Rail.zjbg_Free_Trans1, Mass_Rail.zjbg_Free_Trans1_Mapping, Mass_Rail.zjbg_Mapping, DOF_Rail.zjbg);
    Mass_Rail.zjbg_Free_Trans2_Mapping = Mass_Rail.zjbg_Mapping;
    
    %%% Stiff Matrix, Stiff_Rail, Pos_Node
    Stiff_Rail.zjbg_Free_Trans2 = Matrix_PosTrans(Stiff_Rail.zjbg_Free_Trans1, Stiff_Rail.zjbg_Free_Trans1_Mapping, Stiff_Rail.zjbg_Mapping, DOF_Rail.zjbg);
    Stiff_Rail.zjbg_Free_Trans2_Mapping = Stiff_Rail.zjbg_Mapping;
    
    RailPad.Stiff_X = 20e6;
    RailPad.Stiff_Y = 20e6;
    RailPad.Stiff_Z = 275e6;
    RailPad.Damp_X = 20e3;
    RailPad.Damp_Y = 20e3;
    Baseplate.Stiff_Z = 27.5e6/5;
    Baseplate.Damp_Z = 20e3/5;
    
    Target_Mapping = Mass_Rail.zjbg_Mapping;
    Stiff_AddCons = zeros(size(Stiff_Rail.zjbg));
    Damp_AddCons = zeros(size(Damp_Rail.zjbg));
    for i = 1:1:length(Pos_Node.Baseplate_L)
        num = Pos_Node.Baseplate_L(i,1);
        bools = (cell2mat(Target_Mapping(:,2))==num);
        Stiff_AddCons(bools,bools) = Stiff_AddCons(bools,bools) + Baseplate.Stiff_Z;
        Damp_AddCons(bools,bools)  = Damp_AddCons(bools,bools)  + Baseplate.Damp_Z;
    end
    for i = 1:4:length(Pos_Node.zjbg)
        num_Rail = Pos_Node.zjbg(i,1);
        bools = ((Pos_Node.Baseplate_L(:,2)==Pos_Node.zjbg(i,2)) & (Pos_Node.Baseplate_L(:,3)==Pos_Node.zjbg(i,3)));
        num_Baseplate = Pos_Node.Baseplate_L(bools,1);
        
        bools_RailDof_UX = (cell2mat(Target_Mapping(:,2))==num_Rail) & (strcmp(Target_Mapping(:,3),'UX'));
        bools_RailDof_UY = (cell2mat(Target_Mapping(:,2))==num_Rail) & (strcmp(Target_Mapping(:,3),'UY'));
        bools_RailDof_UZ = (cell2mat(Target_Mapping(:,2))==num_Rail) & (strcmp(Target_Mapping(:,3),'UZ'));
        bools_BaseplateDof_UZ = (cell2mat(Target_Mapping(:,2))==num_Baseplate) & (strcmp(Target_Mapping(:,3),'UZ'));
        
        Stiff_AddCons(bools_RailDof_UX,bools_RailDof_UX) = Stiff_AddCons(bools_RailDof_UX,bools_RailDof_UX) + RailPad.Stiff_X;
        Stiff_AddCons(bools_RailDof_UY,bools_RailDof_UY) = Stiff_AddCons(bools_RailDof_UY,bools_RailDof_UY) + RailPad.Stiff_Y;
        Stiff_AddCons(bools_RailDof_UZ,bools_RailDof_UZ) = Stiff_AddCons(bools_RailDof_UZ,bools_RailDof_UZ) + RailPad.Stiff_Z;
        Stiff_AddCons(bools_BaseplateDof_UZ,bools_BaseplateDof_UZ) = Stiff_AddCons(bools_BaseplateDof_UZ,bools_BaseplateDof_UZ) + RailPad.Stiff_Z;
        Stiff_AddCons(bools_RailDof_UZ,bools_BaseplateDof_UZ) = Stiff_AddCons(bools_RailDof_UZ,bools_BaseplateDof_UZ) - RailPad.Stiff_Z;
        Stiff_AddCons(bools_BaseplateDof_UZ,bools_RailDof_UZ) = Stiff_AddCons(bools_BaseplateDof_UZ,bools_RailDof_UZ) - RailPad.Stiff_Z;
        Damp_AddCons(bools_RailDof_UX,bools_RailDof_UX) = Damp_AddCons(bools_RailDof_UX,bools_RailDof_UX) + RailPad.Damp_X;
        Damp_AddCons(bools_RailDof_UY,bools_RailDof_UY) = Damp_AddCons(bools_RailDof_UY,bools_RailDof_UY) + RailPad.Damp_Y;
    end
    
    Comp = (Stiff_Rail.zjbg - Stiff_Rail.zjbg_Free_Trans2) - Stiff_AddCons;
    min(min(Comp))
    max(max(Comp))
    
    %%% Damp Matrix, Damp_Rail, Pos_Node
    Damp_Rail.zjbg_Trans1 = Matrix_PosTrans(Damp_Rail.zjbg, Damp_Rail.zjbg_Mapping, Mass_Rail.zjbg_Mapping, DOF_Rail.zjbg);
    Damp_Rail.zjbg_Trans1_Mapping = Mass_Rail.zjbg_Mapping;
    
    Comp = Damp_Rail.zjbg_Trans1 - Damp_AddCons;
    min(min(Comp))
    max(max(Comp))    
end

%% 3. DOF_Rail_start
DOF_Rail_start = [0; ...
                  DOF_Rail.zjbg; ...
                  DOF_Rail.zjbg; ...
                  DOF_Rail.zjbg; ...
                  DOF_Rail.zjbg+DOF_Rail.qjbg];
              