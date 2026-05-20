function [InpPar, Par_FW] = Load_Rotation_FW(InpPar, CutFreq_FW)

% load FW_SDITT_230506_Mass21.mat
load FW_SDITT_230506_Mass21_v2.mat
load Par_FW_Rotation.mat

InpPar.Pos_Node.FW = Pos_Node.FW;
Type_i1 = {'AxleBox', 'NRC', 'WBack', 'WOut'};
Type_i2= {'L', 'R'};
for i1 = 1:1:length(Type_i1)
    T1 = Type_i1{i1};
    for i2 = 1:1:length(Type_i2)
        T2 = Type_i2{i2};
        T = [T1, '_', T2];        
        InpPar.Pos_Node.(T1).(T2) = Pos_Node.(T);
    end
end

InpPar.ModeShape_Mapping.FW = Mass_FW.Free_Mapping;
InpPar.DOF_Node.FW = DOF_FW.Free;

Type_i1= {'L', 'R'};
Type_i2 = {'Mid', 'Front', 'Rear'};
for i1 = 1:1:length(Type_i1)
    T1 = Type_i1{i1};
    for i2 = 1:1:length(Type_i2)
        T2 = Type_i2{i2};
        T = ['Tread_', T1, '_', T2];
        InpPar.Pos_Node.Tread.(T1).(T2) = Par_FW.(T);
        if isfield(Par_FW, T)
            Par_FW = rmfield(Par_FW, T);
        end
    end
end
Par_FW = rmfield(Par_FW, 'Tread_Mid');

InpPar.ModeFreq.FW_All = ModeFreq.FW_All;
InpPar.ModeFreq.FW = ModeFreq.FW;
bools = InpPar.ModeFreq.FW(:,2)<=CutFreq_FW;
InpPar.ModeFreq.FW = InpPar.ModeFreq.FW(bools,:);
InpPar.NM_FW = length(find(bools));

InpPar.ModeShape.FW_All = ModeShape.FW_All;
InpPar.ModeShape.FW = ModeShape.FW(:,bools);
InpPar.ModeShape.Tread_L_Mid = ModeShape.Tread_L_Mid(:,bools);
InpPar.ModeShape.Tread_R_Mid = ModeShape.Tread_R_Mid(:,bools);
% InpPar.ModeShape.Tread_Mid = ModeShape.Tread_Mid(:,bools);

Par_FW.Ele_List = Ele_List;
Par_FW.Matrix_J = Par_FW_Rotation.Matrix_J(bools,bools);
Par_FW.Matrix_E = Par_FW_Rotation.Matrix_E(bools,bools);
Par_FW.Matrix_L = Par_FW_Rotation.Matrix_L(bools,1);
Par_FW.Matrix_G = Par_FW_Rotation.Matrix_G(bools,bools);
Par_FW.Matrix_C = Par_FW.Matrix_G*Par_FW.Matrix_G + Par_FW.Matrix_G*Par_FW.Matrix_J - ...
                                   Par_FW.Matrix_J*Par_FW.Matrix_G - Par_FW.Matrix_J*Par_FW.Matrix_J - Par_FW.Matrix_E;

Par_FW.Matrix_F_SumAll = Par_FW_Rotation.Matrix_F_SumAll;
Type_i1 = {'L', 'R'};
Type_i2 = {'Front', 'Rear'};
for i1 = 1:1:2
    T1 = Type_i1{i1};
    for i2 = 1:1:2
        T2 = Type_i2{i2};
        Tar = Par_FW.Matrix_F_SumAll.(T1).(T2).Matrix_F;
        for i3 = 1:1:size(Tar,1)
            Par_FW.Matrix_F_SumAll.(T1).(T2).Matrix_F{i3,1} = Tar{i3,1}(bools,:);
        end
    end
end

clear Pos_Node Mass_FW Stiff_FW DOF_FW 
clear Ele_List NM_FW ModeFreq ModeShape 

