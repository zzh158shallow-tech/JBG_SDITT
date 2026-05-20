function [Target_Mat] = Matrix_PosTrans(Source_Mat, Source_Mapping, Target_Mapping, N_DOF_Rail)

% Source_Mapping = Mass_Rail.zjbg_Cons_Mapping;
% Source_Mat = Mass_Rail.zjbg_Cons;
% Target_Mapping = Mass_Rail.zjbg_Mapping;

% Source_Mat = Mass_Rail.Right_Free_Trans1;
% Source_Mapping = Mass_Rail.Right_Free_Trans1_Mapping;
% Target_Mapping = Mass_Rail.Right_Mapping;
% N_DOF_Rail = DOF_Rail.Right;

%% Method I
Source_2_Target = zeros(N_DOF_Rail,1);
for i = 1:1:N_DOF_Rail
    bools = (cell2mat(Source_Mapping(:,2)) == Target_Mapping{i,2}) & ...
            (strcmp(Source_Mapping(:,3), Target_Mapping{i,3}));        
    Source_2_Target(i,1) = find(bools);
end
Target_Mat_v1 = Source_Mat(Source_2_Target,:);
Target_Mat = Target_Mat_v1(:,Source_2_Target);

%% Method II
% Target_Mat_v1 = zeros(size(Source_Mat));
% Target_Mat = zeros(size(Source_Mat));
% for i = 1:1:N_DOF_Rail
%     bools = (cell2mat(Source_Mapping(:,2)) == Target_Mapping{i,2}) & ...
%             (strcmp(Source_Mapping(:,3), Target_Mapping{i,3}));
%     Target_Mat_v1(i,:) = Source_Mat(bools,:);
% end
% for i = 1:1:N_DOF_Rail
%     bools = (cell2mat(Source_Mapping(:,2)) == Target_Mapping{i,2}) & ...
%             (strcmp(Source_Mapping(:,3), Target_Mapping{i,3}));
%     Target_Mat(:,i) = Target_Mat_v1(:,bools);
% end