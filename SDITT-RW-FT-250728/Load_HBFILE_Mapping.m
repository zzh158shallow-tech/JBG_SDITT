function [Output_Mapping, Output_Dof] = Load_HBFILE_Mapping(filename)

% clc
% clear

% addpath('E:\# Flex-Rigid-20200418\FT-RW\Ansys-LinearTurnout\210608\07(009)-zjbg_L-210608')
% filename = 'HBFILE_SuperEle_zjbg_L_Constraints_DAMP';

%% 导入基本参数
fid = fopen([filename,'.TXT']);
Basic = textscan(fid, repmat('%f',1,5), 'HeaderLines', 1, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
fclose(fid);
% fid = fopen([filename,'.TXT']);
% Value_Matrix = textscan(fid, repmat('%f',1,1), 'HeaderLines', 4+Basic{2}+Basic{3});
% fclose(fid);

Output_Dof = Basic{2}-1;
% Output_Mat = zeros(Output_Dof, Output_Dof);
% for i = 1:1:Output_Dof
%     Output_Mat(:,i) = Value_Matrix{1,1}(Output_Dof*(i-1)+1:Output_Dof*i);   
% end

%% 输出Mapping文件 Output_Mapping
fid = fopen([filename,'.mapping']);
temp = textscan(fid, [repmat('%14.0f',1,2),'%s'], Output_Dof, 'HeaderLines', 1, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
fclose(fid);
%%% 矩阵行/列数，节点编号，自由度数目
% Output_Mapping.Sequ = temp{1};
% Output_Mapping.Node = temp{2};
% Output_Mapping.DOF = temp{3};
Output_Mapping = cell(Output_Dof,3);
for i = 1:1:Output_Dof
    Output_Mapping{i,1} = temp{1,1}(i,1);
    Output_Mapping{i,2} = temp{1,2}(i,1);
    Output_Mapping(i,3) = temp{1,3}(i,1);
end

