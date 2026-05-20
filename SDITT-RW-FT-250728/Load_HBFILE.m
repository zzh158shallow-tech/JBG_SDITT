function [Output_Mat, Output_Mapping, Output_Dof] = Load_HBFILE(filename)

% clc
% clear

% addpath('E:\# Flex-Rigid-20200418\FT-RW\Ansys-LinearTurnout\210608\07(009)-zjbg_L-210608')
% filename = 'HBFILE_zjbg_L_Free_MASS';

%% 导入基本参数
fid = fopen([filename,'.TXT']);
Basic = textscan(fid, repmat('%14.0f',1,5), 1, 'HeaderLines', 1, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
fclose(fid);
N_ind_col = Basic{2};
N_ind_row = Basic{3};
N_Matrix = Basic{4};
N_Load = Basic{5};

fid = fopen([filename,'.txt']);
Basic = textscan(fid, '%s%14.0f%14.0f%14.0f%14.0f', 1, 'HeaderLines', 2);
fclose(fid);
N_row = Basic{2};
N_col = Basic{3};

Output_Dof = N_row;

LS0 = 4;
% 列指针数组
fid = fopen([filename,'.txt']);
Data = textscan(fid, '%14.0f', N_ind_col, 'HeaderLines', LS0);
fclose(fid);
Ind_col = Data{1};
% 行索引数组
fid = fopen([filename,'.txt']);
Data = textscan(fid, '%14.0f', N_ind_row, 'HeaderLines', LS0+N_ind_col);
fclose(fid);
Ind_row = Data{1};
% 矩阵元素数组
fid = fopen([filename,'.txt']);
Data = textscan(fid, '%25.15f', N_Matrix, 'HeaderLines', LS0+N_ind_col+N_ind_row);
fclose(fid);
Value_Matrix = Data{1};
% 右边项数组
if LS0 == 5
    fid = fopen([filename,'.txt']);
    Data = textscan(fid, '%25.15f', N_Load, 'HeaderLines', LS0+N_ind_col+N_ind_row+N_Matrix);
    fclose(fid);
    Value_Load = Data{1};
end

%% 组合为对称矩阵
Output_Mat = zeros(N_row, N_col);

for i_col = 1:1:N_col
    p = Ind_col(i_col);
    q = Ind_col(i_col+1);    
    for i_row = p:1:q-1
        i_row_true = Ind_row(i_row);
        Output_Mat(i_row_true,i_col) = Value_Matrix(i_row);
    end
end
for i_row = 1:1:N_row    
    for i_col = 1:1:N_col
        Output_Mat(i_row,i_col) = Output_Mat(i_col,i_row);
    end
end

%% 输出Mapping文件
fid = fopen([filename,'.mapping']);
temp = textscan(fid, [repmat('%14.0f',1,2),'%s'], N_row, 'HeaderLines', 1, 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
fclose(fid);
%%% 矩阵行/列数，节点编号，自由度数目
Output_Mapping = cell(Output_Dof,3);
for i = 1:1:Output_Dof
    Output_Mapping{i,1} = temp{1,1}(i,1);
    Output_Mapping{i,2} = temp{1,2}(i,1);
    Output_Mapping(i,3) = temp{1,3}(i,1);
end
