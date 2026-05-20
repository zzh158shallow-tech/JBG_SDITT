%% 里程文件转入SIMPACK

% rail.r.prof.file (        1 ,       $RWR_F_WS_F_Rail_CN18_zjbg         ) = 'jbg-20200418-R.prr'    ! Right Profiles
% rail.r.prof.file (        2 ,       $RWR_F_WS_F_Rail_CN18_zjbg         ) = 'jbg-20200418-R.prr'    ! Right Profiles
% rail.r.prof.s (           1 ,       $RWR_F_WS_F_Rail_CN18_zjbg         ) = 0.00000000000000000E+00 ! Right Profile Positions
% rail.r.prof.s (           2 ,       $RWR_F_WS_F_Rail_CN18_zjbg         ) = 1.00000000000000000E+03 ! Right Profile Positions

clc
clear

addpath('G:\# Rail-Wheel Turnouts\2021-11-29 CM 2022\CN-AS-18');
fileanme_Excel = 'CN-AS-18道岔里程文件.xlsx';

clear Data temp
% SheetName = 'zgxg-R';
SheetName = 'zjg_zgyg-R';
Data = readtable(fileanme_Excel, 'Sheet', SheetName, 'ReadVariableNames', true);
% Mileage = table2array( Data(:,2) );
% PrrFile = table2cell( Data(:,3) );
Mileage = table2array( Data(1:end,3) );
PrrFile = table2cell( Data(1:end,4) );

filename_Rail = '$RWR_F_WS_F_Rail_CN18_zjg_zgyg';

fid=fopen('RailProf_SIMP.txt','w+');
for i = 1:1:size(Mileage, 1)
    Text_1 = ['rail.r.prof.file (        ', num2str(i), ' ,       ', filename_Rail, '         ) = ''', PrrFile{i,1}, '''    ! Right Profiles'];
    fprintf(fid,'%s\n', Text_1);
end
for i = 1:1:size(Mileage, 1)
    Text_1 = ['rail.r.prof.s (           ', num2str(i), ' ,       ', filename_Rail, '         ) = ', num2str(Mileage(i,1)), '    ! Right Profile Positions'];
    fprintf(fid,'%s\n', Text_1);
end
fclose(fid);