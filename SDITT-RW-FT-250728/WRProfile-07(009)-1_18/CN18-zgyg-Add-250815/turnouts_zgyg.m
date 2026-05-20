%% 道岔截面线性插值 
clc
clear

%%%%%% 输入参数 %%%%%%
% Mileage_origin = [54.547, 54.597, 54.647, 54.697, 54.747, 54.797, 54.847]';
Mileage_origin = [54.547, 54.697, 54.847]';
for kk = 1:1:length(Mileage_origin)
    Name{kk} = [num2str(Mileage_origin(kk)), 'm'];
end
Rail_Topic_Name_input = 'CN18_zgyg_';
Rail_Topic_Name_output = 'CN18_zgyg_';
d_Mileage = 50;        %%% 实际尖轨尖端里程与图中尖轨尖端距离

for i = 1:1:length(Mileage_origin)-1
%     judgement = 0;
%     while judgement == 0
        %%%%%% 两端截面参数 %%%%%%
        filename1_input =  strcat([Rail_Topic_Name_input Name{i}]);
        filename2_input =  strcat([Rail_Topic_Name_input Name{i+1}]);        
        
        Mileage_1 = Mileage_origin(i,1);
        Mileage_2 = Mileage_origin(i+1,1);

        z1 = Mileage_1;
        z2 = Mileage_2;
        
        %%%%%% 处理CAD导出的数据 %%%%%%
        targ1 =  load([filename1_input,'.txt']);
        targ1 =  sortrows(targ1,1);
%         targ1(:,1) =  (targ1(:,1))/1000;
%         targ1(:,2) =  targ1(:,2)/1000*-1;
        targ2 =  load([filename2_input,'.txt']);
        targ2 =  sortrows(targ2,1);
%         targ2(:,1) =  (targ2(:,1))/1000;
%         targ2(:,2) =  targ2(:,2)/1000*-1;
        
        figure(1)
        hold off;
        plot(targ1(:,1),targ1(:,2),'b--',targ2(:,1),targ2(:,2),'r--');
        title('Fig.1 Section of Various Top Width');
        xlabel('X (m)');
        ylabel('Y (m)');
        set(gca,'ydir','reverse');
        hold on;
        grid on;
        
        %%%%%% 重新输出典型截面数据 %%%%%%
        filename1_output =  [filename1_input,'.txt'];
        filename2_output =  [filename2_input,'.txt'];
%         filename1_output =  strcat(Rail_Topic_Name_output,num2str(Width_Range(i)));
%         filename2_output =  strcat(Rail_Topic_Name_output,num2str(Width_Range(i+1)));
%         fid = fopen([filename1_output '.txt'],'wt'); %新保存源截面文件
%         fprintf(fid,'%12.8f %12.8f\n',targ1');
%         fclose(fid);
%         fun_txt2prr(filename1_output,targ1)
%         fid = fopen([filename2_output '.txt'],'wt'); %新保存源截面文件
%         fprintf(fid,'%12.8f %12.8f\n',targ2');
%         fclose(fid);
%         fun_txt2prr(filename2_output,targ2)
        
        %%% 输出里程和prr.文件的关系
        if i==1
            Flag = 'w+';
        else
            Flag = 'a+';
        end
        fid=fopen('Mileage_prr_2.txt', Flag);
        fprintf(fid,'%8.3f %s\n', Mileage_1+d_Mileage, filename1_output);
%         fclose(fid);
        
        %%%%%% 计算中间截面插值数据 %%%%%%
        targ1x =  targ1(:,1);
        targ1y =  targ1(:,2);
        targ2x =  targ2(:,1);
        targ2y =  targ2(:,2);
        
%         dz3 = 0.05/4;
        dz3 = 0.03;

        for z3 =  Mileage_1+dz3 : dz3 : Mileage_2-dz3    %%%确定步长
            targ3 =  zeros(size(targ1));
%             tar = Mileage_origin;
%             bools = false(size(tar,1),1);
%             bools(1:end-1) = diff(tar(:,1))==0;
%             tar(bools,:) = [];
% %             Mileage_3 = interp1(tar(:,1), tar(:,2), z3, 'spline');
%             Mileage_3 = interp1(tar(:,1), tar(:,2), z3, 'linear');
            Mileage_3 = z3;
            for i2 =  1:1:length(targ2)
                targ3(i2,1) =  targ2x(i2)-(z2-z3)*(targ2x(i2)-targ1x(i2))/(z2-z1);
                targ3(i2,2) =  targ2y(i2)-(z2-z3)*(targ2y(i2)-targ1y(i2))/(z2-z1);
            end
            filename3_output =  [Rail_Topic_Name_output, num2str(z3), 'm'];
            fid_profile =  fopen([filename3_output '.txt'],'wt');
            fprintf(fid_profile,'%12.8f %12.8f\n',targ3');
            fclose(fid_profile);
            fun_txt2prr(filename3_output,targ3)
            
%             fid = fopen('Mileage_prr_1.txt','a+');
            fprintf(fid,'%8.3f %s\n', Mileage_3+d_Mileage, [filename3_output '.txt']);
%             fclose(fid);
            
            figure(1)
            hold on;
            plot(targ3(:,1),targ3(:,2),'Color',[rand(),rand(),rand()]);
            set(gca,'ydir','reverse');
        end
              
        %%%%%% 图例图例 %%%%%%
        lgd  =   num2str([z1, z2, floor(z1)+1:dz3:ceil(z2)-1]','%gmm');%图例步长
        legend(lgd);        
%         judgement =  input('是否同意廓形插值? (0/1)  =   ');

%     end
    
    if i==length(Mileage_origin)-1
%         fid=fopen('Mileage_prr_1.txt','a+');
        fprintf(fid,'%8.3f %s\n', Mileage_2+d_Mileage, filename2_output);
        fclose(fid);
    end
end

%% 单截面数据处理
% clc
% clear
% 
% path_keysec = 'H:\# Mango\2025-07-07 CARS-Turnout_V450\SDITT-RW-FT_Trail-CR400BF\WRProfile-07(009)-1_18\07(009)-cxg-20250801\Key profiles\';
% l4 = '72.2';
% Rail_Topic_Name_input = ['cxg-', l4, '-20250801'];
% Rail_Topic_Name_output = '07(009)-cxg-20250801-';
% targ1 =  load([path_keysec, Rail_Topic_Name_input,'.txt']);
% targ1 =  sortrows(targ1,1);
% targ1(:,1) = targ1(:,1)/1000*1;
% targ1(:,2) = targ1(:,2)/1000*-1;
% 
% filename_output = strcat([Rail_Topic_Name_output l4]);
% fid = fopen([filename_output '.txt'],'wt'); %新保存源截面文件
% fprintf(fid,'%12.8f %12.8f\n',targ1');
% fclose(fid); 
% fun_txt2prr(filename_output,targ1)
% 
% figure(2);
% hold off;
% plot(targ1(:,1),targ1(:,2),'b');
% set(gca,'ydir','reverse');
% grid on;
% legend(Rail_Topic_Name_input);
