%% 道岔截面线性插值 
clear
clc
path_keysec = 'H:\# Mango\2025-07-07 CARS-Turnout_V450\database-CR450BF-GY\database\profile\07(009)-zgyg-20250720\Key profiles\';

%%%%%% 输入参数 %%%%%%
Name = {'D', 'E'};
Width_Range = [9.100 	49.80]';
Mileage_origin = [9.100 	49.80; 
                             53.176 54.419]';
Rail_Topic_Name_input = 'zgyg-';
Rail_Topic_Name_output = '07(009)-zgyg-20250720-cxg-';
d_Mileage = 50;        %%% 实际尖轨尖端里程与图中尖轨尖端距离

for i = 1:1:1
    judgement = 0;
    while judgement == 0
        %%%%%% 两端截面参数 %%%%%%
        filename1_input =  strcat([Rail_Topic_Name_input Name{i}]);
        filename2_input =  strcat([Rail_Topic_Name_input Name{i+1}]);        
        
        z1 =  Width_Range(i);          %起始截面顶宽
        z2 =  Width_Range(i+1);        %结束截面顶宽
        [~,p] = min(abs(z1-Mileage_origin(:,1)));
        [~,q] = min(abs(z2-Mileage_origin(:,1)));
        Mileage_1 = Mileage_origin(p,2);
        Mileage_2 = Mileage_origin(q,2);
        
        %%%%%% 处理CAD导出的数据 %%%%%%
        targ1 =  load([path_keysec, filename1_input,'.txt']);
        targ1 =  sortrows(targ1,1);
        targ1(:,1) =  (targ1(:,1))/1000;
        targ1(:,2) =  targ1(:,2)/1000*-1;
        targ2 =  load([path_keysec,filename2_input,'.txt']);
        targ2 =  sortrows(targ2,1);
        targ2(:,1) =  (targ2(:,1))/1000;
        targ2(:,2) =  targ2(:,2)/1000*-1;
        
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
        filename1_output =  strcat(Rail_Topic_Name_output,num2str(Width_Range(i)));
        filename2_output =  strcat(Rail_Topic_Name_output,num2str(Width_Range(i+1)));
        fid = fopen([filename1_output '.txt'],'wt'); %新保存源截面文件
        fprintf(fid,'%12.8f %12.8f\n',targ1');
        fclose(fid);
        fun_txt2prr(filename1_output,targ1)
        fid = fopen([filename2_output '.txt'],'wt'); %新保存源截面文件
        fprintf(fid,'%12.8f %12.8f\n',targ2');
        fclose(fid);
        fun_txt2prr(filename2_output,targ2)
        
        %%% 输出里程和prr.文件的关系
        if i==1
            fid=fopen('Mileage_prr_2.txt','w+');
            fprintf(fid,'%8.3f %s\n', Mileage_1+d_Mileage, [filename1_output '.prr']);
            fclose(fid);
        elseif i==5 || i==10
            fid=fopen('Mileage_prr_2.txt','a+');
            fprintf(fid,'%8.3f %s\n', Mileage_1+d_Mileage, [filename1_output '.prr']);
            fclose(fid);
        end
        
        %%%%%% 计算中间截面插值数据 %%%%%%
        targ1x =  targ1(:,1);
        targ1y =  targ1(:,2);
        targ2x =  targ2(:,1);
        targ2y =  targ2(:,2);
        
        %%% 方式1：根据顶宽插值       
%         if z2 > 18.5 && z2 < 25
%             dz3 = 0.5;
%         else
            dz3 = 1;
%         end

        for z3 =  floor(z1)+dz3 : dz3 : ceil(z2)-dz3    %%%确定步长
            targ3 =  zeros(size(targ1));
            tar = Mileage_origin;
            bools = false(size(tar,1),1);
            bools(1:end-1) = diff(tar(:,1))==0;
            tar(bools,:) = [];
            Mileage_3 = interp1(tar(:,1), tar(:,2), z3, 'linear');
            for i2 =  1:1:length(targ2)
                targ3(i2,1) =  targ2x(i2)-(z2-z3)*(targ2x(i2)-targ1x(i2))/(z2-z1);
                targ3(i2,2) =  targ2y(i2)-(z2-z3)*(targ2y(i2)-targ1y(i2))/(z2-z1);
            end
            filename3_output =  strcat(Rail_Topic_Name_output,num2str(z3));
            fid =  fopen([filename3_output '.txt'],'wt');
            fprintf(fid,'%12.8f %12.8f\n',targ3');
            fclose(fid);
            fun_txt2prr(filename3_output,targ3)
            
            fid = fopen('Mileage_prr_2.txt','a+');
            fprintf(fid,'%8.3f %s\n', Mileage_3+d_Mileage, [filename3_output '.prr']);
            fclose(fid);
            
            figure(1)
            hold on;
            plot(targ3(:,1),targ3(:,2),'Color',[rand(),rand(),rand()]);
            set(gca,'ydir','reverse');
        end
              
        %%%%%% 图例图例 %%%%%%
        lgd  =   num2str([z1, z2, (z1-mod(z1,dz3)+dz3) : dz3 : (z2-mod(z2,dz3))]','%gmm');%图例步长
        legend(lgd);        
        judgement =  input('是否同意廓形插值? (0/1)  =   ');

    end
    
    fid=fopen('Mileage_prr_2.txt','a+');
    fprintf(fid,'%8.3f %s\n', Mileage_2+d_Mileage, [filename2_output '.prr']);
    fclose(fid);
end

% %%% 单截面数据处理
% clc
% clear
% 
% path_keysec = 'C:\Users\jiayi\Desktop\Flex-Rigid-20200418\Profiles\07(009)-cxg-20200422\Key profiles\';
% Rail_Topic_Name_input = 'cxg-';
% Rail_Topic_Name_output = '07(009)-cxg-20200422-';
% l4 = 'H';
% filename4_input = strcat([Rail_Topic_Name_input l4]);
% targ1 =  load([path_keysec, filename4_input,'.txt']);
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
% legend(filename4_input);
