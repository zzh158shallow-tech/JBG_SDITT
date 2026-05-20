%% 道岔截面线性插值 
clear
clc
% path_keysec = 'C:\Users\jiayi\Desktop\Flex-Rigid-20200418\Profiles\07(009)-qjbg-20200418\Key profiles\';
path_keysec = 'F:\2021-10-10 Braking in S&C\SDITT_FW_PT_MS_220709_SolidSlab_Park\WRProfile-07(009)-1_18\07(009)-qjbg-20200418\Key profiles\';

%%%%%% 输入参数 %%%%%%
Name = {'R'};
Width_Range = [0]';
Mileage_origin = [0 3 15 26.8 27.4 35 40 50 60 65 72.2 83.33 95.18 107.94; 
                  0 0.575 2.889 5.161 5.280 6.569 7.304 8.614 9.753 10.269 10.962 12 13 14]';
% Mileage_origin = [0 3 15 26.8 27.4 35 40 50 60 65 72.2 ; 
%                   0 0.575 2.889 5.161 5.280 6.569 7.304 8.614 9.753 10.269 10.962]';
Rail_Topic_Name_input = 'jbg-';
Rail_Topic_Name_output_qjbg = '07(009)-qjbg-20200418-';

% for i = 1:1:length(Name)-3
% for i = length(Width_Range)-1:1:length(Width_Range)-1
for i = 1:1:1
    judgement = 0;
    while judgement == 0
        %%%%%% 两端截面参数 %%%%%%
        filename1_input =  strcat([Rail_Topic_Name_input Name{i}]);
%         filename2_input =  strcat([Rail_Topic_Name_input Name{i+1}]);        
        
        l1 =  Width_Range(i);          %起始截面顶宽
        Mileage_1 = Mileage_origin(1,2);
%         l2 =  Width_Range(i+1);        %结束截面顶宽
%         [~,p] = min(abs(l1-Mileage_origin(:,1)));
%         [~,q] = min(abs(l2-Mileage_origin(:,1)));
%         Mileage_1 = Mileage_origin(p,2);
%         Mileage_2 = Mileage_origin(q,2);
        
        %%%%%% 处理CAD导出的数据 %%%%%%
        targ1 =  load([path_keysec, filename1_input,'.txt']);
        targ1 =  sortrows(targ1,1);
        targ1(:,1) =  (targ1(:,1))/1000;
        targ1(:,2) =  targ1(:,2)/1000*-1;
%         targ2 =  load([path_keysec,filename2_input,'.txt']);
%         targ2 =  sortrows(targ2,1);
%         targ2(:,1) =  (targ2(:,1))/1000;
%         targ2(:,2) =  targ2(:,2)/1000*-1;
        
        figure(1)
        hold off;
        plot(targ1(:,1),targ1(:,2),'b--');
%         plot(targ1(:,1),targ1(:,2),'b--',targ2(:,1),targ2(:,2),'r--');
        title('Fig.1 Section of Various Top Width');
        xlabel('X (m)');
        ylabel('Y (m)');
        set(gca,'ydir','reverse');
        hold on;
        grid on;
        
        %%%%%% 重新输出典型截面数据 %%%%%%
        filename1_output =  strcat(Rail_Topic_Name_output_qjbg,num2str(Width_Range(i)));
%         filename2_output =  strcat(Rail_Topic_Name_output_qjbg,num2str(Width_Range(i+1)));
        fid = fopen([filename1_output '.txt'],'wt'); %新保存源截面文件
        fprintf(fid,'%12.8f %12.8f\n',targ1');
        fclose(fid);
        fun_txt2prr(filename1_output,targ1)
%         fid = fopen([filename2_output '.txt'],'wt'); %新保存源截面文件
%         fprintf(fid,'%12.8f %12.8f\n',targ2');
%         fclose(fid);
%         fun_txt2prr(filename2_output,targ2)
        
        %%% 输出里程和prr.文件的关系
        if i==1
            fid=fopen('Mileage_prr.txt','w+');
%         else
%             fid=fopen('Mileage_prr.txt','a+');
%         end
        fprintf(fid,'%8.3f %s\n', Mileage_1+50, [filename1_output '.prr']);
%         fprintf(fid,'%8.3f %s\n', Mileage_2+50, [filename2_output '.prr']);
        fclose(fid);
        end
        
        %%%%%% 计算中间截面插值数据 %%%%%%
        targ1x =  targ1(:,1);
        targ1y =  targ1(:,2);
%         targ2x =  targ2(:,1);
%         targ2y =  targ2(:,2);
        
        %%% 方式1：根据顶宽插值       
%         if l2 <= 71
            dz3 = 1;
%         else
%             dz3 = 3;
%         end

        Range_z3 = [1:dz3:72, 83.33 95.18 107.94]
        for z3 =  Range_z3                                          %%%确定步长
%         for z3 =  round(l1)+1 : dz3 : ceil(l2)-1     %%%确定步长
            targ3 =  zeros(size(targ1));
            if z3 <=26.8
                Mileage_3 = interp1(Mileage_origin(:,1), Mileage_origin(:,2), z3, 'linear');
            else
                Mileage_3 = interp1(Mileage_origin(:,1), Mileage_origin(:,2), z3, 'spline');
            end
            for j =  1:1:length(targ1)
                targ3(j,1) =  targ1x(j)+z3/1000;
                targ3(j,2) =  targ1y(j);
%                 targ3(j,1) =  targ2x(j)-(l2-z3)*(targ2x(j)-targ1x(j))/(l2-l1);
%                 targ3(j,2) =  targ2y(j)-(l2-z3)*(targ2y(j)-targ1y(j))/(l2-l1);
            end
            filename3_output =  strcat(Rail_Topic_Name_output_qjbg,num2str(z3));
            fid =  fopen([filename3_output '.txt'],'wt');
            fprintf(fid,'%12.8f %12.8f\n',targ3');
            fclose(fid);
            fun_txt2prr(filename3_output,targ3)
            
            fid = fopen('Mileage_prr.txt','a+');
            fprintf(fid,'%8.3f %s\n', Mileage_3+50, [filename3_output '.prr']);
            fclose(fid);
            
            figure(1)
            hold on;
            plot(targ3(:,1),targ3(:,2),'Color',[rand(),rand(),rand()]);
            set(gca,'ydir','reverse');
        end
              
        %%%%%% 图例图例 %%%%%%
        lgd  =   num2str([l1,1:dz3:72]','%gmm');%图例步长
        legend(lgd);
        % filename_fig =  strcat('Re12c1_jbg_R_',num2str(z1),'_',num2str(z2),'.fig');
        % saveas(1,filename_fig,'fig');
        
        judgement =  input('是否同意廓形插值? (0/1)  =   ');

    end
    
%     fid=fopen('Mileage_prr.txt','a+');
%     fprintf(fid,'%8.3f %s\n', Mileage_2+50, [filename2_output '.prr']);
%     fclose(fid);
end

% %%% 单截面数据处理
% clc
% clear
% 
% path_keysec = 'C:\Users\jiayi\Desktop\Flex-Rigid-20200418\Profiles\07(009)-qjbg-20200418\Key profiles\';
% Rail_Topic_Name_input = 'jbg-';
% Rail_Topic_Name_output = 'jbg-20200418-';
% l4 = 'R';
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
