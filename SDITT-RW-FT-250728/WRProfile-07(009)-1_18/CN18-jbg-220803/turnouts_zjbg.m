%% 道岔截面线性插值 
clc
clear

Rail_Topic_Name_output_zjbg = 'CN18_zjbg_';
Rail_Topic_Name_output_qjbg = 'CN18_qjbg_';

% Layout
Layout.zjbg = load('CN18_FAKOP_zjbg.txt');
Layout.qjbg = load('CN18_FAKOP_qjbg.txt');
Layout.zjbg = Layout.zjbg/1000;
Layout.qjbg = Layout.qjbg/1000;
Layout.zjbg = sortrows(Layout.zjbg, 1);
Layout.qjbg = sortrows(Layout.qjbg, 1);
Layout.qjbg(:,2) = Layout.qjbg(:,2)*-1;

Layout.zjbg = [-2, 0; Layout.zjbg];
Layout.qjbg = [-2, 0; Layout.qjbg];

Choose_Plot = 0;
if Choose_Plot == 1
    figure(1)
    plot(Layout.zjbg(:,1), Layout.zjbg(:,2)); hold on
    plot(Layout.qjbg(:,1), Layout.qjbg(:,2)); hold on
    grid on
    set(gca, 'ydir', 'reverse')
end

targ1 =  load('CN18-qjbg-R.txt');
targ1 =  sortrows(targ1,1);
Range_Mileage = -2:0.25:20.50;
for j = 1:1:2
    figure(j)
    plot(targ1(:,1),targ1(:,2),'b--'); hold on
    xlabel('X (m)');
    ylabel('Y (m)');
    set(gca,'ydir','reverse');
    hold on;
    grid on;
end
lgd_1 = -5;
lgd_2 = -5;

targ1x =  targ1(:,1);
targ1y =  targ1(:,2);
for i = 1:1:length(Range_Mileage)       
            
        % Layout
        Mileage = Range_Mileage(i);
        dy_zjbg = interp1(Layout.zjbg(:,1), Layout.zjbg(:,2), Mileage, 'lienar');
        dy_qjbg = interp1(Layout.qjbg(:,1), Layout.qjbg(:,2), Mileage, 'lienar');
        
        clear targ_zjbg targ_qjbg
        targ_zjbg(:,1) = targ1x+dy_zjbg;
        targ_zjbg(:,2) = targ1y;
        targ_qjbg(:,1) = targ1x+dy_qjbg;
        targ_qjbg(:,2) = targ1y;
        
        filename3_output_zjbg =  [Rail_Topic_Name_output_zjbg, num2str(Mileage), 'm'];
        fid =  fopen([filename3_output_zjbg '.txt'],'wt');
        fprintf(fid,'%12.8f %12.8f\n',targ_zjbg');
        fclose(fid);
        fun_txt2prr(filename3_output_zjbg,targ_zjbg)
        
        filename3_output_qjbg =  [Rail_Topic_Name_output_qjbg, num2str(Mileage), 'm'];
        fid =  fopen([filename3_output_qjbg '.txt'],'wt');
        fprintf(fid,'%12.8f %12.8f\n',targ_qjbg');
        fclose(fid);
        fun_txt2prr(filename3_output_qjbg,targ_qjbg)
        
        if i==1
            fid=fopen('Mileage_prr_zjbg.txt','w+');
        else
            fid = fopen('Mileage_prr_zjbg.txt','a+');
        end
        fprintf(fid,'%8.3f %s\n', Mileage+50, [filename3_output_zjbg '.prr']);
        fclose(fid);
                
        if i==1
            fid=fopen('Mileage_prr_qjbg.txt','w+');
        else
            fid = fopen('Mileage_prr_qjbg.txt','a+');
        end
        fprintf(fid,'%8.3f %s\n', Mileage+50, [filename3_output_qjbg '.prr']);
        fclose(fid);
        
        if mod(i,4)==0
            figure(1)
            hold on;
            plot(targ_zjbg(:,1),targ_zjbg(:,2),'Color',[rand(),rand(),rand()]);
            set(gca,'ydir','reverse');
            lgd_1 = [lgd_1; Mileage];
            
            figure(2)
            hold on;
            plot(targ_qjbg(:,1),targ_qjbg(:,2),'Color',[rand(),rand(),rand()]);
            set(gca,'ydir','reverse');
            lgd_2 = [lgd_2; Mileage];
        end

end

figure(1)
lgd  =   num2str(lgd_1,'%gm');%图例步长
legend(lgd);
figure(2)
lgd  =   num2str(lgd_2,'%gm');%图例步长
legend(lgd);
    
%% 单截面数据处理
% clc
% clear
% 
% path_keysec = 'C:\Users\jiayi\Desktop\Flex-Rigid-20200418\Profiles\07(009)-zjg-20200418\Key profiles\';
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
