%% 道岔截面线性插值 
clc
clear

Rail_Topic_Name_output_zgyg = 'CN18_zgyg_';

% Layout
Layout.zgyg = load('CN18_zgyg.txt');
Layout.zgyg = Layout.zgyg/1000;
Layout.zgyg = sortrows(Layout.zgyg, 1);
Layout.zgyg(:,2) = (Layout.zgyg(:,2)+0.7175)*-1;

Layout.zgyg = [-2, 0; Layout.zgyg];

Layout.zgyg_dz = [0 0
53.347	0
53.477	0.2
54.147	1.4
54.695	2.3
54.747	2.2
55.425	0.8
55.843	0
58.000	0];
Layout.zgyg_dz(:,2) = Layout.zgyg_dz(:,2)/-1000;

Choose_Plot = 0;
if Choose_Plot == 1
    figure(1)
    plot(Layout.zgyg(:,1), Layout.zgyg(:,2)); hold on
    plot(Layout.qjbg(:,1), Layout.qjbg(:,2)); hold on
    grid on
    set(gca, 'ydir', 'reverse')
end

targ1 =  load('CN18-qjbg-R.txt');
targ1 =  sortrows(targ1,1);
Range_Mileage = [52.5:0.1:53.347, 53.347 : 0.05 : 56];
for j = 1:1:1
    figure(j)
    plot(targ1(:,1),targ1(:,2),'b--'); hold on
    xlabel('X (m)');
    ylabel('Y (m)');
    set(gca,'ydir','reverse');
    hold on;
    grid on;
end
lgd_1 = -5;

targ1x =  targ1(:,1);
targ1y =  targ1(:,2);
for i = 1:1:length(Range_Mileage)       
            
        % Layout
        Mileage = Range_Mileage(i);
        dy_zgyg = interp1(Layout.zgyg(:,1), Layout.zgyg(:,2), Mileage, 'lienar');
        dz_zgyg = interp1(Layout.zgyg_dz(:,1), Layout.zgyg_dz(:,2), Mileage, 'lienar');
        
        clear targ_zgyg targ_qjbg
        targ_zgyg(:,1) = targ1x+dy_zgyg;
        targ_zgyg(:,2) = targ1y+dz_zgyg;
        
        filename3_output_zgyg =  [Rail_Topic_Name_output_zgyg, num2str(Mileage), 'm'];
        fid =  fopen([filename3_output_zgyg '.txt'],'wt');
        fprintf(fid,'%12.8f %12.8f\n',targ_zgyg');
        fclose(fid);
        fun_txt2prr(filename3_output_zgyg, targ_zgyg)        
       
        if i==1
            fid=fopen('Mileage_prr_zgyg.txt','w+');
        else
            fid = fopen('Mileage_prr_zgyg.txt','a+');
        end
        fprintf(fid,'%8.3f %s\n', Mileage+50, [filename3_output_zgyg '.prr']);
        fclose(fid);                
        
        if mod(i,2)==0
            figure(1)
            hold on;
            plot(targ_zgyg(:,1),targ_zgyg(:,2),'Color',[rand(),rand(),rand()]);
            set(gca,'ydir','reverse');
            lgd_1 = [lgd_1; Mileage];
        end

end

figure(1)
lgd  =   num2str(lgd_1,'%gm');%图例步长
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
