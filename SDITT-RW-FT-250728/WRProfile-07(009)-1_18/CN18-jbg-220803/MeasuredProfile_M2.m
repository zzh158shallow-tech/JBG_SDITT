clc
clear

Type_Rail = {'qjbg', 'zjg'};

% Width
Width.qjbg = (0:1:72)';
Width.qjbg = sortrows(Width.qjbg);
Width.zjg = [3:1:72, 27.4, 72.2]';
Width.zjg = sortrows(Width.zjg);

% æ‡º‚πÏº‚∂Àæ‡¿Î, º‚πÏ∂•øÌ,  µ≤‚ΩµµÕ÷µ, ÷±º‚πÏ…Ëº∆ΩµµÕ÷µ, ΩµµÕ÷µ±‰ªØ
% TIrr = [0, 0, 0.9, 23, -22.1
%            0.575, 3, -0.4, 14, -14.4
%            2.889, 15, 1.9, 3, -1.1
%            7.304, 40, 1.7, 0, 1.7
%            9.753, 60, 0, 0, 0
%            10.962, 72.2, 0, 0, 0];

% æ‡º‚πÏº‚∂Àæ‡¿Î	º‚πÏ∂•øÌ	ª˘±æπÏΩµµÕ¡ø	º‚πÏΩµµÕ¡ø
TIrr = [0	0	0	0
           0.575	3	0	0
           2.889	15	1.1	0
           4.623	24	0	0
           7.304	40	0	1.7
           8.614	50	0	0
           10.962551541229750	72.2	0	0];

%% Width corresponding with the mileage
Layout_qjbg = [0, 0
                         5.161, 26.7924e-3
                         60.440, 1704.7621e-3
                         117.045, 4849.4953e-3];

% Draft
% if xx(j) < 5.161 || xx(j) > 60.440
%     yy(j,1) = interp1(Layout_qjbg(:,1),Layout_qjbg(:,2),xx(j),'linear');
% else
%     R = 1100-1.435/2;
%     Beta = asin((xx(j)+542.0151e-3)/R);
%     yy(j,1) = R*(1-cos(Beta))+0.012;
% end
% 
% syms xx yy
% R = 1100-1.435/2;
% Beta = asin((xx+542.0151e-3)/R);
% Equ_1 = R*(1-cos( asin((xx+542.0151e-3)/R) ))+0.012 - yy;
% solve(Equ_1)

%% Profile_M2
for kk = 1:1:2
    figure(11+kk); clf
    Width_Range = [];
    Target_Width = Width.(Type_Rail{kk});

    for i = 1:1:length(Target_Width)
        % Mileage
        if Target_Width(i)<26.7924
            Mileage.(Type_Rail{kk})(i,1) = interp1(Layout_qjbg(:,2), Layout_qjbg(:,1), Target_Width(i)/1000, 'linear');
        else
            R = 1100-1.435/2;
            func = @(xx)R*(1-cos( asin((xx+542.0151e-3)/R) ))+0.012 - Target_Width(i)/1000;
            options = optimset('Display','off');
            Coor_0 = 0;
            Mileage.(Type_Rail{kk})(i,1) = fsolve(func, Coor_0, options);
        end
        Mileage.(Type_Rail{kk})(i,2) = Target_Width(i);

        % Load file
        filename = ['07(009)-', Type_Rail{kk}, '-20200418-', num2str(Target_Width(i)), '.txt'];
        data_ori = load(filename);

        % dz
        dz.(Type_Rail{kk})(i,1) = Target_Width(i);
        dz.(Type_Rail{kk})(i,2) = interp1(TIrr(:,1), TIrr(:,2+kk)/1000, Mileage.(Type_Rail{kk})(i,1), 'linear');

        data(:,1) = data_ori(:,1);
        data(:,2) = data_ori(:,2)+dz.(Type_Rail{kk})(i,2);

        % Output file
        filename_oup = ['07(009)-', Type_Rail{kk}, '-220728-', num2str(Target_Width(i)), '-M2'];
        fid = fopen([filename_oup, '.txt'],'wt'); %–¬±£¥Ê‘¥Ωÿ√ÊŒƒº˛
        fprintf(fid,'%12.8f %12.8f\n',data');
        fclose(fid);
        fun_txt2prr(filename_oup,data)

        fid = fopen(['Mileage_prr_', Type_Rail{kk}, '.txt'], 'a+');
        fprintf(fid,'%8.3f %s\n', Mileage.(Type_Rail{kk})(i,1)+50, [filename_oup '.prr']);
        fclose(fid);

        % Plot
        if mod(i,2)==0 && Target_Width(i)<=50
            figure(11+kk);
            hold on
            plot(data(:,1), data(:,2), 'Color',[rand(),rand(),rand()]);
            set(gca,'ydir','reverse'); grid on
            Width_Range = [Width_Range; Target_Width(i)];
        end
    end

    figure(11+kk);
    legend(num2str(Width_Range));

end

figure(10); clf
plot(Mileage.qjbg(:,1), Mileage.qjbg(:,2)); hold on
plot(Mileage.zjg(:,1), Mileage.zjg(:,2), '--'); hold on
grid on
set(gca, 'ydir', 'reverse');

figure(11); clf
plot(Mileage.qjbg(:,1), -dz.qjbg(:,2)); hold on
plot(Mileage.zjg(:,1), dz.zjg(:,2), '--'); hold on
% plot(Mileage.qjbg(:,2), -dz.qjbg(:,2)); hold on
% plot(Mileage.zjg(:,2), dz.zjg(:,2), '--'); hold on
grid on
set(gca, 'ydir', 'reverse');



