%% CN18道岔直尖轨：根据已知顶宽与降低值的关系，求解不同位置的降低值

clc
clear

%%% 顶宽，降低值，
A = ...
    [2	16	0.485
    6.5	NaN	1.12
    22	4.4	3.307
    28	2.8	4.11
    37.8	0.1	5.881
    38	0	5.913
    54	0	8.292
    71	0	10.243];

bools = true(size(A,1), 1);
bools(2) = false;
Range_Width = 2:1:71;
for i = 1:1:length(Range_Width)
    Width = Range_Width(i);
    Width_ReduVal_spline(i,:) = [Width, interp1(A(bools,1), A(bools,2), Width, 'spline')];
    Width_ReduVal_linear(i,:) = [Width, interp1(A(bools,1), A(bools,2), Width, 'linear')];
end

figure(1)
plot(Width_ReduVal_spline(:,1), Width_ReduVal_spline(:,2)); hold on
plot(Width_ReduVal_linear(:,1), Width_ReduVal_linear(:,2)); hold on
plot(A(:,1), A(:,2), '*'); hold on
grid on

%% CN18道岔直尖轨顶宽与里程之间的关系
clc
clear

%%% 顶宽，降低值，距尖轨尖端距离
A = ...
    [0	NaN	0.2
    2	16	0.485
    6.5	NaN	1.12
    22	4.4	3.307
    28	2.8	4.11
    37.8	0.1	5.881
    38	0	5.913
    54	0	8.292
    71	0	10.243];

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

Range_Mileage = (-1:0.1:10.243)';
for i = 1:1:length(Range_Mileage)
    Mileage = Range_Mileage(i);
    
    if Mileage < -1.369
        y0 = 0;
    else
%         R = 1100;
        R = 1100-1.435;
        Beta = asin( (Mileage+1.369)/R );
        y0= R*(1-cos(Beta));
    end
    
    y_guage = interp1(Layout.zjbg(:,1), Layout.zjbg(:,2), Mileage, 'linear');    
    Mileage_Width(i,:) = [Mileage, y_guage+y0];    
    
end

figure(1); clf
plot(Mileage_Width(:,1), Mileage_Width(:,2)*1000); hold on
plot(A(:,3), A(:,1), '*'); hold on
grid on

%% 已知顶宽，插值得到对应的尖轨里程
A = [A; 15.32 NaN 2.30];
A = sortrows(A, 1);

Range_Width = [1:1:71, 37.8]';
Range_Width = sortrows(Range_Width,1);
for i = 1:1:length(Range_Width)
       Width = Range_Width(i);
       if Width<15
           Width_Mileage(i,:) = [Width, interp1(A(:,1), A(:,3), Width, 'spline')];
%            Width_Mileage(i,:) = [Width, interp1(A(:,1), A(:,3), Width, 'linear')];
       else
           Width_Mileage(i,:) = [Width, interp1(Mileage_Width(:,2)*1000, Mileage_Width(:,1), Width, 'linear')];
       end
end

figure(1)
plot(Width_Mileage(:,2), Width_Mileage(:,1)); hold on
plot(A(:,3), A(:,1), '+'); hold on
grid on






