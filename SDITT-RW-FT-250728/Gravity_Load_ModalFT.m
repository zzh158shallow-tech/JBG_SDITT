function [Pxt_Gravity, Pxt_Track] = Gravity_Load_ModalFT(InpPar, Par_Vehicle)

% global InpPar.N_track InpPar.N_RV InpPar.NM_FW Par_Vehicle InpPar.Type_Rail_All InpPar.Type_Baseplate InpPar.Type_SpaceIron
% global InpPar.Pos_Node InpPar.N_Node InpPar.DOF_Node InpPar.ModeShape

Pxt_Gravity = zeros(InpPar.N_track+InpPar.NM_FW*4+InpPar.N_RV,1);

%% Vehicle Bodies
pos = InpPar.N_track+InpPar.NM_FW*4;
Pxt_Gravity(pos+1,1)  = 9.81*Par_Vehicle.Mw;
Pxt_Gravity(pos+6,1)  = 9.81*Par_Vehicle.Mw;
Pxt_Gravity(pos+11,1) = 9.81*Par_Vehicle.Mw;
Pxt_Gravity(pos+16,1) = 9.81*Par_Vehicle.Mw;
Pxt_Gravity(pos+21,1) = 9.81*Par_Vehicle.Mb;
Pxt_Gravity(pos+26,1) = 9.81*Par_Vehicle.Mb;
Pxt_Gravity(pos+31,1) = 9.81*Par_Vehicle.Mc;

% Pxt_Gravity(pos+1,1)  = 9.81*(Par_Vehicle.Mw+Par_Vehicle.Mb/2+Par_Vehicle.Mc/4);
% Pxt_Gravity(pos+6,1)  = 9.81*(Par_Vehicle.Mw+Par_Vehicle.Mb/2+Par_Vehicle.Mc/4);
% Pxt_Gravity(pos+11,1) = 9.81*(Par_Vehicle.Mw+Par_Vehicle.Mb/2+Par_Vehicle.Mc/4);
% Pxt_Gravity(pos+16,1) = 9.81*(Par_Vehicle.Mw+Par_Vehicle.Mb/2+Par_Vehicle.Mc/4);

%% Rail Mass
% InpPar .Pos_Node, RailPro, BaseplatePro, InpPar.ModeShape
load('RailPro.mat')
load('BaseplatePro.mat')
Density = 7850;

clear Body_Gravity
% Body_Gravity, Pxt_Track, Pxt_Gravity
if strcmp(InpPar.Choose_Turnout, '07(009)')
    Range_i = 1:1:length(InpPar.Type_Rail_All);
elseif strcmp(InpPar.Choose_Turnout, 'CN18')
    Range_i = [1:1:5, 7, 8];
end
for i1 = Range_i
    T1 = InpPar.Type_Rail_All{i1};
    T1_L = [T1, '_Length'];
    len = InpPar.N_Node.(T1);
    
    % Body_Gravity: Area, Length, Mass
    % Area
    Body_Gravity.(T1)(:,1) = RailPro .(T1)(:,5);
    if strcmp(InpPar.Choose_Turnout, 'CN18') && i1==5
        T1_add = InpPar.Type_Rail_All{i1+1};
        Body_Gravity.(T1) = [Body_Gravity.(T1); RailPro.(T1_add)(2:end,5)];
    end
    % Length
    Tar_Length = InpPar.Pos_Node.(T1_L);
%     if strcmp(InpPar.Choose_Turnout, 'CN18') && i1==7
%         Tar_Length = [sqrt(diff(RailPro.cghjg(1:2,2))^2+diff(RailPro.cghjg(1:2,3))^2); Tar_Length];
%     end
    Body_Gravity.(T1)(1,2) = Tar_Length(1,1)/2;
    Body_Gravity.(T1)(end,2) = Tar_Length(end,1)/2;
    Body_Gravity.(T1)(2:end-1,2) = (Tar_Length(1:end-1,1)+Tar_Length(2:end,1))/2;
    % Mass
    Body_Gravity.(T1)(:,3) = Body_Gravity.(T1)(:,1).*Body_Gravity.(T1)(:,2)*Density;
    % Pxt_Track
    Pxt_Track.(T1) = zeros(InpPar.DOF_Node.(T1),1);
    pos = (2:4:(len-1)*4+2);
    Pxt_Track.(T1)(pos,1) = Body_Gravity.(T1)(:,3).*9.81;
    % Pxt_Gravity    
    Pxt_Gravity(1:InpPar.N_track,1) = Pxt_Gravity(1:InpPar.N_track,1) + InpPar.ModeShape.(T1)' * Pxt_Track.(T1);    
end

%% Baseplate Mass
% BaseplatePro
% Body_Gravity: Height, Width, Length, Mass
if isfield(InpPar.ModeShape, InpPar.Type_Baseplate{1})
    for i1 = 1:1:length(InpPar.Type_Baseplate)
        T1 = InpPar.Type_Baseplate{i1};
        T1_L = [T1, '_Div_Length'];
        i2_max = size(InpPar.Pos_Node.([T1, '_cell']),1);
        t_2 = 0;

        for i2 = 1:1:i2_max
            i3_max = cell2mat(InpPar.Pos_Node.([T1, '_cell'])(i2,4));
            num_baseplate = cell2mat(InpPar.Pos_Node.([T1, '_cell'])(i2,1));
            if num_baseplate<0
                T2 = ['B_m', num2str(abs(num_baseplate))];
            else
                T2 = ['B_', num2str(num_baseplate)];
            end
            Body_Gravity.(T1).(T2) = zeros(i3_max,4);
            % Length
            Body_Gravity.(T1).(T2)(1,3) = InpPar.Pos_Node.(T1_L).(T2)(1,1)/2;
            Body_Gravity.(T1).(T2)(end,3) = InpPar.Pos_Node.(T1_L).(T2)(end,1)/2;
            Body_Gravity.(T1).(T2)(2:end-1,3) = (InpPar.Pos_Node.(T1_L).(T2)(1:end-1,1)+InpPar.Pos_Node.(T1_L).(T2)(2:end,1))/2;
            % Height
            pos = t_2+1:1:t_2+i3_max;
            Body_Gravity.(T1).(T2)(1,1) = BaseplatePro.ND.(T1)(pos(1),5);
            Body_Gravity.(T1).(T2)(end,1) = BaseplatePro.ND.(T1)(pos(end),5);
            Body_Gravity.(T1).(T2)(2:end-1,1) = ( BaseplatePro.ND.(T1)(pos(2:end-1),5)+BaseplatePro.ND.(T1)(pos(1:end-2),5) )/2;
            % Width
            Body_Gravity.(T1).(T2)(:,2) = BaseplatePro.ND.(T1)(pos,6);
            % Mass
            Body_Gravity.(T1).(T2)(:,4) = Body_Gravity.(T1).(T2)(:,1).*Body_Gravity.(T1).(T2)(:,2).*Body_Gravity.(T1).(T2)(:,3).*Density;
            % Pxt_Track
            Pxt_Track.(T1).(T2) = zeros(i3_max*2,1);
            pp = 1 : 2 : 2*(i3_max-1)+1;
            Pxt_Track.(T1).(T2)(pp,1) = Body_Gravity.(T1).(T2)(:,4).*9.81;
            % Pxt_Gravity
            Pxt_Gravity(1:InpPar.N_track,1) = Pxt_Gravity(1:InpPar.N_track,1) + InpPar.ModeShape.(T1).(T2)' * Pxt_Track.(T1).(T2);
            t_2 = t_2+i3_max;
        end
    end
end

%% SpaceIron Mass
% Body_Gravity: Height, Width, Length, Massd
if isfield(InpPar.ModeShape, InpPar.Type_SpaceIron{1})
    for i1 = 1:1:length(InpPar.Type_SpaceIron)
        T1 = InpPar.Type_SpaceIron{i1};
        i2_max = length(fieldnames(InpPar.Pos_Node.(T1)));

        for i2 = 1:1:i2_max
            T2 = ['S_', num2str(i2)];
            Body_Gravity.(T1).(T2) = zeros(InpPar.N_Node.(T1).(T2),4);

            % Col 3: Length
            Body_Gravity.(T1).(T2)(1,3) = (InpPar.Pos_Node.(T1).(T2)(2,3)-InpPar.Pos_Node.(T1).(T2)(1,3))/2;
            Body_Gravity.(T1).(T2)(end,3) = (InpPar.Pos_Node.(T1).(T2)(end,3)-InpPar.Pos_Node.(T1).(T2)(end-1,3))/2;
            Body_Gravity.(T1).(T2)(2:end-1,3) = (InpPar.Pos_Node.(T1).(T2)(3:end,3)-InpPar.Pos_Node.(T1).(T2)(1:end-2,3))/2;
            % Col 1-2: Height, Width
            Body_Gravity.(T1).(T2)(:,1:2) = InpPar.Pos_Node.(T1).(T2)(:,5:6);
            % Col 4: Mass
            Body_Gravity.(T1).(T2)(:,4) = Body_Gravity.(T1).(T2)(:,1).*Body_Gravity.(T1).(T2)(:,2).*Body_Gravity.(T1).(T2)(:,3).*Density;
            % Pxt_Track
            len = InpPar.N_Node.(T1).(T2)*6-4;
            Pxt_Track.(T1).(T2) = zeros(len,1);
            pp = [2, 7:6:7+6*(InpPar.N_Node.(T1).(T2)-3), len-2];
            Pxt_Track.(T1).(T2)(pp,1) = Body_Gravity.(T1).(T2)(:,4).*9.81;
            % Pxt_Gravity
            Pxt_Gravity(1:InpPar.N_track,1) = Pxt_Gravity(1:InpPar.N_track,1) + InpPar.ModeShape.(T1).(T2)' * Pxt_Track.(T1).(T2);
        end
    end
end

