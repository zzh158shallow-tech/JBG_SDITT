function Pxt_Gravity = Gravity_Load_FT(N_track, N_sum_rail, N_Rail, N_RV, NM_FW, Par_Vehicle, Par_Track, Type_Track, Type_Rail, ...
                                       RailNodes_LSR, RailNodes_LDR, RailNodes_RSR, RailNodes_RDR, RailNodes_LCR, RailNodes_RCR, ...
                                       SleeperNodes_count, SleeperNodes_pos_y)

Pxt_Gravity = zeros(N_track+NM_FW*4+N_RV,1);

% 车辆系统重力做功
Pxt_Gravity(N_track+NM_FW*4+1,1)  = 9.8*Par_Vehicle.Mw;
Pxt_Gravity(N_track+NM_FW*4+6,1)  = 9.8*Par_Vehicle.Mw;
Pxt_Gravity(N_track+NM_FW*4+11,1) = 9.8*Par_Vehicle.Mw;
Pxt_Gravity(N_track+NM_FW*4+16,1) = 9.8*Par_Vehicle.Mw;
Pxt_Gravity(N_track+NM_FW*4+21,1) = 9.8*Par_Vehicle.Mb;
Pxt_Gravity(N_track+NM_FW*4+26,1) = 9.8*Par_Vehicle.Mb;
Pxt_Gravity(N_track+NM_FW*4+31,1) = 9.8*Par_Vehicle.Mc;

% 弹性道岔系统重力做功
if strcmp(Type_Track,'Flexible Track')
    % 道岔钢轨
%     Type_Rail = {'LSR', 'LDR', 'RSR', 'RDR', 'LCR', 'RCR'};
    for i = 1:1:length(Type_Rail)
        target = eval(['RailNodes_',Type_Rail{i}]);
        target_mass = zeros(length(target),1);
        for k = 1:1:length(target)
            if k == 1
                target_mass(k,1) = target(k,1).Mass*target(k,1).Length / 2;
            elseif k == length(target)
                target_mass(k,1) = target(k-1,1).Mass*target(k-1,1).Length / 2;
            elseif ~isempty(target(k-1,1).Mass) && ~isempty(target(k,1).Mass)
                target_mass(k,1) = (target(k,1).Mass*target(k,1).Length + target(k-1,1).Mass*target(k-1,1).Length) /2;
            elseif isempty(target(k-1,1).Mass) && ~isempty(target(k,1).Mass)
                target_mass(k,1) = target(k,1).Mass*target(k,1).Length / 2;
            elseif ~isempty(target(k-1,1).Mass) && isempty(target(k,1).Mass)
                target_mass(k,1) = target(k-1,1).Mass*target(k-1,1).Length / 2;
            end
        end
        eval(['RailNodes_',Type_Rail{i},'_ElementMass = target_mass;']);
        
        for k = 1:1:length(target)
           pos = 2*(N_sum_rail(i)+k-1)+1;
           Pxt_Gravity(pos,1) = Pxt_Gravity(pos,1) + target_mass(k,1)*9.8;            
        end
    end
    
    % 道岔岔枕
    for i = 1:1:length(SleeperNodes_count)
        for k = 1:1:SleeperNodes_count(i,1)
            if k== 1
                dL = SleeperNodes_pos_y(i,k+1)-SleeperNodes_pos_y(i,k);
            elseif k == SleeperNodes_count(i,1)
                dL = SleeperNodes_pos_y(i,k)-SleeperNodes_pos_y(i,k-1);
            else
                dL = SleeperNodes_pos_y(i,k+1)-SleeperNodes_pos_y(i,k-1);
            end
            pos = 4*N_Rail+2*(sum(SleeperNodes_count(1:i-1))+(k-1))+1;
            Pxt_Gravity(pos,1) = Pxt_Gravity(pos,1) + dL/2*Par_Track.Ms*9.8;
        end
    end
end
