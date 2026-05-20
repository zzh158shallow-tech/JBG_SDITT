function [RailNodes_LSR, RailNodes_LDR, RailNodes_RSR, RailNodes_RDR, RailNodes_LCR, RailNodes_RCR, RailNodes_num_v2, RailNodes_pos_v2] = ...
Get_FEM_Rail_P2_Diverging(Density_r, RailNodes_LSR, RailNodes_LDR, RailNodes_RSR, RailNodes_RDR, RailNodes_LCR, RailNodes_RCR, RailNodes_num_v1)

%% LSR, RDR
bools_LSR_RDR = zeros(length(RailNodes_LSR),1);
for i = 3:2:length(RailNodes_LSR)
    bools_LSR_RDR(i) = 1;
end
RailNodes_LSR(bools_LSR_RDR==0,:) = [];
RailNodes_RDR(bools_LSR_RDR==0,:) = [];

for k = 1:1:length(RailNodes_LSR)-1
    if ~isempty(RailNodes_LSR(k,1).Area)
        loc = [RailNodes_LSR(k,1).location; RailNodes_LSR(k+1,1).location];
        RailNodes_LSR(k,1).Length = sqrt((loc(2,1)-loc(1,1))^2+(loc(2,2)-loc(1,2))^2+(loc(2,3)-loc(1,3))^2);
        RailNodes_LSR(k,1).Mass = RailNodes_LSR(k,1).Area * Density_r;
    end
end

for k = 1:1:length(RailNodes_RDR)-1
    if ~isempty(RailNodes_RDR(k,1).Area)
        loc = [RailNodes_RDR(k,1).location; RailNodes_RDR(k+1,1).location];
        RailNodes_RDR(k,1).Length = sqrt((loc(2,1)-loc(1,1))^2+(loc(2,2)-loc(1,2))^2+(loc(2,3)-loc(1,3))^2);
        RailNodes_RDR(k,1).Mass = RailNodes_RDR(k,1).Area * Density_r;
    end
end

%% LCR, RCR
bools_LCR_RCR = zeros(length(RailNodes_LCR),1);
for i = 1:2:length(RailNodes_LCR)
    bools_LCR_RCR(i) = 1;
end
RailNodes_LCR(bools_LCR_RCR==0,:) = [];
RailNodes_RCR(bools_LCR_RCR==0,:) = [];

for k = 1:1:length(RailNodes_LCR)-1
    if ~isempty(RailNodes_LCR(k,1).Area)
        loc = [RailNodes_LCR(k,1).location; RailNodes_LCR(k+1,1).location];
        RailNodes_LCR(k,1).Length = sqrt((loc(2,1)-loc(1,1))^2+(loc(2,2)-loc(1,2))^2+(loc(2,3)-loc(1,3))^2);
        RailNodes_LCR(k,1).Mass = RailNodes_LCR(k,1).Area * Density_r;
    end
end

for k = 1:1:length(RailNodes_RCR)-1
    if ~isempty(RailNodes_RCR(k,1).Area)
        loc = [RailNodes_RCR(k,1).location; RailNodes_RCR(k+1,1).location];
        RailNodes_RCR(k,1).Length = sqrt((loc(2,1)-loc(1,1))^2+(loc(2,2)-loc(1,2))^2+(loc(2,3)-loc(1,3))^2);
        RailNodes_RCR(k,1).Mass = RailNodes_RCR(k,1).Area * Density_r;
    end
end

%% LDR
bools_LDR = ones(length(RailNodes_LDR),1);
for i = 158+1:2:278-1
    bools_LDR(i,1) = 0;
end
for i = 350+1:2:length(RailNodes_LDR)-1
    bools_LDR(i,1) = 0;
end

RailNodes_LDR(bools_LDR==0,:) = [];

for k = 1:1:length(RailNodes_LDR)-1
    if ~isempty(RailNodes_LDR(k,1).Area)
        loc = [RailNodes_LDR(k,1).location; RailNodes_LDR(k+1,1).location];
        RailNodes_LDR(k,1).Length = sqrt((loc(2,1)-loc(1,1))^2+(loc(2,2)-loc(1,2))^2+(loc(2,3)-loc(1,3))^2);
        RailNodes_LDR(k,1).Mass = RailNodes_LDR(k,1).Area * Density_r;
    end
end

%% RSR
bools_RSR = ones(length(RailNodes_RSR),1);
for i = 158+1:2:294-1
    bools_RSR(i,1) = 0;
end
for i = 295+1:2:length(RailNodes_RSR)-1
    bools_RSR(i,1) = 0;
end

RailNodes_RSR(bools_RSR==0,:) = [];

for k = 1:1:length(RailNodes_RSR)-1
    if ~isempty(RailNodes_RSR(k,1).Area)
        loc = [RailNodes_RSR(k,1).location; RailNodes_RSR(k+1,1).location];
        RailNodes_RSR(k,1).Length = sqrt((loc(2,1)-loc(1,1))^2+(loc(2,2)-loc(1,2))^2+(loc(2,3)-loc(1,3))^2);
        RailNodes_RSR(k,1).Mass = RailNodes_RSR(k,1).Area * Density_r;
    end
end

%%
RailNodes_num_v1_temp = RailNodes_num_v1;
RailNodes_num_v1_temp(467:507,2) = RailNodes_num_v1_temp(467:507,5);
RailNodes_num_v1_temp(:,5) = [];
RailNodes_num_v2 = zeros(size(RailNodes_num_v1_temp));
RailNodes_pos_v2 = cell(size(RailNodes_num_v1_temp));
for i = 1:1:size(RailNodes_num_v1_temp,2)
    if i==1
        target = RailNodes_LSR;
    elseif i==2
        target = RailNodes_LDR;
    elseif i==3
        target = RailNodes_RSR;
    elseif i==4
        target = RailNodes_RDR;
    elseif i==5
        target = RailNodes_LCR;
    elseif i==6
        target = RailNodes_RCR;
    end
    for j = 1:1:length(target)
       [~,m] = min(abs(target(j,1).number-RailNodes_num_v1_temp(:,i)));
       RailNodes_num_v2(m,i) = target(j,1).number;
       RailNodes_pos_v2{m,i} = target(j,1).location;
    end
end

RailNodes_num_v2(1:2,:) = [];
RailNodes_pos_v2(1:2,:) = [];

