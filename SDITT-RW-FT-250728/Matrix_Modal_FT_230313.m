%% 基于模态叠加法，构建 M_track, K_track, C_track
function [M_track, K_track, C_track, DR, InpPar] = Matrix_Modal_FT_230313(InpPar, CutFreq_FT, DR_Normal, DR_Typical)
% 2021/06/19修改：适应 Left/Right 两侧轨道部件的约束模态
% 2021/07/05修改：通过阻尼比计算阻尼矩阵
% 2023/03/13修改：道岔 Abaqus 模型 T33

bools = InpPar.ModeFreq.FT_All(:,2) <= CutFreq_FT;
InpPar.N_track = length(find(bools));
InpPar.ModeFreq.FT = InpPar.ModeFreq.FT_All(bools,:);

if strcmp(InpPar.Choose_Turnout, '07(009)')
    Range_i = 1:1:length(InpPar.Type_Rail_All);
elseif strcmp(InpPar.Choose_Turnout, 'CN18')
    Range_i = [1:1:5, 7, 8];
end
for i1 = Range_i
    InpPar.ModeShape.(InpPar.Type_Rail_All{i1})(:,~bools) = [];
end

if isfield(InpPar.ModeShape, InpPar.Type_Baseplate{1})
    for i1 = 1:1:length(InpPar.Type_Baseplate)
        T1 = InpPar.Type_Baseplate{i1};
        i2_max = size(InpPar.Pos_Node.([T1, '_cell']),1);
        for i2 = 1:1:i2_max
            num_baseplate = cell2mat(InpPar.Pos_Node.([T1, '_cell'])(i2,1));
            if num_baseplate<0
                T2 = ['B_m', num2str(abs(num_baseplate))];
            else
                T2 = ['B_', num2str(num_baseplate)];
            end
            InpPar.ModeShape.(T1).(T2)(:,~bools) = [];
        end
    end
end

if isfield(InpPar.ModeShape, InpPar.Type_SpaceIron{1})
    for i1 = 1:1:length(InpPar.Type_SpaceIron)
        T1 = InpPar.Type_SpaceIron{i1};
        i2_max = length(fieldnames(InpPar.Pos_Node.(T1)));
        for i2 = 1:1:i2_max
            T2 = ['S_', num2str(i2)];
            InpPar.ModeShape.(T1).(T2)(:,~bools) = [];
        end
    end
end

% 220712-DampingRatio, InpPar.ModeFreqd
M_track = eye(InpPar.N_track, InpPar.N_track);
K_track = diag( (2*pi*InpPar.ModeFreq.FT(:,2)).^2 );

if ~isempty(DR_Typical)
    DR_Typical = sortrows(DR_Typical,1);
    bools_del = diff(DR_Typical(:,1))==0;
    DR_Typical(bools_del,:) = [];
end
for i1 = 1:1:InpPar.N_track
    DR(i1,1) = InpPar.ModeFreq.FT(i1,2);
    if ~isempty(DR_Typical)
        pos = find(i1-DR_Typical(:,1)==0);
    else
        pos = [];
    end
    if isempty(pos)
        DR(i1,2) = interp1(DR_Normal(:,1), DR_Normal(:,2), DR(i1,1), 'linear');
    else
        DR(i1,2) = DR_Typical(pos,2);
    end
end
% C_track = zeros(InpPar.N_track, InpPar.N_track);
C_track = diag( 2*DR(:,2).*(2*pi*InpPar.ModeFreq.FT(:,2)) );

Choose_Plot = 0;
if Choose_Plot==1
    figure(99); clf
    plot(DR(:,1), DR(:,2), '*', 'MarkerSize', 3); grid on
    set(gca,'YMinorTick','on','YScale','log');
end


% a. Rayleigh Damping
% C_track = (Par_Track.Rayleigh_Alpha*M_track+Par_Track.Rayleigh_Beta*K_track) + (InpPar.ModeShape.FT)'*Damp_Rail.Cons_Trans*InpPar.ModeShape.FT;
% C_track = (Par_Track.Rayleigh_Alpha*M_track+Par_Track.Rayleigh_Beta*K_track);

% b. Dampign Ratio (Constant)
% DampingRatio = 0.02;
% C_track = 2*DampingRatio*diag(2*pi*InpPar.ModeFreq.FT(:,2));
% C_track = 2*DampingRatio*diag(2*pi*InpPar.ModeFreq.FT(:,2)) + (InpPar.ModeShape.FT)'*Damp_Rail.Cons_Trans*InpPar.ModeShape.FT;

% c. Dampign Ratio (Interp1)
% for i = 1:1:NM_Rail.FT
%     Target_DP = interp1(DampingRatio(:,1), DampingRatio(:,2), InpPar.ModeFreq.FT(i,2), 'linear');
%     C_track(i,i) = 2*Target_DP*(2*pi*InpPar.ModeFreq.FT(i,2));
% end
% C_track = C_track + (InpPar.ModeShape.FT)'*Damp_Rail.Cons_Trans*InpPar.ModeShape.FT;



