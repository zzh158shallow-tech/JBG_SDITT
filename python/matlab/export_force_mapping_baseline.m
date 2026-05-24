function export_force_mapping_baseline(j1, cut_freq, output_path)
% Export WR_Force_ModalFT and WR_Force_VehicleSys_RotationIII baselines.

if nargin < 1 || isempty(j1)
    j1 = 54;
end
if nargin < 2 || isempty(cut_freq)
    cut_freq = 50;
end
if nargin < 3 || isempty(output_path)
    output_path = fullfile(tempdir, sprintf('sditt_force_mapping_j%g_cut%g.mat', j1, cut_freq));
end

script_dir = fileparts(mfilename('fullpath'));
python_dir = fileparts(script_dir);
repo_root = fileparts(python_dir);
matlab_dir = fullfile(repo_root, 'SDITT-RW-FT-250728');

addpath(matlab_dir);
old_dir = pwd;
cleanup = onCleanup(@() cd(old_dir));
cd(matlab_dir);

InpPar = struct;
InpPar.Choose_Turnout = '07(009)';
InpPar.Type_Track = 'FT-Modal';
InpPar.Type_Vehicle = 'CRH380A_v6';
InpPar.VehicleDir = 'Face';
InpPar.Type_Side = {'L','R'};
InpPar.Exp_WS = {'FF','FR','RF','RR'};
InpPar.Exp_DummyRail = {'L1','R1','R2','R3'};
InpPar.Exp_DummyRail_WheelSide = {'L','R','R','R'};
InpPar.Type_Rail = {'zjbg'; 'qjbg'; 'zjg_zgyg'; 'cxg'};
InpPar.Nw = 4;
InpPar.N_ConPatch = 4;
InpPar.NM_FW = 0;
InpPar.Vlc = 350 / 3.6;

Par_Vehicle = Par_Vehicle_CRH380A_v6(InpPar);
Par_Vehicle.Distance_Vehicle = [0, 2*Par_Vehicle.Ll1, 2*Par_Vehicle.Ll2, 2*(Par_Vehicle.Ll1+Par_Vehicle.Ll2)];
Par_Track = Par_TrackSystem(InpPar);
Par_FW = struct;

load('Mat_FT_S8b.mat', 'ModeFreq', 'ModeShape', 'Pos_Node', 'N_Node', 'DOF_Node');
InpPar.ModeFreq = ModeFreq;
InpPar.ModeShape = ModeShape;
InpPar.Pos_Node = Pos_Node;
InpPar.N_Node = N_Node;
InpPar.DOF_Node = DOF_Node;

DR_Normal = [0, 0.20; 2500, 0.20];
DR_Typical = [];
[~, ~, ~, ~, InpPar] = Matrix_Modal_FT_230313(InpPar, cut_freq, DR_Normal, DR_Typical);
InpPar = Cal_RailBeam_230518(InpPar, 0);

[ShapeFunction, RailBeam_Motion] = Cal_ShapeFunction_Beam188_FWV(InpPar, Par_Vehicle, j1);

n_contact = InpPar.Nw * InpPar.N_ConPatch;
Pjcc = zeros(n_contact, 1);
Pjch = zeros(n_contact, 1);
Prhxf = zeros(n_contact, 6);
legacy_idx = [1, 2, 5, 6, 9, 10, 13, 14];
legacy_scale = [1.0, 1.5, 0.8, 1.2, 0.6, 1.1, 0.7, 0.9];
for k = 1:length(legacy_idx)
    idx = legacy_idx(k);
    scale = legacy_scale(k);
    Pjcc(idx, 1) = 100.0 * scale;
    Prhxf(idx, :) = [2.0, 0.0, 4.0, 0.0, 5.0, 6.0] * scale;
end
extra_idx = [4, 8, 12, 16];
extra_scale = [0.5, 0.4, 0.3, 0.2];
for k = 1:length(extra_idx)
    idx = extra_idx(k);
    scale = extra_scale(k);
    Pjcc(idx, 1) = 50.0 * scale;
    Prhxf(idx, :) = [1.0, 0.0, 2.5, 0.0, 3.0, 4.0] * scale;
end

Con_WS = struct;
Con_WS.FF = struct;
xlcs = 1;
total_dof = InpPar.N_track + InpPar.NM_FW * InpPar.Nw + InpPar.N_RV;
Pxt0 = zeros(total_dof, 1);
Zwy = zeros(total_dof, 4);

[Pxt_Modal, Pxt_Track] = WR_Force_ModalFT(InpPar, xlcs, Pxt0, Pjcc, Pjch, Prhxf, Con_WS, ShapeFunction);
[Pxt_All, Q_temp] = WR_Force_VehicleSys_RotationIII(InpPar, Par_Vehicle, Par_Track, Par_FW, Pxt_Modal, Zwy, Pjcc, Pjch, Prhxf, Con_WS);

save(output_path, ...
    'j1', 'cut_freq', 'xlcs', ...
    'ShapeFunction', 'RailBeam_Motion', ...
    'Pjcc', 'Pjch', 'Prhxf', 'Pxt_Modal', 'Pxt_Track', 'Pxt_All', 'Q_temp', '-v7');
fprintf('saved %s\n', output_path);
end
