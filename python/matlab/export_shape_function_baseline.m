function export_shape_function_baseline(j1, cut_freq, output_path)
% Export Cal_ShapeFunction_Beam188_FWV output for Python migration checks.

if nargin < 1 || isempty(j1)
    j1 = 54;
end
if nargin < 2 || isempty(cut_freq)
    cut_freq = 50;
end
if nargin < 3 || isempty(output_path)
    output_path = fullfile(tempdir, sprintf('sditt_shape_function_j%g_cut%g.mat', j1, cut_freq));
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
InpPar.VehicleDir = 'Face';
InpPar.Exp_WS = {'FF','FR','RF','RR'};
InpPar.Exp_DummyRail = {'L1','R1','R2','R3'};
InpPar.Type_Rail = {'zjbg'; 'qjbg'; 'zjg_zgyg'; 'cxg'};
InpPar.Nw = 4;
InpPar.N_ConPatch = 4;
InpPar.Vlc = 350 / 3.6;

Par_Vehicle = Par_Vehicle_CRH380A_v6(InpPar);
Par_Vehicle.Distance_Vehicle = [0, 2*Par_Vehicle.Ll1, 2*Par_Vehicle.Ll2, 2*(Par_Vehicle.Ll1+Par_Vehicle.Ll2)];

load('Mat_FT_S8b.mat', 'ModeFreq', 'ModeShape', 'Pos_Node', 'N_Node', 'DOF_Node');
InpPar.ModeFreq = ModeFreq;
InpPar.ModeShape = ModeShape;
InpPar.Pos_Node = Pos_Node;
InpPar.N_Node = N_Node;
InpPar.DOF_Node = DOF_Node;

DR_Normal = [0, 0.20; 2500, 0.20];
DR_Typical = [];
[~, ~, ~, ~, InpPar] = Matrix_Modal_FT_230313(InpPar, cut_freq, DR_Normal, DR_Typical);
Choose_zjgUnEven = 0;
InpPar = Cal_RailBeam_230518(InpPar, Choose_zjgUnEven);

[ShapeFunction, RailBeam_Motion] = Cal_ShapeFunction_Beam188_FWV(InpPar, Par_Vehicle, j1);

save(output_path, 'j1', 'cut_freq', 'ShapeFunction', 'RailBeam_Motion', '-v7');
fprintf('saved %s\n', output_path);
end
