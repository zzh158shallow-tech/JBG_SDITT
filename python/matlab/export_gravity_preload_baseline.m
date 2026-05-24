function export_gravity_preload_baseline(cut_freq, output_path)
% Export MATLAB Gravity_Load_ModalFT output for Python migration checks.

if nargin < 1 || isempty(cut_freq)
    cut_freq = 50;
end
if nargin < 2 || isempty(output_path)
    output_path = fullfile(tempdir, sprintf('sditt_gravity_preload_cut%g.mat', cut_freq));
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
InpPar.Vlc = 350 / 3.6;
InpPar.Nw = 4;
InpPar.NM_FW = 0;
InpPar.N_RV = 35;
InpPar.Type_Rail_All = {'zjbg', 'qjg_cgyg', 'zjg_zgyg', 'qjbg', 'cxg', 'dxg', 'cgjg', 'hg'};
InpPar.Type_Baseplate = { ...
    'Switch_L', 'Switch_R', ...
    'Closure_zjbg', 'Closure_qjg_cgyg', 'Closure_zjg_zgyg', 'Closure_qjbg', ...
    'Crossing_L', 'Crossing_Center', 'Crossing_R', ...
    'Plain_Thr_L', 'Plain_Thr_R', 'Plain_Div_L', 'Plain_Div_R'};

Par_Vehicle = Par_Vehicle_CRH380A_v6(InpPar);
InpPar.N_RV = InpPar.N_RV + 16;

load('Mat_FT_S8b.mat', 'ModeFreq', 'ModeShape_Mapping', 'ModeShape', 'Pos_Node', 'N_Node', 'DOF_Node', 'Type_SpaceIron');
InpPar.ModeFreq = ModeFreq;
InpPar.ModeShape_Mapping = ModeShape_Mapping;
InpPar.ModeShape = ModeShape;
InpPar.Pos_Node = Pos_Node;
InpPar.N_Node = N_Node;
InpPar.DOF_Node = DOF_Node;
InpPar.Type_SpaceIron = Type_SpaceIron;

DR_Normal = [0, 0.20; 2500, 0.20];
DR_Typical = [];
[~, ~, ~, ~, InpPar] = Matrix_Modal_FT_230313(InpPar, cut_freq, DR_Normal, DR_Typical);
[Pxt_Gravity, Pxt_Track] = Gravity_Load_ModalFT(InpPar, Par_Vehicle);

n_track = InpPar.N_track;
total_dof = size(Pxt_Gravity, 1);
save(output_path, 'Pxt_Gravity', 'Pxt_Track', 'cut_freq', 'n_track', 'total_dof', '-v7');
fprintf('saved %s\n', output_path);
end
