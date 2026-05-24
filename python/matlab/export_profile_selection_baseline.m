function export_profile_selection_baseline(j1, output_path)
% Export Get_Profile_P1_Through_210624 output for Python migration checks.

if nargin < 1 || isempty(j1)
    j1 = 54;
end
if nargin < 2 || isempty(output_path)
    output_path = fullfile(tempdir, sprintf('sditt_profile_selection_j%g.mat', j1));
end

script_dir = fileparts(mfilename('fullpath'));
python_dir = fileparts(script_dir);
repo_root = fileparts(python_dir);
matlab_dir = fullfile(repo_root, 'SDITT-RW-FT-250728');

addpath(matlab_dir);
addpath(fullfile(matlab_dir, 'WRProfile-07(009)-1_18', 'wheel'));
addpath(fullfile(matlab_dir, 'WRProfile-07(009)-1_18', '07(009)-Mileage'));
addpath(fullfile(matlab_dir, 'WRProfile-07(009)-1_18', '07(009)-qjbg-20200418'));
addpath(fullfile(matlab_dir, 'WRProfile-07(009)-1_18', '07(009)-zjg-20200418'));
addpath(fullfile(matlab_dir, 'WRProfile-07(009)-1_18', '07(009)-zgyg-20200424'));
addpath(fullfile(matlab_dir, 'WRProfile-07(009)-1_18', '07(009)-zgyg-20200509'));
addpath(fullfile(matlab_dir, 'WRProfile-07(009)-1_18', '07(009)-zgyg-20250720'));
addpath(fullfile(matlab_dir, 'WRProfile-07(009)-1_18', '07(009)-zgyg-20250722'));
addpath(fullfile(matlab_dir, 'WRProfile-07(009)-1_18', '07(009)-cxg-20200422'));
addpath(fullfile(matlab_dir, 'WRProfile-07(009)-1_18', '07(009)-cxg-20250801'));

InpPar = struct;
InpPar.Choose_Turnout = '07(009)';
InpPar.VehicleDir = 'Face';
InpPar.Exp_WS = {'FF','FR','RF','RR'};
InpPar.Exp_DummyRail = {'L1','R1','R2','R3'};
InpPar.Nw = 4;
InpPar.Vlc = 350 / 3.6;
InpPar.num_interp_Bezier = 1000;

Par_Vehicle = Par_Vehicle_CRH380A_v6(InpPar);
Par_Vehicle.Distance_Vehicle = [0, 2*Par_Vehicle.Ll1, 2*Par_Vehicle.Ll2, 2*(Par_Vehicle.Ll1+Par_Vehicle.Ll2)];

InpPar = Bezier_InterpProfile_210620(InpPar, 0, 0, 0);
RailPro_ProCS = struct;
RailPro_ProCS = Get_Profile_P1_Through_210624(InpPar, j1, Par_Vehicle, RailPro_ProCS, {'L1', 'R1', 'R2', 'R3'});

rails = {'L1', 'R1', 'R2', 'R3'};
stations = InpPar.Exp_WS;
profile_nums = NaN(length(rails), length(stations));
profile_rows = zeros(length(rails), length(stations));
radius_rows = zeros(length(rails), length(stations));
for i = 1:length(rails)
    for k = 1:length(stations)
        rail = rails{i};
        station = stations{k};
        num_field = [station, '_Profile_num'];
        profile_field = [station, '_Profile'];
        radius_field = [station, '_Radius'];
        if ~isempty(RailPro_ProCS.(rail).(num_field))
            profile_nums(i,k) = RailPro_ProCS.(rail).(num_field);
        end
        profile_rows(i,k) = size(RailPro_ProCS.(rail).(profile_field), 1);
        radius_rows(i,k) = size(RailPro_ProCS.(rail).(radius_field), 1);
    end
end

L1_FF_Profile = RailPro_ProCS.L1.FF_Profile;
R1_FF_Profile = RailPro_ProCS.R1.FF_Profile;
R2_FF_Profile = RailPro_ProCS.R2.FF_Profile;
R3_FF_Profile = RailPro_ProCS.R3.FF_Profile;
R1_FF_FrontProfile = RailPro_ProCS.R1.FF_FrontProfile;
R1_FF_RearProfile = RailPro_ProCS.R1.FF_RearProfile;
R2_FF_FrontProfile = RailPro_ProCS.R2.FF_FrontProfile;
R2_FF_RearProfile = RailPro_ProCS.R2.FF_RearProfile;

save(output_path, ...
    'j1', 'profile_nums', 'profile_rows', 'radius_rows', ...
    'L1_FF_Profile', 'R1_FF_Profile', 'R2_FF_Profile', 'R3_FF_Profile', ...
    'R1_FF_FrontProfile', 'R1_FF_RearProfile', ...
    'R2_FF_FrontProfile', 'R2_FF_RearProfile', '-v7');
fprintf('saved %s\n', output_path);
end
