function export_full_case_short_run_baseline(mat_path, output_path, stage_name, max_steps)
% Export a MATLAB SDITT output/preload MAT file into the Python short-run JSON schema.
%
% This helper does not modify or rerun the original MATLAB driver. It is meant
% for comparing Python short-run output against an already generated MATLAB
% short-run/full-run checkpoint such as CRH380A_009_Face_V350_zgygPf2.mat or a
% Pre_*.mat file.

if nargin < 1 || isempty(mat_path)
    error('mat_path is required');
end
if nargin < 2 || isempty(output_path)
    output_path = fullfile(tempdir, 'sditt_full_case_short_run_matlab_baseline.json');
end
if nargin < 3 || isempty(stage_name)
    stage_name = 'Cal';
end
if nargin < 4 || isempty(max_steps)
    max_steps = 1;
end

if iscell(mat_path)
    mat_paths = mat_path;
else
    mat_paths = {mat_path};
end
if iscell(stage_name)
    stage_names = stage_name;
else
    stage_names = {stage_name};
end
if numel(stage_names) ~= numel(mat_paths)
    error('stage_name count must match mat_path count');
end

stages = cell(1, numel(mat_paths));
first_data = [];
for stage_index = 1:numel(mat_paths)
    data = load(mat_paths{stage_index});
    if stage_index == 1
        first_data = data;
    end
    stages{stage_index} = local_stage_snapshot(data, stage_names{stage_index}, max_steps);
end

snapshot = struct;
snapshot.schema = 'sditt-full-case-short-run-v1';
snapshot.source = 'matlab';
snapshot.settings = struct;
snapshot.settings.cut_freq = [];
snapshot.settings.dt = local_get_scalar(first_data, {'drtaT'}, NaN);
snapshot.settings.n_steps_per_stage = max_steps;
snapshot.settings.use_sparse = false;
snapshot.preparation = struct;
snapshot.preparation.total_dof = local_total_dof(first_data);
snapshot.preparation.n_track = local_get_nested_scalar(first_data, {'InpPar', 'N_track'}, NaN);
snapshot.preparation.wheelsets = {{'FF', 'FR', 'RF', 'RR'}};
snapshot.preparation.missing_stages = {{}};
snapshot.stages = stages;

encoded = jsonencode(snapshot);
fid = fopen(output_path, 'w');
if fid < 0
    error('cannot open output_path: %s', output_path);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s', encoded);
fprintf('saved %s\n', output_path);
end


function stage = local_stage_snapshot(data, stage_name, max_steps)
row = local_output_row(data);
stage = struct;
stage.name = stage_name;
stage.convergence_history = local_convergence_history(data, row);
stage.iteration_diagnostics = local_iteration_diagnostics(data, row);
stage.output_table = local_output_table(row);
stage.output_rows = row;
if max_steps < 1
    stage.output_rows = struct([]);
    stage.output_table = [];
end
end


function diagnostics = local_iteration_diagnostics(data, row)
diagnostics = struct('step_index', {}, 'iteration', {}, 'time', {}, 'dt', {}, ...
    'front_mileage', {}, 'normal_error', {}, 'normal_tangential_error', {}, ...
    'gravity_plus_damper_force', {}, 'total_force', {}, ...
    'modal_displacement', {}, 'modal_velocity', {}, 'modal_acceleration', {});
iterations = double(row.iterations);
if isnan(iterations) || iterations < 1 || ~isfield(data, 'Con_Int') || ~isstruct(data.Con_Int)
    return
end
for index = 1:iterations
    entry = struct;
    entry.step_index = row.step_index;
    entry.iteration = index;
    entry.time = row.time;
    entry.dt = row.dt;
    entry.front_mileage = row.front_mileage;
    entry.normal_error = local_iteration_error(data, 'ZP_IntError_Nor', row.step_index, index);
    entry.normal_tangential_error = local_iteration_error(data, 'ZP_IntError_NorTan', row.step_index, index);
    entry.gravity_plus_damper_force = local_con_int_row(data.Con_Int, 'Pxt_before_WR', index);
    entry.total_force = local_con_int_row(data.Con_Int, 'Pxt', index);
    entry.modal_displacement = local_con_int_row(data.Con_Int, 'Zwy', index);
    entry.modal_velocity = local_con_int_row(data.Con_Int, 'Zsd', index);
    entry.modal_acceleration = local_con_int_row(data.Con_Int, 'Zjsd', index);
    diagnostics(index, 1) = entry;
end
end


function row = local_con_int_row(con_int, name, index)
row = [];
if isfield(con_int, name)
    values = con_int.(name);
    if isnumeric(values) && size(values, 1) >= index
        row = values(index, :);
    end
end
end


function row = local_output_row(data)
pre = local_pre_struct(data);
pjc = local_get_value(data, pre, 'Pjc', []);
pjch = local_get_value(data, pre, 'Pjch', []);
pjcc = local_get_value(data, pre, 'Pjcc', []);
prhx = local_get_value(data, pre, 'Prhx', []);
prhxf = local_get_value(data, pre, 'Prhxf', []);
zwy = local_final_state(data, 'ZP_Dis', 'Zwy');
zsd = local_final_state(data, 'ZP_Vel', 'Zsd');
zjsd = local_final_state(data, 'ZP_Acc', 'Zjsd');
[vehicle_displacement, vehicle_velocity, vehicle_acceleration] = local_vehicle_state(data, zwy, zsd, zjsd);

row = struct;
row.step_index = local_get_scalar(data, {'xlcs_short_run', 'xlcs'}, 1);
row.iterations = local_get_scalar(data, {'xcs_PreviousStep'}, NaN);
row.time = local_get_scalar(data, {'T'}, NaN);
row.dt = local_get_scalar(data, {'drtaT'}, NaN);
row.front_mileage = local_get_value(data, pre, 'j1', NaN);
row.normal_error = local_iteration_error(data, 'ZP_IntError_Nor', row.step_index, row.iterations);
row.normal_tangential_error = local_iteration_error(data, 'ZP_IntError_NorTan', row.step_index, row.iterations);
row.d0_by_wheelset = local_d0_by_wheelset(local_get_value(data, pre, 'd0', NaN));
row.relvel_max_by_wheelset = struct;
row.contact_diagnostics = local_contact_diagnostics(local_get_value(data, pre, 'Con_WS', struct));
row.pjc = pjc;
row.pjch = pjch;
row.pjcc = pjcc;
row.prhx = prhx;
row.prhxf = prhxf;
row.patch_force_y = local_patch_force_y(pjch, prhxf);
row.patch_force_z = local_patch_force_z(pjcc, prhxf);
row.wheelset_lateral_force = [];
row.wheelset_vertical_force = [];
row.dis_rail = local_get_scalar_or_array(data, 'Dis_Rail');
row.vel_rail = local_get_scalar_or_array(data, 'Vel_Rail');
row.acc_rail = local_get_scalar_or_array(data, 'Acc_Rail');
row.rail_diagnostics = local_rail_diagnostics(data);
row.vehicle_displacement = vehicle_displacement;
row.vehicle_velocity = vehicle_velocity;
row.vehicle_acceleration = vehicle_acceleration;
row.total_force = local_final_state(data, 'ZP_Load', 'Pxt');
row.gravity_force = local_gravity_force(data);
row.contact_force = local_contact_force(row.total_force, row.gravity_force);
row.norms = struct;
row.norms.gravity_force = norm(row.gravity_force(:));
row.norms.contact_force = norm(row.contact_force(:));
row.norms.total_force = norm(row.total_force(:));
row.norms.vehicle_displacement = norm(vehicle_displacement(:));
row.norms.rail_displacement = norm(row.dis_rail(:));
row.norms.pjcc = norm(pjcc(:));
row.norms.prhxf = norm(prhxf(:));
end


function gravity_force = local_gravity_force(data)
if isfield(data, 'Pxt_Gravity') && ~isempty(data.Pxt_Gravity)
    gravity = data.Pxt_Gravity;
    if isrow(gravity)
        gravity = gravity';
    end
    gravity_force = gravity(:);
else
    gravity_force = [];
end
end


function contact_force = local_contact_force(total_force, gravity_force)
if ~isempty(gravity_force)
    contact_force = total_force(:) - gravity_force(:);
else
    contact_force = [];
end
end


function diagnostics = local_contact_diagnostics(con_ws)
diagnostics = struct;
wheelsets = {'FF', 'FR', 'RF', 'RR'};
sides = {'L', 'R'};
for wheel_index = 1:numel(wheelsets)
    wheelset = wheelsets{wheel_index};
    if ~isstruct(con_ws) || ~isfield(con_ws, wheelset)
        continue
    end
    con = con_ws.(wheelset);
    for side_index = 1:numel(sides)
        side = sides{side_index};
        side_data = struct;
        side_data.normal_force = local_nested_value(con, {'Normal_Force', side}, []);
        side_data.con_wheel_2 = local_nested_value(con, {'Con_wheel_2', side}, []);
        side_data.con_wheel_2_full = side_data.con_wheel_2;
        side_data.con_rail_1 = local_nested_value(con, {'Con_rail_1', side}, []);
        side_data.profile_r = local_nested_value(con, {'profile_r', side}, []);
        side_data.con_wheel_2_a = local_nested_value(con, {'Con_wheel_2_II', 'a', side}, []);
        side_data.con_rail_1_a = local_nested_value(con, {'Con_rail_1_II', 'a', side}, []);
        side_data.ver_pen_a = local_nested_value(con, {'Ver_Pen_a', side}, []);
        side_data.elastic_normal_force = local_nested_value(con, {'Normal_Force', side}, []);
        side_data.area_stripes = local_nested_value(con, {'Area_STRIPES', side}, []);
        side_data.epsilon = local_nested_value(con, {'Epsilon', side}, []);
        side_data.elastic_permeability_unit = local_nested_value(con, {'elastic_permeability_Unit', side}, []);
        side_data.prh = local_nested_value(con, {'Prh', side}, []);
        side_data.prhx_t = local_nested_value(con, {'Prhx_T', side}, []);
        side_data.prhxf_t = local_nested_value(con, {'Prhxf_T', side}, []);
        side_data.rhxs = local_nested_value(con, {'RHXS', side}, []);
        side_data.rhlv = local_nested_value(con, {'RHLv', side}, []);
        side_data.a2 = local_nested_value(con, {'a2', side}, []);
        side_data.b2 = local_nested_value(con, {'b2', side}, []);
        side_data.vjd = local_nested_value(con, {'Vjd', side}, []);
        side_data.vjd_r = local_nested_value(con, {'Vjd_r', side}, []);
        side_data.vsdc = local_nested_value(con, {'Vsdc', side}, []);
        side_data.vjsdc = local_nested_value(con, {'Vjsdc', side}, []);
        side_data.vgd = local_nested_value(con, {'Vgd', side}, []);
        side_data.stripes = local_stripes_payload(local_nested_value(con, {'Con_STRIPES', side}, {}));
        side_data.nor_gap_concs = local_norgap_payload(local_nested_value(con, {'NorGap_ConCS', side}, {}));
        diagnostics.(wheelset).(side) = side_data;
    end
end
end


function diagnostics = local_rail_diagnostics(data)
diagnostics = struct;
diagnostics.modal_displacement = local_track_state(data, 'ZP_Dis', 'Zwy');
diagnostics.modal_velocity = local_track_state(data, 'ZP_Vel', 'Zsd');
diagnostics.modal_acceleration = local_track_state(data, 'ZP_Acc', 'Zjsd');
diagnostics.rail_beam_motion = struct;
diagnostics.shape_entries = struct;
if isfield(data, 'RailBeam_Motion') && isstruct(data.RailBeam_Motion)
    diagnostics.rail_beam_motion = data.RailBeam_Motion;
end
if ~isfield(data, 'ShapeFunction') || ~isstruct(data.ShapeFunction) || ...
        ~isfield(data, 'DynStatus_Rail') || ~isstruct(data.DynStatus_Rail)
    return
end

names = fieldnames(data.ShapeFunction);
for index = 1:numel(names)
    name = names{index};
    suffix = '_Y';
    if length(name) <= length(suffix) || ~strcmp(name(end-length(suffix)+1:end), suffix)
        continue
    end
    if contains(name, '_Mapping_DynStatus_')
        continue
    end
    prefix = name(1:end-length(suffix));
    underscore = strfind(prefix, '_');
    if isempty(underscore)
        rail_name = prefix;
    else
        rail_name = prefix(underscore(1)+1:end);
    end
    dyn_name = [rail_name, '_Vel'];
    if isfield(data.DynStatus_Rail, dyn_name)
        dyn_velocity = local_dyn_track_vector(data.DynStatus_Rail.(dyn_name));
    else
        dyn_velocity = [];
    end
    entry = struct;
    entry.y = local_rail_component(data.ShapeFunction, prefix, 'Y', dyn_velocity);
    entry.z = local_rail_component(data.ShapeFunction, prefix, 'Z', dyn_velocity);
    entry.roty = local_rail_component(data.ShapeFunction, prefix, 'ROTY', dyn_velocity);
    entry.rotz = local_rail_component(data.ShapeFunction, prefix, 'ROTZ', dyn_velocity);
    diagnostics.shape_entries.(prefix) = entry;
end
end


function state = local_track_state(data, history_name, fallback_name)
state = [];
if isfield(data, 'InpPar') && isfield(data.InpPar, 'N_track')
    n_track = double(data.InpPar.N_track);
else
    return
end
full_state = local_final_state(data, history_name, fallback_name);
if isempty(full_state)
    return
end
state = full_state(1:n_track);
end


function component = local_rail_component(shape_function, prefix, component_name, dyn_velocity)
component = struct;
shape_name = [prefix, '_', component_name];
mapping_name = [prefix, '_Mapping_DynStatus_', component_name];
if isfield(shape_function, shape_name)
    component.shape = shape_function.(shape_name);
else
    component.shape = [];
end
if isfield(shape_function, mapping_name)
    component.mapping = shape_function.(mapping_name);
else
    component.mapping = [];
end
if ~isempty(component.mapping) && ~isempty(dyn_velocity)
    component.dyn_velocity = dyn_velocity(component.mapping);
else
    component.dyn_velocity = [];
end
if ~isempty(component.shape) && ~isempty(component.dyn_velocity)
    component.velocity = component.shape * component.dyn_velocity;
else
    component.velocity = [];
end
end


function vector = local_dyn_track_vector(status)
if isempty(status)
    vector = [];
    return
end
vector = reshape(status(:, [2, 3, 5, 6])', [], 1);
end


function value = local_nested_value(root, path, default_value)
value = default_value;
target = root;
for index = 1:numel(path)
    name = path{index};
    if isstruct(target) && isfield(target, name)
        target = target.(name);
    else
        return
    end
end
value = target;
end


function payload = local_stripes_payload(stripes)
payload = struct('curvature', {}, 'wheel_points', {}, 'rail_points', {}, ...
    'stripes', {}, 'normal_force', {}, 'area', {});
if ~iscell(stripes)
    return
end
for index = 1:size(stripes, 1)
    entry = struct;
    entry.curvature = [];
    entry.stripes = [];
    entry.normal_force = 0;
    entry.area = 0;
    if size(stripes, 2) >= 3 && isnumeric(stripes{index, 3})
        entry.curvature = stripes{index, 3};
    end
    if size(stripes, 2) >= 1 && isnumeric(stripes{index, 1})
        entry.wheel_points = stripes{index, 1};
    else
        entry.wheel_points = [];
    end
    if size(stripes, 2) >= 2 && isnumeric(stripes{index, 2})
        entry.rail_points = stripes{index, 2};
    else
        entry.rail_points = [];
    end
    if size(stripes, 2) >= 4 && isnumeric(stripes{index, 4})
        stripe_rows = stripes{index, 4};
        entry.stripes = stripe_rows;
        if ~isempty(stripe_rows)
            entry.normal_force = sum(stripe_rows(:, 3));
        end
    end
    payload(index, 1) = entry;
end
end


function payload = local_norgap_payload(norgap)
payload = struct('nor_gap_6', {}, 'nor_gap_8', {}, 'nor_gap_11', {}, ...
    'stripe_y', {}, 'stripe_penetration', {}, 'stripe_dy', {});
if ~iscell(norgap)
    return
end
for index = 1:size(norgap, 1)
    entry = struct;
    entry.nor_gap_6 = [];
    entry.nor_gap_8 = [];
    entry.nor_gap_11 = [];
    entry.stripe_y = [];
    entry.stripe_penetration = [];
    entry.stripe_dy = [];
    if size(norgap, 2) >= 6 && isnumeric(norgap{index, 6})
        entry.nor_gap_6 = norgap{index, 6};
    end
    if size(norgap, 2) >= 8 && isnumeric(norgap{index, 8})
        entry.nor_gap_8 = norgap{index, 8};
    end
    if size(norgap, 2) >= 11 && isnumeric(norgap{index, 11})
        entry.nor_gap_11 = norgap{index, 11};
        positive = entry.nor_gap_11(:, 2) > 0;
        if any(positive)
            entry.stripe_y = linspace(entry.nor_gap_11(find(positive, 1), 1), ...
                entry.nor_gap_11(find(positive, 1, 'last'), 1), 51)';
            entry.stripe_penetration = interp1(entry.nor_gap_11(:, 1), entry.nor_gap_11(:, 2), ...
                entry.stripe_y, 'linear');
            entry.stripe_dy = repmat((entry.stripe_y(end) - entry.stripe_y(1)) / 50, 51, 1);
        end
    end
    payload(index, 1) = entry;
end
end


function history = local_convergence_history(data, row)
iterations = double(row.iterations);
if isnan(iterations) || iterations < 1
    history = [];
    return
end
history = NaN(iterations, 7);
for index = 1:iterations
    history(index, :) = [
        double(row.step_index), ...
        index, ...
        double(row.time), ...
        double(row.dt), ...
        double(row.front_mileage), ...
        local_iteration_error(data, 'ZP_IntError_Nor', row.step_index, index), ...
        local_iteration_error(data, 'ZP_IntError_NorTan', row.step_index, index) ...
    ];
end
end


function value = local_iteration_error(data, name, step_index, iteration)
value = NaN;
if ~isfield(data, name) || isnan(iteration)
    return
end
errors = data.(name);
row_index = double(iteration) + 1;
column_index = double(step_index);
if row_index >= 1 && row_index <= size(errors, 1) && column_index >= 1 && column_index <= size(errors, 2)
    value = double(errors(row_index, column_index));
end
end


function table = local_output_table(row)
table = [double(row.step_index), double(row.time), double(row.dt), double(row.front_mileage), ...
    double(row.iterations), double(row.normal_error), double(row.normal_tangential_error)];
end


function pre = local_pre_struct(data)
if isfield(data, 'Pre')
    pre = data.Pre;
else
    pre = struct;
end
end


function value = local_get_value(data, pre, name, default_value)
if isfield(data, name)
    value = data.(name);
elseif isfield(pre, name)
    value = pre.(name);
elseif strcmp(name, 'Pjc') && isfield(pre, 'NF')
    nf = pre.NF;
    value = zeros(size(nf, 1), 2);
    value(:, 2) = nf(:, end);
else
    value = default_value;
end
end


function value = local_get_scalar(data, names, default_value)
value = default_value;
for index = 1:numel(names)
    name = names{index};
    if isfield(data, name)
        candidate = data.(name);
        if isnumeric(candidate) && ~isempty(candidate)
            value = double(candidate(1));
            return
        end
    end
end
end


function value = local_get_nested_scalar(data, names, default_value)
value = default_value;
target = data;
for index = 1:numel(names)
    name = names{index};
    if isstruct(target) && isfield(target, name)
        target = target.(name);
    else
        return
    end
end
if isnumeric(target) && ~isempty(target)
    value = double(target(1));
end
end


function value = local_get_scalar_or_array(data, name)
if isfield(data, name)
    value = data.(name);
else
    value = [];
end
end


function total = local_total_dof(data)
if isfield(data, 'DOF_sum')
    total = double(data.DOF_sum);
elseif isfield(data, 'ZP_Dis') && ~isempty(data.ZP_Dis)
    total = size(data.ZP_Dis, 2);
else
    total = NaN;
end
end


function state = local_final_state(data, history_name, fallback_name)
if isfield(data, history_name) && ~isempty(data.(history_name))
    history = data.(history_name);
    row_index = local_get_scalar(data, {'xlcs_short_run', 'xlcs'}, size(history, 1));
    if ~isnan(row_index) && row_index >= 1 && row_index <= size(history, 1) && any(~isnan(history(row_index, :)))
        state = history(row_index, :);
    else
        finite_rows = find(any(~isnan(history), 2));
        if isempty(finite_rows)
            state = history(end, :);
        else
            state = history(finite_rows(end), :);
        end
    end
elseif isfield(data, fallback_name) && ~isempty(data.(fallback_name))
    fallback = data.(fallback_name);
    if size(fallback, 2) >= 4
        state = fallback(:, 4).';
    else
        state = fallback(:).';
    end
else
    state = [];
end
end


function [displacement, velocity, acceleration] = local_vehicle_state(data, zwy, zsd, zjsd)
start_index = 1;
if isfield(data, 'InpPar')
    inp = data.InpPar;
    if isfield(inp, 'N_track') && isfield(inp, 'NM_FW') && isfield(inp, 'Nw')
        start_index = double(inp.N_track) + double(inp.NM_FW) * double(inp.Nw) + 1;
    elseif isfield(inp, 'N_track')
        start_index = double(inp.N_track) + 1;
    end
end
displacement = local_tail(zwy, start_index);
velocity = local_tail(zsd, start_index);
acceleration = local_tail(zjsd, start_index);
end


function value = local_tail(array, start_index)
if isempty(array) || start_index > numel(array)
    value = [];
else
    value = array(start_index:end);
end
end


function values = local_d0_by_wheelset(d0)
if isempty(d0) || ~isnumeric(d0)
    d0_value = NaN;
else
    d0_value = double(d0(1));
end
values = struct;
values.FF = d0_value;
values.FR = d0_value;
values.RF = d0_value;
values.RR = d0_value;
end


function force = local_patch_force_y(pjch, prhxf)
if isempty(pjch) || isempty(prhxf)
    force = [];
else
    force = -prhxf(:, 2) - pjch(:, 1);
end
end


function force = local_patch_force_z(pjcc, prhxf)
if isempty(pjcc) || isempty(prhxf)
    force = [];
else
    force = -prhxf(:, 3) - pjcc(:, 1);
end
end
