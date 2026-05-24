function export_iteration_output_baseline(snapshot_path, output_path)
% Recompute short-run convergence/output summary from a frozen JSON snapshot.
%
% This helper is intentionally narrow: it validates the MATLAB-side summary
% formulas used by the step-9 migration checks without rerunning the full
% vehicle-turnout simulation. The snapshot is produced by Python from a short
% two-stage run and contains the per-iteration contact arrays needed for the
% convergence/error tables.

if nargin < 1 || isempty(snapshot_path)
    error('snapshot_path is required');
end
if nargin < 2 || isempty(output_path)
    output_path = fullfile(tempdir, 'sditt_iteration_output_baseline.mat');
end

snapshot = jsondecode(fileread(snapshot_path));
stages = snapshot.stages;

preload_convergence_history = zeros(0, 7);
preload_output_table = zeros(0, 7);
cal_convergence_history = zeros(0, 7);
cal_output_table = zeros(0, 7);

for stage_index = 1:numel(stages)
    stage = stages(stage_index);
    [convergence_history, output_table] = local_stage_tables(stage.iteration_records);
    if strcmp(stage.name, 'Preload')
        preload_convergence_history = convergence_history;
        preload_output_table = output_table;
    elseif strcmp(stage.name, 'Cal')
        cal_convergence_history = convergence_history;
        cal_output_table = output_table;
    else
        error('unsupported stage %s', stage.name);
    end
end

save(output_path, ...
    'preload_convergence_history', 'preload_output_table', ...
    'cal_convergence_history', 'cal_output_table', '-v7');
fprintf('saved %s\n', output_path);
end


function [convergence_history, output_table] = local_stage_tables(records)
n_records = numel(records);
convergence_history = zeros(n_records, 7);

previous_step_index = NaN;
previous_normal_force = [];
previous_combined_force_norm = [];
output_rows = zeros(0, 7);

for record_index = 1:n_records
    record = records(record_index);
    if record.step_index ~= previous_step_index
        previous_step_index = record.step_index;
        previous_normal_force = [];
        previous_combined_force_norm = [];
    end

    pjc = double(record.pjc);
    pjch = double(record.pjch);
    pjcc = double(record.pjcc);
    prhxf = double(record.prhxf);

    current_normal_force = local_normal_force(pjc, pjcc);
    current_combined_force_norm = local_combined_force_norm(pjch, pjcc, prhxf);

    normal_error = local_relative_error(previous_normal_force, current_normal_force);
    normal_tangential_error = local_relative_error(previous_combined_force_norm, current_combined_force_norm);

    convergence_history(record_index, :) = [ ...
        double(record.step_index), ...
        double(record.iteration), ...
        double(record.time), ...
        double(record.dt), ...
        double(record.front_mileage), ...
        normal_error, ...
        normal_tangential_error];

    previous_normal_force = current_normal_force;
    previous_combined_force_norm = current_combined_force_norm;

    if logical(record.converged)
        output_rows(end+1, :) = [ ...
            double(record.step_index), ...
            double(record.time), ...
            double(record.dt), ...
            double(record.front_mileage), ...
            double(record.iteration), ...
            normal_error, ...
            normal_tangential_error];
    end
end

output_table = output_rows;
end


function normal_force = local_normal_force(pjc, pjcc)
if ~isempty(pjc) && size(pjc, 2) >= 2 && any(abs(pjc(:, 2)) > 0.0)
    normal_force = pjc(:, 2);
elseif ~isempty(pjcc)
    normal_force = pjcc(:, 1);
else
    normal_force = zeros(0, 1);
end
end


function combined_force_norm = local_combined_force_norm(pjch, pjcc, prhxf)
if isempty(prhxf)
    combined_force_norm = zeros(0, 1);
    return
end
combined_force = zeros(size(prhxf, 1), 3);
combined_force(:, 1) = prhxf(:, 1);
combined_force(:, 2) = prhxf(:, 2) + pjch(:, 1);
combined_force(:, 3) = prhxf(:, 3) + pjcc(:, 1);
combined_force_norm = sqrt(sum(combined_force.^2, 2));
end


function error_value = local_relative_error(previous, current)
current = current(:);
if isempty(current)
    error_value = 0.0;
    return
end
if isempty(previous)
    error_value = 1.0;
    return
end

previous = previous(:);
denominator = max(abs(current), eps);
relative = abs(current - previous) ./ denominator;
both_zero = (abs(current) <= eps) & (abs(previous) <= eps);
relative(both_zero) = 0.0;
error_value = max(relative);
end
