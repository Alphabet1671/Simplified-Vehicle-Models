function generate_unitire_reference(outputPath, tireModelDir)
%GENERATE_UNITIRE_REFERENCE Write reference vectors for the Python tire port.
%
% The vectors come from the reviewed function unitire_simple_solve.m and the
% fitted parameter file Hoosier_R20_Combined_Simple.mat. The Python port must
% reproduce every row of the output file.
%
% Run this function from MATLAB:
%
%   generate_unitire_reference('tests/fixtures/hoosier_43075_r20_reference.csv', ...
%       '/Users/me/Dev/FSAE-VD-Personal-Scripts/Yaw Dynamics/Tire Model')
%
% The second argument names the directory that holds the MATLAB tire model.
% The reviewed local directory is:
%   /Users/alphabet1671/Dev/FSAE-VD-Personal-Scripts/Yaw Dynamics/Tire Model

if nargin < 1 || isempty(outputPath)
    error('Name the output file.');
end
if nargin < 2 || isempty(tireModelDir)
    error('Name the directory that holds unitire_simple_solve.m.');
end

addpath(tireModelDir);
loaded = load(fullfile(tireModelDir, 'Hoosier_R20_Combined_Simple.mat'));
tire = loaded.tire;

% One row holds: slip angle, slip ratio, load, and hub speed.
cases = [
    0.00       0.00     1000    50
    0.01       0.00     1000    50
    0.05       0.00     1000    50
    0.10       0.00     1000    50
    0.20       0.00     1000    50
    0.30       0.00     1000    50
   -0.01       0.00     1000    50
   -0.10       0.00     1000    50
   -0.30       0.00     1000    50
    0.00      -0.20     1000    50
    0.00      -0.10     1000    50
    0.00      -0.02     1000    50
    0.00       0.02     1000    50
    0.00       0.10     1000    50
    0.00       0.20     1000    50
    0.05      -0.05     1000    50
    0.05       0.05     1000    50
    0.15      -0.05     1000    50
    0.15       0.05     1000    50
    0.10       0.00      100    50
    0.10       0.00      300    50
    0.10       0.00      660    50
    0.10       0.00     1500    50
    0.10       0.00     1000     0.1
    0.10       0.00     1000     1
    0.10       0.00     1000     5
    0.10       0.00     1000   200
    0.10       0.05        0    50
    0.10       0.05     1000     0.05
];

out = unitire_simple_solve(cases(:, 1), cases(:, 2), cases(:, 3), cases(:, 4), tire);

rows = [ ...
    cases(:, 1), cases(:, 2), cases(:, 3), cases(:, 4), ...
    out.F_tire(:, 1), out.F_tire(:, 2), out.F_tire(:, 3), ...
    out.M_tire(:, 1), out.M_tire(:, 2), out.M_tire(:, 3), ...
    out.Re, out.Rl, out.mu_x, out.mu_y, out.Dx, out.ly, ...
    out.debug.phi, out.debug.Fbar, out.debug.lambda, out.Sx, out.Sy];

header = { ...
    'slip_angle_rad', 'slip_ratio', 'vertical_load_n', 'wheel_speed_rad_s', ...
    'longitudinal_force_n', 'lateral_force_n', 'vertical_force_n', ...
    'overturning_moment_nm', 'rolling_resistance_moment_nm', 'aligning_moment_nm', ...
    'effective_radius_m', 'loaded_radius_m', 'mu_x', 'mu_y', 'pneumatic_trail_m', ...
    'relaxation_length_m', 'phi', 'f_bar', 'lambda', 'sx', 'sy'};

file = fopen(outputPath, 'w');
fprintf(file, '%s\n', strjoin(header, ','));
fprintf(file, '%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n', rows.');
fclose(file);

fprintf('WROTE %d ROWS TO %s\n', size(rows, 1), outputPath);
end
