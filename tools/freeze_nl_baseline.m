function freeze_nl_baseline(out_dir)
% Runs the nonlinear test suite on Flight_simulator_nl and saves the results to
% <repo>/results/<name> (default name: baseline_nonlinear_v1):
%   console_output.txt   all printed tables
%   *.png / *.fig        every figure of every stage
% Usage: freeze_nl_baseline   or   freeze_nl_baseline(out_dir)   (runtime about 20-30 min)
%
% Stages:
%   1 simulink_multiseed_summary   10 seeds x 4 controller cases (compare with the linear baseline)
%   2 simulink_actuator_compare    ideal / actuators / actuators + sensors
%   3 nl_maneuvers(1)              large-amplitude recovery, actuators + sensors on
%   4 nl_gust_severity (ideal)     gust scale sweep, linear vs nonlinear
%   5 nl_gust_severity (realistic) same with actuators + sensors on
%   6 nl_vs_linear                 same-seed trajectory comparison at 10 % gust

root = fileparts(fileparts(mfilename('fullpath')));
if nargin < 1, out_dir = fullfile(root, 'results', 'baseline_nonlinear_v1'); end
mdl = 'Flight_simulator_nl';
if ~bdIsLoaded(mdl), load_system(mdl); end
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

assignin('base', 'sim_model', mdl);
assignin('base', 'fb_on', 1); assignin('base', 'alt_on', 1);
assignin('base', 'trk_on', 1); assignin('base', 'gust_on', 1);

stages = {'simulink_multiseed_summary', 'simulink_actuator_compare', 'nl_maneuvers(1);', ...
          'nl_gust_severity([0.5 1 2 3 4 6], 5, 0);', 'nl_gust_severity([1 2 3 4], 5, 1);', 'nl_vs_linear;'};
names  = {'multiseed', 'actuator_compare', 'maneuvers', 'gust_severity_ideal', 'gust_severity_realistic', 'nl_vs_linear'};

txt = fullfile(out_dir, 'console_output.txt');
if exist(txt, 'file'), delete(txt); end
diary(txt);
cleanup = onCleanup(@() diary('off'));
fprintf('Nonlinear baseline created %s\nModel: %s,  psi_int_lim = %g\n', char(datetime('now')), mdl, evalin('base', 'psi_int_lim'));

for k = 1:numel(stages)
    fprintf('\n================ %d  %s ================\n', k, stages{k});
    close all;
    assignin('base', 'act_on', 0);  assignin('base', 'sens_on', 0);
    assignin('base', 'fb_on', 1); assignin('base', 'alt_on', 1);
    assignin('base', 'trk_on', 1); assignin('base', 'gust_on', 1);
    try
        evalin('base', stages{k});
    catch ME
        fprintf('STAGE FAILED: %s\n', ME.message);
        continue
    end
    figs = findall(0, 'type', 'figure');
    for f = 1:numel(figs)
        base = sprintf('%02d_%s_fig%d', k, names{k}, f);
        try, exportgraphics(figs(f), fullfile(out_dir, [base '.png']), 'Resolution', 200);
        catch, try, saveas(figs(f), fullfile(out_dir, [base '.png'])); catch, end, end
        try, savefig(figs(f), fullfile(out_dir, [base '.fig'])); catch, end
    end
end

evalin('base', 'clear sim_model');
diary('off');
fprintf('Nonlinear baseline saved in %s\n', out_dir);
end
