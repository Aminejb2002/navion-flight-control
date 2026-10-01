function freeze_linear_baseline(out_dir)
% Runs the linear reference tests (ideal actuators/sensors) and saves the results to
% <repo>/results/<name> (default name: baseline_linear_v2):
%   console_output.txt   all printed tables
%   *.png / *.fig        every figure of every stage
% Usage: freeze_linear_baseline   or   freeze_linear_baseline(out_dir)
% Stages run in the base workspace; the code version is tracked with git tags.

root = fileparts(fileparts(mfilename('fullpath')));
if nargin < 1, out_dir = fullfile(root, 'results', 'baseline_linear_v2'); end
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

% model must be loaded for the test scripts
if ~bdIsLoaded('Flight_simulator')
    load_system('Flight_simulator');
end

% reference condition: ideal actuators/sensors, full controller
assignin('base', 'act_on', 0);
assignin('base', 'sens_on', 0);
assignin('base', 'fb_on', 1); assignin('base', 'alt_on', 1);
assignin('base', 'trk_on', 1); assignin('base', 'gust_on', 1);

stages = {'simulink_multiseed_summary', 'flying_qualities_check', ...
          'simulink_actuator_compare', 'robustness_montecarlo_act'};

txt = fullfile(out_dir, 'console_output.txt');
if exist(txt, 'file'), delete(txt); end
diary(txt);
cleanup = onCleanup(@() diary('off'));
fprintf('Baseline created %s\n', datestr(now));          %#ok<TNOW1,DATST>
fprintf('Conditions: act_on = 0, sens_on = 0 for the first stage (the compare and\n');
fprintf('Monte Carlo scripts set their own actuator/sensor cases).\n');

for k = 1:numel(stages)
    fprintf('\n================ %s ================\n', stages{k});
    close all;
    assignin('base', 'act_on', 0);
    assignin('base', 'sens_on', 0);
    evalin('base', stages{k});
    figs = findall(0, 'type', 'figure');
    for f = 1:numel(figs)
        base = sprintf('%02d_%s_fig%d', k, stages{k}, f);
        try
            exportgraphics(figs(f), fullfile(out_dir, [base '.png']), 'Resolution', 200);
        catch
            saveas(figs(f), fullfile(out_dir, [base '.png']));
        end
        try
            savefig(figs(f), fullfile(out_dir, [base '.fig']));
        catch
        end
    end
end

diary('off');
fprintf('Baseline saved in %s\n', out_dir);
end
