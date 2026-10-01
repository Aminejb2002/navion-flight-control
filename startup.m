% Adds the project folders to the MATLAB path.
% Runs automatically when MATLAB starts in this folder; otherwise run once per session.
root = fileparts(mfilename('fullpath'));
folders = {'src/params', 'src/linear', 'src/control', 'src/nonlinear', ...
           'analysis', 'experiments', 'tests', 'tools', 'visualization', 'models'};
for k = 1:numel(folders)
    addpath(fullfile(root, folders{k}));
end
