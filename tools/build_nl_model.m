function build_nl_model(mdl)
% Converts Flight_simulator_nl (a copy of the linear model) to the nonlinear plant.
%   1. Linear State-Space plants -> Goto/From tags (output layouts unchanged):
%        lon_u = [elevator; gust(alpha_g); thrust]   lon_y = [V; gamma; alpha; q; theta]
%        lat_u = [aileron; rudder; gust(beta_g)]     lat_y = [r; beta; p; phi]
%   2. Kinematics subsystem -> "Nonlinear Aircraft" (12-state rigid body), same outports.
%   3. nl_setup is appended to the model InitFcn.
% Usage: build_nl_model [(mdl)]   -- the model is modified in memory, not saved.

if nargin < 1, mdl = 'Flight_simulator_nl'; end
if strcmp(mdl, 'Flight_simulator')
    error('Refusing to modify the baseline model. Use the Save-As copy Flight_simulator_nl.');
end
if ~bdIsLoaded(mdl), load_system(mdl); end
if ~isempty(find_system(mdl, 'SearchDepth', 2, 'Name', 'Nonlinear Aircraft'))
    error('%s already contains a Nonlinear Aircraft block. Re-open the model from disk (bdclose, open) to start over.', mdl);
end

ac  = [mdl '/Aircraft Model'];
lon = [ac '/Longitudinal Control Loop'];
lat = [ac '/Lateral-Directional Control Loop'];
kin = [ac '/Position, Attitude and Flight-Path Kinematics'];

% ---- 1. plants -> tags
swap_plant(lon, 'State-Space',  'lon_u', 'lon_y');
swap_plant(lat, 'State-Space1', 'lat_u', 'lat_y');

% ---- 2. kinematics -> nonlinear aircraft
ph  = get_param(kin, 'PortHandles');
pos = get_param(kin, 'Position');
out_names = find_port_names(kin, 'Outport');
expect = {'North', 'East', 'altitude', 'pitch', 'yaw', 'DCM', 'Vo'};
if ~isequal(out_names(:)', expect)
    error('Kinematics outputs are [%s], expected [%s].', strjoin(out_names, ', '), strjoin(expect, ', '));
end

% store outport destinations, then disconnect the subsystem
dsts = cell(1, 7);
for k = 1:7
    lh = get_param(ph.Outport(k), 'Line');
    if lh ~= -1
        dsts{k} = get_param(lh, 'DstPortHandle');
        delete_line(lh);
    end
end
for k = 1:numel(ph.Inport)
    lh = get_param(ph.Inport(k), 'Line');
    if lh == -1, continue, end
    src = get_param(lh, 'SrcPortHandle');
    delete_line(lh);
    % terminate sources left unconnected
    if src > 0 && get_param(src, 'Line') == -1
        sp = get_param(src, 'Position');
        t = add_block('simulink/Sinks/Terminator', [ac '/Term_' num2str(k)], ...
            'Position', [sp(1)+30 sp(2)-8 sp(1)+50 sp(2)+8]);
        tph = get_param(t, 'PortHandles');
        add_line(ac, src, tph.Inport(1));
    end
end
delete_block(kin);

nl = [ac '/Nonlinear Aircraft'];
build_nl_subsystem(nl, pos);
nph = get_param(nl, 'PortHandles');
for k = 1:7
    for j = 1:numel(dsts{k})
        if dsts{k}(j) > 0, add_line(ac, nph.Outport(k), dsts{k}(j)); end
    end
end

% ---- 3. InitFcn
init = get_param(mdl, 'InitFcn');
if ~contains(init, 'nl_setup')
    set_param(mdl, 'InitFcn', [init newline 'nl_setup;']);
end

try
    set_param(mdl, 'SimulationCommand', 'update');
catch ME
    fprintf('Diagram update reported: %s\n', ME.message);
end
fprintf('Done. Nonlinear Aircraft built in %s. Check the diagram, then save (Ctrl+S).\n', mdl);
end

% ----------------------------------------------------------------------
function swap_plant(sub, name, tag_in, tag_out)
ss = [sub '/' name];
if isempty(find_system(sub, 'SearchDepth', 1, 'Name', name))
    error('Block %s not found.', ss);
end
ph  = get_param(ss, 'PortHandles');
pos = get_param(ss, 'Position');

% input line: source and remaining destinations
li   = get_param(ph.Inport(1), 'Line');
src  = get_param(li, 'SrcPortHandle');
dall = get_param(li, 'DstPortHandle');
dall = dall(dall ~= ph.Inport(1) & dall > 0);
delete_line(li);

% output line destinations
lo   = get_param(ph.Outport(1), 'Line');
dout = get_param(lo, 'DstPortHandle');
dout = dout(dout > 0);
delete_line(lo);

delete_block(ss);

g = add_block('simulink/Signal Routing/Goto', [sub '/Goto_' tag_in], 'GotoTag', tag_in, ...
    'TagVisibility', 'global', 'Position', [pos(1) pos(2)+60 pos(1)+60 pos(2)+80]);
gp = get_param(g, 'PortHandles');
add_line(sub, src, gp.Inport(1));
for j = 1:numel(dall), add_line(sub, src, dall(j)); end

f = add_block('simulink/Signal Routing/From', [sub '/From_' tag_out], 'GotoTag', tag_out, ...
    'Position', [pos(1) pos(2) pos(1)+60 pos(2)+20]);
fp = get_param(f, 'PortHandles');
for j = 1:numel(dout), add_line(sub, fp.Outport(1), dout(j)); end
end

% ----------------------------------------------------------------------
function names = find_port_names(sys, type)
b = find_system(sys, 'SearchDepth', 1, 'BlockType', type);
num = zeros(numel(b), 1);  names = cell(numel(b), 1);
for k = 1:numel(b)
    num(k) = str2double(get_param(b{k}, 'Port'));
    names{k} = get_param(b{k}, 'Name');
end
[~, i] = sort(num);
names = names(i);
end

% ----------------------------------------------------------------------
function build_nl_subsystem(nl, pos)
add_block('built-in/SubSystem', nl, 'Position', pos);

outs = {'North', 'East', 'altitude', 'pitch', 'yaw', 'DCM', 'Vo'};
for k = 1:7
    add_block('simulink/Sinks/Out1', [nl '/' outs{k}], 'Port', num2str(k), ...
        'Position', [900 40+50*k 930 54+50*k]);
end

add_block('simulink/Signal Routing/From', [nl '/From_lon_u'], 'GotoTag', 'lon_u', 'Position', [40 60 100 80]);
add_block('simulink/Signal Routing/From', [nl '/From_lat_u'], 'GotoTag', 'lat_u', 'Position', [40 110 100 130]);
add_block('simulink/Continuous/Integrator', [nl '/states'], 'InitialCondition', 'nl_x0', ...
    'Position', [520 60 560 100]);
add_block('simulink/User-Defined Functions/MATLAB Function', [nl '/f_dyn'], 'Position', [250 50 400 140]);
add_block('simulink/User-Defined Functions/MATLAB Function', [nl '/f_out'], 'Position', [620 40 780 380]);
add_block('simulink/Signal Routing/Goto', [nl '/Goto_lon_y'], 'GotoTag', 'lon_y', 'TagVisibility', 'global', ...
    'Position', [820 440 880 460]);
add_block('simulink/Signal Routing/Goto', [nl '/Goto_lat_y'], 'GotoTag', 'lat_y', 'TagVisibility', 'global', ...
    'Position', [820 480 880 500]);

dyn = [ ...
'function xdot = f_dyn(x, lon_u, lat_u)\n' ...
'%% 12-state nonlinear Navion, inputs are deviations from trim\n' ...
'u = [lon_u(1); lat_u(1); lat_u(2); lon_u(3)];   %% elevator, aileron, rudder, thrust\n' ...
'g = [lat_u(3); lon_u(2)];                       %% beta_g, alpha_g [rad]\n' ...
'xdot = navion_nl_fcn(x, u, g);\n'];

outc = [ ...
'function [lon_y, lat_y, N, E, h, theta, psi, DCM, Va] = f_out(x)\n' ...
'P  = nl_const();\n' ...
'Vg = sqrt(x(1)^2 + x(2)^2 + x(3)^2);\n' ...
'st = sin(x(8)); ct = cos(x(8)); sp = sin(x(7)); cp = cos(x(7)); ss = sin(x(9)); cs = cos(x(9));\n' ...
'Rb2n = [ct*cs, sp*st*cs - cp*ss, cp*st*cs + sp*ss;\n' ...
'        ct*ss, sp*st*ss + cp*cs, cp*st*ss - sp*cs;\n' ...
'        -st,   sp*ct,            cp*ct];\n' ...
'vn    = Rb2n * x(1:3);\n' ...
'gamma = asin(max(-1, min(1, -vn(3)/Vg)));\n' ...
'alpha = atan2(x(3), x(1));\n' ...
'beta  = asin(max(-1, min(1, x(2)/Vg)));\n' ...
'%% deviations from trim\n' ...
'lon_y = [Vg - P.V0; gamma; alpha - P.alpha_tr; x(5); x(8) - P.theta_tr];\n' ...
'lat_y = [x(6); beta; x(4); x(7)];\n' ...
'N = x(10); E = x(11); h = x(12); theta = x(8); psi = x(9);\n' ...
'DCM = Rb2n.''; %% earth -> body, as the Dryden block expects\n' ...
'Va = Vg;\n'];

set_chart_code([nl '/f_dyn'], sprintf(dyn));
set_chart_code([nl '/f_out'], sprintf(outc));
fix_nl_blocks(bdroot(nl));

% wiring
add_line(nl, 'states/1', 'f_dyn/1');
add_line(nl, 'From_lon_u/1', 'f_dyn/2');
add_line(nl, 'From_lat_u/1', 'f_dyn/3');
add_line(nl, 'f_dyn/1', 'states/1');
add_line(nl, 'states/1', 'f_out/1');
add_line(nl, 'f_out/1', 'Goto_lon_y/1');
add_line(nl, 'f_out/2', 'Goto_lat_y/1');
for k = 1:7
    add_line(nl, sprintf('f_out/%d', k+2), sprintf('%s/1', outs{k}));
end
end

function set_chart_code(blk, code)
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);
if isempty(ch), error('Could not access the MATLAB Function block %s.', blk); end
ch.Script = code;
end
