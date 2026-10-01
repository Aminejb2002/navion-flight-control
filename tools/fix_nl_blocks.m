function fix_nl_blocks(mdl)
% Sets explicit port sizes on the f_dyn and f_out MATLAB Function blocks of the
% Nonlinear Aircraft subsystem (the integrator loop leaves widths unspecified).
% Usage: fix_nl_blocks [(mdl)]
if nargin < 1, mdl = 'Flight_simulator_nl'; end
nl = [mdl '/Aircraft Model/Nonlinear Aircraft'];
set_sizes([nl '/f_dyn'], {'x','12'; 'lon_u','3'; 'lat_u','3'; 'xdot','12'});
set_sizes([nl '/f_out'], {'x','12'; 'lon_y','5'; 'lat_y','4'; 'N','1'; 'E','1'; 'h','1'; ...
                          'theta','1'; 'psi','1'; 'DCM','[3 3]'; 'Va','1'});
fprintf('Port sizes set.\n');
end

function set_sizes(blk, list)
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);
if isempty(ch), error('Block %s not found.', blk); end
for k = 1:size(list, 1)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', list{k,1});
    if isempty(d), error('Data %s not found in %s.', list{k,1}, blk); end
    d(1).Props.Array.Size = list{k,2};
end
end
