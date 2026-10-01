% nl_vs_linear.m
function nl_vs_linear(cmp_seed, cmp_scales)
% Linear (Flight_simulator) vs nonlinear (Flight_simulator_nl) model, same Dryden seed and controller case.
%   nl_vs_linear   or   nl_vs_linear(1010, [0.1 1])   (seed, gust scales)
% Prints rms deviation from each model's gust-free run; rel diff = rms(dev_NL - dev_lin) / rms(dev_lin).
lin_mdl = 'Flight_simulator';
nl_mdl  = 'Flight_simulator_nl';
if nargin < 1, cmp_seed = 1010; end
if nargin < 2, cmp_scales = 0.1; end     % gust_on values (gust scale)
cases = {0 0 0; 1 0 0; 1 1 0; 1 1 1};          % fb_on, alt_on, trk_on
case_names = {'Open loop', 'Dampers', 'Alt/speed hold', 'Hold + track'};
% nav columns: [North East altitude roll pitch yaw]; angles are reported in deg
sig_names  = {'North', 'East', 'altitude', 'bank', 'pitch', 'yaw'};
sig_cols   = [1 2 3 4 5 6];

mdls = {lin_mdl, nl_mdl};
for m_i = 1:2
    if ~bdIsLoaded(mdls{m_i}), load_system(mdls{m_i}); end
    blk{m_i} = find_seed_block(mdls{m_i});
    fld = get_param(blk{m_i}, 'DialogParameters');  fld = fieldnames(fld);
    sf{m_i} = fld{find(contains(lower(fld), 'seed'), 1)};
    old_seed{m_i} = get_param(blk{m_i}, sf{m_i});
    set_param(blk{m_i}, sf{m_i}, mat2str([cmp_seed cmp_seed+1 cmp_seed+2 cmp_seed+3]));
end
cleanup = onCleanup(@() cellfun(@(b, f, s) set_param(b, f, s), blk, sf, old_seed));

assignin('base', 'act_on', 0);  assignin('base', 'sens_on', 0);
fprintf('\nSeed %d.  rms of deviation from the gust-free run;  last column: rms(NL - lin)/rms(lin)\n', cmp_seed);
for sc = cmp_scales
    fprintf('\n=== gust scale %.2f ===\n', sc);
    fprintf('%-16s %-9s %12s %12s %10s\n', 'case', 'signal', 'linear', 'nonlinear', 'rel diff');
    for c = 1:4
        assignin('base', 'fb_on', cases{c,1});  assignin('base', 'alt_on', cases{c,2});
        assignin('base', 'trk_on', cases{c,3});
        dev = cell(1, 2);  tt = cell(1, 2);
        for m_i = 1:2
            assignin('base', 'gust_on', 0);   r0 = sim(mdls{m_i});
            assignin('base', 'gust_on', sc);  r1 = sim(mdls{m_i});
            n = min(size(r0.nav, 1), size(r1.nav, 1));
            dev{m_i} = r1.nav(1:n, :) - r0.nav(1:n, :);
            tt{m_i} = r1.tout(1:n);
        end
        n = min(size(dev{1}, 1), size(dev{2}, 1));
        for s = 1:numel(sig_cols)
            a = dev{1}(1:n, sig_cols(s));  b = dev{2}(1:n, sig_cols(s));
            sc_u = 1;  if ismember(sig_cols(s), [4 5 6]), sc_u = 180/pi; end
            fprintf('%-16s %-9s %12.4g %12.4g %9.1f%%\n', case_names{c}, sig_names{s}, ...
                sc_u*rms(a), sc_u*rms(b), 100*rms(b - a) / max(rms(a), eps));
        end
        % sideslip-like term chi - psi, with chi approximated as (dE/dt)/V0 (small-angle) [deg]
        V0n = 53.64;
        for m_i = 1:2
            dE = gradient(dev{m_i}(:,2), tt{m_i});
            xe = rad2deg(dE / V0n - dev{m_i}(:,6));
            fprintf('%-16s %-9s %s: rms %.4g  mean %.4g deg   | yaw mean %.4g deg\n', case_names{c}, 'chi-psi', mdls{m_i}, rms(xe), mean(xe), rad2deg(mean(dev{m_i}(:,6))));
        end
    end
end
assignin('base', 'fb_on', 1); assignin('base', 'alt_on', 1); assignin('base', 'trk_on', 1); assignin('base', 'gust_on', 1);
end

function blk = find_seed_block(mdl)
% Returns the block of the Dryden gust subsystem that exposes a seed parameter.
wrapper = [mdl '/Aircraft Model/Gust Disturbance (Dryden)'];
cand = find_system(wrapper, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'SubSystem');
blk = '';
for k = 1:numel(cand)
    try, dp = fieldnames(get_param(cand{k}, 'DialogParameters')); catch, continue, end
    if any(contains(lower(dp), 'seed')), blk = cand{k}; return, end
end
error('No seed block found in %s.', mdl);
end
