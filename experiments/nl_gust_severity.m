function nl_gust_severity(scales, nseeds, act)
% Gust-scale sweep, linear (Flight_simulator) vs nonlinear (Flight_simulator_nl) model, hold + track controller, same seeds.
%   nl_gust_severity                  scales [0.5 1 2 3 4 6], 5 seeds, ideal actuators
%   nl_gust_severity([1 2 4], 3, 1)   custom scales and seed count, actuators + sensors on
% Prints mean-over-seeds metrics per scale and plots them against gust scale.
if nargin < 1, scales = [0.5 1 2 3 4 6]; end
if nargin < 2, nseeds = 5; end
if nargin < 3, act = 0; end
mdls = {'Flight_simulator', 'Flight_simulator_nl'};
for m_i = 1:2
    if ~bdIsLoaded(mdls{m_i}), load_system(mdls{m_i}); end
    blk{m_i} = find_seed_block(mdls{m_i});
    dp = fieldnames(get_param(blk{m_i}, 'DialogParameters'));
    sf{m_i} = dp{find(contains(lower(dp), 'seed'), 1)};
    old{m_i} = get_param(blk{m_i}, sf{m_i});
end
cleanup = onCleanup(@() cellfun(@(b, f, s) set_param(b, f, s), blk, sf, old));
assignin('base', 'act_on', act);  assignin('base', 'sens_on', act);
assignin('base', 'fb_on', 1); assignin('base', 'alt_on', 1); assignin('base', 'trk_on', 1);

ns = numel(scales);
% M(scale, model[lin nl], metric, seed); metrics: rms alt, rms bank, max bank, rms cross, bounded
M = nan(ns, 2, 5, nseeds);
ref = cell(1, 2);
% gust-free reference run per model
assignin('base', 'gust_on', 0);
for m_i = 1:2, ref{m_i} = sim(mdls{m_i}); end
for si = 1:nseeds
    s0 = 2000 + 10*si;
    for m_i = 1:2
        set_param(blk{m_i}, sf{m_i}, mat2str([s0 s0+1 s0+2 s0+3]));
    end
    for k = 1:ns
        assignin('base', 'gust_on', scales(k));
        for m_i = 1:2
            % nav columns: [North East altitude roll pitch yaw]; "bounded" = |alt dev| < 100 m and |bank| < 45 deg
            r = sim(mdls{m_i});  nav = r.nav;  nr = ref{m_i}.nav;  n = min(size(nav,1), size(nr,1));
            da = nav(1:n,3) - nr(1:n,3);
            M(k, m_i, 1, si) = rms(da);
            M(k, m_i, 2, si) = rms(rad2deg(nav(1:n,4)));
            M(k, m_i, 3, si) = max(abs(rad2deg(nav(1:n,4))));
            M(k, m_i, 4, si) = rms(nav(1:n,2) - nr(1:n,2));
            M(k, m_i, 5, si) = max(abs(da)) < 100 && max(abs(rad2deg(nav(1:n,4)))) < 45;
        end
    end
    fprintf('seed %d/%d done\n', si, nseeds);
end
assignin('base', 'gust_on', 1);

%% Results
fprintf('\nGust severity sweep, hold + track, %d seeds, actuators/sensors %s (mean over seeds)\n', nseeds, onoff(act));
fprintf('%-6s | %-23s | %-23s | %-23s | %-23s | %s\n', 'scale', 'rms altitude [m]', 'rms bank [deg]', 'max bank [deg]', 'rms cross-track [m]', 'bounded');
fprintf('%-6s | %-11s %-11s | %-11s %-11s | %-11s %-11s | %-11s %-11s | %s\n', '', 'linear', 'nonlinear', 'linear', 'nonlinear', 'linear', 'nonlinear', 'linear', 'nonlinear', 'lin/nl');
for k = 1:ns
    mm = squeeze(mean(M(k,:,:,:), 4));      % [model x metric]
    fprintf('%-6.2f | %-11.2f %-11.2f | %-11.2f %-11.2f | %-11.2f %-11.2f | %-11.2f %-11.2f | %d/%d, %d/%d\n', scales(k), ...
        mm(1,1), mm(2,1), mm(1,2), mm(2,2), mm(1,3), mm(2,3), mm(1,4), mm(2,4), ...
        round(mm(1,5)*nseeds), nseeds, round(mm(2,5)*nseeds), nseeds);
end

figure('Name', 'Gust severity', 'Color', 'w', 'Position', [80 80 1000 380]);
lab = {'rms altitude [m]', 'rms bank [deg]', 'rms cross-track [m]'};  idx = [1 2 4];
for p = 1:3
    subplot(1, 3, p);  hold on;  grid on;
    plot(scales, squeeze(mean(M(:,1,idx(p),:), 4)), '-o', 'LineWidth', 1.5);
    plot(scales, squeeze(mean(M(:,2,idx(p),:), 4)), '-s', 'LineWidth', 1.5);
    xlabel('gust scale'); ylabel(lab{p});
    if p == 1, legend('linear model', 'nonlinear model', 'Location', 'northwest'); end
end
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
function s = onoff(a), if a, s = 'ON'; else, s = 'OFF'; end, end
