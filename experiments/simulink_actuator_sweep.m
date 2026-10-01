% simulink_actuator_sweep.m
% One-at-a-time sensitivity sweep of actuator and sensor parameters (servo bandwidth,
% rate limit, thrust lag, sensor lag) with the full controller under Dryden gust.
% Other parameters stay nominal; deviations are relative to the ideal, gust-free plant.
% Prints one table per parameter and plots rms metrics. Optional base variable sim_model
% selects the model (default 'Flight_simulator').

mdl = 'Flight_simulator';
if exist('sim_model', 'var'), mdl = sim_model; end
n_seeds   = 3;
seed_base = 1000;

nom = struct('wn_act', 15, 'rate_lim', deg2rad(30), 'tau_thr', 1.0, 'tau_sens', 0.02);

% scale converts the stored value to the displayed unit
sweeps = struct('var', {}, 'vals', {}, 'label', {}, 'scale', {}, 'unit', {});
sweeps(1) = struct('var', 'wn_act',   'vals', [1 2 3 5 15],         'label', 'servo natural freq',  'scale', 1,         'unit', 'rad/s');
sweeps(2) = struct('var', 'rate_lim', 'vals', deg2rad([0.5 1 2 3 5 30]), 'label', 'surface rate limit', 'scale', 180/pi, 'unit', 'deg/s');
sweeps(3) = struct('var', 'tau_thr',  'vals', [1 4 8 16 32],        'label', 'thrust lag',          'scale', 1,         'unit', 's');
sweeps(4) = struct('var', 'tau_sens', 'vals', [0.02 0.25 0.5 1 2], 'label', 'sensor lag',      'scale', 1,         'unit', 's');

%% Locate the Dryden seed block
wrapper = [mdl '/Aircraft Model/Gust Disturbance (Dryden)'];
cand = find_system(wrapper, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'SubSystem');
dry_blk = '';
for cand_idx = 1:numel(cand)
    try
        dp = fieldnames(get_param(cand{cand_idx}, 'DialogParameters'));
    catch
        continue
    end
    if any(contains(lower(dp), 'seed'))
        dry_blk = cand{cand_idx};
        break
    end
end
if isempty(dry_blk), error('No block with a seed parameter found.'); end
dp = fieldnames(get_param(dry_blk, 'DialogParameters'));
seed_field = dp{find(contains(lower(dp), 'seed'), 1)};
seed_old = get_param(dry_blk, seed_field);

%% Reference run: ideal plant, loops on, gust off
assignin('base', 'fb_on', 1); assignin('base', 'alt_on', 1); assignin('base', 'trk_on', 1);
assignin('base', 'gust_on', 0); assignin('base', 'act_on', 0); assignin('base', 'sens_on', 0);
sw_ref = sim(mdl);
nav_ref = sw_ref.nav;

assignin('base', 'gust_on', 1);
assignin('base', 'act_on', 1);
assignin('base', 'sens_on', 1);

%% Sweep (nav columns: [North East altitude roll pitch yaw])
results = cell(numel(sweeps), 1);
for sw = 1:numel(sweeps)
    sw_vals = sweeps(sw).vals;
    sw_res = nan(numel(sw_vals), 5);      % rms bank, pitch dev, alt dev, cross-track, bounded fraction
    for v_idx = 1:numel(sw_vals)
        assignin('base', 'wn_act',   nom.wn_act);
        assignin('base', 'rate_lim', nom.rate_lim);
        assignin('base', 'tau_thr',  nom.tau_thr);
        assignin('base', 'tau_sens', nom.tau_sens);
        assignin('base', sweeps(sw).var, sw_vals(v_idx));

        sw_m = nan(n_seeds, 4);
        sw_ok = false(n_seeds, 1);
        for seed_idx = 1:n_seeds
            s0 = seed_base + 10 * seed_idx;
            set_param(dry_blk, seed_field, mat2str([s0 s0+1 s0+2 s0+3]));
            try
                sw_r = sim(mdl);
                sw_nav = sw_r.nav;
                d_alt = sw_nav(:, 3) - nav_ref(:, 3);
                sw_m(seed_idx, :) = [ ...
                    sqrt(mean(rad2deg(sw_nav(:, 4)).^2)), ...
                    sqrt(mean(rad2deg(sw_nav(:, 5) - nav_ref(:, 5)).^2)), ...
                    sqrt(mean(d_alt.^2)), ...
                    sqrt(mean((sw_nav(:, 2) - nav_ref(:, 2)).^2))];
                % bounded run: finite output, altitude deviation < 500 m, rms bank < 10 deg
                sw_ok(seed_idx) = all(isfinite(sw_nav(:))) && max(abs(d_alt)) < 500 && sqrt(mean(rad2deg(sw_nav(:, 4)).^2)) < 10;
            catch sw_err
                fprintf('  run failed: %s\n', sw_err.message);
                sw_ok(seed_idx) = false;
            end
        end
        sw_res(v_idx, :) = [mean(sw_m, 1, 'omitnan'), mean(sw_ok)];
        fprintf('%s = %.3g done\n', sweeps(sw).var, sw_vals(v_idx));
    end
    results{sw} = sw_res;
end

set_param(dry_blk, seed_field, seed_old);
evalin('base', 'clear wn_act rate_lim tau_thr tau_sens');   % model InitFcn restores nominal values
assignin('base', 'act_on', 1); assignin('base', 'sens_on', 0); assignin('base', 'gust_on', 1);

%% Print
fprintf('\nSensitivity sweep (mean over %d seeds, full controller, nominal marked *)\n', n_seeds);
for sw = 1:numel(sweeps)
    fprintf('\n%s [%s]\n', sweeps(sw).label, sweeps(sw).unit);
    fprintf('%10s %10s %10s %10s %10s %8s\n', 'value', 'bank', 'pitch', 'alt', 'cross', 'bounded');
    for v_idx = 1:numel(sweeps(sw).vals)
        mark = ' ';
        if abs(sweeps(sw).vals(v_idx) - nom.(sweeps(sw).var)) < 1e-9, mark = '*'; end
        sw_row = results{sw}(v_idx, :);
        fprintf('%9.3g%s %10.2f %10.2f %10.2f %10.2f %8.2f\n', ...
            sweeps(sw).vals(v_idx) * sweeps(sw).scale, mark, sw_row(1), sw_row(2), sw_row(3), sw_row(4), sw_row(5));
    end
end

%% Figure
figure(30); clf;
names = {'rms bank [deg]', 'rms pitch dev [deg]', 'rms altitude dev [m]', 'rms cross-track [m]'};
for sw = 1:numel(sweeps)
    for k = 1:4
        subplot(4, 4, (k - 1) * 4 + sw); hold on; grid on
        plot(sweeps(sw).vals * sweeps(sw).scale, results{sw}(:, k), 'o-', 'LineWidth', 1.4);
        if k == 1, title(sprintf('%s [%s]', sweeps(sw).label, sweeps(sw).unit)); end
        if sw == 1, ylabel(names{k}); end
        set(gca, 'XScale', 'log');
    end
end
