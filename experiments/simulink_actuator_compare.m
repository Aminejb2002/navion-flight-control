% simulink_actuator_compare.m
% Full controller (dampers + alt/speed hold + track hold) under Dryden gust on three plants:
% ideal, + actuators, + actuators and sensors (act_on / sens_on), same seeds for all.
% Prints mean +/- std over seeds. Optional base variable sim_model selects the model (default 'Flight_simulator').

mdl = 'Flight_simulator';
if exist('sim_model', 'var'), mdl = sim_model; end
n_seeds = 10;
seed_base = 1000;

plants = {0, 0; 1, 0; 1, 1};                 % {act_on, sens_on}
plant_names = {'Ideal', '+Actuators', '+Act+Sensors'};
n_plants = size(plants, 1);

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
ref = sim(mdl);
nav_ref = ref.nav;

%% Gust runs (nav columns: [North East altitude roll pitch yaw]; deviations are from the reference)
rms_bank  = nan(n_seeds, n_plants);
rms_pitch = nan(n_seeds, n_plants);
rms_alt   = nan(n_seeds, n_plants);
rms_cross = nan(n_seeds, n_plants);
stable    = true(n_seeds, n_plants);

assignin('base', 'gust_on', 1);
for seed_idx = 1:n_seeds
    s0 = seed_base + 10 * seed_idx;
    set_param(dry_blk, seed_field, mat2str([s0 s0+1 s0+2 s0+3]));
    for p_idx = 1:n_plants
        assignin('base', 'act_on',  plants{p_idx, 1});
        assignin('base', 'sens_on', plants{p_idx, 2});
        r = sim(mdl);
        nav = r.nav;
        d_alt = nav(:, 3) - nav_ref(:, 3);
        rms_bank(seed_idx, p_idx)  = sqrt(mean(rad2deg(nav(:, 4)).^2));
        rms_pitch(seed_idx, p_idx) = sqrt(mean(rad2deg(nav(:, 5) - nav_ref(:, 5)).^2));
        rms_alt(seed_idx, p_idx)   = sqrt(mean(d_alt.^2));
        rms_cross(seed_idx, p_idx) = sqrt(mean((nav(:, 2) - nav_ref(:, 2)).^2));
        % bounded run: finite output and altitude deviation below 500 m
        stable(seed_idx, p_idx) = all(isfinite(nav(:))) && max(abs(d_alt)) < 500;
    end
    fprintf('seed %d/%d done\n', seed_idx, n_seeds);
end

set_param(dry_blk, seed_field, seed_old);
assignin('base', 'act_on', 1); assignin('base', 'sens_on', 0); assignin('base', 'gust_on', 1);

%% Summary
metrics = {rms_bank, rms_pitch, rms_alt, rms_cross};
metric_names = {'rms bank [deg]', 'rms pitch dev [deg]', 'rms altitude dev [m]', 'rms cross-track [m]'};

fprintf('\nMean +/- std over %d seeds (full controller)\n', n_seeds);
fprintf('%-24s', '');
for p_idx = 1:n_plants, fprintf('%-20s', plant_names{p_idx}); end
fprintf('\n');
for m_idx = 1:numel(metrics)
    fprintf('%-24s', metric_names{m_idx});
    for p_idx = 1:n_plants
        v = metrics{m_idx}(:, p_idx);
        fprintf('%-20s', sprintf('%.2f +/- %.2f', mean(v), std(v)));
    end
    fprintf('\n');
end
fprintf('%-24s', 'runs bounded');
for p_idx = 1:n_plants, fprintf('%-20s', sprintf('%d/%d', sum(stable(:, p_idx)), n_seeds)); end
fprintf('\n');
