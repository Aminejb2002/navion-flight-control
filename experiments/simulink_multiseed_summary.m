% simulink_multiseed_summary.m
% Gust response over several Dryden seeds for four controller configurations
% (open loop, dampers, alt/speed hold, hold + track). Prints mean +/- std per metric and
% plots bar charts. Optional base variable sim_model selects the model (default 'Flight_simulator').

mdl = 'Flight_simulator';
if exist('sim_model', 'var'), mdl = sim_model; end
n_seeds = 10;
seed_base = 1000;

% columns: fb_on, alt_on, gust_on, trk_on
cases = {0, 0, 1, 0; 1, 0, 1, 0; 1, 1, 1, 0; 1, 1, 1, 1};
case_names = {'Open loop', 'Dampers', 'Alt/speed hold', 'Hold + track'};

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
    if any(contains(lower(dp), 'seed')) && contains(lower(get_param(cand{cand_idx}, 'MaskType')), 'dryden')
        dry_blk = cand{cand_idx};
        break
    end
end
if isempty(dry_blk)
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
end
if isempty(dry_blk)
    fprintf('Blocks inside the wrapper:\n');
    fprintf('  %s\n', cand{:});
    error('No block with a seed parameter found.');
end
dp = fieldnames(get_param(dry_blk, 'DialogParameters'));
seed_field = dp{find(contains(lower(dp), 'seed'), 1)};
seed_old = get_param(dry_blk, seed_field);
fprintf('Seed block: %s  (%s = %s)\n', dry_blk, seed_field, seed_old);

%% Reference run: open loop, gust off
assignin('base', 'trk_on', 0); assignin('base', 'fb_on', 0); assignin('base', 'alt_on', 0); assignin('base', 'gust_on', 0);
ref = sim(mdl);
t_ref = ref.tout;
nav_ref = ref.nav;

%% Gust runs (nav columns: [North East altitude roll pitch yaw]; deviations are from the reference)
rms_bank = nan(n_seeds, 4);
rms_pitch = nan(n_seeds, 4);
rms_alt = nan(n_seeds, 4);
max_alt = nan(n_seeds, 4);
rms_along = nan(n_seeds, 4);
rms_cross = nan(n_seeds, 4);

for seed_idx = 1:n_seeds
    s0 = seed_base + 10 * seed_idx;
    set_param(dry_blk, seed_field, mat2str([s0 s0+1 s0+2 s0+3]));
    for case_idx = 1:4
        assignin('base', 'trk_on',  cases{case_idx, 4});
        assignin('base', 'fb_on',   cases{case_idx, 1});
        assignin('base', 'alt_on',  cases{case_idx, 2});
        assignin('base', 'gust_on', cases{case_idx, 3});
        r = sim(mdl);
        nav = r.nav;
        d_alt = nav(:, 3) - nav_ref(:, 3);
        rms_bank(seed_idx, case_idx)  = rms(rad2deg(nav(:, 4)));
        rms_pitch(seed_idx, case_idx) = rms(rad2deg(nav(:, 5) - nav_ref(:, 5)));
        rms_alt(seed_idx, case_idx)   = rms(d_alt);
        max_alt(seed_idx, case_idx)   = max(abs(d_alt));
        rms_along(seed_idx, case_idx) = rms(nav(:, 1) - nav_ref(:, 1));
        rms_cross(seed_idx, case_idx) = rms(nav(:, 2) - nav_ref(:, 2));
    end
    fprintf('seed %d/%d done\n', seed_idx, n_seeds);
end

set_param(dry_blk, seed_field, seed_old);
assignin('base', 'fb_on', 1); assignin('base', 'alt_on', 1); assignin('base', 'gust_on', 1); assignin('base', 'trk_on', 1);

%% Summary
metrics = {rms_bank, rms_pitch, rms_alt, max_alt, rms_along, rms_cross};
metric_names = {'rms bank [deg]', 'rms pitch dev [deg]', 'rms altitude dev [m]', ...
                'max |altitude dev| [m]', 'rms along-track [m]', 'rms cross-track [m]'};

fprintf('\nMean +/- std over %d seeds\n', n_seeds);
fprintf('%-26s', '');
for case_idx = 1:4, fprintf('%-20s', case_names{case_idx}); end
fprintf('\n');
for m_idx = 1:numel(metrics)
    fprintf('%-26s', metric_names{m_idx});
    for case_idx = 1:4
        v = metrics{m_idx}(:, case_idx);
        fprintf('%-20s', sprintf('%.2f +/- %.2f', mean(v), std(v)));
    end
    fprintf('\n');
end

set(0, 'DefaultFigureVisible', 'on');
figure(20); clf;
col = [0.90 0.40 0.15; 0.10 0.45 0.85; 0.10 0.70 0.45; 0.55 0.30 0.75];
for m_idx = 1:numel(metrics)
    subplot(2, 3, m_idx); hold on; grid on
    mu = mean(metrics{m_idx}); sd = std(metrics{m_idx});
    for case_idx = 1:4
        bar(case_idx, mu(case_idx), 'FaceColor', col(case_idx, :));
    end
    errorbar(1:4, mu, sd, 'k', 'LineStyle', 'none', 'LineWidth', 1.2);
    set(gca, 'XTick', 1:4, 'XTickLabel', {'Open', 'Damp', 'Hold', 'Track'});
    title(metric_names{m_idx});
end
