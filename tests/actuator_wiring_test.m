% actuator_wiring_test.m
% Checks that the surface rate limiter and the sensor lag act on the control loops.
% Runs the full controller under gust (model's current seed) with nominal and extreme
% parameters and prints bank, pitch and altitude rms. Optional base variable sim_model
% selects the model (default 'Flight_simulator').

mdl = 'Flight_simulator';
if exist('sim_model', 'var'), mdl = sim_model; end
assignin('base', 'fb_on', 1); assignin('base', 'alt_on', 1);
assignin('base', 'trk_on', 1); assignin('base', 'gust_on', 1);

% columns: label, act_on, sens_on, rate_lim [rad/s], tau_sens [s]
tests = { ...
    'nominal (act on, sens on)',      1, 1, deg2rad(30),  0.02;
    'rate limit 0.2 deg/s',           1, 1, deg2rad(0.2), 0.02;
    'sensor lag 2 s',                 1, 1, deg2rad(30),  2;
    'actuators OFF, sensors OFF',     0, 0, deg2rad(30),  0.02};

fprintf('\n%-30s %12s %12s %12s\n', 'case', 'bank rms', 'pitch rms', 'alt rms');
for t_idx = 1:size(tests, 1)
    assignin('base', 'act_on',   tests{t_idx, 2});
    assignin('base', 'sens_on',  tests{t_idx, 3});
    assignin('base', 'rate_lim', tests{t_idx, 4});
    assignin('base', 'tau_sens', tests{t_idx, 5});
    wt_r = sim(mdl);
    wt_nav = wt_r.nav;
    fprintf('%-30s %12.4f %12.4f %12.3f\n', tests{t_idx, 1}, ...
        sqrt(mean(rad2deg(wt_nav(:, 4)).^2)), ...
        sqrt(mean(rad2deg(wt_nav(:, 5) - wt_nav(1, 5)).^2)), ...
        sqrt(mean((wt_nav(:, 3) - mean(wt_nav(:, 3))).^2)));
end

evalin('base', 'clear rate_lim tau_sens');   % model InitFcn restores nominal values
assignin('base', 'act_on', 1); assignin('base', 'sens_on', 0);
% Expected: the rate-limit and sensor-lag cases differ clearly from nominal.
fprintf('\nIf "rate limit 0.2" equals nominal -> rate limiter not wired.\n');
fprintf('If "sensor lag 2 s" leaves pitch rms equal to nominal -> q sensor not in the path.\n');
