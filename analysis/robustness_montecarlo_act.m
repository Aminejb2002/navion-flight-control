% robustness_montecarlo_act.m
% Monte Carlo over aircraft parameter uncertainty on the linear model, with
% actuators, sensor lags and Dryden-shaped gust. Each sample is evaluated on the
% ideal and on the realistic plant. Run as a script; prints nominal margins and
% summary tables, and plots rms histograms (ideal vs realistic) for scale 1.

vis_old = get(0, 'DefaultFigureVisible');
set(0, 'DefaultFigureVisible', 'off');
evalc('longitudinal_pitch_control');
evalc('lateral_directional_control');
close all;
set(0, 'DefaultFigureVisible', 'on');

%% Settings
n_samples   = 2000;
seed_mc     = 1;
range_scale = 1;

% rms gust angle [rad], calibrated so the ideal-plant rms altitude matches the
% Simulink result (3.6 m); the lateral value balances bank (1.6 deg) and
% cross-track (6 m).
cfg.sig_lon = 0.039;
cfg.sig_lat = 0.020;
cfg.L_g     = 533;       % Dryden scale length [m]
cfg.tau_s   = 0.02;      % sensor lag [s]
cfg.lag_lon = 4;         % lagged states: q
cfg.lag_lat = [1 3 4];   % lagged states: r, p, phi
act_wn      = 15;        % servo natural frequency [rad/s]
act_zeta    = 0.7;
tau_thr     = 1.0;       % thrust lag [s]
cfg.act_lon = struct('type', {'second', 'first'}, 'wn', {act_wn, 0}, ...
                     'zeta', {act_zeta, 0}, 'tau', {0, tau_thr});
cfg.act_lat = struct('type', {'second', 'second'}, 'wn', {act_wn, act_wn}, ...
                     'zeta', {act_zeta, act_zeta}, 'tau', {0, 0});

P = struct('V0', V0, 'alpha0', alpha0, 'Theta0', Theta0, 'rho0', rho0, 'M0', M0, 'g', g, ...
    'S', S, 'c', c, 'b', b, 'm', m, 'Ixx', Ixx, 'Iyy', Iyy, 'Izz', Izz, 'Ixz', Ixz, ...
    'CL0', CL0, 'CLalpha', CLalpha, 'CLq', CLq, 'CLeta', CLeta, ...
    'CD0', CD0, 'CDalpha', CDalpha, 'CDq', CDq, 'CDeta', CDeta, ...
    'Cmalpha', Cmalpha, 'Cmq', Cmq, 'Cmeta', Cmeta, ...
    'dCL_dM', dCL_dM, 'dCD_dM', dCD_dM, 'dCm_dM', dCm_dM, ...
    'CYbeta', CYbeta, 'CYp', CYp, 'CYr', CYr, ...
    'Clbeta', Clbeta, 'Clp', Clp, 'Clr', Clr, ...
    'Cnbeta', Cnbeta, 'Cnp', Cnp, 'Cnr', Cnr, ...
    'CYxi', CYxi, 'Clxi', Clxi, 'Cnxi', Cnxi, ...
    'CYzeta', CYzeta, 'Clzeta', Clzeta, 'Cnzeta', Cnzeta);
G = struct('Kq', Kq, 'Kp', Kp, 'Kh', Kh, 'Ki', Ki, 'Kv', Kv, 'Kvi', Kvi, ...
    'Kzeta', Kzeta, 'Kxi', Kxi, 'Kphi', Kphi, 'Kpsi', Kpsi, 'Ky', Ky);
V0_nom = V0;

% Rebuilt nominal model must match the matrices from the design scripts
M0_ = navion_models(P);
err = max([norm(M0_.A_ol - A_ol, 'fro'), norm(M0_.B_ol - B_ol, 'fro'), ...
           norm(M0_.A_ol_lat - A_ol_lat, 'fro'), norm(M0_.B_ol_lat - B_ol_lat, 'fro'), ...
           norm(M0_.B_gust_lon - B_gust_lon, 'fro'), norm(M0_.B_gust_lat - B_gust_lat, 'fro'), ...
           norm(M0_.B_thrust - B_thrust, 'fro')]);
fprintf('Model rebuild check (max Frobenius error vs scripts): %.2e\n', err);
if err > 1e-9
    error('navion_models does not reproduce the nominal matrices.');
end

%% Nominal checks and margins
Ln = closed_loops(M0_, G, V0_nom);

% Loop builder without actuators/sensors must reproduce the designed closed loops
[Ac, Bc, Cc, Dc] = ssdata(Ln.lon.Ct);
A_chk = build_realistic_loop(Ln.lon.A_plant, Ln.lon.B_plant, [M0_.B_gust_lon(1:4); 0], Ac, Bc, Cc, Dc, [], [], 0);
e_ref = sort(real(eig(Ln.A_full)));  e_new = sort(real(eig(A_chk)));
fprintf('Builder check, longitudinal: max eigenvalue difference %.2e\n', max(abs(e_ref - e_new)));
[Ac, Bc, Cc, Dc] = ssdata(Ln.lat.Ct);
A_chk = build_realistic_loop(Ln.lat.A_plant, Ln.lat.B_plant, Ln.B_trk_gust, Ac, Bc, Cc, Dc, [], [], 0);
e_ref = sort(real(eig(Ln.A_trk_hold)));  e_new = sort(real(eig(A_chk)));
fprintf('Builder check, lateral:      max eigenvalue difference %.2e\n', max(abs(e_ref - e_new)));

[Act_lon, Sens_lon] = actuator_ss(cfg.act_lon, 5, cfg.lag_lon, cfg.tau_s);
[Act_lat, Sens_lat] = actuator_ss(cfg.act_lat, 6, cfg.lag_lat, cfg.tau_s);
mg_lon = loop_margins(Ln.lon.A_plant, Ln.lon.B_plant, Ln.lon.Ct, {'elevator', 'thrust'}, Act_lon, Sens_lon);
mg_lat = loop_margins(Ln.lat.A_plant, Ln.lat.B_plant, Ln.lat.Ct, {'aileron', 'rudder'}, Act_lat, Sens_lat);
fprintf('\nNominal margins with actuators and sensors (other loops closed):\n');
fprintf('  %-10s %-26s %-14s\n', 'input', 'gain range [dB]', 'delay margin');
for mg = [mg_lon, mg_lat]
    if isinf(mg.delay), ds = '> 0.60 s'; else, ds = sprintf('%.3f s', mg.delay); end
    fprintf('  %-10s %+6.1f ... %+6.1f          %s\n', mg.name, 20*log10(mg.k_lo), 20*log10(mg.k_hi), ds);
end

%% Monte Carlo over uncertainty scale
names  = {'CLalpha','CLq','CLeta','CDalpha','CD0','Cmalpha','Cmq','Cmeta', ...
          'CYbeta','Clbeta','Clp','Clr','Cnbeta','Cnp','Cnr','Clxi','Cnxi', ...
          'CYzeta','Clzeta','Cnzeta','m','Ixx','Iyy','Izz'};
ranges = [0.2 0.2 0.2 0.2 0.2 0.3 0.2 0.2, ...
          0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2, ...
          0.2 0.2 0.2 0.1 0.15 0.15 0.15];
% Uniform relative ranges per parameter; scales multiply them (1 = +/-20 %, 3 = +/-60 %)
scales = [1 1.5 2 3];
n_sc   = numel(scales);

fprintf('\nMonte Carlo, %d samples per scale (scale 1: derivatives +/-20%%, Cm_alpha +/-30%%, mass +/-10%%, inertia +/-15%%)\n', n_samples);
fprintf('Actuators: servo wn %.0f rad/s zeta %.2f, thrust lag %.1f s, sensor lag %.3f s\n', act_wn, act_zeta, tau_thr, cfg.tau_s);
fprintf('Gust: Dryden transverse, L = %.0f m, rms %.3f rad (lon), %.3f rad (lat)\n', cfg.L_g, cfg.sig_lon, cfg.sig_lat);

summ = nan(n_sc, 10);
for sc_idx = 1:n_sc
    range_scale = scales(sc_idx);
    rng(seed_mc);

    st_lon_i = false(n_samples,1); st_lon_r = false(n_samples,1);
    st_lat_i = false(n_samples,1); st_lat_r = false(n_samples,1);
    alt_i = nan(n_samples,1);  alt_r = nan(n_samples,1);
    bank_i = nan(n_samples,1); bank_r = nan(n_samples,1);
    y_i = nan(n_samples,1);    y_r = nan(n_samples,1);
    sp_zeta = nan(n_samples,1); dr_zeta = nan(n_samples,1);
    dr_zwn = nan(n_samples,1);  dr_wn = nan(n_samples,1);
    roll_T = nan(n_samples,1);  spiral = nan(n_samples,1);
    dr_zeta_ol = nan(n_samples,1); dr_zwn_ol = nan(n_samples,1); dr_wn_ol = nan(n_samples,1);

    for s_idx = 1:n_samples
        Pi = P;
        for k_p = 1:numel(names)
            Pi.(names{k_p}) = P.(names{k_p}) * (1 + range_scale*ranges(k_p)*(2*rand - 1));
        end
        Mi = navion_models(Pi);
        Li = closed_loops(Mi, G, Pi.V0);

        Rm = realistic_metrics(Li, Mi, Pi.V0, cfg);
        st_lon_i(s_idx) = Rm.stab_lon_ideal;  st_lon_r(s_idx) = Rm.stab_lon_real;
        st_lat_i(s_idx) = Rm.stab_lat_ideal;  st_lat_r(s_idx) = Rm.stab_lat_real;
        alt_i(s_idx) = Rm.alt_rms_ideal;    alt_r(s_idx) = Rm.alt_rms_real;
        bank_i(s_idx) = Rm.bank_rms_ideal;  bank_r(s_idx) = Rm.bank_rms_real;
        y_i(s_idx) = Rm.y_rms_ideal;        y_r(s_idx) = Rm.y_rms_real;

        ml = mode_params(Li.A_lon_hold, 'lon');
        sp_zeta(s_idx) = ml.sp_zeta;
        md = mode_params(Li.A_lat_dampers, 'lat');
        dr_zeta(s_idx) = md.dr_zeta;  dr_wn(s_idx) = md.dr_wn;
        dr_zwn(s_idx)  = md.dr_zeta*md.dr_wn;  roll_T(s_idx) = md.roll_T;
        spiral(s_idx)  = md.spiral_pole;
        mo = mode_params(Mi.A_ol_lat, 'lat');
        dr_zeta_ol(s_idx) = mo.dr_zeta;  dr_wn_ol(s_idx) = mo.dr_wn;
        dr_zwn_ol(s_idx)  = mo.dr_zeta*mo.dr_wn;
    end

    % Level 1 criteria (Class I): short period and Dutch roll (Cat B, Cat A), roll mode (Cat B)
    sp_ok_B = sp_zeta >= 0.30 & sp_zeta <= 2.00;
    dr_ok_B = dr_zeta >= 0.08 & dr_zwn >= 0.15 & dr_wn >= 0.4;
    dr_ok_A = dr_zeta >= 0.19 & dr_zwn >= 0.35 & dr_wn >= 1.0;
    dr_ok_A_ol = dr_zeta_ol >= 0.19 & dr_zwn_ol >= 0.35 & dr_wn_ol >= 1.0;
    dr_ok_B_ol = dr_zeta_ol >= 0.08 & dr_zwn_ol >= 0.15 & dr_wn_ol >= 0.4;
    rl_ok_B = roll_T <= 1.4;

    % columns: stab lon, stab lat, SP B, DR B, DR A, DR B OL, DR A OL, roll B, alt p50, alt p95
    pct = @(x, p) prctile_safe(x(~isnan(x)), p);
    summ(sc_idx, :) = [100*mean(st_lon_r), 100*mean(st_lat_r), 100*mean(sp_ok_B), ...
        100*mean(dr_ok_B), 100*mean(dr_ok_A), 100*mean(dr_ok_B_ol), 100*mean(dr_ok_A_ol), ...
        100*mean(rl_ok_B), pct(alt_r, 50), pct(alt_r, 95)];

    if sc_idx == 1
        % detailed print and histogram data for scale 1
        fprintf('\n--- scale 1 detail ---\n');
        fprintf('Stable closed loops           ideal     realistic\n');
        fprintf('  alt + speed hold          %6.1f %%   %6.1f %%\n', 100*mean(st_lon_i), 100*mean(st_lon_r));
        fprintf('  cross-track hold          %6.1f %%   %6.1f %%\n', 100*mean(st_lat_i), 100*mean(st_lat_r));
        fprintf('rms under gust, 5th / median / 95th percentile   ideal                     realistic\n');
        fprintf('  altitude [m]          %7.2f / %7.2f / %7.2f     %7.2f / %7.2f / %7.2f\n', ...
            pct(alt_i,5), pct(alt_i,50), pct(alt_i,95), pct(alt_r,5), pct(alt_r,50), pct(alt_r,95));
        fprintf('  bank [deg]            %7.2f / %7.2f / %7.2f     %7.2f / %7.2f / %7.2f\n', ...
            pct(bank_i,5), pct(bank_i,50), pct(bank_i,95), pct(bank_r,5), pct(bank_r,50), pct(bank_r,95));
        fprintf('  cross-track [m]       %7.2f / %7.2f / %7.2f     %7.2f / %7.2f / %7.2f\n', ...
            pct(y_i,5), pct(y_i,50), pct(y_i,95), pct(y_r,5), pct(y_r,50), pct(y_r,95));
        alt_i1 = alt_i; alt_r1 = alt_r; bank_i1 = bank_i; bank_r1 = bank_r; y_i1 = y_i; y_r1 = y_r;
    end
    fprintf('scale %g done\n', range_scale);
end

fprintf('\nResults versus uncertainty size (realistic plant, all values in %% unless noted)\n');
fprintf('%-7s %-9s %-9s | %-10s %-10s %-10s | %-14s %-14s | %-9s | %-12s\n', ...
    'scale', 'stab lon', 'stab lat', 'SP Cat B', 'DR Cat B', 'DR Cat A', 'DR Cat B (OL)', 'DR Cat A (OL)', 'roll B', 'alt med/p95');
for sc_idx = 1:n_sc
    r_ = summ(sc_idx, :);
    fprintf('%-7g %-9.1f %-9.1f | %-10.1f %-10.1f %-10.1f | %-14.1f %-14.1f | %-9.1f | %.1f / %.1f m\n', ...
        scales(sc_idx), r_(1), r_(2), r_(3), r_(4), r_(5), r_(6), r_(7), r_(8), r_(9), r_(10));
end
fprintf('(OL = open loop, no control system)\n');

alt_i = alt_i1; alt_r = alt_r1; bank_i = bank_i1; bank_r = bank_r1; y_i = y_i1; y_r = y_r1;

figure('Name', 'Monte Carlo realistic', 'Color', 'w', 'Position', [60 60 1200 650]);
data_i = {alt_i, bank_i, y_i};
data_r = {alt_r, bank_r, y_r};
ttl = {'rms altitude [m]', 'rms bank [deg]', 'rms cross-track [m]'};
for k_f = 1:3
    subplot(1, 3, k_f); hold on
    xi = data_i{k_f}(~isnan(data_i{k_f}));
    xr = data_r{k_f}(~isnan(data_r{k_f}));
    edges = linspace(min([xi; xr]), max([xi; xr]), 31);
    ci = histc(xi, edges);  cr = histc(xr, edges);
    stairs(edges, ci, 'LineWidth', 1.4);
    stairs(edges, cr, 'LineWidth', 1.4);
    grid on; title(ttl{k_f});
    if k_f == 1, legend('ideal', 'realistic'); end
end

function q = prctile_safe(x, p)
    % Nearest-rank percentile (p in percent); NaN for empty input.
    x = sort(x(:));
    if isempty(x), q = NaN; return; end
    idx = max(1, min(numel(x), round(p/100*numel(x))));
    q = x(idx);
end
