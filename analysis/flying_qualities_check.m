% flying_qualities_check.m
% Checks the open-loop and closed-loop linear modes against MIL-F-8785C style
% Class I Level 1 limits. Run as a script; prints a pass/fail table per
% configuration. Set fq_category ('A', 'B' or 'C') below.

vis_old = get(0, 'DefaultFigureVisible');
set(0, 'DefaultFigureVisible', 'off');
evalc('longitudinal_pitch_control');
evalc('lateral_directional_control');
close all;
set(0, 'DefaultFigureVisible', 'on');

%% Limits
fq_category = 'B';

% sp_zeta: [min max]; ph_zeta: min; dr_*: Dutch roll zeta, zeta*wn, wn minimum;
% roll_T: max roll-mode time constant [s]; spiral_T2: min time to double [s]
lim = struct();
lim.A = struct('sp_zeta', [0.35 1.30], 'ph_zeta', 0.04, 'dr_zeta', 0.19, 'dr_zwn', 0.35, 'dr_wn', 1.0, 'roll_T', 1.0, 'spiral_T2', 12);
lim.B = struct('sp_zeta', [0.30 2.00], 'ph_zeta', 0.04, 'dr_zeta', 0.08, 'dr_zwn', 0.15, 'dr_wn', 0.4, 'roll_T', 1.4, 'spiral_T2', 20);
lim.C = struct('sp_zeta', [0.35 1.30], 'ph_zeta', 0.04, 'dr_zeta', 0.08, 'dr_zwn', 0.15, 'dr_wn', 1.0, 'roll_T', 1.0, 'spiral_T2', 12);
L = lim.(fq_category);

lon_names = {'Open loop', 'Pitch damper', 'Pitch damper + attitude hold'};
lon_A = {A_ol, A_damped, A_hold};
lat_names = {'Open loop', 'Yaw damper', 'Yaw + roll damper'};
lat_A = {A_ol_lat, A_yaw_damped, A_lat_hold};

pf = @(ok) ternary_str(ok, 'ok  ', 'FAIL');

%% Report
fprintf('Flying qualities, Class I, Category %s, Level 1 limits\n', fq_category);

fprintf('\nLongitudinal\n');
fprintf('%-30s %-26s %-22s\n', '', 'short period', 'phugoid');
for k_c = 1:numel(lon_A)
    [sp, ph] = long_modes(lon_A{k_c});
    ok_sp = sp.zeta >= L.sp_zeta(1) && sp.zeta <= L.sp_zeta(2);
    ok_ph = isnan(ph.zeta) || ph.zeta >= L.ph_zeta;
    fprintf('%-30s zeta=%5.3f wn=%5.2f  %s   zeta=%6.3f  %s\n', lon_names{k_c}, ...
        sp.zeta, sp.wn, pf(ok_sp), ph.zeta, pf(ok_ph));
end
fprintf('  limits: short-period zeta %.2f to %.2f, phugoid zeta >= %.2f\n', L.sp_zeta(1), L.sp_zeta(2), L.ph_zeta);

fprintf('\nLateral-directional\n');
fprintf('%-30s %-40s %-18s %-20s\n', '', 'Dutch roll', 'roll mode', 'spiral');
for k_c = 1:numel(lat_A)
    m = lat_modes(lat_A{k_c});
    % zeta*wn requirement grows with the phi/beta ratio of the Dutch roll mode
    zwn_req = L.dr_zwn + max(0, 0.014*(m.dr_wn^2*m.dr_phibeta - 20));
    ok_dr = m.dr_zeta >= L.dr_zeta && m.dr_zeta*m.dr_wn >= zwn_req && m.dr_wn >= L.dr_wn;
    ok_rl = m.roll_T <= L.roll_T;
    if m.spiral_pole >= 0
        t2 = log(2)/m.spiral_pole;
        ok_sr = t2 >= L.spiral_T2;
        sr_str = sprintf('T2=%6.1f s', t2);
    else
        ok_sr = true;
        sr_str = sprintf('stable (T=%.0f s)', -1/m.spiral_pole);
    end
    fprintf('%-30s zeta=%5.3f wn=%5.2f zwn=%5.2f  %s  T=%5.2f s  %s  %s  %s\n', lat_names{k_c}, ...
        m.dr_zeta, m.dr_wn, m.dr_zeta*m.dr_wn, pf(ok_dr), m.roll_T, pf(ok_rl), sr_str, pf(ok_sr));
end
fprintf('  limits: Dutch roll zeta >= %.2f, zeta*wn >= %.2f, wn >= %.2f;  roll T <= %.1f s;  spiral T2 >= %d s\n', ...
    L.dr_zeta, L.dr_zwn, L.dr_wn, L.roll_T, L.spiral_T2);

%% Local functions
function [sp, ph] = long_modes(A)
    % Short period = faster complex pair, phugoid = slower one (NaN if absent).
    p = eig(A);
    p = p(imag(p) > 1e-9);
    [~, order] = sort(abs(p), 'descend');
    p = p(order);
    sp.wn = abs(p(1));  sp.zeta = -real(p(1))/sp.wn;
    if numel(p) >= 2
        ph.wn = abs(p(end));  ph.zeta = -real(p(end))/ph.wn;
    else
        ph.wn = NaN;  ph.zeta = NaN;
    end
end

function m = lat_modes(A)
    % State order [v p r phi]. Dutch roll = complex pair; roll mode = most negative
    % real pole; spiral = least negative (or positive) real pole.
    p = eig(A);
    [V, D] = eig(A);
    lam = diag(D);
    cidx = find(imag(lam) > 1e-9);
    [~, jdr] = max(abs(lam(cidx)));
    idr = cidx(jdr);
    m.dr_wn = abs(lam(idr));
    m.dr_zeta = -real(lam(idr))/m.dr_wn;
    % |phi/beta| of the Dutch roll eigenvector (v is proportional to beta)
    m.dr_phibeta = abs(V(4, idr)/V(2, idr));
    r = sort(real(p(abs(imag(p)) < 1e-9)));
    m.roll_T = -1/r(1);
    m.spiral_pole = r(end);
end

function s = ternary_str(cond, a, b)
    if cond, s = a; else, s = b; end
end
