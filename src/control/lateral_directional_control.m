%% LATERAL-DIRECTIONAL FLIGHT CONTROL: YAW DAMPER + ROLL DAMPER
% Linear lateral-directional model of the Navion at sea-level cruise (data: navion_params.m).
% States x = [r; beta; p; Phi], controls [xi; zeta] (aileron, rudder).

navion_params;

%% 1) Static lateral stability
fprintf('Cnbeta = %.4f (weathercock stability requires > 0)\n', Cnbeta);
fprintf('Clbeta = %.4f (dihedral effect requires < 0)\n', Clbeta);

%% 2) Dimensional derivatives
qbar0_lat = 0.5*rho0*V0^2;
Delta = Ixx*Izz - Ixz^2;

Ybeta = (qbar0_lat*S)/(m*V0) * (CYbeta - CD0);
Yp    = (qbar0_lat*S)/(m*V0) * (b/(2*V0)) * CYp;
Yr    = (qbar0_lat*S)/(m*V0) * (b/(2*V0)) * CYr;
Yxi   = (qbar0_lat*S)/(m*V0) * CYxi;
Yzeta = (qbar0_lat*S)/(m*V0) * CYzeta;

Lbeta = (qbar0_lat*S*b)/Delta * (Izz*Clbeta + Ixz*Cnbeta);
Lp    = (qbar0_lat*S*b)/Delta * (b/(2*V0)) * (Izz*Clp + Ixz*Cnp);
Lr    = (qbar0_lat*S*b)/Delta * (b/(2*V0)) * (Izz*Clr + Ixz*Cnr);
Lxi   = (qbar0_lat*S*b)/Delta * (Izz*Clxi + Ixz*Cnxi);
Lzeta = (qbar0_lat*S*b)/Delta * (Izz*Clzeta + Ixz*Cnzeta);

Nbeta = (qbar0_lat*S*b)/Delta * (Ixz*Clbeta + Ixx*Cnbeta);
Np    = (qbar0_lat*S*b)/Delta * (b/(2*V0)) * (Ixz*Clp + Ixx*Cnp);
Nr    = (qbar0_lat*S*b)/Delta * (b/(2*V0)) * (Ixz*Clr + Ixx*Cnr);
Nxi   = (qbar0_lat*S*b)/Delta * (Ixz*Clxi + Ixx*Cnxi);
Nzeta = (qbar0_lat*S*b)/Delta * (Ixz*Clzeta + Ixx*Cnzeta);

fprintf('\nDerivatives:\n');
fprintf('  Ybeta=%.4g  Yp=%.4g  Yr=%.4g\n', Ybeta, Yp, Yr);
fprintf('  Lbeta=%.4g  Lp=%.4g  Lr=%.4g\n', Lbeta, Lp, Lr);
fprintf('  Nbeta=%.4g  Np=%.4g  Nr=%.4g\n', Nbeta, Np, Nr);

%% 3) Open-loop model
A_ol_lat = [ Nr,               Nbeta,  Np,               0;
             Yr - cos(alpha0), Ybeta,  Yp + sin(alpha0), (g/V0)*cos(Theta0);
             Lr,               Lbeta,  Lp,               0;
             tan(Theta0),      0,      1,                0 ];

B_ol_lat = [ Nxi,  Nzeta;
             Yxi,  Yzeta;
             Lxi,  Lzeta;
             0,    0 ];

report_poles(A_ol_lat, 'Open-loop poles');

figure;
p = eig(A_ol_lat);
plot(real(p), imag(p), 'x', 'MarkerSize', 12, 'LineWidth', 2);
grid on; xlabel('Real'); ylabel('Imaginary'); title('Open-loop poles');

%% 4) Roll, Dutch-roll and spiral approximations (Nelson)
T_roll_approx    = -1/Lp;
omega0_DR_approx = sqrt(Nbeta + Nr*Ybeta);
zeta_DR_approx   = -(Nr + Ybeta)/(2*omega0_DR_approx);
inv_T_spiral_approx = (g/V0)*(Nbeta*Lr - Nr*Lbeta)/Nbeta;

fprintf('\nAnalytic approximations:\n');
fprintf('  Roll mode : T_R = %.4f s\n', T_roll_approx);
fprintf('  Dutch roll: wn=%.4f rad/s  zeta=%.4f\n', omega0_DR_approx, zeta_DR_approx);
fprintf('  Spiral    : 1/T_S = %+.4g\n', inv_T_spiral_approx);

%% 5) Yaw damper: zeta = zeta_c - Kzeta*r
% Gain search on r (state 1): all modes stable, Dutch-roll damping 0.5.
zeta_DR_target = 0.50;

Kzeta_range = linspace(-2, 2, 8000);
best_err = inf; Kzeta = 0; found_stable_zeta = false;
for k = Kzeta_range
    p_cl = eig(A_ol_lat - B_ol_lat(:,2)*[k 0 0 0]);
    if ~all(real(p_cl) < -1e-6)
        continue
    end
    cplx = p_cl(imag(p_cl) > 0);
    if ~isempty(cplx)
        found_stable_zeta = true;
        [~, idr] = max(abs(cplx));            % Dutch roll = highest-frequency complex pair
        err = abs(-real(cplx(idr))/abs(cplx(idr)) - zeta_DR_target);
        if err < best_err
            best_err = err; Kzeta = k;
        end
    end
end
if ~found_stable_zeta
    warning('No stabilizing Kzeta found.');
end
fprintf('\nKzeta = %.4f (target zeta_DR = %.2f)\n', Kzeta, zeta_DR_target);

A_yaw_damped = A_ol_lat - B_ol_lat(:,2)*[Kzeta 0 0 0];
report_poles(A_yaw_damped, sprintf('Yaw damper (Kzeta=%.4f)', Kzeta));

%% 6) Roll damper: xi = xi_c - Kxi*p, on top of the yaw damper
% Gain search on p (state 3): all modes stable, roll-mode time constant T_R = 0.14 s.
TR_target = 0.14;

real_ol = eig(A_yaw_damped);
real_ol = real_ol(abs(imag(real_ol)) < 1e-9);
TR_open = -1/min(real(real_ol));
Kxi_range = linspace(-5, 5, 8000);
best_err = inf; Kxi = 0; found_stable_xi = false;
for k = Kxi_range
    p_cl = eig(A_yaw_damped - B_ol_lat(:,1)*[0 0 k 0]);
    if ~all(real(p_cl) < -1e-6)
        continue
    end
    real_poles = p_cl(imag(p_cl) == 0);
    if ~isempty(real_poles)
        found_stable_xi = true;
        [~, i] = min(real(real_poles));
        TR_k = -1/real(real_poles(i));
        if TR_k > 0
            err = abs(TR_k - TR_target);
            if err < best_err
                best_err = err; Kxi = k;
            end
        end
    end
end
if ~found_stable_xi
    warning('No stabilizing Kxi found.');
end
if TR_open <= TR_target
    % open-loop roll mode already meets the target
    Kxi = 0;
    fprintf('Roll mode already fast (T_R = %.3f s <= %.2f s): no roll damper needed, Kxi = 0.\n', TR_open, TR_target);
end
fprintf('\nKxi = %.4f (target T_R = %.2f s)\n', Kxi, TR_target);

A_lat_hold = A_yaw_damped - B_ol_lat(:,1)*[0 0 Kxi 0];
report_poles(A_lat_hold, sprintf('Yaw + roll damper (Kzeta=%.4f, Kxi=%.4f)', Kzeta, Kxi));

% Crosswind gust input v/V: acts through the beta column of A (beta_aero = beta + v/V)
B_gust_lat = A_ol_lat(:,2);

% Simulink plant: A_ol_lat, inputs [xi; zeta; v/V0], C = eye(4)
B_lat_open = [B_ol_lat, B_gust_lat];

fprintf('\nKxi=%.4f  Kzeta=%.4f\n', Kxi, Kzeta);

%% 7) Crosswind 1-cosine gust response
t = 0:0.01:60;
gust_dur = 2*H_gust/V0;
gust = zeros(size(t));
idx = t <= gust_dur;
gust(idx) = (Vm/2)*(1 - cos(pi*t(idx)/(gust_dur/2)));
d_beta_gust = gust/V0;

sys_ol   = ss(A_ol_lat,       B_gust_lat, eye(4), zeros(4,1));
sys_yaw  = ss(A_yaw_damped,   B_gust_lat, eye(4), zeros(4,1));
sys_hold = ss(A_lat_hold,     B_gust_lat, eye(4), zeros(4,1));

y_ol   = lsim(sys_ol,   d_beta_gust, t);
y_yaw  = lsim(sys_yaw,  d_beta_gust, t);
y_hold = lsim(sys_hold, d_beta_gust, t);

fprintf('\nGust response, Phi [deg]:\n');
fprintf('  Open loop       : peak=%.3f  t=60s=%.3f\n', max(abs(rad2deg(y_ol(:,4)))),   rad2deg(y_ol(end,4)));
fprintf('  Yaw damper      : peak=%.3f  t=60s=%.3f\n', max(abs(rad2deg(y_yaw(:,4)))),  rad2deg(y_yaw(end,4)));
fprintf('  Yaw+roll damper : peak=%.3f  t=60s=%.3f\n', max(abs(rad2deg(y_hold(:,4)))), rad2deg(y_hold(end,4)));

figure;
subplot(3,1,1);
plot(t, rad2deg(y_ol(:,4))); grid on; ylabel('\Phi [deg]');
title('Open loop');
subplot(3,1,2);
plot(t, rad2deg(y_yaw(:,4))); grid on; ylabel('\Phi [deg]');
title(sprintf('Yaw damper (Kzeta=%.4f)', Kzeta));
subplot(3,1,3);
plot(t, rad2deg(y_hold(:,4))); grid on; ylabel('\Phi [deg]'); xlabel('time [s]');
title(sprintf('Yaw + roll damper (Kzeta=%.4f, Kxi=%.4f)', Kzeta, Kxi));

%% 8) Bank, heading and cross-track hold
% Cascade: y_err -> psi_cmd -> phi_cmd -> xi
%   psi_cmd = -Ky*y
%   phi_cmd = Kpsi*(psi_cmd - psi)
%   xi      = xi_c - Kxi*p - Kphi*(Phi - phi_cmd)
Kphi = -1.0;
Kpsi = 1.0;
Ky   = 0.005;

% States [r; beta; p; Phi; psi; y]: psi_dot = r, y_dot = V0*(psi + beta - v/V0)
A_trk = [A_ol_lat, zeros(4,2);
         1 0 0 0 0 0;
         0 V0 0 0 V0 0];
B_trk_xi   = [B_ol_lat(:,1); 0; 0];
B_trk_zeta = [B_ol_lat(:,2); 0; 0];
B_trk_gust = [B_gust_lat; 0; -V0];

A_trk_dampers = A_trk - B_trk_zeta*[Kzeta 0 0 0 0 0] - B_trk_xi*[0 0 Kxi 0 0 0];
A_trk_hold    = A_trk - B_trk_zeta*[Kzeta 0 0 0 0 0] ...
                      - B_trk_xi*[0 0 Kxi Kphi Kphi*Kpsi Kphi*Kpsi*Ky];

report_poles(A_trk_hold, sprintf('Track hold (Kphi=%.2f, Kpsi=%.2f, Ky=%.4f)', Kphi, Kpsi, Ky));

t_trk = 0:0.02:300;
tau_g = 4;
sig_g = 0.015;
n_seeds_trk = 20;
a_g = exp(-0.02/tau_g);
trk_names = {'Dampers', 'Track hold'};
trk_sys = {A_trk_dampers, A_trk_hold};
trk_bank = zeros(n_seeds_trk, 2);
trk_y = zeros(n_seeds_trk, 2);
for seed_trk = 1:n_seeds_trk
    rng(seed_trk);
    w_g = randn(numel(t_trk), 1);
    g_trk = zeros(numel(t_trk), 1);
    for k_g = 2:numel(t_trk)
        g_trk(k_g) = a_g*g_trk(k_g-1) + sqrt(1 - a_g^2)*sig_g*w_g(k_g);
    end
    for c_trk = 1:2
        y_trk = lsim(ss(trk_sys{c_trk}, B_trk_gust, eye(6), zeros(6,1)), g_trk, t_trk);
        trk_bank(seed_trk, c_trk) = sqrt(mean(rad2deg(y_trk(:,4)).^2));
        trk_y(seed_trk, c_trk)    = sqrt(mean(y_trk(:,6).^2));
    end
end

fprintf('\nColoured-noise gust, %d seeds, 300 s (mean):\n', n_seeds_trk);
for c_trk = 1:2
    fprintf('  %-11s bank rms = %.2f deg   cross-track rms = %.1f m\n', ...
        trk_names{c_trk}, mean(trk_bank(:,c_trk)), mean(trk_y(:,c_trk)));
end
fprintf('\nKphi=%.4f  Kpsi=%.4f  Ky=%.4f\n', Kphi, Kpsi, Ky);

%% Local functions
function report_poles(A, label)
    p = eig(A);
    fprintf('\n%s:\n', label);
    for i = 1:length(p)
        if imag(p(i)) == 0
            if real(p(i)) ~= 0
                fprintf('  s = %+8.4f            (real, T = %6.3f s)\n', real(p(i)), -1/real(p(i)));
            else
                fprintf('  s = %+8.4f            (integrator)\n', real(p(i)));
            end
        else
            wn = abs(p(i));
            fprintf('  s = %+8.4f %+8.4fi   wn=%.4f rad/s  zeta=%.4f\n', real(p(i)), imag(p(i)), wn, -real(p(i))/wn);
        end
    end
end
