%% LONGITUDINAL FLIGHT CONTROL: PITCH DAMPER + PITCH ATTITUDE HOLD
% Linear longitudinal model of the Navion at sea-level cruise (data: navion_params.m).
% States x = [V; gamma; alpha; q], augmented with theta = gamma + alpha.

navion_params;

%% 1) Static margin
SM = -Cmalpha/CLalpha;
fprintf('Static margin SM = %.3f\n', SM);

%% 2) Dimensional derivatives
qbar0 = 0.5*rho0*V0^2;

Xv   = -(qbar0*S)/(m*V0)   * (M0*dCD_dM + 2*CD0);
Xa   =  (qbar0*S)/m        * (CL0 - CDalpha);
Xq   = -(qbar0*S)/m        * (c/(2*V0)) * CDq;
Xeta = -(qbar0*S)/m        * CDeta;

Zv   = -(qbar0*S)/(m*V0^2) * (M0*dCL_dM + 2*CL0);
Za   = -(qbar0*S)/(m*V0)   * (CLalpha + CD0);
Zq   = -(qbar0*S)/(m*V0)   * (c/(2*V0)) * CLq;
Zeta = -(qbar0*S)/(m*V0)   * CLeta;

Mv   =  (1/Iyy)*(qbar0*S*c)/V0 * M0*dCm_dM;
Ma   =  (1/Iyy)* qbar0*S*c     * Cmalpha;
Mq   =  (1/Iyy)* qbar0*S*c * (c/(2*V0)) * Cmq;
Meta =  (1/Iyy)* qbar0*S*c     * Cmeta;

fprintf('\nDerivatives:\n');
fprintf('  Xv=%.4g  Xa=%.4g  Xq=%.4g\n', Xv, Xa, Xq);
fprintf('  Zv=%.4g  Za=%.4g  Zq=%.4g\n', Zv, Za, Zq);
fprintf('  Mv=%.4g  Ma=%.4g  Mq=%.4g\n', Mv, Ma, Mq);

%% 3) Open-loop model
A_ol = [ Xv,  -g,   Xa-g,  Xq;
        -Zv,   0,  -Za,   -Zq;
         Zv,   0,   Za,    Zq+1;
         Mv,   0,   Ma,    Mq ];
B_ol = [ Xeta; -Zeta; Zeta; Meta ];

report_poles(A_ol, 'Open-loop poles');

figure;
p = eig(A_ol);
plot(real(p), imag(p), 'x', 'MarkerSize', 12, 'LineWidth', 2);
grid on; xlabel('Real'); ylabel('Imaginary'); title('Open-loop poles');

%% 4) Short-period and phugoid approximations (Nelson)
omega0_SP_approx = sqrt(Za*Mq - Ma*(Zq+1));
zeta_SP_approx   = -(Za + Mq)/(2*omega0_SP_approx);
omega0_PH_approx = sqrt(2)*g/V0;
zeta_PH_approx   = (CD0/CL0)/sqrt(2);

fprintf('\nAnalytic approximations:\n');
fprintf('  Short period: wn=%.4f rad/s  zeta=%.4f\n', omega0_SP_approx, zeta_SP_approx);
fprintf('  Phugoid     : wn=%.4f rad/s  zeta=%.4f\n', omega0_PH_approx, zeta_PH_approx);

%% 5) Pitch damper: eta = eta_c - Kq*q
% Gain search (k < 0 and k > 0) on the short-period model [alpha; q],
% target zeta_SP = 0.70.
n_alpha = -Za*V0/g;
zeta_SP_target = 0.70;
CAP_target     = 1.00;
omega0_SP_target = sqrt(CAP_target*n_alpha);
fprintf('\nTarget: n_alpha=%.4f g/rad, wn=%.4f rad/s, zeta=%.2f\n', ...
    n_alpha, omega0_SP_target, zeta_SP_target);

A_sp = [Za, Zq+1; Ma, Mq];
B_sp = [Zeta; Meta];

Kq_range = linspace(-5, 5, 8000);
best_err = inf; Kq = 0;
for k = Kq_range
    p = eig(A_sp - B_sp*[0, k]);
    if imag(p(1)) ~= 0
        zt  = -real(p(1))/abs(p(1));
        err = abs(zt - zeta_SP_target);
        if err < best_err
            best_err = err; Kq = k;
        end
    end
end
if isinf(best_err)
    warning('No complex short-period poles found in the Kq search range.');
end

A_cl_sp = A_sp - B_sp*[0, Kq];
report_poles(A_cl_sp, sprintf('Closed-loop short period (Kq=%.4f)', Kq));
p_cl = eig(A_cl_sp);
fprintf('  CAP achieved = %.3f g/rad\n', abs(p_cl(1))^2/n_alpha);

%% 6) Attitude hold: eta = eta_c - Kq*q - Kp*theta
% Designed on the reduced [alpha; q; theta] model. Since theta = gamma + alpha,
% the 5-state model carries an invariant s = 0 mode (theta - gamma - alpha = const).
A_aug3   = [A_cl_sp, zeros(2,1); 0 1 0];
eta_col3 = [B_sp; 0];

Kp_range = linspace(-5, 5, 20001);
target_pole = -0.5;
best_err = inf; Kp = 0; found_stable = false;
for k = Kp_range
    p = eig(A_aug3 - eta_col3*[0 0 k]);
    if all(real(p) < -1e-6)
        found_stable = true;
        real_poles = p(imag(p) == 0);
        if ~isempty(real_poles)
            [~, i] = min(abs(real_poles - target_pole));
            err = abs(real_poles(i) - target_pole);
            if err < best_err
                best_err = err; Kp = k;
            end
        end
    end
end
if ~found_stable
    warning('No stabilizing Kp found.');
end
report_poles(A_aug3 - eta_col3*[0 0 Kp], sprintf('Reduced model with attitude hold (Kp=%.4f)', Kp));

%% 7) Full 5-state model, x = [V; gamma; alpha; q; theta]
A_aug = [A_ol, zeros(4,1); 0 0 0 1 0];
B_aug = [B_ol; 0];

A_damped = A_aug - B_aug*[0 0 0 Kq 0];
A_hold   = A_damped - B_aug*[0 0 0 0 Kp];

report_poles(A_damped, 'Full model: pitch damper');
report_poles(A_hold,   sprintf('Full model: pitch damper + attitude hold (Kp=%.4f)', Kp));

% Vertical gust input w/V: acts through the alpha column of the aerodynamic
% terms (alpha_aero = alpha + w/V); gravity and kinematic terms excluded.
B_gust_lon = [Xa; -Za; Za; Ma; 0];

% Thrust input [per N], along the body x-axis through the CG
XdT = cos(alpha0)/m;
ZdT = -sin(alpha0)/(m*V0);
B_thrust = [XdT; -ZdT; ZdT; 0; 0];

% Simulink plant: A_aug, inputs [eta; w/V0; dT], C = eye(5)
B_lon_open = [B_aug, B_gust_lon, B_thrust];

fprintf('\nKq=%.4f  Kp=%.4f\n', Kq, Kp);

%% 8) Vertical 1-cosine gust response
t = 0:0.01:60;
gust_dur = 2*H_gust/V0;
gust = zeros(size(t));
idx = t <= gust_dur;
gust(idx) = (Vm/2)*(1 - cos(pi*t(idx)/(gust_dur/2)));
d_alpha_gust = gust/V0;

sys_ol   = ss(A_aug,    B_gust_lon, eye(5), zeros(5,1));
sys_damp = ss(A_damped, B_gust_lon, eye(5), zeros(5,1));
sys_hold = ss(A_hold,   B_gust_lon, eye(5), zeros(5,1));

y_ol   = lsim(sys_ol,   d_alpha_gust, t);
y_damp = lsim(sys_damp, d_alpha_gust, t);
y_hold = lsim(sys_hold, d_alpha_gust, t);

fprintf('\nGust response, theta [deg]:\n');
fprintf('  Open loop  : peak=%.3f  t=60s=%.3f\n', max(abs(rad2deg(y_ol(:,5)))),   rad2deg(y_ol(end,5)));
fprintf('  Damper     : peak=%.3f  t=60s=%.3f\n', max(abs(rad2deg(y_damp(:,5)))), rad2deg(y_damp(end,5)));
fprintf('  Damper+hold: peak=%.3f  t=60s=%.3f\n', max(abs(rad2deg(y_hold(:,5)))), rad2deg(y_hold(end,5)));

%% 8b) Altitude hold: PI on altitude error -> theta command
% theta_cmd = Kh*(h_cmd - h) + Ki*integral(h_cmd - h),  eta_c = Kp*theta_cmd
% Design model: [V; gamma; alpha; q; h] with theta = gamma + alpha and the
% pitch damper / attitude hold (Kq, Kp) closed.
Kh = 0.012;     % [rad/m]
Ki = 0.0008;    % [rad/(m s)]

A_h = [A_ol, zeros(4,1); 0 V0 0 0 0];
B_h = [B_ol; 0];
th_row = [0 1 1 0 0];
eh_row = [0 0 0 0 1];
K_in   = [0 0 0 Kq 0] + Kp*th_row;
A_alt  = [A_h - B_h*K_in - B_h*Kp*Kh*eh_row,  B_h*Kp*Ki;
          -eh_row,                            0];
report_poles(A_alt, sprintf('Altitude hold (Kh=%.4f, Ki=%.5f)', Kh, Ki));

A_noalt = A_h - B_h*K_in;
sys_h_noalt = ss(A_noalt, [B_gust_lon(1:4); 0], eh_row, 0);
sys_h_alt   = ss(A_alt,   [B_gust_lon(1:4); 0; 0], [eh_row 0], 0);
dh_noalt = lsim(sys_h_noalt, d_alpha_gust, t);
dh_alt   = lsim(sys_h_alt,   d_alpha_gust, t);
fprintf('\nAltitude deviation after gust [m]:\n');
fprintf('  Dampers only     : peak=%.2f  t=60s=%.2f\n', max(abs(dh_noalt)), dh_noalt(end));
fprintf('  + altitude hold  : peak=%.2f  t=60s=%.2f\n', max(abs(dh_alt)),   dh_alt(end));
fprintf('\nKh=%.4f  Ki=%.5f\n', Kh, Ki);

%% 8c) Speed hold: PI on airspeed -> thrust
% dT = -(Kv*dV + Kvi*integral(dV)) [N], designed with the altitude loop closed.
% State order: [V; gamma; alpha; q; h; integral(h error); integral(dV)].
Kv  = 200;      % [N/(m/s)]
Kvi = 10;       % [N/m]

A_full = zeros(7);
A_full(1:5,1:5) = A_h - B_h*K_in - B_h*Kp*Kh*eh_row;
A_full(1:5,6)   = B_h*Kp*Ki;
A_full(6,5)     = -1;
A_full(1:5,1)   = A_full(1:5,1) - B_thrust(1:5)*Kv;
A_full(1:5,7)   = -B_thrust(1:5)*Kvi;
A_full(7,1)     = 1;
report_poles(A_full, sprintf('Altitude + speed hold (Kv=%.0f, Kvi=%.0f)', Kv, Kvi));

C_chk = [1 0 0 0 0 0 0; 0 0 0 0 1 0 0; -Kv 0 0 0 0 0 -Kvi];   % dV, dh, dT
y_full = lsim(ss(A_full, [B_gust_lon(1:4); 0; 0; 0], C_chk, zeros(3,1)), d_alpha_gust, t);
fprintf('\nGust response with altitude + speed hold:\n');
fprintf('  peak dV=%.2f m/s  peak dh=%.2f m  peak thrust=%.0f N\n', ...
    max(abs(y_full(:,1))), max(abs(y_full(:,2))), max(abs(y_full(:,3))));
fprintf('\nKv=%.0f  Kvi=%.0f\n', Kv, Kvi);

figure;
subplot(3,1,1);
plot(t, rad2deg(y_ol(:,5))); grid on; ylabel('\theta [deg]');
title('Open loop');
subplot(3,1,2);
plot(t, rad2deg(y_damp(:,5))); grid on; ylabel('\theta [deg]');
title(sprintf('Pitch damper (Kq=%.4f)', Kq));
subplot(3,1,3);
plot(t, rad2deg(y_hold(:,5))); grid on; ylabel('\theta [deg]'); xlabel('time [s]');
title(sprintf('Pitch damper + attitude hold (Kq=%.4f, Kp=%.4f)', Kq, Kp));

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
