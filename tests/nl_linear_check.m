% nl_linear_check.m
% Verifies the nonlinear Navion model against the linear model. Run as a script; it
% trims at the design condition, linearizes numerically (central differences), and
% prints the eigenvalues, mode properties and control derivatives next to navion_models.

P = nl_params();
[x0, u0, tr] = trim_navion(P, P.V0, 500);

fprintf('Trim at V = %.2f m/s, h = 500 m:\n', P.V0);
fprintf('  alpha = %.4f rad (linear model: %.4f)   eta = %.5f rad   T = %.1f N   residual %.1e\n', ...
    tr.alpha, P.alpha0, tr.eta, tr.T, tr.residual);

%% Numerical linearization (12 states, 4 inputs)
n = 12;  mu = 4;
A = zeros(n);  B = zeros(n, mu);
w0 = zeros(3, 1);
for j = 1:n
    d = zeros(n, 1);  d(j) = 1e-6 * max(1, abs(x0(j)));
    A(:, j) = (navion_nl(x0 + d, u0, w0, P) - navion_nl(x0 - d, u0, w0, P)) / (2*d(j));
end
for j = 1:mu
    d = zeros(mu, 1);  d(j) = 1e-6;
    B(:, j) = (navion_nl(x0, u0 + d, w0, P) - navion_nl(x0, u0 - d, w0, P)) / (2*d(j));
end

% state indices of the decoupled subsets
lon = [1 3 5 8];            % u w q theta
lat = [2 4 6 7];            % v p r phi
A_lon = A(lon, lon);
A_lat = A(lat, lat);

%% Linear baseline
M = navion_models(P);

show = @(name, e_lin, e_nl) fprintf('%s\n   linear   : %s\n   nonlinear: %s\n', name, fmt(e_lin), fmt(e_nl));

fprintf('\nLongitudinal eigenvalues (short period and phugoid):\n');
show('', sort_modes(eig(M.A_ol)), sort_modes(eig(A_lon)));
fprintf('\nLateral-directional eigenvalues (Dutch roll, roll, spiral):\n');
show('', sort_modes(eig(M.A_ol_lat)), sort_modes(eig(A_lat)));

%% Mode properties
ml = mode_params(M.A_ol, 'lon');   mn = mode_params(A_lon, 'lon');
fprintf('\nShort period : zeta %.3f vs %.3f,  wn %.3f vs %.3f rad/s  (linear vs nonlinear)\n', ml.sp_zeta, mn.sp_zeta, ml.sp_wn, mn.sp_wn);
fprintf('Phugoid      : zeta %.4f vs %.4f\n', ml.ph_zeta, mn.ph_zeta);
dl = mode_params(M.A_ol_lat, 'lat');  dn = mode_params(A_lat, 'lat');
fprintf('Dutch roll   : zeta %.3f vs %.3f,  wn %.3f vs %.3f rad/s\n', dl.dr_zeta, dn.dr_zeta, dl.dr_wn, dn.dr_wn);
fprintf('Roll mode    : T %.3f s vs %.3f s\n', dl.roll_T, dn.roll_T);
fprintf('Spiral pole  : %.5f vs %.5f 1/s\n', dl.spiral_pole, dn.spiral_pole);

% control effectiveness (elevator -> q_dot, aileron -> p_dot, rudder -> r_dot)
fprintf('\nControl derivatives (linear vs nonlinear):\n');
fprintf('  M_eta   %.3f vs %.3f\n', M.B_ol(4), B(5, 1));
fprintf('  L_xi    %.3f vs %.3f\n', M.B_ol_lat(3, 1), B(4, 2));
fprintf('  N_zeta  %.3f vs %.3f\n', M.B_ol_lat(1, 2), B(6, 3));

function s = fmt(e)
    % Formats eigenvalues as fixed-width text (real or complex).
    s = '';
    for k = 1:numel(e)
        if abs(imag(e(k))) < 1e-9
            s = [s sprintf('%9.4f        ', real(e(k)))]; %#ok<AGROW>
        else
            s = [s sprintf('%8.4f%+8.4fi  ', real(e(k)), imag(e(k)))]; %#ok<AGROW>
        end
    end
end

function e = sort_modes(e)
    % Sorts by magnitude (fast to slow) and keeps one of each conjugate pair.
    [~, i] = sort(abs(e), 'descend');
    e = e(i);
    e = e(imag(e) >= -1e-9);
end
