function res = nl_maneuvers(act, mdl)
% Large-amplitude recovery from off-trim initial states, gust off, full controller (mdl default 'Flight_simulator_nl').
%   nl_maneuvers  (actuators + sensors ON)    nl_maneuvers(0)  (ideal actuators)
% Offsets enter through the base variable nl_dx0; prints a result table and plots recovery histories.
if nargin < 1, act = 1; end
if nargin < 2, mdl = 'Flight_simulator_nl'; end
if ~bdIsLoaded(mdl), load_system(mdl); end

d2r = pi/180;
% {case name, offset struct}; fields E [m], phi/theta/psi [rad]
cases = { ...
  'East offset 100 m',   struct('E', 100); ...
  'East offset 300 m',   struct('E', 300); ...
  'East offset 1000 m',  struct('E', 1000); ...
  'Bank 10 deg',         struct('phi', 10*d2r); ...
  'Bank 20 deg',         struct('phi', 20*d2r); ...
  'Bank 30 deg',         struct('phi', 30*d2r); ...
  'Bank 45 deg',         struct('phi', 45*d2r); ...
  'Bank 60 deg',         struct('phi', 60*d2r); ...
  'Pitch +5 deg',        struct('theta', 5*d2r); ...
  'Pitch +10 deg',       struct('theta', 10*d2r); ...
  'Pitch -10 deg',       struct('theta', -10*d2r); ...
  'Heading 20 deg',      struct('psi', 20*d2r); ...
  'Heading 60 deg',      struct('psi', 60*d2r)};

assignin('base', 'act_on', act);  assignin('base', 'sens_on', act);
assignin('base', 'fb_on', 1); assignin('base', 'alt_on', 1); assignin('base', 'trk_on', 1);
assignin('base', 'gust_on', 0);
cleanup = onCleanup(@() evalin('base', 'clear nl_dx0'));

nc = size(cases, 1);
res = struct('name', {}, 'peak_bank', {}, 'alt_dev_min', {}, 'alt_dev_max', {}, 'peak_pitch', {}, ...
             'settle', {}, 'east_end', {}, 'ok', {});
t_all = cell(nc, 1);  y_all = cell(nc, 1);
fprintf('\nLarge-amplitude recovery, gust off, actuators/sensors %s\n', onoff(act));
fprintf('%-20s %10s %12s %12s %10s %11s %10s %7s\n', 'case', 'peak bank', 'alt dev min', 'alt dev max', 'peak pitch', 'settle [s]', 'East end', 'ok');
for k = 1:nc
    % 12-state offset vector: 7 = phi, 8 = theta, 9 = psi, 11 = East
    dx = zeros(12, 1);  c = cases{k, 2};
    if isfield(c, 'phi'),   dx(7)  = c.phi;   end
    if isfield(c, 'theta'), dx(8)  = c.theta; end
    if isfield(c, 'psi'),   dx(9)  = c.psi;   end
    if isfield(c, 'E'),     dx(11) = c.E;     end
    assignin('base', 'nl_dx0', dx);
    try
        r = sim(mdl);
    catch ME
        fprintf('%-20s simulation failed: %s\n', cases{k,1}, ME.message);
        continue
    end
    % nav columns: [North East altitude roll pitch yaw]
    nav = r.nav;  t = r.tout;
    alt = nav(:,3) - nav(1,3);
    bank = rad2deg(nav(:,4));  pit = rad2deg(nav(:,5) - nav(1,5) + dx(8));
    % settle time: after this instant bank < 1 deg, |East| < max(10 m, 2% of offset), |alt dev| < 5 m (NaN if never)
    tol_e = max(10, 0.02 * abs(dx(11)));
    bad = abs(bank) > 1 | abs(nav(:,2)) > tol_e | abs(alt) > 5;
    last_bad = find(bad, 1, 'last');
    if isempty(last_bad), settle = 0; elseif last_bad >= numel(t), settle = NaN; else, settle = t(last_bad + 1); end
    % ok: settles, bank < 90 deg and altitude deviation < 400 m throughout
    ok = isfinite(settle) && max(abs(bank)) < 90 && max(abs(alt)) < 400;
    res(end+1) = struct('name', cases{k,1}, 'peak_bank', max(abs(bank)), 'alt_dev_min', min(alt), ...
        'alt_dev_max', max(alt), 'peak_pitch', max(abs(pit)), 'settle', settle, 'east_end', nav(end,2), 'ok', ok); %#ok<AGROW>
    t_all{k} = t;  y_all{k} = [bank, nav(:,2), alt];
    fprintf('%-20s %8.1f deg %11.1f m %11.1f m %8.1f deg %11.1f %9.1f m %7s\n', cases{k,1}, max(abs(bank)), ...
        min(alt), max(alt), max(abs(pit)), settle, nav(end,2), tf(ok));
end

figure('Name', 'Large-amplitude recovery', 'Color', 'w', 'Position', [60 60 1100 700]);
ylab = {'bank [deg]', 'East [m]', 'altitude dev [m]'};
groups = {1:3, 4:8, 9:11, 12:13};  gname = {'East offset', 'Initial bank', 'Initial pitch', 'Initial heading'};
for g = 1:4
    for p = 1:3
        subplot(3, 4, (p-1)*4 + g);  hold on;  grid on;
        for k = groups{g}
            if ~isempty(t_all{k}), plot(t_all{k}, y_all{k}(:, p), 'LineWidth', 1.2); end
        end
        if p == 1, title(gname{g}); end
        if g == 1, ylabel(ylab{p}); end
        if p == 3, xlabel('t [s]'); end
    end
end
end

function s = onoff(a), if a, s = 'ON'; else, s = 'OFF'; end, end
function s = tf(a), if a, s = 'yes'; else, s = 'NO'; end, end
