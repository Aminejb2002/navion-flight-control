function nl_flight_viz(varargin)
% 3-D animation of the nonlinear Navion model: commanded track, uncontrolled (open loop)
% and controlled (full autopilot) aircraft in the same gust, posed with simulated attitude.
% Usage: nl_flight_viz  |  nl_flight_viz('gust', 3, 'offset', 400)  |  nl_flight_viz('video', '')
% Output goes to <repo>/results/media.
% Options (name, value):
%   gust     gust scale relative to nominal                 (default 2)
%   offset   initial cross-track offset East [m]            (default 250)
%   psi0     initial heading error [rad]                    (default 0)
%   seed     Dryden seed                                    (default 1010)
%   speedup  playback speed vs real time                    (default 10)
%   fps      video frame rate                               (default 30)
%   act      1 = actuators + sensors on                     (default 1)
%   video    output file ('' = none)                        (default 'nl_flight.mp4')
%   snap     snapshot times [s] saved as PNG ([] = none)    (default [0 40 100 220])
%   model    Simulink model                                 (default 'Flight_simulator_nl')
% Aircraft symbols are scaled with the view; positions are true.

o = struct('gust', 2, 'offset', 250, 'psi0', 0, 'seed', 1010, 'speedup', 10, 'fps', 30, ...
           'act', 1, 'video', 'nl_flight.mp4', 'snap', [0 40 100 220], 'model', 'Flight_simulator_nl');
for k = 1:2:numel(varargin), o.(varargin{k}) = varargin{k+1}; end

out_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results', 'media');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

% ------------------------------------------------------------------ simulate
mdl = o.model;
if ~bdIsLoaded(mdl), load_system(mdl); end
blk = find_seed_block(mdl);
dp = fieldnames(get_param(blk, 'DialogParameters'));
sf = dp{find(contains(lower(dp), 'seed'), 1)};
old_seed = get_param(blk, sf);
set_param(blk, sf, mat2str([o.seed o.seed+1 o.seed+2 o.seed+3]));
cleanup = onCleanup(@() restore_state(blk, sf, old_seed));

assignin('base', 'act_on', o.act);  assignin('base', 'sens_on', o.act);
fprintf('Simulating (3 runs)...\n');

% 1) commanded track: gust off, no offset, controller on
evalin('base', 'clear nl_dx0');
set_flags(1, 1, 1, 0);
r = sim(mdl);  R(1).t = r.tout;  R(1).nav = r.nav;

% initial state deviation for runs 2 and 3: [.. psi(9) .. East(11) ..]
dx = zeros(12, 1);  dx(11) = o.offset;  dx(9) = o.psi0;
assignin('base', 'nl_dx0', dx);

% 2) uncontrolled: feedback off, gust on
set_flags(0, 0, 0, o.gust);
r = sim(mdl);  R(2).t = r.tout;  R(2).nav = r.nav;

% 3) controlled: full controller, gust on
set_flags(1, 1, 1, o.gust);
r = sim(mdl);  R(3).t = r.tout;  R(3).nav = r.nav;
evalin('base', 'clear nl_dx0');
set_flags(1, 1, 1, 1);

% ------------------------------------------------------------------ frame data
T  = min([R(1).t(end), R(2).t(end), R(3).t(end)]);
tf = (0:o.speedup/o.fps:T)';   Nf = numel(tf);
alt0 = R(1).nav(1, 3);
Pn = cell(1, 3);  An = cell(1, 3);
for i = 1:3
    nv = interp1(R(i).t, R(i).nav, tf);          % [N E alt roll pitch yaw]
    Pn{i} = [nv(:,2), nv(:,1), nv(:,3) - alt0];  % plot axes [East North Up], Up relative to start
    An{i} = nv(:, 4:6);                          % [phi theta psi]
end
C = (Pn{1} + Pn{2} + Pn{3}) / 3;
D = zeros(Nf, 1);
for i = 1:3, D = max(D, sqrt(sum((Pn{i} - C).^2, 2))); end
Wv = movmean(max(300, 1.7*D), max(3, round(2*o.fps)));
Cs = movmean(C, max(3, round(o.fps)), 1);

% ------------------------------------------------------------------ figure
bg  = [0.055 0.065 0.085];  bg2 = [0.085 0.10 0.13];  fg = [0.86 0.89 0.93];  mu = [0.55 0.60 0.68];
col = {[0.45 0.85 0.55], [1.00 0.50 0.22], [0.25 0.62 1.00]};     % commanded, uncontrolled, controlled
nm  = {'commanded track', 'uncontrolled (open loop)', 'controlled (autopilot)'};
f = figure('Color', bg, 'Position', [40 40 1280 720], 'Resize', 'off', 'MenuBar', 'none', 'ToolBar', 'none', ...
           'InvertHardcopy', 'off', 'Name', 'Navion flight');

ax = axes('Parent', f, 'Position', [0.0 0.0 0.69 1.0], 'Color', bg, 'Visible', 'off');
hold(ax, 'on');  daspect(ax, [1 1 1]);  view(ax, -32, 20);  ax.Projection = 'perspective';
light(ax, 'Position', [-1 -1.5 2], 'Style', 'infinite');
light(ax, 'Position', [1 1 0.6], 'Style', 'infinite', 'Color', [0.35 0.4 0.5]);

hGrid = plot3(ax, nan, nan, nan, '-', 'Color', [0.22 0.27 0.35], 'LineWidth', 0.7);
hA = gobjects(1, 3);  hT = hA;  hS = hA;  hD = hA;  hL = hA;
for i = 1:3
    alpha = 1;  if i == 1, alpha = 0.40; end
    hS(i) = plot3(ax, nan, nan, nan, '-', 'Color', [col{i} 0.35], 'LineWidth', 1.2);
    hT(i) = plot3(ax, nan, nan, nan, '-', 'Color', col{i}, 'LineWidth', 1.8);
    hD(i) = plot3(ax, nan, nan, nan, ':', 'Color', [col{i} 0.6], 'LineWidth', 0.8);
    hA(i) = make_aircraft(ax, col{i}, alpha);
    hL(i) = text(ax, 0, 0, 0, nm{i}, 'Color', col{i}, 'FontSize', 9, 'FontWeight', 'bold', ...
                 'HorizontalAlignment', 'center', 'Clipping', 'off');
end

% overlay: title, legend, readouts
ov = axes('Parent', f, 'Position', [0 0 1 1], 'Visible', 'off', 'XLim', [0 1], 'YLim', [0 1], 'HitTest', 'off');
hold(ov, 'on');
text(ov, 0.02, 0.955, 'Navion  -  nonlinear 6-DoF flight simulation', 'Color', fg, 'FontSize', 15, 'FontWeight', 'bold');
text(ov, 0.02, 0.918, sprintf('same Dryden gust (x%.3g nominal, seed %d)  |  start %.0f m off the track  |  aircraft symbols not to scale', ...
     o.gust, o.seed, o.offset), 'Color', mu, 'FontSize', 10);
for i = 1:3
    plot(ov, 0.027, 0.865 - 0.032*(i-1), 's', 'MarkerFaceColor', col{i}, 'MarkerEdgeColor', 'none', 'MarkerSize', 9);
    text(ov, 0.040, 0.865 - 0.032*(i-1), nm{i}, 'Color', fg, 'FontSize', 10);
end
hTime = text(ov, 0.02, 0.045, '', 'Color', fg, 'FontSize', 12, 'FontName', 'FixedWidth');
hErr  = text(ov, 0.02, 0.015, '', 'Color', mu, 'FontSize', 10, 'FontName', 'FixedWidth');

% right-hand panels
t_pl = tf;
E_u = Pn{2}(:,1);  E_c = Pn{3}(:,1);
H_u = Pn{2}(:,3);  H_c = Pn{3}(:,3);
B_u = rad2deg(An{2}(:,1));  B_c = rad2deg(An{3}(:,1));
pan = struct();
pan.top = mkpanel(f, [0.725 0.575 0.255 0.36], bg2, mu, fg, 'Ground track', 'East [m]', 'North [km]');
plot(pan.top, Pn{1}(:,1), Pn{1}(:,2)/1000, '--', 'Color', col{1}, 'LineWidth', 1.2);
plot(pan.top, E_u, Pn{2}(:,2)/1000, '-', 'Color', col{2}, 'LineWidth', 1.4);
plot(pan.top, E_c, Pn{3}(:,2)/1000, '-', 'Color', col{3}, 'LineWidth', 1.4);
hTop = gobjects(1, 3);
for i = 1:3, hTop(i) = plot(pan.top, nan, nan, 'o', 'MarkerFaceColor', col{i}, 'MarkerEdgeColor', bg, 'MarkerSize', 7); end

rows = {'East of track [m]', E_u, E_c, 0.395; 'Altitude deviation [m]', H_u, H_c, 0.225; 'Bank angle [deg]', B_u, B_c, 0.055};
hCur = gobjects(1, 3);  hDot = gobjects(2, 3);
for p = 1:3
    ap = mkpanel(f, [0.725 rows{p,4} 0.255 0.135], bg2, mu, fg, rows{p,1}, 'time [s]', '');
    plot(ap, [0 T], [0 0], '--', 'Color', col{1}, 'LineWidth', 1.0);
    plot(ap, t_pl, rows{p,2}, '-', 'Color', col{2}, 'LineWidth', 1.3);
    plot(ap, t_pl, rows{p,3}, '-', 'Color', col{3}, 'LineWidth', 1.3);
    xlim(ap, [0 T]);
    yl = [min([rows{p,2}; rows{p,3}; 0]), max([rows{p,2}; rows{p,3}; 0])];
    pad = 0.08 * max(diff(yl), 1);   ylim(ap, yl + [-pad pad]);
    hCur(p) = plot(ap, [0 0], ylim(ap), '-', 'Color', [fg 0.5], 'LineWidth', 0.8);
    hDot(1, p) = plot(ap, 0, 0, 'o', 'MarkerFaceColor', col{2}, 'MarkerEdgeColor', bg, 'MarkerSize', 6);
    hDot(2, p) = plot(ap, 0, 0, 'o', 'MarkerFaceColor', col{3}, 'MarkerEdgeColor', bg, 'MarkerSize', 6);
end
dts = {E_u, E_c; H_u, H_c; B_u, B_c};

% ------------------------------------------------------------------ rendering
    function render(k)
        c = Cs(k, :);  W = Wv(k);
        xlim(ax, [c(1)-W, c(1)+W]);  ylim(ax, [c(2)-1.3*W, c(2)+1.3*W]);
        zf = c(3) - 0.45*W;          zlim(ax, [zf, c(3) + 0.55*W]);
        % floor grid
        g = 100;  if W > 450, g = 200; end
        xs = (floor((c(1)-W)/g):ceil((c(1)+W)/g)) * g;
        ys = (floor((c(2)-1.3*W)/g):ceil((c(2)+1.3*W)/g)) * g;
        gx = [xs; xs; nan(1, numel(xs))];  gy = [repmat(c(2)-1.3*W, 1, numel(xs)); repmat(c(2)+1.3*W, 1, numel(xs)); nan(1, numel(xs))];
        hx = [repmat(c(1)-W, 1, numel(ys)); repmat(c(1)+W, 1, numel(ys)); nan(1, numel(ys))];  hy = [ys; ys; nan(1, numel(ys))];
        X = [gx(:); hx(:)];  Y = [gy(:); hy(:)];
        set(hGrid, 'XData', X, 'YData', Y, 'ZData', zf * ones(size(X)));
        s = 0.085 * W / 5.09;
        for i = 1:3
            P = Pn{i}(k, :);
            set(hA(i), 'Matrix', pose(P, An{i}(k, :), s));
            set(hT(i), 'XData', Pn{i}(1:k,1), 'YData', Pn{i}(1:k,2), 'ZData', Pn{i}(1:k,3));
            set(hS(i), 'XData', Pn{i}(1:k,1), 'YData', Pn{i}(1:k,2), 'ZData', (zf + 0.02) * ones(k, 1));
            set(hD(i), 'XData', [P(1) P(1)], 'YData', [P(2) P(2)], 'ZData', [P(3) zf]);
            set(hL(i), 'Position', [P(1), P(2), P(3) + 0.16*W]);
            set(hTop(i), 'XData', P(1), 'YData', P(2)/1000);
        end
        for p = 1:3
            set(hCur(p), 'XData', [tf(k) tf(k)]);
            set(hDot(1, p), 'XData', tf(k), 'YData', dts{p, 1}(k));
            set(hDot(2, p), 'XData', tf(k), 'YData', dts{p, 2}(k));
        end
        set(hTime, 'String', sprintf('t = %5.1f s    (%dx real time)', tf(k), o.speedup));
        set(hErr, 'String', sprintf('East of track:  open loop %7.1f m    controlled %6.1f m', E_u(k), E_c(k)));
        drawnow;
    end

% ------------------------------------------------------------------ snapshots + video
for ts = o.snap(:)'
    k = min(Nf, max(1, round(ts / (tf(2) - tf(1))) + 1));
    render(k);
    fn = fullfile(out_dir, sprintf('nl_flight_snap_t%03d.png', round(ts)));
    try, exportgraphics(f, fn, 'Resolution', 150, 'BackgroundColor', bg);
    catch, print(f, fn, '-dpng', '-r110'); end
    fprintf('saved %s\n', fn);
end

if ~isempty(o.video)
    try
        vw = VideoWriter(fullfile(out_dir, o.video), 'MPEG-4');
    catch
        o.video = strrep(o.video, '.mp4', '.avi');
        vw = VideoWriter(fullfile(out_dir, o.video), 'Motion JPEG AVI');
    end
    vw.FrameRate = o.fps;
    try, vw.Quality = 95; catch, end
    open(vw);
    fprintf('Rendering %d frames to %s ...\n', Nf, o.video);
    for k = 1:Nf
        render(k);
        writeVideo(vw, getframe(f));
        if mod(k, 100) == 0, fprintf('  %d/%d\n', k, Nf); end
    end
    close(vw);
    fprintf('Video saved: %s\n', fullfile(out_dir, o.video));
end
end

% ====================================================================== helpers
function set_flags(fb, alt, trk, gust)
assignin('base', 'fb_on', fb);  assignin('base', 'alt_on', alt);
assignin('base', 'trk_on', trk);  assignin('base', 'gust_on', gust);
end

function restore_state(blk, sf, old_seed)
try, set_param(blk, sf, old_seed); catch, end
try, evalin('base', 'clear nl_dx0'); catch, end
assignin('base', 'fb_on', 1);  assignin('base', 'alt_on', 1);
assignin('base', 'trk_on', 1);  assignin('base', 'gust_on', 1);
end

function blk = find_seed_block(mdl)
wrapper = [mdl '/Aircraft Model/Gust Disturbance (Dryden)'];
cand = find_system(wrapper, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'SubSystem');
blk = '';
for k = 1:numel(cand)
    try, dp = fieldnames(get_param(cand{k}, 'DialogParameters')); catch, continue, end
    if any(contains(lower(dp), 'seed')), blk = cand{k}; return, end
end
error('No seed block found in %s.', mdl);
end

function a = mkpanel(f, pos, bg2, mu, fg, ttl, xl, yl)
a = axes('Parent', f, 'Position', pos, 'Color', bg2, 'XColor', mu, 'YColor', mu, 'GridColor', mu, ...
         'GridAlpha', 0.18, 'FontSize', 8, 'Box', 'off', 'TickDir', 'out');
hold(a, 'on');  grid(a, 'on');
title(a, ttl, 'Color', fg, 'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'left', 'Units', 'normalized', 'Position', [0 1.02 0]);
xlabel(a, xl, 'Color', mu, 'FontSize', 8);  ylabel(a, yl, 'Color', mu, 'FontSize', 8);
end

function M = pose(P, ang, s)
% Pose matrix: body (x fwd, y right, z down) -> plot [East North Up], scaled by s, at P
phi = ang(1);  th = ang(2);  ps = ang(3);
sp = sin(phi); cp = cos(phi); st = sin(th); ct = cos(th); ss = sin(ps); cs = cos(ps);
Rb2n = [ct*cs, sp*st*cs - cp*ss, cp*st*cs + sp*ss;
        ct*ss, sp*st*ss + cp*cs, cp*st*ss - sp*cs;
        -st,   sp*ct,            cp*ct];
Pm = [0 1 0; 1 0 0; 0 0 -1];            % NED -> [East North Up]
M = eye(4);
M(1:3, 1:3) = s * Pm * Rb2n;
M(1:3, 4) = P(:);
end

function hg = make_aircraft(ax, c, alpha)
% Low-wing single-engine aircraft symbol, Navion proportions (span 10.2 m, length 8.4 m).
hg = hgtransform('Parent', ax);
cw = min(1, c * 0.82);  cc = [0.80 0.92 1.0];
opt = {'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'BackFaceLighting', 'lit', 'FaceAlpha', alpha, ...
       'AmbientStrength', 0.45, 'DiffuseStrength', 0.8, 'SpecularStrength', 0.25};
% fuselage
xs = linspace(-5.6, 2.8, 48)';
rx = interp1([-5.6 -4.8 -3.0 -1.5 0.0 1.2 2.2 2.8], [0.07 0.16 0.33 0.55 0.62 0.60 0.46 0.10], xs, 'pchip');
th = linspace(0, 2*pi, 28);
X = repmat(xs, 1, numel(th));  Y = rx * cos(th);  Z = 1.05 * rx * sin(th) - 0.05;
surf(X, Y, Z, 'Parent', hg, 'FaceColor', c, opt{:});
% canopy
[xe, ye, ze] = ellipsoid(-0.4, 0, -0.55, 1.2, 0.42, 0.38, 16);
surf(xe, ye, ze, 'Parent', hg, 'FaceColor', cc, opt{1:end-8}, 'FaceAlpha', min(alpha, 0.75));
% wing (taper, dihedral, thickness)
y = linspace(-5.09, 5.09, 25);  a = abs(y) / 5.09;
LE = 0.9 - 0.35*a;  TE = -1.0 + 0.45*a;  zc = 0.55 - 0.60*a;  tt = 0.16 * (1 - 0.6*a);
surf([LE; TE], [y; y], [zc - tt/2; zc - tt/2], 'Parent', hg, 'FaceColor', cw, opt{:});
surf([LE; TE], [y; y], [zc + tt/2; zc + tt/2], 'Parent', hg, 'FaceColor', cw, opt{:});
% horizontal tail
y2 = linspace(-1.85, 1.85, 13);  a2 = abs(y2) / 1.85;
surf([-4.55 - 0.35*a2; -5.75 + 0.15*a2], [y2; y2], -0.05 * ones(2, 13), 'Parent', hg, 'FaceColor', cw, opt{:});
% fin
zz = linspace(-0.25, -2.1, 10);  fr = (-0.25 - zz) / 1.85;
surf([-4.35 - 0.75*fr; -5.75 - 0.05*fr], zeros(2, 10), [zz; zz], 'Parent', hg, 'FaceColor', c, opt{:});
% propeller disc
tp = linspace(0, 2*pi, 40);
patch('Parent', hg, 'XData', 2.85*ones(size(tp)), 'YData', cos(tp), 'ZData', sin(tp), 'FaceColor', [0.85 0.9 0.95], ...
      'FaceAlpha', 0.14 * alpha, 'EdgeColor', 'none');
end
