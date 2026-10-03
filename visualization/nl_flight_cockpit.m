function nl_flight_cockpit(varargin)
% NL_FLIGHT_COCKPIT  Chase-camera replay of a controlled nonlinear Navion flight with flight instruments.
%   nl_flight_cockpit
%   nl_flight_cockpit('gust', 3, 'offset', 250, 'video', '')
%
%   Simulates one flight with the full autopilot (Dryden gust, initial cross-track
%   offset) and replays it over schematic scenery next to six instruments: attitude,
%   ground speed, altitude, heading, climb rate, and track/altitude error.
%   Snapshots and the video are written to results/media.
%
%   Options (name, value):
%     gust      gust scale relative to nominal                 2
%     offset    initial cross-track offset East [m]            40
%     psi0      initial heading error [rad]                    0
%     seed      Dryden seed                                    1010
%     duration  simulated flight time [s]                      60
%     speedup   playback speed relative to real time           1
%     fps       video frame rate                               30
%     act       1 = actuators and sensors on                   1
%     video     video file name, '' for none                   'nl_cockpit.mp4'
%     snap      snapshot times [s], [] for none                [0 15 30 55]
%     model     Simulink model                                 'Flight_simulator_nl'
%
%   The speed shown is the inertial ground speed from the position history.

o = struct('gust', 2, 'offset', 40, 'psi0', 0, 'seed', 1010, 'duration', 60, 'speedup', 1, ...
           'fps', 30, 'act', 1, 'video', 'nl_cockpit.mp4', 'snap', [0 15 30 55], ...
           'model', 'Flight_simulator_nl');
for k = 1:2:numel(varargin), o.(varargin{k}) = varargin{k+1}; end

root = fileparts(fileparts(mfilename('fullpath')));
if exist('navion_params', 'file') ~= 2
    run(fullfile(root, 'startup.m'));
end
out_dir = fullfile(root, 'results', 'media');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

D = simulate_flight(o);

% Flight data: nav = [N E alt roll pitch yaw], angles in rad
[tu, iu] = unique(D.t);
nv = D.nav(iu, :);
nv(:,6) = unwrap(nv(:,6));
P = [nv(:,2), nv(:,1), nv(:,3)];                  % [East North Up]
w = min(200, max(3, round(1 / median(diff(tu)))));
vel = zeros(size(P));
for c = 1:3, vel(:,c) = gradient(P(:,c), tu); end
vel = movmean(vel, w, 1);

T  = tu(end);
tf = (0:o.speedup/o.fps:T)';  Nf = numel(tf);
Pn  = interp1(tu, P, tf);
An  = interp1(tu, nv(:,4:6), tf);                 % [phi theta psi]
spd = interp1(tu, sqrt(sum(vel.^2, 2)), tf);      % ground speed [m/s]
vs  = interp1(tu, vel(:,3), tf);                  % climb rate [m/s]
psis = movmean(An(:,3), max(3, round(0.7*o.fps)));

% Figure and sky
bg = [0.055 0.065 0.085];  fg = [0.86 0.89 0.93];  mu = [0.55 0.60 0.68];
f = figure('Color', bg, 'Position', [40 40 1280 720], 'Resize', 'off', 'MenuBar', 'none', ...
           'ToolBar', 'none', 'InvertHardcopy', 'off', 'Name', 'Navion cockpit replay');

SW = 0.63;                                        % width of the 3-D view
bgax = axes('Parent', f, 'Position', [0 0 SW 1]);
image(bgax, sky_image(0.33));
axis(bgax, 'off');  set(bgax, 'Position', [0 0 SW 1]);

% Scene
N0 = Pn(1,2);  N1 = Pn(end,2);  Nr = N0 + 0.70 * (N1 - N0);
alt_max = max(Pn(:,3));
ex = [-25000 25000];  ey = [N0 - 6000, N1 + 20000];
ax = axes('Parent', f, 'Position', [0 0 SW 1], 'Color', 'none', 'XColor', 'none', 'YColor', 'none', ...
          'ZColor', 'none', 'XTick', [], 'YTick', [], 'ZTick', [], 'Box', 'off', ...
          'XLim', ex, 'YLim', ey, 'ZLim', [-5 alt_max + 1500], 'DataAspectRatio', [1 1 1], ...
          'Projection', 'perspective', 'Clipping', 'on', 'SortMethod', 'childorder');
hold(ax, 'on');
light(ax, 'Position', [-1 -1.5 2], 'Style', 'infinite');
light(ax, 'Position', [1 1 0.6], 'Style', 'infinite', 'Color', [0.35 0.4 0.5]);

draw_ground(ax, ex, ey, N0, N1);
draw_airport(ax, Nr, 1800);
draw_roads(ax, ex, ey);
draw_villages(ax, N0, N1);
draw_clouds(ax, N0, N1, alt_max);
plot3(ax, [0 0], [N0 - 5000, N1 + 8000], [1.0 1.0], '--', 'Color', [0.45 0.85 0.55], 'LineWidth', 1.5);
hA = make_aircraft(ax, [0.25 0.62 1.00]);

% Title and readouts
ov = axes('Parent', f, 'Position', [0 0 1 1], 'Visible', 'off', 'XLim', [0 1], 'YLim', [0 1], 'HitTest', 'off');
hold(ov, 'on');
text(ov, 0.015, 0.955, 'Navion  -  nonlinear 6-DoF flight, controlled', 'Color', [1 1 1], 'FontSize', 15, 'FontWeight', 'bold');
text(ov, 0.015, 0.918, sprintf('Dryden gust x%.3g nominal (seed %d)  |  start %.0f m off the track  |  schematic scenery', ...
     o.gust, o.seed, o.offset), 'Color', [1 1 1], 'FontSize', 10);
hTime = text(ov, 0.015, 0.04, '', 'Color', [1 1 1], 'FontSize', 12, 'FontName', 'FixedWidth');

% Instruments, 2 columns x 3 rows, on an opaque panel so scenery does not show between them
axes('Parent', f, 'Position', [SW 0 1-SW 1], 'Color', bg, 'XColor', 'none', 'YColor', 'none', ...
     'XTick', [], 'YTick', [], 'Box', 'off', 'HitTest', 'off');
cw = 0.1825;  ch = 0.31;  x0 = SW + 0.005;
pos = @(c, r) [x0 + c*cw, 0.67 - r*0.335, cw, ch];
S.hor = make_horizon(f, pos(0,0), bg, fg, mu);
S.spd = make_speed(f, pos(1,0), bg, fg, mu);
S.alt = make_altimeter(f, pos(0,1), bg, fg, mu);
S.hdg = make_heading(f, pos(1,1), bg, fg, mu);
S.vsi = make_vsi(f, pos(0,2), bg, fg, mu);
S.dev = make_deviation(f, pos(1,2), bg, fg, mu);

S.ax = ax;  S.hA = hA;  S.hTime = hTime;  S.o = o;
S.Pn = Pn;  S.An = An;  S.spd_v = spd;  S.vs = vs;  S.psis = psis;  S.tf = tf;
S.alt_ref = Pn(1,3);                              % altitude-hold command = initial altitude

% Snapshots and video
for ts = o.snap(:)'
    k = min(Nf, max(1, round(ts / (tf(2) - tf(1))) + 1));
    render_frame(S, k);
    fn = fullfile(out_dir, sprintf('nl_cockpit_snap_t%03d.png', round(ts)));
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
    fprintf('Rendering %d frames to %s (%.0f s of video) ...\n', Nf, o.video, Nf / o.fps);
    for k = 1:Nf
        render_frame(S, k);
        writeVideo(vw, getframe(f));
        if mod(k, 100) == 0, fprintf('  %d/%d\n', k, Nf); end
    end
    close(vw);
    fprintf('Video saved: %s\n', fullfile(out_dir, o.video));
end
end

function render_frame(S, k)
P  = S.Pn(k, :);
ps = S.psis(k);  fw = [sin(ps) cos(ps) 0];
campos(S.ax, P - 42*fw + [0 0 9]);
camtarget(S.ax, P + 110*fw + [0 0 -14]);
camup(S.ax, [0 0 1]);
camva(S.ax, 42);

set(S.hA, 'Matrix', pose(P, S.An(k, :), 1));

phi = rad2deg(S.An(k,1));  th = rad2deg(S.An(k,2));  hd = mod(round(rad2deg(S.An(k,3))), 360);
update_horizon(S.hor, th, phi);
update_speed(S.spd, S.spd_v(k));
update_altimeter(S.alt, P(3));
update_heading(S.hdg, hd);
update_vsi(S.vsi, S.vs(k));
update_deviation(S.dev, P(1), P(3) - S.alt_ref);
set(S.hTime, 'String', sprintf('t = %5.1f s    (%dx real time)', S.tf(k), S.o.speedup));
drawnow;
end

%% Simulation

function D = simulate_flight(o)
mdl = o.model;
if ~bdIsLoaded(mdl), load_system(mdl); end
blk = find_seed_block(mdl);
dp = fieldnames(get_param(blk, 'DialogParameters'));
sf = dp{find(contains(lower(dp), 'seed'), 1)};
old_seed = get_param(blk, sf);
old_stop = get_param(mdl, 'StopTime');
set_param(blk, sf, mat2str([o.seed o.seed+1 o.seed+2 o.seed+3]));
set_param(mdl, 'StopTime', num2str(o.duration));
cleanup = onCleanup(@() restore_state(mdl, blk, sf, old_seed, old_stop)); %#ok<NASGU>

assignin('base', 'act_on', o.act);  assignin('base', 'sens_on', o.act);
fprintf('Simulating %g s ...\n', o.duration);
dx = zeros(12, 1);  dx(11) = o.offset;  dx(9) = o.psi0;    % state offsets: East (11), heading (9)
assignin('base', 'nl_dx0', dx);
set_flags(1, 1, 1, o.gust);
r = sim(mdl);
D.t = r.tout;  D.nav = r.nav;
end

function set_flags(fb, alt, trk, gust)
assignin('base', 'fb_on', fb);  assignin('base', 'alt_on', alt);
assignin('base', 'trk_on', trk);  assignin('base', 'gust_on', gust);
end

function restore_state(mdl, blk, sf, old_seed, old_stop)
try, set_param(blk, sf, old_seed); catch, end
try, set_param(mdl, 'StopTime', old_stop); catch, end
try, evalin('base', 'clear nl_dx0'); catch, end
set_flags(1, 1, 1, 1);
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

%% Scenery

function G = sky_image(hz)
% Sky gradient with two mountain ridges; hz is the horizon position from the top.
nr = 256;  nc = 512;
yy = repmat(linspace(0, 1, nr)', 1, nc);  xx = linspace(0, 1, nc);
zen = [0.16 0.36 0.70];  pale = [0.80 0.88 0.95];  haze = [0.55 0.66 0.55];
far_col = [0.62 0.70 0.80];  near_col = [0.48 0.58 0.68];
h1 = 0.050 + 0.030*sin(2*pi*3*xx + 1.3) + 0.020*sin(2*pi*7*xx + 0.4) + 0.010*sin(2*pi*17*xx + 2.1);
h2 = 0.028 + 0.016*sin(2*pi*5*xx + 0.7) + 0.010*sin(2*pi*13*xx + 1.9) + 0.005*sin(2*pi*31*xx);
far  = yy <= hz + 0.03 & yy >= hz - repmat(h1, nr, 1);
near = yy <= hz + 0.03 & yy >= hz - repmat(h2, nr, 1);
G = zeros(nr, nc, 3);
for ch = 1:3
    up = zen(ch) + (pale(ch) - zen(ch)) * min(1, yy / hz) .^ 0.8;
    dn = pale(ch) + (haze(ch) - pale(ch)) * min(1, max(0, (yy - hz) / (1 - hz)));
    layer = (yy <= hz) .* up + (yy > hz) .* dn;
    layer(far)  = far_col(ch);
    layer(near) = near_col(ch);
    G(:,:,ch) = layer;
end
end

function draw_ground(ax, ex, ey, N0, N1)
tile = 250;
[X, Y] = meshgrid(ex(1):tile:ex(2), ey(1):tile:ey(2));
rng(3);
base = [0.30 0.47 0.22];  haze = [0.64 0.74 0.70];
dist = hypot(X / 9000, (Y - 0.5*(N0 + N1)) / 20000);
hz = min(0.75, dist .^ 1.5);
r1 = 0.07 * (rand(size(X)) - 0.5);  r2 = 0.05 * (rand(size(X)) - 0.5);
Cd = zeros(size(X, 1), size(X, 2), 3);
for ch = 1:3
    b = base(ch) + r1 * (1 - 0.4*(ch == 3)) + r2 * (ch == 1);
    Cd(:,:,ch) = (1 - hz) .* b + hz .* haze(ch);
end
surf(ax, X, Y, zeros(size(X)), Cd, 'FaceColor', 'flat', 'EdgeColor', 'none', 'FaceLighting', 'none');
end

function draw_airport(ax, Nr, xo)
% Airfield beside the track with the runway crossing east-west, centred xo metres east of
% the track at northing Nr, so it reads as scenery and not as an approach.
ax = hgtransform('Parent', ax);
M = [0 -1 0 Nr + xo;  1 0 0 Nr;  0 0 1 0;  0 0 0 1];
set(ax, 'Matrix', M);
rl = 2400;  rw = 60;
gry = [0.42 0.43 0.45];  wht = [0.95 0.95 0.95];
quad = @(x, y, z, col) patch(x, y, z * ones(1, 4), col, 'EdgeColor', 'none', 'Parent', ax);
quad([45 65 65 45], [Nr-rl/2+20 Nr-rl/2+20 Nr+rl/2-20 Nr+rl/2-20], 1.2, gry);
quad([65 140 140 65], [Nr-rl/2+40 Nr-rl/2+40 Nr-rl/2+320 Nr-rl/2+320], 1.2, gry);
for yc = [Nr-rl/2+40, Nr+rl/2-60]
    quad([rw/2 45 45 rw/2], [yc yc yc+20 yc+20], 1.2, gry);
end
quad([-rw/2 rw/2 rw/2 -rw/2], [Nr-rl/2 Nr-rl/2 Nr+rl/2 Nr+rl/2], 1.5, [0.28 0.29 0.31]);
for yc = (Nr-rl/2+80):60:(Nr+rl/2-80)
    quad([-1 1 1 -1], [yc yc yc+30 yc+30], 1.8, wht);
end
for yt = [Nr-rl/2+10, Nr+rl/2-50]
    for xt = -18:6:18
        quad([xt xt+2.5 xt+2.5 xt], [yt yt yt+30 yt+30], 1.8, wht);
    end
end
box3(ax, 170, Nr-rl/2+330, 60, 140, 14, [0.70 0.72 0.76]);
box3(ax, 170, Nr-rl/2+520, 60, 140, 14, [0.70 0.72 0.76]);
box3(ax, 170, Nr-rl/2+710, 60, 140, 14, [0.62 0.66 0.72]);
box3(ax, 110, Nr-rl/2+120, 50, 50, 10, [0.80 0.78 0.72]);
box3(ax, 110, Nr-rl/2+60, 12, 12, 32, [0.85 0.85 0.88]);
box3(ax, 110, Nr-rl/2+60, 18, 18, 4, [0.20 0.45 0.65], 32);
end

function draw_roads(ax, ex, ey)
rng(4);
ev = -16000:2300:16000;  ev = ev + 300*randn(size(ev));
nh = ey(1):2000:ey(2);   nh = nh + 250*randn(size(nh));
X = [ev; ev; nan(size(ev))];
Y = [ey(1)*ones(size(ev)); ey(2)*ones(size(ev)); nan(size(ev))];
Xh = [ex(1)*ones(size(nh)); ex(2)*ones(size(nh)); nan(size(nh))];
Yh = [nh; nh; nan(size(nh))];
X = [X(:); Xh(:)];  Y = [Y(:); Yh(:)];
plot3(ax, X, Y, 0.6*ones(size(X)), '-', 'Color', [0.60 0.57 0.50], 'LineWidth', 1.0);
end

function draw_villages(ax, N0, N1)
rng(6);
nvil = 14;  nb = 18;
V = zeros(8*nvil*nb, 3);  F = zeros(6*nvil*nb, 4);  C = zeros(6*nvil*nb, 3);
fc = [0 1 2 3; 4 5 6 7; 0 1 5 4; 1 2 6 5; 2 3 7 6; 3 0 4 7];
cols = [0.80 0.76 0.68; 0.72 0.35 0.28; 0.85 0.83 0.80];
n = 0;
for v = 1:nvil
    cx = 12000 * (rand - 0.5);  cy = N0 + (N1 - N0 + 4000) * rand;
    if abs(cx) < 500, cx = cx + 1200 * sign(cx + 0.1); end     % keep the track clear
    for b = 1:nb
        bx = cx + 160 * randn;  by = cy + 160 * randn;
        w = 14 + 14*rand;  l = 14 + 18*rand;  h = 6 + 8*rand;
        x = bx + w/2 * [-1 1 1 -1];  y = by + l/2 * [-1 -1 1 1];
        V(8*n+1:8*n+8, :) = [x' y' zeros(4,1); x' y' h*ones(4,1)];
        F(6*n+1:6*n+6, :) = fc + 8*n + 1;
        C(6*n+1:6*n+6, :) = repmat(cols(randi(3), :), 6, 1);
        n = n + 1;
    end
end
patch('Parent', ax, 'Vertices', V, 'Faces', F, 'FaceVertexCData', C, 'FaceColor', 'flat', ...
      'EdgeColor', 'none', 'FaceLighting', 'flat');
end

function draw_clouds(ax, N0, N1, alt)
rng(5);
th = linspace(0, 2*pi, 16);
for i = 1:55
    cx = 14000 * (rand - 0.5);  cy = N0 - 1500 + (N1 - N0 + 9000) * rand;  cz = alt + 600 + 700 * rand;
    for j = 1:3
        a = 250 + 450*rand;  b = a * (0.45 + 0.2*rand);
        rr = 1 + 0.18 * randn(size(th));
        x = cx + 0.7*a*(j-2) + a * rr .* cos(th);
        y = cy + 0.4*b*(j-2) + b * rr .* sin(th);
        patch(x, y, cz * ones(size(th)), [1 1 1], 'FaceAlpha', 0.55, 'EdgeColor', 'none', ...
              'FaceLighting', 'none', 'Parent', ax);
    end
end
end

function box3(ax, cx, cy, wx, wy, h, col, z0)
if nargin < 8, z0 = 0; end
x = cx + wx/2 * [-1 1 1 -1];  y = cy + wy/2 * [-1 -1 1 1];
V = [x' y' z0*ones(4,1); x' y' (z0+h)*ones(4,1)];
F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch('Parent', ax, 'Vertices', V, 'Faces', F, 'FaceColor', col, 'EdgeColor', 'none', 'FaceLighting', 'flat');
end

function M = pose(P, ang, s)
% Body (x forward, y right, z down) to plot axes [East North Up], scaled by s, at position P.
phi = ang(1);  th = ang(2);  ps = ang(3);
sp = sin(phi); cp = cos(phi); st = sin(th); ct = cos(th); ss = sin(ps); cs = cos(ps);
Rb2n = [ct*cs, sp*st*cs - cp*ss, cp*st*cs + sp*ss;
        ct*ss, sp*st*ss + cp*cs, cp*st*ss - sp*cs;
        -st,   sp*ct,            cp*ct];
Pm = [0 1 0; 1 0 0; 0 0 -1];
M = eye(4);
M(1:3, 1:3) = s * Pm * Rb2n;
M(1:3, 4) = P(:);
end

function hg = make_aircraft(ax, c)
% Low-wing single-engine aircraft, Navion proportions (span 10.2 m, length 8.4 m).
hg = hgtransform('Parent', ax);
cw = min(1, c * 0.82);  cc = [0.80 0.92 1.0];
opt = {'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'BackFaceLighting', 'lit', ...
       'AmbientStrength', 0.45, 'DiffuseStrength', 0.8, 'SpecularStrength', 0.25};
xs = linspace(-5.6, 2.8, 48)';
rx = interp1([-5.6 -4.8 -3.0 -1.5 0.0 1.2 2.2 2.8], [0.07 0.16 0.33 0.55 0.62 0.60 0.46 0.10], xs, 'pchip');
th = linspace(0, 2*pi, 28);
X = repmat(xs, 1, numel(th));  Y = rx * cos(th);  Z = 1.05 * rx * sin(th) - 0.05;
surf(X, Y, Z, 'Parent', hg, 'FaceColor', c, opt{:});
[xe, ye, ze] = ellipsoid(-0.4, 0, -0.55, 1.2, 0.42, 0.38, 16);
surf(xe, ye, ze, 'Parent', hg, 'FaceColor', cc, opt{:}, 'FaceAlpha', 0.75);
y = linspace(-5.09, 5.09, 25);  a = abs(y) / 5.09;
LE = 0.9 - 0.35*a;  TE = -1.0 + 0.45*a;  zc = 0.55 - 0.60*a;  tt = 0.16 * (1 - 0.6*a);
surf([LE; TE], [y; y], [zc - tt/2; zc - tt/2], 'Parent', hg, 'FaceColor', cw, opt{:});
surf([LE; TE], [y; y], [zc + tt/2; zc + tt/2], 'Parent', hg, 'FaceColor', cw, opt{:});
y2 = linspace(-1.85, 1.85, 13);  a2 = abs(y2) / 1.85;
surf([-4.55 - 0.35*a2; -5.75 + 0.15*a2], [y2; y2], -0.05 * ones(2, 13), 'Parent', hg, 'FaceColor', cw, opt{:});
zz = linspace(-0.25, -2.1, 10);  fr = (-0.25 - zz) / 1.85;
surf([-4.35 - 0.75*fr; -5.75 - 0.05*fr], zeros(2, 10), [zz; zz], 'Parent', hg, 'FaceColor', c, opt{:});
tp = linspace(0, 2*pi, 40);
patch('Parent', hg, 'XData', 2.85*ones(size(tp)), 'YData', cos(tp), 'ZData', sin(tp), ...
      'FaceColor', [0.85 0.9 0.95], 'FaceAlpha', 0.14, 'EdgeColor', 'none');
end

%% Instruments

function a = mkgauge(f, pos, ttl, bg, fg)
a = axes('Parent', f, 'Position', pos, 'Color', bg, 'XColor', 'none', 'YColor', 'none', 'XTick', [], 'YTick', []);
hold(a, 'on');  axis(a, 'equal');  xlim(a, [-1.3 1.3]);  ylim(a, [-1.3 1.3]);
text(a, 0, 1.24, ttl, 'Color', fg, 'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
end

function xy = polar(a_deg, r)
% Points at angle a_deg (degrees, clockwise from up) and radius r.
xy = [r * sin(deg2rad(a_deg(:))), r * cos(deg2rad(a_deg(:)))];
end

function disc(a, r, col, ec)
th = linspace(0, 2*pi, 120);
patch(a, r*cos(th), r*sin(th), col, 'EdgeColor', ec, 'LineWidth', 1.5);
end

function h = needle(a, col)
h = patch(a, nan(4,1), nan(4,1), col, 'EdgeColor', 'none');
end

function set_needle(h, ang_deg, len, wid, tail)
u = polar(ang_deg, 1);  n = [-u(2) u(1)];
tip = len*u;  b1 = -tail*u + wid*n;  b2 = -tail*u - wid*n;
set(h, 'XData', [tip(1); b1(1); b2(1); tip(1)], 'YData', [tip(2); b1(2); b2(2); tip(2)]);
end

function dial_ticks(a, a0, a1, nmaj, nmin, fg, labels, rlab)
% Ticks from angle a0 to a1 with nmaj labelled intervals and nmin subdivisions each.
for i = 0:nmaj*nmin
    ang = a0 + (a1 - a0) * i / (nmaj*nmin);
    if mod(i, nmin) == 0, r0 = 0.82; lw = 1.6; else, r0 = 0.88; lw = 0.8; end
    p0 = polar(ang, r0);  p1 = polar(ang, 0.95);
    plot(a, [p0(1) p1(1)], [p0(2) p1(2)], '-', 'Color', fg, 'LineWidth', lw);
    if mod(i, nmin) == 0 && ~isempty(labels)
        pl = polar(ang, rlab);
        text(a, pl(1), pl(2), labels{i/nmin + 1}, 'Color', fg, 'FontSize', 9, ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    end
end
end

function h = make_horizon(f, pos, bg, fg, mu) %#ok<INUSD>
a = mkgauge(f, pos, 'Attitude', bg, fg);
h.k = 0.028;                                      % display radius per degree of pitch
th = linspace(0, 2*pi, 120)';
h.disc = [cos(th) sin(th)];
h.sky = patch(a, nan, nan, [0.25 0.55 0.90], 'EdgeColor', 'none');
h.gnd = patch(a, nan, nan, [0.52 0.36 0.20], 'EdgeColor', 'none');
h.hl  = plot(a, nan, nan, '-', 'Color', [1 1 1], 'LineWidth', 1.5);
h.lad = plot(a, nan, nan, '-', 'Color', [1 1 1], 'LineWidth', 1.0);
h.lab = gobjects(1, 4);
for i = 1:4
    h.lab(i) = text(a, 0, 0, '', 'Color', [1 1 1], 'FontSize', 7, 'HorizontalAlignment', 'center');
end
plot(a, cos(th), sin(th), '-', 'Color', [0.7 0.74 0.8], 'LineWidth', 2);
for ang = [-60 -45 -30 -20 -10 0 10 20 30 45 60]
    if any(ang == [-60 -30 0 30 60]), r1 = 1.12; else, r1 = 1.07; end
    p0 = polar(ang, 1.0);  p1 = polar(ang, r1);
    plot(a, [p0(1) p1(1)], [p0(2) p1(2)], '-', 'Color', fg, 'LineWidth', 1.2);
end
h.ptr = patch(a, nan(3,1), nan(3,1), [1 0.8 0.2], 'EdgeColor', 'none');
plot(a, [-0.55 -0.2 -0.2], [0 0 -0.08], '-', 'Color', [1 0.8 0.2], 'LineWidth', 2.5);
plot(a, [0.55 0.2 0.2], [0 0 -0.08], '-', 'Color', [1 0.8 0.2], 'LineWidth', 2.5);
plot(a, 0, 0, 's', 'MarkerFaceColor', [1 0.8 0.2], 'MarkerEdgeColor', 'none', 'MarkerSize', 5);
h.txt = text(a, 0, -1.2, '', 'Color', fg, 'FontSize', 8, 'HorizontalAlignment', 'center', 'FontName', 'FixedWidth');
end

function update_horizon(h, pitch, roll)
ph = deg2rad(roll);
u = [-sin(ph), cos(ph)];  wv = [cos(ph), sin(ph)];      % display up and right directions
d = -h.k * pitch;                                        % horizon line: u.p = d
Qs = clip_half(h.disc, u, d, +1);  Qg = clip_half(h.disc, u, d, -1);
if size(Qs, 1) < 3, Qs = zeros(3, 2); end
if size(Qg, 1) < 3, Qg = zeros(3, 2); end
set(h.sky, 'XData', Qs(:,1), 'YData', Qs(:,2));
set(h.gnd, 'XData', Qg(:,1), 'YData', Qg(:,2));
if abs(d) < 1
    s = sqrt(1 - d^2);  p0 = d*u - s*wv;  p1 = d*u + s*wv;
    set(h.hl, 'XData', [p0(1) p1(1)], 'YData', [p0(2) p1(2)]);
else
    set(h.hl, 'XData', nan, 'YData', nan);
end

angs = [-20 -15 -10 -5 5 10 15 20];
X = nan(1, 3*numel(angs));  Y = X;
for j = 1:numel(angs)
    c = h.k * angs(j) + d;
    half = 0.13 * (1 + (mod(abs(angs(j)), 10) == 0));
    p0 = c*u - half*wv;  p1 = c*u + half*wv;
    if norm(p0) < 0.93 && norm(p1) < 0.93
        X(3*j-2:3*j-1) = [p0(1) p1(1)];  Y(3*j-2:3*j-1) = [p0(2) p1(2)];
    end
end
set(h.lad, 'XData', X, 'YData', Y);

labs = [-20 -10 10 20];
for j = 1:4
    p = (h.k * labs(j) + d)*u + 0.36*wv;
    if norm(p) < 0.85
        set(h.lab(j), 'Position', [p(1) p(2) 0], 'String', sprintf('%d', abs(labs(j))), 'Visible', 'on');
    else
        set(h.lab(j), 'Visible', 'off');
    end
end

tip = u * 0.97;  n = wv;
b1 = tip * 0.88 + 0.045*n;  b2 = tip * 0.88 - 0.045*n;
set(h.ptr, 'XData', [tip(1); b1(1); b2(1)], 'YData', [tip(2); b1(2); b2(2)]);
set(h.txt, 'String', sprintf('pitch %+5.1f   bank %+5.1f deg', pitch, roll));
end

function Q = clip_half(Pp, u, d, s)
% Clip the convex polygon Pp to the half plane s*(u.p - d) >= 0.
g = s * (Pp * u(:) - d);  n = size(Pp, 1);
Q = zeros(2*n, 2);  m = 0;
for i = 1:n
    j = mod(i, n) + 1;
    if g(i) >= 0, m = m + 1;  Q(m, :) = Pp(i, :); end
    if (g(i) >= 0) ~= (g(j) >= 0)
        t = g(i) / (g(i) - g(j));
        m = m + 1;  Q(m, :) = Pp(i, :) + t * (Pp(j, :) - Pp(i, :));
    end
end
Q = Q(1:m, :);
end

function h = make_speed(f, pos, bg, fg, mu)
a = mkgauge(f, pos, 'Ground speed [m/s]', bg, fg);
disc(a, 1.0, [0.10 0.11 0.14], mu);
labs = arrayfun(@(v) sprintf('%d', v), 0:10:100, 'UniformOutput', false);
dial_ticks(a, -135, 135, 10, 2, fg, labs, 0.66);
ang = linspace(-135 + 270*0.45, -135 + 270*0.65, 30);
pp = [polar(ang, 0.99); flipud(polar(ang, 0.96))];
patch(a, pp(:,1), pp(:,2), [0.3 0.8 0.4], 'EdgeColor', 'none');
h.n = needle(a, [1 0.8 0.2]);
plot(a, 0, 0, 'o', 'MarkerFaceColor', [0.3 0.3 0.34], 'MarkerEdgeColor', 'none', 'MarkerSize', 8);
h.txt = text(a, 0, -0.45, '', 'Color', fg, 'FontSize', 11, 'HorizontalAlignment', 'center', 'FontName', 'FixedWidth');
end

function update_speed(h, v)
set_needle(h.n, -135 + 270 * min(max(v, 0), 100) / 100, 0.80, 0.035, 0.15);
set(h.txt, 'String', sprintf('%.1f', v));
end

function h = make_altimeter(f, pos, bg, fg, mu)
a = mkgauge(f, pos, 'Altitude [m]', bg, fg);
disc(a, 1.0, [0.10 0.11 0.14], mu);
labs = [arrayfun(@(v) sprintf('%d', v), 0:9, 'UniformOutput', false), {''}];
dial_ticks(a, 0, 360, 10, 5, fg, labs, 0.66);
h.n = needle(a, [1 0.8 0.2]);
h.n2 = needle(a, [0.85 0.88 0.92]);
plot(a, 0, 0, 'o', 'MarkerFaceColor', [0.3 0.3 0.34], 'MarkerEdgeColor', 'none', 'MarkerSize', 8);
h.txt = text(a, 0, -1.2, '', 'Color', fg, 'FontSize', 10, 'HorizontalAlignment', 'center', 'FontName', 'FixedWidth');
end

function update_altimeter(h, alt)
set_needle(h.n, 360 * mod(alt, 1000) / 1000, 0.80, 0.035, 0.12);
set_needle(h.n2, 360 * mod(alt, 10000) / 10000, 0.50, 0.05, 0.08);
set(h.txt, 'String', sprintf('%.0f m', alt));
end

function h = make_heading(f, pos, bg, fg, mu)
a = mkgauge(f, pos, 'Heading [deg]', bg, fg);
disc(a, 1.0, [0.10 0.11 0.14], mu);
h.tick = plot(a, nan, nan, '-', 'Color', fg, 'LineWidth', 1.2);
h.lab = gobjects(1, 12);
for i = 1:12
    h.lab(i) = text(a, 0, 0, '', 'Color', fg, 'FontSize', 9, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
end
h.names = {'N','3','6','E','12','15','S','21','24','W','30','33'};
plot(a, [0 0], [0.7 -0.5], '-', 'Color', [1 0.8 0.2], 'LineWidth', 2.5);
plot(a, [-0.3 0.3], [0.05 0.05], '-', 'Color', [1 0.8 0.2], 'LineWidth', 2.5);
plot(a, [-0.12 0.12], [-0.45 -0.45], '-', 'Color', [1 0.8 0.2], 'LineWidth', 2.5);
patch(a, [-0.06; 0.06; 0], [1.12; 1.12; 1.0], [1 0.8 0.2], 'EdgeColor', 'none');
h.txt = text(a, 0, -1.2, '', 'Color', fg, 'FontSize', 10, 'HorizontalAlignment', 'center', 'FontName', 'FixedWidth');
end

function update_heading(h, hd)
b = deg2rad(0:10:350) - deg2rad(hd);
r0 = 0.90 - 0.06 * (mod(0:10:350, 30) == 0);
X = [r0 .* sin(b); 0.97 * sin(b); nan(1, numel(b))];
Y = [r0 .* cos(b); 0.97 * cos(b); nan(1, numel(b))];
set(h.tick, 'XData', X(:)', 'YData', Y(:)');
p = polar((0:11)*30 - hd, 0.68);
for i = 1:12
    set(h.lab(i), 'Position', [p(i,1) p(i,2) 0], 'String', h.names{i});
end
set(h.txt, 'String', sprintf('heading %03.0f', hd));
end

function h = make_vsi(f, pos, bg, fg, mu)
a = mkgauge(f, pos, 'Climb rate [m/s]', bg, fg);
disc(a, 1.0, [0.10 0.11 0.14], mu);
for v = -5:0.5:5
    ang = -90 + 30*v;
    if mod(v, 1) == 0, r0 = 0.82; lw = 1.6; else, r0 = 0.88; lw = 0.8; end
    p0 = polar(ang, r0);  p1 = polar(ang, 0.95);
    plot(a, [p0(1) p1(1)], [p0(2) p1(2)], '-', 'Color', fg, 'LineWidth', lw);
    if mod(v, 1) == 0 && abs(v) >= 1
        pl = polar(ang, 0.66);
        text(a, pl(1), pl(2), sprintf('%d', abs(v)), 'Color', fg, 'FontSize', 9, ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    end
end
text(a, 0.22, 0.45, 'UP', 'Color', mu, 'FontSize', 8, 'HorizontalAlignment', 'center');
text(a, 0.22, -0.45, 'DN', 'Color', mu, 'FontSize', 8, 'HorizontalAlignment', 'center');
h.n = needle(a, [1 0.8 0.2]);
plot(a, 0, 0, 'o', 'MarkerFaceColor', [0.3 0.3 0.34], 'MarkerEdgeColor', 'none', 'MarkerSize', 8);
h.txt = text(a, 0, -1.2, '', 'Color', fg, 'FontSize', 10, 'HorizontalAlignment', 'center', 'FontName', 'FixedWidth');
end

function update_vsi(h, v)
set_needle(h.n, -90 + 30 * min(max(v, -5), 5), 0.80, 0.035, 0.12);
set(h.txt, 'String', sprintf('%+.1f m/s', v));
end

function h = make_deviation(f, pos, bg, fg, mu)
% Cross-pointer: the vertical bar shows the cross-track error (scale along the bottom),
% the horizontal bar shows the altitude error (scale along the right edge).
a = mkgauge(f, pos, 'Track / altitude error', bg, fg);
grn = [0.45 0.85 0.55];  blu = [0.25 0.62 1.0];
patch(a, [-1; 1; 1; -1], [-1; -1; 1; 1], [0.10 0.11 0.14], 'EdgeColor', mu, 'LineWidth', 1.5);
for e = -300:100:300
    x = 0.85 * e / 300;
    plot(a, [x x], [-1 -0.93], '-', 'Color', fg, 'LineWidth', 1.0);
    if abs(e) < 300, text(a, x, -0.84, sprintf('%d', e), 'Color', mu, 'FontSize', 7, 'HorizontalAlignment', 'center'); end
end
for d = -20:10:20
    y = 0.85 * d / 20;
    plot(a, [0.93 1], [y y], '-', 'Color', fg, 'LineWidth', 1.0);
    text(a, 0.88, y, sprintf('%d', d), 'Color', mu, 'FontSize', 7, 'HorizontalAlignment', 'right');
end
plot(a, [0 0], [-0.9 0.9], '--', 'Color', grn, 'LineWidth', 1.2);
plot(a, [-0.9 0.9], [0 0], '--', 'Color', grn, 'LineWidth', 1.2);
h.vbar = plot(a, [0 0], [-0.8 0.8], '-', 'Color', blu, 'LineWidth', 3);
h.hbar = plot(a, [-0.8 0.8], [0 0], '-', 'Color', blu, 'LineWidth', 3);
h.dot = plot(a, 0, 0, 'o', 'MarkerFaceColor', blu, 'MarkerEdgeColor', 'none', 'MarkerSize', 10);
h.txt = text(a, 0, -1.2, '', 'Color', fg, 'FontSize', 8, 'HorizontalAlignment', 'center', 'FontName', 'FixedWidth');
end

function update_deviation(h, e, de)
x = 0.85 * min(max(e, -300), 300) / 300;
y = 0.85 * min(max(de, -20), 20) / 20;
set(h.vbar, 'XData', [x x]);
set(h.hbar, 'YData', [y y]);
set(h.dot, 'XData', x, 'YData', y);
set(h.txt, 'String', sprintf('track %+6.1f m   alt %+5.1f m', e, de));
end
