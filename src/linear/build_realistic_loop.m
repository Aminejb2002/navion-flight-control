function [Acl, Bgcol, idx] = build_realistic_loop(A, B, Bg, Ac, Bc, Cc, Dc, act, lagidx, tau_s)
% Closed loop of plant, controller, actuators and sensor lags.
%   plant       x_dot  = A x + B u_p + Bg g
%   controller  xc_dot = Ac xc + Bc z,   u_c = -(Cc xc + Dc z)
%   actuators   u_c -> u_p, one per input; act(j) has fields type ('second' or
%               'first') and wn, zeta (second order) or tau (first order); [] = ideal
%   sensors     first-order lag tau_s on the plant states in lagidx; other states unlagged
% Returns the state matrix of [x; xc; xa; xs], the gust input column, and the
% index ranges idx.x, idx.c, idx.a, idx.s.

n  = size(A, 1);
m  = size(B, 2);
nc = size(Ac, 1);

%% Actuator matrices
if isempty(act)
    na = 0;  Aa = zeros(0);  Ba = zeros(0, m);  Ca = zeros(m, 0);
else
    na = 0;
    for j = 1:m
        if strcmp(act(j).type, 'second'), na = na + 2; else, na = na + 1; end
    end
    Aa = zeros(na);  Ba = zeros(na, m);  Ca = zeros(m, na);
    p = 0;
    for j = 1:m
        if strcmp(act(j).type, 'second')
            w = act(j).wn;  z = act(j).zeta;
            Aa(p+1:p+2, p+1:p+2) = [0 1; -w^2 -2*z*w];
            Ba(p+2, j) = w^2;
            Ca(j, p+1) = 1;
            p = p + 2;
        else
            Aa(p+1, p+1) = -1/act(j).tau;
            Ba(p+1, j)   = 1/act(j).tau;
            Ca(j, p+1)   = 1;
            p = p + 1;
        end
    end
end

%% Sensor matrices
if isempty(lagidx) || tau_s <= 0
    lagidx = [];
end
ns = numel(lagidx);
Mx = eye(n);
E  = zeros(ns, n);
for k = 1:ns
    Mx(lagidx(k), lagidx(k)) = 0;
    E(k, lagidx(k)) = 1;
end

%% Assembly
N = n + nc + na + ns;
ix = 1:n;
ic = n + (1:nc);
ia = n + nc + (1:na);
is = n + nc + na + (1:ns);

Acl = zeros(N);
Acl(ix, ix) = A;

if nc > 0
    Acl(ic, ix) = Bc * Mx;
    Acl(ic, ic) = Ac;
    if ns > 0, Acl(ic, is) = Bc * E'; end
end

if na > 0
    % plant driven by actuator outputs
    Acl(ix, ia) = B * Ca;
    % actuators driven by controller output
    Acl(ia, ia) = Aa;
    if nc > 0, Acl(ia, ic) = -Ba * Cc; end
    Acl(ia, ix) = -Ba * Dc * Mx;
    if ns > 0, Acl(ia, is) = -Ba * Dc * E'; end
else
    % ideal actuators: plant driven by controller output directly
    if nc > 0, Acl(ix, ic) = -B * Cc; end
    Acl(ix, ix) = A - B * Dc * Mx;
    if ns > 0, Acl(ix, is) = -B * Dc * E'; end
end

if ns > 0
    Acl(is, ix) = E / tau_s;
    Acl(is, is) = -eye(ns) / tau_s;
end

Bgcol = [Bg(:); zeros(N - n, 1)];
idx = struct('x', ix, 'c', ic, 'a', ia, 's', is);
end
