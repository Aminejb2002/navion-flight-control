function [Act, Sens] = actuator_ss(act, n, lagidx, tau_s)
% LTI actuator models (struct array act, see build_realistic_loop) and first-order
% sensor lags (time constant tau_s) on the n states listed in lagidx.
% Act: commanded -> actual surface/thrust.  Sens: true -> sensed state (n x n).

Act = [];
for j = 1:numel(act)
    if strcmp(act(j).type, 'second')
        g = ss(tf(act(j).wn^2, [1, 2*act(j).zeta*act(j).wn, act(j).wn^2]));
    else
        g = ss(tf(1, [act(j).tau, 1]));
    end
    if isempty(Act), Act = g; else, Act = append(Act, g); end
end

ns = numel(lagidx);
if ns == 0 || tau_s <= 0
    Sens = ss(eye(n));
    return
end
Mx = eye(n);
E  = zeros(ns, n);
for k = 1:ns
    Mx(lagidx(k), lagidx(k)) = 0;
    E(k, lagidx(k)) = 1;
end
Sens = ss(-eye(ns)/tau_s, E/tau_s, E', Mx);
end
