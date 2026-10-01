function H = dryden_filters(V, L, sigma, kind)
% Dryden turbulence shaping filter (MIL-F-8785C) from unit white noise to
% gust speed, scaled so the output rms equals sigma.
%   kind = 'transverse' for v_g and w_g, 'longitudinal' for u_g.
% L: scale length [m], V: airspeed [m/s], sigma: rms gust speed [m/s].

T = L/V;
s = tf('s');
if strcmp(kind, 'transverse')
    H0 = (1 + 2*sqrt(3)*T*s) / (1 + 2*T*s)^2;
else
    H0 = 1 / (1 + T*s);
end
H0 = ss(H0);
[A, B, C] = ssdata(H0);
P = lyap(A, B*B');
scale = sigma / sqrt(C*P*C');
H = scale*H0;
end
