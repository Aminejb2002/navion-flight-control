function xdot = navion_nl_fcn(x, u, g)
% Nonlinear Navion model for a Simulink MATLAB Function block.
%   x : 12 states [u v w p q r phi theta psi N E h]
%   u : deviations from trim [elevator; aileron; rudder; thrust] (rad, rad, rad, N)
%   g : gust angles [beta_g; alpha_g] (rad), same convention as the linear models
%#codegen
P = nl_const();
utot = [P.eta0 + u(1); u(2); u(3); P.T0 + u(4)];
wind = [0; -P.V0*g(1); -P.V0*g(2)];          % gust velocity in body axes [m/s]
xdot = navion_nl(x, utot, wind, P);
end
