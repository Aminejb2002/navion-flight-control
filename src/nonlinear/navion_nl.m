function [xdot, out] = navion_nl(x, u, wind, P)
% Nonlinear 6-DoF rigid-body model of the Navion (flat, non-rotating Earth).
%
% State x = [u v w p q r phi theta psi N E h]'
%   u v w   : inertial velocity in body axes [m/s]
%   p q r   : body rates [rad/s]
%   phi theta psi : Euler angles [rad]
%   N E h   : North, East position and altitude above the ground [m]
% Input u = [eta xi zeta T]'  elevator, aileron, rudder [rad], thrust [N]
% wind = [uw vw ww]' gust velocity in body axes [m/s]
% P: parameter struct (nl_params). Output out: alpha, beta, Va, CL, CD, Cm.
% Aerodynamics: coefficients linear about trim (Nelson data), evaluated at the
% true angle of attack and sideslip.

eta = u(1);  xi = u(2);  zeta = u(3);  T = u(4);

ub = x(1); vb = x(2); wb = x(3);
p  = x(4); q  = x(5); r  = x(6);
phi = x(7); theta = x(8); psi = x(9);

% air-relative velocity
ua = ub - wind(1);  va = vb - wind(2);  wa = wb - wind(3);
Va = sqrt(ua^2 + va^2 + wa^2);
alpha = atan2(wa, ua);
beta  = asin(max(-1, min(1, va / max(Va, 1e-3))));

qbar = 0.5 * P.rho0 * Va^2;
Mach = P.M0 * Va / P.V0;
pb = p * P.b / (2*Va);  qc = q * P.c / (2*Va);  rb = r * P.b / (2*Va);

da = alpha - P.alpha0;      % coefficients are referenced to trim
CL = P.CL0 + P.CLalpha*da + P.CLq*qc + P.CLeta*eta + P.dCL_dM*(Mach - P.M0);
CD = P.CD0 + P.CDalpha*da + P.CDq*qc + P.CDeta*eta + P.dCD_dM*(Mach - P.M0);
Cm =         P.Cmalpha*da + P.Cmq*qc + P.Cmeta*eta + P.dCm_dM*(Mach - P.M0);
CY = P.CYbeta*beta + P.CYp*pb + P.CYr*rb + P.CYxi*xi + P.CYzeta*zeta;
Cl = P.Clbeta*beta + P.Clp*pb + P.Clr*rb + P.Clxi*xi + P.Clzeta*zeta;
Cn = P.Cnbeta*beta + P.Cnp*pb + P.Cnr*rb + P.Cnxi*xi + P.Cnzeta*zeta;

L = qbar*P.S*CL;  D = qbar*P.S*CD;  Y = qbar*P.S*CY;

% Aerodynamic forces: wind axes (drag, side force, lift) -> body axes
ca = cos(alpha);  sa = sin(alpha);  cb = cos(beta);  sb = sin(beta);
Fx_w = -D;  Fy_w = Y;  Fz_w = -L;
% rotation by beta about z, then alpha about y
Rw2b = [ca*cb, -ca*sb, -sa;
        sb,     cb,     0;
        sa*cb, -sa*sb,  ca];
F_aero = Rw2b * [Fx_w; Fy_w; Fz_w];

Fx = F_aero(1) + T;  Fy = F_aero(2);  Fz = F_aero(3);

Mx = qbar*P.S*P.b*Cl;
My = qbar*P.S*P.c*Cm;
Mz = qbar*P.S*P.b*Cn;

% translational dynamics (body axes, inertial velocity)
st = sin(theta); ct = cos(theta); sp = sin(phi); cp = cos(phi);
udot = r*vb - q*wb - P.g*st      + Fx/P.m;
vdot = p*wb - r*ub + P.g*ct*sp   + Fy/P.m;
wdot = q*ub - p*vb + P.g*ct*cp   + Fz/P.m;

% rotational dynamics
I = [P.Ixx, 0, -P.Ixz; 0, P.Iyy, 0; -P.Ixz, 0, P.Izz];
om = [p; q; r];
omdot = I \ ([Mx; My; Mz] - cross(om, I*om));

% Euler angle kinematics
phidot   = p + (q*sp + r*cp) * st/ct;
thetadot = q*cp - r*sp;
psidot   = (q*sp + r*cp) / ct;

% position (body -> NED rotation; h = -down)
cps = cos(psi); sps = sin(psi);
Rb2n = [ct*cps, sp*st*cps - cp*sps, cp*st*cps + sp*sps;
        ct*sps, sp*st*sps + cp*cps, cp*st*sps - sp*cps;
        -st,    sp*ct,              cp*ct];
vn = Rb2n * [ub; vb; wb];

xdot = [udot; vdot; wdot; omdot; phidot; thetadot; psidot; vn(1); vn(2); -vn(3)];

out.alpha = alpha;  out.beta = beta;  out.Va = Va;
out.CL = CL;  out.CD = CD;  out.Cm = Cm;
end
