% Navion reference aircraft parameters (Nelson, Flight Stability and Automatic Control).
% Trim condition, mass properties, aerodynamic coefficients and gust definition.

% Trim (sea-level cruise, 176 ft/s)
V0     = 53.64;                 % true airspeed [m/s]
gamma0 = 0;                     % flight-path angle [rad]
alpha0 = 0.05;                  % angle of attack [rad]
Theta0 = gamma0 + alpha0;       % pitch attitude [rad]
rho0   = 1.225;                 % air density [kg/m^3]
M0     = 0.16;                  % Mach number
g      = 9.81;                  % gravity [m/s^2]
m_per_deg = 111320;             % metres per degree (flat-earth conversion)
h_init = 500;                   % initial altitude [m]; also the altitude-hold command

% Geometry and mass properties
S   = 17.09;                    % wing area [m^2]
c   = 1.74;                     % mean aerodynamic chord [m]
b   = 10.18;                    % wingspan [m]
m   = 1247.4;                   % mass [kg]
Ixx = 1420.9;                   % [kg m^2]
Iyy = 4067;                     % [kg m^2]
Izz = 4786.0;                   % [kg m^2]
Ixz = 0;                        % [kg m^2]

% Longitudinal coefficients
CL0 = 0.41;  CLalpha = 4.44;   CLq = 3.80;   CLeta = 0.355;
CD0 = 0.05;  CDalpha = 0.33;   CDq = 0;      CDeta = 0;
Cmalpha = -0.683;  Cmq = -9.96;  Cmeta = -0.923;
dCL_dM = 0;  dCD_dM = 0;  dCm_dM = 0;

% Lateral-directional coefficients
CYbeta = -0.564;  CYp = 0;       CYr = 0;
Clbeta = -0.074;  Clp = -0.410;  Clr = 0.107;
Cnbeta =  0.0701; Cnp = -0.0575; Cnr = -0.125;
CYxi   = 0;       Clxi   = -0.134;  Cnxi   = -0.0035;
CYzeta = 0.157;   Clzeta = 0.0107;  Cnzeta = -0.072;

% Discrete 1-cosine gust
H_gust = 30;                    % gradient distance [m]
Vm     = 6;                     % peak gust speed [m/s]

% Heading-intercept limit of the track-hold loop [rad]
psi_int_lim = 0.7;
