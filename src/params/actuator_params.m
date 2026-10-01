% Actuator, engine and sensor parameters for Flight_simulator.slx.
% Run after navion_params. Units: rad, rad/s, s.
% Base values are only set if not already defined, so scripts can override them.
% Nominal values are engineering assumptions (autopilot servo), not Navion data.

%% Switches (1 = realistic, 0 = ideal / bypass)
if ~exist('act_on', 'var'),   act_on   = 1;  end
if ~exist('sens_on', 'var'),  sens_on  = 0;  end

%% Control surface actuators (Nonlinear Second-Order Actuator block)
if ~exist('wn_act', 'var'),   wn_act   = 15;           end  % [rad/s]
if ~exist('zeta_act', 'var'), zeta_act = 0.7;          end  % [-]
if ~exist('pos_lim', 'var'),  pos_lim  = deg2rad(25);  end  % +/- [rad]
if ~exist('rate_lim', 'var'), rate_lim = deg2rad(30);  end  % +/- [rad/s]

% Per-surface values (aileron, rudder, elevator), derived from the base values
wn_ail = wn_act;   zeta_ail = zeta_act;   pos_ail = pos_lim;   rate_ail = rate_lim;
wn_rud = wn_act;   zeta_rud = zeta_act;   pos_rud = pos_lim;   rate_rud = rate_lim;
wn_ele = wn_act;   zeta_ele = zeta_act;   pos_ele = pos_lim;   rate_ele = rate_lim;

%% Engine / thrust (first-order lag)
if ~exist('tau_thr', 'var'),  tau_thr  = 1.0;  end  % [s]

%% Sensors (first-order lag on p, q, r, Phi, Theta, Psi)
if ~exist('tau_sens', 'var'), tau_sens = 0.02; end  % [s]

%% Solver step size
% Second-order actuator block requires wn_act*dt <= ~0.1 (dt <= 0.005 s for wn = 15 rad/s).
