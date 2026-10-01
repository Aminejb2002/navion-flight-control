function [x0, u0, info] = trim_navion(P, V, h)
% Level-flight trim of the nonlinear Navion model at airspeed V [m/s], altitude h [m].
% Unknowns: angle of attack, elevator, thrust (Newton iteration, central-difference
% Jacobian). Conditions: udot = wdot = qdot = 0, gamma = 0, no sideslip, wings level.
% Returns state x0, input u0 = [eta; 0; 0; T] and info (alpha, eta, T, residual).

z = [P.alpha0; 0; 200];                 % initial guess [alpha; eta; T]
for it = 1:50
    r0 = trim_res(z, P, V);
    J = zeros(3);
    for j = 1:3
        dz = zeros(3, 1);  dz(j) = 1e-6 * max(1, abs(z(j)));
        J(:, j) = (trim_res(z + dz, P, V) - trim_res(z - dz, P, V)) / (2*dz(j));
    end
    step = -J \ r0;
    z = z + step;
    if norm(step) < 1e-12, break; end
end
info.iterations = it;
info.residual = norm(trim_res(z, P, V));
info.alpha = z(1);  info.eta = z(2);  info.T = z(3);

[x0, u0] = trim_state(z, V, h);
end

function r = trim_res(z, P, V)
[x, u] = trim_state(z, V, 0);
xd = navion_nl(x, u, zeros(3, 1), P);
r = [xd(1); xd(3); xd(5)];
end

function [x, u] = trim_state(z, V, h)
alpha = z(1);
x = [V*cos(alpha); 0; V*sin(alpha); 0; 0; 0; 0; alpha; 0; 0; 0; h];
u = [z(2); 0; 0; z(3)];
end
