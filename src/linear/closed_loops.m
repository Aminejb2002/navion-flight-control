function L = closed_loops(M, G, V0)
% Closed-loop matrices and controller descriptions for the gain set G (models M, speed V0).

%% Longitudinal: pitch damper + attitude hold, states [V; gamma; alpha; q; theta]
L.A_lon_hold = M.A_aug - M.B_aug*[0 0 0 G.Kq G.Kp];

%% Altitude + speed hold, plant states [V; gamma; alpha; q; h],
% controller states [int(h_cmd-h); int(dV)]
A_h = [M.A_ol, zeros(4,1); 0 V0 0 0 0];
B_h = [M.B_ol; 0];
Bt  = M.B_thrust;
th_row = [0 1 1 0 0];
eh_row = [0 0 0 0 1];
K_in   = [0 0 0 G.Kq 0] + G.Kp*th_row;

L.A_full = zeros(7);
L.A_full(1:5,1:5) = A_h - B_h*K_in - B_h*G.Kp*G.Kh*eh_row;
L.A_full(1:5,6)   = B_h*G.Kp*G.Ki;
L.A_full(6,5)     = -1;
L.A_full(1:5,1)   = L.A_full(1:5,1) - Bt*G.Kv;
L.A_full(1:5,7)   = -Bt*G.Kvi;
L.A_full(7,1)     = 1;

L.lon.A_plant = A_h;
L.lon.B_plant = [B_h, Bt];
Bc = [0 0 0 0 -1; 1 0 0 0 0];
Cc = [G.Kp*G.Ki 0; 0 -G.Kvi];
Dc = [-(K_in + G.Kp*G.Kh*eh_row); -G.Kv 0 0 0 0];
L.lon.Ct = ss(zeros(2), Bc, -Cc, -Dc);

%% Lateral: dampers, states [r; beta; p; Phi]
L.A_lat_dampers = M.A_ol_lat ...
    - M.B_ol_lat(:,2)*[G.Kzeta 0 0 0] - M.B_ol_lat(:,1)*[0 0 G.Kxi 0];

%% Bank / heading / cross-track hold, states [r; beta; p; Phi; psi; y]
A_trk = [M.A_ol_lat, zeros(4,2);
         1 0 0 0 0 0;
         0 V0 0 0 V0 0];
B_trk = [M.B_ol_lat; 0 0; 0 0];
K_lat = [0 0 G.Kxi G.Kphi G.Kphi*G.Kpsi G.Kphi*G.Kpsi*G.Ky;
         G.Kzeta 0 0 0 0 0];
L.A_trk_hold = A_trk - B_trk*K_lat;
L.B_trk_gust = [M.B_gust_lat; 0; -V0];

L.lat.A_plant = A_trk;
L.lat.B_plant = B_trk;
L.lat.Ct = ss(K_lat);
end
