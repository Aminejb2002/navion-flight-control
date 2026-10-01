function R = realistic_metrics(Li, Mi, V0, cfg)
% Stability and rms gust response (Dryden) of the longitudinal (altitude + speed
% hold) and lateral (cross-track hold) loops, for the ideal plant and with
% actuators and sensor lags. Li: closed_loops output, Mi: navion_models output.
% cfg fields: act_lon, act_lat, lag_lon, lag_lat, tau_s, sig_lon, sig_lat, L_g

R = struct();
Hg = dryden_filters(V0, cfg.L_g, cfg.sig_lon, 'transverse');
[Ag, Bgf, Cg, ~] = ssdata(Hg);

%% Longitudinal: states [V gamma alpha q h | ctrl 2]
[Ac, Bc, Cc, Dc] = ssdata(Li.lon.Ct);
Bg_lon = [Mi.B_gust_lon(1:4); 0];
[A_i, Bg_i, ~] = build_realistic_loop(Li.lon.A_plant, Li.lon.B_plant, Bg_lon, Ac, Bc, Cc, Dc, [], [], 0);
[A_r, Bg_r, ~] = build_realistic_loop(Li.lon.A_plant, Li.lon.B_plant, Bg_lon, Ac, Bc, Cc, Dc, cfg.act_lon, cfg.lag_lon, cfg.tau_s);
[R.stab_lon_ideal, R.alt_rms_ideal] = gust_rms(A_i, Bg_i, 5, Ag, Bgf, Cg);
[R.stab_lon_real,  R.alt_rms_real]  = gust_rms(A_r, Bg_r, 5, Ag, Bgf, Cg);

%% Lateral: states [r beta p Phi psi y]
Hg = dryden_filters(V0, cfg.L_g, cfg.sig_lat, 'transverse');
[Ag, Bgf, Cg, ~] = ssdata(Hg);
[Ac, Bc, Cc, Dc] = ssdata(Li.lat.Ct);
Bg_lat = Li.B_trk_gust;
[A_i, Bg_i, ~] = build_realistic_loop(Li.lat.A_plant, Li.lat.B_plant, Bg_lat, Ac, Bc, Cc, Dc, [], [], 0);
[A_r, Bg_r, ~] = build_realistic_loop(Li.lat.A_plant, Li.lat.B_plant, Bg_lat, Ac, Bc, Cc, Dc, cfg.act_lat, cfg.lag_lat, cfg.tau_s);
[R.stab_lat_ideal, R.bank_rms_ideal, R.y_rms_ideal] = gust_rms(A_i, Bg_i, [4 6], Ag, Bgf, Cg);
[R.stab_lat_real,  R.bank_rms_real,  R.y_rms_real]  = gust_rms(A_r, Bg_r, [4 6], Ag, Bgf, Cg);
R.bank_rms_ideal = rad2deg(R.bank_rms_ideal);
R.bank_rms_real  = rad2deg(R.bank_rms_real);
end

function [stable, r1, r2] = gust_rms(Acl, Bgcol, states, Ag, Bgf, Cg)
% Stability and rms of the listed states for gust filter (Ag, Bgf, Cg) driven by unit white noise.
r1 = NaN;  r2 = NaN;
stable = all(real(eig(Acl)) < 0);
if ~stable, return; end
N  = size(Acl, 1);
ng = size(Ag, 1);
Aa = [Acl, Bgcol * Cg; zeros(ng, N), Ag];
Ba = [zeros(N, 1); Bgf];
Pc = lyap(Aa, Ba * Ba');
r1 = sqrt(Pc(states(1), states(1)));
if numel(states) > 1
    r2 = sqrt(Pc(states(2), states(2)));
end
end
