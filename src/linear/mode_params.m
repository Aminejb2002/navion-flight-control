function s = mode_params(A, kind)
% Modal data of the state matrix A for flying-qualities checks.
%   kind = 'lon': short period (highest-frequency pair) and phugoid
%   kind = 'lat': Dutch roll (state order [r beta p Phi]), roll time constant, spiral pole

[V, D] = eig(A);
lam = diag(D);
cidx = find(imag(lam) > 1e-9);

if strcmp(kind, 'lon')
    if isempty(cidx)
        s.sp_wn = NaN;  s.sp_zeta = 1;  s.ph_zeta = NaN;
        return
    end
    [~, order] = sort(abs(lam(cidx)), 'descend');
    cidx = cidx(order);
    s.sp_wn = abs(lam(cidx(1)));
    s.sp_zeta = -real(lam(cidx(1)))/s.sp_wn;
    if numel(cidx) >= 2
        s.ph_zeta = -real(lam(cidx(end)))/abs(lam(cidx(end)));
    else
        s.ph_zeta = NaN;
    end
else
    if isempty(cidx)
        s.dr_wn = NaN;  s.dr_zeta = NaN;  s.dr_phibeta = NaN;
        r = sort(real(lam));
        s.roll_T = -1/r(1);  s.spiral_pole = r(end);
        return
    end
    [~, jdr] = max(abs(lam(cidx)));
    idr = cidx(jdr);
    s.dr_wn = abs(lam(idr));
    s.dr_zeta = -real(lam(idr))/s.dr_wn;
    s.dr_phibeta = abs(V(4, idr)/V(2, idr));
    r = sort(real(lam(abs(imag(lam)) < 1e-9)));
    % NaN when roll and spiral have merged into an oscillation
    if numel(r) >= 1, s.roll_T = -1/r(1); else, s.roll_T = NaN; end
    if numel(r) >= 2, s.spiral_pole = r(end); else, s.spiral_pole = NaN; end
end
end
