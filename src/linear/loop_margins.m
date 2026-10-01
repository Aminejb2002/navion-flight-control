function R = loop_margins(A, B, Ct, names, Act, Sens)
% Gain and delay margins at each plant input, one loop at a time with the other
% loops closed. Plant x_dot = A x + B u_plant with actuators Act; controller Ct
% and sensors Sens in the feedback path (u_cmd = -Ct*Sens*x). Delays use a
% 3rd-order Pade approximation (grid up to 0.6 s).

n = size(A, 1);
m = size(B, 2);
Pl = ss(A, B, eye(n), zeros(n, m)) * Act;
K  = Ct * Sens;

k_grid  = logspace(-1.5, 1.5, 300);
Td_grid = 0:0.005:0.6;

R = struct('name', {}, 'k_lo', {}, 'k_hi', {}, 'delay', {});
for ch = 1:m
    st = false(size(k_grid));
    for ik = 1:numel(k_grid)
        d = ones(1, m);
        d(ch) = k_grid(ik);
        st(ik) = is_stable(feedback(Pl*ss(diag(d)), K));
    end
    [~, i1] = min(abs(k_grid - 1));
    lo = i1;
    while lo > 1 && st(lo-1), lo = lo - 1; end
    hi = i1;
    while hi < numel(k_grid) && st(hi+1), hi = hi + 1; end

    delay = Inf;
    for it = 1:numel(Td_grid)
        if Td_grid(it) == 0, continue; end
        T = Td_grid(it);
        num = [-T^3/120, T^2/10, -T/2, 1];
        den = [ T^3/120, T^2/10,  T/2, 1];
        pd = ss(tf(num, den));
        Dl = [];
        for j = 1:m
            if j == ch, blk = pd; else, blk = ss(1); end
            if isempty(Dl), Dl = blk; else, Dl = append(Dl, blk); end
        end
        if ~is_stable(feedback(Pl*Dl, K))
            delay = Td_grid(it);
            break
        end
    end

    R(ch).name  = names{ch};
    R(ch).k_lo  = k_grid(lo);
    R(ch).k_hi  = k_grid(hi);
    R(ch).delay = delay;
end
end

function ok = is_stable(sys)
    ok = max(real(pole(sys))) < -1e-8;
end
