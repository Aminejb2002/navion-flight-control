function M = navion_models(P)
% Linear Navion models from the parameter struct P (stability-axis derivatives,
% Nelson). Longitudinal states [V; gamma; alpha; q], inputs [eta]; lateral
% states [r; beta; p; Phi], inputs [xi; zeta].

qbar0 = 0.5*P.rho0*P.V0^2;

Xv   = -(qbar0*P.S)/(P.m*P.V0)   * (P.M0*P.dCD_dM + 2*P.CD0);
Xa   =  (qbar0*P.S)/P.m          * (P.CL0 - P.CDalpha);
Xq   = -(qbar0*P.S)/P.m          * (P.c/(2*P.V0)) * P.CDq;
Xeta = -(qbar0*P.S)/P.m          * P.CDeta;

Zv   = -(qbar0*P.S)/(P.m*P.V0^2) * (P.M0*P.dCL_dM + 2*P.CL0);
Za   = -(qbar0*P.S)/(P.m*P.V0)   * (P.CLalpha + P.CD0);
Zq   = -(qbar0*P.S)/(P.m*P.V0)   * (P.c/(2*P.V0)) * P.CLq;
Zeta = -(qbar0*P.S)/(P.m*P.V0)   * P.CLeta;

Mv   =  (1/P.Iyy)*(qbar0*P.S*P.c)/P.V0 * P.M0*P.dCm_dM;
Ma   =  (1/P.Iyy)* qbar0*P.S*P.c       * P.Cmalpha;
Mq   =  (1/P.Iyy)* qbar0*P.S*P.c * (P.c/(2*P.V0)) * P.Cmq;
Meta =  (1/P.Iyy)* qbar0*P.S*P.c       * P.Cmeta;

M.A_ol = [ Xv,  -P.g,   Xa-P.g,  Xq;
          -Zv,   0,    -Za,     -Zq;
           Zv,   0,     Za,      Zq+1;
           Mv,   0,     Ma,      Mq ];
M.B_ol = [ Xeta; -Zeta; Zeta; Meta ];

M.A_aug = [M.A_ol, zeros(4,1); 0 0 0 1 0];
M.B_aug = [M.B_ol; 0];
M.B_gust_lon = [Xa; -Za; Za; Ma; 0];

XdT = cos(P.alpha0)/P.m;
ZdT = -sin(P.alpha0)/(P.m*P.V0);
M.B_thrust = [XdT; -ZdT; ZdT; 0; 0];

Delta = P.Ixx*P.Izz - P.Ixz^2;
k_l = (qbar0*P.S*P.b)/Delta;
k_s = (qbar0*P.S)/(P.m*P.V0);
bv  = P.b/(2*P.V0);

Ybeta = k_s * (P.CYbeta - P.CD0);
Yp    = k_s * bv * P.CYp;
Yr    = k_s * bv * P.CYr;
Yxi   = k_s * P.CYxi;
Yzeta = k_s * P.CYzeta;

Lbeta = k_l * (P.Izz*P.Clbeta + P.Ixz*P.Cnbeta);
Lp    = k_l * bv * (P.Izz*P.Clp + P.Ixz*P.Cnp);
Lr    = k_l * bv * (P.Izz*P.Clr + P.Ixz*P.Cnr);
Lxi   = k_l * (P.Izz*P.Clxi + P.Ixz*P.Cnxi);
Lzeta = k_l * (P.Izz*P.Clzeta + P.Ixz*P.Cnzeta);

Nbeta = k_l * (P.Ixz*P.Clbeta + P.Ixx*P.Cnbeta);
Np    = k_l * bv * (P.Ixz*P.Clp + P.Ixx*P.Cnp);
Nr    = k_l * bv * (P.Ixz*P.Clr + P.Ixx*P.Cnr);
Nxi   = k_l * (P.Ixz*P.Clxi + P.Ixx*P.Cnxi);
Nzeta = k_l * (P.Ixz*P.Clzeta + P.Ixx*P.Cnzeta);

M.A_ol_lat = [ Nr,                  Nbeta,  Np,                     0;
               Yr - cos(P.alpha0),  Ybeta,  Yp + sin(P.alpha0),     (P.g/P.V0)*cos(P.Theta0);
               Lr,                  Lbeta,  Lp,                     0;
               tan(P.Theta0),       0,      1,                      0 ];
M.B_ol_lat = [ Nxi,  Nzeta;
               Yxi,  Yzeta;
               Lxi,  Lzeta;
               0,    0 ];
M.B_gust_lat = M.A_ol_lat(:,2);
end
