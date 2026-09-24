%% ========================================================================
%  vessel_6dof.m
%
%  Six-degree-of-freedom manoeuvring and seakeeping model of a small outboard
%  fishing vessel, with the coefficients estimated from measurable hull data.
%  It runs the same zigzag you use at sea, plus a turning circle, and then
%  extracts the EQUIVALENT NOMOTO PARAMETERS so the 6-DOF model can be checked
%  against your trials and against the 1-DOF model used by the autopilot.
%
%  DOF and conventions (Fossen): body frame at the CG, x forward, y starboard,
%  z down.  nu = [u v w p q r],  eta = [x y z phi theta psi] in NED.
%      M*nu_dot + C(nu)*nu + D(nu)*nu + g(eta) = tau_prop + tau_wave
%      eta_dot = J(eta)*nu
%
%  HOW HONEST IS THIS MODEL?  Read before you cite any number from it.
%  -------------------------------------------------------------------------
%  * Sway/yaw hydrodynamics use Clarke-type regression formulas (Clarke,
%    Gedling & Hine, 1982/83). They were fitted to displacement SHIP hulls.
%    Your 7 m outboard boat is semi-planing at speed, so treat them as order-of-
%    magnitude estimates and check the published coefficients against the
%    original paper before quoting them in the thesis.
%  * Roll, heave and pitch use hydrostatic stiffness (which you can measure)
%    with added inertia and damping tuned to natural periods and damping ratios
%    (which you can also measure: a heel test and a roll-decay test).
%  * Wave excitation is Froude-Krylov plus effective wave slope, not diffraction
%    theory. The yaw moment in waves is the weakest part: it uses an assumed
%    lever arm.
%  * CAL.* holds three calibration factors. The intended workflow is: run a
%    zigzag here, compare the extracted Nomoto K and T with the ones identified
%    from your sea trial, and adjust CAL until they agree. Until that is done,
%    this is a structurally correct model with uncertain numbers.
%
%  OUTPUTS
%    * plots of all six DOF, the track, and the turning circle
%    * zigzag_6dof_log.mat with [t psi delta], ready for vessel_parameters.m
%    * printed manoeuvring metrics (overshoot angles, tactical diameter, heel)
%      and the equivalent Nomoto K, T, alpha
%
%  Units: SI inside the model (rad, m, s); degrees only for display and for
%  the exported log, to match the other scripts.
%% ========================================================================

clear; clc; close all;

%% 0) MEASURED HULL AND MACHINERY DATA  ======================================
% ---- Hull ------------------------------------------------------------------
H.Lpp  = 7.0;      % [m]   length between perpendiculars
H.B    = 2.2;      % [m]   beam at waterline
H.T    = 0.45;     % [m]   draft (hull only, outboard leg excluded)
H.Cb   = 0.45;     % [-]   block coefficient (displacement/(L*B*T))
H.Cwp  = 0.75;     % [-]   waterplane area coefficient (A_w/(L*B))
H.rho  = 1025;     % [kg/m^3] sea water density
H.g    = 9.81;     % [m/s^2]

% ---- Weights and stability (inclining test, heel test, roll decay test) ----
H.mass   = H.rho*H.Lpp*H.B*H.T*H.Cb;   % [kg] displacement; replace with weighed mass
H.GMt    = 0.80;   % [m]   transverse metacentric height (heel test)
H.GMl    = 8.0;    % [m]   longitudinal metacentric height (~ Lpp for small craft)
H.rGyr44 = 0.38*H.B;    % [m] roll radius of gyration  (~0.35-0.40 B)
H.rGyr55 = 0.25*H.Lpp;  % [m] pitch radius of gyration (~0.25 Lpp)
H.rGyr66 = 0.25*H.Lpp;  % [m] yaw radius of gyration
H.zHull  = 0.5*H.T;     % [m] depth below the CG where the hull side force acts
H.Trol   = 3.0;    % [s] measured roll natural period (roll decay test)
H.zetaRol= 0.12;   % [-] roll damping ratio from the same decay test
H.zetaHv = 0.35;   % [-] heave damping ratio (assumed)
H.zetaPt = 0.35;   % [-] pitch damping ratio (assumed)
H.Cdcf   = 0.8;    % [-] cross-flow drag coefficient of the hull sections
                   %     (this is what creates the non-linear yaw damping, i.e.
                   %      the alpha term of the Nomoto model)
% Skeg / keel. A bare hull with the thrust at the transom is often DIRECTIONALLY
% UNSTABLE: an identification then returns a negative K and T, which is the model
% telling you the boat diverges instead of settling into a steady turn. Real
% fishing boats carry a keel or skeg, so measure yours.
H.skegArea = 0.15;  % [m^2]   lateral area of the skeg/keel (0 disables it)
H.skegX    = -3.0;  % [m]     longitudinal position relative to the CG (aft is negative)
H.skegZ    =  0.45; % [m]     depth below the CG
H.skegCla  = 2.0;   % [1/rad] lift-curve slope of that low-aspect-ratio surface

% ---- Outboard engine and steering -----------------------------------------
E.Tmax   = 2200;   % [N]  full-throttle thrust (bollard pull, measured with a scale)
E.Umax   = 8.0;    % [m/s] top speed at full throttle (GPS) -> sets the resistance
E.xProp  = -H.Lpp/2;   % [m] longitudinal position of the leg, relative to the CG
E.zProp  =  0.55;      % [m] depth of the propeller below the CG (positive down)
E.dMax   = 27;     % [deg]  steering limit used in the tests
E.rate   = 15.6;   % [deg/s] steering rate limit (from vessel_parameters.m)
E.throttle = 0.6;  % [-]   throttle fraction used in the manoeuvres

% ---- Sea state --------------------------------------------------------------
SEA.on     = true;
SEA.Hs     = 0.6;      % [m]   significant wave height
SEA.Tp     = 5.0;      % [s]   peak period
SEA.gamma  = 3.3;      % [-]   JONSWAP peak factor (1 = Pierson-Moskowitz)
SEA.betaDeg= 135;      % [deg] wave direction relative to the initial heading
                       %       (0 = following, 90 = beam, 180 = head seas)
SEA.nComp  = 40;       % [-]   number of wave components
SEA.seed   = 1;

% ---- Calibration factors (the honest knobs) ---------------------------------
CAL.hull      = 1.0;   % scales the linear sway/yaw derivatives (K and T)
CAL.crossFlow = 1.0;   % scales the cross-flow drag (non-linearity, alpha)
CAL.wave      = 1.0;   % scales all wave excitation

% ---- Simulation -------------------------------------------------------------
SIM.dt   = 0.01;       % [s] integration step
SIM.tEnd = 160;        % [s] length of each manoeuvre

%% 1) DERIVED GEOMETRY AND MASS  =============================================
nabla = H.mass/H.rho;                 % [m^3] displaced volume
Aw    = H.Cwp*H.Lpp*H.B;              % [m^2] waterplane area
m     = H.mass;
Ix    = m*H.rGyr44^2;
Iy    = m*H.rGyr55^2;
Iz    = m*H.rGyr66^2;
U0    = 0.8*E.Umax;                   % [m/s] reference speed for the linear derivatives

% --- Added mass -------------------------------------------------------------
% Surge: 5 % of the mass is the usual rule of thumb for ships.
Xudot = -0.05*m;
% Sway and yaw: Clarke-type regressions, non-dimensional in the prime system
%   Y_vdot' = -pi*(T/L)^2*(1 + 0.16*Cb*B/T - 5.1*(B/L)^2)
%   Y_rdot' = -pi*(T/L)^2*(0.67*B/L - 0.0033*(B/T)^2)
%   N_vdot' = -pi*(T/L)^2*(1.1*B/L - 0.041*B/T)
%   N_rdot' = -pi*(T/L)^2*(1/12 + 0.017*Cb*B/T - 0.33*B/L)
% dimensionalised by 0.5*rho*L^3 (sway) up to 0.5*rho*L^5 (yaw).
LL = H.Lpp;  BB = H.B;  TT = H.T;  CB = H.Cb;
k0 = -pi*(TT/LL)^2;
Yvdot = k0*(1 + 0.16*CB*BB/TT - 5.1*(BB/LL)^2) * 0.5*H.rho*LL^3;
Yrdot = k0*(0.67*BB/LL - 0.0033*(BB/TT)^2)     * 0.5*H.rho*LL^4;
Nvdot = k0*(1.1*BB/LL - 0.041*BB/TT)           * 0.5*H.rho*LL^4;
Nrdot = k0*(1/12 + 0.017*CB*BB/TT - 0.33*BB/LL)* 0.5*H.rho*LL^5;
% Heave, roll and pitch added inertia: chosen so the natural periods come out
% at the measured/expected values.
%   heave:  w_n^2 = rho*g*Aw/(m + Zwdot),  roll: w_n^2 = rho*g*nabla*GMt/(Ix + Kpdot)
Zwdot = -1.0*m;                                   % ~100 % of the mass (blunt hull)
Kpdot = -(H.rho*H.g*nabla*H.GMt*(H.Trol/(2*pi))^2 - Ix);   % from the measured roll period
Mqdot = -0.8*Iy;

MA = -[Xudot 0     0     0     0     0
       0     Yvdot 0     0     0     Yrdot
       0     0     Zwdot 0     0     0
       0     0     0     Kpdot 0     0
       0     0     0     0     Mqdot 0
       0     Nvdot 0     0     0     Nrdot];
MRB = diag([m m m Ix Iy Iz]);        % body frame at the CG, so no coupling terms
M   = MRB + MA;
Minv = inv(M);

%% 2) DAMPING AND RESTORING  =================================================
% --- Linear sway/yaw derivatives (Clarke), evaluated at the reference speed --
%   Y_v' = -pi*(T/L)^2*(1 + 0.4*Cb*B/T)
%   Y_r' = -pi*(T/L)^2*(-0.5 + 2.2*B/L - 0.08*B/T)
%   N_v' = -pi*(T/L)^2*(0.5 + 2.4*T/L)
%   N_r' = -pi*(T/L)^2*(0.25 + 0.039*B/T - 0.56*B/L)
% They are linear in speed, so they are stored PER UNIT SPEED and multiplied by
% the actual |u| at every step. Freezing them at one speed is a common and
% expensive mistake: it changes the directional stability of the hull.
YvU = k0*(1 + 0.4*CB*BB/TT)                  * 0.5*H.rho*LL^2 * CAL.hull;
YrU = k0*(-0.5 + 2.2*BB/LL - 0.08*BB/TT)     * 0.5*H.rho*LL^3 * CAL.hull;
NvU = k0*(0.5 + 2.4*TT/LL)                   * 0.5*H.rho*LL^3 * CAL.hull;
NrU = k0*(0.25 + 0.039*BB/TT - 0.56*BB/LL)   * 0.5*H.rho*LL^4 * CAL.hull;
Yv = YvU*U0;  Yr = YrU*U0;  Nv = NvU*U0;  Nr = NrU*U0;      % at the reference speed
Ys_U = -0.5*H.rho*H.skegArea*H.skegCla;      % skeg side force per unit speed

% --- Surge resistance: calibrated so full thrust gives the measured top speed
kSurge = E.Tmax/E.Umax^2;            % [N/(m/s)^2]  R = kSurge*u*|u|

% --- Heave, roll, pitch: stiffness from hydrostatics, damping from the ratios
Kz   = H.rho*H.g*Aw;                 % [N/m]     heave restoring
Kphi = H.rho*H.g*nabla*H.GMt;        % [N*m/rad] roll restoring
Kth  = H.rho*H.g*nabla*H.GMl;        % [N*m/rad] pitch restoring
Dz   = 2*H.zetaHv *sqrt(Kz  *(m  - Zwdot));
Dphi = 2*H.zetaRol*sqrt(Kphi*(Ix - Kpdot));
Dth  = 2*H.zetaPt *sqrt(Kth *(Iy - Mqdot));

% --- Drift-induced added resistance ------------------------------------------
% A boat in a turn carries a drift angle, and the hull then drags much harder.
% This is what makes a real boat lose 20-40 % of its speed in a hard turn, so
% calibrate cDrift against the speed loss you measure with GPS.
cDrift = 0.3;
Xvv    = cDrift*0.5*H.rho*H.Cdcf*TT*LL;

% --- Cross-flow drag strips (non-linear sway/yaw damping) --------------------
nStrip = 11;
xStrip = linspace(-LL/2, LL/2, nStrip).';
dxS    = LL/(nStrip - 1);

% Pack everything the derivative function needs
par = struct('M', M, 'Minv', Minv, 'MA', MA, 'm', m, 'Ix', Ix, 'Iy', Iy, 'Iz', Iz, ...
    'Yv', Yv, 'Yr', Yr, 'Nv', Nv, 'Nr', Nr, 'kSurge', kSurge, ...
    'Kz', Kz, 'Kphi', Kphi, 'Kth', Kth, 'Dz', Dz, 'Dphi', Dphi, 'Dth', Dth, ...
    'rho', H.rho, 'g', H.g, 'T', TT, 'Cdcf', H.Cdcf*CAL.crossFlow, ...
    'xStrip', xStrip, 'dxS', dxS, 'zHull', H.zHull, 'Xvv', Xvv, ...
    'YvU', YvU, 'YrU', YrU, 'NvU', NvU, 'NrU', NrU, ...
    'YsU', Ys_U, 'skegX', H.skegX, 'skegZ', H.skegZ, ...
    'Tmax', E.Tmax, 'xProp', E.xProp, 'zProp', E.zProp);

fprintf('--- Estimated coefficients ---\n');
fprintf('Displacement %.0f kg, nabla %.2f m^3, Aw %.1f m^2\n', m, nabla, Aw);
fprintf('Added mass: Xudot %.0f, Yvdot %.0f, Nrdot %.0f kg(m^2)\n', Xudot, Yvdot, Nrdot);
fprintf('Linear derivatives at %.1f m/s: Yv %.0f, Yr %.0f, Nv %.0f, Nr %.0f\n', U0, Yv, Yr, Nv, Nr);
fprintf('Natural periods: roll %.1f s, heave %.1f s, pitch %.1f s\n', ...
    2*pi*sqrt((Ix - Kpdot)/Kphi), 2*pi*sqrt((m - Zwdot)/Kz), 2*pi*sqrt((Iy - Mqdot)/Kth));

%% 2b) LINEAR YAW STABILITY AND ANALYTIC NOMOTO INDICES  =====================
% From the linear sway-yaw model (CG at the origin):
%   [m-Yvdot, -Yrdot; -Nvdot, Iz-Nrdot]*[vdot; rdot] = [Yv, Yr-m*u; Nv, Nr]*[v;r]
%                                                      + [Ydelta; Ndelta]*delta
% which gives r/delta = K(1 + T3 s)/((1 + T1 s)(1 + T2 s)) and the first-order
% Nomoto indices K and T = T1 + T2 - T3. A negative time constant means the hull
% is directionally unstable, and no first-order identification will make sense.
uRef = 0.8*E.Umax*E.throttle;
Ydel = -E.Tmax*E.throttle^2;                  % side force per radian of steering
Ndel = E.xProp*Ydel;
m11 = m - Yvdot;    m12 = -Yrdot;
m21 = -Nvdot;       m22 = Iz - Nrdot;
n11 = -(YvU + Ys_U)*uRef;
n12 = -((YrU + Ys_U*H.skegX)*uRef - m*uRef);
n21 = -(NvU + Ys_U*H.skegX)*uRef;
n22 = -(NrU + Ys_U*H.skegX^2)*uRef;
A2 = m11*m22 - m12*m21;
A1 = m11*n22 + n11*m22 - m12*n21 - n12*m21;
A0 = n11*n22 - n12*n21;
num1 = m11*Ndel - m21*Ydel;
num0 = n11*Ndel - n21*Ydel;
Klin  = num0/A0;                              % [1/s] per radian of steering
T3lin = num1/num0;
Tlin  = A1/A0 - T3lin;                        % T1 + T2 - T3
Tpoles = roots([A2 A1 A0]);
fprintf('\nLinear yaw analysis at u = %.1f m/s:\n', uRef);
% K is the same number in deg/s per deg as in rad/s per rad, so no conversion.
fprintf('   K = %.4f 1/s, T = %.2f s, T3 = %.2f s\n', Klin, Tlin, T3lin);
if any(real(Tpoles) > 0) || Tlin < 0
    fprintf(['   DIRECTIONALLY UNSTABLE hull: a zigzag on it cannot identify a\n' ...
        '   sensible first-order model, and the helm can never be left alone.\n' ...
        '   Increase H.skegArea or move the skeg further aft.\n']);
else
    fprintf('   Directionally stable.\n');
end
% How well can a FIRST-order Nomoto model represent this hull? The zero T3 is
% what the first-order model throws away. With the thrust vectored at the
% transom, T3 is large: the yaw moment acts almost instantly while the hull
% still has to accelerate, so the response has a fast term the first-order model
% cannot reproduce. When T3/T is large, a zigzag identification will return K
% and T that reproduce the RESPONSE but are individually unreliable, sliding
% along a constant K/T ratio. K/T -- the yaw acceleration per degree of rudder,
% which is what a controller actually feels -- stays well identified.
fprintf('   T3/T = %.2f, K/T = %.4f 1/s^2\n', T3lin/Tlin, Klin/Tlin);
if T3lin/Tlin > 0.3
    fprintf(['   Large T3: consider quoting the second-order Nomoto model\n' ...
        '   (K, T1, T2, T3) in the thesis and reducing to first order only for\n' ...
        '   control design, where K/T is what matters.\n']);
end

%% 3) WAVE FIELD  =============================================================
wave = buildWaveField(SEA, H);

%% 4) MANOEUVRE 1: 20/20 ZIGZAG  =============================================
zz = struct('amp', 20, 'check', 20, 'tExec', 10, 'rate', E.rate, 'dMax', E.dMax);
R1 = runManoeuvre('zigzag', zz, par, wave, E, SIM);

%% 5) MANOEUVRE 2: TURNING CIRCLE AT MAXIMUM RUDDER  =========================
tc = struct('amp', E.dMax, 'tExec', 10, 'rate', E.rate, 'dMax', E.dMax);
R2 = runManoeuvre('turn', tc, par, wave, E, SIM);

%% 6) MANOEUVRING METRICS AND EQUIVALENT NOMOTO MODEL  =======================
% Zigzag overshoot angles
os = zigzagOvershoots(R1.psi*180/pi, R1.switchIdx, zz.check);
fprintf('\n--- Zigzag %d/%d ---\n', zz.amp, zz.check);
for i = 1:min(2, numel(os))
    if ~isnan(os(i)), fprintf('Overshoot %d: %.1f deg\n', i, os(i)); end
end
fprintf('Max heel during the zigzag: %.1f deg\n', max(abs(R1.phi))*180/pi);

% Turning circle: steady turn rate, speed loss, heel, tactical diameter
iEnd  = R2.t > R2.t(end) - 20;                       % last 20 s = steady turn
rSS   = mean(R2.r(iEnd));                            % [rad/s]
uSS   = mean(sqrt(R2.u(iEnd).^2 + R2.v(iEnd).^2));
fprintf('\n--- Turning circle at %d deg ---\n', E.dMax);
fprintf('Steady turn rate %.2f deg/s, turning radius %.1f m (%.1f Lpp)\n', ...
    rSS*180/pi, uSS/abs(rSS), uSS/abs(rSS)/H.Lpp);
fprintf('Speed in the turn %.2f m/s (%.0f %% of approach speed)\n', uSS, 100*uSS/R2.u(1));
fprintf('Steady heel %.1f deg (outward is negative for a starboard turn)\n', ...
    mean(R2.phi(iEnd))*180/pi);

% --- Equivalent Nomoto model -------------------------------------------------
% K from the steady turn:            r_ss = K*delta   (delta in deg, r in deg/s)
% alpha from the cubic term:         r + alpha*r^3 = K*delta  fitted at two rudder angles
% T from the yaw-rate rise time:     r(t) = r_ss*(1 - exp(-t/T))
Kdeg  = abs(rSS*180/pi)/E.dMax;
iExec = find(R2.t >= tc.tExec, 1);
r63   = 0.63*rSS;
i63   = find(abs(R2.r(iExec:end)) >= abs(r63), 1) + iExec - 1;
Tnom  = R2.t(i63) - tc.tExec;
fprintf('\nEquivalent Nomoto model (linear fit at %d deg rudder):\n', E.dMax);
fprintf('   K = %.4f 1/s, T = %.2f s   (K'' = %.2f, T'' = %.2f non-dimensional)\n', ...
    Kdeg, Tnom, Kdeg*H.Lpp/uSS*pi/180*180/pi, Tnom*uSS/H.Lpp);
fprintf(['Compare these with the K and T identified from your sea trial with ' ...
    'vessel_parameters.m,\nand adjust CAL.hull (K, T) and CAL.crossFlow ' ...
    '(the alpha non-linearity) until they match.\n']);

% Export the zigzag in the same format as nomoto_zigzag_sim.m so the existing
% identification code can extract K, T, alpha and delta_r from this 6-DOF run.
dec = max(1, round(0.1/SIM.dt));            % export at ~10 Hz, like the real log
iDec = 1:dec:numel(R1.t);
zigzagData = struct('t', R1.t(iDec), 'psi', R1.psi(iDec)*180/pi, ...
    'delta', R1.delta(iDec), 'r', R1.r(iDec)*180/pi, ...
    'H', [R1.psi(iDec)*180/pi R1.delta(iDec)]);
save('zigzag_6dof_log.mat', 'zigzagData');
fprintf('\nSaved zigzag_6dof_log.mat (set ID.file to it in vessel_parameters.m)\n');

%% 7) PLOTS  ==================================================================
figure('Name', '6-DOF zigzag', 'Color', 'w');
subplot(3, 1, 1); hold on; grid on; box on;
plot(R1.t, R1.psi*180/pi, 'b-', 'LineWidth', 1.3);
plot(R1.t, R1.delta, 'r--', 'LineWidth', 1.0);
ylabel('[deg]'); legend({'\psi', '\delta'}, 'Location', 'eastoutside');
title(sprintf('%d/%d zigzag, 6-DOF model', zz.amp, zz.check));
subplot(3, 1, 2); hold on; grid on; box on;
plot(R1.t, R1.r*180/pi, 'b-', 'LineWidth', 1.2);
plot(R1.t, R1.phi*180/pi, 'Color', [0.85 0.3 0.1], 'LineWidth', 1.2);
plot(R1.t, R1.theta*180/pi, 'Color', [0.1 0.6 0.2], 'LineWidth', 1.0);
ylabel('[deg/s], [deg]');
legend({'yaw rate r', 'roll \phi', 'pitch \theta'}, 'Location', 'eastoutside');
subplot(3, 1, 3); hold on; grid on; box on;
plot(R1.t, R1.u, 'b-', 'LineWidth', 1.2);
plot(R1.t, R1.v, 'r-', 'LineWidth', 1.0);
plot(R1.t, R1.z, 'k-', 'LineWidth', 1.0);
xlabel('Time [s]'); ylabel('[m/s], [m]');
legend({'surge u', 'sway v', 'heave z'}, 'Location', 'eastoutside');

figure('Name', '6-DOF tracks', 'Color', 'w');
subplot(1, 2, 1); hold on; grid on; box on; axis equal;
plot(R1.y, R1.x, 'b-', 'LineWidth', 1.3);
xlabel('East [m]'); ylabel('North [m]'); title('Zigzag track');
subplot(1, 2, 2); hold on; grid on; box on; axis equal;
plot(R2.y, R2.x, 'b-', 'LineWidth', 1.3);
plot(R2.y(1), R2.x(1), 'go', 'MarkerFaceColor', 'g');
xlabel('East [m]'); ylabel('North [m]');
title(sprintf('Turning circle at %d deg', E.dMax));

%% ========================================================================
%  LOCAL FUNCTIONS
%% ========================================================================

function R = runManoeuvre(kind, mv, par, wave, E, SIM)
% Integrates the 6-DOF model through a zigzag or a turning circle.
    dt = SIM.dt;
    N  = round(SIM.tEnd/dt) + 1;
    t  = (0:N-1).'*dt;

    nu  = [0.8*E.Umax*E.throttle; zeros(5, 1)];   % start near the equilibrium speed
    eta = zeros(6, 1);
    delta = 0;  deltaCmd = 0;  active = false;
    R.t = t;
    R.psi = zeros(N,1); R.r = zeros(N,1); R.phi = zeros(N,1); R.theta = zeros(N,1);
    R.u = zeros(N,1); R.v = zeros(N,1); R.z = zeros(N,1);
    R.x = zeros(N,1); R.y = zeros(N,1); R.delta = zeros(N,1);
    R.switchIdx = [];

    for k = 1:N
        % --- steering command -------------------------------------------------
        psiRel = eta(6)*180/pi;
        if ~active && t(k) >= mv.tExec
            active = true;  deltaCmd = mv.amp;
        elseif active && strcmp(kind, 'zigzag')
            if deltaCmd > 0 && psiRel >= mv.check
                deltaCmd = -mv.amp;  R.switchIdx(end+1) = k;
            elseif deltaCmd < 0 && psiRel <= -mv.check
                deltaCmd = +mv.amp;  R.switchIdx(end+1) = k;
            end
        end
        step  = mv.rate*dt;                              % steering rate limit
        delta = delta + max(-step, min(step, deltaCmd - delta));
        delta = max(-mv.dMax, min(mv.dMax, delta));

        R.psi(k) = eta(6);  R.phi(k) = eta(4);  R.theta(k) = eta(5);
        R.r(k)   = nu(6);   R.u(k)   = nu(1);   R.v(k)     = nu(2);
        R.x(k)   = eta(1);  R.y(k)   = eta(2);  R.z(k)     = eta(3);
        R.delta(k) = delta;

        if k < N
            [nu, eta] = rk4_6dof(nu, eta, delta, E.throttle, par, wave, t(k), dt);
        end
    end
end

function [nu, eta] = rk4_6dof(nu, eta, delta, thr, par, wave, t, h)
% Classical RK4 on the 12-state model, with the rudder held over the step.
    [k1n, k1e] = deriv6(nu,          eta,          delta, thr, par, wave, t);
    [k2n, k2e] = deriv6(nu + h/2*k1n, eta + h/2*k1e, delta, thr, par, wave, t + h/2);
    [k3n, k3e] = deriv6(nu + h/2*k2n, eta + h/2*k2e, delta, thr, par, wave, t + h/2);
    [k4n, k4e] = deriv6(nu + h*k3n,  eta + h*k3e,  delta, thr, par, wave, t + h);
    nu  = nu  + h/6*(k1n + 2*k2n + 2*k3n + k4n);
    eta = eta + h/6*(k1e + 2*k2e + 2*k3e + k4e);
end

function [dnu, deta] = deriv6(nu, eta, delta, thr, par, wave, t)
% Right-hand side of  M*nu_dot = tau - C(nu)*nu - D(nu)*nu - g(eta)
    u = nu(1);  v = nu(2);  w = nu(3);  p = nu(4);  q = nu(5);  r = nu(6);
    phi = eta(4);  th = eta(5);  psi = eta(6);

    % --- Coriolis and centripetal ------------------------------------------------
    nu1 = nu(1:3);  nu2 = nu(4:6);
    CRB = [ zeros(3), -par.m*smtrx(nu1)
           -par.m*smtrx(nu1), -smtrx(diag([par.Ix par.Iy par.Iz])*nu2) ];
    A11 = par.MA(1:3, 1:3);  A12 = par.MA(1:3, 4:6);
    A21 = par.MA(4:6, 1:3);  A22 = par.MA(4:6, 4:6);
    CA  = [ zeros(3),               -smtrx(A11*nu1 + A12*nu2)
           -smtrx(A11*nu1 + A12*nu2), -smtrx(A21*nu1 + A22*nu2) ];

    % --- Damping -------------------------------------------------------------------
    % Surge: quadratic resistance. Sway/yaw: linear derivatives (scaled with the
    % actual speed) plus cross-flow drag integrated along the hull, which is what
    % makes the turn rate saturate at large rudder (the Nomoto alpha term).
    Uref = max(abs(u), 0.3);                          % derivatives scale with speed
    Xd   = -par.kSurge*u*abs(u) - par.Xvv*v*abs(v)*sign(u + 1e-9);
    Ylin = Uref*(par.YvU*v + par.YrU*r);
    Nlin = Uref*(par.NvU*v + par.NrU*r);
    Fskeg = par.YsU*Uref*(v + par.skegX*r);           % skeg / keel side force
    Ylin  = Ylin + Fskeg;
    Nlin  = Nlin + par.skegX*Fskeg;
    vSec = v + par.xStrip*r;                          % lateral speed of each strip
    fSec = -0.5*par.rho*par.Cdcf*par.T*abs(vSec).*vSec;
    Ycf  = sum(fSec)*par.dxS;
    Ncf  = sum(par.xStrip.*fSec)*par.dxS;
    Yh   = Ylin + Ycf;
    Nh   = Nlin + Ncf;
    Kh   = -par.zHull*(Yh - Fskeg) - par.skegZ*Fskeg - par.Dphi*p;  % side forces heel the boat
    Zh   = -par.Dz*w;
    Mh   = -par.Dth*q;

    % --- Restoring (linearised hydrostatics) ------------------------------------
    gEta = [0; 0; par.Kz*eta(3); par.Kphi*sin(phi); par.Kth*sin(th); 0];

    % --- Outboard: the thrust vector is steered, so it drives yaw AND roll -------
    % Sign convention: positive delta = wheel to starboard. The leg swings so the
    % thrust pushes the STERN to port, which sends the bow to starboard, i.e. a
    % positive yaw rate. That is why Fy carries a minus sign here. Get this
    % backwards and the whole autopilot becomes positive feedback.
    Th  = par.Tmax*thr^2;
    dR  = delta*pi/180;
    Fx  = Th*cos(dR);
    Fy  = -Th*sin(dR);
    tauP = [Fx; Fy; 0; -par.zProp*Fy; par.zProp*Fx; par.xProp*Fy];

    % --- Waves ----------------------------------------------------------------------
    tauW = waveForces(t, eta, wave);

    tau = tauP + tauW + [Xd; Yh; Zh; Kh; Mh; Nh];
    dnu = par.Minv*(tau - (CRB + CA)*nu - gEta);

    % --- Kinematics -----------------------------------------------------------------
    cph = cos(phi); sph = sin(phi); cth = cos(th); sth = sin(th);
    cps = cos(psi); sps = sin(psi);
    J1 = [ cps*cth, -sps*cph + cps*sth*sph,  sps*sph + cps*cph*sth
           sps*cth,  cps*cph + sph*sth*sps, -cps*sph + sth*sps*cph
          -sth,      cth*sph,                cth*cph ];
    J2 = [ 1, sph*sth/cth, cph*sth/cth
           0, cph,        -sph
           0, sph/cth,     cph/cth ];
    deta = [J1*nu(1:3); J2*nu(4:6)];
end

function S = smtrx(a)
% Skew-symmetric matrix: S(a)*b = a x b
    S = [ 0,    -a(3),  a(2)
          a(3),  0,    -a(1)
         -a(2),  a(1),  0 ];
end

function wave = buildWaveField(SEA, H)
% JONSWAP components with random phases. Amplitudes from the spectrum, so the
% significant wave height comes out at the requested value.
    wave.on = SEA.on;
    if ~SEA.on
        wave.n = 0;  return;
    end
    rng(SEA.seed);
    wp  = 2*pi/SEA.Tp;
    wLo = 0.4*wp;  wHi = 3.0*wp;
    dW  = (wHi - wLo)/SEA.nComp;
    w   = wLo + ((1:SEA.nComp).' - 0.5)*dW;
    sig = 0.07*ones(size(w));  sig(w > wp) = 0.09;
    alphaPM = 0.0081;
    S   = alphaPM*H.g^2*w.^-5 .* exp(-1.25*(wp./w).^4) .* ...
          SEA.gamma.^exp(-(w - wp).^2 ./ (2*sig.^2*wp^2));
    amp = sqrt(2*S*dW);
    amp = amp*(SEA.Hs/4)/sqrt(sum(amp.^2)/2);     % Hs = 4*sqrt(m0)
    wave.n     = SEA.nComp;
    wave.w     = w;
    wave.k     = w.^2/H.g;                        % deep water
    wave.amp   = amp;
    wave.phase = 2*pi*rand(SEA.nComp, 1);
    wave.beta  = SEA.betaDeg*pi/180;
    wave.rho   = H.rho;  wave.g = H.g;
    wave.nabla = H.mass/H.rho;
    wave.Aw    = H.Cwp*H.Lpp*H.B;
    wave.GMt   = H.GMt;  wave.GMl = H.GMl;
    wave.L     = H.Lpp;  wave.T = H.T;
    wave.slopeEff = 0.7;      % effective wave slope factor for roll (assumption)
    wave.yawLever = 0.10;     % yaw moment lever as a fraction of Lpp (assumption)
    wave.scale = 1.0;
end

function tau = waveForces(t, eta, wave)
% First-order (Froude-Krylov) wave excitation, plus the effective-slope roll
% moment. This is an approximation: no diffraction, no radiation memory, and the
% yaw moment uses an assumed lever arm. Calibrate with CAL.wave.
    tau = zeros(6, 1);
    if ~wave.on || wave.n == 0
        return;
    end
    beta = wave.beta - eta(6);                  % wave direction relative to the bow
    ph   = wave.w*t + wave.phase - wave.k*(eta(1)*cos(wave.beta) + eta(2)*sin(wave.beta));
    dec  = exp(-wave.k*wave.T/2);               % pressure decay over the draft
    a    = wave.amp.*dec;
    s    = sin(ph);  c = cos(ph);

    accX = sum(a.*wave.w.^2 .* s)*cos(beta);    % orbital acceleration components
    accY = sum(a.*wave.w.^2 .* s)*sin(beta);
    elev = sum(a.*c);                           % effective wave elevation at the hull
    slope= sum(a.*wave.k.*s);                   % wave slope

    tau(1) = wave.rho*wave.nabla*accX;                       % surge
    tau(2) = wave.rho*wave.nabla*accY;                       % sway
    tau(3) = -wave.rho*wave.g*wave.Aw*elev;                  % heave (Smith correction in dec)
    tau(4) = wave.rho*wave.g*wave.nabla*wave.GMt*wave.slopeEff*slope*sin(beta);   % roll
    tau(5) = -wave.rho*wave.g*wave.nabla*wave.GMl*0.3*slope*cos(beta);            % pitch
    tau(6) = tau(2)*wave.yawLever*wave.L;                    % yaw, assumed lever
    tau    = tau*wave.scale;
end

function os = zigzagOvershoots(psiDeg, switchIdx, checkAngle)
% Overshoot angle after each rudder reversal.
    n  = numel(switchIdx);
    os = nan(n, 1);
    edges = [switchIdx(:); numel(psiDeg)];
    for i = 1:n
        seg = psiDeg(edges(i):edges(i+1));
        s   = sign(psiDeg(switchIdx(i)));
        [mx, im] = max(s*seg);
        if im < numel(seg), os(i) = mx - checkAngle; end
    end
end