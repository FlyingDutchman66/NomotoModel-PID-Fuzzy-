%% ========================================================================
%  nomoto_zigzag_sim.m
%
%  Base yaw dynamics of a surface vessel using the FIRST-ORDER NON-LINEAR
%  NOMOTO MODEL, excited by a standard Kempf / "Z" (zigzag) manoeuvre and
%  integrated with the classical 4th-order Runge-Kutta (RK4) method.
%
%  Reference model: Lan, Zheng, Chu & Ding, "Parameter Prediction of the
%  Non-Linear Nomoto Model for Different Ship Loading Conditions Using
%  Support Vector Regression", J. Mar. Sci. Eng. 2023, 11, 903, Eq. (1):
%
%        T*r_dot + r + alpha*r^3 = K*(delta + delta_r)
%        psi_dot = r
%
%  -------------------------------------------------------------------------
%  UNIT CONVENTION  (read this before changing any parameter)
%  -------------------------------------------------------------------------
%  Everything in this script is in DEGREES, DEGREES/SECOND and SECONDS,
%  which are the units the paper reports its identified parameters in.
%  This matters because the cubic coefficient alpha is NOT unit-invariant:
%
%        alpha [s^2/rad^2] = alpha [s^2/deg^2] * (180/pi)^2
%        e.g. 0.029 s^2/deg^2  ->  95.2 s^2/rad^2
%
%  If you move this model into a radian-based controller and keep
%  alpha = 0.029, the non-linear term becomes ~3300x too weak without any
%  error message. K [1/s] and T [s] are the same in both unit systems;
%  delta_r only needs the usual deg->rad conversion.
%
%  -------------------------------------------------------------------------
%  SIGN CONVENTION  (same as Fig. 1 of the paper)
%  -------------------------------------------------------------------------
%  psi   : heading, clockwise from North (compass convention)
%  r     : yaw rate, positive = turning to starboard (heading increasing)
%  delta : rudder / steering angle, positive = the direction that produces
%          positive r (because K > 0). On real hardware, confirm this sign
%          on the bench: an inverted actuator sign turns a stable heading
%          loop into positive feedback.
%
%  -------------------------------------------------------------------------
%  WHERE TO PLUG IN YOUR AUTOPILOT LATER
%  -------------------------------------------------------------------------
%  * zigzagLogic()      -> replace with your heading controller (outer loop)
%  * steeringActuator() -> replace with a model of your steering actuator /
%                          inner position loop
%  * nomotoYawAccel()   -> the ONLY place the ship model lives; swap the
%                          parameter struct p (or schedule it on speed)
%
%  Requires MATLAB R2016b or newer (local functions inside a script).
%  No toolboxes needed.
%% ========================================================================

clear; clc; close all;

%% 1) MANOEUVRING PARAMETERS OF THE NOMOTO MODEL  --------------------------
% Placeholder values: MILS-identified parameters printed in Fig. 4(e) of
% the reference paper (1:266 scaled KVLCC2 free-running model, Lpp = 1.2 m).
% They describe a 1.2 m tanker model, not any real vessel you will control.
% Replace them with parameters identified from your own zigzag trials.

% K : TURNING (GAIN) INDEX  [1/s]
%     Steady-state yaw rate produced per degree of rudder in the linear
%     region (r_ss ~ K*delta when alpha*r^2 << 1). Large K = the vessel turns
%     quickly for a given rudder angle. K rises with speed because more
%     flow passes the rudder (or more thrust is vectored, for an outboard),
%     so a fixed-gain heading controller becomes more aggressive at higher
%     speed. That dependence is the main argument for gain scheduling.
p.K = 0.655;

% T : TIME CONSTANT (TIME COEFFICIENT)  [s]
%     Ratio of yaw inertia (including hydrodynamic added inertia) to yaw
%     damping. In the linear case, the time needed to reach ~63 % of the
%     steady yaw rate after a step of rudder. Small T = responsive vessel;
%     large T = sluggish vessel, bigger zigzag overshoot, harder course
%     keeping. K/T is the yaw acceleration per degree of rudder at the
%     instant the rudder moves ("initial rudder effectiveness").
p.T = 2.936;

% alpha : NON-LINEAR (CUBIC) YAW DAMPING COEFFICIENT  [s^2/deg^2]
%     Extra damping that grows with r^3.
%     alpha > 0 : "hardening" damping. The steady turn rate saturates at
%                 large rudder angles instead of growing linearly.
%     alpha < 0 : "softening" damping. Net damping (1 + alpha*r^2) falls as
%                 yaw rate rises and reaches zero at |r| = 1/sqrt(|alpha|);
%                 beyond that the model diverges, so it is only valid below
%                 that yaw rate. Many rows of the paper's appendix have
%                 alpha < 0, so this case is not hypothetical.
p.alpha = 0.029;

% delta_r : EFFECTIVE NEUTRAL RUDDER ANGLE  [deg]
%     A constant, rudder-equivalent bias that lumps together propeller
%     side force / torque, hull asymmetry, trim, rudder-sensor zero offset
%     and steady environmental loads. With delta = 0 the vessel does NOT go
%     straight: it turns at r ~ K*delta_r. Holding a straight course needs
%     delta = -delta_r. In a closed-loop autopilot, cancelling this bias is
%     exactly the job of the INTEGRAL term of the heading controller.
p.delta_r = 1.249;

% delta (the actual rudder angle) is not a constant parameter: it is the
% input to the model, produced every step by the zigzag logic and the
% steering actuator model below.

%% 2) SIMULATION AND TEST SETTINGS  ----------------------------------------
cfg.h    = 0.1;     % [s] integration AND controller step (10 Hz, the same
                    %     logging rate the paper used for its lake tests)
cfg.tEnd = 120;     % [s] total simulated time

% --- Zigzag (Kempf / "Z") manoeuvre ---------------------------------------
% Notation "A/B": rudder angle A, heading-change check angle B.
%   1) Steady straight approach on the initial heading psi0.
%   2) Execute: rudder to +A (or -A).
%   3) When the heading has changed by +B from psi0, reverse rudder to -A.
%   4) When the heading has changed by -B from psi0, reverse to +A. Repeat.
% After each reversal the heading keeps swinging past the check angle
% because the vessel's yaw inertia (T) must first be overcome. That excess
% is the OVERSHOOT ANGLE, the main quantity a zigzag test measures.
zz.rudderAmp  = 20;   % [deg] A  (set 10 for a 10/10 test)
zz.checkAngle = 20;   % [deg] B
zz.firstDir   = +1;   % +1: first execute to starboard, -1: to port
zz.tExecute   = 5;    % [s]  straight approach before the first execute

% --- Steering actuator ----------------------------------------------------
% A real rudder (or outboard steering) cannot jump instantly; it slews at a
% finite rate, and the overshoot angle depends noticeably on that rate.
% The paper Froude-scaled its model's rudder rate (38.24 deg/s) from the
% full-scale ship (2.34 deg/s) for exactly this reason. The default below
% matches the placeholder parameters (same 1.2 m model).
% Set rateLimit = Inf for an ideal, instantaneous rudder. For your own
% vessel, use the measured slew rate of your steering mechanism.
act.rateLimit = 38.24;   % [deg/s]
act.maxAngle  = 35;      % [deg] mechanical limit (hard stop)

% --- Initial conditions -----------------------------------------------------
x0.psi = 0;      % [deg]   initial heading; the zigzag is referenced to it
x0.r   = 0;      % [deg/s] initial yaw rate
x0.xE  = 0;      % [m]     initial East position
x0.yN  = 0;      % [m]     initial North position
% The rudder starts at the angle that holds a straight course (-delta_r),
% so the approach phase is a true equilibrium of the model.
delta0 = -p.delta_r;

% --- Planar kinematics (Eq. 2 of the paper) -------------------------------
% The Nomoto model covers yaw only. To draw a track we assume a constant
% surge speed U and zero sway (v = 0), i.e. no drift angle. Good enough to
% visualise the manoeuvre; not a replacement for a 3-DOF model.
kin.U = 0.5;     % [m/s] assumed surge speed (not given in the paper)

% --- Synthetic measurement noise for the exported training data -----------
% The simulation itself stays noise-free ("truth"). Noise is added only to
% the exported copy, so it can be used to test LS / MILS identification
% code under sensor-like conditions. Tune to your heading sensor.
meas.addNoise = true;
meas.sigmaPsi = 0.2;    % [deg]   heading noise standard deviation
meas.sigmaR   = 0.3;    % [deg/s] yaw-rate noise standard deviation
meas.seed     = 1;      % random seed, for reproducible datasets
saveData      = true;   % write zigzag_training_data.mat to the current folder

%% 3) SANITY CHECKS ON THE PARAMETER SET  ----------------------------------
assert(p.T > 0, 'T must be positive: it is an inertia-to-damping ratio.');
assert(p.K > 0, ['K must be positive with this sign convention. If your ' ...
    'identified K is negative, flip the rudder sign instead.']);

% Analytic steady turning rates at +/- A (root of r + alpha*r^3 = K*(delta+delta_r))
rssPos = steadyYawRate(+zz.rudderAmp, p);
rssNeg = steadyYawRate(-zz.rudderAmp, p);

if p.alpha < 0
    rCrit = 1/sqrt(-p.alpha);
    fprintf(['Note: alpha < 0, model is only valid for |r| < %.2f deg/s ' ...
        '(net damping vanishes there).\n'], rCrit);
    if isnan(rssPos) || isnan(rssNeg)
        warning(['No bounded steady turn exists at %g deg rudder with ' ...
            'this alpha: the simulation may diverge.'], zz.rudderAmp);
    end
end

% RK4 step-size check. Linearising Eq. (1) about a yaw rate r gives the pole
%   lambda = -(1 + 3*alpha*r^2)/T.
% RK4 is stable for h*|lambda| < ~2.8 and accurate for h*|lambda| << 1.
rRef = max(abs([rssPos, rssNeg]));
if ~isfinite(rRef), rRef = 0; end
lambdaMax = abs((1 + 3*p.alpha*rRef^2)/p.T);
fprintf('RK4 check: h*|lambda| = %.3f (keep well below 1)\n', cfg.h*lambdaMax);
if cfg.h*lambdaMax > 1
    warning('Step size is coarse relative to the yaw dynamics; reduce cfg.h.');
end

%% 4) PRE-ALLOCATION  -------------------------------------------------------
N    = floor(cfg.tEnd/cfg.h) + 1;
t    = (0:N-1).' * cfg.h;   % time vector [s]
X    = zeros(N, 4);         % state history, columns [psi r xE yN]
dCmd = zeros(N, 1);         % commanded rudder angle  [deg]
dAct = zeros(N, 1);         % actual (applied) rudder [deg]
switchLog = [];             % sample indices of rudder reversals

x        = [x0.psi; x0.r; x0.xE; x0.yN];   % current state
psi0     = x0.psi;                          % zigzag reference heading
deltaCmd = delta0;                          % current rudder command
delta    = delta0;                          % current actual rudder
zzActive = false;                           % false during the approach

%% 5) MAIN SIMULATION LOOP  -------------------------------------------------
for k = 1:N

    % (a) Log the state at t(k)
    X(k, :) = x.';

    % (b) RUDDER COMMAND: zigzag test logic.
    %     This block plays the role of the "controller". To test your
    %     autopilot later, replace it with: deltaCmd = controller(psi_ref, x)
    psiRel = x(1) - psi0;                 % heading change from psi0 [deg]
    if ~zzActive
        if t(k) >= zz.tExecute            % first execute
            zzActive = true;
            deltaCmd = zz.firstDir * zz.rudderAmp;
        end
    else
        deltaCmdNew = zigzagLogic(psiRel, deltaCmd, zz);
        if deltaCmdNew ~= deltaCmd        % a reversal happened this step
            switchLog(end+1) = k; %#ok<AGROW>
            deltaCmd = deltaCmdNew;
        end
    end

    % (c) STEERING ACTUATOR: rate limit + saturation -> actual rudder angle
    delta = steeringActuator(delta, deltaCmd, act, cfg.h);

    dCmd(k) = deltaCmd;
    dAct(k) = delta;

    % (d) INTEGRATE ONE STEP WITH RK4.
    %     delta is held constant over [t(k), t(k)+h] (zero-order hold),
    %     exactly like a digital controller between two updates.
    if k < N
        x = rk4Step(@shipDerivatives, x, delta, p, kin, cfg.h);
        if any(~isfinite(x))
            error(['State diverged at t = %.1f s. With alpha < 0 the ' ...
                'model is only valid for |r| < 1/sqrt(|alpha|).'], t(k+1));
        end
    end
end

%% 6) POST-PROCESSING: ZIGZAG METRICS  -------------------------------------
psiRelHist = X(:, 1) - psi0;
[os, osIdx] = zigzagOvershoots(psiRelHist, switchLog, zz.checkAngle);

fprintf('\n=== %g/%g zigzag, first-order non-linear Nomoto model ===\n', ...
    zz.rudderAmp, zz.checkAngle);
fprintf('K = %.3f 1/s | T = %.3f s | alpha = %.4f s^2/deg^2 | delta_r = %.3f deg\n', ...
    p.K, p.T, p.alpha, p.delta_r);
fprintf('Steady yaw rate at %+g deg rudder: %6.2f deg/s (linear model: %6.2f)\n', ...
    +zz.rudderAmp, rssPos, p.K*(+zz.rudderAmp + p.delta_r));
fprintf('Steady yaw rate at %+g deg rudder: %6.2f deg/s (linear model: %6.2f)\n', ...
    -zz.rudderAmp, rssNeg, p.K*(-zz.rudderAmp + p.delta_r));
if ~isempty(switchLog)
    fprintf('Time from execute to first check angle: %.1f s\n', ...
        t(switchLog(1)) - zz.tExecute);
end
for i = 1:numel(os)
    if ~isnan(os(i))
        fprintf('Overshoot %d: %6.2f deg  (peak at t = %5.1f s)\n', i, os(i), t(osIdx(i)));
    end
end
fprintf('Max |r| reached: %.2f deg/s\n', max(abs(X(:, 2))));
if p.alpha < 0 && max(abs(X(:, 2))) > 0.8*rCrit
    warning('Max |r| reached %.0f%% of the validity limit 1/sqrt(|alpha|).', ...
        100*max(abs(X(:, 2)))/rCrit);
end

%% 7) VISUALISATION  --------------------------------------------------------
figure('Name', 'Nomoto zigzag: time histories', 'Color', 'w');

% --- (a) Heading change and rudder angle (as in Fig. 8 of the paper) ------
ax1 = subplot(3, 1, 1); hold on; grid on; box on;
h1 = plot(t, psiRelHist, 'b-', 'LineWidth', 1.4);
h2 = plot(t, dAct, 'r--', 'LineWidth', 1.1);
h3 = plot([t(1) t(end)],  zz.checkAngle*[1 1], 'k:', 'LineWidth', 1.0);
plot([t(1) t(end)], -zz.checkAngle*[1 1], 'k:', 'LineWidth', 1.0);
ok = ~isnan(osIdx);
h4 = plot(t(osIdx(ok)), psiRelHist(osIdx(ok)), 'kv', 'MarkerFaceColor', 'k');
okIdx = find(ok).';
for i = okIdx(1:min(2, end))   % label the 1st and 2nd overshoot (the values usually reported)
    if psiRelHist(osIdx(i)) >= 0, va = 'bottom'; else, va = 'top'; end
    text(t(osIdx(i)), psiRelHist(osIdx(i)), sprintf('  OS_%d = %.1f^\\circ', i, os(i)), ...
        'VerticalAlignment', va, 'FontSize', 8);
end
ylabel('\psi - \psi_0, \delta  [deg]');
title(sprintf('%g/%g zigzag: heading change and rudder angle', ...
    zz.rudderAmp, zz.checkAngle));
legend([h1 h2 h3 h4], {'\psi - \psi_0', '\delta (actual)', ...
    '\pm check angle', 'overshoot peak'}, 'Location', 'eastoutside');

% --- (b) Yaw rate -----------------------------------------------------------
% During a zigzag r usually does not fully settle before the next reversal;
% the dashed lines show the steady turning rates it is heading towards.
ax2 = subplot(3, 1, 2); hold on; grid on; box on;
g1 = plot(t, X(:, 2), 'b-', 'LineWidth', 1.4);
g2 = plot([t(1) t(end)], rssPos*[1 1], '--', 'Color', [0 0.6 0]);
plot([t(1) t(end)], rssNeg*[1 1], '--', 'Color', [0 0.6 0]);
ylabel('r  [deg/s]');
title('Yaw angular velocity');
legend([g1 g2], {'r', 'steady r at \pm\delta'}, 'Location', 'eastoutside');

% --- (c) Rudder command vs actual (shows the actuator rate limit) ----------
ax3 = subplot(3, 1, 3); hold on; grid on; box on;
k1 = stairs(t, dCmd, 'k-', 'LineWidth', 1.0);
k2 = plot(t, dAct, 'r-', 'LineWidth', 1.2);
ylabel('\delta  [deg]'); xlabel('Time  [s]');
title(sprintf('Steering actuator (rate limit %.4g deg/s)', act.rateLimit));
legend([k1 k2], {'\delta_{cmd}', '\delta (rate-limited)'}, 'Location', 'eastoutside');

linkaxes([ax1 ax2 ax3], 'x');
xlim(ax1, [t(1) t(end)]);

% --- Track in the Earth-fixed frame -------------------------------------------
figure('Name', 'Nomoto zigzag: track', 'Color', 'w');
hold on; grid on; box on; axis equal;
m1 = plot(X(:, 3), X(:, 4), 'b-', 'LineWidth', 1.4);
m2 = plot(X(1, 3), X(1, 4), 'go', 'MarkerFaceColor', 'g');
m3 = plot(X(switchLog, 3), X(switchLog, 4), 'rx', 'MarkerSize', 8, 'LineWidth', 1.5);
xlabel('East, X  [m]'); ylabel('North, Y  [m]');
title(sprintf('Track (constant U = %.2f m/s, v = 0 assumed)', kin.U));
legend([m1 m2 m3], {'track', 'start', 'rudder reversal'}, 'Location', 'best');

%% 8) EXPORT TRAINING DATA  ---------------------------------------------------
% H = [psi delta] mirrors the "preliminary training set" H(t) of Sec. 4.3.1
% of the paper, so the file can be fed straight into LS / MILS code.
rng(meas.seed);
psiMeas = X(:, 1);
rMeas   = X(:, 2);
if meas.addNoise
    psiMeas = psiMeas + meas.sigmaPsi*randn(N, 1);
    rMeas   = rMeas   + meas.sigmaR  *randn(N, 1);
end

zigzagData.t        = t;            % [s]
zigzagData.psi      = psiMeas;      % [deg]   measured heading (noisy if enabled)
zigzagData.r        = rMeas;        % [deg/s] measured yaw rate
zigzagData.delta    = dAct;         % [deg]   applied rudder angle
zigzagData.deltaCmd = dCmd;         % [deg]   commanded rudder angle
zigzagData.psiTrue  = X(:, 1);      % [deg]   noise-free heading
zigzagData.rTrue    = X(:, 2);      % [deg/s] noise-free yaw rate
zigzagData.pos      = X(:, 3:4);    % [m]     [East North]
zigzagData.H        = [psiMeas dAct];
zigzagData.params   = p;            % ground-truth Nomoto parameters
zigzagData.settings = struct('cfg', cfg, 'zz', zz, 'act', act, ...
                             'kin', kin, 'meas', meas);

if saveData
    save('zigzag_training_data.mat', 'zigzagData');
    fprintf('Saved zigzag_training_data.mat (%d samples at %g Hz)\n', N, 1/cfg.h);
end

%% ========================================================================
%  LOCAL FUNCTIONS
%% ========================================================================

function rdot = nomotoYawAccel(r, delta, p)
% FIRST-ORDER NON-LINEAR NOMOTO MODEL, Eq. (1) of the reference paper:
%
%        T*r_dot = K*(delta + delta_r) - r - alpha*r^3
%
%   K*(delta + delta_r) : rudder forcing (plus the bias delta_r), written as
%                         the yaw rate it would eventually sustain
%   - r                 : linear hydrodynamic yaw damping
%   - alpha*r^3         : non-linear yaw damping (turn-rate saturation)
%   / T                 : converts the net "rate error" into yaw
%                         acceleration; a large T means slow response
%
% This is the ONLY place the ship model lives. Changing vessel, speed or
% loading means changing p (or scheduling p on speed); the integrator and
% controller code need not change.
    rdot = (p.K*(delta + p.delta_r) - r - p.alpha*r.^3) / p.T;
end

function dx = shipDerivatives(x, delta, p, kin)
% Time derivative of the augmented state x = [psi; r; xE; yN].
% Yaw comes from the Nomoto model; position comes from Eq. (2) with the
% assumptions u = U (constant) and v = 0.
%
% Kinematics note: for X = East, Y = North and psi clockwise from North,
%        X_dot = u*sin(psi) + v*cos(psi)
%        Y_dot = u*cos(psi) - v*sin(psi)
% The second row of R(psi) as printed in Eq. (3) of the paper has the
% opposite sign (it gives Y_dot = -u when heading due North), so the
% standard form above is used here.
    psi = x(1);
    r   = x(2);
    dx  = zeros(4, 1);
    dx(1) = r;                              % psi_dot = r
    dx(2) = nomotoYawAccel(r, delta, p);    % r_dot from the Nomoto model
    dx(3) = kin.U * sind(psi);              % East rate  [m/s]
    dx(4) = kin.U * cosd(psi);              % North rate [m/s]
end

function xNext = rk4Step(f, x, delta, p, kin, h)
% Classical 4th-order Runge-Kutta step.
% The input delta is held constant across the step (zero-order hold).
% Because rudder changes only happen at step boundaries, RK4 keeps its
% 4th-order accuracy inside each step.
    k1 = f(x,              delta, p, kin);
    k2 = f(x + 0.5*h*k1,   delta, p, kin);
    k3 = f(x + 0.5*h*k2,   delta, p, kin);
    k4 = f(x +     h*k3,   delta, p, kin);
    xNext = x + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
end

function deltaCmd = zigzagLogic(psiRel, deltaCmd, zz)
% Bang-bang switching law of the zigzag test.
% The rudder is reversed only when the heading has passed the check angle
% on the side the current rudder is pushing it towards. This built-in
% hysteresis stops the rudder from chattering while the heading is still
% overshooting beyond the check angle.
    if deltaCmd > 0 && psiRel >= zz.checkAngle
        deltaCmd = -zz.rudderAmp;
    elseif deltaCmd < 0 && psiRel <= -zz.checkAngle
        deltaCmd = +zz.rudderAmp;
    end
end

function delta = steeringActuator(delta, deltaCmd, act, h)
% Rate-limited, saturated steering actuator.
% Moves the actual rudder towards the command by at most rateLimit*h per
% step, then clips it to the mechanical limit. With rateLimit = Inf the
% rudder follows the command instantly.
    maxStep = act.rateLimit * h;
    delta   = delta + max(-maxStep, min(maxStep, deltaCmd - delta));
    delta   = max(-act.maxAngle, min(act.maxAngle, delta));
end

function rss = steadyYawRate(delta, p)
% Steady turning rate for a constant rudder angle: the root of
%        alpha*r^3 + r - K*(delta + delta_r) = 0
% that lies on the stable branch (same sign as the forcing, smallest |r|).
% Returns NaN if no bounded steady turn exists (possible when alpha < 0).
    c = p.K*(delta + p.delta_r);
    if p.alpha == 0
        rss = c;
        return;
    end
    rt = roots([p.alpha 0 1 -c]);
    rt = real(rt(abs(imag(rt)) < 1e-9));
    rt = rt(sign(rt) == sign(c));
    if isempty(rt)
        rss = NaN;
    else
        [~, i] = min(abs(rt));
        rss = rt(i);
    end
end

function [os, osIdx] = zigzagOvershoots(psiRel, switchIdx, checkAngle)
% Overshoot angle after each rudder reversal: the largest heading excursion
% beyond the check angle before the heading turns back.
% os(i) = NaN if the simulation ended before the i-th peak was reached.
    n     = numel(switchIdx);
    os    = nan(n, 1);
    osIdx = nan(n, 1);
    edges = [switchIdx(:); numel(psiRel)];
    for i = 1:n
        seg = psiRel(edges(i):edges(i+1));
        s   = sign(psiRel(switchIdx(i)));   % side where the check was hit
        [m, im] = max(s*seg);
        if im < numel(seg)                  % peak lies inside the segment
            os(i)    = m - checkAngle;
            osIdx(i) = edges(i) + im - 1;
        end
    end
end