%% ========================================================================
%  vessel_parameters.m (Updated for Marlin Cruiser 480)
%
%  Computes EVERY parameter used by nomoto_zigzag_sim.m and
%  heading_control_comparison.m from things you can measure, instead of
%  leaving them as placeholders. Each block states the equation, the inputs
%  it needs and how to measure them.
%
%  WHAT COMES FROM WHERE
%  -------------------------------------------------------------------------
%   Ship     K, T, alpha, delta_r   <- identification on a zigzag log (LS, MILS
%                                      and an output-error fit), or a scaling
%                                      estimate before you have a log
%   Actuator wMax, tauM, uDead,     <- motor datasheet + belt/steering ratios
%            backlash, encRes,         + three bench tests
%            inner Kp
%   Control  Kp0, Ki0, Kd0, wn      <- pole placement, capped by actuator
%                                      bandwidth, sample rate and rudder authority
%   Fuzzy    Ke, Kec, cp, ci, cd    <- error scaling from the physics, gain
%                                      ranges from the same bandwidth and a
%                                      rudder-noise budget
%   Waves    sigma, Tp              <- inverted from heading oscillations you
%                                      measure with the autopilot OFF
%   Sensors  sigmaPsi, sigmaR       <- standard deviation of a static log
%
%  UNITS: degrees, deg/s, seconds, metres. Requires MATLAB R2016b+, no toolboxes.
%% ========================================================================
clear; clc; close all;
%% 0) MEASURED INPUTS  =======================================================
% ---- 0.1 Vessel (Marlin Cruiser 480 Specs) ---------------------------------
M.Lpp   = 4.80;    % [m]   length between perpendiculars (Hull maximum)[cite: 8]
M.Beam  = 2.10;    % [m]   beam / width of the boat[cite: 8]
M.U     = 4.0;     % [m/s] service speed for this parameter set (GPS)
% น้ำหนักรวมประมาณการ: เรือเปล่า 440 kg + เครื่องยนต์ (~110 kg) + น้ำมัน 70L (~50 kg) + ผู้โดยสาร/อุปกรณ์
M.Mass  = 440 + 110 + 50 + 100; % [kg] Total estimated displacement mass

% ---- 0.2 Steering mechanism (measure once, with a protractor and the AS5600)
M.Vbatt   = 12.0;  % [V]    supply voltage at the BTS7960
M.Vnom    = 12.0;  % [V]    voltage the motor's no-load speed is quoted at
M.n0      = 200;   % [rpm]  motor no-load speed at Vnom (datasheet or tachometer)
M.loadFac = 0.7;   % [-]    fraction of no-load speed reached under steering load
                   %        (measure: time a full lock-to-lock sweep at 100 % duty)
M.iBelt   = 3.0;   % [-]    belt reduction = wheel pulley teeth / motor pulley teeth
M.turnsLL = 3.0;   % [turns] steering-wheel turns lock to lock
M.dRange  = 60;    % [deg]  total outboard steering travel lock to lock
M.encBits = 12;    % [-]    AS5600 resolution (12 bit)
M.encOnWheel = true;  % true: magnet on the steering-wheel shaft
                      % false: magnet on the motor shaft (before the belt)
% ---- 0.3 Three bench tests -------------------------------------------------
M.t63     = 0.10;  % [s]   step test: time for the steering speed to reach 63 %
                   %       of its final value after a step in duty
M.uStart  = 0.08;  % [-]   dead-zone test: smallest duty that makes the wheel
                   %       move at all, under load (PWM_start / PWM_max)
M.backlash = 2.0;  % [deg] play test: drive one way, stop, reverse, and record
                   %       how far the motor turns before the outboard moves
                   %       (expressed as outboard degrees)
% ---- 0.4 Sensors and loop rates --------------------------------------------
M.sigmaPsiMeas = 0.3;  % [deg]   std of a STATIC heading log (boat tied up, engine
                       %         running so vibration and magnetic noise are included)
M.sigmaRMeas   = 0.2;  % [deg/s] std of the same log's yaw rate
M.Ts           = 0.1;  % [s]     outer (heading) loop period you will run on the STM32
M.delaySamples = 1;    % [-]     outer-loop delay (computation + link), in samples
% ---- 0.5 Sea state, measured with the autopilot OFF on a steady course -------
M.sigmaPsiWave = 3.0;  % [deg] std of heading oscillation caused by waves
M.Twave        = 5.0;  % [s]   dominant (encounter) period of that oscillation
M.jonswapGamma = 3.3;  % [-]   3.3 = JONSWAP, 1 = Pierson-Moskowitz
% ---- 0.6 Design choices (not measurements) ----------------------------------
D.zetaInner = 0.8;   % [-]   damping wanted from the inner rudder loop
D.zetaOuter = 0.9;   % [-]   damping wanted from the heading loop
D.dMaxMech  = M.dRange/2;   % [deg] hard stop = half the lock-to-lock travel
D.dMax      = 0.9*D.dMaxMech;  % [deg] controller limit, kept inside the hard stop
D.eLin      = 15;    % [deg] heading error up to which you want the heading loop
                     %       to stay out of rudder saturation
D.eFuzzMax  = 30;    % [deg] error that saturates the fuzzy input universe
D.rudNoiseBudget = 1.0;  % [deg] RMS rudder motion you accept from sensor noise alone
% ---- 0.7 Zigzag log for identification --------------------------------------
ID.file     = 'zigzag_training_data.mat';
ID.smoothN  = 11;    % [samples] zero-phase moving average applied to the heading
ID.innovLen = 4;     % [-] MILS innovation length p
ID.rateHz   = 10;    % [Hz] the log is decimated to this rate before fitting
ID.useOE    = true;  % refine with an output-error fit
ID.trueVals = [];    % [K T alpha delta_r]
%% 1) STEERING ACTUATOR  =====================================================
A.wMax = (M.n0 * M.Vbatt/M.Vnom * M.loadFac / 60) / M.iBelt * (M.dRange / M.turnsLL);
A.tauM = M.t63;
A.uDead = M.uStart;
A.backlash = M.backlash;
encStep = 360/2^M.encBits;
if M.encOnWheel
    A.encRes  = encStep * (M.dRange/M.turnsLL)/360;
    A.encoder = 'rudder';
else
    A.encRes  = encStep * (M.dRange/M.turnsLL)/360 / M.iBelt;
    A.encoder = 'motor';
end
A.dMaxMech = D.dMaxMech;
A.wnInner = 1/(2*D.zetaInner*A.tauM);
A.Kp      = 1/(4*D.zetaInner^2*A.tauM*A.wMax);
A.Ki      = 0;
A.dt      = roundToNice(A.tauM/20);
%% 2) SHIP: NOMOTO PARAMETERS  ===============================================
haveLog = ~isempty(ID.file) && exist(ID.file, 'file') == 2;
if haveLog
    [tLog, psiLog, deltaLog] = loadZigzagLog(ID.file);
    dec = max(1, round(1/(ID.rateHz*median(diff(tLog)))));
    tLog = tLog(1:dec:end);  psiLog = psiLog(1:dec:end);  deltaLog = deltaLog(1:dec:end);
    h    = median(diff(tLog));
    cand = {identifyNomoto(psiLog, deltaLog, h, 'LS',   ID), ...
            identifyNomoto(psiLog, deltaLog, h, 'MILS', ID)};
    if ID.useOE
        cand{end+1} = refineOutputError(cand{2}, tLog, psiLog, deltaLog);
    end
    fprintf('--- Nomoto identification from %s (h = %.3f s, %d samples) ---\n', ...
        ID.file, h, numel(tLog));
    fprintf('%-8s %8s %8s %10s %9s %9s %7s\n', ...
        'method', 'K [1/s]', 'T [s]', 'alpha', 'delta_r', 'RMSE psi', 'PCC');
    rmse = zeros(1, numel(cand));
    for q = 1:numel(cand)
        v = validateModel(cand{q}, tLog, psiLog, deltaLog);
        rmse(q) = v.RMSE;
        fprintf('%-8s %8.3f %8.3f %10.5f %9.3f %9.3f %7.4f\n', cand{q}.method, ...
            cand{q}.K, cand{q}.T, cand{q}.alpha, cand{q}.delta_r, v.RMSE, v.PCC);
    end
    if ~isempty(ID.trueVals)
        fprintf('%-8s %8.3f %8.3f %10.5f %9.3f  (values used to generate the log)\n', ...
            'truth', ID.trueVals(1), ID.trueVals(2), ID.trueVals(3), ID.trueVals(4));
    end
    [~, best] = min(rmse);
    id = cand{best};
    P.lo.K = id.K;  P.lo.T = id.T;  P.lo.alpha = id.alpha;  P.lo.delta_r = id.delta_r;
    fprintf('Using the %s fit.\n\n', id.method);
else
    % ปรับสเกลเริ่มต้นให้สอดคล้องกับขนาดเรือ Cruiser 480 (Lpp = 4.8 ม.)
    Kp_nd = 1.5;   Tp_nd = 2.0;
    P.lo.K       = Kp_nd*M.U/M.Lpp;
    P.lo.T       = Tp_nd*M.Lpp/M.U;
    P.lo.alpha   = 0;
    P.lo.delta_r = 0;
    fprintf(['--- No zigzag log: using the scaling estimate for Cruiser 480 ---\n' ...
        'K'' = %.1f, T'' = %.1f at U = %.1f m/s, Lpp = %.2f m -> K = %.3f 1/s, T = %.2f s\n\n'], ...
        Kp_nd, Tp_nd, M.U, M.Lpp, P.lo.K, P.lo.T);
end
P.hi   = P.lo;
P.hi.K = 1.37*P.lo.K;
P.hi.T = 0.64*P.lo.T;
assert(P.lo.K > 0 && P.lo.T > 0, ['Identification returned K = %.3f, T = %.3f. ' ...
    'Check the rudder sign convention.'], P.lo.K, P.lo.T);
rMax = steadyYawRate(D.dMax, P.lo);
if isnan(rMax)
    rMax = P.lo.K*(D.dMax + P.lo.delta_r);
    warning('No bounded steady turn at %g deg rudder; using the linear estimate.', D.dMax);
end
%% 3) OUTER (HEADING) CONTROLLER  ============================================
C.wnActuator = A.wnInner/5;
C.wnSample   = 0.15/M.Ts;
C.wnAuthority = sqrt(P.lo.K*D.dMax/(P.lo.T*D.eLin));
C.wnDelay    = 0.3/max(M.delaySamples*M.Ts, eps);
C.wnMax      = min([C.wnActuator, C.wnSample, C.wnAuthority, C.wnDelay]);
C.wn         = 0.8*C.wnMax;
C.zeta       = D.zetaOuter;
g0 = polePlacementPID(P.lo.K, P.lo.T, C.wn, C.zeta);
C.dMax = D.dMax;
%% 4) FUZZY SCALING GAINS  ===================================================
F.Ke  = 3/D.eFuzzMax;
F.Kec = 3/rMax;
S.tauRf = 0.3;
aRf     = M.Ts/(S.tauRf + M.Ts);
sigmaRf = M.sigmaRMeas*sqrt(aRf/(2 - aRf));
F.cp = min(0.5, (C.wnMax/C.wn)^2 - 1);
kdRoom = (D.rudNoiseBudget^2 - (g0.Kp*M.sigmaPsiMeas)^2);
if kdRoom > 0
    F.cd = min(0.5, sqrt(kdRoom)/(g0.Kd*sigmaRf) - 1);
else
    F.cd = 0;
end
F.cd = max(F.cd, 0);
F.ci = 1.0;
%% 5) ENVIRONMENT AND SENSORS  ===============================================
W.Tp    = M.Twave;
wp      = 2*pi/W.Tp;
W.sigma = M.sigmaPsiWave*wp*sqrt(1 + (wp*P.lo.T)^2)/P.lo.K;
W.gamma = M.jonswapGamma;
W.nComp = 60;
S.Ts       = M.Ts;
S.delay    = M.delaySamples;
S.sigmaPsi = M.sigmaPsiMeas;
S.sigmaR   = M.sigmaRMeas;
%% 6) CONSISTENCY CHECKS  ====================================================
fprintf('--- Checks for Marlin Cruiser 480 ---\n');
lamShip  = (1 + 3*P.lo.alpha*rMax^2)/P.lo.T;
lamMotor = 1/A.tauM;
lamMax   = max(lamShip, lamMotor);
fprintf('RK4 step: dt*|lambda_max| = %.3f  (want < 0.1; dt = %.4f s)\n', A.dt*lamMax, A.dt);
[~, iLim] = min([C.wnActuator, C.wnSample, C.wnAuthority, C.wnDelay]);
limNames = {'inner loop', 'sample rate', 'rudder authority', 'loop delay'};
fprintf('Heading bandwidth wn = %.3f rad/s, limited by: %s (ceiling %.3f rad/s)\n', ...
    C.wn, limNames{iLim}, C.wnMax);
rudNoise = sqrt((g0.Kp*S.sigmaPsi)^2 + (g0.Kd*sigmaRf)^2);
fprintf('Rudder noise with base gains: %.2f deg RMS (budget %.2f deg)\n', ...
    rudNoise, D.rudNoiseBudget);
rudWaveRate = g0.Kp*M.sigmaPsiWave*wp;
fprintf('Steering speed: %.1f deg/s available, ~%.1f deg/s demanded by wave chasing\n', ...
    A.wMax, rudWaveRate);
fprintf('Backlash %.2f deg = %.0f encoder counts; dead zone costs up to %.2f deg of rudder\n', ...
    A.backlash, A.backlash/A.encRes, A.uDead/A.Kp);
fprintf('Max steady turn rate at %g deg rudder: %.2f deg/s\n\n', D.dMax, rMax);
%% 7) OUTPUT  ================================================================
fprintf('--- Computed parameters ---\n');
fprintf('Ship     K = %.4f 1/s   T = %.3f s   alpha = %.5f s^2/deg^2   delta_r = %.3f deg\n', ...
    P.lo.K, P.lo.T, P.lo.alpha, P.lo.delta_r);
fprintf('Actuator wMax = %.1f deg/s   tauM = %.3f s   uDead = %.3f   backlash = %.2f deg\n', ...
    A.wMax, A.tauM, A.uDead, A.backlash);
fprintf('         encRes = %.4f deg (%s side)   inner Kp = %.4f duty/deg   dt = %.4f s\n', ...
    A.encRes, A.encoder, A.Kp, A.dt);
fprintf('Control  wn = %.3f rad/s   zeta = %.2f   Kp0 = %.3f   Ki0 = %.4f   Kd0 = %.3f\n', ...
    C.wn, C.zeta, g0.Kp, g0.Ki, g0.Kd);
fprintf('Fuzzy    Ke = %.4f   Kec = %.4f   cp = %.2f   ci = %.2f   cd = %.2f\n', ...
    F.Ke, F.Kec, F.cp, F.ci, F.cd);
fprintf('Waves    Tp = %.1f s   gamma = %.1f   sigma = %.1f deg (rudder-equivalent)\n', ...
    W.Tp, W.gamma, W.sigma);
save('vessel_params.mat', 'P', 'A', 'S', 'C', 'F', 'W', 'g0', 'M', 'D');
fprintf('\nSaved vessel_params.mat\n');

%% ========================================================================
%  LOCAL FUNCTIONS (คงเดิมตามต้นฉบับ)
%% ========================================================================
function id = identifyNomoto(psi, delta, h, method, ID)
    psi = psi(:);  delta = delta(:);
    if ID.smoothN > 1, psi = movingAverage(psi, ID.smoothN); end
    n  = numel(psi);
    t  = (3:n).';
    yv = psi(2:end) - psi(1:end-1);
    y1 = yv(t-1);
    y0 = yv(t-2);
    Y  = y1 - y0;
    X  = [h^2*delta(t-2), h^2*ones(numel(t), 1), h*y0, y0.^3/h];
    switch upper(method)
        case 'LS', Avec = X\Y;
        case 'MILS'
            p = ID.innovLen; Avec = zeros(4, 1); Pinv = 1e-6*eye(4);
            for k = p:numel(Y)
                Xm = X(k-p+1:k, :).'; Ym = Y(k-p+1:k);
                E = Ym - Xm.'*Avec; Pinv = Pinv + Xm*Xm.'; Avec = Avec + Pinv\(Xm*E);
            end
        otherwise, error('method must be LS or MILS');
    end
    id.method = upper(method); id.Avec = Avec;
    id.T = -1/Avec(3); id.K = Avec(1)*id.T;
    id.delta_r = Avec(2)/Avec(1); id.alpha = -Avec(4)*id.T;
    id.resid = std(Y - X*Avec);
end

function id = refineOutputError(id0, t, psi, delta)
    h = median(diff(t)); r0 = (psi(2) - psi(1))/h;
    rEst = diff(psi)/h; Kgss = std(rEst)/max(std(delta), eps);
    starts = {[Kgss, 2.0, 0, 0, r0], [0.3, 2.0, 0, 0, r0]};
    if id0.K > 0 && id0.T > 0, starts = [{[id0.K, id0.T, id0.alpha, id0.delta_r, r0]}, starts]; end
    opt = optimset('MaxFunEvals', 5000, 'MaxIter', 5000, 'TolX', 1e-5, 'TolFun', 1e-5);
    best = inf; q = starts{1};
    for i = 1:numel(starts)
        qi = fminsearch(@(qq) oeCost(qq, psi, delta, h), starts{i}, opt);
        ci = oeCost(qi, psi, delta, h);
        if ci < best, best = ci; q = qi; end
    end
    id = struct('method', 'OE', 'K', q(1), 'T', q(2), 'alpha', q(3), ...
                 'delta_r', q(4), 'r0', q(5), 'Avec', [], 'resid', best);
end

function c = oeCost(q, psi, delta, h)
    if q(1) <= 0.005 || q(1) > 5 || q(2) <= 0.05 || q(2) > 60 || abs(q(3)) > 1 || abs(q(4)) > 20
        c = 1e6; return;
    end
    n = numel(psi); x = [psi(1); q(5)]; ps = zeros(n, 1); ps(1) = psi(1);
    for k = 1:n-1
        x = rk4Heading(x, delta(k), q(1:4), h);
        ps(k+1) = x(1);
    end
    c = sqrt(mean((ps - psi).^2));
end

function v = validateModel(id, t, psi, delta)
    h = median(diff(t)); pv = [id.K id.T id.alpha id.delta_r];
    n = numel(psi); ps = zeros(n, 1); ps(1) = psi(1);
    r = sub_get_r(id, h, psi);
    for k = 1:n-1
        x = [ps(k); r]; x = rk4Heading(x, delta(k), pv, h);
        ps(k+1) = x(1); r = x(2);
    end
    v.psiSim = ps; v.RMSE = sqrt(mean((ps - psi).^2)); v.PCC = corrcoefPair(ps, psi);
end

function r = sub_get_r(id, h, psi)
    if isfield(id, 'r0') && ~isempty(id.r0), r = id.r0; else, r = (psi(2) - psi(1))/h; end
end

function x = rk4Heading(x, delta, pv, h)
    k1 = headingDeriv(x, delta, pv);
    k2 = headingDeriv(x + 0.5*h*k1, delta, pv);
    k3 = headingDeriv(x + 0.5*h*k2, delta, pv);
    k4 = headingDeriv(x + h*k3, delta, pv);
    x = x + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
end

function dx = headingDeriv(x, delta, pv)
    dx = [x(2); (pv(1)*(delta + pv(4)) - x(2) - pv(3)*x(2)^3)/pv(2)];
end

function g = polePlacementPID(K, T, wn, zeta)
    g.Kp = T*wn^2/K; g.Kd = (2*zeta*wn*T - 1)/K; g.Ki = wn*g.Kp/10;
end

function rss = steadyYawRate(delta, p)
    c = p.K*(delta + p.delta_r);
    if p.alpha == 0, rss = c; return; end
    rt = roots([p.alpha 0 1 -c]); rt = real(rt(abs(imag(rt)) < 1e-9)); rt = rt(sign(rt) == sign(c));
    if isempty(rt), rss = NaN; else, [~, i] = min(abs(rt)); rss = rt(i); end
end

function y = movingAverage(x, n)
    n = 2*floor(n/2) + 1; k = (n - 1)/2; y = zeros(size(x));
    for i = 1:numel(x)
        lo = max(1, i - k); hi = min(numel(x), i + k); y(i) = mean(x(lo:hi));
    end
end

function c = corrcoefPair(a, b)
    a = a(:) - mean(a); b = b(:) - mean(b);
    c = sum(a.*b)/sqrt(sum(a.^2)*sum(b.^2));
end

function [t, psi, delta] = loadZigzagLog(file)
    [~, ~, ext] = fileparts(file);
    if strcmpi(ext, '.csv')
        D = csvread(file); t = D(:, 1); psi = D(:, 2); delta = D(:, 3); return;
    end
    S = load(file);
    if isfield(S, 'zigzagData'), z = S.zigzagData; t = z.t; psi = z.psi; delta = z.delta;
    else
        f = fieldnames(S); D = S.(f{1}); t = D(:, 1); psi = D(:, 2); delta = D(:, 3);
    end
end

function v = roundToNice(x)
    e = 10^floor(log10(x)); m = x/e;
    if m >= 5, v = 5*e; elseif m >= 2, v = 2*e; else, v = 1*e; end
end