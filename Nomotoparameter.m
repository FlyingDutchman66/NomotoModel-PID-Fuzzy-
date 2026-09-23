%% ========================================================================
%  heading_control_comparison.m
%
%  Course-keeping autopilot for a small outboard vessel: side-by-side test of
%     Strategy A : fixed-gain PID heading loop       + fixed inner rudder loop
%     Strategy B : Fuzzy Cascade PID (fuzzy-scheduled
%                  outer PID gains)                   + the SAME inner loop
%
%  Plant: first-order non-linear Nomoto model integrated with RK4, as in
%  nomoto_zigzag_sim.m (Lan et al., J. Mar. Sci. Eng. 2023, 11, 903, Eq. 1).
%
%  SIGNAL FLOW (identical for both strategies except the outer controller)
%  -------------------------------------------------------------------------
%   psi_ref -> [OUTER heading ctrl] -delta_ref-> [INNER rudder loop] -u-> [BTS7960+DC motor]
%               PID  or  Fuzzy-PID                P on AS5600 angle          | theta_m
%                  ^                                     ^                   v
%                  |                                     '-- AS5600 --- [belt backlash]
%                  |                                                         | delta
%                  |                                                         v
%                  '--- BNO055 heading + gyro (noise, 10 Hz, delay) <--- [Nomoto ship] <-- waves,
%                                                                                        wind/current
%  FAIRNESS RULES (what makes the comparison defensible in the thesis)
%  -------------------------------------------------------------------------
%  1) Both strategies use the SAME base gains Kp0, Ki0, Kd0, designed once by
%     pole placement on the linearised Nomoto model at the design speed.
%  2) The fuzzy layer can only scale those gains inside declared limits.
%  3) Same inner loop, actuator, sensors, delay, saturation and anti-windup.
%  4) Same wave phases and sensor noise for both strategies in every run
%     (common random numbers), so differences come from the controller and
%     not from a lucky noise realisation.
%  => Any difference in the results is caused by the fuzzy scheduling alone.
%
%  UNITS: degrees, deg/s and seconds throughout (alpha is in s^2/deg^2).
%  Requires MATLAB R2016b+ (local functions in scripts). No toolboxes: the
%  fuzzy inference is written out by hand so it can be ported line by line
%  to the STM32 (or an ArduPilot Lua script).
%
%  WHAT TO REPLACE WITH YOUR OWN DATA (every value below is a placeholder)
%   * Section 1: K, T, alpha, delta_r from your zigzag trials
%   * Section 2: motor speed, time constant, dead zone, backlash (bench tests)
%   * Section 5: your IMC-PID gains and your 49-rule tables / scaling
%% ========================================================================

clear; clc; close all;

%% 1) SHIP MODEL: first-order non-linear Nomoto  -------------------------------
%     T*r_dot + r + alpha*r^3 = K*(delta + delta_r + delta_dist),  psi_dot = r
% Design-speed parameters (placeholders from Fig. 4e of the reference paper,
% a 1.2 m scaled tanker model). Replace with your own identified values.
P.lo.K       = 0.655;   % [1/s]        turning index
P.lo.T       = 2.936;   % [s]          time constant
P.lo.alpha   = 0.029;   % [s^2/deg^2]  cubic yaw damping
P.lo.delta_r = 1.249;   % [deg]        effective neutral rudder angle

% Higher-speed parameter set (used by scenario S3). The ratios come from the
% paper's appendix (light load, 20 deg zigzag, 2000 -> 3000 rpm): the mean K
% rises by x1.37 and the mean T falls by x0.64. More speed = more rudder
% authority, so a PID tuned at low speed becomes more aggressive than designed.
P.hi         = P.lo;
P.hi.K       = 1.37 * P.lo.K;
P.hi.T       = 0.64 * P.lo.T;

%% 2) STEERING ACTUATOR: BTS7960 + DC motor + belt-driven wheel  ---------------
% All angles are RUDDER-EQUIVALENT degrees (outboard steering angle): the
% belt and cable ratios are already folded in. Measure these on your rig.
A.dt       = 0.005;     % [s]     inner-loop period (200 Hz) = integration step
A.wMax     = 60;        % [deg/s] steering speed at 100 % duty
A.tauM     = 0.10;      % [s]     motor + load mechanical time constant
A.uDead    = 0.08;      % [-]     duty below which static friction stalls the motor
A.backlash = 2.0;       % [deg]   total free play between motor and outboard
A.dMaxMech = 35;        % [deg]   mechanical hard stop
A.encoder  = 'rudder';  % where the AS5600 measures the angle:
                        %  'rudder': after the belt -> backlash INSIDE the inner loop
                        %  'motor' : motor side     -> backlash OUTSIDE the inner loop,
                        %            so the heading loop sees it as a rudder dead band
A.encRes   = 0.1;       % [deg]   AS5600 resolution in rudder-equivalent degrees
                        %         (12-bit = 0.088 deg at the magnet, times the gearing)
% Inner position loop, FIXED and identical for both strategies.
% With a P controller the motor + loop behaves like a 2nd-order system:
%   wn^2 = wMax*Kp/tauM,  2*zeta*wn = 1/tauM  ->  Kp = 0.065 gives wn ~ 6.2 rad/s, zeta ~ 0.8
A.Kp       = 0.065;     % [duty/deg]
A.Ki       = 0;         % [duty/(deg*s)] keep 0 with a rudder-side encoder:
                        %  integral action + backlash inside the loop -> hunting

%% 3) SENSORS AND TIMING  ------------------------------------------------------
S.Ts       = 0.1;       % [s]     outer (heading) loop period, 10 Hz
S.delay    = 1;         % [samples] outer-loop computation/communication delay
S.sigmaPsi = 0.3;       % [deg]   heading noise (assumed, BNO055 fusion output)
S.sigmaR   = 0.2;       % [deg/s] gyro yaw-rate noise (assumed)
S.tauRf    = 0.3;       % [s]     low-pass filter on measured yaw rate (D-term input)

%% 4) ENVIRONMENT: waves + wind/current  ---------------------------------------
% Waves enter as a RUDDER-EQUIVALENT yaw disturbance delta_w(t) inside the
% Nomoto equation. Its spectral SHAPE is JONSWAP (gamma = 1 gives
% Pierson-Moskowitz). Its SIZE (sigma) is an assumption: going from wave
% height to "degrees of rudder" needs hull data you do not have yet, so state
% sigma explicitly in the thesis and, ideally, calibrate it against heading
% oscillations measured on the water. Encounter frequency is held fixed.
W.Tp     = 5.0;         % [s]     peak (encounter) period
W.gamma  = 3.3;         % [-]     JONSWAP peak enhancement (1 = Pierson-Moskowitz)
W.sigma  = 10;          % [deg]   std of the rudder-equivalent wave disturbance
W.nComp  = 60;          % [-]     number of harmonic components

%% 5) CONTROLLERS  -------------------------------------------------------------
C.dMax = 30;            % [deg]   steering command limit (inside the hard stop)

% --- Base PID gains: pole placement on the linearised Nomoto model -----------
%   plant psi/delta = K/(s(Ts+1)),  control delta = Kp*e + Ki*int(e) - Kd*r
%   closed loop:  T s^3 + (1 + K Kd) s^2 + K Kp s + K Ki = 0
%   => Kp = T wn^2/K,  Kd = (2 zeta wn T - 1)/K,  Ki = wn Kp/10
%   (Fossen, Handbook of Marine Craft Hydrodynamics and Motion Control)
% Swap in your IMC-PID gains here if you prefer; BOTH strategies use g0.
C.wn   = 0.6;           % [rad/s] well below the inner loop (~6 rad/s)
C.zeta = 0.9;
g0 = polePlacementPID(P.lo.K, P.lo.T, C.wn, C.zeta);

% --- Fuzzy gain scheduler (Strategy B) ----------------------------------------
% Inputs : e  = heading error         [deg]   -> e_n  = Ke *e,  clipped to [-3, 3]
%          ec = error rate = -r_filt  [deg/s] -> ec_n = Kec*ec, clipped to [-3, 3]
%          (with a piecewise-constant setpoint de/dt = -r; using the filtered
%           gyro rate avoids differentiating a noisy heading)
% Sets   : 7 triangular sets NB NM NS ZO PS PM PB on inputs and outputs
% Engine : Mamdani, min for AND/implication, max aggregation, centroid
% Output : u_p, u_i, u_d in [-3, 3] scale the base gains
%          Kp = Kp0*(1 + cp*u_p/3),  Ki = Ki0*(1 + ci*u_i/3),  Kd = Kd0*(1 + cd*u_d/3)
%          (the centroid never reaches +/-3 exactly, max ~ +/-2.67, so the real
%           limits are ~89 % of cp, ci, cd)
F.Ke  = 3/30;           % |e|  = 30 deg  reaches the edge of the universe
F.Kec = 3/6;            % |ec| = 6 deg/s reaches the edge (~ max steady turn rate)
F.cp  = 0.5;            % Kp may change by up to ~ +/-45 %
F.ci  = 1.0;            % Ki may change by up to ~ +/-89 %
F.cd  = 0.5;            % Kd may change by up to ~ +/-45 %

% --- Rule tables: rows = e (NB..PB), columns = ec (NB..PB) ----------------------
% >>> PASTE YOUR OWN 49-RULE TABLES HERE, SAME LAYOUT. <<<
% Defaults are written from three heuristics. "Approaching" means e and ec
% have opposite signs, i.e. the error is shrinking.
%   Kp: large |e| -> raise Kp for a fast turn; approaching fast -> lower Kp
%       to limit overshoot; on course but yawing fast (waves) -> lower Kp
%   Ki: large |e| -> cut Ki (stops windup-driven overshoot); small |e| and
%       slow yaw -> raise Ki to remove wind/current bias quickly
%   Kd: approaching fast at small/medium |e| -> raise Kd to brake before the
%       target; large |e|, or on course with fast wave yaw -> lower Kd
% The tables are POINT-SYMMETRIC, rule(e, ec) = rule(-e, -ec), so turns to
% port and to starboard are treated identically. The script warns if a pasted
% table breaks this symmetry.
NB = -3; NM = -2; NS = -1; ZO = 0; PS = 1; PM = 2; PB = 3;
%             ec: NB  NM  NS  ZO  PS  PM  PB
F.rulesKp = [     PB  PB  PB  PB  PM  PS  ZO     % e = NB
                  PM  PM  PM  PM  PS  ZO  NS     % e = NM
                  PS  PS  PS  PS  ZO  NS  NM     % e = NS
                  NS  NS  ZO  ZO  ZO  NS  NS     % e = ZO
                  NM  NS  ZO  PS  PS  PS  PS     % e = PS
                  NS  ZO  PS  PM  PM  PM  PM     % e = PM
                  ZO  PS  PM  PB  PB  PB  PB ];  % e = PB

F.rulesKi = [     NB  NB  NB  NB  NB  NB  NB     % e = NB
                  NM  NM  NM  NM  NM  NM  NM     % e = NM
                  PS  PS  PS  PS  ZO  ZO  ZO     % e = NS
                  ZO  ZO  PS  PM  PS  ZO  ZO     % e = ZO
                  ZO  ZO  ZO  PS  PS  PS  PS     % e = PS
                  NM  NM  NM  NM  NM  NM  NM     % e = PM
                  NB  NB  NB  NB  NB  NB  NB ];  % e = PB

F.rulesKd = [     NM  NM  NM  NM  NM  NS  NS     % e = NB
                  NS  NS  NS  NS  ZO  PS  PM     % e = NM
                  ZO  ZO  ZO  ZO  PS  PM  PB     % e = NS
                  NS  NS  ZO  ZO  ZO  NS  NS     % e = ZO
                  PB  PM  PS  ZO  ZO  ZO  ZO     % e = PS
                  PM  PS  ZO  NS  NS  NS  NS     % e = PM
                  NS  NS  NM  NM  NM  NM  NM ];  % e = PB

% Output universe discretisation for the centroid
F.y     = -3:0.05:3;
F.outMF = max(0, 1 - abs(F.y - (-3:3).'));     % 7 x numel(y), NB..PB

checkRuleTable(F.rulesKp, 'Kp');
checkRuleTable(F.rulesKi, 'Ki');
checkRuleTable(F.rulesKd, 'Kd');

fprintf('Base PID gains (pole placement, wn = %.2f rad/s, zeta = %.2f):\n', C.wn, C.zeta);
fprintf('   Kp0 = %.3f deg/deg   Ki0 = %.4f 1/s   Kd0 = %.3f s\n\n', g0.Kp, g0.Ki, g0.Kd);

%% 6) TEST SCENARIOS  ----------------------------------------------------------
% One factor per scenario, so each result can be attributed to one cause:
%   S1 tracking only, S2 disturbance rejection only, S3 plant variation only.
%  ref   : [time heading] breakpoints, held until the next row           [s deg]
%  bias  : [time value]   wind/current yaw bias, rudder-equivalent       [s deg]
%  speed : [time s]       0 = design speed (P.lo), 1 = P.hi, linear between
%  band  : settling band for the step metrics                            [deg]
scn(1).name   = 'S1 Course changes, calm water';
scn(1).tEnd   = 185;
scn(1).ref    = [0 0; 5 30; 65 0; 125 -30];
scn(1).waves  = false;
scn(1).bias   = [0 0];
scn(1).speed  = [0 0];
scn(1).band   = 2;
scn(1).nSeeds = 1;

scn(2).name   = 'S2 Course keeping in waves + wind/current step';
scn(2).tEnd   = 180;
scn(2).ref    = [0 0];
scn(2).waves  = true;
scn(2).bias   = [0 0; 90 3];
scn(2).speed  = [0 0];
scn(2).band   = 3;
scn(2).nSeeds = 5;

scn(3).name   = 'S3 Speed increase (design -> high speed), calm water';
scn(3).tEnd   = 180;
scn(3).ref    = [0 0; 10 30; 110 0];   % step 1 at design speed, step 2 at high speed
scn(3).waves  = false;
scn(3).bias   = [0 0];
scn(3).speed  = [0 0; 50 0; 60 1];
scn(3).band   = 2;
scn(3).nSeeds = 3;

ctrlNames = {'PID', 'Fuzzy-PID'};

%% 7) RUN ALL SCENARIOS  -------------------------------------------------------
res = cell(numel(scn), 1);
tic;
for iS = 1:numel(scn)
    res{iS}.metrics = cell(scn(iS).nSeeds, 2);
    res{iS}.log     = cell(1, 2);
    for seed = 1:scn(iS).nSeeds
        env = buildEnvironment(scn(iS), W, S, A, P, seed);   % shared by both strategies
        for iC = 1:2
            L = simulateRun(iC, env, A, S, C, F, g0);
            res{iS}.metrics{seed, iC} = runMetrics(L, scn(iS));
            if seed == 1, res{iS}.log{iC} = L; end
        end
    end
    fprintf('%-48s done (%d seed(s), %.1f s elapsed)\n', scn(iS).name, scn(iS).nSeeds, toc);
end

%% 8) RESULTS TABLE  -----------------------------------------------------------
metricList = { 'IAE',       'IAE of heading error',   'deg*s'
               'RMSe',      'RMS heading error',      'deg'
               'rudRMS',    'RMS rudder angle',       'deg'
               'rudRate',   'Mean rudder rate',       'deg/s'
               'revPerMin', 'Rudder reversals',       '1/min'
               'effort',    'Motor effort int(u^2)',  'duty^2*s' };

for iS = 1:numel(scn)
    fprintf('\n%s  (%d seed(s), mean +/- std)\n', scn(iS).name, scn(iS).nSeeds);
    fprintf('%-34s %19s %19s %9s\n', 'Metric', ctrlNames{1}, ctrlNames{2}, 'Change');
    for q = 1:size(metricList, 1)
        v = metricValues(res{iS}.metrics, metricList{q, 1});    % nSeeds x 2
        m = mean(v, 1);  s = std(v, 0, 1);
        fprintf('%-34s %9.3f +/- %-7.3f %9.3f +/- %-7.3f %+8.1f%%\n', ...
            sprintf('%s [%s]', metricList{q, 2}, metricList{q, 3}), ...
            m(1), s(1), m(2), s(2), 100*(m(2) - m(1))/m(1));
    end
    nSteps = size(scn(iS).ref, 1) - 1;
    for i = 1:nSteps
        dPsi = scn(iS).ref(i+1, 2) - scn(iS).ref(i, 2);
        os = zeros(1, 2);  ts = zeros(1, 2);
        for iC = 1:2
            os(iC) = mean(cellfun(@(M) M.stepOS(i), res{iS}.metrics(:, iC)));
            ts(iC) = mean(cellfun(@(M) M.stepTs(i), res{iS}.metrics(:, iC)));
        end
        fprintf('  Step %d (%+4.0f deg at t = %3.0f s): overshoot %5.1f%% vs %5.1f%%, settling (+/-%g deg) %5.1f s vs %5.1f s\n', ...
            i, dPsi, scn(iS).ref(i+1, 1), os(1), os(2), scn(iS).band, ts(1), ts(2));
    end
end
fprintf('\nChange = (Fuzzy-PID - PID)/PID. Negative is better for every metric listed.\n');
fprintf('Settling time NaN = never stayed inside the band before the next step.\n');

%% 9) PLOTS  ------------------------------------------------------------------
col = {[0.00 0.35 0.80], [0.85 0.20 0.10]};

% --- S1: course changes in calm water ------------------------------------------
L1 = res{1}.log;
figure('Name', 'S1 course changes', 'Color', 'w');
subplot(3, 1, 1); hold on; grid on; box on;
stairs(L1{1}.tO, L1{1}.ref, 'k--', 'LineWidth', 1.0);
for iC = 1:2, plot(L1{iC}.t, L1{iC}.psi, 'Color', col{iC}, 'LineWidth', 1.3); end
ylabel('\psi [deg]'); title(scn(1).name);
legend({'reference', ctrlNames{:}}, 'Location', 'eastoutside');
subplot(3, 1, 2); hold on; grid on; box on;
for iC = 1:2, plot(L1{iC}.t, L1{iC}.delta, 'Color', col{iC}, 'LineWidth', 1.1); end
ylabel('\delta [deg]'); title('Actual steering angle');
legend(ctrlNames, 'Location', 'eastoutside');
subplot(3, 1, 3); hold on; grid on; box on;
plot(L1{2}.tO, L1{2}.gain, 'LineWidth', 1.1);
ylabel('gain / base gain'); xlabel('Time [s]'); title('Fuzzy-scheduled gains (Strategy B)');
legend({'K_p/K_{p0}', 'K_i/K_{i0}', 'K_d/K_{d0}'}, 'Location', 'eastoutside');

% --- S2: course keeping in waves ------------------------------------------------
L2 = res{2}.log;
figure('Name', 'S2 course keeping in waves', 'Color', 'w');
subplot(3, 1, 1); hold on; grid on; box on;
for iC = 1:2, plot(L2{iC}.tO, L2{iC}.e, 'Color', col{iC}, 'LineWidth', 1.1); end
ylabel('e [deg]'); title([scn(2).name ' (seed 1)']);
legend(ctrlNames, 'Location', 'eastoutside');
subplot(3, 1, 2); hold on; grid on; box on;
for iC = 1:2, plot(L2{iC}.t, L2{iC}.delta, 'Color', col{iC}, 'LineWidth', 1.0); end
ylabel('\delta [deg]'); title('Actual steering angle');
legend(ctrlNames, 'Location', 'eastoutside');
subplot(3, 1, 3); hold on; grid on; box on;
plot(L2{1}.t, L2{1}.dist, 'Color', [0.4 0.4 0.4]);
ylabel('\delta_{dist} [deg]'); xlabel('Time [s]');
title('Rudder-equivalent disturbance (waves + wind/current)');

% --- S3: speed change -------------------------------------------------------------
L3 = res{3}.log;
figure('Name', 'S3 speed change', 'Color', 'w');
subplot(3, 1, 1); hold on; grid on; box on;
stairs(L3{1}.tO, L3{1}.ref, 'k--', 'LineWidth', 1.0);
for iC = 1:2, plot(L3{iC}.t, L3{iC}.psi, 'Color', col{iC}, 'LineWidth', 1.2); end
ylabel('\psi [deg]'); title([scn(3).name ' (seed 1)']);
legend({'reference', ctrlNames{:}}, 'Location', 'eastoutside');
subplot(3, 1, 2); hold on; grid on; box on;
for iC = 1:2, plot(L3{iC}.t, L3{iC}.delta, 'Color', col{iC}, 'LineWidth', 1.0); end
ylabel('\delta [deg]'); title('Actual steering angle');
legend(ctrlNames, 'Location', 'eastoutside');
subplot(3, 1, 3); hold on; grid on; box on;
plot(L3{1}.t, L3{1}.KT(:, 1)/P.lo.K, 'LineWidth', 1.2);
plot(L3{1}.t, L3{1}.KT(:, 2)/P.lo.T, 'LineWidth', 1.2);
ylabel('ratio to design value'); xlabel('Time [s]'); title('Plant parameters');
legend({'K / K_{design}', 'T / T_{design}'}, 'Location', 'eastoutside');

% --- Fuzzy control surfaces ----------------------------------------------------
[EN, ECN] = meshgrid(-3:0.15:3);
Z = zeros([size(EN) 3]);
for a = 1:numel(EN)
    [ia, ja] = ind2sub(size(EN), a);
    Z(ia, ja, :) = reshape(fuzzyGainAdjust(EN(a)/F.Ke, ECN(a)/F.Kec, F), 1, 1, 3);
end
surfTitles = {'\Delta K_p output u_p', '\Delta K_i output u_i', '\Delta K_d output u_d'};
figure('Name', 'Fuzzy surfaces', 'Color', 'w', 'Position', [100 100 1100 380]);
for q = 1:3
    subplot(1, 3, q);
    surf(EN, ECN, Z(:, :, q), 'EdgeAlpha', 0.3);
    xlabel('e_n'); ylabel('ec_n'); zlabel('u'); title(surfTitles{q});
    view(-35, 30); grid on;
end

% --- Summary bar chart ------------------------------------------------------------
keyM   = {'IAE', 'rudRate', 'revPerMin'};
keyLab = {'IAE [deg s]', 'Mean rudder rate [deg/s]', 'Rudder reversals [1/min]'};
figure('Name', 'Summary', 'Color', 'w', 'Position', [100 100 1100 380]);
for q = 1:numel(keyM)
    Y = zeros(numel(scn), 2);
    for iS = 1:numel(scn)
        Y(iS, :) = mean(metricValues(res{iS}.metrics, keyM{q}), 1);
    end
    subplot(1, numel(keyM), q);
    bar(Y); grid on;
    set(gca, 'XTickLabel', {'S1', 'S2', 'S3'});
    title(keyLab{q});
    if q == 1, legend(ctrlNames, 'Location', 'northwest'); end
end

%% 10) SAVE  -------------------------------------------------------------------
save('controller_comparison_results.mat', 'res', 'scn', 'P', 'A', 'S', 'C', 'F', 'W', 'g0');
fprintf('Saved controller_comparison_results.mat\n');

%% ========================================================================
%  LOCAL FUNCTIONS
%% ========================================================================

function L = simulateRun(iC, env, A, S, C, F, g0)
% One closed-loop run.  iC = 1: fixed-gain PID,  iC = 2: fuzzy-scheduled PID.
    dt    = A.dt;
    nOut  = round(S.Ts/dt);              % inner steps per outer update
    N     = numel(env.t);
    NO    = numel(env.tO);
    encRudder = strcmp(A.encoder, 'rudder');
    thMax = A.dMaxMech + A.backlash/2;   % motor-side travel limit

    % --- Initial condition: steady on psi_ref(0), rudder holding course ---------
    % delta = -delta_r cancels the neutral-rudder bias, so t = 0 is an
    % equilibrium. The integrator starts at the same value (bumpless engage,
    % as if the autopilot were switched on after steady manual steering).
    dr0   = env.PV(1, 4);
    x     = [env.refO(1); 0; -dr0; 0];   % [psi; r; theta_m; omega_m]
    delta = -dr0;                        % rudder angle after the backlash
    ctl.I = -dr0;                        % outer integrator state [deg]
    rf    = 0;                           % filtered yaw rate [deg/s]
    queue = repmat(-dr0, 1, S.delay + 1);% outer-loop delay line
    dRef  = -dr0;                        % rudder reference for the inner loop
    uInI  = 0;                           % inner integrator state
    aRf   = S.Ts/(S.tauRf + S.Ts);       % yaw-rate low-pass coefficient

    % --- Logs --------------------------------------------------------------------
    L.t = env.t;   L.tO = env.tO;   L.ref = env.refO;
    L.psi = zeros(N, 1);  L.r = zeros(N, 1);  L.delta = zeros(N, 1);  L.u = zeros(N, 1);
    L.e = zeros(NO, 1);   L.dRef = zeros(NO, 1);  L.gain = ones(NO, 3);
    L.dist = env.dist;    L.KT = env.PV(:, 1:2);

    j = 0;
    for k = 1:N
        % ===== OUTER LOOP: heading controller, every S.Ts =======================
        if mod(k - 1, nOut) == 0
            j    = j + 1;
            psiM = x(1) + env.nPsi(j);                  % BNO055 heading
            rM   = x(2) + env.nR(j);                    % gyro yaw rate
            rf   = rf + aRf*(rM - rf);                  % filtered yaw rate
            e    = wrap180(env.refO(j) - psiM);         % measured heading error

            g = g0;                                     % Strategy A: base gains
            if iC == 2                                  % Strategy B: fuzzy scheduling
                du   = fuzzyGainAdjust(e, -rf, F);      % ec = de/dt = -r
                g.Kp = g0.Kp*(1 + F.cp*du(1)/3);
                g.Ki = g0.Ki*(1 + F.ci*du(2)/3);
                g.Kd = g0.Kd*(1 + F.cd*du(3)/3);
            end
            [dNew, ctl] = headingPID(e, rf, g, ctl, C.dMax, S.Ts);

            queue = [queue(2:end) dNew];                % S.delay-sample delay
            dRef  = queue(1);

            L.e(j)      = wrap180(env.refO(j) - x(1));  % TRUE error, for metrics
            L.dRef(j)   = dRef;
            L.gain(j,:) = [g.Kp/g0.Kp, g.Ki/g0.Ki, g.Kd/g0.Kd];
        end

        % ===== INNER LOOP: rudder position, every dt ===============================
        if encRudder, pos = delta; else, pos = x(3); end
        posM = A.encRes*round(pos/A.encRes);            % AS5600 quantisation
        eIn  = dRef - posM;
        uIn  = A.Kp*eIn + uInI;
        u    = min(max(uIn, -1), 1);                    % BTS7960 duty limit
        if A.Ki > 0 && u == uIn                         % inner anti-windup
            uInI = uInI + A.Ki*eIn*dt;
        end
        uEff = deadzone(u, A.uDead);                    % static friction

        L.psi(k) = x(1);  L.r(k) = x(2);  L.delta(k) = delta;  L.u(k) = u;

        % ===== PLANT: integrate ship + motor over one step (RK4) ====================
        if k < N
            x = rk4Plant(x, delta + env.dist(k), env.PV(k, :), uEff, A, dt);
            if abs(x(3)) > thMax                        % motor-side end stop
                x(3) = sign(x(3))*thMax;  x(4) = 0;
            end
            delta = backlash(x(3), delta, A.backlash);  % belt/cable free play
            delta = min(max(delta, -A.dMaxMech), A.dMaxMech);
        end
    end
end

function [dCmd, ctl] = headingPID(e, rf, g, ctl, dMax, Ts)
% Outer heading PID used by BOTH strategies (only the gains differ).
%  * D acts on the measured (filtered) yaw rate, not on the error, so a
%    setpoint step does not produce a derivative kick.
%  * The integrator accumulates Ki*e rather than e, so changing Ki on the fly
%    (fuzzy scheduling) does not make the output jump (bumpless).
%  * Conditional integration: stop integrating while the output is saturated
%    in the same direction as the error (anti-windup).
    uUnsat = g.Kp*e + ctl.I - g.Kd*rf;
    dCmd   = min(max(uUnsat, -dMax), dMax);
    if dCmd == uUnsat || sign(e) ~= sign(uUnsat)
        ctl.I = min(max(ctl.I + g.Ki*e*Ts, -dMax), dMax);
    end
end

function du = fuzzyGainAdjust(e, ec, F)
% Mamdani fuzzy inference: (e, ec) -> [u_p u_i u_d], each in [-3, 3].
% Only the (at most 4) rules with non-zero firing strength are evaluated.
    en  = min(max(F.Ke*e,  -3), 3);
    ecn = min(max(F.Kec*ec, -3), 3);
    me  = triMF7(en);
    mec = triMF7(ecn);
    ie  = find(me  > 0);
    iec = find(mec > 0);
    R   = {F.rulesKp, F.rulesKi, F.rulesKd};
    du  = zeros(1, 3);
    for q = 1:3
        agg = zeros(size(F.y));
        for i = ie
            for jj = iec
                w   = min(me(i), mec(jj));                          % AND = min
                agg = max(agg, min(w, F.outMF(R{q}(i, jj) + 4, :)));% implication min, aggregation max
            end
        end
        du(q) = sum(F.y .* agg) / sum(agg);                         % centroid
    end
end

function mu = triMF7(x)
% Membership of x in the 7 triangular sets NB..PB centred at -3..3 (width 2).
    mu = max(0, 1 - abs(x - (-3:3)));
end

function g = polePlacementPID(K, T, wn, zeta)
% PID pole placement for the linear first-order Nomoto model.
    g.Kp = T*wn^2/K;
    g.Kd = (2*zeta*wn*T - 1)/K;
    g.Ki = wn*g.Kp/10;
    assert(g.Kd > 0, 'Kd <= 0: raise wn or zeta (need 2*zeta*wn*T > 1).');
end

function x = rk4Plant(x, dTot, pv, uEff, A, h)
% Classical RK4 step. Inputs are held constant over the step (zero-order hold).
    k1 = plantDeriv(x,            dTot, pv, uEff, A);
    k2 = plantDeriv(x + 0.5*h*k1, dTot, pv, uEff, A);
    k3 = plantDeriv(x + 0.5*h*k2, dTot, pv, uEff, A);
    k4 = plantDeriv(x +     h*k3, dTot, pv, uEff, A);
    x  = x + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
end

function dx = plantDeriv(x, dTot, pv, uEff, A)
% x = [psi; r; theta_m; omega_m],  pv = [K T alpha delta_r]
% dTot = actual rudder + rudder-equivalent disturbance.
    dx = [ x(2)
           nomotoYawAccel(x(2), dTot, pv)
           x(4)
           (A.wMax*uEff - x(4))/A.tauM ];      % 1st-order DC motor speed response
end

function rdot = nomotoYawAccel(r, dTot, pv)
% First-order non-linear Nomoto model: T r_dot = K (delta + delta_r) - r - alpha r^3
    rdot = (pv(1)*(dTot + pv(4)) - r - pv(3)*r^3) / pv(2);
end

function d = backlash(theta, d, b)
% Play (hysteresis) model: the rudder moves only once the motor side has
% taken up the free play b; inside the gap the rudder stays where it is.
    if theta - d > b/2
        d = theta - b/2;
    elseif theta - d < -b/2
        d = theta + b/2;
    end
end

function y = deadzone(u, d)
% Motor dead zone: duty below d produces no motion; above it, speed rises
% linearly to full speed at |u| = 1.
    y = sign(u)*max(abs(u) - d, 0)/(1 - d);
end

function a = wrap180(a)
% Wrap an angle difference to [-180, 180) deg. Essential for the heading
% error on the real boat (e.g. ref 350 deg, heading 10 deg -> e = -20 deg).
    a = mod(a + 180, 360) - 180;
end

function env = buildEnvironment(sc, W, S, A, P, seed)
% Everything random or time-varying for one run, generated ONCE per seed and
% shared by both strategies (common random numbers).
    rng(seed);
    env.t    = (0:A.dt:sc.tEnd).';
    nOut     = round(S.Ts/A.dt);
    env.tO   = env.t(1:nOut:end);
    env.refO = stepHold(sc.ref, env.tO);
    NO       = numel(env.tO);
    if sc.waves
        dw = waveDisturbance(env.t, W);
    else
        dw = zeros(size(env.t));
    end
    env.nPsi = S.sigmaPsi*randn(NO, 1);
    env.nR   = S.sigmaR  *randn(NO, 1);
    env.dist = dw + stepHold(sc.bias, env.t);
    s        = holdLinear(sc.speed, env.t);             % 0 = P.lo, 1 = P.hi
    pLo = [P.lo.K P.lo.T P.lo.alpha P.lo.delta_r];
    pHi = [P.hi.K P.hi.T P.hi.alpha P.hi.delta_r];
    env.PV   = (1 - s)*pLo + s*pHi;                     % N x 4, [K T alpha delta_r]
end

function dw = waveDisturbance(t, W)
% Rudder-equivalent wave disturbance: sum of harmonics whose amplitudes
% follow a JONSWAP spectrum shape, scaled to standard deviation W.sigma.
% Frequencies are jittered inside their bins so the signal does not repeat.
    wp  = 2*pi/W.Tp;
    wLo = 0.4*wp;  wHi = 3.0*wp;
    dW  = (wHi - wLo)/W.nComp;
    w   = wLo + ((1:W.nComp).' - 0.5)*dW + (rand(W.nComp, 1) - 0.5)*dW;
    sig = 0.07*ones(size(w));  sig(w > wp) = 0.09;
    Sw  = w.^-5 .* exp(-1.25*(wp./w).^4) .* W.gamma.^exp(-(w - wp).^2 ./ (2*sig.^2*wp^2));
    amp = sqrt(2*Sw*dW);
    amp = amp*W.sigma/sqrt(sum(amp.^2)/2);              % variance = sum(amp^2)/2
    ph  = 2*pi*rand(W.nComp, 1);
    dw  = zeros(size(t));
    for i = 1:W.nComp
        dw = dw + amp(i)*cos(w(i)*t + ph(i));
    end
end

function y = stepHold(tab, t)
% Piecewise-constant signal from [time value] breakpoints.
    y = zeros(size(t));
    for i = 1:size(tab, 1)
        y(t >= tab(i, 1)) = tab(i, 2);
    end
end

function y = holdLinear(tab, t)
% Piecewise-linear signal from [time value] breakpoints, held flat outside.
    if size(tab, 1) == 1
        y = tab(1, 2)*ones(size(t));
        return;
    end
    y = interp1(tab(:, 1), tab(:, 2), min(max(t, tab(1, 1)), tab(end, 1)));
end

function M = runMetrics(L, sc)
% Performance and actuator-wear metrics for one run (true heading, not measured).
    TsO = L.tO(2) - L.tO(1);
    dt  = L.t(2) - L.t(1);
    Tt  = L.t(end);
    M.IAE       = sum(abs(L.e))*TsO;                    % integral of |e|
    M.RMSe      = sqrt(mean(L.e.^2));
    M.rudRMS    = sqrt(mean(L.delta.^2));
    M.rudRate   = sum(abs(diff(L.delta)))/Tt;           % mean steering speed = wear proxy
    M.revPerMin = countReversals(L.delta, 0.5)/(Tt/60); % direction changes > 0.5 deg
    M.effort    = sum(L.u.^2)*dt;                       % motor effort proxy
    % Step-response metrics for every setpoint change
    nSteps   = size(sc.ref, 1) - 1;
    M.stepOS = nan(1, nSteps);
    M.stepTs = nan(1, nSteps);
    for i = 1:nSteps
        t0   = sc.ref(i+1, 1);
        tgt  = sc.ref(i+1, 2);
        dPsi = tgt - sc.ref(i, 2);
        if i < nSteps, t1 = sc.ref(i+2, 1); else, t1 = L.tO(end) + TsO; end
        idx  = find(L.tO >= t0 & L.tO < t1);
        psiW = L.ref(idx) - L.e(idx);                   % true heading in the window
        M.stepOS(i) = max(0, max(sign(dPsi)*(psiW - tgt)))/abs(dPsi)*100;
        out = find(abs(psiW - tgt) > sc.band, 1, 'last');
        if isempty(out)
            M.stepTs(i) = 0;
        elseif out == numel(idx)
            M.stepTs(i) = NaN;                          % never settled in the window
        else
            M.stepTs(i) = L.tO(idx(out + 1)) - t0;
        end
    end
end

function n = countReversals(x, thr)
% Number of direction changes of x, ignoring moves smaller than thr
% (hysteresis), i.e. how often the belt and motor have to reverse.
    n = 0;  dirn = 0;  hi = x(1);  lo = x(1);
    for k = 2:numel(x)
        v = x(k);
        if dirn == 0
            if v > x(1) + thr, dirn = 1;  hi = v;
            elseif v < x(1) - thr, dirn = -1;  lo = v; end
        elseif dirn == 1
            if v > hi, hi = v;
            elseif v < hi - thr, n = n + 1;  dirn = -1;  lo = v; end
        else
            if v < lo, lo = v;
            elseif v > lo + thr, n = n + 1;  dirn = 1;  hi = v; end
        end
    end
end

function v = metricValues(Mcell, name)
% nSeeds x 2 matrix of one metric from the cell array of metric structs.
    v = cellfun(@(M) M.(name), Mcell);
end

function checkRuleTable(R, name)
% Sanity checks for a pasted 7x7 rule table.
    assert(isequal(size(R), [7 7]), '%s rule table must be 7x7.', name);
    assert(all(ismember(R(:), -3:3)), '%s rule table entries must be NB..PB (-3..3).', name);
    if ~isequal(R, rot90(R, 2))
        warning(['%s rule table is not point-symmetric (rule(e,ec) ~= rule(-e,-ec)): ' ...
            'turns to port and starboard will get different gains.'], name);
    end
end