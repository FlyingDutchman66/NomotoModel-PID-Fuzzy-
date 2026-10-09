%% sim_heading_fusion.m
% -------------------------------------------------------------------------
% สนามทดลอง Complementary Filter สำหรับหาหัวเรือ
%
% สคริปต์นี้ใช้เฉพาะ MATLAB พื้นฐาน ไม่เรียก Toolbox ใด ๆ
% จึงรันได้แน่นอนและเป็นจุดอ้างอิงที่เชื่อถือได้ ก่อนจะไปใช้
% Sensor Fusion and Tracking Toolbox สร้างข้อมูลเสมือนจริงในขั้นถัดไป
%
% สิ่งที่สคริปต์นี้แสดง
%   1. ไจโรล้วน เพี้ยนสะสมจนใช้ไม่ได้ภายในไม่กี่นาที
%   2. COG จาก GPS ล้วน ไม่เพี้ยนสะสม แต่สั่นและมาช้า
%   3. สองอย่างรวมกัน ดีกว่าทั้งสองอย่างที่เอามารวม
%
% วิธีรัน   : พิมพ์ sim_heading_fusion ใน MATLAB
% เกณฑ์ผ่าน : ตารางสรุปแสดง RMS ของตัวกรองรวมต่ำกว่าของ COG ล้วน
%             และค่า bias ที่ประมาณได้เข้าใกล้ค่าจริง 0.8 องศาต่อวินาที
% -------------------------------------------------------------------------

clear; clc; close all;
rng(42);                       % ตรึงเลขสุ่มเพื่อให้ผลซ้ำได้

%% ---- พารามิเตอร์ของการจำลอง ----
dt        = 0.02;              % คาบของลูปควบคุม 50 Hz
T         = 300;               % ระยะเวลาจำลอง หน่วยวินาที
gpsRate   = 1;                 % GPS อัปเดต 1 Hz

gyroBias  = 0.8;               % ค่าเบี่ยงเบนคงที่ของไจโร [deg/s] - ตัวร้าย
gyroNoise = 0.15;              % สัญญาณรบกวนของไจโร [deg/s]
cogNoise  = 2.5;               % สัญญาณรบกวนของ COG [deg]

waveAmp   = 4.0;               % คลื่นทำให้หัวเรือส่าย [deg]
wavePer   = 6.0;               % คาบคลื่น [s]

Kp        = 0.25;              % เกนดึงกลับเข้าหา COG
Ki        = 0.012;             % เกนเรียนรู้ค่า bias ของไจโร

N        = round(T / dt);
gpsEvery = round(1 / gpsRate / dt);
t        = (0:N-1)' * dt;

%% ---- สร้างหัวเรือจริงที่เราจะพยายามประมาณให้ได้ ----
% มุ่งหน้า 90 องศา แล้วเลี้ยวไป 135 องศาที่วินาทีที่ 100 ตลอดเวลามีคลื่นกวน
baseCourse = 90 + min(45, max(0, (t - 100) * 1.5));
psiTrue    = wrap360(baseCourse + waveAmp * sin(2*pi*t / wavePer));

% อัตราการหมุนจริง หาจากผลต่างของมุมที่บีบช่วงแล้ว
rateTrue        = [0; wrap180(diff(psiTrue)) / dt];
rateTrue(1)     = rateTrue(2);

%% ---- จำลองเซ็นเซอร์ ----
gyroMeas = rateTrue + gyroBias + gyroNoise * randn(N,1);

%% ---- เดินลูปประมาณค่า ----
psiGyro  = zeros(N,1);   psiGyro(1)  = 90;   % ไจโรล้วน
psiFused = zeros(N,1);   psiFused(1) = 90;   % ตัวกรองรวม
biasHat  = zeros(N,1);                       % ค่า bias ที่ตัวกรองเดาได้
cogHold  = zeros(N,1);   cogHold(1)  = 90;   % COG ค่าล่าสุดที่ได้รับ

b   = 0;
cog = 90;

for k = 2:N
    % --- ไจโรล้วน สะสมไปเรื่อย ๆ โดยไม่มีใครแก้ ---
    psiGyro(k) = wrap360(psiGyro(k-1) + gyroMeas(k) * dt);

    % --- ตัวกรองรวม ขั้นทำนาย ใช้ไจโรที่หัก bias ที่เดาได้ออกแล้ว ---
    psi = wrap360(psiFused(k-1) + (gyroMeas(k) - b) * dt);

    % --- ขั้นแก้ไข ทำเฉพาะตอนที่ GPS ส่งค่าใหม่มา ---
    if mod(k-1, gpsEvery) == 0
        cog = wrap360(psiTrue(k) + cogNoise * randn);
        e   = wrap180(cog - psi);      % ต้องบีบช่วง ไม่งั้นพังที่ศูนย์องศา
        psi = wrap360(psi + Kp * e);   % ดึงมุมเข้าหา COG ทีละนิด
        b   = b - Ki * e;              % เรียนรู้ว่าไจโรเอียงไปทางไหน
    end

    psiFused(k) = psi;
    biasHat(k)  = b;
    cogHold(k)  = cog;
end

%% ---- สรุปผล ----
errGyro  = abs(wrap180(psiGyro  - psiTrue));
errCog   = abs(wrap180(cogHold  - psiTrue));
errFused = abs(wrap180(psiFused - psiTrue));

rmsOf = @(v) sqrt(mean(v.^2));

fprintf('ผลหลังจำลอง %g วินาที\n', T);
fprintf('%-22s %12s %12s\n', 'วิธี', 'RMS [deg]', 'สุดท้าย [deg]');
fprintf('%s\n', repmat('-', 1, 48));
fprintf('%-22s %12.3f %12.2f\n', 'ไจโรล้วน',      rmsOf(errGyro),  errGyro(end));
fprintf('%-22s %12.3f %12.2f\n', 'COG ล้วน',       rmsOf(errCog),   errCog(end));
fprintf('%-22s %12.3f %12.2f\n', 'Complementary', rmsOf(errFused), errFused(end));
fprintf('\nbias ที่ประมาณได้ %.4f deg/s   ค่าจริง %.4f deg/s\n', b, gyroBias);

if rmsOf(errFused) < rmsOf(errCog)
    fprintf('\nPASS : ตัวกรองรวมดีกว่าแหล่งข้อมูลที่ดีที่สุดที่เอามารวม\n');
else
    fprintf('\nFAIL : ตัวกรองรวมยังไม่ดีกว่า COG ล้วน ต้องจูนเกนใหม่\n');
end

%% ---- กราฟแสดงผล
figure('Name','Heading Fusion Performance','Position',[100 100 1000 850], 'Color', 'w');

% --- Subplot 1: Heading Comparison ---
subplot(3,1,1);
plot(t, psiTrue, 'k-', 'LineWidth', 1.8); hold on;
plot(t, psiGyro, 'r--', 'LineWidth', 1.0);
plot(t, psiFused, 'b-', 'LineWidth', 1.2);
ylabel('Heading (deg)'); 
grid on; grid minor;
legend({'True Trajectory', 'Gyro Only', 'Complementary Filter'}, ...
    'Location', 'eastoutside', 'Box', 'on');
title('Heading Estimation Performance: Sensor Fusion vs Raw Sensors', ...
    'FontWeight', 'bold', 'FontSize', 11);

% --- Subplot 2: Error Analysis ---
subplot(3,1,2);
plot(t, wrap180(cogHold - psiTrue), 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8); hold on;
plot(t, wrap180(psiFused - psiTrue), 'b-', 'LineWidth', 1.2);
ylabel('Error (deg)'); 
grid on; grid minor; 
ylim([-15 15]);
legend({'GPS COG Only Error', 'Fused Filter Error'}, ...
    'Location', 'eastoutside', 'Box', 'on');
title('Heading Error Comparison and Noise Reduction', ...
    'FontWeight', 'bold', 'FontSize', 11);

% --- Subplot 3: Gyro Bias Convergence ---
subplot(3,1,3);
plot(t, biasHat, 'b-', 'LineWidth', 1.5); hold on;
yline(gyroBias, 'k--', 'LineWidth', 1.5);
xlabel('Time (s)'); 
ylabel('Bias (deg/s)'); 
grid on; grid minor;
legend({'Estimated Bias', 'True Bias'}, ...
    'Location', 'eastoutside', 'Box', 'on');
title('Online Gyro Bias Estimation and Convergence', ...
    'FontWeight', 'bold', 'FontSize', 11);

%% ---- ฟังก์ชันช่วย ต้องให้ผลตรงกับ nav_math.c ทุกประการ ----
function a = wrap180(x)
    a = mod(x + 180, 360) - 180;
end

function a = wrap360(x)
    a = mod(x, 360);
end
