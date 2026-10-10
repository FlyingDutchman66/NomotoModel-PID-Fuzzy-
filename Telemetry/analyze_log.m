function analyze_log(filename)
% ANALYZE_LOG  วิเคราะห์ไฟล์ล็อกของระบบออโตไพลอต แยกผลตามโหมดการทำงาน
%
%   analyze_log('LOG_0011.CSV')
%
% สิ่งที่สคริปต์นี้ตอบ
%   1. ออโตไพลอตกดความคลาดของหัวเรือลงได้เท่าไร เทียบกับการขับด้วยมือ
%   2. พลังงานต่อหนึ่งกิโลเมตรของระยะคืบหน้าตามเส้นทาง แยกขับเคลื่อนกับหางเสือ
%   3. ตัวเลขนั้นนิ่งพอจะอ้างได้หรือไม่ และข้อมูลที่หายไปกระทบหรือไม่
%
% ข้อสำคัญสองข้อเรื่องการอ่านผล
%
%   ข้อแรก ตัวเลขพลังงานรวมบนเรือจำลองมีอคติเข้าข้างการขับด้วยมือ เพราะมอเตอร์
%   หางเสือของเรือจำลองกินไฟเป็นสัดส่วนที่สูงผิดปกติเทียบกับมอเตอร์ขับเคลื่อน
%   บนเรือประมงจริง เครื่องยนต์มีกำลังระดับกิโลวัตต์ ส่วนมอเตอร์หางเสือ
%   อยู่ระดับสิบวัตต์ จึงต้องรายงานสองตัวเลขคู่กันเสมอ คือพลังงานรวม
%   ซึ่งเป็นความจริงของเรือจำลอง และพลังงานขับเคลื่อนเพียงอย่างเดียว
%   ซึ่งเป็นตัวแทนที่ใกล้เคียงกับเรือขนาดจริง
%
%   ข้อสอง ระวังการรายงาน XTE เป็นหลักฐานชิ้นที่สอง สคริปต์จะตรวจให้เองว่า
%   ในไฟล์ที่กำลังอ่านอยู่ XTE เป็นสัดส่วนตายตัวกับความคลาดของหัวเรือหรือไม่
%   ถ้าใช่ แปลว่าสองค่านี้คือตัวเลขเดียวกันในสองหน่วย ไม่ใช่หลักฐานอิสระ
%
% ตรวจสอบความถูกต้องแล้วด้วยการคำนวณเส้นทางที่สองในภาษา Python
% และเทียบกับค่าสะสมที่เฟิร์มแวร์บันทึกเอง ทั้งสามทางให้ผลตรงกัน

if nargin < 1
    filename = 'LOG_0011.CSV';
end

V_BUS_FALLBACK = 12.5;   % ใช้เมื่อคอลัมน์แรงดันว่าง

%% ---- อ่านไฟล์ โดยข้ามบรรทัดที่ขึ้นต้นด้วย # ----
opts = detectImportOptions(filename, 'CommentStyle', '#');
opts = setvartype(opts, 'double');
T = readtable(filename, opts);

fprintf('ไฟล์ %s : %d แถว %d คอลัมน์\n', filename, height(T), width(T));

%% ---- แยกตามโหมด ----
MODE_MANUAL = 1;
MODE_AUTO   = 2;
names = {'ขับด้วยมือ', 'ออโตไพลอต'};
modes = [MODE_MANUAL, MODE_AUTO];

stats = struct([]);
for i = 1:numel(modes)
    S = summarise(T(T.mode == modes(i), :), V_BUS_FALLBACK);
    S.name = names{i};
    if isempty(stats)
        stats = S;
    else
        stats(i) = S;  %#ok<AGROW>
    end
end

%% ---- ความสมบูรณ์ของข้อมูล ----
dt_ms   = diff(T.t_ms);
nominal = median(dt_ms);
missing = sum(dt_ms > nominal * 1.5);
drops   = T.drops(end) - T.drops(1);

fprintf('\n--- ความสมบูรณ์ของข้อมูล ---\n');
fprintf('คาบปกติ %.0f ms\n', nominal);
fprintf('ช่องว่างในลำดับเวลา %d จุด\n', missing);
fprintf('เรคคอร์ดที่ระบบรายงานว่าทิ้งไป %d ก้อน\n', drops);

if drops > 0
    % ข้อมูลที่หายไปทำให้พลังงานถูกนับน้อยกว่าจริงก็ต่อเมื่อเราอินทิเกรตกระแสเอง
    % แต่สคริปต์นี้ใช้ผลต่างของค่าสะสมที่เฟิร์มแวร์บวกไว้บนเรือตั้งแต่ก่อนข้อมูลหาย
    % จึงไม่ได้รับผลกระทบ ตัวตรวจทานด้านล่างเป็นคนยืนยันข้อนี้ ไม่ใช่การสันนิษฐาน
    worst = 0;
    for i = 1:numel(stats)
        worst = max([worst, stats(i).e_prop_gap, stats(i).e_steer_gap]);
    end
    if worst < 0.02
        fprintf(['ข้อมูลหายแต่ตัวเลขยังเชื่อถือได้  ค่าสะสมกับการอินทิเกรตซ้ำ\n', ...
                 '         ต่างกันไม่เกิน %.2f %% ซึ่งแปลว่าคอลัมน์สะสมเชื่อมช่องว่างให้แล้ว\n'], ...
                worst * 100);
    else
        fprintf(['คำเตือน  ค่าสะสมกับการอินทิเกรตซ้ำต่างกันถึง %.1f %%\n', ...
                 '         ตัวเลขพลังงานในตารางยังอ้างอิงไม่ได้จนกว่าจะหาสาเหตุได้\n'], ...
                worst * 100);
    end
end

%% ---- ตารางการรักษาทิศทาง ----
fprintf('\n--- การรักษาทิศทาง ---\n');
fprintf('%-12s %8s %8s %8s %8s\n', 'โหมด', 'RMS', 'p95', 'max', 'XTE RMS');
for i = 1:numel(stats)
    fprintf('%-12s %8.2f %8.2f %8.2f %8.2f\n', stats(i).name, ...
            stats(i).rms_err, stats(i).p95_err, stats(i).max_err, stats(i).rms_xte);
end

% ตรวจว่า XTE เป็นหลักฐานอิสระจริงหรือเป็นแค่ความคลาดของหัวเรือที่ถูกคูณค่าคงที่
for i = 1:numel(stats)
    if stats(i).xte_is_proportional
        fprintf(['หมายเหตุ  ใน %s ค่า XTE เป็นสัดส่วนคงที่ %.2f เท่าของความคลาดหัวเรือ\n', ...
                 '          จึงไม่ใช่หลักฐานอิสระ ห้ามรายงานเป็นผลชิ้นที่สองในเล่ม\n'], ...
                stats(i).name, abs(stats(i).xte_ratio));
    end
end

%% ---- ตารางพลังงาน ----
fprintf('\n--- พลังงาน ---\n');
fprintf('%-12s %9s %9s %9s %10s %10s\n', 'โหมด', 'E_prop', 'E_steer', ...
        's_along', 'รวม Wh/km', 'ขับเคลื่อน');
for i = 1:numel(stats)
    fprintf('%-12s %9.4f %9.4f %9.2f %10.3f %10.3f\n', stats(i).name, ...
            stats(i).e_prop, stats(i).e_steer, stats(i).along, ...
            stats(i).whkm_total, stats(i).whkm_prop);
end

%% ---- ความนิ่งของตัวชี้วัด ----
% ถ้าครึ่งแรกกับครึ่งหลังให้ค่าต่างกันมาก แปลว่าตัวเลขขึ้นกับช่วงเวลาที่เลือก
% ไม่ใช่คุณสมบัติของระบบ และยังอ้างในเล่มไม่ได้
fprintf('\n--- ความนิ่งของตัวชี้วัด ครึ่งแรกเทียบครึ่งหลัง ---\n');
fprintf('%-12s %11s %11s %9s\n', 'โหมด', 'ครึ่งแรก', 'ครึ่งหลัง', 'ต่างกัน');
for i = 1:numel(stats)
    d = 100 * abs(stats(i).whkm_second - stats(i).whkm_first) / stats(i).whkm_first;
    fprintf('%-12s %11.3f %11.3f %8.2f %%\n', stats(i).name, ...
            stats(i).whkm_first, stats(i).whkm_second, d);
end

%% ---- ส่วนต่าง ----
d_tot  = NaN;
d_prop = NaN;
if numel(stats) == 2
    d_tot  = 100 * (stats(2).whkm_total - stats(1).whkm_total) / stats(1).whkm_total;
    d_prop = 100 * (stats(2).whkm_prop  - stats(1).whkm_prop)  / stats(1).whkm_prop;
    fprintf('\nส่วนต่างเทียบการขับด้วยมือ  รวม %+.2f %%   ขับเคลื่อนอย่างเดียว %+.2f %%\n', ...
            d_tot, d_prop);
    fprintf('สัดส่วนพลังงานหางเสือของออโตไพลอต %.1f %% ของพลังงานขับเคลื่อน\n', ...
            100 * stats(2).e_steer / stats(2).e_prop);
end

%% ---- กราฟสามแผง ----
figure('Name', 'Autopilot log analysis', 'Position', [100 100 1000 720]);

% แผงบน กินความกว้างทั้งแถว
subplot(2, 2, [1 2]);
hold on; grid on;
for i = 1:numel(stats)
    plot(stats(i).t_rel, stats(i).err, 'LineWidth', 1.0);
end
yline(0, 'k:');
xlabel('เวลาในโหมดนั้น [s]');
ylabel('ความคลาด [deg]');
title('หัวเรือเทียบทิศเป้าหมาย');
legend(names, 'Location', 'best');

% แผงล่างซ้าย การกระจายตัว นี่คือภาพที่ควรใส่ในสไลด์
subplot(2, 2, 3);
hold on; grid on;
edges = -25:1:25;
for i = 1:numel(stats)
    histogram(stats(i).err, edges, 'Normalization', 'probability', ...
              'FaceAlpha', 0.55);
end
xlabel('ความคลาดของหัวเรือ [deg]');
ylabel('สัดส่วนของเวลา');
title('การกระจายตัวของความคลาด');
legend(names, 'Location', 'northeast');

% แผงล่างขวา ส่วนต่างพลังงานเป็นเปอร์เซ็นต์
%
% เหตุผลที่ไม่พล็อตพลังงานสะสมเทียบระยะทาง คือส่วนต่างที่สนใจอยู่ระดับไม่กี่
% เปอร์เซ็นต์ ซึ่งบนแกนที่เริ่มจากศูนย์จะกลายเป็นเส้นสองเส้นที่ทับกันจนดูไม่ออก
% กราฟนี้วัดสิ่งเดียวกันแต่แสดงเฉพาะส่วนต่าง จึงเห็นได้ และแกนยังเริ่มที่ศูนย์จริง
subplot(2, 2, 4);
if numel(stats) == 2
    vals = [d_prop, d_tot];
    b = bar(vals, 0.55, 'FaceColor', 'flat');
    b.CData(1, :) = [0.11 0.48 0.55];   % ดีขึ้น
    b.CData(2, :) = [0.76 0.44 0.24];   % แย่ลง
    set(gca, 'XTickLabel', {'ขับเคลื่อน', 'รวมหางเสือ'});
    ylabel('ส่วนต่าง [%]');
    title('ออโตไพลอตประหยัดขึ้นหรือเปลืองขึ้น');
    grid on;
    yline(0, 'k-', 'LineWidth', 1.0);

    span = max(abs(vals)) * 1.6 + 1;
    ylim([-span span]);
    for k = 1:numel(vals)
        if vals(k) >= 0
            va = 'bottom';
        else
            va = 'top';
        end
        text(k, vals(k), sprintf('%+.2f %%', vals(k)), ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', va, ...
             'FontWeight', 'bold');
    end
end

end % analyze_log


function S = summarise(D, v_fallback)
% สรุปหนึ่งโหมด การอินทิเกรตใช้กฎสี่เหลี่ยมคางหมูให้ตรงกับที่เฟิร์มแวร์ทำ

S = struct();

t = D.t_ms / 1000;
S.t_rel = t - t(1);

S.err     = D.hdg_err_deg;
S.rms_err = sqrt(mean(S.err .^ 2, 'omitnan'));
S.p95_err = prctile(abs(S.err), 95);
S.max_err = max(abs(S.err));
S.rms_xte = sqrt(mean(D.xte_m .^ 2, 'omitnan'));

% XTE เป็นหลักฐานอิสระ หรือเป็นความคลาดของหัวเรือที่ถูกคูณค่าคงที่
% เกณฑ์คือถ้าอัตราส่วนแทบไม่แกว่งเลย แสดงว่าสองคอลัมน์ผูกกันด้วยสมการเดียว
use   = abs(S.err) > 0.5;
ratio = D.xte_m(use) ./ S.err(use);
S.xte_ratio = median(ratio, 'omitnan');
S.xte_is_proportional = (std(ratio, 'omitnan') < 0.05 * abs(S.xte_ratio));

v = D.vbat_v;
v(isnan(v) | v <= 0) = v_fallback;

% พลังงานในโหมดนี้ คำนวณสองทางแล้วเทียบกัน
% ทางแรกคือผลต่างของค่าสะสมที่เฟิร์มแวร์บันทึกไว้ ซึ่งทนต่อข้อมูลที่หาย
% ทางที่สองคืออินทิเกรตกระแสเองเทียบคอลัมน์เวลาจริง ไม่ใช่คาบคงที่
% การอินทิเกรตเทียบเวลาจริงสำคัญมาก เพราะถ้าคูณด้วยคาบคงที่ เรคคอร์ดที่หายไป
% จะทำให้พลังงานถูกนับน้อยกว่าจริง และโหมดที่ข้อมูลหายจะดูประหยัดเกินความเป็นจริง
S.e_prop  = D.e_prop_wh(end)  - D.e_prop_wh(1);
S.e_steer = D.e_steer_wh(end) - D.e_steer_wh(1);

e_prop_check  = trapz(t, v .* D.i_prop_a)  / 3600;
e_steer_check = trapz(t, v .* D.i_steer_a) / 3600;

S.e_prop_check  = e_prop_check;
S.e_steer_check = e_steer_check;
S.e_prop_gap    = abs(e_prop_check  - S.e_prop)  / max(abs(S.e_prop),  eps);
S.e_steer_gap   = abs(e_steer_check - S.e_steer) / max(abs(S.e_steer), eps);

S.along  = D.s_along_m(end)  - D.s_along_m(1);
S.ground = D.s_ground_m(end) - D.s_ground_m(1);
S.weave  = 100 * (S.ground - S.along) / S.along;

S.whkm_total = 1000 * (S.e_prop + S.e_steer) / S.along;
S.whkm_prop  = 1000 *  S.e_prop             / S.along;

% ความนิ่ง แบ่งครึ่งแล้วคำนวณตัวชี้วัดเดียวกันกับแต่ละครึ่ง
mid = floor(height(D) / 2);
S.whkm_first  = whkm_segment(D, 1, mid + 1);
S.whkm_second = whkm_segment(D, mid, height(D));

end % summarise


function w = whkm_segment(D, i1, i2)
% ตัวชี้วัดของช่วงแถว i1 ถึง i2 คำนวณจากผลต่างของค่าสะสมเท่านั้น
eP = D.e_prop_wh(i2)  - D.e_prop_wh(i1);
eS = D.e_steer_wh(i2) - D.e_steer_wh(i1);
sA = D.s_along_m(i2)  - D.s_along_m(i1);
if sA <= 0
    w = NaN;
else
    w = 1000 * (eP + eS) / sA;
end
end % whkm_segment