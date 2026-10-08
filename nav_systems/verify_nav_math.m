%% verify_nav_math.m
% -------------------------------------------------------------------------
% ตรวจสอบสูตรใน nav_math.c อย่างเป็นอิสระด้วย MATLAB
%
% สคริปต์นี้ไม่ได้อ่านโค้ด C แต่คำนวณโจทย์ชุดเดียวกันด้วยเครื่องมือของ MATLAB
% แล้วเทียบกับค่าที่ test_nav_math.c ใช้เป็นค่าคาดหวัง
% ถ้าสองทางตรงกัน แปลว่าสูตรถูกต้องโดยมีพยานสองปากที่ไม่เกี่ยวข้องกัน
%
% วิธีรัน   : เปิด MATLAB แล้วพิมพ์  verify_nav_math
% เกณฑ์ผ่าน : ทุกแถวขึ้น PASS และบรรทัดสุดท้ายขึ้น ALL CHECKS PASSED
% -------------------------------------------------------------------------

clear; clc;

R = 6371000;          % รัศมีโลกเฉลี่ย หน่วยเมตร ต้องตรงกับ NAV_EARTH_RADIUS_M

hasMapping = (exist('distance', 'file') == 2) && ...
             (exist('azimuth',  'file') == 2);

if hasMapping
    fprintf('ใช้ Mapping Toolbox เป็นตัวตรวจสอบ\n\n');
else
    fprintf('ไม่พบ Mapping Toolbox จะใช้สูตรที่เขียนเองใน MATLAB แทน\n\n');
end

%% ชุดโจทย์ ต้องตรงกับที่อยู่ใน test_nav_math.c ทุกตัว
%  คอลัมน์ : ชื่อ, lat1, lon1, lat2, lon2, ระยะคาดหวัง(m), ทิศคาดหวัง(deg)
cases = {
    'same point',          13.6512, 100.4947, 13.6512, 100.4947,       0.000000,   0.000000
    '1 deg lat at equator',  0.0,     0.0,      1.0,     0.0,     111194.926645,   0.000000
    '1 deg lon at equator',  0.0,     0.0,      0.0,     1.0,     111194.926645,  90.000000
    'KMUTT 500 m north',   13.6512, 100.4947, 13.6557, 100.4947,     500.377170,   0.000000
    'KMUTT NE diagonal',   13.6512, 100.4947, 13.6557, 100.4993,     705.286076,  44.807895
    '5.5 km due south',    13.6512, 100.4947, 13.6012, 100.4947,    5559.746332, 180.000000
    'due west',            13.6512, 100.4947, 13.6512, 100.4447,    5402.686199, 270.005900
    'antimeridian',         0.0,   179.9,      0.0,  -179.9,      22238.985329,  90.000000
};

tolDist = 1e-3;       % ยอมคลาดเคลื่อน 1 มิลลิเมตร
tolBrg  = 1e-4;       % ยอมคลาดเคลื่อน 0.0001 องศา
failed  = 0;

fprintf('%-22s %14s %14s   %10s %10s\n', ...
        'case', 'dist MATLAB', 'dist expect', 'brg MAT', 'brg exp');
fprintf('%s\n', repmat('-', 1, 78));

for k = 1:size(cases, 1)
    name = cases{k,1};
    la1  = cases{k,2};  lo1 = cases{k,3};
    la2  = cases{k,4};  lo2 = cases{k,5};
    wantD = cases{k,6}; wantB = cases{k,7};

    if hasMapping
        gotD = distance(la1, lo1, la2, lo2, R);
        gotB = azimuth(la1, lo1, la2, lo2);
    else
        gotD = haversineLocal(la1, lo1, la2, lo2, R);
        gotB = bearingLocal(la1, lo1, la2, lo2);
    end
    gotB = mod(gotB, 360);

    okD = abs(gotD - wantD) <= tolDist;
    % ทิศของจุดที่ซ้อนทับกันไม่มีความหมาย จึงข้ามการตรวจ
    if wantD == 0
        okB = true;
    else
        okB = abs(mod(gotB - wantB + 180, 360) - 180) <= tolBrg;
    end

    if ~(okD && okB)
        failed = failed + 1;
        tag = 'FAIL';
    else
        tag = 'PASS';
    end

    fprintf('[%s] %-17s %14.6f %14.6f   %10.6f %10.6f\n', ...
            tag, name, gotD, wantD, gotB, wantB);
end

%% ตรวจการบีบมุม
fprintf('\n');
wrapCases = [   0,    0
               90,   90
              190, -170
              359,   -1
             -190,  170
              720,    0 ];

for k = 1:size(wrapCases, 1)
    in   = wrapCases(k,1);
    want = wrapCases(k,2);
    if exist('wrapTo180', 'file') == 2
        got = wrapTo180(in);
    else
        got = mod(in + 180, 360) - 180;
    end
    if abs(got - want) <= 1e-9
        tag = 'PASS';
    else
        tag = 'FAIL';
        failed = failed + 1;
    end
    fprintf('[%s] wrap180(%7.1f) = %8.1f   want %8.1f\n', tag, in, got, want);
end

fprintf('\n');
if failed == 0
    fprintf('ALL CHECKS PASSED\n');
else
    fprintf('%d CHECK(S) FAILED\n', failed);
end

%% ฟังก์ชันสำรองเมื่อไม่มี Mapping Toolbox
function d = haversineLocal(la1, lo1, la2, lo2, R)
    p1 = deg2rad(la1);  p2 = deg2rad(la2);
    dp = deg2rad(la2 - la1);
    dl = deg2rad(lo2 - lo1);
    a  = sin(dp/2)^2 + cos(p1) * cos(p2) * sin(dl/2)^2;
    d  = 2 * R * atan2(sqrt(a), sqrt(1 - a));
end

function b = bearingLocal(la1, lo1, la2, lo2)
    p1 = deg2rad(la1);  p2 = deg2rad(la2);
    dl = deg2rad(lo2 - lo1);
    y  = sin(dl) * cos(p2);
    x  = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
    b  = rad2deg(atan2(y, x));
end
