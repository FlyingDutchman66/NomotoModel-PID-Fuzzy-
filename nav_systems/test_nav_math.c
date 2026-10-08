/**
 * test_nav_math.c
 * ---------------------------------------------------------------------------
 * ชุดทดสอบของ nav_math.c — รันบนโน้ตบุ๊กหรือ OnlineGDB เท่านั้น ไม่ขึ้นเรือ
 *
 * บน OnlineGDB : วางไฟล์นี้ในแท็บ main.c
 *                แท็บอื่นคือ nav_types.h, nav_math.h, nav_math.c
 * บนเครื่องตัวเอง :
 *     gcc -Wall -Wextra -o test_nav_math test_nav_math.c nav_math.c -lm
 *     ./test_nav_math
 *
 * ค่าอ้างอิงทั้งหมดในไฟล์นี้คำนวณด้วยสูตร Haversine บนโลกทรงกลม R = 6371 km
 * สคริปต์ verify_nav_math.m ตรวจค่าเดียวกันนี้ซ้ำด้วย MATLAB อย่างเป็นอิสระ
 *
 * เกณฑ์ผ่าน   : ทุกบรรทัดขึ้น PASS และบรรทัดสุดท้ายขึ้น ALL TESTS PASSED
 * เกณฑ์ไม่ผ่าน : มีบรรทัดใดขึ้น FAIL (exit code = 1)
 * ---------------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "nav_math.h"

static int g_failed = 0;

static void check(const char *label, double got, double want, double tol)
{
    bool ok = (fabs(got - want) <= tol);
    if (!ok) g_failed++;
    printf("[%s] %-42s got=%14.6f want=%14.6f\n",
           ok ? "PASS" : "FAIL", label, got, want);
}

static void check_bool(const char *label, bool got, bool want)
{
    bool ok = (got == want);
    if (!ok) g_failed++;
    printf("[%s] %-42s got=%-5s want=%s\n",
           ok ? "PASS" : "FAIL", label,
           got ? "true" : "false", want ? "true" : "false");
}

/* จุดอ้างอิงแถว KMUTT บางมด */
#define LAT_A  13.6512
#define LON_A 100.4947

int main(void)
{
    printf("=== nav_wrap180 ===\n");
    check("0 stays 0",            nav_wrap180(0.0f),      0.0f,   1e-4);
    check("90 stays 90",          nav_wrap180(90.0f),    90.0f,   1e-4);
    check("190 becomes -170",     nav_wrap180(190.0f), -170.0f,   1e-4);
    check("359 becomes -1",       nav_wrap180(359.0f),   -1.0f,   1e-4);
    check("-190 becomes 170",     nav_wrap180(-190.0f), 170.0f,   1e-4);
    check("720 becomes 0",        nav_wrap180(720.0f),    0.0f,   1e-4);
    check("-540 becomes 180|-180",
          fabs(nav_wrap180(-540.0f)),                   180.0f,   1e-4);

    /* กับดักเลขศูนย์องศา : หัวเรือ 359 เป้าหมาย 1 ต้องเลี้ยวขวา 2 องศา
     * ไม่ใช่เลี้ยวซ้าย 358 องศา นี่คือเหตุผลทั้งหมดที่ต้องมีฟังก์ชันนี้ */
    check("error 1 - 359 is +2 not -358",
          nav_wrap180(1.0f - 359.0f), 2.0f, 1e-4);
    check("error 359 - 1 is -2 not +358",
          nav_wrap180(359.0f - 1.0f), -2.0f, 1e-4);

    printf("\n=== nav_wrap360 ===\n");
    check("0 stays 0",          nav_wrap360(0.0f),      0.0f, 1e-4);
    check("360 becomes 0",      nav_wrap360(360.0f),    0.0f, 1e-4);
    check("-1 becomes 359",     nav_wrap360(-1.0f),   359.0f, 1e-4);
    check("450 becomes 90",     nav_wrap360(450.0f),   90.0f, 1e-4);
    check("-450 becomes 270",   nav_wrap360(-450.0f), 270.0f, 1e-4);

    printf("\n=== nav_haversine_m ===\n");
    check("same point is zero",
          nav_haversine_m(LAT_A, LON_A, LAT_A, LON_A), 0.0, 1e-9);
    check("1 deg of latitude at equator",
          nav_haversine_m(0.0, 0.0, 1.0, 0.0), 111194.926645, 1e-3);
    check("1 deg of longitude at equator",
          nav_haversine_m(0.0, 0.0, 0.0, 1.0), 111194.926645, 1e-3);
    check("symmetric A to B equals B to A",
          nav_haversine_m(LAT_A, LON_A, 13.6557, 100.4993)
        - nav_haversine_m(13.6557, 100.4993, LAT_A, LON_A), 0.0, 1e-9);
    check("KMUTT to 500 m north",
          nav_haversine_m(LAT_A, LON_A, 13.6557, LON_A), 500.377170, 1e-3);
    check("KMUTT diagonal north-east",
          nav_haversine_m(LAT_A, LON_A, 13.6557, 100.4993), 705.286076, 1e-3);
    check("5.5 km due south",
          nav_haversine_m(LAT_A, LON_A, 13.6012, LON_A), 5559.746332, 1e-3);
    /* ข้ามเส้นแบ่งวันสากล ผลต่างลองจิจูดดิบคือ 359.8 แต่ระยะจริงสั้นมาก */
    check("across the antimeridian",
          nav_haversine_m(0.0, 179.9, 0.0, -179.9), 22238.985329, 1e-3);

    printf("\n=== nav_bearing_deg ===\n");
    check("due north is 0",
          nav_bearing_deg(0.0, 0.0, 1.0, 0.0), 0.0, 1e-6);
    check("due east at equator is 90",
          nav_bearing_deg(0.0, 0.0, 0.0, 1.0), 90.0, 1e-6);
    check("due south is 180",
          nav_bearing_deg(LAT_A, LON_A, 13.6012, LON_A), 180.0, 1e-6);
    check("north from KMUTT is 0",
          nav_bearing_deg(LAT_A, LON_A, 13.6557, LON_A), 0.0, 1e-6);
    check("north-east diagonal",
          nav_bearing_deg(LAT_A, LON_A, 13.6557, 100.4993), 44.807895, 1e-4);
    /* ไม่ใช่ 270.000000 พอดี เพราะเส้นทางสั้นสุดบนทรงกลมไม่ใช่เส้นขนานละติจูด */
    check("due west is slightly past 270",
          nav_bearing_deg(LAT_A, LON_A, LAT_A, 100.4447), 270.005900, 1e-4);
    check("result always inside [0,360)",
          nav_bearing_deg(LAT_A, LON_A, 13.6012, 100.4447) >= 0.0
       && nav_bearing_deg(LAT_A, LON_A, 13.6012, 100.4447) < 360.0, 1.0, 0.0);
    check("across the antimeridian is 90",
          nav_bearing_deg(0.0, 179.9, 0.0, -179.9), 90.0, 1e-6);

    printf("\n=== nav_is_fresh ===\n");
    {
        NavState_t s;
        memset(&s, 0, sizeof(s));
        s.nav_valid    = true;
        s.timestamp_ms = 10000;

        check_bool("fresh data accepted",  nav_is_fresh(&s, 10200, 500), true);
        check_bool("exactly at the limit", nav_is_fresh(&s, 10500, 500), true);
        check_bool("stale data rejected",  nav_is_fresh(&s, 10501, 500), false);

        s.nav_valid = false;
        check_bool("invalid flag rejected", nav_is_fresh(&s, 10000, 500), false);

        check_bool("null pointer rejected", nav_is_fresh(NULL, 0, 500), false);

        /* ตัวนับมิลลิวินาที 32 บิตวนรอบทุก 49.7 วัน
         * ข้อมูลที่บันทึกไว้ก่อนวนรอบ 100 ms ต้องยังถือว่าสดอยู่ */
        s.nav_valid    = true;
        s.timestamp_ms = 0xFFFFFF9CU;          /* เหลืออีก 100 ms จะวนรอบ */
        check_bool("survives millisecond rollover",
                   nav_is_fresh(&s, 150U, 500), true);
    }

    printf("\n");
    if (g_failed == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("%d TEST(S) FAILED\n", g_failed);
    return 1;
}
