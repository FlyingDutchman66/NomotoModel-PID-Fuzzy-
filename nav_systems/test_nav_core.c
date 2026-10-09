/**
 * test_nav_core.c
 * ---------------------------------------------------------------------------
 * ชุดทดสอบของ nav_core.c — รันบนโน้ตบุ๊กหรือ OnlineGDB เท่านั้น ไม่ขึ้นเรือ
 *
 * บน OnlineGDB : main.c = ไฟล์นี้
 *                แท็บอื่น = nav_types.h, nmea.h, nmea.c,
 *                           nav_math.h, nav_math.c,
 *                           nav_fusion.h, nav_fusion.c,
 *                           nav_guidance.h, nav_guidance.c,
 *                           nav_core.h, nav_core.c
 * บนเครื่องตัวเอง :
 *     gcc -Wall -Wextra -o test_nav_core test_nav_core.c \
 *         nav_core.c nav_guidance.c nav_fusion.c nav_math.c nmea.c -lm
 *     ./test_nav_core
 *
 * เกณฑ์ผ่าน : ทุกบรรทัดขึ้น PASS และบรรทัดสุดท้ายขึ้น ALL TESTS PASSED
 * ---------------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "nav_core.h"
#include "nav_math.h"

static int g_failed = 0;

static void check_bool(const char *label, bool got, bool want)
{
    bool ok = (got == want);
    if (!ok) g_failed++;
    printf("[%s] %-48s got=%-5s want=%s\n",
           ok ? "PASS" : "FAIL", label,
           got ? "true" : "false", want ? "true" : "false");
}

static void check_near(const char *label, double got, double want, double tol)
{
    bool ok = (fabs(got - want) <= tol);
    if (!ok) g_failed++;
    printf("[%s] %-48s got=%11.4f want=%11.4f\n",
           ok ? "PASS" : "FAIL", label, got, want);
}

/* เส้นทางรูปตัว L เดียวกับที่ใช้ทดสอบ LOS */
static const NavWaypoint_t g_wps[3] = {
    { 13.65120000, 100.49470000 },
    { 13.65569661, 100.49470000 },
    { 13.65569661, 100.49932742 }
};

static const double K_M_PER_DEG = 6371000.0 * 3.14159265358979323846 / 180.0;

static void offset_from(double lat0, double lon0,
                        double north_m, double east_m,
                        double *lat, double *lon)
{
    *lat = lat0 + north_m / K_M_PER_DEG;
    *lon = lon0 + east_m / (K_M_PER_DEG
                            * cos(lat0 * 3.14159265358979323846 / 180.0));
}

/* สร้าง GpsRmc_t ที่ผ่านการแกะมาแล้ว เพื่อป้อนเข้า nav_core โดยตรง
 * การแกะประโยค NMEA ถูกทดสอบแยกไว้ใน test_nmea.c แล้ว */
static GpsRmc_t make_fix(double lat, double lon, float sog, float cog)
{
    GpsRmc_t f;
    memset(&f, 0, sizeof(f));
    f.lat_deg   = lat;
    f.lon_deg   = lon;
    f.sog_mps   = sog;
    f.cog_deg   = cog;
    f.cog_valid = true;
    f.utc_hour  = 8;
    f.utc_min   = 59;
    f.utc_sec   = 32;
    return f;
}

int main(void)
{
    printf("=== startup : nothing is trusted before data arrives ===\n");
    {
        NavCore_t n;
        check_bool("init with defaults",
                   nav_core_init(&n, g_wps, 3, NULL), true);

        const NavState_t *s = nav_core_state(&n, 1000u);
        check_bool("state pointer is never null", s != NULL, true);
        check_bool("not valid before any fix", s->nav_valid, false);
        check_near("  gps age is the sentinel, not zero",
                   (double)s->gps_age_ms, 4294967295.0, 0);

        /* ไจโรอย่างเดียวไม่พอที่จะทำให้ข้อมูลเชื่อถือได้ */
        nav_core_on_imu(&n, 0.0f, 1000u);
        nav_core_on_imu(&n, 0.0f, 1020u);
        s = nav_core_state(&n, 1020u);
        check_bool("IMU alone does not make nav valid", s->nav_valid, false);

        check_bool("init rejects a one-point path",
                   nav_core_init(&n, g_wps, 1, NULL), false);
        check_bool("init rejects null core",
                   nav_core_init(NULL, g_wps, 3, NULL), false);
    }

    printf("\n=== first fix brings the system to life ===\n");
    {
        NavCore_t n;
        nav_core_init(&n, g_wps, 3, NULL);

        double lat, lon;
        offset_from(g_wps[0].lat_deg, g_wps[0].lon_deg, 100.0, 0.0, &lat, &lon);

        GpsRmc_t fix = make_fix(lat, lon, 2.0f, 5.0f);
        nav_core_on_gps(&n, &fix, 2000u);

        const NavState_t *s = nav_core_state(&n, 2000u);
        check_bool("valid after the first good fix", s->nav_valid, true);
        check_near("  heading snapped to first COG", s->heading_deg, 5.0, 1e-3);
        check_near("  position copied through", s->latitude_deg, lat, 1e-9);
        check_near("  speed copied through",    s->sog_mps, 2.0, 1e-4);
        check_near("  gps age is zero",  (double)s->gps_age_ms, 0.0, 0);
        check_near("  aiming at waypoint 1",    s->wp_index, 1, 0);
        check_near("  on the path, so no cross track error",
                   s->cross_track_error_m, 0.0, 1e-2);
        check_near("  desired heading is due north",
                   s->desired_heading_deg, 0.0, 1e-3);

        /* หัวใจของสัญญา : heading_error ต้องถูกคำนวณและบีบช่วงให้แล้ว
         * หัวเรือ 5 องศา เป้าหมาย 0 องศา ผลต่างคือ -5 */
        check_near("  heading error is computed and wrapped",
                   s->heading_error_deg, -5.0, 1e-3);
    }

    printf("\n=== the error wraps correctly across zero ===\n");
    {
        NavCore_t n;
        nav_core_init(&n, g_wps, 3, NULL);

        double lat, lon;
        offset_from(g_wps[0].lat_deg, g_wps[0].lon_deg, 100.0, 0.0, &lat, &lon);

        /* หัวเรือ 359 องศา เส้นทางมุ่งเหนือคือ 0 องศา
         * คำตอบที่ถูกคือ +1 ไม่ใช่ -359 */
        GpsRmc_t fix = make_fix(lat, lon, 2.0f, 359.0f);
        nav_core_on_gps(&n, &fix, 2000u);

        const NavState_t *s = nav_core_state(&n, 2000u);
        check_near("heading 359 towards 0 gives +1, not -359",
                   s->heading_error_deg, 1.0, 1e-3);
    }

    printf("\n=== seq proves the producer is still alive ===\n");
    {
        NavCore_t n;
        nav_core_init(&n, g_wps, 3, NULL);

        double lat, lon;
        offset_from(g_wps[0].lat_deg, g_wps[0].lon_deg, 100.0, 0.0, &lat, &lon);
        GpsRmc_t fix = make_fix(lat, lon, 2.0f, 0.0f);

        nav_core_on_gps(&n, &fix, 2000u);
        uint16_t seq_a = nav_core_state(&n, 2000u)->seq;

        nav_core_on_imu(&n, 1.0f, 2020u);
        nav_core_on_imu(&n, 1.0f, 2040u);
        uint16_t seq_b = nav_core_state(&n, 2040u)->seq;

        check_bool("seq advances as new data arrives", seq_b > seq_a, true);

        /* เรียก state ซ้ำโดยไม่มีข้อมูลใหม่ ต้องไม่ขยับ seq
         * ไม่งั้น Control จะเข้าใจผิดว่ามีข้อมูลสดเข้ามาตลอด */
        uint16_t seq_c = nav_core_state(&n, 2100u)->seq;
        check_near("reading alone does not advance seq",
                   seq_c, seq_b, 0);
    }

    printf("\n=== GPS dropout must raise the fail-safe flag ===\n");
    {
        NavCore_t n;
        NavCoreConfig_t cfg;
        nav_core_default_config(&cfg);
        cfg.max_fix_age_ms = 3000u;
        nav_core_init(&n, g_wps, 3, &cfg);

        double lat, lon;
        offset_from(g_wps[0].lat_deg, g_wps[0].lon_deg, 100.0, 0.0, &lat, &lon);
        GpsRmc_t fix = make_fix(lat, lon, 2.0f, 0.0f);
        nav_core_on_gps(&n, &fix, 10000u);

        check_bool("valid right after the fix",
                   nav_core_state(&n, 10000u)->nav_valid, true);
        check_bool("still valid at 2.9 s old",
                   nav_core_state(&n, 12900u)->nav_valid, true);
        check_bool("still valid exactly at the limit",
                   nav_core_state(&n, 13000u)->nav_valid, true);
        check_bool("goes invalid one millisecond past the limit",
                   nav_core_state(&n, 13001u)->nav_valid, false);

        /* สำคัญที่สุด : ไจโรยังเดินอยู่ แต่นั่นไม่ทำให้ข้อมูลกลับมาน่าเชื่อถือ
         * เพราะตำแหน่งเรือเป็นของเก่าไปแล้ว */
        nav_core_on_imu(&n, 0.5f, 14000u);
        nav_core_on_imu(&n, 0.5f, 14020u);
        const NavState_t *s = nav_core_state(&n, 14020u);
        check_bool("IMU activity does not revive a stale fix",
                   s->nav_valid, false);
        check_near("  age keeps growing", (double)s->gps_age_ms, 4020.0, 0);

        /* fix ใหม่เข้ามา ต้องกลับมาใช้ได้ทันที */
        nav_core_on_gps(&n, &fix, 15000u);
        check_bool("a fresh fix restores validity",
                   nav_core_state(&n, 15000u)->nav_valid, true);
    }

    printf("\n=== millisecond counter rollover ===\n");
    {
        /* ตัวนับ 32 บิตวนรอบทุก 49.7 วัน
         * fix ที่มาก่อนวนรอบ 100 ms ต้องยังนับว่าสดอยู่หลังวนรอบ 50 ms */
        NavCore_t n;
        nav_core_init(&n, g_wps, 3, NULL);

        double lat, lon;
        offset_from(g_wps[0].lat_deg, g_wps[0].lon_deg, 100.0, 0.0, &lat, &lon);
        GpsRmc_t fix = make_fix(lat, lon, 2.0f, 0.0f);

        nav_core_on_gps(&n, &fix, 0xFFFFFF9Cu);   /* อีก 100 ms จะวนรอบ */
        const NavState_t *s = nav_core_state(&n, 50u);   /* วนรอบไปแล้ว 50 ms */
        check_near("age computed across the rollover",
                   (double)s->gps_age_ms, 150.0, 0);
        check_bool("  still considered valid", s->nav_valid, true);
    }

    printf("\n=== full voyage through both legs ===\n");
    {
        NavCore_t n;
        NavCoreConfig_t cfg;
        nav_core_default_config(&cfg);
        nav_core_init(&n, g_wps, 3, &cfg);

        const double dt_imu = 0.02;      /* 50 Hz */
        const double speed  = 2.0;
        const double tau    = 3.0;

        double lat, lon;
        offset_from(g_wps[0].lat_deg, g_wps[0].lon_deg, 10.0, 25.0, &lat, &lon);

        double psi_true = 0.0;           /* หัวเรือจริง เริ่มชี้เหนือ */
        uint32_t t_ms   = 1000u;
        bool finished   = false;
        double worst_settled = 0.0;
        uint8_t leg_now = 1;
        uint32_t leg_start_ms = t_ms;
        int invalid_after_start = 0;
        bool started = false;

        for (int i = 0; i < 30000 && !finished; i++) {
            t_ms += (uint32_t)(dt_imu * 1000.0);

            /* ไจโรที่มี bias ปนมาจริง ๆ แบบที่เซ็นเซอร์ถูก ๆ เป็น */
            const NavState_t *s = nav_core_state(&n, t_ms);
            double err = nav_wrap180((float)(s->desired_heading_deg - psi_true));
            double rate_true = (err / tau);
            psi_true = nav_wrap360((float)(psi_true + rate_true * dt_imu));

            nav_core_on_imu(&n, (float)(rate_true + 0.8), t_ms);

            /* เรือเคลื่อนที่ */
            double rad = psi_true * 3.14159265358979323846 / 180.0;
            lat += speed * cos(rad) * dt_imu / K_M_PER_DEG;
            lon += speed * sin(rad) * dt_imu
                   / (K_M_PER_DEG * cos(lat * 3.14159265358979323846 / 180.0));

            /* GPS มาวินาทีละครั้ง */
            if (i % 50 == 0) {
                GpsRmc_t fix = make_fix(lat, lon, (float)speed,
                                        (float)psi_true);
                nav_core_on_gps(&n, &fix, t_ms);
            }

            s = nav_core_state(&n, t_ms);

            if (s->nav_valid) started = true;
            if (started && !s->nav_valid) invalid_after_start++;

            if (s->wp_index != leg_now) {
                leg_now      = s->wp_index;
                leg_start_ms = t_ms;
            }
            if (started && (t_ms - leg_start_ms) > 40000u) {
                double a = fabs(s->cross_track_error_m);
                if (a > worst_settled) worst_settled = a;
            }

            if (s->wp_index == 2 && s->dist_to_wp_m < 16.0f) {
                /* เข้าใกล้ waypoint สุดท้ายแล้ว */
                finished = true;
            }
        }

        const NavState_t *s = nav_core_state(&n, t_ms);
        printf("        worst cross track once settled = %7.3f m\n",
               worst_settled);
        printf("        gyro bias learned              = %7.4f deg/s\n",
               n.fusion.gyro_bias_dps);
        printf("        final wp index                 = %7u\n\n",
               s->wp_index);

        check_bool("voyage completed", finished, true);
        check_near("never dropped out mid-voyage", invalid_after_start, 0, 0);
        check_bool("tracked the path within a metre", worst_settled < 1.0, true);
        check_near("learned the gyro bias",
                   n.fusion.gyro_bias_dps, 0.8, 0.25);
        check_bool("state still valid at the end", s->nav_valid, true);
    }

    printf("\n");
    if (g_failed == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("%d TEST(S) FAILED\n", g_failed);
    return 1;
}
