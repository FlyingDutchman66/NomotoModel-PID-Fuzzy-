/**
 * test_nav_guidance.c
 * ---------------------------------------------------------------------------
 * ชุดทดสอบของ nav_guidance.c — รันบนโน้ตบุ๊กหรือ OnlineGDB เท่านั้น ไม่ขึ้นเรือ
 *
 * บน OnlineGDB : main.c = ไฟล์นี้
 *                แท็บอื่น = nav_types.h, nav_math.h, nav_math.c,
 *                           nav_guidance.h, nav_guidance.c
 * บนเครื่องตัวเอง :
 *     gcc -Wall -Wextra -o test_nav_guidance \
 *         test_nav_guidance.c nav_guidance.c nav_math.c -lm
 *     ./test_nav_guidance
 *
 * ส่วนสุดท้ายขับเรือจำลองแบบจลนศาสตร์ตามคำสั่งที่ LOS ให้มา
 * เพื่อดูว่าเรือกลับเข้าเส้นทางได้จริง ไม่ใช่แค่สูตรถูกบนกระดาษ
 *
 * เกณฑ์ผ่าน : ทุกบรรทัดขึ้น PASS และบรรทัดสุดท้ายขึ้น ALL TESTS PASSED
 * ---------------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "nav_guidance.h"
#include "nav_math.h"

static int g_failed = 0;

static void check_near(const char *label, double got, double want, double tol)
{
    bool ok = (fabs(got - want) <= tol);
    if (!ok) g_failed++;
    printf("[%s] %-44s got=%11.4f want=%11.4f\n",
           ok ? "PASS" : "FAIL", label, got, want);
}

static void check_bool(const char *label, bool got, bool want)
{
    bool ok = (got == want);
    if (!ok) g_failed++;
    printf("[%s] %-44s got=%-5s want=%s\n",
           ok ? "PASS" : "FAIL", label,
           got ? "true" : "false", want ? "true" : "false");
}

static void check_less(const char *label, double got, double limit)
{
    bool ok = (got < limit);
    if (!ok) g_failed++;
    printf("[%s] %-44s got=%11.4f limit<%10.4f\n",
           ok ? "PASS" : "FAIL", label, got, limit);
}

/* ---- เส้นทางทดสอบ รูปตัว L แถว KMUTT บางมด ----
 *   WP0 จุดเริ่ม
 *   WP1 อยู่เหนือ WP0 ขึ้นไป 500 m
 *   WP2 อยู่ตะวันออกของ WP1 ไป 500 m   */
static const NavWaypoint_t g_wps[3] = {
    { 13.65120000, 100.49470000 },
    { 13.65569661, 100.49470000 },
    { 13.65569661, 100.49932742 }
};

/* แปลงระยะเป็นเมตรให้เป็นพิกัด โดยอ้างอิงจุดกำเนิดที่กำหนด */
static void offset_from(double lat0, double lon0,
                        double north_m, double east_m,
                        double *lat, double *lon)
{
    const double k = 6371000.0 * 3.14159265358979323846 / 180.0;
    *lat = lat0 + north_m / k;
    *lon = lon0 + east_m / (k * cos(lat0 * 3.14159265358979323846 / 180.0));
}

int main(void)
{
    printf("=== nav_path_init rejects bad input ===\n");
    {
        NavPath_t p;
        check_bool("null path",        nav_path_init(NULL, g_wps, 3, 20, 15), false);
        check_bool("null waypoints",   nav_path_init(&p, NULL, 3, 20, 15),    false);
        check_bool("only one waypoint",nav_path_init(&p, g_wps, 1, 20, 15),   false);
        check_bool("zero lookahead",   nav_path_init(&p, g_wps, 3, 0, 15),    false);
        check_bool("negative radius",  nav_path_init(&p, g_wps, 3, 20, -1),   false);
        check_bool("valid setup",      nav_path_init(&p, g_wps, 3, 20, 15),   true);
        check_near("  starts aiming at wp 1", p.wp_index, 1, 0);
        check_bool("  not finished yet", p.finished, false);
    }

    printf("\n=== geometry on the first leg (due north, 500 m) ===\n");
    {
        NavPath_t p;
        NavGuidance_t g;
        double lat, lon;

        /* บนเส้นพอดี กลางทาง : ไม่มีความผิดพลาด จึงมุ่งหน้าตามเส้นทางตรง ๆ */
        nav_path_init(&p, g_wps, 3, 20.0f, 15.0f);
        offset_from(g_wps[0].lat_deg, g_wps[0].lon_deg, 250.0, 0.0, &lat, &lon);
        check_bool("update succeeds", nav_los_update(&p, lat, lon, &g), true);
        check_near("  path bearing is due north", g.path_bearing_deg, 0.0, 1e-3);
        check_near("  cross track is zero",       g.cross_track_error_m, 0.0, 1e-3);
        check_near("  along track is halfway",    g.along_track_m, 250.0, 1e-2);
        check_near("  segment length",            g.segment_length_m, 500.0, 1e-2);
        check_near("  distance to waypoint",      g.dist_to_wp_m, 250.0, 1e-2);
        check_near("  desired heading equals path", g.desired_heading_deg, 0.0, 1e-3);

        /* ออกขวา 10 m กับ lookahead 20 m : atan(10/20) = 26.565 องศา
         * ต้องหันซ้ายเท่านั้น ได้ 360 - 26.565 = 333.435 */
        nav_path_init(&p, g_wps, 3, 20.0f, 15.0f);
        offset_from(g_wps[0].lat_deg, g_wps[0].lon_deg, 250.0, 10.0, &lat, &lon);
        nav_los_update(&p, lat, lon, &g);
        check_near("right of path gives positive cross track",
                   g.cross_track_error_m, 10.0, 1e-2);
        check_near("  steers left to come back",
                   g.desired_heading_deg, 333.4349, 1e-2);

        /* ออกซ้าย 10 m : ต้องหันขวาเป็นมุมเท่ากันแต่คนละทาง */
        nav_path_init(&p, g_wps, 3, 20.0f, 15.0f);
        offset_from(g_wps[0].lat_deg, g_wps[0].lon_deg, 250.0, -10.0, &lat, &lon);
        nav_los_update(&p, lat, lon, &g);
        check_near("left of path gives negative cross track",
                   g.cross_track_error_m, -10.0, 1e-2);
        check_near("  steers right to come back",
                   g.desired_heading_deg, 26.5651, 1e-2);

        /* ออกไกลขึ้นเป็น 40 m : มุมต้องชันขึ้น แต่ไม่มีทางถึง 90 องศา */
        nav_path_init(&p, g_wps, 3, 20.0f, 15.0f);
        offset_from(g_wps[0].lat_deg, g_wps[0].lon_deg, 250.0, 40.0, &lat, &lon);
        nav_los_update(&p, lat, lon, &g);
        check_near("further off path steers harder",
                   g.desired_heading_deg, 296.5651, 1e-2);
        check_less("  but never turns a full right angle",
                   fabs(nav_wrap180(g.desired_heading_deg - g.path_bearing_deg)),
                   90.0);
    }

    printf("\n=== waypoint switching ===\n");
    {
        NavPath_t p;
        NavGuidance_t g;
        double lat, lon;

        nav_path_init(&p, g_wps, 3, 20.0f, 15.0f);

        /* ยังเหลือระยะ 100 m ตามแนวเส้นทาง ยังไม่ควรสลับ */
        offset_from(g_wps[0].lat_deg, g_wps[0].lon_deg, 400.0, 0.0, &lat, &lon);
        nav_los_update(&p, lat, lon, &g);
        check_near("still on leg 1 when 100 m remain", g.wp_index, 1, 0);

        /* เหลือ 10 m ซึ่งน้อยกว่ารัศมี 15 m ต้องสลับไปช่วงถัดไป */
        offset_from(g_wps[0].lat_deg, g_wps[0].lon_deg, 490.0, 0.0, &lat, &lon);
        nav_los_update(&p, lat, lon, &g);
        check_near("switches to leg 2 inside the radius", g.wp_index, 2, 0);
        check_near("  new path bearing is due east",
                   g.path_bearing_deg, 90.0, 1e-2);
        check_bool("  not finished yet", g.finished, false);

        /* ถึง waypoint สุดท้าย */
        offset_from(g_wps[1].lat_deg, g_wps[1].lon_deg, 0.0, 495.0, &lat, &lon);
        nav_los_update(&p, lat, lon, &g);
        check_bool("finished at the last waypoint", g.finished, true);
    }

    printf("\n=== overshoot does not trap the boat ===\n");
    {
        /* อาการพังคลาสสิก : เรือเลยจุด B ไปแล้วแต่ยังออกนอกเส้นไกล
         * ถ้าใช้ระยะตรงถึง B เป็นเกณฑ์ เรือจะวนกลับมาหา B ไม่รู้จบ
         * เกณฑ์ตามแนวเส้นทางแก้ปัญหานี้ */
        NavPath_t p;
        NavGuidance_t g;
        double lat, lon;

        nav_path_init(&p, g_wps, 3, 20.0f, 15.0f);
        /* เลยปลายช่วงไป 30 m และออกนอกเส้นทางขวา 60 m
         * ระยะตรงถึง B คือราว 67 m ซึ่งไกลกว่ารัศมี 15 m มาก */
        offset_from(g_wps[0].lat_deg, g_wps[0].lon_deg, 530.0, 60.0, &lat, &lon);
        nav_los_update(&p, lat, lon, &g);
        check_near("advances past a waypoint it overshot", g.wp_index, 2, 0);
        check_less("  despite being far from that waypoint", 15.0,
                   sqrt(30.0 * 30.0 + 60.0 * 60.0));
    }

    printf("\n=== closed loop: can the boat actually get back on track ===\n");
    {
        /* เรือจำลองแบบจลนศาสตร์
         *   ความเร็วคงที่ 2 m/s
         *   หัวเรือไล่ตาม desired heading แบบหน่วงอันดับหนึ่ง เวลาคงที่ 3 s
         *     ซึ่งเลียนแบบว่าเรือหมุนตัวไม่ได้ทันที
         * เริ่มที่ออกนอกเส้นทางขวา 30 m แล้วดูว่ากลับเข้าเส้นได้ไหม */
        NavPath_t p;
        NavGuidance_t g;

        const double dt    = 0.1;
        const double speed = 2.0;
        const double tau   = 3.0;

        nav_path_init(&p, g_wps, 3, 20.0f, 15.0f);

        double lat, lon;
        offset_from(g_wps[0].lat_deg, g_wps[0].lon_deg, 20.0, 30.0, &lat, &lon);

        double psi       = 0.0;      /* หัวเรือเริ่มต้นชี้เหนือ */
        double worst_settled = 0.0;  /* ความผิดพลาดสูงสุดเฉพาะช่วงที่นิ่งแล้ว */
        double final_cross = 0.0;
        int    steps = (int)(600.0 / dt);
        bool   ever_finished = false;

        /* ทุกครั้งที่สลับ waypoint เรือจะอยู่ห่างจากเส้นทางใหม่ทันที
         * เพราะมันเพิ่งเลี้ยวหัวมุม นั่นเป็นเรื่องปกติไม่ใช่ความล้มเหลว
         * จึงเริ่มจับคะแนนหลังอยู่บนขานั้นมาแล้ว 40 วินาที */
        uint8_t  leg_now   = 1;
        double   leg_start = 0.0;
        int      n_settled = 0;

        const double k_m_per_deg =
            6371000.0 * 3.14159265358979323846 / 180.0;

        for (int i = 0; i < steps; i++) {
            double t = i * dt;

            if (!nav_los_update(&p, lat, lon, &g)) {
                break;
            }
            if (g.finished) {
                ever_finished = true;
                break;
            }

            /* หัวเรือไล่ตามคำสั่งแบบหน่วง ใช้ผลต่างที่บีบช่วงแล้วเสมอ */
            double err = nav_wrap180((float)(g.desired_heading_deg - psi));
            psi = nav_wrap360((float)(psi + (err / tau) * dt));

            /* เดินเรือไปข้างหน้าตามหัวเรือปัจจุบัน */
            double rad   = psi * 3.14159265358979323846 / 180.0;
            double dn    = speed * cos(rad) * dt;
            double de    = speed * sin(rad) * dt;
            lat += dn / k_m_per_deg;
            lon += de / (k_m_per_deg
                         * cos(lat * 3.14159265358979323846 / 180.0));

            if (g.wp_index != leg_now) {
                leg_now   = g.wp_index;
                leg_start = t;
            }
            if ((t - leg_start) > 40.0) {
                double a = fabs(g.cross_track_error_m);
                if (a > worst_settled) {
                    worst_settled = a;
                }
                n_settled++;
            }
            final_cross = g.cross_track_error_m;
        }

        printf("        worst cross track once settled = %7.3f m\n",
               worst_settled);
        printf("        samples scored                 = %7d\n", n_settled);
        printf("        cross track at the end         = %7.3f m\n\n",
               final_cross);

        check_bool("boat reaches the final waypoint", ever_finished, true);
        check_less("both legs were scored, not just one", 200, n_settled);
        check_less("settles within one metre of the path",
                   worst_settled, 1.0);
    }

    printf("\n");
    if (g_failed == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("%d TEST(S) FAILED\n", g_failed);
    return 1;
}
