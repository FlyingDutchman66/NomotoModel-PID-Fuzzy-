/**
 * test_telem_accum.c
 * ---------------------------------------------------------------------------
 * ชุดทดสอบของตัวสะสมและริงบัฟเฟอร์ รันบนโน้ตบุ๊กหรือ OnlineGDB
 *
 * ค่าที่คาดหวังทุกตัวในไฟล์นี้คำนวณด้วยมือจากฟิสิกส์ ไม่ได้ลอกจากผลรันครั้งแรก
 * ซึ่งเป็นเรื่องสำคัญ เพราะเทสต์ที่ลอกค่าจากโค้ดจะผ่านเสมอแม้โค้ดจะผิด
 *
 * ตัวอย่างการคำนวณด้วยมือที่ใช้ซ้ำหลายข้อ
 *   แรงดัน 12.5 V กระแส 4.0 A ได้กำลัง 50 W
 *   50 W เป็นเวลา 60 s ได้พลังงาน 3000 W*s
 *   3000 หารด้วย 3600 ได้ 0.833333 Wh
 * ---------------------------------------------------------------------------
 */

#include "telem_accum.h"
#include "telem_ring.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

static int g_failed = 0;
static int g_total  = 0;

static void check_bool(const char *label, bool got, bool want)
{
    ++g_total;
    bool ok = (got == want);
    if (!ok) { ++g_failed; }
    printf("[%s] %-56s got=%-5s want=%s\n",
           ok ? "PASS" : "FAIL", label,
           got ? "true" : "false", want ? "true" : "false");
}

static void check_u32(const char *label, uint32_t got, uint32_t want)
{
    ++g_total;
    bool ok = (got == want);
    if (!ok) { ++g_failed; }
    printf("[%s] %-56s got=%-11lu want=%lu\n",
           ok ? "PASS" : "FAIL", label,
           (unsigned long)got, (unsigned long)want);
}

static void check_near(const char *label, double got, double want, double tol)
{
    ++g_total;
    bool ok = (fabs(got - want) <= tol);
    if (!ok) { ++g_failed; }
    printf("[%s] %-56s got=%-11.6f want=%.6f\n",
           ok ? "PASS" : "FAIL", label, got, want);
}

/* ชุดข้อมูลเข้าที่ทุกอย่างใช้ได้ เรือวิ่งนิ่ง ๆ ที่กระแสคงที่ */
static TelemAccumIn_t steady(uint32_t t_ms)
{
    TelemAccumIn_t in;
    memset(&in, 0, sizeof(in));
    in.t_ms         = t_ms;
    in.vbat_v       = 12.5f;
    in.i_prop_a     = 4.0f;
    in.i_prop_valid = true;
    in.i_steer_a    = 0.8f;
    in.i_steer_valid = true;
    in.sog_mps      = 2.0f;
    in.sog_valid    = true;
    in.along_m      = 0.0f;
    in.along_valid  = false;
    in.wp_index     = 0u;
    return in;
}

int main(void)
{
    printf("=== 1. constant power integrates to the hand-computed energy ===\n");
    {
        TelemAccum_t a;
        telem_accum_init(&a);

        /* 601 ตัวอย่างที่ห่างกัน 100 ms คือ 600 ช่วง รวม 60 วินาที */
        for (uint32_t k = 0u; k <= 600u; ++k) {
            TelemAccumIn_t in = steady(k * 100u);
            telem_accum_update(&a, &in);
        }

        TelemRecord_t r;
        memset(&r, 0, sizeof(r));
        telem_accum_apply(&a, &r);

        check_u32("sees exactly sixty seconds", a.total_ms, 60000u);
        check_near("  propulsion energy at 50 W for 60 s",
                   r.e_prop_wh, 0.833333, 1e-4);
        check_near("  steering energy at 10 W for 60 s",
                   r.e_steer_wh, 0.166667, 1e-4);
        check_near("  ground track at 2 m/s for 60 s",
                   r.s_ground_m, 120.0, 0.01);
        check_u32("no gaps", a.gap_ms, 0u);
        check_u32("no blind propulsion time", a.prop_blind_ms, 0u);
        check_u32("along-track time all blind, since none was supplied",
                  a.along_blind_ms, 60000u);
    }

    printf("\n=== 2. a linear ramp is where the trapezoid earns its place ===\n");
    {
        /* กระแสไต่จาก 0 ถึง 8 A เป็นเส้นตรงใน 60 วินาที
         * ค่าเฉลี่ยคือ 4 A ซึ่งให้พลังงานเท่ากับข้อแรกเป๊ะ ๆ
         * กฎสี่เหลี่ยมคางหมูให้คำตอบที่ถูกต้องสมบูรณ์เมื่อสัญญาณเป็นเส้นตรง
         * ถ้าใครเปลี่ยนไปใช้ค่าล่าสุดคูณคาบเวลา ข้อนี้จะคลาดไปราวครึ่งหนึ่ง
         * ของหนึ่งขั้น และตัวเลขจะเอนไปทางเดียวอย่างเป็นระบบ */
        TelemAccum_t a;
        telem_accum_init(&a);

        for (uint32_t k = 0u; k <= 600u; ++k) {
            TelemAccumIn_t in = steady(k * 100u);
            in.i_prop_a = 8.0f * (float)k / 600.0f;
            telem_accum_update(&a, &in);
        }

        check_near("ramp integrates to the same energy as its mean",
                   a.e_prop_wh, 0.833333, 1e-4);
    }

    printf("\n=== 3. a stalled loop is clamped and the lost time is counted ===\n");
    {
        TelemAccum_t a;
        telem_accum_init(&a);

        TelemAccumIn_t in = steady(0u);
        telem_accum_update(&a, &in);

        /* กระโดดไป 5 วินาที ซึ่งเกินเพดาน 500 ms สิบเท่า */
        in = steady(5000u);
        telem_accum_update(&a, &in);

        check_u32("only the ceiling is integrated", a.total_ms, 500u);
        check_u32("  the rest is recorded as a gap", a.gap_ms, 4500u);
        check_near("  energy counts half a second, not five",
                   a.e_prop_wh, 50.0 * 0.5 / 3600.0, 1e-6);
        check_near("  distance counts half a second too",
                   a.s_ground_m, 1.0, 1e-4);
    }

    printf("\n=== 4. a missing sensor never becomes a silent zero ===\n");
    {
        TelemAccum_t a;
        telem_accum_init(&a);

        /* เรือที่ยังไม่ได้ติดเซ็นเซอร์กระแสขับเคลื่อน แต่ติดของหางเสือแล้ว */
        for (uint32_t k = 0u; k <= 100u; ++k) {
            TelemAccumIn_t in = steady(k * 100u);
            in.i_prop_valid = false;
            telem_accum_update(&a, &in);
        }

        check_near("propulsion energy stays at zero", a.e_prop_wh, 0.0, 1e-9);
        check_u32("  and the blind time says why", a.prop_blind_ms, 10000u);
        check_near("  steering energy still accumulates",
                   a.e_steer_wh, 10.0 * 10.0 / 3600.0, 1e-5);

        float frac = -1.0f;
        check_bool("blind fraction is available",
                   telem_accum_blind_fraction(&a, &frac), true);
        check_near("  and it is the whole run", (double)frac, 1.0, 1e-6);
    }

    printf("\n=== 5. switching legs must not erase the distance already made ===\n");
    {
        /* นี่คือข้อที่สำคัญที่สุดในไฟล์นี้
         * ถ้าเอาผลต่างของพิกัดตามเส้นทางมาบวกตรง ๆ โดยไม่ระวังการสลับเลก
         * ระยะคืบหน้าสะสมจะได้ 80 เมตรแทนที่จะเป็น 180 เมตร
         * เพราะความยาวเลกแรกถูกหักออกตอนค่ากระโดดกลับไปศูนย์ */
        TelemAccum_t a;
        telem_accum_init(&a);

        TelemAccumIn_t in = steady(0u);
        in.along_valid = true;
        in.along_m     = 0.0f;
        in.wp_index    = 0u;
        telem_accum_update(&a, &in);

        for (uint32_t k = 1u; k <= 100u; ++k) {
            in = steady(k * 100u);
            in.along_valid = true;
            in.along_m     = (float)k;
            in.wp_index    = 0u;
            telem_accum_update(&a, &in);
        }
        check_near("first leg accumulates its hundred metres",
                   a.s_along_m, 100.0, 1e-3);

        /* ข้าม waypoint พิกัดตามเส้นทางเริ่มนับใหม่จากศูนย์ */
        in = steady(10100u);
        in.along_valid = true;
        in.along_m     = 0.0f;
        in.wp_index    = 1u;
        telem_accum_update(&a, &in);

        check_near("  the leg change adds nothing and subtracts nothing",
                   a.s_along_m, 100.0, 1e-3);
        check_u32("  and it is counted", a.leg_changes, 1u);

        for (uint32_t k = 1u; k <= 80u; ++k) {
            in = steady(10100u + k * 100u);
            in.along_valid = true;
            in.along_m     = (float)k;
            in.wp_index    = 1u;
            telem_accum_update(&a, &in);
        }
        check_near("second leg adds on top, never replaces",
                   a.s_along_m, 180.0, 1e-3);
    }

    printf("\n=== 6. the boat pushed backwards makes negative progress ===\n");
    {
        /* ถ้าคลื่นผลักเรือถอยหลังจริง การบีบค่าให้เป็นศูนย์คือการโกงตัวชี้วัด
         * เพราะพลังงานยังถูกใช้ไปแล้วแต่ระยะคืบหน้าถูกปั้นให้ไม่ลด */
        TelemAccum_t a;
        telem_accum_init(&a);

        TelemAccumIn_t in = steady(0u);
        in.along_valid = true; in.along_m = 50.0f; in.wp_index = 0u;
        telem_accum_update(&a, &in);

        in = steady(100u);
        in.along_valid = true; in.along_m = 47.5f; in.wp_index = 0u;
        telem_accum_update(&a, &in);

        check_near("losing ground is recorded as a loss",
                   a.s_along_m, -2.5, 1e-3);
    }

    printf("\n=== 7. the millisecond counter rolling over changes nothing ===\n");
    {
        /* t_ms เป็น uint32 จึงวนรอบทุก 49.7 วัน การลบแบบ unsigned
         * ให้ผลต่างที่ถูกต้องข้ามรอยต่อโดยไม่ต้องเขียนเงื่อนไขพิเศษ */
        TelemAccum_t a;
        telem_accum_init(&a);

        const uint32_t start = 0xFFFFFF00u;   /* เหลือ 256 ms ก่อนวนรอบ */
        for (uint32_t k = 0u; k <= 100u; ++k) {
            TelemAccumIn_t in = steady(start + k * 100u);
            telem_accum_update(&a, &in);
        }

        check_u32("ten seconds measured across the rollover",
                  a.total_ms, 10000u);
        check_u32("  with no stall falsely reported", a.gap_ms, 0u);
        check_near("  and energy unaffected",
                   a.e_prop_wh, 50.0 * 10.0 / 3600.0, 1e-5);
    }

    printf("\n=== 8. being called twice in the same millisecond ===\n");
    {
        TelemAccum_t a;
        telem_accum_init(&a);

        TelemAccumIn_t in = steady(0u);
        telem_accum_update(&a, &in);
        in = steady(100u);
        telem_accum_update(&a, &in);

        const float e_once = a.e_prop_wh;
        telem_accum_update(&a, &in);   /* เวลาเดิมซ้ำ */
        telem_accum_update(&a, &in);

        check_near("energy is not counted twice",
                   a.e_prop_wh, (double)e_once, 1e-9);
        check_u32("  and neither is time", a.total_ms, 100u);
    }

    printf("\n=== 9. the headline metric ===\n");
    {
        TelemAccum_t a;
        telem_accum_init(&a);

        /* 60 วินาทีที่ 50 W ขับเคลื่อนและ 10 W หางเสือ รวม 60 W
         * พลังงานรวม 60 * 60 / 3600 = 1.0 Wh
         * ระยะคืบหน้า 120 เมตร ดังนั้น 1.0 Wh ต่อ 0.12 km = 8.3333 Wh/km */
        for (uint32_t k = 0u; k <= 600u; ++k) {
            TelemAccumIn_t in = steady(k * 100u);
            in.along_valid = true;
            in.along_m     = 0.2f * (float)k;   /* 2 m/s ที่ 100 ms */
            in.wp_index    = 0u;
            telem_accum_update(&a, &in);
        }

        check_near("along-track progress", a.s_along_m, 120.0, 0.01);
        check_near("total energy", a.e_prop_wh + a.e_steer_wh, 1.0, 1e-4);

        float whkm = -1.0f;
        check_bool("metric is available", telem_accum_wh_per_km(&a, 10.0f, &whkm), true);
        check_near("  watt-hours per along-track kilometre",
                   (double)whkm, 8.3333, 1e-2);

        TelemAccum_t tiny;
        telem_accum_init(&tiny);
        check_bool("metric refuses to divide by a standstill",
                   telem_accum_wh_per_km(&tiny, 10.0f, &whkm), false);
    }

    printf("\n=== 10. the ring buffer keeps order and never blocks ===\n");
    {
        TelemRing_t q;
        telem_ring_init(&q);

        check_u32("starts empty", telem_ring_count(&q), 0u);
        check_bool("  and is not full", telem_ring_is_full(&q), false);

        TelemRecord_t out;
        check_bool("popping an empty queue fails cleanly",
                   telem_ring_pop(&q, &out), false);

        for (uint32_t k = 0u; k < 10u; ++k) {
            TelemRecord_t r;
            memset(&r, 0, sizeof(r));
            r.t_ms = 1000u + k;
            check_bool("push succeeds while there is room",
                       telem_ring_push(&q, &r), true);
        }
        check_u32("ten records waiting", telem_ring_count(&q), 10u);

        bool order_ok = true;
        for (uint32_t k = 0u; k < 10u; ++k) {
            if (!telem_ring_pop(&q, &out) || out.t_ms != 1000u + k) {
                order_ok = false;
            }
        }
        check_bool("oldest comes out first, in order", order_ok, true);
        check_u32("and the queue is empty again", telem_ring_count(&q), 0u);
    }

    printf("\n=== 11. an SD card that stalls for too long ===\n");
    {
        TelemRing_t q;
        telem_ring_init(&q);

        /* เขียนจนเต็มความจุพอดี */
        for (uint32_t k = 0u; k < TELEM_RING_CAP; ++k) {
            TelemRecord_t r;
            memset(&r, 0, sizeof(r));
            r.t_ms = k;
            (void)telem_ring_push(&q, &r);
        }
        check_u32("filled to capacity", telem_ring_count(&q), TELEM_RING_CAP);
        check_bool("  and reports itself full", telem_ring_is_full(&q), true);
        check_u32("  high water mark reached capacity",
                  q.high_water, TELEM_RING_CAP);
        check_u32("  nothing dropped yet", q.drops, 0u);

        /* การ์ดยังค้าง อีกห้าเรคคอร์ดถูกทิ้ง */
        for (uint32_t k = 0u; k < 5u; ++k) {
            TelemRecord_t r;
            memset(&r, 0, sizeof(r));
            r.t_ms = 9000u + k;
            check_bool("push is refused rather than blocking",
                       telem_ring_push(&q, &r), false);
        }
        check_u32("five records dropped and counted", q.drops, 5u);
        check_u32("  the queue is unchanged", telem_ring_count(&q),
                  TELEM_RING_CAP);

        /* ข้อมูลเก่าที่เก็บไว้แล้วต้องไม่ถูกข้อมูลใหม่ทำลาย */
        TelemRecord_t out;
        check_bool("the oldest record survived the overflow",
                   telem_ring_pop(&q, &out) && out.t_ms == 0u, true);
    }

    printf("\n=== 12. a thousand cycles around the ring ===\n");
    {
        /* อาการที่เทสต์นี้เฝ้าไว้คือดัชนีที่วนไม่ถูก ซึ่งจะโผล่หลังผ่านรอยต่อ
         * ไม่ใช่ตอนเริ่ม การวิ่งพันรอบจึงผ่านรอยต่อหลายสิบครั้ง */
        TelemRing_t q;
        telem_ring_init(&q);

        bool ok = true;
        for (uint32_t k = 0u; k < 1000u; ++k) {
            TelemRecord_t r;
            memset(&r, 0, sizeof(r));
            r.t_ms = k;
            if (!telem_ring_push(&q, &r)) { ok = false; }

            TelemRecord_t out;
            if (!telem_ring_pop(&q, &out) || out.t_ms != k) { ok = false; }
        }
        check_bool("every record came back exactly once, in order", ok, true);
        check_u32("nothing was ever dropped", q.drops, 0u);
        check_u32("the queue drained completely", telem_ring_count(&q), 0u);
    }

    printf("\n=== 13. the memory this costs on the board ===\n");
    {
        /* ไม่ใช่การทดสอบความถูกต้อง แต่คือตัวเลขที่ต้องตอบได้ตอนสอบ */
        printf("       sizeof(TelemRecord_t) = %lu bytes\n",
               (unsigned long)sizeof(TelemRecord_t));
        printf("       sizeof(TelemRing_t)   = %lu bytes  (%lu slots)\n",
               (unsigned long)sizeof(TelemRing_t),
               (unsigned long)TELEM_RING_SLOTS);
        printf("       sizeof(TelemAccum_t)  = %lu bytes\n",
               (unsigned long)sizeof(TelemAccum_t));
        const double secs = (double)TELEM_RING_CAP / 10.0;
        printf("       queue cushion at 10 Hz = %.1f seconds\n", secs);
        check_bool("the cushion covers a quarter-second card stall",
                   secs >= 1.0, true);
        check_bool("and the queue fits a modest RAM budget",
                   sizeof(TelemRing_t) < 16384u, true);
    }

    printf("\n=== 14. does float survive a two-hour trip ===\n");
    {
        /* ข้อ 1 ได้ระยะทาง 119.9993 แทนที่จะเป็น 120 พอดี นั่นคือการปัดเศษ
         * ของ float 32 บิตที่สะสมจากการบวก 600 ครั้ง คำถามที่ต้องตอบคือ
         * พอวิ่งจริงสองชั่วโมงซึ่งเป็นการบวก 72000 ครั้ง ความคลาดจะโตแค่ไหน
         *
         * เหตุผลที่คาดว่ายังปลอดภัยคือขนาดของขั้นเทียบกับขนาดของผลรวม
         * ที่พลังงานสะสม 100 Wh ความละเอียดของ float อยู่ราว 1.2e-5 Wh
         * ขณะที่แต่ละขั้นคือ 1.4e-3 Wh ซึ่งใหญ่กว่าร้อยเท่า จึงไม่มีขั้นไหน
         * ถูกปัดหายทั้งก้อน ความคลาดจึงเดินแบบสุ่มไม่ใช่สะสมทางเดียว
         * แต่การคาดคะเนไม่ใช่หลักฐาน ข้อนี้จึงวัดจริง */
        TelemAccum_t a;
        telem_accum_init(&a);

        const uint32_t n = 72000u;          /* 2 ชั่วโมงที่ 10 Hz */
        for (uint32_t k = 0u; k <= n; ++k) {
            TelemAccumIn_t in = steady(k * 100u);
            telem_accum_update(&a, &in);
        }

        const double hours    = (double)n * 0.1 / 3600.0;
        const double want_e   = 50.0 * hours;              /* 100 Wh */
        const double want_s   = 2.0 * (double)n * 0.1;     /* 14400 m */
        const double err_e    = fabs((double)a.e_prop_wh - want_e) / want_e;
        const double err_s    = fabs((double)a.s_ground_m - want_s) / want_s;

        printf("       energy   %.4f Wh vs %.4f exact, relative error %.2e\n",
               (double)a.e_prop_wh, want_e, err_e);
        printf("       distance %.2f m vs %.2f exact, relative error %.2e\n",
               (double)a.s_ground_m, want_s, err_s);

        /* เกณฑ์ 0.1 เปอร์เซ็นต์ เพราะความคลาดของเซ็นเซอร์กระแสราคาถูก
         * อยู่ในระดับ 1 ถึง 3 เปอร์เซ็นต์ การปัดเศษต้องเล็กกว่านั้นอย่างน้อย
         * สิบเท่าจึงจะไม่ใช่ตัวจำกัดความแม่นของบทวิเคราะห์ */
        check_bool("energy drift stays under 0.1 percent", err_e < 1e-3, true);
        check_bool("distance drift stays under 0.1 percent", err_s < 1e-3, true);
    }

    printf("\n-----------------------------------------------\n");
    printf("%d checks, %d failed\n", g_total, g_failed);
    printf("%s\n", (g_failed == 0) ? "PASS : every check held" : "FAIL");
    return (g_failed == 0) ? 0 : 1;
}
