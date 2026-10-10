/**
 * demo_run.c
 * ---------------------------------------------------------------------------
 * การทดสอบแบบประกอบร่าง ขับทั้งสี่โมดูลพร้อมกันเหมือนบนเรือจริง
 *
 *   telem_accum  สะสมพลังงานและระยะทาง
 *   telem_ring   กั้นระหว่างลูปควบคุมกับการ์ด
 *   telem_format แปลงเป็นข้อความ
 *   telem_types  สัญญาที่ทั้งสามตัวใช้ร่วมกัน
 *
 * ลำดับในลูปด้านล่างคือลำดับเดียวกับที่เฟิร์มแวร์จริงต้องทำ
 *   วัดค่า แล้วป้อนตัวสะสม แล้วเติมค่าสะสมลงเรคคอร์ด แล้วหยอดคิว
 *   งานเขียนการ์ดอยู่คนละที่ หยิบจากคิวเมื่อว่าง
 *
 * ข้อมูลในนี้เป็นข้อมูลสังเคราะห์ ไม่ใช่การวัดจากเรือ จุดประสงค์คือพิสูจน์ว่า
 * ท่อลำเลียงข้อมูลทำงานครบวงจร และเพื่อให้มีไฟล์จริงไปลองเขียนบทวิเคราะห์
 * ตัวเลขที่ออกมาจึงใช้ตรวจสอบโปรแกรมได้ แต่ใช้อ้างผลการทดลองไม่ได้
 *
 * แบบจำลองที่ใช้ตั้งใจให้ยุติธรรมกับทั้งสองโหมด ไม่ได้ปรับให้ออโตไพลอตชนะ
 *   โหมดมือ      หัวเรือแกว่างกว้างเพราะคนเหนื่อย แต่มอเตอร์หางเสือไม่ทำงาน
 *                เพราะคนหมุนพวงมาลัยเอง จึงแทบไม่กินไฟ
 *   โหมดอัตโนมัติ หัวเรือนิ่งกว่ามาก แต่มอเตอร์หางเสือทำงานตลอดเวลา
 *                จึงมีต้นทุนพลังงานของตัวเองที่ต้องนับเข้าไปด้วย
 *
 * กำลังขับเคลื่อนคงที่ทั้งสองโหมดเพราะคันเร่งไม่ขยับ สิ่งที่ต่างกันคือ
 * ความเร็วที่ลดลงตามแรงต้านของหางเสือที่เอียง และระยะคืบหน้าที่ลดลง
 * ตามโคไซน์ของมุมที่หัวเรือเบนออกจากเส้นทาง
 * ---------------------------------------------------------------------------
 */

#include "telem_accum.h"
#include "telem_ring.h"
#include "telem_format.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#define DEG2RAD   0.017453292519943295

/* --- พารามิเตอร์ของแบบจำลอง รวมไว้ที่เดียวเพื่อให้เห็นว่าอะไรถูกสมมติ --- */
#define V_BUS         12.5f    /* แรงดันบัส */
#define I_PROP_A      4.60f    /* กระแสขับเคลื่อน คันเร่งคงที่ */
#define SOG_NOM       2.40f    /* ความเร็วเมื่อหางเสือตรง */
#define DRAG_PER_DEG  0.0040f  /* สัดส่วนความเร็วที่เสียต่อหนึ่งองศาของหางเสือ */
#define I_STEER_IDLE  0.040f   /* กระแสหางเสือตอนมอเตอร์ไม่ทำงาน */

static const double wp_lat[2] = { 13.65120000, 13.66000000 };
static const double wp_lon[2] = { 100.49470000, 100.49470000 };

/* ความคลาดของหัวเรือในแต่ละโหมด หน่วยองศา
 * โหมดมือใช้ผลรวมของคลื่นไซน์คาบยาวบวกการล้าที่โตขึ้นตามเวลา
 * โหมดอัตโนมัติใช้คลื่นคาบสั้นที่แอมพลิจูดเล็กกว่าราวสี่เท่า */
static double heading_err_deg(bool automatic, double t)
{
    if (automatic) {
        return 2.40 * sin(0.55 * t) + 0.80 * sin(1.70 * t + 0.7);
    }
    const double fatigue = 1.0 + 0.0020 * t;   /* คนเริ่มล้า แกว่างกว้างขึ้น */
    return fatigue * (9.0 * sin(0.090 * t) + 5.0 * sin(0.230 * t + 1.1));
}

int main(void)
{
    TelemMeta_t meta;
    memset(&meta, 0, sizeof(meta));
    meta.boat_id     = "SIM-01";
    meta.fw_version  = "telem-0.3";
    meta.run_index   = 11u;
    meta.utc_start   = "2026-10-10T04:00:00Z";
    meta.lookahead_m = 25.0f;
    meta.wp_radius_m = 15.0f;
    meta.log_rate_hz = 10.0f;
    meta.wp_count    = 2u;
    meta.wp_lat      = wp_lat;
    meta.wp_lon      = wp_lon;

    char name[TELEM_FILENAME_MAX];
    char head[TELEM_HEADER_MAX];
    char line[TELEM_LINE_MAX];

    (void)telem_make_filename(name, sizeof(name), meta.run_index);
    FILE *fp = fopen(name, "w");
    if (fp == NULL) { return 1; }
    if (telem_format_header(head, sizeof(head), &meta) < 0) { return 1; }
    fputs(head, fp);

    TelemRing_t  ring;
    TelemAccum_t whole;     /* สะสมทั้งเที่ยว ค่านี้ลงไฟล์ */
    TelemAccum_t manual;    /* สะสมเฉพาะโหมดมือ สำหรับสรุปท้ายโปรแกรม */
    TelemAccum_t automatic; /* สะสมเฉพาะโหมดอัตโนมัติ */

    telem_ring_init(&ring);
    telem_accum_init(&whole);
    telem_accum_init(&manual);
    telem_accum_init(&automatic);

    /* ช่วงเวลาของการทดลอง หน่วยตัวอย่างที่ 10 Hz */
    const uint32_t n_standby = 300u;    /* 30 วินาที ยังไม่จับดาวเทียม */
    const uint32_t n_manual  = 1500u;   /* 150 วินาที ขับด้วยมือ */
    const uint32_t n_auto    = 1500u;   /* 150 วินาที ออโตไพลอต */
    const uint32_t n_total   = n_standby + n_manual + n_auto;

    double lat = wp_lat[0], lon = wp_lon[0];
    double along_m = 0.0;
    float  rud_prev = 0.0f;

    uint32_t written = 0u, popped = 0u;

    for (uint32_t k = 0u; k < n_total; ++k) {
        const uint32_t t_ms = k * 100u;
        const bool in_standby = (k < n_standby);
        const bool is_auto    = (k >= n_standby + n_manual);
        const double t_mode   = in_standby ? 0.0
                              : (is_auto ? (double)(k - n_standby - n_manual) * 0.1
                                         : (double)(k - n_standby) * 0.1);

        TelemRecord_t r;
        memset(&r, 0, sizeof(r));
        r.t_ms   = t_ms;
        r.vbat_v = V_BUS;

        TelemAccumIn_t in;
        memset(&in, 0, sizeof(in));
        in.t_ms   = t_ms;
        in.vbat_v = V_BUS;

        if (in_standby) {
            /* รอจับดาวเทียม ไม่มีพิกัด ไม่มีการเคลื่อนที่ ไม่มีกระแสขับเคลื่อน */
            r.mode    = (uint8_t)TELEM_MODE_STANDBY;
            r.loop_us = 320u;
            r.i_steer_a = I_STEER_IDLE; r.i_steer_valid = true;
            r.i_prop_valid = true;      /* เซ็นเซอร์มี แต่กระแสเป็นศูนย์ */

            in.i_prop_a = 0.0f;  in.i_prop_valid = true;
            in.i_steer_a = I_STEER_IDLE; in.i_steer_valid = true;
            in.sog_valid = false;
            in.along_valid = false;
        } else {
            const double err  = heading_err_deg(is_auto, t_mode);
            /* มุมหางเสือที่ใช้แก้ ออโตไพลอตตอบสนองแรงกว่าคนราวสองเท่า */
            const double rud  = (is_auto ? -1.90 : -0.80) * err;
            const double sog  = (double)SOG_NOM
                              * (1.0 - (double)DRAG_PER_DEG * fabs(rud));
            /* ระยะคืบหน้าตามเส้นทางคือองค์ประกอบของความเร็วที่ขนานกับเส้นทาง */
            const double d_along = sog * 0.1 * cos(err * DEG2RAD);
            along_m += d_along;
            lat     += d_along / 111194.0;

            /* กระแสหางเสือ โหมดมือมอเตอร์ไม่ทำงาน โหมดอัตโนมัติทำงานตามอัตราการหมุน */
            float i_steer;
            if (is_auto) {
                const double rate = fabs(rud - (double)rud_prev) / 0.1;  /* องศาต่อวินาที */
                i_steer = (float)(0.20 + 0.075 * rate);
                if (i_steer > 2.50f) { i_steer = 2.50f; }
            } else {
                i_steer = I_STEER_IDLE;
            }
            rud_prev = (float)rud;

            r.utc_sod   = 14400.0f + (float)(t_ms / 1000.0);
            r.utc_valid = true;
            r.nav_valid = true;
            r.lat_deg   = lat;
            r.lon_deg   = lon;
            r.sog_mps   = (float)sog;
            r.cog_deg   = (float)(0.0 + err);
            r.cog_valid = true;
            r.heading_deg         = (float)(0.0 + err);
            r.yaw_rate_dps        = 0.0f;
            r.desired_heading_deg = 0.0f;
            r.heading_error_deg   = (float)(-err);
            r.cross_track_error_m = (float)(0.6 * err);
            r.dist_to_wp_m        = (float)(900.0 - along_m);
            r.wp_index  = 1u;
            r.fix_type  = 2u;
            r.sat_count = 11u;
            r.hdop      = 0.9f;
            r.gps_age_ms = (k % 10u) * 100u;

            r.mode              = is_auto ? (uint8_t)TELEM_MODE_AUTO
                                          : (uint8_t)TELEM_MODE_MANUAL;
            r.rudder_cmd_deg    = (float)rud;
            r.rudder_meas_deg   = (float)rud + 0.35f;
            r.rudder_meas_valid = true;
            if (is_auto) {
                r.kp = (float)(1.85 + 0.12 * fabs(err));
                r.ki = 0.042f;
                r.kd = 0.610f;
                r.gains_valid = true;
            }
            r.ctrl_flags = (fabs(rud) > 20.0) ? TELEM_F_RUD_SAT : 0u;
            r.i_prop_a      = I_PROP_A;  r.i_prop_valid  = true;
            r.i_steer_a     = i_steer;   r.i_steer_valid = true;
            r.loop_us       = (uint16_t)(900u + (k % 7u) * 11u);

            in.i_prop_a  = I_PROP_A;  in.i_prop_valid  = true;
            in.i_steer_a = i_steer;   in.i_steer_valid = true;
            in.sog_mps   = (float)sog; in.sog_valid     = true;
            in.along_m   = (float)along_m;
            in.along_valid = true;
            in.wp_index  = 1u;
        }

        /* ---- ลำดับจริงในลูปควบคุม ---- */
        telem_accum_update(&whole, &in);
        telem_accum_apply(&whole, &r);
        r.drop_count = ring.drops;

        if (!in_standby) {
            telem_accum_update(is_auto ? &automatic : &manual, &in);
        }

        (void)telem_ring_push(&ring, &r);

        /* ---- งานเขียนการ์ด อยู่คนละที่กับลูปควบคุม ----
         * จำลองการ์ดที่ค้าง 4 วินาทีระหว่างทาง เพื่อดูว่าคิวรับไหวไหม
         * และถ้าไม่ไหว ตัวนับบอกความจริงหรือเปล่า */
        const bool card_stalled = (k >= 2000u && k < 2040u);
        if (!card_stalled) {
            TelemRecord_t out;
            /* ปกติเขียนได้เร็วกว่าที่ข้อมูลเข้ามา จึงระบายได้หลายก้อนต่อรอบ */
            for (int i = 0; i < 3; ++i) {
                if (!telem_ring_pop(&ring, &out)) { break; }
                if (telem_format_line(line, sizeof(line), &out) < 0) { return 1; }
                fputs(line, fp);
                ++popped;
            }
        }
        ++written;
    }

    /* ระบายคิวที่เหลือตอนปิดไฟล์ */
    {
        TelemRecord_t out;
        while (telem_ring_pop(&ring, &out)) {
            if (telem_format_line(line, sizeof(line), &out) < 0) { return 1; }
            fputs(line, fp);
            ++popped;
        }
    }
    fclose(fp);

    /* ---------------- สรุป ---------------- */
    printf("wrote %s\n", name);
    printf("samples produced %lu, records written %lu, dropped %lu, high water %lu of %lu\n",
           (unsigned long)written, (unsigned long)popped,
           (unsigned long)ring.drops, (unsigned long)ring.high_water,
           (unsigned long)TELEM_RING_CAP);

    float wh_m = 0.0f, wh_a = 0.0f;
    const bool ok_m = telem_accum_wh_per_km(&manual,    10.0f, &wh_m);
    const bool ok_a = telem_accum_wh_per_km(&automatic, 10.0f, &wh_a);

    printf("\n%-10s %9s %9s %9s %9s %9s\n",
           "mode", "E_prop", "E_steer", "s_along", "s_ground", "Wh/km");
    printf("%-10s %9.4f %9.4f %9.2f %9.2f %9.3f\n", "manual",
           manual.e_prop_wh, manual.e_steer_wh,
           manual.s_along_m, manual.s_ground_m, ok_m ? wh_m : -1.0f);
    printf("%-10s %9.4f %9.4f %9.2f %9.2f %9.3f\n", "auto",
           automatic.e_prop_wh, automatic.e_steer_wh,
           automatic.s_along_m, automatic.s_ground_m, ok_a ? wh_a : -1.0f);

    /* ตัวเลขชุดที่สองคือพลังงานขับเคลื่อนเพียงอย่างเดียว
     * ซึ่งเป็นตัวแทนที่ใกล้เคียงกับเรือขนาดจริง เพราะบนเรือจริง
     * พลังงานของมอเตอร์หางเสือเล็กจนไม่มีผลเทียบกับเครื่องยนต์ */
    const float pk_m = manual.e_prop_wh    * 1000.0f / manual.s_along_m;
    const float pk_a = automatic.e_prop_wh * 1000.0f / automatic.s_along_m;
    printf("\npropulsion only, Wh per along-track km : manual %.3f, auto %.3f  (%+.2f %%)\n",
           pk_m, pk_a, 100.0f * (pk_a - pk_m) / pk_m);

    const float wv_m = 100.0f * (manual.s_ground_m - manual.s_along_m)
                     / manual.s_along_m;
    const float wv_a = 100.0f * (automatic.s_ground_m - automatic.s_along_m)
                     / automatic.s_along_m;
    printf("weave penalty : manual %.2f %%, auto %.2f %%\n", wv_m, wv_a);

    return 0;
}
