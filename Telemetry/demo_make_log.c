/* สร้างไฟล์ล็อกตัวอย่างสั้น ๆ เพื่อดูหน้าตาจริงและเอาไปลองเปิดใน MATLAB
 * ไม่ใช่ส่วนหนึ่งของเฟิร์มแวร์ รันบนโน้ตบุ๊กอย่างเดียว */
#include "telem_format.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

int main(void)
{
    static const double wp_lat[3] = { 13.65120000, 13.65569661, 13.65569661 };
    static const double wp_lon[3] = { 100.49470000, 100.49470000, 100.49932742 };

    TelemMeta_t m;
    memset(&m, 0, sizeof(m));
    m.boat_id = "SIM-01";  m.fw_version = "nav-0.5";  m.run_index = 7u;
    m.utc_start = "2026-10-09T12:35:19Z";
    m.lookahead_m = 25.0f; m.wp_radius_m = 15.0f; m.log_rate_hz = 10.0f;
    m.wp_count = 3u; m.wp_lat = wp_lat; m.wp_lon = wp_lon;

    char head[TELEM_HEADER_MAX];
    char line[TELEM_LINE_MAX];
    char name[TELEM_FILENAME_MAX];

    (void)telem_make_filename(name, sizeof(name), m.run_index);
    FILE *fp = fopen(name, "w");
    if (fp == NULL) { return 1; }

    if (telem_format_header(head, sizeof(head), &m) < 0) { return 1; }
    fputs(head, fp);

    TelemRecord_t r;
    memset(&r, 0, sizeof(r));

    /* 20 รอบแรก ยังไม่จับดาวเทียม ระบบรอ ไม่มีพิกัด */
    for (int k = 0; k < 20; ++k) {
        r.t_ms    = (uint32_t)(k * 100);
        r.mode    = (uint8_t)TELEM_MODE_STANDBY;
        r.vbat_v  = 12.60f;
        r.loop_us = 310u;
        if (telem_format_line(line, sizeof(line), &r) < 0) { return 1; }
        fputs(line, fp);
    }

    /* จากนั้นเรือออกตัวในโหมดอัตโนมัติ */
    double lat = 13.6512000, lon = 100.4947000;
    float  e_prop = 0.0f, e_steer = 0.0f, s_along = 0.0f, s_ground = 0.0f;

    for (int k = 20; k < 120; ++k) {
        double t = k * 0.1;
        float  err = (float)(3.2 * sin(t * 0.55));     /* เรือส่ายเพราะคลื่น */
        float  i_p = (float)(4.6 + 0.35 * sin(t * 1.3));
        float  i_s = (float)(0.25 + 0.55 * fabs(cos(t * 0.55)));
        float  sog = 2.30f;

        lat     += 2.0e-6;
        e_prop  += (float)(12.5 * i_p * 0.1 / 3600.0);
        e_steer += (float)(12.5 * i_s * 0.1 / 3600.0);
        s_ground += sog * 0.1f;
        s_along  += sog * 0.1f * (float)cos(err * 3.14159265 / 180.0);

        r.t_ms      = (uint32_t)(k * 100);
        r.utc_sod   = 45319.0f + (float)t;  r.utc_valid = true;
        r.nav_valid = true;
        r.lat_deg   = lat;  r.lon_deg = lon;
        r.sog_mps   = sog;
        r.cog_deg   = 41.0f + err;  r.cog_valid = true;
        r.heading_deg         = 41.0f + err;
        r.yaw_rate_dps        = (float)(1.8 * cos(t * 0.55));
        r.desired_heading_deg = 41.0f;
        r.heading_error_deg   = -err;
        r.cross_track_error_m = err * 0.6f;
        r.dist_to_wp_m        = 500.0f - s_along;
        r.wp_index = 1u; r.fix_type = 2u; r.sat_count = 11u;
        r.hdop = 0.9f; r.gps_age_ms = (uint32_t)((k % 10) * 100);

        r.mode = (uint8_t)TELEM_MODE_AUTO;
        r.rudder_cmd_deg  = -err * 1.9f;
        r.rudder_meas_deg = -err * 1.9f + 0.4f;  r.rudder_meas_valid = true;
        r.kp = 1.85f + 0.12f * (float)fabs(err);
        r.ki = 0.042f;
        r.kd = 0.61f;
        r.gains_valid = true;
        r.ctrl_flags  = (fabsf(err) > 3.0f) ? TELEM_F_RUD_SAT : 0u;

        r.vbat_v = 12.50f;
        r.i_prop_a = i_p;  r.i_prop_valid = true;
        r.i_steer_a = i_s; r.i_steer_valid = true;
        r.e_prop_wh = e_prop; r.e_steer_wh = e_steer;
        r.s_along_m = s_along; r.s_ground_m = s_ground;
        r.loop_us = (uint16_t)(900 + (k % 7) * 11);

        if (telem_format_line(line, sizeof(line), &r) < 0) { return 1; }
        fputs(line, fp);
    }

    fclose(fp);
    printf("wrote %s\n", name);
    return 0;
}
