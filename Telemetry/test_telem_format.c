/**
 * test_telem_format.c
 * ---------------------------------------------------------------------------
 * ชุดทดสอบของ telem_format รันบนโน้ตบุ๊กหรือ OnlineGDB ไม่ต้องมีฮาร์ดแวร์
 *
 * สิ่งที่ชุดนี้เฝ้าไว้ เรียงตามความสำคัญ
 *   1. จำนวนคอลัมน์ต้องเท่ากันทุกบรรทัด ไม่ว่าข้อมูลจะดีหรือแย่แค่ไหน
 *   2. ช่องที่เชื่อไม่ได้ต้องว่าง ไม่ใช่ศูนย์
 *   3. บัฟเฟอร์เล็กเกินไปต้องฟ้อง และต้องไม่เขียนเลยขอบ
 *   4. บรรทัดที่ยาวที่สุดเท่าที่เป็นไปได้ต้องยังพอดีกับ TELEM_LINE_MAX
 * ---------------------------------------------------------------------------
 */

#include "telem_format.h"

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
    printf("[%s] %-54s got=%-5s want=%s\n",
           ok ? "PASS" : "FAIL", label,
           got ? "true" : "false", want ? "true" : "false");
}

static void check_int(const char *label, long got, long want)
{
    ++g_total;
    bool ok = (got == want);
    if (!ok) { ++g_failed; }
    printf("[%s] %-54s got=%-11ld want=%ld\n",
           ok ? "PASS" : "FAIL", label, got, want);
}

static void check_str(const char *label, const char *got, const char *want)
{
    ++g_total;
    bool ok = (strcmp(got, want) == 0);
    if (!ok) { ++g_failed; }
    printf("[%s] %-54s got=\"%s\" want=\"%s\"\n",
           ok ? "PASS" : "FAIL", label, got, want);
}

/* ดึงคอลัมน์ลำดับที่ idx นับจากศูนย์ ออกมาเป็นสตริง
 * คืนช่องว่างเป็นสตริงว่าง ซึ่งคือสิ่งที่เราอยากตรวจเป็นหลัก */
static void field(const char *line, int idx, char *out, size_t cap)
{
    const char *p = line;
    int cur = 0;

    while (cur < idx && *p != '\0' && *p != '\n') {
        if (*p == ',') { ++cur; }
        ++p;
    }

    size_t n = 0;
    while (*p != '\0' && *p != '\n' && *p != ',' && n + 1u < cap) {
        out[n++] = *p++;
    }
    out[n] = '\0';
}

/* เรคคอร์ดที่ทุกอย่างใช้ได้ เรือกำลังวิ่งในโหมดอัตโนมัติ */
static TelemRecord_t make_good(void)
{
    TelemRecord_t r;
    memset(&r, 0, sizeof(r));

    r.t_ms      = 123456u;
    r.utc_sod   = 45319.25f;
    r.utc_valid = true;

    r.nav_valid           = true;
    r.lat_deg             = 13.6512345;
    r.lon_deg             = 100.4947891;
    r.sog_mps             = 2.35f;
    r.cog_deg             = 41.75f;
    r.cog_valid           = true;
    r.heading_deg         = 43.10f;
    r.yaw_rate_dps        = -1.25f;
    r.desired_heading_deg = 38.40f;
    r.heading_error_deg   = -4.70f;
    r.cross_track_error_m = 1.85f;
    r.dist_to_wp_m        = 212.40f;
    r.wp_index            = 1u;
    r.fix_type            = 2u;
    r.sat_count           = 11u;
    r.hdop                = 0.9f;
    r.gps_age_ms          = 180u;

    r.mode              = (uint8_t)TELEM_MODE_AUTO;
    r.rudder_cmd_deg    = -6.25f;
    r.rudder_meas_deg   = -5.90f;
    r.rudder_meas_valid = true;
    r.kp                = 1.8500f;
    r.ki                = 0.0420f;
    r.kd                = 0.6100f;
    r.gains_valid       = true;
    r.ctrl_flags        = TELEM_F_I_FROZEN;

    r.vbat_v        = 12.46f;
    r.i_prop_a      = 4.812f;
    r.i_steer_a     = 0.735f;
    r.i_prop_valid  = true;
    r.i_steer_valid = true;
    r.e_prop_wh     = 3.2150f;
    r.e_steer_wh    = 0.1840f;

    r.s_along_m  = 287.60f;
    r.s_ground_m = 291.15f;

    r.drop_count = 0u;
    r.loop_us    = 940u;

    return r;
}

int main(void)
{
    char line[TELEM_LINE_MAX];
    char head[TELEM_HEADER_MAX];
    char f[64];

    printf("=== 1. column line ===\n");
    {
        int n = telem_format_columns(line, sizeof(line));
        check_bool("columns written", n > 0, true);
        check_int("column count matches TELEM_FIELD_COUNT",
                  telem_count_fields(line), TELEM_FIELD_COUNT);
        check_bool("ends with newline", line[n - 1] == '\n', true);
        check_bool("does not start with hash", line[0] != '#', true);

        field(line, 0, f, sizeof(f));
        check_str("  first column name", f, "t_ms");
        field(line, TELEM_FIELD_COUNT - 1, f, sizeof(f));
        check_str("  last column name", f, "loop_us");
    }

    printf("\n=== 2. a healthy record ===\n");
    {
        TelemRecord_t r = make_good();
        int n = telem_format_line(line, sizeof(line), &r);
        check_bool("line written", n > 0, true);
        check_int("field count", telem_count_fields(line), TELEM_FIELD_COUNT);
        check_bool("ends with newline", line[n - 1] == '\n', true);

        field(line, 0, f, sizeof(f));
        check_str("  t_ms", f, "123456");
        field(line, 3, f, sizeof(f));
        check_str("  lat keeps seven decimals", f, "13.6512345");
        field(line, 4, f, sizeof(f));
        check_str("  lon keeps seven decimals", f, "100.4947891");
        field(line, 21, f, sizeof(f));
        check_str("  kp keeps four decimals", f, "1.8500");
        field(line, 26, f, sizeof(f));
        check_str("  propulsion current in milliamp steps", f, "4.812");
        field(line, 28, f, sizeof(f));
        check_str("  cumulative energy", f, "3.2150");
        field(line, TELEM_FIELD_COUNT - 1, f, sizeof(f));
        check_str("  loop time is the last field", f, "940");
    }

    printf("\n=== 3. nothing is trusted yet ===\n");
    {
        TelemRecord_t r;
        memset(&r, 0, sizeof(r));
        r.t_ms = 50u;

        int n = telem_format_line(line, sizeof(line), &r);
        check_bool("line written", n > 0, true);
        check_int("field count is still complete",
                  telem_count_fields(line), TELEM_FIELD_COUNT);

        field(line, 1, f, sizeof(f));
        check_str("  utc is blank, not zero", f, "");
        field(line, 2, f, sizeof(f));
        check_str("  nav_valid is an answer, so it is written", f, "0");
        field(line, 3, f, sizeof(f));
        check_str("  lat is blank, never null island", f, "");
        field(line, 4, f, sizeof(f));
        check_str("  lon is blank, never null island", f, "");
        field(line, 6, f, sizeof(f));
        check_str("  cog is blank while stationary", f, "");
        field(line, 9, f, sizeof(f));
        check_str("  desired heading is blank", f, "");
        field(line, 20, f, sizeof(f));
        check_str("  measured rudder is blank with no encoder", f, "");
        field(line, 21, f, sizeof(f));
        check_str("  kp is blank before control sends gains", f, "");
        field(line, 26, f, sizeof(f));
        check_str("  current is blank with no sensor fitted", f, "");
        field(line, 28, f, sizeof(f));
        check_str("  cumulative energy is written as zero", f, "0.0000");
    }

    printf("\n=== 4. a boat with only one current sensor fitted ===\n");
    {
        TelemRecord_t r = make_good();
        r.i_steer_valid = false;
        r.i_steer_a     = 0.0f;

        (void)telem_format_line(line, sizeof(line), &r);
        check_int("field count", telem_count_fields(line), TELEM_FIELD_COUNT);
        field(line, 26, f, sizeof(f));
        check_str("  propulsion current still present", f, "4.812");
        field(line, 27, f, sizeof(f));
        check_str("  steering current blank", f, "");
    }

    printf("\n=== 5. the longest line that can ever occur ===\n");
    {
        TelemRecord_t r = make_good();
        r.t_ms                = 4294967295u;
        r.utc_sod             = 86399.99f;
        r.lat_deg             = -89.1234567;
        r.lon_deg             = -179.1234567;
        r.sog_mps             = -99.99f;
        r.cog_deg             = 359.99f;
        r.heading_deg         = 359.99f;
        r.yaw_rate_dps        = -999.99f;
        r.desired_heading_deg = 359.99f;
        r.heading_error_deg   = -179.99f;
        r.cross_track_error_m = -99999.99f;
        r.dist_to_wp_m        = 99999.99f;
        r.wp_index            = 255u;
        r.fix_type            = 255u;
        r.sat_count           = 255u;
        r.hdop                = 99.9f;
        r.gps_age_ms          = 4294967295u;
        r.mode                = 255u;
        r.rudder_cmd_deg      = -999.99f;
        r.rudder_meas_deg     = -999.99f;
        r.kp                  = -9999.9999f;
        r.ki                  = -9999.9999f;
        r.kd                  = -9999.9999f;
        r.ctrl_flags          = 255u;
        r.vbat_v              = -99.99f;
        r.i_prop_a            = -999.999f;
        r.i_steer_a           = -999.999f;
        r.e_prop_wh           = -9999.9999f;
        r.e_steer_wh          = -9999.9999f;
        r.s_along_m           = -999999.99f;
        r.s_ground_m          = -999999.99f;
        r.drop_count          = 4294967295u;
        r.loop_us             = 65535u;

        int n = telem_format_line(line, sizeof(line), &r);
        check_bool("worst case still fits TELEM_LINE_MAX", n > 0, true);
        check_int("field count", telem_count_fields(line), TELEM_FIELD_COUNT);
        printf("       worst case length = %d bytes, budget = %d\n",
               n, TELEM_LINE_MAX - 1);
        check_bool("at least 20 percent headroom left",
                   n > 0 && n < (TELEM_LINE_MAX * 4) / 5, true);
    }

    printf("\n=== 6. a buffer that is too small must not write half a line ===\n");
    {
        TelemRecord_t r = make_good();
        char guard[64];
        memset(guard, 0x7E, sizeof(guard));

        int n = telem_format_line(guard, 32u, &r);
        check_int("returns the space error", n, TELEM_ERR_SPACE);
        check_bool("result is a valid terminated string",
                   memchr(guard, '\0', 32u) != NULL, true);

        bool untouched = true;
        for (size_t i = 32u; i < sizeof(guard); ++i) {
            if ((unsigned char)guard[i] != 0x7Eu) { untouched = false; }
        }
        check_bool("nothing written past the end of the buffer", untouched, true);

        check_int("zero capacity is refused too",
                  telem_format_line(guard, 0u, &r), TELEM_ERR_SPACE);
        check_int("null buffer is refused",
                  telem_format_line(NULL, 100u, &r), TELEM_ERR_ARG);
        check_int("null record is refused",
                  telem_format_line(line, sizeof(line), NULL), TELEM_ERR_ARG);
    }

    printf("\n=== 7. the file header describes itself ===\n");
    {
        static const double wp_lat[3] = { 13.65120000, 13.65569661, 13.65569661 };
        static const double wp_lon[3] = { 100.49470000, 100.49470000, 100.49932742 };

        TelemMeta_t m;
        memset(&m, 0, sizeof(m));
        m.boat_id     = "SIM-01";
        m.fw_version  = "nav-0.5";
        m.run_index   = 7u;
        m.utc_start   = "2026-10-09T12:35:19Z";
        m.lookahead_m = 25.0f;
        m.wp_radius_m = 15.0f;
        m.log_rate_hz = 10.0f;
        m.wp_count    = 3u;
        m.wp_lat      = wp_lat;
        m.wp_lon      = wp_lon;

        int n = telem_format_header(head, sizeof(head), &m);
        check_bool("header written", n > 0, true);
        check_bool("starts with a comment", head[0] == '#', true);
        check_bool("records the format version",
                   strstr(head, "# format_version,1\n") != NULL, true);
        check_bool("records the boat",
                   strstr(head, "# boat_id,SIM-01\n") != NULL, true);
        check_bool("records the run index",
                   strstr(head, "# run_index,7\n") != NULL, true);
        check_bool("records the tuning actually used",
                   strstr(head, "# los_lookahead_m,25.00\n") != NULL, true);
        check_bool("records the first waypoint",
                   strstr(head, "# waypoint,0,13.6512000,100.4947000\n") != NULL,
                   true);
        check_bool("records the last waypoint",
                   strstr(head, "# waypoint,2,13.6556966,100.4993274\n") != NULL,
                   true);

        /* บรรทัดสุดท้ายของส่วนหัวต้องเป็นชื่อคอลัมน์ ไม่มี # นำหน้า
         * เพราะ readtable ของ MATLAB จะใช้บรรทัดนี้เป็นชื่อตัวแปร */
        const char *last = strrchr(head, '\n');
        check_bool("header ends with a newline", last != NULL, true);
        if (last != NULL && last != head) {
            const char *prev = last - 1;
            while (prev > head && *prev != '\n') { --prev; }
            if (*prev == '\n') { ++prev; }
            check_bool("last header line is the column row",
                       prev[0] != '#', true);
            check_int("  and it has every column",
                      telem_count_fields(prev), TELEM_FIELD_COUNT);
        }

        check_int("too many waypoints is refused",
                  (m.wp_count = TELEM_META_WP_MAX + 1u,
                   telem_format_header(head, sizeof(head), &m)),
                  TELEM_ERR_ARG);
        m.wp_count = 3u;
        check_int("missing boat id is refused",
                  (m.boat_id = NULL,
                   telem_format_header(head, sizeof(head), &m)),
                  TELEM_ERR_ARG);
        m.boat_id = "SIM-01";
        check_int("a small buffer is refused, not truncated",
                  telem_format_header(head, 40u, &m), TELEM_ERR_SPACE);
    }

    printf("\n=== 8. file names the FAT filesystem accepts ===\n");
    {
        char name[TELEM_FILENAME_MAX];

        check_int("length of a generated name",
                  telem_make_filename(name, sizeof(name), 7u), 12);
        check_str("  run 7", name, "LOG_0007.CSV");
        (void)telem_make_filename(name, sizeof(name), 0u);
        check_str("  run 0", name, "LOG_0000.CSV");
        (void)telem_make_filename(name, sizeof(name), 9999u);
        check_str("  run 9999", name, "LOG_9999.CSV");
        check_int("five digits would break 8.3, so it is refused",
                  telem_make_filename(name, sizeof(name), 10000u),
                  TELEM_ERR_ARG);
        check_int("a buffer one byte short is refused",
                  telem_make_filename(name, 12u, 7u), TELEM_ERR_SPACE);
    }

    printf("\n=== 9. the field counter itself ===\n");
    {
        check_int("counts a plain row", telem_count_fields("a,b,c"), 3);
        check_int("blank fields still count", telem_count_fields("a,,c"), 3);
        check_int("stops at the newline", telem_count_fields("a,b\nc,d,e"), 2);
        check_int("a single field", telem_count_fields("x"), 1);
    }

    printf("\n-----------------------------------------------\n");
    printf("%d checks, %d failed\n", g_total, g_failed);
    printf("%s\n", (g_failed == 0) ? "PASS : every check held" : "FAIL");
    return (g_failed == 0) ? 0 : 1;
}
