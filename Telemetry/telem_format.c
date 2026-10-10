/**
 * telem_format.c
 * ---------------------------------------------------------------------------
 * การตัดสินใจสามข้อที่อธิบายโค้ดในไฟล์นี้
 *
 * ข้อแรก จำนวนคอลัมน์คงที่เสมอ แม้ค่าจะไม่มี
 *   เมื่อข้อมูลใช้ไม่ได้ จะเว้นช่องว่างไว้ ไม่ใช่เติมศูนย์
 *   เหตุผลคือพิกัด 0,0 อยู่กลางอ่าวกินี ถ้าเติมศูนย์เวลายังไม่จับดาวเทียม
 *   กราฟเส้นทางจะมีเส้นลากจากอ่าวไทยไปแอฟริกา ซึ่งเคยทำให้คนเสียเวลา
 *   ไล่หาบั๊กที่ไม่มีอยู่จริงมานับไม่ถ้วน ช่องว่างถูกอ่านเป็น NaN โดย MATLAB
 *   และถูกข้ามโดยอัตโนมัติตอนพล็อต
 *
 * ข้อสอง เขียนลงบัฟเฟอร์ชั่วคราวก่อน แล้วค่อยตัดสินว่าสำเร็จหรือไม่
 *   ถ้าเนื้อที่ไม่พอ จะคืนค่าผิดพลาดโดยไม่ทิ้งบรรทัดที่ขาดครึ่งไว้ในไฟล์
 *
 * ข้อสาม ความละเอียดทศนิยมเลือกตามความหมายทางกายภาพ ไม่ใช่ตามใจ
 *   พิกัดเจ็ดตำแหน่งคือราวหนึ่งเซนติเมตร ซึ่งละเอียดกว่า GPS หลายเท่า
 *   มุมสองตำแหน่งคือ 0.01 องศา ละเอียดกว่าที่เข็มทิศใด ๆ จะวัดได้
 *   กระแสสามตำแหน่งคือหนึ่งมิลลิแอมป์ ละเอียดพอสำหรับการอินทิเกรตพลังงาน
 *   ละเอียดกว่านี้คือการเปลืองเนื้อที่เพื่อบันทึกสัญญาณรบกวน
 * ---------------------------------------------------------------------------
 */

#include "telem_format.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* --------------------------------------------------------------------------
 * ตัวช่วยเขียนต่อท้ายแบบปลอดภัย
 *
 * snprintf คืนจำนวนตัวอักษรที่ "อยากเขียน" ไม่ใช่จำนวนที่เขียนได้จริง
 * นี่คือกับดักคลาสสิก ถ้าเอาค่าคืนไปบวกเลื่อนตัวชี้โดยไม่ตรวจ
 * ตัวชี้จะวิ่งเลยปลายบัฟเฟอร์ แล้วการเขียนครั้งถัดไปจะทับหน่วยความจำคนอื่น
 * โครงสร้างนี้จึงจำสถานะผิดพลาดไว้ และหยุดเขียนทันทีเมื่อเต็ม
 * -------------------------------------------------------------------------- */
typedef struct {
    char  *base;
    size_t cap;     /* ขนาดบัฟเฟอร์ทั้งหมด รวมที่สำหรับตัวปิดสตริง */
    size_t len;     /* จำนวนตัวอักษรที่เขียนแล้ว ไม่นับตัวปิดสตริง */
    bool   full;    /* true เมื่อเคยเขียนไม่พอแม้ครั้งเดียว */
} Cur_t;

static void cur_init(Cur_t *c, char *buf, size_t cap)
{
    c->base = buf;
    c->cap  = cap;
    c->len  = 0;
    c->full = false;
    if (cap > 0u) {
        buf[0] = '\0';
    } else {
        c->full = true;
    }
}

static void cur_printf(Cur_t *c, const char *fmt, ...)
{
    if (c->full) {
        return;
    }

    size_t left = c->cap - c->len;   /* อย่างน้อย 1 เสมอ เพราะ len < cap */

    va_list ap;
    va_start(ap, fmt);
    int want = vsnprintf(c->base + c->len, left, fmt, ap);
    va_end(ap);

    if (want < 0) {
        c->full = true;
        return;
    }
    if ((size_t)want >= left) {
        /* เขียนไม่หมด ถือว่าล้มเหลวทั้งบรรทัด */
        c->full = true;
        c->base[c->len] = '\0';
        return;
    }
    c->len += (size_t)want;
}

/* ตัวเลขทศนิยมที่อาจว่าง ใช้กับทุกค่าที่ความถูกต้องขึ้นกับธงสถานะ */
static void cur_optf(Cur_t *c, bool valid, const char *fmt, double v)
{
    if (valid) {
        cur_printf(c, fmt, v);
    }
    /* ไม่ valid ก็ไม่เขียนอะไรเลย เหลือไว้เป็นช่องว่างระหว่างคอมมาสองตัว */
}

static int cur_finish(const Cur_t *c)
{
    if (c->full) {
        return TELEM_ERR_SPACE;
    }
    return (int)c->len;
}


/* --------------------------------------------------------------------------
 * ชื่อคอลัมน์
 *
 * ลำดับในสตริงนี้ต้องตรงกับลำดับการเขียนใน telem_format_line เป๊ะ ๆ
 * ทั้งสองที่อยู่ในไฟล์เดียวกันโดยตั้งใจ เพื่อให้แก้พร้อมกันได้ในหน้าจอเดียว
 * -------------------------------------------------------------------------- */
static const char TELEM_COLUMNS[] =
    "t_ms,utc_sod,nav_valid,"
    "lat_deg,lon_deg,sog_mps,cog_deg,"
    "hdg_deg,yaw_rate_dps,psi_d_deg,hdg_err_deg,"
    "xte_m,dist_wp_m,wp_idx,fix,sats,hdop,gps_age_ms,"
    "mode,rud_cmd_deg,rud_meas_deg,kp,ki,kd,flags,"
    "vbat_v,i_prop_a,i_steer_a,e_prop_wh,e_steer_wh,"
    "s_along_m,s_ground_m,drops,loop_us";


int telem_format_columns(char *buf, size_t cap)
{
    if (buf == NULL) {
        return TELEM_ERR_ARG;
    }

    Cur_t c;
    cur_init(&c, buf, cap);
    cur_printf(&c, "%s\n", TELEM_COLUMNS);
    return cur_finish(&c);
}


int telem_format_header(char *buf, size_t cap, const TelemMeta_t *meta)
{
    if (buf == NULL || meta == NULL) {
        return TELEM_ERR_ARG;
    }
    if (meta->boat_id == NULL || meta->fw_version == NULL) {
        return TELEM_ERR_ARG;
    }
    if (meta->wp_count > TELEM_META_WP_MAX) {
        return TELEM_ERR_ARG;
    }
    if (meta->wp_count > 0u && (meta->wp_lat == NULL || meta->wp_lon == NULL)) {
        return TELEM_ERR_ARG;
    }

    Cur_t c;
    cur_init(&c, buf, cap);

    cur_printf(&c, "# course-keeping autopilot telemetry log\n");
    cur_printf(&c, "# format_version,%d\n", TELEM_FORMAT_VERSION);
    cur_printf(&c, "# field_count,%d\n", TELEM_FIELD_COUNT);
    cur_printf(&c, "# boat_id,%s\n", meta->boat_id);
    cur_printf(&c, "# fw_version,%s\n", meta->fw_version);
    cur_printf(&c, "# run_index,%u\n", (unsigned)meta->run_index);
    cur_printf(&c, "# utc_start,%s\n",
               (meta->utc_start != NULL) ? meta->utc_start : "unknown");
    cur_printf(&c, "# log_rate_hz,%.2f\n", (double)meta->log_rate_hz);
    cur_printf(&c, "# los_lookahead_m,%.2f\n", (double)meta->lookahead_m);
    cur_printf(&c, "# wp_radius_m,%.2f\n", (double)meta->wp_radius_m);
    cur_printf(&c, "# wp_count,%u\n", (unsigned)meta->wp_count);

    for (uint8_t i = 0u; i < meta->wp_count; ++i) {
        cur_printf(&c, "# waypoint,%u,%.7f,%.7f\n",
                   (unsigned)i, meta->wp_lat[i], meta->wp_lon[i]);
    }

    cur_printf(&c, "# units,t_ms=ms utc_sod=s lat_deg=deg lon_deg=deg\n");
    cur_printf(&c, "# units,sog_mps=m/s angles=deg rates=deg/s distances=m\n");
    cur_printf(&c, "# units,vbat_v=V currents=A energies=Wh loop_us=us\n");
    cur_printf(&c, "# note,empty field means the value was not trustworthy\n");
    cur_printf(&c, "%s\n", TELEM_COLUMNS);

    return cur_finish(&c);
}


int telem_format_line(char *buf, size_t cap, const TelemRecord_t *r)
{
    if (buf == NULL || r == NULL) {
        return TELEM_ERR_ARG;
    }

    Cur_t c;
    cur_init(&c, buf, cap);

    /* [1] เวลา */
    cur_printf(&c, "%lu,", (unsigned long)r->t_ms);
    cur_optf(&c, r->utc_valid, "%.2f", (double)r->utc_sod);
    cur_printf(&c, ",");

    /* [2] นำทาง
     * nav_valid เขียนเป็น 0 หรือ 1 เสมอ เพราะตัวมันเองคือคำตอบ ไม่ใช่ค่าที่สงสัย
     * พิกัดและทิศเดินเรือเว้นว่างเมื่อเชื่อไม่ได้ ด้วยเหตุผลเรื่องพิกัด 0,0 ข้างบน */
    cur_printf(&c, "%d,", r->nav_valid ? 1 : 0);
    cur_optf(&c, r->nav_valid, "%.7f", r->lat_deg);
    cur_printf(&c, ",");
    cur_optf(&c, r->nav_valid, "%.7f", r->lon_deg);
    cur_printf(&c, ",");
    cur_printf(&c, "%.2f,", (double)r->sog_mps);
    cur_optf(&c, r->cog_valid, "%.2f", (double)r->cog_deg);
    cur_printf(&c, ",");

    cur_printf(&c, "%.2f,", (double)r->heading_deg);
    cur_printf(&c, "%.2f,", (double)r->yaw_rate_dps);
    cur_optf(&c, r->nav_valid, "%.2f", (double)r->desired_heading_deg);
    cur_printf(&c, ",");
    cur_optf(&c, r->nav_valid, "%.2f", (double)r->heading_error_deg);
    cur_printf(&c, ",");
    cur_optf(&c, r->nav_valid, "%.2f", (double)r->cross_track_error_m);
    cur_printf(&c, ",");
    cur_optf(&c, r->nav_valid, "%.2f", (double)r->dist_to_wp_m);
    cur_printf(&c, ",");

    cur_printf(&c, "%u,", (unsigned)r->wp_index);
    cur_printf(&c, "%u,", (unsigned)r->fix_type);
    cur_printf(&c, "%u,", (unsigned)r->sat_count);
    cur_printf(&c, "%.1f,", (double)r->hdop);
    cur_printf(&c, "%lu,", (unsigned long)r->gps_age_ms);

    /* [3] ควบคุม */
    cur_printf(&c, "%u,", (unsigned)r->mode);
    cur_printf(&c, "%.2f,", (double)r->rudder_cmd_deg);
    cur_optf(&c, r->rudder_meas_valid, "%.2f", (double)r->rudder_meas_deg);
    cur_printf(&c, ",");
    cur_optf(&c, r->gains_valid, "%.4f", (double)r->kp);
    cur_printf(&c, ",");
    cur_optf(&c, r->gains_valid, "%.4f", (double)r->ki);
    cur_printf(&c, ",");
    cur_optf(&c, r->gains_valid, "%.4f", (double)r->kd);
    cur_printf(&c, ",");
    cur_printf(&c, "%u,", (unsigned)r->ctrl_flags);

    /* [4] ไฟฟ้า
     * พลังงานสะสมเขียนเสมอ แม้เซ็นเซอร์จะยังไม่มี เพราะค่าศูนย์ที่เพิ่มขึ้นไม่ได้
     * ไม่หลอกใคร ต่างจากพิกัดศูนย์ที่ชี้ไปผิดที่บนแผนที่ */
    cur_printf(&c, "%.2f,", (double)r->vbat_v);
    cur_optf(&c, r->i_prop_valid, "%.3f", (double)r->i_prop_a);
    cur_printf(&c, ",");
    cur_optf(&c, r->i_steer_valid, "%.3f", (double)r->i_steer_a);
    cur_printf(&c, ",");
    cur_printf(&c, "%.4f,", (double)r->e_prop_wh);
    cur_printf(&c, "%.4f,", (double)r->e_steer_wh);

    /* [5] ระยะทางสะสม */
    cur_printf(&c, "%.2f,", (double)r->s_along_m);
    cur_printf(&c, "%.2f,", (double)r->s_ground_m);

    /* [6] สุขภาพระบบ */
    cur_printf(&c, "%lu,", (unsigned long)r->drop_count);
    cur_printf(&c, "%u\n", (unsigned)r->loop_us);

    return cur_finish(&c);
}


int telem_make_filename(char *buf, size_t cap, uint16_t index)
{
    if (buf == NULL) {
        return TELEM_ERR_ARG;
    }
    if (index > 9999u) {
        return TELEM_ERR_ARG;
    }

    Cur_t c;
    cur_init(&c, buf, cap);
    cur_printf(&c, "LOG_%04u.CSV", (unsigned)index);
    return cur_finish(&c);
}


int telem_count_fields(const char *line)
{
    if (line == NULL) {
        return TELEM_ERR_ARG;
    }

    int n = 1;
    for (const char *p = line; *p != '\0' && *p != '\n'; ++p) {
        if (*p == ',') {
            ++n;
        }
    }
    return n;
}
