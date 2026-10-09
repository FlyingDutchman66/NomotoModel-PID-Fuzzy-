#include "nav_core.h"
#include "nav_math.h"

#include <stddef.h>
#include <string.h>

void nav_core_default_config(NavCoreConfig_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    cfg->fusion_kp         = 0.25f;
    cfg->fusion_ki         = 0.012f;
    cfg->min_cog_speed_mps = 1.0f;
    cfg->lookahead_m       = 20.0f;
    cfg->accept_radius_m   = 15.0f;
    cfg->max_fix_age_ms    = 3000u;
}

bool nav_core_init(NavCore_t *n, const NavWaypoint_t *wps, uint8_t count,
                   const NavCoreConfig_t *cfg)
{
    if (n == NULL) {
        return false;
    }

    memset(n, 0, sizeof(*n));

    if (cfg != NULL) {
        n->cfg = *cfg;
    } else {
        nav_core_default_config(&n->cfg);
    }

    if (n->cfg.max_fix_age_ms == 0u) {
        return false;
    }

    nav_fusion_init(&n->fusion, n->cfg.fusion_kp, n->cfg.fusion_ki,
                    n->cfg.min_cog_speed_mps);

    if (!nav_path_init(&n->path, wps, count,
                       n->cfg.lookahead_m, n->cfg.accept_radius_m)) {
        return false;
    }

    n->have_fix    = false;
    n->have_imu    = false;
    n->last_fix_ms = 0u;
    n->last_imu_ms = 0u;

    /* snapshot เริ่มต้นต้องประกาศตัวเองว่าใช้ไม่ได้
     * เผื่อกรณีที่ Control เริ่มอ่านก่อนที่เซ็นเซอร์ตัวแรกจะมา */
    n->state.nav_valid = false;
    n->state.seq       = 0u;

    return true;
}

/* ประกอบ snapshot ใหม่ทั้งก้อน เรียกจากทุกจุดที่สถานะเปลี่ยน
 *
 * เขียนลงโครงสร้างชั่วคราวก่อนแล้วค่อยคัดลอกทีเดียว
 * เพื่อให้ผู้อ่านไม่มีทางเห็นก้อนที่ประกอบไม่เสร็จ */
static void rebuild_snapshot(NavCore_t *n, uint32_t now_ms)
{
    NavState_t s = n->state;     /* เริ่มจากของเดิม แล้วทับเฉพาะที่เปลี่ยน */

    s.timestamp_ms = now_ms;
    s.seq          = (uint16_t)(n->state.seq + 1u);

    s.heading_deg  = nav_fusion_heading(&n->fusion);

    /* ผลต่างมุมที่บีบช่วงแล้ว คือสิ่งที่ตกลงกันไว้ว่าฝั่งเราเป็นคนคำนวณ
     * ถ้าปล่อยให้ Control ลบเอง จะพังที่รอยต่อศูนย์องศาแน่นอน */
    s.heading_error_deg = nav_wrap180(s.desired_heading_deg - s.heading_deg);

    s.gps_age_ms = n->have_fix ? (now_ms - n->last_fix_ms) : 0xFFFFFFFFu;

    /* nav_valid ถูกคำนวณใหม่ทุกครั้ง ไม่ใช่ตัวแปรที่ใครตั้งค่าค้างไว้ได้
     * สามเงื่อนไขต้องเป็นจริงพร้อมกัน */
    s.nav_valid = n->have_fix
               && nav_fusion_is_ready(&n->fusion)
               && (s.gps_age_ms <= n->cfg.max_fix_age_ms);

    n->state = s;
}

void nav_core_on_imu(NavCore_t *n, float gyro_z_dps, uint32_t now_ms)
{
    if (n == NULL) {
        return;
    }

    if (!n->have_imu) {
        /* ตัวอย่างแรก ยังไม่มี dt ให้คำนวณ จำเวลาไว้แล้วรอรอบหน้า */
        n->last_imu_ms = now_ms;
        n->have_imu    = true;
        return;
    }

    /* ลบแบบ unsigned ทนการวนรอบของตัวนับ 32 บิตได้เอง */
    uint32_t elapsed_ms = now_ms - n->last_imu_ms;
    n->last_imu_ms = now_ms;

    float dt_s = (float)elapsed_ms * 0.001f;
    nav_fusion_predict(&n->fusion, gyro_z_dps, dt_s);

    /* อัตราการหมุนที่ส่งให้ Control คือค่าที่หัก bias ออกแล้ว
     * เพราะนั่นคืออัตราที่เรือหมุนจริง ไม่ใช่ค่าดิบจากเซ็นเซอร์ */
    n->state.yaw_rate_dps = gyro_z_dps - n->fusion.gyro_bias_dps;

    rebuild_snapshot(n, now_ms);
}

void nav_core_on_gps(NavCore_t *n, const GpsRmc_t *fix, uint32_t now_ms)
{
    if (n == NULL || fix == NULL) {
        return;
    }

    /* ขั้นที่ 1 ให้ COG แก้ไขตัวกรองหัวเรือ
     * ตัวกรองมีประตูของมันเองอยู่แล้ว ถ้าเรือช้าเกินไปมันจะปฏิเสธเอง */
    nav_fusion_update_cog(&n->fusion, fix->cog_deg,
                          fix->sog_mps, fix->cog_valid);

    /* ขั้นที่ 2 ให้พิกัดใหม่คำนวณคำสั่งนำทาง */
    NavGuidance_t g;
    if (nav_los_update(&n->path, fix->lat_deg, fix->lon_deg, &g)) {
        n->state.desired_heading_deg = g.desired_heading_deg;
        n->state.cross_track_error_m = g.cross_track_error_m;
        n->state.dist_to_wp_m        = g.dist_to_wp_m;
        n->state.wp_index            = g.wp_index;
    }

    n->state.latitude_deg  = fix->lat_deg;
    n->state.longitude_deg = fix->lon_deg;
    n->state.sog_mps       = fix->sog_mps;
    n->state.cog_deg       = fix->cog_deg;

    n->last_fix_ms = now_ms;
    n->have_fix    = true;

    /* ขั้นที่ 3 ประกอบ snapshot ใหม่ */
    rebuild_snapshot(n, now_ms);
}

const NavState_t *nav_core_state(NavCore_t *n, uint32_t now_ms)
{
    if (n == NULL) {
        return NULL;
    }

    /* ต้องคำนวณอายุและความถูกต้องใหม่ทุกครั้งที่มีคนถาม
     * ไม่ใช่แค่ตอนที่มีข้อมูลเข้ามา เพราะเหตุการณ์ที่เราต้องจับให้ได้
     * คือเหตุการณ์ที่ "ไม่มีข้อมูลเข้ามาเลย" ซึ่งไม่มีอะไรมาปลุกเรา */
    uint32_t age = n->have_fix ? (now_ms - n->last_fix_ms) : 0xFFFFFFFFu;

    n->state.gps_age_ms = age;
    n->state.nav_valid  = n->have_fix
                       && nav_fusion_is_ready(&n->fusion)
                       && (age <= n->cfg.max_fix_age_ms);

    return &n->state;
}

void nav_core_set_fix_quality(NavCore_t *n, uint8_t fix_type,
                              uint8_t sat_count, float hdop)
{
    if (n == NULL) {
        return;
    }
    n->state.fix_type  = fix_type;
    n->state.sat_count = sat_count;
    n->state.hdop      = hdop;
}
