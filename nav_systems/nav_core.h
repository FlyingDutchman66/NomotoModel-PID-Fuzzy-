/**
 * nav_core.h
 * ---------------------------------------------------------------------------
 * ตัวประกอบร่างของฝั่ง Navigation
 *
 * รวม nmea + nav_fusion + nav_guidance เข้าด้วยกัน แล้วผลิต NavState_t
 * ซึ่งเป็นสัญญาที่ตกลงไว้กับฝั่ง Control
 *
 * นี่คือไฟล์เดียวที่ Tankhun ต้องรู้จัก เขาไม่ต้องรู้ว่าข้างในมีอะไร
 *
 * โครงของลูปหลักบนบอร์ด
 *
 *     NavCore_t nav;
 *     nav_core_init(&nav, waypoints, 3, &cfg);
 *
 *     while (1) {
 *         if (imu_has_new_sample()) {
 *             nav_core_on_imu(&nav, imu_read_gyro_z(), millis());
 *         }
 *         if (gps_line_ready()) {
 *             GpsRmc_t fix;
 *             if (nmea_parse_rmc(gps_line(), &fix)) {
 *                 nav_core_on_gps(&nav, &fix, millis());
 *             }
 *         }
 *         const NavState_t *s = nav_core_state(&nav, millis());
 *         control_step(s);          // ของ Tankhun
 *     }
 * ---------------------------------------------------------------------------
 */

#ifndef NAV_CORE_H
#define NAV_CORE_H

#include "nav_types.h"
#include "nav_fusion.h"
#include "nav_guidance.h"
#include "nmea.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ค่าตั้งต้นที่แนะนำ แยกเป็น struct เพื่อให้จูนได้โดยไม่ต้องแก้ลายเซ็นฟังก์ชัน */
typedef struct {
    float    fusion_kp;          /* 0.25  จาก sim_heading_fusion.m          */
    float    fusion_ki;          /* 0.012 จาก sim_heading_fusion.m          */
    float    min_cog_speed_mps;  /* 1.0 สำหรับเรือจริง ลดลงถ้าเรือจำลองช้า   */
    float    lookahead_m;        /* 20.0 ควรมากกว่าความคลาดเคลื่อน GPS 5 เท่า */
    float    accept_radius_m;    /* 15.0 ต้องมากกว่าความคลาดเคลื่อนของ GPS   */
    uint32_t max_fix_age_ms;     /* 3000 เก่ากว่านี้ถือว่าขาดการติดต่อ       */
} NavCoreConfig_t;

typedef struct {
    NavFusion_t fusion;
    NavPath_t   path;
    NavState_t  state;

    NavCoreConfig_t cfg;

    uint32_t last_fix_ms;        /* เวลาที่ได้ fix ล่าสุด                   */
    uint32_t last_imu_ms;        /* ใช้คำนวณ dt ของขั้นทำนาย                */
    bool     have_fix;           /* เคยได้ fix อย่างน้อยหนึ่งครั้งแล้วหรือยัง */
    bool     have_imu;           /* เคยได้ไจโรอย่างน้อยหนึ่งครั้งแล้วหรือยัง  */
} NavCore_t;

/* เติมค่าแนะนำลงใน cfg เรียกก่อนแล้วค่อยแก้เฉพาะตัวที่อยากเปลี่ยน */
void nav_core_default_config(NavCoreConfig_t *cfg);

/**
 * ตั้งค่าเริ่มต้นทั้งระบบ เรียกครั้งเดียวตอนเปิดเครื่อง
 *
 * wps   : อาร์เรย์ waypoint ต้องมีอายุยืนกว่า NavCore_t
 * count : อย่างน้อย 2
 * cfg   : NULL ได้ จะใช้ค่าแนะนำทั้งหมด
 *
 * return : true = พร้อมใช้งาน, false = พารามิเตอร์ผิด
 */
bool nav_core_init(NavCore_t *n, const NavWaypoint_t *wps, uint8_t count,
                   const NavCoreConfig_t *cfg);

/**
 * ป้อนค่าไจโร เรียกทุกครั้งที่ IMU มีตัวอย่างใหม่ ราว 50 ถึง 100 Hz
 *
 * gyro_z_dps : อัตราการหมุนรอบแกนตั้ง บวกเมื่อหัวเรือหันขวา
 * now_ms     : เวลาปัจจุบันหน่วยมิลลิวินาที
 *
 * ครั้งแรกจะไม่ทำอะไรนอกจากจำเวลาไว้ เพราะยังไม่มี dt ให้คำนวณ
 */
void nav_core_on_imu(NavCore_t *n, float gyro_z_dps, uint32_t now_ms);

/**
 * ป้อนผลที่แกะได้จากประโยค RMC เรียกเฉพาะตอน nmea_parse_rmc คืน true
 *
 * ฟังก์ชันนี้ทำสามอย่างต่อกัน
 *   1. ส่ง COG ให้ตัวกรองหัวเรือแก้ไข
 *   2. ส่งพิกัดให้ LOS คำนวณทิศเป้าหมายและความผิดพลาดด้านข้าง
 *   3. ปิดท้ายด้วยการประกอบ snapshot ใหม่ทั้งก้อน
 */
void nav_core_on_gps(NavCore_t *n, const GpsRmc_t *fix, uint32_t now_ms);

/**
 * ขอ snapshot ปัจจุบัน เรียกได้ทุกเมื่อ ทุกความถี่
 *
 * ฟังก์ชันนี้คำนวณ gps_age_ms และ nav_valid ใหม่ตามเวลาที่ส่งเข้ามา
 * จึงต้องเรียกผ่านฟังก์ชันนี้เสมอ ห้ามอ่าน n->state ตรง ๆ
 *
 * return : ตัวชี้ไปยัง snapshot ไม่เคยเป็น NULL เว้นแต่ n เป็น NULL
 */
const NavState_t *nav_core_state(NavCore_t *n, uint32_t now_ms);

/**
 * ป้อนคุณภาพสัญญาณจากประโยค GGA ซึ่งยังไม่ได้เขียนตัวแกะ
 * มีไว้เพื่อให้ฟิลด์เหล่านี้มีที่มาที่ชัดเจนเมื่อเพิ่มตัวแกะ GGA ภายหลัง
 * ถ้าไม่เคยเรียก ฟิลด์ทั้งสามจะเป็นศูนย์ และไม่มีผลต่อ nav_valid
 */
void nav_core_set_fix_quality(NavCore_t *n, uint8_t fix_type,
                              uint8_t sat_count, float hdop);

#ifdef __cplusplus
}
#endif

#endif /* NAV_CORE_H */
