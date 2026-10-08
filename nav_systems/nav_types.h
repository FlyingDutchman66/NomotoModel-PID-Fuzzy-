/**
 * nav_types.h
 * ---------------------------------------------------------------------------
 * Interface contract between Navigation (Napat) and Control (Tankhun)
 * Project : Course-Keeping Autopilot for Small Outboard Fishing Vessels
 * ---------------------------------------------------------------------------
 * Hardware-agnostic. No HAL, no Arduino.h, no vendor headers.
 * Compiles on STM32, ESP32, and on a PC for unit testing.
 *
 * UNIT CONVENTION (ทั้งโปรเจกต์ใช้ชุดนี้ ห้ามแก้โดยไม่แจ้งทีม)
 *   angle          : degree            [deg]
 *   angular rate   : degree per second [deg/s]
 *   speed          : meter per second  [m/s]   (knot ใช้เฉพาะตอนแสดงผล)
 *   distance       : meter             [m]
 *   time           : millisecond since boot [ms]
 *   heading frame  : 0 = True North, เพิ่มตามเข็มนาฬิกา, ช่วง [0, 360)
 */

#ifndef NAV_TYPES_H
#define NAV_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * คุณภาพของ GPS fix  (ค่าตรงกับ field ที่ 6 ของ $GPGGA)
 * -------------------------------------------------------------------------- */
typedef enum {
    NAV_FIX_NONE  = 0,   /* ยังไม่จับดาวเทียม - ห้ามใช้พิกัด */
    NAV_FIX_GPS   = 1,   /* fix ปกติ */
    NAV_FIX_DGPS  = 2,   /* differential - แม่นขึ้น */
    NAV_FIX_RTK_F = 4,
    NAV_FIX_RTK_L = 5
} NavFixType_t;

/* --------------------------------------------------------------------------
 * NavState_t
 * snapshot เดียวที่ Navigation ผลิตออกมา และ Control อ่านไปใช้
 * อัปเดตทั้งก้อนพร้อมกันเสมอ ห้ามอัปเดตทีละ field
 * -------------------------------------------------------------------------- */
typedef struct {

    /* ---- [1] TIME & VALIDITY : ต้องเช็กก่อนใช้ค่าอื่นทุกครั้ง ---- */
    uint32_t timestamp_ms;        /* เวลาที่ snapshot นี้ถูกสร้างเสร็จ */
    uint32_t gps_age_ms;          /* ผ่านมากี่ ms ตั้งแต่ GPS fix ล่าสุด */
    uint16_t seq;                 /* +1 ทุกครั้งที่อัปเดต ใช้ตรวจว่าข้อมูลค้างไหม */
    bool     nav_valid;           /* false = Control ต้องเข้า fail-safe */

    /* ---- [2] MEASURED STATE : เรืออยู่ไหน หันไปทางไหน ---- */
    double   latitude_deg;        /* + = North */
    double   longitude_deg;       /* + = East  */
    float    sog_mps;             /* Speed Over Ground */
    float    cog_deg;             /* Course Over Ground [0,360) จาก GPS */
    float    heading_deg;         /* psi  : หัวเรือหลัง fusion [0,360) */
    float    yaw_rate_dps;        /* r    : อัตราการหมุนหัวเรือ จาก IMU gyro Z */

    /* ---- [3] GUIDANCE OUTPUT : Control ใช้ 2 ตัวนี้เป็นหลัก ---- */
    float    desired_heading_deg; /* psi_d : จาก LOS guidance [0,360) */
    float    heading_error_deg;   /* e = wrap180(psi_d - psi) ช่วง [-180,180] */
    float    cross_track_error_m; /* + = เรืออยู่ทางขวาของเส้นทาง */
    float    dist_to_wp_m;        /* ระยะถึง waypoint ปัจจุบัน */
    uint8_t  wp_index;            /* กำลังวิ่งไป waypoint ลำดับที่เท่าไร */

    /* ---- [4] FIX QUALITY : ใช้ตัดสินใจและใช้ตอน debug ---- */
    uint8_t  fix_type;            /* NavFixType_t */
    uint8_t  sat_count;
    float    hdop;                /* < 2.0 ถือว่าดี, > 5.0 ถือว่าแย่ */

} NavState_t;

/* --------------------------------------------------------------------------
 * Helper ที่ Navigation รับผิดชอบ (implement ใน nav_math.c)
 * -------------------------------------------------------------------------- */

/* บีบมุมใด ๆ ให้อยู่ในช่วง [-180, 180] - ใช้ทุกครั้งที่ลบมุมกัน */
float nav_wrap180(float angle_deg);

/* บีบมุมใด ๆ ให้อยู่ในช่วง [0, 360) */
float nav_wrap360(float angle_deg);

/* true = snapshot ยังสดและเชื่อถือได้ ณ เวลา now_ms */
bool  nav_is_fresh(const NavState_t *s, uint32_t now_ms, uint32_t max_age_ms);

#ifdef __cplusplus
}
#endif

#endif /* NAV_TYPES_H */
