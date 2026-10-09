/**
 * nav_guidance.h
 * ---------------------------------------------------------------------------
 * Line-of-Sight (LOS) Guidance สำหรับเกาะเส้นทางระหว่าง waypoint
 * ชั้นตรรกะล้วน ไม่เรียกฮาร์ดแวร์ ไม่จองหน่วยความจำ
 *
 * ปัญหาที่แก้
 *   ถ้าเล็งไปที่ waypoint ปลายทางตรง ๆ เรือที่ถูกคลื่นซัดออกนอกเส้นทาง
 *   จะวิ่งเฉียงเข้าหาปลายทางโดยไม่กลับเข้าเส้นทางเดิมเลย
 *   LOS แก้ด้วยการเล็งไปยังจุดบนเส้นทางที่อยู่ข้างหน้าเราเป็นระยะ lookahead
 *   ยิ่งเรือออกนอกเส้นมาก มุมที่หันเข้าหาเส้นก็ยิ่งชัน
 *
 * สูตรหลัก
 *   chi_d = chi_p + atan( -e / Delta )
 *   เมื่อ chi_p คือทิศของเส้นทาง  e คือระยะออกนอกเส้น  Delta คือ lookahead
 *
 * ระบบพิกัดท้องถิ่น
 *   แปลงละติจูดลองจิจูดเป็นเมตรบนระนาบราบ โดยใช้ waypoint ต้นทางของช่วง
 *   ปัจจุบันเป็นจุดกำเนิด ในระยะไม่กี่กิโลเมตรความคลาดเคลื่อนจากการสมมติ
 *   ว่าโลกแบนน้อยกว่าความคลาดเคลื่อนของ GPS เองมาก
 *
 * เครื่องหมาย
 *   cross_track_error_m เป็นบวกเมื่อเรืออยู่ทางขวาของเส้นทาง
 *   ตรงกับที่ประกาศไว้ใน nav_types.h
 * ---------------------------------------------------------------------------
 */

#ifndef NAV_GUIDANCE_H
#define NAV_GUIDANCE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* จำนวน waypoint สูงสุดที่ระบบรองรับ ตั้งไว้เผื่อการทดสอบภาคสนาม */
#define NAV_MAX_WAYPOINTS 32

typedef struct {
    double lat_deg;
    double lon_deg;
} NavWaypoint_t;

typedef struct {
    const NavWaypoint_t *wp;      /* อาร์เรย์ที่ผู้เรียกเป็นเจ้าของ ห้ามเป็น NULL */
    uint8_t wp_count;             /* จำนวน waypoint ต้องอย่างน้อย 2            */
    uint8_t wp_index;             /* ลำดับของ waypoint ปลายทางของช่วงปัจจุบัน  */

    float lookahead_m;            /* Delta ระยะเล็งไปข้างหน้า                  */
    float accept_radius_m;        /* เข้าใกล้เท่านี้ถือว่าถึงแล้ว               */

    bool  finished;               /* true เมื่อผ่าน waypoint สุดท้ายไปแล้ว     */
} NavPath_t;

typedef struct {
    float desired_heading_deg;    /* chi_d ทิศที่ควรหันหัวเรือ [0,360)        */
    float path_bearing_deg;       /* chi_p ทิศของเส้นทางช่วงนี้ [0,360)       */
    float cross_track_error_m;    /* e บวก = อยู่ทางขวาของเส้นทาง             */
    float along_track_m;          /* เดินมาตามเส้นทางแล้วกี่เมตร              */
    float segment_length_m;       /* ความยาวของช่วงนี้                        */
    float dist_to_wp_m;           /* ระยะตรงถึง waypoint ปลายทาง              */
    uint8_t wp_index;             /* กำลังมุ่งไป waypoint ลำดับที่เท่าไร       */
    bool  finished;               /* ถึงปลายทางสุดท้ายแล้ว                    */
} NavGuidance_t;

/**
 * ตั้งค่าเส้นทาง เรียกครั้งเดียวก่อนเริ่มเดินทาง
 *
 * wps             : อาร์เรย์ waypoint ต้องมีอายุยืนกว่า NavPath_t
 * count           : จำนวน waypoint อย่างน้อย 2 ไม่เกิน NAV_MAX_WAYPOINTS
 * lookahead_m     : ควรมากกว่าความคลาดเคลื่อนของ GPS หลายเท่า
 *                   เล็กไปเรือจะส่ายตามสัญญาณรบกวน ใหญ่ไปเรือจะกลับเข้าเส้นช้า
 * accept_radius_m : ต้องใหญ่กว่าความคลาดเคลื่อนของ GPS
 *                   ถ้าเล็กกว่า เรืออาจวนรอบ waypoint โดยไม่เคยถือว่าถึง
 *
 * return : true = ค่าที่ให้มาใช้ได้, false = พารามิเตอร์ผิด (path ไม่ถูกตั้ง)
 */
bool nav_path_init(NavPath_t *p, const NavWaypoint_t *wps, uint8_t count,
                   float lookahead_m, float accept_radius_m);

/**
 * คำนวณคำสั่งนำทางจากตำแหน่งปัจจุบัน เรียกทุกครั้งที่ได้พิกัดใหม่
 *
 * ฟังก์ชันนี้แก้ไข p ด้วย เพราะมันเลื่อน wp_index เองเมื่อถึง waypoint
 *
 * return : true = out ถูกเขียนแล้ว, false = พารามิเตอร์ผิด (out ไม่ถูกแตะ)
 *          เมื่อเดินทางจบ จะยัง return true แต่ out->finished เป็น true
 *          และ desired_heading_deg จะค้างที่ทิศของช่วงสุดท้าย
 */
bool nav_los_update(NavPath_t *p, double lat_deg, double lon_deg,
                    NavGuidance_t *out);

#ifdef __cplusplus
}
#endif

#endif /* NAV_GUIDANCE_H */
