#include "nav_math.h"

#include <math.h>
#include <stddef.h>   /* NULL */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

/* --------------------------------------------------------------------------
 * การบีบมุม
 * -------------------------------------------------------------------------- */

float nav_wrap180(float angle_deg)
{
    /* เลื่อนขึ้น 180 แล้วหารเอาเศษ 360 จากนั้นเลื่อนกลับลง
     * วิธีนี้ทำงานถูกต้องกับค่าลบและค่าที่เกินหลายรอบ
     *
     * หมายเหตุ : 180.0 พอดีจะออกมาเป็น -180.0
     * ในทางควบคุมถือว่าสมมูลกัน เพราะทั้งคู่แปลว่าต้องกลับลำ
     * ส่วนทิศที่จะหมุนให้ตัวควบคุมเลือกเองตามสถานการณ์ */
    float a = fmodf(angle_deg + 180.0f, 360.0f);
    if (a < 0.0f) {
        a += 360.0f;
    }
    return a - 180.0f;
}

float nav_wrap360(float angle_deg)
{
    float a = fmodf(angle_deg, 360.0f);
    if (a < 0.0f) {
        a += 360.0f;
    }
    /* กันกรณีปัดเศษทำให้ได้ 360.0 พอดี ซึ่งต้องเป็น 0.0 */
    if (a >= 360.0f) {
        a = 0.0f;
    }
    return a;
}

bool nav_is_fresh(const NavState_t *s, uint32_t now_ms, uint32_t max_age_ms)
{
    if (s == NULL || !s->nav_valid) {
        return false;
    }
    /* ลบแบบ unsigned ทนการวนรอบของตัวนับเวลาได้เอง
     * ตัวนับ 32 บิตหน่วยมิลลิวินาทีจะวนรอบทุก 49.7 วัน
     * ถ้าเขียนเป็น (int)now - (int)stamp จะได้ค่าติดลบมหาศาลตอนวนรอบ
     * แล้วระบบจะเชื่อว่าข้อมูลสดทั้งที่ค้างมานาน */
    uint32_t age = now_ms - s->timestamp_ms;
    return age <= max_age_ms;
}

/* --------------------------------------------------------------------------
 * ระยะทางและทิศทาง
 * -------------------------------------------------------------------------- */

double nav_haversine_m(double lat1_deg, double lon1_deg,
                       double lat2_deg, double lon2_deg)
{
    double phi1 = lat1_deg * DEG2RAD;
    double phi2 = lat2_deg * DEG2RAD;
    double dphi = (lat2_deg - lat1_deg) * DEG2RAD;
    double dlam = (lon2_deg - lon1_deg) * DEG2RAD;

    double sin_dphi = sin(dphi * 0.5);
    double sin_dlam = sin(dlam * 0.5);

    double a = sin_dphi * sin_dphi
             + cos(phi1) * cos(phi2) * sin_dlam * sin_dlam;

    /* กันค่าหลุดช่วงจากการปัดเศษ ก่อนเข้า sqrt */
    if (a < 0.0) a = 0.0;
    if (a > 1.0) a = 1.0;

    /* ใช้ atan2 แทน asin เพราะ asin สูญเสียความแม่นยำเมื่อสองจุดอยู่
     * คนละซีกโลก ส่วน atan2 แม่นสม่ำเสมอทุกระยะ */
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    return NAV_EARTH_RADIUS_M * c;
}

double nav_bearing_deg(double lat1_deg, double lon1_deg,
                       double lat2_deg, double lon2_deg)
{
    double phi1 = lat1_deg * DEG2RAD;
    double phi2 = lat2_deg * DEG2RAD;
    double dlam = (lon2_deg - lon1_deg) * DEG2RAD;

    double y = sin(dlam) * cos(phi2);
    double x = cos(phi1) * sin(phi2)
             - sin(phi1) * cos(phi2) * cos(dlam);

    /* atan2 คืนค่าในช่วง (-pi, pi] จึงต้องบีบเป็น [0, 360) */
    double deg = atan2(y, x) * RAD2DEG;
    deg = fmod(deg, 360.0);
    if (deg < 0.0) {
        deg += 360.0;
    }
    if (deg >= 360.0) {
        deg = 0.0;
    }
    return deg;
}
