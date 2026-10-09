#include "nav_guidance.h"
#include "nav_math.h"

#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

/* ความยาวช่วงที่สั้นกว่านี้ถือว่า waypoint สองจุดทับกัน หน่วยเมตร */
#define SEGMENT_MIN_LEN_M 0.01

bool nav_path_init(NavPath_t *p, const NavWaypoint_t *wps, uint8_t count,
                   float lookahead_m, float accept_radius_m)
{
    if (p == NULL || wps == NULL) {
        return false;
    }
    if (count < 2 || count > NAV_MAX_WAYPOINTS) {
        return false;
    }
    if (lookahead_m <= 0.0f || accept_radius_m <= 0.0f) {
        return false;
    }

    p->wp              = wps;
    p->wp_count        = count;
    p->wp_index        = 1;      /* ช่วงแรกคือจาก wp[0] ไป wp[1] */
    p->lookahead_m     = lookahead_m;
    p->accept_radius_m = accept_radius_m;
    p->finished        = false;

    return true;
}

/* แปลงพิกัดเป็นเมตรบนระนาบราบ โดยอ้างอิงจุดกำเนิดที่ส่งเข้ามา
 *
 * หนึ่งองศาละติจูดยาวเท่ากันทุกที่ แต่หนึ่งองศาลองจิจูดสั้นลงเมื่อเข้าใกล้ขั้ว
 * จึงต้องคูณด้วย cos ของละติจูด ที่กรุงเทพตัวคูณนี้คือราว 0.9717
 * ถ้าลืมคูณ ระยะทางแนวตะวันออกตะวันตกจะเพี้ยนไปราวสามเปอร์เซ็นต์ */
static void to_local_ne(double lat_deg, double lon_deg,
                        double lat0_deg, double lon0_deg,
                        double cos_lat0,
                        double *north_m, double *east_m)
{
    const double k = NAV_EARTH_RADIUS_M * DEG2RAD;   /* เมตรต่อองศาละติจูด */

    *north_m = (lat_deg - lat0_deg) * k;
    *east_m  = (lon_deg - lon0_deg) * k * cos_lat0;
}

bool nav_los_update(NavPath_t *p, double lat_deg, double lon_deg,
                    NavGuidance_t *out)
{
    if (p == NULL || out == NULL || p->wp == NULL) {
        return false;
    }
    if (p->wp_count < 2 || p->wp_index >= p->wp_count) {
        return false;
    }

    /* วนหาเพราะอาจผ่าน waypoint หลายจุดรวดเดียวถ้าข้อมูลขาดหายไปนาน
     * หรือถ้า waypoint สองจุดวางใกล้กันมาก */
    for (;;) {
        const NavWaypoint_t *A = &p->wp[p->wp_index - 1];
        const NavWaypoint_t *B = &p->wp[p->wp_index];

        /* ใช้ A เป็นจุดกำเนิดของระนาบท้องถิ่น ทำให้ระยะทุกตัวเล็ก
         * และความคลาดเคลื่อนจากการสมมติว่าโลกแบนน้อยที่สุด */
        const double cos_lat0 = cos(A->lat_deg * DEG2RAD);

        double nB, eB, nP, eP;
        to_local_ne(B->lat_deg, B->lon_deg, A->lat_deg, A->lon_deg,
                    cos_lat0, &nB, &eB);
        to_local_ne(lat_deg, lon_deg, A->lat_deg, A->lon_deg,
                    cos_lat0, &nP, &eP);

        double seg_len = sqrt(nB * nB + eB * eB);

        /* waypoint สองจุดทับกัน ข้ามไปจุดถัดไป */
        if (seg_len < SEGMENT_MIN_LEN_M) {
            if (p->wp_index + 1 < p->wp_count) {
                p->wp_index++;
                continue;
            }
            p->finished = true;
            seg_len = SEGMENT_MIN_LEN_M;
        }

        /* ทิศของเส้นทาง วัดจากทิศเหนือ เพิ่มตามเข็มนาฬิกา
         * atan2 รับ (east, north) ตามลำดับนี้ ไม่ใช่ (y, x) แบบคณิตศาสตร์ปกติ */
        double chi_p = atan2(eB, nB);

        /* ฉายตำแหน่งเรือลงบนแกนของเส้นทาง
         *   along  คือเดินมาตามเส้นแล้วเท่าไร
         *   cross  คือออกนอกเส้นไปเท่าไร บวกคือทางขวา */
        double cp = cos(chi_p);
        double sp = sin(chi_p);
        double along =  nP * cp + eP * sp;
        double cross = -nP * sp + eP * cp;

        double dn = nB - nP;
        double de = eB - eP;
        double dist_to_wp = sqrt(dn * dn + de * de);

        /* ถึง waypoint แล้วหรือยัง
         * ใช้ระยะที่เหลือตามแนวเส้นทางเป็นเกณฑ์ ไม่ใช่ระยะตรงถึง B
         * เพราะถ้าเรือเลยจุด B ไปแล้วแต่ออกนอกเส้นมาก ระยะตรงจะยังไกล
         * แล้วเรือจะวนกลับมาหา B ไม่รู้จบ ซึ่งเป็นอาการพังที่พบบ่อยที่สุด */
        if (!p->finished && (seg_len - along) <= (double)p->accept_radius_m) {
            if (p->wp_index + 1 < p->wp_count) {
                p->wp_index++;
                continue;              /* คำนวณใหม่กับช่วงถัดไปทันที */
            }
            p->finished = true;        /* นี่คือ waypoint สุดท้าย */
        }

        /* หัวใจของ LOS
         * เรืออยู่ขวาของเส้น (cross บวก) ต้องหันซ้าย มุมที่บวกเข้าไปจึงติดลบ
         * ยิ่ง cross มาก มุมยิ่งชัน แต่ไม่มีทางเกิน 90 องศาเพราะเป็น atan */
        double chi_d = chi_p + atan2(-cross, (double)p->lookahead_m);

        out->desired_heading_deg = nav_wrap360((float)(chi_d  * RAD2DEG));
        out->path_bearing_deg    = nav_wrap360((float)(chi_p  * RAD2DEG));
        out->cross_track_error_m = (float)cross;
        out->along_track_m       = (float)along;
        out->segment_length_m    = (float)seg_len;
        out->dist_to_wp_m        = (float)dist_to_wp;
        out->wp_index            = p->wp_index;
        out->finished            = p->finished;

        return true;
    }
}
