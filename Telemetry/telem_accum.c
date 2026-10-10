/**
 * telem_accum.c
 * ---------------------------------------------------------------------------
 * เรื่องที่ยากที่สุดในไฟล์นี้ไม่ใช่การอินทิเกรต แต่คือการนับระยะคืบหน้า
 * เมื่อเรือสลับเลก
 *
 * พิกัดตามแนวเส้นทางที่ชั้น guidance ให้มา เป็นค่าที่นับจากต้นเลกปัจจุบัน
 * พอเรือข้าม waypoint ค่านี้จะกระโดดกลับไปใกล้ศูนย์เพราะเริ่มนับเลกใหม่
 * ถ้าเอาผลต่างมาบวกตรง ๆ โดยไม่ระวัง ระยะคืบหน้าสะสมจะถูกหักออกไป
 * เท่ากับความยาวเลกที่เพิ่งวิ่งจบ ซึ่งแปลว่ายิ่งวิ่งไกลตัวเลขยิ่งน้อย
 *
 * โค้ดนี้จึงเฝ้า wp_index ไว้ และเมื่อมันเปลี่ยน จะตั้งค่าอ้างอิงใหม่
 * โดยไม่บวกอะไรในจังหวะนั้น ผลคือเสียระยะไปเสี้ยวหนึ่งของหนึ่งตัวอย่าง
 * ต่อหนึ่ง waypoint ซึ่งที่ 10 Hz และความเร็ว 2.3 เมตรต่อวินาที
 * คือราว 0.23 เมตรต่อจุด เทียบกับเส้นทางหลายร้อยเมตรแล้วไม่มีผล
 * แต่ต้องรู้ว่ามันมีอยู่ จึงมีตัวนับ leg_changes ไว้ให้คำนวณขอบเขตได้
 *
 * อีกเรื่องที่ตั้งใจเลือก คือระยะที่เรือแล่นคำนวณจากการอินทิเกรตความเร็ว
 * ไม่ใช่จากการบวกระยะระหว่างพิกัดสองจุดที่ติดกัน
 * เหตุผลคือความเร็วที่ GPS ให้มาได้จากการวัดการเลื่อนดอปเปลอร์ของคลื่นวิทยุ
 * ซึ่งนิ่งกว่าการหาผลต่างของตำแหน่งมาก ตำแหน่งมีสัญญาณรบกวนราวสองเมตร
 * ถ้าเอาสองจุดที่ห่างกันหนึ่งวินาทีมาลบกัน สัญญาณรบกวนนั้นจะกลายเป็น
 * ระยะทางปลอมที่บวกเพิ่มเรื่อย ๆ และยิ่งสุ่มตัวอย่างถี่ยิ่งเพี้ยนมาก
 * ซึ่งเป็นกับดักที่ทำให้ระยะทางดูไกลกว่าจริงในระบบที่ทำแบบนั้น
 * ---------------------------------------------------------------------------
 */

#include "telem_accum.h"

#include <string.h>

/* วัตต์คูณวินาที หารด้วยค่านี้ได้วัตต์ชั่วโมง */
#define WS_PER_WH   3600.0f


void telem_accum_init(TelemAccum_t *a)
{
    if (a == NULL) {
        return;
    }
    memset(a, 0, sizeof(*a));
}


/* อินทิเกรตหนึ่งช่วงด้วยกฎสี่เหลี่ยมคางหมู
 *
 * เมื่อตัวอย่างก่อนหน้าใช้ไม่ได้ จะถอยไปใช้กฎสี่เหลี่ยมผืนผ้าด้วยค่าปัจจุบัน
 * ซึ่งคือสิ่งที่ถูกต้องที่สุดเท่าที่ทำได้ เพราะเราไม่รู้ว่าค่าก่อนหน้าเป็นเท่าไร */
static float integrate_step(bool now_ok, float now_v,
                            bool prev_ok, float prev_v,
                            float dt_s)
{
    if (!now_ok) {
        return 0.0f;
    }
    if (prev_ok) {
        return 0.5f * (prev_v + now_v) * dt_s;
    }
    return now_v * dt_s;
}


void telem_accum_update(TelemAccum_t *a, const TelemAccumIn_t *in)
{
    if (a == NULL || in == NULL) {
        return;
    }

    const float p_prop_w  = in->vbat_v * in->i_prop_a;
    const float p_steer_w = in->vbat_v * in->i_steer_a;

    /* ตัวอย่างแรก ตั้งค่าอ้างอิงแล้วออก ยังไม่มีช่วงเวลาให้อินทิเกรต */
    if (!a->started) {
        a->started        = true;
        a->t_prev_ms      = in->t_ms;

        a->p_prop_prev_w  = p_prop_w;
        a->p_prop_prev_ok = in->i_prop_valid;
        a->p_steer_prev_w = p_steer_w;
        a->p_steer_prev_ok = in->i_steer_valid;
        a->sog_prev_mps   = in->sog_mps;
        a->sog_prev_ok    = in->sog_valid;

        if (in->along_valid) {
            a->leg_started  = true;
            a->leg_index    = in->wp_index;
            a->along_prev_m = in->along_m;
        }
        return;
    }

    /* การลบแบบ unsigned ให้ผลต่างที่ถูกต้องแม้ตัวนับเวลาวนรอบ */
    const uint32_t dt_raw = (uint32_t)(in->t_ms - a->t_prev_ms);

    if (dt_raw == 0u) {
        /* เรียกซ้ำด้วยเวลาเดิม ไม่มีอะไรให้สะสม แต่ยังรับค่าใหม่ไว้เป็นค่าอ้างอิง
         * เพื่อให้ตัวอย่างถัดไปใช้กฎสี่เหลี่ยมคางหมูกับค่าที่สดที่สุด */
        a->p_prop_prev_w   = p_prop_w;
        a->p_prop_prev_ok  = in->i_prop_valid;
        a->p_steer_prev_w  = p_steer_w;
        a->p_steer_prev_ok = in->i_steer_valid;
        a->sog_prev_mps    = in->sog_mps;
        a->sog_prev_ok     = in->sog_valid;
        return;
    }

    uint32_t dt_used = dt_raw;
    if (dt_used > TELEM_ACCUM_DT_MAX_MS) {
        dt_used    = TELEM_ACCUM_DT_MAX_MS;
        a->gap_ms += (dt_raw - dt_used);
    }

    const float dt_s = (float)dt_used * 0.001f;
    a->total_ms += dt_used;

    /* ---- พลังงาน ---- */
    if (in->i_prop_valid) {
        a->e_prop_wh += integrate_step(true, p_prop_w,
                                       a->p_prop_prev_ok, a->p_prop_prev_w,
                                       dt_s) / WS_PER_WH;
    } else {
        a->prop_blind_ms += dt_used;
    }

    if (in->i_steer_valid) {
        a->e_steer_wh += integrate_step(true, p_steer_w,
                                        a->p_steer_prev_ok, a->p_steer_prev_w,
                                        dt_s) / WS_PER_WH;
    } else {
        a->steer_blind_ms += dt_used;
    }

    /* ---- ระยะที่เรือแล่นจริง ---- */
    if (in->sog_valid) {
        a->s_ground_m += integrate_step(true, in->sog_mps,
                                        a->sog_prev_ok, a->sog_prev_mps,
                                        dt_s);
    }

    /* ---- ระยะคืบหน้าตามเส้นทาง ---- */
    if (in->along_valid) {
        if (!a->leg_started) {
            a->leg_started  = true;
            a->leg_index    = in->wp_index;
            a->along_prev_m = in->along_m;
        } else if (in->wp_index != a->leg_index) {
            /* สลับเลก ตั้งค่าอ้างอิงใหม่โดยไม่บวก ไม่เช่นนั้นจะหักออกเท่าความยาวเลกเดิม */
            a->leg_index    = in->wp_index;
            a->along_prev_m = in->along_m;
            a->leg_changes += 1u;
        } else {
            /* ยอมให้ติดลบได้ ถ้าคลื่นผลักเรือถอยหลังจริง นั่นคือความคืบหน้าติดลบจริง */
            a->s_along_m   += (in->along_m - a->along_prev_m);
            a->along_prev_m = in->along_m;
        }
    } else {
        a->along_blind_ms += dt_used;
    }

    /* ---- เก็บค่าปัจจุบันไว้เป็นค่าก่อนหน้าของรอบถัดไป ---- */
    a->t_prev_ms       = in->t_ms;
    a->p_prop_prev_w   = p_prop_w;
    a->p_prop_prev_ok  = in->i_prop_valid;
    a->p_steer_prev_w  = p_steer_w;
    a->p_steer_prev_ok = in->i_steer_valid;
    a->sog_prev_mps    = in->sog_mps;
    a->sog_prev_ok     = in->sog_valid;
}


void telem_accum_apply(const TelemAccum_t *a, TelemRecord_t *r)
{
    if (a == NULL || r == NULL) {
        return;
    }
    r->e_prop_wh  = a->e_prop_wh;
    r->e_steer_wh = a->e_steer_wh;
    r->s_along_m  = a->s_along_m;
    r->s_ground_m = a->s_ground_m;
}


bool telem_accum_wh_per_km(const TelemAccum_t *a, float min_along_m,
                           float *out_wh_per_km)
{
    if (a == NULL || out_wh_per_km == NULL) {
        return false;
    }
    if (min_along_m < 1.0f) {
        min_along_m = 1.0f;
    }
    if (a->s_along_m < min_along_m) {
        return false;
    }

    const float wh = a->e_prop_wh + a->e_steer_wh;
    *out_wh_per_km = wh * 1000.0f / a->s_along_m;
    return true;
}


bool telem_accum_blind_fraction(const TelemAccum_t *a, float *out_fraction)
{
    if (a == NULL || out_fraction == NULL) {
        return false;
    }
    if (a->total_ms == 0u) {
        return false;
    }
    *out_fraction = (float)a->prop_blind_ms / (float)a->total_ms;
    return true;
}
