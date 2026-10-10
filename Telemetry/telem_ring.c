/**
 * telem_ring.c
 * ---------------------------------------------------------------------------
 * ลำดับคำสั่งในสองฟังก์ชันหลักไม่ใช่เรื่องสไตล์ แต่คือเรื่องความถูกต้อง
 *
 * ใน push ต้องคัดลอกข้อมูลลงช่องให้เสร็จก่อน แล้วจึงขยับ head
 * เพราะ head คือสัญญาณที่บอกผู้บริโภคว่าช่องนี้พร้อมแล้ว ถ้าขยับก่อนคัดลอก
 * ผู้บริโภคที่ทำงานอยู่คนละจังหวะอาจหยิบช่องที่ยังเขียนไม่เสร็จไปเขียนลงการ์ด
 *
 * ใน pop ก็ด้วยเหตุผลกลับกัน ต้องคัดลอกข้อมูลออกให้เสร็จก่อน แล้วจึงขยับ tail
 * เพราะ tail คือสัญญาณที่บอกผู้ผลิตว่าช่องนี้ว่างให้ทับได้แล้ว
 *
 * บน Cortex-M ที่ไม่เรียงลำดับคำสั่งใหม่ข้ามการเขียนหน่วยความจำแบบนี้
 * การประกาศ volatile เพียงพอสำหรับโจทย์นี้ ถ้าย้ายไปสถาปัตยกรรม
 * ที่เรียงลำดับใหม่ได้อิสระกว่า ต้องเพิ่มกำแพงกั้นลำดับหน่วยความจำ
 * ตรงจุดที่คอมเมนต์กำกับไว้ทั้งสองแห่ง
 * ---------------------------------------------------------------------------
 */

#include "telem_ring.h"

#include <string.h>


void telem_ring_init(TelemRing_t *q)
{
    if (q == NULL) {
        return;
    }
    memset(q, 0, sizeof(*q));
}


static uint32_t next_index(uint32_t i)
{
    uint32_t n = i + 1u;
    return (n >= TELEM_RING_SLOTS) ? 0u : n;
}


bool telem_ring_push(TelemRing_t *q, const TelemRecord_t *rec)
{
    if (q == NULL || rec == NULL) {
        return false;
    }

    const uint32_t head = q->head;
    const uint32_t nxt  = next_index(head);

    if (nxt == q->tail) {
        /* ไม่มีที่ว่าง ทิ้งเรคคอร์ดใหม่แล้วนับไว้ ลูปควบคุมไม่เคยรอ */
        q->drops = q->drops + 1u;
        return false;
    }

    q->slot[head] = *rec;

    /* จุดที่ต้องใส่กำแพงกั้นลำดับหน่วยความจำถ้าย้ายสถาปัตยกรรม
     * ข้อมูลต้องอยู่ในช่องครบก่อนที่ head จะประกาศว่าช่องนี้พร้อม */
    q->head = nxt;

    uint32_t fill = (q->head >= q->tail)
                  ? (q->head - q->tail)
                  : (TELEM_RING_SLOTS - q->tail + q->head);
    if (fill > q->high_water) {
        q->high_water = fill;
    }

    return true;
}


bool telem_ring_pop(TelemRing_t *q, TelemRecord_t *out)
{
    if (q == NULL || out == NULL) {
        return false;
    }

    const uint32_t tail = q->tail;

    if (tail == q->head) {
        return false;
    }

    *out = q->slot[tail];

    /* จุดที่ต้องใส่กำแพงกั้นลำดับหน่วยความจำถ้าย้ายสถาปัตยกรรม
     * ข้อมูลต้องถูกคัดลอกออกครบก่อนที่ tail จะประกาศว่าช่องนี้ว่าง */
    q->tail = next_index(tail);

    return true;
}


uint32_t telem_ring_count(const TelemRing_t *q)
{
    if (q == NULL) {
        return 0u;
    }
    const uint32_t h = q->head;
    const uint32_t t = q->tail;
    return (h >= t) ? (h - t) : (TELEM_RING_SLOTS - t + h);
}


bool telem_ring_is_full(const TelemRing_t *q)
{
    if (q == NULL) {
        return false;
    }
    return next_index(q->head) == q->tail;
}
