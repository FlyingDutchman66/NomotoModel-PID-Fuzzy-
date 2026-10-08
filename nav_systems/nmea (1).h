/**
 * nmea.h
 * ---------------------------------------------------------------------------
 * ชั้นตรรกะล้วน (hardware-agnostic) สำหรับแกะข้อความ NMEA 0183
 * ไม่มีการเรียกฮาร์ดแวร์ใด ๆ คอมไพล์ได้ทั้งบน PC, STM32 และ ESP32
 * ---------------------------------------------------------------------------
 */

#ifndef NMEA_H
#define NMEA_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ความยาวสูงสุดของประโยค NMEA 0183 ตามมาตรฐานคือ 82 ไบต์รวม CR LF */
#define NMEA_MAX_SENTENCE_LEN 82

/* --------------------------------------------------------------------------
 * GpsRmc_t
 * ผลลัพธ์ที่แกะได้จากประโยค RMC หนึ่งประโยค
 *
 * นี่คือ "ข้อมูลดิบระดับเซ็นเซอร์" ยังไม่ใช่ NavState_t ที่ส่งให้ Tankhun
 * เพราะ NavState_t เป็นผลลัพธ์หลังรวมกับ IMU และคำนวณ guidance แล้ว
 * -------------------------------------------------------------------------- */
typedef struct {
    double  lat_deg;      /* องศาทศนิยม + = North */
    double  lon_deg;      /* องศาทศนิยม + = East  */

    float   sog_mps;      /* Speed Over Ground แปลงจากนอตเป็นเมตรต่อวินาทีแล้ว */

    float   cog_deg;      /* Course Over Ground เทียบทิศเหนือจริง [0,360)   */
    bool    cog_valid;    /* false = เรือนิ่งเกินกว่าที่ GPS จะบอกทิศได้     */

    uint8_t utc_hour;     /* เวลา UTC ไม่ใช่เวลาไทย (ไทย = UTC + 7)         */
    uint8_t utc_min;
    uint8_t utc_sec;
} GpsRmc_t;

/**
 * ตรวจสอบ checksum ของประโยค NMEA หนึ่งประโยค
 *
 * sentence : สตริงที่ขึ้นต้นด้วย '$' และมี "*HH" ปิดท้าย
 *            (จะมี \r\n ต่อท้ายหรือไม่ก็ได้)
 * return   : true  = ประโยคสมบูรณ์ เอาไปใช้ต่อได้
 *            false = ประโยคเสียหาย ต้องทิ้งทันที
 */
bool nmea_checksum_ok(const char *sentence);

/**
 * แปลงพิกัดรูปแบบ NMEA (ddmm.mmmm) ให้เป็นองศาทศนิยม
 *
 * field    : สตริงพิกัดดิบ เช่น "4807.038" หรือ "01131.000"
 * hemi     : ซีกโลก 'N' 'S' 'E' หรือ 'W'
 * out_deg  : ตัวแปรปลายทาง หน่วยองศา (ใต้และตะวันตกเป็นค่าลบ)
 * return   : true = แปลงสำเร็จ, false = ข้อมูลผิดรูปแบบ (out_deg ไม่ถูกแตะ)
 */
bool nmea_coord_to_deg(const char *field, char hemi, double *out_deg);

/**
 * ดึงฟิลด์ลำดับที่ index ออกจากประโยค NMEA
 *
 * นับจุลภาคเอง ไม่ใช้ strtok เพราะ strtok รวบจุลภาคที่ติดกันเป็นตัวเดียว
 * ทำให้ลำดับฟิลด์เลื่อนเมื่อ GPS ส่งฟิลด์ว่างมาติด ๆ กัน
 *
 * index    : 0 = ชื่อประโยค (เช่น "GPRMC" ไม่รวม '$'), 1 = ฟิลด์ถัดไป
 * out      : บัฟเฟอร์ปลายทาง จะได้สตริงว่างถ้าฟิลด์นั้นว่าง
 * out_size : ขนาดบัฟเฟอร์รวมที่ว่างสำหรับ '\0'
 * return   : true = มีฟิลด์นั้นอยู่จริง (อาจเป็นสตริงว่างได้)
 *            false = ประโยคสั้นเกินไป หรือฟิลด์ยาวเกินบัฟเฟอร์
 */
bool nmea_get_field(const char *sentence, int index,
                    char *out, int out_size);

/**
 * แกะประโยค RMC ทั้งประโยคให้เป็นข้อมูลที่ใช้งานได้
 *
 * ฟังก์ชันนี้ตรวจ checksum ให้เองในตัว ผู้เรียกไม่ต้องตรวจซ้ำ
 * รับได้ทั้ง $GPRMC $GNRMC $GLRMC เพราะดูแค่สามตัวท้ายว่าเป็น RMC
 *
 * out      : ผลลัพธ์ จะถูกเขียนก็ต่อเมื่อ return true เท่านั้น
 * return   : true  = ประโยคถูกต้องและ GPS จับดาวเทียมได้แล้ว (status = A)
 *            false = ประโยคเสีย ไม่ใช่ RMC หรือยังไม่มี fix (status = V)
 */
bool nmea_parse_rmc(const char *sentence, GpsRmc_t *out);

#ifdef __cplusplus
}
#endif

#endif /* NMEA_H */
