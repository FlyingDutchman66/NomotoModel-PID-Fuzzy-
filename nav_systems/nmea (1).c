#include "nmea.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* แปลงอักษรฐานสิบหกหนึ่งตัวเป็นตัวเลข 0-15, คืน -1 ถ้าไม่ใช่ฐานสิบหก */
static int hex_value(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

bool nmea_checksum_ok(const char *sentence)
{
    if (sentence == NULL || sentence[0] != '$') {
        return false;
    }

    /* XOR ทุกตัวอักษรระหว่าง '$' กับ '*' */
    uint8_t sum = 0;
    int i = 1;
    while (sentence[i] != '\0' && sentence[i] != '*') {
        sum ^= (uint8_t)sentence[i];
        i++;
    }

    /* ต้องเจอ '*' แล้วตามด้วยเลขฐานสิบหกอีกสองตัว */
    if (sentence[i] != '*') {
        return false;
    }
    if (sentence[i + 1] == '\0' || sentence[i + 2] == '\0') {
        return false;
    }

    int hi = hex_value(sentence[i + 1]);
    int lo = hex_value(sentence[i + 2]);
    if (hi < 0 || lo < 0) {
        return false;
    }

    uint8_t expected = (uint8_t)((hi << 4) | lo);
    return sum == expected;
}

bool nmea_coord_to_deg(const char *field, char hemi, double *out_deg)
{
    if (field == NULL || out_deg == NULL || field[0] == '\0') {
        return false;
    }

    /* หาจุดทศนิยม ตัวเลขสองหลักก่อนจุดคือ "ลิปดา" ที่เหลือข้างหน้าคือ "องศา" */
    const char *dot = strchr(field, '.');
    if (dot == NULL) {
        return false;
    }

    int deg_digits = (int)(dot - field) - 2;
    if (deg_digits < 1 || deg_digits > 3) {  /* ละติจูด 2 หลัก ลองจิจูด 3 หลัก */
        return false;
    }

    char deg_buf[4] = {0};
    for (int i = 0; i < deg_digits; i++) {
        if (field[i] < '0' || field[i] > '9') {
            return false;
        }
        deg_buf[i] = field[i];
    }

    double degrees = atof(deg_buf);

    /* ส่วนลิปดา ต้องขึ้นต้นด้วยตัวเลข และต้องถูกอ่านจนหมดสตริง
     * (atof คืน 0.0 เงียบ ๆ เมื่อเจอขยะ จึงใช้ strtod แล้วเช็กตัวชี้ปลายทางแทน) */
    const char *min_str = field + deg_digits;
    if (min_str[0] < '0' || min_str[0] > '9') {
        return false;
    }

    char *min_end = NULL;
    double minutes = strtod(min_str, &min_end);
    if (min_end == min_str || *min_end != '\0') {
        return false;
    }

    if (minutes < 0.0 || minutes >= 60.0) {
        return false;
    }

    double result = degrees + (minutes / 60.0);

    if (hemi == 'S' || hemi == 'W') {
        result = -result;
    } else if (hemi != 'N' && hemi != 'E') {
        return false;
    }

    if (result < -180.0 || result > 180.0) {
        return false;
    }

    *out_deg = result;
    return true;
}

/* ==========================================================================
 * ส่วนที่ 2 : แยกฟิลด์และแกะประโยค RMC
 * ========================================================================== */

/* จุดสิ้นสุดของเนื้อหาในประโยค คือก่อน '*' หรือก่อน CR LF หรือท้ายสตริง */
static bool is_end_of_payload(char c)
{
    return (c == '\0' || c == '*' || c == '\r' || c == '\n');
}

bool nmea_get_field(const char *sentence, int index, char *out, int out_size)
{
    if (sentence == NULL || out == NULL || out_size < 1 || index < 0) {
        return false;
    }
    if (sentence[0] != '$') {
        return false;
    }

    int i = 1;            /* ข้าม '$' */
    int field_no = 0;

    /* เดินข้ามฟิลด์ไปจนถึงฟิลด์ที่ต้องการ โดยนับจุลภาคเองทีละตัว */
    while (field_no < index) {
        if (is_end_of_payload(sentence[i])) {
            return false;         /* ประโยคสั้นกว่าที่ขอ */
        }
        if (sentence[i] == ',') {
            field_no++;
        }
        i++;
    }

    /* คัดลอกเนื้อฟิลด์จนกว่าจะเจอจุลภาคถัดไปหรือจบประโยค */
    int n = 0;
    while (!is_end_of_payload(sentence[i]) && sentence[i] != ',') {
        if (n >= out_size - 1) {
            out[0] = '\0';
            return false;         /* ฟิลด์ยาวเกินบัฟเฟอร์ */
        }
        out[n] = sentence[i];
        n++;
        i++;
    }

    out[n] = '\0';
    return true;                  /* สตริงว่างก็ถือว่าสำเร็จ */
}

/* แปลงสตริงเป็น float แบบเข้มงวด ต้องอ่านครบทั้งสตริงจึงจะยอมรับ */
static bool parse_float_strict(const char *s, float *out)
{
    if (s == NULL || out == NULL || s[0] == '\0') {
        return false;
    }
    char *end = NULL;
    double v = strtod(s, &end);
    if (end == s || *end != '\0') {
        return false;
    }
    *out = (float)v;
    return true;
}

/* แปลงตัวเลขสองหลักที่ตำแหน่ง s[0] s[1] เป็นจำนวนเต็ม */
static bool parse_two_digits(const char *s, uint8_t *out)
{
    if (s[0] < '0' || s[0] > '9' || s[1] < '0' || s[1] > '9') {
        return false;
    }
    *out = (uint8_t)((s[0] - '0') * 10 + (s[1] - '0'));
    return true;
}

bool nmea_parse_rmc(const char *sentence, GpsRmc_t *out)
{
    if (sentence == NULL || out == NULL) {
        return false;
    }
    if (!nmea_checksum_ok(sentence)) {
        return false;
    }

    char buf[16];

    /* ---- ฟิลด์ 0 : ชนิดประโยค ----
     * รับทุก talker ID ที่ลงท้ายด้วย RMC เช่น GPRMC GNRMC GLRMC
     * เพราะโมดูลหลายกลุ่มดาวเทียมจะเปลี่ยนไปใช้ GN เมื่อรวมสัญญาณ */
    if (!nmea_get_field(sentence, 0, buf, sizeof(buf))) {
        return false;
    }
    if (strlen(buf) != 5 || strcmp(buf + 2, "RMC") != 0) {
        return false;
    }

    /* ---- ฟิลด์ 2 : สถานะ A = ใช้ได้ , V = ยังไม่มี fix ---- */
    if (!nmea_get_field(sentence, 2, buf, sizeof(buf))) {
        return false;
    }
    if (buf[0] != 'A') {
        return false;
    }

    GpsRmc_t tmp;
    memset(&tmp, 0, sizeof(tmp));

    /* ---- ฟิลด์ 3 และ 4 : ละติจูด ---- */
    char lat_raw[16], lat_hemi[4];
    if (!nmea_get_field(sentence, 3, lat_raw,  sizeof(lat_raw)))  return false;
    if (!nmea_get_field(sentence, 4, lat_hemi, sizeof(lat_hemi))) return false;
    if (!nmea_coord_to_deg(lat_raw, lat_hemi[0], &tmp.lat_deg))   return false;
    if (tmp.lat_deg < -90.0 || tmp.lat_deg > 90.0)                return false;

    /* ---- ฟิลด์ 5 และ 6 : ลองจิจูด ---- */
    char lon_raw[16], lon_hemi[4];
    if (!nmea_get_field(sentence, 5, lon_raw,  sizeof(lon_raw)))  return false;
    if (!nmea_get_field(sentence, 6, lon_hemi, sizeof(lon_hemi))) return false;
    if (!nmea_coord_to_deg(lon_raw, lon_hemi[0], &tmp.lon_deg))   return false;

    /* ---- ฟิลด์ 7 : ความเร็ว หน่วยนอต ต้องแปลงเป็นเมตรต่อวินาที ----
     * 1 นอต = 1 ไมล์ทะเลต่อชั่วโมง = 1852 / 3600 = 0.514444 m/s
     * ถ้าฟิลด์ว่างให้ถือว่านิ่ง เพราะบางโมดูลละฟิลด์นี้ตอนจอด */
    if (!nmea_get_field(sentence, 7, buf, sizeof(buf))) {
        return false;
    }
    if (buf[0] == '\0') {
        tmp.sog_mps = 0.0f;
    } else {
        float knots;
        if (!parse_float_strict(buf, &knots) || knots < 0.0f) {
            return false;
        }
        tmp.sog_mps = knots * 0.514444f;
    }

    /* ---- ฟิลด์ 8 : ทิศทางการเคลื่อนที่ ----
     * ว่างได้จริงเมื่อเรือนิ่ง เพราะ GPS คำนวณทิศจากการขยับเท่านั้น
     * ห้ามแทนด้วย 0 เพราะ 0 แปลว่าหันหัวไปทางทิศเหนือพอดี */
    if (!nmea_get_field(sentence, 8, buf, sizeof(buf))) {
        return false;
    }
    if (buf[0] == '\0') {
        tmp.cog_deg   = 0.0f;
        tmp.cog_valid = false;
    } else {
        float cog;
        if (!parse_float_strict(buf, &cog)) {
            return false;
        }
        if (cog < 0.0f || cog >= 360.0f) {
            return false;
        }
        tmp.cog_deg   = cog;
        tmp.cog_valid = true;
    }

    /* ---- ฟิลด์ 1 : เวลา UTC รูปแบบ hhmmss หรือ hhmmss.ss ---- */
    if (!nmea_get_field(sentence, 1, buf, sizeof(buf))) {
        return false;
    }
    if (strlen(buf) < 6) {
        return false;
    }
    if (!parse_two_digits(buf + 0, &tmp.utc_hour)) return false;
    if (!parse_two_digits(buf + 2, &tmp.utc_min))  return false;
    if (!parse_two_digits(buf + 4, &tmp.utc_sec))  return false;
    if (tmp.utc_hour > 23 || tmp.utc_min > 59 || tmp.utc_sec > 60) {
        return false;
    }

    /* ทุกอย่างผ่าน จึงค่อยเขียนลงตัวแปรของผู้เรียก
     * ถ้าเขียนทีละฟิลด์ระหว่างทาง ผู้เรียกจะได้ข้อมูลครึ่ง ๆ กลาง ๆ เมื่อ fail */
    *out = tmp;
    return true;
}
