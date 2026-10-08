/**
 * test_nmea.c
 * ---------------------------------------------------------------------------
 * ชุดทดสอบของ nmea.c — ไฟล์นี้รันบนโน้ตบุ๊กหรือ OnlineGDB เท่านั้น ไม่ขึ้นเรือ
 *
 * บน OnlineGDB : วางไฟล์นี้ในแท็บ main.c, nmea.h และ nmea.c อยู่แท็บของตัวเอง
 * บนเครื่องตัวเอง :
 *     gcc -Wall -Wextra -o test_nmea test_nmea.c nmea.c
 *     ./test_nmea
 *
 * เกณฑ์ผ่าน   : ทุกบรรทัดขึ้น PASS และบรรทัดสุดท้ายขึ้น ALL TESTS PASSED
 *               (exit code = 0)
 * เกณฑ์ไม่ผ่าน : มีบรรทัดใดขึ้น FAIL (exit code = 1)
 * ---------------------------------------------------------------------------
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "nmea.h"

static int g_failed = 0;

static void check_bool(const char *label, bool got, bool want)
{
    bool ok = (got == want);
    if (!ok) g_failed++;
    printf("[%s] %-46s got=%-5s want=%s\n",
           ok ? "PASS" : "FAIL", label,
           got  ? "true" : "false",
           want ? "true" : "false");
}

static void check_coord(const char *field, char hemi,
                        bool want_ok, double want_deg)
{
    char label[64];
    snprintf(label, sizeof(label), "coord \"%s\" %c", field, hemi);

    double got_deg = 0.0;
    bool got_ok = nmea_coord_to_deg(field, hemi, &got_deg);

    bool ok = (got_ok == want_ok);
    if (ok && want_ok) {
        ok = (fabs(got_deg - want_deg) < 1e-6);
    }
    if (!ok) g_failed++;

    printf("[%s] %-46s got=%.6f want=%.6f\n",
           ok ? "PASS" : "FAIL", label, got_deg, want_deg);
}

static void check_str(const char *label, const char *got, const char *want)
{
    bool ok = (strcmp(got, want) == 0);
    if (!ok) g_failed++;
    printf("[%s] %-46s got=\"%s\" want=\"%s\"\n",
           ok ? "PASS" : "FAIL", label, got, want);
}

static void check_float(const char *label, double got, double want, double tol)
{
    bool ok = (fabs(got - want) <= tol);
    if (!ok) g_failed++;
    printf("[%s] %-46s got=%.6f want=%.6f\n",
           ok ? "PASS" : "FAIL", label, got, want);
}

/* ประโยคทดสอบ พิกัดแถวอ่าวไทย ความเร็ว 4.2 นอต ทิศ 087.5 องศา */
#define RMC_GOOD   "$GPRMC,085932.00,A,1345.3780,N,10030.1080,E,004.2,087.5,071026,,,A*51"
#define RMC_GN     "$GNRMC,085932.00,A,1345.3780,N,10030.1080,E,004.2,087.5,071026,,,A*4F"
#define RMC_NOFIX  "$GPRMC,085933.00,V,,,,,,,071026,,,N*7B"
#define RMC_STILL  "$GPRMC,085934.00,A,1345.3780,N,10030.1080,E,000.0,,071026,,,A*75"
#define GGA_OTHER  "$GPGGA,085932.00,1345.3780,N,10030.1080,E,1,09,0.9,12.5,M,0.0,M,,*6B"
#define RMC_FAST   "$GPRMC,085932.00,A,1345.3780,N,10030.1080,E,012.0,359.9,071026,,,A*58"

int main(void)
{
    printf("=== nmea_checksum_ok ===\n");

    check_bool("valid sentence (classic example)",
        nmea_checksum_ok("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A"),
        true);
    check_bool("valid sentence (Gulf of Thailand)",
        nmea_checksum_ok(RMC_GOOD), true);
    check_bool("lowercase hex checksum",
        nmea_checksum_ok("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6a"),
        true);
    check_bool("no-fix sentence is still well formed",
        nmea_checksum_ok(RMC_NOFIX), true);
    check_bool("corrupted payload must be rejected",
        nmea_checksum_ok("$GPRMC,123519,A,4807.039,N,01131.000,E,022.4,084.4,230394,003.1,W*6A"),
        false);
    check_bool("missing dollar sign",
        nmea_checksum_ok("GPRMC,123519,A*6A"), false);
    check_bool("missing asterisk",
        nmea_checksum_ok("$GPRMC,123519,A"), false);
    check_bool("truncated after asterisk",
        nmea_checksum_ok("$GPRMC,123519,A*6"), false);
    check_bool("non-hex checksum characters",
        nmea_checksum_ok("$GPRMC,123519,A*ZZ"), false);
    check_bool("null pointer",
        nmea_checksum_ok(NULL), false);

    printf("\n=== nmea_coord_to_deg ===\n");

    check_coord("4807.038",   'N', true,   48.117300);
    check_coord("01131.000",  'E', true,   11.516667);
    check_coord("1345.3780",  'N', true,   13.756300);
    check_coord("10030.1080", 'E', true,  100.501800);
    check_coord("3751.65",    'S', true,  -37.860833);
    check_coord("01131.000",  'W', true,  -11.516667);
    check_coord("",           'N', false, 0.0);
    check_coord("123456",     'N', false, 0.0);
    check_coord("4860.000",   'N', false, 0.0);
    check_coord("4807.038",   'X', false, 0.0);
    check_coord("48ab.038",   'N', false, 0.0);

    printf("\n=== nmea_get_field ===\n");
    {
        char f[16];

        nmea_get_field(RMC_GOOD, 0, f, sizeof(f));
        check_str("field 0 is sentence type", f, "GPRMC");

        nmea_get_field(RMC_GOOD, 2, f, sizeof(f));
        check_str("field 2 is status", f, "A");

        nmea_get_field(RMC_GOOD, 3, f, sizeof(f));
        check_str("field 3 is raw latitude", f, "1345.3780");

        nmea_get_field(RMC_GOOD, 8, f, sizeof(f));
        check_str("field 8 is course over ground", f, "087.5");

        /* หัวใจของการไม่ใช้ strtok : ฟิลด์ว่างต้องยังนับลำดับถูก */
        nmea_get_field(RMC_NOFIX, 3, f, sizeof(f));
        check_str("empty field 3 stays empty, not shifted", f, "");
        nmea_get_field(RMC_NOFIX, 9, f, sizeof(f));
        check_str("field 9 after many empties is the date", f, "071026");

        check_bool("index past end of sentence",
            nmea_get_field(RMC_GOOD, 40, f, sizeof(f)), false);

        char tiny[4];
        check_bool("field longer than buffer is rejected",
            nmea_get_field(RMC_GOOD, 3, tiny, sizeof(tiny)), false);
    }

    printf("\n=== nmea_parse_rmc ===\n");
    {
        GpsRmc_t fix;

        memset(&fix, 0, sizeof(fix));
        check_bool("parse good GPRMC", nmea_parse_rmc(RMC_GOOD, &fix), true);
        check_float("  latitude  [deg]", fix.lat_deg,  13.756300, 1e-6);
        check_float("  longitude [deg]", fix.lon_deg, 100.501800, 1e-6);
        check_float("  speed     [m/s]", fix.sog_mps,   2.160665, 1e-4);
        check_float("  course    [deg]", fix.cog_deg,  87.500000, 1e-4);
        check_bool ("  course is usable", fix.cog_valid, true);
        check_float("  utc hour",   fix.utc_hour,  8, 0);
        check_float("  utc minute", fix.utc_min,  59, 0);
        check_float("  utc second", fix.utc_sec,  32, 0);

        memset(&fix, 0, sizeof(fix));
        check_bool("parse GNRMC (multi-constellation)",
            nmea_parse_rmc(RMC_GN, &fix), true);
        check_float("  latitude still correct", fix.lat_deg, 13.756300, 1e-6);

        memset(&fix, 0, sizeof(fix));
        check_bool("boat stationary: sentence still parses",
            nmea_parse_rmc(RMC_STILL, &fix), true);
        check_bool("  course flagged unusable", fix.cog_valid, false);
        check_float("  speed is zero", fix.sog_mps, 0.0, 1e-6);

        memset(&fix, 0, sizeof(fix));
        check_bool("course 359.9 is in range",
            nmea_parse_rmc(RMC_FAST, &fix), true);
        check_float("  course", fix.cog_deg, 359.9, 1e-3);

        check_bool("status V (no fix) is rejected",
            nmea_parse_rmc(RMC_NOFIX, &fix), false);
        check_bool("GGA sentence is not an RMC",
            nmea_parse_rmc(GGA_OTHER, &fix), false);
        check_bool("bad checksum is rejected before parsing",
            nmea_parse_rmc("$GPRMC,085932.00,A,1345.3780,N,10030.1080,E,004.2,087.5,071026,,,A*00",
                           &fix), false);
        check_bool("null sentence", nmea_parse_rmc(NULL, &fix), false);

        /* ผลลัพธ์ต้องไม่ถูกแตะเมื่อ parse ไม่สำเร็จ */
        memset(&fix, 0, sizeof(fix));
        fix.lat_deg = 999.0;
        nmea_parse_rmc(RMC_NOFIX, &fix);
        check_float("output untouched on failure", fix.lat_deg, 999.0, 0);
    }

    printf("\n");
    if (g_failed == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("%d TEST(S) FAILED\n", g_failed);
    return 1;
}
