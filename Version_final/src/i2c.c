#include "i2c.h"
#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "console.h"

extern I2C_HandleTypeDef hi2c1;    // défini dans setup.c

/*
 * i2c_init : ajuste le timing de l'I2C1 selon la fréquence réelle de PCLK1 (100 kHz pour 80 MHz ou 4 MHz)
 * et affiche cette fréquence sur la console.
 */
// I2C1 (PB8 = SCL, PB9 = SDA) : broches et horloge déjà gérées par setup.c
void i2c_init(void)
{
    uint32_t f = HAL_RCC_GetPCLK1Freq();
    hi2c1.Init.Timing = (f == 80000000UL) ? 0x10909CEC : 0x00000E14;   // 100 kHz pour 80 MHz ou 4 MHz
    HAL_I2C_Init(&hi2c1);                                              // ré-applique seulement le timing
    console("I2C : PCLK1 = %lu Hz (timing prevu pour 80 MHz ou 4 MHz)\r\n", (unsigned long)f);
}

/*
 * i2c_scan : teste toutes les adresses I2C (1 à 126) et affiche sur la console
 * celles qui répondent, ce qui permet de vérifier quels modules sont branchés.
 */
void i2c_scan(void)                    // liste les modules présents sur le bus
{
    console("Scan I2C :");
    for (uint8_t a = 1; a < 127; a++)
        if (HAL_I2C_IsDeviceReady(&hi2c1, a << 1, 1, 10) == HAL_OK)
            console(" 0x%02X", a);
    console("\r\n");
}

extern I2C_HandleTypeDef hi2c1;

// RTC (DS1307 / DS3231, adresse 0x68)
#define RTC_ADDR (0x68 << 1)                  // HAL attend l'adresse décalée d'un bit

const char *day_name[] = { "", "MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN" };

/*
 * bcd2bin : convertit un nombre codé en BCD (format de la RTC) en entier binaire.
 */
static uint8_t bcd2bin(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }

/*
 * bin2bcd : convertit un entier binaire (0 à 99) en BCD pour l'écrire dans la RTC.
 */
static uint8_t bin2bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

/*
 * rtc_read : lit les 7 registres de la RTC (secondes à année) et remplit la structure DateTime.
 * Retourne 0 si OK, -1 en cas d'erreur d'accès I2C.
 */
static int rtc_read(DateTime *t)              // 0 = OK, -1 = erreur d'accès
{
    uint8_t r[7];
    if (HAL_I2C_Mem_Read(&hi2c1, RTC_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, r, 7, 50) != HAL_OK)
        return -1;
    t->s     = bcd2bin(r[0] & 0x7F);          // bit 7 = horloge arrêtée (DS1307), masqué
    t->m     = bcd2bin(r[1] & 0x7F);
    t->h     = bcd2bin(r[2] & 0x3F);
    t->dow   = bcd2bin(r[3] & 0x07);
    t->day   = bcd2bin(r[4] & 0x3F);
    t->month = bcd2bin(r[5] & 0x1F);
    t->year  = 2000 + bcd2bin(r[6]);
    return 0;
}

/*
 * rtc_write : écrit la date et l'heure de la structure DateTime dans la RTC (conversion en BCD).
 * Retourne 0 si OK, -1 en cas d'erreur.
 */
int rtc_write(const DateTime *t)       // écrire les secondes avec le bit 7 à 0 démarre l'horloge
{
    uint8_t r[7] = { bin2bcd(t->s), bin2bcd(t->m), bin2bcd(t->h), t->dow,
                     bin2bcd(t->day), bin2bcd(t->month), bin2bcd(t->year - 2000) };
    return HAL_I2C_Mem_Write(&hi2c1, RTC_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, r, 7, 50) == HAL_OK ? 0 : -1;
}

/*
 * rtc_load : lit la RTC avant une modification partielle de la date ou de l'heure.
 * Si les valeurs lues sont absurdes (module neuf), repart d'une date valide (01/01/2000 00:00:00).
 * Retourne 0 si OK, -1 en cas d'erreur d'accès.
 */
// Lit l'heure avant modification ; si le module est neuf (valeurs absurdes), repart d'une date valide
int rtc_load(DateTime *t)
{
    if (rtc_read(t) != 0) return -1;
    if (t->h > 23 || t->m > 59 || t->s > 59 || t->day < 1 || t->day > 31 ||
        t->month < 1 || t->month > 12 || t->dow < 1 || t->dow > 7) {
        t->h = t->m = t->s = 0; t->dow = 1; t->day = 1; t->month = 1; t->year = 2000;
    }
    return 0;
}

DateTime now_dt;
uint8_t rtc_fail = 1;                  // 1 tant que la RTC n'a pas répondu

/*
 * rtc_poll : lit la RTC une fois par seconde, mémorise l'heure dans now_dt
 * et met à jour l'indicateur d'erreur rtc_fail.
 */
// Lit la RTC une fois par seconde et mémorise le résultat
void rtc_poll(void)
{
    static uint32_t last;
    static uint8_t first = 1;
    uint32_t now = HAL_GetTick();
    if (!first && now - last < 1000) return;
    first = 0;
    last = now;
    rtc_fail = (rtc_read(&now_dt) != 0);
}

/*
 * timestamp : écrit dans buf la date et l'heure courantes au format "JJ/MM/AAAA HH:MM:SS"
 * (ou "NA" si la RTC est en erreur).
 */
// Horodatage texte pour les messages et, plus tard, les lignes du fichier SD
void timestamp(char *buf, size_t n)
{
    if (rtc_fail)
        snprintf(buf, n, "NA");
    else
        snprintf(buf, n, "%02d/%02d/%04d %02d:%02d:%02d",
                 now_dt.day, now_dt.month, now_dt.year, now_dt.h, now_dt.m, now_dt.s);
}


#define U16(a, i) ((uint16_t)((a)[i] | ((a)[(i) + 1] << 8)))
#define S16(a, i) ((int16_t)U16(a, i))

static uint8_t  bme_addr;
uint8_t bme_ok;
static uint16_t dig_T1, dig_P1;
static int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static uint8_t  dig_H1, dig_H3;
static int16_t  dig_H2, dig_H4, dig_H5;
static int8_t   dig_H6;
static int32_t  t_fine;

/*
 * bme_init : détecte le BME280 sur le bus I2C (adresse 0x76 ou 0x77), vérifie son identifiant (0x60),
 * lit ses coefficients de calibration et configure la mesure (humidité, température, pression x1, mode normal).
 * Retourne 0 si OK, -1 si le capteur est introuvable, -2 si la puce n'est pas un BME280.
 */
int bme_init(void)
{
    uint8_t id = 0, c[24], h1, h[7];

    for (uint8_t a = 0x76; a <= 0x77 && !bme_addr; a++)
        if (HAL_I2C_Mem_Read(&hi2c1, a << 1, 0xD0, I2C_MEMADD_SIZE_8BIT, &id, 1, 100) == HAL_OK)
            bme_addr = a << 1;
    if (!bme_addr) { console("BME280 : non trouve (0x76/0x77)\r\n"); return -1; }

    console("BME280 : trouve en 0x%02X, ID = 0x%02X\r\n", bme_addr >> 1, id);
    if (id != 0x60) { console("Puce differente d'un BME280 (0x58 = BMP280 sans humidite, 0x61 = BME680)\r\n"); return -2; }

    // Coefficients de calibration (propres à chaque puce)
    HAL_I2C_Mem_Read(&hi2c1, bme_addr, 0x88, I2C_MEMADD_SIZE_8BIT, c, 24, 100);
    dig_T1 = U16(c, 0);  dig_T2 = S16(c, 2);  dig_T3 = S16(c, 4);
    dig_P1 = U16(c, 6);  dig_P2 = S16(c, 8);  dig_P3 = S16(c, 10);
    dig_P4 = S16(c, 12); dig_P5 = S16(c, 14); dig_P6 = S16(c, 16);
    dig_P7 = S16(c, 18); dig_P8 = S16(c, 20); dig_P9 = S16(c, 22);

    HAL_I2C_Mem_Read(&hi2c1, bme_addr, 0xA1, I2C_MEMADD_SIZE_8BIT, &h1, 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, bme_addr, 0xE1, I2C_MEMADD_SIZE_8BIT, h, 7, 100);
    dig_H1 = h1;
    dig_H2 = S16(h, 0);
    dig_H3 = h[2];
    dig_H4 = (int16_t)(((int16_t)(int8_t)h[3] << 4) | (h[4] & 0x0F));
    dig_H5 = (int16_t)(((int16_t)(int8_t)h[5] << 4) | (h[4] >> 4));
    dig_H6 = (int8_t)h[6];

    uint8_t v = 0x01;   // humidité x1 (à écrire AVANT ctrl_meas)
    HAL_I2C_Mem_Write(&hi2c1, bme_addr, 0xF2, I2C_MEMADD_SIZE_8BIT, &v, 1, 100);
    v = 0x27;           // temp x1, pression x1, mode normal
    HAL_I2C_Mem_Write(&hi2c1, bme_addr, 0xF4, I2C_MEMADD_SIZE_8BIT, &v, 1, 100);
    return 0;
}

// Formules de compensation du datasheet Bosch (entiers)
/*
 * comp_T : convertit la valeur brute de température du BME280 en centièmes de °C
 * (formule du datasheet Bosch) et calcule t_fine, nécessaire à la compensation de la pression et de l'humidité.
 */
static int32_t comp_T(int32_t adc)
{
    int32_t v1 = (((adc >> 3) - ((int32_t)dig_T1 << 1)) * dig_T2) >> 11;
    int32_t v2 = (((((adc >> 4) - dig_T1) * ((adc >> 4) - dig_T1)) >> 12) * dig_T3) >> 14;
    t_fine = v1 + v2;
    return (t_fine * 5 + 128) >> 8;                 // centièmes de °C
}

/*
 * comp_P : convertit la valeur brute de pression du BME280 en Pa x 256 (formule du datasheet Bosch,
 * calcul sur 64 bits). Doit être appelée après comp_T. Renvoie 0 en cas de division par zéro évitée.
 */
static uint32_t comp_P(int32_t adc)
{
    int64_t v1 = (int64_t)t_fine - 128000;
    int64_t v2 = v1 * v1 * dig_P6;
    v2 += (v1 * dig_P5) << 17;
    v2 += (int64_t)dig_P4 << 35;
    v1 = ((v1 * v1 * dig_P3) >> 8) + ((v1 * dig_P2) << 12);
    v1 = (((int64_t)1 << 47) + v1) * dig_P1 >> 33;
    if (v1 == 0) return 0;
    int64_t p = 1048576 - adc;
    p = (((p << 31) - v2) * 3125) / v1;
    v1 = ((int64_t)dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    v2 = ((int64_t)dig_P8 * p) >> 19;
    p = ((p + v1 + v2) >> 8) + ((int64_t)dig_P7 << 4);
    return (uint32_t)p;                             // Pa x 256
}

/*
 * comp_H : convertit la valeur brute d'humidité du BME280 en %HR x 1024 (formule du datasheet Bosch),
 * bornée entre 0 et 100 %. Doit être appelée après comp_T.
 */
static uint32_t comp_H(int32_t adc)
{
    int32_t v = t_fine - 76800;
    v = (((((adc << 14) - ((int32_t)dig_H4 << 20) - (dig_H5 * v)) + 16384) >> 15) *
         (((((((v * dig_H6) >> 10) * (((v * dig_H3) >> 11) + 32768)) >> 10) + 2097152) * dig_H2 + 8192) >> 14));
    v -= (((((v >> 15) * (v >> 15)) >> 7) * dig_H1) >> 4);
    if (v < 0) v = 0;
    if (v > 419430400) v = 419430400;
    return (uint32_t)(v >> 12);                     // %HR x 1024
}

/*
 * bme_read : lit les mesures brutes du BME280 (8 octets à partir du registre 0xF7), les compense
 * et renvoie la température (°C), la pression (hPa) et l'humidité (%HR).
 * Retourne 0 si OK, -1 en cas d'erreur I2C.
 */
int bme_read(float *t, float *p, float *h)
{
    uint8_t d[8];
    if (HAL_I2C_Mem_Read(&hi2c1, bme_addr, 0xF7, I2C_MEMADD_SIZE_8BIT, d, 8, 100) != HAL_OK)
        return -1;
    int32_t adc_P = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
    int32_t adc_T = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);
    int32_t adc_H = ((int32_t)d[6] << 8) | d[7];
    *t = comp_T(adc_T) / 100.0f;                    // température d'abord : elle calcule t_fine
    *p = comp_P(adc_P) / 256.0f / 100.0f;           // en hPa
    *h = comp_H(adc_H) / 1024.0f;
    return 0;
}
