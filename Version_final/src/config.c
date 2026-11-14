#include "config.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "console.h"
#include "modes.h"
#include "i2c.h"
#include "stm32l4xx_hal.h"
#include "settings.h"

// Paramètres : copie en RAM, sauvegardée en flash (la L476 n'a pas d'EEPROM)
Params params = DEFAULT_PARAMS;

/* ---------- Sauvegarde en flash : dernière page (2 Ko) de la banque 2 ---------- */
#define PARAMS_ADDR   0x080FF800UL
#define PARAMS_PAGE   255
#define PARAMS_MAGIC  (0x57335700UL + sizeof(Params))   // change si la structure change

typedef struct { uint32_t magic; Params p; uint32_t check; } Stored;

/*
 * checksum : calcule une somme de contrôle FNV-1a sur la structure de paramètres,
 * pour détecter une sauvegarde flash corrompue ou absente.
 */
static uint32_t checksum(const Params *p)   // FNV-1a
{
    const uint8_t *b = (const uint8_t *)p;
    uint32_t h = 2166136261UL;
    for (size_t i = 0; i < sizeof(Params); i++) h = (h ^ b[i]) * 16777619UL;
    return h;
}

/*
 * params_load : charge les paramètres depuis la flash au démarrage.
 * Si la sauvegarde est absente ou invalide (mauvais code magique ou somme de contrôle), les valeurs par défaut sont utilisées.
 */
void params_load(void)
{
    const Stored *s = (const Stored *)PARAMS_ADDR;
    if (s->magic == PARAMS_MAGIC && s->check == checksum(&s->p))
        params = s->p;
    else
        params = (Params)DEFAULT_PARAMS;
}

/*
 * params_save : sauvegarde les paramètres courants en flash : efface la page dédiée,
 * puis programme la structure (code magique, paramètres, somme de contrôle) par mots de 64 bits.
 * Affiche une erreur sur la console si l'opération échoue.
 */
void params_save(void)
{
    uint64_t buf[(sizeof(Stored) + 7) / 8];
    Stored st = { PARAMS_MAGIC, params, checksum(&params) };
    memset(buf, 0xFF, sizeof(buf));
    memcpy(buf, &st, sizeof(st));

    FLASH_EraseInitTypeDef er = { FLASH_TYPEERASE_PAGES, FLASH_BANK_2, PARAMS_PAGE, 1 };
    uint32_t page_err;
    int ok;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    ok = HAL_FLASHEx_Erase(&er, &page_err) == HAL_OK;
    for (size_t i = 0; ok && i < sizeof(buf) / 8; i++)
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, PARAMS_ADDR + 8 * i, buf[i]) == HAL_OK;
    HAL_FLASH_Lock();
    if (!ok) console("Sauvegarde flash : erreur\r\n");
}

/* ---------- Table des paramètres numériques ---------- */
typedef struct { const char *name; int32_t *value; int32_t min, max; } ParamDef;

static const ParamDef param_table[] = {
    { "LOG_INTERVAL",  &params.log_interval, LOG_INTERVAL_MIN,  LOG_INTERVAL_MAX },
    { "FILE_MAX_SIZE", &params.file_max_size, FILE_MAX_SIZE_MIN, FILE_MAX_SIZE_MAX },
    { "TIMEOUT",       &params.timeout,      TIMEOUT_MIN,       TIMEOUT_MAX },
    { "LUMIN",         &params.lumin,        0, 1 },
    { "LUMIN_LOW",     &params.lumin_low,    LUMIN_VALUE_MIN,   LUMIN_VALUE_MAX },
    { "LUMIN_HIGH",    &params.lumin_high,   LUMIN_VALUE_MIN,   LUMIN_VALUE_MAX },
    { "TEMP_AIR",      &params.temp_air,     0, 1 },
    { "MIN_TEMP_AIR",  &params.min_temp_air, TEMP_VALUE_MIN,    TEMP_VALUE_MAX },
    { "MAX_TEMP_AIR",  &params.max_temp_air, TEMP_VALUE_MIN,    TEMP_VALUE_MAX },
    { "HYGR",          &params.hygr,         0, 1 },
    { "HYGR_MINT",     &params.hygr_mint,    TEMP_VALUE_MIN,    TEMP_VALUE_MAX },
    { "HYGR_MAXT",     &params.hygr_maxt,    TEMP_VALUE_MIN,    TEMP_VALUE_MAX },
    { "PRESSURE",      &params.pressure,     0, 1 },
    { "PRESSURE_MIN",  &params.pressure_min, PRESSURE_VALUE_MIN, PRESSURE_VALUE_MAX },
    { "PRESSURE_MAX",  &params.pressure_max, PRESSURE_VALUE_MIN, PRESSURE_VALUE_MAX },
};

/*
 * thresholds_ok : vérifie la cohérence des seuils : chaque seuil bas doit rester strictement inférieur
 * au seuil haut correspondant (luminosité, température, température d'hygrométrie, pression).
 * Retourne 1 si tout est cohérent, 0 sinon.
 */
// Un seuil bas doit rester strictement sous le seuil haut correspondant
static int thresholds_ok(void)
{
    return params.lumin_low < params.lumin_high && params.min_temp_air < params.max_temp_air &&
           params.hygr_mint < params.hygr_maxt && params.pressure_min < params.pressure_max;
}

/*
 * param_command : traite une commande "NOM=valeur" pour un paramètre numérique de la table.
 * Vérifie que la valeur est un entier dans la plage autorisée et que les seuils restent cohérents,
 * l'applique, la sauvegarde en flash et répond sur la console.
 * Retourne 1 si le nom est dans la table (commande traitée, même en cas d'erreur), 0 sinon.
 */
// Valide "NOM=valeur", applique et sauvegarde ; retourne 0 si le nom n'est pas dans la table
static int param_command(const char *name, const char *val)
{
    for (size_t i = 0; i < sizeof(param_table) / sizeof(param_table[0]); i++) {
        const ParamDef *d = &param_table[i];
        if (strcmp(name, d->name)) continue;

        char *end;
        long v = strtol(val, &end, 10);
        if (*val == 0 || *end != 0 || v < d->min || v > d->max) {
            console("Valeur invalide (attendu : %ld a %ld)\r\n", (long)d->min, (long)d->max);
            return 1;
        }
        int32_t old = *d->value;
        *d->value = (int32_t)v;
        if (!thresholds_ok()) {
            *d->value = old;
            console("Valeur invalide (le seuil bas doit rester inferieur au seuil haut)\r\n");
            return 1;
        }
        params_save();
        console("OK\r\n");
        return 1;
    }
    return 0;
}

/* ---------- Commandes de date et d'heure ---------- */
/*
 * cmd_clock : commande CLOCK=HH:MM:SS. Vérifie le format, lit la RTC, remplace l'heure
 * puis réécrit la RTC. Conserve la date en cours.
 */
static void cmd_clock(const char *v)          // CLOCK=HH:MM:SS
{
    int h, m, s;
    DateTime t;
    if (sscanf(v, "%d:%d:%d", &h, &m, &s) != 3 || h < 0 || h > 23 || m < 0 || m > 59 || s < 0 || s > 59) {
        console("Format : CLOCK=HH:MM:SS\r\n");
        return;
    }
    if (rtc_load(&t) != 0) { console("RTC : erreur d'acces\r\n"); return; }
    t.h = h; t.m = m; t.s = s;
    if (rtc_write(&t) == 0) console("OK\r\n");
    else console("RTC : erreur d'ecriture\r\n");
}

/*
 * cmd_date : commande DATE=MM,JJ,AAAA. Vérifie le format (année 2000 à 2099), lit la RTC,
 * remplace la date puis réécrit la RTC. Conserve l'heure en cours.
 */
static void cmd_date(const char *v)           // DATE=MOIS,JOUR,ANNEE
{
    int mo, d, y;
    DateTime t;
    if (sscanf(v, "%d,%d,%d", &mo, &d, &y) != 3 || mo < 1 || mo > 12 || d < 1 || d > 31 || y < 2000 || y > 2099) {
        console("Format : DATE=MM,JJ,AAAA (annee 2000 a 2099)\r\n");
        return;
    }
    if (rtc_load(&t) != 0) { console("RTC : erreur d'acces\r\n"); return; }
    t.month = mo; t.day = d; t.year = y;
    if (rtc_write(&t) == 0) console("OK\r\n");
    else console("RTC : erreur d'ecriture\r\n");
}

/*
 * cmd_day : commande DAY=MON..SUN. Convertit le nom du jour de la semaine en numéro (1 à 7),
 * lit la RTC, remplace le jour de la semaine puis réécrit la RTC.
 */
static void cmd_day(const char *v)            // DAY=MON..SUN
{
    DateTime t;
    int d = 0;
    for (int i = 1; i <= 7; i++)
        if (!strcmp(v, day_name[i])) d = i;
    if (!d) { console("Format : DAY=MON|TUE|WED|THU|FRI|SAT|SUN\r\n"); return; }
    if (rtc_load(&t) != 0) { console("RTC : erreur d'acces\r\n"); return; }
    t.dow = d;
    if (rtc_write(&t) == 0) console("OK\r\n");
    else console("RTC : erreur d'ecriture\r\n");
}

/*
 * config_command : interprète une ligne de commande reçue en mode configuration.
 * Sépare le nom et la valeur au signe '=', puis exécute : VERSION, RESET (valeurs par défaut),
 * un paramètre numérique de la table, CLOCK, DATE, DAY ou TIME (affichage de la date et de l'heure).
 * Répond « Commande inconnue » sinon.
 */
static void config_command(char *s)
{
    char *val = strchr(s, '=');
    if (val) *val++ = 0;                     // sépare le nom et la valeur

    if (!strcmp(s, "VERSION"))
        console("3W v0.1 - lot 0001\r\n");
    else if (!strcmp(s, "RESET")) {
        params = (Params)DEFAULT_PARAMS;
        params_save();
        console("Parametres reinitialises\r\n");
    }
    else if (val && param_command(s, val))
        ;                                     // paramètre numérique traité par la table
    else if (val && !strcmp(s, "CLOCK"))
        cmd_clock(val);
    else if (val && !strcmp(s, "DATE"))
        cmd_date(val);
    else if (val && !strcmp(s, "DAY"))
        cmd_day(val);
    else if (!strcmp(s, "TIME")) {            // pratique pour vérifier : affiche la date et l'heure
        char ts[TS_LEN];
        timestamp(ts, sizeof(ts));
        console("%s (%s)\r\n", ts, rtc_fail ? "?" : day_name[now_dt.dow > 7 ? 0 : now_dt.dow]);
    }
    else
        console("Commande inconnue : %s\r\n", s);
}

/*
 * console_poll : vérifie si une ligne de commande complète a été reçue sur la console.
 * En mode configuration, l'exécute (et relance le délai d'inactivité) ; dans les autres modes, la refuse.
 * Libère ensuite le tampon de commande.
 */
void console_poll(void)
{
    char *cmd = console_cmd();
    if (!cmd) return;
    if (mode == MODE_CONFIG) {
        mode_since = HAL_GetTick();          // activité : le délai de 30 min repart de zéro
        config_command(cmd);
    } else {
        console("Refuse : commandes acceptees uniquement en mode configuration\r\n");
    }
    console_cmd_done();
}
