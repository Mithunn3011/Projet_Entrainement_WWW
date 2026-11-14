#include "modes.h"
#include "settings.h"
#include "stm32l4xx_hal.h"
#include "console.h"
#include "i2c.h"
#include "mesures.h"

#define NUM_LEDS 1

#define CLK_PORT GPIOB
#define CLK_PIN  GPIO_PIN_10   // D6 : horloge (CI)
#define DAT_PORT GPIOA
#define DAT_PIN  GPIO_PIN_8    // D7 : données (DI)

/*
 * led_init : configure les broches d'horloge et de données de la LED RGB (protocole série type Grove Chainable LED)
 * en sorties push-pull.
 */
void led_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pin = CLK_PIN; HAL_GPIO_Init(CLK_PORT, &gpio);
    gpio.Pin = DAT_PIN; HAL_GPIO_Init(DAT_PORT, &gpio);
}

/* ---------- LED ---------- */
/*
 * send_byte : envoie un octet à la LED bit par bit (bit de poids fort en premier)
 * en faisant battre l'horloge et en positionnant la ligne de données.
 */
static void send_byte(uint8_t b)
{
    for (int i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DAT_PORT, DAT_PIN, (b & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_SET);
        b <<= 1;
    }
}

/*
 * send_frame : envoie la trame de 4 octets (préfixe + B + G + R) qui définit la couleur d'une LED.
 * Le préfixe contient les 2 bits de poids fort inversés de chaque composante (exigence du protocole).
 */
static void send_frame(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t prefix = 0xC0 | ((~b & 0xC0) >> 2) | ((~g & 0xC0) >> 4) | ((~r & 0xC0) >> 6);
    send_byte(prefix); send_byte(b); send_byte(g); send_byte(r);
}

/*
 * leds_set_all : règle la couleur (R, G, B) de toutes les LED de la chaîne.
 * Envoie une trame de début (4 octets nuls), la couleur de chaque LED, puis une trame de fin.
 */
void leds_set_all(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < 4; i++) send_byte(0);
    for (int i = 0; i < NUM_LEDS; i++) send_frame(r, g, b);
    for (int i = 0; i < 4; i++) send_byte(0);
}

#define GREEN_PORT GPIOA
#define GREEN_PIN  GPIO_PIN_10 // D2 : bouton vert
#define RED_PORT   GPIOB
#define RED_PIN    GPIO_PIN_3  // D3 : bouton rouge
#define PRESSED    GPIO_PIN_RESET

Button button_green = { GREEN_PORT, GREEN_PIN, 0, 0, 0 };
Button button_red   = { RED_PORT,   RED_PIN,   0, 0, 0 };

/*
 * button_pressed : retourne 1 si le bouton est actuellement enfoncé (niveau bas), 0 sinon.
 */
int button_pressed(const Button *b)
{
    return HAL_GPIO_ReadPin(b->port, b->pin) == PRESSED;
}

/*
 * button_held : détecte un appui long. Retourne 1 une seule fois quand le bouton a été maintenu
 * pendant HOLD_MS ; il faut le relâcher pour que la détection soit à nouveau possible.
 */
// Vrai une seule fois quand le bouton est maintenu HOLD_MS (re-armé au relâchement)
int button_held(Button *b)
{
    uint32_t now = HAL_GetTick();
    if (!button_pressed(b)) { b->active = 0; return 0; }
    if (!b->active) { b->active = 1; b->since = now; b->fired = 0; }
    if (!b->fired && now - b->since >= HOLD_MS) { b->fired = 1; return 1; }
    return 0;
}

/*
 * buttons_init : configure les boutons vert et rouge en entrées avec résistance de pull-up
 * (ils sont actifs à l'état bas).
 */
void buttons_init(void)                       // actifs à l'état bas
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Pin = GREEN_PIN; HAL_GPIO_Init(GREEN_PORT, &gpio);
    gpio.Pin = RED_PIN;   HAL_GPIO_Init(RED_PORT, &gpio);
}

#define CONFIG_TIMEOUT  (CONFIG_TIMEOUT_MIN * 60UL * 1000UL)


static const uint8_t mode_color[][3] = {   // R, G, B
    [MODE_CONFIG]   = {255, 255, 0},       // jaune
    [MODE_STANDARD] = {0, 255, 0},         // vert
    [MODE_ECO]      = {0, 0, 255},         // bleu
    [MODE_MAINT]    = {255, 80, 0},        // orange
};

static const char *mode_name[] = {
    [MODE_CONFIG]   = "CONFIGURATION",
    [MODE_STANDARD] = "STANDARD",
    [MODE_ECO]      = "ECONOMIQUE",
    [MODE_MAINT]    = "MAINTENANCE",
};

Mode mode;
static Mode prev_mode;
uint32_t mode_since;


/*
 * set_mode : change le mode de fonctionnement. Mémorise l'instant du changement, ferme le fichier SD
 * si on passe en maintenance, allume la LED à la couleur du mode et affiche le nouveau mode sur la console.
 */
void set_mode(Mode m)
{
    mode = m;
    mode_since = HAL_GetTick();
    if (m == MODE_MAINT) log_stop();          // ferme le fichier : la carte peut être retirée sans risque
    leds_set_all(mode_color[m][0], mode_color[m][1], mode_color[m][2]);
    console("Mode : %s\r\n", mode_name[m]);
}

/*
 * update_mode : gère les transitions entre modes selon les appuis longs sur les boutons :
 * standard <-> économique (bouton vert), standard/économique -> maintenance et retour (bouton rouge),
 * et retour automatique de configuration vers standard après CONFIG_TIMEOUT d'inactivité.
 */
void update_mode(void)
{
    switch (mode) {
    case MODE_STANDARD:
        if (button_held(&button_green))     set_mode(MODE_ECO);
        else if (button_held(&button_red))  { prev_mode = MODE_STANDARD; set_mode(MODE_MAINT); }
        break;
    case MODE_ECO:
        if (button_held(&button_green))     set_mode(MODE_STANDARD);
        else if (button_held(&button_red))  { prev_mode = MODE_ECO; set_mode(MODE_MAINT); }
        break;
    case MODE_MAINT:
        if (button_held(&button_red))       set_mode(prev_mode);
        break;
    case MODE_CONFIG:
        if (HAL_GetTick() - mode_since >= CONFIG_TIMEOUT) set_mode(MODE_STANDARD);
        break;
    }
}

/* ---------- Erreurs ---------- */
typedef struct {
    const char *text;
    uint8_t a[3], b[3];     // couleurs R, G, B (a en premier, b ensuite)
    uint16_t a_ms;          // durée de a dans la période ; b dure le reste
} ErrInfo;

static const ErrInfo err_info[ERR_COUNT] = {
    [ERR_RTC]        = { "acces a l'horloge RTC",              {255, 0, 0}, {0, 0, 255},     ERR_PERIOD_MS / 2 },
    [ERR_SD_IO]      = { "acces ou ecriture sur la carte SD",  {255, 0, 0}, {255, 255, 255}, ERR_PERIOD_MS / 3 },
    [ERR_SD_FULL]    = { "carte SD pleine",                    {255, 0, 0}, {255, 255, 255}, ERR_PERIOD_MS / 2 },
    [ERR_GPS]        = { "acces aux donnees du GPS",           {255, 0, 0}, {255, 255, 0},   ERR_PERIOD_MS / 2 },
    [ERR_SENSOR]     = { "acces aux donnees d'un capteur",     {255, 0, 0}, {0, 255, 0},     ERR_PERIOD_MS / 2 },
    [ERR_INCOHERENT] = { "donnees incoherentes, verifier le materiel", {255, 0, 0}, {0, 255, 0}, ERR_PERIOD_MS / 3 },
};

static uint8_t err_mask;                      // bit e = erreur e présente

/*
 * error_set : active ou désactive une erreur dans le masque d'erreurs.
 * Affiche un message sur la console uniquement quand l'erreur apparaît (front montant).
 */
void error_set(ErrCode e, int on)
{
    uint8_t bit = 1u << e;
    if (on && !(err_mask & bit)) console("ERREUR E%02d : %s\r\n", e, err_info[e].text);
    if (on) err_mask |= bit;
    else    err_mask &= ~bit;
}

/*
 * error_led : met à jour la LED pour signaler l'erreur la plus prioritaire en la faisant clignoter
 * avec ses deux couleurs ; sans erreur, la LED reprend la couleur du mode courant.
 * Met aussi à jour l'erreur RTC à partir de l'état de l'horloge.
 */
// Affiche l'erreur la plus prioritaire ; sans erreur, la LED reprend la couleur du mode
void error_led(void)
{
    error_set(ERR_RTC, rtc_fail);
    static uint8_t shown, phase;              // erreur affichée (0 = couleur du mode) et phase du clignotement
    uint8_t want = 0;
    for (uint8_t e = 1; e < ERR_COUNT && !want; e++)
        if (err_mask & (1u << e)) want = e;

    if (!want) {
        if (shown) leds_set_all(mode_color[mode][0], mode_color[mode][1], mode_color[mode][2]);
        shown = 0;
        return;
    }

    const ErrInfo *i = &err_info[want];
    uint8_t ph = (HAL_GetTick() % ERR_PERIOD_MS) >= i->a_ms;
    if (want != shown || ph != phase) {
        const uint8_t *c = ph ? i->b : i->a;
        leds_set_all(c[0], c[1], c[2]);
    }
    shown = want;
    phase = ph;
}
