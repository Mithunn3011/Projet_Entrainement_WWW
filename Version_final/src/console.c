#include "console.h"
#include <stdio.h>
#include <stdarg.h>
#include "stm32l4xx_hal.h"
#include "capteurs.h"

extern UART_HandleTypeDef huart2;   // console (USB ST-Link)

static char cmd[64];
static uint8_t cmd_len;
static volatile uint8_t cmd_ready;

/* ---------- Console (USART2, sans printf) ---------- */
/*
 * console : équivalent de printf qui envoie le texte formaté sur la console série (USART2).
 * Le message est limité à 128 caractères.
 */
void console(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
    if (n > 0) HAL_UART_Transmit(&huart2, (uint8_t *)buf, n, 100);
}

/*
 * console_rx_byte : reçoit un caractère tapé sur la console et assemble la ligne de commande.
 * Une fin de ligne (\r ou \n) valide la commande ; les caractères sont ignorés tant que la
 * commande précédente n'a pas été traitée.
 */
// Appelée à chaque octet reçu sur la console : assemble une ligne de commande
void console_rx_byte(char c)
{
    if (cmd_ready) return;
    if (c == '\r' || c == '\n') {
        if (cmd_len > 0) { cmd[cmd_len] = 0; cmd_ready = 1; }
    } else if (cmd_len < sizeof(cmd) - 1) {
        cmd[cmd_len++] = c;
    }
}

/*
 * console_cmd : renvoie la ligne de commande complète reçue, ou NULL si aucune n'est prête.
 */
char *console_cmd(void)                       // ligne complète reçue, ou NULL
{
    return cmd_ready ? cmd : NULL;
}

/*
 * console_cmd_done : indique que la commande a été traitée et remet le tampon à zéro
 * pour accepter la commande suivante.
 */
void console_cmd_done(void)
{
    cmd_len = 0;
    cmd_ready = 0;
}

extern UART_HandleTypeDef huart4;   // GPS SIM28 (port A1 : PA1 = RX)

static uint8_t rx_byte;             // GPS
static uint8_t rx2_byte;            // console

/*
 * UART4_IRQHandler : routine d'interruption de l'UART4 (GPS), qui délègue le traitement à la HAL.
 */
void UART4_IRQHandler(void)  { HAL_UART_IRQHandler(&huart4); }

/*
 * USART2_IRQHandler : routine d'interruption de l'USART2 (console), qui délègue le traitement à la HAL.
 */
void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart2); }

/*
 * serial_start : active les interruptions de l'UART4 (GPS) et de l'USART2 (console)
 * et lance la réception d'un octet à la fois en mode interruption.
 */
void serial_start(void)
{
    HAL_NVIC_SetPriority(UART4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
    HAL_UART_Receive_IT(&huart4, &rx_byte, 1);

    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_UART_Receive_IT(&huart2, &rx2_byte, 1);
}

/*
 * HAL_UART_RxCpltCallback : appelée par la HAL à chaque octet reçu.
 * Transmet l'octet à l'analyseur de la console (USART2) ou à celui du GPS (UART4),
 * puis relance la réception de l'octet suivant.
 */
// Appelée à chaque octet reçu
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h)
{
    if (h == &huart2) {
        console_rx_byte((char)rx2_byte);
        HAL_UART_Receive_IT(&huart2, &rx2_byte, 1);
    } else if (h == &huart4) {
        gps_rx_byte((char)rx_byte);
        HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
    }
}

/*
 * HAL_UART_ErrorCallback : appelée par la HAL en cas d'erreur de réception (overrun, etc.).
 * Relance la réception de l'UART concerné pour qu'elle ne reste pas bloquée.
 */
// Après une erreur (overrun...), la réception s'arrête : on la relance
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h)
{
    if (h == &huart4) HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
    if (h == &huart2) HAL_UART_Receive_IT(&huart2, &rx2_byte, 1);
}
