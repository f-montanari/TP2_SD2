#ifndef SD2_BOARD_H_
#define SD2_BOARD_H_

/*==================[inclusions]=============================================*/
#include "MKL43Z4.h"
#include "stdbool.h"

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

typedef enum {
    BOARD_LED_ID_ROJO = 0,
    BOARD_LED_ID_VERDE,
    BOARD_LED_ID_TOTAL
}board_ledId_enum;

typedef enum {
    BOARD_SW_ID_1 = 0,
    BOARD_SW_ID_3,
    BOARD_SW_ID_TOTAL
}board_swId_enum;

typedef enum {
    BOARD_LED_MSG_OFF = 0,
    BOARD_LED_MSG_ON,
    BOARD_LED_MSG_TOGGLE
}board_ledMsg_enum;

typedef struct {
    PORT_Type *port;
    GPIO_Type *gpio;
    uint32_t pin;
}board_gpioInfo_type;

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions definition]==========================*/

/** \brief inicializaci�n del hardware
 **
 **/
void board_init(void);

/** \brief setea estado del led indicado
 **
 **/
void board_setLed(board_ledId_enum id, board_ledMsg_enum msg);

/** \brief Devuelve estado del pulsador indicado
 **
 ** \return true: si el pulsdor est� apretado
 **         false: si el pulsador no est� apretado
 **/
bool board_getSw(board_swId_enum id);

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* SD2_BOARD_H_ */
