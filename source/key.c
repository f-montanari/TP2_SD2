/*==================[inclusions]=============================================*/
#include "key.h"

/*==================[macros and definitions]=================================*/

typedef enum{
    ESPERANDO_ACTIVACION = 0,
    ESPERANDO_DESACTIVACION,
}estPul_enum;

/*==================[internal data declaration]==============================*/

static estPul_enum estSW[BOARD_SW_ID_TOTAL];
static bool eventSW[BOARD_SW_ID_TOTAL];

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

void key_init(void){
    int32_t i;

    for (i = 0 ; i < BOARD_SW_ID_TOTAL ; i++){
        estSW[i] = ESPERANDO_ACTIVACION;
        eventSW[i] = 0;
    }
}

bool key_getPressEv(board_swId_enum id){
    bool ret = false;

    if (eventSW[id]){
        eventSW[id] = 0;
        ret = true;
    }

    return ret;
}

void key_periodicTask1ms(void){
    int32_t i;

    for (i = 0 ; i < BOARD_SW_ID_TOTAL ; i++){
        switch (estSW[i]){
            case ESPERANDO_ACTIVACION:
                if (board_getSw(i)){
                    eventSW[i] = 1;
                    estSW[i] = ESPERANDO_DESACTIVACION;
                }
                break;

            case ESPERANDO_DESACTIVACION:
                if (!board_getSw(i)){
                    estSW[i] = ESPERANDO_ACTIVACION;
                }
                break;

            default:
                estSW[i] = ESPERANDO_ACTIVACION;
                break;
        }
    }
}


/*==================[end of file]============================================*/
