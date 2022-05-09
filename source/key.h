#ifndef KEY_H_
#define KEY_H_

/*==================[inclusions]=============================================*/
#include "SD2_board_KL43.h"

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions definition]==========================*/
void key_init(void);
bool key_getPressEv(board_swId_enum id);
void key_periodicTask1ms(void);

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* KEY_H_ */
