/* Copyright  DSI FCEIA UNR - Sistemas Digitales 2
 *    DSI: http://www.dsi.fceia.unr.edu.ar/
 *  Diego Alegrechi
 *  Gustavo Muro
 *
 *  Adaptación para detección de Caída Libre:
 *  Franco Montanari
 *  Dante Conti
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*==================[inclusions]=============================================*/
#include "mma8451.h"
#include "fsl_i2c.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "stdbool.h"
#include "fsl_debug_console.h"


/*==================[macros and definitions]=================================*/
#define MMA8451_I2C_ADDRESS     (0x1d)

#if defined CPU_MKL46Z256VLL4
#define INT1_PORT       PORTC
#define INT1_GPIO       GPIOC
#define INT1_PIN        5

#define INT2_PORT		PORTD
#define INT2_GPIO		GPIOD
#define INT2_PIN		1

#else
// TODO: Definir para otro KL43Z
#define INT1_PORT       PORTC
#define INT1_GPIO       GPIOC
#define INT1_PIN        -1

#define INT2_PORT		PORTD
#define INT2_GPIO		GPIOD
#define INT2_PIN		1

#endif

typedef union
{
    struct
    {
        unsigned SRC_DRDY:1;
        unsigned :1;
        unsigned SRC_FF_MT:1;
        unsigned SRC_PULSE:1;
        unsigned SRC_LNDPRT:1;
        unsigned SRC_TRANS:1;
        unsigned SRC_FIFO:1;
        unsigned SRC_ASLP:1;
    };
    uint8_t data;
}INT_SOURCE_t;
#define INT_SOURCE_ADDRESS   0X0C

typedef union
{
    struct
    {
        unsigned XDR:1;
        unsigned YDR:1;
        unsigned ZDR:1;
        unsigned ZYXDR:1;
        unsigned XOW:1;
        unsigned YOW:1;
        unsigned ZOW:1;
        unsigned ZYXOW:1;
    };
    uint8_t data;
}STATUS_t	;
#define STATUS_ADDRESS       0X00


typedef union
{
    struct
    {
        unsigned D:8;
    };
    uint8_t data;
}ASLP_COUNT_t;

#define ASLP_COUNT_ADDRESS   0X29


typedef union
{
    struct
    {
        unsigned ACTIVE:1;
        unsigned F_READ:1;
        unsigned LNOISE:1;
        unsigned DR:3;
        unsigned ASLP_RATE:2;
    };
    uint8_t data;
}CTRL_REG1_t;

#define CTRL_REG1_ADDRESS   0X2A

typedef union
{
    struct
    {
        unsigned MODS:2;
        unsigned SLPE:1;
        unsigned SMODS:2;
        unsigned :1;
        unsigned RST:1;
        unsigned ST:1;
    };
    uint8_t data;
}CTRL_REG2_t;

#define CTRL_REG2_ADDRESS   0X2B

typedef union
{
    struct
    {
        unsigned INT_EN_DRDY:1;
        unsigned :1;
        unsigned INT_EN_FF_MT:1;
        unsigned INT_EN_PULSE:1;
        unsigned INT_EN_LNDPRT:1;
        unsigned INT_EN_TRANS:1;
        unsigned INT_EN_FIFO:1;
        unsigned INT_EN_ASLP:1;
    };
    uint8_t data;
}CTRL_REG4_t;

#define CTRL_REG4_ADDRESS   0X2D

typedef union
{
    struct
    {
        unsigned INT_CFG_DRDY:1;
        unsigned :1;
        unsigned INT_CFG_FF_MT:1;
        unsigned INT_CFG_PULSE:1;
        unsigned INT_CFG_LNDPRT:1;
        unsigned INT_CFG_TRANS:1;
        unsigned INT_CFG_FIFO:1;
        unsigned INT_CFG_ASLP:1;
    };
    uint8_t data;
}CTRL_REG5_t;

#define CTRL_REG5_ADDRESS   0X2E

typedef union
{
    struct
    {
        unsigned :1;
        unsigned :1;
        unsigned :1;
        unsigned XEFE:1;
        unsigned YEFE:1;
        unsigned ZEFE:1;
        unsigned OAE:1;
        unsigned ELE:1;
    };
    uint8_t data;
}FF_MT_CFG;

#define FF_MT_CFG_ADDRESS 0x15

typedef union
{
    struct
    {
        unsigned XHP:1;
        unsigned XHE:1;
        unsigned YHP:1;
        unsigned YHE:1;
        unsigned ZHP:1;
        unsigned ZHE:1;
        unsigned :1;
        unsigned EA:1;
    };
    uint8_t data;
}FF_MT_SRC;

#define FF_MT_SRC_ADDRESS 0x16

typedef union
{
    struct
    {
        unsigned THS:7;
        unsigned DBCNTM:1;
    };
    uint8_t data;
}FF_MT_THS;

#define FF_MT_THS_ADDRESS 0x17

typedef union
{
    struct
    {
        unsigned D:8;
    };
    uint8_t data;
}FF_MT_COUNT;

#define FF_MT_COUNT_ADDRESS 0x18

/*==================[internal data declaration]==============================*/

static int16_t readX, readY, readZ;
bool freefall = false;

/*==================[internal functions declaration]=========================*/
static uint8_t mma8451_read_reg(uint8_t addr)
{
	i2c_master_transfer_t masterXfer;
    uint8_t ret;

	memset(&masterXfer, 0, sizeof(masterXfer));    // pone todo en cero (sizeof())
	masterXfer.slaveAddress = MMA8451_I2C_ADDRESS;
	masterXfer.direction = kI2C_Read;
	masterXfer.subaddress = addr;
	masterXfer.subaddressSize = 1;
	masterXfer.data = &ret;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferBlocking(I2C0, &masterXfer);

	return ret;
}

static void mma8451_write_reg(uint8_t addr, uint8_t data)
{
	i2c_master_transfer_t masterXfer;

    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = MMA8451_I2C_ADDRESS;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = addr;
	masterXfer.subaddressSize = 1;
	masterXfer.data = &data;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferBlocking(I2C0, &masterXfer);
}

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*
 * Configura las interrupciones para los correspondientes INT1 e INT2 del chip.
 */
static void config_port_interrupts(void)
{

	const port_pin_config_t port_int_config = {
			/* Internal pull-up/down resistor is disabled */
		.pullSelect = kPORT_PullDisable,
		/* Slow slew rate is configured */
		.slewRate = kPORT_SlowSlewRate,
		/* Passive filter is disabled */
		.passiveFilterEnable = kPORT_PassiveFilterDisable,
		/* Low drive strength is configured */
		.driveStrength = kPORT_LowDriveStrength,
		/* Pin is configured as PTC3 */
		.mux = kPORT_MuxAsGpio,
	};
	const gpio_pin_config_t gpio_int_config = {
		.pinDirection = kGPIO_DigitalInput,
		.outputLogic = 0U
	};

	PORT_SetPinConfig(INT1_PORT, INT1_PIN, &port_int_config);
	GPIO_PinInit(INT1_GPIO, INT1_PIN, &gpio_int_config);

	PORT_SetPinConfig(INT2_PORT, INT2_PIN, &port_int_config);
	GPIO_PinInit(INT2_GPIO, INT2_PIN, &gpio_int_config);
	
	/* Interrupt polarity active high, or active low. Default value: 0.
	   0: Active low; 1: Active high. VER REGISTRO CTRL_REG3 */
	PORT_SetPinInterruptConfig(INT1_PORT, INT1_PIN, kPORT_InterruptFallingEdge);
	PORT_SetPinInterruptConfig(INT2_PORT, INT2_PIN, kPORT_InterruptFallingEdge);

	NVIC_EnableIRQ(PORTC_PORTD_IRQn);
	NVIC_SetPriority(PORTC_PORTD_IRQn, 0);
}

/*==================[external functions definition]==========================*/
/*
 * Inicializa el sensor en modo de conversión continua, para leer los datos con
 * cada interrupcion "DRDY" (INT1).
 */
void mma8451_init_continuous(void)
{
    CTRL_REG1_t ctrl_reg1;
    CTRL_REG4_t ctrl_reg4;
    CTRL_REG5_t ctrl_reg5;

    // Poner el dispositivo en STANDBY, con un rate de 100Hz (experimental)
    // y modo Low Noise
	ctrl_reg1.ACTIVE = 0;
	ctrl_reg1.F_READ = 0;
	ctrl_reg1.LNOISE = 1;
	ctrl_reg1.ASLP_RATE = 0B00;
	ctrl_reg1.DR= DR_50hz;
	mma8451_write_reg(CTRL_REG1_ADDRESS, ctrl_reg1.data);

	// Habilitar y routear interrupción de Data Ready
	ctrl_reg4.INT_EN_DRDY = 1;
	ctrl_reg4.INT_EN_FF_MT = 0;
	ctrl_reg4.INT_EN_PULSE = 0;
	ctrl_reg4.INT_EN_LNDPRT = 0;
	ctrl_reg4.INT_EN_TRANS = 0;
	ctrl_reg4.INT_EN_FIFO = 0;
	ctrl_reg4.INT_EN_ASLP = 0;

	mma8451_write_reg(CTRL_REG4_ADDRESS, ctrl_reg4.data);

	ctrl_reg5.INT_CFG_DRDY = 1;
	ctrl_reg5.INT_CFG_FF_MT = 0;
	ctrl_reg5.INT_CFG_PULSE = 0;
	ctrl_reg5.INT_CFG_LNDPRT = 0;
	ctrl_reg5.INT_CFG_TRANS = 0;
	ctrl_reg5.INT_CFG_FIFO = 0;
	ctrl_reg5.INT_CFG_ASLP = 0;

	mma8451_write_reg(CTRL_REG5_ADDRESS, ctrl_reg5.data);



	// Poner el dispositivo en modo activo.
	ctrl_reg1.ACTIVE = 1;
    mma8451_write_reg(CTRL_REG1_ADDRESS, ctrl_reg1.data);

    config_port_interrupts();
}

/*
 * Modifica la frecuencia en la que se realizan las mediciones.
 */
void mma8451_setDataRate(DR_enum rate)
{
    CTRL_REG1_t ctr_reg1;
    bool estAct;

    /* antes de modificar data rate es necesario poner ACTIVE = 0 */
    ctr_reg1.data = mma8451_read_reg(CTRL_REG1_ADDRESS);

    /* guarda valor que tiene ACTIVE y luego pone a cero */
    estAct = ctr_reg1.ACTIVE;
    ctr_reg1.ACTIVE = 0;

	mma8451_write_reg(CTRL_REG1_ADDRESS, ctr_reg1.data);

	/* actualiza DR y en la misma escritura va a restaurar ACTIVE */
	ctr_reg1.DR = rate;
	ctr_reg1.ACTIVE = estAct;

	mma8451_write_reg(CTRL_REG1_ADDRESS, ctr_reg1.data);
}

/*
 * Inicializa el sensor en modo "freefall", para detectar caída libre
 * y sacar al micro del modo bajo consumo. (INT2)
 *
 * También modifica el DR para que esté trabajando a una frecuencia
 * mucho menor que la de lectura continua (es decir, trabajando a
 * modo bajo consumo).
 *
 * ODR = 12.5Hz
 *
 * Cabe aclarar que también se buscó la funcionalidad Auto-Wake/Sleep
 * en las notas de aplicación AN4074, pero eso es más útil para detección
 * de movimiento en períodos largos de tiempo (nosotros estaríamos implementando
 * esta funcionalidad directamente desde el micro).
 */
void mma8451_init_freefall(void){

	// Habiendo creado las estructuras faltantes para los registros que
	// pide modificar el manual de configuracion AN4070, procedemos a
	// seguir los pasos.

	// 1) Poner el dispositivo en STANDBY, con un rate de 12.5Hz

	CTRL_REG1_t ctr_reg1;
	ctr_reg1.ACTIVE = 0;
	ctr_reg1.DR = DR_12p5hz;
	mma8451_write_reg(CTRL_REG1_ADDRESS, ctr_reg1.data); // 0x20

	// 2) Configurar para FreeFall usando ELE=1, OAE = 0
	// En este modo, el bit EA es seteado después del "debounce counter"
	//
	// Se limpian EA y el debounce counter cuando FF_MT_SRC es leído.

	FF_MT_CFG cfg;
	cfg.ELE = 1;
	cfg.OAE = 0;
	cfg.XEFE = 1;
	cfg.YEFE = 1;
	cfg.ZEFE = 1;
	mma8451_write_reg(FF_MT_CFG_ADDRESS, cfg.data);

	// 3) Setear el threshold (para qué valor se considera caída libre?)
	// En este caso, tomaremos un valor de 0.2g (experimental)
	// Teniendo 0.2g/0.063 g/bit = 3
	// El bit DBCNTM hace resetear el contador del threshold cuando detecta
	// que no hay más "low-g event", lo cual no queremos para evitar posibles
	// ruidos.

	FF_MT_THS ths;
	ths.THS = 5;
	ths.DBCNTM = 0;
	mma8451_write_reg(FF_MT_THS_ADDRESS, ths.data);

	// 4) Setear el contador de "debounce"
	// Este es un registro que setea la constante de tiempo para un filtro pasa
	// bajos. Cabe aclarar que estos dependen del modo en el que está el chip,
	// (Ver tabla 7 ref AN4070)
	// Usando los datos de ejemplo (posiblemente haya que cambiar configuración
	// para que detecte en modo Low Power) donde usamos un rate de 50 Hz y usamos
	// un debounce de 120 ms
	// En este caso, 120 ms/20 ms = 6 = 110
	FF_MT_COUNT count;
	count.D = 6;
	mma8451_write_reg(FF_MT_COUNT_ADDRESS, count.data);

	// 5) Habilitar funcionalidad de interrupt
	CTRL_REG4_t ctr_reg4;
	ctr_reg4.INT_EN_FF_MT = 1;
	ctr_reg4.INT_EN_DRDY = 0;
	mma8451_write_reg(CTRL_REG4_ADDRESS, ctr_reg4.data);

	// 6) Routear interrupcion INT2 (FreeFall)
	CTRL_REG5_t ctr_reg5;
	ctr_reg5.INT_CFG_DRDY = 1;
	ctr_reg5.INT_CFG_FF_MT = 0;
	mma8451_write_reg(CTRL_REG5_ADDRESS, ctr_reg5.data);

	// 7) Poner dispositivo en modo Activo 50 Hz
	ctr_reg1.data = mma8451_read_reg(CTRL_REG1_ADDRESS);
	ctr_reg1.ACTIVE = 1;
	//ctr_reg1.DR = DR_50hz;
	mma8451_write_reg(CTRL_REG1_ADDRESS, ctr_reg1.data);

	// 8) Habilitar interrupción en la NVIC
	config_port_interrupts();
}

void mma8451_enableDataInterrupt(){
	CTRL_REG1_t ctr_reg1;
	ctr_reg1.ACTIVE = 0;
	ctr_reg1.DR = DR_50hz;
	mma8451_write_reg(CTRL_REG1_ADDRESS, ctr_reg1.data);

	// 5) Habilitar funcionalidad de interrupt
	CTRL_REG4_t ctr_reg4;
	ctr_reg4.INT_EN_FF_MT = 0;
	ctr_reg4.INT_EN_DRDY = 1;
	mma8451_write_reg(CTRL_REG4_ADDRESS, ctr_reg4.data);

	// 7) Poner dispositivo en modo Activo 50 Hz
	ctr_reg1.ACTIVE = 1;
	mma8451_write_reg(CTRL_REG1_ADDRESS, ctr_reg1.data);
}

void mma8451_disableDataInterrupt(){
	CTRL_REG1_t ctr_reg1;
	ctr_reg1.ACTIVE = 0;
	ctr_reg1.DR = DR_12p5hz;
	mma8451_write_reg(CTRL_REG1_ADDRESS, ctr_reg1.data);

	// 5) Habilitar funcionalidad de interrupt
	CTRL_REG4_t ctr_reg4;
	ctr_reg4.INT_EN_FF_MT = 1;
	ctr_reg4.INT_EN_DRDY = 0;
	mma8451_write_reg(CTRL_REG4_ADDRESS, ctr_reg4.data);

	// 6) Routear interrupcion INT2 (FreeFall)
//	CTRL_REG5_t ctr_reg5;
//	ctr_reg5.INT_CFG_DRDY = 0;
//	ctr_reg5.INT_CFG_FF_MT = 0;
//	mma8451_write_reg(CTRL_REG5_ADDRESS, ctr_reg5.data);

	// 7) Poner dispositivo en modo Activo 50 Hz
	ctr_reg1.ACTIVE = 1;
	mma8451_write_reg(CTRL_REG1_ADDRESS, ctr_reg1.data);

}

bool mma8451_getFreefall(){
	return freefall;
}

void mma8451_clearFreefall()
{
	freefall = false;
}

/*
 * Devuelve la aceleración en X como porcentaje de g's
 * Es decir, si acc = 100 -> la placa soporta 1g.
 */
int16_t mma8451_getAcX(void)
{
	return (int16_t)(((int32_t)readX * 100) / (int32_t)4096);
}

int16_t mma8451_getAcY(void)
{
	return (int16_t)(((int32_t)readY * 100) / (int32_t)4096);
}

int16_t mma8451_getAcZ(void)
{
	return (int16_t)(((int32_t)readZ * 100) / (int32_t)4096);
}


void PORTC_PORTD_IRQHandler(void){
	// INT1
#if defined CPU_MKL46Z256VLL4
	if(GPIO_GetPinsInterruptFlags(INT1_GPIO)){
#else
	if(GPIO_PortGetInterruptFlags(INT1_GPIO)){
#endif
    	int16_t readG;
		INT_SOURCE_t intSource;
		STATUS_t status;
    	intSource.data = mma8451_read_reg(INT_SOURCE_ADDRESS);

		if (intSource.SRC_DRDY){
			// clear ISR flag
			mma8451_read_reg(FF_MT_SRC_ADDRESS);

			status.data = mma8451_read_reg(STATUS_ADDRESS);

			if (status.XDR){
				readG   = (int16_t)mma8451_read_reg(0x01)<<8;
				readG  |= mma8451_read_reg(0x02);
				readX = readG >> 2;
			}

			if (status.YDR){
				readG   = (int16_t)mma8451_read_reg(0x03)<<8;
				readG  |= mma8451_read_reg(0x04);
				readY = readG >> 2;
			}

			if (status.ZDR){
				readG   = (int16_t)mma8451_read_reg(0x05)<<8;
				readG  |= mma8451_read_reg(0x06);
				readZ = readG >> 2;
			}
		}
    	PORT_ClearPinsInterruptFlags(INT1_PORT, 1<<INT1_PIN);
    }

    // INT 2
#if defined CPU_MKL46Z256VLL4
	if(GPIO_GetPinsInterruptFlags(INT2_GPIO)){
#else
	if(GPIO_PortGetInterruptFlags(INT2_GPIO)){
#endif

    	// Detectamos la caída, ahora hay que limpiar el registro para que
    	// podamos detectar una nueva si llegamos a necesitarlo.
    	INT_SOURCE_t intFreefallSource; // registro 0X0C

    	intFreefallSource.data = mma8451_read_reg(INT_SOURCE_ADDRESS); // chequeo fuente de IRQ

    	// La bandera EA avisa si se detectó el evento.
    	if(intFreefallSource.SRC_FF_MT){
        	// leer registro para limpiar ISR flag
        	mma8451_read_reg(FF_MT_SRC_ADDRESS); // registro 0x16
    		// Tal vez en vez de poner la bandera en true, deberíamos cambiar acá
			// el modo del procesador en RUN.
			freefall = true;
    	}
    	PORT_ClearPinsInterruptFlags(INT2_PORT, 1<<INT2_PIN);
    }
}

/*==================[end of file]============================================*/

