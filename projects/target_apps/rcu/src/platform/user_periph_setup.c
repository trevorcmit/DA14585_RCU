/****************************************************************************************
* \file user_periph_setup.c
* \brief Peripherals setup and initialization. 
*****************************************************************************************/

/****************************************************************************************
 * INCLUDE FILES
*****************************************************************************************/
#include "rwip_config.h"             // SW configuration
#include <user_periph_setup.h>       // peripheral configuration
#include "gpio.h"
#include "uart.h"                    // UART initialization
#include "user_modules.h"


#if DEVELOPMENT_DEBUG
    /****************************************************************************************
     * \brief Each application reserves its own GPIOs here.
    *****************************************************************************************/
    void GPIO_reservations(void)
    {
        /** Globally reserved GPIOs reservation */
        /*
        * Application specific GPIOs reservation. Used only in Development mode (#if DEVELOPMENT_DEBUG)     
        i.e.  
            RESERVE_GPIO(DESCRIPTIVE_NAME, GPIO_PORT_0, GPIO_PIN_1, PID_GPIO);    //Reserve P_01 as Generic Purpose I/O
        */
        #ifdef CFG_PRINTF_UART2
            RESERVE_GPIO( UART2_TX, UART_TX_PORT,  UART_TX_PIN, PID_UART2_TX);
            #ifndef DISABLE_UART_RX    
            RESERVE_GPIO( UART2_RX, UART_RX_PORT,  UART_RX_PIN, PID_UART2_RX);
            #endif
        #endif

        /******************************************
         * Application specific GPIOs reservation
        *******************************************/  
        user_modules_reserve_gpios();
    
        #ifdef HAS_SPI
            RESERVE_GPIO( SPI_CLK,  SPI_CLK_PORT, SPI_CLK_PIN, PID_SPI_CLK); 
            RESERVE_GPIO( SPI_DI,   SPI_DI_PORT,  SPI_DI_PIN,  PID_SPI_DI ); 
            RESERVE_GPIO( SPI_DO,   SPI_DO_PORT,  SPI_DO_PIN,  PID_SPI_DO );
        #endif

        #ifdef HAS_I2C
            RESERVE_GPIO( I2C_BUS_SDA, I2C_SDA_PORT, I2C_SDA_PIN, PID_I2C_SDA );
            RESERVE_GPIO( I2C_BUS_SCL, I2C_SCL_PORT, I2C_SCL_PIN, PID_I2C_SCL );
        #endif    

        #if defined(INDICATE_IDLE_STATE) && DEVELOPMENT_DEBUG
            RESERVE_GPIO(IDLE_INDICATION, IDLE_INDICATION_PORT, IDLE_INDICATION_PIN, PID_GPIO);
        #endif    

        #if defined(USE_AUDIO_MARK) && DEVELOPMENT_DEBUG
            RESERVE_GPIO(AUDIO_MARK, AUDIO_MARK_PORT, AUDIO_MARK_PIN, PID_GPIO);
        #endif
        #ifdef STREAM_DEBUG
            RESERVE_GPIO(DEBUG, GPIO_PORT_1, GPIO_PIN_3, PID_GPIO);   
        #endif
    }
#endif // DEVELOPMENT_DEBUG


/****************************************************************************************
 * \brief Map port pins
 * \details The Uart and SPI port pins and GPIO ports are mapped
*****************************************************************************************/
void set_pad_functions(void)        // set gpio port function mode
{
    #ifdef HAS_SPI
        GPIO_SetPinFunction( SPI_CLK_PORT, SPI_CLK_PIN, INPUT_PULLUP, PID_GPIO);
        GPIO_SetPinFunction( SPI_DO_PORT,  SPI_DO_PIN,  INPUT_PULLUP, PID_GPIO);
        GPIO_SetPinFunction( SPI_DI_PORT,  SPI_DI_PIN,  INPUT_PULLUP, PID_GPIO);
    #endif

    #ifdef HAS_I2C
        GPIO_SetPinFunction( I2C_SDA_PORT, I2C_SDA_PIN, INPUT, PID_I2C_SDA );
        GPIO_SetPinFunction( I2C_SCL_PORT, I2C_SCL_PIN, INPUT, PID_I2C_SCL );
    #endif
        
    #ifdef CFG_PRINTF_UART2
        GPIO_ConfigurePin(UART_TX_PORT, UART_TX_PIN, OUTPUT, PID_UART2_TX, false );
        #ifndef DISABLE_UART_RX    
            GPIO_ConfigurePin(UART_RX_PORT, UART_RX_PIN, INPUT,  PID_UART2_RX, false );
        #endif
    #endif

    user_modules_init_gpios();
}

#ifdef CFG_PRINTF_UART2
    #ifndef UART2_BAUDRATE
        #define UART2_BAUDRATE 115K2
    #endif
    #define _DEBUG_UART_BAUDRATE(x) UART_BAUDRATE_##x
    #define DEBUG_UART_BAUDRATE(x) _DEBUG_UART_BAUDRATE(x)
    #define _DEBUG_UART_FRAC_BAUDRATE(x) UART_FRAC_BAUDRATE_##x
    #define DEBUG_UART_FRAC_BAUDRATE(x) _DEBUG_UART_FRAC_BAUDRATE(x)
#endif


/****************************************************************************************
 * \brief Enable pad's and peripheral clocks assuming that peripherals' power domain is 
 * down. The Uart and SPi clocks are set.
*****************************************************************************************/
void periph_init(void) 
{
	// Power up peripherals' power domain
    SetBits16(PMU_CTRL_REG, PERIPH_SLEEP, 0);
    while (!(GetWord16(SYS_STAT_REG) & PER_IS_UP)) ; 
    
    SetBits16(CLK_16M_REG, XTAL16_BIAS_SH_ENABLE, 1);
	
	// rom patch
	patch_func();
	
	// Init pads
	set_pad_functions();

    // (Re)Initialize peripherals

    #ifdef CFG_PRINTF_UART2
        // SetBits16(CLK_PER_REG, UART2_ENABLE, 1);
        #if (RWBLE_SW_VERSION_MAJOR >= 8)
            uart2_init(DEBUG_UART_BAUDRATE(UART2_BAUDRATE), DEBUG_UART_FRAC_BAUDRATE(UART2_BAUDRATE), 3);
        #else
            SetBits16(CLK_PER_REG, UART2_ENABLE, 1);
            uart2_init(DEBUG_UART_BAUDRATE(UART2_BAUDRATE), 3);
        #endif    
    #endif

	SetBits16(SYS_CTRL_REG, PAD_LATCH_EN, 1);
}
