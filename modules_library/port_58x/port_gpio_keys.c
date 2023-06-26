/****************************************************************************************
 * \file port_gpio_keys.c
 * \brief GPIO keys module platform adaptation source file
*****************************************************************************************/

/****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup GPIO_KEYS
 * \{
 * \addtogroup PORT_GPIO_KEYS
 * \{
*****************************************************************************************/

#ifdef HAS_GPIO_KEYS

    #ifndef HAS_SYSTICK
        #error "GPIO Keys need SYSTICK. Symbol HAS_SYSTICK must be defined."
    #endif

    #include "port_wkup.h"
    #include "app_gpio_keys.h"
    #include "port_gpio_keys.h"
    #include "port_systick.h"
    #include "port_platform.h"
    #include "systick.h"

    void port_gpio_keys_systick_callback(void)
    {
        app_gpio_keys_debounce();
    }

    void port_gpio_keys_wakeup_handler(void)
    {
        app_gpio_keys_init(); 
        arch_ble_force_wakeup();

        if (GetBits32(&SysTick->CTRL, SysTick_CTRL_ENABLE_Msk) == 0)
        {
            port_systick_start(SYSTICK_GPIO_KEYS_CHANNEL, GPIO_DEBOUNCE_PERIOD_IN_US); 
        }
    }

    void port_gpio_keys_enable_irq(void)
    {
        uint32_t pol = 0;
        bool pressed;
        
        port_systick_stop(SYSTICK_GPIO_KEYS_CHANNEL);
        
        for(uint8_t i = 0; i < GPIO_NUM_OF_KEYS; i++)
        {
            pressed = app_gpio_keys_is_pressed(i);
            if( (app_gpio_keys_pins[i].high == 0) ? pressed : !pressed) {
                pol |= WKUPCT_PIN_SELECT(app_gpio_keys_pins[i].port, app_gpio_keys_pins[i].pin);
            }
        }    
    
        port_wkup_enable_irq(WKUP_GPIO_KEYS_CHANNEL, pol);
    }

    void port_gpio_keys_declare(void) 
    {
        for(int i = 0; i < GPIO_NUM_OF_KEYS; i++) {
            PORT_RESERVE_GPIO(app_gpio_keys_pins[i]);
        }
    }
        
    void port_gpio_keys_init(void)
    {
        for(int i = 0; i < GPIO_NUM_OF_KEYS; i++) {
            PORT_SET_PIN_FUNCTION_DEFAULT(app_gpio_keys_pins[i]);
        }
    }

#endif // HAS_GPIO_KEYS

/******
 * \}
 * \}
 * \}
*******/
