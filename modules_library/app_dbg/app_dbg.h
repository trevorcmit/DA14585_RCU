/*****************************************************************************************
 *
 * \file app_dbg.h
 *
 * \brief The app_dbg provides simple tools for assisting debugging. 
 * 
******************************************************************************************/
 
/*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup DEBUG
 * \{
 * \addtogroup APP_DBG
 *
 * \brief Application debug functions
 * \{
 ****************************************************************************************	 	 
 */
 
#ifndef _APP_DBG_
#define _APP_DBG_

#include "datasheet.h"
#include "core_cm0.h"

typedef enum
{
    APP_DBG_FAULT_NMI   = 0,
    APP_DBG_FAULT_HARD,
    APP_DBG_FAULT_FLASH,
    APP_DBG_FAULT_NONE  = 0xFF
} app_dbg_fault_t;

#define APP_DBG_HARD_ARGS   8
#define APP_DBG_SCB_REGS    6
#define APP_DBG_TOTAL_REGS  (APP_DBG_HARD_ARGS + 1 + APP_DBG_SCB_REGS)
#define APP_DBG_REGS_SIZE   sizeof(app_dbg_fault_t) + APP_DBG_TOTAL_REGS*sizeof(uint32_t)

extern uint32_t app_dbg_registers[];
    
/*****************************************************************************************
 * \brief   
 *
 * \param[in]  fault
******************************************************************************************/  
void app_dbg_write_fault(app_dbg_fault_t fault);

/*****************************************************************************************
 * \brief   
 *
 * \param[in]  msg
******************************************************************************************/  
void app_dbg_write_msg(char *msg);

/*****************************************************************************************
 * \brief   
 *
 * \param[in]  hardfault_args
******************************************************************************************/  
void app_dbg_pack_regs(uint32_t *hardfault_args);
    
#endif //_APP_DBG_

/**
 * \}
 * \}
 * \}
 */
