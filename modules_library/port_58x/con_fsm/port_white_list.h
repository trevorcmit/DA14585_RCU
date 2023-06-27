/*****************************************************************************************
 * \file port_white_list.h
 * \brief White List mgmt header file.
******************************************************************************************/
 
/****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup BONDING
 * \{
 * \addtogroup PORT_WHITELIST
 * \{
*****************************************************************************************/
#ifndef PORT_WHITE_LIST_H_
#define PORT_WHITE_LIST_H_

#include "rwip_config.h"
#include "gapm_task.h"


/*****************************************************************************************
 * \brief       Send a GAPM_WHITE_LIST_MGT_CMD to the stack.
 *
 * \details     Sends a GAPM_WHITE_LIST_MGT_CMD to the stack to add or remove a host
 *              from the White List.
 *
 * \param[in]   add              Controls whether it will be an add or remove operation; true for add, false for remove.
 * \param[in]   peer_addr_type   The BD_ADDR type of the host.
 * \param[in]   peer_addr        The BD_ADDR of the host.
 *
 * \return      void
 *
******************************************************************************************/
void port_white_list_send_mgt_cmd(bool add, uint8_t peer_addr_type, struct bd_addr const *peer_addr);
        
        
/*****************************************************************************************
 * \brief       
******************************************************************************************/        
void port_white_list_clear_list(void);


#endif // PORT_WHITE_LIST_H_


/******
 * \}
 * \}
 * \}
******/
