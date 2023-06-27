/*****************************************************************************************
 *
 * \file app_hid_report.c
 *
 * \brief HID FIFO module source file
 * 
******************************************************************************************/

/*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup HID
 * \{
 * \addtogroup APP_HID
 * \{
******************************************************************************************/

#include "port_platform.h"
#include "app_hid_report.h"
#include "app_hogpd.h"

#ifdef HAS_HID_REPORT

#define APP_HID_ERR_ERROR  (0)
#define APP_HID_ERR_OK     (1)

#define RLOVR_INVALID_INTERSECTION  (0xFFFF)
#define RLOVR_INDICATION_CODE       (0xFEFE)

#if (HID_REPORT_ROLL_OVER_BUF_SIZE < 7)
    #error "Too small Roll-Over buffer!"
#endif

struct roll_over_tag {
        // bits   [7:0] : normal key code
        uint16_t intersections[HID_REPORT_ROLL_OVER_BUF_SIZE];
        int cnt;
};

typedef struct hid_rep_data_s {
        uint16_t length;
        uint8_t *data;
} hid_rep_data_t;

typedef struct hid_rep_node {
        enum KEY_BUFF_TYPE type;
        uint8_t char_id;
        hid_rep_data_t data;
        struct hid_rep_node *pNext;
} hid_rep_node_t;

typedef void (*app_hid_report_send_report_cb_t)(uint8_t char_id, uint8_t *data, uint16_t length); 

#ifdef APP_HID_REPORT_SEND_CB
    static const app_hid_report_send_report_cb_t app_hid_report_send_report_cb = APP_HID_REPORT_SEND_CB;
#else
    static const app_hid_report_send_report_cb_t app_hid_report_send_report_cb = NULL;
#endif

hid_rep_node_t *report_trm_list                                      __PORT_RETAINED;   // Linked list of the pending Reports
hid_rep_node_t *report_free_list                                     __PORT_RETAINED;   // Linked list of the free Reports
hid_rep_node_t report_list[HID_REPORT_FIFO_SIZE]                     __PORT_RETAINED;   // The list of the reports instances (free or used)
uint8_t hid_report[HID_REPORT_FIFO_SIZE][HID_REPORT_MAX_REPORT_SIZE] __PORT_RETAINED;   // Report buffers

uint8_t normal_key_report_st[HID_REPORT_NORMAL_REPORT_SIZE]          __PORT_RETAINED;   // Holds the contents of the last Key Report for normal keys sent to the Host
uint8_t extended_key_report_st[HID_REPORT_EXTENDED_REPORT_SIZE]      __PORT_RETAINED;   // Holds the contents of the last Key Report for special functions sent to the Host
struct roll_over_tag roll_over_info                                  __PORT_RETAINED;   // holds all the keys being pressed during a RollOver (Phantom state) state
const uint8_t empty_report[HID_REPORT_NORMAL_REPORT_SIZE] = {0,0,0,0,0,0,0,0};

/*****************************************************************************************
 * \brief Pulls one member from the beginning of the list
 *
 * \param[in] list The list to use
 *
 * \return the member at the beginning of the list, if found, else NULL
******************************************************************************************/
static hid_rep_node_t *app_hid_report_pull_from_list(hid_rep_node_t **list)
{
        hid_rep_node_t *node;

        if (!(*list)) {
                return NULL;
        }

        node = *list;
        *list = node->pNext;
        node->pNext = NULL;

        return node;
}

/*****************************************************************************************
 * \brief Push one member to the beginning of the list
 *
 * \param[in] list The list to use
 * \param[in] node The node to put in the list
 *
 * \return   APP_HID_ERR_OK if the member is successfully pushed or 
 *           else APP_HID_ERR_ERROR
******************************************************************************************/
static int app_hid_report_return_to_list(hid_rep_node_t **list, hid_rep_node_t *node)
{
        if (!node) {
                return APP_HID_ERR_ERROR;
        }

        node->pNext = *list;
        *list = node;

        return APP_HID_ERR_OK;
}

/*****************************************************************************************
 * \brief Pushes one member at the end of the list
 *
 * \param[in]   list    The list to use
 * \param[in]   node    Member to add to the list
 *
 * \return   APP_HID_ERR_OK if the member is successfully pushed or else APP_HID_ERR_ERROR
******************************************************************************************/
static int app_hid_report_push_to_list(hid_rep_node_t **list, hid_rep_node_t *node)
{
        hid_rep_node_t *p;

        if (!node) {
                return APP_HID_ERR_ERROR;
        }

        if (!(*list)) {
                *list = node;
        }
        else {
                for (p = *list; p->pNext != NULL; p = p->pNext)
                        ;

                p->pNext = node;
                node->pNext = NULL;
        }

        return APP_HID_ERR_OK;
}

/*****************************************************************************************
 * \brief Finds the last instance of a specific type in a list
 *
 * \param[in]   list    The list to use
 * \param[in]   type    The type of the instance to search for (FREE, PRESS, RELEASE, EXTENDED, CUSTOM)
 *
 * \return  the last instance of a specific type in a list, if found, else NULL
******************************************************************************************/
hid_rep_node_t *app_hid_report_search_in_list(hid_rep_node_t *list, enum KEY_BUFF_TYPE type)
{
        hid_rep_node_t *node;
        hid_rep_node_t *last_found = NULL;

        for (node = list; node != NULL; node = node->pNext) {
                if (node->type == type) {
                        last_found = node;
                }
        }

        return last_found;
}

/*****************************************************************************************
 * \brief Sends a boot report
 *
 * \return  false, if sending the HID boot report failed
 *          true, if the HID boot report was successfully sent
******************************************************************************************/
static bool app_hid_report_send_boot_report(void)
{
        hid_rep_node_t *p;
#if (BLE_HID_DEVICE)
        if (hogpd_params.boot_protocol_mode) 
#endif               
        {
                p = app_hid_report_pull_from_list(&report_trm_list);

                ASSERT_WARNING(p);

                if (p->char_id == HID_REPORT_NORMAL_REPORT_IDX) {
                        uint8_t *data = p->data.data;
                        // Fill in the parameter structure
#if (BLE_HID_DEVICE)                    
                        if (app_hogpd_send_report(HID_REPORT_NORMAL_REPORT_IDX, p->data.data, p->data.length, HOGPD_BOOT_KEYBOARD_INPUT_REPORT)) {
#else
    // TODO: Implement alternative ways to send HID reports
    #warning "HOGPD profile is not enabled. HID boot reports cannot be sent"
                        if(false) {
#endif
                               dbg_printf(DBG_HID_LVL, "Sending HOGPD_REPORT_UPD_REQ %02x:[%02x:%02x:%02x:%02x:%02x:%02x]\r\n",
                                        (int )data[0], (int )data[2], (int )data[3], (int )data[4], (int )data[5], (int )data[6],
                                        (int )data[7]);
                                memcpy(normal_key_report_st, data, HID_REPORT_NORMAL_REPORT_SIZE);    

                                p->type = FREE;
                                app_hid_report_push_to_list(&report_free_list, p);
                                return true;
                        }
                        else {
                                app_hid_report_return_to_list(&report_trm_list, p);
                        }
                }
        }     
        return false;
}

/*****************************************************************************************
 * \brief Sends an hid report
 *
 * \return  false, if sending the HID report failed
 *          true, if the HID report was successfully sent
******************************************************************************************/
static bool app_hid_report_send_hid_report(void)
{
        hid_rep_node_t *p;
        p = app_hid_report_pull_from_list(&report_trm_list);

        ASSERT_WARNING(p);
#if (BLE_HID_DEVICE)
        if (app_hogpd_send_report(p->char_id, p->data.data, p->data.length, HOGPD_REPORT)) {
#else
        // TODO: Implement alternative ways to send HID reports
        #warning "HOGPD profile is not enabled. HID reports cannot be sent"
        if(false) {
#endif        
                uint8_t *data = p->data.data;
            
                if(app_hid_report_send_report_cb != NULL) {
                    app_hid_report_send_report_cb(p->char_id, data, p->data.length); 
                }
                
                switch (p->char_id) {
                case HID_REPORT_NORMAL_REPORT_IDX:
                        dbg_printf(DBG_HID_LVL, "Sending HOGPD_REPORT_UPD_REQ %02x:[%02x:%02x:%02x:%02x:%02x:%02x]\r\n",
                                (int )data[0], (int )data[2], (int )data[3], (int )data[4], (int )data[5], (int )data[6],
                                (int )data[7]);
                        memcpy(normal_key_report_st, data, HID_REPORT_NORMAL_REPORT_SIZE);
                        break;
                case HID_REPORT_EXTENDED_REPORT_IDX:
                        dbg_puts(DBG_HID_LVL, "Sending HOGPD_REPORT_UPD_REQ");
                        for(int i = 0; i < HID_REPORT_EXTENDED_REPORT_SIZE; i++) {
                                dbg_printf(DBG_HID_LVL, " %02x", (int)data[i]);
                        }
                        dbg_puts(DBG_HID_LVL, "\r\n");
                        memcpy(extended_key_report_st, data, HID_REPORT_EXTENDED_REPORT_SIZE);
                        break;
                default:
                        break;
                }
                
                p->type = FREE;
                app_hid_report_push_to_list(&report_free_list, p);
                return true;
        }
        else {
                app_hid_report_return_to_list(&report_trm_list, p);
        }
        return false;
}

/*****************************************************************************************
 * \brief Sends the first key report in the trm list, if any, to HOGPD.
 *        A notification will be sent to the Host, if notifications are enabled. 
 *
 * \return  false, if sending the key report failed
 *          true, if the key report was successfully sent
******************************************************************************************/
static bool send_hid_report(void)
{
#if (BLE_HID_DEVICE)    
        if (hogpd_params.boot_protocol_mode &&
            app_hogpd_get_protocol_mode() == HOGP_BOOT_PROTOCOL_MODE) {
#else
        if(false) {
#endif                            
                return app_hid_report_send_boot_report();
        }
        else 
        {
                return app_hid_report_send_hid_report();
        }
}

/*****************************************************************************************
 * \brief Clears rollover data 
******************************************************************************************/
static void clear_rollover_data(void) 
{
        for (int i = 0; i < HID_REPORT_ROLL_OVER_BUF_SIZE; i++)
                roll_over_info.intersections[i] = RLOVR_INVALID_INTERSECTION;
        roll_over_info.cnt = 0;    
}

bool app_hid_report_is_report_list_empty(void)
{
        return report_trm_list == NULL;
}

bool app_hid_report_is_report_list_full(void)
{
        return report_free_list == NULL;
}

void app_hid_report_init(void)
{
        int i;

        hid_rep_node_t *node;
        report_trm_list = NULL;

        for (i = 0; i < HID_REPORT_FIFO_SIZE; i++) {
                memset(hid_report[i], 0, HID_REPORT_MAX_REPORT_SIZE);
                node = &report_list[i];
                node->data.data = hid_report[i];
                node->type = FREE;
                node->pNext = report_free_list;
                report_free_list = node;
        }
        
        clear_rollover_data();
        
        normal_key_report_st[0] = 0xFF;     // invalidate
        extended_key_report_st[0] = 0;
        extended_key_report_st[1] = 0;
        extended_key_report_st[2] = 0;

}

uint8_t app_hid_report_on_system_powered(void)
{    
        return app_hid_report_is_report_list_empty() ? APP_GOTO_SLEEP : APP_BLE_WAKEUP;
}

enum hid_report_status app_hid_report_send_report(void)
{    
        if (app_hid_report_is_report_list_empty()) {
                return HID_REPORT_EMPTY;
        }

        if(send_hid_report() == false) {
                return HID_REPORT_FAILED;
        }

        if (app_hid_report_is_report_list_empty()) {
                return HID_LAST_REPORT_SENT;
        }
        
        return HID_REPORTS_AVAILABLE;
}

/*****************************************************************************************
 * \brief Get the last report pending of the specified type.
 *
 * \param   report_id The HID report id to search for
 *
 * \return  A pointer to the report data if found, else NULL.
******************************************************************************************/
static uint8_t * app_hid_report_get_last_report_data(uint8_t report_id)
{
        hid_rep_node_t *p_report, *p_tmp;

        p_report = NULL;
        p_tmp = report_trm_list;

        // Get the last pending key report of this type, if any
        while (p_tmp) {
                if (p_tmp->char_id == report_id) {
                        p_report = p_tmp;
                }
                p_tmp = p_tmp->pNext;
        }

        if (p_report != NULL) {
                return p_report->data.data;
        }
        else {
            switch(report_id) {
            case HID_REPORT_NORMAL_REPORT_IDX:
                    if (normal_key_report_st[0] != 0xFF) {
                            return normal_key_report_st;
                    }
                    else {
                            return (uint8_t *)empty_report;
                    }
            case HID_REPORT_EXTENDED_REPORT_IDX:
                    return extended_key_report_st;
            default:
                return NULL;
            }
        }
}

uint16_t app_hid_report_get_free_nodes(void)
{
	hid_rep_node_t *node;
    uint16_t count = 0;
	
	for (node = report_free_list; node != NULL; node = node->pNext) {
		if (node->type == FREE) {
            count++;
        }
    }
	return count;
}

void app_hid_report_clear_lists(void)
{
        hid_rep_node_t *pReportInfo;
    
        while (report_trm_list) {
                pReportInfo = app_hid_report_pull_from_list(&report_trm_list);
                pReportInfo->type = FREE;
                app_hid_report_push_to_list(&report_free_list, pReportInfo);
        }  
        
        clear_rollover_data();
        
        // Clear (or invalidate) the content of the last reports sent to the old host
        normal_key_report_st[0] = 0xFF;     // invalidate
        extended_key_report_st[0] = 0;
        extended_key_report_st[1] = 0;
        extended_key_report_st[2] = 0;
        
}

uint8_t * app_hid_report_add_report(uint8_t report_id, enum KEY_BUFF_TYPE type, uint8_t *data, uint16_t size)
{       
        hid_rep_node_t *p_report;

        p_report = app_hid_report_pull_from_list(&report_free_list);
    
#ifdef HID_REPORT_FULL_WARNING
        ASSERT_WARNING(p_report);
#endif        
        if (p_report) {
                p_report->type = type;
                p_report->char_id = report_id;
            
#if (BLE_HID_DEVICE)               
                ASSERT_ERROR(size <= hogpd_reports[report_id].size);
#endif
                ASSERT_ERROR(size <= HID_REPORT_MAX_REPORT_SIZE);
                p_report->data.length = size;
                if(data == NULL) {
                        data = (uint8_t *)empty_report;
                }
                
                memcpy(p_report->data.data, data, p_report->data.length);
                app_hid_report_push_to_list(&report_trm_list, p_report);
                return p_report->data.data;    
        }
        
        return NULL;
}

/*****************************************************************************************
 * \brief Adds a NORMAL key report of the in the trm list. 
 *
 * \param[in]   type        The type of the report to add (PRESS, RELEASE)       
 *
 * \return  A pointer to the newly allocated report
******************************************************************************************/
static uint8_t* add_normal_report(enum KEY_BUFF_TYPE type)
{
        uint8_t *pReportData, *_pReportData;

        _pReportData = app_hid_report_get_last_report_data(HID_REPORT_NORMAL_REPORT_IDX);

        // add one <type> report
        pReportData = app_hid_report_add_report(HID_REPORT_NORMAL_REPORT_IDX, type, _pReportData, HID_REPORT_NORMAL_REPORT_SIZE);

        return pReportData;
}

/*****************************************************************************************
 * \brief Brings all used entries at the beginning of the report
 *
 * \param[in]   buf     The buffer       
 * \param[in]   start   the entry index at which to start sorting       
 * \param[in]   len     the count of the entries to sort     
******************************************************************************************/
static void sort_report_data(uint8_t *buf, int start, int len)
{
        // bring all used entries at the beginning of the report
        for (int i = start; i < start + len; i++) {
                if (buf[i]) {
                        continue;   // not zero
                }
                for (int j = i + 1; j < start + len; j++) {
                        if (buf[j] != 0) {
                                buf[i] = buf[j];
                                buf[j] = 0;
                                break;
                        }
                }
        }
}

/*****************************************************************************************
 * \brief Sorts the rollover data
 *
 * \param[in]   buf     The buffer       
 * \param[in]   start
 * \param[in]   len     the count of the entries to sort     
******************************************************************************************/
static void sort_rollover_data(uint16_t *buf, int start, int len)
{
        // bring all used entries at the beginning of the report
        for (int i = start; i < start + len; i++) {
                if (buf[i] != RLOVR_INVALID_INTERSECTION) {
                        continue;   // valid
                }
                for (int j = i + 1; j < start + len; j++) {
                        if (buf[j] != RLOVR_INVALID_INTERSECTION) {
                                buf[i] = buf[j];
                                buf[j] = RLOVR_INVALID_INTERSECTION;
                                break;
                        }
                }
        }
}

bool app_hid_report_modify_kbd_keyreport(const uint16_t keycode, uint8_t pressed)
{
        int i;
        uint8_t *pReportData, *_pReportData;

        const uint8_t keychar = keycode & 0xFF;
        const uint8_t keymode = keycode >> 8;

        // 1. The Key Report is filled from pos 2 to pos 7. It monitors the state of up to 6 keys. If more are pressed then
        //    RollOver functionality (Phantom state) should be applied.
        // 2. When something is written at a pos the host translates it as a key press.
        // 3. When something is written at (pos+1) the host translates it as a key press for key@(pos+1)
        // 4. If the report has '0' in all other positions then the host we'll be informed for only these two key presses.
        //    The host will think that the last entry written (i.e. key@(pos+1)) is still being pressed and update it
        //    (i.e. in a Word Processing application) with the "key repetition rate" as set in its settings.
        // 5. For the host to be informed for a key release of key@(pos+1) there are two cases:
        //    a. A Key Report is sent with all '0's which clears all key presses.
        //    b. The last Key Report is sent with 0x00@(pos+1) which means that the rest of the keys are still being pressed.
        // 6. Allowed trm sequence for all keys: PRESS (x N) -> RELEASE -> PRESS (x N) -> RELEASE...,
        //                                       RELEASE -> PRESS (x N) -> RELEASE -> PRESS (x N) -> RELEASE...
    
        switch (keymode & 0xFC) {
        case 0x00: // normal key
                if (pressed) {
                        int i;

                        for (i = 0; i < roll_over_info.cnt; i++) {
                                if ((roll_over_info.intersections[i] & 0xFF) == keychar) {
                                        break;
                                }
                        }
                        if (i < roll_over_info.cnt) {
                                ASSERT_ERROR(0);
                        }
                        if (roll_over_info.cnt < HID_REPORT_ROLL_OVER_BUF_SIZE) {
                                roll_over_info.intersections[roll_over_info.cnt] = keychar;
                                roll_over_info.cnt++;
                        }
                    
                        if (roll_over_info.cnt > 7) {
                                return true; // consumed
                        }
                        // add one press report
                        pReportData = add_normal_report(PRESS);
                        if (pReportData == NULL) {
                                return false;
                        }
                        if (roll_over_info.cnt == 7) {
                                // format report as a Roll-Over Report. Modifiers are still reported.
                                for (i = 2; i < 8; i++) {
                                        pReportData[i] = 0x01;
                                }
                                return true;
                        }

                        for (i = 2; i < HID_REPORT_NORMAL_REPORT_SIZE; i++) {
                                if (!pReportData[i]) {
                                        break;
                                }
                        }
                        if (i == HID_REPORT_NORMAL_REPORT_SIZE) { // Full! Should never happen!
                                ASSERT_ERROR(0);
                        }
                        pReportData[i] = keychar;
                }
                else {
                        // released
                        // Roll-Over processing (all key releases are cleared from the Roll-Over buffer)

                        // check if in Phantom state (Roll-Over)
                        if (roll_over_info.cnt > 0) {
                                // find the key and delete it
                                for (i = 0; i < roll_over_info.cnt; i++) {
                                        if ((roll_over_info.intersections[i] & 0xFF) == keychar) {
                                                break;
                                        }
                                }

                                if (i == roll_over_info.cnt) { // not found
                                        return true;   // consume all not found normal releases
                                }
                                else { // found
                                        roll_over_info.intersections[i] = RLOVR_INVALID_INTERSECTION;
                                        sort_rollover_data(roll_over_info.intersections, 0, roll_over_info.cnt);
                                        roll_over_info.cnt--;

                                        if (roll_over_info.cnt > 6) {
                                                return true;   // consume the release. Still in Phantom state.
                                        }
                                        if (roll_over_info.cnt == 6) {
                                                roll_over_info.intersections[6] = RLOVR_INDICATION_CODE;

//                                                if (keycode) {
//                                                        if (keymode != 0x00)    // not a normal key ==> force an HID report update!
//                                                                ret = app_hid_report_modify_keyreport(0, 0, 0);
//                                                }
                                        }
                                }
                        }
                        // check if in Phantom state (Roll-Over)
                        if (roll_over_info.cnt > 6) {
                                ASSERT_ERROR(0);
                        }
                        // add one RELEASE report
                        pReportData = add_normal_report(RELEASE);
                        if (pReportData == NULL) {
                                return false;
                        }
                        if ((roll_over_info.cnt <= 6) && (roll_over_info.intersections[6] == RLOVR_INDICATION_CODE)) {
                                // Phantom state is over. Send a RELEASE report with the current status.
                                for (i = 2; i < 8; i++) {
                                        pReportData[i] = 0;
                                }
                                roll_over_info.intersections[6] = RLOVR_INVALID_INTERSECTION;
                                sort_rollover_data(roll_over_info.intersections, 0, 6);

                                for (i = 0; i < roll_over_info.cnt; i++) {
                                        pReportData[i + 2] = roll_over_info.intersections[i] & 0xFF;
                                }
                                return true;
                        }

                        for (i = 2; i < HID_REPORT_NORMAL_REPORT_SIZE; i++) {
                                if (pReportData[i] == keychar) {
                                        pReportData[i] = 0x00;
                                        break;
                                }
                        }

                        if (i == HID_REPORT_NORMAL_REPORT_SIZE) { // consume release (key not found)
                                return true;
                        }
                        // bring all used entries at the beginning of the report
                        sort_report_data(pReportData, 2, 6);
                }

                break;
        case 0xFC: // modifier key
        {
                // if pressed, add a PRESS report after copying the data of the last report pending
                // if released, add a RELEASE report after copying the data of the last report pending
                char modifier, new_modifier;

                // get last report to find the modifiers' status
                _pReportData = app_hid_report_get_last_report_data(HID_REPORT_NORMAL_REPORT_IDX);
                modifier = _pReportData[0];

                new_modifier = (modifier & (~keychar)) | (pressed ? keychar : 0);

                if (new_modifier != modifier)   // normally this will always be true
                        {
                        enum KEY_BUFF_TYPE type;

                        // add "modifier" report in the trm list
                        if (pressed) {
                                type = PRESS;
                        }
                        else {
                                type = RELEASE;
                        }
                        
                        pReportData = app_hid_report_add_report(HID_REPORT_NORMAL_REPORT_IDX, type, _pReportData, HID_REPORT_NORMAL_REPORT_SIZE);
                        if (pReportData == NULL) {
                                return false;
                        }
                        pReportData[0] = new_modifier;
                }
                break;
        }
        default: // Other key that is not directly reportable in the hid_report
                break;
        }

        return true;
}

bool app_hid_report_add_full_release_report(void)
{    
        uint8_t *pReportData;

        pReportData = app_hid_report_add_report(HID_REPORT_NORMAL_REPORT_IDX, RELEASE, NULL, HID_REPORT_NORMAL_REPORT_SIZE);
        if (pReportData == NULL) {
                return false;
        }

        //clear Roll-Over info
        clear_rollover_data();
        
        return true;
}

bool app_hid_report_create_extended_report(uint8_t byte, uint8_t mask, uint8_t value)
{
        uint8_t *_pReportData, *pReportData;
    
        ASSERT_ERROR(byte < HID_KBD_EXTENDED_REPORT_SIZE);

        _pReportData = app_hid_report_get_last_report_data(HID_REPORT_EXTENDED_REPORT_IDX);

        pReportData = app_hid_report_add_report(HID_REPORT_EXTENDED_REPORT_IDX, EXTENDED, _pReportData, HID_REPORT_EXTENDED_REPORT_SIZE);
        if (pReportData == NULL) {
                return false;
        }
        
        pReportData[byte] &= ~mask;
        pReportData[byte] |= (value & mask);

        return true;
}

#endif // HAS_HID_REPORT

/**
 * \}
 * \}
 * \}
 */
