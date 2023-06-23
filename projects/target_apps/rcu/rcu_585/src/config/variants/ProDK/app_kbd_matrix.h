/**
* \file app_kbd_matrix.h
* \brief HID Keyboard matrix.
*/

#ifndef _APP_KBD_MATRIX_H_
#define _APP_KBD_MATRIX_H_

#include <stdint.h>
#include <stddef.h>

#include "app_kbd_defs.h"
#include <user_hogpd_config.h>

/**
* \brief Enable multi-key combinations
*/
#define MULTI_KEY_COMBINATIONS_ON

/**
* \brief Maximum number of keys for key combination (maximum value is 8)
*/
#define MULTI_KEY_NUM_OF_KEYS       2      // maximum 8

/**
* \brief Custom key definition. These keys are not treated as normal keys. No HID report is
*        created. Instead kbd_params.notify_callback will be called when the corresponding key 
*        or key combinations is pressed. These definitions can be used to define keys in
*        the key matrix or can be used in multi_key_combinations table table definition.
*/

enum custom_keys {
        CUSTOM_KEY_PAIR = 1,
        CUSTOM_KEY_CLEAR_BONDING_DATA,
        CUSTOM_KEY_AUDIO,
        CUSTOM_KEY_MOTION,
        CUSTOM_KEY_MOTION_CLICK,
        CUSTOM_KEY_POWER,
        CUSTOM_KEY_SWITCH_TO_HOST1,
        CUSTOM_KEY_SWITCH_TO_HOST2,
        CUSTOM_KEY_SWITCH_TO_HOST3,
        CUSTOM_KEY_PAIR_WITH_HOST1,
        CUSTOM_KEY_PAIR_WITH_HOST2,
        CUSTOM_KEY_PAIR_WITH_HOST3,
#ifdef MOUSE_GENERATE_TEST_PATTERN    
        CUSTOM_KEY_MOUSE_TEST,
#endif    
#ifdef DEBUG_EMULATE_PACKET_LOSS
        CUSTOM_KEY_INC_PACKET_LOSS,
        CUSTOM_KEY_DEC_PACKET_LOSS,
#endif    
        NUM_OF_CUSTOM_KEYS_PLUS_ONE
};

enum modifier_keys {
        NUM_OF_MOD_KEYS_PLUS_ONE
};

#define PAIR_KEY      KBD_CUSTOM_KEY(CUSTOM_KEY_PAIR)
#define CLRP_KEY      KBD_CUSTOM_KEY(CUSTOM_KEY_CLEAR_BONDING_DATA)
#define AUDIO_KEY     KBD_CUSTOM_KEY(CUSTOM_KEY_AUDIO)
#define MOTION_KEY    KBD_CUSTOM_KEY(CUSTOM_KEY_MOTION)
#define MOTION_CLICK  KBD_CUSTOM_KEY(CUSTOM_KEY_MOTION_CLICK)
#define POWER_KEY     KBD_CUSTOM_KEY(CUSTOM_KEY_POWER)

#define VOL_PLUS_KEY  KBD_SPECIAL_KEY(VOL_PLUS_KEY_CODE)
#define VOL_MINUS_KEY KBD_SPECIAL_KEY(VOL_MINUS_KEY_CODE)

// extra sets for 'hidden modifiers', e.g. the 'Fn' key
#define KBD_NR_SETS (1)

/**
 ****************************************************************************************
 * The key map.
 * 00xx:   regular key
 * FCxx:   modifier key.
 * F8xx:   FN Modifier.
 * F4xy:   special function (x = no of byte in the extended key report, y no of bit set).
 * F4Fy:   custom key
 * K_CODE: unknown key code - nothing is sent to the other side but the key is examined 
 *         for ghosting
 * KEY_UNUSED: if the key is not used
 ****************************************************************************************
 */
static const uint16_t kbd_keymap[KBD_NR_SETS][KBD_NR_ROW_OUTPUTS][KBD_NR_COLUMN_INPUTS] =
{
  {
    /* 0          1       2           3
      ----------------------------------------------
      1          2       3           Enter
      4          5       6           Power
      7          8       9           Vol+
      AUDIO      0       MOTION      Vol-
      ----------------------------------------------*/
    { 0x001E,    0x001F, 0x0020,     0x0028       }, // 0
    { 0x0021,    0x0022, 0x0023,     POWER_KEY    }, // 1
    { 0x0024,    0x0025, 0x0026,     VOL_PLUS_KEY }, // 2
    { AUDIO_KEY, 0x0027, MOTION_KEY, VOL_MINUS_KEY}, // 3
  }
};

/**
 *******************************************************************************************
 * Definition of multi-key combinations. For each combination a row is added in the     
 * multi_key_combinations matrix. In each row the following members must be defined:    
 *    i) an array with MULTI_KEY_NUM_OF_KEYS elements of type multi_key_t containing    
 *       the row and column of each key used in the key combination. If a key is not     
 *       used in the key combination (e.g. two keys are needed when MULTI_KEY_NUM_OF_KEYS 
 *       is 3) then row and column must be set to MULTI_KEY_NOT_USED
 *   ii) The action executed when key combination is pressed. Available actions are
 *       defined in custom_keys. Macro KBD_KEY_TO_NOTIFY_CODE can be used to generate the
 *       appropriate notification code.
 ******************************************************************************************
 */
#ifdef MULTI_KEY_COMBINATIONS_ON

struct multi_key_combinations_t {
    struct multi_key_t key[MULTI_KEY_NUM_OF_KEYS];
    enum kbd_notification notification;
};

static const struct multi_key_combinations_t multi_key_combinations[] = {
    //    row1 col1   row2 col2                          Custom key                     pressed
    { { { 3,   0 }, { 3,   3 } }, KBD_KEY_TO_NOTIFY_CODE(CUSTOM_KEY_CLEAR_BONDING_DATA, true) },
    { { { 0,   0 }, { 0,   2 } }, KBD_KEY_TO_NOTIFY_CODE(CUSTOM_KEY_PAIR,               true) },
    { { { 0,   3 }, { 0,   0 } }, KBD_KEY_TO_NOTIFY_CODE(CUSTOM_KEY_SWITCH_TO_HOST1,    true) },
    { { { 0,   3 }, { 0,   1 } }, KBD_KEY_TO_NOTIFY_CODE(CUSTOM_KEY_SWITCH_TO_HOST2,    true) },
    { { { 0,   3 }, { 0,   2 } }, KBD_KEY_TO_NOTIFY_CODE(CUSTOM_KEY_SWITCH_TO_HOST3,    true) },
    { { { 3,   3 }, { 0,   0 } }, KBD_KEY_TO_NOTIFY_CODE(CUSTOM_KEY_PAIR_WITH_HOST1,    true) },
    { { { 3,   3 }, { 0,   1 } }, KBD_KEY_TO_NOTIFY_CODE(CUSTOM_KEY_PAIR_WITH_HOST2,    true) },
    { { { 3,   3 }, { 0,   2 } }, KBD_KEY_TO_NOTIFY_CODE(CUSTOM_KEY_PAIR_WITH_HOST3,    true) },
#ifdef MOUSE_GENERATE_TEST_PATTERN    
    { { { 1,   0 }, { 1,   3 } }, KBD_KEY_TO_NOTIFY_CODE(CUSTOM_KEY_MOUSE_TEST,         true) },
#endif
};   
#endif

#endif // _APP_KBD_MATRIX_H_
