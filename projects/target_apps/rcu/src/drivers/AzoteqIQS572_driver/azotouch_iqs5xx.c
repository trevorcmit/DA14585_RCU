/*****************************************************************************************
 *
 * \file azotouch_iqs5xx.c
 *
 * \brief Azoteq IQS5xx module library sourcecode
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
******************************************************************************************/

/*****************************************************************************************
 * \addtogroup USER
 * \{
 * \addtogroup USER_DRIVERS
 * \{
 * \addtogroup IQS572_I2C_DRV
 *
 * \{
******************************************************************************************/
#include "i2c_azotouch.h"
#include "azotouch_iqs5xx.h"

/*
 * DEFINES
******************************************************************************************/

#define AZOTOUCH_IQS_550_PRODUCT_NUMBER		40
#define AZOTOUCH_IQS_525_PRODUCT_NUMBER		52
#define AZOTOUCH_IQS_572_PRODUCT_NUMBER		58

#ifndef AZOTOUCH_IQS_XXX_PRODUCT_NUMBER
#define AZOTOUCH_IQS_XXX_PRODUCT_NUMBER		AZOTOUCH_IQS_572_PRODUCT_NUMBER
#endif


/**
 ***************************************************************************************
 * I2C Configuration for the IQS572 module                                                
 ***************************************************************************************
 */
#define I2C_AZOTOUCH_IQS5XX_SLAVE_ADDRESS       0x74                    // The IQS572 I2C device address is 0x74
#define I2C_AZOTOUCH_IQS5XX_SPEED_MODE          I2C_AZOTOUCH_FAST       // Selected Speed mode: I2C_FAST (400 kbits/s)
#define I2C_AZOTOUCH_IQS5XX_ADDRESS_MODE        I2C_AZOTOUCH_7BIT_ADDR  // Addressing mode: I2C_7BIT_ADDR
#define I2C_AZOTOUCH_IQS5XX_ADDRESS_SIZE        2                       // The internal address(register address) size is 2 bytes



// GestureEvents0       1 byte
// GestureEvents1       1 byte
// SystemInfo0          1 byte
// SystemInfo1          1 byte
// NumberOfFingers      1 byte
// RelativeX            2 bytes
// RelativeY            2 bytes
// (AbsX 2 bytes + AbsY 2 bytes + TouchStrength 2 bytes + Area size 1 byte) * AZOTOUCH_MAX_FINGERS
#define AZOTOUCH_TOUCH_AND_GESTURE_DATA_SIZE		((1+1+1+1+1+2+2) + (2+2+2*AZOTOUCH_IQS5XX_USE_TOUCH_STR+1*AZOTOUCH_IQS5XX_USE_FINGER_AREA)* AZOTOUCH_MAX_FINGERS)


/*
*
* Memory map registers, and bit definitions
* The access rights of the memory map are also indicated as follows: 
* (READ)          : Read only
* (READ/WRITE)    : Read & Write
* (READ/WRITE/E2) : Read, Write & default loaded from non-volatile memory
*
*/

// IQS5xx-B000 BIT DEFINITIONS

// GestureEvents0 bit definitions
#define SWIPE_Y_NEG	    	0x20
#define SWIPE_Y_POS	    	0x10
#define SWIPE_X_POS      	0x08
#define SWIPE_X_NEG        	0x04
#define TAP_AND_HOLD     	0x02
#define SINGLE_TAP        	0x01

// GesturesEvents1 bit definitions
#define ZOOM	          	0x04
#define SCROLL		     	0x02
#define TWO_FINGER_TAP    	0x01

// SystemInfo0 bit definitions
#define	SHOW_RESET			0x80
#define	ALP_REATI_OCCURRED  0x40
#define	ALP_ATI_ERROR		0x20
#define	REATI_OCCURRED		0x10
#define	ATI_ERROR			0x08
#define	CHARGING_MODE_2	    0x04
#define CHARGING_MODE_1	    0x02
#define	CHARGING_MODE_0	    0x01

// SystemInfo1 bit definitions
#define	SNAP_TOGGLE			0x10
#define	RR_MISSED			0x08
#define	TOO_MANY_FINGERS	0x04
#define PALM_DETECT			0x02
#define	TP_MOVEMENT			0x01

// SystemControl0 bit definitions
#define ACK_RESET           0x80
#define AUTO_ATI            0x20
#define ALP_RESEED          0x10
#define RESEED              0x08
#define MODE_SELECT_2		0x04
#define MODE_SELECT_1		0x02
#define MODE_SELECT_0		0x01

// SystemControl1 bit definitions
#define RESET             	0x02
#define SUSPEND           	0x01

// SystemConfig0 bit definitions        
#define MANUAL_CONTROL     	0x80
#define SETUP_COMPLETE     	0x40
#define WDT_ENABLE        	0x20
#define ALP_REATI        	0x08
#define REATI            	0x04
#define IO_WAKEUP_SELECT   	0x02
#define IO_WAKE         	0x01

// SystemConfig1 bit definitions 
#define PROX_EVENT        	0x80
#define TOUCH_EVENT       	0x40
#define SNAP_EVENT        	0x20
#define ALP_PROX_EVENT    	0x10
#define REATI_EVENT      	0x08
#define TP_EVENT          	0x04
#define GESTURE_EVENT     	0x02
#define EVENT_MODE        	0x01

// FilterSettings0 bit definitions
#define ALP_COUNT_FILTER    0x08
#define IIR_SELECT			0x04
#define MAV_FILTER     		0x02
#define IIR_FILTER     		0x01

// ALPChannelSetup0 bit definitions             
#define CHARGE_TYPE  		0x80
#define RX_GROUP        	0x40
#define PROX_REV       		0x20
#define ALP_ENABLE          0x10

// IQS525RxToTx bit definitions
#define RX7_TX2     		0x80
#define RX6_TX3     		0x40
#define RX5_TX4     		0x20
#define RX4_TX5    			0x10
#define RX3_TX6    			0x08
#define RX2_TX7     		0x04
#define RX1_TX8     		0x02
#define RX0_TX9     		0x01

// HardwareSettingsA bit definitions    
#define ND_ENABLE         	0x20
#define RX_FLOAT          	0x04

// HardwareSettingsB bit definitions
#define CK_FREQ_2      		0x40
#define CK_FREQ_1     		0x20
#define CK_FREQ_0    		0x10
#define ANA_DEAD_TIME       0x02
#define INCR_PHASE          0x01

// HardwareSettingsC bit definitions
#define STAB_TIME_1     	0x80
#define STAB_TIME_0     	0x40
#define OPAMP_BIAS_1   		0x20
#define OPAMP_BIAS_0     	0x10
#define VTRIP_3				0x08
#define VTRIP_2             0x04
#define VTRIP_1             0x02
#define VTRIP_0             0x01

// HardwareSettingsD bit definitions                          
#define UPLEN_2    			0x40
#define UPLEN_1   			0x20
#define UPLEN_0     		0x10
#define PASSLEN_2           0x04
#define PASSLEN_1           0x02
#define PASSLEN_0           0x01

// XYConfig0 bit definitions                            
#define PALM_REJECT         0x08
#define SWITCH_XY_AXIS      0x04
#define FLIP_Y              0x02
#define FLIP_X              0x01

// SFGestureEnable bit definitions
#define SWIPE_Y_MINUS_EN  	0x20
#define SWIPE_Y_PLUS_EN   	0x10
#define SWIPE_X_PLUS_EN   	0x08
#define SWIPE_X_MINUS_EN  	0x04
#define TAP_AND_HOLD_EN   	0x02
#define SINGLE_TAP_EN     	0x01

// MFGestureEnable bit definitions
#define ZOOM_EN           	0x04
#define SCROLL_EN         	0x02
#define TWO_FINGER_TAP_EN 	0x01


// IQS5xx-B00 MEMORY MAP REGISTERS      

// Device Info Registers
#define PRODUCTNUMBER_ADR		0x0000  //(READ)                //2 BYTES;
#define PROJECTNUMBER_ADR		0x0002  //(READ)                //2 BYTES;
#define MAJORVERSION_ADR		0x0004  //(READ)
#define MINORVERSION_ADR		0x0005  //(READ)
#define BLSTATUS_ADR			0x0006  //(READ)


#define MAXTOUCH_ADR			0x000B  //(READ)
#define PREVCYCLETIME_ADR		0x000C  //(READ)
// Gestures And Event Status Registers
#define GESTUREEVENTS0_ADR		0x000D  //(READ)
#define GESTUREEVENTS1_ADR		0x000E  //(READ)
#define SYSTEMINFO0_ADR			0x000F  //(READ)
#define SYSTEMINFO1_ADR			0x0010  //(READ)

// XY Data Registers
#define NOOFFINGERS_ADR			0x0011  //(READ)
#define RELATIVEX_ADR			0x0012  //(READ)                //2 BYTES;
#define RELATIVEY_ADR			0x0014  //(READ)                //2 BYTES;

// Individual Finger Data
#define ABSOLUTEX_ADR			0x0016  //(READ) 2 BYTES        //ADD 0x0007 FOR FINGER 2; 0x000E FOR FINGER 3; 0x0015 FOR FINGER 4 AND 0x001C FOR FINGER 5
#define ABSOLUTEY_ADR			0x0018  //(READ) 2 BYTES        //ADD 0x0007 FOR FINGER 2; 0x000E FOR FINGER 3; 0x0015 FOR FINGER 4 AND 0x001C FOR FINGER 5
#define TOUCHSTRENGTH_ADR		0x001A  //(READ) 2 BYTES        //ADD 0x0007 FOR FINGER 2; 0x000E FOR FINGER 3; 0x0015 FOR FINGER 4 AND 0x001C FOR FINGER 5
#define AREA_ADR				0x001C  //(READ)                //ADD 0x0007 FOR FINGER 2; 0x000E FOR FINGER 3; 0x0015 FOR FINGER 4 AND 0x001C FOR FINGER 5

// Channel Status Registers
#define PROXSTATUS_ADR			0x0039  //(READ)                //32 BYTES;
#define TOUCHSTATUS_ADR			0x0059  //(READ)                //30 BYTES;
#define SNAPSTATUS_ADR			0x0077  //(READ)                //30 BYTES;

// Data Streaming Registers
#define COUNTS_ADR				0x0095  //(READ)                //300 BYTES;
#define DELTA_ADR				0x01C1  //(READ)                //300 BYTES;
#define ALPCOUNT_ADR			0x02ED  //(READ)                //2 BYTES;
#define ALPINDIVCOUNTS_ADR		0x02EF  //(READ)                //20 BYTES;
#define REFERENCES_ADR			0x0303  //(READ/WRITE)          //300 BYTES;
#define ALPLTA_ADR 				0x042F  //(READ/WRITE)          //2 BYTES;

// System Control Registers
#define SYSTEMCONTROL0_ADR 		0x0431  //(READ/WRITE)
#define SYSTEMCONTROL1_ADR 		0x0432  //(READ/WRITE)

// ATI Settings Registers
#define ALPATICOMP_ADR 			0x0435  //(READ/WRITE)          //10 BYTES;
#define	ATICOMPENSATION_ADR		0x043F  //(READ/WRITE)          //150 BYTES;
#define ATICADJUST_ADR          0x04D5  //(READ/WRITE/E2)       //150 BYTES;
#define GLOBALATIC_ADR          0x056B  //(READ/WRITE/E2)
#define ALPATIC_ADR				0x056C  //(READ/WRITE/E2)
#define ATITARGET_ADR			0x056D  //(READ/WRITE/E2)       //2 BYTES;
#define ALPATITARGET_ADR		0x056F  //(READ/WRITE/E2)       //2 BYTES;
#define REFDRIFTLIMIT_ADR		0x0571  //(READ/WRITE/E2)
#define ALPLTADRIFTLIMIT_ADR	0x0572  //(READ/WRITE/E2)
#define REATILOWERLIMIT_ADR		0x0573  //(READ/WRITE/E2)
#define REATIUPPERLIMIT_ADR		0x0574  //(READ/WRITE/E2)
#define MAXCOUNTLIMIT_ADR		0x0575  //(READ/WRITE/E2)       //2 BYTES;
#define REATIRETRYTIME_ADR		0x0577  //(READ/WRITE/E2)

// Timing Settings Registers
#define ACTIVERR_ADR		    0x057A  //(READ/WRITE/E2)       //2 BYTES;
#define	IDLETOUCHRR_ADR			0x057C  //(READ/WRITE/E2)       //2 BYTES;
#define	IDLERR_ADR				0x057E  //(READ/WRITE/E2)       //2 BYTES;
#define	LP1RR_ADR				0x0580  //(READ/WRITE/E2)       //2 BYTES;
#define	LP2RR_ADR				0x0582  //(READ/WRITE/E2)       //2 BYTES;
#define	ACTIVETIMEOUT_ADR		0x0584  //(READ/WRITE/E2)
#define IDLETOUCHTIMEOUT_ADR	0x0585  //(READ/WRITE/E2)
#define	IDLETIMEOUT_ADR			0x0586  //(READ/WRITE/E2)
#define	LP1TIMEOUT_ADR			0x0587  //(READ/WRITE/E2)
#define	REFUPDATETIME_ADR		0x0588  //(READ/WRITE/E2)
#define	SNAPTIMEOUT_ADR			0x0589  //(READ/WRITE/E2)
#define	I2CTIMEOUT_ADR			0x058A  //(READ/WRITE/E2)

// System Config Registers
#define SYSTEMCONFIG0_ADR  		0x058E  //(READ/WRITE/E2)
#define SYSTEMCONFIG1_ADR  		0x058F  //(READ/WRITE/E2)

// Threshold Settings Registers
#define SNAPTHRESHOLD_ADR       0x0592  //(READ/WRITE/E2)       //2 BYTES;
#define	PROXTHRESHOLD_ADR		0x0594  //(READ/WRITE/E2)
#define	ALPPROXTHRESHOLD_ADR	0x0595  //(READ/WRITE/E2)
#define	GLOBALTOUCHSET_ADR		0x0596  //(READ/WRITE/E2)
#define	GLOBALTOUCHCLEAR_ADR	0x0597  //(READ/WRITE/E2)
#define	INDIVTOUCHADJUST_ADR	0x0598  //(READ/WRITE/E2)       //150 BYTES;

// Filter Settings Registers
#define	FILTERSETTINGS0_ADR		0x0632  //(READ/WRITE/E2)
#define	XYSTATICBETA_ADR		0x0633  //(READ/WRITE/E2)
#define	ALPCOUNTBETA_ADR		0x0634  //(READ/WRITE/E2)
#define	ALP1LTABETA_ADR			0x0635  //(READ/WRITE/E2)
#define	ALP2LTABETA_ADR			0x0636  //(READ/WRITE/E2)
#define	DYNAMICBOTTOMBETA_ADR	0x0637  //(READ/WRITE/E2)
#define	DYNAMICLOWERSPEED_ADR	0x0638  //(READ/WRITE/E2)
#define	DYNAMICUPPERSPEED_ADR	0x0639  //(READ/WRITE/E2)       //2 BYTES;

// Channel Set Up (RX-TX Mapping) Registers
#define	TOTALRX_ADR				0x063D  //(READ/WRITE/E2)
#define	TOTALTX_ADR				0x063E  //(READ/WRITE/E2)
#define	RXMAPPING_ADR			0x063F  //(READ/WRITE/E2)       //10 BYTES;
#define	TXMAPPING_ADR			0x0649  //(READ/WRITE/E2)       //15 BYTES;
#define	ALPCHANNELSETUP0_ADR	0x0658  //(READ/WRITE/E2)
#define	ALPRXSELECT_ADR			0x0659  //(READ/WRITE/E2)       //2 BYTES;
#define	ALPTXSELECT_ADR			0x065B  //(READ/WRITE/E2)       //2 BYTES;
#define IQS525RXTOTX_ADR		0x065D  //(READ/WRITE/E2)

// Hardware Settings Registers
#define	HARDWARESETTINGSA_ADR	0x065F  //(READ/WRITE/E2)
#define	HARDWARESETTINGSB1_ADR	0x0660  //(READ/WRITE/E2)
#define	HARDWARESETTINGSB2_ADR	0x0661  //(READ/WRITE/E2)
#define	HARDWARESETTINGSC1_ADR	0x0662  //(READ/WRITE/E2)
#define	HARDWARESETTINGSC2_ADR	0x0663  //(READ/WRITE/E2)
#define	HARDWARESETTINGSD1_ADR	0x0664  //(READ/WRITE/E2)
#define	HARDWARESETTINGSD2_ADR	0x0665  //(READ/WRITE/E2)

// XY Config Registers
#define	XYCONFIG0_ADR			0x0669  //(READ/WRITE/E2)
#define	MAXMULTITOUCHES_ADR		0x066A  //(READ/WRITE/E2)
#define	FINGERSPLITFACTOR_ADR	0x066B  //(READ/WRITE/E2)
#define	PALMREJECTTHRESHOLD_ADr	0x066C  //(READ/WRITE/E2)
#define	PALMREJECTTIMEOUT_ADR	0x066D  //(READ/WRITE/E2)
#define	XRESOLUTION_ADR			0x066E  //(READ/WRITE/E2)       //2 BYTES;
#define	YRESOLUTION_ADR			0x0670  //(READ/WRITE/E2)       //2 BYTES;
#define	STATIONARYTOUCHTHR_ADR	0x0672  //(READ/WRITE/E2)


#define	DEFAULTREADADR_ADR		0x0675  //(READ/WRITE/E2)

// Debounce Setting Registers
#define	PROXDB_ADR				0x0679  //(READ/WRITE/E2)
#define	TOUCHSNAPDB_ADR			0x067A  //(READ/WRITE/E2)

// Channel Config Registers
#define	ACTIVECHANNELS_ADR	    0x067B  //(READ/WRITE/E2)       //30 BYTES;
#define	SNAPCHANNELS_ADR		0x0699  //(READ/WRITE/E2)       //30 BYTES;

// Gesture Setting Registers
#define	SFGESTUREENABLE_ADR		0x06B7  //(READ/WRITE/E2)
#define	MFGESTUREENABLE_ADR		0x06B8  //(READ/WRITE/E2)
#define	TAPTIME_ADR				0x06B9  //(READ/WRITE/E2)       //2 BYTES;
#define	TAPDISTANCE_ADR			0x06BB  //(READ/WRITE/E2)       //2 BYTES;
#define HOLDTIME_ADR            0x06BD  //(READ/WRITE/E2)       //2 BYTES;
#define	SWIPEINITTIME_ADR		0x06BF  //(READ/WRITE/E2)       //2 BYTES;
#define	SWIPEINITDISTANCE_ADR	0x06C1  //(READ/WRITE/E2)       //2 BYTES;
#define	SWIPECONSTIME_ADR		0x06C2  //(READ/WRITE/E2)       //2 BYTES;
#define	SWIPECONSDISTANCE_ADR	0x06C5  //(READ/WRITE/E2)       //2 BYTES;
#define	SWIPEANGLE_ADR			0x06C7  //(READ/WRITE/E2)
#define	SCROLLINITDISTANCE_ADR  0x06C8  //(READ/WRITE/E2)       //2 BYTES;
#define	SCROLLANGLE_ADR			0x06CA  //(READ/WRITE/E2)
#define	ZOOMINITDISTANCE_ADR	0x06CB  //(READ/WRITE/E2)       //2 BYTES;
#define	ZOOMCONSDISTANCE_ADR	0x06CD  //(READ/WRITE/E2)       //2 BYTES;



/*
* LOCAL FUNCTIONS
*/

// Azoteq Touch Misc Functions
static bool azotouch_iqs5xx_verify_module(void);
static void _azotouch_iqs5xx_ack_reset(void);
#ifdef AZOTOUCH_SNAP_STATUS_SUPPORTED
static void _azotouch_iqs5xx_get_snap_status(void);
#endif



/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
*/

// Snap Status Related variables
#ifdef AZOTOUCH_SNAP_STATUS_SUPPORTED
static uint16_t snapPrevStatus[AZOTOUCH_IQS5XX_TX_CHANNEL_BYTES];
static uint16_t snapCurrStatus[AZOTOUCH_IQS5XX_TX_CHANNEL_BYTES];
#endif // AZOTOUCH_SNAP_STATUS_SUPPORTED



/*****************************************************************************************
 * \brief Acknowledge the reset flag generated from Azoteq's Touch Controller
******************************************************************************************/
static void _azotouch_iqs5xx_ack_reset(void)
{
        uint8_t System_ctrl_0 = ACK_RESET;

        i2c_azotouch_data_write(SYSTEMCONTROL0_ADR, &System_ctrl_0, 1);
}


#ifdef AZOTOUCH_SNAP_STATUS_SUPPORTED

/*****************************************************************************************
 * \brief Parse and process a snap status change
******************************************************************************************/
static void _azotouch_iqs5xx_get_snap_status(void)
{
	uint8_t		ui8TempData[30];
	uint8_t		ui8Tx, ui8Rx;
	uint16_t	snapToggledBits;
	uint16_t 	snapBitMask;
	uint16_t    i;
	
	// Read the Snap Status Bytes of the controller
    if (!i2c_azotouch_random_read(SNAPSTATUS_ADR, ui8TempData, 30)) {
                return;
        }
	
	// Update the previous and current snap statuses
	for (i = 0; i < 15; i++) {
		snapPrevStatus[i] = snapCurrStatus[i];
		snapCurrStatus[i] = ((uint16_t)(ui8TempData[2*i])<<8) + 
							 (uint16_t)ui8TempData[(2*i)+1];
	}	
	
    for (ui8Tx = 0; ui8Tx < AZOTOUCH_IQS5XX_TX_CHANNEL_BYTES; ui8Tx++) {
                // Initialize the Snap Bit Mask
            snapBitMask = ((uint16_t)1<<(AZOTOUCH_IQS5XX_RX_CHANNEL_BYTES-1));
                
                // XOR the previous and current snap status bits in order to see
                // if and where there was a change
                snapToggledBits = snapPrevStatus[ui8Tx] ^ snapCurrStatus[ui8Tx];
            for (ui8Rx = 0; ui8Rx < AZOTOUCH_IQS5XX_RX_CHANNEL_BYTES; ui8Rx++){
                        
                        // If there was a changed bit in the current Rx/Tx channel
                        if (snapBitMask & snapToggledBits){
                                
                                        // Check whether this channel has snap or not
                        if (snapBitMask & snapCurrStatus[ui8Tx]){
				        	// Set 
				        	// On snap SET do something
				        	__ASM("NOP");
                        }else{
                                // Snap release
                                // On snap RELEASE do something
                                __ASM("NOP");
                        }
			}
			
			// Shift the Snap Status Bit mask right
			// for the next iteration
                        snapBitMask>>=1;
			
		}
        }
}
#endif // AZOTOUCH_SNAP_STATUS_SUPPORTED


void azotouch_iqs5xx_process_xy(azoteq_iqs_5xx_evt_t *azoEvt)
{
        uint8_t Azo_Buff[AZOTOUCH_TOUCH_AND_GESTURE_DATA_SIZE];
        uint8_t fingersDetected;

        // Init the I2C master in order to be able to interact with the trackpad module
        i2c_azotouch_init(I2C_AZOTOUCH_IQS5XX_SLAVE_ADDRESS, I2C_AZOTOUCH_IQS5XX_SPEED_MODE,
                          I2C_AZOTOUCH_IQS5XX_ADDRESS_MODE, I2C_AZOTOUCH_IQS5XX_ADDRESS_SIZE);

        // Read the events from the touch controller
        if (!i2c_azotouch_random_read
            (GESTUREEVENTS0_ADR, &Azo_Buff[0], AZOTOUCH_TOUCH_AND_GESTURE_DATA_SIZE)) {
                return;
        }

        if ((Azo_Buff[2] & SHOW_RESET) != 0) {
                // Acknowledge the reset event in order to clean the flag
                _azotouch_iqs5xx_ack_reset();

                // Clear the Element
                memset(azoEvt, 0, sizeof(azoteq_iqs_5xx_evt_t));

                // Mark the action as "reset"
                azoEvt->action = azotouch_iqs5xx_reset;

                // Return immediately. No need to read any other information
                return;
        }


#ifdef AZOTOUCH_SNAP_STATUS_SUPPORTED
        if ((Data_Buff[3] & SNAP_TOGGLE) != 0) {
                // A snap state change has occured,
                // so go ahead and get additional snap status info
                _azotouch_iqs5xx_get_snap_status();
                return;
        }
#endif // AZOTOUCH_SNAP_STATUS_SUPPORTED


        fingersDetected = Azo_Buff[4];

        // Get the absolute and relative coordinates of the fingers
        azoEvt->AbsX = ((Azo_Buff[9] << 8) | (Azo_Buff[10]));
        azoEvt->AbsY = ((Azo_Buff[11] << 8) | (Azo_Buff[12]));
        azoEvt->RelX = ((Azo_Buff[5] << 8) | (Azo_Buff[6]));
        azoEvt->RelY = ((Azo_Buff[7] << 8) | (Azo_Buff[8]));

        if ((Azo_Buff[0]) == SINGLE_TAP) {
                azoEvt->action = azotouch_iqs5xx_track_tap;
        }
        else if ((Azo_Buff[1]) == TWO_FINGER_TAP) {
                azoEvt->action = azotouch_iqs5xx_2_finger_tap;
        }
        else if (!fingersDetected) {
                azoEvt->action = azotouch_iqs5xx_release;
        }

        if (fingersDetected) {
                switch (Azo_Buff[0]) {

                case SINGLE_TAP:
                        azoEvt->action = azotouch_iqs5xx_track_tap;
                        break;

                case TAP_AND_HOLD:
                        azoEvt->action = azotouch_iqs5xx_tap_and_hold;
                        break;

                case SWIPE_X_NEG:
                        azoEvt->action = azotouch_iqs5xx_swipe_x_neg;
                        break;

                case SWIPE_X_POS:
                        azoEvt->action = azotouch_iqs5xx_swipe_x_pos;
                        break;

                case SWIPE_Y_POS:
                        azoEvt->action = azotouch_iqs5xx_swipe_y_pos;
                        break;

                case SWIPE_Y_NEG:
                        azoEvt->action = azotouch_iqs5xx_swipe_y_neg;
                        break;

                default:
                        if ((azoEvt->RelX || azoEvt->RelY) && azoEvt->AbsX != 0xFFFF
                            && azoEvt->AbsY != 0xFFFF) {
                                azoEvt->action = azotouch_iqs5xx_track;
                        }
                        else {
                                return;
                        }
                        break;

                }

                switch (Azo_Buff[1]) {

                case TWO_FINGER_TAP:
                        azoEvt->action = azotouch_iqs5xx_2_finger_tap;
                        break;

                case SCROLL:
                        azoEvt->action = azotouch_iqs5xx_scroll;
                        break;

                case ZOOM:
                        azoEvt->action = azotouch_iqs5xx_zoom;
                        break;
                }



        }

        // HANDLE EVENT

}


bool azotouch_iqs5xx_init_module()
{
        bool stat;

        // Init the I2C master in order to be able to interact with the trackpad module
        i2c_azotouch_init(I2C_AZOTOUCH_IQS5XX_SLAVE_ADDRESS, I2C_AZOTOUCH_IQS5XX_SPEED_MODE,
                          I2C_AZOTOUCH_IQS5XX_ADDRESS_MODE, I2C_AZOTOUCH_IQS5XX_ADDRESS_SIZE);

        stat = azotouch_iqs5xx_verify_module();
        // Init any module-specific settings (factory settings)
        // PLACEHOLDER (put it here if you want any additional settings);


        return stat;
}


void azotouch_iqs5xx_deinit_module()
{
        // De-init Azoteq Touchpad module
        // i2c_azotouch_release();
}


/*****************************************************************************************
 * \brief Pings the module about is product number and version
 *
 * \return bool true if the expected product number is correct
 *              false otherwise
******************************************************************************************/
static bool azotouch_iqs5xx_verify_module(void)
{
        uint8_t azotouchVersionInfo[2];

        /*
         * Dont wait for RDY here, since the device could be in EventMode, and then
         * there will be no communication window to complete this.  Rather do a 
         * forced communication, where clock stretching will be done on the IQS5xx
         * until an appropriate time to complete the i2c.
         */
        if (i2c_azotouch_force_communication()) // Check if forcing communication was successful
        {
                if (i2c_azotouch_random_read(PRODUCTNUMBER_ADR, azotouchVersionInfo, 2)) {
                        return (((azotouchVersionInfo[0] << 8) | azotouchVersionInfo[1]) ==
                                AZOTOUCH_IQS_XXX_PRODUCT_NUMBER);
                }
        }

        return false;
}

/**
 * \}
 * \}
 * \}
 */
