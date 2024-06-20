/***********************************************************
 * lcd_sms1706.c
 *
 *  Created on: 2/07/2021
 *      Author: Dave, completed by Trent
***********************************************************/


#include "esp_system.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

#include "driver/i2c.h"

#include "lcd_sms1706.h"


static const char *DTAG = "DISPLAY";    
uint8_t		u8a_disp_buf[(DISPLAY_BYTES + 1)];				// initial byte is always zero for the start address


// Function declarations
static void 	fv_display_set_large_num(double number, uint8_t max_dp);
static void 	fv_display_set_large_text(char c_text[]);
static void 	fv_display_set_small(char c_text[]);
static void		fv_display_set_g_reg_num(uint8_t g_reg_num);
static void		fv_display_set_pr_reg_num(uint8_t pr_reg_num);
static void		fv_display_set_t_num(uint8_t t_reg_num);
static void		fv_display_set_p_arrow(int8_t i_p_arrow);
static void		fv_display_set_q_arrow(int8_t i_q_arrow);
static void 	fv_display_set_units(uint8_t i_units);
static void	    fv_display_set_signal(uint8_t i_signal);
static void		fv_display_set_connected(uint8_t i_connected);
static void		fv_display_set_batt(uint8_t i_battery);
static void		fv_display_set_exclam(uint8_t i_exclam);
static void		fv_display_set_lock(uint8_t i_lock);
static void		fv_display_set_line(uint8_t i_line);
static void		fv_display_set_relay(uint8_t i_relay);


/**
 * @brief Write a byte to a sensor register
 */
static void fv_i2c_send(uint8_t addr, uint8_t len, uint8_t *data)
{
    i2c_master_write_to_device(0, addr, data, len, 1000);
}



/***********************************************************
	@brief	initialise the display bias, duty etc.
***********************************************************/
void		fv_display_on(void)
{
	uint8_t		buf[5];

	buf[0]		= DISPLAY_MODE_ON;
	fv_i2c_send(DISPLAY_I2C_ADDR, 1, buf);
} // end of fv_display_on ----------------------------------


/***********************************************************
	@brief	turn the display off.
***********************************************************/
void		fv_display_off(void)
{
	uint8_t		buf[5];

	buf[0]		= DISPLAY_MODE_OFF;
	fv_i2c_send(DISPLAY_I2C_ADDR, 1, buf);
} // end of fv_display_off ---------------------------------


/***********************************************************
	@brief	sets the display (contrast) voltage control.
***********************************************************/
void		fv_display_set_V(uint8_t vlcd)
{
	uint8_t		buf[5];

	buf[0]		= DISPLAY_VOLT_ADJ;
	fv_i2c_send(DISPLAY_I2C_ADDR, 1, buf);
} // end of fv_display_set_V -------------------------------


/***********************************************************
	@brief	sets the display blink state.
	@param	blink:	0 = constant, 1 = 2Hz, 2 = 1Hz, 3 = 0.5Hz
***********************************************************/
void		fv_display_blink(uint8_t blink)
{
	uint8_t		buf[5];

	switch (blink) {
	default:
	case	0:
		buf[0]			= DISPLAY_BLINK_OFF;
		break;

	case	1:
		buf[0]			= DISPLAY_BLINK_2HZ;
		break;

	case	2:
		buf[0]			= DISPLAY_BLINK_1HZ;
		break;

	case	3:
		buf[0]			= DISPLAY_BLINK_05HZ;
		break;
	}
	fv_i2c_send(DISPLAY_I2C_ADDR, 1, buf);
} // end of fv_display_blink -------------------------------


/***********************************************************
	@brief	turns all segments off to clear the screen
***********************************************************/
void		fv_display_clear(void)
{
	uint8_t		u8_i;
	uint8_t		buf[DISPLAY_BYTES + 1];						// +1 as first byte is SEG address

	for (u8_i = 0; u8_i < (DISPLAY_BYTES + 1); ++u8_i)
	{
		buf[u8_i]		= 0;
	}
	fv_i2c_send(DISPLAY_I2C_ADDR, (DISPLAY_BYTES + 1), buf);
} // end of fv_display_clear -------------------------------


/***********************************************************
	@brief	turns all segments on - used for display test.
***********************************************************/
void		fv_display_all_on(void)
{
	ESP_LOGI(DTAG,"inside - all on");

	uint8_t		u8_i;
	uint8_t		buf[DISPLAY_BYTES + 1];						// +1 as first byte is SEG address

	for (u8_i = 1; u8_i < (DISPLAY_BYTES + 1); ++u8_i)
	{
		buf[u8_i]		= 0xFF;
	}
	buf[0]				= 0;
	fv_i2c_send(DISPLAY_I2C_ADDR, (DISPLAY_BYTES + 1), buf);
} // end of fv_display_all_on ------------------------------


/***********************************************************
	@brief	sets the display to show as per the display struct content
	@param content: a struct of type display
***********************************************************/
void		fv_display_content(struct display content)
{
	// clear the buffer
	for (uint8_t i = 0; i < (DISPLAY_BYTES + 1); ++i)
	{
		u8a_disp_buf[i]		= 0;
	}

	// to show number or text, show text, else show number on large 8 digit display
	if (content.large_mode == DISPLAY_LARGE_NUMBER)
	{
		fv_display_set_large_num(content.large_number, content.large_num_max_dp);
	}
	else
	{
		fv_display_set_large_text(content.large_text);		
	}

	//show display items
	fv_display_set_small(content.small_text);
	fv_display_set_g_reg_num(content.g_reg_num);
	fv_display_set_connected(content.connected);
	fv_display_set_batt(content.battery);
	fv_display_set_exclam(content.exclamation);
	fv_display_set_q_arrow(content.q_arrow);
	fv_display_set_p_arrow(content.p_arrow);
	fv_display_set_lock(content.lock);
	fv_display_set_line(content.line);
	fv_display_set_units(content.units);
	fv_display_set_signal(content.sig_strength);
	// add-ons to use more of the display features
	fv_display_set_t_num(content.t_reg_num);
	fv_display_set_pr_reg_num(content.pr_reg_num);
	fv_display_set_relay(content.relay);

	// send the output to the display
	fv_i2c_send(DISPLAY_I2C_ADDR, (DISPLAY_BYTES + 1), u8a_disp_buf);
}// END fv_display_content


/***********************************************************
	@brief	sets the display buffer to show GR + digit.
***********************************************************/
static void		fv_display_set_g_reg_num(uint8_t g_reg_num)
{
	uint8_t		byte_index;

	if (g_reg_num == 0)
	{
		//do nothing
	}
	else
	{
		byte_index					= (DISPLAY_GR_LOGO_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= DISPLAY_GR_LOGO_MASK;

		byte_index					= (DISPLAY_GR_DIGIT_SEG / 2) + 1;

		switch (g_reg_num)
		{
			// 0 is off (as noted above)
		// case	0:
		// 	u8a_disp_buf[byte_index]	|= 0xBE; 
		// 	break;

		case	1:
			u8a_disp_buf[byte_index]	|= 0x06;
			break;

		case	2:
			u8a_disp_buf[byte_index]	|= 0x7C;
			break;

		case	3:
			u8a_disp_buf[byte_index]	|= 0x5E;
			break;

		case	4:
			u8a_disp_buf[byte_index]	|= 0xC6;
			break;

		case	5:
			u8a_disp_buf[byte_index]	|= 0xDA;
			break;

		case	6:
			u8a_disp_buf[byte_index]	|= 0xFA;
			break;

		case	7:
			u8a_disp_buf[byte_index]	|= 0x0E;
			break;

		case	8:
			u8a_disp_buf[byte_index]	|= 0xFE;
			break;

		case	9:
			u8a_disp_buf[byte_index]	|= 0xDE;
			break;

		case	10:
			u8a_disp_buf[byte_index]	|= 0xEE;
			break;

		default:
			u8a_disp_buf[byte_index]	|= 0x58;
			break;
		}
	}
} // end of fv_display_set_g_reg_num -----------------------


/***********************************************************
	@brief	sets the display buffer to show PR + 2 digits.
***********************************************************/
static void		fv_display_set_pr_reg_num(uint8_t pr_reg_num)
{
	uint8_t		byte_index;

	if (pr_reg_num == 0)
	{
		// do nothing
	}
	else
	{
		byte_index					= (DISPLAY_PR_LOGO_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= DISPLAY_PR_LOGO_MASK;

		byte_index					= (DISPLAY_PR_DIGIT_SEG / 2);

		switch (pr_reg_num % 10)
		{
		case	0:
			u8a_disp_buf[byte_index]	|= 0xBE; 
			break;

		case	1:
			u8a_disp_buf[byte_index]	|= 0x06;
			break;

		case	2:
			u8a_disp_buf[byte_index]	|= 0x7C;
			break;

		case	3:
			u8a_disp_buf[byte_index]	|= 0x5E;
			break;

		case	4:
			u8a_disp_buf[byte_index]	|= 0xC6;
			break;

		case	5:
			u8a_disp_buf[byte_index]	|= 0xDA;
			break;

		case	6:
			u8a_disp_buf[byte_index]	|= 0xFA;
			break;

		case	7:
			u8a_disp_buf[byte_index]	|= 0x0E;
			break;

		case	8:
			u8a_disp_buf[byte_index]	|= 0xFE;
			break;

		case	9:
			u8a_disp_buf[byte_index]	|= 0xDE;
			break;

		default:
			u8a_disp_buf[byte_index]	|= 0x58;
			break;
		}
		
		byte_index++;
		switch (pr_reg_num / 10)
		{
			// 0 is off, leading zero suppression
		case	0:
			break;

		case	1:
			u8a_disp_buf[byte_index]	|= 0x06;
			break;

		case	2:
			u8a_disp_buf[byte_index]	|= 0x7C;
			break;

		case	3:
			u8a_disp_buf[byte_index]	|= 0x5E;
			break;

		case	4:
			u8a_disp_buf[byte_index]	|= 0xC6;
			break;

		case	5:
			u8a_disp_buf[byte_index]	|= 0xDA;
			break;

		case	6:
			u8a_disp_buf[byte_index]	|= 0xFA;
			break;

		case	7:
			u8a_disp_buf[byte_index]	|= 0x0E;
			break;

		case	8:
			u8a_disp_buf[byte_index]	|= 0xFE;
			break;

		case	9:
			u8a_disp_buf[byte_index]	|= 0xDE;
			break;

		default:
			u8a_disp_buf[byte_index]	|= 0x58;
			break;
		}
	}
} // end of fv_display_set_pr_reg_num -----------------------


/***********************************************************
	@brief	sets the display buffer to show T + digit.

	NOTE: to show zero, set t_reg_num to 10
***********************************************************/
static void		fv_display_set_t_num(uint8_t t_reg_num)
{
	uint8_t		byte_index;

	if (t_reg_num == 0)
	{
		//do nothing
	}
	else
	{
		byte_index					= (DISPLAY_T_LOGO_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= DISPLAY_T_LOGO_MASK;

		byte_index					= (DISPLAY_T_DIGIT_SEG / 2) + 1;

		switch (t_reg_num)
		{
		case	1:
			u8a_disp_buf[byte_index]	|= 0x06;
			break;

		case	2:
			u8a_disp_buf[byte_index]	|= 0x7C;
			break;

		case	3:
			u8a_disp_buf[byte_index]	|= 0x5E;
			break;

		case	4:
			u8a_disp_buf[byte_index]	|= 0xC6;
			break;

		case	5:
			u8a_disp_buf[byte_index]	|= 0xDA;
			break;

		case	6:
			u8a_disp_buf[byte_index]	|= 0xFA;
			break;

		case	7:
			u8a_disp_buf[byte_index]	|= 0x0E;
			break;

		case	8:
			u8a_disp_buf[byte_index]	|= 0xFE;
			break;

		case	9:
			u8a_disp_buf[byte_index]	|= 0xDE;
			break;

		case	10:		// used to show zero
			// u8a_disp_buf[byte_index]	|= 0xEE;
			u8a_disp_buf[byte_index]	|= 0xBE; 
			break;

		default:
			u8a_disp_buf[byte_index]	|= 0x58;
			break;
		}
	}
} // end of fv_display_set_t_num -----------------------


/***********************************************************
	@brief	turns on the wireless connected logo
***********************************************************/
static void		fv_display_set_connected(uint8_t i_connected)
{
	if (i_connected > 0)
	{
		uint8_t		byte_index;

		byte_index					= (DISPLAY_CONN_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= DISPLAY_CONN_MASK;
	}
} // end of fv_display_set_connected -----------------------


/***********************************************************
	@brief	turns on the battery logo
***********************************************************/
static void		fv_display_set_batt(uint8_t i_battery)
{
	if (i_battery > 0)
	{
		uint8_t		byte_index;

		byte_index					= (DISPLAY_BATT_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= DISPLAY_BATT_MASK;
	}
} // end of fv_display_set_batt ----------------------------


/***********************************************************
	@brief	turns on the exclamation mark
***********************************************************/
static void		fv_display_set_exclam(uint8_t i_exclam)
{
	if (i_exclam > 0)
	{
		uint8_t		byte_index;

		byte_index					= (DISPLAY_EXCL_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= DISPLAY_EXCL_MASK;
	}
} // end of fv_display_set_exclam --------------------------


/***********************************************************
	@brief	turns on the L1, L2, L3 signals 
	@param i_line: the line to indicate, 1,2 3
***********************************************************/
static void		fv_display_set_line(uint8_t i_line)
{
	//ESP_LOGI(DTAG,"indisplay l");
	uint8_t		byte_index;

	switch (i_line)
	{
	case	1:	
		byte_index					= (DISPLAY_LX_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= DISPLAY_L1_MASK;
		break;

	case	2:	
		byte_index					= (DISPLAY_LX_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= DISPLAY_L2_MASK;
		break;

	case	3:	
		byte_index					= (DISPLAY_LX_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= DISPLAY_L3_MASK;
		break;

	default:
		break;
	}
} // end of fv_display_set_line ----------------------------


/***********************************************************
	@brief	turns on the R1, R2 indicators 
	@param i_relay: the relay to indicate, 1,2
***********************************************************/
static void		fv_display_set_relay(uint8_t i_relay)
{
	uint8_t		byte_index;

	switch (i_relay)
	{
	case	1:	
		byte_index					= (DISPLAY_RX_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= DISPLAY_R1_MASK;
		break;

	case	2:	
		byte_index					= (DISPLAY_RX_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= DISPLAY_R2_MASK;
		break;

	default:
		break;
	}
} // end of fv_display_set_relay ---------------------------


/***********************************************************
	@brief	show the locks
	@param i_lock: 0 , 1 ,2 number of locks to show
***********************************************************/
void	fv_display_set_lock(uint8_t i_lock)
{
	uint8_t	byte_index;
	uint8_t i_result;

	// 1 lock
	if (i_lock == 1)
	{
		i_result=DISPLAY_LOCK_1_MASK;
	}
	//2 lock
	else if (i_lock == 2)
	{
		i_result=DISPLAY_LOCK_1_MASK + DISPLAY_LOCK_2_MASK;
	}
	//no lcok
	else
	{
		i_result=DISPLAY_BLANK_MASK;
	}

	//apply to display buffer
	byte_index					= (DISPLAY_LOCK_SEG / 2) + 1;
	u8a_disp_buf[byte_index]	|= i_result;
}// END fv_display_set_lock -------------------------------


/***********************************************************
	@brief	signal  meter (bottom right)
	@param i_signal: 0-5 for showing display bars
***********************************************************/
void	fv_display_set_signal(uint8_t i_signal)
{
	//mask is an array, from 0 -5 turns on bars
	uint8_t		byte_index;
	uint8_t 	i_mask[]		= {0x00,0x10,0x11,0x13,0x17,0x1F};
	uint8_t 	i_result		= i_mask[i_signal];

	//apply to display buffer
	byte_index					= (DISPLAY_SIG_STRENGTH_SEG / 2) + 1;
	u8a_disp_buf[byte_index]	|= i_result;
}// END fv_display_set_signal -------------------------------


/***********************************************************
	@brief	Q arrow power factor arrows (Q)
	@param i_q_arrow: 1 postive UP, -1 negative DOWN, 0 none
***********************************************************/
void	fv_display_set_q_arrow(int8_t i_q_arrow)
{
	uint8_t		byte_index;
	int8_t		i_result;

	//UP
	if (i_q_arrow == 1)
	{
		i_result				= DISPLAY_Q_ARROW_POS;		//postitive
	}
	//DOWN
	else if (i_q_arrow == -1)
	{
		i_result				= DISPLAY_Q_ARROW_NEG;		//negative
	}
	//NONE
	else
	{
		i_result				= DISPLAY_BLANK_MASK;		//blank
	}

	//apply to display buffer
	byte_index					= (DISPLAY_Q_ARROW_SEG / 2) + 1;
	u8a_disp_buf[byte_index]	|= i_result;
}//END fv_display_set_q_arrow -------------------------------


/***********************************************************
	@brief	P arrows, + positive, - negative
	@param i_p_arrow: 1 postive, -1 negative
***********************************************************/
void	fv_display_set_p_arrow(int8_t i_p_arrow)
{
	uint8_t		byte_index;
	int8_t		i_result;

	// + P arrow postitive power, import 
	if (i_p_arrow == 1)
	{	
		byte_index				= (DISPLAY_P_ARROW_POS_SEG / 2) + 1;
		i_result				= DISPLAY_P_ARROW_POS;		
	}
	// - P arrow negative power, export
	else if (i_p_arrow == -1)
	{
		byte_index 				= (DISPLAY_P_ARROW_NEG_SEG / 2) + 1;
		i_result				= DISPLAY_P_ARROW_NEG;		
	}
	// no arrows
	else
	{
		byte_index 				= (DISPLAY_P_ARROW_NEG_SEG / 2) + 1;
		byte_index				+= (DISPLAY_P_ARROW_POS_SEG / 2) + 1;
		i_result				= DISPLAY_BLANK_MASK;
	}

	//apply to display buffer
	u8a_disp_buf[byte_index]	|= i_result;
}// END fv_display_set_p_arrow ----------------------------


/***********************************************************
	@brief	units to display after main 8 digit display
	@param i_units: units to display
***********************************************************/
static void 	fv_display_set_units(uint8_t i_units)
{
	uint8_t		byte_index;

	//uses defined names for units to display
	switch (i_units)
	{
	case	DISPLAY_UNIT_HZ:	
		byte_index					= (DISPLAY_UNIT_HZ_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= 0x08;
		break;

	case	DISPLAY_UNIT_KW:	
		byte_index					= (DISPLAY_UNIT_KW_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= 0x68;
		break;

	case	DISPLAY_UNIT_KWH:	
		byte_index					= (DISPLAY_UNIT_KWH_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= 0x68;
		u8a_disp_buf[byte_index+1]	|= 0x04;
		break;

	case	DISPLAY_UNIT_MWH:	
		byte_index					= (DISPLAY_UNIT_MWH_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= 0xE0;
		u8a_disp_buf[byte_index+1]	|= 0x04;
		break;

	case	DISPLAY_UNIT_VAR:	
		byte_index					= (DISPLAY_UNIT_VAR_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= 0x20;
		u8a_disp_buf[byte_index+1]	|= 0x03;
		break;

	case	DISPLAY_UNIT_KVAR:	
		byte_index					= (DISPLAY_UNIT_KVAR_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= 0x28;
		u8a_disp_buf[byte_index+1]	|= 0x03;
		break;

	case	DISPLAY_UNIT_KVARH:	
		byte_index					= (DISPLAY_UNIT_KVARH_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= 0x28;
		u8a_disp_buf[byte_index+1]	|= 0x07;
		break;

	case	DISPLAY_UNIT_MVARH:	
		byte_index					= (DISPLAY_UNIT_MVARH_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= 0xB0;
		u8a_disp_buf[byte_index+1]	|= 0x07;
		break;

	case	DISPLAY_UNIT_V:	
		byte_index					= (DISPLAY_UNIT_V_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= 0x20;
		break;

	case	DISPLAY_UNIT_A:	
		byte_index					= (DISPLAY_UNIT_A_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= 0x01;
		break;

	default:
		break;
	}
}// END fv_display_set_units -------------------------------


/***********************************************************
	@brief	number to display in the large 8 digit section
	@param number: units to display
***********************************************************/
static void 	fv_display_set_large_num(double number, uint8_t max_dp)
{
	uint8_t		byte_index;

	// maps for matching digits
	// lookup digit in this to get pointer to position
	char c_pointers[] 		= "0123456789AbcdEFgGhIijLnoprStuy -.X";
	// for right 4 digits (actual digit mapping crosses 2 bytes)
	uint8_t i_result4321a[]	= {0xD0,0x00,0xE0,0xA0,0x30,0xB0,0xF0,0x00,0xF0,0xB0,0x70,0xF0,0xE0,0xE0,0xF0,0x70,0xB0,0xF0,0x70,0x00,0x00,0x80,0xD0,0x60,0xE0,0x70,0x60,0xB0,0xF0,0xC0,0xB0,0x00,0x20,0xA0,0x00};
	uint8_t i_result4321b[]	= {0x07,0x06,0x03,0x07,0x06,0x05,0x05,0x07,0x07,0x07,0x07,0x04,0x00,0x06,0x01,0x01,0x07,0x05,0x04,0x06,0x04,0x06,0x00,0x04,0x04,0x03,0x00,0x05,0x00,0x04,0x06,0x00,0x00,0x01,0x00};
	// for left 4 digits, 
	uint8_t i_result8765[]	= {0xD7,0x06,0xE3,0xA7,0x36,0xB5,0xF5,0x07,0xF7,0xB7,0x77,0xF4,0xE0,0xE6,0xF1,0x71,0xB7,0xF5,0x74,0x06,0x04,0x86,0xD0,0x64,0xE4,0x73,0x60,0xB5,0xF0,0xC4,0xB6,0x00,0x20,0x08,0xA1,0x00};
	//   					  	0	  1	   2	3	 4	  5	   6	7	 8	  9	   A	b	 c	  d	   E	F	 ...

	// if number outside range of screen - tripple line error
	if (number > 99999999 || number < -9999999)
	{
		// triple line error entire display
		byte_index					= (DISPLAY_L_DIGIT_8_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= i_result8765[34];
		byte_index					= (DISPLAY_L_DIGIT_7_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= i_result8765[34];		
		byte_index					= (DISPLAY_L_DIGIT_6_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= i_result8765[34];		
		byte_index					= (DISPLAY_L_DIGIT_5_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= i_result8765[34];		
		byte_index					= (DISPLAY_L_DIGIT_4_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= i_result4321a[33];
		u8a_disp_buf[byte_index+1]	|= i_result4321b[33];
		byte_index					= (DISPLAY_L_DIGIT_3_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= i_result4321a[33];
		u8a_disp_buf[byte_index+1]	|= i_result4321b[33];
		byte_index					= (DISPLAY_L_DIGIT_2_SEG / 2) + 1;
		u8a_disp_buf[byte_index]	|= i_result4321a[33];
		u8a_disp_buf[byte_index+1]	|= i_result4321b[33];
		byte_index					= (DISPLAY_L_DIGIT_1_SEG / 2) + 1;	
		u8a_disp_buf[byte_index]	|= i_result4321a[33];
		u8a_disp_buf[byte_index+1]	|= i_result4321b[33];
	}
	else	//number within range
	{
		uint8_t valuelen			= 0;
		uint8_t dpplace				= 0;
		char value[10];

		// convert the number to a string
		if (max_dp > 2)
		{
			snprintf(value, 10, "%.3f", number);	// 10 decimal and null
		}
		else if (max_dp == 2)
		{
			snprintf(value, 10, "%.2f", number);	// 10 decimal and null
		}
		else if (max_dp == 1)
		{
			snprintf(value, 10, "%.1f", number);	// 10 decimal and null
		}
		else
		{
			snprintf(value, 10, "%.0f", number);	// 10 decimal and null
		}

		//loop through to check how many characters in total, and where the dicimal place goes
		for (uint8_t i = 0; i < 10; i++)
		{		
			char *position_ptr 		= strchr(c_pointers, value[i]);
			int8_t position 		= (position_ptr == NULL ? -1 : position_ptr - c_pointers);

			// valid character
			if ( (position >= 0) && (position < 33) )
			{
				valuelen++;
			}
			//33 is decimal so we just count number of dp to turn on sepearte bit later
			else if (position == 33)
			{
				dpplace				= i;
			//if positiion is 35 then reached end of string, exit
			}
			else if (position == 35)
			{
				break;
			}
		}
		// we need to check where to display the dp in regards to the digits
		dpplace						= valuelen - dpplace + 1;
		valuelen					= valuelen + 1;

		// we count backwards (right justify) numbers
		uint8_t char_position		= valuelen;

		//loop through the input text, matching address bytes to chars
		for (uint8_t i = 0; i < valuelen; i++)
		{		
			char *position_ptr 		= strchr(c_pointers, value[i]);
			int8_t position 		= (position_ptr == NULL ? -1 : position_ptr - c_pointers);
			
			//33 is decimal so we don't do an incriment on this and we just ignor it
			if (position == 33)
			{
				//do nothing
			}
			else
			{
				char_position--;

				// handle characters positions 5-8.  the 8 digit display is in two section, 1-4, and 5-8, each treated differently
				if (char_position > 4)
				{
					// find position and alocate index
					switch (char_position)
					{
					case	8:
						byte_index	= (DISPLAY_L_DIGIT_8_SEG / 2) + 1;
						break;					

					case	7:
						byte_index	= (DISPLAY_L_DIGIT_7_SEG / 2) + 1;
						break;				

					case	6:
						byte_index	= (DISPLAY_L_DIGIT_6_SEG / 2) + 1;
						break;				

					case	5:
						byte_index	= (DISPLAY_L_DIGIT_5_SEG / 2) + 1;
						break;				

					default:
						byte_index = 0;		
						break;			
					}

					//write the hex value to the position
					if (byte_index > 0)
					{
						u8a_disp_buf[byte_index]	|= i_result8765[position];
					}
				}
				// handle characters positions 1-4.  the 8 digit display is in two section, 1-4, and 5-8, each treated differently
				else
				{
					//printf("4321 :%d - character:%02X %02X\n", char_position,i_result4321a[position],i_result4321b[position]);
					
					// find position and alocate index
					switch (char_position)
					{
					case	4:
						byte_index	= (DISPLAY_L_DIGIT_4_SEG / 2) + 1;
						break;

					case	3:
						byte_index	= (DISPLAY_L_DIGIT_3_SEG / 2) + 1;
						break;

					case	2:
						byte_index	= (DISPLAY_L_DIGIT_2_SEG / 2) + 1;
						break;

					case	1:
						byte_index	= (DISPLAY_L_DIGIT_1_SEG / 2) + 1;				
						break;

					default:
						byte_index 	= 0;	
						break;	
					}

					//write the hex value to the position
					if (byte_index > 0)
					{
						u8a_disp_buf[byte_index]	|= i_result4321a[position];
						u8a_disp_buf[byte_index+1]	|= i_result4321b[position];
					}
				}
			}
		}

		//turn on the dp @ dpplace
		if (dpplace > 1)
		{
			// find position and alocate index
			switch (dpplace)
			{
			case	4:
				byte_index	= (DISPLAY_L_DIGIT_4_SEG / 2) + 1;
				break;

			case	3:
				byte_index	= (DISPLAY_L_DIGIT_3_SEG / 2) + 1;
				break;

			case	2:
				byte_index	= (DISPLAY_L_DIGIT_2_SEG / 2) + 1;
				break;

			default:
				byte_index 	= 0;	
				break;	
			}
			
			//write the hex value to the position
			if (byte_index > 0)
			{
				u8a_disp_buf[byte_index + 1]	|= 0x08;
			}
		}
	}
}// END fv_display_set_large_num ---------------------------


/***********************************************************
	@brief	number to display in the large 8 digit section
	@param number: units to display
***********************************************************/
static void 	fv_display_set_large_text(char c_text[])
{
	uint8_t i_inputlen;
	uint8_t i_char_location		= 8;
	uint8_t	byte_index;

	// check size of passed in, only do first 5
	if (strlen(c_text) > 8)
	{
		i_inputlen				= 8;
	}else{
		i_inputlen 				= strlen(c_text);
	}

	// maps for matching digits
	// lookup digit in this to get pointer to position
	char c_pointers[] 			= "0123456789AbcdEFgGhIijLnoprStuy -.X";
	// for right 4 digits (actual digit mapping crosses 2 bytes)
	uint8_t i_result4321a[]		= {0xD0,0x00,0xE0,0xA0,0x30,0xB0,0xF0,0x00,0xF0,0xB0,0x70,0xF0,0xE0,0xE0,0xF0,0x70,0xB0,0xF0,0x70,0x00,0x00,0x80,0xD0,0x60,0xE0,0x70,0x60,0xB0,0xF0,0xC0,0xB0,0x00,0x20,0xA0,0x00};
	uint8_t i_result4321b[]		= {0x07,0x06,0x03,0x07,0x06,0x05,0x05,0x07,0x07,0x07,0x07,0x04,0x00,0x06,0x01,0x01,0x07,0x05,0x04,0x06,0x04,0x06,0x00,0x04,0x04,0x03,0x00,0x05,0x00,0x04,0x06,0x00,0x00,0x01,0x00};
	// for left 4 digits, 
	uint8_t i_result8765[]		= {0xD7,0x06,0xE3,0xA7,0x36,0xB5,0xF5,0x07,0xF7,0xB7,0x77,0xF4,0xE0,0xE6,0xF1,0x71,0xB7,0xF5,0x74,0x06,0x04,0x86,0xD0,0x64,0xE4,0x73,0x60,0xB5,0xF0,0xC4,0xB6,0x00,0x20,0x08,0xA1,0x00};
	//   					  		0	  1	   2	3	 4	  5	   6	7	 8	  9	   A	b	 c	  d	   E	F	 ...

	//loop through the input text, matching address bytes to chars
	for (uint8_t i = 0; i < i_inputlen; i++)
	{	
		char *position_ptr 		= strchr(c_pointers, c_text[i]);
		int8_t position 		= (position_ptr == NULL ? -1 : position_ptr - c_pointers);
		
		//printf("position:%d",position);
		// handle characters positions 5-8.  the 8 digit display is in two section, 1-4, and 5-8, each treated differently
		if (i_char_location > 4)
		{
			//printf("8765 :%d - character:%02X\n", i_char_location,i_result8765[position]);

			switch (i_char_location)
			{
			case	8:
				byte_index	= (DISPLAY_L_DIGIT_8_SEG / 2) + 1;
				break;

			case	7:
				byte_index	= (DISPLAY_L_DIGIT_7_SEG / 2) + 1;
				break;

			case	6:
				byte_index	= (DISPLAY_L_DIGIT_6_SEG / 2) + 1;
				break;

			case	5:
				byte_index	= (DISPLAY_L_DIGIT_5_SEG / 2) + 1;
				break;

			default:
				byte_index 	= 0;		
				break;			
			}

			//charcter not found, show if invalid char
			if (position < 0)
			{
				position	= 34;
			}

			//write the hex value to the posiotion 
			if (byte_index > 0)
			{
				u8a_disp_buf[byte_index]	|= i_result8765[position];
			}
		}
		// handle characters positions 1-4.  the 8 digit display is in two section, 1-4, and 5-8, each treated differently
		else
		{
			//printf("4321 :%d - character:%02X %02X\n", i_char_location,i_result4321a[position],i_result4321b[position]);

			switch (i_char_location)
			{
			case	4:
				byte_index	= (DISPLAY_L_DIGIT_4_SEG / 2) + 1;
				break;

			case	3:
				byte_index	= (DISPLAY_L_DIGIT_3_SEG / 2) + 1;
				break;

			case	2:
				byte_index	= (DISPLAY_L_DIGIT_2_SEG / 2) + 1;
				break;

			case	1:
				byte_index	= (DISPLAY_L_DIGIT_1_SEG / 2) + 1;				
				break;

			default:
				byte_index 	= 0;	
				break;	
			}

			//charcter not found, show if invalid char
			if (position < 0)
			{
				position	= 33;
			}
			
			//write the hex value to the posiotion
			if (byte_index > 0)
			{
				u8a_disp_buf[byte_index]	|= i_result4321a[position];
				u8a_disp_buf[byte_index+1]	|= i_result4321b[position];
			}
		}
		// count down
		i_char_location--;
	}
}// END fv_display_set_large_text --------------------------


/***********************************************************
	@brief	text output to the small 5 digit. (top left)
	@param c_text[]: text to display
***********************************************************/
void 	fv_display_set_small(char c_text[])
{
	uint8_t i_inputlen;
	uint8_t	byte_index;
	uint8_t	disp_char			= 1;			// the output char of the display

	//all characters ()
	// char c_pointers[] 		= "0123456789AbcdEFgGhIijLnoprStuy -";
	char c_pointers[] 		= "0123456789AbcdEFgGhIijLnoprStuy -.";
	//mapped bits, one extra, three bars, 33, return 33 if an invalid char
	// uint8_t i_result[]		= {0xBE,0x06,0x7C,0x5E,0xC6,0xDA,0xFA,0x0E,0xFE,0xDE,0xEE,0xF2,0x70,0x76,0xF8,0xE8,0xDE,0xFA,0xE2,0x06,0x02,0x16,0xB0,0x62,0x72,0xEC,0x60,0xDA,0xF0,0x32,0xD6,0x00,0x40,0x58};
	uint8_t i_result[]		= {0xBE,0x06,0x7C,0x5E,0xC6,0xDA,0xFA,0x0E,0xFE,0xDE,0xEE,0xF2,0x70,0x76,0xF8,0xE8,0xDE,0xFA,0xE2,0x06,0x02,0x16,0xB0,0x62,0x72,0xEC,0x60,0xDA,0xF0,0x32,0xD6,0x00,0x40,0x10,0x58};
	//   				 		0	  1	   2	3	 4	  5	   6	7	 8	  9	   A	b	 c	  d	   E	F	 g	  G    h    I    i    j    L    n    o    p    r    S    t    u    y         -    .
	// check size of passed in, only do first 5
	if (strlen(c_text) > 5)
	{
		i_inputlen			= 5;
	}
	else
	{
		i_inputlen 			= strlen(c_text);
	}
	
	//loop through the input text, matching address bytes to chars
	for (uint8_t i = 0; i < i_inputlen; i++)
	{
		char *position_ptr 	= strchr(c_pointers, c_text[i]);
		int8_t position 	= (position_ptr == NULL ? -1 : position_ptr - c_pointers);

		// 33 is ---, show if invalid char
		if (position < 0)
		{
			// position		= 33;
			position		= 34;
		}

		// check what position
		// switch (i + 1)
		switch (disp_char)
		{
		case	1:
			byte_index	= (DISPLAY_S_DIGIT_1_SEG / 2) + 1;
			break;

		case	2:
			byte_index	= (DISPLAY_S_DIGIT_2_SEG / 2) + 1;
			break;

		case	3:
			byte_index	= (DISPLAY_S_DIGIT_3_SEG / 2) + 1;
			break;

		case	4:
			byte_index	= (DISPLAY_S_DIGIT_4_SEG / 2) + 1;
			break;

		case	5:
			byte_index	= (DISPLAY_S_DIGIT_5_SEG / 2) + 1;
			break;

		default:
			byte_index 	= 0;		
			break;			
		}

		// update the hex value to the main buffer
		if (byte_index > 0)
		{
			// test code for decimal point
			if (c_text[i + 1] == '.')
			{
				// turn on dp
				u8a_disp_buf[byte_index]	|= 0x01;
				++i;					// skip encoding the .
			}

			++disp_char;

			u8a_disp_buf[byte_index]	|= i_result[position];
		}
	}
} // end of fv_display_set_small ---------------------------






