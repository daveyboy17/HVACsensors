/***********************************************************
 * display.h
 *
 *  Created on: 30/06/2021
 *      Author: Dave, completed by Trent
***********************************************************/


#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_


// SEE bottom of file for mapping of characters
// SMS1706 display using HT16C22 driver
// #define DISPLAY_I2C_ADDR		    (0x3F << 1)
#define DISPLAY_I2C_ADDR		    (0x3F)
#define DISPLAY_MODE_ON			    (0x8C)	    // 80Hz, display On, 1/3bias
#define DISPLAY_MODE_OFF		    (0x80)	    // 80Hz, display On, 1/3bias
#define DISPLAY_BLINK_OFF		    (0xC0)
#define DISPLAY_BLINK_2HZ		    (0xC1)	    // 2Hz
#define DISPLAY_BLINK_1HZ		    (0xC2)	    // 1Hz
#define DISPLAY_BLINK_05HZ		    (0xC3)	    // 0.5Hz
#define DISPLAY_VOLT_ADJ		    (0x70)      // Sets contrast voltage
#define DISPLAY_BYTES			    (21)	    // max byte address is 0x15 but only 40 SEGS are used


// large display 87654321 (in that order on the display)
#define DISPLAY_L_DIGIT_1_SEG		(9)         // SEG 9 10 (add second byte)
#define DISPLAY_L_DIGIT_2_SEG		(7)	        // SEG 7 8  (add second byte)
#define DISPLAY_L_DIGIT_3_SEG		(5)	        // SEG 5 6  (add second byte)
#define DISPLAY_L_DIGIT_4_SEG		(3)	        // SEG 3 4  (add second byte)
#define DISPLAY_L_DIGIT_5_SEG		(32)	    // SEG 32 33
#define DISPLAY_L_DIGIT_6_SEG		(34)	    // SEG 34 35
#define DISPLAY_L_DIGIT_7_SEG		(36)	    // SEG 36 37
#define DISPLAY_L_DIGIT_8_SEG		(38)	    // SEG 38 39

// small display 12345 (in that order on the display)
#define DISPLAY_S_DIGIT_1_SEG		(30)	    // SEG30 (left most digit)
#define DISPLAY_S_DIGIT_2_SEG		(28)	    // SEG28
#define DISPLAY_S_DIGIT_3_SEG		(26)	    // SEG26
#define DISPLAY_S_DIGIT_4_SEG		(24)	    // SEG24
#define DISPLAY_S_DIGIT_5_SEG		(22)	    // SEG22 (right most digit)

// signal indicator - lower right
#define DISPLAY_SIG_STRENGTH_SEG	(0)         // SEG0,SEG1
#define DISPLAY_SIG_STRENGTH_SIZE	(5)		    // SEG0 COM0-3, SEG1 COM0
#define DISPLAY_SIG_STRENGTH_ADDR	(0x00)      // SEG0,SEG1

// GR and digit - top middle
#define DISPLAY_GR_DIGIT_SEG	    (20)		// SEG20,21
#define DISPLAY_GR_LOGO_SEG		    (22)	    // SEG22, COM0
#define DISPLAY_GR_LOGO_MASK	    (0x01)	    // SEG22, COM0

// SEGS 15 - 19 may be for T and PR digits
// T and digit - top right
#define DISPLAY_T_DIGIT_SEG	        (15)		// SEG15,16
#define DISPLAY_T_LOGO_SEG		    (17)	    // SEG17, COM0
#define DISPLAY_T_LOGO_MASK	        (0x01)	    // SEG17, COM0
// PR and digits - top middle right
#define DISPLAY_PR_DIGIT_SEG	    (18)		// SEG18,19
#define DISPLAY_PR_LOGO_SEG		    (20)	    // SEG20, COM0
#define DISPLAY_PR_LOGO_MASK	    (0x01)	    // SEG20, COM0

// Antenna logo - top right
#define DISPLAY_CONN_SEG		    (13)        // SEG 13
#define DISPLAY_CONN_MASK		    (0x80)	    // SEG 13 COM3 odd nibble

// Arrows - right side
#define DISPLAY_Q_ARROW_SEG	        (13)        // SEG 13
#define DISPLAY_Q_ARROW_POS	        (0x40)	    // SEG13 COM3 odd nibble
#define DISPLAY_Q_ARROW_NEG	        (0x10)	    // SEG13 COM3 odd nibble
#define DISPLAY_P_ARROW_POS_SEG	    (13)        // SEG13 
#define DISPLAY_P_ARROW_POS	        (0x20)      // COM1 odd nibble
#define DISPLAY_P_ARROW_NEG_SEG	    (14)        // SEG14 
#define DISPLAY_P_ARROW_NEG	        (0x01)	    // COM0 even nibble

// LX indicator - bottom middle
#define DISPLAY_LX_SEG		        (1)         // SEG1
#define DISPLAY_L1_MASK	            (0x80)	    // COM3 odd nibble
#define DISPLAY_L2_MASK	            (0x40)	    // COM3 odd nibble
#define DISPLAY_L3_MASK	            (0x20)	    // COM3 odd nibble

// RX indicator - bottom middle
#define DISPLAY_RX_SEG		        (2)         // SEG1
#define DISPLAY_R1_MASK	            (0x04)	    // COM3 even nibble
#define DISPLAY_R2_MASK	            (0x08)	    // COM3 even nibble

// P indicator - bottom middle
#define DISPLAY_P_SEG		        (2)         // SEG2
#define DISPLAY_P_MASK	            (0x01)	    // COM3 even nibble

// N indicator - bottom middle
#define DISPLAY_N_SEG		        (2)         // SEG2
#define DISPLAY_N_MASK	            (0x02)	    // COM3 even nibble

// Battery, ! and locks - bottom left
#define DISPLAY_BATT_SEG		    (38)        // SEG 38
#define DISPLAY_BATT_MASK		    (0x08)	    // COM3 even nibble
#define DISPLAY_EXCL_SEG		    (40)        // SEG 40
#define DISPLAY_EXCL_MASK		    (0x08)	    // COM3 even nibble
#define DISPLAY_LOCK_SEG		    (40)        // SEG 40
#define DISPLAY_LOCK_1_MASK		    (0x04)	    // COM3 even nibble
#define DISPLAY_LOCK_2_MASK		    (0x02)	    // COM3 even nibble

// Units - right side
#define DISPLAY_UNIT_HZ_SEG		    (12)        // SEG 12
#define DISPLAY_UNIT_KW_SEG         (10)        // SEG 10
#define DISPLAY_UNIT_KWH_SEG        (10)        // SEG 10
#define DISPLAY_UNIT_MWH_SEG        (11)        // SEG 11
#define DISPLAY_UNIT_VAR_SEG        (10)        // SEG 10
#define DISPLAY_UNIT_KVAR_SEG       (10)        // SEG 10
#define DISPLAY_UNIT_KVARH_SEG      (10)        // SEG 10
#define DISPLAY_UNIT_MVARH_SEG      (11)        // SEG 11
#define DISPLAY_UNIT_V_SEG          (11)        // SEG 11
#define DISPLAY_UNIT_A_SEG          (12)        // SEG 12

// BLANK a segment
#define DISPLAY_BLANK_MASK		    (0x00)	    // All OFF



// enums declarations
enum signal_strength{DISPLAY_SIGNAL_0,DISPLAY_SIGNAL_1,DISPLAY_SIGNAL_2,DISPLAY_SIGNAL_3, DISPLAY_SIGNAL_4, DISPLAY_SIGNAL_5 };
enum locks{DISPLAY_LOCKS_0,DISPLAY_LOCKS_1,DISPLAY_LOCKS_2};
enum arrow{DISPLAY_ARROW_NEGATIVE=-1,DISPLAY_ARROW_NONE,DISPLAY_ARROW_POSITIVE};
enum symbol{DISPLAY_SYMBOL_OFF,DISPLAY_SYMBOL_ON};
enum unit{DISPLAY_UNIT_OFF,DISPLAY_UNIT_HZ,DISPLAY_UNIT_KW,DISPLAY_UNIT_KWH,DISPLAY_UNIT_MWH,DISPLAY_UNIT_VAR,DISPLAY_UNIT_KVAR,DISPLAY_UNIT_KVARH,DISPLAY_UNIT_MVARH,DISPLAY_UNIT_V,DISPLAY_UNIT_A};
enum line_number{DISPLAY_LINE_0,DISPLAY_LINE_1,DISPLAY_LINE_2,DISPLAY_LINE_3};
enum mode{DISPLAY_LARGE_NUMBER,DISPLAY_LARGE_TEXT};

// Struct declarations
struct display
{
    uint8_t g_reg_num;
    uint8_t t_reg_num;
    uint8_t pr_reg_num;
    enum line_number line;
    enum line_number relay;
    enum symbol battery;
    enum symbol exclamation;
    enum locks lock;
    enum arrow p_arrow;
    enum arrow q_arrow;
    enum unit units;
    enum symbol connected;
    enum signal_strength sig_strength;
    char small_text[6];
    enum mode large_mode;
    double large_number;
    uint8_t large_num_max_dp;
    char large_text[9];
};

// Function declarations
void		fv_display_on(void);                        // inititalised display
void		fv_display_off(void);                       // turns display off
void		fv_display_set_V(uint8_t vlcd);             // contrast control
void		fv_display_blink(uint8_t blink);            // blink freq
void		fv_display_clear(void);                     // clear screen
void		fv_display_all_on(void);                    // all screen content on
void		fv_display_content(struct display content); // control content on screen


#endif /* INC_DISPLAY_H_ */



/**
 * 			D7..D4	D3..D0
 * 			COM3-0	COM3-0
 *	addr	SEG		SEG
 *	0		1		0
 *	1		3		2
 *	...
 *	21		43		42
 *
 *	even SEGs in the low nibble
 *
 *	digits > 10 are connected differently
 *	digits 1 - 9	A B C -		F G E D
 *	digits 10 - 13	- C B A		D E G F
 *	digits 14 - 17				D E G F		- C B A
 *
 *
 *					A
 *				F		B
 *					G
 *				E		C
 *					D
 *
 *		0	ABCDEF
 *		1	BC
 *		2	ABDEG
 *		3	ABCDG
 *		4	BCFG
 *		5	ACDFG
 *		6	ACDEFG
 *		7	ABC
 *		8	ABCDEFG
 *		9	ABCDFG
 *		A	ABCEFG
 *		b	CDEFG
 *		c	DEG
 *		d	BCDEG
 *		E	ADEFG
 *		F	AEFG
 *		g	ABCDFG	same as 9
 *		G	ACDEFG
 *		h	CEFG
 *		I	BC		same as 1
 *		i	C
 *		j	BCD
 *		L	DEF
 *		n	CEG
 *		o	CDEG
 *		p	ABEFG
 *		r	EG
 *		S	ACDFG	same as 5
 *		t	DEFG
 *		u	CDE
 *		y	BCDFG
 *		-	G
 *
 */