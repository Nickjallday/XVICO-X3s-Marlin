#include "../../../inc/MarlinConfig.h"

#if HAS_DWIN_LCD
#include <WString.h>
#include <stdio.h>
#include <string.h>


#include "dwin.h"
#include "autotest.h"
#include "../../../core/serial.h"
#include "../../../core/macros.h"

#include "../../../module/motion.h"
#include "../../../module/planner.h"
#include "../../../module/temperature.h"
#include "../../../module/tool_change.h"

#include "../../../gcode/gcode.h"
#include "../../../gcode/queue.h"

#include "../../../sd/cardreader.h"
#include "../../../MarlinCore.h"
#include "../../../libs/buzzer.h"

_stAutotest_t Autotest::testflag;
Autotest autotest;

static float test_temp_hotend_target = 0;
static float test_temp_hotbed_target = 0;
static float test_temp_hotend_celsius = 0;
static float test_temp_hotbed_celsius = 0;
static float test_temp_hotend_first = 0;
static float test_temp_hotbed_first = 0;

static uint8_t swstatus[6]={0,0,0,0,0,0};
static uint8_t old_swstatus[6]={0,0,0,0,0,0};


#define	USED_FONT		font12x24
#define WIDTH  				12
#define HEIGH  				24
#define F_GAP	 				8
#define LSTART	 			5
#define SWWIDTH	 			45
#define TEST_EXTRUDER_AUTO_FAN_TEMPERATURE	40


#define ROW_GAP				(HEIGH+F_GAP)

#define	ID_LINE_TITLE						1
#define	ID_LINE_SD1							2
#define	ID_LINE_SD2							3
#define	ID_LINE_ETEMP						4
#define	ID_LINE_ETEMP_INFO			5
#define	ID_LINE_BTEMP						6
#define	ID_LINE_BTEMP_INFO			7
#define	ID_LINE_FAN							8
#define	ID_LINE_XYMOTOR					9
#define	ID_LINE_ZMOTOR					10
#define	ID_LINE_EXTRUDER				11
#define	ID_LINE_SW							12
#define	ID_LINE_SW_STATE				13
#define	ID_LINE_SW_RESULT				14
#define	ID_LINE_KNOB						15.25

#define	XSTART					0
#define	YPOS(L)					(ROW_GAP*(L-1))
#define	XCENTER(L)			(DWIN_WIDTH/2+L*WIDTH)

#define	DRAW_INT_WHITE_FONT12(a,b,x,y,v) dwinLCD.Draw_IntValue(true, true, 0, USED_FONT, Color_White, a, b, x, y, v)
#define	DRAW_INT_RED_FONT12(a,b,x,y,v) dwinLCD.Draw_IntValue(true, true, 0, USED_FONT, Color_Red, a, b, x, y, v)
#define	DRAW_STRING_FONT12(a,b,x,y,s) dwinLCD.Draw_String(false, false, USED_FONT, a, b, x, y, s)

void Autotest::Check_Rotary(){
	ENCODER_DiffState encoder_diffState = get_encoder_state();

	if (encoder_diffState == ENCODER_DIFF_NO) return;	
	if (encoder_diffState == ENCODER_DIFF_CW && testflag.rotary_counter_rg < 255) testflag.rotary_counter_rg++;
	else if(encoder_diffState == ENCODER_DIFF_CCW && testflag.rotary_counter_rg > 0) testflag.rotary_counter_rg--;
	else if(encoder_diffState == ENCODER_DIFF_ENTER) {
		buzzer.tone(200, 3000);
		testflag.rotary_click_rg++;		
	}	
	Autotest_ShowKnob(testflag.rotary_counter_rg);
}

inline void Autotest::Autotest_ShowKnob(uint8_t rotates){
	dwinLCD.Draw_Rectangle(1, Color_Bg_Blue, LSTART+XCENTER(7), YPOS(ID_LINE_KNOB), LSTART+XCENTER(10), YPOS(ID_LINE_KNOB)+ROW_GAP);	
	if(testflag.rotary_click_rg&0x01)
		DRAW_STRING_FONT12(Color_White, Color_Bg_Blue, LSTART+XCENTER(7), YPOS(ID_LINE_KNOB), F("On"));	
	else
		DRAW_STRING_FONT12(Color_White, Color_Bg_Blue, LSTART+XCENTER(7), YPOS(ID_LINE_KNOB), F("Off"));
	DRAW_INT_WHITE_FONT12(Color_Bg_Blue, 3, LSTART+7*WIDTH, YPOS(ID_LINE_KNOB), rotates);
}

inline void Autotest::AutoTest_ShowTemperature(){
	//hotend temperature
	if(test_temp_hotend_celsius != thermalManager.temp_hotend[0].celsius){
		DRAW_INT_WHITE_FONT12(Color_Bg_Window, 3, XCENTER(0), YPOS(ID_LINE_ETEMP), thermalManager.temp_hotend[0].celsius);
		test_temp_hotend_celsius = thermalManager.temp_hotend[0].celsius;
	}
	if(test_temp_hotend_target != thermalManager.temp_hotend[0].target){
		DRAW_INT_WHITE_FONT12(Color_Bg_Window, 3, XCENTER(4), YPOS(ID_LINE_ETEMP), thermalManager.temp_hotend[0].target);
		test_temp_hotend_target = thermalManager.temp_hotend[0].target;
	}
	//bed temperature
	if(test_temp_hotbed_celsius != thermalManager.temp_bed.celsius){
		DRAW_INT_WHITE_FONT12(Color_Bg_Window, 3, XCENTER(0), YPOS(ID_LINE_BTEMP), thermalManager.temp_bed.celsius);
		test_temp_hotbed_celsius = thermalManager.temp_bed.celsius;
	}
	if(test_temp_hotbed_target != thermalManager.temp_bed.target){
		DRAW_INT_WHITE_FONT12(Color_Bg_Window, 3, XCENTER(4), YPOS(ID_LINE_BTEMP), thermalManager.temp_bed.target);
		test_temp_hotbed_target = thermalManager.temp_bed.target;
	}
	//FAN
	if((thermalManager.temp_hotend[0].celsius <= TEST_EXTRUDER_AUTO_FAN_TEMPERATURE) && (testflag.loops > CHECK_FAN_SPEED)){
		thermalManager.fan_speed[0] = 0;
		if(!testflag.fan_fg){
			testflag.fan_fg = 1;
			dwinLCD.Draw_Rectangle(1, Color_Bg_DeepBlue, 0, YPOS(ID_LINE_FAN), DWIN_WIDTH, YPOS(ID_LINE_FAN)+ROW_GAP);
			DRAW_STRING_FONT12(Color_Bg_Red, Color_Bg_DeepBlue, LSTART, YPOS(ID_LINE_FAN), F("ALL Fan Off"));
		}
	}
}

inline void Autotest::AutoTest_ShowSWStatus(bool bfirst){
	if(bfirst){
		dwinLCD.Draw_Rectangle(1, Color_Bg_Window, 0, YPOS(ID_LINE_SW), DWIN_WIDTH, YPOS(ID_LINE_SW_RESULT)+ROW_GAP);
		DRAW_STRING_FONT12(Color_White, Color_Bg_Window, LSTART, YPOS(ID_LINE_SW), F(" X "));
		DRAW_STRING_FONT12(Color_White, Color_Bg_Window, LSTART+1*SWWIDTH, YPOS(ID_LINE_SW), F(" Y "));
		DRAW_STRING_FONT12(Color_White, Color_Bg_Window, LSTART+2*SWWIDTH, YPOS(ID_LINE_SW), F("Z1 "));
		DRAW_STRING_FONT12(Color_White, Color_Bg_Window, LSTART+3*SWWIDTH, YPOS(ID_LINE_SW), F("Z2 "));
		DRAW_STRING_FONT12(Color_White, Color_Bg_Window, LSTART+4*SWWIDTH, YPOS(ID_LINE_SW), F(" F "));
		DRAW_STRING_FONT12(Color_White, Color_Bg_Window, LSTART+5*SWWIDTH, YPOS(ID_LINE_SW), F(" S "));		
	}
	dwinLCD.Draw_Rectangle(1, Color_Bg_Window, 0, YPOS(ID_LINE_SW_STATE), DWIN_WIDTH,  YPOS(ID_LINE_SW_STATE)+ROW_GAP);
	//X	
	DRAW_STRING_FONT12((swstatus[0]==0x07)?Color_Green:Color_White, Color_Bg_Window, LSTART, YPOS(ID_LINE_SW_STATE), F_STRING_ONOFF(swstatus[0]==0x07));
	//Y
	DRAW_STRING_FONT12((swstatus[1]==0x07)?Color_Green:Color_White, Color_Bg_Window, LSTART+1*SWWIDTH, YPOS(ID_LINE_SW_STATE), F_STRING_ONOFF(swstatus[1]==0x07));
	//Z
	DRAW_STRING_FONT12((swstatus[2]==0x07)?Color_Green:Color_White, Color_Bg_Window, LSTART+2*SWWIDTH, YPOS(ID_LINE_SW_STATE), F_STRING_ONOFF(swstatus[2]==0x07));		
	//Z2
#if PIN_EXISTS(Z2_MIN)
	DRAW_STRING_FONT12((swstatus[3]==0x07)?Color_Green:Color_White, Color_Bg_Window, LSTART+3*SWWIDTH, YPOS(ID_LINE_SW_STATE), F_STRING_ONOFF(swstatus[3]==0x07));
#endif
	//runout sensor
#if PIN_EXISTS(FIL_RUNOUT)
	DRAW_STRING_FONT12((swstatus[4]==0x07)?Color_Green:Color_White, Color_Bg_Window, LSTART+4*SWWIDTH, YPOS(ID_LINE_SW_STATE), F_STRING_ONOFF(swstatus[4]==0x07));		
#endif
	//Probe
#if PIN_EXISTS(Z_MIN_PROBE)
	DRAW_STRING_FONT12((swstatus[5]==0x07)?Color_Green:Color_White, Color_Bg_Window, LSTART+5*SWWIDTH, YPOS(ID_LINE_SW_STATE), F_STRING_ONOFF(swstatus[5]==0x07));
#endif
}

inline void Autotest::AutoTest_Watch_SW(){
		//X
		swstatus[0] <<= 1;
		if(READ(X_MIN_PIN) != X_MIN_ENDSTOP_INVERTING) swstatus[0] |= 0x01; else swstatus[0] &= 0xfe;		
		//Y
		swstatus[1] <<= 1;
		if(READ(Y_MIN_PIN) != Y_MIN_ENDSTOP_INVERTING) swstatus[1] |= 0x01; else swstatus[1] &= 0xfe;
		//Z
		swstatus[2] <<= 1;
		if(READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING) swstatus[2] |= 0x01; else swstatus[2] &= 0xfe;
		
	#if PIN_EXISTS(Z2_MIN)
		swstatus[3] <<= 1;
		if(READ(Z2_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING) swstatus[3] |= 0x01; else swstatus[3] &= 0xfe;	
	#endif

	#if PIN_EXISTS(FIL_RUNOUT)
		swstatus[4] <<= 1;
		if(READ(FIL_RUNOUT_PIN) == FIL_RUNOUT_STATE) swstatus[4] |= 0x01; else swstatus[4] &= 0xfe;	
	#endif

	#if PIN_EXISTS(Z_MIN_PROBE)
		swstatus[5] <<= 1;
		if(READ(Z_MIN_PROBE_PIN) != Z_MIN_PROBE_ENDSTOP_INVERTING) swstatus[5] |= 0x01; else swstatus[5] &= 0xfe;	
	#endif
	swstatus[0] &= 0x07;
	swstatus[1] &= 0x07;
	swstatus[2] &= 0x07;
	swstatus[3] &= 0x07;
	swstatus[4] &= 0x07;
	swstatus[5] &= 0x07;
}

bool Autotest::DWIN_AutoTesting() {
	static millis_t test_next_rts_update_ms = 0;
	static uint16_t test_counter = 0;
	static	uint16_t test_timer = 0;
	static bool test_dir = 0;
	Check_Rotary();

	const millis_t test_ms = millis();
	if (PENDING(test_ms, test_next_rts_update_ms)) return false;
	test_next_rts_update_ms = test_ms + 10;
	
	if(testflag.loops >= CHECK_HOTBED_TEMP) AutoTest_ShowTemperature();
	
	switch(testflag.loops){
		default:
		case CHECK_START:
			testflag.loops = CHECK_SD;
			break;
			
		case CHECK_SD:		
			if(IS_SD_INSERTED()){
				dwinLCD.Draw_Rectangle(1, Color_Bg_Black, 0, YPOS(ID_LINE_SD1), DWIN_WIDTH, YPOS(ID_LINE_SD2)+ROW_GAP);
				DRAW_STRING_FONT12(Color_Red, Color_Bg_Black, LSTART, YPOS(ID_LINE_SD1), F("SD Card OK!"));
			  DRAW_STRING_FONT12(Color_Red, Color_Bg_Black, LSTART, YPOS(ID_LINE_SD2), F("SD Size(M):"));
				DRAW_INT_RED_FONT12(Color_Bg_Black, 5, (strlen("SD Size(M):")+1)*WIDTH, YPOS(ID_LINE_SD2), CardReader::sd2card.cardSize()/2000);
				thermalManager.temp_hotend[0].target = 60;
				thermalManager.temp_bed.target = 50;
				DRAW_STRING_FONT12(Color_White, Color_Bg_Window, LSTART, YPOS(ID_LINE_ETEMP_INFO), F("Hot end Heating..."));
				DRAW_STRING_FONT12(Color_White, Color_Bg_Window, LSTART, YPOS(ID_LINE_BTEMP_INFO), F("Hot bed Heating..."));
		 		test_timer = 0;
				testflag.loops++;
			}
			else{
				if(test_timer++ >= 50) {
					test_timer = 0;
					test_dir = !test_dir;
					if(test_dir){
						dwinLCD.Draw_Rectangle(1, Color_Bg_Red, 0, YPOS(ID_LINE_SD1), DWIN_WIDTH, YPOS(ID_LINE_SD2)+ROW_GAP);
						DRAW_STRING_FONT12(Color_White, Color_Bg_Red, LSTART, YPOS(ID_LINE_SD1), F("Please insert SD Card!"));
						DRAW_STRING_FONT12(Color_White, Color_Bg_Red, LSTART, YPOS(ID_LINE_SD2), F("Or SD Card error!"));
					}
					else{
						dwinLCD.Draw_Rectangle(1, Color_Bg_Black, 0, YPOS(ID_LINE_SD1), DWIN_WIDTH, YPOS(ID_LINE_SD2)+HEIGH);
						DRAW_STRING_FONT12(Color_Red, Color_Bg_Black, LSTART, YPOS(ID_LINE_SD1), F("Please insert SD Card!"));
						DRAW_STRING_FONT12(Color_Red, Color_Bg_Black, LSTART, YPOS(ID_LINE_SD2), F("Or SD Card error!"));
					}
				}
			}
		break;

		case CHECK_HOTEND_TEMP:
			if(test_timer++ > 1500){
				test_timer = 0;
			  dwinLCD.Draw_Rectangle(1, Color_Bg_Window, 0, YPOS(ID_LINE_ETEMP_INFO), DWIN_WIDTH, YPOS(ID_LINE_ETEMP_INFO)+ROW_GAP);
				if((test_temp_hotend_celsius - test_temp_hotend_first) >= 4) {
					DRAW_STRING_FONT12(Color_Green, Color_Bg_Window, LSTART, YPOS(ID_LINE_ETEMP_INFO), F("Hot end Temp. OK!"));
				}
				else DRAW_STRING_FONT12(Color_Red, Color_Bg_Window, LSTART, YPOS(ID_LINE_ETEMP_INFO), F("Please check Hot end!"));
				testflag.loops++;
			}
			break;

		case CHECK_HOTBED_TEMP:
			if(test_timer++ > 1000){
				test_timer = 0;
			  dwinLCD.Draw_Rectangle(1, Color_Bg_Window, 0, YPOS(ID_LINE_BTEMP_INFO), DWIN_WIDTH, YPOS(ID_LINE_BTEMP_INFO)+ROW_GAP);
				if((test_temp_hotbed_celsius - test_temp_hotbed_first) > 3) {
					thermalManager.temp_bed.target = 0;
					DRAW_STRING_FONT12(Color_Green, Color_Bg_Window, LSTART, YPOS(ID_LINE_BTEMP_INFO), F("Hot bed Temp. OK!"));
				}
				else DRAW_STRING_FONT12(Color_Red, Color_Bg_Window, LSTART, YPOS(ID_LINE_BTEMP_INFO), F("Please check Hot bed!"));
				
				thermalManager.temp_bed.target = 0;
				testflag.fan_fg = 0;
				testflag.loops++;
			}
			break;

		case CHECK_FAN_SPEED:
			thermalManager.fan_speed[0] = 255;
			thermalManager.checkExtruderAutoFans();
			DRAW_STRING_FONT12(Color_Red, Color_Bg_Black, LSTART, YPOS(ID_LINE_FAN), F("ALL Fan On..."));
			
			if(thermalManager.temp_hotend[0].celsius >= TEST_EXTRUDER_AUTO_FAN_TEMPERATURE + 10){
				test_timer = 0;
				test_counter = 0;			
				thermalManager.temp_hotend[0].target = 0;
				testflag.loops++;
			}
			break;

		case CHECK_XY_MOTOR:
			dwinLCD.Draw_Rectangle(1, Color_Bg_DeepBlue, 0, YPOS(ID_LINE_XYMOTOR), DWIN_WIDTH, YPOS(ID_LINE_XYMOTOR)+ROW_GAP);
			DRAW_STRING_FONT12(Color_White, Color_Bg_Window, LSTART, YPOS(ID_LINE_XYMOTOR), F("XY Axis Motor On..."));
			if (!planner.is_full()) {
		  	planner.synchronize();
		  	planner.buffer_line(current_position, MMM_TO_MMS(HOMING_FEEDRATE_XY), active_extruder);
		 	}
			if(test_timer++ >= 100){
				test_timer = 0;
				test_dir = !test_dir;
			  if(test_dir) {
					current_position.x += 10;
					#if DISABLED(COREXY)
					current_position.y += 10;
					#endif
			  }
				else {
					#if DISABLED(COREXY)
					current_position.y -= 10;
					#endif
					current_position.x -= 10;
				}
			
				if(test_counter++ >=5){
					test_counter = 0;
					dwinLCD.Draw_Rectangle(1, Color_Bg_DeepBlue, 0, YPOS(ID_LINE_XYMOTOR), DWIN_WIDTH, YPOS(ID_LINE_XYMOTOR)+ROW_GAP);
					DRAW_STRING_FONT12(Color_White, Color_Bg_DeepBlue, LSTART, YPOS(ID_LINE_XYMOTOR), F("XY Axis Motor Off"));
					testflag.loops++;
				}
			
			}
			break;

		case CHECK_Z_MOTOR:
			dwinLCD.Draw_Rectangle(1, Color_Bg_Window, 0, YPOS(ID_LINE_ZMOTOR), DWIN_WIDTH, YPOS(ID_LINE_ZMOTOR)+ROW_GAP);
			DRAW_STRING_FONT12(Color_Red, Color_Bg_DeepBlue, LSTART, YPOS(ID_LINE_ZMOTOR), F("Z Axis Motor On..."));
			if (!planner.is_full()) {
		  	planner.synchronize();
		  	planner.buffer_line(current_position, MMM_TO_MMS(HOMING_FEEDRATE_XY), active_extruder);
		 	}

			if(test_timer++ >= 100){
				test_timer = 0;
				test_dir = !test_dir;
			  if(test_dir) current_position.z += 3;
				else current_position.z -= 2;
			
				if(test_counter++ >=5){
					test_counter = 0;					
					dwinLCD.Draw_Rectangle(1, Color_Bg_DeepBlue, 0, YPOS(ID_LINE_ZMOTOR), DWIN_WIDTH, YPOS(ID_LINE_ZMOTOR)+ROW_GAP);
					DRAW_STRING_FONT12(Color_White, Color_Bg_DeepBlue, LSTART, YPOS(ID_LINE_ZMOTOR), F("Z Axis Motor Off"));
					testflag.loops++;
				}
			}
			break;

		case CHECK_MOTOR_E1:
			 dwinLCD.Draw_Rectangle(1, Color_Bg_DeepBlue, 0, YPOS(ID_LINE_EXTRUDER), DWIN_WIDTH, YPOS(ID_LINE_EXTRUDER)+ROW_GAP);
			 DRAW_STRING_FONT12(Color_White, Color_Bg_DeepBlue, LSTART, YPOS(ID_LINE_EXTRUDER), F("Extruder1 Motor On..."));
			 if(test_timer++ >= 100){
			 	test_timer = 0;
				test_dir = !test_dir;
			  if(test_dir) {
			 		queue.inject_P("T0\nG92 E0\nG1 E10 F3000");
			  }
				else{
					queue.inject_P("T0\nG92 E0\nG1 E-10 F3000");
				}

				if(test_counter++ >=3){
					test_counter = 0;					
					testflag.loops++;
				}
			 }
			break;

#if (E_STEPPERS > 1)
		case CHECK_MOTOR_E2:
			 dwinLCD.Draw_Rectangle(1, Color_Bg_DeepBlue, 0, YPOS(ID_LINE_EXTRUDER), DWIN_WIDTH, YPOS(ID_LINE_EXTRUDER)+ROW_GAP);
			 DRAW_STRING_FONT12(Color_Yellow, Color_Bg_DeepBlue, LSTART, YPOS(ID_LINE_EXTRUDER), F("Extruder2 Motor On..."));
			 if(test_timer++ >= 100){
			 	test_timer = 0;
				test_dir = !test_dir;
			  if(test_dir) {
			 		queue.inject_P("T1\nG92 E0\nG1 E10 F3000");
			  }
				else{
					queue.inject_P("T1\nG92 E0\nG1 E-10 F3000");
				}

				if(test_counter++ >=3){
					test_counter = 0;					
					testflag.loops++;
				}
			 }
			break;
#endif

#if (E_STEPPERS > 2)
		case CHECK_MOTOR_E3:
			 dwinLCD.Draw_Rectangle(1, Color_Bg_DeepBlue, 0, YPOS(ID_LINE_EXTRUDER), DWIN_WIDTH, YPOS(ID_LINE_EXTRUDER)+ROW_GAP);
			 DRAW_STRING_FONT12(Color_Bg_Red, Color_Bg_DeepBlue, LSTART, YPOS(ID_LINE_EXTRUDER), F("Extruder3 Motor On..."));
			 if(test_timer++ >= 100){
			 	test_timer = 0;
				test_dir = !test_dir;
			  if(test_dir) {
			 		queue.inject_P("T2\nG92 E0\nG1 E10 F3000");
			  }
				else{
					queue.inject_P("T2\nG92 E0\nG1 E-10 F3000");
				}

				if(test_counter++ >=3){
					test_counter = 0;
					testflag.loops++;
				}
			 }
			break;
#endif

#if (E_STEPPERS > 3)
		case CHECK_MOTOR_E4:
			dwinLCD.Draw_Rectangle(1, Color_Bg_DeepBlue, 0, YPOS(ID_LINE_EXTRUDER), DWIN_WIDTH, YPOS(ID_LINE_EXTRUDER)+ROW_GAP);
			DRAW_STRING_FONT12(Color_Green, Color_Bg_DeepBlue, LSTART, YPOS(ID_LINE_EXTRUDER), F("Extruder4 Motor On..."));
			if(test_timer++ >= 100){
				test_timer = 0;
				test_dir = !test_dir;
				if(test_dir) {
					queue.inject_P("T3\nG92 E0\nG1 E10 F3000");
				}
				else{
					queue.inject_P("T3\nG92 E0\nG1 E-10 F3000");
				}

				if(test_counter++ >=3){
					test_counter = 0;				
					dwinLCD.Draw_Rectangle(1, Color_Bg_DeepBlue, 0, YPOS(ID_LINE_EXTRUDER), DWIN_WIDTH, YPOS(ID_LINE_EXTRUDER)+ROW_GAP);
					DRAW_STRING_FONT12(Color_White, Color_Bg_DeepBlue, LSTART, YPOS(ID_LINE_EXTRUDER), F("ALL Extruder Motor Off"));					

					AutoTest_ShowSWStatus(1);
					testflag.Endstops = 0;
				#if !PIN_EXISTS(Z2_MIN)	
					testflag.Endstops |= 1<<3;
				#endif
				#if !PIN_EXISTS(FIL_RUNOUT)
					testflag.Endstops |= 1<<4;
				#endif
				#if !PIN_EXISTS(Z_MIN_PROBE)
					testflag.Endstops |= 1<<5;
				#endif					
					dwinLCD.Draw_Rectangle(1, Color_Bg_Window, 0, YPOS(ID_LINE_SW_RESULT), DWIN_WIDTH, YPOS(ID_LINE_SW_RESULT)+ROW_GAP);
					for(uint8_t i=0; i<6; i++){						
						if((testflag.Endstops & (1<<i)) == 0)
							DRAW_STRING_FONT12(Color_White, Color_Bg_Window, LSTART+WIDTH/2+i*SWWIDTH, YPOS(ID_LINE_SW_RESULT), F("--"));
						else
							DRAW_STRING_FONT12(Color_Green, Color_Bg_Window, LSTART+WIDTH/2+i*SWWIDTH, YPOS(ID_LINE_SW_RESULT), F("OK"));
					}
					test_timer = 0;
					testflag.loops++;
				}
			}
		break;
#endif

		case CHECK_ENDSTOPS:
			AutoTest_Watch_SW();			
		  if(++test_timer >= 12){
			 	test_timer = 0;								
				AutoTest_ShowSWStatus(0);
				for(uint8_t i=0; i<6; i++){
					if((testflag.Endstops & (1<<i)) == 0 && (old_swstatus[i] == 0x0 && swstatus[i] == 0x07)){
						testflag.Endstops |= (1<<i);
						dwinLCD.Draw_Rectangle(1, Color_Bg_Window, LSTART+WIDTH/2+i*SWWIDTH, YPOS(ID_LINE_SW_RESULT), LSTART+WIDTH/2+i*SWWIDTH+SWWIDTH, YPOS(ID_LINE_SW_RESULT)+ROW_GAP);
						DRAW_STRING_FONT12(Color_Green, Color_Bg_Window, LSTART+WIDTH/2+i*SWWIDTH, YPOS(ID_LINE_SW_RESULT), F("OK"));							
					}				
				}				
				if(testflag.Endstops == 0x3f){
					testflag.loops++;
					testflag.rotary_click_rg = 0;
					testflag.rotary_counter_rg = 0;
					test_dir = 0;
					test_timer = 0;
				}
				for(uint8_t i=0; i<6; i++) old_swstatus[i] = swstatus[i];
		  }			
			break;

		case CHECK_KEY:	
			AutoTest_Watch_SW();
			if(test_timer++ >= 12){
				test_timer = 0;
				AutoTest_ShowSWStatus(0);
			}
			if((testflag.rotary_click_rg > 3) && (testflag.rotary_counter_rg > 10)){
				testflag.loops++;					
			}
			break;

		case CHECK_END:		
			testflag.loops = 0;
			return true;
	}		
	return false;
}

void Autotest::HMI_StartTest() {	
	testflag.loops = 0;

	test_temp_hotend_first = thermalManager.temp_hotend[0].target = 0;
	test_temp_hotbed_first = thermalManager.temp_bed.target = 0;	

		
	dwinLCD.Draw_Rectangle(1, Color_Bg_Black, 0, 0, DWIN_WIDTH, DWIN_HEIGHT);
	dwinLCD.Draw_Rectangle(1, Color_Bg_Blue, 0, 0, DWIN_WIDTH, HEIGH);
	DRAW_STRING_FONT12(Color_White, Color_Bg_Blue, XCENTER(-8), 0, F("3D Printer Test"));
	dwinLCD.Draw_Rectangle(1, Color_Bg_Blue, 0, DWIN_HEIGHT-HEIGH, DWIN_WIDTH, DWIN_HEIGHT);
	DRAW_STRING_FONT12(Color_White, Color_Bg_Blue, LSTART, DWIN_HEIGHT-HEIGH, F("Rotary:"));
	DRAW_STRING_FONT12(Color_White, Color_Bg_Blue, LSTART+XCENTER(0), DWIN_HEIGHT-HEIGH, F("Buzzer:"));
	dwinLCD.Draw_Rectangle(1, Color_Bg_Window, 0, YPOS(ID_LINE_ETEMP), DWIN_WIDTH, YPOS(ID_LINE_ETEMP)+HEIGH);
	DRAW_STRING_FONT12(Color_White, Color_Bg_Window, LSTART, YPOS(ID_LINE_ETEMP), F("Hot end:"));
	DRAW_STRING_FONT12(Color_White, Color_Bg_Window, XCENTER(3),  YPOS(ID_LINE_ETEMP), F("/"));
	dwinLCD.Draw_Rectangle(1, Color_Bg_Window, 0, YPOS(ID_LINE_BTEMP), DWIN_WIDTH, YPOS(ID_LINE_BTEMP)+HEIGH);
	DRAW_STRING_FONT12(Color_White, Color_Bg_Window, LSTART,  YPOS(ID_LINE_BTEMP), F("Hot bed:"));
	DRAW_STRING_FONT12(Color_White, Color_Bg_Window, XCENTER(3), YPOS(ID_LINE_BTEMP), F("/"));
}
#endif
