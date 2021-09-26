/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once
#include "../../inc/MarlinConfig.h"

#ifdef HAS_DWIN_LCD
#include "../dwin_lcd.h"
#if ANY(HAS_HOTEND, HAS_HEATED_BED, HAS_FAN) && PREHEAT_COUNT
  #define HAS_PREHEAT 1
  #if PREHEAT_COUNT < 2
    #error "Creality DWIN requires two material preheat presets."
  #endif
#endif
#include "../../../feature/pause.h"

#define F_STRING_ONOFF(V) ((V)?F("ON "):F("OFF"))

/**
 * The font size, 0x00-0x09, corresponds to the font size below:
 * 0x00=6*12   0x01=8*16   0x02=10*20  0x03=12*24  0x04=14*28
 * 0x05=16*32  0x06=20*40  0x07=24*48  0x08=28*56  0x09=32*64
 */
#define font6x12  0x00
#define font8x16  0x01
#define font10x20 0x02
#define font12x24 0x03
#define font14x28 0x04
#define font16x32 0x05
#define font20x40 0x06
#define font24x48 0x07
#define font28x56 0x08
#define font32x64 0x09

// Color of strings
#define Color_White       0xFFFF
#define Color_Yellow      0xFF0F
#define Color_Green       0x07E0
#define Color_Red         0xF00F  // Red background color
#define Color_blue        0x001F  // 
#define Color_Bg_Window   0x31E8  //0x6516  //0x31E8// Popup background color//
#define Color_Bg_Blue     0x1125  //1125// Dark blue background color
#define Color_Bg_DeepBlue 0x000F  //1125// Dark blue background color
#define Color_Bg_Black    0x0000  //841// Black background color      
#define Color_Bg_Red      0xF00F  // Red background color
#define Popup_Text_Color  0xD6BA  // Popup font background color
#define Line_Color        0x3A6A  // Split line color
#define Rectangle_Color   0xEE2F  // Blue square cursor color
#define Percent_Color     0xFE29  // Percentage color
#define BarFill_Color     0x10E4  // Fill color of progress bar
#define Select_Color      0x33BB  // Selected color

//
#define VTOOL_START 				0
#define VTOOL_END   				1
#define ZPOS_START 					0
#define ZPOS_END   					1
#define RANDMIX_HEIGHT   		3
#define RANDMIX_EXTRUDES   	4

//
// Menu Index
//
#define MAIN_CASE_PRINT  			0
#define MAIN_CASE_PREPARE 		1
#define MAIN_CASE_CONTROL  		2
#define MAIN_CASE_INFO 				3
#define	MAIN_CASE_END					4

#define PRINT_CASE_TUNE  			0
#define PRINT_CASE_PAUSE 			1
#define PRINT_CASE_STOP  			2
#define	PRINT_CASE_END				PRINT_CASE_STOP

#define MOTION_CASE_RATE  		1
#define MOTION_CASE_ACCEL 		2
#define MOTION_CASE_JERK  		(MOTION_CASE_ACCEL + ENABLED(HAS_CLASSIC_JERK))
#define MOTION_CASE_STEPS 		(MOTION_CASE_JERK + 1)
#define MOTION_CASE_TOTAL 		MOTION_CASE_STEPS

#define PREPARE_CASE_HOME 			1
#define PREPARE_CASE_TEMP 			2
#define PREPARE_CASE_MOVE 			3
#define PREPARE_CASE_LEVELING 	4
#define PREPARE_CASE_LANG 			5
#define PREPARE_CASE_DISA 			6
#define PREPARE_CASE_POWERDOWN 	7
#define PREPARE_CASE_TOTAL 			PREPARE_CASE_POWERDOWN

#define CONTROL_CASE_MIXER 		1
#define CONTROL_CASE_CONFIG 	2
#define CONTROL_CASE_MOTION  	3
#define CONTROL_CASE_SETPLA  	4
#define CONTROL_CASE_SETABS  	5
#define CONTROL_CASE_BLTOUCH 	(CONTROL_CASE_SETABS + ENABLED(BLTOUCH))
#define CONTROL_CASE_SAVE 		(CONTROL_CASE_BLTOUCH + ENABLED(EEPROM_SETTINGS))
#define CONTROL_CASE_LOAD 		(CONTROL_CASE_SAVE + ENABLED(EEPROM_SETTINGS))
#define CONTROL_CASE_RESET 		(CONTROL_CASE_LOAD + ENABLED(EEPROM_SETTINGS))
#define CONTROL_CASE_INFO 		(CONTROL_CASE_RESET + 1)
#define CONTROL_CASE_TOTAL 		CONTROL_CASE_RESET

#define TUNE_CASE_SPEED 			1
#define TUNE_CASE_ETEMP 			(TUNE_CASE_SPEED + ENABLED(HAS_HOTEND))
#define TUNE_CASE_BTEMP 			(TUNE_CASE_ETEMP + ENABLED(HAS_HEATED_BED))
#define TUNE_CASE_FAN 				(TUNE_CASE_BTEMP + ENABLED(HAS_FAN))
#define TUNE_CASE_ZOFF 				(TUNE_CASE_FAN + ENABLED(BABYSTEPPING))
#define TUNE_CASE_MIXER 			(TUNE_CASE_ZOFF + 1)
#define TUNE_CASE_CONFIG 			(TUNE_CASE_MIXER + 1)
#define TUNE_CASE_TOTAL 			TUNE_CASE_CONFIG

#define TEMP_CASE_ETEMP				(0 + ENABLED(HAS_HOTEND))
#define TEMP_CASE_BTEMP				(TEMP_CASE_ETEMP + ENABLED(HAS_HEATED_BED))
#define TEMP_CASE_FAN 				(TEMP_CASE_BTEMP + ENABLED(HAS_FAN))
#define TEMP_CASE_PREHEATPLA 	(TEMP_CASE_FAN + ENABLED(HAS_HOTEND))
#define TEMP_CASE_PREHEATABS 	(TEMP_CASE_PREHEATPLA + ENABLED(HAS_HOTEND))
#define TEMP_CASE_COOL 				(TEMP_CASE_PREHEATABS + ENABLED(HAS_HOTEND))
#define TEMP_CASE_TOTAL 			TEMP_CASE_COOL

#define PREHEAT_CASE_TEMP 	(0 + ENABLED(HAS_HOTEND))
#define PREHEAT_CASE_BED 		(PREHEAT_CASE_TEMP + ENABLED(HAS_HEATED_BED))
#define PREHEAT_CASE_FAN 		(PREHEAT_CASE_BED + ENABLED(HAS_FAN))
#define PREHEAT_CASE_SAVE 	(PREHEAT_CASE_FAN + ENABLED(EEPROM_SETTINGS))
#define PREHEAT_CASE_TOTAL 	PREHEAT_CASE_SAVE

#define BLTOUCH_CASE_RESET  1
#define BLTOUCH_CASE_TEST 	2
#define BLTOUCH_CASE_STOW  	3
#define BLTOUCH_CASE_DEPLOY 4
#define BLTOUCH_CASE_SW  		5
#define BLTOUCH_CASE_TOTAL 	BLTOUCH_CASE_SW

#define LANGUAGE_CASE_EN  	1
#define LANGUAGE_CASE_SP 		2
#define LANGUAGE_CASE_RU  	3
#define LANGUAGE_CASE_FR  	4
#define LANGUAGE_CASE_PO  	5
#define LANGUAGE_CASE_ZH 		6
#define LANGUAGE_CASE_TOTAL 	LANGUAGE_CASE_PO

#define MIXER_CASE_MANUAL  	1
#define MIXER_CASE_AUTO 		2
#define MIXER_CASE_RANDOM  	3
#define MIXER_CASE_VTOOL  	4
#define MIXER_CASE_TOTAL 		MIXER_CASE_VTOOL

#define CONFIG_CASE_RETRACT  		(ENABLED(FWRETRACT))
#define CONFIG_CASE_FILAMENT 		(CONFIG_CASE_RETRACT + ENABLED(FILAMENT_RUNOUT_SENSOR))
#define CONFIG_CASE_POWERLOSS  	(CONFIG_CASE_FILAMENT + ENABLED(POWER_LOSS_RECOVERY))
#define CONFIG_CASE_SHUTDOWN  	(CONFIG_CASE_POWERLOSS + ENABLED(OPTION_AUTOPOWEROFF))
#define CONFIG_TUNE_CASE_TOTAL 	CONFIG_CASE_SHUTDOWN
#define CONFIG_CASE_WIFI    		(CONFIG_CASE_SHUTDOWN + ENABLED(OPTION_WIFI_MODULE))
#define CONFIG_CASE_REPRINT   	(CONFIG_CASE_WIFI + ENABLED(OPTION_REPEAT_PRINTING))
#define	CONFIG_CASE_COATING			(CONFIG_CASE_REPRINT + ENABLED(OPTION_BED_COATING))
#define CONFIG_CASE_LEVELING  	(CONFIG_CASE_COATING + BOTH(ABL_GRID,AUTO_UPDATA_PROBE_Z_OFFSET))
#define CONFIG_CASE_ACTIVELEVEL (CONFIG_CASE_LEVELING + ENABLED(ABL_GRID))
#define CONFIG_CASE_RGB     		(CONFIG_CASE_ACTIVELEVEL + ENABLED(RGB_LED))
#define CONFIG_CASE_M92     		(CONFIG_CASE_RGB + ENABLED(DEBUG_GCODE_M92))
#define CONFIG_CASE_TOTAL 			CONFIG_CASE_M92

#define RETRACT_CASE_AUTO  				1
#define RETRACT_CASE_RETRACT_MM 	2
#define RETRACT_CASE_RETRACT_V 		3
#define RETRACT_CASE_RETRACT_ZHOP 4
#define RETRACT_CASE_RECOVER_MM 	5
#define RETRACT_CASE_RECOVER_V 		6
#define RETRACT_CASE_TOTAL 				RETRACT_CASE_RECOVER_V

#define REPRINT_CASE_ENABLED  	1
#define REPRINT_CASE_TIMES 			2
#define REPRINT_CASE_LENGHT 		3
#define REPRINT_CASE_RESET 			4
#define REPRINT_CASE_FORWARD 		5
#define REPRINT_CASE_BACK 			6
#define REPRINT_CASE_TOTAL 			REPRINT_CASE_BACK

#if ENABLED(MIXING_EXTRUDER)
	#define MANUAL_CASE_EXTRUDER1  		1
	#define MANUAL_CASE_EXTRUDER2 		2
 	#if(MIXING_STEPPERS > 2) 		
		#define MANUAL_CASE_EXTRUDER3  	3
	#endif
	#if(MIXING_STEPPERS > 3) 		
		#define MANUAL_CASE_EXTRUDER4 	4
	#endif
	#define MANUAL_CASE_OK 		(MIXING_STEPPERS+1)
	#define MANUAL_CASE_TOTAL    MANUAL_CASE_OK
#endif

#define AUTO_CASE_ZPOS_START 	1
#define AUTO_CASE_ZPOS_END 	2
#define AUTO_CASE_VTOOL_START 	3
#define AUTO_CASE_VTOOL_END 	4
#define AUTO_CASE_TOTAL    	AUTO_CASE_VTOOL_END

#define RANDOM_CASE_ZPOS_START 1
#define RANDOM_CASE_ZPOS_END 	2
#define RANDOM_CASE_HEIGHT 		3
#define RANDOM_CASE_EXTRUDERS 	4
#define RANDOM_CASE_TOTAL    RANDOM_CASE_EXTRUDERS


#define AXISMOVE_CASE_MOVEX  	1
#define AXISMOVE_CASE_MOVEY 	2
#define AXISMOVE_CASE_MOVEZ  	3
#if HAS_HOTEND
 #define AXISMOVE_CASE_EX1 		4
	#if(E_STEPPERS > 1) 
 #define AXISMOVE_CASE_EX2 		(E_STEPPERS+1)
	#endif
 #if(E_STEPPERS > 2) 
	#define AXISMOVE_CASE_EX3 		(E_STEPPERS+2)	
 #endif
 #if(E_STEPPERS > 3)
	#define AXISMOVE_CASE_EX4 		(E_STEPPERS+3)
 #endif
	#if ENABLED(MIXING_EXTRUDER)
	#define AXISMOVE_CASE_EXALL 	(E_STEPPERS+4)
	#endif
 #define AXISMOVE_CASE_TOTAL   (E_STEPPERS + 3 + ENABLED(MIXING_EXTRUDER))
#endif

#define LEVELING_CASE_POINT1  					1
#define LEVELING_CASE_POINT2 						2
#define LEVELING_CASE_POINT3  					3
#define LEVELING_CASE_POINT4  					4
#define LEVELING_CASE_CATCHOFFSET  			(LEVELING_CASE_POINT4 + ENABLED(AUTO_UPDATA_PROBE_Z_OFFSET))
#define LEVELING_CASE_PROBEZOFFSET  		(LEVELING_CASE_CATCHOFFSET + ENABLED(LCD_BED_LEVELING))
#define LEVELING_CASE_ACTION  					(LEVELING_CASE_PROBEZOFFSET + ENABLED(LCD_BED_LEVELING))
#define LEVELING_CASE_TOTAL 						LEVELING_CASE_ACTION


#define HOME_CASE_ALL  						1
#define HOME_CASE_X 							2
#define HOME_CASE_Y  							3
#define HOME_CASE_Z  							4
#define HOME_CASE_TOTAL 					HOME_CASE_Z

#define INFO_CASE_VERSION  				1
#define INFO_CASE_FIRMWARE  			2
#define INFO_CASE_WEBSITE  				3
#define INFO_CASE_MODEL   				4
#define INFO_CASE_BOARD   				5
#define INFO_CASE_EXTRUDER_NUM  	6
#define INFO_CASE_EXTRUDER_MODEL  7
#define INFO_CASE_DUALZ_DRIVE   	(INFO_CASE_EXTRUDER_MODEL + ENABLED(OPTION_DUALZ_DRIVE)) 
#define INFO_CASE_DUALZ_ENDSTOP  	(INFO_CASE_DUALZ_DRIVE + ENABLED(OPTION_Z2_ENDSTOP))
#define INFO_CASE_BAUDRATE  	  	(INFO_CASE_DUALZ_ENDSTOP + 1)
#define INFO_CASE_PROTOCOL  	  	(INFO_CASE_BAUDRATE + 1)
#define INFO_CASE_PSU  	    			(INFO_CASE_PROTOCOL + 1)
#define INFO_CASE_DATE  		  		(INFO_CASE_PSU + 1)
#define INFO_CASE_THERMISTOR  	  (INFO_CASE_DATE + 1)
#define INFO_CASE_BED  	    			(INFO_CASE_THERMISTOR + 1)
#define INFO_CASE_HOT  	    			(INFO_CASE_BED + 1)
#define INFO_CASE_TOTAL 	 	  		INFO_CASE_HOT

#define DWIN_SCROLL_UPDATE_INTERVAL 			1000
#define DWIN_REMAIN_TIME_UPDATE_INTERVAL 	10000
#define POWERDOWN_MACHINE_TIMER 					(900*1000/DWIN_SCROLL_UPDATE_INTERVAL)


typedef enum{
	ID_SM_START = 0,
	ID_SM_IDEL,
	ID_SM_PRINTING,	
	ID_SM_PAUSING,				//pause >> wait paused
	ID_SM_PAUSED,
	ID_SM_RESUMING,				//Resuming
	ID_SM_STOPED,					//
	ID_SM_RUNOUTING,
	ID_SM_RUNOUT_DONE
}_emDWINState;

typedef enum {
  // Process ID
  DWMENU_MAIN = 0,
  DWMENU_FILE,
  DWMENU_PREPARE,
  DWMENU_CONTROL,
  DWMENU_INFO,
  
  //Print Menu
  DWMENU_PRINTING = 5,
  DWMENU_TUNE,

	//Prepare menu
  DWMENU_HOME,
  DWMENU_TEMPERATURE,
  DWMENU_PREHEAT_PLA,
	DWMENU_PREHEAT_ABS = 10,
  DWMENU_MOVEAXIS,
  DWMENU_MOVEX,
  DWMENU_MOVEY,
  DWMENU_MOVEZ,
	DWMENU_MOVE_EXT1,
	DWMENU_MOVE_EXT2,
	DWMENU_MOVE_EXT3,
	DWMENU_MOVE_EXT4,
	DWMENU_MOVE_EXTALL,		
	DWMENU_LEVELING = 20,
	DWMENU_POP_LEVEL_CATCH,
	DWMENU_LEVEL_CATCHOFFSET,
	DWMENU_LEVEL_SETOFFSET,
	DWMENU_LEVEL_BEDLEVELING,	
	DWMENU_LEVEL_DONECONFIRM,
		
  //Control menu  
  DWMENU_MIXER = 26,
  DWMENU_CONFIG,
  DWMENU_MOTION,
  DWMENU_POWERDOWN,
	DWMENU_LANGUAGE =30,
	DWMENU_SET_ETMP,
	DWMENU_SET_BTMP,
  DWMENU_SET_FANSPEED,
	
	//Tune
	DWMENU_TUNE_PRINTSPEED,
	DWMENU_TUNE_BABYSTEPS,
  
  //Control>>Motion
  DWMENU_SET_MAXSPEED,
  DWMENU_SET_MAXSPEED_VALUE,
  DWMENU_SET_MAXACC,
  DWMENU_SET_MAXACC_VALUE,
  DWMENU_SET_MAXJERK =40,
  DWMENU_SET_MAXJERK_VALUE,
  DWMENU_SET_STEPPREMM,
  DWMENU_SET_STEPPREMM_VALUE,
  DWMENU_SET_ZOFFSET,

	//Control>>Mixer
	DWMENU_MIXER_MANUAL,
	DWMENU_MIXER_AUTO,
	DWMENU_MIXER_RANDOM,
	DWMENU_MIXER_VTOOL,
	//Control>>Mixer>>manual
	DWMENU_MIXER_EXT1,
	DWMENU_MIXER_EXT2 =50,
	DWMENU_MIXER_EXT3,
	DWMENU_MIXER_EXT4,
	//Control>>Mixer>>Gradient
	DWMENU_MIXER_GRADIENT_ZSTART,
	DWMENU_MIXER_GRADIENT_ZEND,
	DWMENU_MIXER_GRADIENT_TSTAR,
	DWMENU_MIXER_GRADIENT_TEND,	
	//Control>>Mixer>>Random
	DWMENU_MIXER_RANDOM_ZSTART,
	DWMENU_MIXER_RANDOM_ZEND,
	DWMENU_MIXER_RANDOM_HEIGHT,
	DWMENU_MIXER_RANDOM_EXTN,	
	
 	//config
 	DWMENU_SET_BLTOUCH =61,
 	DWMENU_SET_RETRACT,
 	//config>>Retract
 	DWMENU_SET_RETRACT_MM,
 	DWMENU_SET_RETRACT_V,
 	DWMENU_SET_RETRACT_ZHOP,
 	DWMENU_SET_UNRETRACT_MM,
 	DWMENU_SET_UNRETRACT_V,
	//WiFi
	DWMENU_SET_WIFIONOFF,
	//BED Coating
	DWMENU_SET_BEDCOATING,
	//Repeat printing
	DWMENU_SET_REPRINT = 70,
	DWMENU_SET_REPRINT_TIMES,
	DWMENU_SET_REPRINT_RUNLENGTH,

	// Pop Menu
	DWMENU_POP_HOME =73,
	DWMENU_POP_STOPPRINT,
	DWMENU_POP_FROD_OPTION,
	DWMENU_POP_FROD_CONFIRM,
	DWMENU_POP_PAUSEORSTOP,	
	DWMENU_POP_WIFI,
	DWMENU_POP_WAITING,

	DWMENU_END
}_emDWIN_MENUID_;

// Picture ID
#define	IMAGE_CACHE_ID1		1

#define	TITLE_X		14
#define	TITLE_Y		7


//#define DEBUG_FILAMENT_RUNOUT 
//#define DEBUG_POWER_LOSS
//
// logo  offset define
//
#define LOGO_OFFSET_X         		20
#define LOGO_OFFSET_Y         		45

//
// Status Area offset define
//
#define State_space_Y         		20
#define State_icon_offset_X    		13
#define State_icon_offset_Y    		381
#define State_text_offset_X    		State_icon_offset_X + STAT_CHR_W*2
#define State_text_offset_Y    		State_icon_offset_Y + 1
#define State_string_offset_X    	State_icon_offset_X + STAT_CHR_W*5
#define State_string_offset_Y    	State_icon_offset_Y + 2

#define State_text_extruder_num		3
#define State_icon_extruder_X 		State_icon_offset_X
#define State_icon_extruder_Y		State_icon_offset_Y
#define State_text_extruder_X		State_text_offset_X
#define State_text_extruder_Y		State_text_offset_Y
#define State_string_extruder_X		State_string_offset_X
#define State_string_extruder_Y		State_string_offset_Y

#define State_text_bed_num			3
#define State_icon_bed_X 			State_icon_offset_X + DWIN_WIDTH/2
#define State_icon_bed_Y			State_icon_offset_Y
#define State_text_bed_X			State_text_offset_X	+ DWIN_WIDTH/2
#define State_text_bed_Y			State_text_offset_Y
#define State_string_bed_X			State_string_offset_X + DWIN_WIDTH/2
#define State_string_bed_Y			State_string_offset_Y

#define State_text_speed_num		3
#define State_icon_speed_X 			State_icon_offset_X
#define State_icon_speed_Y			State_icon_offset_Y + STAT_CHR_H + State_space_Y
#define State_text_speed_X			State_text_offset_X
#define State_text_speed_Y			State_text_offset_Y + STAT_CHR_H + State_space_Y
#define State_string_speed_X		State_string_offset_X
#define State_string_speed_Y		State_string_offset_Y + STAT_CHR_H + State_space_Y

#define State_text_Zoffset_inum		3
#define State_text_Zoffset_fnum		2
#define State_icon_Zoffset_X 		State_icon_offset_X + DWIN_WIDTH/2
#define State_icon_Zoffset_Y		State_icon_offset_Y + STAT_CHR_H + State_space_Y
#define State_text_Zoffset_X		State_text_offset_X + DWIN_WIDTH/2
#define State_text_Zoffset_Y		State_text_offset_Y + STAT_CHR_H + State_space_Y
#define State_string_Zoffset_X		State_string_offset_X + DWIN_WIDTH/2
#define State_string_Zoffset_Y		State_string_offset_Y + STAT_CHR_H + State_space_Y

#define State_text_vtool_num		2

#define State_text_mix_num			3
#define State_icon_mix_X 			13
#define State_icon_mix_Y			45
#define State_text_mix_X			13
#define STATE_TEXT_MIX_Y			State_icon_mix_Y + 25

//
// Menu Area offset define
//
#define Menu_control_start_temp_X		57
#define Menu_control_start_temp_Y		104
#define Menu_control_end_temp_X			84
#define Menu_control_end_temp_Y			116

#define Menu_control_start_motion_X		Menu_control_start_temp_X + 30
#define Menu_control_start_motion_Y		Menu_control_start_temp_Y
#define Menu_control_end_motion_X		Menu_control_end_temp_X + 30
#define Menu_control_end_motion_Y		Menu_control_end_temp_Y

#define Menu_control_start_mixer_X		Menu_control_start_motion_X + 30
#define Menu_control_start_mixer_Y		Menu_control_start_motion_Y
#define Menu_control_end_mixer_X		Menu_control_end_motion_X + 30
#define Menu_control_end_mixer_Y		Menu_control_end_motion_Y

#define Menu_control_start_store_X		Menu_control_start_mixer_X + 30
#define Menu_control_start_store_Y		Menu_control_start_mixer_Y
#define Menu_control_end_store_X		Menu_control_end_mixer_X + 30
#define Menu_control_end_store_Y		Menu_control_end_mixer_Y

#define Menu_control_start_read_X		Menu_control_start_store_X + 30
#define Menu_control_start_read_Y		Menu_control_start_store_Y
#define Menu_control_end_read_X			Menu_control_end_store_X + 30
#define Menu_control_end_read_Y			Menu_control_end_store_Y

#define Menu_control_start_reset_X		Menu_control_start_read_X + 30
#define Menu_control_start_reset_Y		Menu_control_start_read_Y
#define Menu_control_end_reset_X		Menu_control_end_read_X + 30
#define Menu_control_end_reset_Y		Menu_control_end_read_Y

#define Menu_control_start_info_X		Menu_control_start_reset_X + 30
#define Menu_control_start_info_Y		Menu_control_start_reset_Y
#define Menu_control_end_info_X			Menu_control_end_reset_X + 30
#define Menu_control_end_info_Y			Menu_control_end_reset_Y

extern _emDWIN_MENUID_ DwinMenuID;

#define EXTR_STEPPER_LOOP(VAR) for (uint_fast8_t VAR = 0; VAR < E_STEPPERS; VAR++)


typedef enum {
	G29_LEVLE_DEFAULT = 0,
	G29_CATCH_START,
	G29_CATCH_NORMAL,
	G29_CATCH_FAIL1,
	G29_CATCH_FAIL2,
	G29_CATCH_DONE,
	G29_MESH_START,
	G29_MESH_READY,
	G29_MESH_PROBING,	
	G29_MESH_VALUE,
	G29_MESH_DONE
}_emDWIN_G29_MSG;

class DWINLCD_MENU{
	private:
			
	public:		 	
		static uint8_t now, last;	
		static uint8_t index;
  	void set(uint8_t v) { now = last = v; }
  	void reset() { set(0); }
  	bool changed() { bool c = (now != last); if (c) last = now; return c; }
  	bool dec() { if (now) now--; return changed(); }
  	bool inc(uint8_t v) { if (now < (v - 1)) now++; else now = (v - 1); return changed(); }
};

//typedef struct { 
//}select_t;


typedef struct Mixer_Display_cfg{
	uint16_t Area_X_Start = 10;
	uint16_t Area_X_End = 267;
	uint16_t Area_Y_Start = 143;
	uint16_t Area_Y_End = 171;
	uint16_t Extruder_X_Coordinate[MIXING_STEPPERS] = {0};
	uint8_t Extruder_Int_Number[MIXING_STEPPERS] = {0};
	uint16_t VTool_X_Coordinate = 0;
	uint8_t VTool_Int_Number = 0;
	uint8_t Extruder_X_Start_Coordinate[5] = {0,0,34,19,8};
	uint8_t Extruder_X_Start_Gap[5] = {0,0,78,63,52};
	uint8_t Y_Coordinate = 143;
}MIXER_DIS;
extern MIXER_DIS MixerDis;

typedef struct Mixer_Print_cfg{
	uint8_t Mixer_Mode_Rg;
	float  Zpos_Buff = 0;
	int8_t Current_Percent[MIXING_STEPPERS] = {0};
	int8_t Manual_Percent[MIXING_VIRTUAL_TOOLS][MIXING_STEPPERS] = {0};
	int8_t Start_Percent[MIXING_STEPPERS] = {0};
	int8_t End_Percent[MIXING_STEPPERS] = {0};
	int8_t Auto_Percent[MIXING_VIRTUAL_TOOLS][MIXING_STEPPERS] = {0};
	//int8_t occupy_vtool = MIXING_VIRTUAL_TOOLS - 1;
	uint8_t Vtool_Backup = 0;
}MIXER_CFG;
extern MIXER_CFG MixerCfg;

typedef struct Filament_Runout_cfg{
	uint8_t Font = font12x24;
	uint8_t Font_W = 12;
	uint8_t Font_H = 24;
	uint16_t Text_Color = Popup_Text_Color;
	uint16_t Window_Color = Color_Bg_Window;
	uint16_t Text_Pos_Y = 240;
}FIL_CFG;
extern FIL_CFG FIL;

typedef struct {
  TERN_(HAS_HOTEND,     int16_t E_Temp    = 0);
  TERN_(HAS_HEATED_BED, int16_t Bed_Temp  = 0);
  TERN_(HAS_PREHEAT,    int16_t Fan_speed = 0);
  int16_t print_speed     = 100;
  int16_t Max_Feedspeed     = 0;
  uint16_t Max_Acceleration  = 0;
  int16_t Max_Jerk          = 0;
  int16_t Max_Step          = 0;
  int16_t Move_X_scale      = 0;
  int16_t Move_Y_scale      = 0;
  int16_t Move_Z_scale      = 0;
  int16_t Auto_Zstart_scale  = 0;
  int16_t Auto_Zend_scale    = 0;
  int16_t Random_Zstart_scale  = 0;
  int16_t Random_Zend_scale    = 0;
  int16_t Random_Height = 0;
	#if ENABLED(OPTION_BED_COATING)
  int16_t coating_thickness = 0;
	#endif
  uint8_t  Random_Extruders = 0;
  int16_t Retract_MM_scale      = 0;
  int16_t Retract_V_scale      = 0;
	int16_t Retract_ZHOP_scale      = 0;
  int16_t unRetract_MM_scale      = 0;
  int16_t unRetract_V_scale      = 0;
	int16_t Zoffset_Scale      = 0;
	int16_t ProbeZoffset_Scale = 0;
	
  #if HAS_HOTEND	
	int16_t Current_E_Scale[E_STEPPERS];
	float Last_E_Coordinate[E_STEPPERS];
	#if ENABLED(MIXING_EXTRUDER)
	int16_t Current_EAll_Scale    = 0;
	float Last_EAll_Coordinate    = 0;
	#endif
  #endif

	uint8_t Percentrecord = 0;
	uint16_t remain_time = 0;
	millis_t dwin_heat_time = 0;
} HMI_value_t;

typedef struct {  
  bool 	refersh_mix_flag:1,
   			select_flag:1,
   			heat_flag:1,  					// 0: heating done  1: during heating
   			Is_Mixer_Print:1,
   			need_home_flag:1,
   			lcd_sd_status:1
		#if ENABLED(LCD_BED_LEVELING)	 	 
				,Leveling_Menu_Fg:1				
		#endif
		#if ENABLED(OPTION_WIFI_MODULE)
				,wifi_Handshake_ok:1
		#endif		
		#if ENABLED(PREVENT_COLD_EXTRUSION)
			 	,ETempTooLow_flag:1
		#endif
		#if ENABLED(OPTION_AUTOPOWEROFF)
				,Autoshutdown_enabled:1
		#endif
		;

	uint8_t	clean_status_delay = 0;
	uint8_t language;
  uint8_t Title_Menu_Backup = 0;
	
  #if ENABLED(OPTION_WIFI_MODULE)  
  uint8_t wifi_link_timer;
  #endif
  
	#if ENABLED(LCD_BED_LEVELING)	 
  uint8_t Leveling_Case_Total = 4;
  #endif

  float pause_zpos_backup;
	uint8_t killtimes = 0;
 
  #if HAS_FAN
    AxisEnum feedspeed_axis;
  #endif
	
  #if ENABLED(OPTION_AUTOPOWEROFF)
		uint16_t free_close_timer_rg = POWERDOWN_MACHINE_TIMER;
  #endif 

	char show_mode          = 0;    // 0: Printing temperature -1: Temperature control
	uint8_t auto_test_flag = 0x55; //0x55: disable, 0xAA: enabled
  	
  AxisEnum acc_axis, jerk_axis, step_axis;
} HMI_Flag_t;


extern HMI_value_t HMI_ValueStruct;
extern HMI_Flag_t HMI_flag;
extern _emDWINState DWIN_status;

void Redraw_SD_List();
#if ENABLED(OPTION_AUTOPOWEROFF)
void _reset_shutdown_timer();
#endif

// Show ICO
void ICON_Print(bool show);
void ICON_Prepare(bool show);
void ICON_Control(bool show);
FORCE_INLINE void update_Z_Position(bool bshowICON);


void ICON_StartInfo(bool show);

void ICON_Setting(bool show);
void ICON_Pause(bool show);
void ICON_Continue(bool show);
void ICON_Stop(bool show);


// Popup message window
void Popup_Window_Temperature(const char *msg);


#if ENABLED(PREVENT_COLD_EXTRUSION)
  void Popup_Window_ETempTooLow();
#endif

void Popup_Window_Resume();
void Popup_Window_HomeAll(const bool parking/*=false*/);
void Popup_Window_HomeX(const bool parking/*=false*/);
void Popup_Window_HomeY(const bool parking/*=false*/);
void Popup_Window_HomeZ(const bool parking/*=false*/);


#ifdef LCD_BED_LEVELING
void Popup_Remove_Glass();
#endif


void Draw_Main_Menu();

// Variable control
void HMI_Move_X();
void HMI_Move_Y();
void HMI_Move_Z();
void HMI_Move_E();

void HMI_Zoffset();

TERN_(HAS_HOTEND,     void HMI_ETemp());
TERN_(HAS_HEATED_BED, void HMI_BedTemp());
TERN_(HAS_FAN,        void HMI_FanSpeed());

void HMI_PrintSpeed();

void HMI_MaxFeedspeedXYZE();
void HMI_MaxAccelerationXYZE();
void HMI_MaxJerkXYZE();
void HMI_StepXYZE();

// SD Card
void HMI_SDCardInit();
void HMI_SDCardUpdate();

// Main Process
void Icon_print(bool value);
void Icon_control(bool value);
void Icon_temperature(bool value);
void Icon_leveling(bool value);

// Other
void Draw_Status_Area(const bool with_update); // Status Area
void Draw_Mixer_Status_Area(const bool with_update); // Mixer Status Area
void HMI_StartFrame(const bool with_update);   // Prepare the menu view
void HMI_MainMenu();    // Main process screen
void HMI_SelectFile();  // File page
void HMI_Printing();    // Print page
void HMI_Prepare();     // Prepare page
void HMI_Control();     // Control page
void HMI_Leveling();    // Level the page
void HMI_AxisMove();    // Axis movement menu
void HMI_Temperature(); // Temperature menu
void HMI_Motion();      // Sports menu
void HMI_Info();        // Information menu
void HMI_Tune();        // Adjust the menu

#if HAS_PREHEAT
  void HMI_PLAPreheatSetting(); // PLA warm-up setting
  void HMI_ABSPreheatSetting(); // ABS warm-up setting
#endif

void HMI_MaxSpeed();        // Maximum speed submenu
void HMI_MaxAcceleration(); // Maximum acceleration submenu
void HMI_MaxJerk();         // Maximum jerk speed submenu
void HMI_StepPermm();            // Transmission ratio

void HMI_Init();
void DWIN_Update();
void EachMomentUpdate();
void DWIN_HandleScreen();

void DWIN_PopMenu_HomeDone();
void DWIN_PopMenu_LevelingDone();

void Refresh_Percent_display();
void Draw_Print_ProgressMixModel();
void updata_mixer_from_vtool();
void DWIN_Draw_PrintDone_Confirm();

void Clear_Bottom_Area();
void Popup_Window_FMoveStart();
void Popup_Window_BMoveStart();
void Popup_Window_BMoveStop();
void Popup_Window_waiting();
void DWIN_ResumedFromPause();

void DWIN_Pause_Show_Message(const PauseMessage message,	const PauseMode mode/*=PAUSE_MODE_SAME*/, const uint8_t extruder/*=active_extruder*/);
void DWIN_Pause_Show_Message(const PauseMessage message,	const PauseMode mode);
void DWIN_Pause_Show_Message( const PauseMessage message);

#if ENABLED(OPTION_WIFI_MODULE)
void DWIN_Wifi_Show_M117(const char * const message);
#endif

void DWIN_G29_Show_Messge(const _emDWIN_G29_MSG message = G29_LEVLE_DEFAULT,const int pt_index = 0,const int all_points = 0,const float fvalue = 0.0);

extern void srand(unsigned int seed);
extern int rand();
#endif
