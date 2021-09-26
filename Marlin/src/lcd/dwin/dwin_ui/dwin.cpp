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

/**
 * DWIN LCD
 */

#include "../../../inc/MarlinConfig.h"

#if HAS_DWIN_LCD
#include "dwin.h"
#include "../language/dwin_multi_language.h"
#include "rotary_encoder.h"
#include "autotest.h"


#if ENABLED(MIXING_EXTRUDER)
  #include "../../../feature/mixing.h"
#endif

#if ENABLED(OPTION_REPEAT_PRINTING)
  #include "../../../feature/repeat_printing.h"
#endif

#if ENABLED(OPTION_3DTOUCH)
  #include "../../../feature/bltouch.h"
#endif

#if ENABLED(FWRETRACT)
  #include "../../../feature/fwretract.h"
#endif

#include <WString.h>
#include <stdio.h>
#include <string.h>

#include "../../fontutils.h"
#include "../../ultralcd.h"

#include "../../../sd/cardreader.h"

#include "../../../MarlinCore.h"
#include "../../../core/serial.h"
#include "../../../core/macros.h"
#include "../../../gcode/queue.h"

#include "../../../module/temperature.h"
#include "../../../module/printcounter.h"
#include "../../../module/motion.h"
#include "../../../module/planner.h"
#include "../../../module/tool_change.h"
#include "../../lcd/thermistornames.h"
#include "../../../module/settings.h"
#include "../../../libs/buzzer.h"

#if ENABLED(HOST_ACTION_COMMANDS)
  #include "../../../feature/host_actions.h"
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
#include "../../../feature/runout.h"
#endif
#include "../../../feature/pause.h"

#if HAS_BED_PROBE
  #include "../../../module/probe.h"
#endif

#if ENABLED(BABYSTEPPING)
  #include "../../../feature/babystep.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../../feature/powerloss.h"
#endif

#ifndef MACHINE_SIZE
  #define MACHINE_SIZE "300x300x400"
#endif

#ifndef CORP_WEBSITE_C
  #define CORP_WEBSITE_C "www.zonestar3d.com"
#endif

#ifndef CORP_WEBSITE_E
  #define CORP_WEBSITE_E "www.zonestar3d.com"
#endif

#if HAS_LEVELING
  #include "../../../feature/bedlevel/bedlevel.h"
#endif

#define PAUSE_HEAT
#define DWIN_FONT_MENU font8x16
#define DWIN_FONT_STAT font10x20
#define DWIN_FONT_HEAD font10x20
#define DWIN_FONT_MIX  font14x28

#define MENU_CHAR_LIMIT  	24
#define STATUS_Y_START 	 	360
#define STATUS_Y_END 	 		DWIN_HEIGHT

#define STATUS_MIXER_Y_START  STATE_TEXT_MIX_Y
#define STATUS_MIXER_Y_END 	 	STATUS_MIXER_Y_START +28

#define	MENUVALUE_X			210
#define	MENUMORE_X			226
#define	CONFIGVALUE_X		210
#define	MENUONOFF_X			220


// Fan speed limit
#define FANON           255
#define FANOFF          0

// Print speed limit
#define MAX_PRINT_SPEED   500
#define MIN_PRINT_SPEED   10

// Temp limits
#if HAS_HOTEND
  #define MAX_E_TEMP    (HEATER_0_MAXTEMP - (HOTEND_OVERSHOOT))
  #define MIN_E_TEMP    HEATER_0_MINTEMP
#endif

#if HAS_HEATED_BED
  #define MIN_BED_TEMP  BED_MINTEMP
#endif

// Feedspeed limit (max feedspeed = DEFAULT_MAX_FEEDRATE * 2)
#define MIN_MAXFEEDSPEED      1
#define MIN_MAXACCELERATION   10
#define MIN_MAXJERK           0.1
#define MIN_STEP              1


// Mininum unit (0.1) : multiple (10)
#define MINUNITMULT     10
// Maxinum unit (0.01) : multiple (100)
#define MAXUNITMULT     100


#define	MROWS					5			// Menu lines (don't include return)
#define	TROWS					6			// Total menu lines
#define	TITLE_HEIGHT	30		// Title bar height
#define	MLINE					53		// Menu line height
#define	LBLX					55		// Menu item label X
#define	LBLX_INFO			25		// Info Menu item label X
#define	MENU_CHR_W		8			// menu char width
#define	MENU_CHR_H		16		// menu char height
#define	STAT_CHR_W		10		// state char width
#define	STAT_CHR_H		20		// state char height

#define MBASE(L) (49 + MLINE * (L))

/* Value Init */
HMI_value_t HMI_ValueStruct;
HMI_Flag_t HMI_flag = {0};
FIL_CFG FIL;
MIXER_CFG MixerCfg;
MIXER_DIS MixerDis;

_emDWINState DWIN_status = ID_SM_START;

_emDWIN_MENUID_ DwinMenuID = DWMENU_MAIN;

uint8_t DWINLCD_MENU::now;
uint8_t DWINLCD_MENU::last;
uint8_t DWINLCD_MENU::index = MROWS;

DWINLCD_MENU select_main;
DWINLCD_MENU select_file;
DWINLCD_MENU select_print;
DWINLCD_MENU select_prepare;
DWINLCD_MENU select_control;
DWINLCD_MENU select_axis;
DWINLCD_MENU select_temp;
DWINLCD_MENU select_motion;
DWINLCD_MENU select_mixer;
DWINLCD_MENU select_tune;
DWINLCD_MENU select_PLA;
DWINLCD_MENU select_ABS;
DWINLCD_MENU select_feedrate;
DWINLCD_MENU select_accel;
DWINLCD_MENU select_jerk;
DWINLCD_MENU select_step;
DWINLCD_MENU select_manual;
DWINLCD_MENU select_auto;
DWINLCD_MENU select_random;
DWINLCD_MENU select_vtool;
DWINLCD_MENU select_leveling;
DWINLCD_MENU select_home;
DWINLCD_MENU select_option;
DWINLCD_MENU select_bltouch;
DWINLCD_MENU select_powerdown;
DWINLCD_MENU select_language;
DWINLCD_MENU select_info;
DWINLCD_MENU select_config;
DWINLCD_MENU select_retract;
DWINLCD_MENU select_reprint;
		
#if ENABLED(PAUSE_HEAT)
 #if HAS_HOTEND
  uint16_t temphot = 0;
 #endif
 #if HAS_HEATED_BED
  uint16_t tempbed = 0;
 #endif
#endif

#if ENABLED(BABYSTEPPING)
static millis_t Babysteps_timer_first;
static millis_t Babysteps_timer_second;
static float babyz_offset = 0.0;
static float last_babyz_offset = 0.0;
static float prevouis_babyz_offset = 0.0;
#endif

#define	DWIN_Draw_Small_Float21(x,y,v)							dwinLCD.Draw_SignedFloatValue(font8x16, Color_White, Color_Bg_Black, 2, 1, x, y, v)
#define	DWIN_Draw_Small_Float22(x,y,v)							dwinLCD.Draw_SignedFloatValue(font8x16, Color_White, Color_Bg_Black, 2, 2, x, y, v)
#define	DWIN_Draw_Small_Float31(x,y,v)							dwinLCD.Draw_SignedFloatValue(font8x16, Color_White, Color_Bg_Black, 3, 1, x, y, v)
#define	DWIN_Draw_Small_Float32(x,y,v)							dwinLCD.Draw_SignedFloatValue(font8x16, Color_White, Color_Bg_Black, 3, 2, x, y, v)
#define	DWIN_Draw_Small_Float41(x,y,v)							dwinLCD.Draw_SignedFloatValue(font8x16, Color_White, Color_Bg_Black, 4, 1, x, y, v)
#define	DWIN_Draw_Selected_Small_Float21(x,y,v)			dwinLCD.Draw_SignedFloatValue(font8x16, Select_Color, Color_Bg_Black, 2, 1, x, y, v)
#define	DWIN_Draw_Selected_Small_Float22(x,y,v)			dwinLCD.Draw_SignedFloatValue(font8x16, Select_Color, Color_Bg_Black, 2, 2, x, y, v)
#define	DWIN_Draw_Selected_Small_Float31(x,y,v)			dwinLCD.Draw_SignedFloatValue(font8x16, Select_Color, Color_Bg_Black, 3, 1, x, y, v)
#define	DWIN_Draw_Selected_Small_Float32(x,y,v)			dwinLCD.Draw_SignedFloatValue(font8x16, Select_Color, Color_Bg_Black, 3, 2, x, y, v)
#define	DWIN_Draw_Selected_Small_Float41(x,y,v)			dwinLCD.Draw_SignedFloatValue(font8x16, Select_Color, Color_Bg_Black, 4, 1, x, y, v)
#define	DWIN_Draw_Big_Float32(x,y,v)								dwinLCD.Draw_SignedFloatValue(font14x28, Color_White, Color_Bg_Black, 3, 2, x, y, v)
#define	DWIN_Draw_MaskString_Default(x,y,s) 				dwinLCD.Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, x, y, s)
#define DWIN_Draw_MaskString_Default_Color(c,x,y,s)	dwinLCD.Draw_String(false, true, font8x16, c, Color_Bg_Black, x, y, s)
#define	DWIN_Draw_MaskString_FONT12(a,b,x,y,s) 			dwinLCD.Draw_String(false, true, font12x24, a, b, x, y, s)
#define	DWIN_Draw_MaskString_Default_PopMenu(x,y,s) dwinLCD.Draw_String(false, true, font10x20, Popup_Text_Color, Color_Bg_Window, x, y, s)
#define	DWIN_Draw_UnMaskString_Default(x,y,s) 			dwinLCD.Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, x, y, s)
#define	DWIN_Draw_UnMaskString_FONT10(x,y,s) 				dwinLCD.Draw_String(false, false, font10x20, Color_White, Color_Bg_Black, x, y, s)
#define	DWIN_Draw_UnMaskString_FONT10_TITLE(x,y,s) 	dwinLCD.Draw_String(false, false, font10x20, Color_White, Color_Bg_Blue, x, y, s)
#define	DWIN_Draw_IntValue_Default(n,x,y,v) 				dwinLCD.Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, n, x, y, v)
#define DWIN_Draw_MaskIntValue_Default(n,x,y,v) 		dwinLCD.Draw_IntValue(true, true, 1, font8x16, Color_White, Color_Bg_Black, n, x, y, v)
#define DWIN_Draw_UnMaskIntValue_Default(n,x,y,v) 	dwinLCD.Draw_IntValue(false, true, 1, font8x16, Color_White, Color_Bg_Black, n, x, y, v)
#define DWIN_Draw_IntValue_FONT10(n,x,y,v)					dwinLCD.Draw_IntValue(true, true, 0, font10x20, Color_White, Color_Bg_Black, n, x, y, v)
#define DWIN_Draw_Select_IntValue_Default(n,x,y,v)	dwinLCD.Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, n, x, y, v)
#define DWIN_Draw_String_FIL(x, y, s)								dwinLCD.Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, x, y, s)
 

inline bool Apply_Encoder(const ENCODER_DiffState &encoder_diffState, auto &valref) {
  if (encoder_diffState == ENCODER_DIFF_CW)
    valref += EncoderRate.encoderMoveValue;
  else if (encoder_diffState == ENCODER_DIFF_CCW)
    valref -= EncoderRate.encoderMoveValue;
  else if (encoder_diffState == ENCODER_DIFF_ENTER)
    return true;
  return false;
}


#if ENABLED(MIXING_EXTRUDER)
uint8_t Check_Percent_equal(){
	for(uint8_t i=0; i<MIXING_STEPPERS-1; i++) {
		if(MixerCfg.Manual_Percent[mixer.selected_vtool][i] != MixerCfg.Manual_Percent[mixer.selected_vtool][i+1]) return 1;
	}
	return 0;
}

void updata_mixer_from_vtool(){
	float ctot = 0;
	int16_t sum_mix = 0;
  MIXER_STEPPER_LOOP(i) ctot += mixer.color[mixer.selected_vtool][i];
  MIXER_STEPPER_LOOP(i) mixer.mix[i] = (int8_t)(100 * mixer.color[mixer.selected_vtool][i] / ctot);
	for(uint8_t i=0; i<MIXING_STEPPERS-1; i++) sum_mix += mixer.mix[i];
	mixer.mix[MIXING_STEPPERS-1] = 100 - sum_mix;
}
#endif


#define ICON_IMAGE_ID	8
inline void DWIN_Show_ICON(uint8_t picID, uint16_t x, uint16_t y){
	dwinLCD.ICON_Show(ICON_IMAGE_ID, picID, x, y);
}

inline void DWIN_Frame_AreaCopy_ID1(uint8_t ItemID, uint16_t x, uint16_t y){
	dwinLCD.Frame_AreaCopy_Index(IMAGE_CACHE_ID1, HMI_flag.language, ItemID, MultiLangStr_Coordinate, x, y);
}

void ICON_Print() {
	if(select_main.now == MAIN_CASE_PRINT) {
		DWIN_Show_ICON(ICON_Print_1, 17, 130);
		dwinLCD.Draw_Rectangle(0, Color_White, 17, 130, 126, 229);
		DWIN_Show_ICON(Picture_Coordinate[HMI_flag.language][ICON_BIG_PRINT], Print_X_Coordinate[HMI_flag.language], 201);
	}
	else {
		DWIN_Show_ICON( ICON_Print_0, 17, 130);
		DWIN_Frame_AreaCopy_ID1(MTSTRING_MAIN_PRINT, Print_X_Coordinate[HMI_flag.language],201);
	}
	dwinLCD.UpdateLCD();
}

void ICON_Prepare() {
	if (select_main.now == MAIN_CASE_PREPARE) {
		DWIN_Show_ICON(ICON_Prepare_1, 145, 130);
		dwinLCD.Draw_Rectangle(0, Color_White, 145, 130, 254, 229);
		DWIN_Show_ICON(Picture_Coordinate[HMI_flag.language][ICON_BIG_PREPARE], Prepare_X_Coordinate[HMI_flag.language], 201);
	}
	else {
		DWIN_Show_ICON(ICON_Prepare_0, 145, 130);
		DWIN_Frame_AreaCopy_ID1(MTSTRING_MAIN_PREPARE, Prepare_X_Coordinate[HMI_flag.language],201);
	}
 dwinLCD.UpdateLCD();
}

void ICON_Control() {
	if (select_main.now == MAIN_CASE_CONTROL) {
		DWIN_Show_ICON(ICON_Control_1, 17, 246);
		dwinLCD.Draw_Rectangle(0, Color_White, 17, 246, 126, 345);
		DWIN_Show_ICON( Picture_Coordinate[HMI_flag.language][ICON_BIG_CONTROL], Control_X_Coordinate[HMI_flag.language], 318);
	}
	else {
		DWIN_Show_ICON(ICON_Control_0, 17, 246);
		DWIN_Frame_AreaCopy_ID1(MTSTRING_MAIN_CONTROL, Control_X_Coordinate[HMI_flag.language],318);
	}
	dwinLCD.UpdateLCD();
}

void ICON_StartInfo() {
	if (select_main.now == MAIN_CASE_INFO) {
		DWIN_Show_ICON(ICON_Info_1, 145, 246);
		dwinLCD.Draw_Rectangle(0, Color_White, 145, 246, 254, 345);
		DWIN_Show_ICON( Picture_Coordinate[HMI_flag.language][ICON_BIG_INFO], PrinterInfo_X_Coordinate[HMI_flag.language], 318);
	}
	else {
		DWIN_Show_ICON(ICON_Info_0, 145, 246);
		DWIN_Frame_AreaCopy_ID1(MTSTRING_MAIN_INFO, PrinterInfo_X_Coordinate[HMI_flag.language],318);
	}
	dwinLCD.UpdateLCD();
}


void ICON_Tune() {
	if (select_print.now == PRINT_CASE_TUNE) {
		DWIN_Show_ICON(ICON_Setup_1, 8, 252);
		dwinLCD.Draw_Rectangle(0, Color_White, 8, 252, 87, 351);
		DWIN_Show_ICON( Picture_Coordinate[HMI_flag.language][ICON_BIG_TUNE], Tune_X_Coordinate[HMI_flag.language], 325);
	}
	else {
		DWIN_Show_ICON(ICON_Setup_0, 8, 252);
		DWIN_Frame_AreaCopy_ID1(Printing_Menu_Tune, Tune_X_Coordinate[HMI_flag.language], 325);
	}
	dwinLCD.UpdateLCD();
}

void ICON_Pause() {
	if ((select_print.now == PRINT_CASE_PAUSE)) {
		DWIN_Show_ICON( ICON_Pause_1, 96, 252);
		dwinLCD.Draw_Rectangle(0, Color_White, 96, 252, 175, 351);
		DWIN_Show_ICON( Picture_Coordinate[HMI_flag.language][ICON_BIG_PAUSE], Pause_X_Coordinate[HMI_flag.language], 325);
	}
	else {
		DWIN_Show_ICON( ICON_Pause_0, 96, 252);
		DWIN_Frame_AreaCopy_ID1(Printing_Menu_Pause, Pause_X_Coordinate[HMI_flag.language], 325);
	}
	dwinLCD.UpdateLCD();
}

void ICON_Continue() {
	if (select_print.now == PRINT_CASE_PAUSE) {
		DWIN_Show_ICON( ICON_Continue_1, 96, 252);
		dwinLCD.Draw_Rectangle(0, Color_White, 96, 252, 175, 351);
		DWIN_Show_ICON(Picture_Coordinate[HMI_flag.language][ICON_BIG_CONTINUE], Continue_X_Coordinate[HMI_flag.language], 325);
	}
	else {
		DWIN_Show_ICON( ICON_Continue_0, 96, 252);
		DWIN_Frame_AreaCopy_ID1(Printing_Menu_Continue, Continue_X_Coordinate[HMI_flag.language], 325);
	}
	dwinLCD.UpdateLCD();
}

void ICON_Stop() {
	if (select_print.now == PRINT_CASE_STOP) {
		DWIN_Show_ICON( ICON_Stop_1, 184, 252);
		dwinLCD.Draw_Rectangle(0, Color_White, 184, 252, 263, 351);
		DWIN_Show_ICON( Picture_Coordinate[HMI_flag.language][ICON_BIG_STOP], Stop_X_Coordinate[HMI_flag.language], 325);
	}
	else {
		DWIN_Show_ICON( ICON_Stop_0, 184, 252);
		DWIN_Frame_AreaCopy_ID1(Printing_Menu_Stop, Stop_X_Coordinate[HMI_flag.language], 325);
	}
	dwinLCD.UpdateLCD();
}

void ICON_YESorNO(uint8_t Option){
	if (Option == false) {
  	DWIN_Show_ICON( ICON_YES_0, 26, 168);
		DWIN_Show_ICON( ICON_NO_1, 146, 168);
  	dwinLCD.Draw_Rectangle(0, Color_White, 26, 168, 126, 206);
 	}
	else{
		DWIN_Show_ICON( ICON_YES_1, 26, 168);
		DWIN_Show_ICON( ICON_NO_0, 146, 168);
  	dwinLCD.Draw_Rectangle(0, Color_White, 146, 168, 246, 206);
	}
}

void ICON_YESorNO_Powerdown(uint8_t Option){
	if (Option == false) {
  	DWIN_Show_ICON( ICON_NO_0, 26, 228);
		DWIN_Show_ICON( ICON_YES_1, 146, 228);
  	dwinLCD.Draw_Rectangle(0, Color_White, 26, 228, 126, 266);
	}
	else{
		DWIN_Show_ICON( ICON_NO_1, 26, 228);
		DWIN_Show_ICON( ICON_YES_0, 146, 228);
	 	dwinLCD.Draw_Rectangle(0, Color_White, 146, 228, 246, 266);
	}
}

inline void Clear_Title_Bar() {
	dwinLCD.Draw_Rectangle(1, Color_Bg_Blue, 0, 0, DWIN_WIDTH, 30);
}

inline void Draw_Title(const char * const title) {
	DWIN_Draw_UnMaskString_FONT10_TITLE(14, 4, (char*)title);
}

inline void Draw_Title(const __FlashStringHelper * title) {
	DWIN_Draw_UnMaskString_FONT10_TITLE(14, 4, (char*)title);
}

inline void Draw_Wifi_Title(const char * const title) {
	DWIN_Draw_UnMaskString_FONT10_TITLE(14, 65, (char*)title);
}

inline void Draw_Reprint_Title(const char * const title) {
	DWIN_Draw_UnMaskString_FONT10_TITLE(14, 65, (char*)title);
}


inline void Clear_Menu_Area() {
	dwinLCD.Draw_Rectangle(1, Color_Bg_Black, 0, 31, DWIN_WIDTH, STATUS_Y_START);
}

inline void Clear_Main_Window() {
	Clear_Title_Bar();
	Clear_Menu_Area();
}

inline void Clear_Bottom_Area() {
	dwinLCD.Draw_Rectangle(1, Color_Bg_Black, 0, 448, DWIN_WIDTH, STATUS_Y_END);
}

inline void Clear_Popup_Area() {
	Clear_Title_Bar();
	dwinLCD.Draw_Rectangle(1, Color_Bg_Black, 0, 31, DWIN_WIDTH, DWIN_HEIGHT);
}

inline void Draw_More_Icon(const uint8_t line) {
	DWIN_Show_ICON(ICON_More, MENUMORE_X, MBASE(line) - 3);
}

inline void Draw_Menu_Cursor(const uint8_t line) {
	dwinLCD.Draw_Rectangle(1, Rectangle_Color, 0, MBASE(line) - 18, 14, MBASE(line + 1) - 20);
}

inline void Erase_Menu_Cursor(const uint8_t line) {
	dwinLCD.Draw_Rectangle(1, Color_Bg_Black, 0, MBASE(line) - 18, 14, MBASE(line + 1) - 20);
}

inline void Move_Highlight(const int16_t from, const uint16_t newline) {
	Erase_Menu_Cursor(newline - from);
	Draw_Menu_Cursor(newline);
}

void Draw_Popup_Bkgd_105() {
	dwinLCD.Draw_Rectangle(1, Color_Bg_Window, 14, 105, 258, 374);
}

inline void Add_Menu_Line() {
	Move_Highlight(1, MROWS);
	dwinLCD.Draw_Line(Line_Color, 16, MBASE(MROWS + 1) - 20, 256, MBASE(MROWS + 1) - 19);
}

inline void Scroll_Menu(const uint8_t dir) {
	dwinLCD.Frame_AreaMove(1, dir, MLINE, Color_Bg_Black, 0, 31, DWIN_WIDTH, 349);
	switch (dir) {
		case DWIN_SCROLL_DOWN: Move_Highlight(-1, 0); break;
	  case DWIN_SCROLL_UP:  Add_Menu_Line(); break;
	}
}

inline uint16_t nr_sd_menu_items() {
	return card.get_num_Files() + !card.flag.workDirIsRoot;
}

inline void Draw_Menu_Icon(const uint8_t line, const uint8_t icon) {
	DWIN_Show_ICON( icon, 26, MBASE(line) - 3);
}

inline void Erase_Menu_Text(const uint8_t line) {
	dwinLCD.Draw_Rectangle(1, Color_Bg_Black, LBLX, MBASE(line) - 14, 271, MBASE(line) + 28);
}

inline void Draw_Menu_Line(const uint8_t line, const uint8_t icon=0, const char * const label=nullptr) {
	if (label) DWIN_Draw_UnMaskString_Default(LBLX, MBASE(line) - 1, (char*)label);
	if (icon) Draw_Menu_Icon(line, icon);
	dwinLCD.Draw_Line(Line_Color, 16, MBASE(line) + 33, 256, MBASE(line) + 34);
}


// Draw "Back" line at the top
inline void Draw_Back_First(const bool is_sel=true) {
	Draw_Menu_Line(0, ICON_Back);
	DWIN_Frame_AreaCopy_ID1(MTSTRING_MENU_BACK, LBLX, MBASE(0));
	dwinLCD.UpdateLCD();
	if (is_sel) Draw_Menu_Cursor(0);
}

float last_Extr_scale[E_STEPPERS];
inline void _init_Move_Extr(){
	EXTR_STEPPER_LOOP(i){ 
		last_Extr_scale[i] = 0;
		HMI_ValueStruct.Current_E_Scale[i] = 0;
		HMI_ValueStruct.Last_E_Coordinate[i] = 0.0;
	}
}

/**
 * Read and cache the working directory.
 *
 * TODO: New code can follow the pattern of menu_media.cpp
 * and rely on Marlin caching for performance. No need to
 * cache files here.
 */

#ifndef strcasecmp_P
 #define strcasecmp_P(a, b) strcasecmp((a), (b))
#endif

inline void make_name_without_ext(char *dst, char *src, size_t maxlen=MENU_CHAR_LIMIT) {
 char * const name = card.longest_filename();
 size_t pos    = strlen(name); // index of ending nul

 // For files, remove the extension
 // which may be .gcode, .gco, or .g
 if (!card.flag.filenameIsDir)
  while (pos && src[pos] != '.') pos--; // find last '.' (stop at 0)

 size_t len = pos;  // nul or '.'
 if (len > maxlen) { // Keep the name short
  pos    = len = maxlen; // move nul down
  dst[--pos] = '.'; // insert dots
  dst[--pos] = '.';
  dst[--pos] = '.';
 }

 dst[len] = '\0';  // end it

 // Copy down to 0
 while (pos--) dst[pos] = src[pos];
}

/**
 * Display an SD item, adding a CDUP for subfolders.
 */
#define ICON_Folder ICON_More
inline void Draw_SDItem(const uint16_t item, int16_t row=-1) {
	if (row < 0) row = item + 1 + MROWS - select_file.index;
	const bool is_subdir = !card.flag.workDirIsRoot;
	if (is_subdir && item == 0) {
		Draw_Menu_Line(row, ICON_Folder, "..");
		return;
	}

	card.getfilename_sorted(item - is_subdir);
	char * const name = card.longest_filename();

#if ENABLED(SCROLL_LONG_FILENAMES)
	// Init the current selected name
	// This is used during scroll drawing
	if (item == select_file.now - 1) {
		make_name_without_ext(shift_name, name, 100);
		Init_SDItem_Shift();
	}
#endif

	// Draw the file/folder with name aligned left
	char str[strlen(name) + 1];
	make_name_without_ext(str, name);
	Draw_Menu_Line(row, card.flag.filenameIsDir ? ICON_Folder : ICON_File, str);
}

#if ENABLED(SCROLL_LONG_FILENAMES)
inline void Draw_SDItem_Shifted(int8_t &shift) {
	// Limit to the number of chars past the cutoff
	const size_t len = strlen(shift_name);
	NOMORE(shift, _MAX(len - MENU_CHAR_LIMIT, 0U));

	// Shorten to the available space
	const size_t lastchar = _MIN((signed)len, shift + MENU_CHAR_LIMIT);

	const char c = shift_name[lastchar];
	shift_name[lastchar] = '\0';

	const uint8_t row = select_file.now + MROWS - select_file.index; // skip "Back" and scroll
	Erase_Menu_Text(row);
	Draw_Menu_Line(row, 0, &shift_name[shift]);

	shift_name[lastchar] = c;
}

char shift_name[LONG_FILENAME_LENGTH + 1];
int8_t shift_amt; // = 0
millis_t shift_ms; // = 0
// Init the shift name based on the highlighted item
inline void Init_Shift_Name() {
	const bool is_subdir = !card.flag.workDirIsRoot;
	const int8_t filenum = select_file.now - 1 - is_subdir; // Skip "Back" and ".."
	const uint16_t fileCnt = card.get_num_Files();
	if(WITHIN(filenum, 0, fileCnt - 1)){
		card.getfilename_sorted(SD_ORDER(filenum, fileCnt));
		char * const name = card.longest_filename();
		make_name_without_ext(shift_name, name, 100);
	}
}

inline void Init_SDItem_Shift() {
	shift_amt = 0;
	shift_ms = select_file.now > 0 && strlen(shift_name) > MENU_CHAR_LIMIT
	  ? millis() + 750UL : 0;
}
#endif

// Redraw the first set of SD Files
void Redraw_SD_List() {
	select_file.reset();
	select_file.index = MROWS;

	Clear_Menu_Area(); // Leave title bar unchanged
	Draw_Back_First();

	if (card.isMounted()) {
		//if (card.flag.mounted) {
		// As many files as will fit
		LOOP_L_N(i, _MIN(nr_sd_menu_items(), MROWS))
		Draw_SDItem(i, i+1);
		TERN_(SCROLL_LONG_FILENAMES, Init_SDItem_Shift());
	}
	else {
		dwinLCD.Draw_Rectangle(1, Color_Bg_Red, 10, MBASE(3) - 10, DWIN_WIDTH - 10, MBASE(4));
		dwinLCD.Draw_String(false, false, font16x32, Color_Yellow, Color_Bg_Red, ((DWIN_WIDTH) - 8 * 16) / 2, MBASE(3), F("No Media"));
	}
}

inline void SDCard_Up() {
	card.cdup();
	Redraw_SD_List();
	HMI_flag.lcd_sd_status = true; // On next DWIN Update
}

inline void SDCard_Folder(char * const dirname) {
	card.cd(dirname);
	Redraw_SD_List();
	HMI_flag.lcd_sd_status = true; // On next DWIN Update
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//Prepare menu
//
inline void Item_Prepare_Move(const uint8_t row) {
	DWIN_Frame_AreaCopy_ID1(Prepare_Menu_Move, LBLX, MBASE(row));
	Draw_Menu_Line(row, ICON_Axis);
	Draw_More_Icon(row);
}

inline void Item_Prepare_Temp(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Control_Menu_Temperature, LBLX, MBASE(row));
 Draw_Menu_Line(row, ICON_Temperature);
 Draw_More_Icon(row);
}


inline void Item_Prepare_Disable(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Prepare_Menu_Disable_Steppers, LBLX, MBASE(row));
 Draw_Menu_Line(row, ICON_CloseMotor);
}

inline void Item_Prepare_Home(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Prepare_Menu_Auto_Home, LBLX, MBASE(row));
 Draw_Menu_Line(row, ICON_Homing);
 Draw_More_Icon(row);
}

inline void Item_Prepare_Leveling(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Prepare_Menu_Bed, LBLX, MBASE(row));
 //DWIN_Frame_AreaCopy_ID1(Prepare_Menu_Auto, LBLX, MBASE(row));
 DWIN_Frame_AreaCopy_ID1(Prepare_Menu_Leveling, LBLX+Auto_X_Coordinate[HMI_flag.language], MBASE(row));
 Draw_Menu_Line(row, ICON_Leveling0);
 Draw_More_Icon(row);
}

inline void Item_Prepare_Powerdown(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Prepare_Menu_Power_Outage, LBLX, MBASE(row));
 if(HMI_flag.language == 0)
 	DWIN_Frame_AreaCopy_ID1(Prepare_Menu_Power_Off, LBLX+56, MBASE(row));
 Draw_Menu_Line(row, ICON_POWERDOWN);
}

inline void Item_Prepare_Lang(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Prepare_Menu_Language, LBLX, MBASE(row));
 Draw_Menu_Line(row, ICON_Language);
 Draw_More_Icon(row);
}

inline void Draw_Prepare_Menu() {
	Clear_Main_Window();
	Clear_Bottom_Area();

	const int16_t pscroll = MROWS - select_prepare.index; // Scrolled-up lines
	#define PSCROL(L) (pscroll + (L))
	#define PVISI(L) WITHIN(PSCROL(L), 0, MROWS)

	dwinLCD.JPG_CacheTo1(HMI_flag.Title_Menu_Backup);
	DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_PREPARE, TITLE_X, TITLE_Y);
	dwinLCD.JPG_CacheTo1(HMI_flag.language+1);
 
	if (PVISI(0)) Draw_Back_First(select_prepare.now == 0);             // < Back 

	if (PVISI(PREPARE_CASE_HOME)) Item_Prepare_Home(PSCROL(PREPARE_CASE_HOME));   // Auto Home
	if (PVISI(PREPARE_CASE_TEMP)) Item_Prepare_Temp(PSCROL(PREPARE_CASE_TEMP));   // Move >
	if (PVISI(PREPARE_CASE_MOVE)) Item_Prepare_Move(PSCROL(PREPARE_CASE_MOVE));   // Temperature > 
	if (PVISI(PREPARE_CASE_LEVELING)) Item_Prepare_Leveling(PSCROL(PREPARE_CASE_LEVELING));   // Leveling 
	if (PVISI(PREPARE_CASE_DISA)) Item_Prepare_Disable(PSCROL(PREPARE_CASE_DISA)); // Disable Stepper 
	if (PVISI(PREPARE_CASE_LANG)) Item_Prepare_Lang(PSCROL(PREPARE_CASE_LANG));   // Language CN/EN
	if (PVISI(PREPARE_CASE_POWERDOWN)) Item_Prepare_Powerdown(PSCROL(PREPARE_CASE_POWERDOWN));   // Powerdown
	if (select_prepare.now) Draw_Menu_Cursor(PSCROL(select_prepare.now));
}

//
//Prepare >> HOME
//
inline void Draw_Home_Menu() {
	Clear_Main_Window();

	dwinLCD.JPG_CacheTo1(HMI_flag.Title_Menu_Backup);
	DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_HOME, TITLE_X, TITLE_Y);
	dwinLCD.JPG_CacheTo1(HMI_flag.language+1);
	DWIN_Frame_AreaCopy_ID1(Home_Menu_Home, LBLX, MBASE(HOME_CASE_ALL));
	DWIN_Frame_AreaCopy_ID1(Home_Menu_All, LBLX+Home_X_Coordinate[HMI_flag.language], MBASE(HOME_CASE_ALL));
	DWIN_Frame_AreaCopy_ID1(Home_Menu_Home, LBLX, MBASE(HOME_CASE_X));
	DWIN_Frame_AreaCopy_ID1(Home_Menu_X, LBLX+Home_X_Coordinate[HMI_flag.language], MBASE(HOME_CASE_X));
	DWIN_Frame_AreaCopy_ID1(Home_Menu_Home, LBLX, MBASE(HOME_CASE_Y));
	DWIN_Frame_AreaCopy_ID1(Home_Menu_Y, LBLX+Home_X_Coordinate[HMI_flag.language], MBASE(HOME_CASE_Y));
	DWIN_Frame_AreaCopy_ID1(Home_Menu_Home, LBLX, MBASE(HOME_CASE_Z));
	DWIN_Frame_AreaCopy_ID1(Home_Menu_Z, LBLX+Home_X_Coordinate[HMI_flag.language], MBASE(HOME_CASE_Z));

	Draw_Back_First(select_home.now == 0);
	if (select_home.now) Draw_Menu_Cursor(select_home.now);

	uint8_t i,j=ICON_HOME_ALL;
	for(i=HOME_CASE_ALL; i<HOME_CASE_TOTAL+1; i++,j++) {
		Draw_Menu_Line(i,j);
	}
}

//
//Prepare >> Temperature
//
inline void Item_Temperature_ETemp(const uint8_t row) {
	DWIN_Frame_AreaCopy_ID1(Temp_Menu_Hotend, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Temp_Menu_Temp, LBLX+Hotend_X_Coordinate[HMI_flag.language], MBASE(row));
	Draw_Menu_Line(row, ICON_SetEndTemp);
  DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(row), thermalManager.temp_hotend[0].target);
}

inline void Item_Temperature_BTemp(const uint8_t row) {
	DWIN_Frame_AreaCopy_ID1(Temp_Menu_Bed, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Temp_Menu_Temp, LBLX+Bed_X_Coordinate[HMI_flag.language], MBASE(row));
	Draw_Menu_Line(row, ICON_SetEndTemp);
  DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(row), thermalManager.temp_bed.target);
}

inline void Item_Temperature_FANSpeed(const uint8_t row) {
	DWIN_Frame_AreaCopy_ID1(Temp_Menu_Fan_Speed, LBLX, MBASE(row));
	Draw_Menu_Line(row, ICON_FanSpeed);
  DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(row), thermalManager.fan_speed[0]);
}

#if HAS_PREHEAT
 inline void Item_Temperature_Cool(const uint8_t row) {
 	DWIN_Frame_AreaCopy_ID1(Prepare_Menu_Cooldown, LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_Cool);
 }
#endif

inline void Item_Temperature_PLA(const uint8_t row) {
	DWIN_Frame_AreaCopy_ID1(Prepare_Menu_Preheat, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Prepare_Menu_PLA, LBLX+Preheat_X_Coordinate[HMI_flag.language], MBASE(row));
	Draw_Menu_Line(row, ICON_PLAPreheat);
}

inline void Item_Temperature_ABS(const uint8_t row) {
	DWIN_Frame_AreaCopy_ID1(Prepare_Menu_Preheat, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Prepare_Menu_ABS, LBLX+Preheat_X_Coordinate[HMI_flag.language], MBASE(row));
	Draw_Menu_Line(row, ICON_ABSPreheat);
}

inline void Draw_Temperature_Menu() {
	Clear_Main_Window();
#if TEMP_CASE_TOTAL >= 6
  const int16_t tscroll = MROWS - select_temp.index; // Scrolled-up lines
  #define TCSCROL(L) (tscroll + (L))
#else
	#define TCSCROL(L) (L)
#endif
 	#define TCLINE(L) MBASE(TCSCROL(L))
 	#define TCVISI(L) WITHIN(TCSCROL(L), 0, MROWS)

	dwinLCD.JPG_CacheTo1(HMI_flag.Title_Menu_Backup);
	DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_TEMPERATURE, TITLE_X, TITLE_Y);
	dwinLCD.JPG_CacheTo1(HMI_flag.language+1);

	if (TCVISI(0)) Draw_Back_First(select_temp.now == 0);             		 // < Back 
	
	if (TCVISI(TEMP_CASE_ETEMP)) Item_Temperature_ETemp(TCSCROL(TEMP_CASE_ETEMP));
	if (TCVISI(TEMP_CASE_BTEMP)) Item_Temperature_BTemp(TCSCROL(TEMP_CASE_BTEMP));
	if (TCVISI(TEMP_CASE_FAN)) Item_Temperature_FANSpeed(TCSCROL(TEMP_CASE_FAN));
	if (TCVISI(TEMP_CASE_PREHEATPLA)) Item_Temperature_PLA(TCSCROL(TEMP_CASE_PREHEATPLA));
	if (TCVISI(TEMP_CASE_PREHEATABS)) Item_Temperature_ABS(TCSCROL(TEMP_CASE_PREHEATABS));
	if (TCVISI(TEMP_CASE_COOL)) Item_Temperature_Cool(TCSCROL(TEMP_CASE_COOL));

	if (select_temp.now) Draw_Menu_Cursor(select_temp.now);
}

//
//Prepare >> Move
//
inline void Item_Axis_MoveX(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Move_Menu_Move, LBLX, MBASE(row));
 DWIN_Frame_AreaCopy_ID1(Move_Menu_X, LBLX+Move_X_Coordinate[HMI_flag.language], MBASE(row));
 Draw_Menu_Line(row, ICON_MoveX);
 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(row), current_position.x * MINUNITMULT);
}

inline void Item_Axis_MoveY(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Move_Menu_Move, LBLX, MBASE(row));
 DWIN_Frame_AreaCopy_ID1(Move_Menu_Y, LBLX+Move_X_Coordinate[HMI_flag.language], MBASE(row));
 Draw_Menu_Line(row, ICON_MoveY);
 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(row), current_position.y * MINUNITMULT);
}

inline void Item_Axis_MoveZ(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Move_Menu_Move, LBLX, MBASE(row));
 DWIN_Frame_AreaCopy_ID1(Move_Menu_Z, LBLX+Move_X_Coordinate[HMI_flag.language], MBASE(row));
 Draw_Menu_Line(row, ICON_MoveZ);
 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(row), current_position.z * MINUNITMULT);
}
#if HAS_HOTEND
inline void Item_Axis_MoveEX1(const uint8_t row) {
	DWIN_Frame_AreaCopy_ID1(Move_Menu_Extruder, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Move_Menu_1, LBLX+Extruder_X_Coordinate[HMI_flag.language], MBASE(row));
	Draw_Menu_Line(row, ICON_Extruder1);
	 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(row), HMI_ValueStruct.Current_E_Scale[0]);
}

#if (E_STEPPERS > 1)
inline void Item_Axis_MoveEX2(const uint8_t row) {
	DWIN_Frame_AreaCopy_ID1(Move_Menu_Extruder, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Move_Menu_2, LBLX+Extruder_X_Coordinate[HMI_flag.language], MBASE(row));
	Draw_Menu_Line(row, ICON_Extruder2);
	 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(row), HMI_ValueStruct.Current_E_Scale[1]);
}
#endif

#if (E_STEPPERS > 2)
inline void Item_Axis_MoveEX3(const uint8_t row) {
	DWIN_Frame_AreaCopy_ID1(Move_Menu_Extruder, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Move_Menu_3, LBLX+Extruder_X_Coordinate[HMI_flag.language], MBASE(row));
	Draw_Menu_Line(row, ICON_Extruder3);
	 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(row), HMI_ValueStruct.Current_E_Scale[2]);
}
#endif
#if (E_STEPPERS > 3)
inline void Item_Axis_MoveEX4(const uint8_t row) {
	DWIN_Frame_AreaCopy_ID1(Move_Menu_Extruder, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Move_Menu_4, LBLX+Extruder_X_Coordinate[HMI_flag.language], MBASE(row));
	Draw_Menu_Line(row, ICON_Extruder4);
	 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(row), HMI_ValueStruct.Current_E_Scale[3]);
}
#endif
#if ENABLED(MIXING_EXTRUDER)
inline void Item_Axis_MoveEXAll(const uint8_t row) {
	DWIN_Frame_AreaCopy_ID1(Move_Menu_Extruder, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Home_Menu_All, LBLX+Extruder_X_Coordinate[HMI_flag.language], MBASE(row));
	Draw_Menu_Line(row, ICON_Language);
	 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(row), HMI_ValueStruct.Current_EAll_Scale);
}
#endif
#endif

inline void Draw_Move_Menu() {
	Clear_Main_Window();	
	_init_Move_Extr(); 
#if AXISMOVE_CASE_TOTAL >= 6
	const int16_t mcroll = MROWS - select_axis.index; // Scrolled-up lines
	#define MSCROL(L) (mcroll + (L))
#else
	#define MSCROL(L) (L)
#endif
	#define MVISI(L) WITHIN(MSCROL(L), 0, MROWS)

	dwinLCD.JPG_CacheTo1(HMI_flag.Title_Menu_Backup);
	DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_MOVE, TITLE_X, TITLE_Y);
	dwinLCD.JPG_CacheTo1(HMI_flag.language+1);
	if (MVISI(0)) Draw_Back_First(select_axis.now == 0);             // < Back
	
	if (MVISI(AXISMOVE_CASE_MOVEX)) Item_Axis_MoveX(MSCROL(AXISMOVE_CASE_MOVEX));
	if (MVISI(AXISMOVE_CASE_MOVEY)) Item_Axis_MoveY(MSCROL(AXISMOVE_CASE_MOVEY));
	if (MVISI(AXISMOVE_CASE_MOVEZ)) Item_Axis_MoveZ(MSCROL(AXISMOVE_CASE_MOVEZ));	
	if (MVISI(AXISMOVE_CASE_EX1)) Item_Axis_MoveEX1(MSCROL(AXISMOVE_CASE_EX1));
#if(E_STEPPERS > 1)
	if (MVISI(AXISMOVE_CASE_EX2)) Item_Axis_MoveEX2(MSCROL(AXISMOVE_CASE_EX2));
#endif
#if(E_STEPPERS > 2)
	if (MVISI(AXISMOVE_CASE_EX3)) Item_Axis_MoveEX3(MSCROL(AXISMOVE_CASE_EX3));
#endif
#if(E_STEPPERS > 3)
	if (MVISI(AXISMOVE_CASE_EX4)) Item_Axis_MoveEX4(MSCROL(AXISMOVE_CASE_EX4));
#endif	
#if ENABLED(MIXING_EXTRUDER)
	if (MVISI(AXISMOVE_CASE_EXALL)) Item_Axis_MoveEXAll(MSCROL(AXISMOVE_CASE_EXALL));
#endif

	if (select_axis.now) Draw_Menu_Cursor(MSCROL(select_axis.now));
}

//
// Prepare >> Bed Leveling
//
inline void Item_Leveling_Point1(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Leveling_Menu_Point, LBLX, MBASE(row));
 DWIN_Frame_AreaCopy_ID1(Leveling_Menu_1, LBLX+Point_X_Coordinate[HMI_flag.language], MBASE(row));
 Draw_Menu_Line(row, ICON_Leveling_Point1);
}

inline void Item_Leveling_Point2(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Leveling_Menu_Point, LBLX, MBASE(row));
 DWIN_Frame_AreaCopy_ID1(Leveling_Menu_2, LBLX+Point_X_Coordinate[HMI_flag.language], MBASE(row));
 Draw_Menu_Line(row, ICON_Leveling_Point2);
}

inline void Item_Leveling_Point3(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Leveling_Menu_Point, LBLX, MBASE(row));
 DWIN_Frame_AreaCopy_ID1(Leveling_Menu_3, LBLX+Point_X_Coordinate[HMI_flag.language], MBASE(row));
 Draw_Menu_Line(row, ICON_Leveling_Point3);
}

inline void Item_Leveling_Point4(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Leveling_Menu_Point, LBLX, MBASE(row));
 DWIN_Frame_AreaCopy_ID1(Leveling_Menu_4, LBLX+Point_X_Coordinate[HMI_flag.language], MBASE(row));
 Draw_Menu_Line(row, ICON_Leveling_Point4);
}

#if ENABLED(ABL_GRID)
#if ENABLED(AUTO_UPDATA_PROBE_Z_OFFSET)
inline void Item_Leveling_CatechZoffset(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Leveling_Menu_Catch, LBLX, MBASE(row));
 DWIN_Frame_AreaCopy_ID1(Leveling_Menu_Z_Offset, LBLX+Catch_X_Coordinate[HMI_flag.language], MBASE(row));
 Draw_Menu_Line(row, ICON_Leveling_Save);
}
#endif
inline void Item_Leveling_ProbeZoffset(const uint8_t row) {
	DWIN_Frame_AreaCopy_ID1(Tune_Menu_Probe, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Tune_Menu_Z_Offset, LBLX+Probe_X_Coordinate[HMI_flag.language], MBASE(row));
	DWIN_Draw_Small_Float22(MENUVALUE_X, MBASE(row), probe.offset.z*MAXUNITMULT);
 	Draw_Menu_Line(row, ICON_Zoffset);
}
inline void Item_Leveling_Action(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Leveling_Menu_Auto, LBLX, MBASE(row));
 DWIN_Frame_AreaCopy_ID1(Leveling_Menu_Level, LBLX+Auto_X_Coordinate[HMI_flag.language], MBASE(row));
 if(planner.leveling_active)
		DWIN_Draw_MaskString_Default_Color(Color_Green,186, MBASE(row), F("Actived"));
 else
 		dwinLCD.Draw_String(false, true, font8x16, Color_Bg_Red, Color_Bg_Black, 170, MBASE(row), F("Unactived"));
 Draw_Menu_Line(row, ICON_Leveling_Save);
}
#endif

inline void Draw_Leveling_Menu() {
 Clear_Main_Window();
#if ENABLED(ABL_GRID)
	 HMI_flag.Leveling_Case_Total = HMI_flag.Leveling_Menu_Fg?LEVELING_CASE_TOTAL : LEVELING_CASE_POINT4;
	 HMI_flag.need_home_flag = true;
#endif

 const int16_t lscroll = MROWS - select_leveling.index; // Scrolled-up lines
 #define LSCROL(L) (lscroll + (L))
 #define LVISI(L) WITHIN(LSCROL(L), 0, MROWS)

 dwinLCD.JPG_CacheTo1(HMI_flag.Title_Menu_Backup);
 DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_LEVELING, TITLE_X, TITLE_Y);
 dwinLCD.JPG_CacheTo1(HMI_flag.language+1);

 if (LVISI(0)) Draw_Back_First(select_leveling.now == 0);             		 // < Back 
 if (LVISI(LEVELING_CASE_POINT1)) Item_Leveling_Point1(LSCROL(LEVELING_CASE_POINT1));   // point1
 if (LVISI(LEVELING_CASE_POINT2)) Item_Leveling_Point2(LSCROL(LEVELING_CASE_POINT2));   // point2
 if (LVISI(LEVELING_CASE_POINT3)) Item_Leveling_Point3(LSCROL(LEVELING_CASE_POINT3));   // point3
 if (LVISI(LEVELING_CASE_POINT4)) Item_Leveling_Point4(LSCROL(LEVELING_CASE_POINT4));   // point4

 #if ENABLED(ABL_GRID)
 	if(HMI_flag.Leveling_Menu_Fg){			
		#if ENABLED(AUTO_UPDATA_PROBE_Z_OFFSET)
		if (LVISI(LEVELING_CASE_CATCHOFFSET)) Item_Leveling_CatechZoffset(LSCROL(LEVELING_CASE_CATCHOFFSET));
		#endif
		if (LVISI(LEVELING_CASE_PROBEZOFFSET)) Item_Leveling_ProbeZoffset(LSCROL(LEVELING_CASE_PROBEZOFFSET));
		if (LVISI(LEVELING_CASE_ACTION)) Item_Leveling_Action(LSCROL(LEVELING_CASE_ACTION));
 	}
 #endif 
 if (select_leveling.now) Draw_Menu_Cursor(LSCROL(select_leveling.now));
}

//
// Prepare>>Language
//
inline void Item_Language_ZH(const uint8_t row) {
	DWIN_Frame_AreaCopy_ID1(MTSTRING_LANGUAGE_ZH, LBLX, MBASE(row));
	Draw_Menu_Line(row, ICON_CH);
}

inline void Item_Language_EN(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(MTSTRING_LANGUAGE_EN, LBLX, MBASE(row));
 Draw_Menu_Line(row, ICON_EN);
}

inline void Draw_Language_Menu() {
 Clear_Main_Window();
 dwinLCD.JPG_CacheTo1(HMI_flag.Title_Menu_Backup);
 DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_LANGUAGE, TITLE_X, TITLE_Y);
 dwinLCD.JPG_CacheTo1(HMI_flag.language+1);
 DWIN_Frame_AreaCopy_ID1(MTSTRING_LANGUAGE_EN, LBLX, MBASE(LANGUAGE_CASE_EN));
 DWIN_Frame_AreaCopy_ID1(MTSTRING_LANGUAGE_SP, LBLX, MBASE(LANGUAGE_CASE_SP));
 DWIN_Frame_AreaCopy_ID1(MTSTRING_LANGUAGE_RU, LBLX, MBASE(LANGUAGE_CASE_RU));
 DWIN_Frame_AreaCopy_ID1(MTSTRING_LANGUAGE_FR, LBLX, MBASE(LANGUAGE_CASE_FR));
 DWIN_Frame_AreaCopy_ID1(MTSTRING_LANGUAGE_PO, LBLX, MBASE(LANGUAGE_CASE_PO));

 Draw_Back_First(select_language.now == 0);
 if (select_language.now) Draw_Menu_Cursor(select_language.now);

 uint8_t i,j=ICON_EN;
 for(i=LANGUAGE_CASE_EN; i<LANGUAGE_CASE_TOTAL+1; i++,j++) {
 	Draw_Menu_Line(i,j);
 }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//Control menu
//
inline void Item_Control_Mixer(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Control_Menu_Mixer, LBLX, MBASE(row));
 Draw_Menu_Line(row, ICON_Mixer);
 Draw_More_Icon(row);
}

inline void Item_Control_Config(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Control_Menu_Config, LBLX, MBASE(row));
 Draw_Menu_Line(row, ICON_Contact);
 Draw_More_Icon(row);
}

inline void Item_Control_Motion(const uint8_t row) {
	DWIN_Frame_AreaCopy_ID1(Control_Menu_Motion, LBLX, MBASE(row));
	Draw_Menu_Line(row, ICON_Motion);
	Draw_More_Icon(row);
}

inline void Item_Control_PLA(const uint8_t row) {
	DWIN_Frame_AreaCopy_ID1(Prepare_Menu_Preheat, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Prepare_Menu_PLA, LBLX+Preheat_X_Coordinate[HMI_flag.language], MBASE(row));
	Draw_Menu_Line(row, ICON_PLAPreheat);
	Draw_More_Icon(row);
}

inline void Item_Control_ABS(const uint8_t row) {
	DWIN_Frame_AreaCopy_ID1(Prepare_Menu_Preheat, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Prepare_Menu_ABS, LBLX+Preheat_X_Coordinate[HMI_flag.language], MBASE(row));
	Draw_Menu_Line(row, ICON_ABSPreheat);
	Draw_More_Icon(row);	
}

#if ENABLED(BLTOUCH)
inline void Item_Control_BLtouch(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Control_Menu_Bltouch, LBLX, MBASE(row));
 //DWIN_Frame_AreaCopy_ID1(Control_Menu_Settings, LBLX+Load_X_Coordinate[HMI_flag.language], MBASE(row));
 Draw_Menu_Line(row, ICON_BLTouch);
}
#endif

#if ENABLED(EEPROM_SETTINGS)
inline void Item_Control_Save(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Control_Menu_Store, LBLX, MBASE(row));
 DWIN_Frame_AreaCopy_ID1(PLA_ABS_Menu_Settings, LBLX+Store_X_Coordinate[HMI_flag.language], MBASE(row));
 Draw_Menu_Line(row, ICON_WriteEEPROM);
}

inline void Item_Control_Load(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Control_Menu_Load, LBLX, MBASE(row));
 DWIN_Frame_AreaCopy_ID1(PLA_ABS_Menu_Settings, LBLX+Load_X_Coordinate[HMI_flag.language], MBASE(row));
 Draw_Menu_Line(row, ICON_ReadEEPROM);
}

inline void Item_Control_Reset(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Control_Menu_Reset, LBLX, MBASE(row));
 Draw_Menu_Line(row, ICON_ResumeEEPROM);
}
#endif
inline void Item_Control_Info(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Control_Menu_Info, LBLX, MBASE(row));
 Draw_Menu_Line(row, ICON_Info);
 Draw_More_Icon(row);
}

inline void Draw_Control_Menu() {
	Clear_Main_Window();
	Clear_Bottom_Area();

#if CONTROL_CASE_TOTAL >= 6
	const int16_t cscroll = MROWS - select_control.index; // Scrolled-up lines
	#define CCSCROL(L) (cscroll + (L))
#else
	#define CCSCROL(L) (L)
#endif
	#define CCLINE(L) MBASE(CCSCROL(L))
	#define CCVISI(L) WITHIN(CCSCROL(L), 0, MROWS)

	dwinLCD.JPG_CacheTo1(HMI_flag.Title_Menu_Backup);
	DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_CONTROL, TITLE_X, TITLE_Y);
	dwinLCD.JPG_CacheTo1(HMI_flag.language+1);
	if (CCVISI(0)) Draw_Back_First(select_control.now == 0);             // < Back

	if (CCVISI(CONTROL_CASE_MIXER)) Item_Control_Mixer(CCSCROL(CONTROL_CASE_MIXER));
	if (CCVISI(CONTROL_CASE_CONFIG)) Item_Control_Config(CCSCROL(CONTROL_CASE_CONFIG));
	if (CCVISI(CONTROL_CASE_MOTION)) Item_Control_Motion(CCSCROL(CONTROL_CASE_MOTION));
	if (CCVISI(CONTROL_CASE_SETPLA)) Item_Control_PLA(CCSCROL(CONTROL_CASE_SETPLA));
	if (CCVISI(CONTROL_CASE_SETABS)) Item_Control_ABS(CCSCROL(CONTROL_CASE_SETABS));

#if ENABLED(BLTOUCH)
	if (CCVISI(CONTROL_CASE_BLTOUCH)) Item_Control_BLtouch(CCSCROL(CONTROL_CASE_BLTOUCH));
#endif
#if ENABLED(EEPROM_SETTINGS)
	if (CCVISI(CONTROL_CASE_SAVE)) Item_Control_Save(CCSCROL(CONTROL_CASE_SAVE));
	if (CCVISI(CONTROL_CASE_LOAD)) Item_Control_Load(CCSCROL(CONTROL_CASE_LOAD));
	if (CCVISI(CONTROL_CASE_RESET)) Item_Control_Reset(CCSCROL(CONTROL_CASE_RESET));
#endif
	//if (CCVISI(CONTROL_CASE_INFO)) Item_Control_Info(CCSCROL(CONTROL_CASE_INFO));

	if (select_control.now )	Draw_Menu_Cursor(CCSCROL(select_control.now));	
}

//
// Control >> Mixer
//
inline void Draw_Mixer_Menu() {
 Clear_Main_Window();
 
 dwinLCD.JPG_CacheTo1(HMI_flag.Title_Menu_Backup);
 DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_MIXER, TITLE_X, TITLE_Y);
 dwinLCD.JPG_CacheTo1(HMI_flag.language+1);
 DWIN_Frame_AreaCopy_ID1(Mixer_Menu_Mix, LBLX, MBASE(MIXER_CASE_MANUAL));
 DWIN_Frame_AreaCopy_ID1(Mixer_Menu_Gradient, LBLX, MBASE(MIXER_CASE_AUTO));
 DWIN_Frame_AreaCopy_ID1(Mixer_Menu_Random, LBLX, MBASE(MIXER_CASE_RANDOM));
 DWIN_Frame_AreaCopy_ID1(Mixer_Menu_Current, LBLX, MBASE(MIXER_CASE_VTOOL));
 DWIN_Frame_AreaCopy_ID1(Mixer_Menu_Vtool, LBLX+Current_X_Coordinate[HMI_flag.language], MBASE(MIXER_CASE_VTOOL));

 Draw_Back_First(select_mixer.now == 0);
 if (select_mixer.now) Draw_Menu_Cursor(select_mixer.now);

 uint8_t i,j=ICON_Mixer_Manual;
 for(i=MIXER_CASE_MANUAL; i<MIXER_CASE_VTOOL; i++,j++) {
 	Draw_Menu_Line(i,j);
	Draw_More_Icon(i);
 }
 
 //Gradient
	DWIN_Draw_MaskString_Default(190, MBASE(MIXER_CASE_AUTO), F_STRING_ONOFF(mixer.gradient.enabled)); 
	DWIN_Draw_MaskString_Default(190, MBASE(MIXER_CASE_RANDOM), F_STRING_ONOFF(mixer.random_mix.enabled)); 
	Draw_Menu_Line(i++,ICON_S_VTOOL);
	DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(i-1), mixer.selected_vtool);
}

//
// Draw Mixer_Manual Windows
//
inline void Draw_Mixer_Manual_Menu() {
	Clear_Main_Window();

#if MANUAL_CASE_TOTAL >= 6
	const int16_t mscroll = MROWS - select_manual.index; // Scrolled-up lines
	#define MCSCROL(L) (mscroll + (L))
#else
	#define MCSCROL(L) (L)
#endif
	#define MCLINE(L) MBASE(MCSCROL(L))
	#define MCVISI(L) WITHIN(MCSCROL(L), 0, MROWS)

	dwinLCD.JPG_CacheTo1(HMI_flag.Title_Menu_Backup);
	DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_MIX, TITLE_X, TITLE_Y);
	dwinLCD.JPG_CacheTo1(HMI_flag.language+1);
	if (MCVISI(0)) Draw_Back_First(select_manual.now == 0);             // < Back

	DWIN_Frame_AreaCopy_ID1(Mix_Menu_Extruder, LBLX, MCLINE(MANUAL_CASE_EXTRUDER1));
	DWIN_Frame_AreaCopy_ID1(Mix_Menu_1, LBLX+Extruder_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_EXTRUDER1));
#if ENABLED (MIXING_EXTRUDER)
#if(MIXING_STEPPERS == 4)
	DWIN_Frame_AreaCopy_ID1(Mix_Menu_Extruder, LBLX, MCLINE(MANUAL_CASE_EXTRUDER2));
	DWIN_Frame_AreaCopy_ID1(Mix_Menu_2, LBLX+Extruder_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_EXTRUDER2));
	DWIN_Frame_AreaCopy_ID1(Mix_Menu_Extruder, LBLX, MCLINE(MANUAL_CASE_EXTRUDER3));
	DWIN_Frame_AreaCopy_ID1(Mix_Menu_3, LBLX+Extruder_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_EXTRUDER3));
	DWIN_Frame_AreaCopy_ID1(Mix_Menu_Extruder, LBLX, MCLINE(MANUAL_CASE_EXTRUDER4));
	DWIN_Frame_AreaCopy_ID1(Mix_Menu_4, LBLX+Extruder_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_EXTRUDER4));
	DWIN_Frame_AreaCopy_ID1(Mix_Menu_Comit, LBLX, MCLINE(MANUAL_CASE_OK));
	DWIN_Frame_AreaCopy_ID1(Mix_Menu_vtool, LBLX+Comit_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_OK));
	DWIN_Draw_UnMaskString_Default(176, MCLINE(MANUAL_CASE_OK), F("---->"));
	#elif(MIXING_STEPPERS == 3)
	DWIN_Frame_AreaCopy_ID1(Mix_Menu_Extruder, LBLX, MCLINE(MANUAL_CASE_EXTRUDER2));
	DWIN_Frame_AreaCopy_ID1(Mix_Menu_2, LBLX+Extruder_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_EXTRUDER2));
	DWIN_Frame_AreaCopy_ID1(Mix_Menu_Extruder, LBLX, MCLINE(MANUAL_CASE_EXTRUDER3));
	DWIN_Frame_AreaCopy_ID1(Mix_Menu_3, LBLX+Extruder_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_EXTRUDER3));
	DWIN_Frame_AreaCopy_ID1(Mix_Menu_Comit, LBLX, MCLINE(MANUAL_CASE_OK));
	DWIN_Frame_AreaCopy_ID1(Mix_Menu_vtool, LBLX+Comit_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_OK));
	DWIN_Draw_UnMaskString_Default(176, MCLINE(MANUAL_CASE_OK), F("---->"));
	#elif(MIXING_STEPPERS == 2)
	DWIN_Frame_AreaCopy_ID1(Mix_Menu_Extruder, LBLX, MCLINE(MANUAL_CASE_EXTRUDER2));
	DWIN_Frame_AreaCopy_ID1(Mix_Menu_2, LBLX+Extruder_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_EXTRUDER2));
	DWIN_Frame_AreaCopy_ID1(Mix_Menu_Comit, LBLX, MCLINE(MANUAL_CASE_OK));
	DWIN_Frame_AreaCopy_ID1(Mix_Menu_vtool, LBLX+Comit_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_OK));
	DWIN_Draw_UnMaskString_Default(176, MCLINE(MANUAL_CASE_OK), F("---->"));
	#endif
#endif
 
 Draw_Back_First(select_manual.now == 0);
 if (select_manual.now) Draw_Menu_Cursor(select_manual.now);

 uint8_t i,j=ICON_Extruder1;
 for(i=MANUAL_CASE_EXTRUDER1; i<MANUAL_CASE_TOTAL; i++,j++) {
 	Draw_Menu_Line(i,j);
	DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(i), mixer.mix[i-1]);
 }
 Draw_Menu_Line(i++,ICON_C_VTOOL);
 DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(i-1), mixer.selected_vtool);
}

//
// Draw Gradient Mixing Windows
//
inline void Draw_Mixer_Gradient_Menu() {
 Clear_Main_Window();

 #if AUTO_CASE_TOTAL >= 6
  const int16_t ascroll = MROWS - select_auto.index; // Scrolled-up lines
  #define ACSCROL(L) (ascroll + (L))
 #else
  #define ACSCROL(L) (L)
 #endif
 #define ACLINE(L) MBASE(ACSCROL(L))
 #define ACVISI(L) WITHIN(ACSCROL(L), 0, MROWS)

 dwinLCD.JPG_CacheTo1(HMI_flag.Title_Menu_Backup);
 DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_GRADIENT, TITLE_X, TITLE_Y);
 dwinLCD.JPG_CacheTo1(HMI_flag.language+1);
 if (ACVISI(0)) Draw_Back_First(select_auto.now == 0);             // < Back

 DWIN_Frame_AreaCopy_ID1(Gradient_Menu_Start, LBLX, ACLINE(AUTO_CASE_ZPOS_START));
 DWIN_Frame_AreaCopy_ID1(Gradient_Menu_Z, LBLX+Start_X_Coordinate[HMI_flag.language], ACLINE(AUTO_CASE_ZPOS_START));
 DWIN_Frame_AreaCopy_ID1(Gradient_Menu_End, LBLX, ACLINE(AUTO_CASE_ZPOS_END));
 DWIN_Frame_AreaCopy_ID1(Gradient_Menu_Z, LBLX+End_X_Coordinate[HMI_flag.language], ACLINE(AUTO_CASE_ZPOS_END));
 DWIN_Frame_AreaCopy_ID1(Gradient_Menu_Start, LBLX, ACLINE(AUTO_CASE_VTOOL_START));
 DWIN_Frame_AreaCopy_ID1(Gradient_Menu_Vtool, LBLX+Start_X_Coordinate[HMI_flag.language], ACLINE(AUTO_CASE_VTOOL_START));
 DWIN_Frame_AreaCopy_ID1(Gradient_Menu_End, LBLX, ACLINE(AUTO_CASE_VTOOL_END));
 DWIN_Frame_AreaCopy_ID1(Gradient_Menu_Vtool, LBLX+End_X_Coordinate[HMI_flag.language], ACLINE(AUTO_CASE_VTOOL_END));
 
 Draw_Back_First(select_auto.now == 0);
 if (select_auto.now) Draw_Menu_Cursor(select_auto.now);

 Draw_Menu_Line(AUTO_CASE_ZPOS_START,ICON_Zpos_Rise);
 HMI_ValueStruct.Auto_Zstart_scale = mixer.gradient.start_z*MINUNITMULT;
 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(1), HMI_ValueStruct.Auto_Zstart_scale);

 Draw_Menu_Line(AUTO_CASE_ZPOS_END,ICON_Zpos_Drop);
 HMI_ValueStruct.Auto_Zend_scale = mixer.gradient.end_z*MINUNITMULT;
 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(2), HMI_ValueStruct.Auto_Zend_scale);

 Draw_Menu_Line(AUTO_CASE_VTOOL_START,ICON_VTool_Rise);
 DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(3), mixer.gradient.start_vtool);

 Draw_Menu_Line(AUTO_CASE_VTOOL_END,ICON_VTool_Drop);
 DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(4), mixer.gradient.end_vtool);
 /*
 uint8_t i,j=ICON_Extruder1;
 for(i=AUTO_CASE_VTOOL_START; i<AUTO_CASE_TOTAL; i++,j++) {
 	Draw_Menu_Line(i,j);
 	DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(i), MixerCfg.Auto_Percent[mixer.selected_vtool][i-1]);
 }
 */
 //Draw_Menu_Line(i++,ICON_S_VTOOL);
 //DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(i-1), mixer.selected_vtool);
}


//
// Draw Randon Mixing Windows
//
inline void Draw_Mixer_Random_Menu() {
 Clear_Main_Window();

 #if RANDOM_CASE_TOTAL >= 6
  const int16_t rscroll = MROWS - select_random.index; // Scrolled-up lines
  #define RCSCROL(L) (rscroll + (L))
 #else
  #define RCSCROL(L) (L)
 #endif
 #define RCLINE(L) MBASE(RCSCROL(L))
 #define RCVISI(L) WITHIN(RCSCROL(L), 0, MROWS)

 dwinLCD.JPG_CacheTo1(HMI_flag.Title_Menu_Backup);
 DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_RANDOM, TITLE_X, TITLE_Y);
 dwinLCD.JPG_CacheTo1(HMI_flag.language+1);
 if (RCVISI(0)) Draw_Back_First(select_random.now == 0); // < Back

 DWIN_Frame_AreaCopy_ID1(Random_Menu_Start, LBLX, RCLINE(RANDOM_CASE_ZPOS_START));
 DWIN_Frame_AreaCopy_ID1(Random_Menu_Z, LBLX+Start_X_Coordinate[HMI_flag.language], RCLINE(RANDOM_CASE_ZPOS_START));
 DWIN_Frame_AreaCopy_ID1(Random_Menu_End, LBLX, RCLINE(RANDOM_CASE_ZPOS_END));
 DWIN_Frame_AreaCopy_ID1(Random_Menu_Z, LBLX+End_X_Coordinate[HMI_flag.language], RCLINE(RANDOM_CASE_ZPOS_END));

 Draw_Back_First(select_random.now == 0);
 if (select_random.now) Draw_Menu_Cursor(select_random.now);

 Draw_Menu_Line(RANDOM_CASE_ZPOS_START,ICON_Zpos_Rise);
 HMI_ValueStruct.Random_Zstart_scale = mixer.random_mix.start_z*MINUNITMULT;
 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(RANDOM_CASE_ZPOS_START), HMI_ValueStruct.Random_Zstart_scale);
 Draw_Menu_Line(RANDOM_CASE_ZPOS_END,ICON_Zpos_Drop);
 HMI_ValueStruct.Random_Zend_scale = mixer.random_mix.end_z*MINUNITMULT;
 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(RANDOM_CASE_ZPOS_END), HMI_ValueStruct.Random_Zend_scale);

 //Height
 Draw_Menu_Line(RANDOM_CASE_HEIGHT,ICON_Cursor);
 HMI_ValueStruct.Random_Height = mixer.random_mix.height*MINUNITMULT;
 DWIN_Draw_MaskString_Default(LBLX, MBASE(RANDOM_CASE_HEIGHT), F("Height:"));
 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(RANDOM_CASE_HEIGHT), HMI_ValueStruct.Random_Height);
 
 //Extruders
 Draw_Menu_Line(RANDOM_CASE_EXTRUDERS,ICON_Cursor);
 DWIN_Draw_MaskString_Default(LBLX, MBASE(RANDOM_CASE_EXTRUDERS), F("Extruders:"));
 DWIN_Draw_IntValue_Default(1, MENUVALUE_X+4*8, MBASE(RANDOM_CASE_EXTRUDERS), mixer.random_mix.extruders);
}

//
// Control >> Configure
//
#if ENABLED(FWRETRACT) 
inline void Item_Config_Retract(const uint8_t row) {
	DWIN_Draw_MaskString_Default(LBLX, MBASE(row), F("Auto Retract"));
	Draw_Menu_Line(row,ICON_Cursor);
	Draw_More_Icon(row);
}
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
inline void Item_Config_Filament(const uint8_t row) {
	DWIN_Draw_MaskString_Default(LBLX, MBASE(row), F("Runout Sensor:"));
	Draw_Menu_Line(row,ICON_Cursor);	
	DWIN_Draw_MaskString_Default(MENUONOFF_X, MBASE(row), F_STRING_ONOFF(runout.enabled));	
}
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
inline void Item_Config_Powerloss(const uint8_t row) {
	DWIN_Draw_MaskString_Default(LBLX, MBASE(row), F("PowerLossRecovery:"));
	Draw_Menu_Line(row,ICON_Cursor);
	DWIN_Draw_MaskString_Default(MENUONOFF_X, MBASE(row), F_STRING_ONOFF(recovery.enabled));	
}
#endif

#if ENABLED(OPTION_AUTOPOWEROFF)
inline void Item_Config_Shutdown(const uint8_t row) {
	DWIN_Draw_MaskString_Default(LBLX, MBASE(row), F("Auto Shutdown:"));
	Draw_Menu_Line(row,ICON_Cursor);
	DWIN_Draw_MaskString_Default(MENUONOFF_X, MBASE(row), F_STRING_ONOFF(HMI_flag.Autoshutdown_enabled));
}
#endif

#if ENABLED(OPTION_WIFI_MODULE)
inline void Item_Config_Wifi(const uint8_t row) {
 DWIN_Draw_MaskString_Default(LBLX, MBASE(row),F("Wifi:"));
 Draw_Menu_Line(row,ICON_Cursor);
 DWIN_Draw_MaskString_Default(MENUONOFF_X, MBASE(row), F_STRING_ONOFF(WiFi_Enabled)); 
}
#endif

#if ENABLED(OPTION_REPEAT_PRINTING)
inline void Item_Config_Reprint(const uint8_t row) {
	DWIN_Draw_MaskString_Default(LBLX, MBASE(row), F("Repeat printing:"));
	Draw_Menu_Line(row,ICON_Cursor);
	Draw_More_Icon(row);
}
#endif


#if ENABLED(OPTION_BED_COATING)
inline void Item_Config_bedcoating(const uint8_t row) { 
	HMI_ValueStruct.coating_thickness = (int16_t)(coating_thickness * MINUNITMULT);
	DWIN_Draw_MaskString_Default(LBLX, MBASE(row), F("HOME Z OFFSET:"));
	DWIN_Draw_Small_Float21(MENUVALUE_X-8, MBASE(row), HMI_ValueStruct.coating_thickness);
	Draw_Menu_Line(row,ICON_Cursor);
}
#endif

#if ENABLED(ABL_GRID)
#if ENABLED(AUTO_UPDATA_PROBE_Z_OFFSET)
inline void Item_Config_Leveling(const uint8_t row) {
	DWIN_Draw_MaskString_Default(LBLX, MBASE(row), F("Auto Leveling:"));
	DWIN_Draw_MaskString_Default(MENUONOFF_X, MBASE(row),F_STRING_ONOFF(HMI_flag.Leveling_Menu_Fg));	
	Draw_Menu_Line(row,ICON_Cursor);
}
#endif
inline void Item_Config_ActiveLevel(const uint8_t row) {
	DWIN_Draw_MaskString_Default(LBLX, MBASE(row), F("Active Autolevel:"));
	DWIN_Draw_MaskString_Default(MENUONOFF_X, MBASE(row), F_STRING_ONOFF(planner.leveling_active));		
	Draw_Menu_Line(row,ICON_Cursor);
}
#endif

#if ENABLED(DEBUG_GCODE_M92)
inline void Item_Config_M92(const uint8_t row) {
	DWIN_Draw_MaskString_Default(LBLX, MBASE(row), F("M92"));
	Draw_Menu_Line(row,ICON_Cursor);
}
#endif

inline void Draw_Config_Menu() {
	Clear_Main_Window();
	Clear_Bottom_Area();
	//uint8_t id = 0;

	const int16_t coscroll = MROWS - select_config.index; // Scrolled-up lines
	#define COSCROL(L) (coscroll + (L))
	#define COVISI(L) WITHIN(COSCROL(L), 0, MROWS)

	DWIN_Draw_UnMaskString_Default(14, 7, F("Configure"));
	Draw_Back_First(select_config.now == 0);

	if (COVISI(0)) Draw_Back_First(select_config.now == 0);             			// < Back 

	#if ENABLED(FWRETRACT) 
	if (COVISI(CONFIG_CASE_RETRACT)) 	Item_Config_Retract(COSCROL(CONFIG_CASE_RETRACT));   	// Retract >
	#endif 

	#if ENABLED(FILAMENT_RUNOUT_SENSOR) 
	if (COVISI(CONFIG_CASE_FILAMENT)) Item_Config_Filament(COSCROL(CONFIG_CASE_FILAMENT));  // filament runout
	#endif 

	#if ENABLED(POWER_LOSS_RECOVERY) 
	if (COVISI(CONFIG_CASE_POWERLOSS)) Item_Config_Powerloss(COSCROL(CONFIG_CASE_POWERLOSS));  // powerloss
	#endif

	#if ENABLED(OPTION_AUTOPOWEROFF) 
	if (COVISI(CONFIG_CASE_SHUTDOWN)) Item_Config_Shutdown(COSCROL(CONFIG_CASE_SHUTDOWN));  	 // shutdown
	#endif
	
 	if(!IS_SD_PRINTING() && !IS_SD_PAUSED()){
	 #if ENABLED(OPTION_WIFI_MODULE) 
	 if (COVISI(CONFIG_CASE_WIFI)) 	Item_Config_Wifi(COSCROL(CONFIG_CASE_WIFI));  	 			// WIFI
	 #endif

	 #if ENABLED(OPTION_REPEAT_PRINTING) 
	 if (COVISI(CONFIG_CASE_REPRINT)) 	Item_Config_Reprint(COSCROL(CONFIG_CASE_REPRINT));  	 	// repeat print
	 #endif

	 #if ENABLED(OPTION_BED_COATING) 
	 if (COVISI(CONFIG_CASE_COATING)) 	Item_Config_bedcoating(COSCROL(CONFIG_CASE_COATING));  	 	// GLASS LEVELING
	 #endif

	 #if ENABLED(ABL_GRID) 		
	 if (COVISI(CONFIG_CASE_LEVELING)) Item_Config_Leveling(COSCROL(CONFIG_CASE_LEVELING));  	 			// auto LEVELING
	 if (COVISI(CONFIG_CASE_ACTIVELEVEL)) Item_Config_ActiveLevel(COSCROL(CONFIG_CASE_ACTIVELEVEL)); // auto LEVELING
	 #endif
	 
	 #if ENABLED(DEBUG_GCODE_M92)
	 if (COVISI(CONFIG_CASE_M92)) 	Item_Config_M92(COSCROL(CONFIG_CASE_M92));  	 			// M92
	 #endif
 }
 if (select_config.now) Draw_Menu_Cursor(COSCROL(select_config.now));
}

//
// Control >> Config >> Auto retraction
//
#if ENABLED(FWRETRACT) 
inline void Item_Retract_Retract_Enabled(const uint8_t row) {
	DWIN_Draw_MaskString_Default(LBLX, MBASE(row), F("Auto-Retract:"));
	DWIN_Draw_MaskString_Default(MENUONOFF_X, MBASE(row), F_STRING_ONOFF(fwretract.autoretract_enabled));
	Draw_Menu_Line(row,ICON_Cursor);
}

inline void Item_Retract_Retract_mm(const uint8_t row) {
	DWIN_Draw_MaskString_Default(LBLX, MBASE(row), F("Retract MM:"));
	DWIN_Draw_Small_Float22(MENUVALUE_X, MBASE(row), fwretract.settings.retract_length*MAXUNITMULT);
	Draw_Menu_Line(row,ICON_Cursor);
}

inline void Item_Retract_Retract_V(const uint8_t row) {
	DWIN_Draw_MaskString_Default(LBLX, MBASE(row), F("Retract V:"));
	DWIN_Draw_Small_Float32(MENUVALUE_X-8, MBASE(row), fwretract.settings.retract_feedrate_mm_s*MAXUNITMULT);
	Draw_Menu_Line(row,ICON_Cursor);
}

inline void Item_Retract_Retract_ZHop(const uint8_t row) {
	DWIN_Draw_MaskString_Default(LBLX, MBASE(row), F("Retract ZHop:"));
	 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(row), fwretract.settings.retract_zraise*MINUNITMULT);
	Draw_Menu_Line(row,ICON_Cursor);
}

inline void Item_Retract_UnRetract_mm(const uint8_t row) {
	DWIN_Draw_MaskString_Default(LBLX, MBASE(row), F("UnRetract MM:"));
	DWIN_Draw_Small_Float22(MENUVALUE_X, MBASE(row), fwretract.settings.retract_recover_extra*MAXUNITMULT);
	Draw_Menu_Line(row,ICON_Cursor);
}

inline void Item_Retract_UnRetract_V(const uint8_t row) {
	DWIN_Draw_MaskString_Default(LBLX, MBASE(row), F("UnRetract V:"));
	DWIN_Draw_Small_Float32(MENUVALUE_X-8, MBASE(row), fwretract.settings.retract_recover_feedrate_mm_s*MAXUNITMULT);
	Draw_Menu_Line(row,ICON_Cursor);
}

inline void Draw_Retract_Menu() {
	Clear_Main_Window();

	//title
	DWIN_Draw_UnMaskString_Default(14, 7, F("Retract")); 

	//Item
	DWIN_Draw_MaskString_Default(LBLX, MBASE(RETRACT_CASE_AUTO), F("Auto-Retract:"));
	Draw_Menu_Line(RETRACT_CASE_AUTO,ICON_Cursor);
	DWIN_Draw_MaskString_Default(MENUONOFF_X, MBASE(RETRACT_CASE_AUTO), F_STRING_ONOFF(fwretract.autoretract_enabled));	

	DWIN_Draw_MaskString_Default(LBLX, MBASE(RETRACT_CASE_RETRACT_MM), F("Retract MM:"));
	Draw_Menu_Line(RETRACT_CASE_RETRACT_MM,ICON_Cursor);
	DWIN_Draw_Small_Float22(MENUVALUE_X, MBASE(RETRACT_CASE_RETRACT_MM), fwretract.settings.retract_length*MAXUNITMULT); 
	
	DWIN_Draw_MaskString_Default(LBLX, MBASE(RETRACT_CASE_RETRACT_V), F("Retract V:"));
	Draw_Menu_Line(RETRACT_CASE_RETRACT_V,ICON_Cursor);
	DWIN_Draw_Small_Float32(MENUVALUE_X-8, MBASE(RETRACT_CASE_RETRACT_V), fwretract.settings.retract_feedrate_mm_s*MAXUNITMULT);
	
	DWIN_Draw_MaskString_Default(LBLX, MBASE(RETRACT_CASE_RETRACT_ZHOP), F("Retract ZHop:"));
	Draw_Menu_Line(RETRACT_CASE_RETRACT_ZHOP,ICON_Cursor);
	DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(RETRACT_CASE_RETRACT_ZHOP), fwretract.settings.retract_zraise*MINUNITMULT); 
	
	DWIN_Draw_MaskString_Default(LBLX, MBASE(RETRACT_CASE_RECOVER_MM), F("UnRetract MM:"));
	Draw_Menu_Line(RETRACT_CASE_RECOVER_MM,ICON_Cursor);
	DWIN_Draw_Small_Float22(MENUVALUE_X, MBASE(RETRACT_CASE_RECOVER_MM), fwretract.settings.retract_recover_extra*MAXUNITMULT); 
		
	//DWIN_Draw_MaskString_Default(LBLX, MBASE(RETRACT_CASE_RECOVER_V), F("UnRetract V:"));
	//Draw_Menu_Line(RETRACT_CASE_RECOVER_V,ICON_Cursor);
	//DWIN_Draw_Small_Float32(MENUVALUE_X-8, MBASE(RETRACT_CASE_RECOVER_V), fwretract.settings.retract_recover_feedrate_mm_s*100);

	//Curor
	Draw_Back_First(select_retract.now == 0);
	if (select_retract.now) Draw_Menu_Cursor(select_retract.now);
}
#endif


//
// Control >> Config >> Repeat Printing
//
#if ENABLED(OPTION_REPEAT_PRINTING) 
inline void Item_Reprint_Enabled(const uint8_t row) {
	DWIN_Draw_MaskString_Default(LBLX, MBASE(row), F("Repeat Print"));
	Draw_Menu_Line(row,ICON_Cursor);
	DWIN_Draw_MaskString_Default(MENUONOFF_X, MBASE(row), F_STRING_ONOFF(ReprintManager.enabled));
}

inline void Item_Reprint_Times(const uint8_t row) {
	DWIN_Draw_MaskString_Default(LBLX, MBASE(row), F("Reprint Times"));
	Draw_Menu_Line(row,ICON_Cursor);
	DWIN_Draw_IntValue_Default(4, MENUVALUE_X, MBASE(row), ReprintManager.Reprint_times);
}

inline void Item_Reprint_Lenght(const uint8_t row) {
	DWIN_Draw_MaskString_Default(LBLX, MBASE(row), F("Forward Lenght"));
	Draw_Menu_Line(row,ICON_Cursor);
	DWIN_Draw_IntValue_Default(4, MENUVALUE_X, MBASE(row), ReprintManager.Forward_lenght);
}

inline void Item_Reprint_Reset(const uint8_t row) {
	DWIN_Draw_MaskString_Default(LBLX, MBASE(row), F("Reset"));
	Draw_Menu_Line(row,ICON_Cursor);
}

inline void Item_Reprint_Forward(const uint8_t row) {
	DWIN_Draw_MaskString_Default(LBLX, MBASE(row), F("Forward Move"));
	Draw_Menu_Line(row,ICON_Cursor);
}

inline void Item_Reprint_Back(const uint8_t row) {
	DWIN_Draw_MaskString_Default(LBLX, MBASE(row), F("Back Move"));
	Draw_Menu_Line(row,ICON_Cursor);
}

inline void Draw_Reprint_Menu() {
 Clear_Main_Window();

 const int16_t roscroll = MROWS - select_reprint.index; // Scrolled-up lines
 #define ROSCROL(L) (roscroll + (L))
 #define ROVISI(L) WITHIN(ROSCROL(L), 0, MROWS)

 DWIN_Draw_UnMaskString_Default(14, 7, F("Re-print"));
 Draw_Back_First(select_reprint.now == 0);

 if (ROVISI(0)) Draw_Back_First(select_reprint.now == 0);             				// < Back 
 if (ROVISI(REPRINT_CASE_ENABLED)) Item_Reprint_Enabled(ROSCROL(REPRINT_CASE_ENABLED));   	// ENABLED
 if (ROVISI(REPRINT_CASE_TIMES)) Item_Reprint_Times(ROSCROL(REPRINT_CASE_TIMES));  			// repeat print times
 if (ROVISI(REPRINT_CASE_LENGHT)) Item_Reprint_Lenght(ROSCROL(REPRINT_CASE_LENGHT)); 		 	// forward move lenght
 if (ROVISI(REPRINT_CASE_RESET)) Item_Reprint_Reset(ROSCROL(REPRINT_CASE_RESET)); 		 	// reset
 if (ROVISI(REPRINT_CASE_FORWARD)) Item_Reprint_Forward(ROSCROL(REPRINT_CASE_FORWARD)); 		// FORWARD
 if (ROVISI(REPRINT_CASE_BACK)) Item_Reprint_Back(ROSCROL(REPRINT_CASE_BACK)); 		 	  // BACK
 if (select_reprint.now) Draw_Menu_Cursor(ROSCROL(select_reprint.now));
}
#endif


//
// Control >> Motion >> Feedrate
//
inline void Item_Feedrate_MaxX(uint8_t row){
	Draw_Menu_Line(row, ICON_MaxSpeedX);
	DWIN_Frame_AreaCopy_ID1(Feedrate_Menu_Max, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Feedrate_Menu_Feedrate, LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Feedrate_Menu_X, LBLX+Max_X_Coordinate[HMI_flag.language]+Feedrate_X_Coordinate[HMI_flag.language], MBASE(row));	
	DWIN_Draw_IntValue_Default(4,CONFIGVALUE_X, MBASE(row), planner.settings.max_feedrate_mm_s[X_AXIS]);
}

inline void Item_Feedrate_MaxY(uint8_t row){
	Draw_Menu_Line(row, ICON_MaxSpeedY);
	DWIN_Frame_AreaCopy_ID1(Feedrate_Menu_Max, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Feedrate_Menu_Feedrate, LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Feedrate_Menu_Y, LBLX+Max_X_Coordinate[HMI_flag.language]+Feedrate_X_Coordinate[HMI_flag.language], MBASE(row));	
	DWIN_Draw_IntValue_Default(4,CONFIGVALUE_X, MBASE(row), planner.settings.max_feedrate_mm_s[Y_AXIS]);
}

inline void Item_Feedrate_MaxZ(uint8_t row){
	Draw_Menu_Line(row, ICON_MaxSpeedZ);
	DWIN_Frame_AreaCopy_ID1(Feedrate_Menu_Max, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Feedrate_Menu_Feedrate, LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Feedrate_Menu_Z, LBLX+Max_X_Coordinate[HMI_flag.language]+Feedrate_X_Coordinate[HMI_flag.language], MBASE(row));	
	DWIN_Draw_IntValue_Default(4,CONFIGVALUE_X, MBASE(row), planner.settings.max_feedrate_mm_s[Z_AXIS]);
}

inline void Item_Feedrate_MaxE(uint8_t row){
	Draw_Menu_Line(row, ICON_MaxSpeedE);
	DWIN_Frame_AreaCopy_ID1(Feedrate_Menu_Max, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Feedrate_Menu_Feedrate, LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Feedrate_Menu_E, LBLX+Max_X_Coordinate[HMI_flag.language]+Feedrate_X_Coordinate[HMI_flag.language], MBASE(row));	
	DWIN_Draw_IntValue_Default(4,CONFIGVALUE_X, MBASE(row), planner.settings.max_feedrate_mm_s[E_AXIS]);
}

inline void Draw_Max_Speed_Menu() {
	Clear_Main_Window();

	dwinLCD.JPG_CacheTo1(HMI_flag.Title_Menu_Backup);
	DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_FEEDRATE, TITLE_X, TITLE_Y);
	dwinLCD.JPG_CacheTo1(HMI_flag.language+1);
	Draw_Back_First(select_feedrate.now == 0);	
	Item_Feedrate_MaxX(1);
	Item_Feedrate_MaxY(2);
	Item_Feedrate_MaxZ(3);
	Item_Feedrate_MaxE(4);
	if (select_feedrate.now) Draw_Menu_Cursor(select_feedrate.now);
}

//
// Control >> Motion >> Acceleration
//
inline void Item_Accel_MaxX(uint8_t row){
	Draw_Menu_Line(row, ICON_MaxAccX);
	DWIN_Frame_AreaCopy_ID1(Accel_Menu_Max, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Accel_Menu_Accel, LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Accel_Menu_X, LBLX+Max_X_Coordinate[HMI_flag.language]+Accel_X_Coordinate[HMI_flag.language]+6, MBASE(row));	
	DWIN_Draw_IntValue_Default(4,CONFIGVALUE_X, MBASE(row), planner.settings.max_acceleration_mm_per_s2[X_AXIS]);
}

inline void Item_Accel_MaxY(uint8_t row){
	Draw_Menu_Line(row, ICON_MaxAccY);
	DWIN_Frame_AreaCopy_ID1(Accel_Menu_Max, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Accel_Menu_Accel, LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Accel_Menu_Y, LBLX+Max_X_Coordinate[HMI_flag.language]+Accel_X_Coordinate[HMI_flag.language]+6, MBASE(row));	
	DWIN_Draw_IntValue_Default(4,CONFIGVALUE_X, MBASE(row), planner.settings.max_acceleration_mm_per_s2[Y_AXIS]);
}

inline void Item_Accel_MaxZ(uint8_t row){
	Draw_Menu_Line(row, ICON_MaxAccZ);
	DWIN_Frame_AreaCopy_ID1(Accel_Menu_Max, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Accel_Menu_Accel, LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Accel_Menu_Z, LBLX+Max_X_Coordinate[HMI_flag.language]+Accel_X_Coordinate[HMI_flag.language]+6, MBASE(row));	
	DWIN_Draw_IntValue_Default(4,CONFIGVALUE_X, MBASE(row), planner.settings.max_acceleration_mm_per_s2[Z_AXIS]);
}

inline void Item_Accel_MaxE(uint8_t row){
	Draw_Menu_Line(row, ICON_MaxAccE);
	DWIN_Frame_AreaCopy_ID1(Accel_Menu_Max, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Accel_Menu_Accel, LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Accel_Menu_E, LBLX+Max_X_Coordinate[HMI_flag.language]+Accel_X_Coordinate[HMI_flag.language]+6, MBASE(row));	
	DWIN_Draw_IntValue_Default(5,CONFIGVALUE_X-8, MBASE(row), planner.settings.max_acceleration_mm_per_s2[E_AXIS]);
}

inline void Draw_Max_Accel_Menu() {
	Clear_Main_Window();

	dwinLCD.JPG_CacheTo1(HMI_flag.Title_Menu_Backup);
	DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_ACCEL, TITLE_X, TITLE_Y);
	dwinLCD.JPG_CacheTo1(HMI_flag.language+1);
	Draw_Back_First(select_accel.now == 0);
	Item_Accel_MaxX(1);
	Item_Accel_MaxY(2);
	Item_Accel_MaxZ(3);
	Item_Accel_MaxE(4);
	if (select_accel.now) Draw_Menu_Cursor(select_accel.now);
}

//
// Control >> Motion >> Jerk
//
inline void Item_Jerk_MaxX(uint8_t row){
	Draw_Menu_Line(row, ICON_MaxSpeedJerkX);
	DWIN_Frame_AreaCopy_ID1(Jerk_Menu_Max, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Jerk_Menu_Jerk, LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Jerk_Menu_X, LBLX+Max_X_Coordinate[HMI_flag.language]+Jerk_X_Coordinate[HMI_flag.language]+6, MBASE(row));	
	DWIN_Draw_Small_Float21(CONFIGVALUE_X+8, MBASE(row), planner.max_jerk[X_AXIS] * MINUNITMULT);
}

inline void Item_Jerk_MaxY(uint8_t row){
	Draw_Menu_Line(row, ICON_MaxSpeedJerkY);
	DWIN_Frame_AreaCopy_ID1(Jerk_Menu_Max, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Jerk_Menu_Jerk, LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Jerk_Menu_Y, LBLX+Max_X_Coordinate[HMI_flag.language]+Jerk_X_Coordinate[HMI_flag.language]+6, MBASE(row));	
	DWIN_Draw_Small_Float21(CONFIGVALUE_X+8, MBASE(row), planner.max_jerk[Y_AXIS] * MINUNITMULT);
}

inline void Item_Jerk_MaxZ(uint8_t row){
	Draw_Menu_Line(row, ICON_MaxSpeedJerkZ);
	DWIN_Frame_AreaCopy_ID1(Jerk_Menu_Max, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Jerk_Menu_Jerk, LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Jerk_Menu_Z, LBLX+Max_X_Coordinate[HMI_flag.language]+Jerk_X_Coordinate[HMI_flag.language]+6, MBASE(row));	
	DWIN_Draw_Small_Float21(CONFIGVALUE_X+8, MBASE(row), planner.max_jerk[Z_AXIS] * MINUNITMULT);
}

inline void Item_Jerk_MaxE(uint8_t row){
	Draw_Menu_Line(row, ICON_MaxSpeedJerkE);
	DWIN_Frame_AreaCopy_ID1(Jerk_Menu_Max, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Jerk_Menu_Jerk, LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Jerk_Menu_E, LBLX+Max_X_Coordinate[HMI_flag.language]+Jerk_X_Coordinate[HMI_flag.language]+6, MBASE(row));	
	DWIN_Draw_Small_Float21(CONFIGVALUE_X+8, MBASE(row), planner.max_jerk[E_AXIS] * MINUNITMULT);
}

inline void Draw_Max_Jerk_Menu() {
	Clear_Main_Window();

	dwinLCD.JPG_CacheTo1(HMI_flag.Title_Menu_Backup);
	DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_JERK, TITLE_X, TITLE_Y);
	dwinLCD.JPG_CacheTo1(HMI_flag.language+1);
	Draw_Back_First(select_jerk.now == 0);
	Item_Jerk_MaxX(1);
	Item_Jerk_MaxY(2);
	Item_Jerk_MaxZ(3);
	Item_Jerk_MaxE(4);
	if (select_jerk.now) Draw_Menu_Cursor(select_jerk.now); 
}

//
// Control >> Motion >> Step/mm
//
inline void Item_Steps_X(uint8_t row){
	Draw_Menu_Line(row, ICON_StepX);
	DWIN_Frame_AreaCopy_ID1(Step_Menu_Step, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Step_Menu_X, LBLX+Step_X_Coordinate[HMI_flag.language]+6, MBASE(row));
	DWIN_Draw_Small_Float41(CONFIGVALUE_X-8, MBASE(row), planner.settings.axis_steps_per_mm[X_AXIS] * MINUNITMULT);
}

inline void Item_Steps_Y(uint8_t row){
	Draw_Menu_Line(row, ICON_StepY);
	DWIN_Frame_AreaCopy_ID1(Step_Menu_Step, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Step_Menu_Y, LBLX+Step_X_Coordinate[HMI_flag.language]+6, MBASE(row));
	DWIN_Draw_Small_Float41(CONFIGVALUE_X-8, MBASE(row), planner.settings.axis_steps_per_mm[Y_AXIS] * MINUNITMULT);
}

inline void Item_Steps_Z(uint8_t row){
	Draw_Menu_Line(row, ICON_StepZ);
	DWIN_Frame_AreaCopy_ID1(Step_Menu_Step, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Step_Menu_Z, LBLX+Step_X_Coordinate[HMI_flag.language]+6, MBASE(row));
	DWIN_Draw_Small_Float41(CONFIGVALUE_X-8, MBASE(row), planner.settings.axis_steps_per_mm[Z_AXIS] * MINUNITMULT);
}

inline void Item_Steps_E(uint8_t row){
	Draw_Menu_Line(row, ICON_StepE);
	DWIN_Frame_AreaCopy_ID1(Step_Menu_Step, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Step_Menu_E, LBLX+Step_X_Coordinate[HMI_flag.language]+6, MBASE(row));
	DWIN_Draw_Small_Float41(CONFIGVALUE_X-8, MBASE(row), planner.settings.axis_steps_per_mm[E_AXIS] * MINUNITMULT);
}

inline void Draw_Steps_Menu() {
	Clear_Main_Window();

	dwinLCD.JPG_CacheTo1(HMI_flag.Title_Menu_Backup);
	DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_STEP, TITLE_X, TITLE_Y);
	dwinLCD.JPG_CacheTo1(HMI_flag.language+1);
	Draw_Back_First(select_step.now == 0);
	Item_Steps_X(1);
	Item_Steps_Y(2);
	Item_Steps_Z(3);
	Item_Steps_E(4);
	if (select_step.now) Draw_Menu_Cursor(select_step.now); 
}


//
// Control >> Motion
//
inline void Item_Motion_Feedrate(uint8_t row){
	Draw_Menu_Line(row, ICON_MaxSpeed);
	DWIN_Frame_AreaCopy_ID1(Motion_Menu_Feedrate, LBLX, MBASE(row));
	Draw_More_Icon(row);
}

inline void Item_Motion_Accel(uint8_t row){
	Draw_Menu_Line(row, ICON_MaxAccelerated);
	DWIN_Frame_AreaCopy_ID1(Motion_Menu_Acc, LBLX, MBASE(row));
	Draw_More_Icon(row);
}

inline void Item_Motion_Jerk(uint8_t row){
	Draw_Menu_Line(row, ICON_MaxJerk);
	DWIN_Frame_AreaCopy_ID1(Motion_Menu_Jerk, LBLX, MBASE(row));
	Draw_More_Icon(row);
}

inline void Item_Motion_Steps(uint8_t row){
	Draw_Menu_Line(row, ICON_Step);
	DWIN_Frame_AreaCopy_ID1(Motion_Menu_Steps, LBLX, MBASE(row));
	Draw_More_Icon(row);
}

inline void Draw_Motion_Menu() {
	
	Clear_Main_Window();

	dwinLCD.JPG_CacheTo1(HMI_flag.Title_Menu_Backup);
	DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_MOTION, TITLE_X, TITLE_Y);
	dwinLCD.JPG_CacheTo1(HMI_flag.language+1);
	Draw_Back_First(select_motion.now == 0);
	Item_Motion_Feedrate(1);
	Item_Motion_Accel(2);
	Item_Motion_Jerk(3);
	Item_Motion_Steps(4);	
	if (select_motion.now) Draw_Menu_Cursor(select_motion.now);
}


//
// Control >> PreHeat PLA
//
inline void Draw_SetPreHeatPLA_Menu(){
	Clear_Main_Window();

	dwinLCD.JPG_CacheTo1(HMI_flag.Title_Menu_Backup);
	DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_PLA, TITLE_X, TITLE_Y);
	dwinLCD.JPG_CacheTo1(HMI_flag.language+1);
	DWIN_Frame_AreaCopy_ID1(PLA_ABS_Menu_Nozzle, LBLX, MBASE(PREHEAT_CASE_TEMP));
	DWIN_Frame_AreaCopy_ID1(PLA_ABS_Menu_Temp, LBLX+58, MBASE(PREHEAT_CASE_TEMP));
#if HAS_HEATED_BED
	DWIN_Frame_AreaCopy_ID1(PLA_ABS_Menu_Bed, LBLX, MBASE(PREHEAT_CASE_BED));
	DWIN_Frame_AreaCopy_ID1(PLA_ABS_Menu_Temp, LBLX+33, MBASE(PREHEAT_CASE_BED));
#endif
#if HAS_FAN
	DWIN_Frame_AreaCopy_ID1(PLA_ABS_Menu_Fan_Speed, LBLX, MBASE(PREHEAT_CASE_FAN));
#endif
#if ENABLED(EEPROM_SETTINGS)
	DWIN_Frame_AreaCopy_ID1(PLA_ABS_Menu_Store, LBLX, MBASE(PREHEAT_CASE_SAVE));
	DWIN_Frame_AreaCopy_ID1(PLA_ABS_Menu_Settings, LBLX+50, MBASE(PREHEAT_CASE_SAVE));
#endif     
	Draw_Back_First();
	uint8_t i = 0;
	Draw_Menu_Line(++i, ICON_SetEndTemp);
	DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(i), ui.material_preset[0].hotend_temp);
#if HAS_HEATED_BED
	Draw_Menu_Line(++i, ICON_SetBedTemp);
	DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(i), ui.material_preset[0].bed_temp);
#endif
#if HAS_FAN
	Draw_Menu_Line(++i, ICON_FanSpeed);
	DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(i), ui.material_preset[0].fan_speed);
#endif
#if ENABLED(EEPROM_SETTINGS)
	Draw_Menu_Line(++i, ICON_WriteEEPROM);
#endif
} 

//
// Control >> PreHeat ABS
//
inline void Draw_SetPreHeatABS_Menu(){
	Clear_Main_Window();

	dwinLCD.JPG_CacheTo1(HMI_flag.Title_Menu_Backup);
	DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_ABS, TITLE_X, TITLE_Y);
	dwinLCD.JPG_CacheTo1(HMI_flag.language+1);
	DWIN_Frame_AreaCopy_ID1(PLA_ABS_Menu_Nozzle, LBLX, MBASE(PREHEAT_CASE_TEMP));
	DWIN_Frame_AreaCopy_ID1(PLA_ABS_Menu_Temp, LBLX+58, MBASE(PREHEAT_CASE_TEMP));
#if HAS_HEATED_BED
	DWIN_Frame_AreaCopy_ID1(PLA_ABS_Menu_Bed, LBLX, MBASE(PREHEAT_CASE_BED));
	DWIN_Frame_AreaCopy_ID1(PLA_ABS_Menu_Temp, LBLX+33, MBASE(PREHEAT_CASE_BED));
#endif
#if HAS_FAN
	DWIN_Frame_AreaCopy_ID1(PLA_ABS_Menu_Fan_Speed, LBLX, MBASE(PREHEAT_CASE_FAN));
#endif
#if ENABLED(EEPROM_SETTINGS)
	DWIN_Frame_AreaCopy_ID1(PLA_ABS_Menu_Store, LBLX, MBASE(PREHEAT_CASE_SAVE));
	DWIN_Frame_AreaCopy_ID1(PLA_ABS_Menu_Settings, LBLX+50, MBASE(PREHEAT_CASE_SAVE));
#endif     
	Draw_Back_First();

	uint8_t i = 0;
	Draw_Menu_Line(++i, ICON_SetEndTemp);
	DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(i), ui.material_preset[1].hotend_temp);
#if HAS_HEATED_BED
	Draw_Menu_Line(++i, ICON_SetBedTemp);
	DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(i), ui.material_preset[1].bed_temp);
#endif
#if HAS_FAN
	Draw_Menu_Line(++i, ICON_FanSpeed);
	DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(i), ui.material_preset[1].fan_speed);
#endif
#if ENABLED(EEPROM_SETTINGS)
	Draw_Menu_Line(++i, ICON_WriteEEPROM);
#endif
}


//
// Control >> BLTouch
//
#if ENABLED(BLTOUCH)
inline void Draw_Bltouch_Menu() {
 Clear_Main_Window();
 dwinLCD.JPG_CacheTo1(HMI_flag.Title_Menu_Backup);
 DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_BLTOUCH, TITLE_X, TITLE_Y);
 dwinLCD.JPG_CacheTo1(HMI_flag.language+1);
 DWIN_Frame_AreaCopy_ID1(BLTouch_Menu_Reset, LBLX, MBASE(BLTOUCH_CASE_RESET));
 DWIN_Frame_AreaCopy_ID1(BLTouch_Menu_Test, LBLX, MBASE(BLTOUCH_CASE_TEST));
 DWIN_Frame_AreaCopy_ID1(BLTouch_Menu_Stow, LBLX, MBASE(BLTOUCH_CASE_STOW));
 DWIN_Frame_AreaCopy_ID1(BLTouch_Menu_Deploy, LBLX, MBASE(BLTOUCH_CASE_DEPLOY));
 DWIN_Frame_AreaCopy_ID1(BLTouch_Menu_Mode, LBLX, MBASE(BLTOUCH_CASE_SW));

 Draw_Back_First(select_bltouch.now == 0);
 if (select_bltouch.now) Draw_Menu_Cursor(select_bltouch.now);

 uint8_t i,j=ICON_BLTOUCH_RESET;
 for(i=BLTOUCH_CASE_RESET; i<BLTOUCH_CASE_TOTAL+1; i++,j++) {
 	Draw_Menu_Line(i,j);
 }
}
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Info menu
//
inline void Item_Info_Version(const uint8_t row) {
 DWIN_Draw_UnMaskString_Default(LBLX_INFO, MBASE(row), F("Version:"));
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("Version:")+1)*MENU_CHR_W, MBASE(row),  F(FIRMWARE_VERSION));
}

inline void Item_Info_Firmware(const uint8_t row) {
 DWIN_Draw_UnMaskString_Default(LBLX_INFO, MBASE(row), F("Firmware:"));
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("Firmware:")+1)*MENU_CHR_W, MBASE(row),  F(SHORT_BUILD_VERSION));
}

inline void Item_Info_Website(const uint8_t row) {
 DWIN_Draw_UnMaskString_Default(LBLX_INFO, MBASE(row), F("Website:"));
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("Website:")+1)*MENU_CHR_W, MBASE(row), F(WEBSITE_URL));
}

inline void Item_Info_Model(const uint8_t row) {
 DWIN_Draw_UnMaskString_Default(LBLX_INFO, MBASE(row), F("Model:"));
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("Model:")+1)*MENU_CHR_W, MBASE(row), F(CUSTOM_MACHINE_NAME));
}

inline void Item_Info_Board(const uint8_t row) {
 DWIN_Draw_UnMaskString_Default(LBLX_INFO, MBASE(row), F("Board:"));
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("Board:")+1)*MENU_CHR_W, MBASE(row), F(BOARD_INFO_NAME));
}

inline void Item_Info_Extruder_Num(const uint8_t row) {
 DWIN_Draw_UnMaskString_Default(LBLX_INFO, MBASE(row), F("Extruder Num:"));
 #if ENABLED(MIXING_EXTRUDER) 
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("Extruder Num:")+1)*MENU_CHR_W, MBASE(row),F(STRINGIFY(MIXING_STEPPERS)));
 #else
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("Extruder Num:")+1)*MENU_CHR_W, MBASE(row),F(STRINGIFY(EXTRUDERS)));
 #endif
}

#if ENABLED(OPTION_BGM)
#define EXTRUDER_MODEL				"BGM"
#else
#define EXTRUDER_MODEL				"Titan"
#endif
inline void Item_Info_Extruder_Model(const uint8_t row) {
 DWIN_Draw_UnMaskString_Default(LBLX_INFO, MBASE(row), F("Extruder Model:"));
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("Extruder Model:")+1)*MENU_CHR_W, MBASE(row), F(EXTRUDER_MODEL));
}


#if ENABLED(OPTION_DUALZ_DRIVE)
#define Z_Drive						"DUAL Z"
inline void Item_Info_DualZ_Drive(const uint8_t row) {
 DWIN_Draw_UnMaskString_Default(LBLX_INFO, MBASE(row), F("Z Drive:"));
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("Z Drive:")+1)*MENU_CHR_W, MBASE(row), F(Z_Drive));
}
#endif

#if ENABLED(OPTION_Z2_ENDSTOP)
#define Z_Endstop					"DUAL Endstop"
inline void Item_Info_DualZ_Endstop(const uint8_t row) {
 DWIN_Draw_UnMaskString_Default(LBLX_INFO, MBASE(row), F("Z Endstop:"));
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("Z Endstop:")+1)*MENU_CHR_W, MBASE(row), F(Z_Endstop));
}
#endif

inline void Item_Info_Baudrate(const uint8_t row) {
 DWIN_Draw_UnMaskString_Default(LBLX_INFO, MBASE(row), F("Baudrate:"));
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("Baudrate:")+1)*MENU_CHR_W, MBASE(row), F(STRINGIFY(BAUDRATE)));
}

inline void Item_Info_Protocol(const uint8_t row) {
 DWIN_Draw_UnMaskString_Default(LBLX_INFO, MBASE(row), F("Protocol:"));
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("Protocol:")+1)*MENU_CHR_W, MBASE(row), F(PROTOCOL_VERSION));
}

inline void Item_Info_Psu(const uint8_t row) {
 DWIN_Draw_UnMaskString_Default(LBLX_INFO, MBASE(row), F("PSU:"));
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("PSU:")+1)*MENU_CHR_W, MBASE(row), F(PSU_NAME));
}

inline void Item_Info_Date(const uint8_t row) {
 DWIN_Draw_UnMaskString_Default(LBLX_INFO, MBASE(row), F("Date:"));
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("Date:")+1)*MENU_CHR_W, MBASE(row), F(STRING_DISTRIBUTION_DATE));
}

inline void Item_Info_Thermistor(const uint8_t row) {
 DWIN_Draw_UnMaskString_Default(LBLX_INFO, MBASE(row), F("Thermistor:"));
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("Thermistor:")+1)*MENU_CHR_W, MBASE(row), F("100k with 4.7k pull-up"));
 //DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("Thermistor:")+1)*MENU_CHR_W, MBASE(row), (char*)STRINGIFY(THERMISTOR_HEATER_0));
}

inline void Item_Info_Bed(const uint8_t row) {
 DWIN_Draw_UnMaskString_Default(LBLX_INFO, MBASE(row), F("Hot bed:"));
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("Hot bed:"))*MENU_CHR_W, MBASE(row), F("MINTEMP:"));
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("MINTEMP:") + strlen("Hot bed:"))*MENU_CHR_W, MBASE(row), F(STRINGIFY(BED_MINTEMP)));
 DWIN_Draw_UnMaskString_Default(175, MBASE(row), F("MAXTEMP:"));
 DWIN_Draw_UnMaskString_Default(175 + (strlen("MAXTEMP:"))*MENU_CHR_W, MBASE(row), F(STRINGIFY(BED_MAXTEMP)));
}

inline void Item_Info_Hot(const uint8_t row) {
 DWIN_Draw_UnMaskString_Default(LBLX_INFO, MBASE(row), F("Hot end:"));
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("Hot end:"))*MENU_CHR_W, MBASE(row), F("MINTEMP:"));
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("MINTEMP:")+strlen("Hot end:"))*MENU_CHR_W, MBASE(row), F(STRINGIFY(HEATER_0_MINTEMP)));
 DWIN_Draw_UnMaskString_Default(175, MBASE(row), F("MAXTEMP:"));
 DWIN_Draw_UnMaskString_Default(175 + (strlen("MAXTEMP:"))*MENU_CHR_W, MBASE(row), F(STRINGIFY(HEATER_0_MAXTEMP)));
}

inline void Draw_Info_Menu() {
 Clear_Main_Window();

 #if INFO_CASE_TOTAL >= 6
  const int16_t iscroll = MROWS - select_info.index; // Scrolled-up lines
  #define ICSCROL(L) (iscroll + (L))
 #else
  #define ICSCROL(L) (L)
 #endif
 #define ICLINE(L) MBASE(ICSCROL(L))
 #define ICVISI(L) WITHIN(ICSCROL(L), 0, MROWS)
 
 dwinLCD.JPG_CacheTo1(HMI_flag.Title_Menu_Backup);
 DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_INFO, TITLE_X, TITLE_Y);
 dwinLCD.JPG_CacheTo1(HMI_flag.language+1);
 if (ICVISI(0)) Draw_Back_First(select_info.now == 0);             						// < Back

 DWIN_Draw_UnMaskString_Default(LBLX_INFO, ICLINE(INFO_CASE_VERSION), F("Version:"));
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("Version:")+1)*MENU_CHR_W, ICLINE(INFO_CASE_VERSION), F(FIRMWARE_VERSION));  	
 dwinLCD.Draw_Line(Line_Color, 16, ICLINE(INFO_CASE_VERSION) + 33, 256, ICLINE(INFO_CASE_VERSION) + 34);
 DWIN_Draw_UnMaskString_Default(LBLX_INFO, ICLINE(INFO_CASE_FIRMWARE), F("Firmware:"));
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("Firmware:")+1)*MENU_CHR_W, ICLINE(INFO_CASE_FIRMWARE), F(SHORT_BUILD_VERSION)); 
 dwinLCD.Draw_Line(Line_Color, 16, ICLINE(INFO_CASE_FIRMWARE) + 33, 256, ICLINE(INFO_CASE_FIRMWARE) + 34);
 DWIN_Draw_UnMaskString_Default(LBLX_INFO, ICLINE(INFO_CASE_WEBSITE), F("Website:"));
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("Website:")+1)*MENU_CHR_W, ICLINE(INFO_CASE_WEBSITE), F(WEBSITE_URL)); 
 dwinLCD.Draw_Line(Line_Color, 16, ICLINE(INFO_CASE_WEBSITE) + 33, 256, ICLINE(INFO_CASE_WEBSITE) + 34);
 DWIN_Draw_UnMaskString_Default(LBLX_INFO, ICLINE(INFO_CASE_MODEL), F("Model:"));
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("Model:")+1)*MENU_CHR_W, ICLINE(INFO_CASE_MODEL), F(CUSTOM_MACHINE_NAME)); 
 dwinLCD.Draw_Line(Line_Color, 16, ICLINE(INFO_CASE_MODEL) + 33, 256, ICLINE(INFO_CASE_MODEL) + 34);
 DWIN_Draw_UnMaskString_Default(LBLX_INFO, ICLINE(INFO_CASE_BOARD), F("Board:"));
 DWIN_Draw_UnMaskString_Default(LBLX_INFO + (strlen("Board:")+1)*MENU_CHR_W, ICLINE(INFO_CASE_BOARD), F(BOARD_INFO_NAME)); 
 dwinLCD.Draw_Line(Line_Color, 16, ICLINE(INFO_CASE_BOARD) + 33, 256, ICLINE(INFO_CASE_BOARD) + 34);
 if (select_info.now && ICVISI(select_info.now))
  Draw_Menu_Cursor(ICSCROL(select_info.now));  
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//Tune
//
inline void Item_Tune_Speed(const uint8_t row) {
	DWIN_Frame_AreaCopy_ID1(Tune_Menu_Speed, LBLX, MBASE(row));
	Draw_Menu_Line(row, ICON_Setspeed);
	DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(row), feedrate_percentage);
}

inline void Item_Tune_ETemp(const uint8_t row){
	DWIN_Frame_AreaCopy_ID1(Tune_Menu_Hotend, LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_ID1(Tune_Menu_Temp, LBLX+Hotend_X_Coordinate[HMI_flag.language], MBASE(row));
	Draw_Menu_Line(row, ICON_HotendTemp);
	DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(row), thermalManager.temp_hotend[0].target);
}

inline void Item_Tune_BTemp(const uint8_t row){
	DWIN_Frame_AreaCopy_ID1(Tune_Menu_Bed, LBLX, MBASE(row));
  DWIN_Frame_AreaCopy_ID1(Tune_Menu_Temp, LBLX+Bed_X_Coordinate[HMI_flag.language], MBASE(row));
	Draw_Menu_Line(row, ICON_BedTemp);
  DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(row), thermalManager.temp_bed.target);
}

inline void Item_Tune_FanSpeed(const uint8_t row){
	DWIN_Frame_AreaCopy_ID1(Tune_Menu_Fan_Speed, LBLX, MBASE(row));
	Draw_Menu_Line(row, ICON_FanSpeed);
  DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(row), thermalManager.fan_speed[0]);
}

inline void Item_Tune_ZOffset(const uint8_t row){
	DWIN_Frame_AreaCopy_ID1(Tune_Menu_Z_Offset, LBLX, MBASE(row));
	Draw_Menu_Line(row, ICON_Zoffset);
  DWIN_Draw_Small_Float22(MENUVALUE_X, MBASE(row), HMI_ValueStruct.Zoffset_Scale);
}

inline void Item_Tune_Mixer(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Tune_Menu_Mixer, LBLX, MBASE(row));
 Draw_Menu_Line(row, ICON_Mixer);
 Draw_More_Icon(row);
}

inline void Item_Tune_Config(const uint8_t row) {
 DWIN_Frame_AreaCopy_ID1(Tune_Menu_Config, LBLX, MBASE(row));
 Draw_Menu_Line(row, ICON_Contact);
 Draw_More_Icon(row);
}

inline void Draw_Tune_Menu() {
	Clear_Main_Window();
	Clear_Bottom_Area();
	
#if TUNE_CASE_TOTAL >= 6
	const int16_t kscroll = MROWS - select_tune.index; // Scrolled-up lines
	#define KCSCROL(L) (kscroll + (L))
#else
	#define KCSCROL(L) (L)
#endif
	#define KCLINE(L) MBASE(KCSCROL(L))
	#define KCVISI(L) WITHIN(KCSCROL(L), 0, MROWS)

	dwinLCD.JPG_CacheTo1(HMI_flag.Title_Menu_Backup);
	DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_TUNE, TITLE_X, TITLE_Y);
	dwinLCD.JPG_CacheTo1(HMI_flag.language+1);
	if (KCVISI(0)) Draw_Back_First(select_tune.now == 0);

	if (KCVISI(TUNE_CASE_SPEED)) Item_Tune_Speed(KCSCROL(TUNE_CASE_SPEED));
#if HAS_HOTEND
	if (KCVISI(TUNE_CASE_ETEMP)) Item_Tune_ETemp(KCSCROL(TUNE_CASE_ETEMP));
#endif
#if HAS_HEATED_BED
	if (KCVISI(TUNE_CASE_BTEMP)) Item_Tune_BTemp(KCSCROL(TUNE_CASE_BTEMP));	
#endif 
#if HAS_FAN
	if (KCVISI(TUNE_CASE_FAN)) Item_Tune_FanSpeed(KCSCROL(TUNE_CASE_FAN));
#endif
#if ENABLED(BABYSTEPPING)
	if (KCVISI(TUNE_CASE_ZOFF)) Item_Tune_ZOffset(KCSCROL(TUNE_CASE_ZOFF)); 
#endif
	if (KCVISI(TUNE_CASE_MIXER)) Item_Tune_Mixer(KCSCROL(TUNE_CASE_MIXER)); 
	if (KCVISI(TUNE_CASE_CONFIG)) Item_Tune_Config(KCSCROL(TUNE_CASE_CONFIG)); 
	
	if (select_tune.now) Draw_Menu_Cursor(select_tune.now);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Printing UI
//
// The status area is always on-screen, except during
// full-screen modal dialogs. (TODO: Keep alive during dialogs)
//
void Draw_Status_Area(const bool with_update) {
	// Clear the bottom area of the screen
	dwinLCD.Draw_Rectangle(1, Color_Bg_Black, 0, STATUS_Y_START, DWIN_WIDTH, STATUS_Y_END);
	//
	// Status Area
	//
#if HAS_HOTEND
	DWIN_Show_ICON(ICON_HotendTemp, State_icon_extruder_X, State_icon_extruder_Y);
	DWIN_Draw_IntValue_FONT10(State_text_extruder_num, State_text_extruder_X, State_text_extruder_Y, thermalManager.temp_hotend[0].celsius);
	DWIN_Draw_UnMaskString_FONT10(State_string_extruder_X, State_string_extruder_Y, F("/"));
	DWIN_Draw_IntValue_FONT10(State_text_extruder_num, State_text_extruder_X + (State_text_extruder_num + 1) * STAT_CHR_W, State_text_extruder_Y, thermalManager.temp_hotend[0].target);
#endif
#if HOTENDS > 1
	// dwinLCD.ICON_Show(ICON_IMAGE_ID,ICON_HotendTemp, 13, 381);
#endif

#if HAS_HEATED_BED
	DWIN_Show_ICON( ICON_BedTemp, State_icon_bed_X, State_icon_bed_Y);
	DWIN_Draw_IntValue_FONT10(State_text_bed_num, State_text_bed_X, State_text_bed_Y, thermalManager.temp_bed.celsius);
	DWIN_Draw_UnMaskString_FONT10(State_string_bed_X, State_string_bed_Y, F("/"));
	DWIN_Draw_IntValue_FONT10(State_text_bed_num, State_text_bed_X + (State_text_extruder_num + 1) * STAT_CHR_W, State_text_bed_Y, thermalManager.temp_bed.target);
#endif

	DWIN_Show_ICON( ICON_Speed, State_icon_speed_X, State_icon_speed_Y);
	DWIN_Draw_IntValue_FONT10(3, State_text_speed_X, State_text_speed_Y, feedrate_percentage);
	DWIN_Draw_UnMaskString_FONT10(State_string_speed_X, State_string_speed_Y, F("%"));

	//	
	update_Z_Position(true);

	if (with_update) {
		dwinLCD.UpdateLCD();
		delay(5);
	}
}

void Draw_Mixer_Status_Area(const bool with_update) {
	//
	// Mixer Status Area
	//
#if ENABLED(MIXING_EXTRUDER)
	mixer.selected_vtool = MixerCfg.Vtool_Backup;
	updata_mixer_from_vtool();
	Refresh_Percent_display();
#endif

	if (with_update) {
		dwinLCD.UpdateLCD();
		delay(5);
	}
}

//
// Print>>File
//
inline void Draw_Print_File_Menu() {
	Clear_Title_Bar();
	Clear_Bottom_Area();

	dwinLCD.JPG_CacheTo1(HMI_flag.Title_Menu_Backup);
	DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_SDPRINT, TITLE_X, TITLE_Y);
	DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_FILE, 14+Print_File_X_Coordinate[HMI_flag.language], 7);
	dwinLCD.JPG_CacheTo1(HMI_flag.language+1);
	Redraw_SD_List();
}

//
// Draw Babystep Windows
//
inline void Draw_Babystep_Menu() {
	Clear_Main_Window();
	dwinLCD.Draw_String(false, false, font14x28, Color_White, Color_Bg_Black, 10, 160, F("Babysteps:"));
	DWIN_Draw_Big_Float32(170, 160, HMI_ValueStruct.Zoffset_Scale);
}

inline void Draw_Select_Highlight(const bool sel) {
	HMI_flag.select_flag = sel;
	const uint16_t c1 = sel ? Select_Color : Color_Bg_Window,
	     c2 = sel ? Color_Bg_Window : Select_Color;
	dwinLCD.Draw_Rectangle(0, c1, 25, 279, 126, 318);
	dwinLCD.Draw_Rectangle(0, c1, 24, 278, 127, 319);
	dwinLCD.Draw_Rectangle(0, c2, 145, 279, 246, 318);
	dwinLCD.Draw_Rectangle(0, c2, 144, 278, 247, 319);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Popup Menu
inline void Draw_Popup_Bkgd_60() {
 dwinLCD.Draw_Rectangle(1, Color_Bg_Window, 14, 60, 258, 330);
}

inline void Draw_Popup_Bkgd_Wifi() {
 dwinLCD.Draw_Rectangle(1, Color_Bg_Blue, 14, 60, 258, 90);
}

inline void Draw_Popup_Bkgd_Reprint() {
 dwinLCD.Draw_Rectangle(1, Color_Bg_Blue, 14, 60, 258, 90);
}


#if ENABLED(OPTION_REPEAT_PRINTING)
void Popup_Window_BTempCooldown() {
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	Draw_Popup_Bkgd_Reprint();
	Draw_Reprint_Title("Remain Times:");
	dwinLCD.Draw_Rectangle(1, Color_Bg_Blue, 144, 65, 258, 85);
	dwinLCD.Draw_IntValue(true, true, 0, font10x20, Color_White, Color_Bg_Blue, 4, 144, 65, ReprintManager.Reprint_times);
	DWIN_Draw_UnMaskString_Default(14, 7, F("Re-print"));
	dwinLCD.Draw_String(false, true, font10x20, Popup_Text_Color, Color_Bg_Window, 61, 200, F("Wait for Hotbed"));
	dwinLCD.Draw_String(false, true, font10x20, Popup_Text_Color, Color_Bg_Window, 86, 235, F("cool down!"));
	DWIN_Show_ICON( ICON_Confirm_E, 86, 280);
}

void Popup_Window_FMoveStart() {
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	Draw_Popup_Bkgd_Reprint();
	Draw_Reprint_Title("Remain Times:");
	dwinLCD.Draw_Rectangle(1, Color_Bg_Blue, 144, 65, 258, 85);
	dwinLCD.Draw_IntValue(true, true, 0, font10x20, Color_White, Color_Bg_Blue, 4, 144, 65, ReprintManager.Reprint_times);
	DWIN_Draw_UnMaskString_Default(14, 7, F("Re-print"));
	dwinLCD.Draw_String(false, true, font10x20, Popup_Text_Color, Color_Bg_Window, 41, 200, F("Forward Move Start!"));
	DWIN_Show_ICON( ICON_Confirm_E, 86, 280);
}

void Popup_Window_BMoveStart() {
	Clear_Main_Window();
  Draw_Popup_Bkgd_60();
	Draw_Popup_Bkgd_Reprint();
	Draw_Reprint_Title("Remain Times:");
	dwinLCD.Draw_Rectangle(1, Color_Bg_Blue, 144, 65, 258, 85);
	dwinLCD.Draw_IntValue(true, true, 0, font10x20, Color_White, Color_Bg_Blue, 4, 144, 65, ReprintManager.Reprint_times);
  DWIN_Draw_UnMaskString_Default(14, 7, F("Re-print"));
	dwinLCD.Draw_String(false, true, font10x20, Popup_Text_Color, Color_Bg_Window, 56, 200, F("Back Move Start!"));
	DWIN_Show_ICON( ICON_Confirm_E, 86, 280);
}

void Popup_Window_BMoveStop() {
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	Draw_Popup_Bkgd_Reprint();
	Draw_Reprint_Title("Remain Times:");
	dwinLCD.Draw_Rectangle(1, Color_Bg_Blue, 144, 65, 258, 85);
	dwinLCD.Draw_IntValue(true, true, 0, font10x20, Color_White, Color_Bg_Blue, 4, 144, 65, ReprintManager.Reprint_times);
	DWIN_Draw_UnMaskString_Default(14, 7, F("Re-print"));
	dwinLCD.Draw_String(false, true, font10x20, Popup_Text_Color, Color_Bg_Window, 61, 200, F("Back Move Stop!"));
	DWIN_Show_ICON( ICON_Confirm_E, 86, 280);
}
#endif

#if ENABLED(PREVENT_COLD_EXTRUSION)
void Popup_Window_ETempTooLow() {
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	DWIN_Show_ICON( ICON_TempTooLow, 102, 105);
	DWIN_Draw_MaskString_Default_PopMenu((272 - 18*10), 235, F("Nozzle is too cold"));
	DWIN_Show_ICON( ICON_Confirm_E, 86, 280);
}
#endif

//
// Draw Popup Windows
//
void Popup_Window_Temperature(const char *msg) {
	Clear_Popup_Area();
	Draw_Popup_Bkgd_105();
	DWIN_Show_ICON(ICON_TempTooHigh, 102, 165);		
	dwinLCD.Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - strlen(msg)*10)/2, 300, (char*)msg);
#if 1
	for(uint8_t i=0; i<20; i++){
		buzzer.tone(200, 2000);
		buzzer.tone(200, 0);
	}
#endif
}

void Popup_Window_Resume() {
	Clear_Popup_Area();
	Draw_Popup_Bkgd_105();

	DWIN_Draw_MaskString_Default_PopMenu( (272 - 10 * 14) / 2, 115, F("Continue Print"));
	DWIN_Draw_MaskString_Default_PopMenu( (272 - 10 * 22) / 2, 192, F("It looks like the last"));
	DWIN_Draw_MaskString_Default_PopMenu( (272 - 10 * 22) / 2, 212, F("file was interrupted."));
	DWIN_Show_ICON( ICON_Cancel_E,  26, 307);
	DWIN_Show_ICON( ICON_Continue_E, 146, 307);
}

void Popup_Window_HomeAll(const bool parking=false) {
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	DWIN_Show_ICON(ICON_WAITING, 86, 105);
	DWIN_Draw_MaskString_Default_PopMenu( (272 - 10 * (parking ? 7 : 10)) / 2, 230, parking ? F("Parking") : F("Homing XYZ"));
	DWIN_Draw_MaskString_Default_PopMenu( (272 - 10 * 23) / 2, 260, F("Please wait until done."));
}

void Popup_Window_HomeX(const bool parking=false) {
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	DWIN_Show_ICON(ICON_WAITING, 86, 105);
	DWIN_Draw_MaskString_Default_PopMenu( (272 - 10 * (parking ? 7 : 10)) / 2, 230, parking ? F("Parking") : F("Homing X"));
	DWIN_Draw_MaskString_Default_PopMenu( (272 - 10 * 23) / 2, 260, F("Please wait until done."));
}

void Popup_Window_HomeY(const bool parking=false) {
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	DWIN_Show_ICON(ICON_WAITING, 86, 105);
	DWIN_Draw_MaskString_Default_PopMenu( (272 - 10 * (parking ? 7 : 10)) / 2, 230, parking ? F("Parking") : F("Homing Y"));
	DWIN_Draw_MaskString_Default_PopMenu( (272 - 10 * 23) / 2, 260, F("Please wait until done."));
}

void Popup_Window_HomeZ(const bool parking=false) {
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	DWIN_Show_ICON(ICON_WAITING, 86, 105);
	DWIN_Draw_MaskString_Default_PopMenu( (272 - 10 * (parking ? 7 : 10)) / 2, 230, parking ? F("Parking") : F("Homing Z"));
	DWIN_Draw_MaskString_Default_PopMenu( (272 - 10 * 23) / 2, 260, F("Please wait until done."));
}



#ifdef LCD_BED_LEVELING
void Popup_Window_Leveling() {
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	DWIN_Show_ICON( ICON_AutoLeveling, 101, 105);
	DWIN_Draw_MaskString_Default_PopMenu( (272 - 10 * 13) / 2, 230, F("Bed Leveling"));
	DWIN_Draw_MaskString_Default_PopMenu( (272 - 10 * 23) / 2, 260, F("Please wait until done."));	
}

#if ENABLED(AUTO_UPDATA_PROBE_Z_OFFSET)
void Popup_Window_CatchOffset() {
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	DWIN_Show_ICON( ICON_AutoLeveling, 101, 105);
	DWIN_Draw_MaskString_Default_PopMenu( (272 - 10 * 23) / 2, 230, F("Catching Probe Z Offset"));
	DWIN_Draw_MaskString_Default_PopMenu( (272 - 10 * 23) / 2, 260, F("Please wait until done."));
	
}
#endif

void Popup_Remove_Glass() {
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();

	DWIN_Show_ICON( ICON_AutoLeveling, 101, 105);
	dwinLCD.Draw_String(false, true, font8x16, Color_Bg_Red, Color_Bg_Window, (272 - 8 * 13) / 2, 180, F("!!Attention!!"));
	DWIN_Draw_MaskString_Default_PopMenu( (272 - 10 * 23) / 2, 212, F("Please remove the glass"));
	DWIN_Draw_MaskString_Default_PopMenu( (272 - 10 * 15) / 2, 232, F("before catching"));	
	DWIN_Show_ICON( ICON_Confirm_E, 96, 280);
}
#endif

void Popup_window_PauseOrStop() {
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();

	if (select_print.now == PRINT_CASE_PAUSE) 
		DWIN_Draw_MaskString_FONT12(Popup_Text_Color, Color_Bg_Window, (272 - 12 * 12) / 2, 150, F("Pause Print?"));
	else if (select_print.now == PRINT_CASE_STOP) 
		DWIN_Draw_MaskString_FONT12(Popup_Text_Color, Color_Bg_Window, (272 - 12 * 11) / 2, 150, F("Stop Print?"));
	DWIN_Show_ICON( ICON_Confirm_E, 26, 280);
	DWIN_Show_ICON( ICON_Cancel_E, 146, 280);
	Draw_Select_Highlight(true);
}

inline void Popup_Window_waiting(uint8_t msg){
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	DwinMenuID = DWMENU_POP_WAITING;
	DWIN_Show_ICON(ICON_WAITING, 86, 105);
	if(msg == 0)
		DWIN_Draw_String_FIL((272 - 10 * 12)/2, 240, F("Pausing..."));
	else
		DWIN_Draw_String_FIL((272 - 10 * 12)/2, 240, F("Stoping..."));
	DWIN_Draw_String_FIL((272 - 15 * 12)/2, 269, F("Please wait!"));
}

inline void Popup_Window_Wifi_Connect() {
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	Draw_Title(F("WIFI"));
	DWIN_Draw_MaskString_FONT12(Popup_Text_Color, Color_Bg_Window, (DWIN_WIDTH - 12 * 10) / 2 - 24, 240, F("Connection"));
}

inline void Popup_Window_Wifi_Disconnect() {
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	Draw_Title(F("WIFI"));

	DWIN_Draw_MaskString_FONT12(Popup_Text_Color, Color_Bg_Window, (DWIN_WIDTH - 12 * 11) / 2, 240, F("No connect!"));
}

inline void Popup_Window_Powerdown() {
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	Draw_Title(F("Powerdown"));
	DWIN_Show_ICON( ICON_POWER_DOWN, 86, 95);
	DWIN_Show_ICON( ICON_NO_0, 26, 228);
	DWIN_Show_ICON( ICON_YES_1, 146, 228);
	DWIN_Draw_MaskString_FONT12(Popup_Text_Color, Color_Bg_Window, (DWIN_WIDTH - 12 * 17) / 2, 290, F("Confirm Shutdown?"));
}

inline void Draw_Printing_Screen() {
 DWIN_Frame_AreaCopy_ID1(Printing_Menu_Tune, Tune_X_Coordinate[HMI_flag.language], 325);
 DWIN_Frame_AreaCopy_ID1(Printing_Menu_Pause, Pause_X_Coordinate[HMI_flag.language], 325);
 DWIN_Frame_AreaCopy_ID1(Printing_Menu_Stop, Stop_X_Coordinate[HMI_flag.language], 325);
}

inline void Draw_Print_ProgressBar() {
 DWIN_Show_ICON( ICON_Bar, 15, 63);
 dwinLCD.Draw_Rectangle(1, BarFill_Color, 15 + HMI_ValueStruct.Percentrecord * 240 / 100, 63, 256, 83);
 dwinLCD.Draw_IntValue(false, true, 0, font8x16, Color_Bg_Red, BarFill_Color, 2, 117, 65, HMI_ValueStruct.Percentrecord);
 dwinLCD.Draw_String(false, false, font8x16, Color_Bg_Red, BarFill_Color, 133, 65, F("%"));
}

inline void Draw_Print_ProgressElapsed() {
 duration_t elapsed = print_job_timer.duration(); // print timer
 DWIN_Draw_MaskIntValue_Default(2, 42, 212, elapsed.value / 3600);
 DWIN_Draw_MaskString_Default(58, 212, F(":"));
 DWIN_Draw_MaskIntValue_Default(2, 66, 212, (elapsed.value % 3600) / 60);
}

#if ENABLED(OPTION_AUTOPOWEROFF)
inline void Draw_Freedown_Machine() {		
	dwinLCD.Draw_Rectangle(1, Color_Bg_Black, 230, 7, 256, 23);
	if(HMI_flag.free_close_timer_rg >= POWERDOWN_MACHINE_TIMER)
		DWIN_Draw_UnMaskString_Default(230, 7, F("   "));
	else if(HMI_flag.free_close_timer_rg >= 100)
		DWIN_Draw_UnMaskIntValue_Default(3, 230, 7, HMI_flag.free_close_timer_rg);
	else if((HMI_flag.free_close_timer_rg < 100)&&(HMI_flag.free_close_timer_rg >= 10))
		dwinLCD.Draw_IntValue(false, true, 1, font8x16, Color_Yellow, Color_Bg_Black, 3, 230, 7, HMI_flag.free_close_timer_rg);
	else
		dwinLCD.Draw_IntValue(false, true, 1, font8x16, Color_Red, Color_Bg_Black, 3, 230, 7, HMI_flag.free_close_timer_rg);
}
#endif

void Draw_Print_ProgressRemain() {
	if(HMI_ValueStruct.remain_time > 0){
		DWIN_Draw_MaskIntValue_Default(2, 176, 212, HMI_ValueStruct.remain_time / 3600);
		DWIN_Draw_MaskString_Default(192, 212, F(":"));
		DWIN_Draw_MaskIntValue_Default(2, 200, 212, (HMI_ValueStruct.remain_time % 3600) / 60);
	}
	else{
		DWIN_Draw_MaskString_Default(176, 212, F("--:--"));
	}
}

void Draw_Print_ProgressExtruder() {
 #if(MIXING_STEPPERS == 4)
 	DWIN_Show_ICON( ICON_Extruder1_P, 10, 98);
 	DWIN_Show_ICON( ICON_Extruder2_P, 62, 98);
 	DWIN_Show_ICON( ICON_Extruder3_P, 114, 98);
 	DWIN_Show_ICON( ICON_Extruder4_P, 166, 98);
 	DWIN_Show_ICON( ICON_VTool_P, 	218, 98);
 #elif(MIXING_STEPPERS == 3)
 	DWIN_Show_ICON( ICON_Extruder1_P, 21, 98);
 	DWIN_Show_ICON( ICON_Extruder2_P, 84, 98);
 	DWIN_Show_ICON( ICON_Extruder3_P, 147, 98);
 	DWIN_Show_ICON( ICON_VTool_P, 	210, 98);
 #elif(MIXING_STEPPERS == 2)
 	DWIN_Show_ICON( ICON_Extruder1_P, 36, 98);
 	DWIN_Show_ICON( ICON_Extruder2_P, 114, 98);
 	DWIN_Show_ICON( ICON_VTool_P, 	192, 98);
 #else
  DWIN_Show_ICON( ICON_Extruder1_P, 63, 98);
  DWIN_Show_ICON( ICON_VTool_P, 	168, 98);
 #endif
}

void Draw_Print_ProgressMixModel(){
  dwinLCD.Draw_Rectangle(1, Color_Bg_Black, 10, 455, 250, 471);  
	if(mixer.gradient.enabled) {
		DWIN_Draw_MaskString_Default(10, 455, F("Model: Gradient"));
		DWIN_Draw_IntValue_Default(2, 130, 455, mixer.gradient.start_vtool);
		DWIN_Draw_MaskString_Default(146, 455, F("--->"));
		DWIN_Draw_IntValue_Default(2, 178, 455, mixer.gradient.end_vtool);
	}
	else if(mixer.random_mix.enabled) {
		DWIN_Draw_MaskString_Default(10, 455, F("Model: Random"));
	}
	else{
		DWIN_Draw_MaskString_Default(10, 455, F("Current Vtool:"));
		DWIN_Draw_IntValue_Default(2, 130, 455, mixer.selected_vtool);
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//Main Menu
//
void Draw_Main_Menu() {

	DwinMenuID = DWMENU_MAIN;
	
	//Clear_Main_Window();
	dwinLCD.Draw_Rectangle(1, Color_Bg_Black, 0, 0, DWIN_WIDTH, DWIN_HEIGHT);
	DWIN_Show_ICON(ICON_LOGO, LOGO_OFFSET_X, LOGO_OFFSET_Y);


	ICON_Print();
	ICON_Prepare();
	ICON_Control();
	ICON_StartInfo();
	Draw_Status_Area(true);

	//print_job_timer.start();

	#if ENABLED(MIXING_EXTRUDER)
	mixer.selected_vtool = MixerCfg.Vtool_Backup;
	#endif
	
	//Show WiFi ICON_IMAGE_ID
	#if ENABLED(OPTION_WIFI_MODULE)
	if(HMI_flag.wifi_Handshake_ok && WiFi_Enabled){
		DWIN_Show_ICON( ICON_WIFI, 0, 0);
	}
	#endif
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void Start_PrintProcess() {
	DwinMenuID = DWMENU_PRINTING;	
	Clear_Main_Window();
	Clear_Bottom_Area();
	Draw_Printing_Screen();

	dwinLCD.JPG_CacheToN(1,HMI_flag.Title_Menu_Backup);
	DWIN_Frame_AreaCopy_ID1(MTSTRING_TITLE_SDPRINT, TITLE_X, TITLE_Y);
	dwinLCD.JPG_CacheToN(1,HMI_flag.language+1);
	ICON_Tune();
	if(printingIsPaused() || (DWIN_status == ID_SM_RESUMING) || (DWIN_status == ID_SM_PAUSING))
		ICON_Continue(); 
	else 
		ICON_Pause();
	ICON_Stop();

	// Copy into filebuf string before entry
	char * const name = card.longest_filename();
	const int8_t npos = _MAX(0U, DWIN_WIDTH - strlen(name) * MENU_CHR_W) / 2;
	DWIN_Draw_UnMaskString_Default(npos, 40, name);

	DWIN_Show_ICON(ICON_PrintTime, 17, 193);
	DWIN_Show_ICON(ICON_RemainTime, 150, 191);
	DWIN_Show_ICON(ICON_PRINT_TIME, 42, 193);
	DWIN_Show_ICON(ICON_REMAIN_TIME, 176, 193);

	Draw_Print_ProgressBar();
	Draw_Print_ProgressElapsed();
	Draw_Print_ProgressRemain();	
	Draw_Print_ProgressExtruder();
	mixer.selected_vtool = MixerCfg.Vtool_Backup;
	updata_mixer_from_vtool();
	Refresh_Percent_display();
	Draw_Print_ProgressMixModel();
}



void HMI_SetLanguage() {
	dwinLCD.JPG_CacheTo1(HMI_flag.language + 1);
	if(HMI_flag.language < 3) 
		HMI_flag.Title_Menu_Backup = 7;
	else 
		HMI_flag.Title_Menu_Backup = 6;
}

void HMI_Move_X() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Move_X_scale)) {
			DwinMenuID = DWMENU_MOVEAXIS;
			EncoderRate.enabled = false;
			 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_axis.index + AXISMOVE_CASE_MOVEX), HMI_ValueStruct.Move_X_scale);
			if(!planner.is_full()) {		 
			// Wait for planner moves to finish!
			planner.synchronize();
			planner.buffer_line(current_position, MMM_TO_MMS(HOMING_FEEDRATE_XY), active_extruder);
			}
			dwinLCD.UpdateLCD();
			return;
		}
		NOLESS(HMI_ValueStruct.Move_X_scale, (X_MIN_POS) * MINUNITMULT);
		NOMORE(HMI_ValueStruct.Move_X_scale, (X_MAX_POS) * MINUNITMULT);
		current_position.x = (float)HMI_ValueStruct.Move_X_scale / MINUNITMULT;
		DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_axis.index + AXISMOVE_CASE_MOVEX), HMI_ValueStruct.Move_X_scale);
		dwinLCD.UpdateLCD();
	}
}

void HMI_Move_Y() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Move_Y_scale)) {
			DwinMenuID = DWMENU_MOVEAXIS;
			EncoderRate.enabled = false;
			 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_axis.index + AXISMOVE_CASE_MOVEY), HMI_ValueStruct.Move_Y_scale);
			if(!planner.is_full()) {		 
				// Wait for planner moves to finish!
				planner.synchronize();
				planner.buffer_line(current_position, MMM_TO_MMS(HOMING_FEEDRATE_XY), active_extruder);
			}
			dwinLCD.UpdateLCD();
			return;
		}
		NOLESS(HMI_ValueStruct.Move_Y_scale, (Y_MIN_POS) * MINUNITMULT);
		NOMORE(HMI_ValueStruct.Move_Y_scale, (Y_MAX_POS) * MINUNITMULT);
		current_position.y = (float)HMI_ValueStruct.Move_Y_scale / MINUNITMULT;
		DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_axis.index + AXISMOVE_CASE_MOVEY), HMI_ValueStruct.Move_Y_scale);
		dwinLCD.UpdateLCD();
	}
}

void HMI_Move_Z() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Move_Z_scale)) {
			DwinMenuID = DWMENU_MOVEAXIS;
			EncoderRate.enabled = false;
			 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_axis.index + AXISMOVE_CASE_MOVEZ), HMI_ValueStruct.Move_Z_scale);
			if(!planner.is_full()) {		 
			// Wait for planner moves to finish!
				planner.synchronize();
				planner.buffer_line(current_position, MMM_TO_MMS(HOMING_FEEDRATE_Z), active_extruder);
			}
			dwinLCD.UpdateLCD();
			return;
		}
		NOLESS(HMI_ValueStruct.Move_Z_scale, Z_MIN_POS * MINUNITMULT);
		NOMORE(HMI_ValueStruct.Move_Z_scale, Z_MAX_POS * MINUNITMULT);
		current_position.z = (float)HMI_ValueStruct.Move_Z_scale / MINUNITMULT;
		DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_axis.index + AXISMOVE_CASE_MOVEZ), HMI_ValueStruct.Move_Z_scale);
		dwinLCD.UpdateLCD();
	}
}

#if ENABLED(FWRETRACT) 
char Retract_Buf[50]={0};
void HMI_Retract_MM() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Retract_MM_scale)) {
			DwinMenuID = DWMENU_SET_RETRACT;
			EncoderRate.enabled = false;
			DWIN_Draw_Small_Float22(MENUVALUE_X, MBASE(MROWS -select_retract.index + RETRACT_CASE_RETRACT_MM), HMI_ValueStruct.Retract_MM_scale);
			ZERO(Retract_Buf);
			sprintf_P(Retract_Buf,PSTR("M207 S%.2f"),(fwretract.settings.retract_length));
			queue.inject_P(Retract_Buf);
			dwinLCD.UpdateLCD();			
			return;
		}
		NOLESS(HMI_ValueStruct.Retract_MM_scale, 0);
		NOMORE(HMI_ValueStruct.Retract_MM_scale, 100*MAXUNITMULT);
		fwretract.settings.retract_length = (float)HMI_ValueStruct.Retract_MM_scale/MAXUNITMULT;
		DWIN_Draw_Selected_Small_Float22(MENUVALUE_X, MBASE(MROWS -select_retract.index + RETRACT_CASE_RETRACT_MM), HMI_ValueStruct.Retract_MM_scale);
		dwinLCD.UpdateLCD();
	}
}

void HMI_Retract_V() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Retract_V_scale)) {
			DwinMenuID = DWMENU_SET_RETRACT;
			EncoderRate.enabled = false;
			DWIN_Draw_Small_Float32(MENUVALUE_X-8, MBASE(MROWS -select_retract.index + RETRACT_CASE_RETRACT_V), HMI_ValueStruct.Retract_V_scale);
			ZERO(Retract_Buf);
			sprintf_P(Retract_Buf,PSTR("M207 F%.2f"),(MMS_TO_MMM(fwretract.settings.retract_feedrate_mm_s)));
			queue.inject_P(Retract_Buf);
			dwinLCD.UpdateLCD();
			return;
		}
		NOLESS(HMI_ValueStruct.Retract_V_scale, 0);
		NOMORE(HMI_ValueStruct.Retract_V_scale, 100*MAXUNITMULT);
		fwretract.settings.retract_feedrate_mm_s = (float)HMI_ValueStruct.Retract_V_scale/MAXUNITMULT;
		DWIN_Draw_Selected_Small_Float32(MENUVALUE_X-8, MBASE(MROWS -select_retract.index + RETRACT_CASE_RETRACT_V), HMI_ValueStruct.Retract_V_scale);
		dwinLCD.UpdateLCD();
	}
}

void HMI_Retract_ZHOP() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Retract_ZHOP_scale)) {
			DwinMenuID = DWMENU_SET_RETRACT;
			EncoderRate.enabled = false;
			DWIN_Draw_Small_Float32(MENUVALUE_X-8, MBASE(MROWS -select_retract.index + RETRACT_CASE_RETRACT_ZHOP), HMI_ValueStruct.Retract_ZHOP_scale);
			ZERO(Retract_Buf);
			sprintf_P(Retract_Buf,PSTR("M207 Z%.2f"), fwretract.settings.retract_zraise);
			queue.inject_P(Retract_Buf);
			dwinLCD.UpdateLCD();			
			return;
		}
		NOLESS(HMI_ValueStruct.Retract_ZHOP_scale, 0);
		NOMORE(HMI_ValueStruct.Retract_ZHOP_scale, 10*MAXUNITMULT);
		fwretract.settings.retract_zraise = (float)HMI_ValueStruct.Retract_ZHOP_scale/MAXUNITMULT;
		DWIN_Draw_Selected_Small_Float32(MENUVALUE_X-8, MBASE(MROWS -select_retract.index + RETRACT_CASE_RETRACT_ZHOP), HMI_ValueStruct.Retract_ZHOP_scale);
		dwinLCD.UpdateLCD();
	}
}

void HMI_UnRetract_MM() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.unRetract_MM_scale)) {
			DwinMenuID = DWMENU_SET_RETRACT;
			EncoderRate.enabled = false;
			DWIN_Draw_Small_Float22(MENUVALUE_X, MBASE(MROWS -select_retract.index + RETRACT_CASE_RECOVER_MM), HMI_ValueStruct.unRetract_MM_scale);
			ZERO(Retract_Buf);
			sprintf_P(Retract_Buf,PSTR("M208 S%.2f"),(fwretract.settings.retract_recover_extra));
			queue.inject_P(Retract_Buf);
			dwinLCD.UpdateLCD();			
			return;
		}
		NOLESS(HMI_ValueStruct.unRetract_MM_scale, 0);
		NOMORE(HMI_ValueStruct.unRetract_MM_scale, 100*MAXUNITMULT);
		fwretract.settings.retract_recover_extra = (float)HMI_ValueStruct.unRetract_MM_scale/MAXUNITMULT;
		DWIN_Draw_Selected_Small_Float22(MENUVALUE_X, MBASE(MROWS -select_retract.index + RETRACT_CASE_RECOVER_MM), HMI_ValueStruct.unRetract_MM_scale);
		dwinLCD.UpdateLCD();
	}
}

void HMI_UnRetract_V() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.unRetract_V_scale)) {
			DwinMenuID = DWMENU_SET_RETRACT;
			EncoderRate.enabled = false;
			DWIN_Draw_Small_Float32(MENUVALUE_X-8, MBASE(MROWS -select_retract.index + RETRACT_CASE_RECOVER_V), HMI_ValueStruct.unRetract_V_scale);
			ZERO(Retract_Buf);
			sprintf_P(Retract_Buf,PSTR("M208 F%.2f"),(MMS_TO_MMM(fwretract.settings.retract_recover_feedrate_mm_s)));
			queue.inject_P(Retract_Buf);
			dwinLCD.UpdateLCD();			
			return;
		}
		NOLESS(HMI_ValueStruct.unRetract_V_scale, 0);
		NOMORE(HMI_ValueStruct.unRetract_V_scale, 100*MAXUNITMULT);
		fwretract.settings.retract_recover_feedrate_mm_s = (float)HMI_ValueStruct.unRetract_V_scale/MAXUNITMULT;
		DWIN_Draw_Selected_Small_Float32(MENUVALUE_X-8, MBASE(MROWS -select_retract.index + RETRACT_CASE_RECOVER_V), HMI_ValueStruct.unRetract_V_scale);
		dwinLCD.UpdateLCD();
	}
}
#endif

#if ENABLED(MIXING_EXTRUDER)
void HMI_Adjust_Ext_Percent(uint8_t Extruder_Number) {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, MixerCfg.Manual_Percent[mixer.selected_vtool][Extruder_Number-1])) {
			DwinMenuID = DWMENU_MIXER_MANUAL;
			EncoderRate.enabled = false;
			DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(Extruder_Number), MixerCfg.Manual_Percent[mixer.selected_vtool][Extruder_Number-1]);
			dwinLCD.UpdateLCD();
			return;
		}
		NOLESS(MixerCfg.Manual_Percent[mixer.selected_vtool][Extruder_Number-1], 0);
		NOMORE(MixerCfg.Manual_Percent[mixer.selected_vtool][Extruder_Number-1], 100);
		DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(Extruder_Number), MixerCfg.Manual_Percent[mixer.selected_vtool][Extruder_Number-1]);
		dwinLCD.UpdateLCD();
	}
}

void HMI_Adjust_Auto_Zpos_Start() {
	static int16_t last_Z_scale = 0;
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();

	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Auto_Zstart_scale)) {
			DwinMenuID = DWMENU_MIXER_AUTO;
			EncoderRate.enabled = false;
			last_Z_scale = HMI_ValueStruct.Auto_Zstart_scale;
			 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_auto.index + AUTO_CASE_ZPOS_START), HMI_ValueStruct.Auto_Zstart_scale);
			dwinLCD.UpdateLCD();
			return;
	  }
		
	  if ((HMI_ValueStruct.Auto_Zstart_scale - last_Z_scale) > (Z_MAX_POS) * MINUNITMULT)
			HMI_ValueStruct.Auto_Zstart_scale = last_Z_scale + (Z_MAX_POS) * MINUNITMULT;
	  else if ((last_Z_scale - HMI_ValueStruct.Auto_Zstart_scale) > (Z_MAX_POS) * MINUNITMULT)
			HMI_ValueStruct.Auto_Zstart_scale = last_Z_scale - (Z_MAX_POS) * MINUNITMULT;
		
		NOLESS(HMI_ValueStruct.Auto_Zstart_scale, 0);
		NOMORE(HMI_ValueStruct.Auto_Zstart_scale, Z_MAX_POS*MINUNITMULT);
		DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_auto.index + AUTO_CASE_ZPOS_START), HMI_ValueStruct.Auto_Zstart_scale);
		mixer.gradient.start_z = (float)HMI_ValueStruct.Auto_Zstart_scale / MINUNITMULT;
		mixer.refresh_gradient();
		dwinLCD.UpdateLCD();
	}
}

void HMI_Adjust_Auto_Zpos_End() {
	static int16_t last_Z_scale = 0;
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();

	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Auto_Zend_scale)) {
			DwinMenuID = DWMENU_MIXER_AUTO;
			EncoderRate.enabled = false;
			last_Z_scale = HMI_ValueStruct.Auto_Zend_scale;
			 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_auto.index + AUTO_CASE_ZPOS_END), HMI_ValueStruct.Auto_Zend_scale);
			dwinLCD.UpdateLCD();
			return;
	  }
		
	  if ((HMI_ValueStruct.Auto_Zend_scale - last_Z_scale) > (Z_MAX_POS) * MINUNITMULT)
			HMI_ValueStruct.Auto_Zend_scale = last_Z_scale + (Z_MAX_POS) * MINUNITMULT;
	  else if ((last_Z_scale - HMI_ValueStruct.Auto_Zend_scale) > (Z_MAX_POS) * MINUNITMULT)
			HMI_ValueStruct.Auto_Zend_scale = last_Z_scale - (Z_MAX_POS) * MINUNITMULT;
	   NOLESS(HMI_ValueStruct.Auto_Zend_scale, 0);	
		 NOMORE(HMI_ValueStruct.Auto_Zend_scale, Z_MAX_POS*MINUNITMULT);	
		 DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_auto.index + AUTO_CASE_ZPOS_END), HMI_ValueStruct.Auto_Zend_scale);
		 mixer.gradient.end_z = (float)HMI_ValueStruct.Auto_Zend_scale / 10;
		 mixer.refresh_gradient();
		 dwinLCD.UpdateLCD();
	}
}

void HMI_Adjust_Auto_VTool_Start() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, mixer.gradient.start_vtool)) {
			DwinMenuID = DWMENU_MIXER_AUTO;
			EncoderRate.enabled = false;
			DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(MROWS -select_auto.index + AUTO_CASE_VTOOL_START), mixer.gradient.start_vtool);
			mixer.refresh_gradient();			
			dwinLCD.UpdateLCD();
			return;
		}
		NOLESS(mixer.gradient.start_vtool, 0);
		NOMORE(mixer.gradient.start_vtool, MIXING_VIRTUAL_TOOLS-1);
		DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(MROWS -select_auto.index + AUTO_CASE_VTOOL_START), mixer.gradient.start_vtool);
		dwinLCD.UpdateLCD();
	}
}

void HMI_Adjust_Auto_VTool_End() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, mixer.gradient.end_vtool)) {
			DwinMenuID = DWMENU_MIXER_AUTO;
			EncoderRate.enabled = false;
			DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(MROWS -select_auto.index + AUTO_CASE_VTOOL_END), mixer.gradient.end_vtool);
			mixer.refresh_gradient();
			dwinLCD.UpdateLCD();
			return;
		}
		NOLESS(mixer.gradient.end_vtool, 0);
		NOMORE(mixer.gradient.end_vtool, MIXING_VIRTUAL_TOOLS-1);
		DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(MROWS -select_auto.index + AUTO_CASE_VTOOL_END), mixer.gradient.end_vtool);
		dwinLCD.UpdateLCD();
	}
}

void HMI_Adjust_Random_Zpos_Start() {
	static int16_t last_Z_scale = 0;
	
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Random_Zstart_scale)) {
			DwinMenuID = DWMENU_MIXER_RANDOM;
			EncoderRate.enabled = false;
			last_Z_scale = HMI_ValueStruct.Random_Zstart_scale;
			 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_random.index + RANDOM_CASE_ZPOS_START), HMI_ValueStruct.Random_Zstart_scale);
			dwinLCD.UpdateLCD();
			return;
	  }		
	  if ((HMI_ValueStruct.Random_Zstart_scale - last_Z_scale) > (Z_MAX_POS) * MINUNITMULT)
			HMI_ValueStruct.Random_Zstart_scale = last_Z_scale + (Z_MAX_POS) * MINUNITMULT;
	  else if ((last_Z_scale - HMI_ValueStruct.Random_Zstart_scale) > (Z_MAX_POS) * MINUNITMULT)
			HMI_ValueStruct.Random_Zstart_scale = last_Z_scale - (Z_MAX_POS) * MINUNITMULT;
		NOLESS(HMI_ValueStruct.Random_Zstart_scale, 0);
		mixer.random_mix.start_z = (float)HMI_ValueStruct.Random_Zstart_scale / MINUNITMULT;
		mixer.refresh_random_mix();
		DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_random.index + RANDOM_CASE_ZPOS_START), HMI_ValueStruct.Random_Zstart_scale);
		dwinLCD.UpdateLCD();
	}
}

void HMI_Adjust_Random_Zpos_End() {
	static int16_t last_Z_scale = 0;
	
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Random_Zend_scale)) {
			DwinMenuID = DWMENU_MIXER_RANDOM;
			EncoderRate.enabled = false;
			last_Z_scale = HMI_ValueStruct.Random_Zend_scale;
			 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_random.index + RANDOM_CASE_ZPOS_END), HMI_ValueStruct.Random_Zend_scale);			
			dwinLCD.UpdateLCD();
			return;
	  }
		
	  if ((HMI_ValueStruct.Random_Zend_scale - last_Z_scale) > (Z_MAX_POS) * MINUNITMULT)
			HMI_ValueStruct.Random_Zend_scale = last_Z_scale + (Z_MAX_POS) * MINUNITMULT;
	  else if ((last_Z_scale - HMI_ValueStruct.Random_Zend_scale) > (Z_MAX_POS) * MINUNITMULT)
			HMI_ValueStruct.Random_Zend_scale = last_Z_scale - (Z_MAX_POS) * MINUNITMULT;
	   NOLESS(HMI_ValueStruct.Random_Zend_scale, 0);
		 mixer.random_mix.end_z = (float)HMI_ValueStruct.Random_Zend_scale / MINUNITMULT;
		 mixer.refresh_random_mix();
		 DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_random.index + RANDOM_CASE_ZPOS_END), HMI_ValueStruct.Random_Zend_scale);
		 dwinLCD.UpdateLCD();
	}
}

void HMI_Adjust_Random_Height() {
	static float last_Height = 0;
	
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Random_Height)) {
			DwinMenuID = DWMENU_MIXER_RANDOM;
			EncoderRate.enabled = false;
			last_Height = HMI_ValueStruct.Random_Height;
			 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_random.index + RANDOM_CASE_HEIGHT), HMI_ValueStruct.Random_Height);			
			dwinLCD.UpdateLCD();
			return;
	  }
		
	  if ((HMI_ValueStruct.Random_Height - last_Height) > 100 * MINUNITMULT)
			HMI_ValueStruct.Random_Height = last_Height + 100 * MINUNITMULT;
	  else if ((last_Height - HMI_ValueStruct.Random_Height) > Z_MAX_POS * MINUNITMULT)
			HMI_ValueStruct.Random_Height = last_Height - Z_MAX_POS * MINUNITMULT;
	   NOLESS(HMI_ValueStruct.Random_Height, 0);
		 NOMORE(HMI_ValueStruct.Random_Height, Z_MAX_POS * MINUNITMULT);
		 mixer.random_mix.height = (float)HMI_ValueStruct.Random_Height / MINUNITMULT;
		 DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_random.index + RANDOM_CASE_HEIGHT), HMI_ValueStruct.Random_Height);
		 dwinLCD.UpdateLCD();
	}
}


void HMI_Adjust_Random_Extruders() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, mixer.random_mix.extruders)) {
			DwinMenuID = DWMENU_MIXER_RANDOM;
			EncoderRate.enabled = false;
			DWIN_Draw_IntValue_Default(1, MENUVALUE_X+4*8, MBASE(MROWS -select_random.index + RANDOM_CASE_EXTRUDERS), mixer.random_mix.extruders);
			dwinLCD.UpdateLCD();
			return;
		}
		NOLESS(mixer.random_mix.extruders, 1);
		NOMORE(mixer.random_mix.extruders, MIXING_STEPPERS);		
		DWIN_Draw_Select_IntValue_Default(1, MENUVALUE_X+4*8, MBASE(MROWS -select_random.index + RANDOM_CASE_EXTRUDERS), mixer.random_mix.extruders);
		dwinLCD.UpdateLCD();
	}
}
#endif

#if ENABLED(OPTION_BED_COATING)
void HMI_Adjust_Coating_Thickness() {
	static float last_G = 0;
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze(); 
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.coating_thickness)) {
			DwinMenuID = DWMENU_CONFIG;
			EncoderRate.enabled = false;
			last_G = HMI_ValueStruct.coating_thickness;
			DWIN_Draw_Small_Float21(MENUVALUE_X-8, MBASE(MROWS -select_config.index + CONFIG_CASE_COATING), HMI_ValueStruct.coating_thickness);			
			dwinLCD.UpdateLCD();
			coating_thickness = (float)HMI_ValueStruct.coating_thickness/MINUNITMULT;			
			return;
	 }		
		if ((HMI_ValueStruct.coating_thickness - last_G) > 10*MINUNITMULT)
			HMI_ValueStruct.coating_thickness = last_G + 10*MINUNITMULT;
		else if ((last_G - HMI_ValueStruct.coating_thickness) > 10*MINUNITMULT)
			HMI_ValueStruct.coating_thickness = last_G - 10*MINUNITMULT;
		NOLESS(HMI_ValueStruct.coating_thickness, 0);
		NOMORE(HMI_ValueStruct.coating_thickness, 10*MINUNITMULT);		
		DWIN_Draw_Selected_Small_Float21(MENUVALUE_X-8, MBASE(MROWS -select_config.index + CONFIG_CASE_COATING), HMI_ValueStruct.coating_thickness);
		dwinLCD.UpdateLCD();		
	}
}
#endif

#if ENABLED(OPTION_REPEAT_PRINTING)
char Reprint_Buf[50] = {0}; 
void HMI_Reprint_Times() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, ReprintManager.Reprint_times)) {
			DwinMenuID = DWMENU_SET_REPRINT;
			EncoderRate.enabled = false;
			DWIN_Draw_IntValue_Default(4, MENUVALUE_X, MBASE(MROWS -select_reprint.index + REPRINT_CASE_TIMES), ReprintManager.Reprint_times);
			ZERO(Reprint_Buf);
			sprintf_P(Reprint_Buf,PSTR("M180 T%4d"),ReprintManager.Reprint_times);
			queue.inject_P(Reprint_Buf);
			dwinLCD.UpdateLCD();
			return;
		}
		NOLESS(ReprintManager.Reprint_times, 0);
		NOMORE(ReprintManager.Reprint_times, REPEAT_PRINTING_MAX_TIMES);
		DWIN_Draw_Select_IntValue_Default(4, MENUVALUE_X, MBASE(MROWS -select_reprint.index + REPRINT_CASE_TIMES), ReprintManager.Reprint_times);
		dwinLCD.UpdateLCD();
	}
}

void HMI_Forward_Lenght() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, ReprintManager.Forward_lenght)) {
			DwinMenuID = DWMENU_SET_REPRINT;
			EncoderRate.enabled = false;
			DWIN_Draw_IntValue_Default(4, MENUVALUE_X, MBASE(MROWS -select_reprint.index + REPRINT_CASE_LENGHT), ReprintManager.Forward_lenght);
			ZERO(Reprint_Buf);
			sprintf_P(Reprint_Buf,PSTR("M180 L%4d"),ReprintManager.Forward_lenght);
			queue.inject_P(Reprint_Buf);
			dwinLCD.UpdateLCD();
			return;
		}
		NOLESS(ReprintManager.Forward_lenght, 0);
		NOMORE(ReprintManager.Forward_lenght, FORWARD_PRINTING_MAX_LENGHT);
		DWIN_Draw_Select_IntValue_Default(4, MENUVALUE_X, MBASE(MROWS -select_reprint.index + REPRINT_CASE_LENGHT), ReprintManager.Forward_lenght);
		dwinLCD.UpdateLCD();
	}
}
#endif

#if HAS_HOTEND
char E_Buf[50] = {0};
void HMI_Move_Extr(uint8_t extr) {	
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Current_E_Scale[extr])) {
			DwinMenuID = DWMENU_MOVEAXIS;
			EncoderRate.enabled = false;
			last_Extr_scale[extr] = HMI_ValueStruct.Current_E_Scale[extr];
			 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_axis.index + AXISMOVE_CASE_EX1 + extr), HMI_ValueStruct.Current_E_Scale[extr]);
			if(!planner.is_full()) {		 
				planner.synchronize();
				ZERO(E_Buf);
				float temp_E_Coordinate = (float)HMI_ValueStruct.Current_E_Scale[extr]/MINUNITMULT;
				if(temp_E_Coordinate < HMI_ValueStruct.Last_E_Coordinate[extr])
					sprintf_P(E_Buf, PSTR("T%d\nG92 E0\nG1 E-%.1f F100\nG92 E0"), extr, ABS(temp_E_Coordinate - HMI_ValueStruct.Last_E_Coordinate[extr]));
				else
					sprintf_P(E_Buf, PSTR("T%1d\nG92 E0\nG1 E%.1f F100\nG92 E0"), extr, ABS(temp_E_Coordinate - HMI_ValueStruct.Last_E_Coordinate[extr]));				
				HMI_ValueStruct.Last_E_Coordinate[extr] = temp_E_Coordinate;
				queue.inject_P(E_Buf);
		}
		dwinLCD.UpdateLCD();
		return;
	}
	if ((HMI_ValueStruct.Current_E_Scale[extr] - last_Extr_scale[extr]) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
		HMI_ValueStruct.Current_E_Scale[extr] = last_Extr_scale[extr] + (EXTRUDE_MAXLENGTH) * MINUNITMULT;
	else if ((last_Extr_scale[extr] - HMI_ValueStruct.Current_E_Scale[extr]) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
		HMI_ValueStruct.Current_E_Scale[extr] = last_Extr_scale[extr] - (EXTRUDE_MAXLENGTH) * MINUNITMULT;
	
	DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_axis.index + AXISMOVE_CASE_EX1 + extr), HMI_ValueStruct.Current_E_Scale[extr]);
	dwinLCD.UpdateLCD();
	}
}

#if ENABLED(MIXING_EXTRUDER)
void HMI_Move_AllExtr() {
	static float last_EALL_scale = 0;
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
	 	if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Current_EAll_Scale)) {
	  	DwinMenuID = DWMENU_MOVEAXIS;
	  	EncoderRate.enabled = false;
	  	last_EALL_scale = HMI_ValueStruct.Current_EAll_Scale;
			 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_axis.index + AXISMOVE_CASE_EXALL), HMI_ValueStruct.Current_EAll_Scale);
	  	if(!planner.is_full()) {
		    planner.synchronize(); // Wait for planner moves to finish!
				ZERO(E_Buf);
				float temp_E_Coordinate = (float)HMI_ValueStruct.Current_EAll_Scale/MINUNITMULT;
				if(temp_E_Coordinate < HMI_ValueStruct.Last_EAll_Coordinate)
			   	sprintf_P(E_Buf, PSTR("T%d\nG92 E0\nG1 E-%.2f F100\nG92 E0"),MIXING_STEPPERS, ABS(temp_E_Coordinate - HMI_ValueStruct.Last_EAll_Coordinate));
				else
				 	sprintf_P(E_Buf, PSTR("T%d\nG92 E0\nG1 E%.2f F100\nG92 E0"),MIXING_STEPPERS, ABS(temp_E_Coordinate - HMI_ValueStruct.Last_EAll_Coordinate));				
				HMI_ValueStruct.Last_EAll_Coordinate = temp_E_Coordinate;
				queue.inject_P(E_Buf);
	  	}
	  	dwinLCD.UpdateLCD();
	  	return;
	 	}
	 	if ((HMI_ValueStruct.Current_EAll_Scale - last_EALL_scale) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
	  	HMI_ValueStruct.Current_EAll_Scale = last_EALL_scale + (EXTRUDE_MAXLENGTH) * MINUNITMULT;
	 	else if ((last_EALL_scale - HMI_ValueStruct.Current_EAll_Scale) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
	  	HMI_ValueStruct.Current_EAll_Scale = last_EALL_scale - (EXTRUDE_MAXLENGTH) * MINUNITMULT;
		
	 	DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_axis.index + AXISMOVE_CASE_EXALL), HMI_ValueStruct.Current_EAll_Scale);
	 	dwinLCD.UpdateLCD();
	}
}
#endif
#endif

#if ENABLED(BABYSTEPPING)
bool printer_busy() { 
	return planner.movesplanned() || printingIsActive(); 
}

static void Apply_ZOffset(){	
	if((ENABLED(BABYSTEP_WITHOUT_HOMING) || all_axes_known()) && (ENABLED(BABYSTEP_ALWAYS_AVAILABLE) || printer_busy())){				
		babyz_offset = (float)HMI_ValueStruct.Zoffset_Scale/MAXUNITMULT;
		if(last_babyz_offset != babyz_offset){
		   babystep.add_mm(Z_AXIS, babyz_offset - last_babyz_offset);			
			 last_babyz_offset = babyz_offset;
		}
	}		
}

void HMI_Pop_BabyZstep() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Zoffset_Scale)) {
			EncoderRate.enabled = false;			
			Start_PrintProcess();
	  	return;
		}
		NOLESS(HMI_ValueStruct.Zoffset_Scale, -500);
	 	NOMORE(HMI_ValueStruct.Zoffset_Scale, 500); 	
		Apply_ZOffset();
	  DWIN_Draw_Big_Float32(170, 160, HMI_ValueStruct.Zoffset_Scale);
	  dwinLCD.UpdateLCD();
	}
}

void HMI_Zoffset() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	uint8_t zoff_line = select_tune.now + MROWS - select_tune.index;
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Zoffset_Scale)){
			EncoderRate.enabled = false;
			DwinMenuID = DWMENU_TUNE;
			DWIN_Draw_Small_Float22(MENUVALUE_X, MBASE(zoff_line), HMI_ValueStruct.Zoffset_Scale);		
			dwinLCD.UpdateLCD();
			return;
		}
		NOLESS(HMI_ValueStruct.Zoffset_Scale, -500);
		NOMORE(HMI_ValueStruct.Zoffset_Scale, 500);
		Apply_ZOffset();
		DWIN_Draw_Selected_Small_Float22(MENUVALUE_X, MBASE(zoff_line), HMI_ValueStruct.Zoffset_Scale);
		dwinLCD.UpdateLCD();
	}
}
#endif // ENABLED(BABYSTEPPING)

void HMI_SetProbZoffset() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	uint8_t zoff_line = select_leveling.now + MROWS - select_leveling.index;
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.ProbeZoffset_Scale)){
			probe.offset.z = (float)HMI_ValueStruct.ProbeZoffset_Scale/MAXUNITMULT;
			EncoderRate.enabled = false;
			DWIN_Draw_Small_Float22(MENUVALUE_X, MBASE(zoff_line), HMI_ValueStruct.ProbeZoffset_Scale);
			DwinMenuID = DWMENU_LEVELING;
			dwinLCD.UpdateLCD();			
			return;
		}
		NOLESS(HMI_ValueStruct.ProbeZoffset_Scale, (Z_PROBE_OFFSET_RANGE_MIN) * MAXUNITMULT);
		NOMORE(HMI_ValueStruct.ProbeZoffset_Scale, (Z_PROBE_OFFSET_RANGE_MAX) * MAXUNITMULT);		
		DWIN_Draw_Selected_Small_Float22(MENUVALUE_X, MBASE(zoff_line), HMI_ValueStruct.ProbeZoffset_Scale);
		dwinLCD.UpdateLCD();
	}
}

#if HAS_HOTEND
void HMI_ETemp() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		uint8_t temp_line;
		switch (HMI_flag.show_mode) {
			case -1: temp_line = TEMP_CASE_ETEMP + MROWS - select_temp.index; break;
			case -2: temp_line = PREHEAT_CASE_TEMP; break;
			case -3: temp_line = PREHEAT_CASE_TEMP; break;
			default: temp_line = TUNE_CASE_ETEMP + MROWS - select_tune.index;
		}
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.E_Temp)) {
			EncoderRate.enabled = false;
			switch (HMI_flag.show_mode){
				case -1:
					DwinMenuID = DWMENU_TEMPERATURE;
					DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(temp_line), HMI_ValueStruct.E_Temp);
					thermalManager.setTargetHotend(HMI_ValueStruct.E_Temp, 0);
				break;
				
				case -2:
					DwinMenuID = DWMENU_PREHEAT_PLA;
					ui.material_preset[0].hotend_temp = HMI_ValueStruct.E_Temp;
					DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(temp_line), ui.material_preset[0].hotend_temp);
				break;
				
				case -3:
					DwinMenuID = DWMENU_PREHEAT_ABS;
					ui.material_preset[1].hotend_temp = HMI_ValueStruct.E_Temp;
					DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(temp_line), ui.material_preset[1].hotend_temp);
				break;
				
				default:
					DwinMenuID = DWMENU_TUNE;
					DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(temp_line), HMI_ValueStruct.E_Temp);
					thermalManager.setTargetHotend(HMI_ValueStruct.E_Temp, 0);
				break;
			}
			return;
		}
		// E_Temp limit
		NOMORE(HMI_ValueStruct.E_Temp, MAX_E_TEMP);
		NOLESS(HMI_ValueStruct.E_Temp, MIN_E_TEMP);
		// E_Temp value
		DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(temp_line), HMI_ValueStruct.E_Temp);
	}
}
#endif // HAS_HOTEND

#if HAS_HEATED_BED
void HMI_BedTemp() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
		uint8_t bed_line;
		switch (HMI_flag.show_mode) {
			case -1: bed_line = TEMP_CASE_BTEMP + MROWS - select_temp.index; break;
			case -2: bed_line = PREHEAT_CASE_BED; break;
			case -3: bed_line = PREHEAT_CASE_BED; break;
			default: bed_line = TUNE_CASE_BTEMP + MROWS - select_tune.index; break;
		}
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Bed_Temp)) {
			EncoderRate.enabled = false;
			switch (HMI_flag.show_mode) {
				case -1:
					DwinMenuID = DWMENU_TEMPERATURE;
					DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(bed_line), HMI_ValueStruct.Bed_Temp);
					thermalManager.setTargetBed(HMI_ValueStruct.Bed_Temp);
				break;
				
				case -2: 
					DwinMenuID = DWMENU_PREHEAT_PLA;
					ui.material_preset[0].bed_temp = HMI_ValueStruct.Bed_Temp;
					DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(bed_line), ui.material_preset[0].bed_temp);
				break;
				
				case -3: 
					DwinMenuID = DWMENU_PREHEAT_ABS;
					ui.material_preset[1].bed_temp = HMI_ValueStruct.Bed_Temp;
					DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(bed_line), ui.material_preset[1].bed_temp);
				break;
				
				default: 
					DwinMenuID = DWMENU_TUNE;
					DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(bed_line), HMI_ValueStruct.Bed_Temp);
					thermalManager.setTargetBed(HMI_ValueStruct.Bed_Temp);
    		break;
			}   
	   	return;
		}
		// Bed_Temp limit
		NOMORE(HMI_ValueStruct.Bed_Temp, BED_MAX_TARGET);
		NOLESS(HMI_ValueStruct.Bed_Temp, MIN_BED_TEMP);
		// Bed_Temp value
		DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(bed_line), HMI_ValueStruct.Bed_Temp);
	}
}
#endif // HAS_HEATED_BED

#if HAS_PREHEAT
void HMI_FanSpeed() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		uint8_t fan_line;
		switch (HMI_flag.show_mode) {
			case -1: fan_line = TEMP_CASE_FAN + MROWS - select_temp.index; break;
			case -2: fan_line = PREHEAT_CASE_FAN; break;
			case -3: fan_line = PREHEAT_CASE_FAN; break;
			default: fan_line = TUNE_CASE_FAN + MROWS - select_tune.index;
		}
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Fan_speed)) {
			EncoderRate.enabled = false;
			switch (HMI_flag.show_mode){
				case -1: 
					DwinMenuID = DWMENU_TEMPERATURE;
					DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(fan_line), HMI_ValueStruct.Fan_speed);
					thermalManager.set_fan_speed(0, HMI_ValueStruct.Fan_speed);
				break;
				case -2: 
					DwinMenuID = DWMENU_PREHEAT_PLA;
					ui.material_preset[0].fan_speed = HMI_ValueStruct.Fan_speed;
					DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(fan_line), ui.material_preset[0].fan_speed);
				break;

				case -3:
					DwinMenuID = DWMENU_PREHEAT_ABS;
					ui.material_preset[1].fan_speed = HMI_ValueStruct.Fan_speed;
					DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(fan_line), ui.material_preset[1].fan_speed);
				break;
				
				default:
					DwinMenuID = DWMENU_TUNE;
					DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(fan_line), HMI_ValueStruct.Fan_speed);
					thermalManager.set_fan_speed(0, HMI_ValueStruct.Fan_speed);
				break;
			}
			return;	
		}
		// Fan_speed limit
		NOMORE(HMI_ValueStruct.Fan_speed, FANON);
		NOLESS(HMI_ValueStruct.Fan_speed, FANOFF);
		// Fan_speed value
		DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(fan_line), HMI_ValueStruct.Fan_speed);
	}
}
#endif // HAS_PREHEAT

void HMI_PrintSpeed() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.print_speed)) {
			DwinMenuID = DWMENU_TUNE;
			EncoderRate.enabled = false;
			feedrate_percentage = HMI_ValueStruct.print_speed;
			DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(select_tune.now + MROWS - select_tune.index), HMI_ValueStruct.print_speed);
			return;
		}
		// print_speed limit
		NOMORE(HMI_ValueStruct.print_speed, MAX_PRINT_SPEED);
		NOLESS(HMI_ValueStruct.print_speed, MIN_PRINT_SPEED);
		// print_speed value
		DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(select_tune.now + MROWS - select_tune.index), HMI_ValueStruct.print_speed);
	}
}

constexpr float default_max_feedrate[]    = DEFAULT_MAX_FEEDRATE;
constexpr float default_max_acceleration[]  = DEFAULT_MAX_ACCELERATION;
constexpr float default_max_jerk[]      = { DEFAULT_XJERK, DEFAULT_YJERK, DEFAULT_ZJERK, DEFAULT_EJERK };
constexpr float default_axis_steps_per_unit[] = DEFAULT_AXIS_STEPS_PER_UNIT;
void HMI_MaxFeedspeedXYZE() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Max_Feedspeed)) {
			DwinMenuID = DWMENU_SET_MAXSPEED;
			EncoderRate.enabled = false;
			if (WITHIN(HMI_flag.feedspeed_axis, X_AXIS, E_AXIS))
				planner.set_max_feedrate(HMI_flag.feedspeed_axis, HMI_ValueStruct.Max_Feedspeed);
			DWIN_Draw_IntValue_Default(3, CONFIGVALUE_X+8, MBASE(select_feedrate.now), HMI_ValueStruct.Max_Feedspeed);
			return;
		}
		// MaxFeedspeed limit
		if (WITHIN(HMI_flag.feedspeed_axis, X_AXIS, E_AXIS))
			NOMORE(HMI_ValueStruct.Max_Feedspeed, default_max_feedrate[HMI_flag.feedspeed_axis]*2);
		if (HMI_ValueStruct.Max_Feedspeed < MIN_MAXFEEDSPEED) 
			HMI_ValueStruct.Max_Feedspeed = MIN_MAXFEEDSPEED;
		// MaxFeedspeed value
		DWIN_Draw_Select_IntValue_Default(3, CONFIGVALUE_X+8, MBASE(select_feedrate.now), HMI_ValueStruct.Max_Feedspeed);
	}
}

void HMI_MaxAccelerationXYZE() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Max_Acceleration)) {
			DwinMenuID = DWMENU_SET_MAXACC;
			EncoderRate.enabled = false;
			if (HMI_flag.acc_axis == X_AXIS) planner.set_max_acceleration(X_AXIS, HMI_ValueStruct.Max_Acceleration);
			else if (HMI_flag.acc_axis == Y_AXIS) planner.set_max_acceleration(Y_AXIS, HMI_ValueStruct.Max_Acceleration);
			else if (HMI_flag.acc_axis == Z_AXIS) planner.set_max_acceleration(Z_AXIS, HMI_ValueStruct.Max_Acceleration);
		#if HAS_HOTEND
			else if (HMI_flag.acc_axis == E_AXIS) planner.set_max_acceleration(E_AXIS, HMI_ValueStruct.Max_Acceleration);
		#endif
			DWIN_Draw_IntValue_Default(5, CONFIGVALUE_X-8, MBASE(select_accel.now), HMI_ValueStruct.Max_Acceleration);
			return;
		}
		// Max Acceleration limit
		if (WITHIN(HMI_flag.acc_axis, X_AXIS, E_AXIS))
			NOMORE(HMI_ValueStruct.Max_Acceleration, default_max_acceleration[HMI_flag.acc_axis]*2);
		if (HMI_ValueStruct.Max_Acceleration < MIN_MAXACCELERATION) 
			HMI_ValueStruct.Max_Acceleration = MIN_MAXACCELERATION;
		// Max Acceleration value
		DWIN_Draw_Select_IntValue_Default(5, CONFIGVALUE_X-8, MBASE(select_accel.now), HMI_ValueStruct.Max_Acceleration);
	}
}

#if HAS_CLASSIC_JERK
void HMI_MaxJerkXYZE() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Max_Jerk)) {
			DwinMenuID = DWMENU_SET_MAXJERK;
			EncoderRate.enabled = false;
			if(WITHIN(HMI_flag.jerk_axis, X_AXIS, E_AXIS))
				planner.set_max_jerk(HMI_flag.jerk_axis, ((float)HMI_ValueStruct.Max_Jerk / MINUNITMULT));
				DWIN_Draw_Small_Float21(CONFIGVALUE_X+8, MBASE(select_jerk.now), HMI_ValueStruct.Max_Jerk);
				return;
		}
		// Max Jerk limit
		if (WITHIN(HMI_flag.jerk_axis, X_AXIS, E_AXIS)){
			NOMORE(HMI_ValueStruct.Max_Jerk, default_max_jerk[HMI_flag.jerk_axis] * 2 * MINUNITMULT);
			NOLESS(HMI_ValueStruct.Max_Jerk, (MIN_MAXJERK) * MINUNITMULT);
		}
		// Max Jerk value
		DWIN_Draw_Selected_Small_Float21(CONFIGVALUE_X+8, MBASE(select_jerk.now), HMI_ValueStruct.Max_Jerk);
	}
}
#endif // HAS_CLASSIC_JERK

void HMI_StepXYZE() {
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Max_Step)) {
			DwinMenuID = DWMENU_SET_STEPPREMM;
			EncoderRate.enabled = false;
			if (WITHIN(HMI_flag.step_axis, X_AXIS, E_AXIS))
				planner.settings.axis_steps_per_mm[HMI_flag.step_axis] = (float)HMI_ValueStruct.Max_Step / MINUNITMULT;
			DWIN_Draw_Small_Float41(CONFIGVALUE_X-8, MBASE(select_step.now), HMI_ValueStruct.Max_Step);
			return;
		}
		// Step/mm limit
		if (WITHIN(HMI_flag.step_axis, X_AXIS, E_AXIS)){
			NOMORE(HMI_ValueStruct.Max_Step, default_axis_steps_per_unit[HMI_flag.step_axis]*4*MINUNITMULT);
			NOLESS(HMI_ValueStruct.Max_Step, MIN_STEP*MINUNITMULT);
		}
		// Step/mm value
		DWIN_Draw_Selected_Small_Float41(CONFIGVALUE_X-8, MBASE(select_step.now), HMI_ValueStruct.Max_Step);
	}
}

void X_Start_Coordinate_Calculation(uint8_t Extruder_number,uint16_t Coordinate){
	uint8_t j = 0;

	MIXER_STEPPER_LOOP(i){
		if(Coordinate >= 100){
			MixerDis.Extruder_X_Coordinate[i] = MixerDis.Extruder_X_Start_Coordinate[MIXING_STEPPERS]+MixerDis.Extruder_X_Start_Gap[MIXING_STEPPERS]*i; 
			MixerDis.Extruder_Int_Number[i] = 3;
		}
		else if((Coordinate < 100)&&(Coordinate >= 10)){
			MixerDis.Extruder_X_Coordinate[i] = MixerDis.Extruder_X_Start_Coordinate[MIXING_STEPPERS]+7+MixerDis.Extruder_X_Start_Gap[MIXING_STEPPERS]*i;
			MixerDis.Extruder_Int_Number[i] = 2;
		}
		else if(Coordinate < 10){
			MixerDis.Extruder_X_Coordinate[i] = MixerDis.Extruder_X_Start_Coordinate[MIXING_STEPPERS]+14+MixerDis.Extruder_X_Start_Gap[MIXING_STEPPERS]*i;
			MixerDis.Extruder_Int_Number[i] = 1;
		}
		j = i;
  }
	j++;
	if(Extruder_number == MIXING_STEPPERS){
		if((Coordinate < 100)&&(Coordinate >= 10)){ 
			MixerDis.VTool_X_Coordinate = MixerDis.Extruder_X_Start_Coordinate[MIXING_STEPPERS]+7+MixerDis.Extruder_X_Start_Gap[MIXING_STEPPERS]*j;
			MixerDis.VTool_Int_Number = 2;
		}
		else if(Coordinate < 10){
			MixerDis.VTool_X_Coordinate = MixerDis.Extruder_X_Start_Coordinate[MIXING_STEPPERS]+14+MixerDis.Extruder_X_Start_Gap[MIXING_STEPPERS]*j;
			MixerDis.VTool_Int_Number = 1;
		}
	}
}

void Refresh_Percent_display(){	
	dwinLCD.Draw_Rectangle(1, Color_Bg_Black, MixerDis.Area_X_Start, MixerDis.Area_Y_Start, MixerDis.Area_X_End, MixerDis.Area_Y_End);
	MIXER_STEPPER_LOOP(i){
		X_Start_Coordinate_Calculation(i,mixer.mix[i]);
	  dwinLCD.Draw_IntValue(true, true, 0, DWIN_FONT_MIX, Color_White, Color_Bg_Black, MixerDis.Extruder_Int_Number[i], MixerDis.Extruder_X_Coordinate[i], MixerDis.Y_Coordinate, mixer.mix[i]);
		if(mixer.gradient.enabled){
			X_Start_Coordinate_Calculation(MIXING_STEPPERS,10);
			dwinLCD.Draw_String(false, false, DWIN_FONT_MIX, Color_White, Color_Bg_Black, MixerDis.VTool_X_Coordinate, MixerDis.Y_Coordinate, F("Gr"));
		}
		else if(mixer.random_mix.enabled){
			X_Start_Coordinate_Calculation(MIXING_STEPPERS,10);
			dwinLCD.Draw_String(false, false, DWIN_FONT_MIX, Color_White, Color_Bg_Black, MixerDis.VTool_X_Coordinate, MixerDis.Y_Coordinate, F("Rd"));
		}
		else{
      X_Start_Coordinate_Calculation(MIXING_STEPPERS,mixer.selected_vtool);
			dwinLCD.Draw_IntValue(true, true, 0, DWIN_FONT_MIX, Color_White, Color_Bg_Black, MixerDis.VTool_Int_Number, MixerDis.VTool_X_Coordinate, MixerDis.Y_Coordinate, mixer.selected_vtool);
		}
	}
}

//updata Z position
FORCE_INLINE void update_Z_Position(bool bshowICON){	
	static float last_z_pos = -9999.99;	  
	{
		if(bshowICON) DWIN_Show_ICON(ICON_Zoffset, State_icon_Zoffset_X, State_icon_Zoffset_Y);
		if(all_axes_known()){
			if(last_z_pos != current_position.z){
				dwinLCD.Draw_SignedFloatValue(DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_Zoffset_inum, State_text_Zoffset_fnum, State_text_Zoffset_X, State_text_Zoffset_Y, MAXUNITMULT*current_position.z);
				last_z_pos = current_position.z;
			}
		}
		else
			dwinLCD.Draw_String(false, true, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_Zoffset_X,State_text_Zoffset_Y, F("---.-- "));		
	}	
}

FORCE_INLINE void update_variable() {
 #if HAS_HOTEND
  static float last_temp_hotend_target = 0, last_temp_hotend_current = 0;
 #endif
 #if HAS_HEATED_BED
  static float last_temp_bed_target = 0, last_temp_bed_current = 0;
 #endif
 #if HAS_FAN
  static uint8_t last_fan_speed = 0;
 #endif
 #if ENABLED(MIXING_EXTRUDER)
 	static uint8_t last_mixer_percent[MIXING_STEPPERS] = {0};
  static uint8_t last_vtool = 0;
	bool bupdata = false;
 #endif

 /* Tune page temperature update */
	if (DwinMenuID == DWMENU_TUNE){
#if HAS_HOTEND
		if(last_temp_hotend_target != thermalManager.temp_hotend[0].target)
		  DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(TUNE_CASE_ETEMP + MROWS - select_tune.index), thermalManager.temp_hotend[0].target);
#endif
#if HAS_HEATED_BED
		if(last_temp_bed_target != thermalManager.temp_bed.target)
		  DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(TUNE_CASE_BTEMP + MROWS - select_tune.index), thermalManager.temp_bed.target);
#endif
#if HAS_FAN
		if(last_fan_speed != thermalManager.fan_speed[0]) {
		  DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(TUNE_CASE_FAN + MROWS - select_tune.index), thermalManager.fan_speed[0]);
		  last_fan_speed = thermalManager.fan_speed[0];
		}
#endif
	}

 /* Temperature page temperature update */
	if (DwinMenuID == DWMENU_TEMPERATURE) {
#if HAS_HOTEND
		if (last_temp_hotend_target != thermalManager.temp_hotend[0].target)
			DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(TEMP_CASE_ETEMP + MROWS - select_temp.index), thermalManager.temp_hotend[0].target);
#endif
#if HAS_HEATED_BED
		if (last_temp_bed_target != thermalManager.temp_bed.target)
			DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(TEMP_CASE_BTEMP+ MROWS - select_temp.index), thermalManager.temp_bed.target);
#endif
#if HAS_FAN
		if (last_fan_speed != thermalManager.fan_speed[0]) {
			DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(TEMP_CASE_FAN+ MROWS - select_temp.index), thermalManager.fan_speed[0]);
			last_fan_speed = thermalManager.fan_speed[0];
		}
#endif
	}

 /* Bottom temperature update */
#if HAS_HOTEND
	if (last_temp_hotend_current != thermalManager.temp_hotend[0].celsius) {
		DWIN_Draw_IntValue_FONT10(State_text_extruder_num, State_text_extruder_X, State_text_extruder_Y, thermalManager.temp_hotend[0].celsius);
		last_temp_hotend_current = thermalManager.temp_hotend[0].celsius;
	}
	if (last_temp_hotend_target != thermalManager.temp_hotend[0].target) {
		DWIN_Draw_IntValue_FONT10(State_text_extruder_num, State_text_extruder_X + (State_text_extruder_num + 1) * STAT_CHR_W, State_text_extruder_Y, thermalManager.temp_hotend[0].target);
		last_temp_hotend_target = thermalManager.temp_hotend[0].target;
	}
#endif
 
 #if HAS_HEATED_BED
	if (last_temp_bed_current != thermalManager.temp_bed.celsius) {
		DWIN_Draw_IntValue_FONT10(State_text_bed_num, State_text_bed_X, State_text_bed_Y, thermalManager.temp_bed.celsius);
		last_temp_bed_current = thermalManager.temp_bed.celsius;
	}
	if (last_temp_bed_target != thermalManager.temp_bed.target) {
		DWIN_Draw_IntValue_FONT10(State_text_bed_num, State_text_bed_X + (State_text_bed_num + 1) * STAT_CHR_W, State_text_bed_Y, thermalManager.temp_bed.target);
		last_temp_bed_target = thermalManager.temp_bed.target;
	}
 #endif
 
 	/* printing speed*/
	static uint16_t last_speed = 0;
	if (last_speed != feedrate_percentage) {
		DWIN_Draw_IntValue_FONT10(3, State_text_speed_X, State_text_speed_Y, feedrate_percentage);
		last_speed = feedrate_percentage;
	}
	
	/*Z postion*/
	update_Z_Position(true);

	/*Mixing*/
#if ENABLED(MIXING_EXTRUDER)
	if(DwinMenuID == DWMENU_PRINTING){
	//mixing rate changed?
		MIXER_STEPPER_LOOP(i){
			if(last_mixer_percent[i] != mixer.mix[i]){
				bupdata = true;
				break;
			}
		}
		//vool changed?
		if(last_vtool != MixerCfg.Vtool_Backup){
			bupdata = true;
			last_vtool = mixer.selected_vtool = MixerCfg.Vtool_Backup;
			updata_mixer_from_vtool();
			//Draw_Print_ProgressMixModel();
		}

		if(bupdata || HMI_flag.refersh_mix_flag){		
			MIXER_STEPPER_LOOP(i) last_mixer_percent[i] = mixer.mix[i];
			Refresh_Percent_display();
			Draw_Print_ProgressMixModel();
			HMI_flag.refersh_mix_flag = false;
		}
	}	
#endif
}

inline void HMI_SDCardInit() {	
	if (card.flag.workDirIsRoot) {
		//#if !PIN_EXISTS(SD_DETECT)
		card.mount();
		//#endif
	}
	card.cdroot();
}

void HMI_StartFrame(const bool with_update) {	
	Draw_Main_Menu();
	Draw_Status_Area(with_update);
	mixer.selected_vtool = MixerCfg.Vtool_Backup;
	//Draw_Print_ProgressMixModel();
}

/* Main Process */
void HMI_MainMenu() {
 ENCODER_DiffState encoder_diffState = get_encoder_state();
 if (encoder_diffState == ENCODER_DIFF_NO) return;
 HMI_flag.free_close_timer_rg = POWERDOWN_MACHINE_TIMER;
 if (encoder_diffState == ENCODER_DIFF_CW) {
  if (select_main.inc(MAIN_CASE_END)) {
   switch (select_main.now) {
    case MAIN_CASE_PRINT: ICON_Print(); break;
    case MAIN_CASE_PREPARE: ICON_Print(); ICON_Prepare(); break;
    case MAIN_CASE_CONTROL: ICON_Prepare(); ICON_Control(); break;
    case MAIN_CASE_INFO: ICON_Control(); ICON_StartInfo(); break;
   }
  }
 }
 else if (encoder_diffState == ENCODER_DIFF_CCW) {
  if (select_main.dec()) {
   switch (select_main.now) {
    case MAIN_CASE_PRINT: ICON_Print(); ICON_Prepare(); break;
    case MAIN_CASE_PREPARE: ICON_Prepare(); ICON_Control(); break;
    case MAIN_CASE_CONTROL: ICON_Control(); ICON_StartInfo(); break;
    case MAIN_CASE_INFO: ICON_Control(); ICON_StartInfo(); break;
   }
  }
 }
 else if (encoder_diffState == ENCODER_DIFF_ENTER) {
  switch (select_main.now) {
   case MAIN_CASE_PRINT: // Print File
   		if(card.isMounted()){
	    	DwinMenuID = DWMENU_FILE;
				select_file.index = MROWS;
				HMI_SDCardInit();
    		Draw_Print_File_Menu();
   		}
			else{
				buzzer.tone(100, 3000);
				DWIN_Draw_MaskString_Default_Color(Color_Red, 10, 455, F("No SD card? please check!"));
			}
    break;

   case MAIN_CASE_PREPARE: // Prepare
    	DwinMenuID = DWMENU_PREPARE;
    	select_prepare.reset();
    	select_prepare.index = MROWS;
    	Draw_Prepare_Menu();
    break;

   case MAIN_CASE_CONTROL: // Control
    DwinMenuID = DWMENU_CONTROL;
    select_control.reset();
    select_control.index = MROWS;
    Draw_Control_Menu();
    break;

   case MAIN_CASE_INFO: // Leveling or Info
		DwinMenuID = DWMENU_INFO;
		select_info.reset();
		select_info.index = MROWS;
		Draw_Info_Menu();
   	break;
  }
 }
 dwinLCD.UpdateLCD();
}

//
// Watch for media mount / unmount
//
void HMI_SDCardUpdate() {
	if (HMI_flag.lcd_sd_status != card.isMounted()) {
		HMI_flag.lcd_sd_status = card.isMounted();
		// SERIAL_ECHOLNPAIR("HMI_SDCardUpdate: ", int(HMI_flag.lcd_sd_status));
		if(HMI_flag.lcd_sd_status) {
			if(DwinMenuID == DWMENU_FILE) { Redraw_SD_List(); dwinLCD.UpdateLCD();}
		}
		else {
		 // clean file icon
		 if(DwinMenuID == DWMENU_FILE) { Redraw_SD_List();	dwinLCD.UpdateLCD();}
		 else if(DwinMenuID == DWMENU_PRINTING || DwinMenuID == DWMENU_TUNE || printingIsActive()) {
		  	/// TODO: Move card removed abort handling
		  	// to CardReader::manage_media.		  	
		  	wait_for_heatup = wait_for_user = false;
				thermalManager.disable_all_heaters();
				card.flag.abort_sd_printing = true;      		// Let the main loop handle SD abort
			}
		}		
	}
}

// Select (and Print) File
void HMI_SelectFile() {
 ENCODER_DiffState encoder_diffState = get_encoder_state();

 const uint16_t hasUpDir = !card.flag.workDirIsRoot;

 if (encoder_diffState == ENCODER_DIFF_NO) {
  #if ENABLED(SCROLL_LONG_FILENAMES)
   if (shift_ms && select_file.now >= 1 + hasUpDir) {
    // Scroll selected filename every second
    const millis_t ms = millis();
    if (ELAPSED(ms, shift_ms)) {
     const bool was_reset = shift_amt < 0;
     shift_ms = ms + 375UL + was_reset * 250UL; // ms per character
     int8_t shift_new = shift_amt + 1;      // Try to shift by...
     Draw_SDItem_Shifted(shift_new);       // Draw the item
     if (!was_reset && shift_new == 0)      // Was it limited to 0?
      shift_ms = 0;               // No scrolling needed
     else if (shift_new == shift_amt)      // Scroll reached the end
      shift_new = -1;              // Reset
     shift_amt = shift_new;           // Set new scroll
    }
   }
  #endif
  return;
 }

 // First pause is long. Easy.
 // On reset, long pause must be after 0.

 const uint16_t fullCnt = nr_sd_menu_items();

 if (encoder_diffState == ENCODER_DIFF_CW && fullCnt) {
  if (select_file.inc(1 + fullCnt)) {
   const uint8_t itemnum = select_file.now - 1;       // -1 for "Back"
   if (TERN0(SCROLL_LONG_FILENAMES, shift_ms)) {       // If line was shifted
    Erase_Menu_Text(itemnum + MROWS - select_file.index);     // Erase and
    Draw_SDItem(itemnum - 1);                // redraw
   }
	 
   if (select_file.now > MROWS && select_file.now > select_file.index) { // Cursor past the bottom
    select_file.index = select_file.now;              // New bottom line
    Scroll_Menu(DWIN_SCROLL_UP);
    Draw_SDItem(itemnum, MROWS);              // Draw and init the shift name
   }
   else {
    Move_Highlight(1, select_file.now + MROWS - select_file.index); // Just move highlight
    TERN_(SCROLL_LONG_FILENAMES, Init_Shift_Name());     // ...and init the shift name
   }
   TERN_(SCROLL_LONG_FILENAMES, Init_SDItem_Shift());
  }
 }
 else if (encoder_diffState == ENCODER_DIFF_CCW && fullCnt) {
  if (select_file.dec()) {
   const uint8_t itemnum = select_file.now - 1;       // -1 for "Back"
   if (TERN0(SCROLL_LONG_FILENAMES, shift_ms)) {       // If line was shifted
    Erase_Menu_Text(select_file.now + 1 + MROWS - select_file.index); // Erase and
    Draw_SDItem(itemnum + 1);                // redraw
   }
	 
   if (select_file.now < select_file.index - MROWS) {        // Cursor past the top
    select_file.index--;                      // New bottom line
    Scroll_Menu(DWIN_SCROLL_DOWN);
    if (select_file.index == MROWS) {
     Draw_Back_First();
     TERN_(SCROLL_LONG_FILENAMES, shift_ms = 0);
    }
    else {
     Draw_SDItem(itemnum, 0);               // Draw the item (and init shift name)
    }
   }
   else {
    Move_Highlight(-1, select_file.now + MROWS - select_file.index); // Just move highlight
    TERN_(SCROLL_LONG_FILENAMES, Init_Shift_Name());    // ...and init the shift name
   }
   TERN_(SCROLL_LONG_FILENAMES, Init_SDItem_Shift());    // Reset left. Init timer.
  }
 }
 else if (encoder_diffState == ENCODER_DIFF_ENTER) {
  if (select_file.now == 0) { // Back
   select_main.set(MAIN_CASE_PRINT);
   Draw_Main_Menu();
  }
  else if (hasUpDir && select_file.now == 1) { // CD-Up
   SDCard_Up();
   goto HMI_SelectFileExit;
  }
  else {
   const uint16_t filenum = select_file.now - 1 - hasUpDir;
   card.getfilename_sorted(SD_ORDER(filenum, card.get_num_Files()));
	 
   // Enter that folder!
   if (card.flag.filenameIsDir) {
    SDCard_Folder(card.filename);
    goto HMI_SelectFileExit;
   }

		// Reset highlight for next entry
		select_print.reset();
		select_file.reset();

		// Start choice and print SD file
		HMI_flag.heat_flag = true;   
		HMI_flag.show_mode = 0;
		card.openAndPrintFile(card.filename);

		#if FAN_COUNT > 0
		// All fans on for Ender 3 v2 ?
		// The slicer should manage this for us.
		// for (uint8_t i = 0; i < FAN_COUNT; i++)
		// thermalManager.fan_speed[i] = FANON;
		#endif

		#if ENABLED(BABYSTEPPING)
		prevouis_babyz_offset = last_babyz_offset = babyz_offset = 0.0;
		HMI_ValueStruct.Zoffset_Scale = 0;
		#endif
		
		DWIN_status = ID_SM_PRINTING;
		HMI_flag.killtimes = 0;
		Start_PrintProcess();
  }
 }
HMI_SelectFileExit:
 dwinLCD.UpdateLCD();
}

/* Printing */
#if ENABLED(OPTION_AUTOPOWEROFF)
void _setAutoPowerDown(){
	HMI_flag.free_close_timer_rg = POWERDOWN_MACHINE_TIMER;
}
#endif

inline void DWIN_resume_print() {
	TERN_(PARK_HEAD_ON_PAUSE, wait_for_heatup = wait_for_user = false);		
#if ENABLED(PAUSE_HEAT)
	thermalManager.setTargetBed(tempbed);
	thermalManager.setTargetHotend(temphot,0);
#endif
	queue.inject_P(M24_STR);
	#ifdef ACTION_ON_RESUME
	host_action_resume();
	#endif
	print_job_timer.start(); // Also called by M24
}

inline void DWIN_Show_Waiting(){
	buzzer.tone(200, 3000);
	buzzer.tone(10, 0);
	buzzer.tone(200, 3000);				
	Clear_Bottom_Area();
	HMI_flag.clean_status_delay = 3;
	HMI_flag.killtimes++;
	if(HMI_flag.killtimes <= 2){
		DWIN_Draw_MaskString_Default_Color(Color_Red, 10, 455, F("Is processing, please wait!"));		
	}
	else if(HMI_flag.killtimes >= 4){		
		DWIN_Draw_MaskString_Default_Color(Color_Red, 10, 455, F("killed printing!!"));
		HMI_flag.killtimes = 0;
		wait_for_heatup = wait_for_user = false;
		card.flag.abort_sd_printing = true;
	}
	else{
		DWIN_Draw_MaskString_Default_Color(Color_Red, 10, 455, F("Press 3 times to kill"));
	}
}	

void HMI_Printing() {
	ENCODER_DiffState encoder_diffState = get_encoder_state();
	if (encoder_diffState == ENCODER_DIFF_NO) return;	
	// Avoid flicker by updating only the previous menu
	if (encoder_diffState == ENCODER_DIFF_CW) {
		if (select_print.inc(PRINT_CASE_END+1)) {
			switch (select_print.now) {
				case PRINT_CASE_TUNE: 
					ICON_Tune(); 
				break;
				case PRINT_CASE_PAUSE:
					ICON_Tune();
					if (printingIsPaused() && (DWIN_status != ID_SM_RESUMING) && (DWIN_status != ID_SM_PAUSING)) ICON_Continue(); else ICON_Pause();
				break;
				case PRINT_CASE_STOP:
					if (printingIsPaused() && (DWIN_status != ID_SM_RESUMING) && (DWIN_status != ID_SM_PAUSING)) ICON_Continue(); else ICON_Pause();
					ICON_Stop();
				break;
			}
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_CCW) {
		if (select_print.dec()) {
			switch (select_print.now) {
				case PRINT_CASE_TUNE:
					ICON_Tune();
					if (printingIsPaused() && (DWIN_status != ID_SM_RESUMING) && (DWIN_status != ID_SM_PAUSING)) ICON_Continue(); else ICON_Pause();
				break;
				
				case PRINT_CASE_PAUSE:
					if (printingIsPaused() && (DWIN_status != ID_SM_RESUMING) && (DWIN_status != ID_SM_PAUSING)) ICON_Continue(); else ICON_Pause();
					ICON_Stop();
				break;
				
				case PRINT_CASE_STOP: 
					ICON_Stop(); 
				break;
			}
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_ENTER) {
		if(DwinMenuID == DWMENU_POP_STOPPRINT){
			DWIN_status = ID_SM_STOPED;
		}
		else if(DWIN_status == ID_SM_RESUMING || DWIN_status == ID_SM_PAUSING){			
			DWIN_Show_Waiting();			
		}
		else{
			switch (select_print.now) {
				case PRINT_CASE_TUNE: // Tune
					DwinMenuID = DWMENU_TUNE;
					HMI_flag.Is_Mixer_Print = 1;
					HMI_flag.show_mode = 0;
					select_tune.reset();
					select_tune.index = MROWS;
					Draw_Tune_Menu();
				#if ENABLED(BABYSTEPPING)
					Babysteps_timer_first = millis();
				#endif				
				break;

				case PRINT_CASE_PAUSE: // Pause
					if(DWIN_status == ID_SM_PAUSED){
						//resume from Pause			
						ICON_Pause();
						DWIN_resume_print();
						DWIN_status = ID_SM_RESUMING;
					}
					else if(DWIN_status == ID_SM_PRINTING){
						DwinMenuID = DWMENU_POP_PAUSEORSTOP;
						Popup_window_PauseOrStop();
					}					
				break;

				case PRINT_CASE_STOP: // Stop
					if(IS_SD_PRINTING() || IS_SD_PAUSED()){
						DwinMenuID = DWMENU_POP_PAUSEORSTOP;
						Popup_window_PauseOrStop();
					}
					else if(!IS_SD_PRINTING() && !IS_SD_PAUSED()){
						DWIN_status = ID_SM_IDEL;
						Draw_Main_Menu();
					}
				break;

				default: break;
			}
		}
	}
	dwinLCD.UpdateLCD();
}

/* Filament Runout Option window */
void HMI_Filament_Runout_Option() {
	ENCODER_DiffState encoder_diffState = get_encoder_state();
	if (encoder_diffState == ENCODER_DIFF_NO) return;

	if (encoder_diffState == ENCODER_DIFF_CW) {
		if (select_option.inc(2)) ICON_YESorNO(select_option.now);
	}
	else if (encoder_diffState == ENCODER_DIFF_CCW) {
		if (select_option.dec()) ICON_YESorNO(select_option.now);
	}
	else if (encoder_diffState == ENCODER_DIFF_ENTER) {
		switch (select_option.now) {
			case 0: // say yes
				pause_menu_response = PAUSE_RESPONSE_EXTRUDE_MORE;
			break;
			case 1: // say no				
				pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;
				Start_PrintProcess();
			break;
			default: break;
		}
	}
	dwinLCD.UpdateLCD();
}

void HMI_Filament_Runout_Confirm() {
	ENCODER_DiffState encoder_diffState = get_encoder_state();
	if (encoder_diffState == ENCODER_DIFF_NO) return;
	if (encoder_diffState == ENCODER_DIFF_CW) return;
	if (encoder_diffState == ENCODER_DIFF_CCW) return;
	if (encoder_diffState == ENCODER_DIFF_ENTER) {
		DwinMenuID = DWMENU_PRINTING;
		Start_PrintProcess();		
	}	
}

void HMI_Waiting() {
	ENCODER_DiffState encoder_diffState = get_encoder_state();
	if (encoder_diffState == ENCODER_DIFF_NO) return;
	if (encoder_diffState == ENCODER_DIFF_CW) return;
	if (encoder_diffState == ENCODER_DIFF_CCW) return;
	if (encoder_diffState == ENCODER_DIFF_ENTER) {
		DWIN_Show_Waiting();
	}	
}

/* Powerdown window */
void HMI_Powerdown() {
	ENCODER_DiffState encoder_diffState = get_encoder_state();
	if (encoder_diffState == ENCODER_DIFF_NO) return;

	if (encoder_diffState == ENCODER_DIFF_CW) {
		if (select_powerdown.inc(2)) ICON_YESorNO_Powerdown(select_powerdown.now);
	}
	else if (encoder_diffState == ENCODER_DIFF_CCW) {
		if (select_powerdown.dec()) ICON_YESorNO_Powerdown(select_powerdown.now);
	}
	else if (encoder_diffState == ENCODER_DIFF_ENTER) {
		switch (select_powerdown.now) {
			case 0: 
				DwinMenuID = DWMENU_PREPARE;
				Draw_Prepare_Menu();
			break;
			
			case 1: 
				queue.inject_P(PSTR("M81"));
				break;
			default: break;
		}
	}
	dwinLCD.UpdateLCD();
}

/* Pause and Stop window */
void HMI_PauseOrStop() {
	ENCODER_DiffState encoder_diffState = get_encoder_state();
	if(encoder_diffState == ENCODER_DIFF_NO) return;

	if(encoder_diffState == ENCODER_DIFF_CW)
		Draw_Select_Highlight(false);
	else if(encoder_diffState == ENCODER_DIFF_CCW)
		Draw_Select_Highlight(true);
	else if(encoder_diffState == ENCODER_DIFF_ENTER){
		if(select_print.now == PRINT_CASE_PAUSE){
			if(HMI_flag.select_flag){				
			#if ENABLED(PAUSE_HEAT)
				#if HAS_HEATED_BED
			  tempbed = thermalManager.temp_bed.target;
				#endif
				#if HAS_HOTEND
			  temphot = thermalManager.temp_hotend[0].target;
				#endif
			#endif
				Popup_Window_waiting(0);
				DWIN_status = ID_SM_PAUSING;
				#if ENABLED(PARK_HEAD_ON_PAUSE)
				queue.inject_P(PSTR("M25 P\nM24"));
				#else
				queue.inject_P(PSTR("M25"));
				#endif
			}
			else{
				//redraw printing menu
				Start_PrintProcess(); // cancel pause
			}
		}
		else if(select_print.now == PRINT_CASE_STOP){
			if(HMI_flag.select_flag){
				Popup_Window_waiting(1);
				wait_for_heatup = wait_for_user = false;
				card.flag.abort_sd_printing = true;
			#ifdef ACTION_ON_CANCEL
				host_action_cancel();
			#endif
				TERN_(HOST_PROMPT_SUPPORT, host_prompt_open(PROMPT_INFO, PSTR("UI Aborted"), DISMISS_STR));
			}
			else{
				//redraw printing menu
				Start_PrintProcess(); // cancel stop
			}
		}
	}
	dwinLCD.UpdateLCD();
}

inline void HMI_AudioFeedback(const bool success=true) {
	if (success) {
		buzzer.tone(200, 1000);
		buzzer.tone(10, 0);
		buzzer.tone(200, 3000);
	}
	else
		buzzer.tone(20, 1000);
}

/* Prepare */
void HMI_Prepare() {
	ENCODER_DiffState encoder_diffState = get_encoder_state();
	if (encoder_diffState == ENCODER_DIFF_NO) return;

	// Avoid flicker by updating only the previous menu
	if (encoder_diffState == ENCODER_DIFF_CW) {
		if (select_prepare.inc(1 + PREPARE_CASE_TOTAL)) {
			if (select_prepare.now > MROWS && select_prepare.now > select_prepare.index) {
				select_prepare.index = select_prepare.now;

				// Scroll up and draw a blank bottom line
				Scroll_Menu(DWIN_SCROLL_UP);
				Draw_Menu_Icon(MROWS, ICON_Axis + select_prepare.now - 1);
				// Draw "More" icon for sub-menus
				if (select_prepare.index < 7) Draw_More_Icon(MROWS - select_prepare.index + 1);
				if (select_prepare.index == PREPARE_CASE_LANG) Item_Prepare_Lang(MROWS);		
				if (select_prepare.index == PREPARE_CASE_POWERDOWN) Item_Prepare_Powerdown(MROWS);
				if (select_prepare.index == PREPARE_CASE_DISA) Item_Prepare_Disable(MROWS);
				//if (select_prepare.index == PREPARE_CASE_LEVELING) Item_Prepare_Leveling(MROWS);
				//if (select_prepare.index == PREPARE_CASE_MOVE) Item_Prepare_Move(MROWS);
				//if (select_prepare.index == PREPARE_CASE_TEMP) Item_Prepare_Temp(MROWS);
				//if (select_prepare.index == PREPARE_CASE_HOME) Item_Prepare_Home(MROWS);
			}
			else {
				Move_Highlight(1, select_prepare.now + MROWS - select_prepare.index);
			}
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_CCW) {
		if (select_prepare.dec()) {
			if (select_prepare.now < select_prepare.index - MROWS) {
				select_prepare.index--;
				Scroll_Menu(DWIN_SCROLL_DOWN);

				if (select_prepare.index == MROWS)
					Draw_Back_First();
				else
					Draw_Menu_Line(0, ICON_Axis + select_prepare.now - 1);
				if (select_prepare.index < 7) Draw_More_Icon(MROWS - select_prepare.index + 1);

				if (select_prepare.index - MROWS == PREPARE_CASE_HOME) Item_Prepare_Home(0);
				if (select_prepare.index - MROWS == PREPARE_CASE_TEMP ) Item_Prepare_Temp(0);
				if (select_prepare.index - MROWS == PREPARE_CASE_MOVE) Item_Prepare_Move(0);
				//if (select_prepare.index - MROWS == PREPARE_CASE_LEVELING) Item_Prepare_Leveling(0);
				//if (select_prepare.index - MROWS == PREPARE_CASE_DISA) Item_Prepare_Disable(0);				
				//if (select_prepare.index - MROWS == PREPARE_CASE_POWERDOWN) Item_Prepare_Powerdown(0);		
			}
			else {
				Move_Highlight(-1, select_prepare.now + MROWS - select_prepare.index);
			}
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_ENTER) {
		switch (select_prepare.now) {
			case 0: // Back
				select_main.set(MAIN_CASE_PREPARE);
				Draw_Main_Menu();
			break;

			case PREPARE_CASE_HOME: // Homing
				DwinMenuID = DWMENU_HOME;
				select_home.index = MROWS;
				select_home.reset();
				Draw_Home_Menu();	   
			break;

			case PREPARE_CASE_TEMP: // Temperature
				DwinMenuID = DWMENU_TEMPERATURE;
				HMI_flag.show_mode = -1;
				select_temp.reset();
				Draw_Temperature_Menu();
			break;

			case PREPARE_CASE_MOVE: // Axis move
				DwinMenuID = DWMENU_MOVEAXIS;
				select_axis.reset();
				select_axis.index = MROWS;
				Draw_Move_Menu();			
				 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(AXISMOVE_CASE_MOVEX), current_position.x * MINUNITMULT);
				 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(AXISMOVE_CASE_MOVEY), current_position.y * MINUNITMULT);
				 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(AXISMOVE_CASE_MOVEZ), current_position.z * MINUNITMULT);
		#if HAS_HOTEND	   
				 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(AXISMOVE_CASE_EX1), 0);
				 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(AXISMOVE_CASE_EX2), 0);
		#endif
			break;

			case PREPARE_CASE_DISA: // Disable steppers
				queue.inject_P(PSTR("M84"));
			break;

			case PREPARE_CASE_LEVELING: 		// Leveling
				DwinMenuID = DWMENU_LEVELING;
				select_leveling.index = MROWS;
				select_leveling.reset();
				
				Draw_Leveling_Menu();
			break;

			case PREPARE_CASE_POWERDOWN: 		// Powerdown
				DwinMenuID = DWMENU_POWERDOWN;
				Popup_Window_Powerdown();
			break;

			case PREPARE_CASE_LANG: // Toggle Language
				//HMI_SetLanguage();
				//Draw_Prepare_Menu();
				DwinMenuID = DWMENU_LANGUAGE;
				select_language.index = MROWS;
				select_language.reset();
				Draw_Language_Menu();
			break;

			default: break;
		}
	}
	dwinLCD.UpdateLCD();
}

/* Control */
void HMI_Control() {
	ENCODER_DiffState encoder_diffState = get_encoder_state();
	if (encoder_diffState == ENCODER_DIFF_NO) return;

	// Avoid flicker by updating only the previous menu
	if (encoder_diffState == ENCODER_DIFF_CW) {
		if (select_control.inc(1 + CONTROL_CASE_TOTAL)) {
			if (select_control.now > MROWS && select_control.now > select_control.index) {
				select_control.index = select_control.now;
				// Scroll up and draw a blank bottom line
				Scroll_Menu(DWIN_SCROLL_UP);		
				if (select_control.index == CONTROL_CASE_SETPLA ) Item_Control_PLA(MROWS);
				if (select_control.index == CONTROL_CASE_SETABS ) Item_Control_ABS(MROWS);
#if ENABLED(BLTOUCH)
				if(select_control.index == CONTROL_CASE_BLTOUCH) Item_Control_BLtouch(MROWS);
#endif
#if ENABLED(EEPROM_SETTINGS)
				if(select_control.index == CONTROL_CASE_SAVE) Item_Control_Save(MROWS);
				if(select_control.index == CONTROL_CASE_LOAD)	Item_Control_Load(MROWS);
				if(select_control.index == CONTROL_CASE_RESET)	Item_Control_Reset(MROWS);
#endif		
				//if (select_control.index == CONTROL_CASE_INFO) Item_Control_Info(MROWS);
			}
			else {
				Move_Highlight(1, select_control.now + MROWS - select_control.index);
			}
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_CCW) {
		if (select_control.dec()) {
			if (select_control.now < select_control.index - MROWS) {
				select_control.index--;
				Scroll_Menu(DWIN_SCROLL_DOWN);
				if (select_control.index == MROWS) Draw_Back_First();    		
				if (select_control.index - MROWS == CONTROL_CASE_MIXER ) Item_Control_Mixer(0);
				if (select_control.index - MROWS == CONTROL_CASE_CONFIG ) Item_Control_Config(0);
				if (select_control.index - MROWS == CONTROL_CASE_MOTION ) Item_Control_Motion(0);
				if (select_control.index - MROWS == CONTROL_CASE_SETPLA ) Item_Control_PLA(0);
				if (select_control.index - MROWS == CONTROL_CASE_SETABS ) Item_Control_ABS(0);		
			}
			else {
				Move_Highlight(-1, select_control.now + MROWS - select_control.index);
			}
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_ENTER) {
		switch (select_control.now) {
			case 0: // Back
				select_main.set(MAIN_CASE_CONTROL);
				Draw_Main_Menu();
			break;

			case CONTROL_CASE_MIXER: // Mixer
				DwinMenuID = DWMENU_MIXER;
				HMI_flag.Is_Mixer_Print = 0;
				select_mixer.reset();
				Draw_Mixer_Menu();
			break;
			
			case CONTROL_CASE_CONFIG: // Config
				DwinMenuID = DWMENU_CONFIG;
				select_config.reset();
				Draw_Config_Menu();
			break;
			
			case CONTROL_CASE_MOTION: // Motion
				select_motion.reset();
				DwinMenuID = DWMENU_MOTION;    
				Draw_Motion_Menu();
			break;

			case CONTROL_CASE_SETPLA:	// PLA preheat setting
				DwinMenuID = DWMENU_PREHEAT_PLA;
				select_PLA.reset();
				HMI_flag.show_mode = -2;
				Draw_SetPreHeatPLA_Menu();
			break;

			case CONTROL_CASE_SETABS: // ABS preheat setting
				DwinMenuID = DWMENU_PREHEAT_ABS;
				select_ABS.reset();
				HMI_flag.show_mode = -3;
				Draw_SetPreHeatABS_Menu();
			break;	

		#if ENABLED(BLTOUCH)
			case CONTROL_CASE_BLTOUCH: // Bltouch
				DwinMenuID = DWMENU_SET_BLTOUCH;
				select_bltouch.reset();
				Draw_Bltouch_Menu();
			break;
		#endif

		#if ENABLED(EEPROM_SETTINGS)
			case CONTROL_CASE_SAVE: // Write EEPROM    
				HMI_AudioFeedback(settings.save());
			break;
			
			case CONTROL_CASE_LOAD: // Read EEPROM
				HMI_AudioFeedback(settings.load());
			break;
			
			case CONTROL_CASE_RESET: // Reset EEPROM				
				if(card.isMounted()){
					card.closefile();
					card.removeFile("/old_fw.bin");
				}
				settings.reset();
				HMI_AudioFeedback(settings.save());   
				DwinMenuID = DWMENU_CONTROL;
				select_control.reset();
				select_control.index = MROWS;
				Draw_Control_Menu();
				
			break;			
		#endif
	
			case CONTROL_CASE_INFO: // Info
				DwinMenuID = DWMENU_INFO;
				Draw_Info_Menu();
			break;
			
			default: break;
		}
	}
	dwinLCD.UpdateLCD();
}

/* Language */
void HMI_Language() {
	ENCODER_DiffState encoder_diffState = get_encoder_state();
	if (encoder_diffState == ENCODER_DIFF_NO) return;
 
	if (encoder_diffState == ENCODER_DIFF_CW) {
		if (select_language.inc(1 + LANGUAGE_CASE_TOTAL)) {
			if (select_language.now > MROWS && select_language.now > select_language.index) {
				select_language.index = select_language.now;

				// Scroll up and draw a blank bottom line
				Scroll_Menu(DWIN_SCROLL_UP);
				Draw_Menu_Icon(MROWS, LANGUAGE_CASE_EN + select_language.now - 1);
				if(select_language.index == LANGUAGE_CASE_ZH) Item_Language_ZH(MROWS);
			}
			else {
				Move_Highlight(1, select_language.now + MROWS - select_language.index);
			}
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_CCW) {
	  if (select_language.dec()) {
			if (select_language.now < select_language.index - MROWS) {
		    select_language.index--;
		    Scroll_Menu(DWIN_SCROLL_DOWN);

				if (select_language.index == MROWS)
					Draw_Back_First();
				else
					Draw_Menu_Line(0, LANGUAGE_CASE_EN + select_language.now - 1);

				if(select_language.index == LANGUAGE_CASE_ZH) Item_Language_EN(0);
			}
			else {
		    Move_Highlight(-1, select_language.now + MROWS - select_language.index);
			}
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_ENTER) {
		switch (select_language.now) {
			case 0: 										// Back
				DwinMenuID = DWMENU_PREPARE;
				select_prepare.set(PREPARE_CASE_LANG);
				Draw_Prepare_Menu();
			break;

		 case LANGUAGE_CASE_EN: 
		 case LANGUAGE_CASE_SP:
		 case LANGUAGE_CASE_RU:
			HMI_flag.Title_Menu_Backup = 7;
			HMI_flag.language = select_language.now - 1;
			dwinLCD.JPG_CacheToN(1,HMI_flag.language+1);
			HMI_AudioFeedback(settings.save());
			DwinMenuID = DWMENU_PREPARE;
			select_prepare.set(PREPARE_CASE_LANG);
			Draw_Prepare_Menu();
	   break;		
			
		case LANGUAGE_CASE_FR: 
		case LANGUAGE_CASE_PO:
			HMI_flag.Title_Menu_Backup = 6;
			HMI_flag.language = select_language.now - 1;
			dwinLCD.JPG_CacheToN(1,HMI_flag.language+1);
			HMI_AudioFeedback(settings.save());
			DwinMenuID = DWMENU_PREPARE;
			select_prepare.set(PREPARE_CASE_LANG);
			Draw_Prepare_Menu();
		break; 	
			
		case LANGUAGE_CASE_ZH: 
		break;
	 
		default:break;
		} 
	}
	dwinLCD.UpdateLCD();
}

/* Leveling */
char Level_Buf[200]={0};
constexpr uint16_t lfrb[4] = LEVEL_CORNERS_INSET_LFRB;
void HMI_BedLeveling() {
	static bool last_leveling_status = false;	
	ENCODER_DiffState encoder_diffState = get_encoder_state();
	if (encoder_diffState == ENCODER_DIFF_NO) return;
 
	if (encoder_diffState == ENCODER_DIFF_CW) {
		if (select_leveling.inc(1 + HMI_flag.Leveling_Case_Total)) {
			if (select_leveling.now > MROWS && select_leveling.now > select_leveling.index) {
				select_leveling.index = select_leveling.now;
				Scroll_Menu(DWIN_SCROLL_UP);
			#if ENABLED(ABL_GRID)
				if(select_leveling.index == LEVELING_CASE_ACTION) Item_Leveling_Action(MROWS);
				if(select_leveling.index == LEVELING_CASE_PROBEZOFFSET) Item_Leveling_ProbeZoffset(MROWS);
				#if ENABLED(AUTO_UPDATA_PROBE_Z_OFFSET)
				//if(select_leveling.index == LEVELING_CASE_CATCHOFFSET) Item_Leveling_CatechZoffset(MROWS);
				#endif
				//if(select_leveling.index == LEVELING_CASE_POINT4) Item_Leveling_Point4(MROWS);
				//if(select_leveling.index == LEVELING_CASE_POINT3) Item_Leveling_Point3(MROWS);
				//if(select_leveling.index == LEVELING_CASE_POINT2) Item_Leveling_Point2(MROWS);
				//if(select_leveling.index == LEVELING_CASE_POINT1) Item_Leveling_Point1(MROWS);
			#endif
				
			}
			else {
				Move_Highlight(1, select_leveling.now + MROWS - select_leveling.index);
			}
		}
	}
	else if(encoder_diffState == ENCODER_DIFF_CCW) {
			if(select_leveling.dec()) {
				if(select_leveling.now < select_leveling.index - MROWS) {
					select_leveling.index--;
					Scroll_Menu(DWIN_SCROLL_DOWN);
					if (select_leveling.index == MROWS) Draw_Back_First();
					else Draw_Menu_Line(0, ICON_Leveling_Point1 + select_leveling.now - 1);
					if(select_leveling.index - MROWS == LEVELING_CASE_POINT1) Item_Leveling_Point1(0);
					if(select_leveling.index - MROWS == LEVELING_CASE_POINT2) Item_Leveling_Point2(0);
					//if(select_leveling.index - MROWS == LEVELING_CASE_POINT3) Item_Leveling_Point3(0);
					//if(select_leveling.index - MROWS == LEVELING_CASE_POINT4) Item_Leveling_Point4(0);
					//if(select_leveling.index - MROWS == LEVELING_CASE_CATCHOFFSET) Item_Leveling_CatechZoffset(0);
					//if(select_leveling.index - MROWS == LEVELING_CASE_PROBEZOFFSET) Item_Leveling_ProbeZoffset(0);
					//if(select_leveling.index - MROWS == LEVELING_CASE_ACTION) Item_Leveling_Action(0);
				}
			else {
				Move_Highlight(-1, select_leveling.now + MROWS - select_leveling.index);
			}
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_ENTER) {
		if(DwinMenuID == DWMENU_LEVEL_DONECONFIRM){
			DwinMenuID = DWMENU_LEVELING;
		#if ENABLED(AUTO_UPDATA_PROBE_Z_OFFSET)
			select_leveling.set(LEVELING_CASE_CATCHOFFSET);
		#else
			select_leveling.set(LEVELING_CASE_POINT1);
		#endif			
			Draw_Leveling_Menu();
			Draw_Status_Area(true);
			encoder_diffState = ENCODER_DIFF_NO;
			return;
		}
		switch(select_leveling.now) {
			case 0: 										// Back
		    DwinMenuID = DWMENU_PREPARE;
				Clear_Bottom_Area();
				select_prepare.set(PREPARE_CASE_LEVELING);
				Draw_Prepare_Menu();
				HMI_flag.need_home_flag = true;
	    break;
		
			case LEVELING_CASE_POINT1: 										
				DwinMenuID = DWMENU_LEVELING;								
				Clear_Bottom_Area();			
				last_leveling_status = planner.leveling_active;
				set_bed_leveling_enabled(false);				
				ZERO(Level_Buf);
				if(HMI_flag.need_home_flag)
					sprintf_P(Level_Buf,PSTR("G28\nG91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,lfrb[0],lfrb[1],3000,0,500);
				else
					sprintf_P(Level_Buf,PSTR("G28O\nG91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,lfrb[0],lfrb[1],3000,0,500);
				queue.inject_P(Level_Buf);
				planner.synchronize();
				HMI_flag.need_home_flag = false;
				set_bed_leveling_enabled(last_leveling_status);
			break;
				
		 case LEVELING_CASE_POINT2: 										
				DwinMenuID = DWMENU_LEVELING;				
				Clear_Bottom_Area();

				last_leveling_status = planner.leveling_active;
				set_bed_leveling_enabled(false);
				ZERO(Level_Buf);
				if(HMI_flag.need_home_flag)
					sprintf_P(Level_Buf,PSTR("G28\nG91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,X_BED_SIZE-lfrb[2],lfrb[1],3000,0,500);	
				else
					sprintf_P(Level_Buf,PSTR("G28O\nG91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,X_BED_SIZE-lfrb[2],lfrb[1],3000,0,500);	
				queue.inject_P(Level_Buf);
				planner.synchronize();
				HMI_flag.need_home_flag = false;
				set_bed_leveling_enabled(last_leveling_status);
	    break;
				
			case LEVELING_CASE_POINT3: 										
				DwinMenuID = DWMENU_LEVELING;				
				Clear_Bottom_Area();				
				last_leveling_status = planner.leveling_active;
				set_bed_leveling_enabled(false);
				ZERO(Level_Buf);
				if(HMI_flag.need_home_flag)
					sprintf_P(Level_Buf,PSTR("G28\nG91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,X_BED_SIZE-lfrb[2],Y_BED_SIZE-lfrb[3],3000,0,500);
				else
					sprintf_P(Level_Buf,PSTR("G28O\nG91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,X_BED_SIZE-lfrb[2],Y_BED_SIZE-lfrb[3],3000,0,500);
				queue.inject_P(Level_Buf);
				planner.synchronize();
				HMI_flag.need_home_flag = false;
				set_bed_leveling_enabled(last_leveling_status);
	    break;
		
			case LEVELING_CASE_POINT4: 										
				DwinMenuID = DWMENU_LEVELING;								
				Clear_Bottom_Area();

				last_leveling_status = planner.leveling_active;
				set_bed_leveling_enabled(false);
				ZERO(Level_Buf);
				if(HMI_flag.need_home_flag)
					sprintf_P(Level_Buf,PSTR("G28\nG91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,lfrb[0],Y_BED_SIZE-lfrb[3],3000,0,500);
				else
					sprintf_P(Level_Buf,PSTR("G28O\nG91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,lfrb[0],Y_BED_SIZE-lfrb[3],3000,0,500);
				queue.inject_P(Level_Buf);
				planner.synchronize();
				HMI_flag.need_home_flag = false;
				set_bed_leveling_enabled(last_leveling_status);
		break;
		
	#if ENABLED(ABL_GRID)
		#if ENABLED(AUTO_UPDATA_PROBE_Z_OFFSET)
		case LEVELING_CASE_CATCHOFFSET:
			#if ENABLED(OPTION_BED_COATING)
				if((DwinMenuID == DWMENU_POP_LEVEL_CATCH) || (coating_thickness < 2))
			#endif
				{
					DwinMenuID = DWMENU_LEVEL_CATCHOFFSET;					
					set_bed_leveling_enabled(false);
					Clear_Bottom_Area();
					Popup_Window_CatchOffset();
					DWIN_G29_Show_Messge(G29_CATCH_START);					
					queue.inject_P(PSTR("G28\nG29 N\n"));	
				}
			#if ENABLED(OPTION_BED_COATING)
	  	 	else{
					DwinMenuID = DWMENU_POP_LEVEL_CATCH;
					Popup_Remove_Glass();
	  	 	}
			#endif
				planner.synchronize();
				HMI_flag.need_home_flag = false;
			break;
		#endif

		case LEVELING_CASE_PROBEZOFFSET:
			DwinMenuID = DWMENU_LEVEL_SETOFFSET;
    	HMI_ValueStruct.ProbeZoffset_Scale = probe.offset.z*MAXUNITMULT;
    	DWIN_Draw_Selected_Small_Float22(MENUVALUE_X, MBASE(select_leveling.now + MROWS - select_leveling.index), HMI_ValueStruct.ProbeZoffset_Scale);
    	EncoderRate.enabled = true;
		break;
		
	 	case LEVELING_CASE_ACTION:
			DwinMenuID = DWMENU_LEVEL_BEDLEVELING;
			set_bed_leveling_enabled(false);
			Clear_Bottom_Area();
			Popup_Window_Leveling();
			DWIN_G29_Show_Messge(G29_MESH_START);
			if(HMI_flag.need_home_flag)
				queue.inject_P(PSTR("G28\nG29\n"));
			else
				queue.inject_P(PSTR("G28O\nG29\n"));
			planner.synchronize();
			HMI_flag.need_home_flag = false;
		break;
 #endif
		
		default:break;
		} 
	}
	dwinLCD.UpdateLCD();
}

/* Home */
void HMI_Home() {
	ENCODER_DiffState encoder_diffState = get_encoder_state();
	if (encoder_diffState == ENCODER_DIFF_NO) return;

	// Avoid flicker by updating only the previous menu
	if (encoder_diffState == ENCODER_DIFF_CW) {
		if (select_home.inc(1 + HOME_CASE_TOTAL)) {
			Move_Highlight(1, select_home.now);
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_CCW) {
		if (select_home.dec()) {
			Move_Highlight(-1, select_home.now);
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_ENTER) {
		switch (select_home.now) {
			case 0: 										// Back
				DwinMenuID = DWMENU_PREPARE;
				select_leveling.set(PREPARE_CASE_HOME);
				Draw_Prepare_Menu();
			break;

			case HOME_CASE_ALL: 										
				DwinMenuID = DWMENU_POP_HOME;
				select_home.index = MROWS;
				queue.inject_P(PSTR("G28")); 	// G28 will set home_flag
				Popup_Window_HomeAll();
			break;

			case HOME_CASE_X: 										
				DwinMenuID = DWMENU_POP_HOME;
				select_home.index = MROWS;
				queue.inject_P(TERN(HOME_Y_BEFORE_X, PSTR("G28Y\nG28X"),PSTR("G28X")));
				Popup_Window_HomeX();
			break;

			case HOME_CASE_Y: 										
				DwinMenuID = DWMENU_POP_HOME;
				select_home.index = MROWS;
				queue.inject_P(TERN(HOME_X_BEFORE_Y, PSTR("G28X\nG28Y"),PSTR("G28Y")));
				Popup_Window_HomeY();
			break;

			case HOME_CASE_Z: 										
				DwinMenuID = DWMENU_POP_HOME;
				select_home.index = MROWS;
				queue.inject_P(PSTR("G28 Z0"));
				Popup_Window_HomeZ();
			break;

			default:break;
		} 
	}
	dwinLCD.UpdateLCD();
}

/* Axis Move */
void HMI_AxisMove() {
	ENCODER_DiffState encoder_diffState = get_encoder_state();
	if (encoder_diffState == ENCODER_DIFF_NO) return;
 
	#if ENABLED(PREVENT_COLD_EXTRUSION)
	// popup window resume
	if (HMI_flag.ETempTooLow_flag) {
		if (encoder_diffState == ENCODER_DIFF_ENTER) {
			HMI_flag.ETempTooLow_flag = false;
			DwinMenuID = DWMENU_MOVEAXIS;
			select_axis.reset();
			select_axis.index = MROWS;
			Draw_Move_Menu();

			HMI_ValueStruct.Move_X_scale = current_position.x * MINUNITMULT;
			 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(1), HMI_ValueStruct.Move_X_scale);
			HMI_ValueStruct.Move_Y_scale = current_position.y * MINUNITMULT;
			 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(2), HMI_ValueStruct.Move_Y_scale);
			HMI_ValueStruct.Move_Z_scale = current_position.z * MINUNITMULT;
			 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(3), HMI_ValueStruct.Move_Z_scale);
			 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(4), HMI_ValueStruct.Current_E_Scale[0]);
			 DWIN_Draw_Small_Float31(MENUVALUE_X, MBASE(5), HMI_ValueStruct.Current_E_Scale[1]);
			dwinLCD.UpdateLCD();
		}
		return;
	}
 #endif

 // Avoid flicker by updating only the previous menu
	if (encoder_diffState == ENCODER_DIFF_CW) {
		if (select_axis.inc(1 + AXISMOVE_CASE_TOTAL)) {
			if (select_axis.now > MROWS && select_axis.now > select_axis.index) {
				select_axis.index = select_axis.now;

				// Scroll up and draw a blank bottom line
				Scroll_Menu(DWIN_SCROLL_UP);
				//Draw_Menu_Icon(MROWS, ICON_MoveX + select_axis.now - 1);
				if (select_axis.index == AXISMOVE_CASE_EXALL) Item_Axis_MoveEXAll(MROWS);
				if (select_axis.index == AXISMOVE_CASE_EX4) Item_Axis_MoveEX4(MROWS);
				if(select_axis.index == AXISMOVE_CASE_EX3)	Item_Axis_MoveEX3(MROWS);
				//if(select_axis.index == AXISMOVE_CASE_EX2)	Item_Axis_MoveEX2(MROWS);	
				//if(select_axis.index == AXISMOVE_CASE_EX1)	Item_Axis_MoveEX1(MROWS);	
			}
			else {
				Move_Highlight(1, select_axis.now + MROWS - select_axis.index);
			}
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_CCW) {
		if (select_axis.dec()) {
			if (select_axis.now < select_axis.index - MROWS) {
				select_axis.index--;
				Scroll_Menu(DWIN_SCROLL_DOWN);

				if (select_axis.index == MROWS)
					Draw_Back_First();
				else
					Draw_Menu_Line(0, ICON_MoveX + select_axis.now - 1);
				if(select_axis.index - MROWS == AXISMOVE_CASE_MOVEX) Item_Axis_MoveX(0);
				if(select_axis.index - MROWS == AXISMOVE_CASE_MOVEY) Item_Axis_MoveY(0);
				if(select_axis.index - MROWS == AXISMOVE_CASE_MOVEZ) Item_Axis_MoveZ(0);
				//if(select_axis.index - MROWS == AXISMOVE_CASE_EX1) Item_Axis_MoveEX1(0);
			}
			else {
				Move_Highlight(-1, select_axis.now + MROWS - select_axis.index);
			}
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_ENTER) {
		switch (select_axis.now) {
			case 0: // Back
				DwinMenuID = DWMENU_PREPARE;
				select_prepare.set(PREPARE_CASE_MOVE);
				select_prepare.index = MROWS;
				Draw_Prepare_Menu();
			break;
			
			case AXISMOVE_CASE_MOVEX: // X axis move
				DwinMenuID = DWMENU_MOVEX;
				HMI_ValueStruct.Move_X_scale = current_position.x * MINUNITMULT;
				DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_axis.index + AXISMOVE_CASE_MOVEX), HMI_ValueStruct.Move_X_scale);
				EncoderRate.enabled = true;
			break;
				
			case AXISMOVE_CASE_MOVEY: // Y axis move
				DwinMenuID = DWMENU_MOVEY;
				HMI_ValueStruct.Move_Y_scale = current_position.y * MINUNITMULT;
				DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_axis.index + AXISMOVE_CASE_MOVEY), HMI_ValueStruct.Move_Y_scale);
				EncoderRate.enabled = true;
			break;
			
			case AXISMOVE_CASE_MOVEZ: // Z axis move
				DwinMenuID = DWMENU_MOVEZ;
				HMI_ValueStruct.Move_Z_scale = current_position.z * MINUNITMULT;
				DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_axis.index + AXISMOVE_CASE_MOVEZ), HMI_ValueStruct.Move_Z_scale);
				EncoderRate.enabled = true;
			break;

	#if HAS_HOTEND
			case AXISMOVE_CASE_EX1: // Extruder1
				// window tips
				#if ENABLED(PREVENT_COLD_EXTRUSION)
				if (thermalManager.temp_hotend[0].celsius < EXTRUDE_MINTEMP) {
					HMI_flag.ETempTooLow_flag = true;
					Popup_Window_ETempTooLow();
					dwinLCD.UpdateLCD();
					return;
				}
				#endif		
				DwinMenuID = DWMENU_MOVE_EXT1;
				HMI_ValueStruct.Current_E_Scale[0] = HMI_ValueStruct.Last_E_Coordinate[0]*MINUNITMULT;
				DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_axis.index + AXISMOVE_CASE_EX1), HMI_ValueStruct.Current_E_Scale[0]);
				EncoderRate.enabled = true;
			break;

		#if(E_STEPPERS > 1)
			case AXISMOVE_CASE_EX2: // Extruder2
			// window tips
			#if ENABLED(PREVENT_COLD_EXTRUSION)
				if (thermalManager.temp_hotend[0].celsius < EXTRUDE_MINTEMP) {
					HMI_flag.ETempTooLow_flag = true;
					Popup_Window_ETempTooLow();
					dwinLCD.UpdateLCD();
					return;
				}
			#endif
				DwinMenuID = DWMENU_MOVE_EXT2;
				HMI_ValueStruct.Current_E_Scale[1] = HMI_ValueStruct.Last_E_Coordinate[1]*MINUNITMULT;
				DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_axis.index + AXISMOVE_CASE_EX2), HMI_ValueStruct.Current_E_Scale[1]);
				EncoderRate.enabled = true;
			break;
		#endif//(E_STEPPERS > 1)

		#if(E_STEPPERS > 2)
			case AXISMOVE_CASE_EX3: // Extruder3
			// window tips
			#if ENABLED(PREVENT_COLD_EXTRUSION)
				if (thermalManager.temp_hotend[0].celsius < EXTRUDE_MINTEMP) {
					HMI_flag.ETempTooLow_flag = true;
					Popup_Window_ETempTooLow();
					dwinLCD.UpdateLCD();
					return;
				}
			#endif
				DwinMenuID = DWMENU_MOVE_EXT3;
				HMI_ValueStruct.Current_E_Scale[2] = HMI_ValueStruct.Last_E_Coordinate[2]*MINUNITMULT;
				DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_axis.index + AXISMOVE_CASE_EX3), HMI_ValueStruct.Current_E_Scale[2]);
				EncoderRate.enabled = true;
			break;
		#endif//(E_STEPPERS > 2)

		#if(E_STEPPERS > 3)
			case AXISMOVE_CASE_EX4: // Extruder4
			// window tips
			#if ENABLED(PREVENT_COLD_EXTRUSION)
				if (thermalManager.temp_hotend[0].celsius < EXTRUDE_MINTEMP) {
					HMI_flag.ETempTooLow_flag = true;
					Popup_Window_ETempTooLow();
					dwinLCD.UpdateLCD();
					return;
				}
			#endif
				DwinMenuID = DWMENU_MOVE_EXT4;
				HMI_ValueStruct.Current_E_Scale[3] = HMI_ValueStruct.Last_E_Coordinate[3]*MINUNITMULT;
				DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_axis.index + AXISMOVE_CASE_EX4), HMI_ValueStruct.Current_E_Scale[3]);
				EncoderRate.enabled = true;
			break;
		#endif//(E_STEPPERS > 3)

		#if ENABLED(MIXING_EXTRUDER)
			case AXISMOVE_CASE_EXALL: // Extruderall
			// window tips
			#if ENABLED(PREVENT_COLD_EXTRUSION)
				if (thermalManager.temp_hotend[0].celsius < EXTRUDE_MINTEMP) {
					HMI_flag.ETempTooLow_flag = true;
					Popup_Window_ETempTooLow();
					dwinLCD.UpdateLCD();
					return;
				}
			#endif
				DwinMenuID = DWMENU_MOVE_EXTALL;
				HMI_ValueStruct.Current_EAll_Scale = HMI_ValueStruct.Last_EAll_Coordinate*MINUNITMULT;
				DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_axis.index + AXISMOVE_CASE_EXALL), HMI_ValueStruct.Current_EAll_Scale);
				EncoderRate.enabled = true;
			break;
		#endif //end if enable(MIXING_EXTRUDER)
	#endif //end HAS_HOTEND
		}
	}
	dwinLCD.UpdateLCD();
}



/* TemperatureID */
void HMI_Temperature() {
 ENCODER_DiffState encoder_diffState = get_encoder_state();
 if (encoder_diffState == ENCODER_DIFF_NO) return;

 // Avoid flicker by updating only the previous menu
	if (encoder_diffState == ENCODER_DIFF_CW) {
		if (select_temp.inc(1 + TEMP_CASE_TOTAL)){
			if (select_temp.now > MROWS && select_temp.now > select_temp.index) {
				select_temp.index = select_temp.now;
				// Scroll up and draw a blank bottom line
				Scroll_Menu(DWIN_SCROLL_UP);
				if (select_temp.index == TEMP_CASE_COOL) Item_Temperature_Cool(MROWS);
			}
			else {
				Move_Highlight(1, select_temp.now + MROWS - select_temp.index);
			}
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_CCW) {
		if (select_temp.dec()) {
			if (select_temp.now < select_temp.index - MROWS) {
		    select_temp.index--;
		    Scroll_Menu(DWIN_SCROLL_DOWN);
		    if (select_temp.index == MROWS) Draw_Back_First();				
				if (select_temp.index - MROWS == TEMP_CASE_ETEMP) Item_Temperature_ETemp(0);
				if (select_temp.index - MROWS == TEMP_CASE_BTEMP ) Item_Temperature_BTemp(0);
			}
			else {
				Move_Highlight(-1, select_temp.now + MROWS - select_temp.index);
			}
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_ENTER) {
	  switch (select_temp.now) {
	   case 0: // Back
	    DwinMenuID = DWMENU_PREPARE;
	    select_prepare.set(PREPARE_CASE_TEMP);
	    select_control.index = MROWS;
	    Draw_Prepare_Menu();
	    break;
			
	   #if HAS_HOTEND
	    case TEMP_CASE_ETEMP: // Nozzle temperature
	     DwinMenuID = DWMENU_SET_ETMP;
	     HMI_ValueStruct.E_Temp = thermalManager.temp_hotend[0].target;
	     DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(TEMP_CASE_ETEMP + MROWS - select_temp.index), thermalManager.temp_hotend[0].target);
	     EncoderRate.enabled = true;
	     break;
	   #endif
		 
	   #if HAS_HEATED_BED
	    case TEMP_CASE_BTEMP: // Bed temperature
	     DwinMenuID = DWMENU_SET_BTMP;
	     HMI_ValueStruct.Bed_Temp = thermalManager.temp_bed.target;
	     DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(TEMP_CASE_BTEMP + MROWS - select_temp.index), thermalManager.temp_bed.target);
	     EncoderRate.enabled = true;
	     break;
	   #endif
		 
	   #if HAS_FAN
	    case TEMP_CASE_FAN: // Fan speed
	     DwinMenuID = DWMENU_SET_FANSPEED;
	     HMI_ValueStruct.Fan_speed = thermalManager.fan_speed[0];
	     DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(TEMP_CASE_FAN + MROWS - select_temp.index), thermalManager.fan_speed[0]);
	     EncoderRate.enabled = true;
	     break;
	   #endif
		 
		#if (HAS_HOTEND && HAS_PREHEAT)
			case TEMP_CASE_PREHEATPLA: // PLA preheat
				thermalManager.setTargetHotend(ui.material_preset[0].hotend_temp, 0);
				thermalManager.setTargetBed(ui.material_preset[0].bed_temp);
				thermalManager.set_fan_speed(0, ui.material_preset[0].fan_speed);
			break;
						
			case TEMP_CASE_PREHEATABS: // ABS preheat
				thermalManager.setTargetHotend(ui.material_preset[1].hotend_temp, 0);
				thermalManager.setTargetBed(ui.material_preset[1].bed_temp);
				thermalManager.set_fan_speed(0, ui.material_preset[1].fan_speed);
			break;
	 #endif
	 
		#if HAS_PREHEAT
			case TEMP_CASE_COOL: // Cool
				TERN_(HAS_FAN, thermalManager.zero_fan_speeds());
				thermalManager.disable_all_heaters();
			break;
		#endif
		}
	}
	dwinLCD.UpdateLCD();
}

/* Motion */
void HMI_Motion() {
	ENCODER_DiffState encoder_diffState = get_encoder_state();
	if (encoder_diffState == ENCODER_DIFF_NO) return;

	// Avoid flicker by updating only the previous menu
	if (encoder_diffState == ENCODER_DIFF_CW) {
		if (select_motion.inc(1 + MOTION_CASE_TOTAL)) Move_Highlight(1, select_motion.now);
	}
	else if (encoder_diffState == ENCODER_DIFF_CCW) {
		if (select_motion.dec()) Move_Highlight(-1, select_motion.now);
	}
	else if (encoder_diffState == ENCODER_DIFF_ENTER) {
		switch (select_motion.now) {
		case 0: // Back
			DwinMenuID = DWMENU_CONTROL;
			select_control.set(CONTROL_CASE_MOTION);
			select_control.index = MROWS;
			Draw_Control_Menu();
		break;
		
		case MOTION_CASE_RATE:  // Max speed
			DwinMenuID = DWMENU_SET_MAXSPEED;
			select_feedrate.reset();
			Draw_Max_Speed_Menu();
		break;
		
		case MOTION_CASE_ACCEL: // Max acceleration
			DwinMenuID = DWMENU_SET_MAXACC;
			select_accel.reset();
			Draw_Max_Accel_Menu();
		break;
		
	#if HAS_CLASSIC_JERK
		case MOTION_CASE_JERK: // Max jerk
			DwinMenuID = DWMENU_SET_MAXJERK;
			select_jerk.reset();
			Draw_Max_Jerk_Menu();
		break;
	#endif
	
		case MOTION_CASE_STEPS: // Steps per mm
			DwinMenuID = DWMENU_SET_STEPPREMM;
			select_step.reset();
			Draw_Steps_Menu();
		break;
		
		default: break;
		}
	}
	dwinLCD.UpdateLCD();
}


/* Bltouch */
#if ENABLED(BLTOUCH)
void HMI_Option_Bltouch() {
	ENCODER_DiffState encoder_diffState = get_encoder_state();
	if (encoder_diffState == ENCODER_DIFF_NO) return;

	// Avoid flicker by updating only the previous menu
	if (encoder_diffState == ENCODER_DIFF_CW) {
		if (select_bltouch.inc(1 + BLTOUCH_CASE_TOTAL)) {
			Move_Highlight(1, select_bltouch.now);
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_CCW) {
		if (select_bltouch.dec()) {
			Move_Highlight(-1, select_bltouch.now);
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_ENTER) {
		switch (select_bltouch.now) {
			case 0: 					// Back
				DwinMenuID = DWMENU_CONTROL;
				select_control.set(CONTROL_CASE_BLTOUCH);
				select_control.index = MROWS;
				Draw_Control_Menu();
			break;

			case BLTOUCH_CASE_RESET: 	// Reset
				DwinMenuID = DWMENU_SET_BLTOUCH;
				bltouch._reset();
			break;

			case BLTOUCH_CASE_TEST: 	// Test
					DwinMenuID = DWMENU_SET_BLTOUCH;
					bltouch._selftest();
			break;

			case BLTOUCH_CASE_STOW: 	// Stow
					DwinMenuID = DWMENU_SET_BLTOUCH;
					bltouch._stow();
			break;

			case BLTOUCH_CASE_DEPLOY: 	// Proc
					DwinMenuID = DWMENU_SET_BLTOUCH;
					bltouch._deploy();
			break;
			
			case BLTOUCH_CASE_SW: 	// sw
				DwinMenuID = DWMENU_SET_BLTOUCH;
				bltouch._set_SW_mode();
			break;
			
			default: break;
		}
	}
	dwinLCD.UpdateLCD();
}
#endif

#if ENABLED(OPTION_WIFI_MODULE)
/* wifi */
void HMI_Wifi() {
	ENCODER_DiffState encoder_diffState = get_encoder_state();
	if (encoder_diffState == ENCODER_DIFF_NO) return;

	if (encoder_diffState == ENCODER_DIFF_ENTER) {
		if(select_main.now == MAIN_CASE_PREPARE) 
			select_main.set(MAIN_CASE_PREPARE);
		else 
			select_main.set(MAIN_CASE_PRINT);
		Draw_Main_Menu();
	}
	dwinLCD.UpdateLCD();
}
#endif

/* MIXER */
void HMI_Mixer() {
	ENCODER_DiffState encoder_diffState = get_encoder_state();
	if (encoder_diffState == ENCODER_DIFF_NO) return;

	// Avoid flicker by updating only the previous menu
	if (encoder_diffState == ENCODER_DIFF_CW) {
		if (select_mixer.inc(1 + MIXER_CASE_TOTAL)) Move_Highlight(1, select_mixer.now);
	}
	else if (encoder_diffState == ENCODER_DIFF_CCW) {
		if (select_mixer.dec()) Move_Highlight(-1, select_mixer.now);
	}
	else if (encoder_diffState == ENCODER_DIFF_ENTER){
		switch (select_mixer.now){
			case 0: 																// Back
			if(!HMI_flag.Is_Mixer_Print){
				DwinMenuID = DWMENU_CONTROL;
				select_control.set(CONTROL_CASE_MIXER);
				select_control.index = MROWS;
				Draw_Control_Menu();
			}
			else{
				DwinMenuID = DWMENU_TUNE;
				HMI_flag.show_mode = 0;
				select_tune.set(TUNE_CASE_SPEED);
				select_tune.index = MROWS;
				Draw_Tune_Menu();
			}
		break;

			case MIXER_CASE_MANUAL: 	// Manual
				DwinMenuID = DWMENU_MIXER_MANUAL;
				select_manual.reset();
				MixerCfg.Vtool_Backup = mixer.selected_vtool;
				updata_mixer_from_vtool();
				MIXER_STEPPER_LOOP(i) {MixerCfg.Manual_Percent[mixer.selected_vtool][i] = mixer.mix[i];}
				select_manual.index = MROWS;
				MixerCfg.Mixer_Mode_Rg = 0;
			#if ENABLED(POWER_LOSS_RECOVERY)
				recovery.save(true);
			#endif
				Draw_Mixer_Manual_Menu();
			break;

			case MIXER_CASE_AUTO:  	// Auto
				DwinMenuID = DWMENU_MIXER_AUTO;
				select_auto.reset();

				mixer.selected_vtool = mixer.gradient.start_vtool;   
				updata_mixer_from_vtool();
				MIXER_STEPPER_LOOP(i) {MixerCfg.Start_Percent[i] = mixer.mix[i];}
				mixer.selected_vtool = mixer.gradient.end_vtool;   
				updata_mixer_from_vtool();
				MIXER_STEPPER_LOOP(i) {MixerCfg.End_Percent[i] = mixer.mix[i];}
				select_auto.index = MROWS;
				MixerCfg.Mixer_Mode_Rg = 1;
			#if ENABLED(POWER_LOSS_RECOVERY)
				recovery.save(true);
			#endif
				Draw_Mixer_Gradient_Menu();
			//Draw_Print_ProgressMixModel();
			break;

			case MIXER_CASE_RANDOM: 	// Random
				DwinMenuID = DWMENU_MIXER_RANDOM;
				select_random.reset();
				select_random.index = MROWS;
				MixerCfg.Mixer_Mode_Rg = 2;
			#if ENABLED(POWER_LOSS_RECOVERY)
				recovery.save(true);
			#endif
				Draw_Mixer_Random_Menu();
			break;

			case MIXER_CASE_VTOOL: 	// vtool
				DwinMenuID = DWMENU_MIXER_VTOOL;
				DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(MIXER_CASE_VTOOL), mixer.selected_vtool);
				EncoderRate.enabled = true;
			break;

			default: break;
		}
	}
	dwinLCD.UpdateLCD();
}

/* Config */
void HMI_Config() {
 ENCODER_DiffState encoder_diffState = get_encoder_state();
 if (encoder_diffState == ENCODER_DIFF_NO) return;

 // Avoid flicker by updating only the previous menu
	if (encoder_diffState == ENCODER_DIFF_CW) {
		if(IS_SD_PRINTING() || IS_SD_PAUSED()){
			if(select_config.inc(1 + CONFIG_TUNE_CASE_TOTAL)){
				if (select_config.now > MROWS && select_config.now > select_config.index) {
					select_config.index = select_config.now;
					// Scroll up and draw a blank bottom line
					Scroll_Menu(DWIN_SCROLL_UP);
				}
				else 
					Move_Highlight(1, select_config.now + MROWS - select_config.index);
			}
		}
		else if(select_config.inc(1 + CONFIG_CASE_TOTAL)){
			if (select_config.now > MROWS && select_config.now > select_config.index) {
				select_config.index = select_config.now;
				// Scroll up and draw a blank bottom line
				Scroll_Menu(DWIN_SCROLL_UP);
				//Draw_Menu_Icon(MROWS, ICON_Cursor+ select_config.now - 1);

			#if ENABLED(OPTION_REPEAT_PRINTING) 
				if(select_config.index == CONFIG_CASE_REPRINT) Item_Config_Reprint(MROWS);
			#endif

			#if ENABLED(OPTION_BED_COATING) 
				if(select_config.index == CONFIG_CASE_COATING) Item_Config_bedcoating(MROWS);
			#endif

			#if ENABLED(ABL_GRID)
				if(select_config.index == CONFIG_CASE_LEVELING) Item_Config_Leveling(MROWS);
				if(select_config.index == CONFIG_CASE_ACTIVELEVEL) Item_Config_ActiveLevel(MROWS);
			#endif

			#if ENABLED(DEBUG_GCODE_M92)
				if(select_config.index == CONFIG_CASE_M92) Item_Config_M92(MROWS);
			#endif
			}
			else 
				Move_Highlight(1, select_config.now + MROWS - select_config.index);
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_CCW) {
		if (select_config.dec()) {
			if (select_config.now < select_config.index - MROWS) {
				select_config.index--;
				Scroll_Menu(DWIN_SCROLL_DOWN);
				if (select_config.index == MROWS) Draw_Back_First();

			#if ENABLED(FWRETRACT) 
				if(select_config.index - MROWS == CONFIG_CASE_RETRACT) Item_Config_Retract(0);
			#endif

			#if ENABLED(FILAMENT_RUNOUT_SENSOR)
				if(select_config.index - MROWS == CONFIG_CASE_FILAMENT) Item_Config_Filament(0);
			#endif

			#if ENABLED(POWER_LOSS_RECOVERY)
				if(select_config.index - MROWS == CONFIG_CASE_POWERLOSS) Item_Config_Powerloss(0);
			#endif

			#if ENABLED(OPTION_AUTOPOWEROFF)
				if(select_config.index - MROWS == CONFIG_CASE_SHUTDOWN) Item_Config_Shutdown(0);
			#endif				
				if(!IS_SD_PRINTING() && !IS_SD_PAUSED()){
			#if ENABLED(OPTION_WIFI_MODULE)
					if(select_config.index - MROWS == CONFIG_CASE_WIFI) Item_Config_Wifi(0);
			#endif
			#if ENABLED(ABL_GRID)
					if(select_config.index - MROWS == CONFIG_CASE_LEVELING) Item_Config_Leveling(0);
					if(select_config.index - MROWS == CONFIG_CASE_ACTIVELEVEL) Item_Config_ActiveLevel(0);
			#endif
				}
			}
			else {
				Move_Highlight(-1, select_config.now + MROWS - select_config.index);
			}
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_ENTER) {
	  switch (select_config.now) {
			case 0: 									// Back
				if(!IS_SD_PRINTING() && !IS_SD_PAUSED()){
					DwinMenuID = DWMENU_CONTROL;
					select_control.set(CONTROL_CASE_CONFIG);
					select_control.index = MROWS;
					Draw_Control_Menu();
				}
				else{
					DwinMenuID = DWMENU_TUNE;
					HMI_flag.show_mode = 0;
					select_tune.set(TUNE_CASE_SPEED);
					select_tune.index = MROWS;
					Draw_Tune_Menu();
				}
		  break;

 #if ENABLED(FWRETRACT) 
			case CONFIG_CASE_RETRACT: 				// RETRACT
				DwinMenuID = DWMENU_SET_RETRACT;
				select_retract.index = MROWS;
				Draw_Retract_Menu();				
			break;
 #endif
	 
 #if ENABLED(FILAMENT_RUNOUT_SENSOR)
	case CONFIG_CASE_FILAMENT:  				// FILAMENT
		DwinMenuID = DWMENU_CONFIG;
		runout.enabled = !runout.enabled;
		DWIN_Draw_MaskString_Default(MENUONOFF_X, MBASE(CONFIG_CASE_FILAMENT + MROWS - select_config.index), F_STRING_ONOFF(runout.enabled));		
		if(runout.enabled)
			queue.inject_P("M412 S1");		
		else
			queue.inject_P("M412 S0");
   break;
 #endif
	 
 #if ENABLED(POWER_LOSS_RECOVERY)
	 case CONFIG_CASE_POWERLOSS:  			// POWERLOSS	 		
			DwinMenuID = DWMENU_CONFIG;
			recovery.enabled = !recovery.enabled;
			DWIN_Draw_MaskString_Default(MENUONOFF_X, MBASE(CONFIG_CASE_POWERLOSS + MROWS - select_config.index), F_STRING_ONOFF(recovery.enabled));
			if(recovery.enabled)
				queue.inject_P("M413 S1");			
			else
				queue.inject_P("M413 S0");
  break;
 #endif
		
 #if ENABLED(OPTION_AUTOPOWEROFF)
	 case CONFIG_CASE_SHUTDOWN:
		DwinMenuID = DWMENU_CONFIG;
		HMI_flag.Autoshutdown_enabled = !HMI_flag.Autoshutdown_enabled;
		DWIN_Draw_MaskString_Default(MENUONOFF_X, MBASE(CONFIG_CASE_SHUTDOWN + MROWS - select_config.index), F_STRING_ONOFF(HMI_flag.Autoshutdown_enabled));
		break;
 #endif
   
 #if ENABLED(OPTION_WIFI_MODULE)
	 case CONFIG_CASE_WIFI:
			if(IS_SD_PRINTING() || IS_SD_PAUSED())	break;
			
			WiFi_Enabled = !WiFi_Enabled;
			DWIN_Draw_MaskString_Default(MENUONOFF_X, MBASE(CONFIG_CASE_WIFI + MROWS - select_config.index), F_STRING_ONOFF(WiFi_Enabled));
		  if(WiFi_Enabled) {
				DwinMenuID = DWMENU_POP_WIFI;								
				HMI_flag.wifi_Handshake_ok = false;
				HMI_flag.wifi_link_timer = 0;
				Popup_Window_Wifi_Connect();
				WIFI_onoff();
		  }
			else {
				DwinMenuID = DWMENU_CONFIG;
				WIFI_onoff();
			}
			HMI_AudioFeedback(settings.save());
		break;
 #endif
	 		
 #if ENABLED(OPTION_REPEAT_PRINTING)
 	case CONFIG_CASE_REPRINT:
			if(IS_SD_PRINTING() || IS_SD_PAUSED())	break;
			
			DwinMenuID = DWMENU_SET_REPRINT;
			select_reprint.index = MROWS;
			Draw_Reprint_Menu();
		break;
 #endif

 #if ENABLED(OPTION_BED_COATING)
 	case CONFIG_CASE_COATING:
			if(IS_SD_PRINTING() || IS_SD_PAUSED())	break;
			
			DwinMenuID = DWMENU_SET_BEDCOATING;	
			HMI_ValueStruct.coating_thickness = (int16_t)(coating_thickness * MINUNITMULT);
			DWIN_Draw_Selected_Small_Float21(MENUVALUE_X-8, MBASE(CONFIG_CASE_COATING + MROWS -select_config.index), HMI_ValueStruct.coating_thickness);			
		break;
 #endif

 #if ENABLED(ABL_GRID)
  case CONFIG_CASE_LEVELING:
			if(IS_SD_PRINTING() || IS_SD_PAUSED()) 	break;
			
			DwinMenuID = DWMENU_CONFIG;
			HMI_flag.Leveling_Menu_Fg = !HMI_flag.Leveling_Menu_Fg;
			DWIN_Draw_MaskString_Default(MENUONOFF_X, MBASE(CONFIG_CASE_LEVELING + MROWS - select_config.index), F_STRING_ONOFF(HMI_flag.Leveling_Menu_Fg));
			if(HMI_flag.Leveling_Menu_Fg)	set_bed_leveling_enabled(false);
		break;

		case CONFIG_CASE_ACTIVELEVEL:
			if(IS_SD_PRINTING() || IS_SD_PAUSED()) 	break;	
			
			DwinMenuID = DWMENU_CONFIG;			
			planner.leveling_active = !planner.leveling_active;
			DWIN_Draw_MaskString_Default(MENUONOFF_X, MBASE(CONFIG_CASE_ACTIVELEVEL + MROWS - select_config.index), F_STRING_ONOFF(planner.leveling_active));
			set_bed_leveling_enabled(planner.leveling_active);
		break;
 #endif

 #if ENABLED(DEBUG_GCODE_M92)
 	case CONFIG_CASE_M92:
			if(IS_SD_PRINTING() || IS_SD_PAUSED()) 	break;
			
			DwinMenuID = DWMENU_CONFIG;
  		queue.inject_P(PSTR("M92"));
		break;
 #endif
   
   default: break;
  }
 }
 dwinLCD.UpdateLCD();
}

/* Reprint*/
#if ENABLED(OPTION_REPEAT_PRINTING) 
void HMI_Reprint() {
 ENCODER_DiffState encoder_diffState = get_encoder_state();
 if (encoder_diffState == ENCODER_DIFF_NO) return;

 // Avoid flicker by updating only the previous menu
	if (encoder_diffState == ENCODER_DIFF_CW) {
		if (select_reprint.inc(1 + REPRINT_CASE_TOTAL)) {
			if (select_reprint.now > MROWS && select_reprint.now > select_reprint.index) {
				select_reprint.index = select_reprint.now;
				// Scroll up and draw a blank bottom line
				Scroll_Menu(DWIN_SCROLL_UP);
				if(select_reprint.index == REPRINT_CASE_BACK) Item_Reprint_Back(MROWS);
			}
			else {
				Move_Highlight(1, select_reprint.now + MROWS - select_reprint.index);
			}
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_CCW) {
		if (select_reprint.dec()) {
			if (select_reprint.now < select_reprint.index - MROWS) {
				select_reprint.index--;
				Scroll_Menu(DWIN_SCROLL_DOWN);
				if (select_reprint.index == MROWS) Draw_Back_First();
				if(select_reprint.index - MROWS == REPRINT_CASE_ENABLED) Item_Reprint_Enabled(0);
			}
			else {
				Move_Highlight(-1, select_reprint.now + MROWS - select_reprint.index);
			}
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_ENTER) {
		switch (select_reprint.now) {
			case 0: 										// Back
				ReprintManager.Back_Move_Stop();
				DwinMenuID = DWMENU_CONFIG;
				select_config.set(CONFIG_CASE_REPRINT);
				Draw_Config_Menu();
			break;

			case REPRINT_CASE_ENABLED: 					// ENABLED
				DwinMenuID = DWMENU_SET_REPRINT;
				ReprintManager.enabled = !ReprintManager.enabled;
				DWIN_Draw_MaskString_Default(MENUONOFF_X, MBASE(MROWS -select_reprint.index + REPRINT_CASE_ENABLED), F_STRING_ONOFF(ReprintManager.enabled));
				if(ReprintManager.enabled)
					queue.inject_P("M180 S1");
				else			
					queue.inject_P("M180 S0");
			break;

			case REPRINT_CASE_TIMES:  					// reprint times
				DwinMenuID = DWMENU_SET_REPRINT_TIMES;
				DWIN_Draw_Select_IntValue_Default(4, MENUVALUE_X, MBASE(MROWS -select_reprint.index + REPRINT_CASE_TIMES), ReprintManager.Reprint_times);
				EncoderRate.enabled = true;
			break;

			case REPRINT_CASE_LENGHT:  					// FORWARD MOVE LENGHT
				DwinMenuID = DWMENU_SET_REPRINT_RUNLENGTH;
				DWIN_Draw_Select_IntValue_Default(4, MENUVALUE_X, MBASE(MROWS -select_reprint.index + REPRINT_CASE_LENGHT), ReprintManager.Forward_lenght);
				EncoderRate.enabled = true;
			break;

			case REPRINT_CASE_RESET:  					// reset
				DwinMenuID = DWMENU_SET_REPRINT;
				ReprintManager.Back_Move_Start();
				ReprintManager.Is_Reprint_Reset = true;
				ReprintManager.tempbed_counter = 0;
			break;

			case REPRINT_CASE_FORWARD:  					// forward
				DwinMenuID = DWMENU_SET_REPRINT;
				ReprintManager.Forward_Move_Start();
			break;

			case REPRINT_CASE_BACK:  					// BACK
				DwinMenuID = DWMENU_SET_REPRINT;
				ReprintManager.Back_Move_Start();
			break;

			default: break;
		}
	}
	dwinLCD.UpdateLCD();
}
#endif

/* Retract */
#if ENABLED(FWRETRACT) 
void HMI_Retract() {
	ENCODER_DiffState encoder_diffState = get_encoder_state();
	if (encoder_diffState == ENCODER_DIFF_NO) return;

	 // Avoid flicker by updating only the previous menu
	if (encoder_diffState == ENCODER_DIFF_CW) {
		if (select_retract.inc(1 + RETRACT_CASE_TOTAL)) {
			if (select_retract.now > MROWS && select_retract.now > select_retract.index) {
				select_retract.index = select_retract.now;
				// Scroll up and draw a blank bottom line
				Scroll_Menu(DWIN_SCROLL_UP);
				if(select_config.index == RETRACT_CASE_AUTO) Item_Retract_Retract_Enabled(MROWS);
				else if(select_config.index == RETRACT_CASE_RETRACT_MM) Item_Retract_Retract_mm(MROWS);
				else if(select_config.index == RETRACT_CASE_RETRACT_V) Item_Retract_Retract_V(MROWS);
				else if(select_config.index == RETRACT_CASE_RETRACT_ZHOP) Item_Retract_Retract_ZHop(MROWS);
				else if(select_config.index == RETRACT_CASE_RECOVER_MM) Item_Retract_UnRetract_mm(MROWS);
				else if(select_config.index == RETRACT_CASE_RECOVER_V) Item_Retract_UnRetract_V(MROWS);
			}
			else {
				Move_Highlight(1, select_retract.now + MROWS - select_retract.index);
			}
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_CCW) {
		if (select_retract.dec()) {
			if (select_retract.now < select_retract.index - MROWS) {
				select_retract.index--;
				Scroll_Menu(DWIN_SCROLL_DOWN);
				if (select_config.index == MROWS) Draw_Back_First();
				if(select_config.index - MROWS == RETRACT_CASE_AUTO) Item_Retract_Retract_Enabled(MROWS);
				else if(select_config.index - MROWS == RETRACT_CASE_RETRACT_MM) Item_Retract_Retract_mm(MROWS);
				else if(select_config.index - MROWS == RETRACT_CASE_RETRACT_V) Item_Retract_Retract_V(MROWS);
				else if(select_config.index - MROWS == RETRACT_CASE_RETRACT_ZHOP) Item_Retract_Retract_ZHop(MROWS);
				else if(select_config.index - MROWS == RETRACT_CASE_RECOVER_MM) Item_Retract_UnRetract_mm(MROWS);
				else if(select_config.index - MROWS == RETRACT_CASE_RECOVER_V) Item_Retract_UnRetract_V(MROWS);
			}
			else {
				Move_Highlight(-1, select_retract.now + MROWS - select_retract.index);
			}
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_ENTER) {
		switch (select_retract.now) {
			case 0: 									// Back
				DwinMenuID = DWMENU_CONFIG;
				select_config.set(CONFIG_CASE_RETRACT);
				select_config.index = MROWS;
				Draw_Config_Menu();
			break;

			case RETRACT_CASE_AUTO: 					// auto
				DwinMenuID = DWMENU_SET_RETRACT;
				fwretract.autoretract_enabled = !fwretract.autoretract_enabled;
				DWIN_Draw_MaskString_Default(MENUONOFF_X, MBASE(RETRACT_CASE_AUTO), F_STRING_ONOFF(fwretract.autoretract_enabled));
				if(fwretract.autoretract_enabled)
					queue.inject_P("M209 S1");
				else
					queue.inject_P("M209 S0");
			break;
	 
			case RETRACT_CASE_RETRACT_MM:  			// RETRACT_MM
				DwinMenuID = DWMENU_SET_RETRACT_MM;
				HMI_ValueStruct.Retract_MM_scale = fwretract.settings.retract_length*MAXUNITMULT;
				DWIN_Draw_Selected_Small_Float22(MENUVALUE_X, MBASE(MROWS -select_retract.index + RETRACT_CASE_RETRACT_MM), HMI_ValueStruct.Retract_MM_scale);
				EncoderRate.enabled = true;
			break;

			case RETRACT_CASE_RETRACT_V:  			// RETRACT_V
				DwinMenuID = DWMENU_SET_RETRACT_V;
				HMI_ValueStruct.Retract_V_scale = fwretract.settings.retract_feedrate_mm_s*MAXUNITMULT;
				DWIN_Draw_Selected_Small_Float32(MENUVALUE_X-8, MBASE(MROWS -select_retract.index + RETRACT_CASE_RETRACT_V), HMI_ValueStruct.Retract_V_scale);
				EncoderRate.enabled = true;
			break;

			case RETRACT_CASE_RETRACT_ZHOP:  			// RETRACT_ZHOP
				DwinMenuID = DWMENU_SET_RETRACT_ZHOP;
				HMI_ValueStruct.Retract_ZHOP_scale = fwretract.settings.retract_zraise*MAXUNITMULT;
				DWIN_Draw_Selected_Small_Float32(MENUVALUE_X-8, MBASE(MROWS -select_retract.index + RETRACT_CASE_RETRACT_ZHOP), HMI_ValueStruct.Retract_ZHOP_scale);
				EncoderRate.enabled = true;
			break;

			case RETRACT_CASE_RECOVER_MM:  			// RECOVER_MM
				DwinMenuID = DWMENU_SET_UNRETRACT_MM;
				HMI_ValueStruct.unRetract_MM_scale = fwretract.settings.retract_recover_extra*MAXUNITMULT;
				DWIN_Draw_Selected_Small_Float22(MENUVALUE_X, MBASE(MROWS -select_retract.index + RETRACT_CASE_RECOVER_MM), HMI_ValueStruct.unRetract_MM_scale);
				EncoderRate.enabled = true;
			break;

			case RETRACT_CASE_RECOVER_V:  			// RECOVER_V
				DwinMenuID = DWMENU_SET_UNRETRACT_V;
				HMI_ValueStruct.unRetract_V_scale = fwretract.settings.retract_recover_feedrate_mm_s*MAXUNITMULT;
				DWIN_Draw_Selected_Small_Float32(MENUVALUE_X-8, MBASE(MROWS -select_retract.index + RETRACT_CASE_RECOVER_V), HMI_ValueStruct.unRetract_V_scale);
				EncoderRate.enabled = true;
			break;
   
   default: break;
  }
 }
 dwinLCD.UpdateLCD();
}
#endif

/* Mixer_Manual */
void HMI_Mixer_Manual() {
 uint8_t i = 0;
 signed int Temp_Buff[MIXING_STEPPERS] = {0};
 
 ENCODER_DiffState encoder_diffState = get_encoder_state();
 if (encoder_diffState == ENCODER_DIFF_NO) return;

 // Avoid flicker by updating only the previous menu
 if (encoder_diffState == ENCODER_DIFF_CW) {
  if (select_manual.inc(1 + MANUAL_CASE_TOTAL)) Move_Highlight(1, select_manual.now);
 }
 else if (encoder_diffState == ENCODER_DIFF_CCW) {
  if (select_manual.dec()) Move_Highlight(-1, select_manual.now);
 }
 else if (encoder_diffState == ENCODER_DIFF_ENTER) {
  switch (select_manual.now) {
	 case 0: 						// Back
    DwinMenuID = DWMENU_MIXER;
    select_mixer.set(MIXER_CASE_MANUAL);
    Draw_Mixer_Menu();
    break;

	 #if ENABLED(MIXING_EXTRUDER)
		#if(MIXING_STEPPERS == 4)
			case MANUAL_CASE_EXTRUDER1: 	// ex1
   			DwinMenuID = DWMENU_MIXER_EXT1;
				DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(MANUAL_CASE_EXTRUDER1), MixerCfg.Manual_Percent[mixer.selected_vtool][0]);
				DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(5), mixer.selected_vtool);
				//EncoderRate.enabled = true;
    		break;
   		case MANUAL_CASE_EXTRUDER2: 	// ex2
    		DwinMenuID = DWMENU_MIXER_EXT2;
    		DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(MANUAL_CASE_EXTRUDER2), MixerCfg.Manual_Percent[mixer.selected_vtool][1]);
				DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(5), mixer.selected_vtool);
				//EncoderRate.enabled = true;
    		break;
   		case MANUAL_CASE_EXTRUDER3: 	// ex3
    		DwinMenuID = DWMENU_MIXER_EXT3;
    		DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(MANUAL_CASE_EXTRUDER3), MixerCfg.Manual_Percent[mixer.selected_vtool][2]);
				DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(5), mixer.selected_vtool);
				//EncoderRate.enabled = true;
    		break;
	 		case MANUAL_CASE_EXTRUDER4: 	// ex4
    		DwinMenuID = DWMENU_MIXER_EXT4;
    		DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(MANUAL_CASE_EXTRUDER4), MixerCfg.Manual_Percent[mixer.selected_vtool][3]);
	 			DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(5), mixer.selected_vtool);
	 			//EncoderRate.enabled = true;
    		break;
		#elif(MIXING_STEPPERS == 3)
			case MANUAL_CASE_EXTRUDER1: 	// ex1
   			DwinMenuID = DWMENU_MIXER_EXT1;
				DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(MANUAL_CASE_EXTRUDER1), MixerCfg.Manual_Percent[mixer.selected_vtool][0]);
				DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(4), mixer.selected_vtool);
				//EncoderRate.enabled = true;
    		break;
   		case MANUAL_CASE_EXTRUDER2: 	// ex2
    		DwinMenuID = DWMENU_MIXER_EXT2;
    		DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(MANUAL_CASE_EXTRUDER2), MixerCfg.Manual_Percent[mixer.selected_vtool][1]);
				DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(4), mixer.selected_vtool);
				//EncoderRate.enabled = true;
    		break;
   		case MANUAL_CASE_EXTRUDER3: 	// ex3
    		DwinMenuID = DWMENU_MIXER_EXT3;
    		DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(MANUAL_CASE_EXTRUDER3), MixerCfg.Manual_Percent[mixer.selected_vtool][2]);
				DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(4), mixer.selected_vtool);
				//EncoderRate.enabled = true;
    		break;
		#elif(MIXING_STEPPERS == 2)
			case MANUAL_CASE_EXTRUDER1: 	// ex1
   			DwinMenuID = DWMENU_MIXER_EXT1;
				DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(MANUAL_CASE_EXTRUDER1), MixerCfg.Manual_Percent[mixer.selected_vtool][0]);
				DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(3), mixer.selected_vtool);
				//EncoderRate.enabled = true;
    		break;
   		case MANUAL_CASE_EXTRUDER2: 	// ex2
    		DwinMenuID = DWMENU_MIXER_EXT2;
    		DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(MANUAL_CASE_EXTRUDER2), MixerCfg.Manual_Percent[mixer.selected_vtool][1]);
				DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(3), mixer.selected_vtool);
				//EncoderRate.enabled = true;
    		break;
		#endif
	 #endif
		
	 case MANUAL_CASE_OK: 		// OK
	  DwinMenuID = DWMENU_MIXER_MANUAL;
		DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(MANUAL_CASE_OK), mixer.selected_vtool);		
	  #if(MIXING_STEPPERS == 4)
			if(!Check_Percent_equal()){				
				for(i=0;i<4;i++){
					mixer.color[mixer.selected_vtool][i] = mixer.mix[i] = MixerCfg.Manual_Percent[mixer.selected_vtool][i] = 25;
	  			DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(i+1), MixerCfg.Manual_Percent[mixer.selected_vtool][i]);
				}
	  	}
			else 
			{
				for(i=0;i<4;i++){
					if(i < 3) Temp_Buff[i] = 100*MixerCfg.Manual_Percent[mixer.selected_vtool][i]/(MixerCfg.Manual_Percent[mixer.selected_vtool][0]+MixerCfg.Manual_Percent[mixer.selected_vtool][1]+MixerCfg.Manual_Percent[mixer.selected_vtool][2]+MixerCfg.Manual_Percent[mixer.selected_vtool][3]);
					else Temp_Buff[i] = 100 - Temp_Buff[0]-Temp_Buff[1]-Temp_Buff[2];	
 				}

				for(i=0;i<4;i++){
					mixer.color[mixer.selected_vtool][i] = mixer.mix[i] = MixerCfg.Manual_Percent[mixer.selected_vtool][i] = Temp_Buff[i];
	  			DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(i+1), MixerCfg.Manual_Percent[mixer.selected_vtool][i]);
				}
			}
		#elif(MIXING_STEPPERS == 3)
	  	if(!Check_Percent_equal()) {
					for(i=0;i<2;i++){
						mixer.mix[i] = MixerCfg.Manual_Percent[mixer.selected_vtool][i] = 33;
		  			DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(i+1), MixerCfg.Manual_Percent[mixer.selected_vtool][i]);
						dwinLCD.UpdateLCD();
					}
					i++;
					mixer.mix[i] = MixerCfg.Manual_Percent[mixer.selected_vtool][i] = 34;
		 			DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(i+1), MixerCfg.Manual_Percent[mixer.selected_vtool][i]);
	  	}
			else 
			{
				for(i=0;i<3;i++){
					if(i < 2) Temp_Buff[i] = 100*MixerCfg.Manual_Percent[mixer.selected_vtool][i]/(MixerCfg.Manual_Percent[mixer.selected_vtool][0]+MixerCfg.Manual_Percent[mixer.selected_vtool][1]+MixerCfg.Manual_Percent[mixer.selected_vtool][2]);
					else Temp_Buff[i] = 100 - Temp_Buff[0]-Temp_Buff[1];	
				}

				for(i=0;i<3;i++){
					mixer.mix[i] = MixerCfg.Manual_Percent[mixer.selected_vtool][i] = Temp_Buff[i];
	  			DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(i+1), MixerCfg.Manual_Percent[mixer.selected_vtool][i]);
					mixer.color[mixer.selected_vtool][i] = MixerCfg.Manual_Percent[mixer.selected_vtool][i];
				}
			}
		#elif(MIXING_STEPPERS == 2)
	  	if(!Check_Percent_equal()) {
					for(i=0;i<2;i++){
						mixer.mix[i] = MixerCfg.Manual_Percent[mixer.selected_vtool][i] = 50;
		  			DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(i+1), MixerCfg.Manual_Percent[mixer.selected_vtool][i]);
					}
	  	}
			else 
			{
				for(i=0;i<2;i++){
					if(i < 1) Temp_Buff[i] = 100*MixerCfg.Manual_Percent[mixer.selected_vtool][i]/(MixerCfg.Manual_Percent[mixer.selected_vtool][0]+MixerCfg.Manual_Percent[mixer.selected_vtool][1]);
					else Temp_Buff[i] = 100 - Temp_Buff[0];	
				}

				for(i=0;i<2;i++){
					mixer.mix[i] = MixerCfg.Manual_Percent[mixer.selected_vtool][i] = Temp_Buff[i];
	  			DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(i+1), MixerCfg.Manual_Percent[mixer.selected_vtool][i]);
					mixer.color[mixer.selected_vtool][i] = MixerCfg.Manual_Percent[mixer.selected_vtool][i];
					//dwinLCD.UpdateLCD();
				}
			}
		#endif
		mixer.copy_mix_to_collector();
  	mixer.normalize();
   break;
   default: break;
  }
 }
 dwinLCD.UpdateLCD();
}

/* Gradient Mixing */
void HMI_Mixer_Gradient() {
 ENCODER_DiffState encoder_diffState = get_encoder_state();
 if (encoder_diffState == ENCODER_DIFF_NO) return;
 
 // Avoid flicker by updating only the previous menu
 if (encoder_diffState == ENCODER_DIFF_CW) {
  if (select_auto.inc(1 + AUTO_CASE_TOTAL)) {
   if (select_auto.now > MROWS && select_auto.now > select_auto.index) {
    select_auto.index = select_auto.now;
   }
   else {
    Move_Highlight(1, select_auto.now + MROWS - select_auto.index);
   }
  }
 }
 else if (encoder_diffState == ENCODER_DIFF_CCW) {
  if (select_auto.dec()) {
   if (select_auto.now < select_auto.index - MROWS) {
    select_auto.index--;
    Scroll_Menu(DWIN_SCROLL_DOWN);
   }
   else {
    Move_Highlight(-1, select_auto.now + MROWS - select_auto.index);
   }
  }
 }
 else if (encoder_diffState == ENCODER_DIFF_ENTER) {
  switch (select_auto.now) {
	 case 0: 						// Back
    DwinMenuID = DWMENU_MIXER;
    select_mixer.set(MIXER_CASE_AUTO);
    Draw_Mixer_Menu();
    break;
	 case AUTO_CASE_ZPOS_START: 		// zpos_start
    DwinMenuID = DWMENU_MIXER_GRADIENT_ZSTART;
		HMI_ValueStruct.Auto_Zstart_scale = mixer.gradient.start_z*MINUNITMULT;
		DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_auto.index + AUTO_CASE_ZPOS_START), HMI_ValueStruct.Auto_Zstart_scale);
	  EncoderRate.enabled = true;
    break;
	 case AUTO_CASE_ZPOS_END: 		// zpos_end
    DwinMenuID = DWMENU_MIXER_GRADIENT_ZEND;
		HMI_ValueStruct.Auto_Zstart_scale = mixer.gradient.end_z*MINUNITMULT;
		DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_auto.index + AUTO_CASE_ZPOS_END), HMI_ValueStruct.Auto_Zend_scale);
	  EncoderRate.enabled = true;
    break;
	 case AUTO_CASE_VTOOL_START: 		// vtool_start
    DwinMenuID = DWMENU_MIXER_GRADIENT_TSTAR;
    DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(MROWS -select_auto.index + AUTO_CASE_VTOOL_START), mixer.gradient.start_vtool);
		EncoderRate.enabled = true;
    break;
	 case AUTO_CASE_VTOOL_END: 		// vtool_end
    DwinMenuID = DWMENU_MIXER_GRADIENT_TEND;
   DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(MROWS -select_auto.index + AUTO_CASE_VTOOL_END), mixer.gradient.end_vtool);
		EncoderRate.enabled = true;
    break;
   default: break;
  }
 }
 dwinLCD.UpdateLCD();
}

/* Mixer_Auto */
void HMI_Mixer_Random() {
 ENCODER_DiffState encoder_diffState = get_encoder_state();
 if (encoder_diffState == ENCODER_DIFF_NO) return;
 
 // Avoid flicker by updating only the previous menu
 if (encoder_diffState == ENCODER_DIFF_CW) {
  if (select_random.inc(1 + RANDOM_CASE_TOTAL)) {
   if (select_random.now > MROWS && select_random.now > select_random.index) {
    select_random.index = select_random.now;
   }
   else {
    Move_Highlight(1, select_random.now + MROWS - select_random.index);
   }
  }
 }
 else if (encoder_diffState == ENCODER_DIFF_CCW) {
  if (select_random.dec()) {
   if (select_random.now < select_random.index - MROWS) {
    select_random.index--;
    Scroll_Menu(DWIN_SCROLL_DOWN);
   }
   else {
    Move_Highlight(-1, select_random.now + MROWS - select_random.index);
   }
  }
 }
 else if (encoder_diffState == ENCODER_DIFF_ENTER) {
  switch (select_random.now) {
	 case 0: 						// Back
    DwinMenuID = DWMENU_MIXER;
    select_mixer.set(MIXER_CASE_RANDOM);
    Draw_Mixer_Menu();
   break;
		
	 case RANDOM_CASE_ZPOS_START: 		// zpos_start
    DwinMenuID = DWMENU_MIXER_RANDOM_ZSTART;
		HMI_ValueStruct.Random_Zstart_scale = mixer.random_mix.start_z*MINUNITMULT;
		DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_random.index + RANDOM_CASE_ZPOS_START), HMI_ValueStruct.Random_Zstart_scale);
	  EncoderRate.enabled = true;
   break;
	 
	 case RANDOM_CASE_ZPOS_END: 			// zpos_end
    DwinMenuID = DWMENU_MIXER_RANDOM_ZEND;
		HMI_ValueStruct.Random_Zend_scale = mixer.random_mix.end_z*MINUNITMULT;
		DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_random.index + RANDOM_CASE_ZPOS_END), HMI_ValueStruct.Random_Zend_scale);
	  EncoderRate.enabled = true;
   break;
	 
  case RANDOM_CASE_HEIGHT: 		// Height
    DwinMenuID = DWMENU_MIXER_RANDOM_HEIGHT;
		HMI_ValueStruct.Random_Height = mixer.random_mix.height*MINUNITMULT;
		DWIN_Draw_Selected_Small_Float31(MENUVALUE_X, MBASE(MROWS -select_random.index + RANDOM_CASE_HEIGHT), HMI_ValueStruct.Random_Height);
	  EncoderRate.enabled = true;	
   break;
	 
  case RANDOM_CASE_EXTRUDERS: 		// Extruders
    DwinMenuID = DWMENU_MIXER_RANDOM_EXTN;
		DWIN_Draw_Select_IntValue_Default(1, MENUVALUE_X+4*8, MBASE(RANDOM_CASE_EXTRUDERS), mixer.random_mix.extruders);
	  EncoderRate.enabled = true;
   break;
	 
   default: break;
  }
 }
 dwinLCD.UpdateLCD();
}

/* Mixer_Vtool */
void HMI_Adjust_Mixer_Vtool() {
 ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
 if (encoder_diffState != ENCODER_DIFF_NO) {
  if (Apply_Encoder(encoder_diffState, mixer.selected_vtool)) {
   DwinMenuID = DWMENU_MIXER;
   EncoderRate.enabled = false;
	 DWIN_Draw_IntValue_Default(3, MENUVALUE_X+8, MBASE(4), mixer.selected_vtool);
	 MixerCfg.Vtool_Backup = mixer.selected_vtool;
	 #if ENABLED(POWER_LOSS_RECOVERY)
	 recovery.save(true);
	 #endif
	 updata_mixer_from_vtool();
	 dwinLCD.UpdateLCD();
   return;
  }
  NOLESS(mixer.selected_vtool, 0);
	NOMORE(mixer.selected_vtool, (MIXING_VIRTUAL_TOOLS-1));
  DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(4), mixer.selected_vtool);
	MixerCfg.Vtool_Backup = mixer.selected_vtool;
	dwinLCD.UpdateLCD();
 }
}

/* Info */
uint8_t testmode_click_times = 0;
void HMI_Info() {
	ENCODER_DiffState encoder_diffState = get_encoder_state();
	if (encoder_diffState == ENCODER_DIFF_NO) return;

	if (encoder_diffState == ENCODER_DIFF_CW) {
		testmode_click_times = 0;
		if (select_info.inc(1 + INFO_CASE_TOTAL)) {
			if (select_info.now > MROWS && select_info.now > select_info.index) {
				select_info.index = select_info.now;

				// Scroll up and draw a blank bottom line
				Scroll_Menu(DWIN_SCROLL_UP);		
				if(select_info.index == INFO_CASE_EXTRUDER_NUM) Item_Info_Extruder_Num(MROWS);
				if(select_info.index == INFO_CASE_EXTRUDER_MODEL) Item_Info_Extruder_Model(MROWS);		
			#if ENABLED(OPTION_DUALZ_DRIVE)
				if(select_info.index == INFO_CASE_DUALZ_DRIVE) Item_Info_DualZ_Drive(MROWS);
			#endif
			#if ENABLED(OPTION_Z2_ENDSTOP)
				if(select_info.index == INFO_CASE_DUALZ_ENDSTOP) Item_Info_DualZ_Endstop(MROWS);
			#endif
				if(select_info.index == INFO_CASE_BAUDRATE) Item_Info_Baudrate(MROWS);
				if(select_info.index == INFO_CASE_PROTOCOL) Item_Info_Protocol(MROWS);
				if(select_info.index == INFO_CASE_PSU) Item_Info_Psu(MROWS);
				if(select_info.index == INFO_CASE_DATE) Item_Info_Date(MROWS);
				if(select_info.index == INFO_CASE_THERMISTOR) Item_Info_Thermistor(MROWS);
				if(select_info.index == INFO_CASE_BED) Item_Info_Bed(MROWS);
				if(select_info.index == INFO_CASE_HOT) Item_Info_Hot(MROWS);
			}
			else 
				Move_Highlight(1, select_info.now + MROWS - select_info.index);
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_CCW) {
		testmode_click_times = 0;
		if (select_info.dec()) {
			if (select_info.now < select_info.index - MROWS) {
				select_info.index--;
				Scroll_Menu(DWIN_SCROLL_DOWN);		
				if (select_info.index == MROWS){
					Draw_Back_First();
					dwinLCD.Draw_Line(Line_Color, 16, MBASE(1) + 33, 256, MBASE(1) + 34);
				}
				else
					dwinLCD.Draw_Line(Line_Color, 16, MBASE(0) + 33, 256, MBASE(0) + 34);

				if(select_info.index - MROWS == INFO_CASE_BAUDRATE) Item_Info_Baudrate(0);

			#if ENABLED(OPTION_Z2_ENDSTOP)
				if(select_info.index - MROWS == INFO_CASE_DUALZ_ENDSTOP) Item_Info_DualZ_Endstop(0);
			#endif

			#if ENABLED(OPTION_DUALZ_DRIVE)
				if(select_info.index - MROWS == INFO_CASE_DUALZ_DRIVE) Item_Info_DualZ_Drive(0);
			#endif

				if(select_info.index - MROWS == INFO_CASE_EXTRUDER_MODEL) Item_Info_Extruder_Model(0);
				if(select_info.index - MROWS == INFO_CASE_EXTRUDER_NUM) Item_Info_Extruder_Num(0);
				if(select_info.index - MROWS == INFO_CASE_BOARD) Item_Info_Board(0);
				if(select_info.index - MROWS == INFO_CASE_MODEL) Item_Info_Model(0);
				if(select_info.index - MROWS == INFO_CASE_WEBSITE) Item_Info_Website(0);
				if(select_info.index - MROWS == INFO_CASE_FIRMWARE) Item_Info_Firmware(0);
				if(select_info.index - MROWS == INFO_CASE_VERSION) Item_Info_Version(0);
			}
			else {
				Move_Highlight(-1, select_info.now + MROWS - select_info.index);
			}
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_ENTER) {
		switch (select_info.now) {
			case 0: // Back
				select_main.set(MAIN_CASE_INFO);
				Draw_Main_Menu();
			break;
			
			case INFO_CASE_DATE:	 	
				if(++testmode_click_times >= 5){	
					HMI_flag.auto_test_flag = 0xaa;
					autotest.HMI_StartTest();
				}
			break;
			
			default:	break;
		}
	}
	dwinLCD.UpdateLCD();
}

/* Tune */
void HMI_Tune() {
	ENCODER_DiffState encoder_diffState = get_encoder_state();
	if (encoder_diffState == ENCODER_DIFF_NO) return;
	
	if (encoder_diffState == ENCODER_DIFF_CW) {
		if (select_tune.inc(1 + TUNE_CASE_TOTAL)) {
			if (select_tune.now > MROWS && select_tune.now > select_tune.index) {
				select_tune.index = select_tune.now;

				// Scroll up and draw a blank bottom line
				Scroll_Menu(DWIN_SCROLL_UP);

				if(select_tune.index == TUNE_CASE_MIXER) Item_Tune_Mixer(MROWS);
				if(select_tune.index == TUNE_CASE_CONFIG)	Item_Tune_Config(MROWS);
			}
			else {
				Move_Highlight(1, select_tune.now + MROWS - select_tune.index);
			}
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_CCW) {
		if (select_tune.dec()) {
			if (select_tune.now < select_tune.index - MROWS) {
			select_tune.index--;
			Scroll_Menu(DWIN_SCROLL_DOWN);
			if (select_tune.index == MROWS)
			Draw_Back_First();
			else
			Draw_Menu_Line(0, ICON_Setspeed + select_tune.now - 1);

			if(select_tune.index - MROWS == TUNE_CASE_ETEMP) Item_Tune_ETemp(0);
			if(select_tune.index - MROWS == TUNE_CASE_SPEED) Item_Tune_Speed(0);
			}
			else {
				Move_Highlight(-1, select_tune.now + MROWS - select_tune.index);
			}
		}
	}
	else if (encoder_diffState == ENCODER_DIFF_ENTER) {
#if ENABLED(BABYSTEPPING)
	 	Babysteps_timer_second = millis();
		if(Babysteps_timer_second - Babysteps_timer_first < 1000){
			Babysteps_timer_second = Babysteps_timer_first = 0;
			babyz_offset = last_babyz_offset;
			HMI_ValueStruct.Zoffset_Scale = babyz_offset*MAXUNITMULT;
			DwinMenuID = DWMENU_TUNE_BABYSTEPS;
			Draw_Babystep_Menu();
		}
		else{
			Babysteps_timer_first = millis();	
#else
		{
#endif
			switch (select_tune.now) {
			 	case 0: // Back
			  	select_print.set(0);
			  	Start_PrintProcess();
			 	break;
			 	case TUNE_CASE_SPEED: // Print speed
			  	DwinMenuID = DWMENU_TUNE_PRINTSPEED;
			  	HMI_ValueStruct.print_speed = feedrate_percentage;
			  	DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(TUNE_CASE_SPEED + MROWS - select_tune.index), feedrate_percentage);
			  	EncoderRate.enabled = true;
			  	break;
				#if HAS_HOTEND
			 	case TUNE_CASE_ETEMP: // Nozzle temp
			  		DwinMenuID = DWMENU_SET_ETMP;
			  		HMI_ValueStruct.E_Temp = thermalManager.temp_hotend[0].target;
			  		DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(TUNE_CASE_ETEMP + MROWS - select_tune.index), thermalManager.temp_hotend[0].target);
			  	EncoderRate.enabled = true;
			  	break;
				#endif
				#if HAS_HEATED_BED
			 	case TUNE_CASE_BTEMP: // Bed temp
			  	 	DwinMenuID = DWMENU_SET_BTMP;
			  		HMI_ValueStruct.Bed_Temp = thermalManager.temp_bed.target;
			  		DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(TUNE_CASE_BTEMP + MROWS - select_tune.index), thermalManager.temp_bed.target);
			  		EncoderRate.enabled = true;
			  	break;
				#endif
				#if HAS_FAN
			 	case TUNE_CASE_FAN: // Fan speed
			  		DwinMenuID = DWMENU_SET_FANSPEED;
			  		HMI_ValueStruct.Fan_speed = thermalManager.fan_speed[0];
			  		DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(TUNE_CASE_FAN + MROWS - select_tune.index), thermalManager.fan_speed[0]);
			  		EncoderRate.enabled = true;
			  	break;
				#endif
				#if ENABLED(BABYSTEPPING)
			 	case TUNE_CASE_ZOFF: // Z-offset
			   	DwinMenuID = DWMENU_SET_ZOFFSET;			
					babyz_offset = last_babyz_offset;
					HMI_ValueStruct.Zoffset_Scale = last_babyz_offset*MAXUNITMULT;
			  	DWIN_Draw_Selected_Small_Float22(MENUVALUE_X, MBASE(TUNE_CASE_ZOFF + MROWS - select_tune.index), HMI_ValueStruct.Zoffset_Scale);
			  	EncoderRate.enabled = true;
			 	break;
				#endif

			 	case TUNE_CASE_MIXER:
					DwinMenuID = DWMENU_MIXER;
			  	select_mixer.reset();
					select_mixer.index = MROWS;
			  	Draw_Mixer_Menu();
			 		break;

					case TUNE_CASE_CONFIG:
			 		DwinMenuID = DWMENU_CONFIG;
			 		select_config.reset();
					select_config.index = MROWS;
			 		Draw_Config_Menu();
			 		break;
				
			 	default: break;
			} 			
		}
	}
	dwinLCD.UpdateLCD();
}

#if HAS_PREHEAT

 /* PLA Preheat */
 void HMI_PLAPreheatSetting() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
   if (select_PLA.inc(1 + PREHEAT_CASE_TOTAL)) Move_Highlight(1, select_PLA.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
   if (select_PLA.dec()) Move_Highlight(-1, select_PLA.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
   switch (select_PLA.now) {
    case 0: // Back
    DwinMenuID = DWMENU_CONTROL;
			select_control.set(CONTROL_CASE_SETPLA);
			select_control.index = MROWS;
			Draw_Control_Menu();
     break;
    #if HAS_HOTEND
     case PREHEAT_CASE_TEMP: // Nozzle temperature
      DwinMenuID = DWMENU_SET_ETMP;
      HMI_ValueStruct.E_Temp = ui.material_preset[0].hotend_temp;
      DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(PREHEAT_CASE_TEMP), ui.material_preset[0].hotend_temp);
      EncoderRate.enabled = true;
      break;
    #endif
    #if HAS_HEATED_BED
     case PREHEAT_CASE_BED: // Bed temperature
      DwinMenuID = DWMENU_SET_BTMP;
      HMI_ValueStruct.Bed_Temp = ui.material_preset[0].bed_temp;
      DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(PREHEAT_CASE_BED), ui.material_preset[0].bed_temp);
      EncoderRate.enabled = true;
      break;
    #endif
    #if HAS_FAN
     case PREHEAT_CASE_FAN: // Fan speed
      DwinMenuID = DWMENU_SET_FANSPEED;
      HMI_ValueStruct.Fan_speed = ui.material_preset[0].fan_speed;
      DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(PREHEAT_CASE_FAN), ui.material_preset[0].fan_speed);
      EncoderRate.enabled = true;
      break;
    #endif
    #if ENABLED(EEPROM_SETTINGS)
     case PREHEAT_CASE_SAVE: { // Save PLA configuration
      HMI_AudioFeedback(settings.save());
     } break;
    #endif
    default: break;
   }
  }
  dwinLCD.UpdateLCD();
 }

 /* ABS Preheat */
 void HMI_ABSPreheatSetting() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
   if (select_ABS.inc(1 + PREHEAT_CASE_TOTAL)) Move_Highlight(1, select_ABS.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
   if (select_ABS.dec()) Move_Highlight(-1, select_ABS.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
   switch (select_ABS.now) {
    case 0: // Back
			DwinMenuID = DWMENU_CONTROL;
			select_control.set(CONTROL_CASE_SETABS);
			select_control.index = MROWS;
			Draw_Control_Menu();
     break;
    #if HAS_HOTEND
     case PREHEAT_CASE_TEMP: // Set nozzle temperature
      DwinMenuID = DWMENU_SET_ETMP;
      HMI_ValueStruct.E_Temp = ui.material_preset[1].hotend_temp;
      DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(PREHEAT_CASE_TEMP), ui.material_preset[1].hotend_temp);
      EncoderRate.enabled = true;
      break;
    #endif
    #if HAS_HEATED_BED
     case PREHEAT_CASE_BED: // Set bed temperature
      DwinMenuID = DWMENU_SET_BTMP;
      HMI_ValueStruct.Bed_Temp = ui.material_preset[1].bed_temp;
      DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(PREHEAT_CASE_BED), ui.material_preset[1].bed_temp);
      EncoderRate.enabled = true;
      break;
    #endif
    #if HAS_FAN
     case PREHEAT_CASE_FAN: // Set fan speed
      DwinMenuID = DWMENU_SET_FANSPEED;
      HMI_ValueStruct.Fan_speed = ui.material_preset[1].fan_speed;
      DWIN_Draw_Select_IntValue_Default(3, MENUVALUE_X+8, MBASE(PREHEAT_CASE_FAN), ui.material_preset[1].fan_speed);
      EncoderRate.enabled = true;
      break;
    #endif
    #if ENABLED(EEPROM_SETTINGS)
     case PREHEAT_CASE_SAVE: { // Save ABS configuration
      HMI_AudioFeedback(settings.save());
     } break;
    #endif
    default: break;
   }
  }
  dwinLCD.UpdateLCD();
 }

#endif

/* Max Speed */
void HMI_MaxSpeed() {
 ENCODER_DiffState encoder_diffState = get_encoder_state();
 if (encoder_diffState == ENCODER_DIFF_NO) return;

 // Avoid flicker by updating only the previous menu
 if (encoder_diffState == ENCODER_DIFF_CW) {
  if (select_feedrate.inc(1 + 3 + ENABLED(HAS_HOTEND))) Move_Highlight(1, select_feedrate.now);
 }
 else if (encoder_diffState == ENCODER_DIFF_CCW) {
  if (select_feedrate.dec()) Move_Highlight(-1, select_feedrate.now);
 }
 else if (encoder_diffState == ENCODER_DIFF_ENTER) {
  if (WITHIN(select_feedrate.now, 1, 4)) {
   DwinMenuID = DWMENU_SET_MAXSPEED_VALUE;
   HMI_flag.feedspeed_axis = AxisEnum(select_feedrate.now - 1);
   HMI_ValueStruct.Max_Feedspeed = planner.settings.max_feedrate_mm_s[HMI_flag.feedspeed_axis];
   DWIN_Draw_Select_IntValue_Default(3, CONFIGVALUE_X+8, MBASE(select_feedrate.now), HMI_ValueStruct.Max_Feedspeed);
   EncoderRate.enabled = true;
  }
  else { // Back
   select_motion.set(MOTION_CASE_RATE);
   DwinMenuID = DWMENU_MOTION;   
   Draw_Motion_Menu();
  }
 }
 dwinLCD.UpdateLCD();
}

/* Max Acceleration */
void HMI_MaxAcceleration() {
 ENCODER_DiffState encoder_diffState = get_encoder_state();
 if (encoder_diffState == ENCODER_DIFF_NO) return;

 // Avoid flicker by updating only the previous menu
 if (encoder_diffState == ENCODER_DIFF_CW) {
  if (select_accel.inc(1 + 3 + ENABLED(HAS_HOTEND))) Move_Highlight(1, select_accel.now);
 }
 else if (encoder_diffState == ENCODER_DIFF_CCW) {
  if (select_accel.dec()) Move_Highlight(-1, select_accel.now);
 }
 else if (encoder_diffState == ENCODER_DIFF_ENTER) {
  if (WITHIN(select_accel.now, 1, 4)) {
   DwinMenuID = DWMENU_SET_MAXACC_VALUE;
   HMI_flag.acc_axis = AxisEnum(select_accel.now - 1);
   HMI_ValueStruct.Max_Acceleration = planner.settings.max_acceleration_mm_per_s2[HMI_flag.acc_axis];
   DWIN_Draw_Select_IntValue_Default(5, CONFIGVALUE_X-8, MBASE(select_accel.now), HMI_ValueStruct.Max_Acceleration);
   EncoderRate.enabled = true;
  }
  else { // Back
   select_motion.set(MOTION_CASE_ACCEL);
   DwinMenuID = DWMENU_MOTION;   
   Draw_Motion_Menu();
  }
 }
 dwinLCD.UpdateLCD();
}

#if HAS_CLASSIC_JERK
 /* Max Jerk */
 void HMI_MaxJerk() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
   if (select_jerk.inc(1 + 3 + ENABLED(HAS_HOTEND))) Move_Highlight(1, select_jerk.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
   if (select_jerk.dec()) Move_Highlight(-1, select_jerk.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
   if (WITHIN(select_jerk.now, 1, 4)) {
    DwinMenuID = DWMENU_SET_MAXJERK_VALUE;
    HMI_flag.jerk_axis = AxisEnum(select_jerk.now - 1);
    HMI_ValueStruct.Max_Jerk = planner.max_jerk[HMI_flag.jerk_axis] * MINUNITMULT;
    DWIN_Draw_Selected_Small_Float21(CONFIGVALUE_X+8, MBASE(select_jerk.now), HMI_ValueStruct.Max_Jerk);
    EncoderRate.enabled = true;
   }
   else { // Back
    select_motion.set(MOTION_CASE_JERK);
    DwinMenuID = DWMENU_MOTION;    
    Draw_Motion_Menu();
   }
  }
  dwinLCD.UpdateLCD();
 }
#endif // HAS_CLASSIC_JERK

/* Step */
void HMI_StepPermm() {
 ENCODER_DiffState encoder_diffState = get_encoder_state();
 if (encoder_diffState == ENCODER_DIFF_NO) return;

 // Avoid flicker by updating only the previous menu
 if (encoder_diffState == ENCODER_DIFF_CW) {
  if (select_step.inc(1 + 3 + ENABLED(HAS_HOTEND))) Move_Highlight(1, select_step.now);
 }
 else if (encoder_diffState == ENCODER_DIFF_CCW) {
  if (select_step.dec()) Move_Highlight(-1, select_step.now);
 }
 else if (encoder_diffState == ENCODER_DIFF_ENTER) {
  if (WITHIN(select_step.now, 1, 4)) {
   DwinMenuID = DWMENU_SET_STEPPREMM_VALUE;
   HMI_flag.step_axis = AxisEnum(select_step.now - 1);
   HMI_ValueStruct.Max_Step = planner.settings.axis_steps_per_mm[HMI_flag.step_axis] * MINUNITMULT;
   DWIN_Draw_Selected_Small_Float41(CONFIGVALUE_X-8, MBASE(select_step.now), HMI_ValueStruct.Max_Step);
   EncoderRate.enabled = true;
  }
  else { // Back   
   select_motion.set(MOTION_CASE_STEPS);
	 DwinMenuID = DWMENU_MOTION;
   Draw_Motion_Menu();
  }
 }
 dwinLCD.UpdateLCD();
}

void HMI_Init() {
	Encoder_Configuration();
	
	dwinLCD.JPG_ShowAndCache(0);
	HMI_SDCardInit();

	for (uint16_t t = 0; t <= 100; t += 2) {
		DWIN_Show_ICON( ICON_Bar, 15, 260);
		dwinLCD.Draw_Rectangle(1, Color_Bg_Black, 15 + t * 242 / 100, 260, 257, 280);
		dwinLCD.UpdateLCD();
		delay(20);
	}
	HMI_SetLanguage();
	HMI_StartFrame(true);
}

#if ENABLED(DEBUG_GCODE_M92)
void DWIN_Gcode_Show_M92(uint8_t AXIS,float lengh){
	if(AXIS == X_AXIS) {
		DWIN_Draw_UnMaskString_Default(14, 456,F("X:"));
		DWIN_Draw_Small_Float31(30, 456, lengh*10);
	}
	if(AXIS == Y_AXIS) {
		DWIN_Draw_UnMaskString_Default(78, 456,F("Y:"));
		DWIN_Draw_Small_Float31(94, 456, lengh*10);
	}
	if(AXIS == Z_AXIS) {
		DWIN_Draw_UnMaskString_Default(140, 456,F("Z:"));
		DWIN_Draw_Small_Float31(156, 456, lengh*10);
	}
	if(AXIS == E_AXIS) {
		DWIN_Draw_UnMaskString_Default(204, 456,F("E:"));
		DWIN_Draw_Small_Float31(220, 456, lengh*10);
	}
}
#endif

#if ENABLED(OPTION_WIFI_MODULE)
#define MAX_WIFI_MESSAGE_LENGTH 27
#define START_OF_UTF8_CHAR(C) (((C) & 0xC0u) != 0x80U)

void DWIN_Wifi_Show_M117(const char * const message){
	HMI_flag.wifi_Handshake_ok = true;
	HMI_flag.wifi_link_timer = 0;
	WiFi_Connected_fail = false;
	if(IS_SD_PRINTING() || IS_SD_PAUSED()) return;

	char wifi_status_message[MAX_WIFI_MESSAGE_LENGTH+1] = {0};
	DwinMenuID = DWMENU_SET_WIFIONOFF;
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	Draw_Popup_Bkgd_Wifi();
	Draw_Title(F("WIFI"));
	Draw_Wifi_Title("M117");
	DWIN_Show_ICON( ICON_Confirm_E, 86, 231);

	// Here we have a problem. The message is encoded in UTF8, so
	// arbitrarily cutting it will be a problem. We MUST be sure
	// that there is no cutting in the middle of a multibyte character!
	// Get a pointer to the null terminator
	const char* pend = message + strlen(message);
	// If length of supplied UTF8 string is greater than
	// our buffer size, start cutting whole UTF8 chars
	while ((pend - message) > MAX_WIFI_MESSAGE_LENGTH) {
	 --pend;
	 while (!START_OF_UTF8_CHAR(*pend)) --pend;
	};
	
	uint8_t maxLen = pend - message;
	strncpy(wifi_status_message, message, maxLen);
	wifi_status_message[maxLen] = '\0';
	dwinLCD.Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * strlen(wifi_status_message)) / 2, 168, wifi_status_message);	
}
#endif

void DWIN_Draw_PrintDone_Confirm(){
	// show print done confirm
	DwinMenuID = DWMENU_POP_STOPPRINT;
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	DWIN_Draw_MaskString_FONT12(Popup_Text_Color, Color_Bg_Window, (272 - 11 * 12) / 2, 150, F("Print Done,"));
	DWIN_Draw_MaskString_FONT12(Popup_Text_Color, Color_Bg_Window, (272 - 14 * 12) / 2, 200, F("Click to exit!"));
	dwinLCD.Draw_Rectangle(1, Color_Bg_Black, 0, 250, DWIN_WIDTH - 1, STATUS_Y_START);
	dwinLCD.ICON_Show(ICON_IMAGE_ID,ICON_Confirm_E, 86, 283);
}

#if(ENABLED(POWER_LOSS_RECOVERY))
inline void _check_Powerloss_resume(){
	static bool recovery_flag = false;

	auto update_selection = [&](const bool sel) {
			HMI_flag.select_flag = sel;
			const uint16_t c1 = sel ? Color_Bg_Window : Select_Color;
			dwinLCD.Draw_Rectangle(0, c1, 25, 306, 126, 345);
			dwinLCD.Draw_Rectangle(0, c1, 24, 305, 127, 346);
			const uint16_t c2 = sel ? Select_Color : Color_Bg_Window;
			dwinLCD.Draw_Rectangle(0, c2, 145, 306, 246, 345);
			dwinLCD.Draw_Rectangle(0, c2, 144, 305, 247, 346);
		};
	
	if(HMI_flag.lcd_sd_status && recovery.dwin_flag && recovery.enabled){
		recovery.dwin_flag = false;
		recovery_flag = true;
		Popup_Window_Resume();
		update_selection(true);
		/// TODO: Get the name of the current file from someplace
		//
		//(void)recovery.interrupted_file_exists();
		char * const name = card.longest_filename();
		const int8_t npos = _MAX(0U, DWIN_WIDTH - strlen(name) * 10) / 2;
		DWIN_Draw_MaskString_Default_PopMenu( npos, 252, name);
		dwinLCD.UpdateLCD();

		while (recovery_flag) {
			ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
			if (encoder_diffState != ENCODER_DIFF_NO) {
				if (encoder_diffState == ENCODER_DIFF_ENTER) {
					recovery_flag = false;
					if(HMI_flag.select_flag) break;
					queue.inject_P(PSTR("M1000C"));
					HMI_StartFrame(true);
					return;
				}
				else update_selection(encoder_diffState == ENCODER_DIFF_CW);
				dwinLCD.UpdateLCD();
			}
			TERN_(USE_WATCHDOG, HAL_watchdog_refresh());
		}
		recovery.cancel();
		select_print.set(0);
		HMI_flag.show_mode = 0;
		queue.inject_P(PSTR("M1000"));		
		mixer.selected_vtool = MixerCfg.Vtool_Backup;
		DWIN_status = ID_SM_PRINTING;	
		Draw_Status_Area(true);
		Start_PrintProcess();
	}
}
#endif//(ENABLED(POWER_LOSS_RECOVERY))

#if ENABLED(OPTION_WIFI_MODULE)
inline void _check_wifi(){
	if(WiFi_Enabled){
	 	if(!HMI_flag.wifi_Handshake_ok && !WiFi_Connected_fail){
			if(HMI_flag.wifi_link_timer++ > 12){
				HMI_flag.wifi_link_timer = 0;
				WiFi_Connected_fail = true;
				Popup_Window_Wifi_Disconnect();
			}
			else{
				if(DwinMenuID != DWMENU_POP_WIFI){
					DwinMenuID = DWMENU_POP_WIFI;
					HMI_flag.wifi_link_timer = 0;		
					Popup_Window_Wifi_Connect();
				}
				else{
					if((HMI_flag.wifi_link_timer%5) != 0)	
						dwinLCD.Draw_String(false, true, font12x24, Popup_Text_Color, Color_Bg_Window, 172 + 12*(HMI_flag.wifi_link_timer%5 - 1), 240,F("."));
					else 
						dwinLCD.Draw_String(false, true, font12x24, Popup_Text_Color, Color_Bg_Window, 172, 240,F("    "));
				}
			}
		}		
		else if(WiFi_Connected_fail && DwinMenuID == DWMENU_POP_WIFI){
			if(HMI_flag.wifi_link_timer++ > 3){
				HMI_flag.wifi_link_timer = 0;
				DwinMenuID = DWMENU_CONFIG;
				select_config.set(CONFIG_CASE_WIFI);
				select_config.index = MROWS;
				Draw_Config_Menu();
			}
		}
	}
}
#endif

#if ENABLED(OPTION_AUTOPOWEROFF)
inline void _check_autoshutdown(){
	if((HMI_flag.Autoshutdown_enabled) && (DwinMenuID == DWMENU_POP_STOPPRINT || DwinMenuID == DWMENU_MAIN)){
		//is heating?
		if(thermalManager.temp_hotend[0].target > 50*1000/DWIN_SCROLL_UPDATE_INTERVAL || thermalManager.temp_bed.target > 25*1000/DWIN_SCROLL_UPDATE_INTERVAL){
			HMI_flag.free_close_timer_rg = POWERDOWN_MACHINE_TIMER;
		}		
		else if(HMI_flag.free_close_timer_rg == 0){
			queue.inject_P(PSTR("M81"));
			HMI_flag.free_close_timer_rg = POWERDOWN_MACHINE_TIMER;
			return;
		}
		else {
			HMI_flag.free_close_timer_rg--;
			if(HMI_flag.free_close_timer_rg < 10) buzzer.tone(50, 5000);
		}
		Draw_Freedown_Machine();
	}
	else 
		HMI_flag.free_close_timer_rg = POWERDOWN_MACHINE_TIMER;	
}

void _reset_shutdown_timer(){
	if(HMI_flag.Autoshutdown_enabled) HMI_flag.free_close_timer_rg = POWERDOWN_MACHINE_TIMER;
}
#endif

#if ENABLED(OPTION_REPEAT_PRINTING)
inline bool _check_repeatPrint(){
	switch(ReprintManager.Reprint_check_state()){
		case REPRINT_FINISHED:
			select_main.reset();
			DWIN_status = ID_SM_STOPED;
			return true;
			
		case REPRINT_NEXT:
			HMI_flag.show_mode = 0;
			card.openAndPrintFile(card.filename);
			DWIN_status = ID_SM_PRINTING;
			select_print.reset();
			Start_PrintProcess();
			return true; 		
			
		default:
			break;
	}  
	return false;
}
#endif

void DWIN_HandleScreen() {	
 	switch (DwinMenuID){
		case DWMENU_MAIN:    						HMI_MainMenu(); break;
		case DWMENU_FILE:   						HMI_SelectFile(); break;
		case DWMENU_PREPARE:     				HMI_Prepare(); break;
		case DWMENU_CONTROL:     				HMI_Control(); break;
		
		case DWMENU_LEVELING: 					
		case DWMENU_POP_LEVEL_CATCH:		
		case DWMENU_LEVEL_DONECONFIRM:	HMI_BedLeveling(); break;
		case DWMENU_LEVEL_SETOFFSET:    HMI_SetProbZoffset(); break;				
			
		case DWMENU_HOME:    	 					HMI_Home(); break; 

		case DWMENU_POP_STOPPRINT:  		
		case DWMENU_PRINTING:  					HMI_Printing(); break;
		case DWMENU_POP_PAUSEORSTOP:		HMI_PauseOrStop(); break;
		case DWMENU_MOVEAXIS:  					HMI_AxisMove(); break;
		case DWMENU_TEMPERATURE:  	 		HMI_Temperature(); break;
		case DWMENU_MOTION:    					HMI_Motion(); break;
		case DWMENU_MIXER:     					HMI_Mixer(); break;
		case DWMENU_INFO:      					HMI_Info(); break;
		case DWMENU_TUNE:      					HMI_Tune(); break;
	
#if HAS_PREHEAT
		case DWMENU_PREHEAT_PLA:  			HMI_PLAPreheatSetting(); break;
		case DWMENU_PREHEAT_ABS:  			HMI_ABSPreheatSetting(); break;
#endif

	  case DWMENU_SET_MAXSPEED: 			HMI_MaxSpeed(); break;
	  case DWMENU_SET_MAXACC: 				HMI_MaxAcceleration(); break;
	  case DWMENU_SET_MAXJERK:  			HMI_MaxJerk(); break;
	  case DWMENU_SET_STEPPREMM:			HMI_StepPermm(); break;
	  case DWMENU_MOVEX:     					HMI_Move_X(); break;
	  case DWMENU_MOVEY:     					HMI_Move_Y(); break;
	  case DWMENU_MOVEZ:     					HMI_Move_Z(); break;
		
#if HAS_HOTEND
		case DWMENU_SET_ETMP:     			HMI_ETemp(); break;
		case DWMENU_MOVE_EXT1:   				HMI_Move_Extr(0); break;
	#if(E_STEPPERS > 1)
	  case DWMENU_MOVE_EXT2:   				HMI_Move_Extr(1); break;
	#endif
	#if(E_STEPPERS > 2)
		case DWMENU_MOVE_EXT3:   				HMI_Move_Extr(2); break;
	#endif
	#if(E_STEPPERS > 3)
		case DWMENU_MOVE_EXT4:   				HMI_Move_Extr(3); break;
	#endif
	#if ENABLED(MIXING_EXTRUDER)
		case DWMENU_MOVE_EXTALL:   			HMI_Move_AllExtr(); break;	
	#endif   
#endif	

	#if HAS_HEATED_BED
	  case DWMENU_SET_BTMP:   		 		HMI_BedTemp(); break;
	#endif

	#if HAS_PREHEAT
		case DWMENU_SET_FANSPEED:   		HMI_FanSpeed(); break;
	#endif
	
  #if ENABLED(BABYSTEPPING)
	  case DWMENU_SET_ZOFFSET:  			HMI_Zoffset(); break;
	#endif

	  case DWMENU_TUNE_PRINTSPEED:   	HMI_PrintSpeed(); break;
	  case DWMENU_SET_MAXSPEED_VALUE: HMI_MaxFeedspeedXYZE(); break;
	  case DWMENU_SET_MAXACC_VALUE: 	HMI_MaxAccelerationXYZE(); break;
	  case DWMENU_SET_MAXJERK_VALUE: 	HMI_MaxJerkXYZE(); break;
	  case DWMENU_SET_STEPPREMM_VALUE: HMI_StepXYZE(); break;

#if ENABLED(MIXING_EXTRUDER)
		case DWMENU_MIXER_MANUAL:   		HMI_Mixer_Manual(); break;
		case DWMENU_MIXER_AUTO:    			HMI_Mixer_Gradient(); break;
		case DWMENU_MIXER_RANDOM:   		HMI_Mixer_Random(); break;
		case DWMENU_MIXER_VTOOL:    		HMI_Adjust_Mixer_Vtool(); break;
		
#if(MIXING_STEPPERS == 4)
		case DWMENU_MIXER_EXT1:  				HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER1);break;
		case DWMENU_MIXER_EXT2:  				HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER2);break;
		case DWMENU_MIXER_EXT3: 				HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER3);break;
		case DWMENU_MIXER_EXT4:  				HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER4);break;
		
	 #elif(MIXING_STEPPERS == 3)
		case DWMENU_MIXER_EXT1:  				HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER1);break;
		case DWMENU_MIXER_EXT2:  				HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER2);break;
		case DWMENU_MIXER_EXT3:  				HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER3);break;
	 #elif(MIXING_STEPPERS == 2)
		case DWMENU_MIXER_EXT1:  				HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER1);break;
		case DWMENU_MIXER_EXT2:  				HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER2);break;
	 #else
		case DWMENU_MIXER_EXT1:  				HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER1);break;
	 #endif
#endif
	
		case DWMENU_MIXER_GRADIENT_ZSTART:		HMI_Adjust_Auto_Zpos_Start(); break;
		case DWMENU_MIXER_GRADIENT_ZEND:  	 	HMI_Adjust_Auto_Zpos_End(); break;
		case DWMENU_MIXER_GRADIENT_TSTAR:   	HMI_Adjust_Auto_VTool_Start(); break;
		case DWMENU_MIXER_GRADIENT_TEND:   	 	HMI_Adjust_Auto_VTool_End(); break;

		case DWMENU_MIXER_RANDOM_ZSTART:   	 	HMI_Adjust_Random_Zpos_Start(); break;
		case DWMENU_MIXER_RANDOM_ZEND:  	 	 	HMI_Adjust_Random_Zpos_End(); break;
		case DWMENU_MIXER_RANDOM_HEIGHT:   	 	HMI_Adjust_Random_Height(); break;
		case DWMENU_MIXER_RANDOM_EXTN:  	 	 	HMI_Adjust_Random_Extruders(); break;

		case DWMENU_POP_FROD_OPTION:					HMI_Filament_Runout_Option(); break;
		case DWMENU_POP_FROD_CONFIRM:					HMI_Filament_Runout_Confirm(); break;

		case DWMENU_POWERDOWN:								HMI_Powerdown(); break;
		case DWMENU_LANGUAGE:									HMI_Language(); break;

 #if ENABLED(BABYSTEPPING)
		case DWMENU_TUNE_BABYSTEPS:						HMI_Pop_BabyZstep(); break;
	#endif

		case DWMENU_CONFIG:										HMI_Config(); break;
		
#if ENABLED(BLTOUCH)
		case DWMENU_SET_BLTOUCH: 							HMI_Option_Bltouch(); break;
#endif

	#if ENABLED(FWRETRACT) 
		case DWMENU_SET_RETRACT:							HMI_Retract(); break;
		case DWMENU_SET_RETRACT_MM:						HMI_Retract_MM(); break;
		case DWMENU_SET_RETRACT_V:						HMI_Retract_V(); break;
		case DWMENU_SET_RETRACT_ZHOP:					HMI_Retract_ZHOP(); break;		
		case DWMENU_SET_UNRETRACT_MM:					HMI_UnRetract_MM(); break;
		case DWMENU_SET_UNRETRACT_V:					HMI_UnRetract_V(); break;
	#endif

	#if ENABLED(OPTION_WIFI_MODULE)
		case DWMENU_SET_WIFIONOFF:						HMI_Wifi(); break;
	#endif
	
	#if ENABLED(OPTION_BED_COATING)
	  case DWMENU_SET_BEDCOATING:						HMI_Adjust_Coating_Thickness(); break;
	#endif
	
	#if ENABLED(OPTION_REPEAT_PRINTING)
		case DWMENU_SET_REPRINT:							HMI_Reprint(); break;
		case DWMENU_SET_REPRINT_TIMES:				HMI_Reprint_Times(); break;
		case DWMENU_SET_REPRINT_RUNLENGTH:		HMI_Forward_Lenght(); break;
	#endif
	
	case DWMENU_POP_WAITING:								HMI_Waiting(); break;
  default: break;
	}
}

void DWIN_PopMenu_HomeDone() {
	if (DwinMenuID == DWMENU_POP_HOME){
		DwinMenuID = DWMENU_HOME;
		Draw_Home_Menu();
	}	
	dwinLCD.UpdateLCD();
}

inline void DRAW_Pause_Mode(const PauseMode mode){
	switch (mode){
		case PAUSE_MODE_PAUSE_PRINT:
			Draw_Title(F("Print Paused"));
			break;
		case PAUSE_MODE_CHANGE_FILAMENT:
			Draw_Title(F("Filament Change"));
			break;
		case PAUSE_MODE_LOAD_FILAMENT:
			Draw_Title(F("Filament Load"));
			break;
		case PAUSE_MODE_UNLOAD_FILAMENT:
			Draw_Title(F("Filament Unload"));
			break;			
		case PAUSE_MODE_SAME:
		default:break;
	}
}
inline void Popup_window_Pause_Parking(const PauseMode mode) {
 Clear_Main_Window();
 Draw_Popup_Bkgd_60();
 DWIN_Show_ICON(ICON_WAITING, 86, 105);
 DRAW_Pause_Mode(mode);
 DWIN_Draw_String_FIL((DWIN_WIDTH - FIL.Font_W * 15) / 2, 240, F("Parking hotend!"));
}

inline void Popup_window_Pause_Start(const PauseMode mode) {
 Clear_Main_Window();
 Draw_Popup_Bkgd_60();
 DWIN_Show_ICON(ICON_WAITING, 86, 105);
 DRAW_Pause_Mode(mode);
 DWIN_Draw_String_FIL((DWIN_WIDTH - FIL.Font_W * 15) / 2, 240, F("Filament change"));
 DWIN_Draw_String_FIL((DWIN_WIDTH - FIL.Font_W * 9) / 2, 269, F("to start!"));
}

inline void Popup_window_Pause_Heating(const PauseMode mode) {
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	DWIN_Show_ICON(ICON_WAITING, 86, 105);
	DRAW_Pause_Mode(mode);
	DWIN_Draw_String_FIL((DWIN_WIDTH - FIL.Font_W * 8) / 2, 240, F("Wait for"));
	DWIN_Draw_String_FIL((DWIN_WIDTH - FIL.Font_W * 17) / 2, 269, F("Nozzle Heating..."));
}

inline void Popup_window_Pause_Insert(const PauseMode mode) {
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	DRAW_Pause_Mode(mode);
	DWIN_Show_ICON(ICON_Confirm_E, 86, 168);
	DWIN_Draw_String_FIL((DWIN_WIDTH - FIL.Font_W * 15) / 2, 211, F("Insert filament"));
	DWIN_Draw_String_FIL((DWIN_WIDTH - FIL.Font_W * 16) / 2, 240, F("and press button"));
	DWIN_Draw_String_FIL((DWIN_WIDTH - FIL.Font_W * 12) / 2, 269, F("to continue!"));
}

inline void Popup_window_Pause_Unload(const PauseMode mode) {
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	DWIN_Show_ICON( ICON_WAITING, 86, 105);
	DRAW_Pause_Mode(mode);
	DWIN_Draw_String_FIL((DWIN_WIDTH - FIL.Font_W * 8) / 2, 240, F("Wait for"));
	DWIN_Draw_String_FIL((DWIN_WIDTH - FIL.Font_W * 16) / 2, 269, F("Filament Unload!"));
}

inline void Popup_window_Pause_Load(const PauseMode mode) {
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	DWIN_Show_ICON(ICON_WAITING, 86, 105);
	DRAW_Pause_Mode(mode);
	DWIN_Draw_String_FIL((DWIN_WIDTH - FIL.Font_W * 8) / 2, 240, F("Wait for..."));
	DWIN_Draw_String_FIL((DWIN_WIDTH - FIL.Font_W * 14) / 2, 269, F("Filament Load!"));
}

inline void Popup_window_Pause_Purge(const PauseMode mode) {
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	DWIN_Show_ICON(ICON_WAITING, 86, 105);
	DRAW_Pause_Mode(mode);
	DWIN_Draw_String_FIL((DWIN_WIDTH - FIL.Font_W * 8) / 2, 240, F("Wait for..."));
	DWIN_Draw_String_FIL((DWIN_WIDTH - FIL.Font_W * 15) / 2, 269, F("Filament Purge!"));
}

inline void Popup_window_Pause_Option(const PauseMode mode) {
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	DRAW_Pause_Mode(mode);
	DWIN_Show_ICON(ICON_YES_0, 26, 168);
	DWIN_Show_ICON(ICON_NO_1, 146, 168);
	DWIN_Draw_String_FIL((DWIN_WIDTH - FIL.Font_W * 18) / 2, 240, F("Purge More or not!"));
}

inline void Popup_window_Pause_Resume(const PauseMode mode) {
	Clear_Main_Window();
	Draw_Popup_Bkgd_60();
	DWIN_Show_ICON(ICON_WAITING, 86, 105);
	DRAW_Pause_Mode(mode);
	DWIN_Draw_String_FIL((DWIN_WIDTH - FIL.Font_W * 13) / 2, 240, F("Resume Print..."));
}

inline void DRAW_Pause_Message(PauseMessage message, PauseMode mode){
	#if ENABLED(DEBUG_DWIN_LCD)
		SERIAL_ECHOPAIR("Pause message = ", message);
		SERIAL_ECHOLNPAIR("  Pause mode = ", mode);		
	#endif
	DWIN_status = ID_SM_PAUSING;
	DwinMenuID = DWMENU_POP_WAITING;	
	switch (message){
		case PAUSE_MESSAGE_PARKING:
			Clear_Bottom_Area();			
			Popup_window_Pause_Parking(mode);
			DWIN_status = ID_SM_PAUSING;
			DWIN_Draw_MaskString_Default_Color(Color_Red, 10, 455, F("Please wait for pause..."));
			break;
		case PAUSE_MESSAGE_CHANGING:
			Clear_Bottom_Area();
			DWIN_status = ID_SM_RUNOUTING;
			DWIN_Draw_MaskString_Default_Color(Color_Red, 10, 455, F("Changing..."));
			Popup_window_Pause_Start(mode);
			break;
		case PAUSE_MESSAGE_WAITING:
			Clear_Bottom_Area();
			DWIN_status = ID_SM_RUNOUTING;
			DWIN_Draw_MaskString_Default_Color(Color_Red, 10, 455, F("Please wait restart..."));
			break;
		case PAUSE_MESSAGE_UNLOAD:
			Clear_Bottom_Area();
			DWIN_status = ID_SM_RUNOUTING;
			Popup_window_Pause_Unload(mode);
			DWIN_Draw_MaskString_Default_Color(Color_Red, 10, 455, F("Unload Filament..."));
			break;
		case PAUSE_MESSAGE_INSERT:
			Clear_Bottom_Area();
			DWIN_status = ID_SM_RUNOUTING;
			DwinMenuID = DWMENU_POP_FROD_CONFIRM;
			Popup_window_Pause_Insert(mode);
			DWIN_Draw_MaskString_Default_Color(Color_Red, 10, 455, F("Wait for inserting Filament"));
			break;
		case PAUSE_MESSAGE_LOAD:
			Clear_Bottom_Area();
			DWIN_status = ID_SM_RUNOUTING;
			Popup_window_Pause_Load(mode);
			DWIN_Draw_MaskString_Default_Color(Color_Red, 10, 455, F("Loading Filament..."));
			break;
		case PAUSE_MESSAGE_PURGE:
			Clear_Bottom_Area();
			DWIN_status = ID_SM_RUNOUTING;
			DWIN_Draw_MaskString_Default_Color(Color_Red, 10, 455, F("Purging Filament!"));
			Popup_window_Pause_Purge(mode);
			break;
		case PAUSE_MESSAGE_OPTION:
			Clear_Bottom_Area();
			DWIN_status = ID_SM_RUNOUTING;
			DwinMenuID = DWMENU_POP_FROD_OPTION;
			pause_menu_response = PAUSE_RESPONSE_WAIT_FOR;
			DWIN_Draw_MaskString_Default_Color(Color_Red, 10, 455, F("Purge more or not!"));
			Popup_window_Pause_Option(mode);
			break;
		case PAUSE_MESSAGE_RESUME:
			Clear_Bottom_Area();
			DWIN_status = ID_SM_RUNOUT_DONE;
			DWIN_Draw_MaskString_Default_Color(Color_Red, 10, 455, F("Please wait heat done!"));
			Popup_window_Pause_Resume(mode);
			break;			
		case PAUSE_MESSAGE_HEATING:
			Clear_Bottom_Area();
			DWIN_status = ID_SM_PAUSING;
			thermalManager.setTargetBed(tempbed);
			thermalManager.setTargetHotend(temphot,0);
			DWIN_Draw_MaskString_Default_Color(Color_Red, 10, 455, F("Is heating, please wait!"));
			Popup_window_Pause_Heating(mode);				
			break;
		case PAUSE_MESSAGE_HEAT:
			Clear_Bottom_Area();
			DWIN_status = ID_SM_PAUSING;
			thermalManager.setTargetBed(tempbed);
			thermalManager.setTargetHotend(temphot,0);
			DWIN_Draw_MaskString_Default_Color(Color_Red, 10, 455, F("Start to heat!"));
			break;
		case PAUSE_MESSAGE_STATUS:
			Clear_Bottom_Area();
			DWIN_Draw_MaskString_Default_Color(Color_Red, 10, 455, F("Done!"));
			break;
		default:break;
	}
	dwinLCD.UpdateLCD();
}

void DWIN_Pause_Show_Message(const PauseMessage message,const PauseMode mode, const uint8_t extruder){	
	UNUSED(extruder);
	DRAW_Pause_Message(message,mode);
}
void DWIN_Pause_Show_Message(const PauseMessage message, const PauseMode mode){	
	DRAW_Pause_Message(message,mode);
}
void DWIN_Pause_Show_Message(const PauseMessage message){	
	DRAW_Pause_Message(message,PAUSE_MODE_SAME);
}

#if (ABL_GRID)
void DWIN_PopMenu_LevelingDone() {
	if (DwinMenuID == DWMENU_LEVEL_BEDLEVELING) {
		DWIN_G29_Show_Messge(G29_MESH_DONE);		
		DwinMenuID = DWMENU_LEVEL_DONECONFIRM;
		DWIN_Show_ICON( ICON_Confirm_E, 86, 332);		
	}
	#if ENABLED(AUTO_UPDATA_PROBE_Z_OFFSET)
	else if(DwinMenuID == DWMENU_LEVEL_CATCHOFFSET){
		DwinMenuID = DWMENU_LEVELING;
		select_leveling.index = LEVELING_CASE_ACTION;
		select_leveling.set(select_leveling.index);
		Draw_Leveling_Menu();	
	}
	#endif
}

#if(GRID_MAX_POINTS_X >= 3 && GRID_MAX_POINTS_X <= 7)
#define	TABLE_BOTTOM	330
#define	TABLE_LEFT		14
#define	TABLE_RIGHT		258
#define	TABLE_WIDTH		244
#if (GRID_MAX_POINTS_X == 3)
#define	FONT_G29TABLE	font10x20
#define	TABLE_TOP			176
#define	TABLE_HEIGTH	(TABLE_BOTTOM-TABLE_TOP)
#define	BAR_WIDTH 		72
#define	BAR_GAP_X 		8
#define	X_START				(BAR_WIDTH-5*10)/2
#define	ADJUST_X			((TABLE_WIDTH-BAR_WIDTH*GRID_MAX_POINTS_X-BAR_GAP_X*(GRID_MAX_POINTS_X-1))/2)
#define	BAR_HEIGTH		32
#define	BAR_GAP_Y			12	
#define	Y_START				(BAR_HEIGTH-20)/2
#define	ADJUST_Y			((TABLE_HEIGTH - BAR_HEIGTH*GRID_MAX_POINTS_Y - BAR_GAP_Y*(GRID_MAX_POINTS_Y-1))/2)
#elif (GRID_MAX_POINTS_X == 4)
#define	FONT_G29TABLE	font10x20
#define	TABLE_TOP			176
#define	TABLE_HEIGTH	(TABLE_BOTTOM-TABLE_TOP)
#define	BAR_WIDTH 		56
#define	BAR_GAP_X 		6
#define	X_START				(BAR_WIDTH-5*10)/2
#define	ADJUST_X			((TABLE_WIDTH-BAR_WIDTH*GRID_MAX_POINTS_X-BAR_GAP_X*(GRID_MAX_POINTS_X-1))/2)
#define	BAR_HEIGTH		30
#define	BAR_GAP_Y			8
#define	Y_START				(BAR_HEIGTH-20)/2
#define	ADJUST_Y			((TABLE_HEIGTH - BAR_HEIGTH*GRID_MAX_POINTS_Y - BAR_GAP_Y*(GRID_MAX_POINTS_Y-1))/2)
#elif (GRID_MAX_POINTS_X == 5)
#define	FONT_G29TABLE	font8x16
#define	TABLE_TOP			150
#define	TABLE_HEIGTH	(TABLE_BOTTOM-TABLE_TOP)
#define	BAR_WIDTH 		46
#define	BAR_GAP_X 		3
#define	X_START				(BAR_WIDTH-5*8)/2
#define	ADJUST_X			((TABLE_WIDTH-BAR_WIDTH*GRID_MAX_POINTS_X-BAR_GAP_X*(GRID_MAX_POINTS_X-1))/2)
#define	BAR_HEIGTH		28
#define	BAR_GAP_Y			8
#define	Y_START				(BAR_HEIGTH-16)/2
#define	ADJUST_Y			((TABLE_HEIGTH - BAR_HEIGTH*GRID_MAX_POINTS_Y - BAR_GAP_Y*(GRID_MAX_POINTS_Y-1))/2)
#elif (GRID_MAX_POINTS_X == 6)
#define	FONT_G29TABLE	font6x12
#define	TABLE_TOP			150
#define	TABLE_HEIGTH	(TABLE_BOTTOM-TABLE_TOP)
#define	BAR_WIDTH 		36
#define	BAR_GAP_X 		5
#define	X_START				(BAR_WIDTH-5*6)/2
#define	ADJUST_X			((TABLE_WIDTH-BAR_WIDTH*GRID_MAX_POINTS_X-BAR_GAP_X*(GRID_MAX_POINTS_X-1))/2)
#define	BAR_HEIGTH		24
#define	BAR_GAP_Y			6
#define	Y_START				(BAR_HEIGTH-12)/2
#define	ADJUST_Y			((TABLE_HEIGTH - BAR_HEIGTH*GRID_MAX_POINTS_Y - BAR_GAP_Y*(GRID_MAX_POINTS_Y-1))/2)
#elif (GRID_MAX_POINTS_X == 7)
#define	FONT_G29TABLE	font6x12
#define	TABLE_TOP			150
#define	TABLE_HEIGTH	(TABLE_BOTTOM-TABLE_TOP)
#define	BAR_WIDTH 		32
#define	BAR_GAP_X 		3
#define	X_START				(BAR_WIDTH-5*6)/2
#define	ADJUST_X			((TABLE_WIDTH-BAR_WIDTH*GRID_MAX_POINTS_X-BAR_GAP_X*(GRID_MAX_POINTS_X-1))/2)
#define	BAR_HEIGTH		20
#define	BAR_GAP_Y			6
#define	Y_START				(BAR_HEIGTH-12)/2
#define	ADJUST_Y			((TABLE_HEIGTH - BAR_HEIGTH*GRID_MAX_POINTS_Y - BAR_GAP_Y*(GRID_MAX_POINTS_Y-1))/2)
#endif

inline void DWIN_G29_Show_ValueTable(bool bFrame, const uint8_t idx, const int16_t value){	
	uint16_t	x_start, y_start;	
	#if(GRID_MAX_POINTS_X == 3 || GRID_MAX_POINTS_X == 5 || GRID_MAX_POINTS_X == 7)
	//3,5,7
	x_start = TABLE_LEFT + ADJUST_X;	
	#else
	//4,6
	x_start = TABLE_RIGHT - BAR_WIDTH - ADJUST_X;
	#endif
	y_start = TABLE_BOTTOM - BAR_HEIGTH - ADJUST_Y;	
	LOOP_L_N(i, GRID_MAX_POINTS_Y){		
		LOOP_L_N(j, GRID_MAX_POINTS_X){			
			if(bFrame)
				dwinLCD.Draw_Rectangle(1, Color_Bg_DeepBlue, x_start, y_start, x_start+BAR_WIDTH, y_start+BAR_HEIGTH);			
			else if(idx == i*GRID_MAX_POINTS_Y+j+1){
				dwinLCD.Draw_Rectangle(1, Color_Bg_Black, x_start+1, y_start+1, x_start+BAR_WIDTH-1, y_start+BAR_HEIGTH-1);			
				if(ABS(value) > 999)
					dwinLCD.Draw_String(false, true, FONT_G29TABLE, Color_Red, Color_Bg_Black, x_start+X_START,  y_start+Y_START, F("Error"));
				else if(ABS(value) > 199)
					dwinLCD.Draw_SignedFloatValue(FONT_G29TABLE, Color_Red, Color_Bg_Black, 1, 2, x_start+X_START,  y_start+Y_START, value);
				else if(ABS(value) > 100)
					dwinLCD.Draw_SignedFloatValue(FONT_G29TABLE, Color_Yellow, Color_Bg_Black, 1, 2, x_start+X_START,  y_start+Y_START, value);	
				else if(ABS(value) > 50)
					dwinLCD.Draw_SignedFloatValue(FONT_G29TABLE, Color_White, Color_Bg_Black, 1, 2, x_start+X_START,  y_start+Y_START, value);
				else
					dwinLCD.Draw_SignedFloatValue(FONT_G29TABLE, Color_Green, Color_Bg_Black, 1, 2, x_start+X_START,  y_start+Y_START, value);
			}
			if(j<(GRID_MAX_POINTS_X-1)){
				#if((GRID_MAX_POINTS_X == 3 || GRID_MAX_POINTS_X == 5 || GRID_MAX_POINTS_X == 7))
				if(i%2 == 1) 
				#else
				if(i%2 == 0)
				#endif
					x_start -= (BAR_WIDTH+BAR_GAP_X);
				else 
					x_start += (BAR_WIDTH+BAR_GAP_X);			
			}
		}
		y_start -= (BAR_HEIGTH+BAR_GAP_Y);
	}	
}
#endif

void DWIN_G29_Show_Messge(const _emDWIN_G29_MSG message/*=G29_LEVLE_DEFAULT*/,const int pt_index,const int all_points,const float fvalue){
	switch(message){
		case G29_CATCH_START:
			Clear_Bottom_Area();
			DWIN_Draw_MaskString_Default(12, 454,F("To catch offset"));
			break;
		case G29_CATCH_NORMAL:
			Clear_Bottom_Area();
			DWIN_Draw_MaskString_Default(12, 454,F("Catch Point:  "));
			DWIN_Draw_IntValue_Default(1, 12+13*8, 454, pt_index);
			break;
		case G29_CATCH_FAIL1:
			Clear_Bottom_Area();
			DWIN_Draw_MaskString_Default_Color(Color_Red, 12, 454,F("Fail! Move down Probe"));
			break;
		case G29_CATCH_FAIL2:
			Clear_Bottom_Area();			
			DWIN_Draw_MaskString_Default_Color(Color_Red, 12, 454,F("Over range, manual level!"));
			break;
		case G29_CATCH_DONE:
			Clear_Bottom_Area();
			DWIN_Draw_MaskString_Default_Color(Color_Green,12, 454,F("Catched! Probe Z offset="));
			DWIN_Draw_Selected_Small_Float22(12+25*8, 454, (probe.offset.z * MAXUNITMULT));			
			break;			
		case G29_MESH_START:			
			Clear_Bottom_Area();
			DWIN_Draw_MaskString_Default(12, 454,F("Start Probing!"));
			break;
		case G29_MESH_READY:
			Clear_Main_Window();
			Draw_Popup_Bkgd_60();
		#if (GRID_MAX_POINTS_X <= 7)
			#if (GRID_MAX_POINTS_X > 4)
			DWIN_Show_ICON(ICON_AutoLeveling, 101, 80);
			#else
			DWIN_Show_ICON(ICON_AutoLeveling, 101, 105);
			#endif			
			DWIN_G29_Show_ValueTable(true, 0, 0);
		#endif
			break;
		case G29_MESH_PROBING:
			//Clear_Bottom_Area();
			DWIN_Draw_MaskString_Default(12, 454,F("Point:   /    "));			
			DWIN_Draw_IntValue_Default(2, 12+7*8, 454, pt_index);
			DWIN_Draw_IntValue_Default(2, 12+11*8, 454, all_points);
			break;
		case G29_MESH_VALUE:
			#if (GRID_MAX_POINTS_X <= 7)
			DWIN_G29_Show_ValueTable(false, pt_index, fvalue*MAXUNITMULT);
			#else
			DWIN_Draw_MaskString_Default_Color(Color_Red, 130, 454,F("Z offset="));
			dwinLCD.Draw_SignedFloatValue(font8x16, Color_Red, Color_Bg_Black, 2, 2, 130+9*8, 454, (fvalue * MAXUNITMULT));			
			#endif
			break;
		case G29_MESH_DONE:
			Clear_Bottom_Area();			
			DWIN_Draw_MaskString_Default_Color(Color_Green,12, 454,F("Bed Leveling finished!"));						
			dwinLCD.UpdateLCD();
			break;
		default:
			Clear_Bottom_Area();
			break;
	}
	dwinLCD.UpdateLCD();
}
#endif//#if (ABL_GRID)

inline void _Update_printing_Timer(){
	// print process
	if(card.isPrinting()){
		const uint8_t card_pct = card.percentDone();
		static uint8_t last_cardpercentValue = 101;
	
		if(last_cardpercentValue != card_pct) { // print percent
			last_cardpercentValue = card_pct;
			if(card_pct) {
				HMI_ValueStruct.Percentrecord = card_pct;
				if(DwinMenuID == DWMENU_PRINTING) Draw_Print_ProgressBar();
			}
		}
		duration_t elapsed = print_job_timer.duration(); // print timer

		// Print time so far
		static uint16_t last_Printtime = 0;
		const uint16_t min = (elapsed.value % 3600) / 60;
		if (last_Printtime != min) { // 1 minute update
			last_Printtime = min;
			if(DwinMenuID == DWMENU_PRINTING) Draw_Print_ProgressElapsed();
		}

		// Estimate remaining time every 20 seconds
		const millis_t ms = millis();	
		static millis_t next_remain_time_update = 0;
		if(HMI_ValueStruct.Percentrecord >= 1 && ELAPSED(ms, next_remain_time_update) && !HMI_flag.heat_flag) {
			 HMI_ValueStruct.remain_time = (((elapsed.value - HMI_ValueStruct.dwin_heat_time) * 100) / HMI_ValueStruct.Percentrecord) - (elapsed.value - HMI_ValueStruct.dwin_heat_time);
			 next_remain_time_update += 20 * 1000UL;
			 if(DwinMenuID == DWMENU_PRINTING) Draw_Print_ProgressRemain();
		}
	}
}

void DWIN_ResumedFromPause(){
	HMI_flag.clean_status_delay = 0;
	Clear_Bottom_Area();
	DWIN_status = ID_SM_PRINTING;
	Start_PrintProcess();
}

#if ENABLED(DEBUG_DWIN_LCD)
void DWIN_Show_Status(){
	switch(DWIN_status){
		case ID_SM_START:
			SERIAL_ECHOLNPGM("  DWIN_status = ID_SM_START");
		break;
		default:
		case ID_SM_IDEL:
			SERIAL_ECHOLNPGM("  DWIN_status = ID_SM_IDEL");
		break;
		case ID_SM_PRINTING:
			SERIAL_ECHOLNPGM("  DWIN_status = ID_SM_PRINTING");
		break;
		case ID_SM_PAUSING:
			SERIAL_ECHOLNPGM("  DWIN_status = ID_SM_PAUSING");
		break;
		case ID_SM_PAUSED:
			SERIAL_ECHOLNPGM("  DWIN_status = ID_SM_PAUSED");
		break;
		case ID_SM_RESUMING:
			SERIAL_ECHOLNPGM("  DWIN_status = ID_SM_RESUMING");
		break;
		case ID_SM_STOPED:
			SERIAL_ECHOLNPGM("  DWIN_status = ID_SM_STOPED");
		break;		
		case ID_SM_RUNOUT_DONE:
			SERIAL_ECHOLNPGM("  DWIN_status = ID_SM_RUNOUT_DONE");
		break;			
	}
}
#endif

void EachMomentUpdate() {	
	
	static millis_t next_rts_update_ms = 0;
	const millis_t ms = millis();	
	if (PENDING(ms, next_rts_update_ms)) 	return;
	next_rts_update_ms = ms + DWIN_SCROLL_UPDATE_INTERVAL;

	TOGGLE(LED_PIN);
	
	#if ENABLED(DEBUG_DWIN_LCD)
	SERIAL_ECHOLNPAIR("DwinMenuID = ", DwinMenuID);
	DWIN_Show_Status();
	#endif	

	//check repeat printing
 	TERN_(OPTION_REPEAT_PRINTING, _check_repeatPrint());
	
	//check auto power off
	TERN_(OPTION_AUTOPOWEROFF, _check_autoshutdown());
	
	// variable update
 	update_variable();
	
	if(DWIN_status == ID_SM_START){
		//check resume print when power on
		DWIN_status = ID_SM_IDEL;
		TERN_(POWER_LOSS_RECOVERY, _check_Powerloss_resume());
	}
	else if(DWIN_status == ID_SM_IDEL){	  
		//check WiFi
		TERN_(OPTION_WIFI_MODULE, _check_wifi());				
		TERN_(POWER_LOSS_RECOVERY, _check_Powerloss_resume());
	}
	else if(DWIN_status == ID_SM_PRINTING){
		TERN_(POWER_LOSS_RECOVERY,recovery.save(false));
		_Update_printing_Timer();						 
	}	
	else if(DWIN_status == ID_SM_RESUMING || DWIN_status == ID_SM_PAUSING){
		if(HMI_flag.clean_status_delay > 0){
			HMI_flag.clean_status_delay--;
			if(HMI_flag.clean_status_delay == 0){ 
				HMI_flag.killtimes = 0;
				Clear_Bottom_Area();
			}
		}
		else if(DWIN_status == ID_SM_RESUMING && printingIsActive() && wait_for_heatup == false){
			DWIN_status = ID_SM_PRINTING;
			Start_PrintProcess();
		}		
		else if(DWIN_status == ID_SM_PAUSING && printingIsPaused() && !planner.has_blocks_queued() && wait_for_heatup == false){
			DWIN_status = ID_SM_PAUSED;
			Start_PrintProcess();
		}
	}
	else if(DWIN_status == ID_SM_PAUSED){
		HMI_flag.killtimes = 0;
		if(!printingIsPaused()){
				DWIN_status = ID_SM_PRINTING;
				Start_PrintProcess();
		}
	}
	else if(DWIN_status == ID_SM_STOPED){		
		HMI_ValueStruct.Percentrecord = 0;
		HMI_ValueStruct.remain_time = 0;		
		#if ENABLED(OPTION_REPEAT_PRINTING)
		 if(ReprintManager.enabled && (current_position.z >= 20)){
			 planner.synchronize();
			 Popup_Window_BTempCooldown();
			 ReprintManager.Reprint_goon():
			 return;
		 }
		#endif
		TERN_(POWER_LOSS_RECOVERY, recovery.cancel());
		TERN_(OPTION_AUTOPOWEROFF, _setAutoPowerDown());
		mixer.reset_vtools();
		HMI_ValueStruct.print_speed = feedrate_percentage = 100;
		DWIN_status = ID_SM_IDEL;		
		select_main.reset();
		Draw_Main_Menu();		
	}		
#if ENABLED(FILAMENT_RUNOUT_SENSOR)
	else if(DWIN_status == ID_SM_RUNOUT_DONE){
		DWIN_status = ID_SM_PRINTING;				
		Start_PrintProcess();
	}
#endif
	dwinLCD.UpdateLCD();
}

void DWIN_Update() {
	if(HMI_flag.auto_test_flag == 0xaa){
		if(autotest.DWIN_AutoTesting()){
			HMI_flag.auto_test_flag = 0x55;
			Draw_Main_Menu();
		}
	}
	else{				
		EachMomentUpdate();  // Status update		
		DWIN_HandleScreen(); // Rotary encoder update
		HMI_SDCardUpdate();	 // SD card update
	}
}
#endif // HAS_DWIN_LCD
