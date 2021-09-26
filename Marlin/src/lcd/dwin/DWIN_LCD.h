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
#if HAS_DWIN_LCD
/********************************************************************************
 * @file     dwin_lcd.h
 * @author   LEO / Creality3D
 * @date     2019/07/18
 * @version  2.0.1
 * @brief    迪文屏控制操作函数
 ********************************************************************************/

#include <stdint.h>
#include <WString.h>


#define RECEIVED_NO_DATA         0x00
#define RECEIVED_SHAKE_HAND_ACK  0x01

#define FHONE                    0xAA

#define DWIN_SCROLL_UP   					2
#define DWIN_SCROLL_DOWN 					3

#define DWIN_WIDTH  272
#define DWIN_HEIGHT 480

#define ICON_ITEM_NUMBER  160
#define LANG_NUMBER  			5

class DWINLCD{
	public:					
		/*-------------------------------------- System variable function --------------------------------------*/
		// Handshake (1: Success, 0: Fail)
		static bool Handshake(void);
		// Common DWIN startup
		static void Startup(void);
		// Set the backlight luminance
		//  luminance: (0x00-0xFF)
		static void Set_Backlight_Luminance(const uint8_t luminance);
		// Set screen display direction
		//  dir: 0=0°, 1=90°, 2=180°, 3=270°
		static void Frame_SetDir(uint8_t dir);
		// Update display
		static void UpdateLCD(void);		
		/*---------------------------------------- Drawing functions ----------------------------------------*/
		// Clear screen
		//	color: Clear screen color
		static void Frame_Clear(const uint16_t color);
		
		// Draw a point
		//	width: point width	 0x01-0x0F
		//	height: point height 0x01-0x0F
		//	x,y: upper left point
		static void Draw_Point(uint8_t width, uint8_t height, uint16_t x, uint16_t y);
		
		// Draw a line
		//	color: Line segment color
		//	xStart/yStart: Start point
		//	xEnd/yEnd: End point
		static void Draw_Line(uint16_t color, uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);
		
		// Draw a Horizontal line
		//	color: Line segment color
		//	xStart/yStart: Start point
		//	xLength: Line Length
		static void Draw_HLine(uint16_t color, uint16_t xStart, uint16_t yStart, uint16_t xLength) {
			Draw_Line(color, xStart, yStart, xStart + xLength - 1, yStart);
		}
		
		// Draw a Vertical line
		//	color: Line segment color
		//	xStart/yStart: Start point
		//	yLength: Line Length
		static void Draw_VLine(uint16_t color, uint16_t xStart, uint16_t yStart, uint16_t yLength) {
			Draw_Line(color, xStart, yStart, xStart, yStart + yLength - 1);
		}
		
		// Draw a rectangle
		//	mode: 0=frame, 1=fill, 2=XOR fill
		//	color: Rectangle color
		//	xStart/yStart: upper left point
		//	xEnd/yEnd: lower right point
		static void Draw_Rectangle(uint8_t mode, uint16_t color,
														 uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);
		
		// Draw a box
		//	mode: 0=frame, 1=fill, 2=XOR fill
		//	color: Rectangle color
		//	xStart/yStart: upper left point
		//	xSize/ySize: box size
		static void Draw_Box(uint8_t mode, uint16_t color, uint16_t xStart, uint16_t yStart, uint16_t xSize, uint16_t ySize) {
			Draw_Rectangle(mode, color, xStart, yStart, xStart + xSize - 1, yStart + ySize - 1);
		}
		
		// Move a screen area
		//	mode: 0, circle shift; 1, translation
		//	dir: 0=left, 1=right, 2=up, 3=down
		//	dis: Distance
		//	color: Fill color
		//	xStart/yStart: upper left point
		//	xEnd/yEnd: bottom right point
		static void Frame_AreaMove(uint8_t mode, uint8_t dir, uint16_t dis,
														 uint16_t color, uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);
		
		/*---------------------------------------- Text related functions ----------------------------------------*/
		
		// Draw a string
		//	widthAdjust: true=self-adjust character width; false=no adjustment
		//	bShow: true=display background color; false=don't display background color
		//	size: Font size
		//	color: Character color
		//	bColor: Background color
		//	x/y: Upper-left coordinate of the string
		//	*string: The string
		static void Draw_String(bool widthAdjust, bool bShow, uint8_t size,
													uint16_t color, uint16_t bColor, uint16_t x, uint16_t y, char *string);
		
		
		static void Draw_String(bool widthAdjust, bool bShow, uint8_t size, uint16_t color, uint16_t bColor, uint16_t x, uint16_t y, const __FlashStringHelper *title) {
			Draw_String(widthAdjust, bShow, size, color, bColor, x, y, (char *)title);
		}
		
		// Draw a positive integer
		//	bShow: true=display background color; false=don't display background color
		//	zeroFill: true=zero fill; false=no zero fill
		//	zeroMode: 1=leading 0 displayed as 0; 0=leading 0 displayed as a space
		//	size: Font size
		//	color: Character color
		//	bColor: Background color
		//	iNum: Number of digits
		//	x/y: Upper-left coordinate
		//	value: Integer value
		static void Draw_IntValue(uint8_t bShow, bool zeroFill, uint8_t zeroMode, uint8_t size, uint16_t color,
															uint16_t bColor, uint8_t iNum, uint16_t x, uint16_t y, uint16_t value);
		
		// Draw a floating point number
		//	bShow: true=display background color; false=don't display background color
		//	zeroFill: true=zero fill; false=no zero fill
		//	zeroMode: 1=leading 0 displayed as 0; 0=leading 0 displayed as a space
		//	size: Font size
		//	color: Character color
		//	bColor: Background color
		//	iNum: Number of whole digits
		//	fNum: Number of decimal digits
		//	x/y: Upper-left point
		//	value: Float value
		static void Draw_FloatValue(uint8_t bShow, bool zeroFill, uint8_t zeroMode, uint8_t size, uint16_t color,
																uint16_t bColor, uint8_t iNum, uint8_t fNum, uint16_t x, uint16_t y, long value);

		static void Draw_SignedFloatValue(uint8_t size, uint16_t Color, 
																uint16_t bColor, uint8_t iNum, uint8_t fNum, uint16_t x, uint16_t y, long value);		
		
		/*---------------------------------------- Picture related functions ----------------------------------------*/		
		// Draw JPG and cached in #0 virtual display area
		// id: Picture ID
		static void JPG_ShowAndCache(const uint8_t id);
		
		// Draw an Icon
		//	libID: Icon library ID
		//	picID: Icon ID
		//	x/y: Upper-left point
		static void ICON_Show(uint8_t libID, uint8_t picID, uint16_t x, uint16_t y);
		
		// Unzip the JPG picture to a virtual display area
		//	n: Cache index
		//	id: Picture ID
		static void JPG_CacheToN(uint8_t n, uint8_t id);
		
		// Unzip the JPG picture to virtual display area #1
		//	id: Picture ID
		static void JPG_CacheTo1(uint8_t id) { JPG_CacheToN(1, id); }		
		// Copy area from virtual display area to current screen
		//	cacheID: virtual area number
		//	xStart/yStart: Upper-left of virtual area
		//	xEnd/yEnd: Lower-right of virtual area
		//	x/y: Screen paste point
		static void Frame_AreaCopy(uint8_t cacheID, uint16_t xStart, uint16_t yStart,
														 uint16_t xEnd, uint16_t yEnd, uint16_t x, uint16_t y);
		
		//void Frame_AreaCopy_Index(uint8_t PixtureID, uint8_t cacheID, uint16_t xStart, uint16_t yStart,
														// uint16_t xEnd, uint16_t yEnd, uint16_t x, uint16_t y);
		static void Frame_AreaCopy_Index(uint8_t cacheID, uint8_t PixtureID, uint8_t ItemID, uint16_t array[LANG_NUMBER][ICON_ITEM_NUMBER][4], uint16_t x, uint16_t y);				
		// Animate a series of icons
		//	animID: Animation ID	up to 16
		//	animate: animation on or off
		//	libID: Icon library ID
		//	picIDs: Icon starting ID
		//	picIDe: Icon ending ID
		//	x/y: Upper-left point
		//	interval: Display time interval, unit 10mS
		static void ICON_Animation(uint8_t animID, bool animate, uint8_t libID, uint8_t picIDs,
														 uint8_t picIDe, uint16_t x, uint16_t y, uint16_t interval);		
		// Animation Control
		//	state: 16 bits, each bit is the state of an animation id
		static void ICON_AnimationControl(uint16_t state);		
private:				
};

extern DWINLCD dwinLCD;
#endif
