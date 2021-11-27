/*
 * lcd.cpp
 *
 *  Created on: Nov 17, 2021
 *      Author: Nick Nagy
 */

#include <magna_lcd.h>
#include <cmath>

//Functions defines Macros
#define swap(a, b) { int16_t t = a; a = b; b = t; }
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define min(a,b) (((a)<(b))?(a):(b))

magna::ILI9341::ILI9341(magna::ILI9341InitStruct& initStruct) : LCD(ILI9341_WIDTH, ILI9341_HEIGHT) {
	this->CSPort = initStruct.CSPort;
	this->NWEPort = initStruct.NWEPort;
	this->RSPort = initStruct.RSPort;
	this->ResetPort = initStruct.ResetPort;
	this->CSPin = initStruct.CSPin;
	this->NWEPin = initStruct.NWEPin;
	this->RSPin = initStruct.RSPin;
	this->ResetPin = initStruct.ResetPin;
	this->baseAddress = initStruct.MemSwap ? (__IO uint8_t *)0xC0000000 : (__IO uint8_t *)0x60000000;
	this->dataAddress = (__IO uint8_t *)0x60040000; // TODO

	init();
}

void magna::ILI9341::sendCommand(uint8_t command) {
	HAL_GPIO_WritePin(RSPort, RSPin, GPIO_PIN_RESET);
	*baseAddress = command;
	HAL_GPIO_WritePin(NWEPort, NWEPin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(NWEPort, NWEPin, GPIO_PIN_SET);
}

void magna::ILI9341::sendData(uint8_t data) {
	HAL_GPIO_WritePin(RSPort, RSPin, GPIO_PIN_SET);
	*dataAddress = data;
	HAL_GPIO_WritePin(NWEPort, NWEPin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(NWEPort, NWEPin, GPIO_PIN_SET);
}

void magna::ILI9341::writeRegister16(uint8_t reg, uint16_t data) {
	sendCommand(reg);
	sendData(data >> 8);
	sendData(data);
}

void magna::ILI9341::writeRegister32(uint8_t reg, uint32_t data) {
	sendCommand(reg);
	sendData(data >> 24);
	sendData(data >> 16);
	sendData(data >> 8);
	sendData(data);
}

void magna::ILI9341::setCursorPosition(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
	uint32_t t;
	t = (x0 << 16) | x1;
	writeRegister32(ILI9341_COLUMN_ADDR, t);
	t = (y0 << 16) | y1;
	writeRegister32(ILI9341_PAGE_ADDR, t);
	sendCommand (ILI9341_GRAM);
}

void magna::ILI9341::invertRows(uint16_t y0, uint16_t y1) {
	/* define partial area */
	uint32_t t;
	t = (y0 << 16) | y1;
	writeRegister32(ILI9341_PARTIAL_AREA, t);
	/* turn on partial mode */
	sendCommand(ILI9341_PARTIAL_MODE_ON);
	/* invert area */
	sendCommand(ILI9341_DISPLAY_INVERSION_ON);
}

void magna::ILI9341::init() {
	 // Reset display:
	 HAL_GPIO_WritePin(ResetPort, ResetPin, GPIO_PIN_RESET);
	 HAL_Delay(10);
	 HAL_GPIO_WritePin(ResetPort, ResetPin, GPIO_PIN_SET);

	 sendCommand (ILI9341_RESET); // software reset comand
	 HAL_Delay(100);
	 sendCommand (ILI9341_DISPLAY_OFF); // display off
	 //------------power control------------------------------
	 sendCommand (ILI9341_POWER1); // power control
	 sendData   (0x26); // GVDD = 4.75v
	 sendCommand (ILI9341_POWER2); // power control
	 sendData   (0x11); // AVDD=VCIx2, VGH=VCIx7, VGL=-VCIx3
	 //--------------VCOM-------------------------------------
	 sendCommand (ILI9341_VCOM1); // vcom control
	 sendData   (0x35); // Set the VCOMH voltage (0x35 = 4.025v)
	 sendData   (0x3e); // Set the VCOML voltage (0x3E = -0.950v)
	 sendCommand (ILI9341_VCOM2); // vcom control
	 sendData   (0xbe);

	 //------------memory access control------------------------
	 sendCommand (ILI9341_MAC); // memory access control
	 sendData(0x48);

	 sendCommand (ILI9341_PIXEL_FORMAT); // pixel format set
	 sendData   (0x55); // 16bit /pixel

	 sendCommand(ILI9341_FRC);
	 sendData(0);
	 sendData(0x1F);
	 //-------------ddram ----------------------------
	 sendCommand (ILI9341_COLUMN_ADDR); // column set
	 sendData   (0x00); // x0_HIGH---0
	 sendData   (0x00); // x0_LOW----0
	 sendData   (0x00); // x1_HIGH---240
	 sendData   (0xEF); // x1_LOW----240
	 sendCommand (ILI9341_PAGE_ADDR); // page address set
	 sendData   (0x00); // y0_HIGH---0
	 sendData   (0x00); // y0_LOW----0
	 sendData   (0x01); // y1_HIGH---320
	 sendData   (0x3F); // y1_LOW----320

	 sendCommand (ILI9341_TEARING_OFF); // tearing effect off
	 //LCD_write_cmd(ILI9341_TEARING_ON); // tearing effect on
	 //LCD_write_cmd(ILI9341_DISPLAY_INVERSION); // display inversion
	 sendCommand (ILI9341_Entry_Mode_Set); // entry mode set
	 // Deep Standby Mode: OFF
	 // Set the output level of gate driver G1-G320: Normal display
	 // Low voltage detection: Disable
	 sendData   (0x07);
	 //-----------------display------------------------
	 sendCommand (ILI9341_DFC); // display function control
	 //Set the scan mode in non-display area
	 //Determine source/VCOM output in a non-display area in the partial display mode
	 sendData   (0x0a);
	 //Select whether the liquid crystal type is normally white type or normally black type
	 //Sets the direction of scan by the gate driver in the range determined by SCN and NL
	 //Select the shift direction of outputs from the source driver
	 //Sets the gate driver pin arrangement in combination with the GS bit to select the optimal scan mode for the module
	 //Specify the scan cycle interval of gate driver in non-display area when PTG to select interval scan
	 sendData   (0x82);
	 // Sets the number of lines to drive the LCD at an interval of 8 lines
	 sendData   (0x27);
	 sendData   (0x00); // clock divisor

	 sendCommand (ILI9341_SLEEP_OUT); // sleep out
	 HAL_Delay(100);
	 sendCommand (ILI9341_DISPLAY_ON); // display on
	 HAL_Delay(100);
	 sendCommand (ILI9341_GRAM); // memory write
	 HAL_Delay(5);
}

void magna::ILI9341::drawPixel(uint16_t x, uint16_t y, uint16_t color) {
	setCursorPosition(x, y, x, y);
	sendData(color >> 8);
	sendData(color & 0xFF);
}

void magna::ILI9341::fill(uint16_t color) {
	uint32_t n = ILI9341_PIXEL_COUNT;
	if(rotation==1 || rotation==3)
	{
		setCursorPosition(0, 0,   ILI9341_WIDTH -1, ILI9341_HEIGHT -1);
	}
	else if(rotation==2 || rotation==4)
	{
		setCursorPosition(0, 0, ILI9341_HEIGHT -1, ILI9341_WIDTH -1);
	}
	while (n) {
	   n--;
       sendData(color>>8);
	   sendData(color & 0xFF);
	}
}

/* assumes x1>x0 and y1>y0 */
void magna::ILI9341::fillRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color) {
	uint32_t n = ((x1+1)-x0)*((y1+1)-y0);
	if (n>ILI9341_PIXEL_COUNT)
		n=ILI9341_PIXEL_COUNT;
	setCursorPosition(x0, y0, x1, y1);
	while (n) {
		n--;
		sendData(color>>8);
		sendData(color&0xff);
	}
}

void magna::ILI9341::drawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	drawPixel(x0  , y0+r, color);
	drawPixel(x0  , y0-r, color);
	drawPixel(x0+r, y0  , color);
	drawPixel(x0-r, y0  , color);

	while (x<y) {
	  if (f >= 0) {
	    y--;
	    ddF_y += 2;
	    f += ddF_y;
	  }
	  x++;
	  ddF_x += 2;
	  f += ddF_x;

	  drawPixel(x0 + x, y0 + y, color);
	  drawPixel(x0 - x, y0 + y, color);
	  drawPixel(x0 + x, y0 - y, color);
	  drawPixel(x0 - x, y0 - y, color);
	  drawPixel(x0 + y, y0 + x, color);
	  drawPixel(x0 - y, y0 + x, color);
	  drawPixel(x0 + y, y0 - x, color);
	  drawPixel(x0 - y, y0 - x, color);
	}
}

void magna::ILI9341::drawCircleHelper(uint16_t x0, uint16_t y0, uint16_t r, uint8_t cornername, uint16_t color) {
	int16_t f     = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x     = 0;
	int16_t y     = r;
	while (x<y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f     += ddF_y;
		}
		x++;
		ddF_x += 2;
		f     += ddF_x;
		if (cornername & 0x4) {
			drawPixel(x0 + x, y0 + y, color);
			drawPixel(x0 + y, y0 + x, color);
		}
		if (cornername & 0x2) {
			drawPixel(x0 + x, y0 - y, color);
			drawPixel(x0 + y, y0 - x, color);
		}
		if (cornername & 0x8) {
			drawPixel(x0 - y, y0 + x, color);
			drawPixel(x0 - x, y0 + y, color);
		}
		if (cornername & 0x1) {
			drawPixel(x0 - y, y0 - x, color);
			drawPixel(x0 - x, y0 - y, color);
		}
	}
}

void magna::ILI9341::fillCircleHelper(uint16_t x0, uint16_t y0, uint16_t r, uint8_t cornername, uint16_t delta, uint16_t color) {
	int16_t f     = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x     = 0;
	int16_t y     = r;
	while (x<y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f     += ddF_y;
	    }
	    x++;
	    ddF_x += 2;
	    f     += ddF_x;
	    if (cornername & 0x1) {
	    	drawFastVLine(x0+x, y0-y, 2*y+1+delta, color);
	    	drawFastVLine(x0+y, y0-x, 2*x+1+delta, color);
	    }
	    if (cornername & 0x2) {
	    	drawFastVLine(x0-x, y0-y, 2*y+1+delta, color);
	    	drawFastVLine(x0-y, y0-x, 2*x+1+delta, color);
	    }
	}
}

void magna::ILI9341::fillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
	drawFastVLine(x0, y0-r, 2*r+1, color);
	fillCircleHelper(x0, y0, r, 3, 0, color);
}

void magna::ILI9341::drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color) {
	int16_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
	  swap(x0, y0);
	  swap(x1, y1);
	}
	if (x0 > x1) {
	  swap(x0, x1);
	  swap(y0, y1);
	}
	uint16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);
	int16_t err = dx>>1;
	int16_t ystep;
	if (y0 < y1) {
	  ystep = 1;
	} else {
	  ystep = -1;
	}
	for (; x0<=x1; x0++) {
	  if (steep) {
	    drawPixel(y0, x0, color);
	  } else {
	    drawPixel(x0, y0, color);
	  }
	  err -= dy;
	  if (err < 0) {
	    y0 += ystep;
	    err += dx;
	  }
	}
}

void magna::ILI9341::drawFastHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color) {
	drawLine(x, y, x+w-1, y, color);
}

void magna::ILI9341::drawFastVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color) {
	drawLine(x, y, x, y+h-1, color);
}

void magna::ILI9341::drawRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color) {
	drawFastHLine(x0, y0, x1-x0, color);
	drawFastVLine(x0, y0, y1-y0, color);
	drawFastHLine(x0, y1, x1-x0, color);
	drawFastVLine(x1, y0, y1-y0, color);
}

void magna::ILI9341::drawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
	drawLine(x0, y0, x1, y1, color);
	drawLine(x1, y1, x2, y2, color);
	drawLine(x2, y2, x0, y0, color);
}

/* left params as signed ints just to reaffirm logic inside function */
void magna::ILI9341::fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
	int16_t a, b, y, last;

	// Sort coordinates by Y order (y2 >= y1 >= y0)
	if (y0 > y1) {
		swap(y0, y1); swap(x0, x1);
	}
	if (y1 > y2) {
		swap(y2, y1); swap(x2, x1);
	}
	if (y0 > y1) {
		swap(y0, y1); swap(x0, x1);
	}

	if(y0 == y2) { // Handle awkward all-on-same-line case as its own thing
		a = b = x0;
		if(x1 < a)
			a = x1;
		else if(x1 > b)
			b = x1;
		if(x2 < a)
			a = x2;
		else if(x2 > b)
			b = x2;
		drawFastHLine(a, y0, b-a+1, color);
		return;
	}

	int16_t
    	dx01 = x1 - x0,
		dy01 = y1 - y0,
		dx02 = x2 - x0,
		dy02 = y2 - y0,
		dx12 = x2 - x1,
		dy12 = y2 - y1,
		sa   = 0,
		sb   = 0;

	// For upper part of triangle, find scanline crossings for segments
	// 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
	// is included here (and second loop will be skipped, avoiding a /0
	// error there), otherwise scanline y1 is skipped here and handled
	// in the second loop...which also avoids a /0 error here if y0=y1
	// (flat-topped triangle).
	if(y1 == y2) last = y1;   // Include y1 scanline
	else         last = y1-1; // Skip it

	for(y=y0; y<=last; y++) {
		a   = x0 + sa / dy01;
		b   = x0 + sb / dy02;
		sa += dx01;
		sb += dx02;

		if(a > b) swap(a,b);
		drawFastHLine(a, y, b-a+1, color);
	}

	// For lower part of triangle, find scanline crossings for segments
	// 0-2 and 1-2.  This loop is skipped if y1=y2.
	sa = dx12 * (y - y1);
	sb = dx02 * (y - y0);
	for(; y<=y2; y++) {
		a   = x1 + sa / dy12;
		b   = x0 + sb / dy02;
		sa += dx12;
		sb += dx02;

		if(a > b) swap(a,b);
    	drawFastHLine(a, y, b-a+1, color);
	}
}

void magna::ILI9341::drawChar(uint16_t x, uint16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size) {
	if((x >= ILI9341_WIDTH)            || // Clip right
	 (y >= ILI9341_HEIGHT)           || // Clip bottom
     ((x + 6 * size - 1) < 0) || // Clip left
     ((y + 8 * size - 1) < 0))   // Clip top
    return;

    if(!_cp437 && (c >= 176)) c++; // Handle 'classic' charset behavior

    for (int8_t i=0; i<6; i++ ) {
    	uint8_t line;
    	if (i == 5)
    		line = 0x0;
    	else
    		line = pgm_read_byte(font1+(c*5)+i);
    	for (int8_t j = 0; j<8; j++) {
    		if (line & 0x1) {
    			if (size == 1) // default size
    				drawPixel(x+i, y+j, color);
    			else {  // big size
    				fillRect(x+(i*size), y+(j*size), size + x+(i*size), size+1 + y+(j*size), color);
    			}
    		} else if (bg != color) {
    			if (size == 1) // default size
    				drawPixel(x+i, y+j, bg);
    			else {  // big size
    				fillRect(x+i*size, y+j*size, size + x+i*size, size+1 + y+j*size, bg);
    			}
    		}
    		line >>= 1;
    	}
    }
}

void magna::ILI9341::printText(std::string text, uint16_t x, uint16_t y, uint16_t color, uint16_t bg, uint8_t size) {
	int16_t offset;
	offset = size*6;
	for(uint16_t i=0; i<40 && text[i]!=NULL; i++)
	{
		drawChar(x+(offset*i), y, text[i],color,bg,size);
	}
}

void magna::ILI9341::printImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data, uint32_t size) {
	uint32_t n = size;
	setCursorPosition(x, y, w+x-1, h+y-1);
	for(uint32_t i=0; i<n ; i++)
	{
		sendData(data[i]);
	}
}

void magna::ILI9341::setRotation(uint8_t rotate) {
	switch(rotate) {
		case 1:
			rotation = 1;
			sendCommand(ILI9341_MEMCONTROL);
			sendData(ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR);
			break;
		case 2:
			rotation = 2;
			sendCommand(ILI9341_MEMCONTROL);
			sendData(ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR);
			break;
		case 3:
			rotation = 3;
			sendCommand(ILI9341_MEMCONTROL);
			sendData(ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR);
			break;
		case 4:
			rotation = 4;
			sendCommand(ILI9341_MEMCONTROL);
			sendData(ILI9341_MADCTL_MX | ILI9341_MADCTL_MY | ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR);
			break;
		default:
			rotation = 1;
			sendCommand(ILI9341_MEMCONTROL);
			sendData(ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR);
			break;
	}
}

#define DEBUG 0

/**
 * @param x: center x value of arc
 * @param y: center y value of arc
 * NOTE - angles are in radians
 */
void magna::ILI9341::drawArc(uint16_t x, uint16_t y, uint16_t radius, float startAngle, float endAngle, uint16_t color) {
	for (float i = startAngle; i < endAngle; i += 0.05f) {
#if DEBUG
		uint16_t xIdx = (uint16_t)(x + radius*cos(i))
#endif
		drawPixel((uint16_t)(x + radius*cos(i)), (uint16_t)(y - radius*sin(i)), color);
	}
}

/*void magna::ILI9341::fillArc(uint16_t x, uint16_t y, uint16_t minRadius, uint16_t maxRadius, float startAngle, float endAngle, uint16_t color) {
	//for (uint16_t r = minRadius; r <= maxRadius; r++) {
	//	drawArc(x, y, r, startAngle, endAngle, color);
	//}
}*/

#define degToRad(deg) (deg)*0.0174532925f
void magna::ILI9341::fillArc(uint16_t x, uint16_t y, int16_t startAngle, int16_t endAngle, uint16_t rx, uint16_t ry, uint16_t thickness, uint16_t color) {
	// source: https://forum.arduino.cc/t/adafruit_gfx-fillarc/397741/3
	uint8_t seg = 3; // segments are 3 degrees wide
	uint8_t inc = 3; // draw segments every 3 degrees

	uint16_t totalSegments = (endAngle-startAngle)/seg;

	for (int i = startAngle; i < startAngle + totalSegments*seg; i+=inc) {
		// calculate segment start coordinates
		float sx0 = cos(degToRad(i-90));
		float sy0 = sin(degToRad(i-90));
		uint16_t x0 = sx0*(rx-thickness) + x;
		uint16_t y0 = sy0*(ry-thickness) + y;
		uint16_t x1 = sx0*rx + x;
		uint16_t y1 = sy0*ry + y;

		// calculate segment end coorinates
		float sx1 = cos(degToRad(i+seg-90));
		float sy1 = sin(degToRad(i+seg-90));
		int16_t x2 = sx1*(rx-thickness) + x;
		int16_t y2 = sy1*(ry-thickness) + y;
		int16_t x3 = sx1*rx + x;
		int16_t y3 = sy1*ry + y;

		fillTriangle(x0, y0, x1, y1, x2, y2, color);
		fillTriangle(x1, y1, x2, y2, x3, y3, color);
	}
}
