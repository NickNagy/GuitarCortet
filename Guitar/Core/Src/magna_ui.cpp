/*
 * magna_ui.cpp
 *
 *  Created on: Nov 17, 2021
 *      Author: Nick Nagy
 */

#include "magna_ui.h"

void magna::UITextBox::draw() {
	display.drawRect(x, y, x + width, y + height, borderColor);
	display.fillRect(x, y, x + width, y + width, backgroundColor);
	display.printText(text, x + textXOffset, y + textYOffset, textColor, textBackgroundColor, textSize);
}

void magna::UIDial::draw() {

}
