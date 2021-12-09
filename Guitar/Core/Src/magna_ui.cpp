/*
 * magna_ui.cpp
 *
 *  Created on: Nov 17, 2021
 *      Author: Nick Nagy
 */

#include "magna_ui.h"

//
//
// UITEXTBOX
//
//

void magna::UITextBox::draw(magna::LCD& display) {
	display.drawRect(x, y, x + width, y + height, borderColor);
	display.fillRect(x, y, x + width, y + height, backgroundColor);
	display.printText(text, x + textXOffset, y + textYOffset, textColor, textBackgroundColor, textSize);
}

//
//
// UIDIAL
//
//

void magna::UIDial::draw(magna::LCD& display) {
	display.fillRect(x, y, x + width, y + height, borderColor);
	display.fillRect(x + 1, y + 1, x + width - 2, y + height - 2, backgroundColor);
	refresh(display);
}

void magna::UIDial::refresh(magna::LCD& display) {
	uint16_t xc, yc, rx, ry;
	int16_t startAngle, endAngle;
	endAngle = 225;
	rx = width/2;
	ry = height/2;
	// make sure dial arc is circular (rx == ry)
	if (rx < ry) {
		ry = rx;
	} else if (ry < rx) {
		rx = ry;
	}
	xc = x + rx;
	yc = y + ry;
	startAngle = endAngle - (int16_t)(270*(currentValue/(maxValue - minValue)));
	// if angle is larger than previous angle, color out arc with background color
	// (larger magnitude starting angle equates to smaller arc)
	if (lastStartAngle < startAngle) {
		display.fillArc(xc, yc, lastStartAngle, startAngle, rx, ry, dialArcThickness, backgroundColor);
	// o/w append to arc
	} else if (lastStartAngle > startAngle) {
		display.fillArc(xc, yc, startAngle, lastStartAngle, rx, ry, dialArcThickness, dialArcColor);
	}
	lastStartAngle = startAngle;
}

//
//
// EFFECTS USER INTERFACE
//
//

void magna::EffectUserInterface::addDial(std::string stringId, float minValue, float maxValue) {
	uint16_t dialSpaceHeight = height/2; // TODO
	uint16_t dialSpaceY = height - height/2;
	std::shared_ptr<magna::UIDial> dial = std::make_shared<magna::UIDial>(*this, dialStyleSheet, stringId, minValue, maxValue);
	fxDials.push_back(dial);
#define DEBUG_ADD_DIAL 1
#if DEBUG_ADD_DIAL
	int numDials = fxDials.size();
	switch(numDials) {
#else
	switch(fxDials.size()) {
#endif
	case 1:
		fxDials.at(0)->updatePosition(0, dialSpaceY, width, dialSpaceHeight);
		break;
	case 2:
		fxDials.at(0)->updatePosition(0, dialSpaceY, width/2, dialSpaceHeight);
		fxDials.at(1)->updatePosition(width/2, dialSpaceY, width/2, dialSpaceHeight);
		break;
	case 3:
		for (int i = 0; i < 3; i++) {
			fxDials.at(i)->updatePosition(i*width/3, dialSpaceY, width/3, dialSpaceHeight);
		}
		break;
	case 4:
		for (int i = 0; i < 3; i++) {
			fxDials.at(i)->updatePosition(i*width/3, dialSpaceY, width/3, dialSpaceHeight/2);
		}
		fxDials.at(3)->updatePosition(width/2, dialSpaceY + dialSpaceHeight/2, width, dialSpaceHeight/2);
		break;
	case 5:
		for (int i = 0; i < 3; i++) {
			fxDials.at(i)->updatePosition(i*width/3, dialSpaceY, width/3, dialSpaceHeight/2);
		}
		for (int j = 0; j < 2; j++) {
			fxDials.at(j+3)->updatePosition(j*width/2, dialSpaceY + dialSpaceHeight/2, width/2, dialSpaceHeight/2);
		}
		break;
	case 6: // max 6
		for (int i = 0; i < 3; i++) {
			fxDials.at(i)->updatePosition(i*width/3, dialSpaceY, width/3, dialSpaceHeight/2);
		}
		for (int j = 0; j < 3; j++) {
			fxDials.at(j+3)->updatePosition(j*width/3, dialSpaceY + dialSpaceHeight/2, width/3, dialSpaceHeight/2);
		}
		break;
	default: break;
	}
}
