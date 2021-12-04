/*
 * magna_ui.h
 *
 *  Created on: Nov 17, 2021
 *      Author: Nick Nagy
 */

#ifndef INC_MAGNA_UI_H_
#define INC_MAGNA_UI_H_

#include <memory>
#include <vector>
#include <string>

#include "magna_lcd.h"

namespace magna {

typedef struct UIItemStyleSheet{
	uint16_t borderColor, backgroundColor;
}UIItemStyleSheet;

/**
 * @class UIItem
 * @brief virtual class for items on a user interface. Derive for buttons, images, textboxes, etc
 */
class UIItem {
protected:
	uint16_t x, y, width, height, borderColor, backgroundColor;
	std::string stringId;
	magna::LCD& display;
public:
	UIItem(UIItemStyleSheet& styleSheet, magna::LCD& display) : display(display) {
		this->borderColor = styleSheet.borderColor;
		this->backgroundColor = styleSheet.backgroundColor;
	}

	virtual ~UIItem(){}

	void updatePosition(uint16_t x0, uint16_t y0, uint16_t width, uint16_t height) {
		this->x = x;
		this->y = y;
		this->width = width;
		this->height = height;
	}

	virtual void draw() = 0;
};

typedef struct UITextBoxStyleSheet : public UIItemStyleSheet {
	uint16_t textColor, textBackgroundColor;
}UITextBoxStyleSheet;

/**
 * @class UITextBox
 * @brief
 */
class UITextBox : public UIItem {
protected:
	uint16_t textSize, textXOffset, textYOffset, textColor, textBackgroundColor;
	std::string text;
public:
	UITextBox(UITextBoxStyleSheet& styleSheet, magna::LCD& display) : UIItem(styleSheet, display) {
		this->textColor = styleSheet.textColor;
		this->textBackgroundColor = styleSheet.textBackgroundColor;
	}
	~UITextBox() override {}
	void draw() override;
};


typedef struct UIEditableItemStyleSheet : public UIItemStyleSheet {
	uint16_t backgroundHighlightedColor, borderHighlightedColor;
}UIEditableItemStyleSheet;

/**
 * @class UIEditableItem
 * @brief
 */
class UIEditableItem : public UIItem {
protected:
	uint16_t backgroundHighlightedColor, borderHighlightedColor;
	bool isSelected = false;
public:
	UIEditableItem(UIEditableItemStyleSheet& styleSheet, magna::LCD& display) : UIItem(styleSheet, display) {
		this->backgroundHighlightedColor = styleSheet.backgroundHighlightedColor;
		this->borderHighlightedColor = styleSheet.borderHighlightedColor;
	}
	virtual ~UIEditableItem() {}

	void toggleSelected() { isSelected = !isSelected; }
};

typedef struct UIDialStyleSheet : public UIEditableItemStyleSheet{
	// empty for now
}UIDialStyleSheet;

/**
 * @class UIDial
 * @brief
 */
class UIDial : public UIEditableItem {
protected:
	int16_t minValue, maxValue, currentValue;
public:
	UIDial(UIDialStyleSheet& styleSheet, magna::LCD& display, uint16_t minValue, uint16_t maxValue) : UIEditableItem(styleSheet, display) {
		this->minValue = minValue;
		this->maxValue = maxValue;
	};
	~UIDial() override;

	void setValue(int16_t newValue) {
		currentValue = (newValue > maxValue) ? maxValue : ((newValue < minValue) ? minValue : newValue);
	}
	int16_t getValue() { return currentValue; }

	void draw() override;
};

typedef struct UIButtonStyleSheet : public UIEditableItemStyleSheet{
	// TODO
}UIButtonStyleSheet;

/**
 * @class UIButton
 * @brief
 */
class UIButton : public UIEditableItem {
public:
	UIButton(UIButtonStyleSheet& styleSheet, magna::LCD& display) : UIEditableItem(styleSheet, display) {}
	~UIButton() override {}

	virtual void click() = 0;
};


typedef struct UITextButtonStyleSheet : public UITextBoxStyleSheet {

}UITextButtonStyleSheet;
// TODO: check again how inheriting from two parent classes works
/**
 * @class UITextButton
 * @brief
 */
class UITextButton : public UITextBox, public UIButton {

};

typedef struct UIStyleSheet {
	uint16_t backgroundColor;
}UIStyleSheet;

/**
 * @class UserInterface
 * @brief virtual class for defining an entire LCD screen interface
 */
class UserInterface {
protected:
	magna::LCD& display;
	uint16_t backgroundColor;
	std::string stringId;
public:
	UserInterface(UIStyleSheet& styleSheet, magna::LCD& display) : display(display) {
		this->display = display;
		this->backgroundColor = styleSheet.backgroundColor;
	}
	virtual ~UserInterface() {}

	/* define how to draw the LCD display in this function */
	virtual void refreshScreen() { display.fill(backgroundColor); }
};

typedef struct MenuStyleSheet : public UIStyleSheet {
	UITextBoxStyleSheet& titleStyleSheet;
	UITextButtonStyleSheet& textButtonStyleSheet;
}MenuStyleSheet;

/**
 * @class MenuUserInterface
 * @brief this class is a derivation of a UserInterface for defining a menu screen on the LCD
 */
class MenuInterface : public UserInterface {
protected:
	uint8_t selectionIdx;
public:
	MenuInterface(MenuStyleSheet& styleSheet, magna::LCD& display) : UserInterface(styleSheet, display) {
		selectionIdx = 0;
		//this->buttonBorderColor = styleSheet.buttonBorderColor;
		//this->buttonBackgroundColor = styleSheet.buttonBackgroundColor;
		//this->buttonTextColor = styleSheet.buttonTextColor;
	};

	~MenuInterface() override {}

	void incrementSelection() {}
	void decrementSelection() {}

	void refreshScreen() override;
};

typedef struct EffectsUIStyleSheet : public MenuStyleSheet{
	UIDialStyleSheet& dialStyleSheet;
}EffectsUIStyleSheet;

/**
 * @class EffectUserInterface
 * @brief interface class for a processor/instrument effect
 */
class EffectUserInterface : public MenuInterface {
protected:
	std::vector<magna::UIDial> fxDials;
public:
	EffectUserInterface();
};

}

class MultiInterfaceHandler {
private:

public:
	MultiInterfaceHandler() {}
};

#endif /* INC_MAGNA_UI_H_ */
