/*
 * magna_ui.h
 *
 *  Created on: Nov 17, 2021
 *      Author: Nick Nagy
 */

#ifndef INC_MAGNA_UI_H_
#define INC_MAGNA_UI_H_

#include <memory>
#include <queue>
#include <stack>
#include <string>
#include <vector>

#include "magna_lcd.h"
#include "magna_fx.h"

namespace magna {

class UserInterface;
class MenuUserInterface;
class EffectUserInterface;
class App;

struct UIItemStyleSheet{
	uint16_t borderColor, backgroundColor;
	UIItemStyleSheet(uint16_t backgroundColor, uint16_t borderColor) :
		borderColor(borderColor), backgroundColor(backgroundColor) {}
};

/**
 * @class UIItem
 * @brief virtual class for items on a user interface. Derive for buttons, images, textboxes, etc
 */
class UIItem {
protected:
	uint16_t x, y, width, height, borderColor, backgroundColor;
	std::string stringId;
	magna::UserInterface& owner;
public:
	UIItem(magna::UserInterface& owner, UIItemStyleSheet& styleSheet) : owner(owner) {//magna::LCD& display) : display(display) {
		this->borderColor = styleSheet.borderColor;
		this->backgroundColor = styleSheet.backgroundColor;
		stringId = "";
	}

	virtual ~UIItem(){}

	void updatePosition(uint16_t x0, uint16_t y0, uint16_t width, uint16_t height) {
		this->x = x;
		this->y = y;
		this->width = width;
		this->height = height;
	}

	virtual void draw(magna::LCD& display) = 0;
};

struct UITextBoxStyleSheet : public UIItemStyleSheet {
	uint16_t textColor, textBackgroundColor;
	UITextBoxStyleSheet(uint16_t textColor, uint16_t textBackgroundColor,
			uint16_t backgroundColor, uint16_t borderColor):
				textColor(textColor), textBackgroundColor(textBackgroundColor),
				UIItemStyleSheet(backgroundColor, borderColor) {}
};

/**
 * @class UITextBox
 * @brief
 */
class UITextBox : public UIItem {
protected:
	uint16_t textSize, textXOffset, textYOffset, textColor, textBackgroundColor;
	std::string text;
public:
	UITextBox(magna::UserInterface& owner, UITextBoxStyleSheet& styleSheet, std::string text) : UIItem(owner, styleSheet) {
		this->textColor = styleSheet.textColor;
		this->textBackgroundColor = styleSheet.textBackgroundColor;
		this->text = text;
	}
	~UITextBox() override {}

	void updateText(std::string text) {
		this->text = text;
	}

	void draw(magna::LCD& display) override;
};


typedef struct UIEditableItemStyleSheet : public UIItemStyleSheet {
	uint16_t backgroundHighlightedColor, borderHighlightedColor;
	UIEditableItemStyleSheet(uint16_t backgroundHighlightedColor,
			uint16_t borderHighlightedColor, uint16_t backgroundColor,
			uint16_t borderColor) : backgroundHighlightedColor(backgroundHighlightedColor),
					borderHighlightedColor(borderHighlightedColor),
					UIItemStyleSheet(backgroundColor, borderColor) {}
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
	UIEditableItem(magna::UserInterface& owner, UIEditableItemStyleSheet& styleSheet) : UIItem(owner, styleSheet) {
		this->backgroundHighlightedColor = styleSheet.backgroundHighlightedColor;
		this->borderHighlightedColor = styleSheet.borderHighlightedColor;
	}

	~UIEditableItem() override {}

	void toggleSelected() {
		isSelected = !isSelected;
	}

	//void draw(magna::LCD& display) override;

	/*  */
	virtual void refresh(magna::LCD& display) = 0;
};

typedef struct UIDialStyleSheet : public UIEditableItemStyleSheet{
	uint16_t dialArcThickness;
	uint16_t dialArcColor;
	uint16_t textColor;
	UIDialStyleSheet(uint16_t dialArcThickness, uint16_t dialArcColor,
			uint16_t textColor, uint16_t backgroundHighlightedColor, uint16_t borderHighlightedColor,
			uint16_t backgroundColor, uint16_t borderColor) :
			dialArcThickness(dialArcThickness), dialArcColor(dialArcColor), textColor(textColor),
			UIEditableItemStyleSheet(backgroundHighlightedColor, borderHighlightedColor,
					backgroundColor, borderColor) {}
}UIDialStyleSheet;

/**
 * @class UIDial
 * @brief
 */
class UIDial : public UIEditableItem {
protected:
	float minValue, maxValue, currentValue;
	int16_t lastStartAngle, endAngle;
	uint16_t dialArcThickness, dialArcColor, textColor;
	std::string name;
public:
	UIDial(magna::UserInterface& owner, UIDialStyleSheet& styleSheet, std::string name, float minValue, float maxValue) :
		UIEditableItem(owner, styleSheet) , name(name), minValue(minValue), maxValue(maxValue),
		dialArcThickness(styleSheet.dialArcThickness), dialArcColor(styleSheet.dialArcColor)
	{
		endAngle = 225;
		lastStartAngle = endAngle;
	};
	~UIDial() override {}

	void setValue(float newValue) {
		currentValue = (newValue > maxValue) ? maxValue : ((newValue < minValue) ? minValue : newValue);
	}
	float getValue() { return currentValue; }

	void draw(magna::LCD& display) override;
	void refresh(magna::LCD& display) override;
};

struct UIButtonStyleSheet : public UIEditableItemStyleSheet{
	// TODO
};

/**
 * @class UIButton
 * @brief
 */
class UIButton : public UIEditableItem {
public:
	UIButton(magna::UserInterface& owner, UIButtonStyleSheet& styleSheet) : UIEditableItem(owner, styleSheet) {}
	~UIButton() override {}

	void draw(magna::LCD& display) override;

	virtual void click() = 0;
};


struct UITextButtonStyleSheet : public UITextBoxStyleSheet {
	UITextButtonStyleSheet(uint16_t textColor, uint16_t textBackgroundColor,
			uint16_t borderColor, uint16_t backgroundColor) :
				UITextBoxStyleSheet(textColor, textBackgroundColor, borderColor,
						backgroundColor) {}
};
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
	std::queue<std::shared_ptr<magna::UIEditableItem>> refreshQueue;
	uint16_t backgroundColor, width, height;
	std::string stringId;
	bool initialized = false;

public:
	UserInterface(UIStyleSheet& styleSheet, magna::LCD& display) : display(display) {
		this->display = display;
		this->backgroundColor = styleSheet.backgroundColor;
		this->width = display.getWidth();
		this->height = display.getHeight();
	}
	virtual ~UserInterface() {}

	/* should set initialized to true */
	virtual void drawInitialScreen() {
		initialized = true;
	}

	/* define how to draw the LCD display in this function */
	void refreshScreen() {
		if (!initialized) {
			drawInitialScreen();
		}
		while (!refreshQueue.empty()) {
			refreshQueue.front()->refresh(display);
			refreshQueue.pop();
		}
	}
	// clear queue of its contents
	void clearPendingRefresh() {
		std::queue<std::shared_ptr<magna::UIEditableItem>> emptyQueue;
		std::swap(refreshQueue, emptyQueue);
	}
};

struct MenuStyleSheet : public UIStyleSheet {
	UITextBoxStyleSheet& titleStyleSheet;
	UITextButtonStyleSheet& textButtonStyleSheet;
	MenuStyleSheet(UITextBoxStyleSheet& titleStyleSheet, UITextButtonStyleSheet& textButtonStyleSheet) :
		titleStyleSheet(titleStyleSheet), textButtonStyleSheet(textButtonStyleSheet) {}
};

/**
 * @class MenuUserInterface
 * @brief this class is a derivation of a UserInterface for defining a menu screen on the LCD
 */
class MenuUserInterface : public UserInterface {
protected:
	uint8_t selectionIdx;
public:
	MenuUserInterface(MenuStyleSheet& styleSheet, magna::LCD& display) : UserInterface(styleSheet, display) {
		selectionIdx = 0;
		//this->buttonBorderColor = styleSheet.buttonBorderColor;
		//this->buttonBackgroundColor = styleSheet.buttonBackgroundColor;
		//this->buttonTextColor = styleSheet.buttonTextColor;
	}

	~MenuUserInterface() override {}

	void incrementSelection() {}
	void decrementSelection() {}

	//void refreshScreen() override;
};

struct EffectsUIStyleSheet : public MenuStyleSheet{
	UIDialStyleSheet& dialStyleSheet;
	EffectsUIStyleSheet(UIDialStyleSheet& dialStyleSheet, UITextBoxStyleSheet& textBoxStyleSheet,
			UITextButtonStyleSheet& textButtonStyleSheet) : MenuStyleSheet(textBoxStyleSheet, textButtonStyleSheet), dialStyleSheet(dialStyleSheet){}
};

/**
 * @class EffectUserInterface
 * @brief interface class for a processor/instrument effect
 */
class EffectUserInterface : public MenuUserInterface {
protected:
	magna::UIDialStyleSheet& dialStyleSheet;
	// containers cannot properly store references
	std::vector<std::shared_ptr<magna::UIDial>> fxDials;
	magna::UITextBox title;
public:
	EffectUserInterface(magna::EffectsUIStyleSheet& styleSheet, magna::LCD& display, std::string title) :
		MenuUserInterface(styleSheet, display), title(*this, styleSheet.titleStyleSheet, title), dialStyleSheet(styleSheet.dialStyleSheet)
	{

	}
	~EffectUserInterface() override {}

	void drawInitialScreen() override {
		for (int i = 0; i < fxDials.size(); i++) {
			fxDials.at(i)->draw(display);
		}
		initialized = true;
	}

	void addDial(std::string stringId, float minValue, float maxValue);

	void setDial(uint8_t index, int16_t value) {
		if (fxDials.size() > index) {
			std::shared_ptr<magna::UIDial> dial = fxDials.at(index);
			if (value != dial->getValue()) {
				dial->setValue(value);
				refreshQueue.push(dial);
			}
		}
	}
};

/*struct ProcessorAndUIWrapperStruct{
	magna::UserInterface& ui;
	magna::Effect<uint16_t>& fx;
	ProcessorAndUIWrapperStruct(magna::UserInterface& ui, magna::Effect<uint16_t>& fx) : ui(ui), fx(fx) {}
	~ProcessorAndUIWrapperStruct() {}
};

class App {
private:
	std::stack<magna::UserInterface&> screenHistory;
	std::vector<magna::ProcessorAndUIWrapperStruct&> effects;
	magna::UserInterface * currentScreen;
	magna::LCD& display;
	magna::Effect<uint16_t> * currentEffect;
public:
	App(magna::LCD& display) : display(display) {}
	~App() {}

	void returnToPreviousScreen() {
		if (screenHistory.size()) {
			currentScreen->clearPendingRefresh();
			currentScreen = &(screenHistory.top());
			screenHistory.pop();
		}
	}

	void goToNextScreen(magna::UserInterface& nextScreen) {
		currentScreen->clearPendingRefresh();
		screenHistory.push(*currentScreen);
		currentScreen = &nextScreen;
	}

	void goToNextScreen(magna::ProcessorAndUIWrapperStruct& processorAndUI) {

	}
};*/

}

#endif /* INC_MAGNA_UI_H_ */
