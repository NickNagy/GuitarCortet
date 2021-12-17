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

#include "magna_config.h"
#include "magna_lcd.h"
#include "magna_fx.h"

namespace magna {

class UserInterface;
class MenuUserInterface;
class EffectUserInterface;
//class App;

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
#if !INDEPENDENT_UI_ITEMS
	magna::UserInterface& owner;
#endif
	uint16_t x, y, width, height, borderColor, backgroundColor;
	//std::string stringId;
public:
#if INDEPENDENT_UI_ITEMS
	UIItem(UIItemStyleSheet& styleSheet) : x(0), y(0),
#else
	UIItem(magna::UserInterface& owner, UIItemStyleSheet& styleSheet) : owner(owner), x(0), y(0),
#endif
		width(0), height(0), borderColor(styleSheet.borderColor), backgroundColor(styleSheet.backgroundColor)//, stringId("")
	{}

	virtual ~UIItem(){}

	void updatePosition(uint16_t x0, uint16_t y0, uint16_t width, uint16_t height) {
		this->x = x0;
		this->y = y0;
		this->width = width;
		this->height = height;
	}

	virtual void draw(magna::LCD& display) = 0;
};

struct UITextBoxStyleSheet : public UIItemStyleSheet {
	uint16_t textColor, textBackgroundColor;
	UITextBoxStyleSheet(uint16_t textColor, uint16_t textBackgroundColor,
			uint16_t backgroundColor, uint16_t borderColor):
				UIItemStyleSheet(backgroundColor, borderColor),
				textColor(textColor), textBackgroundColor(textBackgroundColor)
				 {}
};

/**
 * @class UITextBox
 * @brief
 */
class UITextBox : public UIItem {
protected:
	uint16_t textSize, textXOffset, textYOffset, textColor, textBackgroundColor;
	//std::string& text;
public:
#if INDEPENDENT_UI_ITEMS
	UITextBox(UITextBoxStyleSheet& styleSheet): UIItem(styleSheet),
#else
	UITextBox(magna::UserInterface& owner, UITextBoxStyleSheet& styleSheet) : UIItem(owner, styleSheet),
#endif
		textColor(styleSheet.textColor), textBackgroundColor(styleSheet.backgroundColor)//, text(text)
	{}
	~UITextBox() override {}

	void updateText(std::string& text) {
		//this->text = text;
	}

	void draw(magna::LCD& display) override;
};


struct UIEditableItemStyleSheet : public UIItemStyleSheet {
	uint16_t backgroundHighlightedColor, borderHighlightedColor;
	UIEditableItemStyleSheet(uint16_t backgroundHighlightedColor,
			uint16_t borderHighlightedColor, uint16_t backgroundColor,
			uint16_t borderColor) : UIItemStyleSheet(backgroundColor, borderColor),
					backgroundHighlightedColor(backgroundHighlightedColor),
					borderHighlightedColor(borderHighlightedColor)
					 {}
};

/**
 * @class UIEditableItem
 * @brief
 */
class UIEditableItem : public UIItem {
protected:
	uint16_t backgroundHighlightedColor, borderHighlightedColor;
	bool isSelected = false;
public:
#if INDEPENDENT_UI_ITEMS
	UIEditableItem(UIEditableItemStyleSheet& styleSheet) : UIItem(styleSheet),
#else
	UIEditableItem(magna::UserInterface& owner, UIEditableItemStyleSheet& styleSheet) : UIItem(owner, styleSheet),
#endif
		backgroundHighlightedColor(styleSheet.backgroundHighlightedColor), borderHighlightedColor(styleSheet.borderHighlightedColor)
	{}

	~UIEditableItem() override {}

	void toggleSelected() {
		isSelected = !isSelected;
	}

	//void draw(magna::LCD& display) override;

	/*  */
	virtual void refresh(magna::LCD& display) = 0;
};

struct UIDialStyleSheet : public UIEditableItemStyleSheet{
	uint16_t dialArcThickness, dialArcColor, textColor;
	UIDialStyleSheet(uint16_t dialArcThickness, uint16_t dialArcColor,
			uint16_t textColor, uint16_t backgroundHighlightedColor, uint16_t borderHighlightedColor,
			uint16_t backgroundColor, uint16_t borderColor) :
			UIEditableItemStyleSheet(backgroundHighlightedColor, borderHighlightedColor, backgroundColor, borderColor),
			dialArcThickness(dialArcThickness), dialArcColor(dialArcColor), textColor(textColor)
			{}
};

/**
 * @class UIDial
 * @brief
 */
class UIDial : public UIEditableItem {
protected:
	float minValue, maxValue, currentValue;
	int16_t endAngle, lastStartAngle;
	uint16_t dialArcThickness, dialArcColor, textColor;
	//std::string& name;
	bool arcInitialized = false;
public:
#if INDEPENDENT_UI_ITEMS
	UIDial(UIDialStyleSheet& styleSheet, float minValue, float maxValue) : //, std::string name) :
		UIEditableItem(styleSheet),
#else
	UIDial(magna::UserInterface& owner, UIDialStyleSheet& styleSheet, float minValue, float maxValue) : //, std::string name) :
		UIEditableItem(owner, styleSheet),
#endif
		minValue(minValue), maxValue(maxValue), currentValue(minValue),
		endAngle(225), lastStartAngle(endAngle), dialArcThickness(styleSheet.dialArcThickness), dialArcColor(styleSheet.dialArcColor)
		//name(name)
	{}
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
#if INDEPENDENT_UI_ITEMS
	UIButton(UIButtonStyleSheet& styleSheet) : UIEditableItem(styleSheet) {}
#else
	UIButton(magna::UserInterface& owner, UIButtonStyleSheet& styleSheet) : UIEditableItem(owner, styleSheet) {}
#endif
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
	bool initialized = false;

public:
	UserInterface(UIStyleSheet& styleSheet, magna::LCD& display) : display(display),
		 backgroundColor(styleSheet.backgroundColor), width(display.getWidth()),
		 height(display.getHeight())
	{}
	virtual ~UserInterface() {}

	/* should set initialized to true */
	virtual void drawInitialScreen() {
		display.fill(backgroundColor);
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
	MenuUserInterface(MenuStyleSheet& styleSheet, magna::LCD& display) : UserInterface(styleSheet, display), selectionIdx(0)
	{}

	~MenuUserInterface() override {}

	void incrementSelection() {}
	void decrementSelection() {}

	//void refreshScreen() override;
};

struct EffectUIStyleSheet : public MenuStyleSheet{
	UIDialStyleSheet& dialStyleSheet;
	EffectUIStyleSheet(UIDialStyleSheet& dialStyleSheet, UITextBoxStyleSheet& textBoxStyleSheet,
			UITextButtonStyleSheet& textButtonStyleSheet) : MenuStyleSheet(textBoxStyleSheet, textButtonStyleSheet),
					dialStyleSheet(dialStyleSheet)
					{}
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
	magna::UITextBox titleBox;
public:
	EffectUserInterface(magna::EffectUIStyleSheet& styleSheet, magna::LCD& display)://, std::string title) :
		MenuUserInterface(styleSheet, display), dialStyleSheet(styleSheet.dialStyleSheet),
#if INDEPENDENT_UI_ITEMS
		titleBox(styleSheet.titleStyleSheet)//, title)
#else
		titleBox(*this, styleSheet.titleStyleSheet)//, title)
#endif
	{}
	~EffectUserInterface() override {}

	void drawInitialScreen() override {
		UserInterface::drawInitialScreen();
		for (unsigned int i = 0; i < fxDials.size(); i++) {
			fxDials.at(i)->draw(display);
		}
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
