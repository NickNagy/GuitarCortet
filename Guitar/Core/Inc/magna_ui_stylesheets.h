/*
 * magna_ui_stylesheets.h
 *
 *  Created on: Jan 7, 2022
 *      Author: Nick Nagy
 */

#ifndef INC_MAGNA_UI_STYLESHEETS_H_
#define INC_MAGNA_UI_STYLESHEETS_H_

namespace magna {
struct UIItemStyleSheet{
	uint16_t borderColor, backgroundColor;
	UIItemStyleSheet(uint16_t backgroundColor, uint16_t borderColor) :
		borderColor(borderColor), backgroundColor(backgroundColor) {}
};

struct UITextBoxStyleSheet : public UIItemStyleSheet {
	uint16_t textColor, textBackgroundColor;
	UITextBoxStyleSheet(uint16_t textColor, uint16_t textBackgroundColor,
			uint16_t backgroundColor, uint16_t borderColor):
				UIItemStyleSheet(backgroundColor, borderColor),
				textColor(textColor), textBackgroundColor(textBackgroundColor)
				 {}
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

struct UIDialStyleSheet : public UIEditableItemStyleSheet{
	uint16_t dialArcThickness, dialArcColor, textColor;
	UIDialStyleSheet(uint16_t dialArcThickness, uint16_t dialArcColor,
			uint16_t textColor, uint16_t backgroundHighlightedColor, uint16_t borderHighlightedColor,
			uint16_t backgroundColor, uint16_t borderColor) :
			UIEditableItemStyleSheet(backgroundHighlightedColor, borderHighlightedColor, backgroundColor, borderColor),
			dialArcThickness(dialArcThickness), dialArcColor(dialArcColor), textColor(textColor)
			{}
};

struct UIButtonStyleSheet : public UIEditableItemStyleSheet{
	// TODO
};

struct UITextButtonStyleSheet : public UITextBoxStyleSheet {
	UITextButtonStyleSheet(uint16_t textColor, uint16_t textBackgroundColor,
			uint16_t borderColor, uint16_t backgroundColor) :
				UITextBoxStyleSheet(textColor, textBackgroundColor, borderColor,
						backgroundColor) {}
};

struct MenuStyleSheet : public UIStyleSheet {
	UITextBoxStyleSheet& titleStyleSheet;
	UITextButtonStyleSheet& textButtonStyleSheet;
	MenuStyleSheet(UITextBoxStyleSheet& titleStyleSheet, UITextButtonStyleSheet& textButtonStyleSheet) :
		titleStyleSheet(titleStyleSheet), textButtonStyleSheet(textButtonStyleSheet) {}
};

struct EffectUIStyleSheet : public MenuStyleSheet{
	UIDialStyleSheet& dialStyleSheet;
	EffectUIStyleSheet(UIDialStyleSheet& dialStyleSheet, UITextBoxStyleSheet& textBoxStyleSheet,
			UITextButtonStyleSheet& textButtonStyleSheet) : MenuStyleSheet(textBoxStyleSheet, textButtonStyleSheet),
					dialStyleSheet(dialStyleSheet)
					{}
};
}

#endif /* INC_MAGNA_UI_STYLESHEETS_H_ */
