/*
 * magna_fx.h
 *
 *  Created on: Nov 17, 2021
 *      Author: Nick Nagy
 */

#ifndef INC_MAGNA_FX_H_
#define INC_MAGNA_FX_H_

#define FX_DEBUG 1

#include <string>
#include <memory>
#include <vector>

namespace magna {

/**
 * @class EffectParameter
 * @brief
 */
class EffectParameter {
private:
	std::string stringId;
	float minValue, maxValue, currentValue, division;
public:
	EffectParameter(std::string stringId, float minValue, float maxValue, float increment) {
		this->stringId = stringId;
		currentValue = minValue;
		division = 4096.0f/(maxValue - minValue); // 4096 = range of ADC values
	}
	void setValue(float rawValue) { this->currentValue = rawValue/division; }
	float getCurrentValue() { return currentValue; }
	float getMinValue() { return minValue; }
	float getMaxValue() { return maxValue; }
	std::string getId() { return stringId; }
};

/**
 * @class Effect
 * @brief virtual class layout for guitar effects processing
 */
template <class T>
class Effect {
protected:
	std::vector<std::unique_ptr<magna::EffectParameter>> params;

	void addParameter(std::string stringId, float minValue, float maxValue, float increment) {
		params.push_back(std::make_unique<magna::EffectParameter>(stringId, minValue, maxValue, increment));
	}
public:
	Effect() {}
	virtual ~Effect(){}

	void updateParameterValue(uint8_t paramIdx, float rawValue) {
		if (paramIdx < params.size()) {
			params.at(paramIdx)->setValue(rawValue);
		}
	}

	float getParameterValue(uint8_t paramIdx) {
		if (paramIdx < params.size()) {
			return params.at(paramIdx)->getCurrentValue();
		}
		return 0.0f;
	}

	/* @return: a copy of an EffectParameter object
	 * can be used to help initialize a UIDial (provides stringId, minValue, maxValue, etc)
	 * should not be used frequently -- once UI is initialized, UIDial should be updated by ADC interface, not FX
	 */
	magna::EffectParameter getParameterCopy() {
		// TODO
	}

	virtual T process(T xn) {
		return xn;
	}
};

/**
 * @class VolumeDummyEffect
 * @brief this is a very simple example class for changing the volume
 * of the guitar out
 */
template <class T>
class VolumeDummyEffect : public magna::Effect<T> {
public:
	VolumeDummyEffect() {
		this->addParameter("Volume", 0.0f, 1.0f, 0.0f);
	}

	~VolumeDummyEffect() override {}

	T process(T xn) override {
#if FX_DEBUG
		float v = this->params.at(0)->getCurrentValue();
		T yn = (T)(xn*v);
		return yn;
#else
		return (T)(xn*this->params.at(0)->getCurrentValue());
#endif
	}
};

}

#endif /* INC_MAGNA_FX_H_ */
