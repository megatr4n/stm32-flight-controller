#pragma once

namespace Core {
    class LowPassFilter {
    private:
        float alpha;
        float previousOutput;
        float isInitialized;


    public:
    explicit LowPassFilter(float alphaValue) : alpha(alphaValue), previousOutput(0.0f), isInitialized(false) {}

    void setAlpha(float newAlpha) {
        if (newAlpha >= 0.0f && newAlpha <= 1.0f) {
            alpha = newAlpha;
        }
    }
    float apply(float rawInput) {
        if (!isInitialized) {
            previousOutput = rawInput;
            isInitialized = true;
            return rawInput;
        }
        previousOutput = (rawInput * alpha) + (previousOutput * (1.0f - alpha));
        return previousOutput;
    }
    void reset() {
        isInitialized = false;
        previousOutput = 0.0f;
    }
};
}