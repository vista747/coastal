#include "plugin.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <vector>

namespace {
constexpr float PI = 3.14159265358979323846f;
constexpr int BUFFER_SIZE = 1 << 21;
constexpr int MAX_GRAINS = 48;

struct Grain {
    bool active = false;
    float lag = 1.f;
    float age = 0.f;
    float duration = 1.f;
    float randomRate = 1.f;    // fixed per-grain pitch offset as a rate ratio
};

struct Randomizer {
    uint32_t state = 0x9e3779b9u;
    float uniform() {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return (state >> 8) * (1.f / 16777216.f);
    }
    float bipolar() { return 2.f * uniform() - 1.f; }
};

inline float limited(float v, float lo, float hi) {
    return rack::math::clamp(v, lo, hi);
}
struct Curve {
    enum Kind { QUADRATIC, LOGARITHMIC, CENTERED };
    float lo, hi;
    Kind kind;
    float toValue(float p) const {
        p = limited(p, 0.f, 1.f);
        switch (kind) {
            case LOGARITHMIC: return lo * std::pow(hi / lo, p);
            case CENTERED: return p < 0.5f ? lo * (1.f - 2.f * p) : hi * (2.f * p - 1.f);
            default: return lo + (hi - lo) * p * p;
        }
    }
    float toPosition(float v) const {
        v = limited(v, lo, hi);
        switch (kind) {
            case LOGARITHMIC: return std::log(v / lo) / std::log(hi / lo);
            case CENTERED: return v < 0.f ? 0.5f * (1.f - v / lo) : 0.5f * (1.f + v / hi);
            default: return std::sqrt((v - lo) / (hi - lo));
        }
    }
};

constexpr Curve SPRAY_CURVE{0.f, 500.f, Curve::QUADRATIC};
constexpr Curve FREQ_CURVE{1.f, 150.f, Curve::LOGARITHMIC};
constexpr Curve PITCH_CURVE{-36.f, 12.f, Curve::CENTERED};
constexpr Curve SIZE_CURVE{0.25f, 4.f, Curve::LOGARITHMIC};
constexpr Curve RAND_CURVE{0.f, 6.f, Curve::QUADRATIC};
struct ThreeDecimalQuantity : ParamQuantity {
    std::string getDisplayValueString() override {
        char text[32];
        std::snprintf(text, sizeof text, "%.3f", getDisplayValue());
        std::string out = text;
        out.erase(out.find_last_not_of('0') + 1);
        if (out.back() == '.') out.pop_back();
        return out == "-0" ? "0" : out;
    }
};
struct CurvedQuantity : ThreeDecimalQuantity {
    Curve curve{0.f, 1.f, Curve::QUADRATIC};
    float getDisplayValue() override { return curve.toValue(getValue()); }
    void setDisplayValue(float v) override { setImmediateValue(curve.toPosition(v)); }
};
} // namespace

struct Daisy : Module {
    enum ParamId {
        DELAY_PARAM, SPRAY_PARAM, FREQ_PARAM, PITCH_PARAM,
        RAND_PARAM, SIZE_PARAM, FEED_PARAM, MIX_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        IN_L_INPUT,
        IN_R_INPUT,
        DELAY_CV_INPUT, SPRAY_CV_INPUT, FREQ_CV_INPUT, PITCH_CV_INPUT,
        RAND_CV_INPUT, SIZE_CV_INPUT, FEED_CV_INPUT, MIX_CV_INPUT,
        INPUTS_LEN
    };
    enum OutputId { OUT_L_OUTPUT, OUT_R_OUTPUT, OUTPUTS_LEN };
    enum LightId { LIGHTS_LEN };

    std::array<std::vector<float>, 2> tape;
    std::array<std::array<Grain, MAX_GRAINS>, 2> grains{};
    std::array<Randomizer, 2> rng{};
    uint32_t writePos = 0;
    std::array<float, 2> samplesToNext{};
    bool staggerRight = true;
    std::array<float, 2> lastWet{};
    float smoothFeedback = 0.f;
    float smoothMix = 0.5f;
    float smoothPitch = 0.f;
    float lastSampleRate = 0.f;

    Daisy() {
        for (auto& t : tape) t.resize(BUFFER_SIZE, 0.f);
        rng[1].state = 0x85ebca6bu;
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam<ThreeDecimalQuantity>(DELAY_PARAM, 0.f, 1000.f, 0.f, "delay time", " ms");
        configCurved(SPRAY_PARAM, SPRAY_CURVE, 0.f, "spray", " ms");
        configCurved(FREQ_PARAM, FREQ_CURVE, 60.f, "frequency", " Hz");
        configCurved(PITCH_PARAM, PITCH_CURVE, 0.f, "pitch", " semitones");
        configCurved(RAND_PARAM, RAND_CURVE, 0.f, "random pitch", " semitones");
        configCurved(SIZE_PARAM, SIZE_CURVE, 1.f, "grain size", " x period");
        configParam<ThreeDecimalQuantity>(FEED_PARAM, 0.f, 0.95f, 0.f, "feedback", " %", 0.f, 100.f);
        configParam<ThreeDecimalQuantity>(MIX_PARAM, 0.f, 1.f, 0.5f, "dry / wet", " %", 0.f, 100.f);

        configInput(IN_L_INPUT, "left / mono input");
        configInput(IN_R_INPUT, "right input");
        const char* cvNames[] = {"delay time CV", "spray CV", "frequency CV", "pitch CV",
                                 "random pitch CV", "grain size CV", "feedback CV", "dry / wet CV"};
        for (int i = 0; i < 8; ++i) configInput(DELAY_CV_INPUT + i, cvNames[i]);
        configOutput(OUT_L_OUTPUT, "left output");
        configOutput(OUT_R_OUTPUT, "right output");
        configBypass(IN_L_INPUT, OUT_L_OUTPUT);
        configBypass(IN_R_INPUT, OUT_R_OUTPUT);
        onReset();
    }

    void configCurved(int paramId, const Curve& curve, float defaultValue,
                      const char* name, const char* unit) {
        auto* q = configParam<CurvedQuantity>(paramId, 0.f, 1.f, curve.toPosition(defaultValue), name, unit);
        q->curve = curve;
    }

    void onReset() override {
        for (auto& t : tape) std::fill(t.begin(), t.end(), 0.f);
        for (auto& pool : grains) for (auto& g : pool) g = Grain{};
        writePos = 0;
        samplesToNext = {{0.f, 0.f}};
        staggerRight = true;
        lastWet = {{0.f, 0.f}};
        smoothFeedback = params[FEED_PARAM].getValue();
        smoothMix = params[MIX_PARAM].getValue();
        smoothPitch = PITCH_CURVE.toValue(params[PITCH_PARAM].getValue());
    }

    float setting(int paramId, int cvId, float lo, float hi) {
        float value = params[paramId].getValue();
        if (inputs[cvId].isConnected()) value += inputs[cvId].getVoltage() * (hi - lo) * 0.1f;
        return limited(value, lo, hi);
    }

    float curvedSetting(int paramId, int cvId, const Curve& curve) {
        float position = params[paramId].getValue();
        if (inputs[cvId].isConnected()) position += inputs[cvId].getVoltage() * 0.1f;
        return curve.toValue(position);
    }

    float readTape(int channel, float position) const {
        float base = std::floor(position);
        float frac = position - base;
        int i0 = static_cast<int>(base) & (BUFFER_SIZE - 1);
        int i1 = (i0 + 1) & (BUFFER_SIZE - 1);
        const auto& t = tape[channel];
        return t[i0] + frac * (t[i1] - t[i0]);
    }

    static float semitonesToRate(float semitones) {
        return std::pow(2.f, limited(semitones, -72.f, 48.f) / 12.f);
    }

    void spawn(int ch, float sr, float duration, float delaySamples, float sprayMs,
               float pitchSemitones, float randomPitchSemitones) {
        Grain* target = nullptr;
        for (auto& g : grains[ch]) {
            if (!g.active) { target = &g; break; }
        }
        if (!target) return;

        float randomPitch = rng[ch].bipolar() * randomPitchSemitones;
        float rate = semitonesToRate(pitchSemitones + randomPitch);
        float jitter = rng[ch].bipolar() * sprayMs * 0.001f * sr;
        float actualDelay = std::max(1.f, delaySamples + jitter);
        float safety = duration * std::max(0.f, rate - 1.f) + 2.f;

        target->lag = actualDelay + safety;
        target->age = 0.f;
        target->duration = duration;
        target->randomRate = semitonesToRate(randomPitch);
        target->active = true;
    }

    void process(const ProcessArgs& args) override {
        const float sr = args.sampleRate;
        if (lastSampleRate != sr) {
            onReset();
            lastSampleRate = sr;
        }
        const float dryL = inputs[IN_L_INPUT].getVoltage();
        const float dryR = inputs[IN_R_INPUT].isConnected()
            ? inputs[IN_R_INPUT].getVoltage() : dryL;
        const float dry[2] = {dryL, dryR};

        const float delay = setting(DELAY_PARAM, DELAY_CV_INPUT, 0.f, 1000.f);
        const float spray = curvedSetting(SPRAY_PARAM, SPRAY_CV_INPUT, SPRAY_CURVE);
        const float freq = curvedSetting(FREQ_PARAM, FREQ_CV_INPUT, FREQ_CURVE);
        float pitch = PITCH_CURVE.toValue(params[PITCH_PARAM].getValue());
        if (inputs[PITCH_CV_INPUT].isConnected()) pitch += inputs[PITCH_CV_INPUT].getVoltage() * 12.f;
        pitch = limited(pitch, PITCH_CURVE.lo, PITCH_CURVE.hi);
        const float randPitch = curvedSetting(RAND_PARAM, RAND_CV_INPUT, RAND_CURVE);
        const float size = curvedSetting(SIZE_PARAM, SIZE_CV_INPUT, SIZE_CURVE);
        const float feedback = setting(FEED_PARAM, FEED_CV_INPUT, 0.f, 0.95f);
        const float mix = setting(MIX_PARAM, MIX_CV_INPUT, 0.f, 1.f);

        const float smooth = std::min(1.f, 200.f / sr); // about 5 ms smoothing
        smoothPitch += std::min(1.f, 100.f / sr) * (pitch - smoothPitch);
        const float pitchRate = semitonesToRate(smoothPitch);
        smoothFeedback += smooth * (feedback - smoothFeedback);
        smoothMix += smooth * (mix - smoothMix);
        for (int ch = 0; ch < 2; ++ch) {
            tape[ch][writePos] = limited(dry[ch] + lastWet[ch] * smoothFeedback, -20.f, 20.f);
        }

        const float period = sr / freq;
        const float duration = std::max(8.f, std::min(sr * 2.f, period * size));
        const float delaySamples = sr * delay * 0.001f;
        const float hop = std::max(1.f, 0.5f * period);
        if (staggerRight) {
            samplesToNext[1] = 0.5f * hop;
            staggerRight = false;
        }

        // Normalise by the expected number of overlapping grains, not the instantaneous window
        // sum. Dividing by the live sum boosts grain edges to full level and clicks at small sizes.
        const float overlapGain = 1.f / std::max(1.f, duration / hop);

        for (int ch = 0; ch < 2; ++ch) {
            samplesToNext[ch] -= 1.f;
            if (samplesToNext[ch] <= 0.f) {
                spawn(ch, sr, duration, delaySamples, spray, pitch, randPitch);
                samplesToNext[ch] += hop;
            }

            float wet = 0.f;
            for (auto& g : grains[ch]) {
                if (!g.active) continue;
                const float phase = g.age / g.duration;
                if (phase >= 1.f) { g.active = false; continue; }
                const float window = 0.5f - 0.5f * std::cos(2.f * PI * phase);
                wet += readTape(ch, static_cast<float>(writePos) - g.lag) * window;
                g.lag = std::max(1.f, g.lag + 1.f - pitchRate * g.randomRate);
                g.age += 1.f;
            }
            wet *= overlapGain;
            lastWet[ch] = limited(wet, -20.f, 20.f);
            const float result = dry[ch] * (1.f - smoothMix) + wet * smoothMix;
            outputs[OUT_L_OUTPUT + ch].setVoltage(limited(result, -20.f, 20.f));
        }
        writePos = (writePos + 1) & (BUFFER_SIZE - 1);
    }
};

struct DaisyWidget : ModuleWidget {
    DaisyWidget(Daisy* module) {
        setModule(module);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/daisy.svg")));

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // 144 x 364.252 units rendered at 150 x 379.5 px
        constexpr float scale = 150.f / 144.f;
        auto at = [&](float x, float y) { return Vec(x * scale, y * scale); };

        constexpr float rowPitch = 28.f;
        constexpr float knobX = 66.88f;
        constexpr float knobY0 = 50.39f; 
        constexpr float cvX = 20.81f;
        constexpr float cvY0 = 49.6f;
        for (int i = 0; i < Daisy::PARAMS_LEN; ++i) {
            addParam(createParamCentered<RoundSmallBlackKnob>(
                at(knobX, knobY0 + rowPitch * i), module, i));
            addInput(createInputCentered<PJ301MPort>(
                at(cvX, cvY0 + rowPitch * i), module, Daisy::DELAY_CV_INPUT + i));
        }

        constexpr float inY = 285.8f;
        constexpr float outY = 327.8f;
        addInput(createInputCentered<PJ301MPort>(at(24.84f, inY), module, Daisy::IN_L_INPUT));
        addInput(createInputCentered<PJ301MPort>(at(53.81f, inY), module, Daisy::IN_R_INPUT));
        addOutput(createOutputCentered<PJ301MPort>(at(90.19f, outY), module, Daisy::OUT_L_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(at(119.16f, outY), module, Daisy::OUT_R_OUTPUT));
    }
};

Model* modelDaisy = createModel<Daisy, DaisyWidget>("daisy");
