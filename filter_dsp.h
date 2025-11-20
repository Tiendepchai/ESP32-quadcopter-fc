
#pragma once

#include <cstdint>
#include <cstddef>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace fc {
namespace filter {

struct IFilter1D {
    virtual void reset(float v) = 0;
    virtual float process(float x) = 0;
    virtual ~IFilter1D() = default;
};

// ===== PT1 Filter =====
class PT1Filter : public IFilter1D {
public:
    PT1Filter()
        : _state(0.0f)
        , _alpha(1.0f)
    {}

    void reset(float v) override {
        _state = v;
    }

    void configure(float fc_hz, float fs_hz) {
        if (fc_hz <= 0.0f || fs_hz <= 0.0f) {
            _alpha = 1.0f;
            return;
        }
        const float w  = 2.0f * static_cast<float>(M_PI) * fc_hz;
        const float dt = 1.0f / fs_hz;
        _alpha = 1.0f - std::exp(-w * dt);
        if (_alpha < 0.0f) _alpha = 0.0f;
        if (_alpha > 1.0f) _alpha = 1.0f;
    }

    float process(float x) override {
        _state = _state + _alpha * (x - _state);
        return _state;
    }

private:
    float _state;
    float _alpha;
};

// ===== Biquad Filter =====
class BiquadFilter : public IFilter1D {
public:
    BiquadFilter()
        : _a1(0.0f), _a2(0.0f)
        , _b0(1.0f), _b1(0.0f), _b2(0.0f)
        , _z1(0.0f), _z2(0.0f)
    {}

    void reset(float v) override {
        _z1 = v * (1.0f - _b0);
        _z2 = (_b2 - _a2) * v;
    }

    void setCoefficients(float b0,float b1,float b2,float a1,float a2) {
        _b0 = b0; _b1 = b1; _b2 = b2;
        _a1 = a1; _a2 = a2;
    }

    float process(float x) override {
        float y      = _b0 * x + _z1;
        float z1_new = _b1 * x - _a1 * y + _z2;
        float z2_new = _b2 * x - _a2 * y;
        _z1 = z1_new;
        _z2 = z2_new;
        return y;
    }

protected:
    float _a1,_a2;
    float _b0,_b1,_b2;
    float _z1,_z2;
};

// ===== Notch Filter =====
class NotchFilter : public BiquadFilter {
public:
    void configure(float f0_hz, float Q, float fs_hz) {
        if (f0_hz <= 0.0f || fs_hz <= 0.0f || Q <= 0.0f) {
            setCoefficients(1.0f,0.0f,0.0f,0.0f,0.0f);
            return;
        }
        const float w0    = 2.0f * static_cast<float>(M_PI) * f0_hz / fs_hz;
        const float cos_w = std::cos(w0);
        const float sin_w = std::sin(w0);
        const float alpha = sin_w / (2.0f * Q);

        float b0 = 1.0f;
        float b1 = -2.0f * cos_w;
        float b2 = 1.0f;
        float a0 = 1.0f + alpha;
        float a1 = -2.0f * cos_w;
        float a2 = 1.0f - alpha;

        b0 /= a0; b1 /= a0; b2 /= a0;
        a1 /= a0; a2 /= a0;

        setCoefficients(b0,b1,b2,a1,a2);
    }
};

// ===== FilterChain (optional) =====
class FilterChain {
public:
    static constexpr std::size_t MAX_FILTERS = 4;

    FilterChain() : _count(0) {
        for (std::size_t i = 0; i < MAX_FILTERS; ++i) {
            _filters[i] = nullptr;
        }
    }

    bool addFilter(IFilter1D* f) {
        if (!f || _count >= MAX_FILTERS) return false;
        _filters[_count++] = f;
        return true;
    }

    void reset(float v) {
        for (std::size_t i = 0; i < _count; ++i) {
            if (_filters[i]) _filters[i]->reset(v);
        }
    }

    float process(float x) {
        float y = x;
        for (std::size_t i = 0; i < _count; ++i) {
            if (_filters[i]) y = _filters[i]->process(y);
        }
        return y;
    }

    std::size_t size() const { return _count; }

private:
    IFilter1D* _filters[MAX_FILTERS];
    std::size_t _count;
};

} // namespace filter
} // namespace fc
