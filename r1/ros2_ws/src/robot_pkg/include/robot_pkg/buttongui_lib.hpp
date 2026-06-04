#pragma once
#include <cstdint>

class ButtonguiLib
{
public:
    enum ButtonState : uint16_t
    {
        Start          = (1 << 0),
        Auto           = (1 << 1),
        Manual         = (1 << 2),
        Debug          = (1 << 3),
        Switch_side    = (1 << 4),
        retry_zone1    = (1 << 5),
        retry_zone3    = (1 << 6),

    };

    ButtonguiLib();

    void setButton(uint8_t button);
    bool isPressed(ButtonState button) const;
    uint8_t getState() const;

private:
    uint8_t button_state_;
};