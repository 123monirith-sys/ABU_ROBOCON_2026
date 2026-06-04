#include "robot_pkg/buttongui_lib.hpp"

ButtonguiLib::ButtonguiLib()
: button_state_(0)
{
}

void ButtonguiLib::setButton(uint8_t button)
{
    button_state_ = button;
}

bool ButtonguiLib::isPressed(ButtonState button) const
{
    return (button_state_ & button);
}

uint8_t ButtonguiLib::getState() const
{
    return button_state_;
}