#ifndef STM32_STATUS_HPP
#define STM32_STATUS_HPP

#include "std_msgs/msg/float32_multi_array.hpp"

#include <cstdint>

class STM32Status
{
public:
    STM32Status();

    void update(
        const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    bool buttonPressed();

    bool resetChanged();

    bool resetActive() const;

    uint8_t getState(uint8_t index) const;
    void stm32_retry_state(uint8_t retry_state);

private:
    uint8_t stm32_s_[3];

    uint8_t stm32_rst_;
    uint8_t stm32_btn_;

    uint8_t last_btn_state_;
    uint8_t last_reset_signal_;
};

#endif