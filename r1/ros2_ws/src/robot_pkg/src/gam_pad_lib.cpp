#include "robot_pkg/gam_pad_lib.hpp"

gamepadlib x5;

// void gamepadlib::updateButtons()
// {
//     btn_ = {
//         .a      = !!(state_btn_ & X5_A),
//         .b      = !!(state_btn_ & X5_B),
//         .x      = !!(state_btn_ & X5_X),
//         .y      = !!(state_btn_ & X5_Y),
//         .up     = !!(state_btn_ & X5_UP),
//         .down   = !!(state_btn_ & X5_DOWN),
//         .left   = !!(state_btn_ & X5_LEFT),
//         .right  = !!(state_btn_ & X5_RIGHT),
//         .lb     = !!(state_btn_ & X5_LB),
//         .rb     = !!(state_btn_ & X5_RB),
//         .lt     = !!(state_btn_ & X5_LT),
//         .rt     = !!(state_btn_ & X5_RT),
//         .l3     = !!(state_btn_ & X5_L3),
//         .r3     = !!(state_btn_ & X5_R3),
//         .shere  = !!(state_btn_ & X5_SHERE),
//         .chiken = !!(state_btn_ & X5_CHIKEN),
//         .menu   = !!(state_btn_ & X5_MENU),
//     };
// }

// void gamepadlib::updateMeihua()
// {
//     meihua_ = {
//         .b0      = !(state_mei_ & B1),
//         .b1      = !(state_mei_ & B2),
//         .b2      = !(state_mei_ & B3),
//         .b3      = !(state_mei_ & B4),
//         .b4      = !(state_mei_ & B5),
//         .b5      = !(state_mei_ & B6),
//         .b6      = !(state_mei_ & B7),
//         .b7      = !(state_mei_ & B8),
//         .b8      = !(state_mei_ & B9),
//         .b9      = !(state_mei_ & B10),
//         .b10     = !(state_mei_ & B11),
//         .b11     = !(state_mei_ & B12),
//         .start_b = !(state_mei_ & Start_b),
//         .retry_b = !(state_mei_ & Retry_b),
//         .start_r = !(state_mei_ & Start_r),
//         .retry_r = !(state_mei_ & Retry_r),
//     };
// }