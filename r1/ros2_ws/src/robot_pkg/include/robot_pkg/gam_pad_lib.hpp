#pragma once
#include <cstdint>
#include <cstring>

// enum X5_ButtonMask : uint32_t {
//     X5_A      = (1u << 0),
//     X5_B      = (1u << 1),
//     X5_X      = (1u << 2),
//     X5_Y      = (1u << 3),
//     X5_UP     = (1u << 4),
//     X5_DOWN   = (1u << 5),
//     X5_LEFT   = (1u << 6),
//     X5_RIGHT  = (1u << 7),
//     X5_LB     = (1u << 8),
//     X5_RB     = (1u << 9),
//     X5_LT     = (1u << 10),
//     X5_RT     = (1u << 11),
//     X5_L3     = (1u << 12),
//     X5_R3     = (1u << 13),
//     X5_SHERE  = (1u << 14),
//     X5_CHIKEN = (1u << 15),
//     X5_MENU   = (1u << 16),
// };
enum X5_ButtonMask 
    {
        X5_A      = (1 << 0),
        X5_B      = (1 << 1),
        X5_X      = (1 << 2),
        X5_Y      = (1 << 3),
        X5_SHERE  = (1 << 4),
        X5_CHIKEN = (1 << 5),
        X5_MENU   = (1 << 6),
        X5_L3     = (1 << 7),
        X5_R3     = (1 << 8),
        X5_LB     = (1 << 9),
        X5_RB     = (1 << 10),
        X5_UP     = (1 << 11),
        X5_DOWN   = (1 << 12),
        X5_LEFT   = (1 << 13),
        X5_RIGHT  = (1 << 14),
        X5_LT     = (1 << 15),
        X5_RT     = (1 << 16)
    };

enum X5_MeihuaMask : uint32_t {
    B0 =(1<<0),
    B1 =(1<<1),
    B2 =(1<<2),
    B3 =(1<<3),
    B4 =(1<<4),
    B5 =(1<<5),
    B6 =(1<<6),
    B7 = (1<<7),
    B8 =(1<<8),
    B9 =(1<<9),
    B10 =(1<<10),
    B11 =(1<<11),
    A600 =(1<<12),
    A400 =(1<<13),
    Home =(1<<14),
    chancel =(1<<15),
    A200   =(1<<16),
    Menual =(1<<17),
    Zone3  =(1<<18),    
    Auto   =(1<<19),
};
struct X5_Button {
    bool a, b, x, y;
    bool up, down, left, right;
    bool lb, rb, lt, rt;
    bool l3, r3;
    bool shere, chiken, menu;
};

class gamepadlib {
public:
    uint32_t getRawButton() const { return state_btn_; }
    uint32_t getRawMeihua() const { return state_mei_; }
    void setButton(uint32_t button) {
        state_btn_ = button;
       // updateButtons();        // ← update struct right away
        //return state_btn_;
    }

    void setMeihua(uint32_t button) {  // ← uint16_t not uint32_t
        state_mei_ = button;
       // updateMeihua();         // ← update st
       
    //    struct right away
        //return state_mei_;
    }

    // Check by bitmask
    bool isPressed(uint32_t mask) const { return (state_btn_ & mask) == mask; }
    bool isMeihua (uint32_t mask) const { return (state_mei_ & mask)== mask; }

    // // Struct style
    // const X5_Button& buttons() const { return btn_;    }
    // const X5_Meihua& meihua()  const { return meihua_; }

private:
    uint32_t  state_btn_ = 0;
    uint32_t  state_mei_ = 0;   

//     X5_Button btn_    = {};
//    // X5_Meihua meihua_ = {};

//     void updateButtons();
//     void updateMeihua();
};

extern gamepadlib x5;