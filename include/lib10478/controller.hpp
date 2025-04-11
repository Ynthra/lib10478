#pragma once
#include "api.h"
#include "pros/misc.h"
#include "units/units.hpp"
#include <array>
#include <vector>

enum Buttons {
    L1 = 6,
    L2,
    R1,
    R2,
    UP,
    DOWN,
    LEFT,
    RIGHT,
    X,
    B,
    Y,
    A,
    POWER
};
enum Controllers{
    MASTER = 0,
    PARTNER
};
enum Sticks{
    LEFT_X = 0,
    LEFT_Y,
    RIGHT_X,
    RIGHT_Y
};

class Controller {
    private:
    class Button{
        private:
        bool prevPressing;
        const Buttons button;
        Controllers controller;

        uint32_t switchedTime = 0;
        public:
        bool pressing;
        bool pressed;
        bool released;
        uint32_t heldTimer;
        uint32_t releasedTimer;

        Button(Buttons button, Controllers controller = Controllers::MASTER)
        :button(button), controller(controller)
        {reset();}
        void reset(){
            this->prevPressing = false;
            this->pressing = false;
            this->pressed = false;
            this->released = false;
            this->heldTimer = 0;
            this->releasedTimer = 0;
        }
        void update(){
            pressing = pros::c::controller_get_digital(pros::controller_id_e_t(controller)
                                                        , pros::controller_digital_e_t(button));

            if(pressing != prevPressing){
                pressed = pressing;
                released = !pressing;
                switchedTime = pros::millis();
            }
            else{
                pressed = false;
                released = false;
            }

            if(pressing){
                heldTimer = pros::millis()-switchedTime;
            }
            else{
                releasedTimer = pros::millis()-switchedTime;     
            }

            prevPressing = pressing;
        }
    };
    std::vector<Button> buttons;
    Controllers controller;
    bool connected = true;

    void update() {
        bool connected = pros::c::controller_is_connected(pros::controller_id_e_t(this->controller));
        if (!connected) {
            if(this->connected){
                for (auto& button : buttons) {
                    button.reset();
                }
            }
    
        }
        else{
            for (auto& button : buttons) {
                button.update();
            }
        }
        this->connected = connected;
    }

    Controller(Controllers controller = Controllers::MASTER)
    : controller(controller)
    {
        for (int i = Buttons::L1; i <= Buttons::POWER; i++) {
            buttons.push_back(Button(Buttons(i), controller));
        }
    }

    public:

    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;



    const Button& operator[](Buttons button) const {
        return buttons[button - L1];
    }
    
    static Controller master;
    static Controller partner;

    static void updateAll() {
        master.update();
        partner.update();
    }
};

inline Controller Controller::master(Controllers::MASTER);
inline Controller Controller::partner(Controllers::PARTNER);