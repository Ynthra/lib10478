#pragma once
#include "api.h"
#include "pros/misc.h"

namespace controller {
    inline pros::Controller master(pros::E_CONTROLLER_MASTER);
    class Button{
        private:
        bool prevpressing = false;
        const pros::controller_digital_e_t button;

        int switchedTime = 0;
        public:
        bool pressing = false;
        bool pressed = false;
        bool released = false;
        int heldTimer = 0;
        int releasedTimer = 0;

        Button(pros::controller_digital_e_t button)
        :button(button)
        {
        }
        void update(){
            pressing = master.get_digital(button);

            if(pressing != prevpressing){
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

            prevpressing = pressing;
        }
    };
    inline Button R1(pros::E_CONTROLLER_DIGITAL_R1);
    inline Button R2(pros::E_CONTROLLER_DIGITAL_R2);
    inline Button L1(pros::E_CONTROLLER_DIGITAL_L1);
    inline Button L2(pros::E_CONTROLLER_DIGITAL_L2);
    inline Button X(pros::E_CONTROLLER_DIGITAL_X);
    inline Button Y(pros::E_CONTROLLER_DIGITAL_Y);
    inline Button A(pros::E_CONTROLLER_DIGITAL_A);
    inline Button B(pros::E_CONTROLLER_DIGITAL_B);
    inline Button Up(pros::E_CONTROLLER_DIGITAL_UP);
    inline Button Down(pros::E_CONTROLLER_DIGITAL_DOWN);
    inline Button Left(pros::E_CONTROLLER_DIGITAL_LEFT);
    inline Button Right(pros::E_CONTROLLER_DIGITAL_RIGHT);

    inline void update(){
        R2.update();
        L1.update();
        L2.update();
        R1.update();
        Up.update();
        A.update();
        X.update();
        Y.update();  
        B.update();
        Right.update();
        Left.update();
        Down.update();
    }
    
    inline float driveCurve(float input, float deadzone, float scale, float maxjoy, float minvolt, float maxvolt) {
        if(fabs(input)>deadzone){
            return (input *
        (powf(2.718, -(scale / (0.1* maxjoy))) + powf(2.718, (fabs(input) - maxjoy) / (0.1*maxjoy)) * (1 - powf(2.718, -(scale / (0.1*maxjoy))))) 
        *(1-(minvolt/maxvolt)) * (maxvolt/maxjoy) + minvolt * (input<0?-1:1));
        }
        else{
            return 0;
        }
    }
}