#pragma once
#include "api.h"

namespace lib10478 
{

//credit to https://github.com/Ryan4253/ryanlib/blob/main/include/ryanlib/StateMachine.hpp
template<typename State, State initState>
class StateMachine{
    private:
    State state = initState;
    pros::Mutex stateLock;

    public:
    /**
     * @brief Construct a new State Machine object
     * 
     */
    StateMachine(){}

    /**
     * @brief Get the current state
     * 
     * @return State the current state
     */
    State getState(){
        stateLock.take();
        State currentState = state;
        stateLock.give();
        return currentState;
    }

    /**
     * @brief Set the current state
     * 
     * @param iState the new state
     */
    void setState(State iState){
        stateLock.take();
        state = iState;
        stateLock.give();
    }
};

} // namespace lib10478