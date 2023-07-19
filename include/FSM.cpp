#include "FSM.hpp"

#include<iostream>

FSM::FSM(State* initialState) : currentState{initialState} {}

/**
 * @brief Get the current state of the FSM.
 */
State* FSM::returnState()
{
    return this->currentState;
}

/**
 * @brief Execute act() function from current state.
 */
void FSM::executeState()
{
    this->currentState->act();

    return;
}

/**
 * @brief Check if a condition for transition is true. If it is, the current state is changed.
 */
void FSM::changeState()
{
    for(auto i : this->currentState->transitions)
    {
        if(i.first())
        {
            this->currentState = i.second;
            break;
        }
    }

    return;
}

/**
 * @brief Call this function in the loop for checking transitions and executing actions.
 */
void FSM::executeFSM()
{
    this->changeState();
    this->executeState();
}