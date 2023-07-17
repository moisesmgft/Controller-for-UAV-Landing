#include "FSM.hpp"

#include<iostream>

FSM::FSM(State* initialState) : currentState{initialState} {}

State* FSM::returnState()
{
    return this->currentState;
}

void FSM::executeState()
{
    this->currentState->act();

    return;
}

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

void FSM::executeFSM()
{
    this->changeState();
    this->executeState();
}