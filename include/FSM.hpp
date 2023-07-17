#ifndef FSM_HPP
#define FSM_HPP

#include "State.hpp"
#include<iostream>
#include<vector>
#include<functional>

#define EDGE(typeS1, instanceS1, typeS2, instanceS2) this->instanceS1.addTransition(std::bind(&state##typeS1::to_state##typeS2, &(this->instanceS1)), &(this->instanceS2))

class FSM
{
private:
    State* currentState;
    void executeState();
    void changeState();
    virtual bool isFinished() = 0;
protected:
    State* returnState();
public:
    void executeFSM();

    FSM(State* initialState); 
};

#endif