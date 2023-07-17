#ifndef STATE_CPP
#define STATE_CPP

#include "State.hpp"
#include<iostream>
#include<utility>
#include<vector>
#include<functional>

void State::addTransition(std::function<bool(void)> funcTransition, State* nextState)
{
    this->transitions.push_back(std::make_pair(funcTransition, nextState));

    return;
}

#endif