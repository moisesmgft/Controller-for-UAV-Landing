#ifndef STATE_CPP
#define STATE_CPP

#include "State.hpp"
#include<iostream>
#include<utility>
#include<vector>
#include<functional>

/**
 * @brief Add a transition. A transition is defined as following:
 * It has a condition and a next state. If the condition is met,
 * the FSM changes to the corresponding state.
 */
void State::addTransition(std::function<bool(void)> funcTransition, State* nextState)
{
    this->transitions.push_back(std::make_pair(funcTransition, nextState));

    return;
}

#endif