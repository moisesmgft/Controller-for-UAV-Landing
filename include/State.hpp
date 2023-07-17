#ifndef STATE_HPP
#define STATE_HPP

#include<iostream>
#include<vector>
#include<functional>

class State
{
private:
    std::vector<std::pair <std::function<bool(void)>, State*>> transitions;
    virtual void act() = 0;
public:
    void addTransition(std::function<bool(void)>, State*); 

    friend class FSM;
};

#endif