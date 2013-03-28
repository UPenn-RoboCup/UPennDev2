/*!
    This class is the parent class of the behavior state machines.
    A behavior is robot behaviors such as ingress, climb ladder, etc.
    The behavior state machine at the operator station manage operator
    control states.  States machines will be driven by events from THOR
    and user interface events, and will drive changes in operator control
    configuration.

    The child behavior state machines will implement
    the State Design pattern.

    http://www.wikipedia.com/wiki/State_Pattern

    initial class
    @author jdavis
    @date 02/21/2013

 */

//TODO initial code

#ifndef BEHAVIORSTATEMACHINE_H
#define BEHAVIORSTATEMACHINE_H

class BehaviorStateMachine
{
public:
    BehaviorStateMachine();
};

#endif // BEHAVIORSTATEMACHINE_H
