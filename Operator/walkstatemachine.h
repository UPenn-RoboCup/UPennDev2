/*!
 This class represents an locomotion state machine for
 operator console behavior.
 Follows the State Pattern.
 http://www.wikipedia.com/wiki/State_Pattern

 @author jdavis
 @date 02/21/2013

 */
#ifndef WALKSTATEMACHINE_H
#define WALKSTATEMACHINE_H

#include "behaviorstatemachine.h"

class WalkStateMachine : public BehaviorStateMachine
{
public:
    WalkStateMachine();
};

#endif // WALKSTATEMACHINE_H
