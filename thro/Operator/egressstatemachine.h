/*!
 This class represents an egress car state machine for
 operator console behavior.
 Follows the State Pattern.
 http://www.wikipedia.com/wiki/State_Pattern

 initial class
 @author jdavis
 @date 02/21/2013

 */
#ifndef EGRESSSTATEMACHINE_H
#define EGRESSSTATEMACHINE_H

#include "behaviorstatemachine.h"

class EgressStateMachine : public BehaviorStateMachine
{
public:
    EgressStateMachine();
};

#endif // EGRESSSTATEMACHINE_H
