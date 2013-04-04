/*!
 This class represents an ingress vehicle state machine for
 operator console behavior.
 Follow the State Pattern.
 http://www.wikipedia.com/wiki/State_Pattern

 @author jdavis
 @date 02/21/2013

 */
#ifndef INGRESSSTATEMACHINE_H
#define INGRESSSTATEMACHINE_H

#include "behaviorstatemachine.h"

class IngressStateMachine : public BehaviorStateMachine
{
public:
    IngressStateMachine();
};

#endif // INGRESSSTATEMACHINE_H
