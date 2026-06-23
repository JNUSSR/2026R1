#pragma once

#include "planned_joint.h"
#include "slope_joint.h"
#include "cylinder.h"
#include "main.h"


/*========== Core Class =============*/
class KFS_Arm {
public:
    KFS_Arm(SlopeJoint& joint_h, SlopeJoint& joint_v, Cylinder& claw)
    : joint_h(joint_h)
    , joint_v(joint_v)
    , claw_(claw)
    {};

    void reachOutTo(float target, float duration = 2.0f);
    void moveVerticallyTo(float target);
    void releaseClaw();
    void tightenClaw();
    void stopJointV();

    bool isJointVMoving()        { return joint_v.IsMoving(); }
    void setJointVMaxSpeed(float speed) { joint_v.setMaxSpeed(speed); }
    void resumeJointVMaxSpeed()         { joint_v.resumeDefaultSpeed(); }

    float getJointVCurPos()    { return joint_v.getCurrentTarget(); }
    float getJointVCurTorque() { return joint_v.getCurrentTorque(); }

private:
    SlopeJoint&     joint_h;
    SlopeJoint&     joint_v;
    Cylinder&       claw_;
};


