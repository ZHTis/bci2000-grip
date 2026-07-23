#include "FlightPhysics.h"

#include <algorithm>
#include <cmath>

FlightPhysics::FlightPhysics()
    : mBody{0, 0, 0}, mForwardSpeed(25), mLiftGain(70), mGravity(30), mDamping(0.98f)
{
}

void FlightPhysics::Configure(float forwardSpeed, float liftGain, float gravity, float damping)
{
    mForwardSpeed = forwardSpeed;
    mLiftGain = liftGain;
    mGravity = gravity;
    mDamping = damping;
}

void FlightPhysics::Reset(float x, float y)
{
    mBody = {x, y, 0};
}

void FlightPhysics::Step(float normalizedGrip, float dt)
{
    mBody.x += mForwardSpeed * dt;
    mBody.velocityY += (mLiftGain * normalizedGrip - mGravity) * dt;
    mBody.velocityY *= std::pow(std::max(0.0f, std::min(1.0f, mDamping)), dt * 60.0f);
    mBody.y += mBody.velocityY * dt;
}

const FlightBody& FlightPhysics::Body() const
{
    return mBody;
}
