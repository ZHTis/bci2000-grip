#ifndef GRIP_FLIGHT_PHYSICS_H
#define GRIP_FLIGHT_PHYSICS_H

struct FlightBody
{
    float x;
    float y;
    float velocityY;
};

class FlightPhysics
{
  public:
    FlightPhysics();
    void Configure(float forwardSpeed, float liftGain, float gravity, float damping);
    void Reset(float x, float y);
    void Step(float normalizedGrip, float dt);
    const FlightBody& Body() const;

  private:
    FlightBody mBody;
    float mForwardSpeed;
    float mLiftGain;
    float mGravity;
    float mDamping;
};

#endif
