#ifndef GRIP_FLIGHT_GAME_STATE_MACHINE_H
#define GRIP_FLIGHT_GAME_STATE_MACHINE_H

class GameStateMachine
{
  public:
    enum State
    {
        Idle = 0,
        Countdown = 1,
        Playing = 2,
        Hit = 3,
        TrialSuccess = 4,
        TrialFailure = 5,
        InterTrial = 6,
        SessionComplete = 7,
    };

    GameStateMachine();
    void Reset();
    void Enter(State);
    State Current() const;
    int BlocksInState() const;
    void AdvanceBlock();

  private:
    State mState;
    int mBlocksInState;
};

#endif
