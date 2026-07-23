#include "GameStateMachine.h"

GameStateMachine::GameStateMachine()
{
    Reset();
}

void GameStateMachine::Reset()
{
    mState = Idle;
    mBlocksInState = 0;
}

void GameStateMachine::Enter(State state)
{
    mState = state;
    mBlocksInState = 0;
}

GameStateMachine::State GameStateMachine::Current() const
{
    return mState;
}

int GameStateMachine::BlocksInState() const
{
    return mBlocksInState;
}

void GameStateMachine::AdvanceBlock()
{
    ++mBlocksInState;
}
