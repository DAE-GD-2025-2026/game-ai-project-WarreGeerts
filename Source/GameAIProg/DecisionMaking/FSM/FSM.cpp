#include "FSM.h"

void GameAI::FSM::State::OnEnter()
{
}

void GameAI::FSM::State::OnUpdate(float DeltaTime)
{
}

void GameAI::FSM::State::OnExit()
{
}

GameAI::FSM::Transition::Transition(State* InFrom, State* InTo, std::function<bool()> InCondition) : From(InFrom),
	To(InTo), Condition(std::move(InCondition))
{
}

void GameAI::FSM::FSM::AddState(std::unique_ptr<State>&& NewState)
{
	States.push_back(std::move(NewState));
}

void GameAI::FSM::FSM::AddTransition(State* From, State* To, std::function<bool()> Condition)
{
	auto T = std::make_unique<Transition>(From, To, std::move(Condition));
	From->OutgoingTransitions.push_back(T.get());
	Transitions.push_back(std::move(T));
}

void GameAI::FSM::FSM::SetInitialState(State* InitialState)
{
	CurrentState = InitialState;
}

void GameAI::FSM::FSM::Start()
{
	// If no initial state was manually set, default to the first one added
	if (!CurrentState && !States.empty())
	{
		CurrentState = States[0].get();
	}

	if (CurrentState)
		CurrentState->OnEnter();
}

void GameAI::FSM::FSM::Stop()
{
	if (CurrentState)
	{
		CurrentState->OnExit();
		CurrentState = nullptr;
	}
}

void GameAI::FSM::FSM::Update(float DeltaTime)
{
	if (!CurrentState) return;

	for (Transition* T : CurrentState->OutgoingTransitions)
	{
		if (T->Condition())
		{
			CurrentState->OnExit();
			CurrentState = T->To;
			CurrentState->OnEnter();
			return; // only one transition per tick
		}
	}

	CurrentState->OnUpdate(DeltaTime);
}
