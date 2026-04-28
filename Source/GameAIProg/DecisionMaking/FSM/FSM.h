#pragma once
#include <functional>
#include <memory>
#include <vector>

namespace GameAI::FSM
{
	class State
	{
	public:
		virtual ~State() = default;

		virtual void OnEnter();
		virtual void OnUpdate(float DeltaTime);
		virtual void OnExit();

		// Outgoing transitions stored directly on the state
		std::vector<class Transition*> OutgoingTransitions;
	};

	class Transition
	{
	public:
		State* From = nullptr;
		State* To = nullptr;
		std::function<bool()> Condition;

		Transition(State* InFrom, State* InTo, std::function<bool()> InCondition);
	};

	class FSM
	{
	public:
		void AddState(std::unique_ptr<State>&& NewState);

		void AddTransition(State* From, State* To, std::function<bool()> Condition);

		void SetInitialState(State* InitialState);

		void Start();

		void Stop();

		void Update(float DeltaTime);

		State* GetCurrentState() const { return CurrentState; }

	private:
		std::vector<std::unique_ptr<State>> States;
		std::vector<std::unique_ptr<Transition>> Transitions;
		State* CurrentState = nullptr;
	};
	
	
	
	
}
