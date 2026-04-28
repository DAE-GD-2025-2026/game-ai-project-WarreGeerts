// Fill out your copyright notice in the Description page of Project Settings.


#include "FSMComponent.h"
#include "FSM.h"

// Sets default values for this component's properties
UFSMComponent::UFSMComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// TODO Setup FSM
	FSMInstance = std::make_unique<GameAI::FSM::FSM>();
}

UFSMComponent::~UFSMComponent() = default;

UFSMComponent::UFSMComponent(FVTableHelper& Helper) : Super(Helper) {}


void UFSMComponent::AddState(std::unique_ptr<GameAI::FSM::State>&& NewState)
{
	// TODO
	FSMInstance->AddState(std::move(NewState));
}

void UFSMComponent::AddTransition(GameAI::FSM::State* From, GameAI::FSM::State* To, std::function<bool()> EvalFunc)
{
	// TODO
	FSMInstance->AddTransition(From, To, std::move(EvalFunc));
}

// Called when the game starts
void UFSMComponent::BeginPlay()
{
	Super::BeginPlay();
}


// Called every frame
void UFSMComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	// TODO
	if (bIsRunning && FSMInstance)
	{
		FSMInstance->Update(DeltaTime);
	}
}

void UFSMComponent::StartLogic()
{
	Super::StartLogic();

	// TODO
	bIsRunning = true;
	FSMInstance->Start(); // starts from whichever state was set as initial
}

void UFSMComponent::StopLogic(const FString& Reason)
{
	// TODO
	bIsRunning = false;
	FSMInstance->Stop();
}

bool UFSMComponent::IsRunning() const
{
	return bIsRunning;
}

