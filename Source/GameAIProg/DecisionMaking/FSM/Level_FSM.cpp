// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_FSM.h"

#include "FSMComponent.h"
#include "GuardStates.h"
#include "DecisionMaking/GameAIController.h"


// Sets default values
ALevel_FSM::ALevel_FSM()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_FSM::BeginPlay()
{
    Super::BeginPlay();

    // 1. Spawn Thief (The player-controlled agent)
    // We spawn him at a different location so they don't collide immediately
    ASteeringAgent* Thief = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, 
        FVector{500, 500, 90}, FRotator::ZeroRotator);
    

    if (Thief)
    {
        GetWorld()->GetFirstPlayerController()->Possess(Thief);
    }
    
    // 2. Spawn Guard
    Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, 
        FVector{0, 0, 90}, FRotator::ZeroRotator);
    
    if (AGameAIController* AIController = Cast<AGameAIController>(Agent->GetController()))
    {
        UFSMComponent* FSMComp = Cast<UFSMComponent>(AIController->GetBrainComponent());
        UBlackboardComponent* BB = AIController->GetBlackboardComponent();
        
        if (FSMComp && BB && Thief)
        {
            // IMPORTANT: Pass the Thief to the Blackboard
            BB->SetValueAsObject("ThiefActor", Thief);

            // Define Patrol Path
            TArray<FVector> PatrolPoints = { FVector(1000, 0, 90), FVector(1000, 1000, 90), FVector(0, 1000, 90) };

            // Create States
            auto Patrol = std::make_unique<PatrolState>(Agent, BB, PatrolPoints);
            auto Chase = std::make_unique<ChaseState>(Agent, BB);
            auto Search = std::make_unique<SearchState>(Agent, BB);

            auto* PatrolPtr = Patrol.get();
            auto* ChasePtr = Chase.get();
            auto* SearchPtr = Search.get();

            // Add States and Transitions
            FSMComp->AddState(std::move(Patrol));
            FSMComp->AddState(std::move(Chase));
            FSMComp->AddState(std::move(Search));

            // Wiring (Same as your current code)
            FSMComp->AddTransition(PatrolPtr, ChasePtr, [BB]() { return BB->GetValueAsBool("IsTargetVisible"); });
            FSMComp->AddTransition(ChasePtr, SearchPtr, [BB]() { return !BB->GetValueAsBool("IsTargetVisible"); });
            FSMComp->AddTransition(SearchPtr, ChasePtr, [BB]() { return BB->GetValueAsBool("IsTargetVisible"); });
            FSMComp->AddTransition(SearchPtr, PatrolPtr, [BB]() { return BB->GetValueAsFloat("SearchTimer") > 5.0f; });

            // 3. SET INITIAL STATE 
            // You need to make sure FSMInstance->SetInitialState(PatrolPtr) is called 
            // inside your UFSMComponent or FSM class logic!
            
            AIController->RunFiniteStateMachine();
        }
    }
}

// Called every frame
void ALevel_FSM::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

