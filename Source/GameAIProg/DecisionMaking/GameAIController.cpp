#include "GameAIController.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "FSM/FSMComponent.h"
#include "Kismet/GameplayStatics.h" // Added for finding the player

AGameAIController::AGameAIController()
{
	PrimaryActorTick.bCanEverTick = true;
	BrainComponent = CreateDefaultSubobject<UFSMComponent>(TEXT("FSMComponent"));
}

void AGameAIController::BeginPlay()
{
	Super::BeginPlay();
	InitFiniteStateMachine();
}

void AGameAIController::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	APawn* MyPawn = GetPawn();
	// Try to find the ThiefActor from the blackboard
	AActor* Thief = Cast<AActor>(Blackboard->GetValueAsObject("ThiefActor"));

	if (MyPawn && Thief)
	{
		// 1. Calculate distance
		float Distance = FVector::Dist(MyPawn->GetActorLocation(), Thief->GetActorLocation());
        
		// 2. Check Line of Sight
		bool bHasLOS = LineOfSightTo(Thief);

		// 3. Update Blackboard (This triggers the FSM Transitions!)
		// Adjust 1000.0f to whatever detection radius you want
		bool bIsVisible = (Distance < 1000.0f && bHasLOS);
		Blackboard->SetValueAsBool("IsTargetVisible", bIsVisible);
        
		// Also keep track of our own location for the states to use
		Blackboard->SetValueAsVector("SelfLocation", MyPawn->GetActorLocation());
	}
}

void AGameAIController::InitFiniteStateMachine()
{
	UFSMComponent* FSMComp = FindComponentByClass<UFSMComponent>();
	if (ensure(FSMComp) && FSMBlackboardAsset)
	{
		// UseBlackboard initializes the component and the asset
		UBlackboardComponent* BBComp;
		if (UseBlackboard(FSMBlackboardAsset, BBComp))
		{
			Blackboard = BBComp;
		}
	}
}

void AGameAIController::RunFiniteStateMachine()
{
	if (UFSMComponent* FSMComp = Cast<UFSMComponent>(BrainComponent))
	{
		FSMComp->StartLogic();
	}
}