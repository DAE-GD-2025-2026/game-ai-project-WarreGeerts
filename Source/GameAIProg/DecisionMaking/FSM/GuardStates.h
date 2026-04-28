#pragma once
#include "FSM.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"

class GuardStateBase : public GameAI::FSM::State {
protected:
    ASteeringAgent* Agent;
    UBlackboardComponent* BB;
public:
    GuardStateBase(ASteeringAgent* InAgent, UBlackboardComponent* InBB) : Agent(InAgent), BB(InBB) {}
};

// PATROL STATE
class PatrolState : public GuardStateBase {
    TArray<FVector> Path;
    int32 CurrentIdx = 0;
    std::unique_ptr<Arrive> ArriveBehavior; // Use Arrive for smooth pathing

public:
    PatrolState(ASteeringAgent* A, UBlackboardComponent* B, TArray<FVector> P) 
        : GuardStateBase(A, B), Path(P) 
    {
        ArriveBehavior = std::make_unique<Arrive>();
        ArriveBehavior->SetTargetRadius(100.0f);
    }

    virtual void OnEnter() override {
        Agent->SetSteeringBehavior(ArriveBehavior.get());
    }
    
    virtual void OnUpdate(float DeltaTime) override {
        if (Path.Num() == 0) return;

        FVector Target3D = Path[CurrentIdx];
        // Set the target for the behavior
        ArriveBehavior->SetTarget(FTargetData{FVector2D(Target3D.X, Target3D.Y)});

        if (FVector::Dist(Agent->GetActorLocation(), Target3D) < 150.f) {
            CurrentIdx = (CurrentIdx + 1) % Path.Num();
        }
    }
};

// CHASE STATE
class ChaseState : public GuardStateBase {
    std::unique_ptr<Seek> SeekBehavior;

public:
    ChaseState(ASteeringAgent* A, UBlackboardComponent* B) : GuardStateBase(A, B) {
        SeekBehavior = std::make_unique<Seek>();
    }

    virtual void OnEnter() override {
        Agent->SetSteeringBehavior(SeekBehavior.get());
    }
    
    virtual void OnUpdate(float DeltaTime) override {
        AActor* Thief = Cast<AActor>(BB->GetValueAsObject("ThiefActor"));
        if (Thief) {
            FVector ThiefLoc = Thief->GetActorLocation();
            SeekBehavior->SetTarget(FTargetData{FVector2D(ThiefLoc.X, ThiefLoc.Y)});
            
            // Remember where we last saw them for the Search state
            BB->SetValueAsVector("LastSpottedLocation", ThiefLoc);
        }
    }
};

// SEARCH STATE
class SearchState : public GuardStateBase {
    std::unique_ptr<Arrive> ArriveBehavior;
    std::unique_ptr<Wander> WanderBehavior;
    float Timer = 0.0f;
    bool bReachedLastLocation = false;

public:
    SearchState(ASteeringAgent* A, UBlackboardComponent* B) : GuardStateBase(A, B) {
        ArriveBehavior = std::make_unique<Arrive>();
        WanderBehavior = std::make_unique<Wander>();
    }
    
    virtual void OnEnter() override { 
        Timer = 0.0f; 
        bReachedLastLocation = false;
        Agent->SetSteeringBehavior(ArriveBehavior.get());
    }
    
    virtual void OnUpdate(float DeltaTime) override {
        Timer += DeltaTime;
        BB->SetValueAsFloat("SearchTimer", Timer);
        
        FVector LastLoc3D = BB->GetValueAsVector("LastSpottedLocation");

        if (!bReachedLastLocation) {
            ArriveBehavior->SetTarget(FTargetData{FVector2D(LastLoc3D.X, LastLoc3D.Y)});
            
            if (FVector::Dist(Agent->GetActorLocation(), LastLoc3D) < 150.f) {
                bReachedLastLocation = true;
                Agent->SetSteeringBehavior(WanderBehavior.get());
            }
        }
        // If bReachedLastLocation is true, the Wander behavior is already active on the agent
    }
};