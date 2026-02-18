// Fill out your copyright notice in the Description page of Project Settings.

#include "SteeringAgent.h"

#include "AIController.h"


// Sets default values
ASteeringAgent::ASteeringAgent()
{
	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ASteeringAgent::BeginPlay()
{
	Super::BeginPlay();
}

void ASteeringAgent::BeginDestroy()
{
	Super::BeginDestroy();
}

// Called every frame
void ASteeringAgent::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (SteeringBehavior)
	{
		SteeringOutput Output = SteeringBehavior->CalculateSteering(DeltaTime, *this);
		//check if Output isn't empty but valid
		if (Output.IsValid)
		{
			//linear
			AddMovementInput(FVector{Output.LinearVelocity, 0.f});

			//angular
			if (!IsAutoOrienting())
			{
				if (AAIController* AIController = Cast<AAIController>(GetController()))
				{
					float const DeltaYaw = FMath::Clamp(Output.AngularVelocity, -1.f, 1.f) * DeltaTime *
						GetMaxAngularSpeed();

					FRotator const CurrentRotation{GetActorForwardVector().ToOrientationRotator()};
					FRotator const DeltaRotation{0, DeltaYaw, 0};
					FRotator const DesiredRotation{CurrentRotation + DeltaRotation};

					if (!FMath::IsNearlyEqual(CurrentRotation.Yaw, DesiredRotation.Yaw))
					{
						AIController->SetControlRotation(DesiredRotation);
						FaceRotation(DesiredRotation);
					}
				}
			}
		}
	}
}

// Called to bind functionality to input
void ASteeringAgent::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
}

void ASteeringAgent::SetSteeringBehavior(ISteeringBehavior* NewSteeringBehavior)
{
	SteeringBehavior = NewSteeringBehavior;
}
