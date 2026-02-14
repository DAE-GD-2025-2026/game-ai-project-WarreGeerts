#include "SteeringBehaviors.h"

#include <ThirdParty/hlslcc/hlslcc/src/hlslcc_lib/ir_hierarchical_visitor.h>

#include "Dataflow/DataflowEngineUtil.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//DEFAULT CLASS
//*******
void ISteeringBehavior::DebugLines(ASteeringAgent& Agent, FVector2D& LinearVelocity)
{
	const FVector Agent3D{Agent.GetPosition(), 0.0f};

	constexpr float LineLength = 300.f;
	const FVector Velocity3D{LinearVelocity, 0.0f};

	const FVector ToEnd = Velocity3D;
	const FVector Dir = ToEnd.GetSafeNormal();
	const FVector FixedEnd = Agent3D + Dir * LineLength;
	DrawDebugLine(Agent.GetWorld(), Agent3D, FixedEnd, FColor::Green);

	const float AgentDegrees{Agent.GetRotation()};
	const FRotator AgentRotation{0.f, AgentDegrees, 0.f};
	const FVector AgentForward{AgentRotation.Vector()};
	const FVector VelDir{AgentForward};
	const FVector VelEnd{Agent3D + VelDir * LineLength};
	DrawDebugLine(Agent.GetWorld(), Agent3D, VelEnd, FColor::Purple);

	DrawDebugCircle(Agent.GetWorld(), FVector(Target.Position, 0.0f), 20, 10, FColor::Red,
	                false, -1, 0, 0, FVector(1, 0, 0),
	                FVector(0, 1, 0));
}

// TODO: Do the Week01 assignment :^)
//SEEK
//*******
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	Steering.LinearVelocity = Target.Position - Agent.GetPosition();

	DebugLines(Agent, Steering.LinearVelocity);

	return Steering;
}

//FLEE
//*******
SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	Steering.LinearVelocity *= -1;

	DebugLines(Agent, Steering.LinearVelocity);

	return Steering;
}

//ARRIVE
//*******
SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	if (!MaxSpeedSet)
	{
		MaxSpeed = Agent.GetMaxLinearSpeed();
		MaxSpeedSet = true;
	}

	float CurrentSpeed{Agent.GetMaxLinearSpeed()};
	constexpr float SlowRadius{500};
	constexpr float TargetRadius{100};

	DrawDebugCircle(Agent.GetWorld(), FVector(Agent.GetPosition(), 0.0f), SlowRadius, 20, FColor::Blue,
	                false, -1, 0, 0, FVector(1, 0, 0),
	                FVector(0, 1, 0));
	DrawDebugCircle(Agent.GetWorld(), FVector(Agent.GetPosition(), 0.0f), TargetRadius, 20, FColor::Orange,
	                false, -1, 0, 0, FVector(1, 0, 0),
	                FVector(0, 1, 0));

	if (FVector2D::Distance(Agent.GetPosition(), Target.Position) < TargetRadius)
	{
		//set speed to 0
		CurrentSpeed = 0.f;
	}
	else if (FVector2D::Distance(Agent.GetPosition(), Target.Position) <= SlowRadius)
	{
		//slow down with distance
		const float Distance{static_cast<float>(FVector2D::Distance(Agent.GetPosition(), Target.Position))};
		//distance / total distance
		const float DistancePercentile{(Distance - TargetRadius) / SlowRadius};
		const float Speed{MaxSpeed * DistancePercentile};
		CurrentSpeed = Speed;
	}
	else
	{
		//MaxSpeed
		CurrentSpeed = MaxSpeed;
	}

	Agent.SetMaxLinearSpeed(CurrentSpeed);
	SteeringOutput Steering{Seek::CalculateSteering(DeltaT, Agent)};
	return Steering;
}
