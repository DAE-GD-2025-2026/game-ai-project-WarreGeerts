#include "SteeringBehaviors.h"

#include "Dataflow/DataflowEngineUtil.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//DEFAULT CLASS
//*******
void ISteeringBehavior::DebugLines(ASteeringAgent& Agent, FVector2D& LinearVelocity)
{
	FVector Target3D{Target.Position, 0.0f};
	FVector Agent3D{Agent.GetPosition(), 0.0f};

	constexpr float LineLength = 300.f;
	const FVector Velocity3D{LinearVelocity, 0.0f};
	
	FVector ToEnd = Velocity3D;
	FVector Dir = ToEnd.GetSafeNormal();
	FVector FixedEnd = Agent3D + Dir * LineLength;
	DrawDebugLine(Agent.GetWorld(), Agent3D, FixedEnd, FColor::Green);

	const float AgentDegrees{Agent.GetRotation()};
	const FRotator AgentRotation{0.f, AgentDegrees, 0.f};
	const FVector AgentForward{AgentRotation.Vector()};
	const FVector VelDir{AgentForward};
	const FVector VelEnd{Agent3D + VelDir * LineLength};
	DrawDebugLine(Agent.GetWorld(), Agent3D, VelEnd, FColor::Purple);

	DrawDebugCircle(Agent.GetWorld(), Target3D, 20, 10, FColor::Red, false,
	                -1, 0, 0, FVector(1, 0, 0),
	                FVector(0, 1, 0));
}

//SEEK
//*******
// TODO: Do the Week01 assignment :^)
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	Steering.LinearVelocity = Target.Position - Agent.GetPosition();

	DebugLines(Agent, Steering.LinearVelocity);

	return Steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{Seek::CalculateSteering(DeltaT, Agent)};
	Steering.LinearVelocity *= -1;

	DebugLines(Agent, Steering.LinearVelocity);

	return Steering;
}
