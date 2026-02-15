#include "SteeringBehaviors.h"

#include <ThirdParty/hlslcc/hlslcc/src/hlslcc_lib/ir_hierarchical_visitor.h>

#include "Dataflow/DataflowEngineUtil.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

bool ISteeringBehavior::sm_MaxSpeedSet{false};
float ISteeringBehavior::m_MaxSpeed{0};

SteeringOutput ISteeringBehavior::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	SetMaxSpeed(Agent);

	if (!m_MaxSpeedReset)
	{
		Agent.SetMaxLinearSpeed(m_MaxSpeed);
		m_MaxSpeedReset = true;
	}
	return Steering;
}

void ISteeringBehavior::SetMaxSpeed(const ASteeringAgent& Agent)
{
	if (!sm_MaxSpeedSet)
	{
		m_MaxSpeed = Agent.GetMaxLinearSpeed();
		sm_MaxSpeedSet = true;
	}
}

//DEFAULT CLASS
//*******
void ISteeringBehavior::DebugLines(ASteeringAgent& Agent, FVector2D& LinearVelocity)
{
	const FVector Agent3D{Agent.GetPosition(), 0.0f};
	const FVector Velocity3D{LinearVelocity, 0.0f};

	//--point to end
	const FVector ToTarget{FVector(m_Target.Position, 0.0f) - Agent3D};
	const FVector Dir{ToTarget.GetSafeNormal()};
	constexpr float LineLength{200.f};
	const FVector FixedEnd{Agent3D + Dir * LineLength};

	DrawDebugLine(Agent.GetWorld(), Agent3D, FixedEnd, FColor::Green);

	//--Linear velocity facing out from front of Agent
	constexpr float MaxLineLength{300.f};
	constexpr float MaxSpeedForViz{600.f};
	const float Speed{static_cast<float>(Velocity3D.Size())};

	//Get length based on speed
	const float LengthFactor{FMath::Clamp(Speed / MaxSpeedForViz, 0.f, 1.f)};
	const float LineLengthLin{MaxLineLength * LengthFactor};

	//Get front facing direction of Agent
	const float AgentDegrees{Agent.GetRotation()};
	const FRotator AgentRotation{0.f, AgentDegrees, 0.f};
	const FVector AgentForward{AgentRotation.Vector()};
	//Calc end point depending on rotation of Agent
	const FVector VelEnd{Agent3D + AgentForward * LineLengthLin};
	DrawDebugLine(Agent.GetWorld(), Agent3D, VelEnd, FColor::Magenta);

	//--target point
	DrawDebugCircle(Agent.GetWorld(), FVector(m_Target.Position, 0.0f), 20, 10, FColor::Red,
	                false, -1, 0, 0, FVector(1, 0, 0),
	                FVector(0, 1, 0));
}


//TODO: Do the Week01 assignment :^)
//SEEK
//*******
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	ISteeringBehavior::CalculateSteering(DeltaT, Agent);
	SteeringOutput Steering{};

	FVector2D CurrentVelocity{Agent.GetVelocity()};

	FVector2D ToTarget{m_Target.Position - Agent.GetPosition()};
	FVector2D DesiredVelocity{ToTarget.GetSafeNormal() * Agent.GetMaxLinearSpeed()};

	FVector2D SteeringForce{DesiredVelocity - CurrentVelocity};
	constexpr float MaxForce{100};
	SteeringForce = SteeringForce.GetClampedToMaxSize(MaxForce);

	SteeringForce /= Agent.GetMass();

	FVector2D NewVelocity{CurrentVelocity + SteeringForce};
	NewVelocity = NewVelocity.GetClampedToMaxSize(Agent.GetMaxLinearSpeed());

	Steering.LinearVelocity = NewVelocity;


	DebugLines(Agent, Steering.LinearVelocity);
	return Steering;
}

//FLEE
//*******
SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	ISteeringBehavior::CalculateSteering(DeltaT, Agent);
	SteeringOutput Steering{};

	FVector2D CurrentVelocity{Agent.GetVelocity()};

	FVector2D ToTarget{Agent.GetPosition() - m_Target.Position};
	FVector2D DesiredVelocity{ToTarget.GetSafeNormal() * Agent.GetMaxLinearSpeed()};

	FVector2D SteeringForce{DesiredVelocity - CurrentVelocity};
	constexpr float MaxForce{100};
	SteeringForce = SteeringForce.GetClampedToMaxSize(MaxForce);

	SteeringForce /= Agent.GetMass();

	FVector2D NewVelocity{CurrentVelocity + SteeringForce};
	NewVelocity = NewVelocity.GetClampedToMaxSize(Agent.GetMaxLinearSpeed());

	Steering.LinearVelocity = NewVelocity;


	DebugLines(Agent, Steering.LinearVelocity);
	return Steering;
}

//ARRIVE
//*******
SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	ISteeringBehavior::CalculateSteering(DeltaT, Agent);

	float CurrentSpeed{Agent.GetMaxLinearSpeed()};
	constexpr float SlowRadius{500};
	constexpr float TargetRadius{100};

	DrawDebugCircle(Agent.GetWorld(), FVector(Agent.GetPosition(), 0.0f), SlowRadius, 20, FColor::Blue,
	                false, -1, 0, 0, FVector(1, 0, 0),
	                FVector(0, 1, 0));
	DrawDebugCircle(Agent.GetWorld(), FVector(Agent.GetPosition(), 0.0f), TargetRadius, 20, FColor::Orange,
	                false, -1, 0, 0, FVector(1, 0, 0),
	                FVector(0, 1, 0));


	if (FVector2D::Distance(Agent.GetPosition(), m_Target.Position) < TargetRadius)
	{
		//set speed to 0
		CurrentSpeed = 0.f;
	}
	else if (FVector2D::Distance(Agent.GetPosition(), m_Target.Position) <= SlowRadius)
	{
		//slow down with distance
		const float Distance{static_cast<float>(FVector2D::Distance(Agent.GetPosition(), m_Target.Position))};
		//distance / total distance
		const float DistancePercentile{(Distance - TargetRadius) / SlowRadius};
		const float Speed{m_MaxSpeed * DistancePercentile};
		CurrentSpeed = Speed;
	}
	else
	{
		//MaxSpeed
		CurrentSpeed = m_MaxSpeed;
	}

	Agent.SetMaxLinearSpeed(CurrentSpeed);
	SteeringOutput Steering{Seek::CalculateSteering(DeltaT, Agent)};
	return Steering;
}

//FACE
//*******
SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	ISteeringBehavior::CalculateSteering(DeltaT, Agent);
	Agent.SetMaxLinearSpeed(0);

	SteeringOutput Steering{};

	const FVector ToTarget{(FVector(m_Target.Position, 0.0f) - Agent.GetActorLocation()).GetSafeNormal()};
	const float DesiredYaw{static_cast<float>(FMath::Atan2(ToTarget.Y, ToTarget.X) * (180.0f / PI))};
	const float CurrentYaw{static_cast<float>(Agent.GetActorRotation().Yaw)};
	const float DeltaYaw{FMath::FindDeltaAngleDegrees(CurrentYaw, DesiredYaw)};
	constexpr float MaxTurnRate{180.0f}; // deg/s, tune this
	Steering.AngularVelocity = FMath::Clamp(DeltaYaw * 1.5f, -MaxTurnRate, MaxTurnRate); // Proportional gain ~1.5

	DebugLines(Agent, Steering.LinearVelocity);

	return Steering;
}

//PURSUIT
//*******
SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	FVector2D CurrentVelocity{Agent.GetVelocity()};

	const float T{
		static_cast<float>(FVector2D::Distance(Agent.GetPosition(), m_Target.Position)) / Agent.GetMaxLinearSpeed()
	};

	FVector2D ToTarget{m_Target.Position - Agent.GetPosition() * T};
	FVector2D DesiredVelocity{ToTarget.GetSafeNormal() * Agent.GetMaxLinearSpeed()};

	FVector2D SteeringForce{DesiredVelocity - CurrentVelocity};
	constexpr float MaxForce{100};
	SteeringForce = SteeringForce.GetClampedToMaxSize(MaxForce);

	SteeringForce /= Agent.GetMass();

	FVector2D NewVelocity{CurrentVelocity + SteeringForce};
	NewVelocity = NewVelocity.GetClampedToMaxSize(Agent.GetMaxLinearSpeed());

	Steering.LinearVelocity = NewVelocity;


	DebugLines(Agent, Steering.LinearVelocity);
	return Steering;
}

SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	ISteeringBehavior::CalculateSteering(DeltaT, Agent);
	SteeringOutput Steering{};

	FVector2D CurrentVelocity{Agent.GetVelocity()};
	const float T{
		static_cast<float>(FVector2D::Distance(Agent.GetPosition(), m_Target.Position)) / Agent.GetMaxLinearSpeed()
	};

	FVector2D ToTarget{Agent.GetPosition() - m_Target.Position * T};
	FVector2D DesiredVelocity{ToTarget.GetSafeNormal() * Agent.GetMaxLinearSpeed()};

	FVector2D SteeringForce{DesiredVelocity - CurrentVelocity};
	constexpr float MaxForce{100};
	SteeringForce = SteeringForce.GetClampedToMaxSize(MaxForce);

	SteeringForce /= Agent.GetMass();

	FVector2D NewVelocity{CurrentVelocity + SteeringForce};
	NewVelocity = NewVelocity.GetClampedToMaxSize(Agent.GetMaxLinearSpeed());

	Steering.LinearVelocity = NewVelocity;


	DebugLines(Agent, Steering.LinearVelocity);
	return Steering;
}


