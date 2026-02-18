#include "SteeringBehaviors.h"
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
void ISteeringBehavior::DebugLines(ASteeringAgent& Agent, SteeringOutput steering, FVector2D Target)
{
	const FVector Agent3D{Agent.GetPosition(), 0.0f};
	const FVector Velocity3D{steering.LinearVelocity, 0.0f};

	FVector2D _Target;
	if (Target.IsNearlyZero())
		_Target = m_Target.Position;
	else
		_Target = Target;

	//--point to end
	const FVector ToTarget{FVector(_Target, 0.0f) - Agent3D};
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
	DrawDebugCircle(Agent.GetWorld(), FVector(_Target, 0.0f), 20, 10, FColor::Red,
	                false, -1, 0, 0, FVector(1, 0, 0),
	                FVector(0, 1, 0));
}


//TODO: Do the Week01 assignment :^)
//SEEK
//*******
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	Agent.SetIsAutoOrienting(false);

	SteeringOutput Steering{};

	//linear vel
	FVector2D AgentForward{Agent.GetActorForwardVector().GetSafeNormal2D()};
	Steering.LinearVelocity = AgentForward * Agent.GetMaxAngularSpeed();

	//angular vel	
	const float CurrentRotation{FMath::DegreesToRadians(Agent.GetRotation())};
	Steering.AngularVelocity = FMath::Atan2(m_Target.Position.Y - Agent.GetPosition().Y,
	                                        m_Target.Position.X - Agent.GetPosition().X) - CurrentRotation;

	if (Steering.AngularVelocity > PI)
	{
		Steering.AngularVelocity -= 2 * PI;
	}
	else if (Steering.AngularVelocity < -PI)
	{
		Steering.AngularVelocity += 2 * PI;
	}

	DebugLines(Agent, Steering); // Line to target

	return Steering;
}

//FLEE
//*******
SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{Seek::CalculateSteering(DeltaT, Agent)};

	Steering.AngularVelocity = -Steering.AngularVelocity;

	return Steering;
}

//ARRIVE
//*******
SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	ISteeringBehavior::CalculateSteering(DeltaT, Agent);

	float CurrentSpeed;
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
	//ISteeringBehavior::CalculateSteering(DeltaT, Agent);

	SteeringOutput Steering{};

	const float CurrentRotation{FMath::DegreesToRadians(Agent.GetRotation())};
	Steering.AngularVelocity = FMath::Atan2(m_Target.Position.Y - Agent.GetPosition().Y,
	                                        m_Target.Position.X - Agent.GetPosition().X) - CurrentRotation;

	if (Steering.AngularVelocity > PI)
	{
		Steering.AngularVelocity -= 2 * PI;
	}
	else if (Steering.AngularVelocity < -PI)
	{
		Steering.AngularVelocity += 2 * PI;
	}

	DebugLines(Agent, Steering.LinearVelocity);

	return Steering;
}

//PURSUIT
//*******
SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	Agent.SetIsAutoOrienting(false);

	SteeringOutput Steering{};

	//linear vel
	FVector2D AgentForward{Agent.GetActorForwardVector().GetSafeNormal2D()};
	Steering.LinearVelocity = AgentForward * Agent.GetMaxAngularSpeed();

	//angular vel	
	const float CurrentRotation{FMath::DegreesToRadians(Agent.GetRotation())};
	Steering.AngularVelocity = FMath::Atan2(m_Target.Position.Y - Agent.GetPosition().Y,
											m_Target.Position.X - Agent.GetPosition().X) - CurrentRotation;

	if (Steering.AngularVelocity > PI)
	{
		Steering.AngularVelocity -= 2 * PI;
	}
	else if (Steering.AngularVelocity < -PI)
	{
		Steering.AngularVelocity += 2 * PI;
	}

	DebugLines(Agent, Steering); // Line to target

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

SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	const FVector AgentForward{
		FMath::Cos(Agent.GetRotation() * PI / 180.f), FMath::Sin(Agent.GetRotation() * PI / 180.f), 0.f
	};

	FVector2D CircleCenter{AgentForward * m_OffsetDistance * 25};

	//Calc end point depending on rotation of Agent
	const FVector CirclePos{FVector(Agent.GetPosition(), 0.0f) + AgentForward * m_OffsetDistance * 25.f};
	//Draw Circle visualizer
	DrawDebugCircle(Agent.GetWorld(), CirclePos, m_Radius * 25, 20, FColor::Blue,
	                false, -1, 0, 0, FVector(1, 0, 0),
	                FVector(0, 1, 0));

	//Get displacement force
	FVector2D Displacement{0, -1};
	Displacement *= m_Radius * 25;
	//Randomly change it so it's not the same every time
	m_WanderAngle += FMath::FRandRange(-m_MaxAngleChange, m_MaxAngleChange);
	//Change current angle
	float Length{static_cast<float>(Displacement.Size())};
	Displacement.X = Length * FMath::Cos(m_WanderAngle);
	Displacement.Y = Length * FMath::Sin(m_WanderAngle);
	//Get wander force
	FVector2D WanderForce{CircleCenter + Displacement};

	//Seek
	FVector2D CurrentVelocity{Agent.GetVelocity()};
	FVector2D DesiredVelocity{WanderForce.GetSafeNormal() * Agent.GetMaxLinearSpeed()};
	FVector2D SteeringForce{DesiredVelocity - CurrentVelocity};

	constexpr float MaxForce{100};
	SteeringForce = SteeringForce.GetClampedToMaxSize(MaxForce);
	SteeringForce /= Agent.GetMass();

	FVector2D NewVelocity{CurrentVelocity + SteeringForce};
	NewVelocity = NewVelocity.GetClampedToMaxSize(Agent.GetMaxLinearSpeed());

	Steering.LinearVelocity = NewVelocity;

	DebugLines(Agent, Steering.LinearVelocity, WanderForce + Agent.GetPosition());
	return Steering;
}
