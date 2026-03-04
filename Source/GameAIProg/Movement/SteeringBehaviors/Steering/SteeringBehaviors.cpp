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
void ISteeringBehavior::DebugLines(ASteeringAgent& Agent, SteeringOutput Steering, FVector2D Target)
{
	const FVector Agent3D{Agent.GetPosition(), 0.0f};

	FVector2D _Target;
	if (Target.IsNearlyZero())
		_Target = m_Target.Position;
	else
		_Target = Target;

	//GREEN LINE: Target direction
	const FVector Target3D{_Target, 0.0f};
	const FVector ToTarget{Target3D - Agent3D};
	constexpr float GreenLineLength{200.f};
	const FVector FixedEnd{Agent3D + ToTarget.GetSafeNormal() * GreenLineLength};
	DrawDebugLine(Agent.GetWorld(), Agent3D, FixedEnd, FColor::Green, false, -1);

	//RED CIRCLE: Target position
	DrawDebugCircle(Agent.GetWorld(), FVector(_Target, 0.0f), 20, 10, FColor::Red,
	                false, -1, 0, 0, FVector(1, 0, 0),
	                FVector(0, 1, 0));

	//MAGENTA LINE: Linear velocity
	constexpr float MaxLineLength{300.f};
	constexpr float MaxSpeedForViz{600.f};
	const float Speed{static_cast<float>(Steering.LinearVelocity.Size())};
	const float LengthFactor{FMath::Clamp(Speed / MaxSpeedForViz, 0.f, 1.f)};
	const float LineLengthLin{MaxLineLength * LengthFactor};

	const FRotator AgentRotation{0.f, Agent.GetRotation(), 0.f};
	const FVector AgentForward{AgentRotation.Vector()};
	const FVector VelEnd{Agent3D + AgentForward * LineLengthLin};
	DrawDebugLine(Agent.GetWorld(), Agent3D, VelEnd, FColor::Magenta, false, -1);
}


//SEEK
//*******
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	Agent.SetIsAutoOrienting(false);

	SteeringOutput Steering{};

	//Linear Velocity
	Steering.LinearVelocity = FVector2D(Agent.GetActorForwardVector().GetSafeNormal2D() * Agent.GetMaxLinearSpeed());

	//Angular velocity
	const FVector2D ToTarget{(m_Target.Position - Agent.GetPosition()).GetSafeNormal()};
	const float TargetRotationRad{static_cast<float>(FMath::Atan2(ToTarget.Y, ToTarget.X))};
	const float CurrentRotationRad{FMath::DegreesToRadians(Agent.GetRotation())};

	Steering.AngularVelocity = TargetRotationRad - CurrentRotationRad;

	if (Steering.AngularVelocity > PI)
		Steering.AngularVelocity -= 2 * PI;
	else if (Steering.AngularVelocity < -PI)
		Steering.AngularVelocity += 2 * PI;

	//Clamp to maxSpeed
	Steering.AngularVelocity = FMath::Clamp(Steering.AngularVelocity,
	                                        -Agent.GetMaxAngularSpeed(),
	                                        Agent.GetMaxAngularSpeed());

	DebugLines(Agent, Steering);

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

	DrawDebugCircle(Agent.GetWorld(), FVector(Agent.GetPosition(), 0.0f), m_Radius * 5.f, 20, FColor::Blue,
	                false, -1, 0, 0, FVector(1, 0, 0),
	                FVector(0, 1, 0));
	DrawDebugCircle(Agent.GetWorld(), FVector(Agent.GetPosition(), 0.0f), m_Radius, 20, FColor::Orange,
	                false, -1, 0, 0, FVector(1, 0, 0),
	                FVector(0, 1, 0));


	if (FVector2D::Distance(Agent.GetPosition(), m_Target.Position) < m_Radius)
	{
		//set speed to 0
		CurrentSpeed = 0.f;
	}
	else if (FVector2D::Distance(Agent.GetPosition(), m_Target.Position) <= m_Radius * 5.f)
	{
		//slow down with distance
		const float Distance{static_cast<float>(FVector2D::Distance(Agent.GetPosition(), m_Target.Position))};
		//distance / total distance
		const float DistancePercentile{(Distance - m_Radius) / m_Radius * 5.f};
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

void Arrive::SetTargetRadius(float radius)
{
	m_Radius = radius;
}

//FACE
//*******
SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	Agent.SetIsAutoOrienting(false);

	SteeringOutput Steering{};

	const FVector2D ToTarget{(m_Target.Position - Agent.GetPosition()).GetSafeNormal()};
	const float TargetRot{static_cast<float>(FMath::Atan2(ToTarget.Y, ToTarget.X))};
	const float CurrRot{FMath::DegreesToRadians(Agent.GetRotation())};

	Steering.AngularVelocity = TargetRot - CurrRot;
	if (Steering.AngularVelocity > PI) Steering.AngularVelocity -= 2 * PI;
	else if (Steering.AngularVelocity < -PI) Steering.AngularVelocity += 2 * PI;
	Steering.AngularVelocity = FMath::Clamp(Steering.AngularVelocity, -Agent.GetMaxAngularSpeed(),
	                                        Agent.GetMaxAngularSpeed());

	DebugLines(Agent, Steering);
	return Steering;
}

//PURSUIT
//*******
SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	Agent.SetIsAutoOrienting(false);
	SteeringOutput Steering{};

	//Predict future position with T
	const float T{
		static_cast<float>(FVector2D::Distance(Agent.GetPosition(), m_Target.Position) / Agent.GetMaxLinearSpeed())
	};
	const FVector2D PredictedPos{m_Target.Position + m_Target.LinearVelocity * T};

	//Distance check
	if (FVector2D::Distance(Agent.GetPosition(), PredictedPos) < 20.0f)
	{
		Steering.LinearVelocity = FVector2D::ZeroVector;
		Steering.AngularVelocity = 0.0f;
		return Steering;
	}

	//Linear Velocity
	Steering.LinearVelocity = FVector2D(Agent.GetActorForwardVector().GetSafeNormal2D() * Agent.GetMaxLinearSpeed());

	//Angular velocity
	const FVector2D ToPredicted{(PredictedPos - Agent.GetPosition()).GetSafeNormal()};
	const float TargetRot{static_cast<float>(FMath::Atan2(ToPredicted.Y, ToPredicted.X))};
	const float CurrRot{FMath::DegreesToRadians(Agent.GetRotation())};

	Steering.AngularVelocity = TargetRot - CurrRot;
	if (Steering.AngularVelocity > PI) Steering.AngularVelocity -= 2 * PI;
	else if (Steering.AngularVelocity < -PI) Steering.AngularVelocity += 2 * PI;
	Steering.AngularVelocity = FMath::Clamp(Steering.AngularVelocity, -Agent.GetMaxAngularSpeed(),
	                                        Agent.GetMaxAngularSpeed());

	DebugLines(Agent, Steering);
	return Steering;
}

//EVADE
//*******
SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	constexpr float Radius{500};
	DrawDebugCircle(Agent.GetWorld(), FVector(Agent.GetPosition(), 0.0f), Radius, 20, FColor::Red,
	                false, -1, 0, 0, FVector(1, 0, 0),
	                FVector(0, 1, 0));

	if (FVector2D::Distance(m_Target.Position, Agent.GetPosition()) > Radius)
	{
		Steering.IsValid = false;
	}
	Agent.SetIsAutoOrienting(false);

	//Predict future position with T
	const float Distance{static_cast<float>(FVector2D::Distance(Agent.GetPosition(), m_Target.Position))};
	const float T{Distance / Agent.GetMaxLinearSpeed()};
	const FVector2D PredictedPos{m_Target.Position + m_Target.LinearVelocity * T};

	const FVector2D FleeDirection{(Agent.GetPosition() - PredictedPos).GetSafeNormal()};

	//Linear Velocity
	Steering.LinearVelocity = FVector2D(Agent.GetActorForwardVector().GetSafeNormal2D() * Agent.GetMaxLinearSpeed());

	//Angular Velocity
	const float TargetRotation{static_cast<float>(FMath::Atan2(FleeDirection.Y, FleeDirection.X))};
	const float CurrentRotation{FMath::DegreesToRadians(Agent.GetRotation())};

	Steering.AngularVelocity = TargetRotation - CurrentRotation;
	while (Steering.AngularVelocity > PI) Steering.AngularVelocity -= 2 * PI;
	while (Steering.AngularVelocity < -PI) Steering.AngularVelocity += 2 * PI;

	Steering.AngularVelocity = FMath::Clamp(Steering.AngularVelocity,
	                                        -Agent.GetMaxAngularSpeed(),
	                                        Agent.GetMaxAngularSpeed());

	DebugLines(Agent, Steering, Agent.GetPosition() + FVector2D(FleeDirection.X, FleeDirection.Y) * 100);
	return Steering;
}

//WANDER
//*******
SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	Agent.SetIsAutoOrienting(false);
	SteeringOutput Steering{};

	//Forward vector
	const FVector2D AgentForward{
		FMath::Cos(FMath::DegreesToRadians(Agent.GetRotation())),
		FMath::Sin(FMath::DegreesToRadians(Agent.GetRotation()))
	};

	//Wander circle
	const FVector2D CircleCenter{AgentForward * m_OffsetDistance};

	m_WanderAngle += FMath::FRandRange(-m_MaxAngleChange, m_MaxAngleChange);

	const FVector2D Displacement{
		m_Radius * FMath::Cos(m_WanderAngle),
		m_Radius * FMath::Sin(m_WanderAngle)
	};

	const FVector2D WanderTargetDir{(CircleCenter + Displacement).GetSafeNormal()};

	//Linear Velocity
	Steering.LinearVelocity = AgentForward * Agent.GetMaxLinearSpeed();

	//Angular Velocity
	const float TargetRotation{static_cast<float>(FMath::Atan2(WanderTargetDir.Y, WanderTargetDir.X))};
	const float CurrentRotation{FMath::DegreesToRadians(Agent.GetRotation())};

	Steering.AngularVelocity = TargetRotation - CurrentRotation;
	while (Steering.AngularVelocity > PI) Steering.AngularVelocity -= 2 * PI;
	while (Steering.AngularVelocity < -PI) Steering.AngularVelocity += 2 * PI;

	Steering.AngularVelocity = FMath::Clamp(Steering.AngularVelocity,
	                                        -Agent.GetMaxAngularSpeed(),
	                                        Agent.GetMaxAngularSpeed());

	//Debug: circle at correct position, line to world target
	const FVector CirclePos{
		FVector(Agent.GetPosition().X, Agent.GetPosition().Y, 0.0f) +
		FVector(AgentForward, 0.0f) * m_OffsetDistance
	};
	DrawDebugCircle(Agent.GetWorld(), CirclePos, m_Radius, 20, FColor::Blue, false, -1);

	const FVector2D WorldTargetPos{Agent.GetPosition() + WanderTargetDir * 100};
	DebugLines(Agent, Steering, WorldTargetPos);

	return Steering;
}
