#pragma once

#include <Movement/SteeringBehaviors/SteeringHelpers.h>
#include "Kismet/KismetMathLibrary.h"


class ASteeringAgent;

// SteeringBehavior base, all steering behaviors should derive from this.
class ISteeringBehavior
{
public:
	ISteeringBehavior() = default;
	virtual ~ISteeringBehavior() = default;

	// Override to implement your own behavior
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent);

	virtual void DebugLines(ASteeringAgent& Agent, FVector2D& LinearVelocity, FVector2D Target = FVector2D::Zero()) final; //display debug lines
	static void SetMaxSpeed(const ASteeringAgent& Agent); //Static to make sure it's for all instances of this class

	void SetTarget(const FTargetData& NewTarget) { m_Target = NewTarget; }

	template <class T, std::enable_if_t<std::is_base_of_v<ISteeringBehavior, T>>* = nullptr>
	T* As()
	{
		return static_cast<T*>(this);
	}

protected:
	static float m_MaxSpeed; //static to make sure it's global for all instances
	FTargetData m_Target;

private:
	static bool sm_MaxSpeedSet; //static to make sure it doesn't get reset when a new instance is made
	bool m_MaxSpeedReset{false};
};

// Your own SteeringBehaviors should follow here...
//SEEK
class Seek : public ISteeringBehavior
{
public:
	Seek() = default;
	virtual ~Seek() override = default;
	
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

//FLEE
class Flee : public ISteeringBehavior
{
public:
	Flee() = default;
	virtual ~Flee() override = default;
	
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

//ARRIVE
class Arrive : public Seek
{
public:
	Arrive() = default;
	virtual ~Arrive() override = default;
	
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;

private:
	bool m_MaxSpeedSet{false};
};

//FACE
class Face : public ISteeringBehavior
{
public:
	Face() = default;
	virtual ~Face() override = default;
	
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

//PURSUIT
class Pursuit : public ISteeringBehavior
{
public:
	Pursuit() = default;
	virtual ~Pursuit() override = default;
	
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

//EVADE
class Evade : public ISteeringBehavior
{
public:
	Evade() = default;
	virtual ~Evade() override = default;
	
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

//WANDER
class Wander : public Seek
{
public:
	Wander() = default;
	virtual ~Wander() override = default;
	
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;

	void SetWanderOffset(float Offset) { m_OffsetDistance = Offset; }
	void SetWanderRadius(float Radius) { m_Radius = Radius; }
	void SetMaxAngleChange(float rad) { m_MaxAngleChange = rad; }

protected:
	float m_OffsetDistance{6.f};
	float m_Radius{4.f};
	float m_MaxAngleChange{45.f / 180.f * PI};
	float m_WanderAngle{0.f};
};
  