#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"

//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	m_Target = FTargetData(pFlock->GetAverageNeighborPos());
	SteeringOutput steeringOutput = Seek::CalculateSteering(deltaT, pAgent);

	return steeringOutput;
}

//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	//for loop van alle neigbors
	//afstand berekenen

	SteeringOutput steering{};
	FVector2D outputVelocity{};
	
	if (pFlock->GetNrOfNeighbors() == 0)
	{
		return steering;
	}

	for (auto Neighbor : pFlock->GetNeighbors())
	{
		if (!Neighbor) continue;

		const FVector2D toAgent = pAgent.GetPosition() - Neighbor->GetPosition();

		FVector2D pushForce = toAgent;
		if (pushForce.SquaredLength() > 0.001f)
		{
			pushForce /= pushForce.SquaredLength();
			outputVelocity += pushForce;
		}
	}

	steering.LinearVelocity = outputVelocity;

	return steering;
}

//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput Alignment::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	//ga over alle agents en ga richting gemiddelede velocity (GetAvrNeighborVel)
	SteeringOutput steering{};
	steering.LinearVelocity = pFlock->GetAverageNeighborVelocity();
	return steering;
}
