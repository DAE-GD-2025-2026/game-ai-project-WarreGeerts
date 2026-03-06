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

	FVector2D outputVelocity{};
	
	for (auto Neighbor : pFlock->GetNeighbors())
	{
		const FVector2D toAgent = pAgent.GetPosition() - Neighbor->GetPosition();
	
		FVector2D pushForce = toAgent;
		pushForce /= pushForce.SquaredLength();

		outputVelocity += pushForce;
	}	
	
	SteeringOutput steering = Seek::CalculateSteering(deltaT, pAgent);
	steering.LinearVelocity = outputVelocity;
	
	return steering;

}

//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput Alignment::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	
	//ga over alle agents en ga richting gemiddelede velocity
	
	return Seek::CalculateSteering(deltaT, pAgent);
}