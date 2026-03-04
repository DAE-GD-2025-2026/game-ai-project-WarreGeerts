#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"

//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput steeringOutput = Seek::CalculateSteering(deltaT, pAgent);	

	return steeringOutput;
}

//*********************
//SEPARATION (FLOCKING)
/*SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput steeringOutput = Seek::CalculateSteering(deltaT, pAgent);	

	const FVector2D toAgent = pAgent.GetPosition() - ;
	FVector2D pushForce = toAgent;
	
	pushForce /= pushForce.SquaredLength();
	
	steeringOutput.LinearVelocity += pushForce;
	
	return steeringOutput;
}*/

//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput Alignment::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	return Seek::CalculateSteering(deltaT, pAgent);
}