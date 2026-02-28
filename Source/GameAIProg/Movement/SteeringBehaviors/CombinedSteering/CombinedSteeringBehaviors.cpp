#include "CombinedSteeringBehaviors.h"
#include <algorithm>
#include "../SteeringAgent.h"
#include "DynamicMesh/DynamicMesh3.h"

BlendedSteering::BlendedSteering(const std::vector<WeightedBehavior>& WeightedBehaviors)
	: WeightedBehaviors(WeightedBehaviors)
{
};

//****************
//BLENDED STEERING
SteeringOutput BlendedSteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput BlendedSteering = {};
	float totalWeight{0.f};

	for (const auto& Wb : WeightedBehaviors)
	{
		if (Wb.pBehavior)
		{
			SteeringOutput steering{Wb.pBehavior->CalculateSteering(DeltaT, Agent)};
			if (steering.IsValid)
			{
				BlendedSteering.LinearVelocity += steering.LinearVelocity * Wb.Weight;
				BlendedSteering.AngularVelocity += steering.AngularVelocity * Wb.Weight;
				totalWeight += Wb.Weight;
			}
		}
	}

	if (totalWeight > 0.f)
	{
		BlendedSteering.LinearVelocity /= totalWeight;
		BlendedSteering.AngularVelocity /= totalWeight;
		BlendedSteering.IsValid = true;
	}

	return BlendedSteering;
}

//*****************
//PRIORITY STEERING
SteeringOutput PrioritySteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering = {};

	for (ISteeringBehavior* const pBehavior : m_PriorityBehaviors)
	{
		Steering = pBehavior->CalculateSteering(DeltaT, Agent);

		if (Steering.IsValid)
			break;
	}

	//If non of the behavior return a valid output, last behavior is returned
	return Steering;
}
