#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"


Flock::Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent* const pAgentToEvade,
	bool bTrimWorld)
	: pWorld{pWorld}
	  , FlockSize{FlockSize}
	  , pAgentToEvade{pAgentToEvade}
{
	Agents.SetNum(FlockSize);

	pNeighbors.SetNum(FlockSize);
	NrOfNeighbors = 0;
	
	for (int NrOfAgents = 0; NrOfAgents < FlockSize;)
	{
		FVector position{FVector::ZeroVector};
		constexpr int radius{1000};
		position.X = FMath::RandRange(-radius, radius);
		position.Z = FMath::RandRange(-radius, radius);

		Agents[NrOfAgents] = pWorld->SpawnActor<ASteeringAgent>(AgentClass, FVector{position.X, 0, position.Z},
		                                                        FRotator::ZeroRotator);
		if (Agents[NrOfAgents])
		{
			const std::vector<BlendedSteering::WeightedBehavior> FlockBehaviors{
				{pCohesionBehavior.get(), .5f}, {pSeparationBehavior.get(), .5f}
			};
			pBlendedSteering = std::make_unique<BlendedSteering>(FlockBehaviors);
			Agents[NrOfAgents]->SetSteeringBehavior(pBlendedSteering.get());
			++NrOfAgents;
		}
	}
}

Flock::~Flock()
{
	// TODO: Cleanup any additional data
}

void Flock::Tick(float DeltaTime)
{
	// TODO: update the flock
	// TODO: for every agent:
	// TODO: register the neighbors for this agent (-> fill the memory pool with the neighbors for the currently evaluated agent)
	// TODO: update the agent (-> the steeringbehaviors use the neighbors in the memory pool)
	// TODO: trim the agent to the world

	for (auto& pAgent : Agents)
	{
		if (!pAgent) continue;

		RegisterNeighbors(pAgent);

		pAgent->Tick(DeltaTime);
	}
}

void Flock::RenderDebug()
{
	// TODO: Render all the agents in the flock
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive,
		             ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scrollwheel: zoom cam.");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();

		// TODO: implement ImGUI checkboxes for debug rendering here

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

		// TODO: implement ImGUI sliders for steering behavior weights here
		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
	// TODO: Debugrender the neighbors for the first agent in the flock
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	if (!pAgent) return;

	NrOfNeighbors = 0;

	const FVector2D agentPos{pAgent->GetPosition()};
	const float radiusSq{NeighborhoodRadius * NeighborhoodRadius};

	for (auto& pOther : Agents)
	{
		if (!pOther || pOther == pAgent) continue;

		const FVector2D otherPos{pOther->GetPosition()};
		const FVector2D toOther{otherPos - agentPos};

		if (toOther.SquaredLength() <= radiusSq)
		{
			pNeighbors[NrOfNeighbors] = pOther;
			++NrOfNeighbors;

			if (NrOfNeighbors >= pNeighbors.Num()) break;
		}
	}
}
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D avgPosition{FVector2D::ZeroVector};
	if (NrOfNeighbors == 0) return avgPosition;

	FVector2D sum{FVector2D::ZeroVector};
	for (auto idx{0}; idx < NrOfNeighbors; ++idx)
	{
		sum += pNeighbors[idx]->GetPosition();
	}

	avgPosition = sum / static_cast<float>(NrOfNeighbors);
	return avgPosition;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D avgVelocity = FVector2D::ZeroVector;
	if (NrOfNeighbors == 0) return avgVelocity;

	FVector2D sum{FVector2D::ZeroVector};
	for (auto idx{0}; idx < NrOfNeighbors; ++idx)
	{
		sum += FVector2D(pNeighbors[idx]->GetVelocity());
	}

	avgVelocity = sum / static_cast<float>(NrOfNeighbors);
	return avgVelocity;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	// TODO: Implement
}
