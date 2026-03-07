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
	  , WorldSize{WorldSize}
	  , pAgentToEvade{pAgentToEvade}
	  , TrimWorld{bTrimWorld}
{
	pSeparationBehavior = std::make_unique<Separation>(this);
	pCohesionBehavior = std::make_unique<Cohesion>(this);
	pVelMatchBehavior = std::make_unique<Alignment>(this);
	pSeekBehavior = std::make_unique<Seek>();
	pWanderBehavior = std::make_unique<Wander>();
	pEvadeBehavior = std::make_unique<Evade>();


	const std::vector<BlendedSteering::WeightedBehavior> FlockBehaviors{
		{pCohesionBehavior.get(), .3f},
		{pSeparationBehavior.get(), .5f},
		{pVelMatchBehavior.get(), .3f},
		{pSeekBehavior.get(), .2f},
		{pWanderBehavior.get(), .2f},
	};
	pBlendedSteering = std::make_unique<BlendedSteering>(FlockBehaviors);


	std::vector<ISteeringBehavior*> PriorityBehaviors{
		{pEvadeBehavior.get()},
		{pBlendedSteering.get()}
	};
	pPrioritySteering = std::make_unique<PrioritySteering>(PriorityBehaviors);

	pPartitionedSpace = std::make_unique<CellSpace>(pWorld, WorldSize * 2.f, WorldSize * 2.f,
	                                                NrOfCellsX, NrOfCellsX, FlockSize);

	Agents.SetNum(FlockSize);
#ifndef GAMEAI_USE_SPACE_PARTITIONING
	pNeighbors.SetNum(FlockSize);
#endif
	OldPositions.SetNum(FlockSize);

	NrOfNeighbors = 0;

	for (int NrOfAgents = 0; NrOfAgents < FlockSize;)
	{
		FVector Position{FVector::ZeroVector};
		constexpr int Radius{1000};
		Position.X = FMath::RandRange(-Radius, Radius);
		Position.Z = FMath::RandRange(-Radius, Radius);

		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

		Agents[NrOfAgents] = pWorld->SpawnActor<ASteeringAgent>(AgentClass, FVector{Position.X, Position.Z, 0.f},
		                                                        FRotator::ZeroRotator, SpawnParams);

		if (Agents[NrOfAgents])
		{
			Agents[NrOfAgents]->SetSteeringBehavior(pPrioritySteering.get());
			Agents[NrOfAgents]->SetActorTickEnabled(false);

			pPartitionedSpace->AddAgent(*Agents[NrOfAgents]);
			OldPositions[NrOfAgents] = Agents[NrOfAgents]->GetPosition();
			++NrOfAgents;
		}
	}
}

Flock::~Flock()
{
	for (ASteeringAgent* Agent : Agents)
	{
		if (Agent)
		{
			Agent->Destroy();
		}
	}
}

void Flock::Tick(float DeltaTime)
{
	// TODO: update the flock
	// TODO: for every agent:
	// TODO: register the neighbors for this agent (-> fill the memory pool with the neighbors for the currently evaluated agent)
	// TODO: update the agent (-> the steeringbehaviors use the neighbors in the memory pool)
	// TODO: trim the agent to the world


	if (pAgentToEvade && pEvadeBehavior)
	{
		FTargetData Target;
		Target.Position = pAgentToEvade->GetPosition();
		Target.LinearVelocity = pAgentToEvade->GetLinearVelocity();
		pEvadeBehavior->SetTarget(Target);
	}

	for (auto& pAgent : Agents)
	{
		if (!pAgent) continue;

#ifdef GAMEAI_USE_SPACE_PARTITIONING
		pPartitionedSpace->UpdateAgentCell(*pAgent, OldPositions[Agents.IndexOfByKey(pAgent)]);
#endif

		OldPositions[Agents.IndexOfByKey(pAgent)] = pAgent->GetPosition();


#ifdef GAMEAI_USE_SPACE_PARTITIONING
		pPartitionedSpace->RegisterNeighbors(*pAgent, NeighborhoodRadius);
#else
		RegisterNeighbors(pAgent);
#endif

		pAgent->Tick(DeltaTime);

		if (TrimWorld)
		{
			FVector AgentPos{pAgent->GetActorLocation()};
			bool change{false};

			if (AgentPos.X > WorldSize)
			{
				AgentPos.X = -WorldSize;
				change = true;
			}
			else if (AgentPos.X < -WorldSize)
			{
				AgentPos.X = WorldSize;
				change = true;
			}

			if (AgentPos.Y > WorldSize)
			{
				AgentPos.Y = -WorldSize;
				change = true;
			}
			else if (AgentPos.Y < -WorldSize)
			{
				AgentPos.Y = WorldSize;
				change = true;
			}

			if (change)
			{
				pAgent->SetActorLocation(AgentPos);
			}
		}
	}
}

void Flock::RenderDebug()
{
	// TODO: Render all the agents in the flock
	// ONLY FIRST AGENT gets debug
	for (ASteeringAgent* Agent : Agents)
	{
		if (Agent)
			Agent->SetDebugRenderingEnabled(false);
	}
	if (Agents.Num() > 0 && Agents[0])
	{
		Agents[0]->SetDebugRenderingEnabled(DebugRenderSteering);
	}


	if (DebugRenderNeighborhood)
	{
		RenderNeighborhood();
	}

#ifdef GAMEAI_USE_SPACE_PARTITIONING
	if (DebugRenderPartitions)
	{
		pPartitionedSpace->RenderCells();
	}
#endif


	if (TrimWorld)
	{
		DrawDebugBox(
			pWorld,
			FVector::ZeroVector,
			FVector(WorldSize, WorldSize, 0.0f),
			FColor::Red,
			false,
			-1.f,
			0,
			5.f
		);
	}
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
		ImGui::Checkbox("Debug Render Steering", &DebugRenderSteering);
		ImGui::Checkbox("Debug Render Neighborhood", &DebugRenderNeighborhood);
		ImGui::Checkbox("Debug Render Partitioning", &DebugRenderPartitions);

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

		// TODO: implement ImGUI sliders for steering behavior weights here
		if (pBlendedSteering)
		{
			auto& Weigths{pBlendedSteering->GetWeightedBehaviorsRef()};
			if (Weigths.size() >= 5)
			{
				ImGuiHelpers::ImGuiSliderFloatWithSetter("Cohesion", Weigths[0].Weight, 0.f, 1.f,
				                                         [&Weigths](const float InVal)
				                                         {
					                                         Weigths[0].Weight = InVal;
				                                         }, "%.2f");
				ImGuiHelpers::ImGuiSliderFloatWithSetter("Separation", Weigths[1].Weight, 0.f, 1.f,
				                                         [&Weigths](const float InVal)
				                                         {
					                                         Weigths[1].Weight = InVal;
				                                         }, "%.2f");
				ImGuiHelpers::ImGuiSliderFloatWithSetter("Velocity Match", Weigths[2].Weight, 0.f, 1.f,
				                                         [&Weigths](const float InVal)
				                                         {
					                                         Weigths[2].Weight = InVal;
				                                         }, "%.2f");
				ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek", Weigths[3].Weight, 0.f, 1.f,
				                                         [&Weigths](const float InVal)
				                                         {
					                                         Weigths[3].Weight = InVal;
				                                         }, "%.2f");
				ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander", Weigths[4].Weight, 0.f, 1.f,
				                                         [&Weigths](const float InVal)
				                                         {
					                                         Weigths[4].Weight = InVal;
				                                         }, "%.2f");
			}
		}
		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
	// TODO: Debugrender the neighbors for the first agent in the flock
	if (Agents.Num() == 0 || !Agents[0]) return;

#ifdef GAMEAI_USE_SPACE_PARTITIONING
	pPartitionedSpace->RegisterNeighbors(*Agents[0], NeighborhoodRadius);
#else
	RegisterNeighbors(Agents[0]);
#endif

	FVector pos = FVector(Agents[0]->GetPosition(), 0);
	DrawDebugCircle(pWorld, pos, NeighborhoodRadius, 32, FColor::Yellow, false,
	                -1, 0, 0, FVector(1, 0, 0), FVector(0, 1, 0));


#ifdef GAMEAI_USE_SPACE_PARTITIONING
	FVector Radius{NeighborhoodRadius, NeighborhoodRadius, 0};

	DrawDebugBox(pWorld, pos, Radius, FColor::Yellow, false, -1, 0, 2.f);

	for (auto idx{0}; idx < GetNrOfNeighbors(); ++idx)
	{
		if (GetNeighbors()[idx])
		{
			DrawDebugSphere(pWorld, FVector(GetNeighbors()[idx]->GetPosition(), 10.f), 15.f,
			                16, FColor::Cyan, false, -1.f);
			DrawDebugLine(pWorld, pos,
			              FVector(GetNeighbors()[idx]->GetPosition(), 0.f),
			              FColor::Yellow, false, -1.f);
		}
	}
#else
	for (ASteeringAgent* PNeighbor : pNeighbors)
	{
		if (PNeighbor)
		{
			DrawDebugSphere(pWorld, FVector(PNeighbor->GetPosition(), 10.f), 15.f, 16, FColor::Cyan, false, -1.f);
			DrawDebugLine(pWorld, pos, FVector(PNeighbor->GetPosition(), 0.f), FColor::Yellow, false,
			              -1.f);
		}
	}
#endif
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	if (!pAgent) return;

	NrOfNeighbors = 0;
	pNeighbors.Reset();

	const FVector2D agentPos{pAgent->GetPosition()};

	for (auto& pOther : Agents)
	{
		if (!pOther || pOther == pAgent) continue;


		if (FVector2D::DistSquared(agentPos, pOther->GetPosition()) <= (NeighborhoodRadius * NeighborhoodRadius))
		{
			pNeighbors.Add(pOther);
			++NrOfNeighbors;
		}
	}
}

int Flock::GetNrOfNeighbors() const
{
	return NrOfNeighbors;
}

const TArray<ASteeringAgent*>& Flock::GetNeighbors() const
{
	return pNeighbors;
}
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
	if (NrOfNeighbors == 0 || NrOfNeighbors > GetNeighbors().Num())
		return FVector2D::ZeroVector;

	FVector2D sum{FVector2D::ZeroVector};

	for (auto idx{0}; idx < NrOfNeighbors; ++idx)
	{
		sum += GetNeighbors()[idx]->GetPosition();
	}

	FVector2D avgPosition = sum / static_cast<float>(NrOfNeighbors);
	return avgPosition;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	if (NrOfNeighbors == 0 || NrOfNeighbors > GetNeighbors().Num())
		return FVector2D::ZeroVector;


	FVector2D sum{FVector2D::ZeroVector};
	for (auto idx{0}; idx < NrOfNeighbors; ++idx)
	{
		sum += GetNeighbors()[idx]->GetLinearVelocity();
	}

	FVector2D avgVelocity = sum / static_cast<float>(NrOfNeighbors);
	return avgVelocity;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	if (pSeekBehavior)
	{
		pSeekBehavior->SetTarget(Target);
	}
}
