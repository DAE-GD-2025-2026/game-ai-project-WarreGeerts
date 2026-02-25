#include "Level_CombinedSteering.h"

#include "imgui.h"


// Sets default values
ALevel_CombinedSteering::ALevel_CombinedSteering()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_CombinedSteering::BeginPlay()
{
	Super::BeginPlay();

	//Drunk agent 50/50
	pDrunkAgent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{0, 0, 90}, FRotator::ZeroRotator);
	auto* wander{new Wander()};
	const std::vector<BlendedSteering::WeightedBehavior> DrunkBehaviors{
		{pSeek, 0.5f}, {wander, 0.5f}
	};
	pBlendedSteering = new BlendedSteering(DrunkBehaviors);
	pDrunkAgent->SetSteeringBehavior(pBlendedSteering);

	//Priority 
	pEvadingAgent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{0, 0, 90}, FRotator::ZeroRotator);
	auto* wanderEvade{new Wander()};
	std::vector<ISteeringBehavior*> evadingBehaviors {pEvade, wanderEvade};
	auto* pPrioritySteering { new PrioritySteering(evadingBehaviors)};
	pEvadingAgent->SetSteeringBehavior(pPrioritySteering);
}

void ALevel_CombinedSteering::BeginDestroy()
{
	Super::BeginDestroy();
}

// Called every frame
void ALevel_CombinedSteering::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

#pragma region UI
	//UI
	{
		//Setup
		bool windowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Game AI", &windowActive,
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
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();
		ImGui::Spacing();

		if (ImGui::Checkbox("Debug Rendering", &CanDebugRender))
		{
			// TODO: Handle the debug rendering of your agents here :)
		}
		ImGui::Checkbox("Trim World", &TrimWorld->bShouldTrimWorld);
		if (TrimWorld->bShouldTrimWorld)
		{
			ImGuiHelpers::ImGuiSliderFloatWithSetter("Trim Size",
			                                         TrimWorld->GetTrimWorldSize(), 1000.f, 3000.f,
			                                         [this](float InVal) { TrimWorld->SetTrimWorldSize(InVal); });
		}

		ImGui::Spacing();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();


		ImGuiHelpers::ImGuiSliderFloatWithSetter(
			"Seek",
			pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f,
			[this](float InVal)
			{
				pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = InVal;
			}, "%.2f");

		ImGuiHelpers::ImGuiSliderFloatWithSetter(
			"Wander",
			pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight, 0.f, 1.f,
			[this](float InVal)
			{
				pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight = InVal;
			}, "%.2f");

		//End
		ImGui::End();
	}
#pragma endregion
	
	pSeek->SetTarget(MouseTarget);
	
	FTargetData Target;
	Target.Position = pDrunkAgent->GetPosition();
	Target.Orientation = pDrunkAgent->GetRotation();
	Target.LinearVelocity = pDrunkAgent->GetLinearVelocity();
	Target.AngularVelocity = pDrunkAgent->GetAngularVelocity();
	
	pEvade->SetTarget(Target);
}
