//

#include "NavVolumeArea.h"
#include "NavVolumeSubsystem.h"
#include "Components/BrushComponent.h"

ANavVolumeArea::ANavVolumeArea(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
	UBrushComponent* BrushComp = GetBrushComponent();
	BrushComp->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
	BrushComp->Mobility = EComponentMobility::Static;

	BrushColor = FColor(200, 200, 200, 255);
	bColored = true;

#if WITH_EDITORONLY_DATA
	// bIsSpatiallyLoaded = false;
#endif
}

void ANavVolumeArea::BeginPlay()
{
	Super::BeginPlay();
	GetWorld()->GetSubsystem<UNavVolumeSubsystem>()->CreateNavigableVolume(GetBrushComponent()->Bounds.GetBox());
}

void ANavVolumeArea::PostRegisterAllComponents()
{
	Super::PostRegisterAllComponents();
}

void ANavVolumeArea::PostUnregisterAllComponents()
{
	Super::PostUnregisterAllComponents();
}

void ANavVolumeArea::PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
}

void ANavVolumeArea::PostEditUndo()
{
	Super::PostEditUndo();
}

void ANavVolumeArea::OnPostEngineInit()
{
	
}

