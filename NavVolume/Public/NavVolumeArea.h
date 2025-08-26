// 

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "NavVolumeArea.generated.h"

class UBrushComponent;

UCLASS()
class NAVVOLUME_API ANavVolumeArea : public AVolume
{
	GENERATED_BODY()

public:
	ANavVolumeArea(const FObjectInitializer& ObjectInitializer);

	virtual void BeginPlay() override;
	virtual void PostRegisterAllComponents() override;
	virtual void PostUnregisterAllComponents() override;
	
#if WITH_EDITOR
	virtual void PostEditChangeProperty( struct FPropertyChangedEvent& PropertyChangedEvent) override;
	virtual void PostEditUndo() override;

	static void OnPostEngineInit();
#endif // WITH_EDITOR
};
