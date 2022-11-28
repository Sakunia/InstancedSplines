// Copyright Ben de Hullu & Coffee Stain Studios 2022. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/SplineMeshComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Launch/Resources/Version.h"
#include "InstancedSplineMeshComponent.generated.h"

/* Default data that only lives on the CDO.*/
USTRUCT()
struct INSTANCEDSPLINESCOMPONENT_API FSplineDataSettings
{
	GENERATED_BODY()

public:
	UPROPERTY( EditDefaultsOnly )
	int32 StartPositionPrimitiveDataStartIndex;
	
	UPROPERTY( EditDefaultsOnly )
	int32 StartTangentPrimitiveDataStartIndex;

	UPROPERTY( EditDefaultsOnly )
	int32 EndPositionPrimitiveDataStartIndex;
	
	UPROPERTY( EditDefaultsOnly )
	int32 EndTangentPrimitiveDataStartIndex;
};

USTRUCT()
struct INSTANCEDSPLINESCOMPONENT_API FSplineMeshInstanceEntry
{
	GENERATED_BODY()

public:
	UPROPERTY(EditDefaultsOnly)
	FVector StartPos;

	UPROPERTY(EditDefaultsOnly)
	FVector StartTangent;

	UPROPERTY(EditDefaultsOnly)
	FVector EndPos;
	
	UPROPERTY(EditDefaultsOnly)
	FVector EndTangent;

	FSplineMeshInstanceEntry() :
		StartPos(ForceInitToZero),
		StartTangent(ForceInitToZero),
		EndPos(ForceInitToZero),
		EndTangent(ForceInitToZero)
	{ }
	
	FSplineMeshInstanceEntry(const FVector& inStartPos, const FVector& inStartTangent, const FVector& inEndPos, const FVector& inEndTangent) :
		StartPos(inStartPos),
		StartTangent(inStartTangent),
		EndPos(inEndPos),
		EndTangent(inEndTangent)
	{ }

	FSplineMeshInstanceEntry(const FTransform& inStartPos, const FVector& inStartTangent, const FVector& inEndPos, const FVector& inEndTangent) :
		StartPos(inStartPos.GetLocation()),
		StartTangent(inStartTangent),
		EndPos(inEndPos),
		EndTangent(inEndTangent)
	{ }
};

/**
 * 
 */
UCLASS(ClassGroup = Rendering, meta = (BlueprintSpawnableComponent), Blueprintable)
class INSTANCEDSPLINESCOMPONENT_API UInstancedSplineMeshComponent : public UInstancedStaticMeshComponent
{
	GENERATED_BODY()
	
	UInstancedSplineMeshComponent();

	/* Begin USceneComponent Interface */
	virtual FBoxSphereBounds CalcBounds(const FTransform& BoundTransform) const override;
	/* End USceneComponent Interface*/

	/* Begin UInstancedStaticMeshComponent Interface */
	
	/* Native add instance, 
	 * Implemented a checkf with a fence to ensure the user calls the correct function to add instances. */
#if ENGINE_MAJOR_VERSION == 4
	virtual int32 AddInstance(const FTransform& InstanceTransform) override;
#else
	virtual int32 AddInstance(const FTransform& InstanceTransform, bool bWorldSpace = false) override;
#endif
	
	virtual void ClearInstances() override;
	/* Begin USceneComponent Interface */
	
	// Gets start position from per instance custom data.
	UFUNCTION(BlueprintCallable, Category = "Components|InstanceSplineMeshComponent")
	FVector GetInstanceStartPos(int32 Instance) const;

	// Gets start tangent from per instance custom data.
	UFUNCTION(BlueprintCallable, Category = "Components|InstanceSplineMeshComponent")
	FVector GetInstanceStartTangent(int32 Instance) const;
	
	// Gets end position from per instance custom data.
	UFUNCTION(BlueprintCallable, Category = "Components|InstanceSplineMeshComponent")
	FVector GetInstanceEndPos(int32 Instance) const;

	// Gets end tangent from per instance custom data.
	UFUNCTION(BlueprintCallable, Category = "Components|InstanceSplineMeshComponent")
	FVector GetInstanceEndTangent(int32 Instance) const;
	
	// Add instance.
	UFUNCTION(BlueprintCallable, Category = "Components|InstanceSplineMeshComponent")
	virtual int32 AddSplineInstance(const FVector StartPosition, const FVector StartTangent, const FVector EndPosition, const FVector EndTangent, const bool bMarkDirty = true);

protected:
	/* Calculates the instance transform on the spline.*/
	FTransform CalcTransformInstance( const float Alpha, const FVector& StartPos, const FVector& StartTangent, const FVector& EndPos, const FVector& EndTangent ) const;

	/* Calculate the bound of the instance. */
	FBoxSphereBounds CalculateInstanceBounds(const FTransform& LocalToWorld, const FVector& Start, const FVector& StartTangent, const FVector& End, const FVector& EndTangent) const;

	/* Sets up the primitive data for this specific instance. assigns the values to the per instance custom primitive data with the user assigned offset value.*/
	virtual void SetupPerInstancePrimitiveData(const int32 InstanceIndex, const FVector& StartPosition, const FVector StartTangent, const FVector& EndPosition, const FVector& EndTangent );

private:
	/* helper function */
	static FVector GetAxisMask(ESplineMeshAxis::Type InAxis);

public:
	/* Forward vector. */
	UPROPERTY(EditAnywhere, Category=SplineMesh)
	TEnumAsByte<ESplineMeshAxis::Type> ForwardAxis;
	
	FVector SplineUpDirection = FVector(0,0,1);

	// TODO, do we need this? could i store this in the per instance custom data
	// Could be done with detail panel override or just editor only data.
	/* Array of the instances. */
	UPROPERTY(EditAnywhere, Category="Instances")
	TArray<FSplineMeshInstanceEntry> SplineInstances;

protected:
	/* The start offset value for the per instance primitive data, use full for when dealing with other per instance primitive data entries.*/
	UPROPERTY(EditDefaultsOnly)
	int32 NumPerInstancePrimitiveDataCountOffset = 0;
	
	// TODO ensure we only have this once per instance.
	UPROPERTY(EditDefaultsOnly)
	FSplineDataSettings DefaultSplineShaderBindSettings;

	
private:
	// TODO make defines for these functions so we can just compile them out.
	FORCEINLINE void InvalidateAddFence()	{ bValidAddFence = false; }
	FORCEINLINE void SetAddFenceValid()		{ bValidAddFence = true; }
	
	/* used to check if the instances are added in the correct way. */
	uint8 bValidAddFence:1;

	// TODO
	uint8 bRuntimeOnly:1;
};
