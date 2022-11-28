// Copyright Ben de Hullu & Coffee Stain Studios 2022. All Rights Reserved.


#include "InstancedSplineMeshComponent.h"
#include "Engine/StaticMesh.h"

UInstancedSplineMeshComponent::UInstancedSplineMeshComponent()
{
	// Setup defaults.
	/*	0,1,2.	Start XYZ		// TODO make this based on the instance pos instead.
	 *	3,4,5.	Start Tangent
	 *	6,7,8	End Pos
	 *	9,10,11	End Tangent
	 *	12		Length			// TODO read this from the X scale instead.
	 *	13						// TODO make this a static bool in the shader instead. */
	
	NumCustomDataFloats = 14;

	bRuntimeOnly = false;
}

FBoxSphereBounds UInstancedSplineMeshComponent::CalcBounds(const FTransform& BoundTransform) const
{
	// Compute bounding box, based on `USplineMeshComponent::CalcBounds`
	const bool IsInvalid = !GetStaticMesh() || GetInstanceCount() == 0;
	if ( IsInvalid )
	{
		return FBoxSphereBounds(FBox(ForceInit));
	}

	// Fist instance.
	FBoxSphereBounds RenderBounds = CalculateInstanceBounds(
		BoundTransform,
		GetInstanceStartPos(0),
		GetInstanceStartTangent(0),
		GetInstanceEndPos(0),
		GetInstanceEndTangent(0));

	for ( int32 i = 1; i < GetInstanceCount(); i++ )
	{
		RenderBounds = RenderBounds + CalculateInstanceBounds(
			BoundTransform,
			GetInstanceStartPos(i),
			GetInstanceStartTangent(i),
			GetInstanceEndPos(i),
			GetInstanceEndTangent(i) );
	}
	
	return RenderBounds;
}

FBoxSphereBounds UInstancedSplineMeshComponent::CalculateInstanceBounds(const FTransform& LocalToWorld, const FVector& Start, const FVector& StartTangent, const FVector& End, const FVector& EndTangent) const
{
	if (!GetStaticMesh())
	{
		return FBoxSphereBounds(FBox(ForceInit));
	}
	
	float MinT = 0.0f;
	float MaxT = 1.0f;

	const FBoxSphereBounds MeshBounds = GetStaticMesh()->GetBounds();
	
	const FVector AxisMask = GetAxisMask(ForwardAxis);
	const FVector FlattenedMeshOrigin = MeshBounds.Origin * AxisMask;
	const FVector FlattenedMeshExtent = MeshBounds.BoxExtent * AxisMask;
	const FBox MeshBoundingBox = FBox(FlattenedMeshOrigin - FlattenedMeshExtent, FlattenedMeshOrigin + FlattenedMeshExtent);
	
	FBox BoundingBox(ForceInit);
	BoundingBox += MeshBoundingBox.TransformBy(CalcTransformInstance(MinT, Start, StartTangent, End, EndTangent));
	BoundingBox += MeshBoundingBox.TransformBy(CalcTransformInstance(MaxT, Start, StartTangent, End, EndTangent));

	
	// Work out coefficients of the cubic spline derivative equation dx/dt
	const FVector A = 6.0f * Start + 3.0f * StartTangent + 3.0f * EndTangent - 6.0f * End;
	const FVector B = -6.0f * Start - 4.0f * StartTangent - 2.0f * EndTangent + 6.0f * End;
	const FVector C = StartTangent;

	// Minima/maxima happen where dx/dt == 0, calculate t values
	const FVector Discriminant = B * B - 4.0f * A * C;

	// Work out minima/maxima component-by-component.
	// Negative discriminant means no solution; A == 0 implies coincident start/end points
	if (Discriminant.X > 0.0f && !FMath::IsNearlyZero(A.X))
	{
		const float SqrtDiscriminant = FMath::Sqrt(Discriminant.X);
		const float Denominator = 0.5f / A.X;
		const float T0 = (-B.X + SqrtDiscriminant) * Denominator;
		const float T1 = (-B.X - SqrtDiscriminant) * Denominator;

		if (T0 >= MinT && T0 <= MaxT)
		{
			BoundingBox += MeshBoundingBox.TransformBy(CalcTransformInstance(T0, Start, StartTangent, End, EndTangent));
		}

		if (T1 >= MinT && T1 <= MaxT)
		{
			BoundingBox += MeshBoundingBox.TransformBy(CalcTransformInstance(T1, Start, StartTangent, End, EndTangent));
		}
	}

	if (Discriminant.Y > 0.0f && !FMath::IsNearlyZero(A.Y))
	{
		const float SqrtDiscriminant = FMath::Sqrt(Discriminant.Y);
		const float Denominator = 0.5f / A.Y;
		const float T0 = (-B.Y + SqrtDiscriminant) * Denominator;
		const float T1 = (-B.Y - SqrtDiscriminant) * Denominator;

		if (T0 >= MinT && T0 <= MaxT)
		{
			BoundingBox += MeshBoundingBox.TransformBy(CalcTransformInstance(T0, Start, StartTangent, End, EndTangent));
		}

		if (T1 >= MinT && T1 <= MaxT)
		{
			BoundingBox += MeshBoundingBox.TransformBy(CalcTransformInstance(T1, Start, StartTangent, End, EndTangent));
		}
	}

	if (Discriminant.Z > 0.0f && !FMath::IsNearlyZero(A.Z))
	{
		const float SqrtDiscriminant = FMath::Sqrt(Discriminant.Z);
		const float Denominator = 0.5f / A.Z;
		const float T0 = (-B.Z + SqrtDiscriminant) * Denominator;
		const float T1 = (-B.Z - SqrtDiscriminant) * Denominator;

		if (T0 >= MinT && T0 <= MaxT)
		{
			BoundingBox += MeshBoundingBox.TransformBy(CalcTransformInstance(T0, Start, StartTangent, End, EndTangent));
		}

		if (T1 >= MinT && T1 <= MaxT)
		{
			BoundingBox += MeshBoundingBox.TransformBy(CalcTransformInstance(T1, Start, StartTangent, End, EndTangent));
		}
	}

	return FBoxSphereBounds(BoundingBox.TransformBy(LocalToWorld));
}

FVector UInstancedSplineMeshComponent::GetInstanceStartPos(int32 Instance) const
{
	const int32 InstanceStartId = NumCustomDataFloats * Instance;
	const float X = PerInstanceSMCustomData[ InstanceStartId + NumPerInstancePrimitiveDataCountOffset + DefaultSplineShaderBindSettings.StartPositionPrimitiveDataStartIndex + 0 ];
	const float Y = PerInstanceSMCustomData[ InstanceStartId + NumPerInstancePrimitiveDataCountOffset + DefaultSplineShaderBindSettings.StartPositionPrimitiveDataStartIndex + 1 ];
	const float Z = PerInstanceSMCustomData[ InstanceStartId + NumPerInstancePrimitiveDataCountOffset + DefaultSplineShaderBindSettings.StartPositionPrimitiveDataStartIndex + 2 ];

	return FVector(X,Y,Z);
}

FVector UInstancedSplineMeshComponent::GetInstanceStartTangent(int32 Instance) const
{
	const int32 InstanceStartId = NumCustomDataFloats * Instance;
	const float X = PerInstanceSMCustomData[ InstanceStartId + NumPerInstancePrimitiveDataCountOffset + DefaultSplineShaderBindSettings.StartTangentPrimitiveDataStartIndex + 0 ];
	const float Y = PerInstanceSMCustomData[ InstanceStartId + NumPerInstancePrimitiveDataCountOffset + DefaultSplineShaderBindSettings.StartTangentPrimitiveDataStartIndex + 1 ];
	const float Z = PerInstanceSMCustomData[ InstanceStartId + NumPerInstancePrimitiveDataCountOffset + DefaultSplineShaderBindSettings.StartTangentPrimitiveDataStartIndex + 2 ];
	
	return FVector( X, Y, Z);
}

FVector UInstancedSplineMeshComponent::GetInstanceEndPos(int32 Instance) const
{
	const int32 InstanceStartId = NumCustomDataFloats * Instance;
	const float X = PerInstanceSMCustomData[ InstanceStartId + NumPerInstancePrimitiveDataCountOffset + DefaultSplineShaderBindSettings.EndPositionPrimitiveDataStartIndex + 0 ];
	const float Y = PerInstanceSMCustomData[ InstanceStartId + NumPerInstancePrimitiveDataCountOffset + DefaultSplineShaderBindSettings.EndPositionPrimitiveDataStartIndex + 1 ];
	const float Z = PerInstanceSMCustomData[ InstanceStartId + NumPerInstancePrimitiveDataCountOffset + DefaultSplineShaderBindSettings.EndPositionPrimitiveDataStartIndex + 2 ];
    
    return FVector( X, Y, Z);
}

FVector UInstancedSplineMeshComponent::GetInstanceEndTangent(int32 Instance) const
{
	const int32 InstanceStartId = NumCustomDataFloats * Instance;
	const float X = PerInstanceSMCustomData[ InstanceStartId + NumPerInstancePrimitiveDataCountOffset + DefaultSplineShaderBindSettings.EndTangentPrimitiveDataStartIndex + 0 ];
	const float Y = PerInstanceSMCustomData[ InstanceStartId + NumPerInstancePrimitiveDataCountOffset + DefaultSplineShaderBindSettings.EndTangentPrimitiveDataStartIndex + 1 ];
	const float Z = PerInstanceSMCustomData[ InstanceStartId + NumPerInstancePrimitiveDataCountOffset + DefaultSplineShaderBindSettings.EndTangentPrimitiveDataStartIndex + 2 ];
	
	return FVector( X, Y, Z);
}

static FVector SplineEvalPos(const FVector& StartPos, const FVector& StartTangent, const FVector& EndPos, const FVector& EndTangent, float A)
{
	const float A2 = A * A;
	const float A3 = A2 * A;

	return (((2 * A3) - (3 * A2) + 1) * StartPos) + ((A3 - (2 * A2) + A) * StartTangent) + ((A3 - A2) * EndTangent) + (((-2 * A3) + (3 * A2)) * EndPos);
}

static FVector SplineEvalDir(const FVector& StartPos, const FVector& StartTangent, const FVector& EndPos, const FVector& EndTangent, float A)
{
	const FVector C = (6 * StartPos) + (3 * StartTangent) + (3 * EndTangent) - (6 * EndPos);
	const FVector D = (-6 * StartPos) - (4 * StartTangent) - (2 * EndTangent) + (6 * EndPos);
	const FVector E = StartTangent;

	const float A2 = A * A;

	return ((C * A2) + (D * A) + E).GetSafeNormal();
}

FTransform UInstancedSplineMeshComponent::CalcTransformInstance( const float Alpha, const FVector& StartPos, const FVector& StartTangent, const FVector& EndPos, const FVector& EndTangent ) const
{
	// Then find the point and direction of the spline at this point along
	FVector SplinePos = SplineEvalPos(StartPos, StartTangent, EndPos, EndTangent, Alpha);
	const FVector SplineDir = SplineEvalDir(StartPos, StartTangent, EndPos, EndTangent, Alpha);

	// Find base frenet frame
	const FVector BaseXVec = (SplineUpDirection ^ SplineDir).GetSafeNormal();
	const FVector BaseYVec = (SplineDir ^ BaseXVec).GetSafeNormal();

	// We do NOT support Roll, Scale and offset right now
#if 0
	// Offset the spline by the desired amount
	const FVector2D SliceOffset = FMath::Lerp<FVector2D>(StartOffset, EndOffset, Alpha);
	SplinePos += SliceOffset.X * BaseXVec;
	SplinePos += SliceOffset.Y * BaseYVec;

	// Apply roll to frame around spline
	const float UseRoll = FMath::Lerp(StartRoll, EndRoll, Alpha);
	const float CosAng = FMath::Cos(UseRoll);
	const float SinAng = FMath::Sin(UseRoll);
	const FVector XVec = (CosAng * BaseXVec) - (SinAng * BaseYVec);
	const FVector YVec = (CosAng * BaseYVec) + (SinAng * BaseXVec);

	// Find scale at this point along spline
	const FVector2D UseScale = FMath::Lerp(SplineParams.StartScale, SplineParams.EndScale, HermiteAlpha);
#endif
	
	// Build overall transform
	FTransform SliceTransform;
	switch (ForwardAxis)
	{
	case ESplineMeshAxis::X:
		SliceTransform = FTransform(SplineDir, BaseXVec, BaseYVec, SplinePos);
		break;
	case ESplineMeshAxis::Y:
		SliceTransform = FTransform(BaseYVec, SplineDir, BaseXVec, SplinePos);
		break;
	case ESplineMeshAxis::Z:
		SliceTransform = FTransform(BaseXVec, BaseYVec, SplineDir, SplinePos);
		break;
	default:
		check(0);
		break;
	}

	return SliceTransform;
}

int32 UInstancedSplineMeshComponent::AddInstance(const FTransform& InstanceTransform)
{
	checkf( bValidAddFence, TEXT("Don't call AddInstance(const FTransform& InstanceTransform) on spline mesh components, call AddSplineInstance instead."));
	
	return Super::AddInstance(InstanceTransform);
}

void UInstancedSplineMeshComponent::ClearInstances()
{
	Super::ClearInstances();
	
	SplineInstances.Empty();
}

int32 UInstancedSplineMeshComponent::AddSplineInstance( const FVector StartPosition, const FVector StartTangent, const FVector EndPosition, const FVector EndTangent, const bool bMarkDirty)
{
	// Fence to ensure we add the instance in the correct way.
	SetAddFenceValid();
	
	const int32 Index = AddInstance( FTransform::Identity );
	SetupPerInstancePrimitiveData( Index, StartPosition, StartTangent, EndPosition, EndTangent );
	
	if ( bMarkDirty )
	{
		MarkRenderStateDirty();
	}
	
	// Invalidate the fence again.
	InvalidateAddFence();


	// add instance to array
	if (!bRuntimeOnly)
	{
		SplineInstances.Add( FSplineMeshInstanceEntry(StartPosition, StartTangent, EndPosition, EndTangent) );
	}
	
	return Index;
}

void UInstancedSplineMeshComponent::SetupPerInstancePrimitiveData(const int32 InstanceIndex, const FVector& StartPosition, const FVector StartTangent, const FVector& EndPosition, const FVector& EndTangent)
{
	const int32 StartPosIds[ 3 ] = {
		DefaultSplineShaderBindSettings.StartPositionPrimitiveDataStartIndex + NumPerInstancePrimitiveDataCountOffset + 0,
		DefaultSplineShaderBindSettings.StartPositionPrimitiveDataStartIndex + NumPerInstancePrimitiveDataCountOffset + 1,
		DefaultSplineShaderBindSettings.StartPositionPrimitiveDataStartIndex + NumPerInstancePrimitiveDataCountOffset + 2 };

	const int32 StartTangentIds[ 3 ] = {
		DefaultSplineShaderBindSettings.StartTangentPrimitiveDataStartIndex + NumPerInstancePrimitiveDataCountOffset + 0,
		DefaultSplineShaderBindSettings.StartTangentPrimitiveDataStartIndex + NumPerInstancePrimitiveDataCountOffset + 1,
		DefaultSplineShaderBindSettings.StartTangentPrimitiveDataStartIndex + NumPerInstancePrimitiveDataCountOffset + 2 };

	const int32 EndPosIds[ 3 ] = {
		DefaultSplineShaderBindSettings.EndPositionPrimitiveDataStartIndex + NumPerInstancePrimitiveDataCountOffset + 0,
		DefaultSplineShaderBindSettings.EndPositionPrimitiveDataStartIndex + NumPerInstancePrimitiveDataCountOffset + 1,
		DefaultSplineShaderBindSettings.EndPositionPrimitiveDataStartIndex + NumPerInstancePrimitiveDataCountOffset + 2 };

	const int32 EndTangentIds[ 3 ] = {
		DefaultSplineShaderBindSettings.EndTangentPrimitiveDataStartIndex + NumPerInstancePrimitiveDataCountOffset + 0,
		DefaultSplineShaderBindSettings.EndTangentPrimitiveDataStartIndex + NumPerInstancePrimitiveDataCountOffset + 1,
		DefaultSplineShaderBindSettings.EndTangentPrimitiveDataStartIndex + NumPerInstancePrimitiveDataCountOffset + 2 };

	// Start pos
	SetCustomDataValue( InstanceIndex,StartPosIds[0], StartPosition.X,false);
	SetCustomDataValue( InstanceIndex,StartPosIds[1], StartPosition.Y,false);
	SetCustomDataValue( InstanceIndex,StartPosIds[2], StartPosition.Z,false);

	// Start tangent
	SetCustomDataValue( InstanceIndex,StartTangentIds[0], StartTangent.X,false);
	SetCustomDataValue( InstanceIndex,StartTangentIds[1], StartTangent.Y,false);
	SetCustomDataValue( InstanceIndex,StartTangentIds[2], StartTangent.Z,false);

	// End pos
	SetCustomDataValue( InstanceIndex,EndPosIds[0], EndPosition.X,false);
	SetCustomDataValue( InstanceIndex,EndPosIds[1], EndPosition.Y,false);
	SetCustomDataValue( InstanceIndex,EndPosIds[2], EndPosition.Z,false);

	// End tangent
	SetCustomDataValue( InstanceIndex,EndTangentIds[0], EndTangent.X,false);
	SetCustomDataValue( InstanceIndex,EndTangentIds[1], EndTangent.Y,false);
	SetCustomDataValue( InstanceIndex,EndTangentIds[2], EndTangent.Z,false);
}

FVector UInstancedSplineMeshComponent::GetAxisMask(ESplineMeshAxis::Type InAxis)
{
	switch (InAxis)
	{
	case ESplineMeshAxis::X:
		return FVector(0, 1, 1);
	case ESplineMeshAxis::Y:
		return FVector(1, 0, 1);
	case ESplineMeshAxis::Z:
		return FVector(1, 1, 0);
	default:
		check(0);
		return FVector::ZeroVector;
	}
}
