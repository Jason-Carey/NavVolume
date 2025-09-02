#pragma once

#include "Engine/OverlapResult.h"
#include "PhysicsEngine/BodySetup.h"

namespace NavVolume::Voxel
{
	template <int32 VoxelSize>
	struct TVoxelTraits
	{
		static_assert(VoxelSize % 2 == 0, "VoxelSize must be an even number.");
		static constexpr int32 HalfVoxelSize = VoxelSize / 2.0;
		static constexpr float InvVoxelSize = 1.0 / VoxelSize;
	};

	template<int32 VoxelSize, typename ShapeType>
	struct TShapeTest
	{
		static_assert(!std::is_same_v<ShapeType, ShapeType>, "ShapeTest does not have a specialization for this type.");
	};
	
	template<int32 VoxelSize>
	struct TShapeTest<VoxelSize, FKSphereElem>
	{
		float RadiusSquared;
			
		TShapeTest(const FKSphereElem& SphereShape)
		{
			RadiusSquared = FMath::Square(SphereShape.Radius + TVoxelTraits<VoxelSize>::HalfVoxelSize);
		}

		FORCEINLINE bool IsInside(const FVector& LocalPosition) const
		{
			return LocalPosition.SizeSquared() <= RadiusSquared;
		}
	};

	template<int32 VoxelSize>
	struct TShapeTest<VoxelSize, FKBoxElem>
	{
		float HalfExtentX;
		float HalfExtentY;
		float HalfExtentZ;
	
		TShapeTest(const FKBoxElem& BoxShape)
		{
			HalfExtentX = BoxShape.X * 0.5 + TVoxelTraits<VoxelSize>::HalfVoxelSize;
			HalfExtentY = BoxShape.Y * 0.5 + TVoxelTraits<VoxelSize>::HalfVoxelSize;
			HalfExtentZ = BoxShape.Z * 0.5 + TVoxelTraits<VoxelSize>::HalfVoxelSize;
		}
			
		FORCEINLINE bool IsInside(const FVector& LocalPosition) const
		{
			return FMath::Abs(LocalPosition.X) <= HalfExtentX &&
				   FMath::Abs(LocalPosition.Y) <= HalfExtentY &&
				   FMath::Abs(LocalPosition.Z) <= HalfExtentZ ;
		}
	};

	template<int32 VoxelSize>
	struct TShapeTest<VoxelSize, FKSphylElem>
	{
		float RadiusSquared;
		float HalfLength;
			
		TShapeTest(const FKSphylElem& CapsuleShape)
		{
			RadiusSquared = FMath::Square(CapsuleShape.Radius + TVoxelTraits<VoxelSize>::HalfVoxelSize);
			HalfLength = 0.5 * CapsuleShape.Length; // This is not extended because RadiusSquared already considers the voxel overestimation
		}

		FORCEINLINE bool IsInside(const FVector& LocalPosition) const
		{
			const FVector ClosestPointOnLine(0, 0, FMath::Clamp(LocalPosition.Z, -HalfLength, HalfLength));
			return (ClosestPointOnLine - LocalPosition).SizeSquared() <= RadiusSquared;
		}
	};

	template<int32 VoxelSize>
	struct TShapeTest<VoxelSize, FKConvexElem>
	{
		FConvexVolume ConvexVolume;
			
		TShapeTest(const FKConvexElem& ConvexShape)
		{
			// Temp plane storage as ConvexVolume uses an InlineAllocator
			TArray<FPlane> TempPlanes;
			ConvexShape.GetPlanes(TempPlanes);
			ConvexVolume.Planes.Append(MoveTemp(TempPlanes));
			ConvexVolume.Init();
		}

		FORCEINLINE bool IsInside(const FVector& LocalPosition) const
		{
			return ConvexVolume.IntersectSphere(LocalPosition, TVoxelTraits<VoxelSize>::HalfVoxelSize);
		}
	};
	
	/** Snaps a point to the voxel grid - axis aligned. */
	template<int32 VoxelSize, typename VectorType>
	FORCEINLINE FIntVector SnapToVoxelAxis(const VectorType& Vector) 
	{
		return FIntVector(
			FMath::RoundToInt(Vector.X * TVoxelTraits<VoxelSize>::InvVoxelSize) * VoxelSize,
			FMath::RoundToInt(Vector.Y * TVoxelTraits<VoxelSize>::InvVoxelSize) * VoxelSize,
			FMath::RoundToInt(Vector.Z * TVoxelTraits<VoxelSize>::InvVoxelSize) * VoxelSize
		);
	}
	
	/** Reduce the voxel so that each axis-increment is equal to the voxel size. */
	template<int32 VoxelSize, typename VectorType>
	FORCEINLINE FIntVector QuantizeVoxel(const VectorType& Vector)
	{
		return FIntVector(
			Vector.X * TVoxelTraits<VoxelSize>::InvVoxelSize,
			Vector.Y * TVoxelTraits<VoxelSize>::InvVoxelSize,
			Vector.Z * TVoxelTraits<VoxelSize>::InvVoxelSize
		);
	}
	
	/** Expand the voxel to its world space position. */
	template<int32 VoxelSize, typename VectorType>
	FORCEINLINE FIntVector DequantizeVoxel(const VectorType& Vector)
	{
		return FIntVector(
			Vector.X * VoxelSize,
			Vector.Y * VoxelSize,
			Vector.Z * VoxelSize
		);
	}

	/** If an appropriate test exists for the passed shape it will be voxelized by transforming bounded world coordinates into local space and testing intersection. */
	template<int32 VoxelSize, typename ShapeType, typename TransformType, typename BoundsType, typename ForEachFunc>
	void Voxelize(const ShapeType& Shape, TransformType&& WorldTransform, BoundsType&& WorldBounds, ForEachFunc&& ForEachVoxel)
	{
		const TShapeTest<VoxelSize, ShapeType>ShapeTest(Shape);
		
		// Calculate the inverse matrix to transform voxels from world space into the geometry's local space
		// NOTE: Non-uniform scaling necessitates using FMatrix over FTransform for this use case
		const FMatrix LocalToWorld = (Shape.GetTransform() * WorldTransform).ToMatrixWithScale();
		const FMatrix WorldToLocal = LocalToWorld.Inverse();

		// Expand the AABB to align with the voxel grid
		const FIntVector VoxelMin = SnapToVoxelAxis<VoxelSize>(WorldBounds.Min - TVoxelTraits<VoxelSize>::HalfVoxelSize);
		const FIntVector VoxelMax = SnapToVoxelAxis<VoxelSize>(WorldBounds.Max + TVoxelTraits<VoxelSize>::HalfVoxelSize);

		for (int32 X = VoxelMin.X; X <= VoxelMax.X; X += VoxelSize)
		for (int32 Y = VoxelMin.Y; Y <= VoxelMax.Y; Y += VoxelSize)
		for (int32 Z = VoxelMin.Z; Z <= VoxelMax.Z; Z += VoxelSize)
		{
			const FVector WorldPosition = FVector(X, Y, Z);
			const FVector LocalPosition = WorldToLocal.TransformPosition(WorldPosition);
			
			if (ShapeTest.IsInside(LocalPosition))
			{
				ForEachVoxel(FIntVector(WorldPosition));
			}
		}
	}

	/** Transform an array of shapes sharing a transform and boundary into a voxel representation. */
	template<int32 VoxelSize, typename ShapeType, typename AllocatorType, typename TransformType, typename BoundsType, typename ForEachFunc>
	void Voxelize(const TArray<ShapeType, AllocatorType>& Shapes, TransformType&& WorldTransform, BoundsType&& WorldBounds, ForEachFunc&& ForEachVoxel)
	{
		for(const ShapeType& Shape : Shapes)
		{
			Voxelize<VoxelSize, ShapeType>(
				Shape,
				Forward<TransformType>(WorldTransform),
				Forward<BoundsType>(WorldBounds),
				Forward<ForEachFunc>(ForEachVoxel)
			);
		}
	}
} // namespace NavVolume::Voxel
