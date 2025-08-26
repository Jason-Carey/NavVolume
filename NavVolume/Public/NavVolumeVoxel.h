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
	template<int32 VoxelSize>
	FORCEINLINE FVector SnapToVoxelAxis(const FVector& Point) 
	{
		return FVector(
			FMath::RoundToInt(Point.X * TVoxelTraits<VoxelSize>::InvVoxelSize) * VoxelSize,
			FMath::RoundToInt(Point.Y * TVoxelTraits<VoxelSize>::InvVoxelSize) * VoxelSize,
			FMath::RoundToInt(Point.Z * TVoxelTraits<VoxelSize>::InvVoxelSize) * VoxelSize
		);
	}

	/** Snaps an AABB to the voxel grid - will expand the boundary by half voxel size to ensure full mesh containment. */
	template<int32 VoxelSize>
	FORCEINLINE FBox SnapToVoxelAxis(const FBox& AABB) 
	{
		return FBox(
			SnapToVoxelAxis<VoxelSize>(AABB.Min - TVoxelTraits<VoxelSize>::HalfVoxelSize),
			SnapToVoxelAxis<VoxelSize>(AABB.Max + TVoxelTraits<VoxelSize>::HalfVoxelSize)
		);
	}

	template<int32 VoxelSize, typename VectorType>
	FORCEINLINE FIntVector QuantizeVoxel(const VectorType& Vector)
	{
		return FIntVector(
			static_cast<float>(Vector.X) * TVoxelTraits<VoxelSize>::InvVoxelSize,
			static_cast<float>(Vector.Y) * TVoxelTraits<VoxelSize>::InvVoxelSize,
			static_cast<float>(Vector.Z) * TVoxelTraits<VoxelSize>::InvVoxelSize
		);
	}

	template<int32 VoxelSize, typename VectorType>
	FORCEINLINE FIntVector DequantizeVoxel(const VectorType& Vector)
	{
		return FIntVector(
			static_cast<float>(Vector.X) * VoxelSize,
			static_cast<float>(Vector.Y) * VoxelSize,
			static_cast<float>(Vector.Z) * VoxelSize
		);
	}

	/** If an appropriate test exists for the passed shape it will be voxelized by transforming bounded world coordinates into local space and testing intersection. */
	template<int32 VoxelSize, typename ShapeType, typename ForEachFunc>
	void Voxelize(const ShapeType& Shape, const FTransform& WorldTransform, const FBox& WorldBounds, ForEachFunc&& ForEachVoxel)
	{
		const TShapeTest<VoxelSize, ShapeType>ShapeTest(Shape);
		
		// FTransform NormTransform = (Shape.GetTransform() * WorldTransform);
		// NormTransform.NormalizeRotation();
		
		// The geometry is at the origin with identity rotation/scale
		// Calculate the inverse matrix to transform voxels from world space into the geometry's local space
		// NOTE: Non-uniform scaling necessitates using FMatrix over FTransform for this use case
		const FMatrix LocalToWorld = (Shape.GetTransform() * WorldTransform).ToMatrixWithScale();
		const FMatrix WorldToLocal = LocalToWorld.Inverse();

		// Expand the AABB to align with the voxel grid
		const FBox VoxelBounds = SnapToVoxelAxis<VoxelSize>(WorldBounds);
		
		const FIntVector VoxelMin = FIntVector(VoxelBounds.Min);
		const FIntVector VoxelMax = FIntVector(VoxelBounds.Max);

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

	template<int32 VoxelSize, typename ShapeType, typename AllocatorType, typename ForEachFunc>
	void Voxelize(const TArray<ShapeType, AllocatorType>& Shapes, const FTransform& WorldTransform, const FBox& WorldBounds, ForEachFunc&& ForEachVoxel)
	{
		for(const ShapeType& Shape : Shapes)
		{
			Voxelize<VoxelSize, ShapeType>(Shape, WorldTransform, WorldBounds, Forward<ForEachFunc>(ForEachVoxel));
		}
	}
} // namespace NavVolume::Voxel
