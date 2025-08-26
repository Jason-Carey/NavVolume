// 

// ReSharper disable CppPassValueParameterByConstReference
#pragma once

#include "CoreMinimal.h"

#include "Tasks/Task.h"
#include "Tasks/Pipe.h"
#include "Algo/Unique.h"
#include "Subsystems/WorldSubsystem.h"

#include "NavVolumeVoxel.h"
#include "NavVolumeMorton.h"
#include "NavVolumeOctree.h"

#include "NavVolumeSubsystem.generated.h"

namespace NavVolume::Task
{
	using namespace NavVolume::Voxel;
	using namespace NavVolume::Morton;
	using namespace UE::Tasks;

	template<int32 N>
	using TMakeIndexSequence = TMakeIntegerSequence<int32, N>; // Can't find a UE5 alias for std::make_index_sequence

	template<int32... Indices>
	using TIndexSequence = TIntegerSequence<int32, Indices...>;

	template<int32 VoxelSize, typename... ArgTypes>
	struct TVoxelizer;

	template<int32 VoxelSize>
	struct TVoxelizer<VoxelSize>
	{
		template<typename TransformType, typename BoundsType, typename... ArgTypes>
		static decltype(auto) MakeVoxelizer(TransformType&& InTransform, BoundsType&& InBounds, ArgTypes&&... InArgs)
		{
			return TVoxelizer<VoxelSize, TransformType, BoundsType, ArgTypes...>(
				Forward<TransformType>(InTransform),
				Forward<BoundsType>(InBounds),
				ForwardAsTuple(Forward<ArgTypes>(InArgs)...) // This should preserve lvalue and rvalues
			);
		}
	};
	
	template<int32 VoxelSize, typename TransformType, typename BoundsType, typename... ArgTypes>
	struct TVoxelizer<VoxelSize, TransformType, BoundsType, ArgTypes...>
	{
		/** Encodes each voxel-collider intersection as a quantized morton code. */
		TArray<FMortonCode> operator()()
		{
			MortonCodes.Reset(); // Prepare for re-use as the data is being moved
			Execute(TMakeIndexSequence<TTupleArity<decltype(Args)>::Value>());
			return MoveTemp(MortonCodes);
		}

	private:
		friend struct TVoxelizer<VoxelSize>;
		
		/** Constructs a TVoxelizer with type forwarding such that it can support pass-by-reference or pass-by-value for each argument. */
		TVoxelizer(TransformType&& InTransform, BoundsType&& InBounds, TTuple<ArgTypes...>&& InArgs)
			: Transform(Forward<TransformType>(InTransform))
			, Bounds(Forward<BoundsType>(InBounds))
			, Args(MoveTemp(InArgs)) { } 

		/** Operates as a 'for each' loop on tuple values calling NavVolume::Voxel::Voxelize on each type. */
		template<size_t... Indices>
		void Execute(TIndexSequence<Indices...>) 
		{
			const auto EncodeVoxel = [&](const FIntVector& WorldPosition)
			{
				const FMortonCode MortonCode = EncodeMorton(QuantizeVoxel<VoxelSize>(WorldPosition));
				MortonCodes.Add(MortonCode);
			};
			
			(Voxelize<VoxelSize>(Args.template Get<Indices>(), Forward<TransformType>(Transform), Forward<BoundsType>(Bounds), EncodeVoxel), ...);
		}
		
		TransformType Transform;
		BoundsType Bounds;
		TTuple<ArgTypes...> Args;
		TArray<FMortonCode> MortonCodes;
	};

	template<int32 VoxelSize>
	class TAsyncVoxelizer
	{
	public:
		using FVoxelizerHandle = TTask<TArray<FMortonCode>>;
		
		void Enqueue(const FKAggregateGeom& AggGeom, FTransform&& WorldTransform, FBox&& WorldBounds)
		{
			const FVoxelizerHandle TaskHandle = Launch(
				UE_SOURCE_LOCATION,
				TVoxelizer<VoxelSize>::MakeVoxelizer(MoveTemp(WorldTransform), MoveTemp(WorldBounds), AggGeom.BoxElems, AggGeom.ConvexElems, AggGeom.SphereElems, AggGeom.SphylElems)
			);
			
			TaskHandles.Add(TaskHandle);
		}

		void Run()
		{
			//Launch(UE_SOURCE_LOCATION,
			//[&]
			//{
				for (int32 Index = 0; Index < TaskHandles.Num(); ++Index)
				{
					TArray<FMortonCode> TempMortonCodes = TaskHandles[Index].GetResult();
					MortonCodes.Append(MoveTemp(TempMortonCodes));
				}
					
				MortonCodes.Sort();
				MortonCodes.SetNum(Algo::Unique(MortonCodes));  UE_LOG(LogTemp, Warning, TEXT("Morton Codes: %d"), MortonCodes.Num());
			//}
			//, TaskHandles);
		}
		
	//private:
		TArray<FMortonCode> MortonCodes;
		TArray<FVoxelizerHandle> TaskHandles;
	};
}

/**
 * 
 */
UCLASS()
class NAVVOLUME_API UNavVolumeSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	static constexpr int32 VoxelSize = 32;
	
	using FSparseVoxelOctree = NavVolume::Octree::TSparseVoxelOctree<VoxelSize>;
	using FAsyncVoxelizer = NavVolume::Task::TAsyncVoxelizer<VoxelSize>;
	FAsyncVoxelizer AsyncVoxelizer;
	
	TArray<FOverlapResult> GetBoxOverlaps(const FVector& Center, const FVector& Extents, const FQuat& Rotation, const ECollisionChannel Channel)
	{
		TArray<FOverlapResult> Overlaps;
		GetWorld()->OverlapMultiByChannel(Overlaps, Center, Rotation, Channel, FCollisionShape::MakeBox(Extents));
		return MoveTemp(Overlaps);
	}

	FORCEINLINE TArray<FOverlapResult> GetBoxOverlaps(const FBox& AABB, const ECollisionChannel Channel)
	{
		return GetBoxOverlaps(AABB.GetCenter(), AABB.GetExtent(), FQuat::Identity, Channel);
	}
	
	void CreateNavigableVolume(const FBox& WorldBounds)
	{
		TArray<FOverlapResult> Overlaps = GetBoxOverlaps(WorldBounds, ECC_WorldStatic);

		for (const FOverlapResult& Overlap : Overlaps)
		{
			INavRelevantInterface* Interface = Cast<INavRelevantInterface>(Overlap.Component);
	
			if (Interface != nullptr && Interface->IsNavigationRelevant())
			{
				AsyncVoxelizer.Enqueue(
					Interface->GetNavigableGeometryBodySetup()->AggGeom,
					Interface->GetNavigableGeometryTransform(),
					Interface->GetNavigationBounds()
				);
			}
		}
		
		AsyncVoxelizer.Run();


		const FSparseVoxelOctree Octree(MoveTemp(AsyncVoxelizer.MortonCodes));
		Octree.DebugDraw(GetWorld());
	}
};


/*



template<int32 VoxelSize>
struct TVoxelizeTask
{
FKAggregateGeom& AggGeom;
FTransform WorldTransform;
FBox WorldBounds;
TArray<FMortonCode>& MortonCodes;
		
void operator()()
{
// Allocate approximate memory usage
const FVector Difference = WorldBounds.Max - WorldBounds.Min;
const int32 NumVoxels = FMath::Abs((Difference.X * Difference.Y * Difference.Z) * TVoxelTraits<VoxelSize>::InvVoxelSize);
MortonCodes.Reserve(MortonCodes.Num() + NumVoxels);

// Voxelize each shape
Voxelize(AggGeom.SphereElems);
Voxelize(AggGeom.BoxElems);
Voxelize(AggGeom.SphylElems);
Voxelize(AggGeom.ConvexElems);


MortonCodes.Sort();

UE_LOG(LogTemp, Warning, TEXT("%lld"), MortonCodes[0]);

for (int32 Index = 1; Index < MortonCodes.Num(); ++Index)
{
const bool bIsSame = MortonCodes[Index] == MortonCodes[Index - 1];

if (bIsSame)
{
UE_LOG(LogTemp, Error, TEXT("%lld"), MortonCodes[Index]);
}
else
{
UE_LOG(LogTemp, Warning, TEXT("%lld"), MortonCodes[Index]);
}
}

UE_LOG(LogTemp, Warning, TEXT("Voxel Task End, Morton Code Num: %d"), MortonCodes.Num());
}

private:
template<typename ShapeType, typename AllocatorType>
void Voxelize(const TArray<ShapeType, AllocatorType>& Shapes)
{
auto EncodeVoxel = [&](const FIntVector& WorldPosition)
{
const FMortonCode MortonCode = EncodeMorton(QuantizeVoxel<VoxelSize>(WorldPosition));
MortonCodes.Add(MortonCode);
};
			
for(const ShapeType& Shape : Shapes)
{
VoxelizeShape<VoxelSize, ShapeType>(Shape, WorldTransform, WorldBounds, EncodeVoxel);
}
}
};

template<int32 VoxelSize>
	class TVoxelizer2
	{
	public:
		TArray<FMortonCode> operator()()
		{
			Execute(AggGeom.BoxElems);
			return MoveTemp(MortonCodes);
		}

	private:
		template<typename ShapeType, typename AllocatorType>
		void Execute(const TArray<ShapeType, AllocatorType>& Shapes)
		{
			auto EncodeVoxel = [&](const FIntVector& WorldPosition)
			{
				const FMortonCode MortonCode = EncodeMorton(QuantizeVoxel<VoxelSize>(WorldPosition));
				MortonCodes.Add(MortonCode);
			};
			
			Voxelize<VoxelSize>(Shapes, WorldTransform, WorldBounds, EncodeVoxel);
		}
		
	public:
		const FKAggregateGeom& AggGeom;
		const FTransform WorldTransform;
		const FBox WorldBounds;
		TArray<FMortonCode> MortonCodes;
	};

*/