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
		using FReturnType = TArray<FMortonCode>;
		using FTaskHandle = TTask<FReturnType>;
		
		template<typename TransformType, typename BoundsType, typename... ArgTypes>
		static decltype(auto) MakeVoxelizer(TransformType&& InTransform, BoundsType&& InBounds, ArgTypes&&... InArgs)
		{
			return TVoxelizer<VoxelSize, TransformType, BoundsType, ArgTypes...>(
				Forward<TransformType>(InTransform),
				Forward<BoundsType>(InBounds),
				ForwardAsTuple(Forward<ArgTypes>(InArgs)...) // This should preserve lvalue and rvalues
			);
		}

		template<typename TransformType, typename BoundsType, typename... ArgTypes>
		static FORCEINLINE FTaskHandle LaunchVoxelizerAsync(TransformType&& InTransform, BoundsType&& InBounds, ArgTypes&&... InArgs)
		{
			auto Voxelizer = MakeVoxelizer(
				Forward<TransformType>(InTransform),
				Forward<BoundsType>(InBounds),
				Forward<ArgTypes>(InArgs)...
			);

			return Launch(UE_SOURCE_LOCATION, MoveTemp(Voxelizer));
		}

		template<typename TransformType, typename BoundsType, typename... ArgTypes>
		static FORCEINLINE FReturnType LaunchVoxelizer(TransformType&& InTransform, BoundsType&& InBounds, ArgTypes&&... InArgs)
		{
			auto Voxelizer = MakeVoxelizer(
				Forward<TransformType>(InTransform),
				Forward<BoundsType>(InBounds),
				Forward<ArgTypes>(InArgs)...
			);

			return Voxelizer();
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
	using FVoxelizer = NavVolume::Task::TVoxelizer<VoxelSize>;
	
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
		TArray<FVoxelizer::FTaskHandle> TaskHandles;
		TaskHandles.Reserve(Overlaps.Num());
		
		for (const FOverlapResult& Overlap : Overlaps)
		{
			INavRelevantInterface* Interface = Cast<INavRelevantInterface>(Overlap.Component);
			
			if (Interface != nullptr && Interface->IsNavigationRelevant())
			{
				const FKAggregateGeom& AggGeom = Interface->GetNavigableGeometryBodySetup()->AggGeom;
				
				FVoxelizer::FTaskHandle TaskHandle = FVoxelizer::LaunchVoxelizerAsync(
					Interface->GetNavigableGeometryTransform(),
					Interface->GetNavigationBounds(),
					AggGeom.BoxElems, AggGeom.ConvexElems, AggGeom.SphereElems, AggGeom.SphylElems
				);
				
				TaskHandles.Add(TaskHandle);
			}
		}

		// This is all just temporary - it's a huge sync point
		// Can be launched as a task with TaskHandles as a dependency 
		FVoxelizer::FReturnType MortonCodes;
		
		for (int32 Index = 0; Index < TaskHandles.Num(); ++Index)
		{
			FVoxelizer::FReturnType TempMortonCodes = TaskHandles[Index].GetResult();
			MortonCodes.Append(MoveTemp(TempMortonCodes));
		}
					
		MortonCodes.Sort();
		MortonCodes.SetNum(Algo::Unique(MortonCodes));
		UE_LOG(LogTemp, Warning, TEXT("Morton Codes: %d"), MortonCodes.Num());
		
		const FSparseVoxelOctree Octree(MoveTemp(MortonCodes));
		Octree.DebugDraw(GetWorld());
	}
};