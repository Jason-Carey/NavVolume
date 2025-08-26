#pragma once

#include "CoreMinimal.h"
#include "NavVolumeMorton.h"
#include "NavVolumeVoxel.h"
#include <bit>

namespace NavVolume::Octree
{
	using namespace NavVolume::Morton;
	using namespace NavVolume::Voxel;

	template<int32 VoxelSize>
	struct NAVVOLUME_API TSparseVoxelOctree 
	{
		struct FNode
		{
			int32 FirstChildIndex = INDEX_NONE;
			uint8 ChildBitMask = 0;
		};

		struct FNodeLocation
		{
			int32 Level;
			int32 NodeIndex;
		};

		typedef TArray<FMortonCode> FLeaves;
		typedef TArray<FNode> FLevel;
		typedef TArray<FLevel> FLevels;
		
		TSparseVoxelOctree(TArray<FMortonCode>&& InCodes)
		{
			TArray<FMortonCode> ItrCodes = MoveTemp(InCodes);
			TArray<FMortonCode> CurCodes;

			constexpr int32 MaxNumLevels = 21; // 21-bits per axis
			
			for (int32 LevelIndex = 0; LevelIndex < MaxNumLevels; ++LevelIndex)
			{
				FLevel& CurLevel = Levels.InsertDefaulted_GetRef(LevelIndex); 

				for (int32 ItrIndex = 0; ItrIndex < ItrCodes.Num(); ++ItrIndex)
				{
					const FMortonCode ItrCode = ItrCodes[ItrIndex];
					const FMortonCode CurCode = ItrCode >> 3; 

					// Shifting 3-bits of a code can produce up to 8 of the same code (e.g. 1 node with 1-8 octants)
					// For every unique code encountered create a new node and save the code for next level construction
					if (CurCodes.IsEmpty() || CurCodes.Last() != CurCode)
					{
						FNode CurNode { ItrIndex };
						CurCodes.Add(CurCode);
						CurLevel.Add(CurNode);
					}

					// The last 3-bits of a code represent the relative octant
					CurLevel.Last().ChildBitMask |= 1 << (ItrCode & 7);
				}
				
				if (LevelIndex == 0)
				{
					Leaves = MoveTemp(ItrCodes);
				}
				
				ItrCodes = MoveTemp(CurCodes);
				
				if (ItrCodes.Num() <= 1)
				{
					return;
				}
			}
		}

		FORCEINLINE FMortonCode GetMortonCode(FNode Node, const int32 Level) const
		{
			check(Level >= 0 && Level < NumLevels());
			
			int32 ItrLevel = Level;
			
			// Follow the parent-child chain until at the last node before a leaf
			while (ItrLevel > 0)
				Node = Levels[--ItrLevel][Node.FirstChildIndex];

			// 64-bits set to 1 - can't just right shift as that skews the XYZ coordinates
			constexpr uint64 LevelMask = 0xFFFFFFFFFFFFFFFF;
			
			// Access the morton code at the leaf - set 3-bits to 0 for each level
			return Leaves[Node.FirstChildIndex] & LevelMask << (3 * (Level + 1));
		}

		FORCEINLINE FIntVector GetPosition(const FNode Node, const int32 Level) const
		{
			const FMortonCode MortonCode = GetMortonCode(Node, Level);
			const int32 CenterOffset = TVoxelTraits<VoxelSize>::HalfVoxelSize * ((2 << Level) - 1); // (2^(n+1)) - 1 where n>0
			return DequantizeVoxel<VoxelSize>(DecodeMorton(MortonCode)) + FIntVector(CenterOffset);
		}

		FORCEINLINE int32 GetHalfSize(const int32 Level) const
		{
			check(Level >= 0 && Level < NumLevels());
			return TVoxelTraits<VoxelSize>::HalfVoxelSize * (2 << Level); // (2^(n+1)) where n>0
		}

		FORCEINLINE int32 NumLevels() const
		{
			return Levels.Num();
		}
		
		FORCEINLINE bool HasChild(const FNode Node, const uint8 RelativeOctant) const
		{
			// Interval [1, 8]
			check(RelativeOctant < 9 && RelativeOctant > 0);
			return Node.ChildBitMask & (1 << RelativeOctant);
		}
		
		FORCEINLINE int32 ChildIndex(const FNode Node, const uint8 RelativeOctant) const
		{
			check(HasChild(Node, RelativeOctant));
			const uint8 LowerBits = Node.ChildBitMask & (0xFF >> (9 - RelativeOctant));
			return Node.FirstChildIndex + std::popcount<uint8>(LowerBits);
		}

		void DebugDrawLevel(const UWorld* World, const int32 Level, const FColor Color) const
		{
#if WITH_EDITOR
			const FLevel& CurLevel = Levels[Level];

			for (const FNode& Node : CurLevel)
			{
				const FVector Center = FVector(GetPosition(Node, Level));
				const FVector Extent = FVector(GetHalfSize(Level)); 
				DrawDebugBox(World, Center, Extent, FQuat::Identity, Color, true);
			}
#endif // WITH_EDITOR
		}

		void DebugDraw(const UWorld* World, const FColor Color1 = FColor::Red, const FColor Color2 = FColor::Green) const
		{
#if WITH_EDITOR
			for (const FMortonCode& Code : Leaves)
			{
				const FVector Center = FVector(DequantizeVoxel<VoxelSize>(DecodeMorton(Code)));
				const FVector Extent = FVector(TVoxelTraits<VoxelSize>::HalfVoxelSize);
				DrawDebugBox(World, Center, Extent, FQuat::Identity, Color1, true);
			}

			for (int32 Level = 0; Level < NumLevels(); ++Level)
			{
				DebugDrawLevel(World, Level, Level % 2 == 0 ? Color2 : Color1);
			}
#endif // WITH_EDITOR
		}
		
		FLevels Levels;
		FLeaves Leaves;
	};
} // namespace NavVolume::Octree
