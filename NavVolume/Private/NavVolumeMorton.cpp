#include "NavVolumeMorton.h"

// Anonymous - content shouldn't be needed outside this file.
namespace
{
	/** Casting negative values will overflow - desirable to a degree, but doesn't make spatial sense (e.g. -1 would be far from 0).
	 *	Instead, normalize the values by adding (1 << 20) - which is half of a 21-bit unsigned integer.
	 *	This will result in an (unsigned) number line that acts like: <-3, -2, -1, 0, 1, 2, 3>
	 *	This makes spatial sense but does constrain each coordinate to be within [-1,048,576, +1,048,575].
	 *	It does also mean that world space 0 is not index 0 in the array; however, I am only really interested in the sorted spatial ordering.
	 */
	constexpr uint32 SignOffset = 1 << 20;
	
	/** Inserts two zero bits between each bit - the 11 most significant bits are discarded. */
	FORCEINLINE uint64 Part(uint64 N)
	{
		N &= 0x1fffff; // Only keep 21-bits
		N = (N ^ (N << 32)) & 0x1f00000000ffff;
		N = (N ^ (N << 16)) & 0x1f0000ff0000ff;
		N = (N ^ (N <<  8)) & 0x100f00f00f00f00f;
		N = (N ^ (N <<  4)) & 0x10c30c30c30c30c3;
		N = (N ^ (N <<  2)) & 0x1249249249249249;
		return N;
	}

	/** Extracts every third bit to recover the original 21-bit value. */
	FORCEINLINE uint64 Compact(uint64 N)
	{
		N &= 0x1249249249249249;
		N = (N ^ (N >>  2)) & 0x10c30c30c30c30c3;
		N = (N ^ (N >>  4)) & 0x100f00f00f00f00f;
		N = (N ^ (N >>  8)) & 0x1f0000ff0000ff;
		N = (N ^ (N >> 16)) & 0x1f00000000ffff;
		N = (N ^ (N >> 32)) & 0x1fffff; // Only keep 21-bits
		return N;
	}

	/** Adds an offset to the integer moving zero to the middle value of a 21-bit integer. */
	FORCEINLINE uint32 Normalize(const int32 N)
	{
		return static_cast<uint32>(N + SignOffset);
	}

	/** Removes an offset from the integer so it be represented as a signed value. */
	FORCEINLINE int32 Denormalize(const uint32 N)
	{
		return static_cast<int32>(N) - SignOffset;
	}
}

NavVolume::Morton::FMortonCode NavVolume::Morton::EncodeMorton(const int32 X, const int32 Y, const int32 Z)
{
	// Create spacing between each coordinate then shift and combine
	return Part(Normalize(X)) << 0 |
		   Part(Normalize(Y)) << 1 |
		   Part(Normalize(Z)) << 2 ;
}

NavVolume::Morton::FMortonCode NavVolume::Morton::EncodeMorton(const FIntVector Point)
{
	return EncodeMorton(Point.X, Point.Y, Point.Z);
}

FIntVector NavVolume::Morton::DecodeMorton(const FMortonCode Code)
{
	// Not implemented through DecodeXYZ because I do not want to inline them.
	// The code is shifted to move the coordinate into the third bit location for extraction.
	return FIntVector(
		Denormalize(Compact(Code >> 0)), // X
		Denormalize(Compact(Code >> 1)), // Y
		Denormalize(Compact(Code >> 2))	 // Z
	);
}

int32 NavVolume::Morton::DecodeMortonX(const FMortonCode Code)
{
	return Denormalize(Compact(Code));
}

int32 NavVolume::Morton::DecodeMortonY(const FMortonCode Code)
{
	return Denormalize(Compact(Code >> 1));
}

int32 NavVolume::Morton::DecodeMortonZ(const FMortonCode Code)
{
	return Denormalize(Compact(Code >> 2));
}

