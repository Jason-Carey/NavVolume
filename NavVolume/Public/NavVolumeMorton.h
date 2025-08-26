// https://stackoverflow.com/questions/1024754/how-to-compute-a-3d-morton-number-interleave-the-bits-of-3-ints
// https://fgiesen.wordpress.com/2009/12/13/decoding-morton-codes/

#pragma once

/** Given an integer vector in world space, interleave the bits of XYZ into an unsigned 64-bit integer (a morton code).
 *	This is helpful as each vector near each other in world space will be sorted near each other in an array - meaning better cache coherence.
 * 
 *	Given an integer vector <1, 2, 3>:
 *		1 = 01 = x1, x0
 *		2 = 10 = y1, y0
 *		3 = 11 = z1, z0
 *		
 *	Interleave bits:
 *		= z1, y1, x1, z0, y0, x0
 *		=  1,  1,  0,  1,  0,  1
 *		
 *	NOTE: Each coordinate has a supported range of [-1,048,576, 1,048,575].
 */
namespace NavVolume::Morton
{
	typedef uint64 FMortonCode;

	NAVVOLUME_API FORCEINLINE FMortonCode EncodeMorton(const FIntVector Point);
	
	NAVVOLUME_API /** FORCEINLINE */ FMortonCode EncodeMorton(const int32 X, const int32 Y, const int32 Z);

	NAVVOLUME_API /** FORCEINLINE */ FIntVector DecodeMorton(const FMortonCode Code);

	NAVVOLUME_API /** FORCEINLINE */ int32 DecodeMortonX(const FMortonCode Code);

	NAVVOLUME_API /** FORCEINLINE */ int32 DecodeMortonY(const FMortonCode Code);

	NAVVOLUME_API /** FORCEINLINE */ int32 DecodeMortonZ(const FMortonCode Code);
};
