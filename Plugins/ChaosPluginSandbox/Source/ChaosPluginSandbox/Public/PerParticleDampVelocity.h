// Copyright Epic Games, Inc. All Rights Reserved.
#pragma once

#include "Chaos/Matrix.h"
#include "Chaos/PerParticleRule.h"
#include "GenericPlatform/GenericPlatformMath.h"

namespace ChaosPluginSandbox
{
	class CHAOSPLUGINSANDBOX_API FPerParticleDampVelocity : public Chaos::FPerParticleRule
	{
	public:
		FPerParticleDampVelocity(const Chaos::FReal Coefficient = (Chaos::FReal)0.01)
		    : MCoefficient(Coefficient)
		{
		}
		virtual ~FPerParticleDampVelocity() {}

		void UpdatePositionBasedState(const Chaos::FPBDParticles& Particles, const int32 Offset, const int32 Range);

		// Apply damping without first checking for kinematic particles
		inline void ApplyFast(Chaos::FPBDParticles& Particles, const Chaos::FReal /*Dt*/, const int32 Index) const
		{
			const Chaos::FVec3 R = Particles.X(Index) - MXcm;
			const Chaos::FVec3 Dv = MVcm - Particles.V(Index) + Chaos::FVec3::CrossProduct(R, MOmega);
			Particles.V(Index) += MCoefficient * Dv;
		}

	private:
		Chaos::FReal MCoefficient;
		Chaos::FVec3 MXcm, MVcm, MOmega;
		TArray<int32> ActiveIndices;
	};
}

// Support ISPC enable/disable in non-shipping builds
#if !INTEL_ISPC
const bool bChaosPluginSandbox_DampVelocity_ISPC_Enabled = false;
#elif UE_BUILD_SHIPPING
const bool bChaosPluginSandbox_DampVelocity_ISPC_Enabled = true;
#else
extern CHAOSPLUGINSANDBOX_API bool bChaosPluginSandbox_DampVelocity_ISPC_Enabled;
#endif
