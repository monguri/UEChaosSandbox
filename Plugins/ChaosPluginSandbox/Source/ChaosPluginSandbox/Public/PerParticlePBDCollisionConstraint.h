// Copyright Epic Games, Inc. All Rights Reserved.
#pragma once

#include "Chaos/Real.h"
#include "Chaos/PerParticleRule.h"
#include "Chaos/Transform.h"
#include "Chaos/PBDActiveView.h"
#include "Chaos/GeometryParticlesfwd.h"

#include "HAL/PlatformMath.h"

// Support ISPC enable/disable in non-shipping builds
#if !INTEL_ISPC
const bool bChaosPluginSandbox_PerParticleCollision_ISPC_Enabled = false;
#elif UE_BUILD_SHIPPING
const bool bChaosPluginSandbox_PerParticleCollision_ISPC_Enabled = true;
#else
extern CHAOSPLUGINSANDBOX_API bool bChaosPluginSandbox_PerParticleCollision_ISPC_Enabled;
#endif

namespace ChaosPluginSandbox
{
template<Chaos::EGeometryParticlesSimType SimType>
class CHAOSPLUGINSANDBOX_API TPerParticlePBDCollisionConstraint final
{
	struct FVelocityConstraint
	{
		Chaos::FVec3 Velocity;
		Chaos::FVec3 Normal;
	};

public:
	typedef Chaos::TKinematicGeometryParticlesImp<Chaos::FReal, 3, SimType> FCollisionParticles;

	TPerParticlePBDCollisionConstraint(const Chaos::TPBDActiveView<FCollisionParticles>& InParticlesActiveView, TArray<bool>& Collided, TArray<uint32>& DynamicGroupIds, TArray<uint32>& KinematicGroupIds, const TArray<Chaos::FReal>& PerGroupThickness, const TArray<Chaos::FReal>& PerGroupFriction)
	    : bFastPositionBasedFriction(true), MCollisionParticlesActiveView(InParticlesActiveView), MCollided(Collided), MDynamicGroupIds(DynamicGroupIds), MKinematicGroupIds(KinematicGroupIds), MPerGroupThickness(PerGroupThickness), MPerGroupFriction(PerGroupFriction) {}

	~TPerParticlePBDCollisionConstraint() {}

	inline void ApplyRange(Chaos::FPBDParticles& Particles, const Chaos::FReal Dt, const int32 Offset, const int32 Range) const
	{
		if (bRealTypeCompatibleWithISPC && bChaosPluginSandbox_PerParticleCollision_ISPC_Enabled && bFastPositionBasedFriction )
		{
			ApplyHelperISPC(Particles, Dt, Offset, Range);
		}
		else
		{
			ApplyHelper(Particles, Dt, Offset, Range);
		}
	}

	void ApplyFriction(Chaos::FPBDParticles& Particles, const Chaos::FReal Dt, const int32 Index) const
	{
		check(!bFastPositionBasedFriction);  // Do not call this function if this is setup to run with fast PB friction

		if (!MVelocityConstraints.Contains(Index))
		{
			return;
		}
		const Chaos::FReal VN = FVec3::DotProduct(Particles.V(Index), MVelocityConstraints[Index].Normal);
		const Chaos::FReal VNBody = FVec3::DotProduct(MVelocityConstraints[Index].Velocity, MVelocityConstraints[Index].Normal);
		const Chaos::FVec3 VTBody = MVelocityConstraints[Index].Velocity - VNBody * MVelocityConstraints[Index].Normal;
		const Chaos::FVec3 VTRelative = Particles.V(Index) - VN * MVelocityConstraints[Index].Normal - VTBody;
		const Chaos::FReal VTRelativeSize = VTRelative.Size();
		const Chaos::FReal VNMax = FMath::Max(VN, VNBody);
		const Chaos::FReal VNDelta = VNMax - VN;
		const Chaos::FReal CoefficientOfFriction = MPerGroupFriction[MDynamicGroupIds[Index]];
		check(CoefficientOfFriction > 0);
		const FReal Friction = CoefficientOfFriction * VNDelta < VTRelativeSize ? CoefficientOfFriction * VNDelta / VTRelativeSize : 1;
		Particles.V(Index) = VNMax * MVelocityConstraints[Index].Normal + VTBody + VTRelative * (1 - Friction);
	}

private:
	inline void ApplyHelper(Chaos::FPBDParticles& Particles, const Chaos::FReal Dt, const int32 Offset, const int32 Range) const
	{
		const uint32 DynamicGroupId = MDynamicGroupIds[Offset];  // Particle group Id, must be the same across the entire range
		const FReal PerGroupFriction = MPerGroupFriction[DynamicGroupId];
		const FReal PerGroupThickness = MPerGroupThickness[DynamicGroupId];

		if (PerGroupFriction > (FReal)KINDA_SMALL_NUMBER)
		{
			PhysicsParallelFor(Range - Offset, [this, &Particles, Dt, Offset, DynamicGroupId, PerGroupFriction, PerGroupThickness](int32 i)
			{
				const int32 Index = Offset + i;

				if (Particles.InvM(Index) == (FReal)0.)
				{
					return;  // Continue
				}

				MCollisionParticlesActiveView.SequentialFor([this, &Particles, &Dt, &Index, DynamicGroupId, PerGroupFriction, PerGroupThickness](FCollisionParticles& CollisionParticles, int32 i)
				{
					const uint32 KinematicGroupId = MKinematicGroupIds[i];  // Collision group Id

					if (KinematicGroupId != (uint32)INDEX_NONE && DynamicGroupId != KinematicGroupId)
					{
						return; // Bail out if the collision groups doesn't match the particle group id, or use INDEX_NONE (= global collision that affects all particle)
					}
					FVec3 Normal;
					const FRigidTransform3 Frame(CollisionParticles.X(i), CollisionParticles.R(i));
					const FReal Phi = CollisionParticles.Geometry(i)->PhiWithNormal(Frame.InverseTransformPosition(Particles.P(Index)), Normal);

					const FReal Penetration = PerGroupThickness - Phi; // This is related to the Normal impulse
					if (Penetration > (FReal)0.)
					{
						const FVec3 NormalWorld = Frame.TransformVector(Normal);
						Particles.P(Index) += Penetration * NormalWorld;

						if (bFastPositionBasedFriction)
						{
							FVec3 VectorToPoint = Particles.P(Index) - CollisionParticles.X(i);
							const FVec3 RelativeDisplacement = (Particles.P(Index) - Particles.X(Index)) - (CollisionParticles.V(i) + FVec3::CrossProduct(CollisionParticles.W(i), VectorToPoint)) * Dt; // This corresponds to the tangential velocity multiplied by dt (friction will drive this to zero if it is high enough)
							const FVec3 RelativeDisplacementTangent = RelativeDisplacement - FVec3::DotProduct(RelativeDisplacement, NormalWorld) * NormalWorld; // Project displacement into the tangential plane
							const FReal RelativeDisplacementTangentLength = RelativeDisplacementTangent.Size();
							if (RelativeDisplacementTangentLength >= SMALL_NUMBER)
							{
								const FReal PositionCorrection = FMath::Min<FReal>(Penetration * PerGroupFriction, RelativeDisplacementTangentLength);
								const FReal CorrectionRatio = PositionCorrection / RelativeDisplacementTangentLength;
								Particles.P(Index) -= CorrectionRatio * RelativeDisplacementTangent;
							}
						}
						else
						{
							// Note, to fix: Only use fast position based friction for now, since adding to TMaps here is not thread safe when calling Apply on multiple threads (will cause crash)
							FVelocityConstraint Constraint;
							FVec3 VectorToPoint = Particles.P(Index) - CollisionParticles.X(i);
							Constraint.Velocity = CollisionParticles.V(i) + FVec3::CrossProduct(CollisionParticles.W(i), VectorToPoint);
							Constraint.Normal = Frame.TransformVector(Normal);
						
							MVelocityConstraints.Add(Index, Constraint);
						}
					}
				});
			});
		}
		else
		{
			PhysicsParallelFor(Range - Offset, [this, &Particles, Dt, Offset, DynamicGroupId, PerGroupFriction, PerGroupThickness](int32 i)
			{
				const int32 Index = Offset + i;

				if (Particles.InvM(Index) == 0)
				{
					return;  // Continue
				}

				MCollisionParticlesActiveView.SequentialFor([this, &Particles, &Dt, &Index, DynamicGroupId, PerGroupFriction, PerGroupThickness](FCollisionParticles& CollisionParticles, int32 i)
				{
					const uint32 KinematicGroupId = MKinematicGroupIds[i];  // Collision group Id

					if (KinematicGroupId != (uint32)INDEX_NONE && DynamicGroupId != KinematicGroupId)
					{
						return; // Bail out if the collision groups doesn't match the particle group id, or use INDEX_NONE (= global collision that affects all particle)
					}
					FVec3 Normal;
					const FRigidTransform3 Frame(CollisionParticles.X(i), CollisionParticles.R(i));
					const FReal Phi = CollisionParticles.Geometry(i)->PhiWithNormal(Frame.InverseTransformPosition(Particles.P(Index)), Normal);

					const FReal Penetration = PerGroupThickness - Phi; // This is related to the Normal impulse
					if (Penetration > (FReal)0.)
					{
						const FVec3 NormalWorld = Frame.TransformVector(Normal);
						Particles.P(Index) += Penetration * NormalWorld;
					}
				});
			});
		}
	}

	void ApplyHelperISPC(Chaos::FPBDParticles& Particles, const Chaos::FReal Dt, int32 Offset, int32 Range) const;

private:
	bool bFastPositionBasedFriction;
	// TODO(mlentine): Need a bb hierarchy
	const Chaos::TPBDActiveView<FCollisionParticles>& MCollisionParticlesActiveView;
	TArray<bool>& MCollided;
	const TArray<uint32>& MDynamicGroupIds;
	const TArray<uint32>& MKinematicGroupIds;
	mutable TMap<int32, FVelocityConstraint> MVelocityConstraints;
	const TArray<Chaos::FReal>& MPerGroupThickness;
	const TArray<Chaos::FReal>& MPerGroupFriction;
};
}
