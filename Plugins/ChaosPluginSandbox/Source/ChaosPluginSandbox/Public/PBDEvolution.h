// Copyright Epic Games, Inc. All Rights Reserved.
#pragma once

#include "Chaos/KinematicGeometryParticles.h"
#include "Chaos/PerParticleGravity.h"
#include "Chaos/VelocityField.h"
#include "Chaos/PBDParticles.h"
#include "Chaos/PBDActiveView.h"
#include "Chaos/Vector.h"

namespace ChaosPluginSandbox
{

class CHAOSPLUGINSANDBOX_API FPBDEvolution : public Chaos::TArrayCollection
{
 public:
	using FGravityForces = Chaos::FPerParticleGravity;

	// TODO(mlentine): Init particles from some type of input
	FPBDEvolution(Chaos::FPBDParticles&& InParticles, Chaos::FKinematicGeometryClothParticles&& InGeometryParticles, TArray<Chaos::TVec3<int32>>&& CollisionTriangles, int32 NumIterations = 1, Chaos::FReal CollisionThickness = 0, Chaos::FReal SelfCollisionsThickness = 0, Chaos::FReal CoefficientOfFriction = 0, Chaos::FReal Damping = 0.04);
	~FPBDEvolution() {}

	// Advance one time step. Filter the input time step if specified.
	void AdvanceOneTimeStep(const Chaos::FReal Dt, const bool bSmoothDt = true);

	// Remove all particles, will also reset all rules
	void ResetParticles();

	// Add particles and initialize group ids. Return the index of the first added particle.
	int32 AddParticleRange(int32 NumParticles, uint32 GroupId, bool bActivate);

	// Return the number of particles of the block starting at Offset
	int32 GetParticleRangeSize(int32 Offset) const { return MParticlesActiveView.GetRangeSize(Offset); }

	// Set a block of particles active or inactive, using the index of the first added particle to identify the block.
	void ActivateParticleRange(int32 Offset, bool bActivate)  { MParticlesActiveView.ActivateRange(Offset, bActivate); }

	// Particles accessors
	const Chaos::FPBDParticles& Particles() const { return MParticles; }
	Chaos::FPBDParticles& Particles() { return MParticles; }
	const Chaos::TPBDActiveView<Chaos::FPBDParticles>& ParticlesActiveView() { return MParticlesActiveView; }

	const TArray<uint32>& ParticleGroupIds() const { return MParticleGroupIds; }

	// Remove all collision particles
	void ResetCollisionParticles(int32 NumParticles = 0);

	// Add collision particles and initialize group ids. Return the index of the first added particle.
	// Use INDEX_NONE as GroupId for collision particles that affect all particle groups.
	int32 AddCollisionParticleRange(int32 NumParticles, uint32 GroupId, bool bActivate);

	// Set a block of collision particles active or inactive, using the index of the first added particle to identify the block.
	void ActivateCollisionParticleRange(int32 Offset, bool bActivate) { MCollisionParticlesActiveView.ActivateRange(Offset, bActivate); }

	// Return the number of particles of the block starting at Offset
	int32 GetCollisionParticleRangeSize(int32 Offset) const { return MCollisionParticlesActiveView.GetRangeSize(Offset); }

	// Collision particles accessors
	const Chaos::FKinematicGeometryClothParticles& CollisionParticles() const { return MCollisionParticles; }
	Chaos::FKinematicGeometryClothParticles& CollisionParticles() { return MCollisionParticles; }
	const TArray<uint32>& CollisionParticleGroupIds() const { return MCollisionParticleGroupIds; }
	const Chaos::TPBDActiveView<Chaos::FKinematicGeometryClothParticles>& CollisionParticlesActiveView() { return MCollisionParticlesActiveView; }

	// Reset all constraint init and rule functions.
	void ResetConstraintRules() { MConstraintInits.Reset(); MConstraintRules.Reset(); MConstraintInitsActiveView.Reset(); MConstraintRulesActiveView.Reset();  };

	// Add constraints. Return the index of the first added constraint.
	int32 AddConstraintInitRange(int32 NumConstraints, bool bActivate);
	int32 AddConstraintRuleRange(int32 NumConstraints, bool bActivate);
	
	// Return the number of particles of the block starting at Offset
	int32 GetConstraintInitRangeSize(int32 Offset) const { return MConstraintInitsActiveView.GetRangeSize(Offset); }
	int32 GetConstraintRuleRangeSize(int32 Offset) const { return MConstraintRulesActiveView.GetRangeSize(Offset); }

	// Set a block of constraints active or inactive, using the index of the first added particle to identify the block.
	void ActivateConstraintInitRange(int32 Offset, bool bActivate) { MConstraintInitsActiveView.ActivateRange(Offset, bActivate); }
	void ActivateConstraintRuleRange(int32 Offset, bool bActivate) { MConstraintRulesActiveView.ActivateRange(Offset, bActivate); }

	// Constraint accessors
	const TArray<TFunction<void(const Chaos::FPBDParticles&, const Chaos::FReal)>>& ConstraintInits() const { return MConstraintInits; }
	TArray<TFunction<void(const Chaos::FPBDParticles&, const Chaos::FReal)>>& ConstraintInits() { return MConstraintInits; }
	const TArray<TFunction<void(Chaos::FPBDParticles&, const Chaos::FReal)>>& ConstraintRules() const { return MConstraintRules; }
	TArray<TFunction<void(Chaos::FPBDParticles&, const Chaos::FReal)>>& ConstraintRules() { return MConstraintRules; }
	
	void SetKinematicUpdateFunction(TFunction<void(Chaos::FPBDParticles&, const Chaos::FReal, const Chaos::FReal, const int32)> KinematicUpdate) { MKinematicUpdate = KinematicUpdate; }
	void SetCollisionKinematicUpdateFunction(TFunction<void(Chaos::FKinematicGeometryClothParticles&, const Chaos::FReal, const Chaos::FReal, const int32)> KinematicUpdate) { MCollisionKinematicUpdate = KinematicUpdate; }

	TFunction<void(Chaos::FPBDParticles&, const Chaos::FReal, const int32)>& GetForceFunction(const uint32 GroupId = 0) { return MGroupForceRules[GroupId]; }
	const TFunction<void(Chaos::FPBDParticles&, const Chaos::FReal, const int32)>& GetForceFunction(const uint32 GroupId = 0) const { return MGroupForceRules[GroupId]; }

	FGravityForces& GetGravityForces(const uint32 GroupId = 0) { check(GroupId < TArrayCollection::Size()); return MGroupGravityForces[GroupId]; }
	const FGravityForces& GetGravityForces(const uint32 GroupId = 0) const { check(GroupId < TArrayCollection::Size()); return MGroupGravityForces[GroupId]; }

	Chaos::FVelocityField& GetVelocityField(const uint32 GroupId = 0) { check(GroupId < Chaos::TArrayCollection::Size()); return MGroupVelocityFields[GroupId]; }
	const Chaos::FVelocityField& GetVelocityField(const uint32 GroupId = 0) const { check(GroupId < Chaos::TArrayCollection::Size()); return MGroupVelocityFields[GroupId]; }

	void ResetSelfCollision() { MCollisionTriangles.Reset(); MDisabledCollisionElements.Reset(); };
	TArray<Chaos::TVector<int32, 3>>& CollisionTriangles() { return MCollisionTriangles; }
	TSet<Chaos::TVector<int32, 2>>& DisabledCollisionElements() { return MDisabledCollisionElements; }

	int32 GetIterations() const { return MNumIterations; }
	void SetIterations(const int32 Iterations) { MNumIterations = Iterations; }

	Chaos::FReal GetSelfCollisionThickness(const uint32 GroupId = 0) const { check(GroupId < Chaos::TArrayCollection::Size()); return MGroupSelfCollisionThicknesses[GroupId]; }
	void SetSelfCollisionThickness(const Chaos::FReal SelfCollisionThickness, const uint32 GroupId = 0) { check(GroupId < TArrayCollection::Size()); MGroupSelfCollisionThicknesses[GroupId] = SelfCollisionThickness; }

	Chaos::FReal GetCollisionThickness(const uint32 GroupId = 0) const { check(GroupId < Chaos::TArrayCollection::Size()); return MGroupCollisionThicknesses[GroupId]; }
	void SetCollisionThickness(const Chaos::FReal CollisionThickness, const uint32 GroupId = 0) { check(GroupId < Chaos::TArrayCollection::Size()); MGroupCollisionThicknesses[GroupId] = CollisionThickness; }

	Chaos::FReal GetCoefficientOfFriction(const uint32 GroupId = 0) const { check(GroupId < Chaos::TArrayCollection::Size()); return MGroupCoefficientOfFrictions[GroupId]; }
	void SetCoefficientOfFriction(const Chaos::FReal CoefficientOfFriction, const uint32 GroupId = 0) { check(GroupId < TArrayCollection::Size()); MGroupCoefficientOfFrictions[GroupId] = CoefficientOfFriction; }

	Chaos::FReal GetDamping(const uint32 GroupId = 0) const { check(GroupId < Chaos::TArrayCollection::Size()); return MGroupDampings[GroupId]; }
	void SetDamping(const Chaos::FReal Damping, const uint32 GroupId = 0) { check(GroupId < Chaos::TArrayCollection::Size()); MGroupDampings[GroupId] = Damping; }

	bool GetUseCCD(const uint32 GroupId = 0) const { check(GroupId < Chaos::TArrayCollection::Size()); return MGroupUseCCDs[GroupId]; }
	void SetUseCCD(const bool bUseCCD, const uint32 GroupId = 0) { check(GroupId < Chaos::TArrayCollection::Size()); MGroupUseCCDs[GroupId] = bUseCCD; }

	UE_DEPRECATED(4.27, "Use GetCollisionStatus() instead")
	const bool Collided(int32 index) { return MCollided[index]; }

	const TArray<bool>& GetCollisionStatus() { return MCollided; }
	const TArray<Chaos::FVec3>& GetCollisionContacts() const { return MCollisionContacts; }
	const TArray<Chaos::FVec3>& GetCollisionNormals() const { return MCollisionNormals; }

	Chaos::FReal GetTime() const { return MTime; }

 private:
	// Add simulation groups and set default values
	void AddGroups(int32 NumGroups);
	// Reset simulation groups
	void ResetGroups();
	// Selected versions of the pre-iteration updates (euler step, force, velocity field. damping updates)..
	template<bool bForceRule, bool bVelocityField, bool bDampVelocityRule>
	void PreIterationUpdate(const Chaos::FReal Dt, const int32 Offset, const int32 Range, const int32 MinParallelBatchSize);

private:
	Chaos::FPBDParticles MParticles;
	Chaos::TPBDActiveView<Chaos::FPBDParticles> MParticlesActiveView;
	Chaos::FKinematicGeometryClothParticles MCollisionParticles;
	Chaos::TPBDActiveView<Chaos::FKinematicGeometryClothParticles> MCollisionParticlesActiveView;

	TArray<Chaos::TVector<int32, 3>> MCollisionTriangles;       // Used for self-collisions
	TSet<Chaos::TVector<int32, 2>> MDisabledCollisionElements;  // 

	Chaos::TArrayCollectionArray<Chaos::FRigidTransform3> MCollisionTransforms;  // Used for CCD to store the initial state before the kinematic update
	Chaos::TArrayCollectionArray<bool> MCollided;
	Chaos::TArrayCollectionArray<uint32> MCollisionParticleGroupIds;  // Used for per group parameters for collision particles
	Chaos::TArrayCollectionArray<uint32> MParticleGroupIds;  // Used for per group parameters for particles
	TArray<Chaos::FVec3> MCollisionContacts;
	TArray<Chaos::FVec3> MCollisionNormals;

	Chaos::TArrayCollectionArray<FGravityForces> MGroupGravityForces;
	Chaos::TArrayCollectionArray<Chaos::FVelocityField> MGroupVelocityFields;
	Chaos::TArrayCollectionArray<TFunction<void(Chaos::FPBDParticles&, const Chaos::FReal, const int32)>> MGroupForceRules;
	Chaos::TArrayCollectionArray<Chaos::FReal> MGroupCollisionThicknesses;
	Chaos::TArrayCollectionArray<Chaos::FReal> MGroupSelfCollisionThicknesses;
	Chaos::TArrayCollectionArray<Chaos::FReal> MGroupCoefficientOfFrictions;
	Chaos::TArrayCollectionArray<Chaos::FReal> MGroupDampings;
	Chaos::TArrayCollectionArray<bool> MGroupUseCCDs;
	
	TArray<TFunction<void(const Chaos::FPBDParticles&, const Chaos::FReal)>> MConstraintInits;
	Chaos::TPBDActiveView<TArray<TFunction<void(const Chaos::FPBDParticles&, const Chaos::FReal)>>> MConstraintInitsActiveView;
	TArray<TFunction<void(Chaos::FPBDParticles&, const Chaos::FReal)>> MConstraintRules;
	Chaos::TPBDActiveView<TArray<TFunction<void(Chaos::FPBDParticles&, const Chaos::FReal)>>> MConstraintRulesActiveView;

	TFunction<void(Chaos::FPBDParticles&, const Chaos::FReal, const Chaos::FReal, const int32)> MKinematicUpdate;
	TFunction<void(Chaos::FKinematicGeometryClothParticles&, const Chaos::FReal, const Chaos::FReal, const int32)> MCollisionKinematicUpdate;

	int32 MNumIterations;
	Chaos::FVec3 MGravity;
	Chaos::FReal MCollisionThickness;
	Chaos::FReal MSelfCollisionThickness;
	Chaos::FReal MCoefficientOfFriction;
	Chaos::FReal MDamping;
	Chaos::FReal MTime;
	Chaos::FReal MSmoothDt;
};
}

// Support ISPC enable/disable in non-shipping builds
#if !INTEL_ISPC
const bool bChaosPluginSandbox_PostIterationUpdates_ISPC_Enabled = false;
#elif UE_BUILD_SHIPPING
const bool bChaosPluginSandbox_PostIterationUpdates_ISPC_Enabled = true;
#else
extern CHAOSPLUGINSANDBOX_API bool bChaosPluginSandbox_PostIterationUpdates_ISPC_Enabled;
#endif
