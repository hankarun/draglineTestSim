#pragma once

#include <PxPhysicsAPI.h>
#include <raylib.h>

class Dragline;

// Simple-shape renderer for every dragline part. Call inside BeginMode3D().
namespace render
{
inline Vector3 toRay(const physx::PxVec3& v) { return {v.x, v.y, v.z}; }

void ground();
void lowerWorks();                              // crawlers + carbody tub (static)
void house(const physx::PxTransform& pose);     // house, cab, counterweight, gantry
void boom(const physx::PxTransform& housePose); // lattice, tip sheave, pendants

// Rope: straight when taut, sagging toward `sagHint` when slack.
void rope(const physx::PxVec3& from, const physx::PxVec3& to, float length, const physx::PxVec3& sagHint,
          Color color);
// One simulated chain link (capsule along its local X axis).
void chainLink(const physx::PxTransform& pose, float length, int index, Color color);

void yoke(const physx::PxTransform& pose);
void spreader(const physx::PxTransform& pose);
void bucket(const physx::PxTransform& pose);
void dragSocket(const physx::PxTransform& pose);

void dragline(const Dragline& d); // all of the above
} // namespace render
