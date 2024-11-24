#ifndef BULLET3_BRIDGE_H
#define BULLET3_BRIDGE_H

#include "glfw/linmath.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct btDynamicsWorldWrapper btDynamicsWorldWrapper;
typedef struct btRigidBodyWrapper btRigidBodyWrapper;

/**
 * Physics world
 * 
 * ----------------------------------------------------------------------------
 */
btDynamicsWorldWrapper* btDynamicsWorld_create();
void btDynamicsWorld_destroy(btDynamicsWorldWrapper *world);

// methods
void btDynamicsWorld_setGravity(btDynamicsWorldWrapper *world, vec3 *gravity);
void btDynamicsWorld_getGravity(btDynamicsWorldWrapper *world, vec3 *gravity);

/**
 * Rigid body
 * 
 * ----------------------------------------------------------------------------
 */
btRigidBodyWrapper* btRigidBody_create_sphere(float radius, float mass);
void btRigidBody_destroy(btRigidBodyWrapper* body);

// methods
void btRigidBody_setPosition(btRigidBodyWrapper* body, vec3 *position);
void btRigidBody_setMass(btRigidBodyWrapper* body, float mass);

void btDynamicsWorld_addRigidBody(btDynamicsWorldWrapper* world, btRigidBodyWrapper* body);
void btDynamicsWorld_stepSimulation(btDynamicsWorldWrapper* world, float timeStep);

void btRigidBody_getPosition(btRigidBodyWrapper* bodyWrapper, vec3* position);
void btRigidBody_getQuaternion(btRigidBodyWrapper* bodyWrapper, quat* quaternion);
void btRigidBody_getOpenGLMatrix(btRigidBodyWrapper* bodyWrapper, mat4x4* matrix);

#ifdef __cplusplus
}
#endif

#endif // BULLET3_BRIDGE_H
