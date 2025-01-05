#ifndef BULLET3_BRIDGE_H
#define BULLET3_BRIDGE_H

#include "glfw/linmath.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct btDynamicsWorldWrapper btDynamicsWorldWrapper;
typedef struct btRigidBodyWrapper btRigidBodyWrapper;
typedef struct btCollisionShapeWrapper btCollisionShapeWrapper;

/**
 * Physics world
 * 
 * ----------------------------------------------------------------------------
 */
btDynamicsWorldWrapper *btDynamicsWorld_create();
void btDynamicsWorld_destroy(btDynamicsWorldWrapper *world);

// methods
void btDynamicsWorld_setGravity(btDynamicsWorldWrapper *world, vec3 *gravity);
void btDynamicsWorld_getGravity(btDynamicsWorldWrapper *world, vec3 *gravity);

/**
 * Collision shape
 * 
 * ----------------------------------------------------------------------------
 */
void btCollisionShape_destroy(btCollisionShapeWrapper *shape);

// shpere
btCollisionShapeWrapper *btCollisionShape_create_sphere(float radius);

// static plane
btCollisionShapeWrapper *btCollisionShape_create_static_plane(vec3 *normal, float constant);

/**
 * Rigid body
 * 
 * ----------------------------------------------------------------------------
 */
btRigidBodyWrapper *btRigidBody_create(btCollisionShapeWrapper* shape, float mass);
void btRigidBody_destroy(btRigidBodyWrapper *body);

// methods
void btRigidBody_setPosition(btRigidBodyWrapper *body, vec3 *position);
void btRigidBody_setMass(btRigidBodyWrapper *body, float mass);

void btDynamicsWorld_addRigidBody(btDynamicsWorldWrapper *world, btRigidBodyWrapper *body);
void btDynamicsWorld_stepSimulation(btDynamicsWorldWrapper *world, float timeStep);

void btRigidBody_getPosition(btRigidBodyWrapper *bodyWrapper, vec3 *position);
void btRigidBody_getLinearVelocity(btRigidBodyWrapper *bodyWrapper, vec3 *velocity);
void btRigidBody_getAngularVelocity(btRigidBodyWrapper *bodyWrapper, vec3 *velocity);

void btRigidBody_getQuaternion(btRigidBodyWrapper *bodyWrapper, quat *quaternion);
void btRigidBody_getOpenGLMatrix(btRigidBodyWrapper *bodyWrapper, mat4x4 *matrix);


#ifdef __cplusplus
}
#endif

#endif // BULLET3_BRIDGE_H
