#ifndef BULLET3_BRIDGE_H
#define BULLET3_BRIDGE_H

#include "glfw/linmath.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct btDynamicsWorldWrapper btDynamicsWorldWrapper;
typedef struct btRigidBodyWrapper btRigidBodyWrapper;
typedef struct btCollisionShapeWrapper btCollisionShapeWrapper;
typedef struct btTypedConstraintWrapper btTypedConstraintWrapper;

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
void btDynamicsWorld_addRigidBody(btDynamicsWorldWrapper *world, btRigidBodyWrapper *body);

void btDynamicsWorld_addConstraint(btDynamicsWorldWrapper *world, btTypedConstraintWrapper *constraint, bool noclip);
void btDynamicsWorld_removeConstraint(btDynamicsWorldWrapper *world, btTypedConstraintWrapper *constraint);

void btDynamicsWorld_stepSimulation(btDynamicsWorldWrapper *world, float timeStep);

/**
 * Collision shape
 * 
 * ----------------------------------------------------------------------------
 */
void btCollisionShape_destroy(btCollisionShapeWrapper *shape);

// shpere
btCollisionShapeWrapper *btCollisionShape_create_sphere(float radius);

// box
btCollisionShapeWrapper *btCollisionShape_create_box(vec3 *halfExtents);

// cylinder
btCollisionShapeWrapper *btCollisionShape_create_cylinder(vec3 *halfExtents);
btCollisionShapeWrapper *btCollisionShape_create_cylinderX(vec3 *halfExtents);
btCollisionShapeWrapper *btCollisionShape_create_cylinderZ(vec3 *halfExtents);

// static plane
btCollisionShapeWrapper *btCollisionShape_create_static_plane(vec3 *normal, float constant);

/**
 * Contraints
 * 
 * ----------------------------------------------------------------------------
 */
void btTypedConstraint_destroy(btTypedConstraintWrapper *constraint);

// point to point
btTypedConstraintWrapper *btPoint2PointConstraint_create(btRigidBodyWrapper *bodyA, btRigidBodyWrapper *bodyB, vec3 *pivotA, vec3 *pivotB);

// hinge
btTypedConstraintWrapper *btHingeConstraint_create(btRigidBodyWrapper *bodyA, btRigidBodyWrapper *bodyB, vec3 *pivotA, vec3 *pivotB, vec3 *axisA, vec3 *axisB);

/**
 * Rigid body
 * 
 * ----------------------------------------------------------------------------
 */
btRigidBodyWrapper *btRigidBody_create(btCollisionShapeWrapper* shape, float mass);
void btRigidBody_destroy(btRigidBodyWrapper *body);

// methods
void btRigidBody_setPosition(btRigidBodyWrapper *body, vec3 *position);
void btRigidBody_setQuaternion(btRigidBodyWrapper *body, quat *quaternion);
void btRigidBody_setMass(btRigidBodyWrapper *body, float mass);

void btRigidBody_getPosition(btRigidBodyWrapper *bodyWrapper, vec3 *position);
void btRigidBody_getLinearVelocity(btRigidBodyWrapper *bodyWrapper, vec3 *velocity);
void btRigidBody_getAngularVelocity(btRigidBodyWrapper *bodyWrapper, vec3 *velocity);

void btRigidBody_getQuaternion(btRigidBodyWrapper *bodyWrapper, quat *quaternion);
void btRigidBody_getOpenGLMatrix(btRigidBodyWrapper *bodyWrapper, mat4x4 *matrix);

void btRigidBody_applyForce(btRigidBodyWrapper *body, vec3 *force, vec3 *rel_pos);
void btRigidBody_applyCentralForce(btRigidBodyWrapper *body, vec3 *force);
void btRigidBody_applyTorque(btRigidBodyWrapper *body, vec3 *torque);
void btRigidBody_applyImpulse(btRigidBodyWrapper *body, vec3 *impulse, vec3 *rel_pos);
void btRigidBody_applyCentralImpulse(btRigidBodyWrapper *body, vec3 *impulse);
void btRigidBody_applyTorqueImpulse(btRigidBodyWrapper *body, vec3 *torque);

void btRigidBody_activate(btRigidBodyWrapper *body);


#ifdef __cplusplus
}
#endif

#endif // BULLET3_BRIDGE_H
