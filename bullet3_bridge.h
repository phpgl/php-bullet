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

// -- Point to point
btTypedConstraintWrapper *btPoint2PointConstraint_create(btRigidBodyWrapper *bodyA, btRigidBodyWrapper *bodyB, vec3 *pivotA, vec3 *pivotB);

// -- Hinge
btTypedConstraintWrapper *btHingeConstraint_create(btRigidBodyWrapper *bodyA, btRigidBodyWrapper *bodyB, vec3 *pivotA, vec3 *pivotB, vec3 *axisA, vec3 *axisB);

void btHingeConstraint_setLimit(btTypedConstraintWrapper *constraint, float low, float high, float softness, float biasFactor, float relaxationFactor);

// -- Slider
btTypedConstraintWrapper *btSliderConstraint_create(btRigidBodyWrapper *bodyA, btRigidBodyWrapper *bodyB, mat4x4 *frameInA, mat4x4 *frameInB, bool useLinearReferenceFrameA);

// -- Generic6DofSpring
btTypedConstraintWrapper *btGeneric6DofSpringConstraint_create(btRigidBodyWrapper *bodyA, btRigidBodyWrapper *bodyB, mat4x4 *frameInA, mat4x4 *frameInB, bool useLinearReferenceFrameA);

// -- Generic6DofSpring2
btTypedConstraintWrapper *btGeneric6DofSpring2Constraint_create(btRigidBodyWrapper *bodyA, btRigidBodyWrapper *bodyB, mat4x4 *frameInA, mat4x4 *frameInB);

void btGeneric6DofSpring2Constraint_setFrames(btTypedConstraintWrapper *constraint, mat4x4 *frameInA, mat4x4 *frameInB);

void btGeneric6DofSpring2Constraint_setLinearLowerLimit(btTypedConstraintWrapper *constraint, const vec3 *linearLower);

void btGeneric6DofSpring2Constraint_setLinearUpperLimit(btTypedConstraintWrapper *constraint, const vec3 *linearUpper);

void btGeneric6DofSpring2Constraint_setAngularLowerLimit(btTypedConstraintWrapper *constraint, const vec3 *angularLower);
void btGeneric6DofSpring2Constraint_setAngularLowerLimitReversed(btTypedConstraintWrapper *constraint, const vec3 *angularLower);

void btGeneric6DofSpring2Constraint_setAngularUpperLimit(btTypedConstraintWrapper *constraint, const vec3 *angularUpper);
void btGeneric6DofSpring2Constraint_setAngularUpperLimitReversed(btTypedConstraintWrapper *constraint, const vec3 *angularUpper);

void btGeneric6DofSpring2Constraint_setLimit(btTypedConstraintWrapper *constraint, int axis, float lo, float hi);
void btGeneric6DofSpring2Constraint_setLimitReversed(btTypedConstraintWrapper *constraint, int axis, float lo, float hi);

void btGeneric6DofSpring2Constraint_setAxis(btTypedConstraintWrapper *constraint, vec3 *axis1, vec3 *axis2);

void btGeneric6DofSpring2Constraint_setBounce(btTypedConstraintWrapper *constraint, int index, float bounce);

void btGeneric6DofSpring2Constraint_enableMotor(btTypedConstraintWrapper *constraint, int index, bool onOff);
void btGeneric6DofSpring2Constraint_setServo(btTypedConstraintWrapper *constraint, int index, bool onOff);
void btGeneric6DofSpring2Constraint_setTargetVelocity(btTypedConstraintWrapper *constraint, int index, float velocity);
void btGeneric6DofSpring2Constraint_setServoTarget(btTypedConstraintWrapper *constraint, int index, float target);
void btGeneric6DofSpring2Constraint_setMaxMotorForce(btTypedConstraintWrapper *constraint, int index, float force);

void btGeneric6DofSpring2Constraint_enableSpring(btTypedConstraintWrapper *constraint, int index, bool onOff);
void btGeneric6DofSpring2Constraint_setStiffness(btTypedConstraintWrapper *constraint, int index, float stiffness, bool limitIfNeeded);
void btGeneric6DofSpring2Constraint_setDamping(btTypedConstraintWrapper *constraint, int index, float damping, bool limitIfNeeded);

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
void btRigidBody_setRestitution(btRigidBodyWrapper *body, float restitution);
void btRigidBody_setFriction(btRigidBodyWrapper *body, float friction);
void btRigidBody_setRollingFriction(btRigidBodyWrapper *body, float friction);
void btRigidBody_setSpinningFriction(btRigidBodyWrapper *body, float friction);
void btRigidBody_setContactStiffnessAndDamping(btRigidBodyWrapper *body, float stiffness, float damping);

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
