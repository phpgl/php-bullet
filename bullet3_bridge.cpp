#include "bullet3_bridge.h"
#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"

#include <iostream>
#include <vector>
#include <string>

struct btDynamicsWorldWrapper {
    btDefaultCollisionConfiguration* collisionConfiguration;
    btCollisionDispatcher *dispatcher;
    btBroadphaseInterface* overlappingPairCache;
    btSequentialImpulseConstraintSolver *solver;
    btDiscreteDynamicsWorld* dynamicsWorld;
};

struct btRigidBodyWrapper {
    btCollisionShape* shape;
    btDefaultMotionState* motionState;
    btRigidBody* rigidBody;
    btDynamicsWorld* assignedWorld = nullptr;
};

struct btCollisionShapeWrapper {
    btCollisionShape* shape;
};

struct btTypedConstraintWrapper {
    btTypedConstraint* constraint;
};

/**
 * Common Helpers
 * 
 * ----------------------------------------------------------------------------
 */
btVector3 vec3_to_btVector3(const vec3 *v)
{
    return btVector3((*v)[0], (*v)[1], (*v)[2]);
}

btQuaternion quat_to_btQuaternion(quat *q)
{
    return btQuaternion((*q)[1], (*q)[2], (*q)[3], (*q)[0]);
}

btTransform mat4x4_to_btTransform(mat4x4 *m)
{
    btTransform transform;
    transform.setFromOpenGLMatrix((*m)[0]);
    return transform;
}

void btVector3_to_vec3(btVector3 *v, vec3 *out)
{
    (*out)[0] = v->getX();
    (*out)[1] = v->getY();
    (*out)[2] = v->getZ();
}

void btQuaternion_to_quat(btQuaternion *q, quat *out)
{
    (*out)[0] = q->getW();
    (*out)[1] = q->getX();
    (*out)[2] = q->getY();
    (*out)[3] = q->getZ();
}

/**
 * Physics world
 * 
 * ----------------------------------------------------------------------------
 */

btDynamicsWorldWrapper *btDynamicsWorld_create() {
    btDynamicsWorldWrapper *wrapper = new btDynamicsWorldWrapper;
    wrapper->collisionConfiguration = new btDefaultCollisionConfiguration();
    wrapper->dispatcher = new btCollisionDispatcher(wrapper->collisionConfiguration);
    wrapper->overlappingPairCache = new btDbvtBroadphase();
    wrapper->solver = new btSequentialImpulseConstraintSolver();
    wrapper->dynamicsWorld = new btDiscreteDynamicsWorld(wrapper->dispatcher, wrapper->overlappingPairCache, wrapper->solver, wrapper->collisionConfiguration);

    // // add a ground plane for testing
    // btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);
    // btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
    // btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
    // btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);

    // // add 20 random static boxes for testing
    // for (int i = 0; i < 20; i++) {
    //     btCollisionShape* boxShape = new btBoxShape(btVector3(20, 20, 20));
    //     btDefaultMotionState* boxMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(rand() % 50 - 25, rand() % 50 + 25, rand() % 50 - 25)));
    //     btRigidBody::btRigidBodyConstructionInfo boxRigidBodyCI(0, boxMotionState, boxShape, btVector3(0, 0, 0));
    //     btRigidBody* boxRigidBody = new btRigidBody(boxRigidBodyCI);
    //     wrapper->dynamicsWorld->addRigidBody(boxRigidBody);
    // }


    // // add friction to the ground
    // groundRigidBody->setFriction(0.5);
    // groundRigidBody->setRestitution(0.2);


    // wrapper->dynamicsWorld->addRigidBody(groundRigidBody);
    
    return wrapper;
}

void btDynamicsWorld_destroy(btDynamicsWorldWrapper *wrapper) {
    delete wrapper->dynamicsWorld;
    delete wrapper->solver;
    delete wrapper->overlappingPairCache;
    delete wrapper->dispatcher;
    delete wrapper->collisionConfiguration;
    delete wrapper;
}

void btDynamicsWorld_setGravity(btDynamicsWorldWrapper *world, vec3 *gravity) {
    world->dynamicsWorld->setGravity(vec3_to_btVector3(gravity));
}

void btDynamicsWorld_getGravity(btDynamicsWorldWrapper *world, vec3 *gravity) {
    btVector3 btGravity = world->dynamicsWorld->getGravity();
    btVector3_to_vec3(&btGravity, gravity);
}

void btDynamicsWorld_addRigidBody(btDynamicsWorldWrapper *worldWrapper, btRigidBodyWrapper *bodyWrapper) {
    worldWrapper->dynamicsWorld->addRigidBody(bodyWrapper->rigidBody);
    bodyWrapper->assignedWorld = worldWrapper->dynamicsWorld;
}

void btDynamicsWorld_addConstraint(btDynamicsWorldWrapper *worldWrapper, btTypedConstraintWrapper *constraint, bool noclip) {
    worldWrapper->dynamicsWorld->addConstraint(constraint->constraint, noclip);
}

void btDynamicsWorld_removeConstraint(btDynamicsWorldWrapper *worldWrapper, btTypedConstraintWrapper *constraint) {
    worldWrapper->dynamicsWorld->removeConstraint(constraint->constraint);
}

void btDynamicsWorld_stepSimulation(btDynamicsWorldWrapper *worldWrapper, float timeStep) {
    worldWrapper->dynamicsWorld->stepSimulation(timeStep);
}

void btDynamicsWorld_setDebugRenderer(btDynamicsWorldWrapper *world, btDebugRendererWrapper *renderer)
{
    world->dynamicsWorld->setDebugDrawer(renderer->debugDrawer);
}

void btDynamicsWorld_debugDrawWorld(btDynamicsWorldWrapper *world)
{
    world->dynamicsWorld->debugDrawWorld();
}

void btDynamicsWorld_enableDebugDrawing(btDynamicsWorldWrapper *world)
{
    phpglfwBtDebugDraw *debugDrawer = new phpglfwBtDebugDraw();
    debugDrawer->setDebugMode(
        btIDebugDraw::DBG_DrawWireframe | 
        btIDebugDraw::DBG_DrawConstraints 
        // btIDebugDraw::DBG_DrawAabb | 
        // btIDebugDraw::DBG_DrawContactPoints | 
        // btIDebugDraw::DBG_DrawNormals
    );
    world->dynamicsWorld->setDebugDrawer(debugDrawer);
}

void btDynamicsWorld_setDebugDrawVP(btDynamicsWorldWrapper *world, mat4x4 *vpMatrix)
{
    phpglfwBtDebugDraw *debugDrawer = static_cast<phpglfwBtDebugDraw *>(world->dynamicsWorld->getDebugDrawer());
    debugDrawer->setViewProjection(&(*vpMatrix)[0][0]);
}

/**
 * Collision shape
 * 
 * ----------------------------------------------------------------------------
 */

void btCollisionShape_destroy(btCollisionShapeWrapper *shape)
{
    delete shape->shape;
    delete shape;
}

btCollisionShapeWrapper *btCollisionShape_create_sphere(float radius)
{
    btCollisionShapeWrapper *wrapper = new btCollisionShapeWrapper;
    wrapper->shape = new btSphereShape(radius);
    return wrapper;
}

btCollisionShapeWrapper *btCollisionShape_create_box(vec3 *halfExtents)
{
    btCollisionShapeWrapper *wrapper = new btCollisionShapeWrapper;
    wrapper->shape = new btBoxShape(vec3_to_btVector3(halfExtents));
    return wrapper;
}

btCollisionShapeWrapper *btCollisionShape_create_cylinder(vec3 *halfExtents)
{
    btCollisionShapeWrapper *wrapper = new btCollisionShapeWrapper;
    wrapper->shape = new btCylinderShape(vec3_to_btVector3(halfExtents));
    return wrapper;
}

btCollisionShapeWrapper *btCollisionShape_create_cylinderX(vec3 *halfExtents)
{
    btCollisionShapeWrapper *wrapper = new btCollisionShapeWrapper;
    wrapper->shape = new btCylinderShapeX(vec3_to_btVector3(halfExtents));
    return wrapper;
}

btCollisionShapeWrapper *btCollisionShape_create_cylinderZ(vec3 *halfExtents)
{
    btCollisionShapeWrapper *wrapper = new btCollisionShapeWrapper;
    wrapper->shape = new btCylinderShapeZ(vec3_to_btVector3(halfExtents));
    return wrapper;
}

btCollisionShapeWrapper *btCollisionShape_create_static_plane(vec3 *normal, float constant)
{
    btCollisionShapeWrapper *wrapper = new btCollisionShapeWrapper;
    wrapper->shape = new btStaticPlaneShape(vec3_to_btVector3(normal), constant);
    return wrapper;
}

/**
 * Contraints
 * 
 * ----------------------------------------------------------------------------
 */

void btTypedConstraint_destroy(btTypedConstraintWrapper *constraint)
{
    delete constraint->constraint;
    delete constraint;
}

btTypedConstraintWrapper *btPoint2PointConstraint_create(btRigidBodyWrapper *bodyA, btRigidBodyWrapper *bodyB, vec3 *pivotA, vec3 *pivotB)
{
    btTypedConstraintWrapper *wrapper = new btTypedConstraintWrapper;
    wrapper->constraint = new btPoint2PointConstraint(*bodyA->rigidBody, *bodyB->rigidBody, vec3_to_btVector3(pivotA), vec3_to_btVector3(pivotB));
    return wrapper;
}

btTypedConstraintWrapper *btHingeConstraint_create(btRigidBodyWrapper *bodyA, btRigidBodyWrapper *bodyB, vec3 *pivotA, vec3 *pivotB, vec3 *axisA, vec3 *axisB)
{
    btTypedConstraintWrapper *wrapper = new btTypedConstraintWrapper;
    wrapper->constraint = new btHingeConstraint(*bodyA->rigidBody, *bodyB->rigidBody, vec3_to_btVector3(pivotA), vec3_to_btVector3(pivotB), vec3_to_btVector3(axisA), vec3_to_btVector3(axisB));
    return wrapper;
}

void btHingeConstraint_setLimit(btTypedConstraintWrapper *constraint, float low, float high, float softness, float biasFactor, float relaxationFactor)
{
    btHingeConstraint *hinge = static_cast<btHingeConstraint *>(constraint->constraint);
    hinge->setLimit(low, high, softness, biasFactor, relaxationFactor);
}

btTypedConstraintWrapper *btSliderConstraint_create(btRigidBodyWrapper *bodyA, btRigidBodyWrapper *bodyB, mat4x4 *frameInA, mat4x4 *frameInB, bool useLinearReferenceFrameA)
{
    btTypedConstraintWrapper *wrapper = new btTypedConstraintWrapper;
    wrapper->constraint = new btSliderConstraint(*bodyA->rigidBody, *bodyB->rigidBody, mat4x4_to_btTransform(frameInA), mat4x4_to_btTransform(frameInB), useLinearReferenceFrameA);
    return wrapper;
}

btTypedConstraintWrapper *btGeneric6DofSpringConstraint_create(btRigidBodyWrapper *bodyA, btRigidBodyWrapper *bodyB, mat4x4 *frameInA, mat4x4 *frameInB, bool useLinearReferenceFrameA)
{
    btTypedConstraintWrapper *wrapper = new btTypedConstraintWrapper;
    wrapper->constraint = new btGeneric6DofSpringConstraint(*bodyA->rigidBody, *bodyB->rigidBody, mat4x4_to_btTransform(frameInA), mat4x4_to_btTransform(frameInB), useLinearReferenceFrameA);
    return wrapper;
}
// btRigidBody & rbA, btRigidBody & rbB, const btTransform& frameInA, const btTransform& frameInB, RotateOrder rotOrder = RO_XYZ);
btTypedConstraintWrapper *btGeneric6DofSpring2Constraint_create(btRigidBodyWrapper *bodyA, btRigidBodyWrapper *bodyB, mat4x4 *frameInA, mat4x4 *frameInB)
{
    btTypedConstraintWrapper *wrapper = new btTypedConstraintWrapper;
    wrapper->constraint = new btGeneric6DofSpring2Constraint(*bodyA->rigidBody, *bodyB->rigidBody, mat4x4_to_btTransform(frameInA), mat4x4_to_btTransform(frameInB), RO_XYZ);
    return wrapper;
}

void btGeneric6DofSpring2Constraint_setFrames(btTypedConstraintWrapper *constraint, mat4x4 *frameInA, mat4x4 *frameInB)
{
    btGeneric6DofSpring2Constraint *g6dof = static_cast<btGeneric6DofSpring2Constraint *>(constraint->constraint);
    g6dof->setFrames(mat4x4_to_btTransform(frameInA), mat4x4_to_btTransform(frameInB));
}

void btGeneric6DofSpring2Constraint_setLinearLowerLimit(btTypedConstraintWrapper *constraint, const vec3 *linearLower)
{
    btGeneric6DofSpring2Constraint *g6dof = static_cast<btGeneric6DofSpring2Constraint *>(constraint->constraint);
    g6dof->setLinearLowerLimit(vec3_to_btVector3(linearLower));
}

void btGeneric6DofSpring2Constraint_setLinearUpperLimit(btTypedConstraintWrapper *constraint, const vec3 *linearUpper)
{
    btGeneric6DofSpring2Constraint *g6dof = static_cast<btGeneric6DofSpring2Constraint *>(constraint->constraint);
    g6dof->setLinearUpperLimit(vec3_to_btVector3(linearUpper));
}

void btGeneric6DofSpring2Constraint_setAngularLowerLimit(btTypedConstraintWrapper *constraint, const vec3 *angularLower)
{
    btGeneric6DofSpring2Constraint *g6dof = static_cast<btGeneric6DofSpring2Constraint *>(constraint->constraint);
    g6dof->setAngularLowerLimit(vec3_to_btVector3(angularLower));
}
void btGeneric6DofSpring2Constraint_setAngularLowerLimitReversed(btTypedConstraintWrapper *constraint, const vec3 *angularLower)
{
    btGeneric6DofSpring2Constraint *g6dof = static_cast<btGeneric6DofSpring2Constraint *>(constraint->constraint);
    g6dof->setAngularLowerLimitReversed(vec3_to_btVector3(angularLower));
}

void btGeneric6DofSpring2Constraint_setAngularUpperLimit(btTypedConstraintWrapper *constraint, const vec3 *angularUpper)
{
    btGeneric6DofSpring2Constraint *g6dof = static_cast<btGeneric6DofSpring2Constraint *>(constraint->constraint);
    g6dof->setAngularUpperLimit(vec3_to_btVector3(angularUpper));
}
void btGeneric6DofSpring2Constraint_setAngularUpperLimitReversed(btTypedConstraintWrapper *constraint, const vec3 *angularUpper)
{
    btGeneric6DofSpring2Constraint *g6dof = static_cast<btGeneric6DofSpring2Constraint *>(constraint->constraint);
    g6dof->setAngularUpperLimitReversed(vec3_to_btVector3(angularUpper));
}

void btGeneric6DofSpring2Constraint_setLimit(btTypedConstraintWrapper *constraint, int axis, float lo, float hi)
{
    btGeneric6DofSpring2Constraint *g6dof = static_cast<btGeneric6DofSpring2Constraint *>(constraint->constraint);
    g6dof->setLimit(axis, lo, hi);
}
void btGeneric6DofSpring2Constraint_setLimitReversed(btTypedConstraintWrapper *constraint, int axis, float lo, float hi)
{
    btGeneric6DofSpring2Constraint *g6dof = static_cast<btGeneric6DofSpring2Constraint *>(constraint->constraint);
    g6dof->setLimitReversed(axis, lo, hi);
}

void btGeneric6DofSpring2Constraint_setAxis(btTypedConstraintWrapper *constraint, vec3 *axis1, vec3 *axis2)
{
    btGeneric6DofSpring2Constraint *g6dof = static_cast<btGeneric6DofSpring2Constraint *>(constraint->constraint);
    g6dof->setAxis(vec3_to_btVector3(axis1), vec3_to_btVector3(axis2));
}

void btGeneric6DofSpring2Constraint_setBounce(btTypedConstraintWrapper *constraint, int index, float bounce)
{
    btGeneric6DofSpring2Constraint *g6dof = static_cast<btGeneric6DofSpring2Constraint *>(constraint->constraint);
    g6dof->setBounce(index, bounce);
}

void btGeneric6DofSpring2Constraint_enableMotor(btTypedConstraintWrapper *constraint, int index, bool onOff)
{
    btGeneric6DofSpring2Constraint *g6dof = static_cast<btGeneric6DofSpring2Constraint *>(constraint->constraint);
    g6dof->enableMotor(index, onOff);
}

void btGeneric6DofSpring2Constraint_setServo(btTypedConstraintWrapper *constraint, int index, bool onOff)
{
    btGeneric6DofSpring2Constraint *g6dof = static_cast<btGeneric6DofSpring2Constraint *>(constraint->constraint);
    g6dof->setServo(index, onOff);
}

void btGeneric6DofSpring2Constraint_setTargetVelocity(btTypedConstraintWrapper *constraint, int index, float velocity)
{
    btGeneric6DofSpring2Constraint *g6dof = static_cast<btGeneric6DofSpring2Constraint *>(constraint->constraint);
    g6dof->setTargetVelocity(index, velocity);
}

void btGeneric6DofSpring2Constraint_setServoTarget(btTypedConstraintWrapper *constraint, int index, float target)
{
    btGeneric6DofSpring2Constraint *g6dof = static_cast<btGeneric6DofSpring2Constraint *>(constraint->constraint);
    g6dof->setServoTarget(index, target);
}

void btGeneric6DofSpring2Constraint_setMaxMotorForce(btTypedConstraintWrapper *constraint, int index, float force)
{
    btGeneric6DofSpring2Constraint *g6dof = static_cast<btGeneric6DofSpring2Constraint *>(constraint->constraint);
    g6dof->setMaxMotorForce(index, force);
}

void btGeneric6DofSpring2Constraint_enableSpring(btTypedConstraintWrapper *constraint, int index, bool onOff)
{
    btGeneric6DofSpring2Constraint *g6dof = static_cast<btGeneric6DofSpring2Constraint *>(constraint->constraint);
    g6dof->enableSpring(index, onOff);
}

void btGeneric6DofSpring2Constraint_setStiffness(btTypedConstraintWrapper *constraint, int index, float stiffness, bool limitIfNeeded)
{
    btGeneric6DofSpring2Constraint *g6dof = static_cast<btGeneric6DofSpring2Constraint *>(constraint->constraint);
    g6dof->setStiffness(index, stiffness, limitIfNeeded);
}

void btGeneric6DofSpring2Constraint_setDamping(btTypedConstraintWrapper *constraint, int index, float damping, bool limitIfNeeded)
{
    btGeneric6DofSpring2Constraint *g6dof = static_cast<btGeneric6DofSpring2Constraint *>(constraint->constraint);
    g6dof->setDamping(index, damping, limitIfNeeded);
}

void btGeneric6DofSpring2Constraint_setEquilibriumPoint(btTypedConstraintWrapper *constraint)
{
    btGeneric6DofSpring2Constraint *g6dof = static_cast<btGeneric6DofSpring2Constraint *>(constraint->constraint);
    g6dof->setEquilibriumPoint();
}

/**
 * Rigid body
 * 
 * ----------------------------------------------------------------------------
 */
btRigidBodyWrapper *btRigidBody_create(btCollisionShapeWrapper* shape, float mass)
{
    btRigidBodyWrapper *wrapper = new btRigidBodyWrapper;
    wrapper->shape = shape->shape;

    btVector3 localInertia(0, 0, 0);
    if (mass != 0.0f) {
        wrapper->shape->calculateLocalInertia(mass, localInertia);
    }

    wrapper->motionState = new btDefaultMotionState();

    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, wrapper->motionState, wrapper->shape, localInertia);
    wrapper->rigidBody = new btRigidBody(rbInfo);

    return wrapper;
}


void btRigidBody_destroy(btRigidBodyWrapper *wrapper) {
    if (wrapper->assignedWorld != nullptr) {
        wrapper->assignedWorld->removeRigidBody(wrapper->rigidBody);
    }

    delete wrapper->rigidBody;
    delete wrapper->motionState;
    delete wrapper;
}

void btRigidBody_setPosition(btRigidBodyWrapper *body, vec3 *position) {
    body->rigidBody->getWorldTransform().setOrigin(vec3_to_btVector3(position));
}

void btRigidBody_setQuaternion(btRigidBodyWrapper *body, quat *orientation) {
    body->rigidBody->getWorldTransform().setRotation(quat_to_btQuaternion(orientation));
}

void btRigidBody_setMass(btRigidBodyWrapper *body, float mass)
{
    btVector3 localInertia(0, 0, 0);
    if (mass != 0.0f)
        body->shape->calculateLocalInertia(mass, localInertia);
    body->rigidBody->setMassProps(mass, localInertia);
}

void btRigidBody_setRestitution(btRigidBodyWrapper *body, float restitution) {
    body->rigidBody->setRestitution(restitution);
}
void btRigidBody_setFriction(btRigidBodyWrapper *body, float friction) {
    body->rigidBody->setFriction(friction);
}
void btRigidBody_setRollingFriction(btRigidBodyWrapper *body, float rollingFriction) {
    body->rigidBody->setRollingFriction(rollingFriction);
}
void btRigidBody_setSpinningFriction(btRigidBodyWrapper *body, float spinningFriction) {
    body->rigidBody->setSpinningFriction(spinningFriction);
}
void btRigidBody_setContactStiffnessAndDamping(btRigidBodyWrapper *body, float stiffness, float damping) {
    body->rigidBody->setContactStiffnessAndDamping(stiffness, damping);
}

void btRigidBody_getPosition(btRigidBodyWrapper *bodyWrapper, vec3 *position) {
    btVector3 btPosition = bodyWrapper->rigidBody->getWorldTransform().getOrigin();
    btVector3_to_vec3(&btPosition, position);
}

void btRigidBody_getLinearVelocity(btRigidBodyWrapper *bodyWrapper, vec3 *velocity) {
    btVector3 btVelocity = bodyWrapper->rigidBody->getLinearVelocity();
    btVector3_to_vec3(&btVelocity, velocity);
}

void btRigidBody_getAngularVelocity(btRigidBodyWrapper *bodyWrapper, vec3 *velocity) {
    btVector3 btVelocity = bodyWrapper->rigidBody->getAngularVelocity();
    btVector3_to_vec3(&btVelocity, velocity);
}

void btRigidBody_getQuaternion(btRigidBodyWrapper *bodyWrapper, quat* quaternion) {
    btQuaternion btQuaternion = bodyWrapper->rigidBody->getWorldTransform().getRotation();
    btQuaternion_to_quat(&btQuaternion, quaternion);
}

void btRigidBody_getOpenGLMatrix(btRigidBodyWrapper *bodyWrapper, mat4x4* matrix) {
    bodyWrapper->rigidBody->getWorldTransform().getOpenGLMatrix(matrix[0][0]);
}

void btRigidBody_applyForce(btRigidBodyWrapper *body, vec3 *force, vec3 *rel_pos) {
    body->rigidBody->applyForce(vec3_to_btVector3(force), vec3_to_btVector3(rel_pos));
}

void btRigidBody_applyCentralForce(btRigidBodyWrapper *body, vec3 *force) {
    body->rigidBody->applyCentralForce(vec3_to_btVector3(force));
}

void btRigidBody_applyTorque(btRigidBodyWrapper *body, vec3 *torque) {
    body->rigidBody->applyTorque(vec3_to_btVector3(torque));
}

void btRigidBody_applyImpulse(btRigidBodyWrapper *body, vec3 *impulse, vec3 *rel_pos) {
    body->rigidBody->applyImpulse(vec3_to_btVector3(impulse), vec3_to_btVector3(rel_pos));
}

void btRigidBody_applyCentralImpulse(btRigidBodyWrapper *body, vec3 *impulse) {
    body->rigidBody->applyCentralImpulse(vec3_to_btVector3(impulse));
}

void btRigidBody_applyTorqueImpulse(btRigidBodyWrapper *body, vec3 *torque) {
    body->rigidBody->applyTorqueImpulse(vec3_to_btVector3(torque));
}

void btRigidBody_activate(btRigidBodyWrapper *body) {
    body->rigidBody->activate();
}

void btRigidBody_disableDeactivation(btRigidBodyWrapper *body) {
    body->rigidBody->setActivationState(DISABLE_DEACTIVATION);
}

/**
 * Debug renderer
 * 
 * ----------------------------------------------------------------------------
 */


btDebugRendererWrapper *btDebugRenderer_create()
{
    btDebugRendererWrapper *wrapper = new btDebugRendererWrapper;
    wrapper->debugDrawer = new phpglfwBtDebugDraw();
    return wrapper;
}


void btDebugRenderer_destroy(btDebugRendererWrapper *renderer)
{
    delete renderer->debugDrawer;
    delete renderer;
}

void btDebugRenderer_setVP(btDebugRendererWrapper *renderer, mat4x4 *vpMatrix)
{
    renderer->debugDrawer->setViewProjection(&(*vpMatrix)[0][0]);
}
