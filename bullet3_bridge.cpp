#include "bullet3_bridge.h"
#include "btBulletDynamicsCommon.h"

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
btVector3 vec3_to_btVector3(vec3 *v)
{
    return btVector3((*v)[0], (*v)[1], (*v)[2]);
}

btQuaternion quat_to_btQuaternion(quat *q)
{
    return btQuaternion((*q)[1], (*q)[2], (*q)[3], (*q)[0]);
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