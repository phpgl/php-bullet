#include "bullet3_bridge.h"
#include "btBulletDynamicsCommon.h"

struct btDynamicsWorldWrapper {
    btDefaultCollisionConfiguration* collisionConfiguration;
    btCollisionDispatcher* dispatcher;
    btBroadphaseInterface* overlappingPairCache;
    btSequentialImpulseConstraintSolver* solver;
    btDiscreteDynamicsWorld* dynamicsWorld;
};

struct btRigidBodyWrapper {
    btCollisionShape* shape;
    btDefaultMotionState* motionState;
    btRigidBody* rigidBody;
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

btDynamicsWorldWrapper* btDynamicsWorld_create() {
    btDynamicsWorldWrapper* wrapper = new btDynamicsWorldWrapper;
    wrapper->collisionConfiguration = new btDefaultCollisionConfiguration();
    wrapper->dispatcher = new btCollisionDispatcher(wrapper->collisionConfiguration);
    wrapper->overlappingPairCache = new btDbvtBroadphase();
    wrapper->solver = new btSequentialImpulseConstraintSolver();
    wrapper->dynamicsWorld = new btDiscreteDynamicsWorld(wrapper->dispatcher, wrapper->overlappingPairCache, wrapper->solver, wrapper->collisionConfiguration);

    // add a ground plane for testing
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);
    btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
    btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
    btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);

    // add friction to the ground
    groundRigidBody->setFriction(0.5);
    groundRigidBody->setRestitution(0.2);


    wrapper->dynamicsWorld->addRigidBody(groundRigidBody);
    


    return wrapper;
}

void btDynamicsWorld_destroy(btDynamicsWorldWrapper* wrapper) {
    delete wrapper->dynamicsWorld;
    delete wrapper->solver;
    delete wrapper->overlappingPairCache;
    delete wrapper->dispatcher;
    delete wrapper->collisionConfiguration;
    delete wrapper;
}

void btDynamicsWorld_setGravity(btDynamicsWorldWrapper* world, vec3* gravity) {
    world->dynamicsWorld->setGravity(vec3_to_btVector3(gravity));
}

void btDynamicsWorld_getGravity(btDynamicsWorldWrapper* world, vec3* gravity) {
    btVector3 btGravity = world->dynamicsWorld->getGravity();
    btVector3_to_vec3(&btGravity, gravity);
}

/**
 * Rigid body
 * 
 * ----------------------------------------------------------------------------
 */

btRigidBodyWrapper* btRigidBody_create_sphere(float radius, float mass) {
    btRigidBodyWrapper* wrapper = new btRigidBodyWrapper;
    wrapper->shape = new btSphereShape(radius);
    btVector3 localInertia(0, 0, 0);
    if (mass != 0.0f)
        wrapper->shape->calculateLocalInertia(mass, localInertia);
    wrapper->motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 50, 0)));
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, wrapper->motionState, wrapper->shape, localInertia);
    wrapper->rigidBody = new btRigidBody(rbInfo);


    // add friction to the sphere
    wrapper->rigidBody->setFriction(0.5);

    // make em bouncy
    wrapper->rigidBody->setRestitution(0.9);

    return wrapper;
}

void btRigidBody_destroy(btRigidBodyWrapper* wrapper) {
    delete wrapper->rigidBody;
    delete wrapper->motionState;
    delete wrapper->shape;
    delete wrapper;
}

void btRigidBody_setPosition(btRigidBodyWrapper* body, vec3 *position)
{
    body->rigidBody->getWorldTransform().setOrigin(vec3_to_btVector3(position));
}


void btRigidBody_setMass(btRigidBodyWrapper* body, float mass)
{
    btVector3 localInertia(0, 0, 0);
    if (mass != 0.0f)
        body->shape->calculateLocalInertia(mass, localInertia);
    body->rigidBody->setMassProps(mass, localInertia);
}


void btDynamicsWorld_addRigidBody(btDynamicsWorldWrapper* worldWrapper, btRigidBodyWrapper* bodyWrapper) {
    worldWrapper->dynamicsWorld->addRigidBody(bodyWrapper->rigidBody);
}

void btDynamicsWorld_stepSimulation(btDynamicsWorldWrapper* worldWrapper, float timeStep) {
    worldWrapper->dynamicsWorld->stepSimulation(timeStep);
}

void btRigidBody_getPosition(btRigidBodyWrapper* bodyWrapper, vec3* position) {
    btVector3 btPosition = bodyWrapper->rigidBody->getWorldTransform().getOrigin();
    btVector3_to_vec3(&btPosition, position);
}


void btRigidBody_getQuaternion(btRigidBodyWrapper* bodyWrapper, quat* quaternion) {
    btQuaternion btQuaternion = bodyWrapper->rigidBody->getWorldTransform().getRotation();
    btQuaternion_to_quat(&btQuaternion, quaternion);
}

void btRigidBody_getOpenGLMatrix(btRigidBodyWrapper* bodyWrapper, mat4x4* matrix) {
    bodyWrapper->rigidBody->getWorldTransform().getOpenGLMatrix(matrix[0][0]);
}
    