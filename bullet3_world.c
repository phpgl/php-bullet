/**
 * php-bullet3 - Bullet Physics Bindings for PHP
 * 
 * World Module
 *
 * Copyright (c) 2018-2024 Mario Döring
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "bullet3_world.h"
#include "bullet3_constraint.h"
#include "bullet3_arginfo.h"
#include "glfw/phpglfw_math.h"

#define PHPBULLET_DEFINE_COLLISION_SHAPE_IMPL(type_lower) \
    zend_class_entry *phpbullet3_##type_lower##_ce; \
    \
    zend_class_entry *phpbullet3_get_##type_lower##_ce() \
    { \
        return phpbullet3_##type_lower##_ce; \
    } \
    zend_always_inline phpbullet3_##type_lower##_object *phpbullet3_##type_lower##_from_zobj_p(zend_object *obj) \
    { \
        return (phpbullet3_##type_lower##_object*)((char*)(obj) - XtOffsetOf(phpbullet3_##type_lower##_object, std)); \
    } \
    static zend_object_handlers phpbullet3_##type_lower##_object_handlers; \
    \
    zend_object *phpbullet3_##type_lower##_create_object(zend_class_entry *class_type) \
    { \
        phpbullet3_##type_lower##_object *intern; \
        intern = zend_object_alloc(sizeof(phpbullet3_##type_lower##_object), class_type); \
        \
        zend_object_std_init(&intern->std, class_type); \
        object_properties_init(&intern->std, class_type); \
        \
        intern->std.handlers = &phpbullet3_##type_lower##_object_handlers; \
        intern->bt_shape = NULL; \
        \
        return &intern->std; \
    } \
    static void phpbullet3_##type_lower##_free_handler(zend_object *object) \
    { \
        phpbullet3_##type_lower##_object *intern = phpbullet3_##type_lower##_from_zobj_p(object); \
        \
        if (intern->bt_shape != NULL) { \
            btCollisionShape_destroy(intern->bt_shape); \
        } \
        \
        zend_object_std_dtor(&intern->std); \
    }
    \

#define PHPBULLET_DEFINE_COLLISION_SHAPE_REGISTER(class, type_lower) \
    phpbullet3_##type_lower##_ce = register_class_##class(phpbullet3_collision_shape_ce); \
    phpbullet3_##type_lower##_ce->create_object = phpbullet3_##type_lower##_create_object; \
    \
    memcpy(&phpbullet3_##type_lower##_object_handlers, zend_get_std_object_handlers(), sizeof(zend_object_handlers)); \
    phpbullet3_##type_lower##_object_handlers.offset = XtOffsetOf(phpbullet3_##type_lower##_object, std); \
    phpbullet3_##type_lower##_object_handlers.free_obj = phpbullet3_##type_lower##_free_handler;


zend_class_entry *phpbullet3_world_ce;
zend_class_entry *phpbullet3_rigidbody_ce;
zend_class_entry *phpbullet3_collision_shape_ce;

zend_class_entry *phpbullet3_get_world_ce()
{
    return phpbullet3_world_ce;
}

zend_class_entry *phpbullet3_get_rigidbody_ce()
{
    return phpbullet3_rigidbody_ce;
}

zend_class_entry *phpbullet3_get_collision_shape_ce()
{
    return phpbullet3_collision_shape_ce;
}

zend_always_inline phpbullet3_world_object *phpbullet3_world_from_zobj_p(zend_object *obj)
{
    return (phpbullet3_world_object*)((char*)(obj) - XtOffsetOf(phpbullet3_world_object, std));
}

zend_always_inline phpbullet3_rigidbody_object *phpbullet3_rigidbody_from_zobj_p(zend_object *obj)
{
    return (phpbullet3_rigidbody_object*)((char*)(obj) - XtOffsetOf(phpbullet3_rigidbody_object, std));
}

static zend_object_handlers phpbullet3_world_object_handlers;
static zend_object_handlers phpbullet3_rigidbody_object_handlers;

// shapes
PHPBULLET_DEFINE_COLLISION_SHAPE_IMPL(shape_sphere);
PHPBULLET_DEFINE_COLLISION_SHAPE_IMPL(shape_box);
PHPBULLET_DEFINE_COLLISION_SHAPE_IMPL(shape_cylinder);
PHPBULLET_DEFINE_COLLISION_SHAPE_IMPL(shape_cylinderX);
PHPBULLET_DEFINE_COLLISION_SHAPE_IMPL(shape_cylinderZ);
PHPBULLET_DEFINE_COLLISION_SHAPE_IMPL(shape_static_plane);

/**
 * Bullet\World
 * 
 * ----------------------------------------------------------------------------
 */
zend_object *phpbullet3_world_create_object(zend_class_entry *class_type)
{
    phpbullet3_world_object *intern;
    intern = zend_object_alloc(sizeof(phpbullet3_world_object), class_type);

    zend_object_std_init(&intern->std, class_type);
    object_properties_init(&intern->std, class_type);

    intern->std.handlers = &phpbullet3_world_object_handlers;
    intern->bt_world = btDynamicsWorld_create();

    return &intern->std;
}


static void phpbullet3_world_free_handler(zend_object *object)
{
    phpbullet3_world_object *intern = phpbullet3_world_from_zobj_p(object);

    if (intern->bt_world != NULL) {
        btDynamicsWorld_destroy(intern->bt_world);
    }

    zend_object_std_dtor(&intern->std);
}

/**
 * Bullet\World::setGravity
 */
PHP_METHOD(Bullet_World, setGravity)
{
    zval *gravity;
    phpbullet3_world_object *intern = phpbullet3_world_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "O", &gravity, phpglfw_get_math_vec3_ce()) == FAILURE) {
        return;
    }

    phpglfw_math_vec3_object *vec3_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(gravity));

    btDynamicsWorld_setGravity(intern->bt_world, &vec3_ptr->data);
}

/**
 * Bullet\World::getGravity
 */
PHP_METHOD(Bullet_World, getGravity)
{
    phpbullet3_world_object *intern = phpbullet3_world_from_zobj_p(Z_OBJ_P(getThis()));

    // construct a vec3 object
    object_init_ex(return_value, phpglfw_get_math_vec3_ce());
    phpglfw_math_vec3_object *vec_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(return_value));

    // read the gravity from the world
    btDynamicsWorld_getGravity(intern->bt_world, &vec_ptr->data);
}

/**
 * Bullet\World::addRigidBody
 */
PHP_METHOD(Bullet_World, addRigidBody)
{
    zval *rigidbody;
    phpbullet3_world_object *intern = phpbullet3_world_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "O", &rigidbody, phpbullet3_get_rigidbody_ce()) == FAILURE) {
        return;
    }

    phpbullet3_rigidbody_object *rigidbody_ptr = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(rigidbody));

    btDynamicsWorld_addRigidBody(intern->bt_world, rigidbody_ptr->bt_rigidbody);
}

/**
 * Bullet\World::addConstraint
 */
PHP_METHOD(Bullet_World, addConstraint)
{
    zval *constraint_zv;
    zend_bool disableCollisions = 0;

    phpbullet3_world_object *intern = phpbullet3_world_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "O|b", &constraint_zv, phpbullet3_get_constraint_ce(), &disableCollisions) == FAILURE) {
        return;
    }

    phpbullet3_constraint_wrapper_object *constraint_obj = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(constraint_zv));

    btDynamicsWorld_addConstraint(
        intern->bt_world,
        constraint_obj->bt_constraint,
        (bool) disableCollisions
    );
}

/**
 * Bullet\World::removeConstraint
 */
PHP_METHOD(Bullet_World, removeConstraint)
{
    zval *constraint_zv;

    phpbullet3_world_object *intern = phpbullet3_world_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "O", &constraint_zv, phpbullet3_get_constraint_ce()) == FAILURE) {
        return;
    }

    phpbullet3_constraint_wrapper_object *constraint_obj = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(constraint_zv));

    btDynamicsWorld_removeConstraint(
        intern->bt_world,
        constraint_obj->bt_constraint
    );
}

/**
 * Bullet\World::stepSimulation
 */
PHP_METHOD(Bullet_World, stepSimulation)
{
    double timeStep;
    zend_long maxSubSteps = 1;
    double fixedTimeStep = 0.01666666;
    phpbullet3_world_object *intern = phpbullet3_world_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "d|ld", &timeStep, &maxSubSteps, &fixedTimeStep) == FAILURE) {
        return;
    }

    for (int i = 0; i < maxSubSteps; ++i) {
        btDynamicsWorld_stepSimulation(intern->bt_world, timeStep);
    }
}

/**
 * Bullet\CollisionShape
 * 
 * ----------------------------------------------------------------------------
 * The collision shape constrcutors
 */

/**
 * Bullet\SphereShape::__construct
 */
PHP_METHOD(Bullet_SphereShape, __construct)
{
    double radius;
    phpbullet3_shape_sphere_object *intern = phpbullet3_shape_sphere_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "d", &radius) == FAILURE) {
        return;
    }

    intern->bt_shape = btCollisionShape_create_sphere(radius);
}

/**
 * Bullet\BoxShape::__construct
 */
PHP_METHOD(Bullet_BoxShape, __construct)
{
    zval *halfExtents;
    phpbullet3_shape_box_object *intern = phpbullet3_shape_box_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "O", &halfExtents, phpglfw_get_math_vec3_ce()) == FAILURE) {
        return;
    }

    phpglfw_math_vec3_object *vec3_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(halfExtents));
    intern->bt_shape = btCollisionShape_create_box(&vec3_ptr->data);
}

/**
 * Bullet\CylinderShape::__construct
 */
PHP_METHOD(Bullet_CylinderShape, __construct)
{
    zval *halfExtents;
    phpbullet3_shape_cylinder_object *intern = phpbullet3_shape_cylinder_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "O", &halfExtents, phpglfw_get_math_vec3_ce()) == FAILURE) {
        return;
    }

    phpglfw_math_vec3_object *vec3_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(halfExtents));
    intern->bt_shape = btCollisionShape_create_cylinder(&vec3_ptr->data);
}

/**
 * Bullet\CylinderShapeX::__construct
 */
PHP_METHOD(Bullet_CylinderShapeX, __construct)
{
    zval *halfExtents;
    phpbullet3_shape_cylinderX_object *intern = phpbullet3_shape_cylinderX_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "O", &halfExtents, phpglfw_get_math_vec3_ce()) == FAILURE) {
        return;
    }

    phpglfw_math_vec3_object *vec3_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(halfExtents));
    intern->bt_shape = btCollisionShape_create_cylinderX(&vec3_ptr->data);
}

/**
 * Bullet\CylinderShapeZ::__construct
 */
PHP_METHOD(Bullet_CylinderShapeZ, __construct)
{
    zval *halfExtents;
    phpbullet3_shape_cylinderZ_object *intern = phpbullet3_shape_cylinderZ_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "O", &halfExtents, phpglfw_get_math_vec3_ce()) == FAILURE) {
        return;
    }

    phpglfw_math_vec3_object *vec3_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(halfExtents));
    intern->bt_shape = btCollisionShape_create_cylinderZ(&vec3_ptr->data);
}

/**
 * Bullet\StaticPlaneShape::__construct
 */
PHP_METHOD(Bullet_StaticPlaneShape, __construct)
{
    zval *normal;
    double constant = 0.0f;
    phpbullet3_shape_static_plane_object *intern = phpbullet3_shape_static_plane_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "O|d", &normal, phpglfw_get_math_vec3_ce(), &constant) == FAILURE) {
        return;
    }

    phpglfw_math_vec3_object *vec3_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(normal));
    intern->bt_shape = btCollisionShape_create_static_plane(&vec3_ptr->data, constant);
}

/**
 * Bullet\RigidBody
 * 
 * ----------------------------------------------------------------------------
 */
zend_object *phpbullet3_rigidbody_create_object(zend_class_entry *class_type)
{
    phpbullet3_rigidbody_object *intern;
    intern = zend_object_alloc(sizeof(phpbullet3_rigidbody_object), class_type);

    zend_object_std_init(&intern->std, class_type);
    object_properties_init(&intern->std, class_type);

    intern->std.handlers = &phpbullet3_rigidbody_object_handlers;

    return &intern->std;
}

static void phpbullet3_rigidbody_free_handler(zend_object *object)
{
    phpbullet3_rigidbody_object *intern = phpbullet3_rigidbody_from_zobj_p(object);

    if (intern->bt_rigidbody != NULL) {
        btRigidBody_destroy(intern->bt_rigidbody);
    }

    zend_object_std_dtor(&intern->std);
}

/**
 * Bullet\RigidBody::__construct
 */
PHP_METHOD(Bullet_RigidBody, __construct)
{
    zval *shape;
    double mass = 1.0f;
    phpbullet3_rigidbody_object *intern = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "O|d", &shape, phpbullet3_get_collision_shape_ce(), &mass) == FAILURE) {
        return;
    }

    phpbullet3_shape_sphere_object *shape_ptr = phpbullet3_shape_sphere_from_zobj_p(Z_OBJ_P(shape));
    intern->bt_rigidbody = btRigidBody_create(shape_ptr->bt_shape, mass);

    // also assign the collisionShape property
    zend_update_property(phpbullet3_rigidbody_ce, Z_OBJ_P(getThis()), "collisionShape", sizeof("collisionShape") - 1, shape);
}

/**
 * Bullet\RigidBody::setPosition
 */
PHP_METHOD(Bullet_RigidBody, setPosition)
{
    zval *position;
    phpbullet3_rigidbody_object *intern = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "O", &position, phpglfw_get_math_vec3_ce()) == FAILURE) {
        return;
    }

    phpglfw_math_vec3_object *vec3_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(position));

    btRigidBody_setPosition(intern->bt_rigidbody, &vec3_ptr->data);
}

/**
 * Bullet\RigidBody::getPosition
 */
PHP_METHOD(Bullet_RigidBody, getPosition)
{
    phpbullet3_rigidbody_object *intern = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(getThis()));

    // construct a vec3 object
    object_init_ex(return_value, phpglfw_get_math_vec3_ce());
    phpglfw_math_vec3_object *vec_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(return_value));

    btRigidBody_getPosition(intern->bt_rigidbody, &vec_ptr->data);
}

/**
 * Bullet\RigidBody::getLinearVelocity
 */
PHP_METHOD(Bullet_RigidBody, getLinearVelocity)
{
    phpbullet3_rigidbody_object *intern = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(getThis()));

    // construct a vec3 object
    object_init_ex(return_value, phpglfw_get_math_vec3_ce());
    phpglfw_math_vec3_object *vec_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(return_value));

    btRigidBody_getLinearVelocity(intern->bt_rigidbody, &vec_ptr->data);
}

/**
 * Bullet\RigidBody::getAngularVelocity
 */
PHP_METHOD(Bullet_RigidBody, getAngularVelocity)
{
    phpbullet3_rigidbody_object *intern = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(getThis()));

    // construct a vec3 object
    object_init_ex(return_value, phpglfw_get_math_vec3_ce());
    phpglfw_math_vec3_object *vec_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(return_value));

    btRigidBody_getAngularVelocity(intern->bt_rigidbody, &vec_ptr->data);
}

/**
 * Bullet\RigidBody::setMass
 */
PHP_METHOD(Bullet_RigidBody, setMass)
{
    double mass;
    phpbullet3_rigidbody_object *intern = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "d", &mass) == FAILURE) {
        return;
    }

    btRigidBody_setMass(intern->bt_rigidbody, mass);
}

/**
 * Bullet\RigidBody::setRestitution
 */
PHP_METHOD(Bullet_RigidBody, setRestitution)
{
    double restitution;
    phpbullet3_rigidbody_object *intern = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "d", &restitution) == FAILURE) {
        return;
    }

    btRigidBody_setRestitution(intern->bt_rigidbody, restitution);
}

/**
 * Bullet\RigidBody::setFriction
 */
PHP_METHOD(Bullet_RigidBody, setFriction)
{
    double friction;
    phpbullet3_rigidbody_object *intern = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "d", &friction) == FAILURE) {
        return;
    }

    btRigidBody_setFriction(intern->bt_rigidbody, friction);
}

/**
 * Bullet\RigidBody::setRollingFriction
 */
PHP_METHOD(Bullet_RigidBody, setRollingFriction)
{
    double rollingFriction;
    phpbullet3_rigidbody_object *intern = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "d", &rollingFriction) == FAILURE) {
        return;
    }

    btRigidBody_setRollingFriction(intern->bt_rigidbody, rollingFriction);
}

/**
 * Bullet\RigidBody::setSpinningFriction
 */
PHP_METHOD(Bullet_RigidBody, setSpinningFriction)
{
    double spinningFriction;
    phpbullet3_rigidbody_object *intern = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "d", &spinningFriction) == FAILURE) {
        return;
    }

    btRigidBody_setSpinningFriction(intern->bt_rigidbody, spinningFriction);
}

/**
 * Bullet\RigidBody::setContactStiffnessAndDamping
 */
PHP_METHOD(Bullet_RigidBody, setContactStiffnessAndDamping)
{
    double stiffness, damping;
    phpbullet3_rigidbody_object *intern = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "dd", &stiffness, &damping) == FAILURE) {
        return;
    }

    btRigidBody_setContactStiffnessAndDamping(intern->bt_rigidbody, stiffness, damping);
}

/**
 * Buller\RigidBody::getTransform (mat4)
 */
PHP_METHOD(Bullet_RigidBody, getTransform)
{
    phpbullet3_rigidbody_object *intern = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(getThis()));

    // construct a mat4 object
    object_init_ex(return_value, phpglfw_get_math_mat4_ce());
    phpglfw_math_mat4_object *mat_ptr = phpglfw_math_mat4_objectptr_from_zobj_p(Z_OBJ_P(return_value));

    // read the position from the rigidbody
    btRigidBody_getOpenGLMatrix(intern->bt_rigidbody, &mat_ptr->data);
}

/**
 * Bullet\RigidBody::getOrientation
 */
PHP_METHOD(Bullet_RigidBody, getOrientation)
{
    phpbullet3_rigidbody_object *intern = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(getThis()));

    // construct a quat object
    object_init_ex(return_value, phpglfw_get_math_quat_ce());
    phpglfw_math_quat_object *quat_ptr = phpglfw_math_quat_objectptr_from_zobj_p(Z_OBJ_P(return_value));

    // read the position from the rigidbody
    btRigidBody_getQuaternion(intern->bt_rigidbody, &quat_ptr->data);
}

/**
 * Bullet\RigidBody::setOrientation
 */
PHP_METHOD(Bullet_RigidBody, setOrientation)
{
    zval *quat;
    phpbullet3_rigidbody_object *intern = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "O", &quat, phpglfw_get_math_quat_ce()) == FAILURE) {
        return;
    }

    phpglfw_math_quat_object *quat_ptr = phpglfw_math_quat_objectptr_from_zobj_p(Z_OBJ_P(quat));

    btRigidBody_setQuaternion(intern->bt_rigidbody, &quat_ptr->data);
}

/**
 * Bullet\RigidBody::applyForce
 */
PHP_METHOD(Bullet_RigidBody, applyForce)
{
    zval *force, *rel_pos;
    phpbullet3_rigidbody_object *intern = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "OO", &force, phpglfw_get_math_vec3_ce(), &rel_pos, phpglfw_get_math_vec3_ce()) == FAILURE) {
        return;
    }

    phpglfw_math_vec3_object *force_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(force));
    phpglfw_math_vec3_object *rel_pos_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(rel_pos));

    btRigidBody_applyForce(intern->bt_rigidbody, &force_ptr->data, &rel_pos_ptr->data);
}

/**
 * Bullet\RigidBody::applyCentralForce
 */
PHP_METHOD(Bullet_RigidBody, applyCentralForce)
{
    zval *force;
    phpbullet3_rigidbody_object *intern = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "O", &force, phpglfw_get_math_vec3_ce()) == FAILURE) {
        return;
    }

    phpglfw_math_vec3_object *vec3_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(force));

    btRigidBody_applyCentralForce(intern->bt_rigidbody, &vec3_ptr->data);
}

/**
 * Bullet\RigidBody::applyTorque
 */
PHP_METHOD(Bullet_RigidBody, applyTorque)
{
    zval *torque;
    phpbullet3_rigidbody_object *intern = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "O", &torque, phpglfw_get_math_vec3_ce()) == FAILURE) {
        return;
    }

    phpglfw_math_vec3_object *vec3_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(torque));

    btRigidBody_applyTorque(intern->bt_rigidbody, &vec3_ptr->data);
}

/**
 * Bullet\RigidBody::applyImpulse
 */
PHP_METHOD(Bullet_RigidBody, applyImpulse)
{
    zval *impulse, *rel_pos;
    phpbullet3_rigidbody_object *intern = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "OO", &impulse, phpglfw_get_math_vec3_ce(), &rel_pos, phpglfw_get_math_vec3_ce()) == FAILURE) {
        return;
    }

    phpglfw_math_vec3_object *impulse_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(impulse));
    phpglfw_math_vec3_object *rel_pos_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(rel_pos));

    btRigidBody_applyImpulse(intern->bt_rigidbody, &impulse_ptr->data, &rel_pos_ptr->data);
}

/**
 * Bullet\RigidBody::applyCentralImpulse
 */
PHP_METHOD(Bullet_RigidBody, applyCentralImpulse)
{
    zval *impulse;
    phpbullet3_rigidbody_object *intern = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "O", &impulse, phpglfw_get_math_vec3_ce()) == FAILURE) {
        return;
    }

    phpglfw_math_vec3_object *vec3_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(impulse));

    btRigidBody_applyCentralImpulse(intern->bt_rigidbody, &vec3_ptr->data);
}

/**
 * Bullet\RigidBody::applyTorqueImpulse
 */
PHP_METHOD(Bullet_RigidBody, applyTorqueImpulse)
{
    zval *torque;
    phpbullet3_rigidbody_object *intern = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS() , "O", &torque, phpglfw_get_math_vec3_ce()) == FAILURE) {
        return;
    }

    phpglfw_math_vec3_object *vec3_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(torque));

    btRigidBody_applyTorqueImpulse(intern->bt_rigidbody, &vec3_ptr->data);
}



/**
 * Bullet\RigidBody::activate
 */
PHP_METHOD(Bullet_RigidBody, activate)
{
    phpbullet3_rigidbody_object *intern = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(getThis()));

    btRigidBody_activate(intern->bt_rigidbody);
}

/**
 * Module registration
 * 
 * ----------------------------------------------------------------------------
 */
void phpbullet3_register_world_module(INIT_FUNC_ARGS)
{
    // init & register the world class
    phpbullet3_world_ce = register_class_Bullet_World();
    phpbullet3_world_ce->create_object = phpbullet3_world_create_object;

    memcpy(&phpbullet3_world_object_handlers, zend_get_std_object_handlers(), sizeof(zend_object_handlers));
    phpbullet3_world_object_handlers.offset = XtOffsetOf(phpbullet3_world_object, std);
    phpbullet3_world_object_handlers.free_obj = phpbullet3_world_free_handler;

    // init & register the rigidbody class
    phpbullet3_rigidbody_ce = register_class_Bullet_RigidBody();
    phpbullet3_rigidbody_ce->create_object = phpbullet3_rigidbody_create_object;

    memcpy(&phpbullet3_rigidbody_object_handlers, zend_get_std_object_handlers(), sizeof(zend_object_handlers));
    phpbullet3_rigidbody_object_handlers.offset = XtOffsetOf(phpbullet3_rigidbody_object, std);
    phpbullet3_rigidbody_object_handlers.free_obj = phpbullet3_rigidbody_free_handler;

    // shape interface
    phpbullet3_collision_shape_ce = register_class_Bullet_CollisionShape();

    // collision shape
    PHPBULLET_DEFINE_COLLISION_SHAPE_REGISTER(Bullet_SphereShape, shape_sphere);
    PHPBULLET_DEFINE_COLLISION_SHAPE_REGISTER(Bullet_BoxShape, shape_box);
    PHPBULLET_DEFINE_COLLISION_SHAPE_REGISTER(Bullet_CylinderShape, shape_cylinder);
    PHPBULLET_DEFINE_COLLISION_SHAPE_REGISTER(Bullet_CylinderShapeX, shape_cylinderX);
    PHPBULLET_DEFINE_COLLISION_SHAPE_REGISTER(Bullet_CylinderShapeZ, shape_cylinderZ);
    PHPBULLET_DEFINE_COLLISION_SHAPE_REGISTER(Bullet_StaticPlaneShape, shape_static_plane);
}