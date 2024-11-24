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
#include "bullet3_arginfo.h"
#include "glfw/phpglfw_math.h"

zend_class_entry *phpbullet3_world_ce;
zend_class_entry *phpbullet3_rigidbody_ce;

zend_class_entry *phpbullet3_get_world_ce()
{
    return phpbullet3_world_ce;
}

zend_class_entry *phpbullet3_get_rigidbody_ce()
{
    return phpbullet3_rigidbody_ce;
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
    intern->bt_rigidbody = btRigidBody_create_sphere(1.0f, 1.0f);

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

    // read the position from the rigidbody
    btRigidBody_getPosition(intern->bt_rigidbody, &vec_ptr->data);
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
}