/**
 * php-bullet3 - Bullet Physics Bindings for PHP
 * 
 * Constraints Module
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
#include "glfw/phpglfw_math.h" 
#include "bullet3_arginfo.h"

#define PHPBULLET_DEFINE_CONSTRAINT_IMPL(type_lower) \
    zend_class_entry *phpbullet3_##type_lower##_ce; \
    zend_class_entry *phpbullet3_get_##type_lower##_ce() { \
        return phpbullet3_##type_lower##_ce; \
    } \

#define PHPBULLET_DEFINE_CONSTRAINT_REGISTER(class, type_lower) \
    phpbullet3_##type_lower##_ce = register_class_##class(phpbullet3_constraint_ce); \
    phpbullet3_##type_lower##_ce->create_object = phpbullet3_constraint_wrapper_create_object; \

zend_class_entry *phpbullet3_constraint_ce = NULL;
zend_class_entry *phpbullet3_get_constraint_ce() {
    return phpbullet3_constraint_ce;
}

zend_always_inline phpbullet3_constraint_wrapper_object *phpbullet3_constraint_from_zobj_p(zend_object *obj) {
    return (phpbullet3_constraint_wrapper_object*)((char*)(obj) - XtOffsetOf(phpbullet3_constraint_wrapper_object, std));
}

static zend_object_handlers phpbullet3_constraint_wrapper_object_handlers;

// BEGIN constraint class impelementations
PHPBULLET_DEFINE_CONSTRAINT_IMPL(point2point)
PHPBULLET_DEFINE_CONSTRAINT_IMPL(hinge)
// END constraint class impelementations

zend_object *phpbullet3_constraint_wrapper_create_object(zend_class_entry *class_type)
{
    phpbullet3_constraint_wrapper_object *intern;
    intern = zend_object_alloc(sizeof(phpbullet3_constraint_wrapper_object), class_type);

    zend_object_std_init(&intern->std, class_type);
    object_properties_init(&intern->std, class_type);

    intern->std.handlers = &phpbullet3_constraint_wrapper_object_handlers;
    intern->bt_constraint = NULL;

    return &intern->std;
}


static void phpbullet3_constraint_wrapper_free_handler(zend_object *object)
{
    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(object);

    if (intern->bt_constraint != NULL) {
        btTypedConstraint_destroy(intern->bt_constraint);
        intern->bt_constraint = NULL;
    }

    zend_object_std_dtor(&intern->std);
}

/**
 * Bullet\Point2PointConstraint::__construct
 */
PHP_METHOD(Bullet_Point2PointConstraint, __construct)
{
    zval *bodyA_zv, *bodyB_zv;
    zval *pivotA_zv, *pivotB_zv;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "OOOO",
            &bodyA_zv, phpbullet3_get_rigidbody_ce(),
            &bodyB_zv, phpbullet3_get_rigidbody_ce(),
            &pivotA_zv, phpglfw_get_math_vec3_ce(),
            &pivotB_zv, phpglfw_get_math_vec3_ce()) == FAILURE)
    {
        return;
    }

    phpbullet3_rigidbody_object *rbA =
        phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(bodyA_zv));
    phpbullet3_rigidbody_object *rbB =
        phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(bodyB_zv));

    phpglfw_math_vec3_object *pivotA_ptr =
        phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(pivotA_zv));
    phpglfw_math_vec3_object *pivotB_ptr =
        phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(pivotB_zv));


    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    // Create the actual Bullet constraint
    intern->bt_constraint = btPoint2PointConstraint_create(
        rbA->bt_rigidbody,
        rbB->bt_rigidbody,
        &pivotA_ptr->data,
        &pivotB_ptr->data
    );
}

/**
 * Bullet\HingeConstraint::__construct
 */
PHP_METHOD(Bullet_HingeConstraint, __construct)
{
    zval *bodyA_zv, *bodyB_zv;
    zval *pivotA_zv, *pivotB_zv;
    zval *axisA_zv, *axisB_zv;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "OOOOOO",
            &bodyA_zv, phpbullet3_get_rigidbody_ce(),
            &bodyB_zv, phpbullet3_get_rigidbody_ce(),
            &pivotA_zv, phpglfw_get_math_vec3_ce(),
            &pivotB_zv, phpglfw_get_math_vec3_ce(),
            &axisA_zv, phpglfw_get_math_vec3_ce(),
            &axisB_zv, phpglfw_get_math_vec3_ce()) == FAILURE)
    {
        return;
    }

    phpbullet3_rigidbody_object *rbA = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(bodyA_zv));
    phpbullet3_rigidbody_object *rbB = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(bodyB_zv));

    phpglfw_math_vec3_object *pivotA_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(pivotA_zv));
    phpglfw_math_vec3_object *pivotB_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(pivotB_zv));
    phpglfw_math_vec3_object *axisA_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(axisA_zv));
    phpglfw_math_vec3_object *axisB_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(axisB_zv));

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    // Create the actual Bullet constraint
    intern->bt_constraint = btHingeConstraint_create(
        rbA->bt_rigidbody,
        rbB->bt_rigidbody,
        &pivotA_ptr->data,
        &pivotB_ptr->data,
        &axisA_ptr->data,
        &axisB_ptr->data
    );
}


/**
 * Register the “Constraint” module
 */
void phpbullet3_register_constraint_module(INIT_FUNC_ARGS)
{   
    // contraint interface1
    phpbullet3_constraint_ce = register_class_Bullet_Constraint();

    memcpy(&phpbullet3_constraint_wrapper_object_handlers, zend_get_std_object_handlers(), sizeof(zend_object_handlers));
    phpbullet3_constraint_wrapper_object_handlers.offset = XtOffsetOf(phpbullet3_constraint_wrapper_object, std);
    phpbullet3_constraint_wrapper_object_handlers.free_obj = phpbullet3_constraint_wrapper_free_handler;
    
    // contraints
    PHPBULLET_DEFINE_CONSTRAINT_REGISTER(Bullet_Point2PointConstraint, point2point);
    PHPBULLET_DEFINE_CONSTRAINT_REGISTER(Bullet_HingeConstraint, hinge);
}