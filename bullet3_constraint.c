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
PHPBULLET_DEFINE_CONSTRAINT_IMPL(slider)
PHPBULLET_DEFINE_CONSTRAINT_IMPL(generic6dofspring)
PHPBULLET_DEFINE_CONSTRAINT_IMPL(generic6dofspring2)
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
 * Bullet\HingeConstraint::setLimit
 */
PHP_METHOD(Bullet_HingeConstraint, setLimit)
{
    double low, high;
    double softness = 0.9f;
    double biasFactor = 0.3f;
    double relaxationFactor = 1.0f;

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "dd|ddd",
            &low,
            &high,
            &softness,
            &biasFactor,
            &relaxationFactor) == FAILURE)
    {
        return;
    }

    btHingeConstraint_setLimit(
        intern->bt_constraint,
        low,
        high,
        softness,
        biasFactor,
        relaxationFactor
    );
}

/**
 * Bullet\SliderConstraint::__construct
 */
PHP_METHOD(Bullet_SliderConstraint, __construct)
{
    zval *bodyA_zv, *bodyB_zv;
    zval *frameInA_zv, *frameInB_zv;
    zend_bool useLinearReferenceFrameA = false;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "OOOO|b",
            &bodyA_zv, phpbullet3_get_rigidbody_ce(),
            &bodyB_zv, phpbullet3_get_rigidbody_ce(),
            &frameInA_zv, phpglfw_get_math_mat4_ce(),
            &frameInB_zv, phpglfw_get_math_mat4_ce(),
            &useLinearReferenceFrameA) == FAILURE)
    {
        return;
    }

    phpbullet3_rigidbody_object *rbA = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(bodyA_zv));
    phpbullet3_rigidbody_object *rbB = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(bodyB_zv));

    phpglfw_math_mat4_object *frameInA_ptr = phpglfw_math_mat4_objectptr_from_zobj_p(Z_OBJ_P(frameInA_zv));
    phpglfw_math_mat4_object *frameInB_ptr = phpglfw_math_mat4_objectptr_from_zobj_p(Z_OBJ_P(frameInB_zv));

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    // Create the actual Bullet constraint
    intern->bt_constraint = btSliderConstraint_create(
        rbA->bt_rigidbody,
        rbB->bt_rigidbody,
        &frameInA_ptr->data,
        &frameInB_ptr->data,
        useLinearReferenceFrameA
    );
}

/**
 * Bullet\Generic6DofSpringConstraint::__construct
 */
PHP_METHOD(Bullet_Generic6DofSpringConstraint, __construct)
{
    zval *bodyA_zv, *bodyB_zv;
    zval *frameInA_zv, *frameInB_zv;
    zend_bool useLinearReferenceFrameA = false;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "OOOO|b",
            &bodyA_zv, phpbullet3_get_rigidbody_ce(),
            &bodyB_zv, phpbullet3_get_rigidbody_ce(),
            &frameInA_zv, phpglfw_get_math_mat4_ce(),
            &frameInB_zv, phpglfw_get_math_mat4_ce(),
            &useLinearReferenceFrameA) == FAILURE)
    {
        return;
    }

    phpbullet3_rigidbody_object *rbA = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(bodyA_zv));
    phpbullet3_rigidbody_object *rbB = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(bodyB_zv));

    phpglfw_math_mat4_object *frameInA_ptr = phpglfw_math_mat4_objectptr_from_zobj_p(Z_OBJ_P(frameInA_zv));
    phpglfw_math_mat4_object *frameInB_ptr = phpglfw_math_mat4_objectptr_from_zobj_p(Z_OBJ_P(frameInB_zv));

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    // Create the actual Bullet constraint
    intern->bt_constraint = btGeneric6DofSpringConstraint_create(
        rbA->bt_rigidbody,
        rbB->bt_rigidbody,
        &frameInA_ptr->data,
        &frameInB_ptr->data,
        useLinearReferenceFrameA
    );
}

/**
 * Bullet\Generic6DofSpring2Constraint::__construct
 */
PHP_METHOD(Bullet_Generic6DofSpring2Constraint, __construct)
{
    zval *bodyA_zv, *bodyB_zv;
    zval *frameInA_zv, *frameInB_zv;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "OOOO",
            &bodyA_zv, phpbullet3_get_rigidbody_ce(),
            &bodyB_zv, phpbullet3_get_rigidbody_ce(),
            &frameInA_zv, phpglfw_get_math_mat4_ce(),
            &frameInB_zv, phpglfw_get_math_mat4_ce()
        ) == FAILURE)
    {
        return;
    }

    phpbullet3_rigidbody_object *rbA = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(bodyA_zv));
    phpbullet3_rigidbody_object *rbB = phpbullet3_rigidbody_from_zobj_p(Z_OBJ_P(bodyB_zv));

    phpglfw_math_mat4_object *frameInA_ptr = phpglfw_math_mat4_objectptr_from_zobj_p(Z_OBJ_P(frameInA_zv));
    phpglfw_math_mat4_object *frameInB_ptr = phpglfw_math_mat4_objectptr_from_zobj_p(Z_OBJ_P(frameInB_zv));

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    // Create the actual Bullet constraint
    intern->bt_constraint = btGeneric6DofSpring2Constraint_create(
        rbA->bt_rigidbody,
        rbB->bt_rigidbody,
        &frameInA_ptr->data,
        &frameInB_ptr->data
        // useLinearReferenceFrameA
    );
}

/**
 * Bullet\Generic6DofSpring2Constraint::setFrames
 */
PHP_METHOD(Bullet_Generic6DofSpring2Constraint, setFrames)
{
    zval *frameInA_zv, *frameInB_zv;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "OO",
            &frameInA_zv, phpglfw_get_math_mat4_ce(),
            &frameInB_zv, phpglfw_get_math_mat4_ce()) == FAILURE)
    {
        return;
    }

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    phpglfw_math_mat4_object *frameInA_ptr = phpglfw_math_mat4_objectptr_from_zobj_p(Z_OBJ_P(frameInA_zv));
    phpglfw_math_mat4_object *frameInB_ptr = phpglfw_math_mat4_objectptr_from_zobj_p(Z_OBJ_P(frameInB_zv));

    btGeneric6DofSpring2Constraint_setFrames(
        intern->bt_constraint,
        &frameInA_ptr->data,
        &frameInB_ptr->data
    );
}

/**
 * Bullet\Generic6DofSpring2Constraint::setLinearLowerLimit
 */
PHP_METHOD(Bullet_Generic6DofSpring2Constraint, setLinearLowerLimit)
{
    zval *linearLower_zv;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "O",
            &linearLower_zv, phpglfw_get_math_vec3_ce()) == FAILURE)
    {
        return;
    }

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    phpglfw_math_vec3_object *linearLower_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(linearLower_zv));

    btGeneric6DofSpring2Constraint_setLinearLowerLimit(
        intern->bt_constraint,
        &linearLower_ptr->data
    );
}

/**
 * Bullet\Generic6DofSpring2Constraint::setLinearUpperLimit
 */
PHP_METHOD(Bullet_Generic6DofSpring2Constraint, setLinearUpperLimit)
{
    zval *linearUpper_zv;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "O",
            &linearUpper_zv, phpglfw_get_math_vec3_ce()) == FAILURE)
    {
        return;
    }

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    phpglfw_math_vec3_object *linearUpper_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(linearUpper_zv));

    btGeneric6DofSpring2Constraint_setLinearUpperLimit(
        intern->bt_constraint,
        &linearUpper_ptr->data
    );
}

/**
 * Bullet\Generic6DofSpring2Constraint::setAngularLowerLimit
 */
PHP_METHOD(Bullet_Generic6DofSpring2Constraint, setAngularLowerLimit)
{
    zval *angularLower_zv;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "O",
            &angularLower_zv, phpglfw_get_math_vec3_ce()) == FAILURE)
    {
        return;
    }

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    phpglfw_math_vec3_object *angularLower_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(angularLower_zv));

    btGeneric6DofSpring2Constraint_setAngularLowerLimit(
        intern->bt_constraint,
        &angularLower_ptr->data
    );
}

/**
 * Bullet\Generic6DofSpring2Constraint::setAngularLowerLimitReversed
 */
PHP_METHOD(Bullet_Generic6DofSpring2Constraint, setAngularLowerLimitReversed)
{
    zval *angularLower_zv;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "O",
            &angularLower_zv, phpglfw_get_math_vec3_ce()) == FAILURE)
    {
        return;
    }

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    phpglfw_math_vec3_object *angularLower_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(angularLower_zv));

    btGeneric6DofSpring2Constraint_setAngularLowerLimitReversed(
        intern->bt_constraint,
        &angularLower_ptr->data
    );
}

/**
 * Bullet\Generic6DofSpring2Constraint::setAngularUpperLimit
 */
PHP_METHOD(Bullet_Generic6DofSpring2Constraint, setAngularUpperLimit)
{
    zval *angularUpper_zv;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "O",
            &angularUpper_zv, phpglfw_get_math_vec3_ce()) == FAILURE)
    {
        return;
    }

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    phpglfw_math_vec3_object *angularUpper_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(angularUpper_zv));

    btGeneric6DofSpring2Constraint_setAngularUpperLimit(
        intern->bt_constraint,
        &angularUpper_ptr->data
    );
}

/**
 * Bullet\Generic6DofSpring2Constraint::setAngularUpperLimitReversed
 */
PHP_METHOD(Bullet_Generic6DofSpring2Constraint, setAngularUpperLimitReversed)
{
    zval *angularUpper_zv;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "O",
            &angularUpper_zv, phpglfw_get_math_vec3_ce()) == FAILURE)
    {
        return;
    }

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    phpglfw_math_vec3_object *angularUpper_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(angularUpper_zv));

    btGeneric6DofSpring2Constraint_setAngularUpperLimitReversed(
        intern->bt_constraint,
        &angularUpper_ptr->data
    );
}

/**
 * Bullet\Generic6DofSpring2Constraint::setLimit
 */
PHP_METHOD(Bullet_Generic6DofSpring2Constraint, setLimit)
{
    zend_long axis;
    double lo, hi;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "ldd",
            &axis,
            &lo,
            &hi) == FAILURE)
    {
        return;
    }

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    btGeneric6DofSpring2Constraint_setLimit(
        intern->bt_constraint,
        axis,
        lo,
        hi
    );
}

/**
 * Bullet\Generic6DofSpring2Constraint::setLimitReversed
 */
PHP_METHOD(Bullet_Generic6DofSpring2Constraint, setLimitReversed)
{
    zend_long axis;
    double lo, hi;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "ldd",
            &axis,
            &lo,
            &hi) == FAILURE)
    {
        return;
    }

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    btGeneric6DofSpring2Constraint_setLimitReversed(
        intern->bt_constraint,
        axis,
        lo,
        hi
    );
}

PHP_METHOD(Bullet_Generic6DofSpring2Constraint, setAxis)
{
    zval *axis1_zv, *axis2_zv;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "OO",
            &axis1_zv, phpglfw_get_math_vec3_ce(),
            &axis2_zv, phpglfw_get_math_vec3_ce()) == FAILURE)
    {
        return;
    }

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    phpglfw_math_vec3_object *axis1_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(axis1_zv));
    phpglfw_math_vec3_object *axis2_ptr = phpglfw_math_vec3_objectptr_from_zobj_p(Z_OBJ_P(axis2_zv));

    btGeneric6DofSpring2Constraint_setAxis(
        intern->bt_constraint,
        &axis1_ptr->data,
        &axis2_ptr->data
    );
}

PHP_METHOD(Bullet_Generic6DofSpring2Constraint, setBounce)
{
    zend_long index;
    double bounce;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "ld",
            &index,
            &bounce) == FAILURE)
    {
        return;
    }

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    btGeneric6DofSpring2Constraint_setBounce(
        intern->bt_constraint,
        index,
        bounce
    );
}

PHP_METHOD(Bullet_Generic6DofSpring2Constraint, enableMotor)
{
    zend_long index;
    zend_bool onOff;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "lb",
            &index,
            &onOff) == FAILURE)
    {
        return;
    }

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    btGeneric6DofSpring2Constraint_enableMotor(
        intern->bt_constraint,
        index,
        onOff
    );
}

PHP_METHOD(Bullet_Generic6DofSpring2Constraint, setServo)
{
    zend_long index;
    zend_bool onOff;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "lb",
            &index,
            &onOff) == FAILURE)
    {
        return;
    }

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    btGeneric6DofSpring2Constraint_setServo(
        intern->bt_constraint,
        index,
        onOff
    );
}

PHP_METHOD(Bullet_Generic6DofSpring2Constraint, setTargetVelocity)
{
    zend_long index;
    double velocity;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "ld",
            &index,
            &velocity) == FAILURE)
    {
        return;
    }

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    btGeneric6DofSpring2Constraint_setTargetVelocity(
        intern->bt_constraint,
        index,
        velocity
    );
}

PHP_METHOD(Bullet_Generic6DofSpring2Constraint, setServoTarget)
{
    zend_long index;
    double target;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "ld",
            &index,
            &target) == FAILURE)
    {
        return;
    }

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    btGeneric6DofSpring2Constraint_setServoTarget(
        intern->bt_constraint,
        index,
        target
    );
}

PHP_METHOD(Bullet_Generic6DofSpring2Constraint, setMaxMotorForce)
{
    zend_long index;
    double force;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "ld",
            &index,
            &force) == FAILURE)
    {
        return;
    }

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    btGeneric6DofSpring2Constraint_setMaxMotorForce(
        intern->bt_constraint,
        index,
        force
    );
}

PHP_METHOD(Bullet_Generic6DofSpring2Constraint, enableSpring)
{
    zend_long index;
    zend_bool onOff;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "lb",
            &index,
            &onOff) == FAILURE)
    {
        return;
    }

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    btGeneric6DofSpring2Constraint_enableSpring(
        intern->bt_constraint,
        index,
        onOff
    );
}

PHP_METHOD(Bullet_Generic6DofSpring2Constraint, setStiffness)
{
    zend_long index;
    double stiffness;
    zend_bool limitIfNeeded;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "ld|b",
            &index,
            &stiffness,
            &limitIfNeeded) == FAILURE)
    {
        return;
    }

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    btGeneric6DofSpring2Constraint_setStiffness(
        intern->bt_constraint,
        index,
        stiffness,
        limitIfNeeded
    );
}

PHP_METHOD(Bullet_Generic6DofSpring2Constraint, setDamping)
{
    zend_long index;
    double damping;
    zend_bool limitIfNeeded;

    if (zend_parse_parameters(ZEND_NUM_ARGS(), "ld|b",
            &index,
            &damping,
            &limitIfNeeded) == FAILURE)
    {
        return;
    }

    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    btGeneric6DofSpring2Constraint_setDamping(
        intern->bt_constraint,
        index,
        damping,
        limitIfNeeded
    );
}

PHP_METHOD(Bullet_Generic6DofSpring2Constraint, setEquilibriumPoint)
{
    phpbullet3_constraint_wrapper_object *intern = phpbullet3_constraint_from_zobj_p(Z_OBJ_P(getThis()));

    btGeneric6DofSpring2Constraint_setEquilibriumPoint(
        intern->bt_constraint
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
    PHPBULLET_DEFINE_CONSTRAINT_REGISTER(Bullet_SliderConstraint, slider);
    PHPBULLET_DEFINE_CONSTRAINT_REGISTER(Bullet_Generic6DofSpringConstraint, generic6dofspring);
    PHPBULLET_DEFINE_CONSTRAINT_REGISTER(Bullet_Generic6DofSpring2Constraint, generic6dofspring2);
}