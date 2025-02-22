/* This is a generated file, edit the .stub.php file instead.
 * Stub hash: 688cad90715e2e23ab5ff89be64e8da876061fb7 */

ZEND_BEGIN_ARG_WITH_RETURN_TYPE_INFO_EX(arginfo_Bullet_bullet3_test, 0, 0, IS_VOID, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_WITH_RETURN_TYPE_INFO_EX(arginfo_class_Bullet_World_setGravity, 0, 1, IS_VOID, 0)
	ZEND_ARG_OBJ_INFO(0, gravity, GL\\Math\\Vec3, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_WITH_RETURN_OBJ_INFO_EX(arginfo_class_Bullet_World_getGravity, 0, 0, GL\\Math\\Vec3, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_WITH_RETURN_TYPE_INFO_EX(arginfo_class_Bullet_World_addRigidBody, 0, 1, IS_VOID, 0)
	ZEND_ARG_OBJ_INFO(0, rigidBody, Bullet\\RigidBody, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_WITH_RETURN_TYPE_INFO_EX(arginfo_class_Bullet_World_addConstraint, 0, 1, IS_VOID, 0)
	ZEND_ARG_OBJ_INFO(0, constraint, Bullet\\Constraint, 0)
	ZEND_ARG_TYPE_INFO_WITH_DEFAULT_VALUE(0, noCollide, _IS_BOOL, 0, "false")
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_WITH_RETURN_TYPE_INFO_EX(arginfo_class_Bullet_World_removeConstraint, 0, 1, IS_VOID, 0)
	ZEND_ARG_OBJ_INFO(0, constraint, Bullet\\Constraint, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_WITH_RETURN_TYPE_INFO_EX(arginfo_class_Bullet_World_stepSimulation, 0, 1, IS_VOID, 0)
	ZEND_ARG_TYPE_INFO(0, timeStep, IS_DOUBLE, 0)
	ZEND_ARG_TYPE_INFO_WITH_DEFAULT_VALUE(0, maxSubSteps, IS_LONG, 0, "1")
	ZEND_ARG_TYPE_INFO_WITH_DEFAULT_VALUE(0, fixedTimeStep, IS_DOUBLE, 0, "0.01666666")
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_INFO_EX(arginfo_class_Bullet_SphereShape___construct, 0, 0, 1)
	ZEND_ARG_TYPE_INFO(0, radius, IS_DOUBLE, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_INFO_EX(arginfo_class_Bullet_BoxShape___construct, 0, 0, 1)
	ZEND_ARG_OBJ_INFO(0, halfExtents, GL\\Math\\Vec3, 0)
ZEND_END_ARG_INFO()

#define arginfo_class_Bullet_CylinderShape___construct arginfo_class_Bullet_BoxShape___construct

#define arginfo_class_Bullet_CylinderShapeX___construct arginfo_class_Bullet_BoxShape___construct

#define arginfo_class_Bullet_CylinderShapeZ___construct arginfo_class_Bullet_BoxShape___construct

ZEND_BEGIN_ARG_INFO_EX(arginfo_class_Bullet_StaticPlaneShape___construct, 0, 0, 1)
	ZEND_ARG_OBJ_INFO(0, normal, GL\\Math\\Vec3, 0)
	ZEND_ARG_TYPE_INFO_WITH_DEFAULT_VALUE(0, constant, IS_DOUBLE, 0, "0.0")
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_INFO_EX(arginfo_class_Bullet_Point2PointConstraint___construct, 0, 0, 4)
	ZEND_ARG_OBJ_INFO(0, bodyA, Bullet\\RigidBody, 0)
	ZEND_ARG_OBJ_INFO(0, bodyB, Bullet\\RigidBody, 0)
	ZEND_ARG_OBJ_INFO(0, pivotInA, GL\\Math\\Vec3, 0)
	ZEND_ARG_OBJ_INFO(0, pivotInB, GL\\Math\\Vec3, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_INFO_EX(arginfo_class_Bullet_HingeConstraint___construct, 0, 0, 6)
	ZEND_ARG_OBJ_INFO(0, bodyA, Bullet\\RigidBody, 0)
	ZEND_ARG_OBJ_INFO(0, bodyB, Bullet\\RigidBody, 0)
	ZEND_ARG_OBJ_INFO(0, pivotInA, GL\\Math\\Vec3, 0)
	ZEND_ARG_OBJ_INFO(0, pivotInB, GL\\Math\\Vec3, 0)
	ZEND_ARG_OBJ_INFO(0, axisInA, GL\\Math\\Vec3, 0)
	ZEND_ARG_OBJ_INFO(0, axisInB, GL\\Math\\Vec3, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_INFO_EX(arginfo_class_Bullet_RigidBody___construct, 0, 0, 1)
	ZEND_ARG_OBJ_INFO(0, collisionShape, Bullet\\CollisionShape, 0)
	ZEND_ARG_TYPE_INFO_WITH_DEFAULT_VALUE(0, mass, IS_DOUBLE, 0, "0.0")
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_WITH_RETURN_TYPE_INFO_EX(arginfo_class_Bullet_RigidBody_setPosition, 0, 1, IS_VOID, 0)
	ZEND_ARG_OBJ_INFO(0, position, GL\\Math\\Vec3, 0)
ZEND_END_ARG_INFO()

#define arginfo_class_Bullet_RigidBody_getPosition arginfo_class_Bullet_World_getGravity

#define arginfo_class_Bullet_RigidBody_getLinearVelocity arginfo_class_Bullet_World_getGravity

#define arginfo_class_Bullet_RigidBody_getAngularVelocity arginfo_class_Bullet_World_getGravity

ZEND_BEGIN_ARG_WITH_RETURN_OBJ_INFO_EX(arginfo_class_Bullet_RigidBody_getOrientation, 0, 0, GL\\Math\\Quat, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_WITH_RETURN_TYPE_INFO_EX(arginfo_class_Bullet_RigidBody_setOrientation, 0, 1, IS_VOID, 0)
	ZEND_ARG_OBJ_INFO(0, orientation, GL\\Math\\Quat, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_WITH_RETURN_TYPE_INFO_EX(arginfo_class_Bullet_RigidBody_setMass, 0, 1, IS_VOID, 0)
	ZEND_ARG_TYPE_INFO(0, mass, IS_DOUBLE, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_WITH_RETURN_OBJ_INFO_EX(arginfo_class_Bullet_RigidBody_getTransform, 0, 0, GL\\Math\\Mat4, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_WITH_RETURN_TYPE_INFO_EX(arginfo_class_Bullet_RigidBody_applyForce, 0, 2, IS_VOID, 0)
	ZEND_ARG_OBJ_INFO(0, force, GL\\Math\\Vec3, 0)
	ZEND_ARG_OBJ_INFO(0, relPos, GL\\Math\\Vec3, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_WITH_RETURN_TYPE_INFO_EX(arginfo_class_Bullet_RigidBody_applyCentralForce, 0, 1, IS_VOID, 0)
	ZEND_ARG_OBJ_INFO(0, force, GL\\Math\\Vec3, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_WITH_RETURN_TYPE_INFO_EX(arginfo_class_Bullet_RigidBody_applyTorque, 0, 1, IS_VOID, 0)
	ZEND_ARG_OBJ_INFO(0, torque, GL\\Math\\Vec3, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_WITH_RETURN_TYPE_INFO_EX(arginfo_class_Bullet_RigidBody_applyImpulse, 0, 2, IS_VOID, 0)
	ZEND_ARG_OBJ_INFO(0, impulse, GL\\Math\\Vec3, 0)
	ZEND_ARG_OBJ_INFO(0, relPos, GL\\Math\\Vec3, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_WITH_RETURN_TYPE_INFO_EX(arginfo_class_Bullet_RigidBody_applyCentralImpulse, 0, 1, IS_VOID, 0)
	ZEND_ARG_OBJ_INFO(0, impulse, GL\\Math\\Vec3, 0)
ZEND_END_ARG_INFO()

#define arginfo_class_Bullet_RigidBody_applyTorqueImpulse arginfo_class_Bullet_RigidBody_applyTorque

#define arginfo_class_Bullet_RigidBody_activate arginfo_Bullet_bullet3_test


ZEND_FUNCTION(Bullet_bullet3_test);
ZEND_METHOD(Bullet_World, setGravity);
ZEND_METHOD(Bullet_World, getGravity);
ZEND_METHOD(Bullet_World, addRigidBody);
ZEND_METHOD(Bullet_World, addConstraint);
ZEND_METHOD(Bullet_World, removeConstraint);
ZEND_METHOD(Bullet_World, stepSimulation);
ZEND_METHOD(Bullet_SphereShape, __construct);
ZEND_METHOD(Bullet_BoxShape, __construct);
ZEND_METHOD(Bullet_CylinderShape, __construct);
ZEND_METHOD(Bullet_CylinderShapeX, __construct);
ZEND_METHOD(Bullet_CylinderShapeZ, __construct);
ZEND_METHOD(Bullet_StaticPlaneShape, __construct);
ZEND_METHOD(Bullet_Point2PointConstraint, __construct);
ZEND_METHOD(Bullet_HingeConstraint, __construct);
ZEND_METHOD(Bullet_RigidBody, __construct);
ZEND_METHOD(Bullet_RigidBody, setPosition);
ZEND_METHOD(Bullet_RigidBody, getPosition);
ZEND_METHOD(Bullet_RigidBody, getLinearVelocity);
ZEND_METHOD(Bullet_RigidBody, getAngularVelocity);
ZEND_METHOD(Bullet_RigidBody, getOrientation);
ZEND_METHOD(Bullet_RigidBody, setOrientation);
ZEND_METHOD(Bullet_RigidBody, setMass);
ZEND_METHOD(Bullet_RigidBody, getTransform);
ZEND_METHOD(Bullet_RigidBody, applyForce);
ZEND_METHOD(Bullet_RigidBody, applyCentralForce);
ZEND_METHOD(Bullet_RigidBody, applyTorque);
ZEND_METHOD(Bullet_RigidBody, applyImpulse);
ZEND_METHOD(Bullet_RigidBody, applyCentralImpulse);
ZEND_METHOD(Bullet_RigidBody, applyTorqueImpulse);
ZEND_METHOD(Bullet_RigidBody, activate);


static const zend_function_entry ext_functions[] = {
	ZEND_NS_FALIAS("Bullet", bullet3_test, Bullet_bullet3_test, arginfo_Bullet_bullet3_test)
	ZEND_FE_END
};


static const zend_function_entry class_Bullet_World_methods[] = {
	ZEND_ME(Bullet_World, setGravity, arginfo_class_Bullet_World_setGravity, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_World, getGravity, arginfo_class_Bullet_World_getGravity, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_World, addRigidBody, arginfo_class_Bullet_World_addRigidBody, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_World, addConstraint, arginfo_class_Bullet_World_addConstraint, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_World, removeConstraint, arginfo_class_Bullet_World_removeConstraint, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_World, stepSimulation, arginfo_class_Bullet_World_stepSimulation, ZEND_ACC_PUBLIC)
	ZEND_FE_END
};


static const zend_function_entry class_Bullet_CollisionShape_methods[] = {
	ZEND_FE_END
};


static const zend_function_entry class_Bullet_SphereShape_methods[] = {
	ZEND_ME(Bullet_SphereShape, __construct, arginfo_class_Bullet_SphereShape___construct, ZEND_ACC_PUBLIC)
	ZEND_FE_END
};


static const zend_function_entry class_Bullet_BoxShape_methods[] = {
	ZEND_ME(Bullet_BoxShape, __construct, arginfo_class_Bullet_BoxShape___construct, ZEND_ACC_PUBLIC)
	ZEND_FE_END
};


static const zend_function_entry class_Bullet_CylinderShape_methods[] = {
	ZEND_ME(Bullet_CylinderShape, __construct, arginfo_class_Bullet_CylinderShape___construct, ZEND_ACC_PUBLIC)
	ZEND_FE_END
};


static const zend_function_entry class_Bullet_CylinderShapeX_methods[] = {
	ZEND_ME(Bullet_CylinderShapeX, __construct, arginfo_class_Bullet_CylinderShapeX___construct, ZEND_ACC_PUBLIC)
	ZEND_FE_END
};


static const zend_function_entry class_Bullet_CylinderShapeZ_methods[] = {
	ZEND_ME(Bullet_CylinderShapeZ, __construct, arginfo_class_Bullet_CylinderShapeZ___construct, ZEND_ACC_PUBLIC)
	ZEND_FE_END
};


static const zend_function_entry class_Bullet_StaticPlaneShape_methods[] = {
	ZEND_ME(Bullet_StaticPlaneShape, __construct, arginfo_class_Bullet_StaticPlaneShape___construct, ZEND_ACC_PUBLIC)
	ZEND_FE_END
};


static const zend_function_entry class_Bullet_Constraint_methods[] = {
	ZEND_FE_END
};


static const zend_function_entry class_Bullet_Point2PointConstraint_methods[] = {
	ZEND_ME(Bullet_Point2PointConstraint, __construct, arginfo_class_Bullet_Point2PointConstraint___construct, ZEND_ACC_PUBLIC)
	ZEND_FE_END
};


static const zend_function_entry class_Bullet_HingeConstraint_methods[] = {
	ZEND_ME(Bullet_HingeConstraint, __construct, arginfo_class_Bullet_HingeConstraint___construct, ZEND_ACC_PUBLIC)
	ZEND_FE_END
};


static const zend_function_entry class_Bullet_RigidBody_methods[] = {
	ZEND_ME(Bullet_RigidBody, __construct, arginfo_class_Bullet_RigidBody___construct, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_RigidBody, setPosition, arginfo_class_Bullet_RigidBody_setPosition, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_RigidBody, getPosition, arginfo_class_Bullet_RigidBody_getPosition, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_RigidBody, getLinearVelocity, arginfo_class_Bullet_RigidBody_getLinearVelocity, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_RigidBody, getAngularVelocity, arginfo_class_Bullet_RigidBody_getAngularVelocity, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_RigidBody, getOrientation, arginfo_class_Bullet_RigidBody_getOrientation, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_RigidBody, setOrientation, arginfo_class_Bullet_RigidBody_setOrientation, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_RigidBody, setMass, arginfo_class_Bullet_RigidBody_setMass, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_RigidBody, getTransform, arginfo_class_Bullet_RigidBody_getTransform, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_RigidBody, applyForce, arginfo_class_Bullet_RigidBody_applyForce, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_RigidBody, applyCentralForce, arginfo_class_Bullet_RigidBody_applyCentralForce, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_RigidBody, applyTorque, arginfo_class_Bullet_RigidBody_applyTorque, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_RigidBody, applyImpulse, arginfo_class_Bullet_RigidBody_applyImpulse, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_RigidBody, applyCentralImpulse, arginfo_class_Bullet_RigidBody_applyCentralImpulse, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_RigidBody, applyTorqueImpulse, arginfo_class_Bullet_RigidBody_applyTorqueImpulse, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_RigidBody, activate, arginfo_class_Bullet_RigidBody_activate, ZEND_ACC_PUBLIC)
	ZEND_FE_END
};

static zend_class_entry *register_class_Bullet_World(void)
{
	zend_class_entry ce, *class_entry;

	INIT_NS_CLASS_ENTRY(ce, "Bullet", "World", class_Bullet_World_methods);
	class_entry = zend_register_internal_class_ex(&ce, NULL);

	return class_entry;
}

static zend_class_entry *register_class_Bullet_CollisionShape(void)
{
	zend_class_entry ce, *class_entry;

	INIT_NS_CLASS_ENTRY(ce, "Bullet", "CollisionShape", class_Bullet_CollisionShape_methods);
	class_entry = zend_register_internal_interface(&ce);

	return class_entry;
}

static zend_class_entry *register_class_Bullet_SphereShape(zend_class_entry *class_entry_Bullet_CollisionShape)
{
	zend_class_entry ce, *class_entry;

	INIT_NS_CLASS_ENTRY(ce, "Bullet", "SphereShape", class_Bullet_SphereShape_methods);
	class_entry = zend_register_internal_class_ex(&ce, NULL);
	zend_class_implements(class_entry, 1, class_entry_Bullet_CollisionShape);

	return class_entry;
}

static zend_class_entry *register_class_Bullet_BoxShape(zend_class_entry *class_entry_Bullet_CollisionShape)
{
	zend_class_entry ce, *class_entry;

	INIT_NS_CLASS_ENTRY(ce, "Bullet", "BoxShape", class_Bullet_BoxShape_methods);
	class_entry = zend_register_internal_class_ex(&ce, NULL);
	zend_class_implements(class_entry, 1, class_entry_Bullet_CollisionShape);

	return class_entry;
}

static zend_class_entry *register_class_Bullet_CylinderShape(zend_class_entry *class_entry_Bullet_CollisionShape)
{
	zend_class_entry ce, *class_entry;

	INIT_NS_CLASS_ENTRY(ce, "Bullet", "CylinderShape", class_Bullet_CylinderShape_methods);
	class_entry = zend_register_internal_class_ex(&ce, NULL);
	zend_class_implements(class_entry, 1, class_entry_Bullet_CollisionShape);

	return class_entry;
}

static zend_class_entry *register_class_Bullet_CylinderShapeX(zend_class_entry *class_entry_Bullet_CylinderShape)
{
	zend_class_entry ce, *class_entry;

	INIT_NS_CLASS_ENTRY(ce, "Bullet", "CylinderShapeX", class_Bullet_CylinderShapeX_methods);
	class_entry = zend_register_internal_class_ex(&ce, NULL);
	zend_class_implements(class_entry, 1, class_entry_Bullet_CylinderShape);

	return class_entry;
}

static zend_class_entry *register_class_Bullet_CylinderShapeZ(zend_class_entry *class_entry_Bullet_CylinderShape)
{
	zend_class_entry ce, *class_entry;

	INIT_NS_CLASS_ENTRY(ce, "Bullet", "CylinderShapeZ", class_Bullet_CylinderShapeZ_methods);
	class_entry = zend_register_internal_class_ex(&ce, NULL);
	zend_class_implements(class_entry, 1, class_entry_Bullet_CylinderShape);

	return class_entry;
}

static zend_class_entry *register_class_Bullet_StaticPlaneShape(zend_class_entry *class_entry_Bullet_CollisionShape)
{
	zend_class_entry ce, *class_entry;

	INIT_NS_CLASS_ENTRY(ce, "Bullet", "StaticPlaneShape", class_Bullet_StaticPlaneShape_methods);
	class_entry = zend_register_internal_class_ex(&ce, NULL);
	zend_class_implements(class_entry, 1, class_entry_Bullet_CollisionShape);

	return class_entry;
}

static zend_class_entry *register_class_Bullet_Constraint(void)
{
	zend_class_entry ce, *class_entry;

	INIT_NS_CLASS_ENTRY(ce, "Bullet", "Constraint", class_Bullet_Constraint_methods);
	class_entry = zend_register_internal_interface(&ce);

	return class_entry;
}

static zend_class_entry *register_class_Bullet_Point2PointConstraint(zend_class_entry *class_entry_Bullet_Constraint)
{
	zend_class_entry ce, *class_entry;

	INIT_NS_CLASS_ENTRY(ce, "Bullet", "Point2PointConstraint", class_Bullet_Point2PointConstraint_methods);
	class_entry = zend_register_internal_class_ex(&ce, NULL);
	zend_class_implements(class_entry, 1, class_entry_Bullet_Constraint);

	return class_entry;
}

static zend_class_entry *register_class_Bullet_HingeConstraint(zend_class_entry *class_entry_Bullet_Constraint)
{
	zend_class_entry ce, *class_entry;

	INIT_NS_CLASS_ENTRY(ce, "Bullet", "HingeConstraint", class_Bullet_HingeConstraint_methods);
	class_entry = zend_register_internal_class_ex(&ce, NULL);
	zend_class_implements(class_entry, 1, class_entry_Bullet_Constraint);

	return class_entry;
}

static zend_class_entry *register_class_Bullet_RigidBody(void)
{
	zend_class_entry ce, *class_entry;

	INIT_NS_CLASS_ENTRY(ce, "Bullet", "RigidBody", class_Bullet_RigidBody_methods);
	class_entry = zend_register_internal_class_ex(&ce, NULL);

	zval property_collisionShape_default_value;
	ZVAL_UNDEF(&property_collisionShape_default_value);
	zend_string *property_collisionShape_name = zend_string_init("collisionShape", sizeof("collisionShape") - 1, 1);
	zend_string *property_collisionShape_class_Bullet_CollisionShape = zend_string_init("Bullet\\CollisionShape", sizeof("Bullet\\CollisionShape")-1, 1);
	zend_declare_typed_property(class_entry, property_collisionShape_name, &property_collisionShape_default_value, ZEND_ACC_PUBLIC, NULL, (zend_type) ZEND_TYPE_INIT_CLASS(property_collisionShape_class_Bullet_CollisionShape, 0, 0));
	zend_string_release(property_collisionShape_name);

	return class_entry;
}
