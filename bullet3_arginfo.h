/* This is a generated file, edit the .stub.php file instead.
 * Stub hash: c0d5fe61bff4e0c03157a18ab2b929f8c137799d */

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

ZEND_BEGIN_ARG_WITH_RETURN_TYPE_INFO_EX(arginfo_class_Bullet_World_stepSimulation, 0, 1, IS_VOID, 0)
	ZEND_ARG_TYPE_INFO(0, timeStep, IS_DOUBLE, 0)
	ZEND_ARG_TYPE_INFO_WITH_DEFAULT_VALUE(0, maxSubSteps, IS_LONG, 0, "1")
	ZEND_ARG_TYPE_INFO_WITH_DEFAULT_VALUE(0, fixedTimeStep, IS_DOUBLE, 0, "0.01666666")
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_WITH_RETURN_TYPE_INFO_EX(arginfo_class_Bullet_RigidBody_setPosition, 0, 1, IS_VOID, 0)
	ZEND_ARG_OBJ_INFO(0, position, GL\\Math\\Vec3, 0)
ZEND_END_ARG_INFO()

#define arginfo_class_Bullet_RigidBody_getPosition arginfo_class_Bullet_World_getGravity

ZEND_BEGIN_ARG_WITH_RETURN_OBJ_INFO_EX(arginfo_class_Bullet_RigidBody_getOrientation, 0, 0, GL\\Math\\Quat, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_WITH_RETURN_TYPE_INFO_EX(arginfo_class_Bullet_RigidBody_setMass, 0, 1, IS_VOID, 0)
	ZEND_ARG_TYPE_INFO(0, mass, IS_DOUBLE, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_WITH_RETURN_OBJ_INFO_EX(arginfo_class_Bullet_RigidBody_getTransform, 0, 0, GL\\Math\\Mat4, 0)
ZEND_END_ARG_INFO()


ZEND_FUNCTION(Bullet_bullet3_test);
ZEND_METHOD(Bullet_World, setGravity);
ZEND_METHOD(Bullet_World, getGravity);
ZEND_METHOD(Bullet_World, addRigidBody);
ZEND_METHOD(Bullet_World, stepSimulation);
ZEND_METHOD(Bullet_RigidBody, setPosition);
ZEND_METHOD(Bullet_RigidBody, getPosition);
ZEND_METHOD(Bullet_RigidBody, getOrientation);
ZEND_METHOD(Bullet_RigidBody, setMass);
ZEND_METHOD(Bullet_RigidBody, getTransform);


static const zend_function_entry ext_functions[] = {
	ZEND_NS_FALIAS("Bullet", bullet3_test, Bullet_bullet3_test, arginfo_Bullet_bullet3_test)
	ZEND_FE_END
};


static const zend_function_entry class_Bullet_World_methods[] = {
	ZEND_ME(Bullet_World, setGravity, arginfo_class_Bullet_World_setGravity, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_World, getGravity, arginfo_class_Bullet_World_getGravity, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_World, addRigidBody, arginfo_class_Bullet_World_addRigidBody, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_World, stepSimulation, arginfo_class_Bullet_World_stepSimulation, ZEND_ACC_PUBLIC)
	ZEND_FE_END
};


static const zend_function_entry class_Bullet_RigidBody_methods[] = {
	ZEND_ME(Bullet_RigidBody, setPosition, arginfo_class_Bullet_RigidBody_setPosition, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_RigidBody, getPosition, arginfo_class_Bullet_RigidBody_getPosition, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_RigidBody, getOrientation, arginfo_class_Bullet_RigidBody_getOrientation, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_RigidBody, setMass, arginfo_class_Bullet_RigidBody_setMass, ZEND_ACC_PUBLIC)
	ZEND_ME(Bullet_RigidBody, getTransform, arginfo_class_Bullet_RigidBody_getTransform, ZEND_ACC_PUBLIC)
	ZEND_FE_END
};

static zend_class_entry *register_class_Bullet_World(void)
{
	zend_class_entry ce, *class_entry;

	INIT_NS_CLASS_ENTRY(ce, "Bullet", "World", class_Bullet_World_methods);
	class_entry = zend_register_internal_class_ex(&ce, NULL);

	return class_entry;
}

static zend_class_entry *register_class_Bullet_RigidBody(void)
{
	zend_class_entry ce, *class_entry;

	INIT_NS_CLASS_ENTRY(ce, "Bullet", "RigidBody", class_Bullet_RigidBody_methods);
	class_entry = zend_register_internal_class_ex(&ce, NULL);

	return class_entry;
}
