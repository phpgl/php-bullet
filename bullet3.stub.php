<?php

/**
 * @generate-class-entries
 * @undocumentable
 */

namespace Bullet;

class World
{
    public function setGravity(\GL\Math\Vec3 $gravity): void {}
    public function getGravity(): \GL\Math\Vec3 {}

    public function addRigidBody(RigidBody $rigidBody): void {}

    public function addConstraint(Constraint $constraint, bool $noCollide = false): void {}
    public function removeConstraint(Constraint $constraint): void {}
    
    public function stepSimulation(float $timeStep, int $maxSubSteps = 1, float $fixedTimeStep = 0.01666666): void {}
}

interface CollisionShape
{
}

class SphereShape implements CollisionShape
{
    public function __construct(float $radius) {}
}

class BoxShape implements CollisionShape
{
    public function __construct(\GL\Math\Vec3 $halfExtents) {}
}

class CylinderShape implements CollisionShape
{
    public function __construct(\GL\Math\Vec3 $halfExtents) {}
}

class CylinderShapeX implements CollisionShape
{
    public function __construct(\GL\Math\Vec3 $halfExtents) {}
}

class CylinderShapeZ implements CollisionShape
{
    public function __construct(\GL\Math\Vec3 $halfExtents) {}
}

class StaticPlaneShape implements CollisionShape
{
    public function __construct(\GL\Math\Vec3 $normal, float $constant = 0.0) {}
}

interface Constraint
{
}

class Point2PointConstraint implements Constraint
{
    public function __construct(
        RigidBody $bodyA,
        RigidBody $bodyB,
        \GL\Math\Vec3 $pivotInA,
        \GL\Math\Vec3 $pivotInB
    ) {}
}

class HingeConstraint implements Constraint
{
    public function __construct(
        RigidBody $bodyA,
        RigidBody $bodyB,
        \GL\Math\Vec3 $pivotInA,
        \GL\Math\Vec3 $pivotInB,
        \GL\Math\Vec3 $axisInA,
        \GL\Math\Vec3 $axisInB
    ) {}

    public function setLimit(float $low, float $high, float $softness = 0.9, float $biasFactor = 0.3, float $relaxationFactor = 1.0): void {}
}

class SliderConstraint implements Constraint
{
    public function __construct(
        RigidBody $bodyA,
        RigidBody $bodyB,
        \GL\Math\Mat4 $frameInA,
        \GL\Math\Mat4 $frameInB,
        bool $useLinearReferenceFrameA = true
    ) {}
}

class Generic6DofSpringConstraint implements Constraint
{
    public function __construct(
        RigidBody $bodyA,
        RigidBody $bodyB,
        \GL\Math\Mat4 $frameInA,
        \GL\Math\Mat4 $frameInB,
        bool $useLinearReferenceFrameA = true
    ) {}
}

class Generic6DofSpring2Constraint implements Constraint
{
    public function __construct(
        RigidBody $bodyA,
        RigidBody $bodyB,
        \GL\Math\Mat4 $frameInA,
        \GL\Math\Mat4 $frameInB
    ) {}


    public function setFrames(\GL\Math\Mat4 $frameInA, \GL\Math\Mat4 $frameInB): void {}
    public function setLinearLowerLimit(\GL\Math\Vec3 $linearLower): void {}
    
    public function setLinearUpperLimit(\GL\Math\Vec3 $linearUpper): void {}
    
    public function setAngularLowerLimit(\GL\Math\Vec3 $angularLower): void {}
    public function setAngularLowerLimitReversed(\GL\Math\Vec3 $angularLower): void {}

    public function setAngularUpperLimit(\GL\Math\Vec3 $angularUpper): void {}
    public function setAngularUpperLimitReversed(\GL\Math\Vec3 $angularUpper): void {}

    public function setLimit(int $axis, float $lo, float $hi): void {}
    public function setLimitReversed(int $axis, float $lo, float $hi): void {}

    public function setAxis(int $index, \GL\Math\Vec3 $axis): void {}

    public function setBounce(int $index, float $bounce): void {}

    public function enableMotor(int $index, bool $onOff): void {}
    public function setServo(int $index, bool $onOff): void {}
    public function setTargetVelocity(int $index, float $velocity): void {}
    public function setServoTarget(int $index, float $target): void {}
    public function setMaxMotorForce(int $index, float $force): void {}

    public function enableSpring(int $index, bool $onOff): void {}
    public function setStiffness(int $index, float $stiffness, bool $limitIfNeeded = true): void {}
    public function setDamping(int $index, float $damping, bool $limitIfNeeded = true): void {}
}

class RigidBody
{
    public CollisionShape $collisionShape;

    public function __construct(CollisionShape $collisionShape, float $mass = 0.0) {}

    public function setPosition(\GL\Math\Vec3 $position): void {}
    public function getPosition(): \GL\Math\Vec3 {}
    public function getLinearVelocity(): \GL\Math\Vec3 {}
    public function getAngularVelocity(): \GL\Math\Vec3 {}
    public function getOrientation(): \GL\Math\Quat {}
    public function setOrientation(\GL\Math\Quat $orientation): void {}
    public function setMass(float $mass): void {}
    public function setRestitution(float $restitution): void {}
    public function setFriction(float $friction): void {}
    public function setRollingFriction(float $rollingFriction): void {}
    public function setSpinningFriction(float $spinningFriction): void {}
    public function setContactStiffnessAndDamping(float $stiffness, float $damping): void {}

    public function getTransform(): \GL\Math\Mat4 {}
    public function applyForce(\GL\Math\Vec3 $force, \GL\Math\Vec3 $relPos): void {}
    public function applyCentralForce(\GL\Math\Vec3 $force): void {}
    public function applyTorque(\GL\Math\Vec3 $torque): void {}
    public function applyImpulse(\GL\Math\Vec3 $impulse, \GL\Math\Vec3 $relPos): void {}
    public function applyCentralImpulse(\GL\Math\Vec3 $impulse): void {}
    public function applyTorqueImpulse(\GL\Math\Vec3 $torque): void {}

    public function activate(): void {}
}

function bullet3_test(): void {}

