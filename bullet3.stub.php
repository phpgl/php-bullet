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

class StaticPlaneShape implements CollisionShape
{
    public function __construct(\GL\Math\Vec3 $normal, float $constant = 0.0) {}
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
    public function setMass(float $mass): void {}
    public function getTransform(): \GL\Math\Mat4 {}
}

function bullet3_test(): void {}

