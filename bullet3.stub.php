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

class RigidBody
{
    public function setPosition(\GL\Math\Vec3 $position): void {}
    public function getPosition(): \GL\Math\Vec3 {}
    public function getOrientation(): \GL\Math\Quat {}
    public function setMass(float $mass): void {}
    public function getTransform(): \GL\Math\Mat4 {}
}

function bullet3_test(): void {}

