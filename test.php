<?php 

use GL\Math\Vec3;

$world = new Bullet\World();
$world->setGravity(new Vec3(0, -9.81, 0));

$body1 = new Bullet\RigidBody();
$body1->setMass(1);
$body1->setPosition(new Vec3(0, 10, 0));

$body2 = new Bullet\RigidBody();
$body2->setMass(1);
$body2->setPosition(new Vec3(0, 15, 0));

$world->addRigidBody($body1);
$world->addRigidBody($body2);

for ($i = 0; $i < 100; $i++) {
    $world->stepSimulation(1/60);
    echo "Body1: " . $body1->getPosition() . PHP_EOL;
    echo "Body2: " . $body2->getPosition() . PHP_EOL;
}

var_dump($world->getGravity());

// echo Bullet\bullet3_test();