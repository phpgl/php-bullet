<?php

namespace App;

use Bullet\BoxShape;
use Bullet\CylinderShape;
use Bullet\CylinderShapeX;
use Bullet\Generic6DofSpring2Constraint;
use Bullet\HingeConstraint;
use Bullet\Point2PointConstraint;
use Bullet\RigidBody;
use Bullet\SliderConstraint;
use Bullet\SphereShape;
use Bullet\StaticPlaneShape;
use Bullet\World;
use Error;
use GL\Math\GLM;
use GL\Math\Mat4;
use GL\Math\Quat;
use GL\Math\Vec2;
use GL\Math\Vec3;
use GL\VectorGraphics\{VGAlign, VGColor, VGContext};
use VISU\Component\VISULowPoly\DynamicRenderableModel;
use VISU\Graphics\{RenderTarget, Viewport, Camera, CameraProjectionMode};
use VISU\Graphics\Rendering\RenderContext;
use VISU\Geo\Transform;
use VISU\Graphics\Rendering\Pass\BackbufferData;
use VISU\Graphics\Rendering\Pass\CameraData;
use VISU\Graphics\Rendering\Renderer\Debug3DRenderer;
use VISU\Graphics\Rendering\Resource\RenderTargetResource;
use VISU\OS\{InputActionMap, Key};

use VISU\Quickstart\QuickstartApp;
use VISU\Quickstart\Render\QuickstartDebugMetricsOverlay;
use VISU\System\VISUCameraSystem;
use VISU\System\VISULowPoly\LPModelCollection;
use VISU\System\VISULowPoly\LPObjLoader;
use VISU\System\VISULowPoly\LPRenderingSystem;


define('ANGULAR_LIMIT', 1e10);

class AppExample2 extends QuickstartApp
{
    /**
     * You do not have to use a camera at all if you don't want to.
     * But for sake of this example we will use one to determine a fixed viewport.
     * This is what you would typically do in a 2D game.
     */
    private Camera $camera;

    private LPRenderingSystem $renderingSystem;
    private VISUCameraSystem $cameraSystem;
    private Debug3DRenderer $dbg3D;

    /**
     * Bullet physics world
     */
    private World $physicsWorld;

    /**
     * Car body
     */
    private RigidBody $carBodyRigidBody;

    /**
     * Car wheels
     */
    private int $frontLeftWheel;
    private int $frontRightWheel;
    private int $rearLeftWheel;
    private int $rearRightWheel;

    private float $steerAngle = 0.0;
    private const MAX_STEER   = 0.5;
    private const STEER_SPEED = 0.05; 
    private const STEER_AXIS  = 4;   // rot Y (yaw)
    private const DRIVE_AXIS  = 3;   // rot X

    /**
     * A function that is invoked once the app is ready to run.
     * This happens exactly just before the game loop starts.
     * 
     * Here you can prepare your game state, register services, callbacks etc.
     */
    public function ready() : void
    {
        parent::ready();

        // initalize dependencies
        // --------------------------------------------------------------------

        // we use a low poly rendering system in this example
        $this->renderingSystem = new LPRenderingSystem(
            $this->gl,
            $this->container->get('shaders'),
            $this->container->get('models')
        );

        // basic camera system
        $this->cameraSystem = new VISUCameraSystem($this->input, $this->dispatcher);

        // 3D debug renderer
        $this->dbg3D = new Debug3DRenderer($this->gl);
        Debug3DRenderer::setGlobalInstance($this->dbg3D);

        // initalize systems
        $this->renderingSystem->register($this->entities);
        $this->cameraSystem->register($this->entities);

        // register components
        $this->entities->registerComponent(Transform::class);
        $this->entities->registerComponent(RigidBody::class);
        $this->entities->registerComponent(Camera::class);
        $this->entities->registerComponent(Point2PointConstraint::class);
        $this->entities->registerComponent(SliderConstraint::class);
        $this->entities->registerComponent(HingeConstraint::class);
        $this->entities->registerComponent(Generic6DofSpring2Constraint::class);

        // load models
        $objectCollection = $this->container->getTyped(LPModelCollection::class, 'models');
        $objectLoader = $this->container->getTyped(LPObjLoader::class, 'models.loader');

        // load the space kit models
        // these are smaller then our unit space of 1 unit = 1 meter
        // so we scale them up by a factor of 4.0 to ~ match our environment scale
        // $objectLoader->loadAllInDirectory(VISU_PATH_RESOURCES . '/models/spacekit', $objectCollection, 4.0);
        $objectLoader->loadAllInDirectory(VISU_PATH_RESOURCES . '/models/primitives', $objectCollection, 1.0);

        // You can bind actions to keys in VISU 
        // this way you can decouple your game logic from the actual key bindings
        // and provides a comfortable way to access input state
        $actions = new InputActionMap;
        $actions->bindButton('moveForward', Key::UP);
        $actions->bindButton('moveBackward', Key::DOWN);
        $actions->bindButton('turnLeft', Key::LEFT);
        $actions->bindButton('turnRight', Key::RIGHT);

        $this->inputContext->registerAndActivate('main', $actions);

        // create a camera
        $this->camera = new Camera(CameraProjectionMode::perspective);
        $cameraEntity = $this->entities->create();
        $camera = $this->entities->attach($cameraEntity, $this->camera);
        $camera->transform->position->y = 0;
        $camera->transform->position->z = 50;
        $this->cameraSystem->setActiveCameraEntity($cameraEntity);

        // load the inconsolata font to display the current score
        if ($this->vg->createFont('inconsolata', VISU_PATH_FRAMEWORK_RESOURCES_FONT . '/inconsolata/Inconsolata-Regular.ttf') === -1) {
            throw new Error('Inconsolata font could not be loaded.');
        }

        // // spawn a turret
        // $turret = $this->entities->create();
        // $this->entities->attach($turret, new Transform());
        // $model = $this->entities->attach($turret, new DynamicRenderableModel());
        // $model->modelIdentifier = 'turret_single.obj';

        // create the physics world
        $this->physicsWorld = new World();
        $this->physicsWorld->enableDebugDrawing();
        $this->physicsWorld->setGravity(new Vec3(0, -9.81, 0));

        $shpereShape = new SphereShape(1);
        $lastRigidBody = null;
        for ($i = 0; $i < 200; $i++) 
        {
            $entity = $this->entities->create();
            $rigidbody = new RigidBody($shpereShape, 5);
            $rigidbody->setPosition(new Vec3(mt_rand(0, 100), mt_rand(0, 100), mt_rand(0, 100)));

            $model = $this->entities->attach($entity, new DynamicRenderableModel());
            $model->modelIdentifier = 'sphere.obj';

            $transform = $this->entities->attach($entity, new Transform());
            $transform->position = $rigidbody->getPosition();
            // $transform->scale = new Vec3(0.6);

            $this->physicsWorld->addRigidBody($rigidbody);
            $this->entities->attach($entity, $rigidbody);
            $lastRigidBody = $rigidbody;
        }

        // Create ground plane
        // -----------------------------------------------------------
        $ground = $this->entities->create();
        $worldUp = new Vec3(0, 1, 0);

        $groundRigidBody = new RigidBody(new StaticPlaneShape($worldUp, 0), 0); 
        $groundRigidBody->setFriction(1); // a bit sticky
        $model = $this->entities->attach($ground, new DynamicRenderableModel());
        $model->modelIdentifier = 'cube.obj';

        $transform = $this->entities->attach($ground, new Transform());
        $transform->scale = new Vec3(10000, 0.1, 10000);
        $transform->position = new Vec3(0, 0, 0);

        $this->physicsWorld->addRigidBody($groundRigidBody);
        $this->entities->attach($ground, $groundRigidBody);

        // Create a car body
        // -----------------------------------------------------------
        $carBodyShape = new BoxShape(new Vec3(2, 0.5, 4)); 
        $carBody = $this->entities->create();

        $this->carBodyRigidBody = new RigidBody($carBodyShape, 1.0);
        $this->carBodyRigidBody->setPosition(new Vec3(0, 5, 0)); 

        $model = $this->entities->attach($carBody, new DynamicRenderableModel());
        $model->modelIdentifier = 'cube.obj';

        $transform = $this->entities->attach($carBody, new Transform());
        $transform->position = $this->carBodyRigidBody->getPosition();
        $transform->scale = new Vec3(2, 0.5, 4);

        $this->physicsWorld->addRigidBody($this->carBodyRigidBody);
        $this->entities->attach($carBody, $this->carBodyRigidBody);

        // Create wheels
        // -----------------------------------------------------------
        $wheelShape = new CylinderShapeX(new Vec3(0.5, 0.5, 0.5));
        $wheelPositions = [
            new Vec3(-1.5, 3.8, -2), // front-left
            new Vec3( 1.5, 3.8, -2), // front-right
            new Vec3(-1.5, 3.8,  2), // rear-left
            new Vec3( 1.5, 3.8,  2), // rear-right
        ];

        $localWheelAxis = new Vec3(1, 0, 0);
        $suspensionTravel = 0.25;
        $maxSteer = 0.5;


        foreach ($wheelPositions as $i => $pos) {
            $wheelEntity   = $this->entities->create();
            $wheelRigidBody = new RigidBody($wheelShape, 0.5);
            $wheelRigidBody->setPosition($pos);
            $wheelRigidBody->setFriction(2);
            $wheelRigidBody->disableDeactivation();

            $model = $this->entities->attach($wheelEntity, new DynamicRenderableModel());
            $model->modelIdentifier = 'cylinderx.obj';

            $wheelTransform = $this->entities->attach($wheelEntity, new Transform());
            $wheelTransform->position = $wheelRigidBody->getPosition();
            $wheelTransform->scale    = new Vec3(0.5);

            $this->physicsWorld->addRigidBody($wheelRigidBody);
            $this->entities->attach($wheelEntity, $wheelRigidBody);
            
            // spring suspension
            $frameA = new Mat4();
            $frameA->translate($pos - $this->carBodyRigidBody->getPosition());

            $frameB = new Mat4();
            // $frameB->rotate(GLM::radians(90), new Vec3(0, 0, 1));  

            $suspension = new Generic6DofSpring2Constraint(
                $this->carBodyRigidBody,
                $wheelRigidBody,
                $frameA,
                $frameB
            );

            $suspension->setLinearLowerLimit(new Vec3(0, -$suspensionTravel, 0));
            $suspension->setLinearUpperLimit(new Vec3(0,  $suspensionTravel, 0));

            $suspension->enableSpring(1, true);
            $suspension->setStiffness(1, 40.0);
            $suspension->setDamping(1, 0.2);
            $suspension->setEquilibriumPoint(); 

            // enable motor on rear wheels
            if ($i >= 2) {
                $suspension->setAngularLowerLimit(new Vec3(-ANGULAR_LIMIT, 0, 0));
                $suspension->setAngularUpperLimit(new Vec3( ANGULAR_LIMIT, 0, 0));

                $suspension->enableMotor(3, true);
                $suspension->setMaxMotorForce(3, 1000.0);
                $suspension->setTargetVelocity(3, 0.0);
            }

            // configure front wheels
            if ($i <= 1) {
                $suspension->setAngularLowerLimit(new Vec3(-ANGULAR_LIMIT, -$maxSteer, 0));
                $suspension->setAngularUpperLimit(new Vec3( ANGULAR_LIMIT,  $maxSteer, 0));

                $suspension->enableMotor(self::STEER_AXIS, true);
                $suspension->setServo(self::STEER_AXIS, true);
                $suspension->setMaxMotorForce(self::STEER_AXIS, 500.0);
                $suspension->setTargetVelocity(self::STEER_AXIS, 4.0);   // ~230 °/s
                $suspension->setServoTarget(self::STEER_AXIS, 0.0);
            }

            // add to physics world & ecs
            $this->physicsWorld->addConstraint($suspension, true);
            $this->entities->attach($wheelEntity, $suspension);

            // assign wheel entity references
            // (this is a dump way to do this)
            switch ($i) {
                case 0: $this->frontLeftWheel  = $wheelEntity; break;
                case 1: $this->frontRightWheel = $wheelEntity; break;
                case 2: $this->rearLeftWheel   = $wheelEntity; break;
                case 3: $this->rearRightWheel  = $wheelEntity; break;
            }
        }
    }

    /**
     * Prepare / setup additional render passes before the quickstart draw pass 
     * This is an "setup" method meaning you should not emit any draw calls here, but 
     * rather add additional render passes to the pipeline.
     * 
     * @param RenderContext $context
     * @param RenderTargetResource $renderTarget
     * @return void 
     */
    public function setupDrawBefore(RenderContext $context, RenderTargetResource $renderTarget) : void
    {
        // Let the camera system handle camera movement
        $this->cameraSystem->render($this->entities, $context);

        // Let the low-poly rendering system handle model drawing
        $this->renderingSystem->setRenderTarget($renderTarget);
        $this->renderingSystem->render($this->entities, $context);
    }

    /**
     * Draw the scene. (You most definetly want to use this)
     * 
     * This is called from within the Quickstart render pass where the pipeline is already
     * prepared, a VG frame is also already started.
     */
    public function draw(RenderContext $context, RenderTarget $renderTarget) : void
    {
        QuickstartDebugMetricsOverlay::debugString('Steering angle: ' . round($this->steerAngle, 2));
        
        $this->physicsWorld->setDebugDrawVP($context->data->get(CameraData::class)->projectionView);
        $this->physicsWorld->debugDrawWorld();
    }

    /**
     * Update the games state
     * This method might be called multiple times per frame, or not at all if
     * the frame rate is very high.
     * 
     * The update method should step the game forward in time, this is the place
     * where you would update the position of your game objects, check for collisions
     * and so on. 
     */
    public function update() : void
    {
        parent::update();
        $this->cameraSystem->update($this->entities);

        $camera = $this->cameraSystem->getActiveCamera($this->entities);
        $carPos = $this->carBodyRigidBody->getPosition();
        $carRot = $this->carBodyRigidBody->getOrientation();

        $localOffset = new Vec3(0, 2, 20);
        $worldOffset = $carRot * $localOffset;
        $camera->transform->position = $carPos + $worldOffset;
        $lookAtTarget = $carPos + ($carRot * new Vec3(0, 0, 5)); 
        $camera->transform->lookAt($lookAtTarget);



        // Car control
        // ------------------------------------------------------------
        // wake it up
        $this->carBodyRigidBody->activate();

        $orientation = $this->carBodyRigidBody->getOrientation();
        $localForward = new Vec3(0, 0, -1);
        $worldForward = $orientation * $localForward;

        // force
        $engineForce = 100.0;
        $steeringTorque = 30.0;

        $flWheelSusp = $this->entities->get($this->frontLeftWheel, Generic6DofSpring2Constraint::class);
        $frWheelSusp = $this->entities->get($this->frontRightWheel, Generic6DofSpring2Constraint::class);
        $rlWheelSusp = $this->entities->get($this->rearLeftWheel, Generic6DofSpring2Constraint::class);
        $rrWheelSusp = $this->entities->get($this->rearRightWheel, Generic6DofSpring2Constraint::class);

        // $flWheelSusp->setTargetVelocity(3, 0.0);
        // $frWheelSusp->setTargetVelocity(3, 0.0);
        $rlWheelSusp->enableMotor(3, false);
        $rrWheelSusp->enableMotor(3, false);

        if ($this->inputContext->actions->isButtonDown('moveForward')) {
            $rlWheelSusp->enableMotor(3, true);
            $rrWheelSusp->enableMotor(3, true);

            $rlWheelSusp->setTargetVelocity(3, 10.0);
            $rrWheelSusp->setTargetVelocity(3, 10.0);
        }

        if ($this->inputContext->actions->isButtonDown('moveBackward')) {
            $rlWheelSusp->enableMotor(3, true);
            $rrWheelSusp->enableMotor(3, true);

            $rlWheelSusp->setTargetVelocity(3, -10.0);
            $rrWheelSusp->setTargetVelocity(3, -10.0);
        }

        if ($this->inputContext->actions->isButtonDown('turnLeft')) {
            $this->steerAngle += self::STEER_SPEED;
        }

        if ($this->inputContext->actions->isButtonDown('turnRight')) {
            $this->steerAngle -= self::STEER_SPEED;
        }

        $this->steerAngle = max(-self::MAX_STEER, min(self::MAX_STEER, $this->steerAngle));
        $flWheelSusp->setServoTarget(self::STEER_AXIS, -$this->steerAngle);
        $frWheelSusp->setServoTarget(self::STEER_AXIS, -$this->steerAngle);

        // update the physics world
        $this->physicsWorld->stepSimulation(1 / 60);

        // update the position of the entities
        foreach ($this->entities->viewWith(RigidBody::class, Transform::class) as $entity => [$rigidbody, $transform]) {
            $transform = $this->entities->get($entity, Transform::class);
            $transform->setPosition($rigidbody->getPosition());
            $transform->setOrientation($rigidbody->getOrientation());
        }
    }
}
