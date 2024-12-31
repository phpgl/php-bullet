<?php 

namespace App;

use Bullet\RigidBody;
use Bullet\SphereShape;
use Bullet\World;
use Error;
use GL\Math\Vec2;
use GL\Math\Vec3;
use GL\VectorGraphics\{VGAlign, VGColor, VGContext};
use VISU\Component\VISULowPoly\DynamicRenderableModel;
use VISU\Graphics\{RenderTarget, Viewport, Camera, CameraProjectionMode};
use VISU\Graphics\Rendering\RenderContext;
use VISU\Geo\Transform;
use VISU\Graphics\Rendering\Pass\BackbufferData;
use VISU\Graphics\Rendering\Renderer\Debug3DRenderer;
use VISU\Graphics\Rendering\Resource\RenderTargetResource;
use VISU\OS\{InputActionMap, Key};

use VISU\Quickstart\QuickstartApp;
use VISU\Quickstart\Render\QuickstartPassData;
use VISU\System\VISUCameraSystem;
use VISU\System\VISULowPoly\LPModelCollection;
use VISU\System\VISULowPoly\LPObjLoader;
use VISU\System\VISULowPoly\LPRenderingSystem;

class AppExample1 extends QuickstartApp
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
     * The positon & velocity of the ball in this example
     */
    private World $physicsWorld;

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
        $this->entities->registerComponent(RigidBody::class);
        $this->entities->registerComponent(Camera::class);

        // load models
        $objectCollection = $this->container->getTyped(LPModelCollection::class, 'models');
        $objectLoader = $this->container->getTyped(LPObjLoader::class, 'models.loader');

        // load the space kit models
        // these are smaller then our unit space of 1 unit = 1 meter
        // so we scale them up by a factor of 4.0 to ~ match our environment scale
        $objectLoader->loadAllInDirectory(VISU_PATH_RESOURCES . '/models/spacekit', $objectCollection, 4.0);
        $objectLoader->loadAllInDirectory(VISU_PATH_RESOURCES . '/models/primitives', $objectCollection, 1.0);

        // You can bind actions to keys in VISU 
        // this way you can decouple your game logic from the actual key bindings
        // and provides a comfortable way to access input state
        $actions = new InputActionMap;
        $actions->bindButton('bounce', Key::SPACE);
        $actions->bindButton('pushRight', Key::D);
        $actions->bindButton('pushLeft', Key::A);

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
        $this->physicsWorld->setGravity(new Vec3(0, -9.81, 0));

        $shpereShape = new SphereShape(1);

        for ($i = 0; $i < 500; $i++) 
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
        $this->cameraSystem->render($this->entities, $context);

        // let the rendering system handle the rest
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
        // update the position of the entities
        foreach ($this->entities->view(RigidBody::class) as $entity => $rigidbody) {
            $transform = $this->entities->get($entity, Transform::class);

            $position = $rigidbody->getPosition();
            $position = $position + ($rigidbody->getLinearVelocity() * 1 / 60) * $context->compensation;

            $transform->setPosition($position);
        }
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

        // randomly delete a rigidbody
        if (mt_rand(0, 100) < 5) {
            if ($entity = $this->entities->firstWith(RigidBody::class)) {
                $this->entities->destroy($entity);
            }
        }

        // handle key presses
        // if ($this->inputContext->actions->didButtonPress('bounce')) {
        //     $this->ballVelocity->y = -3.0;
        // }

        // if ($this->inputContext->actions->isButtonDown('pushRight')) {
        //     $this->ballVelocity->x = $this->ballVelocity->x + 0.1;
        // }

        // if ($this->inputContext->actions->isButtonDown('pushLeft')) {
        //     $this->ballVelocity->x = $this->ballVelocity->x - 0.1;
        // }

        // update the physics world
        $this->physicsWorld->stepSimulation(1 / 60);

        // update the position of the entities
        foreach ($this->entities->view(RigidBody::class) as $entity => $rigidbody) {
            $transform = $this->entities->get($entity, Transform::class);
            $transform->setPosition($rigidbody->getPosition());
            $transform->setOrientation($rigidbody->getOrientation());
        }
    }
}