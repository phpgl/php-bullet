/**
 * php-bullet3 - Bullet Physics Bindings for PHP
 * 
 * World Module
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
#ifndef PHP_BULLET_WORLD_H
#define PHP_BULLET_WORLD_H 1

#include <php.h>
#include "bullet3_bridge.h"

#define PHPBULLET_DEFINE_COLLISION_SHAPE_METHODS(type_lower)                                  \
    typedef struct _phpbullet3_##type_lower##_object {                                        \
        btCollisionShapeWrapper *bt_shape;                                                    \
        zend_object std;                                                                      \
    } phpbullet3_##type_lower##_object;                                                       \
                                                                                              \
    zend_class_entry *phpbullet3_get_##type_lower##_ce();                                     \
                                                                                              \
    phpbullet3_##type_lower##_object *phpbullet3_##type_lower##_from_zobj_p(zend_object *obj);\

typedef struct _phpbullet3_world_object {
    btDynamicsWorldWrapper *bt_world;
    zend_object std;
} phpbullet3_world_object; 

typedef struct _phpbullet3_rigidbody_object {
    btRigidBodyWrapper *bt_rigidbody;
    zend_object std;
} phpbullet3_rigidbody_object;

zend_class_entry *phpbullet3_get_world_ce();
zend_class_entry *phpbullet3_get_rigidbody_ce();
zend_class_entry *phpbullet3_get_collision_shape_ce();

phpbullet3_world_object *phpbullet3_world_from_zobj_p(zend_object *obj);
phpbullet3_rigidbody_object *phpbullet3_rigidbody_from_zobj_p(zend_object *obj);

PHPBULLET_DEFINE_COLLISION_SHAPE_METHODS(shape_sphere);
PHPBULLET_DEFINE_COLLISION_SHAPE_METHODS(shape_box);
PHPBULLET_DEFINE_COLLISION_SHAPE_METHODS(shape_cylinder);
PHPBULLET_DEFINE_COLLISION_SHAPE_METHODS(shape_static_plane);

void phpbullet3_register_world_module(INIT_FUNC_ARGS);

#endif