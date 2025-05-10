/**
 * php-bullet3 - Bullet Physics Bindings for PHP
 * 
 * Contraints Module
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
#ifndef PHP_BULLET_CONSTRAINT_H
#define PHP_BULLET_CONSTRAINT_H 1

#include <php.h>
#include "bullet3_bridge.h"

#define PHPBULLET_DEFINE_CONSTRAINT_DECL(type_lower) \
    zend_class_entry *phpbullet3_get_##type_lower##_ce();

typedef struct _phpbullet3_constraint_wrapper_object {
    btTypedConstraintWrapper *bt_constraint;   
    zend_object std;   
} phpbullet3_constraint_wrapper_object;

zend_class_entry *phpbullet3_get_constraint_ce();

// BEGIN constraint class entries
PHPBULLET_DEFINE_CONSTRAINT_DECL(point2point)
PHPBULLET_DEFINE_CONSTRAINT_DECL(hinge)
PHPBULLET_DEFINE_CONSTRAINT_DECL(slider)
PHPBULLET_DEFINE_CONSTRAINT_DECL(generic6dofspring)
PHPBULLET_DEFINE_CONSTRAINT_DECL(generic6dofspring2)
// END constraint class entries

phpbullet3_constraint_wrapper_object *phpbullet3_constraint_from_zobj_p(zend_object *obj);

void phpbullet3_register_constraint_module(INIT_FUNC_ARGS);

#endif