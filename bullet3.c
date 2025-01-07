/* bullet3 extension for PHP */

#ifdef HAVE_CONFIG_H
# include "config.h"
#endif

#include "php.h"
#include "ext/standard/info.h"
#include "php_bullet3.h"
#include "bullet3_arginfo.h"
#include "bullet3_bridge.h"
#include "bullet3_world.h" 
#include "bullet3_constraint.h"
 

/* For compatibility with older PHP versions */
#ifndef ZEND_PARSE_PARAMETERS_NONE
#define ZEND_PARSE_PARAMETERS_NONE() \
    ZEND_PARSE_PARAMETERS_START(0, 0) \
    ZEND_PARSE_PARAMETERS_END()
#endif

PHP_MINIT_FUNCTION(bullet3)
{
    // world module
    phpbullet3_register_world_module(INIT_FUNC_ARGS_PASSTHRU);
    phpbullet3_register_constraint_module(INIT_FUNC_ARGS_PASSTHRU);

    return SUCCESS;
}

/* {{{ string bullet3_test() */
PHP_FUNCTION(Bullet_bullet3_test)
{
    ZEND_PARSE_PARAMETERS_NONE();
}
/* }}} */

/* {{{ PHP_RINIT_FUNCTION */
PHP_RINIT_FUNCTION(bullet3)
{
#if defined(ZTS) && defined(COMPILE_DL_BULLET3)
    ZEND_TSRMLS_CACHE_UPDATE();
#endif

    return SUCCESS;
}
/* }}} */

/* {{{ PHP_MINFO_FUNCTION */
PHP_MINFO_FUNCTION(bullet3)
{
    php_info_print_table_start();
    php_info_print_table_header(2, "bullet3 support", "enabled");
    php_info_print_table_end();
}
/* }}} */

zend_module_entry bullet3_module_entry = {
    STANDARD_MODULE_HEADER,
    "bullet3",                    /* Extension name */
    ext_functions,                /* zend_function_entry */
    PHP_MINIT(bullet3),           /* PHP_MINIT - Module initialization */
    NULL,                         /* PHP_MSHUTDOWN - Module shutdown */
    PHP_RINIT(bullet3),           /* PHP_RINIT - Request initialization */
    NULL,                         /* PHP_RSHUTDOWN - Request shutdown */
    PHP_MINFO(bullet3),           /* PHP_MINFO - Module info */
    PHP_BULLET3_VERSION,          /* Version */
    STANDARD_MODULE_PROPERTIES
};
/* }}} */

#ifdef COMPILE_DL_BULLET3
# ifdef ZTS
ZEND_TSRMLS_CACHE_DEFINE()
# endif
ZEND_GET_MODULE(bullet3)
#endif
