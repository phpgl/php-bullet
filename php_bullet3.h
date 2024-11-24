/* bullet3 extension for PHP */

#ifndef PHP_BULLET3_H
# define PHP_BULLET3_H

extern zend_module_entry bullet3_module_entry;
# define phpext_bullet3_ptr &bullet3_module_entry

# define PHP_BULLET3_VERSION "0.1.0"

# if defined(ZTS) && defined(COMPILE_DL_BULLET3)
ZEND_TSRMLS_CACHE_EXTERN()
# endif

#endif	/* PHP_BULLET3_H */
