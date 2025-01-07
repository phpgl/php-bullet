dnl config.m4 for extension bullet3

PHP_ARG_ENABLE([bullet3],
  [whether to enable bullet3 support],
  [AS_HELP_STRING([--enable-bullet3],
    [Enable bullet3 support])],
  [no])

if test "$PHP_BULLET3" != "no"; then

  AC_MSG_CHECKING([for bullet header files])
  AC_CHECK_HEADERS([BulletDynamics/btBulletDynamicsCommon.h],
    [have_bullet3="yes"],
    [have_bullet3="no"])

  if test "$have_bullet3" = "no"; then
    AC_MSG_NOTICE([Bullet3 not found, will build from 'bulletphysics' subdirectory])

    BULLET3_SUBDIR="${abs_srcdir}/bulletphysics"

    dnl check if the dir exists
    if test -d "$BULLET3_SUBDIR"; then

      AC_MSG_NOTICE([Configuring Bullet3 with CMake])

      BULLET3_BUILD_DIR="${abs_builddir}/bullet3_build"
      mkdir -p "$BULLET3_BUILD_DIR"

      dnl run CMake to configure the build
    #   if ! (cd "$BULLET3_BUILD_DIR" && cmake "$BULLET3_SUBDIR" -DCMAKE_INSTALL_PREFIX="$BULLET3_BUILD_DIR"); then
    #   if ! (cd "$BULLET3_BUILD_DIR" && cmake "$BULLET3_SUBDIR" -DCMAKE_INSTALL_PREFIX="$BULLET3_BUILD_DIR" -DBUILD_UNIT_TESTS=OFF -DBUILD_BULLET3=ON -DBUILD_EXTRAS=OFF -DBUILD_CPU_DEMOS=OFF -DBUILD_OPENGL3_DEMOS=OFF -DBUILD_SHARED_LIBS=OFF); then
      if ! (cd "$BULLET3_BUILD_DIR" && cmake "$BULLET3_SUBDIR" \
        -DCMAKE_INSTALL_PREFIX="$BULLET3_BUILD_DIR" \
        -DCMAKE_OSX_SYSROOT=$(xcrun --sdk macosx --show-sdk-path) \
        -DBUILD_UNIT_TESTS=OFF \
        -DBUILD_BULLET3=ON \
        -DBUILD_EXTRAS=OFF \
        -DBUILD_CPU_DEMOS=OFF \
        -DBUILD_OPENGL3_DEMOS=OFF \
        -DBUILD_SHARED_LIBS=OFF \
        -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY="$BULLET3_BUILD_DIR/lib" \
        -DCMAKE_LIBRARY_OUTPUT_DIRECTORY="$BULLET3_BUILD_DIR/lib" \
        -DCMAKE_RUNTIME_OUTPUT_DIRECTORY="$BULLET3_BUILD_DIR/bin"); then
        AC_MSG_ERROR([CMake configuration for Bullet3 failed])
      fi

      if ! make -C "$BULLET3_BUILD_DIR"; then
        AC_MSG_ERROR([Failed to build Bullet3])
      fi

    else
      AC_MSG_ERROR([Bullet3 library not found and 'bulletphysics' subdirectory is missing. Please install Bullet3 or ensure the subdirectory is present.])
    fi
  else
    AC_MSG_NOTICE([Bullet3 found])
    # BULLET3_CFLAGS=""
    BULLET3_LIBS="-lBulletDynamics -lBulletCollision -lLinearMath"
  fi

  # add bullet libs
  PHP_ADD_LIBRARY_WITH_PATH([BulletDynamics], [$BULLET3_BUILD_DIR/lib], [BULLET3_SHARED_LIBADD])
  PHP_ADD_LIBRARY_WITH_PATH([BulletCollision], [$BULLET3_BUILD_DIR/lib], [BULLET3_SHARED_LIBADD])
  PHP_ADD_LIBRARY_WITH_PATH([LinearMath], [$BULLET3_BUILD_DIR/lib], [BULLET3_SHARED_LIBADD])
  
  PHP_SUBST(BULLET3_SHARED_LIBADD)

  # dnl Include the Bullet3 headers and libraries
  # PHP_EVAL_INCLINE([$BULLET3_CFLAGS])
  # PHP_EVAL_LIBLINE([$BULLET3_LIBS], BULLET3_SHARED_LIBADD)
  # PHP_SUBST(BULLET3_SHARED_LIBADD)
  PHP_ADD_INCLUDE([$BULLET3_SUBDIR/src])

  # dump include path
  AC_MSG_NOTICE([Bullet3 include path: $BULLET3_SUBDIR/src])
  AC_MSG_NOTICE([BULLET3_SHARED_LIBADD: $BULLET3_SHARED_LIBADD])
  AC_MSG_NOTICE([BULLET3_BUILD_DIR: $BULLET3_BUILD_DIR])
  AC_MSG_NOTICE([BULLET3_SUBDIR: $BULLET3_SUBDIR])

  AC_DEFINE([HAVE_BULLET3], [1], [Define if Bullet3 support is enabled])

  dnl cpp lib
  PHP_REQUIRE_CXX()

  dnl register the extension
  PHP_NEW_EXTENSION([bullet3], [bullet3.c bullet3_world.c bullet3_constraint.c bullet3_bridge.cpp], [$ext_shared], , , [$BULLET3_SHARED_LIBADD])
fi
