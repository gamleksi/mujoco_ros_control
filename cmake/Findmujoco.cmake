
find_path(mujoco_INCLUDE_DIR
        NAMES mujoco.h
        PATHS
        $ENV{HOME}/.mujoco/mjpro150/include
        NO_DEFAULT_PATH
        )

find_library(mujoco_LIBRARIES
        NAMES libmujoco150nogl.so
        PATHS
        $ENV{HOME}/.mujoco/mjpro150/bin
        NO_DEFAULT_PATH
        )

if (mujoco_LIBRARIES AND mujoco_INCLUDE_DIR)
#    message("mujoco include dir: ${mujoco_INCLUDE_DIR}")
#    message("mujoco lib: ${mujoco_LIBRARIES}")
    set(HAVE_mujoco TRUE)
else (mujoco_LIBRARIES AND mujoco_INCLUDE_DIR)
    if (mujoco_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find mujoco!")
    endif (mujoco_FIND_REQUIRED)
    set(HAVE_mujoco FALSE)
endif (mujoco_LIBRARIES AND mujoco_INCLUDE_DIR)


mark_as_advanced(mujoco_INCLUDE_DIR mujoco_LIBRARIES HAVE_mujoco)