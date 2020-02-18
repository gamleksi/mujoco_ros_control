
find_path(mujoco_INCLUDE_DIR
        NAMES mujoco.h
        PATHS 
	$ENV{HOME}/.mujoco/mjpro200/include
        $ENV{HOME}/.mujoco/mujoco200/include
	NO_DEFAULT_PATH
        )

find_library(mujoco_LIBRARIES
        NAMES libmujoco200.so
        PATHS
	$ENV{HOME}/.mujoco/mjpro200/bin 
        $ENV{HOME}/.mujoco/mujoco200/bin
	NO_DEFAULT_PATH
        )

#find_library(nvidia_LIBRARIES
#        NAMES libGL.so
#        PATHS /usr/lib/nvidia-390
#        NO_DEFAULT_PATH
#        )

find_library(libglew_LIBRARIES
        NAMES libglew.so 
        PATHS
	$ENV{HOME}/.mujoco/mjpro200/bin
	$ENV{HOME}/.mujoco/mujoco200/bin
        NO_DEFAULT_PATH
        )

if (mujoco_LIBRARIES AND mujoco_INCLUDE_DIR AND libglew_LIBRARIES)
    message("Mujoco library: ${mujoco_LIBRARIES}")
#    message("nvidia library: ${nvidia_LIBRARIES}")
    message("libglew library: ${libglew_LIBRARIES}")
    message("Mujoco dir: ${mujoco_INCLUDE_DIR}")
    set(FOUND_mujoco TRUE)
else (mujoco_LIBRARIES AND mujoco_INCLUDE_DIR AND libglew_LIBRARIES)
    if (mujoco_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find mujoco!")
    endif (mujoco_FIND_REQUIRED)
    set(FOUND_mujoco FALSE)
endif (mujoco_LIBRARIES AND mujoco_INCLUDE_DIR AND libglew_LIBRARIES)


mark_as_advanced(mujoco_INCLUDE_DIR mujoco_LIBRARIES FOUND_mujoco libglew_LIBRARIES glfw3_LIBRARIES)
