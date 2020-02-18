
find_path(glfw3_INCLUDE_DIR
        NAMES glfw3.h
        PATHS
        $ENV{HOME}/.mujoco/mjpro200/include
        $ENV{HOME}/.mujoco/mujoco200/include	
	NO_DEFAULT_PATH
        )

find_library(glfw3_LIBRARIES
        NAMES libglfw.so.3
        PATHS
        $ENV{HOME}/.mujoco/mjpro200/bin
	$ENV{HOME}/.mujoco/mujoco200/bin
        NO_DEFAULT_PATH
        )

if (glfw3_LIBRARIES AND glfw3_INCLUDE_DIR)
    message("GLFW3 library: ${glfw3_LIBRARIES}")
    message("GLFW3 dir: ${glfw3_INCLUDE_DIR}")
    set(FOUND_glfw3 TRUE)
else (glfw3_LIBRARIES AND glfw3_INCLUDE_DIR)
    if (glfw3_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find glfw3!")
    endif (glfw3_FIND_REQUIRED)
    set(FOUND_glfw3 FALSE)
endif (glfw3_LIBRARIES AND glfw3_INCLUDE_DIR)


mark_as_advanced(glfw3_INCLUDE_DIR glfw3_LIBRARIES FOUND_glfw3)
