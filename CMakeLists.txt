cmake_minimum_required(VERSION 3.10)
project(sentry_chassis_controller)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        roslint
        controller_interface
        hardware_interface
        forward_command_controller
        pluginlib
        control_toolbox
    )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS
    include
    ## This is only necessary because Eigen3 sets a non-standard EIGEN3_INCLUDE_DIR variable
    LIBRARIES
    ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        roslint
        controller_interface
        hardware_interface
        forward_command_controller
        pluginlib
        control_toolbox
        DEPENDS
    ## find_package(Eigen3) provides a non standard EIGEN3_INCLUDE_DIR instead of Eigen3_INCLUDE_DIRS.
    ## Therefore, the DEPEND does not work as expected and we need to add the directory to the INCLUDE_DIRS
    # Eigen3

    ## Boost is not part of the DEPENDS since it is only used in source files,
    ## Dependees do not depend on Boost when they depend on this package.
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    # Set manually because Eigen sets a non standard INCLUDE DIR
)

## Declare a cpp library
add_library(${PROJECT_NAME}
    src/${PROJECT_NAME}.cpp)

## Declare cpp executables


## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    )


## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )



#############
## Install ##
#############

# Mark executables and/or libraries for installation

# Mark cpp header files for installation


# Mark other files for installation


#############
## Testing ##
#############



##########################
## Static code analysis ##
##########################

roslint_cpp()
