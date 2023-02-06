cmake_minimum_required(VERSION 3.10)
project(_raisim_gym_torch)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/raisimGymTorch/env/bin)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/raisimGymTorch/env/bin)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/raisimGymTorch/env/bin)



include_directories(
/raisim_build/include/OGRE
/raisim_build/include/OGRE/Bites
/raisim_build/include/OGRE/HLMS
/raisim_build/include/OGRE/MeshLodGenerator
/raisim_build/include/OGRE/Overlay
/raisim_build/include/OGRE/Paging
/raisim_build/include/OGRE/Plugins
/raisim_build/include/OGRE/Plugins/BSPSceneManager
/raisim_build/include/OGRE/Plugins/DotScene
/raisim_build/include/OGRE/Plugins/FreeImageCodec
/raisim_build/include/OGRE/Plugins/OctreeSceneManager
/raisim_build/include/OGRE/Plugins/OctreeZone
/raisim_build/include/OGRE/Plugins/PCZSceneManager
/raisim_build/include/OGRE/Plugins/ParticleFX
/raisim_build/include/OGRE/Plugins/STBICodec
/raisim_build/include/OGRE/Property
/raisim_build/include/OGRE/RTShaderSystem
/raisim_build/include/OGRE/RenderSystems
/raisim_build/include/OGRE/RenderSystems/GL
/raisim_build/include/OGRE/RenderSystems/GL/GL
/raisim_build/include/OGRE/RenderSystems/GL3Plus
/raisim_build/include/OGRE/RenderSystems/GL3Plus/GL
/raisim_build/include/OGRE/Terrain
/raisim_build/include/OGRE/Threading
/raisim_build/include/OGRE/Volume
/raisim_build/include/assimp
/raisim_build/include/assimp/Compiler
/raisim_build/include/ccd
/raisim_build/include/irrXML
/raisim_build/include/ode
/raisim_build/include/ode/extras
/raisim_build/include/png
/raisim_build/include/pybind11
/raisim_build/include/pybind11/detail
/raisim_build/include/raisim
/raisim_build/include/raisim/constraints
/raisim_build/include/raisim/contact
/raisim_build/include/raisim/math
/raisim_build/include/raisim/object
/raisim_build/include/raisim/object/ArticulatedSystem
/raisim_build/include/raisim/object/singleBodies
/raisim_build/include/raisim/object/terrain
/raisim_build/include/raisimMine
/raisim_build/include/tinyxml_rai
/raisim_build/include/z
)




######################
### help functions ###
######################
MACRO(SUBDIRLIST result curdir)
    FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
    SET(dirlist "")
    FOREACH(child ${children})
        IF(IS_DIRECTORY ${curdir}/${child})
            LIST(APPEND dirlist ${child})
        ENDIF()
    ENDFOREACH()
    SET(${result} ${dirlist})
ENDMACRO()

####################
### dependencies ###
####################
set(Dependencies)

add_subdirectory(
	../thirdParty/pybind11 pybind11
)


find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(raisimOgre CONFIG REQUIRED)

if (UNIX AND NOT APPLE AND NOT ANDROID AND NOT WEBGL AND NOT WIN32)
    set(RAISIM_OS linux)
    list(APPEND CMAKE_PREFIX_PATH ../raisim/${RAISIM_OS})
elseif(APPLE)
    set(RAISIM_OS mac)
    list(APPEND CMAKE_PREFIX_PATH ../raisim/${RAISIM_OS})
    list(APPEND Dependencies OpenMP::OpenMP_CXX)
elseif(WIN32)
    set(RAISIM_OS win32)
    list(APPEND CMAKE_PREFIX_PATH ../raisim/${RAISIM_OS}/mt_debug)
    list(APPEND CMAKE_PREFIX_PATH ../thirdParty)
    list(APPEND Dependencies Ws2_32)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/raisimGymTorch/env/bin)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/raisimGymTorch/env/bin)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/raisimGymTorch/env/bin)
endif()

find_package(raisim CONFIG REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

#######################
### src and linking ###
#######################
SUBDIRLIST(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR}/raisimGymTorch/env/envs)
set(RAISIMGYM_ENV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/raisimGymTorch/env/envs)

FOREACH(subdir ${SUBDIRS})
    pybind11_add_module(${subdir} raisimGymTorch/env/raisim_gym.cpp raisimGymTorch/env/Yaml.cpp)
    target_link_libraries(${subdir} PRIVATE raisim::raisimOgre raisim::raisim ${Dependencies})
    target_include_directories(${subdir} PUBLIC ${EIGEN3_INCLUDE_DIRS} ${RAISIMGYM_ENV_DIR}/${subdir} ${CMAKE_CURRENT_SOURCE_DIR}/raisimGymTorch/env)
    target_compile_options(${subdir} PRIVATE -mtune=native -fPIC -O3 -g -mno-avx2)
    target_compile_definitions(${subdir} PRIVATE "-DRAISIMGYM_TORCH_ENV_NAME=${subdir}")

    if (CMAKE_BUILD_TYPE STREQUAL "DEBUG" OR CMAKE_BUILD_TYPE STREQUAL "Debug")
        message("[RAISIM_GYM] BUILDING THE DEBUG APP for ${subdir}")
        add_executable(${subdir}_debug_app raisimGymTorch/env/debug_app.cpp raisimGymTorch/env/Yaml.cpp)
        target_link_libraries(${subdir}_debug_app PRIVATE raisim::raisim)
        target_include_directories(${subdir}_debug_app PUBLIC raisimGymTorch/env/envs/${subdir} ${EIGEN3_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/raisimGymTorch/env)
        if(WIN32)
            target_link_libraries(${subdir}_debug_app PRIVATE Ws2_32)
        else()
            target_compile_options(${subdir}_debug_app PRIVATE -mtune=native -fPIC -g -O0 -mno-avx2)
        endif()
    endif()
ENDFOREACH()
