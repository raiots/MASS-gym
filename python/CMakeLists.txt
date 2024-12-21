cmake_minimum_required(VERSION 2.8.6)

project(mss)

add_compile_options(-fPIC)
add_compile_options(-std=gnu++11)
add_compile_options(-Wdeprecated-declarations)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

include(FindOpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(DART REQUIRED COMPONENTS collision-bullet CONFIG)
find_package(PythonLibs REQUIRED)
find_package(pybind11 REQUIRED)
find_package(TinyXML REQUIRED)



file(GLOB srcs "*.h" "*.cpp")

include_directories(${PYTHON_INCLUDE_DIR})
include_directories(${DART_INCLUDE_DIRS})

add_library(pyMyWorldPtr SHARED ${srcs})

target_link_libraries(pyMyWorldPtr ${DART_LIBRARIES} ${PYTHON_LIBRARIES} ${TinyXML_LIBRARIES} pybind11::module)
set_target_properties(pyMyWorldPtr PROPERTIES PREFIX "" )

add_library(pyMyDARTHelper SHARED ${srcs})

target_link_libraries(pyMyDARTHelper ${DART_LIBRARIES} ${PYTHON_LIBRARIES} ${TinyXML_LIBRARIES} pybind11::module)
set_target_properties(pyMyDARTHelper PROPERTIES PREFIX "" )

add_library(pyMySkeletonPtr SHARED ${srcs})

target_link_libraries(pyMySkeletonPtr ${DART_LIBRARIES} ${PYTHON_LIBRARIES} ${TinyXML_LIBRARIES} pybind11::module)
set_target_properties(pyMySkeletonPtr PROPERTIES PREFIX "" )

add_library(pyBVH SHARED ${srcs})

target_link_libraries(pyBVH ${DART_LIBRARIES} ${PYTHON_LIBRARIES} ${TinyXML_LIBRARIES} pybind11::module)
set_target_properties(pyBVH PROPERTIES PREFIX "" )

add_library(pyMyIsometry3d SHARED ${srcs})

target_link_libraries(pyMyIsometry3d ${DART_LIBRARIES} ${PYTHON_LIBRARIES} ${TinyXML_LIBRARIES} pybind11::module)
set_target_properties(pyMyIsometry3d PROPERTIES PREFIX "" )

add_library(pyMyBodyNodePtr SHARED ${srcs})

target_link_libraries(pyMyBodyNodePtr ${DART_LIBRARIES} ${PYTHON_LIBRARIES} ${TinyXML_LIBRARIES} pybind11::module)
set_target_properties(pyMyBodyNodePtr PROPERTIES PREFIX "" )

add_library(pyEnv SHARED ${srcs})

target_link_libraries(pyEnv ${DART_LIBRARIES} ${PYTHON_LIBRARIES} ${TinyXML_LIBRARIES} pybind11::module)
set_target_properties(pyEnv PROPERTIES PREFIX "" )



