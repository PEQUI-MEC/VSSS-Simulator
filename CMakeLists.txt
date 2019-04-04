#Para acrescentar novos arquivos:
#Acrescente o .cpp em set(SOURCE_FILES ...) e o diretorio dos .h e .hpp em include_directories()

#Para incluir novas biliotecas:
#utilize pkg_check_modules(NOME pacotes) para defini-la, seguido de link_directories(${NOME_LIBRARY_DIRS}),
#acrescente ${NOME_INCLUDE_DIRS} em include_directories()
#e insira target_link_libraries(VSSS ${NOME_LIBRARIES}) no final do arquivo

cmake_minimum_required(VERSION 3.5)
project(VSSS-Simulator)

set(CMAKE_CXX_STANDARD 17)
set(LIB_DIR ${PROJECT_SOURCE_DIR}/lib)
include("${PROJECT_SOURCE_DIR}/lib/mujoco200_linux/mujoco.cmake")

FIND_PACKAGE(PkgConfig REQUIRED)
pkg_search_module(GLFW3 REQUIRED glfw3)
find_package(X11 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        tf2 roscpp rospy std_msgs geometry_msgs vsss_msgs)

#Define arquivos a serem compilados. Novos arquivos devem ser inclusos aqui
set(SOURCE_FILES
        src/main.cpp ${MUJOCO_SRC} src/Simulator.cpp src/Simulator.h
        src/SimulatorGUI.cpp src/SimulatorGUI.h src/SimuRobotControl.cpp
        src/SimuRobotControl.h src/include.h src/SimuRobot.h)

link_directories(${MUJOCO_BIN_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

#Define diretorios com .h e .hpp. Novos diretorios devem ser inclusos aqui
include_directories(
    ${PROJECT_SOURCE_DIR}/src/
    ${GLFW3_INCLUDE_DIR}
    ${X11_INCLUDE_DIR}
    ${MUJOCO_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++17 -O3 -mavx -pthread -Wl,-rpath -lmujoco200nogl")

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} libmujoco200.so)
target_link_libraries(${PROJECT_NAME} ${GLFW3_STATIC_LIBRARIES})
target_link_libraries(${PROJECT_NAME} GL GLU glut)
target_link_libraries(${PROJECT_NAME} libglew.so)
target_link_libraries(${PROJECT_NAME} ${X11_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
