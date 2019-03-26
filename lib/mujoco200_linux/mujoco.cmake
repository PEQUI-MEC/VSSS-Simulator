set(USE_GL 1)
set(MUJOCO_DIR ${LIB_DIR}/mujoco200_linux)
set(MUJOCO_BIN_DIR ${MUJOCO_DIR}/bin)
set(MUJOCO_INCLUDE_DIR ${MUJOCO_DIR}/include)
set(MUJOCO_SRC
    ${MUJOCO_INCLUDE_DIR}/mjrender.h
    ${MUJOCO_INCLUDE_DIR}/mjxmacro.h
    ${MUJOCO_INCLUDE_DIR}/mjdata.h
    ${MUJOCO_INCLUDE_DIR}/glfw3.h
    ${MUJOCO_INCLUDE_DIR}/uitools.h
    ${MUJOCO_INCLUDE_DIR}/mjmodel.h
    ${MUJOCO_INCLUDE_DIR}/mjui.h
    ${MUJOCO_INCLUDE_DIR}/mjvisualize.h
    ${MUJOCO_INCLUDE_DIR}/uitools.c
    ${MUJOCO_INCLUDE_DIR}/mujoco.h)
