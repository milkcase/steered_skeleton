# 只供独立预演构建使用，覆盖输出目录而不修改骨架CMakeLists或覆盖正常固件。
# 动力调试构建也复用此输出隔离，使用另一独立构建目录，不改零输出预演ELF。
if(PROJECT_NAME MATCHES "^steered_(chassis|gimbal)$")
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/firmware")
endif()
