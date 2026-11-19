# 两板 Main 固件保持统一的输出路径，供烧录和 Ozone 工程直接打开。
if(PROJECT_NAME MATCHES "^steered_(chassis|gimbal)$")
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/firmware")
endif()
