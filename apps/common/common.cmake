# SPDX-License-Identifier: GPL-3.0-or-later
#
# Shared by the apps: USB device with the CDC-ACM shell and DFU through
# MCUboot (app_usb.h), system thread pinning (app_cpu.h), crash records
# across resets (app_crash.h).

include(${ZEPHYR_BASE}/samples/subsys/usb/common/common.cmake)

target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
target_sources(app PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/src/app_usb.c
  ${CMAKE_CURRENT_LIST_DIR}/src/app_cpu.c
  ${CMAKE_CURRENT_LIST_DIR}/src/app_crash.c)
# The double exception breadcrumb is a hand-written Xtensa vector (xtensa/corebits.h,
# DEPC/EXCCAUSE/EPC1): every current user is an S3 app, gated the same way by
# CONFIG_USB_DEVICE_STACK_NEXT, but guard by architecture here too so this file
# stays safe to include from a RISC-V (C6) app as well.
if(CONFIG_XTENSA)
  target_sources(app PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src/app_crash_dx.S)
endif()
