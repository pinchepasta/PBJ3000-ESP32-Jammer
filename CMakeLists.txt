# SPDX-License-Identifier: GPL-3.0-or-later

if(CONFIG_ESP_SDR)
  zephyr_include_directories(include)
endif()

add_subdirectory(lib)

# The apps here take their version from VERSION (a link in each app directory). As in
# Zephyr's example-application, app_version.h depends on the git index too, so its
# APP_BUILD_VERSION (git describe) follows new commits.
cmake_path(IS_PREFIX CMAKE_CURRENT_LIST_DIR "${APPLICATION_SOURCE_DIR}" NORMALIZE in_repo)
if(TARGET app_version_h AND in_repo)
  find_package(Git QUIET)
  if(GIT_FOUND)
    execute_process(
      COMMAND ${GIT_EXECUTABLE} rev-parse --absolute-git-dir
      WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
      OUTPUT_VARIABLE git_dir
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
      RESULT_VARIABLE ret
    )
    if(NOT ret)
      set_property(TARGET app_version_h PROPERTY APP_VERSION_DEPENDS ${git_dir}/index)
    endif()
  endif()
endif()
