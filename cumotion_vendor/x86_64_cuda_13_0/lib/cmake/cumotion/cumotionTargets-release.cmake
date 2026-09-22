#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "cumotion::cumotion" for configuration "Release"
set_property(TARGET cumotion::cumotion APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(cumotion::cumotion PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcumotion.so.1.1.0"
  IMPORTED_SONAME_RELEASE "libcumotion.so.1"
  )

list(APPEND _cmake_import_check_targets cumotion::cumotion )
list(APPEND _cmake_import_check_files_for_cumotion::cumotion "${_IMPORT_PREFIX}/lib/libcumotion.so.1.1.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
