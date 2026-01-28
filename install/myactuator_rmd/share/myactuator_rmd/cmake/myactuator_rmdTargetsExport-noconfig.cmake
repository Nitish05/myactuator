#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "myactuator_rmd::myactuator_rmd" for configuration ""
set_property(TARGET myactuator_rmd::myactuator_rmd APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(myactuator_rmd::myactuator_rmd PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libmyactuator_rmd.so"
  IMPORTED_SONAME_NOCONFIG "libmyactuator_rmd.so"
  )

list(APPEND _cmake_import_check_targets myactuator_rmd::myactuator_rmd )
list(APPEND _cmake_import_check_files_for_myactuator_rmd::myactuator_rmd "${_IMPORT_PREFIX}/lib/libmyactuator_rmd.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
