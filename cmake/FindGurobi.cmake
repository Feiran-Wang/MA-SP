# Find the Gurobi C and C++ APIs.
#
# Search priority:
#   1. GUROBI_ROOT CMake cache variable
#   2. GUROBI_HOME environment variable
#   3. /opt/gurobi-current
#   4. Other versioned /opt/gurobi* installations (newest first)
#
# Both an installation prefix (for example /opt/gurobi1302) and its platform
# directory (for example /opt/gurobi1302/linux64) are accepted.
#
# Result variables:
#   Gurobi_FOUND
#   Gurobi_VERSION
#   Gurobi_HOME
#   Gurobi_INCLUDE_DIRS
#   Gurobi_LIBRARIES
#   Gurobi_LIBRARY_DIR
#
# Imported targets:
#   Gurobi::C
#   Gurobi::CXX

set(GUROBI_ROOT "" CACHE PATH "Gurobi installation prefix or platform directory")

# A changed SDK root must not reuse libraries cached from another installation.
unset(Gurobi_CXX_LIBRARY CACHE)
unset(Gurobi_CXX_LIBRARY)
unset(Gurobi_C_LIBRARY CACHE)
unset(Gurobi_C_LIBRARY)

set(_Gurobi_root_hints)
if(GUROBI_ROOT)
  list(APPEND _Gurobi_root_hints "${GUROBI_ROOT}")
endif()
if(DEFINED ENV{GUROBI_HOME} AND NOT "$ENV{GUROBI_HOME}" STREQUAL "")
  list(APPEND _Gurobi_root_hints "$ENV{GUROBI_HOME}")
endif()
list(APPEND _Gurobi_root_hints "/opt/gurobi-current")

file(GLOB _Gurobi_opt_roots LIST_DIRECTORIES true "/opt/gurobi*")
list(SORT _Gurobi_opt_roots COMPARE NATURAL ORDER DESCENDING)
list(APPEND _Gurobi_root_hints ${_Gurobi_opt_roots})
list(REMOVE_DUPLICATES _Gurobi_root_hints)

set(_Gurobi_platform_root)
foreach(_Gurobi_hint IN LISTS _Gurobi_root_hints)
  foreach(_Gurobi_suffix IN ITEMS "" linux64 macos_universal2 mac64 win64)
    if(_Gurobi_suffix)
      set(_Gurobi_candidate "${_Gurobi_hint}/${_Gurobi_suffix}")
    else()
      set(_Gurobi_candidate "${_Gurobi_hint}")
    endif()

    if(EXISTS "${_Gurobi_candidate}/include/gurobi_c++.h")
      set(_Gurobi_platform_root "${_Gurobi_candidate}")
      break()
    endif()
  endforeach()

  if(_Gurobi_platform_root)
    break()
  endif()
endforeach()

if(_Gurobi_platform_root)
  set(Gurobi_HOME "${_Gurobi_platform_root}")
  set(Gurobi_INCLUDE_DIR "${Gurobi_HOME}/include")
  set(Gurobi_LIBRARY_DIR "${Gurobi_HOME}/lib")

  find_library(
    Gurobi_CXX_LIBRARY
    NAMES gurobi_c++
    PATHS "${Gurobi_LIBRARY_DIR}"
    NO_DEFAULT_PATH
  )

  # Gurobi's C API library includes the major/minor ABI in its file name
  # (for example libgurobi130.so). Discover that name instead of hard-coding it.
  file(
    GLOB _Gurobi_c_library_candidates
    LIST_DIRECTORIES false
    "${Gurobi_LIBRARY_DIR}/libgurobi[0-9]*.so"
    "${Gurobi_LIBRARY_DIR}/libgurobi[0-9]*.dylib"
    "${Gurobi_LIBRARY_DIR}/gurobi[0-9]*.lib"
  )
  list(SORT _Gurobi_c_library_candidates COMPARE NATURAL ORDER DESCENDING)

  foreach(_Gurobi_library IN LISTS _Gurobi_c_library_candidates)
    get_filename_component(_Gurobi_library_name "${_Gurobi_library}" NAME)
    if(
      _Gurobi_library_name MATCHES "^libgurobi[0-9]+\\.(so|dylib)$"
      OR _Gurobi_library_name MATCHES "^gurobi[0-9]+\\.lib$"
    )
      set(Gurobi_C_LIBRARY "${_Gurobi_library}")
      break()
    endif()
  endforeach()

  if(NOT Gurobi_C_LIBRARY)
    find_library(
      Gurobi_C_LIBRARY
      NAMES gurobi
      PATHS "${Gurobi_LIBRARY_DIR}"
      NO_DEFAULT_PATH
    )
  endif()

  set(_Gurobi_version_header "${Gurobi_INCLUDE_DIR}/gurobi_c.h")
  if(EXISTS "${_Gurobi_version_header}")
    file(
      STRINGS "${_Gurobi_version_header}" _Gurobi_version_lines
      REGEX "^#define GRB_VERSION_(MAJOR|MINOR|TECHNICAL)[ \t]+[0-9]+"
    )
    foreach(_Gurobi_version_line IN LISTS _Gurobi_version_lines)
      if(_Gurobi_version_line MATCHES "GRB_VERSION_MAJOR[ \t]+([0-9]+)")
        set(_Gurobi_version_major "${CMAKE_MATCH_1}")
      elseif(_Gurobi_version_line MATCHES "GRB_VERSION_MINOR[ \t]+([0-9]+)")
        set(_Gurobi_version_minor "${CMAKE_MATCH_1}")
      elseif(_Gurobi_version_line MATCHES "GRB_VERSION_TECHNICAL[ \t]+([0-9]+)")
        set(_Gurobi_version_patch "${CMAKE_MATCH_1}")
      endif()
    endforeach()

    if(
      DEFINED _Gurobi_version_major
      AND DEFINED _Gurobi_version_minor
      AND DEFINED _Gurobi_version_patch
    )
      set(
        Gurobi_VERSION
        "${_Gurobi_version_major}.${_Gurobi_version_minor}.${_Gurobi_version_patch}"
      )
    endif()
  endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  Gurobi
  REQUIRED_VARS
    Gurobi_INCLUDE_DIR
    Gurobi_C_LIBRARY
    Gurobi_CXX_LIBRARY
  VERSION_VAR Gurobi_VERSION
)

if(Gurobi_FOUND)
  set(Gurobi_INCLUDE_DIRS "${Gurobi_INCLUDE_DIR}")
  # Ordering matters for static libgurobi_c++: the C++ wrapper must precede
  # the versioned Gurobi C/shared library on the final linker command.
  set(Gurobi_LIBRARIES "${Gurobi_CXX_LIBRARY};${Gurobi_C_LIBRARY}")

  if(NOT TARGET Gurobi::C)
    if(Gurobi_C_LIBRARY MATCHES "\\.(so|dylib)$")
      add_library(Gurobi::C SHARED IMPORTED)
    else()
      add_library(Gurobi::C UNKNOWN IMPORTED)
    endif()
    set_target_properties(
      Gurobi::C
      PROPERTIES
        IMPORTED_LOCATION "${Gurobi_C_LIBRARY}"
        # Gurobi's Linux shared library does not publish a DT_SONAME. Ask
        # CMake to link by -l<name> so the executable records a relocatable
        # NEEDED entry instead of the discovery machine's absolute path.
        IMPORTED_NO_SONAME TRUE
        INTERFACE_INCLUDE_DIRECTORIES "${Gurobi_INCLUDE_DIR}"
    )
  endif()

  if(NOT TARGET Gurobi::CXX)
    add_library(Gurobi::CXX UNKNOWN IMPORTED)
    set_target_properties(
      Gurobi::CXX
      PROPERTIES
        IMPORTED_LOCATION "${Gurobi_CXX_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${Gurobi_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES Gurobi::C
    )
  endif()
endif()

mark_as_advanced(
  Gurobi_INCLUDE_DIR
  Gurobi_C_LIBRARY
  Gurobi_CXX_LIBRARY
)
