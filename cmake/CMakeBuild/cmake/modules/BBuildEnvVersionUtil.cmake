#[[.rst:
BBuildEnvVersionUtil
--------------------

Version file parsing utilities and other version related functions and macros. 


#]]

function( bb_get_version_from_h_file version_ )
  
  # set default values for optional arguments
  set( _version_fmt "STD" )
  set( _header_file "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/include/${PROJECT_NAME}/version.h" )
  
  if( ${ARGC} GREATER 3 )
    message( FATAL_ERROR "bb_get_version_from_h_file() called with too many arguments." )
  elseif( ${ARGC} EQUAL 3 )
    get_filename_component( _header_file "${ARGV1}" ABSOLUTE )
    # Check for optional arguments: ONLY_DOTS -> replace "_" and "-" with "."
    # message( "bb_get_version_from_h_file(): ARGC=${ARGC} ARGV2=${ARGV2}" )
    set( _version_fmt "${ARGV2}" )
    #message( STATUS "bb_get_version_from_h_file -> ${_version_fmt}" )
    if( NOT _version_fmt MATCHES "^(ONLY_DOTS|CMAKE_VERSION_FMT)$" ) 
      message( FATAL_ERROR "bb_get_version_from_h_file() called with an illegal argument ${_version_fmt}" )
    endif()
  elseif( ${ARGC} EQUAL 2 )
    # The second argument is either a filename or a format option
    if( "${ARGV1}" MATCHES "^(ONLY_DOTS|CMAKE_VERSION_FMT)$" )
      set( _version_fmt "${ARGV1}" )
    else()
      get_filename_component( _header_file "${ARGV1}" ABSOLUTE )
    endif()
  endif()
  
  if( NOT EXISTS ${_header_file} )
    message( FATAL_ERROR "bb_get_version_from_h_file(): input file ${_header_file} does not exist." )
  endif()
  
  #
  # h_file: its directory in capital letters is the expected version prefix.
  # example: <path>/BoostBuildTest/version.h -> prefix=BOOSTBUILDTEST_
  # #if !defined( BOOSTBUILDTEST_VERSION )
  # #define BOOSTBUILDTEST_VERSION "1.60.0-9"
  # ...
  # The version expression may either be a quoted string or a sequence of digits.
  get_filename_component( _dir ${_header_file} DIRECTORY )
  get_filename_component( _parent_dir ${_dir} NAME )

  #message( STATUS "bb_get_version_from_h_file(): _dir=${_dir} _parent_dir=${_parent_dir}" )
  
  string( TOUPPER ${_parent_dir} _version_prefix )
  
  set( _regex_version_def "^[\t ]*#[\t ]*define[\t ]+${_version_prefix}_VERSION[\t ]+" )
  
  unset( _version )
  file( STRINGS "${_header_file}" _header_version_line REGEX "${_regex_version_def}" )
  #message( STATUS "version line: '${_header_version_line}'" )
  if( "${_header_version_line}" MATCHES "${_regex_version_def}\"([0-9._-]+[0-9]).*\"" )
    #message( STATUS "extracted version ID: ${CMAKE_MATCH_1}" )
    set( _version "${CMAKE_MATCH_1}" )
  elseif( "${_header_version_line}" MATCHES "${_regex_version_def}([0-9]+)[\t ]*" )
    #message( STATUS "extracted version ID: ${CMAKE_MATCH_1}" )
    set( _version "${CMAKE_MATCH_1}" )
  elseif( "${_header_version_line}" MATCHES "${_regex_version_def}0x([0-9a-f]+)[\t ]*" )
    # hex numbers shall be used with care, CMake has no builtin support for them.
    #message( STATUS "extracted version ID: ${CMAKE_MATCH_1}" )
    set( _version "${CMAKE_MATCH_1}" )    
  endif()
  if( NOT DEFINED _version )
    message( FATAL_ERROR "${_header_file} does not seem to contain a supported version expression." )
    return()
  endif()
  if( _version_fmt MATCHES "^(ONLY_DOTS|CMAKE_VERSION_FMT)$" )
    string( REGEX REPLACE "[_-]" "." _version "${_version}" )
  endif()
  if( _version_fmt STREQUAL "CMAKE_VERSION_FMT" )
    bb_to_cmake_version_format( ${_version} _version )
  endif()     
  set( ${version_} "${_version}" PARENT_SCOPE )
endfunction( bb_get_version_from_h_file )

# cmake version expressions are limited to <major>[.<minor>[.<patch>[.<tweak>]]].
function( bb_to_cmake_version_format version_in_ version_out_ )
  string( REGEX REPLACE "[-_.]" ";" _version_list "${version_in_}" )
  list( LENGTH _version_list _version_len )
  while( _version_len GREATER 4 )
    list( REMOVE_AT _version_list -1 )
    list( LENGTH _version_list _version_len )
  endwhile()
  string( REPLACE ";" "."  _version "${_version_list}" )
  set( ${version_out_} "${_version}" PARENT_SCOPE )
endfunction()

macro( bb_version_split version_in_ version_out_ )
  string( REGEX REPLACE "[-_.]" ";" ${version_out_} "${version_in_}" )
endmacro()

function( bb_to_rpm_version version_ rpm_version_ rpm_release_ )
  if( ${version_} MATCHES "^([0-9._]+)[-._]([0-9]+)" )
    set( ${rpm_version_} "${CMAKE_MATCH_1}" PARENT_SCOPE )
    set( ${rpm_release_} "${CMAKE_MATCH_2}" PARENT_SCOPE )
  else()
    message( FATAL_ERROR "bb_to_rpm_version(): version ${version_} specification not supported." )
  endif()  
endfunction()


function( bb_version_list_sort version_list_sorted_ version1_ )
  unset( _version_list_sorted )
  set( _version_list_in ${version1_} ${ARGN} )
  
  list( LENGTH _version_list_in _version_len )
  while( _version_len GREATER 0 )
    list( GET _version_list_in 0 _version_cur )
    foreach( _v IN LISTS  _version_list_in )
      if( ${_v} VERSION_GREATER ${_version_cur} )
        set( _version_cur ${_v} )
      endif()
    endforeach()
    # Note: list( REMOVE_ITEM ...) would remove all list elements of value ${_version_cur} which may 
    # be confusing if the list contains duplicate elements.       
    # list( REMOVE_ITEM _version_list_in ${_version_cur} )
    list( FIND _version_list_in ${_version_cur}  _list_pos )
    if( NOT _list_pos EQUAL -1 )
      list( REMOVE_AT _version_list_in ${_list_pos} )
    endif()
    
    list( APPEND _version_list_sorted ${_version_cur} )
    list( LENGTH _version_list_in _version_len )
  endwhile()
  set( ${version_list_sorted_} ${_version_list_sorted} PARENT_SCOPE )
endfunction()

