#[[.rst:
BBuildEnvBootstrap
------------------

This module provides out-of-tree checkout and update support for CMakeBuild.  

How to Use
^^^^^^^^^^

Copy ``BBuildEnvBootstrap.cmake`` and ``BBuildEnvSvn.cmake`` to ``${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/cmake/bootstrap``
and add the following section to the top of the top-level CMakeLists.txt.

.. code-block:: cmake

  include( ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/cmake/bootstrap/BBuildEnvBootstrap.cmake )
  bb_svn_checkout_cmakebuild( ${CMAKE_SOURCE_DIR}/../CMakeBuild-3.10.2-4 "https://bslinux3.hhi.fraunhofer.de/svn/svn_CMakeBuild/tags/3.10.2-4" )


#]]

if( CMAKE_VERSION VERSION_GREATER_EQUAL 3.10 )
  include_guard( GLOBAL )
endif()


set( BBuildEnvBootstrap_VERSION 3.10.2.4 )
message( STATUS "Loading BBuildEnvBootstrap ${BBuildEnvBootstrap_VERSION}" )


macro( bb_svn_checkout_cmakebuild checkout_dir_ svn_url_ )
  
  option( CHECKOUT_CMAKEBUILD_ONLY "checkout CMakeBuild only" OFF )
  
  set( _cmakebuild_update_script "${CMAKE_BINARY_DIR}/ExtSrcDirs/CMakeBuild-svnupdate.cmake" )
  _bb_write_svn_co_cmakebuild_script( "${_cmakebuild_update_script}" ${checkout_dir_} ${svn_url_} ${ARGN} )

  execute_process(
     COMMAND ${CMAKE_COMMAND} -P ${_cmakebuild_update_script}
     RESULT_VARIABLE error_code
  )    
  if( error_code )
    message( FATAL_ERROR "Failed to checkout or update: '${checkout_dir_}'" )
  endif()

  if( CHECKOUT_CMAKEBUILD_ONLY )
    return()
  endif()     
endmacro()


macro( bb_git_checkout_cmakebuild checkout_dir_ git_tag_ )
  
  #message( STATUS "bb_git_checkout_cmakebuild(): entering: ${ARGV}" )
  
  option( CHECKOUT_CMAKEBUILD_ONLY "checkout CMakeBuild only" OFF )
  
  if( NOT Git_FOUND )
    find_package( Git MODULE REQUIRED )
  endif()
  set( _cmakebuild_update_script "${CMAKE_BINARY_DIR}/ExtSrcDirs/CMakeBuild-gitupdate.cmake" )
  _bb_write_git_co_cmakebuild_script( "${_cmakebuild_update_script}" "${GIT_EXECUTABLE}" ${checkout_dir_} ${git_tag_} ${ARGN} )

  execute_process(
     COMMAND ${CMAKE_COMMAND} -P ${_cmakebuild_update_script}
     RESULT_VARIABLE error_code
  )    
  if( error_code )
    message( FATAL_ERROR "Failed to checkout or update: '${checkout_dir_}'" )
  endif()

  #message( STATUS "bb_git_checkout_cmakebuild(): leaving" )

  if( CHECKOUT_CMAKEBUILD_ONLY )
    return()
  endif()     
endmacro()


function( _bb_write_svn_co_cmakebuild_script script_filename_ checkout_dir_  svn_url_  )

  set( _optional_args ${ARGN} )

  get_filename_component( _script_dir "${script_filename_}" DIRECTORY )
  if( NOT EXISTS "${_script_dir}" )
    file( MAKE_DIRECTORY "${_script_dir}" )
  endif()

  set( _svn_cmake_script "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/cmake/bootstrap/BBuildEnvSvn.cmake" )
  if( NOT EXISTS "${_svn_cmake_script}" )
    message( FATAL_ERROR "${_svn_cmake_script} does not exist. \nYou may need to update ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/cmake/bootstrap" )
  endif()

  file( WRITE ${script_filename_}
"# Written by _bb_write_svn_co_cmakebuild_script ${BBuildEnvBootstrap_VERSION}
cmake_policy( VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} )
if( NOT Subversion_FOUND )
  find_package( Subversion QUIET MODULE REQUIRED )
endif()
include( \"${_svn_cmake_script}\" )

bb_svn_co_external_dir_helper( \"${checkout_dir_}\" ${svn_url_} ${_optional_args} )
" )
  
endfunction()


function( _bb_write_git_co_cmakebuild_script script_filename_ git_EXECUTABLE_ checkout_dir_  git_tag_  )

  set( _optional_args ${ARGN} )

  get_filename_component( _script_dir "${script_filename_}" DIRECTORY )
  if( NOT EXISTS "${_script_dir}" )
    file( MAKE_DIRECTORY "${_script_dir}" )
  endif()

  set( _git_cmake_script "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/cmake/bootstrap/BBuildEnvGit.cmake" )
  if( NOT EXISTS "${_git_cmake_script}" )
    message( FATAL_ERROR "${_git_cmake_script} does not exist. \nYou may need to update ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/cmake/bootstrap" )
  endif()

  file( WRITE ${script_filename_}
"# Written by _bb_write_git_co_cmakebuild_script ${BBuildEnvBootstrap_VERSION}
cmake_policy( VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} )
include( \"${_git_cmake_script}\" )

bb_git_co_external_dir_helper( \"${git_EXECUTABLE_}\" \"${checkout_dir_}\" ${git_tag_} ${_optional_args} )
" )
  
endfunction()


