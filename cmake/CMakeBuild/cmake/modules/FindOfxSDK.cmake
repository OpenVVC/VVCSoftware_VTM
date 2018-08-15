#.rst:
# FindOfxSDK
# ----------
#
# This module finds headers and requested component libraries associated with 
# an OFX SDK.
#
# OfxSDK_DIR
#  Optional external hint to locate an OfxSDK. If defined it will replace the
#  default SDK search path and must be set before find_package() is called.
#
# OfxSDK_ROOT_DIR
# 
# OfxSDK::API
#  Target to expose the OFX API header files.
#
# OfxSDK_VERSION
#  OFX API version; e.g. 1.3
#
# OFX_PLUGIN_PATH
#  OFX plugin install path
#

set( _ofxsdk_super_root "${CMAKE_SOURCE_DIR}/OfxSDK/include/OfxSDK" )

# Just to show the support for multiple versions side-by-side.
set( _ofx_api_version_list 1.4 1.3 )

set( _ofxsdk_required_vars OfxSDK_ROOT_DIR OfxSDK_INCLUDE_DIR )


# Usage:
#  OFX_ADD_PLUGIN( OfxPlugin1 src_file1.cpp src_file2.cpp ... )
#
macro( OFX_ADD_PLUGIN name_ src_file_ )
  # define a prefix for temporary variables to make name clashes unlikely.
  set( _prfx "_ofx_add_plugin_" )
  set( ${_prfx}src_files ${src_file_} ${ARGN} )

  # create a shared library/module
  add_library( ${name_} MODULE ${${_prfx}src_files} )

  if( WIN32 )
    if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
      set( ${_prfx}platform_dir "Win64" )
    else()
      set( ${_prfx}platform_dir "Win32" )
    endif()
  elseif( APPLE )
    set( ${_prfx}platform_dir "MacOS" )
  elseif( UNIX )
    # Linux 64 bit assumed
    set( ${_prfx}platform_dir "Linux-x86-64" )
  else()
    message( FATAL_ERROR "Unsupported platform, please contact technical support." )
  endif()

  set_target_properties( ${name_} PROPERTIES PREFIX "" 
                                             SUFFIX ".ofx"
                                             LIBRARY_OUTPUT_DIRECTORY_DEBUG          ${OFX_PLUGIN_PATH}/${name_}.ofx.bundle/Contents/${${_prfx}platform_dir}
                                             LIBRARY_OUTPUT_DIRECTORY_RELEASE        ${OFX_PLUGIN_PATH}/${name_}.ofx.bundle/Contents/${${_prfx}platform_dir} 
                                             LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO ${OFX_PLUGIN_PATH}/${name_}.ofx.bundle/Contents/${${_prfx}platform_dir}
                                             LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL     ${OFX_PLUGIN_PATH}/${name_}.ofx.bundle/Contents/${${_prfx}platform_dir} )
  # remove temporary variables.                                            
  unset( ${_prfx}src_files )                                             
  unset( ${_prfx}platform_dir )
endmacro( OFX_ADD_PLUGIN )


function( _OfxSDK_getPluginInstallPath plugin_install_path_ )
  message( STATUS "_OfxSDK_getPluginInstallPath: entering." )
  if( DEFINED ENV{OFX_PLUGIN_PATH} )
    set( ${plugin_install_path_} "$ENV{OFX_PLUGIN_PATH}" PARENT_SCOPE )
  else()
    set( ${plugin_install_path_} "${CMAKE_SOURCE_DIR}/ofx-plugins" PARENT_SCOPE )
    #message( STATUS "environment variable OFX_PLUGIN_PATH does not exist, using plugin_path=${OFX_PLUGIN_PATH}" )
  endif()
endfunction()


function( _OfxSDK_findRootDir root_dir_ )
  unset( _root_dir )
  if( DEFINED OfxSDK_DIR )
    if( NOT EXISTS ${OfxSDK_DIR} )
      message( FATAL_ERROR "OfxSDK_DIR=${OfxSDK_DIR} does not exist." )
    else()
      set( _root_dir "${OfxSDK_DIR}" )
    endif()
  else()    
    if( EXISTS ${_ofxsdk_super_root} )
      if( OfxSDK_FIND_VERSION_EXACT )
        message( STATUS "FindOfxSDK: searching for ${OfxSDK_FIND_VERSION} exactly." ) 
        if( EXISTS "${_ofxsdk_super_root}/OFX_V${OfxSDK_FIND_VERSION}_SDK" )
          set( _root_dir "${_ofxsdk_super_root}/OFX_V${OfxSDK_FIND_VERSION}_SDK" )
        endif()
      else()
        foreach( _v ${_ofx_api_version_list} )
          if( EXISTS "${_ofxsdk_super_root}/OFX_V${_v}_API" )
            set( _root_dir "${_ofxsdk_super_root}/OFX_V${_v}_API" )
          endif()      
          if( DEFINED _root_dir )
            break()
          endif()        
        endforeach()
      endif()  
    endif()
  endif()
  if( DEFINED _root_dir )
    set( ${root_dir_} ${_root_dir} PARENT_SCOPE )
  else()
    set( ${root_dir_} ${root_dir_}-NOTFOUND PARENT_SCOPE )
  endif()
endfunction()

function( _OfxSDK_getVersion sdk_version_ sdk_root_dir_ sdk_include_dir_ )
  get_filename_component( _ofx_sdk_dir_name ${sdk_include_dir_} NAME )
  if( _ofx_sdk_dir_name MATCHES "^OFX_V([0-9.]+)_API$" )
    # message( STATUS "_OfxSDK_getVersion(): deriving version from ${_ae_sdk_dir}" )
    set( ${sdk_version_} ${CMAKE_MATCH_1} PARENT_SCOPE )
  else()
    # no way yet to figure out the OFX API version automatically given the set of include files.
    set( ${sdk_version_} 1.3 PARENT_SCOPE )
  endif()
endfunction()


if( DEFINED OfxSDK_INCLUDE_DIR )
  message( FATAL_ERROR "FindOfxSDK: OfxSDK_INCLUDE_DIR already defined -> unexpected." )
endif()

_OfxSDK_findRootDir( OfxSDK_ROOT_DIR )
if( OfxSDK_ROOT_DIR )
  if( EXISTS "${OfxSDK_ROOT_DIR}/ofxCore.h" )
    set( OfxSDK_INCLUDE_DIR ${OfxSDK_ROOT_DIR} )
    if( OfxSDK_FIND_VERSION_EXACT )
      set( OfxSDK_VERSION ${OfxSDK_FIND_VERSION} )
    else()
      _OfxSDK_getVersion( OfxSDK_VERSION ${OfxSDK_ROOT_DIR} ${OfxSDK_INCLUDE_DIR} )
    endif()
    if( NOT TARGET OfxSDK::API )
      add_library( OfxSDK::API INTERFACE IMPORTED )
      set_target_properties( OfxSDK::API PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${OfxSDK_INCLUDE_DIR}" )
    endif()
    if( NOT DEFINED OFX_PLUGIN_PATH )
      _OfxSDK_getPluginInstallPath( _OFX_PLUGIN_PATH )
      set( OFX_PLUGIN_PATH "${_OFX_PLUGIN_PATH}" CACHE PATH "OFX plugin install path" )
    endif()
  endif()
endif()

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args( OfxSDK REQUIRED_VARS ${_ofxsdk_required_vars} VERSION_VAR OfxSDK_VERSION )
