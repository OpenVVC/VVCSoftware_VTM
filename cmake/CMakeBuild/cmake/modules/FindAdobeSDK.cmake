#.rst:
# FindAdobeSDK
# ------------
#
# This module finds headers and plugin installation paths associated with 
# an Adobe After Effects SDK.
#
# Default search path:
#   ${CMAKE_SOURCE_DIR}/AdobeSDKs/include/AdobeSDKs/AE_CC<version>_SDK
#   ${CMAKE_SOURCE_DIR}/AdobeSDKs/include/AdobeSDKs/AE_CS<version>_SDK
#
# AdobeSDK_DEBUG
#  Optional boolean variable to enable additional trace messages.
#
# AdobeSDK_DIR
#  Optional external hint to locate an AdobeSDK. If defined it will replace the
#  default SDK search path and must be set before find_package() is called.
#
# AdobeSDK_USE_COMMON_PLUGIN_INSTALL_PATH
#  Optional boolean value to switch between After Effects plugin path and a common plugin path.
# 
# AdobeSDK_PLUGIN_INSTALL_PATH_SUFFIX
#  Optional subdirectory to be appended to the plugin install path. It must be set before
#  find_package() is called.
#
# AdobeSDK_ROOT_DIR
#  Top level directory of the selected AdobeSDK.
#
# AdobeSDK_INCLUDE_DIR
#  Top level include directory of the selected Adobe SDK.
#
# AdobeSDK_INCLUDE_DIRS
#  Top level include directory of the selected Adobe SDK and its subdirectory SP.
#
# AdobeSDK_UTIL_DIR
#   Contains the path to the Util directory inside the AdobeSDK.
#
# AdobeSDK_RESOURCE_DIR
#   Contains the path to the Resources directory inside the AdobeSDK.
#
# AdobeSDK_VERSION
#  Adobe SDK version; e.g. 5.5
#
# Macro to add an AE plugin: 
#  ADOBESDK_ADD_AE_PLUGIN( <name> src_file1.cpp ... [file.rc] [file.r] [PIPL_EXTRA_INCLUDE_DIRS dir1 ...] )
#  ADOBESDK_ADD_AE_PLUGIN( <name> src_file1.cpp ... [file.rc] [file.r] )
#

# svn_AdobeSDKs stores a subset of different SDKs side-by-side at <_adobesdk_super_root>.
set( _adobesdk_super_root "${CMAKE_SOURCE_DIR}/AdobeSDKs/include/AdobeSDKs" )

# Just to show the support for multiple versions side-by-side.
# Use find_package( AdobeSDK 5.5 EXACT MODULE REQUIRED ) to tell the finder
# to search for 5.5 only.
set( _adobesdk_version_list 2017 6 5.5 5 4 )

# Source: Adobe After Effects CC 2017 SDK Guide, Table 2: API Versions
set( _adobe_ae_version_list
     #                 Release         Effect API Version (AE_EffectVers.h)
     14.0              # CC 2017       13.13
     13.8              # CC 2015.3     13.11
     13.7              # CC 2015       13.10
     13.6              # CC 2015       13.10
     13.5 13.5.1       # CC 2015       13.9
     13.2 13.1 13.0    # CC 2014       13.7
     12.2              # CC 12.2       13.6
     12.1              # CC 12.1       13.5
     12.0              # CC 12.0       13.4
     11.0.1            # CS6.0.1       13.3
     11.0 11           # CS6           13.2
     10.5              # CS5.5         13.1
     10.0              # CS5           13.0
    )
     
# map: api version -> sdk version
set( _adobesdk_api_sdk_version_map
      v13.14           2017 
      v13.13           2017      
      v13.11           2015.3    
      v13.10           2015      
      v13.9            2015      
      v13.7            2014      
      v13.6            12.2      
      v13.5            12.1      
      v13.4            12.0      
      v13.3            6.0.1     
      v13.2            6         
      v13.1            5.5       
      v13.0            5         
    )     
     
# Fallback plugin path if no other installation information or hints are available.
set( _ae_plugin_install_path_fallback "${CMAKE_SOURCE_DIR}/ae-plugins" )    
     
set( _adobesdk_required_vars AdobeSDK_ROOT_DIR AdobeSDK_INCLUDE_DIR AdobeSDK_UTIL_DIR AdobeSDK_RESOURCE_DIR )
if( WIN32 )
  list( APPEND _adobesdk_required_vars AdobeSDK_PIPL_TOOL )
endif()

# Usage:
#   ADOBESDK_ADD_AE_PLUGIN( SamplePlugin16Bit src_file1.cpp ... [file.rc] [file.r] [PIPL_EXTRA_INCLUDE_DIRS dir1 ...] )
#   ADOBESDK_ADD_AE_PLUGIN( SamplePlugin16Bit src_file1.cpp ... [file.rc] [file.r] )
#
macro( ADOBESDK_ADD_AE_PLUGIN name_ src_file_ )
  set( _prfx "_add_ae_plugin_" )
  set( ${_prfx}tmp_vars ${_prfx}opt_args ${_prfx}src_files ${_prfx}pipl_src_file ${_prfx}pipl_src_fname ${_prfx}pipl_extra_include_dirs ${_prfx}pipl_include_dirs )
  
  set( ${_prfx}opt_args ${src_file_} ${ARGN} )

  _AdobeSDK_addAePluginHandleArgs( ${_prfx}src_files ${_prfx}pipl_src_file ${_prfx}pipl_extra_include_dirs ${${_prfx}opt_args} )
  
  #message( STATUS "ADOBESDK_ADD_AE_PLUGIN: src_files=${${_prfx}src_files}" )
  #message( STATUS "ADOBESDK_ADD_AE_PLUGIN: pipl_src_file=${${_prfx}pipl_src_file}" )
  #message( STATUS "ADOBESDK_ADD_AE_PLUGIN: pipl_extra_include_dirs=${${_prfx}pipl_extra_include_dirs}" )

  if( MSVC )
    #message( STATUS "ADOBESDK_ADD_AE_PLUGIN: pipl_file=${${_prfx}pipl_src_file}" )
    #message( STATUS "ADOBESDK_ADD_AE_PLUGIN: pipl_fname=${${_prfx}pipl_src_fname}" )
    set( ${_prfx}pipl_include_dirs "/I \"${AdobeSDK_INCLUDE_DIR}\"" )
    if( DEFINED ${_prfx}pipl_extra_include_dirs )
      foreach( _fpath ${${_prfx}pipl_extra_include_dirs} )
        if( IS_ABSOLUTE ${_fpath} )
          list( APPEND ${_prfx}pipl_include_dirs "/I \"${_fpath}\"" )
        else()
          list( APPEND ${_prfx}pipl_include_dirs "/I \"${CMAKE_CURRENT_SOURCE_DIR}/${_fpath}\"" )
        endif()
      endforeach()
    endif()  
    #message( STATUS "ADOBESDK_ADD_AE_PLUGIN: pipl includes: ${${_prfx}pipl_include_dirs}" )

    # get pipl basename without extension
    get_filename_component( ${_prfx}pipl_src_fname ${${_prfx}pipl_src_file} NAME_WE )
    
    add_custom_command( OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${${_prfx}pipl_src_fname}.rc 
                        COMMAND cl.exe ${${_prfx}pipl_include_dirs} /EP ${${_prfx}pipl_src_file} > ${${_prfx}pipl_src_fname}.rr 
                        COMMAND ${AdobeSDK_PIPL_TOOL} ${CMAKE_CURRENT_BINARY_DIR}/${${_prfx}pipl_src_fname}.rr ${CMAKE_CURRENT_BINARY_DIR}/${${_prfx}pipl_src_fname}.rrc
                        COMMAND cl.exe /D MSWindows /EP ${CMAKE_CURRENT_BINARY_DIR}/${${_prfx}pipl_src_fname}.rrc > ${CMAKE_CURRENT_BINARY_DIR}/${${_prfx}pipl_src_fname}.rc
                        MAIN_DEPENDENCY ${${_prfx}pipl_src_file} )    
    
    add_library( ${name_} MODULE ${${_prfx}src_files} ${CMAKE_CURRENT_BINARY_DIR}/${${_prfx}pipl_src_fname}.rc )
    
  elseif( XCODE )
    # xcode is supposed to know how to deal with *.r files.
    add_library( ${name_} MODULE ${${_prfx}src_files} ${${_prfx}pipl_src_file} )
  endif()

  set_target_properties( ${name_} PROPERTIES PREFIX "" 
                                  SUFFIX ".aex"
                                  LIBRARY_OUTPUT_DIRECTORY_DEBUG          ${AdobeSDK_PLUGIN_INSTALL_PATH}
                                  LIBRARY_OUTPUT_DIRECTORY_RELEASE        ${AdobeSDK_PLUGIN_INSTALL_PATH}
                                  LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO ${AdobeSDK_PLUGIN_INSTALL_PATH}
                                  LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL     ${AdobeSDK_PLUGIN_INSTALL_PATH} )

  # get rid of temporary variables
  foreach( _var ${${_prfx}tmp_vars} )
    if( DEFINED ${_var} )
      #message( STATUS "ADOBESDK_ADD_AE_PLUGIN: disposing var=${_var}" )
      unset( ${_var} )
    endif()
  endforeach()
  unset( _prfx )
endmacro()

# 
function( _AdobeSDK_addAePluginHandleArgs src_files_ pipl_src_file_ pipl_extra_include_dirs_ )  
  set( PIPL_EXTRA_INCLUDE_DIRS_FOUND OFF )
  unset( _pipl_include_dirs )
  unset( _pipl_src_file )
  unset( _src_files )
  
  foreach( _arg ${ARGN} )
    #message( STATUS "_AdobeSDK_addAePluginHandleArgs: processing ${_arg}" )
    if( ${_arg} STREQUAL "PIPL_EXTRA_INCLUDE_DIRS" ) 
      set( PIPL_EXTRA_INCLUDE_DIRS_FOUND ON )
      #message( STATUS "ADOBESDK_ADD_AE_PLUGIN: PIPL_EXTRA_INCLUDE_DIRS found" )
      continue()
    endif()
    if( PIPL_EXTRA_INCLUDE_DIRS_FOUND )
      #message( STATUS "_AdobeSDK_addAePluginHandleArgs: processing pipl include dir=${_arg}" )
      if( IS_ABSOLUTE ${_arg} )
        list( APPEND _pipl_include_dirs ${_arg} )
      else()
        list( APPEND _pipl_include_dirs ${CMAKE_CURRENT_SOURCE_DIR}/${_arg} )
      endif()
    else()
      get_filename_component( _fname_ext ${_arg} EXT )
      if( _fname_ext STREQUAL ".r" )
        if( WIN32 )
          if( IS_ABSOLUTE ${_arg} )
            set( _pipl_src_file ${_arg} )
          else()
            get_filename_component( _pipl_src_file ${CMAKE_CURRENT_SOURCE_DIR}/${_arg} ABSOLUTE )
          endif()
        elseif( APPLE )
          set( _pipl_src_file ${_arg} )
        endif()
      else()
        list( APPEND _src_files ${_arg} )
      endif()
    endif()    
  endforeach()
  if( DEFINED _src_files )
    set( ${src_files_} ${_src_files} PARENT_SCOPE )
  endif()
  if( DEFINED _pipl_src_file )
    set( ${pipl_src_file_} ${_pipl_src_file} PARENT_SCOPE )
  endif()  
  if( DEFINED _pipl_include_dirs )
    set( ${pipl_extra_include_dirs_} ${_pipl_include_dirs} PARENT_SCOPE )
  endif()
endfunction()

function( _AdobeSDK_findRootDir root_dir_ )
  unset( _root_dir )
  if( DEFINED AdobeSDK_DIR )
    if( NOT EXISTS ${AdobeSDK_DIR} )
      message( FATAL_ERROR "AdobeSDK_DIR=${AdobeSDK_DIR} does not exist." )
    else()
      set( _root_dir "${AdobeSDK_DIR}" )
    endif()
  else()    
    if( EXISTS ${_adobesdk_super_root} )
      if( AdobeSDK_FIND_VERSION_EXACT )
        #message( STATUS "FindAdobeSDK: searching for ${AdobeSDK_FIND_VERSION} exactly." ) 
        if( EXISTS "${_adobesdk_super_root}/AE_CC${AdobeSDK_FIND_VERSION}_SDK" )
          set( _root_dir "${_adobesdk_super_root}/AE_CC${AdobeSDK_FIND_VERSION}_SDK" )
        elseif( EXISTS "${_adobesdk_super_root}/AE_CS${AdobeSDK_FIND_VERSION}_SDK" )
          set( _root_dir "${_adobesdk_super_root}/AE_CS${AdobeSDK_FIND_VERSION}_SDK" )
        endif()
      else()
        foreach( _v ${_adobesdk_version_list} )
          if( EXISTS "${_adobesdk_super_root}/AE_CC${_v}_SDK" )
            set( _root_dir "${_adobesdk_super_root}/AE_CC${_v}_SDK" )
          elseif( EXISTS "${_adobesdk_super_root}/AE_CS${_v}_SDK" )
            set( _root_dir "${_adobesdk_super_root}/AE_CS${_v}_SDK" )
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

function( _AdobeSDK_getAeApiVersion ae_api_version_ header_file_ )
  unset( _version_list )
  set( _regex_version_def "^#define[\t ]+(PF_PLUG_IN_VERSION|PF_PLUG_IN_SUBVERS)[\t ]+([0-9]+)" )
  file( STRINGS "${header_file_}" _header_version_lines REGEX "${_regex_version_def}" )
  # message( STATUS "extracted header lines: ${_header_version_lines}" )
  foreach( _line ${_header_version_lines} )
    # message( STATUS "processing header line: ${_line}" )
    if( _line MATCHES "${_regex_version_def}" )
      # message( STATUS "${CMAKE_MATCH_1} -> ${CMAKE_MATCH_2}" )
      list( APPEND _version_list "${CMAKE_MATCH_2}" )
    endif()
  endforeach()
  #message( STATUS "_AdobeSDK_getAeApiVersion: version ${_version_list}" )
  if( DEFINED _version_list )
    # Compose a doted separated version string
    string( REPLACE ";" "." _version_str "${_version_list}" )
  else()
    message( FATAL_ERROR "_AdobeSDK_getAeApiVersion() failed to extract version information, please contact the maintainer for further support." )
  endif()
  set( ${ae_api_version_} "${_version_str}" PARENT_SCOPE )
endfunction()

function( _AdobeSDK_getSdkVersion sdk_version_ sdk_root_dir_ sdk_include_dir_ )
  # Try to match sdk_root_dir_ to AE_CC<VERSION>_SDK or AE_CS<VERSION>_SDK.
  get_filename_component( _ae_sdk_dir ${sdk_root_dir_} NAME )
  if( _ae_sdk_dir MATCHES "^AE_C[SC]([0-9.]+)_SDK$" )
    # message( STATUS "_AdobeSDK_getSdkVersion(): deriving version from ${_ae_sdk_dir}" )
    set( ${sdk_version_} ${CMAKE_MATCH_1} PARENT_SCOPE )
  else()
    _AdobeSDK_getAeApiVersion( _ae_api_version "${sdk_include_dir_}/AE_EffectVers.h" )
    #message( STATUS "FindAdobeSDK: API VERSION=${_api_version}" )
    #message( STATUS "FindAdobeSDK: API VERSION MAP=${_adobesdk_api_sdk_version_map}" )
    list( FIND _adobesdk_api_sdk_version_map v${_ae_api_version} _index )
    if( _index EQUAL -1 )
      message( FATAL_ERROR "_AdobeSDK_getSdkVersion(): no mapping from AE API ${_ae_api_version} -> SDK version defined, please contact technical support." )
    else()
      math( EXPR _index_next "${_index} + 1" )
      #message( STATUS "FindAdobeSDK: API VERSION=${_api_version} -> ${_index_next}" )
      list( GET _adobesdk_api_sdk_version_map ${_index_next} _sdk_version )
      set( ${sdk_version_} ${_sdk_version} PARENT_SCOPE )
    endif()
  endif()
endfunction()

function( _AdobeSDK_findPiplTool pipl_tool_ sdk_root_dir_ )
  if( EXISTS "${sdk_root_dir_}/Resources/PiPLtool.exe" )
    set( _pipl_tool "${sdk_root_dir_}/Resources/PiPLtool.exe" )
  else()
    get_filename_component( _sdk_dir ${sdk_root_dir_} NAME )
    if( EXISTS "${sdk_root_dir_}/../../../bin/${_sdk_dir}/PiPLtool.exe" )
      get_filename_component( _pipl_tool "${sdk_root_dir_}/../../../bin/${_sdk_dir}/PiPLtool.exe" REALPATH )
    endif()
  endif()
  if( DEFINED _pipl_tool )
    set( ${pipl_tool_} ${_pipl_tool} PARENT_SCOPE )
  endif()
endfunction()

function( _AdobeSDK_getPluginInstallPath plugin_install_path_ )
  #message( STATUS "_AdobeSDK_getPluginInstallPath: entering." )
  if( WIN32 )
    if( AdobeSDK_USE_COMMON_PLUGIN_INSTALL_PATH )
      set( _reg_sz_value CommonPluginInstallPath )
    else()
      set( _reg_sz_value PluginInstallPath )      
    endif()
    foreach( _v ${_adobe_ae_version_list} )
      set( _plugin_install_path "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Adobe\\After Effects\\${_v};${_reg_sz_value}]" )
      get_filename_component( _ae_plugin_install_path ${_plugin_install_path} ABSOLUTE )
      if( AdobeSDK_DEBUG )
        message( STATUS "_AdobeSDK_getPluginInstallPath: processing registry path ${_plugin_install_path}" )
        if( NOT _ae_plugin_install_path STREQUAL "/registry" ) 
          message( STATUS "_AdobeSDK_getPluginInstallPath: resolved path ${_ae_plugin_install_path}" )
        endif()
      endif()
      if( EXISTS "${_ae_plugin_install_path}" )
        #message( STATUS "_AdobeSDK_getPluginPath: found plugin install path: ${_ae_plugin_install_path}" )
        break()
      else()
        unset( _ae_plugin_install_path )
      endif()
    endforeach()
  elseif( APPLE )
    # N.B. not tested at all
    #
    # CC 2015.3: Developers can find paths to the default location of plug-ins, scripts, and presets on Mac OS X
    # in a new plist file (same as the paths in the Windows registry): /Library/Preferences/com.Adobe.After Effects.paths.plist
    foreach( _v ${_adobe_ae_version_list} )
      set( _ae_plugin_install_path "/Applications/Adobe After Effects ${_v}/Plug-ins" )
      #CC 2017: Version is locked at 7.0 for all CC versions, or CSx for earlier versions.
      #/Library/Application Support/Adobe/Common/Plug-ins/[version]/MediaCore
      if( EXISTS "${_ae_plugin_install_path}" )
        # message( STATUS "_AdobeSDK_getPluginPath: found plugin install path: ${_ae_plugin_install_path}" )
        break()
      else()
        unset( _ae_plugin_install_path )
      endif()      
    endforeach()  
  endif()
  if( NOT DEFINED _ae_plugin_install_path )
    set( _ae_plugin_install_path ${_ae_plugin_install_path_fallback} )
    if( AdobeSDK_DEBUG )
      message( STATUS "_AdobeSDK_getPluginInstallPath: using fallback plugin install path ${_ae_plugin_install_path}" )
    endif()
  endif()
  if( DEFINED AdobeSDK_PLUGIN_INSTALL_PATH_SUFFIX )
    set( _ae_plugin_install_path "${_ae_plugin_install_path}/${AdobeSDK_PLUGIN_INSTALL_PATH_SUFFIX}" )
  endif()
  set( ${plugin_install_path_} "${_ae_plugin_install_path}" PARENT_SCOPE )  
endfunction()


if( DEFINED AdobeSDK_INCLUDE_DIR )
  message( FATAL_ERROR "FindAdobeSDK: AdobeSDK_INCLUDE_DIR already defined -> unexpected." )
endif()

if( MSVC OR XCODE )
  _AdobeSDK_findRootDir( AdobeSDK_ROOT_DIR )
  if( AdobeSDK_ROOT_DIR )
    # message( STATUS "FindAdobeSDK: AdobeSDK_ROOT_DIR=${AdobeSDK_ROOT_DIR}" )
    if( EXISTS "${AdobeSDK_ROOT_DIR}/AE_EffectVers.h" )
      set( AdobeSDK_INCLUDE_DIR ${AdobeSDK_ROOT_DIR} )
    elseif( EXISTS "${AdobeSDK_ROOT_DIR}/Headers/AE_EffectVers.h" )
      set( AdobeSDK_INCLUDE_DIR "${AdobeSDK_ROOT_DIR}/Headers" )
    endif()
    if( DEFINED AdobeSDK_INCLUDE_DIR )
      _AdobeSDK_getSdkVersion( AdobeSDK_VERSION ${AdobeSDK_ROOT_DIR} ${AdobeSDK_INCLUDE_DIR} )
      #message( STATUS "FindAdobeSDK: AdobeSDK_VERSION=${AdobeSDK_VERSION}" )
      if( EXISTS "${AdobeSDK_INCLUDE_DIR}/SP" )
        set( AdobeSDK_INCLUDE_DIRS "${AdobeSDK_INCLUDE_DIR}" "${AdobeSDK_INCLUDE_DIR}/SP" )
      else()
        set( AdobeSDK_INCLUDE_DIRS "${AdobeSDK_INCLUDE_DIR}" )
      endif()
    endif()
    if( EXISTS "${AdobeSDK_ROOT_DIR}/Util" )
      set( AdobeSDK_UTIL_DIR "${AdobeSDK_ROOT_DIR}/Util" )
    endif()
    if( EXISTS "${AdobeSDK_ROOT_DIR}/Resources" )
      set( AdobeSDK_RESOURCE_DIR "${AdobeSDK_ROOT_DIR}/Resources" )
    endif()
      if( NOT DEFINED AdobeSDK_PLUGIN_INSTALL_PATH )
        _AdobeSDK_getPluginInstallPath( _AdobeSDK_PLUGIN_INSTALL_PATH )
        set( AdobeSDK_PLUGIN_INSTALL_PATH ${_AdobeSDK_PLUGIN_INSTALL_PATH} CACHE PATH "Adobe After Effects plugin install path" )
        unset( _AdobeSDK_PLUGIN_INSTALL_PATH )
        # mark_as_advanced( AdobeSDK_PLUGIN_INSTALL_PATH )
      endif()
      if( WIN32 )
        _AdobeSDK_findPiplTool( AdobeSDK_PIPL_TOOL ${AdobeSDK_ROOT_DIR} )
      endif()    
  endif()
else()
  message( FATAL_ERROR "FindAdobeSDK: Adobe After Effects plugin development requires MSVC or XCODE. Please check your configuration." )
endif()

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args( AdobeSDK REQUIRED_VARS ${_adobesdk_required_vars} VERSION_VAR AdobeSDK_VERSION )
