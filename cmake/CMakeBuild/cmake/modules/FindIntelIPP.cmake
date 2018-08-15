#[[.rst:
FindIntelIPP
------------

Find Intel Performance Primitives libraries (IPP) searching default system installation paths only.


Variables for using IPP
^^^^^^^^^^^^^^^^^^^^^^^

This module sets the following variables:

``IntelIPP_FOUND``
  True, if IPP is found.
``IntelIPP_VERSION``
  Detected IPP version.

Imported Targets
^^^^^^^^^^^^^^^^

This module defines the :prop_tgt:`IMPORTED` targets:

``IntelIPP::cc``
  IPP color conversion library.
``IntelIPP::ch``
  IPP string operations library. 
``IntelIPP::cv``
  IPP computer vision library. 
``IntelIPP::dc``
  IPP data compression library. 
``IntelIPP::i``
  IPP image processing library. 
``IntelIPP::s``
  IPP signal processing library. 
``IntelIPP::vm``
  IPP vector math library. 
``IntelIPP::core``
  IPP core library.
``IntelIPP::iw``
  Integration wrappers available in IPP version 2018.0.0 or higher.

#]]

# Include some utility functions used by Intel performance library find modules.
include( ${CMAKE_CURRENT_LIST_DIR}/IntelPerfLibCommon.cmake )


macro( IntelIPP_define_target_dependencies ipp_version_ )

  # Add dependencies for all IPP targets: _IntelIPP_target_<domain>_depends = ...
  #
  # Domain             Domain Code  Depends on
  # ----------------------------------------------
  # Color Conversion   CC           Core, VM, S, I
  # String Operations  CH           Core, VM, S
  # Computer Vision    CV           Core, VM, S, I
  # Data Compression   DC           Core, VM, S
  # Image Processing   I            Core, VM, S
  # Signal Processing  S            Core, VM
  # Vector Math        VM           Core
  set( _IntelIPP_target_vm_depends                                        IntelIPP::core )
  set( _IntelIPP_target_s_depends                            IntelIPP::vm IntelIPP::core )
  set( _IntelIPP_target_i_depends  IntelIPP::s               IntelIPP::vm IntelIPP::core )
  set( _IntelIPP_target_dc_depends IntelIPP::s               IntelIPP::vm IntelIPP::core )
  set( _IntelIPP_target_cv_depends IntelIPP::i  IntelIPP::s  IntelIPP::vm IntelIPP::core )
    
  set( _IntelIPP_target_ch_depends IntelIPP::s               IntelIPP::vm IntelIPP::core )
  set( _IntelIPP_target_cc_depends IntelIPP::i  IntelIPP::s  IntelIPP::vm IntelIPP::core )
  
  # >= 2018.0.0
  set( _IntelIPP_target_iw_depends IntelIPP::cc IntelIPP::cv IntelIPP::i IntelIPP::s IntelIPP::vm IntelIPP::core )
  
  foreach( _v vm s i dc cv ch cc iw )
    list( APPEND _IntelIPP_tmp_vars _IntelIPP_target_${_v}_depends )
  endforeach()

endmacro()


macro( IntelIPP_getLibDir lib_dir_ root_dir_ )
  if( WIN32 )
    if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
		  set( ${lib_dir_} "${root_dir_}/lib/intel64_win" )
    else()
		  set( ${lib_dir_} "${root_dir_}/lib/ia32_win" )
	  endif()
  elseif( APPLE )
    set( ${lib_dir_} "${root_dir_}/lib" )
  else()
    # Linux assumed
    if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
      set( ${lib_dir_} "${root_dir_}/lib/intel64_lin" )
    else()
      set( ${lib_dir_} "${root_dir_}/lib/ia32_lin" )        
    endif()
  endif()  
endmacro()

macro( IntelIPP_getLibName lib_name_ domain_ )
  if( ${domain_} STREQUAL "iw" )
    if( WIN32 )
      set( ${lib_name_} "ipp_${domain_}.lib" )
    else()
      set( ${lib_name_} "libipp_${domain_}.a" )
    endif()
  else()
    if( WIN32 )
      set( ${lib_name_} "ipp${domain_}mt.lib" )
    else()
      set( ${lib_name_} "libipp${domain_}.a" )
    endif()
  endif()
endmacro()

macro( IntelIPP_init )
  set( IntelIPP_USE_STATIC_LIBS ON )
  set( IntelIPP_USE_MULTITHREAD OFF )
  foreach( _v _IntelIPP_tmp_vars IntelIPP_ROOT_DIR IntelIPP_VERSION )
    if( DEFINED ${_v} )
      unset( ${_v} )
    endif()
  endforeach()
endmacro()


function( IntelIPP_checkTargets target1_ )
  set( _target_list ${target1_} )
  list( APPEND _target_list ${ARGN} )
  foreach( _tgt IN LISTS _target_list )
    #message( STATUS "checking target IntelIPP::${_tgt}" )
    get_target_property( _prop_value IntelIPP::${_tgt} IMPORTED_LOCATION )
    if( _prop_value )
      if( NOT EXISTS "${_prop_value}" )
        message( FATAL_ERROR "Imported target IntelIPP::${_tgt} defines IMPORTED_LOCATION
 ${_prop_value}       
 which does not exist. Please contact technical support." )
      endif()
    endif()
  endforeach()
endfunction()


# Setup internal variables and check user provided configuration options.
IntelIPP_init()

# <package>_FIND_QUIETLY
#set( IntelIPP_ALL_FOUND FALSE )

set( _IntelIPP_tmp_vars _IntelIPP_root_dir_paths _IntelIPP_version_list )

# Enumerate all IPP installation paths and return version information for each path.
IntelPerfLib_getRootDirPaths( _IntelIPP_root_dir_paths _IntelIPP_version_list IPP )
if( NOT _IntelIPP_root_dir_paths )
  # No IPP version installed.
else()
  #message( STATUS "_ipp_root_dir_paths: ${_IntelIPP_root_dir_paths}" )
  #message( STATUS "_ipp_version_list:   ${_IntelIPP_version_list}" )

  # Select the correct version if find_package() has been called with version hints
  if( DEFINED IntelIPP_FIND_VERSION )
    if( IntelIPP_FIND_VERSION_EXACT )
      IntelPerfLib_selectRootDir( IntelIPP_ROOT_DIR IntelIPP_VERSION "${_IntelIPP_root_dir_paths}" "${_IntelIPP_version_list}" ${IntelIPP_FIND_VERSION} EXACT )
    else()
      IntelPerfLib_selectRootDir( IntelIPP_ROOT_DIR IntelIPP_VERSION "${_IntelIPP_root_dir_paths}" "${_IntelIPP_version_list}" ${IntelIPP_FIND_VERSION} )
    endif()
  else()
    IntelPerfLib_selectRootDir( IntelIPP_ROOT_DIR IntelIPP_VERSION "${_IntelIPP_root_dir_paths}" "${_IntelIPP_version_list}" )
  endif()

  if( IntelIPP_ROOT_DIR )
    #
    set( _IntelIPP_INCLUDE_DIR "${IntelIPP_ROOT_DIR}/include" )
    IntelIPP_getLibDir( _IntelIPP_lib_dir "${IntelIPP_ROOT_DIR}" )

    IntelIPP_define_target_dependencies( ${IntelIPP_VERSION} )
    # message( STATUS "_IntelIPP_tmp_vars=${_IntelIPP_tmp_vars}" )

    set( _IntelIPP_target_list "core" "vm" "s" "i" "dc" "cv" "ch" "cc" )
    if( IntelIPP_VERSION VERSION_GREATER_EQUAL 2018.0.0 )
      list( APPEND _IntelIPP_target_list "iw" )
    endif()

    foreach( _tgt IN LISTS _IntelIPP_target_list )
      # message( STATUS "processing IPP target ${_tgt}" )
      if( NOT TARGET IntelIPP::${_tgt} )
        IntelIPP_getLibName( _IntelIPP_lib_name ${_tgt} )
        add_library( IntelIPP::${_tgt} STATIC IMPORTED )
        set_target_properties( IntelIPP::${_tgt} PROPERTIES IMPORTED_LOCATION "${_IntelIPP_lib_dir}/${_IntelIPP_lib_name}" )
        if( ${_tgt} STREQUAL "core" )
          set_target_properties( IntelIPP::${_tgt} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_IntelIPP_INCLUDE_DIR}" )
        endif()
        if( DEFINED _IntelIPP_target_${_tgt}_depends )
          set_target_properties( IntelIPP::${_tgt} PROPERTIES INTERFACE_LINK_LIBRARIES "${_IntelIPP_target_${_tgt}_depends}" )
        endif()
      endif()
    endforeach()
    IntelIPP_checkTargets( ${_IntelIPP_target_list} )
  endif()
endif()


if( DEFINED _IntelIPP_tmp_vars )
  list( REMOVE_DUPLICATES _IntelIPP_tmp_vars )
  # Dispose any temporary variables avoiding unwanted pollution of the calling namespace.
  foreach( _v ${_IntelIPP_tmp_vars} )
    # message( STATUS "FindIntelIPP.cmake: unsetting ${_v}" )
    unset( ${_v} )
  endforeach()
  unset( _IntelIPP_tmp_vars )
endif()

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args( IntelIPP REQUIRED_VARS IntelIPP_ROOT_DIR VERSION_VAR IntelIPP_VERSION )

