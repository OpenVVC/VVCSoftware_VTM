#[[.rst:
FindIntelMKL
------------

Find Intel Math Kernel Libraries (MKL) searching default system installation paths only.

Variables for using MKL
^^^^^^^^^^^^^^^^^^^^^^^

This module sets the following variables:

``IntelMKL_FOUND``
  True, if MKL is found.
``IntelMKL_VERSION``
  Detected MKL version.

Imported Targets
^^^^^^^^^^^^^^^^

This module defines the :prop_tgt:`IMPORTED` targets:

``IntelMKL::rt``
  Single dynamic MKL. 


#]]

# Include some utility functions used by Intel performance library find modules.
include( ${CMAKE_CURRENT_LIST_DIR}/IntelPerfLibCommon.cmake )


macro( IntelMKL_getLibDir lib_dir_ root_dir_ )
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


macro( IntelMKL_getLibName lib_name_ domain_ )
  if( WIN32 )
    set( ${lib_name_} "mkl_${domain_}.lib" )
  else()
    if( IntelMKL_USE_STATIC_LIBS )
      set( ${lib_name_} "libmkl_${domain_}.a" )
    else()
      if( APPLE )
        set( ${lib_name_} "libmkl_${domain_}.dylib" )
      else()
        set( ${lib_name_} "libmkl_${domain_}.so" )
      endif()
    endif()
  endif()
endmacro()


macro( IntelMKL_init )
  #set( IntelMKL_USE_STATIC_LIBS ON )
  #set( IntelMKL_USE_MULTITHREAD OFF )
  foreach( _v _IntelMKL_tmp_vars IntelMKL_ROOT_DIR IntelMKL_VERSION )
    if( DEFINED ${_v} )
      unset( ${_v} )
    endif()
  endforeach()
endmacro()


function( IntelMKL_findDlls mkl_dlls_ redist_dir_ redist_dir_compiler_ )
  # Assuming single dynamic MKL
  unset( _mkl_dlls )
  
  # Required DLLS
  foreach( _dll rt core intel_thread avx avx2 def vml_avx vml_avx2 vml_def )
    if( EXISTS "${redist_dir_}/mkl_${_dll}.dll" )
      list( APPEND _mkl_dlls mkl_${_dll}.dll )
    else()
      message( FATAL_ERROR "Required DLL '${redist_dir_}/mkl_${_dll}.dll' does not exist." )
    endif()
  endforeach()
  
  # Optional DLLS
  foreach( _dll avx512 vml_avx512 )
    if( EXISTS "${redist_dir_}/mkl_${_dll}.dll" )
      list( APPEND _mkl_dlls mkl_${_dll}.dll )
    endif()
  endforeach()
  
  # Threading support
  if( EXISTS "${redist_dir_compiler_}/libiomp5md.dll" )
    list( APPEND _mkl_dlls "${redist_dir_compiler_}/libiomp5md.dll" )
    if( EXISTS "${redist_dir_compiler_}/libiomp5md.pdb" )
      list( APPEND _mkl_dlls "${redist_dir_compiler_}/libiomp5md.pdb" )
    endif()
  else()
    message( FATAL_ERROR "Required DLL '${redist_dir_compiler_}/libiomp5md.dll' does not exist." )
  endif()
  if( _mkl_dlls )
    set( ${mkl_dlls_} ${_mkl_dlls} PARENT_SCOPE )
  else()
    set( ${mkl_dlls_} ${mkl_dlls_}-NOTFOUND PARENT_SCOPE )
  endif()
endfunction()  

# Setup internal variables and check user provided configuration options.
IntelMKL_init()

set( _IntelMKL_tmp_vars _IntelMKL_root_dir_paths _IntelMKL_version_list )

# Enumerate all IPP installation paths and return version information for each path.
IntelPerfLib_getRootDirPaths( _IntelMKL_root_dir_paths _IntelMKL_version_list MKL )
if( NOT _IntelMKL_root_dir_paths )
  # No MKL version installed.
else()
  #message( STATUS "_ipp_root_dir_paths: ${_IntelMKL_root_dir_paths}" )
  #message( STATUS "_ipp_version_list:   ${_IntelMKL_version_list}" )

  # Select the correct version if find_package() has been called with version hints
  if( DEFINED IntelMKL_FIND_VERSION )
    if( IntelMKL_FIND_VERSION_EXACT )
      IntelPerfLib_selectRootDir( IntelMKL_ROOT_DIR IntelMKL_VERSION "${_IntelMKL_root_dir_paths}" "${_IntelMKL_version_list}" ${IntelMKL_FIND_VERSION} EXACT )      
    else()
      IntelPerfLib_selectRootDir( IntelMKL_ROOT_DIR IntelMKL_VERSION "${_IntelMKL_root_dir_paths}" "${_IntelMKL_version_list}" ${IntelMKL_FIND_VERSION} )      
    endif()
  else()
    IntelPerfLib_selectRootDir( IntelMKL_ROOT_DIR IntelMKL_VERSION "${_IntelMKL_root_dir_paths}" "${_IntelMKL_version_list}" )    
  endif()
  
  if( IntelMKL_ROOT_DIR )

    set( _IntelMKL_INCLUDE_DIR "${IntelMKL_ROOT_DIR}/include" )
    IntelMKL_getLibDir( _IntelMKL_lib_dir "${IntelMKL_ROOT_DIR}" )
    if( WIN32 )
      IntelPerfLib_getRedistDir( _IntelMKL_redist_dir "${IntelMKL_ROOT_DIR}" MKL )
      IntelPerfLib_getRedistDir( _IntelMKL_redist_dir_compiler "${IntelMKL_ROOT_DIR}" COMPILER )
      # list( APPEND( _IntelMKL_tmp_vars _IntelMKL_redist_dir _IntelMKL_redist_dir_compiler )
    endif()

    if( IntelMKL_USE_STATIC_LIBS )
      # NOT WORKING YET
      # ===============
      # # linux/x86_64/static/int32:  -Wl,--start-group ${MKLROOT}/lib/intel64/libmkl_intel_lp64.a ${MKLROOT}/lib/intel64/libmkl_intel_thread.a ${MKLROOT}/lib/intel64/libmkl_core.a -Wl,--end-group -liomp5 -lpthread -lm -ldl
      
      add_library( IntelMKL::core STATIC IMPORTED )
      IntelMKL_getLibName( _IntelMKL_lib_name "core" )
      set_target_properties( IntelMKL::core PROPERTIES IMPORTED_LOCATION "${_IntelMKL_lib_dir}/${_IntelMKL_lib_name}" )
      set_target_properties( IntelMKL::core PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_IntelMKL_INCLUDE_DIR}" )
      
      add_library( IntelMKL::intel_thread STATIC IMPORTED )
      IntelMKL_getLibName( _IntelMKL_lib_name "intel_thread" )
      set_target_properties( IntelMKL::intel_thread PROPERTIES IMPORTED_LOCATION "${_IntelMKL_lib_dir}/${_IntelMKL_lib_name}" )
      #set_target_properties( IntelMKL::core PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_IntelMKL_INCLUDE_DIR}" )
      
      add_library( IntelMKL::intel_lp64 STATIC IMPORTED )
      IntelMKL_getLibName( _IntelMKL_lib_name "intel_lp64" )
      set_target_properties( IntelMKL::intel_lp64 PROPERTIES IMPORTED_LOCATION "${_IntelMKL_lib_dir}/${_IntelMKL_lib_name}" )
      #set_target_properties( IntelMKL::core PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_IntelMKL_INCLUDE_DIR}" )
        
      IntelPerfLib_getRedistDir( _IntelMKL_redist_dir_compiler "${IntelMKL_ROOT_DIR}" COMPILER )
      add_library( IntelMKL::iomp5 SHARED IMPORTED )
      set_target_properties( IntelMKL::iomp5 PROPERTIES IMPORTED_LOCATION "${_IntelMKL_redist_dir_compiler}/libiomp5.so" )
      set_target_properties( IntelMKL::core PROPERTIES INTERFACE_LINK_LIBRARIES "IntelMKL::iomp5;-ldl" )
    elseif( IntelMKL_USE_SHARED_LIBS )
      # NOT WORKING YET
      # ===============    
      add_library( IntelMKL::core SHARED IMPORTED )
      IntelMKL_getLibName( _IntelMKL_lib_name "core" )
      set_target_properties( IntelMKL::core PROPERTIES IMPORTED_LOCATION "${_IntelMKL_lib_dir}/${_IntelMKL_lib_name}" )
      set_target_properties( IntelMKL::core PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_IntelMKL_INCLUDE_DIR}" )
      
      add_library( IntelMKL::intel_thread SHARED IMPORTED )
      IntelMKL_getLibName( _IntelMKL_lib_name "intel_thread" )
      set_target_properties( IntelMKL::intel_thread PROPERTIES IMPORTED_LOCATION "${_IntelMKL_lib_dir}/${_IntelMKL_lib_name}" )
      #set_target_properties( IntelMKL::core PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_IntelMKL_INCLUDE_DIR}" )
      
      add_library( IntelMKL::intel_lp64 SHARED IMPORTED )
      IntelMKL_getLibName( _IntelMKL_lib_name "intel_lp64" )
      set_target_properties( IntelMKL::intel_lp64 PROPERTIES IMPORTED_LOCATION "${_IntelMKL_lib_dir}/${_IntelMKL_lib_name}" )
      #set_target_properties( IntelMKL::core PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_IntelMKL_INCLUDE_DIR}" )
        
      IntelPerfLib_getRedistDir( _IntelMKL_redist_dir_compiler "${IntelMKL_ROOT_DIR}" COMPILER )
      add_library( IntelMKL::iomp5 SHARED IMPORTED )
      set_target_properties( IntelMKL::iomp5 PROPERTIES IMPORTED_LOCATION "${_IntelMKL_redist_dir_compiler}/libiomp5.so" )
      set_target_properties( IntelMKL::core PROPERTIES INTERFACE_LINK_LIBRARIES "IntelMKL::iomp5;-ldl;-lm" )      
    else()
      # Use the single dynamic MKL.
      if( NOT TARGET IntelMKL::rt )
        add_library( IntelMKL::rt SHARED IMPORTED )
        IntelMKL_getLibName( _IntelMKL_lib_name "rt" )
        if( WIN32 )
          set_target_properties( IntelMKL::rt PROPERTIES IMPORTED_LOCATION "${_IntelMKL_redist_dir}/mkl_rt.dll"
                                                         IMPORTED_IMPLIB   "${_IntelMKL_lib_dir}/${_IntelMKL_lib_name}" )
        else()
          set_target_properties( IntelMKL::rt PROPERTIES IMPORTED_LOCATION "${_IntelMKL_lib_dir}/${_IntelMKL_lib_name}" )
        endif()
        set_target_properties( IntelMKL::rt PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_IntelMKL_INCLUDE_DIR}" )
  
        if( APPLE )
          IntelPerfLib_getRedistDir( _IntelMKL_redist_dir_compiler "${IntelMKL_ROOT_DIR}" COMPILER )
          add_library( IntelMKL::iomp5 SHARED IMPORTED )
          set_target_properties( IntelMKL::iomp5 PROPERTIES IMPORTED_LOCATION "${_IntelMKL_redist_dir_compiler}/libiomp5.dylib" )
          set_target_properties( IntelMKL::rt PROPERTIES INTERFACE_LINK_LIBRARIES IntelMKL::iomp5 )
        endif()
  
        if( WIN32 )
          IntelMKL_findDlls( _IntelMKL_NEEDED_DLLS "${_IntelMKL_redist_dir}" "${_IntelMKL_redist_dir_compiler}" )

          add_custom_target( CopyIntelMKLDlls ${CMAKE_COMMAND} -E make_directory
                               $<$<CONFIG:Debug>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}>
                               $<$<CONFIG:Release>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}>
                               $<$<CONFIG:RelWithDebInfo>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO}>
                               $<$<CONFIG:MinSizeRel>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL}>
                               COMMAND ${CMAKE_COMMAND} -E copy_if_different 
                               ${_IntelMKL_NEEDED_DLLS}
                               $<$<CONFIG:Debug>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}>
                               $<$<CONFIG:Release>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}>
                               $<$<CONFIG:RelWithDebInfo>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO}>
                               $<$<CONFIG:MinSizeRel>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL}> 
                               WORKING_DIRECTORY "${_IntelMKL_redist_dir}" )
        
          add_dependencies( IntelMKL::rt CopyIntelMKLDlls )
        endif()
      endif()
    endif()
  endif()
endif()  

if( DEFINED _IntelMKL_tmp_vars )
  list( REMOVE_DUPLICATES _IntelMKL_tmp_vars )
  # Dispose any temporary variables avoiding unwanted pollution of the calling namespace.
  foreach( _v ${_IntelMKL_tmp_vars} )
    #message( STATUS "FindIntelMKL.cmake: unsetting ${_v}" )
    unset( ${_v} )
  endforeach()
  unset( _IntelMKL_tmp_vars )
endif()

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args( IntelMKL REQUIRED_VARS IntelMKL_ROOT_DIR VERSION_VAR IntelMKL_VERSION )

