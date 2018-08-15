#[[.rst:
FindCUDASamples
---------------

Find CUDA samples associated with the selected CUDA toolkit. 

IMPORTED Targets
^^^^^^^^^^^^^^^^

This module defines the :prop_tgt:`IMPORTED` targets:

``CUDASamples::helper``
 Defined if the CUDA samples are installed.
``CUDASamples::freeimage``
 Defined if the CUDA samples are installed and component FreeImage was requested.
``CUDASamples::utilnpp``
 Defined if the CUDA samples are installed and component UtilNPP was requested.

Result Variables
^^^^^^^^^^^^^^^^

This module sets the following variables:

``CUDASamples_FOUND``
 True, if CUDA samples are installed.
``CUDASamples_ROOT_DIR``
 Path to CUDA samples directory.

#]]

if( FindCUDASamples_DEBUG )
  message( STATUS "${CMAKE_CURRENT_LIST_FILE}: entering." )
endif()


function( CUDASamples_getVersion version_ )
  unset( _cuda_version )
  
  if( DEFINED CMAKE_CUDA_COMPILER_VERSION )
    string( REGEX REPLACE "([0-9]+)\\.([0-9]+)([0-9.]+)?" "\\1.\\2" _cuda_version ${CMAKE_CUDA_COMPILER_VERSION} )
  elseif( CUDA_FOUND ) 
    set( _cuda_version ${CUDA_VERSION} )
  endif()

  if( DEFINED _cuda_version )
    set( ${version_} ${_cuda_version} PARENT_SCOPE )
  endif()
endfunction()


function( CUDASamples_findDirs root_dir_ include_dir_ cuda_version_ )

  string( REGEX REPLACE "([0-9]+)\\.([0-9]+)([0-9.]+)?" "\\1.\\2" _cuda_version_major_minor ${cuda_version_} )
  if( WIN32 )
    file( TO_CMAKE_PATH "$ENV{ProgramData}/NVIDIA Corporation/CUDA Samples/v${_cuda_version_major_minor}" _cuda_samples_root_dir )
  else()
    if( CUDA_FOUND ) 
      set( _cuda_samples_root_dir "${CUDA_TOOLKIT_ROOT_DIR}/samples" )
    else()
      # FindCUDA.cmake hasn't been called or failed for some reason. Let's try standard installation paths. 
      if( APPLE )
        set( _cuda_samples_root_dir "/Developer/NVIDIA/CUDA-${_cuda_version_major_minor}/samples" )
      else()
        # Linux assumed
        set( _cuda_samples_root_dir "/usr/local/cuda-${_cuda_version_major_minor}/samples" )        
      endif()
    endif()
  endif()
  find_path( CUDASamples_common_inc helper_cuda.h PATHS "${_cuda_samples_root_dir}" PATH_SUFFIXES common/inc NO_DEFAULT_PATH )
  mark_as_advanced( CUDASamples_common_inc )  
  if( CUDASamples_common_inc )
    set( ${root_dir_} "${_cuda_samples_root_dir}" PARENT_SCOPE )
    set( ${include_dir_} "${CUDASamples_common_inc}" PARENT_SCOPE )
  else()
    set( ${root_dir_} "${root_dir_}-NOTFOUND" PARENT_SCOPE )
    set( ${include_dir_} "${include_dir_}-NOTFOUND" PARENT_SCOPE )  
  endif()
endfunction( CUDASamples_findDirs )


unset( CUDASamples_VERSION )
unset( CUDASamples_ROOT_DIR )
unset( CUDASamples_INCLUDE_DIR )

CUDASamples_getVersion( CUDASamples_VERSION )

if( DEFINED CUDASamples_VERSION )
  
  if( CUDASamples_VERSION VERSION_GREATER_EQUAL 5.0 )
    CUDASamples_findDirs( CUDASamples_ROOT_DIR CUDASamples_INCLUDE_DIR ${CUDASamples_VERSION} )
    if( CUDASamples_INCLUDE_DIR )
      if( NOT TARGET CUDASamples::helper )
        add_library( CUDASamples::helper INTERFACE IMPORTED )
        set_target_properties( CUDASamples::helper PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CUDASamples_INCLUDE_DIR}" )
      endif()
    endif()
    foreach( _comp ${CUDASamples_FIND_COMPONENTS} )
      if( _comp STREQUAL "UtilNPP" )
        if( EXISTS "${CUDASamples_ROOT_DIR}/7_CUDALibraries/common/UtilNPP" )
          set( CUDASamples_${_comp}_FOUND TRUE )
          set( CUDASamples_UtilNPP_INCLUDE_DIR "${CUDASamples_ROOT_DIR}/7_CUDALibraries/common/UtilNPP" )
          if( NOT TARGET CUDASamples::utilnpp )
            add_library( CUDASamples::utilnpp INTERFACE IMPORTED )
            set_target_properties( CUDASamples::utilnpp PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CUDASamples_UtilNPP_INCLUDE_DIR}" )
          endif() 
        else()
          set( CUDASamples_${_comp}_FOUND FALSE )
        endif()
      elseif( _comp STREQUAL "FreeImage" )
        set( CUDASamples_${_comp}_FOUND FALSE )
        set( _CUDASamples_FreeImage_ROOT_DIR "${CUDASamples_ROOT_DIR}/7_CUDALibraries/common/FreeImage" )
        if( WIN32 )
          if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
            # In CUDA 9.1 NVIDIA changed FreeImage64.lib to FreeImage.lib.
            find_library( CUDASamples_libfreeimage NAMES FreeImage64.lib FreeImage.lib PATHS "${_CUDASamples_FreeImage_ROOT_DIR}" PATH_SUFFIXES lib/x64 NO_DEFAULT_PATH )
          else()
            find_library( CUDASamples_libfreeimage name FreeImage.lib PATHS "${_CUDASamples_FreeImage_ROOT_DIR}" PATH_SUFFIXES lib/Win32 NO_DEFAULT_PATH )
          endif()
          if( CUDASamples_libfreeimage )
            get_filename_component( _CUDASamples_libfreeimage_dir ${CUDASamples_libfreeimage} DIRECTORY )
            get_filename_component( _CUDASamples_libfreeimage_fname ${CUDASamples_libfreeimage} NAME_WE )
            set( CUDASamples_libfreeimage_dll "${_CUDASamples_libfreeimage_dir}/${_CUDASamples_libfreeimage_fname}.dll" )
          endif()
        elseif( APPLE )
          find_library( CUDASamples_libfreeimage name libfreeimage.a PATHS "${_CUDASamples_FreeImage_ROOT_DIR}" PATH_SUFFIXES lib/darwin NO_DEFAULT_PATH )
        elseif( UNIX )        
          find_library( CUDASamples_libfreeimage name libfreeimage.a PATHS "${_CUDASamples_FreeImage_ROOT_DIR}" PATH_SUFFIXES lib/linux/x86_64 NO_DEFAULT_PATH )
        endif()
        mark_as_advanced( CUDASamples_libfreeimage )
        if( CUDASamples_libfreeimage AND EXISTS "${_CUDASamples_FreeImage_ROOT_DIR}/include" )
          set( CUDASamples_${_comp}_FOUND TRUE )
          set( CUDASamples_FreeImage_INCLUDE_DIR "${_CUDASamples_FreeImage_ROOT_DIR}/include" )
          if( NOT TARGET CUDASamples::freeimage )
            if( WIN32 )
              add_library( CUDASamples::freeimage SHARED IMPORTED )
              set_target_properties( CUDASamples::freeimage PROPERTIES IMPORTED_IMPLIB "${CUDASamples_libfreeimage}" )
              if( EXISTS "${CUDASamples_libfreeimage_dll}" )
                set_target_properties( CUDASamples::freeimage PROPERTIES IMPORTED_LOCATION "${CUDASamples_libfreeimage_dll}" )
                if( CMAKE_VERSION VERSION_GREATER_EQUAL 3.8 )
                  # message( STATUS "FindCUDASamples: using add_custom_target( ... VERBATIM COMMAND_EXPAND_LISTS )" )
                  add_custom_target( Copy${_comp}RuntimeFiles ${CMAKE_COMMAND} -E make_directory
                                     $<$<CONFIG:Debug>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}>
                                     $<$<CONFIG:Release>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}>
                                     $<$<CONFIG:RelWithDebInfo>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO}>
                                     $<$<CONFIG:MinSizeRel>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL}>
                                     COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CUDASamples_libfreeimage_dll}
                                     $<$<CONFIG:Debug>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}>
                                     $<$<CONFIG:Release>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}>
                                     $<$<CONFIG:RelWithDebInfo>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO}>
                                     $<$<CONFIG:MinSizeRel>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL}> 
                                     VERBATIM COMMAND_EXPAND_LISTS )
                else()
                  add_custom_target( Copy${_comp}RuntimeFiles ${CMAKE_COMMAND} -E make_directory
                                     $<$<CONFIG:Debug>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}>
                                     $<$<CONFIG:Release>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}>
                                     $<$<CONFIG:RelWithDebInfo>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO}>
                                     $<$<CONFIG:MinSizeRel>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL}>
                                     COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CUDASamples_libfreeimage_dll}
                                     $<$<CONFIG:Debug>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}>
                                     $<$<CONFIG:Release>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}>
                                     $<$<CONFIG:RelWithDebInfo>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO}>
                                     $<$<CONFIG:MinSizeRel>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL}> )
                endif()
                add_dependencies( CUDASamples::freeimage Copy${_comp}RuntimeFiles )
              endif()
            else()
              add_library( CUDASamples::freeimage STATIC IMPORTED )
              set_target_properties( CUDASamples::freeimage PROPERTIES IMPORTED_LOCATION "${CUDASamples_libfreeimage}" )
            endif()
            set_target_properties( CUDASamples::freeimage PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CUDASamples_FreeImage_INCLUDE_DIR}" )
          endif()
        endif()
      endif()
    endforeach()
  endif()
endif()

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args( CUDASamples REQUIRED_VARS CUDASamples_ROOT_DIR CUDASamples_INCLUDE_DIR VERSION_VAR CUDASamples_VERSION HANDLE_COMPONENTS )
if( FindCUDASamples_DEBUG )
  message( STATUS "${CMAKE_CURRENT_LIST_FILE}: leaving." )
endif()
