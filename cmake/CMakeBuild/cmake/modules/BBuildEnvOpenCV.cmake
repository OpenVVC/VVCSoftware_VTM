#[[.rst:
BBuildEnvOpenCV
---------------

Functions and macros supporting OpenCV development.

#]]

# Copy OpenCV Windows DLLs given the list of OpenCV components as supported by find_package( OpenCV ).
# The macro autodetects a shared OpenCV configuration and does nothing if static OpenCV libraries are enabled.
macro( bb_add_target_CopyOpenCVRuntimeFiles opencv_comp1_ )
  if( MSVC )
    if( NOT TARGET CopyOpenCVRuntimeFiles )
      # Autodection of static or shared OpenCV based on target property IMPORTED_LOCATION_RELEASE
      get_target_property( _prop_value "opencv_core" IMPORTED_LOCATION_RELEASE )
      if( _prop_value )
        get_filename_component( _opencv_lib_ext ${_prop_value} EXT )
        if( _opencv_lib_ext MATCHES "^\\.(dll|DLL)$" )
          unset( _opencv_dll_list_release )
          unset( _opencv_dll_list_debug )
          _bb_find_opencv_dlls( _opencv_bin_dir _opencv_dll_list_release _opencv_dll_list_debug ${ARGV} )
          if( NOT _opencv_dll_list_release )
            message( FATAL_ERROR "bb_add_target_CopyOpenCVRuntimeFiles( ${ARGV} ): installation or configuration error: no release DLLs found." )
          endif()
          if( NOT _opencv_dll_list_debug )
            message( FATAL_ERROR "bb_add_target_CopyOpenCVRuntimeFiles( ${ARGV} ): installation or configuration error: no debug DLLs found." )
          endif()
          #message( STATUS "bb_add_target_CopyOpenCVRuntimeFiles: ${_opencv_bin_dir}" )
          #message( STATUS "bb_add_target_CopyOpenCVRuntimeFiles: ${_opencv_dll_list_release}" )
          #message( STATUS "bb_add_target_CopyOpenCVRuntimeFiles: ${_opencv_dll_list_debug}" )
          unset( _opencv_files )
          foreach( _opencv_dll_fname ${_opencv_dll_list_debug} )
            list( APPEND _opencv_files "$<$<CONFIG:Debug>:${_opencv_dll_fname}>" )
          endforeach()
          foreach( _opencv_dll_fname ${_opencv_dll_list_release} )
            list( APPEND _opencv_files "$<$<NOT:$<CONFIG:Debug>>:${_opencv_dll_fname}>" )
          endforeach()
          if( CMAKE_VERSION VERSION_GREATER_EQUAL 3.8 )
            add_custom_target( CopyOpenCVRuntimeFiles ALL ${CMAKE_COMMAND} -E make_directory
                                 $<$<CONFIG:Debug>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}>
                                 $<$<CONFIG:Release>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}>
                                 $<$<CONFIG:RelWithDebInfo>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO}>
                                 $<$<CONFIG:MinSizeRel>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL}>
                               COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_opencv_files}
                                 $<$<CONFIG:Debug>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}>
                                 $<$<CONFIG:Release>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}>
                                 $<$<CONFIG:RelWithDebInfo>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO}>
                                 $<$<CONFIG:MinSizeRel>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL}>
                               WORKING_DIRECTORY ${_opencv_bin_dir} 
                               VERBATIM COMMAND_EXPAND_LISTS )
          else()
            add_custom_target( CopyOpenCVRuntimeFiles ALL ${CMAKE_COMMAND} -E make_directory
                                 $<$<CONFIG:Debug>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}>
                                 $<$<CONFIG:Release>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}>
                                 $<$<CONFIG:RelWithDebInfo>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO}>
                                 $<$<CONFIG:MinSizeRel>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL}>
                               COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_opencv_files}
                                 $<$<CONFIG:Debug>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}>
                                 $<$<CONFIG:Release>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}>
                                 $<$<CONFIG:RelWithDebInfo>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO}>
                                 $<$<CONFIG:MinSizeRel>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL}>
                               WORKING_DIRECTORY ${_opencv_bin_dir} )
          endif()
        else()
          #message( STATUS "bb_add_target_CopyOpenCVRuntimeFiles(): OpenCV static libraries configured, no need to copy DLLs" )        
        endif()
      else()
        message( FATAL_ERROR "bb_add_target_CopyOpenCVRuntimeFiles(): target opencv_core does not have property IMPORTED_LOCATION_RELEASE." )
      endif()
    endif()
  endif()  
endmacro()

function( _bb_find_opencv_dlls opencv_bin_dir_ opencv_dll_list_release_ opencv_dll_list_debug_ opencv_comp1_ )
  set( _opencv_comp_list ${opencv_comp1_} ${ARGN} )
  unset( _opencv_target_list )
  unset( _opencv_dll_list )
  unset( _opencv_dll_fname_list )
  bb_get_imp_targets_from_components( _opencv_target_list "opencv_" RELEASE ${_opencv_comp_list} )
  if( DEFINED _opencv_target_list )
    bb_get_dsos_from_imp_targets( _opencv_dll_list RELEASE ${_opencv_target_list} )
    if( _opencv_dll_list )
      list( GET _opencv_dll_list 0 _libopencv )
      # message( STATUS "_bb_find_qt5_dlls(): first libqt5=${_libqt5}" )
      get_filename_component( _opencv_bin_dir ${_libopencv} DIRECTORY )
      set( ${opencv_bin_dir_} ${_opencv_bin_dir} PARENT_SCOPE )
    endif()
    foreach( _lib IN LISTS _opencv_dll_list )
      get_filename_component( _lib_fname ${_lib} NAME )
      list( APPEND _opencv_dll_fname_list ${_lib_fname} )
    endforeach()
    set( ${opencv_dll_list_release_} ${_opencv_dll_fname_list} PARENT_SCOPE )    
  endif()
  
  unset( _opencv_target_list )
  unset( _opencv_dll_list )
  unset( _opencv_dll_fname_list )
  bb_get_imp_targets_from_components( _opencv_target_list "opencv_" DEBUG ${_opencv_comp_list} )
  if( DEFINED _opencv_target_list )
    bb_get_dsos_from_imp_targets( _opencv_dll_list DEBUG ${_opencv_target_list} )
    foreach( _lib IN LISTS _opencv_dll_list )
      get_filename_component( _lib_fname ${_lib} NAME )
      list( APPEND _opencv_dll_fname_list ${_lib_fname} )
    endforeach()    
    set( ${opencv_dll_list_debug_} ${_opencv_dll_fname_list} PARENT_SCOPE )
  endif()
endfunction()
