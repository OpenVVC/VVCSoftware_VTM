#[[.rst:
BBuildEnvQt5
------------

Functions and macros supporting Qt5 development.

#]]



#
# bb_find_qt5_dir( qt5_dir qt5_version... [qt5_root] [OPTIONAL])
#
# 2nd signature is reserved for future extensions and a noop for the time being.
# bb_find_qt5_dir( qt5_dir SYSTEM )
#
function( bb_find_qt5_dir qt5_dir_ qt_version_ )
  
  set( _qt5_dir_required TRUE )
  
  if( "${qt_version_}" STREQUAL "SYSTEM" ) 
    if( NOT CMAKE_SYSTEM_NAME STREQUAL "Linux" )
      message( FATAL_ERROR "bb_find_qt5_dir(): bb_find_qt5_dir( ${qt5_dir_} ${qt_version_} is not supported." )
    endif()
    # Linux: Qt5 system directories are supposed to be found without any additional hints. 
    return()
  endif()  
  
  # Initialize Qt root directory before analyzing function arguments.
  if( WIN32 )
    set( _qt_root_dir "C:/Qt" )
  elseif( UNIX )
    # works for Linux and MacOSX
    set( _qt_root_dir "$ENV{HOME}/Qt" )
  endif()
  
  set( _qt_version_list ${qt_version_} )
  # Collect any optional arguments 
  foreach( _arg IN LISTS ARGN )
    if( _arg STREQUAL "OPTIONAL" )
      set( _qt5_dir_required FALSE )
    else()
      list( APPEND _qt_version_list ${_arg} )
    endif()
  endforeach()
  
  if( ${ARGC} GREATER_EQUAL 3 )
    # Analyze the last element of _qt_version_list, it may be a version, qt5_root or the flag OPTIONAL
    list( GET _qt_version_list -1 _last_arg )
    if( NOT _last_arg MATCHES "^[0-9.]+$" )
      # not a version -> qt5_root specified
      set( _qt_root_dir ${_last_arg} )
      list( REMOVE_AT _qt_version_list -1 )
    endif()
  endif()
  
  if( MSVC )  
    if( CMAKE_VS_PLATFORM_NAME STREQUAL "x64" )
      set( _qt_msvc_64 "_64" )
    else()
      unset( _qt_msvc_64 )
    endif()
  endif()
  
  unset( _qt5_dir )
  if( EXISTS "${_qt_root_dir}" )     
    foreach( _ver IN LISTS _qt_version_list )
      if( MSVC )
        if( MSVC_VERSION VERSION_GREATER_EQUAL 1910 )
          set( _qt5_dir "${_qt_root_dir}/${_ver}/msvc2017${_qt_msvc_64}/lib/cmake/Qt5" )
        elseif( MSVC_VERSION VERSION_EQUAL 1900 )
          set( _qt5_dir "${_qt_root_dir}/${_ver}/msvc2015${_qt_msvc_64}/lib/cmake/Qt5" )
        elseif(  MSVC_VERSION VERSION_EQUAL 1800 )
          set( _qt5_dir "${_qt_root_dir}/${_ver}/msvc2013${_qt_msvc_64}/lib/cmake/Qt5" )
        endif()
      elseif( APPLE )
        set( _qt5_dir "${_qt_root_dir}/${_ver}/clang_64/lib/cmake/Qt5" )
      elseif( CMAKE_SYSTEM_NAME STREQUAL "Linux" ) 
        set( _qt5_dir "${_qt_root_dir}/${_ver}/gcc_64/lib/cmake/Qt5" )
      else()
        message( FATAL_ERROR "bb_find_qt5_dir(): this platform and/or toolset is not yet supported." )
      endif()    
      if( EXISTS "${_qt5_dir}" )
        # message( FATAL_ERROR "bb_find_qt5_dir(): ${_qt5_dir} does not exist, check Qt5 installation paths and the version requested." )
        break()
      endif()
      unset( _qt5_dir )
    endforeach()
  endif()
  
  if( NOT DEFINED _qt5_dir )
    if( _qt5_dir_required )
      message( FATAL_ERROR "bb_find_qt5_dir(): no suitable Qt5 directory found, check ${_qt_root_dir}." )
    else()
      message( STATUS "bb_find_qt5_dir(): no suitable Qt5 directory found in ${_qt_root_dir}." )
    endif()
  else()
    set( ${qt5_dir_} "${_qt5_dir}" PARENT_SCOPE )
  endif()  
endfunction( bb_find_qt5_dir )


macro( bb_add_target_CopyQt5RuntimeFiles qt5_comp1_ )
  if( MSVC )
    if( NOT TARGET CopyQt5RuntimeFiles )
      _bb_find_qt5_dlls( _qt5_bin_dir _qt5_dll_list_release _qt5_dll_list_debug ${ARGV} )
      #message( STATUS "bb_add_target_CopyQt5RuntimeFiles: ${_qt5_bin_dir}" )
      #message( STATUS "bb_add_target_CopyQt5RuntimeFiles: ${_qt5_dll_list_release}" )
      #message( STATUS "bb_add_target_CopyQt5RuntimeFiles: ${_qt5_dll_list_debug}" )
      unset( _qt5_files )
      foreach( _qt5_dll_fname ${_qt5_dll_list_debug} )
        list( APPEND _qt5_files "$<$<CONFIG:Debug>:${_qt5_dll_fname}>" )
      endforeach()
      foreach( _qt5_dll_fname ${_qt5_dll_list_release} )
        list( APPEND _qt5_files "$<$<NOT:$<CONFIG:Debug>>:${_qt5_dll_fname}>" )
      endforeach()
      if( CMAKE_VERSION VERSION_GREATER_EQUAL 3.8 )
        add_custom_target( CopyQt5RuntimeFiles ALL ${CMAKE_COMMAND} -E make_directory
                             $<$<CONFIG:Debug>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}>
                             $<$<CONFIG:Release>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}>
                             $<$<CONFIG:RelWithDebInfo>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO}>
                             $<$<CONFIG:MinSizeRel>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL}>
                           COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_qt5_files}
                             $<$<CONFIG:Debug>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}>
                             $<$<CONFIG:Release>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}>
                             $<$<CONFIG:RelWithDebInfo>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO}>
                             $<$<CONFIG:MinSizeRel>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL}>
                           WORKING_DIRECTORY ${_qt5_bin_dir} 
                           VERBATIM COMMAND_EXPAND_LISTS )
      else()
        add_custom_target( CopyQt5RuntimeFiles ALL ${CMAKE_COMMAND} -E make_directory
                             $<$<CONFIG:Debug>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}>
                             $<$<CONFIG:Release>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}>
                             $<$<CONFIG:RelWithDebInfo>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO}>
                             $<$<CONFIG:MinSizeRel>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL}>
                           COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_qt5_files}
                             $<$<CONFIG:Debug>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}>
                             $<$<CONFIG:Release>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}>
                             $<$<CONFIG:RelWithDebInfo>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO}>
                             $<$<CONFIG:MinSizeRel>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL}>
                           WORKING_DIRECTORY ${_qt5_bin_dir} )
      endif()
    endif()
  endif()  
endmacro()


# Optional argument: NO_PDB_FILES RELEASE DEBUG ALL
function( bb_find_qt5_dsos qt5_dll_list_ )
   
  set( _qt5_comp_list ${ARGN} )
  if( NO_PDB_FILES IN_LIST _qt5_comp_list )
    set( _no_pdb_files TRUE )
    list( REMOVE_ITEM _qt5_comp_list NO_PDB_FILES )
  else()
    set( _no_pdb_files FALSE )
  endif()
  set( _configuration "RELEASE" )
  if( RELEASE IN_LIST _qt5_comp_list )
    list( REMOVE_ITEM _qt5_comp_list RELEASE )
  endif()
  if( DEBUG IN_LIST _qt5_comp_list )
    list( REMOVE_ITEM _qt5_comp_list DEBUG )
    set( _configuration "DEBUG" )
  endif()
  
  if( ALL IN_LIST _qt5_comp_list )
    # Linux only: collect all DSOs
    bb_find_qt5_dsos_complete( _qt5_dll_list ${_configuration} )
    set( ${qt5_dll_list_} ${_qt5_dll_list} PARENT_SCOPE )
    return()
  endif()
  
  unset( _qt5_target_list )
  bb_get_imp_targets_from_components( _qt5_target_list "Qt5::" ${_configuration} ${_qt5_comp_list} )
  #message( STATUS "bb_find_qt5_dsos(): ${_qt5_target_list}" )
  
  unset( _qt5_bin_dir )
  unset( _qt5_dll_list )
  # Walk _qt5_target_list and collect paths to dlls.
  foreach( _qt5_target ${_qt5_target_list} )
    # "IMPORTED_LOCATION" "IMPORTED_LOCATION_RELEASE" "IMPORTED_LOCATION_DEBUG"
    get_target_property( _prop_value ${_qt5_target} "IMPORTED_LOCATION_${_configuration}" )
    if( NOT _prop_value )
      message( WARNING "_bb_get_qt5_dlls(): no IMPORTED_LOCATION_${_configuration} for target ${_qt5_target}" )
      get_target_property( _prop_value ${_qt5_target} "IMPORTED_LOCATION" )
    endif()
    if( _prop_value )
      # message( STATUS "${_qt5_target}: IMPORTED_LOCATION_RELEASE=${_prop_value}" )
      if( NOT DEFINED _qt5_bin_dir )
        get_filename_component( _qt5_bin_dir ${_prop_value} DIRECTORY )
      endif()
      list( APPEND _qt5_dll_list ${_prop_value} )
      if( _configuration STREQUAL "DEBUG" )
        if( WIN32 AND NOT ${_no_pdb_files} )
          # Add PDB file if available
          get_filename_component( _filenm ${_prop_value} NAME_WE )
          if( EXISTS "${_qt5_bin_dir}/${_filenm}.pdb" )
            list( APPEND _qt5_dll_list "${_qt5_bin_dir}/${_filenm}.pdb" )
          endif()
        endif()      
      endif()
    endif()
    if( _qt5_target STREQUAL "Qt5::Core" )
      if( CMAKE_SYSTEM_NAME STREQUAL "Linux" )
        # On Linux Qt5::Core depends on some libicu*.so files which are not defined as LINK dependencies.
        _bb_find_qt5_additional_dsos( ${_qt5_bin_dir} _libicu_list )
        if( _libicu_list )
          list( APPEND _qt5_dll_list ${_libicu_list} )
        endif()
        if( NOT DBus IN_LIST _qt5_comp_list )
          list( APPEND _qt5_dll_list "${_qt5_bin_dir}/libQt5DBus.so.${Qt5_VERSION}" )
        endif()
      endif()
    endif()
  endforeach()
  
  if( CMAKE_SYSTEM_NAME STREQUAL "Linux" )
    # Collect symlinks representing the DSO's soname.
    _bb_get_qt5_dso_symlinks( _qt5_symlink_list ${_qt5_dll_list} )
    if( _qt5_symlink_list )
      list( APPEND _qt5_dll_list ${_qt5_symlink_list} )
    endif()  
  endif()
  
  if( DEFINED _qt5_dll_list )
    set( ${qt5_dll_list_} ${_qt5_dll_list} PARENT_SCOPE )
  endif()
endfunction( bb_find_qt5_dsos )


function( bb_find_qt5_dsos_complete qt5_dll_list_ )

  if( NOT CMAKE_SYSTEM_NAME STREQUAL "Linux" )
    message( FATAL_ERROR "bb_find_qt5_dsos_complete() not implemented on this platform, please contact technical support." )
  endif()

  if( NOT TARGET "Qt5::Core" )
    message( FATAL_ERROR "Target Qt5::Core does not exist." )
  endif()
  unset( _qt5_dll_list )
  get_target_property( _prop_value "Qt5::Core" "IMPORTED_LOCATION_RELEASE" ) 
  if( _prop_value )
    get_filename_component( _qt5_bin_dir ${_prop_value} DIRECTORY )
    if( CMAKE_SYSTEM_NAME STREQUAL "Linux" )
      file( GLOB _file_list "${_qt5_bin_dir}/libQt5*.so.*" "${_qt5_bin_dir}/libicu*.so.*" )
      foreach( _file IN LISTS _file_list )
        if( NOT IS_SYMLINK ${_file} )
          list( APPEND _qt5_dll_list ${_file} )
        endif()
      endforeach()
    endif()
  else()
    message( FATAL_ERROR "Target Qt5::Core does not define property IMPORTED_LOCATION_RELEASE." )
  endif()
  
  if( CMAKE_SYSTEM_NAME STREQUAL "Linux" )
    # Collect symlinks representing the DSO's soname.
    _bb_get_qt5_dso_symlinks( _qt5_symlink_list ${_qt5_dll_list} )
    if( _qt5_symlink_list )
      list( APPEND _qt5_dll_list ${_qt5_symlink_list} )
    endif()  
  endif()
    
  if( DEFINED _qt5_dll_list )
    set( ${qt5_dll_list_} ${_qt5_dll_list} PARENT_SCOPE )
  endif()
endfunction( bb_find_qt5_dsos_complete )


function( _bb_find_qt5_dlls qt5_bin_dir_ qt5_dll_list_release_ qt5_dll_list_debug_ )
  unset( _qt5_dll_list_release )
  unset( _qt5_dll_list_debug )
  bb_find_qt5_dsos( _qt5_dll_list_release ${ARGN} )
  if( _qt5_dll_list_release )
    list( GET _qt5_dll_list_release 0 _libqt5 )
    # message( STATUS "_bb_find_qt5_dlls(): first libqt5=${_libqt5}" )
    get_filename_component( _qt5_bin_dir ${_libqt5} DIRECTORY )
    set( ${qt5_bin_dir_} ${_qt5_bin_dir} PARENT_SCOPE )
  endif()
  
  # Strip directory and return a list of basenames.
  unset( _qt5_dll_list )
  if( DEFINED _qt5_dll_list_release )
    foreach( _lib IN LISTS _qt5_dll_list_release )
      get_filename_component( _lib_fname ${_lib} NAME )
      list( APPEND _qt5_dll_list ${_lib_fname} )
    endforeach()
    set( ${qt5_dll_list_release_} ${_qt5_dll_list} PARENT_SCOPE )
  endif()
  
  bb_find_qt5_dsos( _qt5_dll_list_debug ${ARGN} DEBUG )
  
  # Strip directory and return a list of basenames.
  unset( _qt5_dll_list )
  if( DEFINED _qt5_dll_list_debug )
    foreach( _lib IN LISTS _qt5_dll_list_debug )
      get_filename_component( _lib_fname ${_lib} NAME )
      list( APPEND _qt5_dll_list ${_lib_fname} )
    endforeach()
    set( ${qt5_dll_list_debug_} ${_qt5_dll_list} PARENT_SCOPE )
  endif()
endfunction( _bb_find_qt5_dlls )


function( _bb_find_qt5_additional_dsos qt5_bin_dir_ qt5_additional_dso_list_ )
  if( CMAKE_SYSTEM_NAME STREQUAL "Linux" )
    file( GLOB _lib_add_glob_list "${qt5_bin_dir_}/libicu*.so.*" "${qt5_bin_dir_}/libQt5XcbQpa*.so.*" "${qt5_bin_dir_}/libQt5Egl*Device*.so.*" )        
    if( _lib_add_glob_list )
      # message( STATUS "_bb_get_qt5_icu_dsos(): libicu=${_libicu_glob_list}" )
      # Get rid of any symbolic links
      unset( _lib_add_list )
      foreach( _lib_add IN LISTS _lib_add_glob_list )
        if( NOT IS_SYMLINK ${_lib_add} )
          list( APPEND _lib_add_list ${_lib_add} )
        endif()
      endforeach()
      # message( STATUS "_bb_get_qt5_icu_dsos(): libicu(filtered)=${_libicu_list}" )
      if( DEFINED _lib_add_list )
        set( ${qt5_additional_dso_list_} ${_lib_add_list} PARENT_SCOPE )
      endif()
    endif()
  endif()
endfunction()

function( _bb_get_qt5_dso_symlinks qt5_dso_symlink_list_ qt5_dso1_ )
  set( _qt5_dso_list ${qt5_dso1_} ${ARGN} )
  unset( _qt5_dso_symlink_list )
  foreach( _lib IN LISTS _qt5_dso_list )
    get_filename_component( _lib_dir ${_lib} DIRECTORY )
    get_filename_component( _lib_fname ${_lib} NAME )
    if( CMAKE_SYSTEM_NAME STREQUAL "Linux" )
      # libXXX.so.x.y.z -> soname=libXXX.so.x
      # DON'T assume x==5 as some internal dsos have a complete different major version.
      if( _lib_fname MATCHES "^(lib.*\\.so)+\\.([0-9]+)" )
        set( _lib_symlink_fname "${CMAKE_MATCH_1}.${CMAKE_MATCH_2}" )
        if( EXISTS ${_lib_dir}/${_lib_symlink_fname} )
          list( APPEND _qt5_dso_symlink_list ${_lib_dir}/${_lib_symlink_fname} )
        endif()
      endif()
    endif()    
  endforeach()
  if( _qt5_dso_symlink_list )
    set( ${qt5_dso_symlink_list_} ${_qt5_dso_symlink_list} PARENT_SCOPE )
  endif()  
endfunction() 

function( bb_get_qt5_plugin_dir plugin_dir_ )
  if( NOT TARGET Qt5::qmake )
    message( FATAL_ERROR "installation problem: target Qt5::qmake does not exist." )
  endif()
  get_target_property( _qmake_cmd Qt5::qmake "IMPORTED_LOCATION" )
  if( _qmake_cmd )
    # message( STATUS "qmake=${_qmake_cmd}" )
    
    execute_process( COMMAND ${_qmake_cmd} -query QT_INSTALL_PLUGINS
                     RESULT_VARIABLE _retv_child
                     OUTPUT_VARIABLE _plugin_dir
                     OUTPUT_STRIP_TRAILING_WHITESPACE )
    if( _retv_child EQUAL 0 )
      set( ${plugin_dir_} ${_plugin_dir} PARENT_SCOPE )
    else()
      message( FATAL_ERROR "${_qmake_cmd} -query QT_INSTALL_PLUGINS failed, retv=${_retv_child}" )
    endif()
  endif()
endfunction() 


