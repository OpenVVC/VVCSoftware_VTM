#[[.rst:
BBuildEnvBoost
--------------

Functions and macros helping to find locally built Boost libraries and simplify their use on 
certain platforms.

#]]


# Boost -> CMake compatibility matrix
#set( boost_version_cmake_version_list
#     1_65_1-3.9.3
#     1_65_0-3.9.3 
#     1_64_0-3.8.0 
#     1_63_0-3.7.2
#     1_62_0-3.7.0
#     1_61_0-3.5.1
#   )

# FindBoost.cmake does not add the proper defines to the imported targets Boost::<target> which breaks
# linking to shared boost libraries in some cases or platforms.

# By default BOOST_<COMP>_DYN_LINK is defined if shared BOOST libraries are enabled but some
# BOOST libraries don't follow that naming convention.
set( _bb_boost_coroutine_defs_shared BOOST_COROUTINES_DYN_LINK )

set( _bb_boost_math_tr1f_defs_shared BOOST_MATH_TR1_DYN_LINK )
set( _bb_boost_math_tr1l_defs_shared BOOST_MATH_TR1_DYN_LINK )
set( _bb_boost_math_c99_defs_shared  BOOST_MATH_TR1_DYN_LINK )
set( _bb_boost_math_c99f_defs_shared BOOST_MATH_TR1_DYN_LINK )
set( _bb_boost_math_c99l_defs_shared BOOST_MATH_TR1_DYN_LINK )

set( _bb_boost_unit_test_framework_defs_shared BOOST_TEST_DYN_LINK )
set( _bb_boost_prg_exec_monitor_defs_shared    BOOST_TEST_DYN_LINK )


function( bb_find_boost_root boost_dir_ )
  #message( STATUS "bb_find_boost_root: home: ${bb_home_dir}" )
  #message( STATUS "bb_find_boost_root: proj_home: ${bb_proj_home}" )

  set( _boost_root_required TRUE )
  if( ARGC GREATER 1 )
    # Process optional arguments
    foreach( _arg IN LISTS ARGN )
      if( _arg STREQUAL "OPTIONAL" )
        set( _boost_root_required FALSE )
      endif()
    endforeach()
  endif()

  unset( _bb_boost_root )
  if( _boost_root_required ) 
    bb_set_external_dir( _bb_boost_root "${boost_dir_}" )
  else()
    bb_set_external_dir( _bb_boost_root "${boost_dir_}" OPTIONAL )
  endif()
  if( NOT DEFINED _bb_boost_root )
    return()
  endif()

  # configure our BOOST library directory
  bb_get_boost_toolset_subdir( _bb_boost_toolset )
  set( _bb_boost_librarydir "${_bb_boost_root}/lib/${_bb_boost_toolset}/${bb_platform_dir}" )

  # FindBoost.cmake requires a hint to recognize certain compilers/platforms when searching for 
  # library names. 
  unset( _bb_boost_compiler )
  string( REGEX REPLACE "([0-9]+)\\.([0-9]+)([0-9.]+)?" "\\1\\2" _bb_compiler_version_majorminor ${CMAKE_CXX_COMPILER_VERSION} )
  
  if( MINGW )
    # MinGW is either a native or a cross compiler.
    set( _bb_boost_compiler "-mgw${_bb_compiler_version_majorminor}" )    
  elseif( CMAKE_CROSSCOMPILING )
    # No cross compiling support yet -> MinGW is a special case
  elseif( APPLE )
    # Assume Apple xcode/clang as the selected toolchain.
    # No automatic boost find support implemented by cmake 3.7.0 yet, hence Boost_COMPILER is required
    # helping cmake to find BOOST library files.
    if( CMAKE_CXX_COMPILER_ID MATCHES "^(AppleClang|Clang)$" )
      set( _bb_boost_compiler "-clang-darwin42" )
    elseif( CMAKE_CXX_COMPILER_ID STREQUAL "Intel" )
      set( _bb_boost_compiler "-il${_bb_compiler_version_majorminor}" )
    endif()
  elseif( MSVC )
    if( CMAKE_CXX_COMPILER_ID STREQUAL "Intel" )
      set( _bb_boost_compiler "-iw" )
    endif()
  elseif( UNIX )
    if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" )
      # Ubuntu 17.10: g++-7 emits just 7 via g++ -dumpversion which FindBoost.cmake uses instead of CMAKE_CXX_COMPILER_VERSION.
      set( _bb_boost_compiler "-gcc${_bb_compiler_version_majorminor}" )
    elseif( CMAKE_CXX_COMPILER_ID STREQUAL "Clang" )
      # FindBoost.cmake has no support for clang/linux yet (cmake 3.7.0)
      set( _bb_boost_compiler "-clang${_bb_compiler_version_majorminor}" )
    elseif( CMAKE_CXX_COMPILER_ID STREQUAL "Intel" )
      # FindBoost.cmake has no support for clang/linux yet (cmake 3.7.0)
      set( _bb_boost_compiler "-il${_bb_compiler_version_majorminor}" )
    endif()
  endif()

  # system paths usually contain outdated boost versions, disabled by design.
  set( Boost_NO_SYSTEM_PATHS ON PARENT_SCOPE )
  # always enable MT  
  set( Boost_USE_MULTITHREAD ON PARENT_SCOPE )

  # and export local variables the caller must know about
  set( BOOST_ROOT ${_bb_boost_root} PARENT_SCOPE )
  set( BOOST_LIBRARYDIR ${_bb_boost_librarydir} PARENT_SCOPE )
  if( DEFINED _bb_boost_compiler )
    set( Boost_COMPILER ${_bb_boost_compiler} PARENT_SCOPE )
  endif()
endfunction( bb_find_boost_root )


macro( bb_find_boost_save_fnd_context fnd_ctx )
  if( CMAKE_CROSSCOMPILING )
    # message( STATUS "bb_find_boost_save_fnd_context(): CMAKE_FIND_ROOT_PATH_MODE_LIBRARY=${CMAKE_FIND_ROOT_PATH_MODE_LIBRARY}" )
    # message( STATUS "bb_find_boost_save_fnd_context(): CMAKE_FIND_ROOT_PATH_MODE_INCLUDE=${CMAKE_FIND_ROOT_PATH_MODE_INCLUDE}" )
  
    # find_package must be told not to expect the BOOST libraries inside "CMAKE_FIND_ROOT_PATH".
    if( DEFINED CMAKE_FIND_ROOT_PATH_MODE_LIBRARY )
      set( ${fnd_ctx}_CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ${CMAKE_FIND_ROOT_PATH_MODE_LIBRARY} )
      set( CMAKE_FIND_ROOT_PATH_MODE_LIBRARY NEVER )
    else()
      unset( ${fnd_ctx}_CMAKE_FIND_ROOT_PATH_MODE_LIBRARY )
    endif()
    if( DEFINED CMAKE_FIND_ROOT_PATH_MODE_INCLUDE )
      set( ${fnd_ctx}_CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ${CMAKE_FIND_ROOT_PATH_MODE_INCLUDE} )
      set( CMAKE_FIND_ROOT_PATH_MODE_INCLUDE NEVER )
    else()
      unset( ${fnd_ctx}_CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ) 
    endif()
  endif()
endmacro()


macro( bb_find_boost_restore_fnd_context fnd_ctx )
  if( CMAKE_CROSSCOMPILING )
    # Restore CMAKE_FIND_ROOT_PATH settings
    if( DEFINED ${fnd_ctx}_CMAKE_FIND_ROOT_PATH_MODE_LIBRARY )
      set( CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ${${fnd_ctx}_CMAKE_FIND_ROOT_PATH_MODE_LIBRARY} )
      unset( ${fnd_ctx}_CMAKE_FIND_ROOT_PATH_MODE_LIBRARY )
    endif()
    if( DEFINED ${fnd_ctx}_CMAKE_FIND_ROOT_PATH_MODE_INCLUDE )
      set( CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ${${fnd_ctx}_CMAKE_FIND_ROOT_PATH_MODE_INCLUDE} )
      unset( ${fnd_ctx}_CMAKE_FIND_ROOT_PATH_MODE_INCLUDE )
    endif()
    # message( STATUS "bb_find_boost_restore_fnd_context(): CMAKE_FIND_ROOT_PATH_MODE_LIBRARY=${CMAKE_FIND_ROOT_PATH_MODE_LIBRARY}" )
    # message( STATUS "bb_find_boost_restore_fnd_context(): CMAKE_FIND_ROOT_PATH_MODE_INCLUDE=${CMAKE_FIND_ROOT_PATH_MODE_INCLUDE}" )
  endif()
endmacro()


function( bb_get_boost_toolset_subdir toolset_ )
  
  if( CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" )
    
    # Special handling of msvc as in Boost.Build the msvc version is related to the IDE but in cmake 
    # it's related to the compiler version. 
    if( MSVC_VERSION VERSION_GREATER_EQUAL 1910 )
      # VS 2017 updated cl to 19.11.x, 19.12.x which is also mapped to msvc-14.1 by Boost.Build/1.65.0
      set( _boost_toolset "msvc-14.1" )
    elseif( MSVC_VERSION VERSION_EQUAL 1900 )
      set( _boost_toolset "msvc-14.0" )
    elseif( MSVC_VERSION VERSION_EQUAL 1800 )
      set( _boost_toolset "msvc-12.0" )
    elseif( MSVC_VERSION VERSION_EQUAL 1700 )
      set( _boost_toolset "msvc-11.0" )
    elseif( MSVC_VERSION VERSION_EQUAL 1600 )
      set( _boost_toolset "msvc-10.0" )
    else()
      message( FATAL_ERROR "bb_get_boost_toolset_subdir(): unsupported MSVC version ${MSVC_VERSION} found, please contact technical support." )
    endif()
  else()
    if( DEFINED bb_toolset_subdir )
      set( _boost_toolset ${bb_toolset_subdir} )
    else()
      # message( FATAL_ERROR "bb_get_boost_toolset_subdir(): variable bb_toolset_subdir is undefined, please contact technical support." )
      _bb_get_cxx_compiler_version_major_minor( _compiler_version_major_minor )
      _bb_get_toolset_subdir( _boost_toolset ${_compiler_version_major_minor} )
    endif()
  endif()  
  set( ${toolset_} ${_boost_toolset} PARENT_SCOPE )
endfunction()


# Add missing Boost definitions
function( bb_add_boost_definitions boost_comp_ )

  set( _boost_comps ${boost_comp_} ${ARGN} )
  #message( STATUS "bb_add_boost_definitions: ${_boost_comps}" )
  if( NOT "boost" IN_LIST _boost_comps )
    list( APPEND _boost_comps "boost" )
  endif()

  if( NOT Boost_FOUND )
    message( WARNING "bb_add_boost_definitions() called but Boost_FOUND=FALSE, is this intended?")
    return()
  endif()
  
  unset( _boost_targets )
  set( _add_target_boost_config FALSE )
  
  if( MSVC )
    if( ( Boost_VERSION EQUAL 106600 ) AND ( MSVC_VERSION VERSION_GREATER 1911 ) )
      set( _add_target_boost_config TRUE )
    endif()
    if( _add_target_boost_config )
      if( NOT TARGET Boost::config )
        _bb_get_boost_targets_recursive( _boost_targets ${_boost_comps} )
        add_library( Boost::config INTERFACE IMPORTED )
        set_target_properties( Boost::config PROPERTIES INTERFACE_COMPILE_DEFINITIONS "BOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE" )
        foreach( _tgt IN LISTS _boost_targets )
          # message( STATUS "processing target ${_tgt} to silence MSVC version warnings." )
          get_target_property( _prop_value ${_tgt} INTERFACE_LINK_LIBRARIES )
          if( _prop_value )
            # message( STATUS "${_tgt} has interface link libraries: ${_prop_value}" )
            list( APPEND _prop_value "Boost::config" )
            set_target_properties( ${_tgt} PROPERTIES INTERFACE_LINK_LIBRARIES "${_prop_value}" )
          else()
            set_target_properties( ${_tgt} PROPERTIES INTERFACE_LINK_LIBRARIES "Boost::config" )
          endif()
        endforeach()
        #foreach( _tgt IN LISTS _boost_targets )
        #  message( STATUS "checking target ${_tgt}" )
        #  get_target_property( _prop_value ${_tgt} INTERFACE_LINK_LIBRARIES )
        #  if( _prop_value )
        #    message( STATUS "${_tgt} has interface link libraries: ${_prop_value}" )
        #  endif()
        #endforeach()
      endif()
    endif()
  endif()
    
  if( Boost_USE_STATIC_LIBS )
    #message( STATUS "bb_add_boost_definitions: static boost libraries selected -> no extra definitions." )
    return()
  endif()

  # expand BOOST component list via target list to deal with internal dependencies properly; e.g. filesystem depends on system
  if( NOT DEFINED _boost_targets )
    _bb_get_boost_targets_recursive( _boost_targets ${_boost_comps} )
  endif()
  
  string( REPLACE "Boost::" "" _boost_comps "${_boost_targets}" )
  #message( STATUS "expanded boost component list: ${_boost_comps}" )

  foreach( _comp IN LISTS _boost_comps )
    set( _target "Boost::${_comp}" )
    #message( STATUS "bb_add_boost_definitions: processing target ${_target} ..." )
    if( NOT TARGET ${_target} )
      message( FATAL_ERROR "bb_add_boost_definitions: target ${_target} does not exist." )
    else()
      get_target_property( _prop_value ${_target} TYPE )
      if( _prop_value )
        # message( STATUS "target ${_target} is ${_prop_value}" )
        if( _prop_value STREQUAL "INTERFACE_LIBRARY" )
          #message( STATUS "bb_add_boost_definitions: skipping target ${_target}" )
          continue()
        endif()
      endif()
      get_target_property( _prop_value ${_target} "INTERFACE_COMPILE_DEFINITIONS" )
      if( _prop_value )
        message( FATAL_ERROR "bb_add_boost_definitions: target ${_target} has unexpected INTERFACE_COMPILE_DEFINITIONS: ${_prop_value}" )
      else()
        if( DEFINED _bb_boost_${_comp}_defs_shared )
          set( _new_comp_definitions ${_bb_boost_${_comp}_defs_shared} )
        else()
          string( TOUPPER ${_comp} _comp_uc )
          set( _new_comp_definitions BOOST_${_comp_uc}_DYN_LINK )
        endif()
        set_target_properties( ${_target} PROPERTIES INTERFACE_COMPILE_DEFINITIONS "${_new_comp_definitions}" )
        #message( STATUS "bb_add_boost_definitions: target ${_target} -> INTERFACE_COMPILE_DEFINITIONS ${_new_comp_definitions}" )
      endif()
    endif()
  endforeach()
endfunction( bb_add_boost_definitions )


function( bb_add_target_boost_asio )
  if( NOT Boost_FOUND )
    message( WARNING "bb_add_target_boost_asio() called but Boost_FOUND=FALSE, is this intended?" )
    return()
  endif()
  
  if( NOT TARGET Boost::asio )
    if( NOT TARGET Boost::system )
      message( FATAL_ERROR "Boost::asio depends on Boost::system which is missing. Please check the list of components passed to find_package( Boost ...)" )
      return() 
    endif()
    add_library( Boost::asio INTERFACE IMPORTED )
    if( MINGW )
      set_target_properties( Boost::asio PROPERTIES INTERFACE_LINK_LIBRARIES "Boost::system;ws2_32" )
    else()
      set_target_properties( Boost::asio PROPERTIES INTERFACE_LINK_LIBRARIES "Boost::system" )
    endif()
    
    if( MSVC AND ( CMAKE_CXX_COMPILER_ID STREQUAL "Intel" ) )
      if( Boost_VERSION EQUAL 106600 )
        set_target_properties( Boost::asio PROPERTIES INTERFACE_COMPILE_DEFINITIONS "BOOST_ASIO_MSVC=${MSVC_VERSION}" )
      endif()
    endif()
  endif()  
endfunction()


# bb_add_target_CopyBoostRuntimeFiles( filesystem chrono ... )
macro( bb_add_target_CopyBoostRuntimeFiles boost_comp_ )
  if( WIN32 AND NOT Boost_USE_STATIC_LIBS )
    if( NOT TARGET CopyQt5RuntimeFiles )
      bb_get_boost_dlls( _boost_bin_dir _boost_dll_list_release _boost_dll_list_debug ${ARGV} )
      unset( _boost_dll_file_list )
      foreach( _fname ${_boost_dll_list_release} )
        list( APPEND _boost_dll_file_list "$<$<NOT:$<CONFIG:Debug>>:${_fname}>" )
      endforeach()      
      foreach( _fname ${_boost_dll_list_debug} )
        list( APPEND _boost_dll_file_list "$<$<CONFIG:Debug>:${_fname}>" )
      endforeach()
      message( STATUS "bb_add_target_CopyBoostRuntimeFiles: creating custom target" )
      if( CMAKE_VERSION VERSION_GREATER_EQUAL 3.8 )
        add_custom_target( CopyBoostRuntimeFiles ALL ${CMAKE_COMMAND} -E make_directory
                             $<$<CONFIG:Debug>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}>
                             $<$<CONFIG:Release>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}>
                             $<$<CONFIG:RelWithDebInfo>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO}>
                             $<$<CONFIG:MinSizeRel>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL}>
                           COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_boost_dll_file_list}
                             $<$<CONFIG:Debug>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}>
                             $<$<CONFIG:Release>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}>
                             $<$<CONFIG:RelWithDebInfo>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO}>
                             $<$<CONFIG:MinSizeRel>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL}>
                           WORKING_DIRECTORY ${_boost_bin_dir} 
                           VERBATIM COMMAND_EXPAND_LISTS )
      else()
        add_custom_target( CopyBoostRuntimeFiles ALL ${CMAKE_COMMAND} -E make_directory
                             $<$<CONFIG:Debug>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}>
                             $<$<CONFIG:Release>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}>
                             $<$<CONFIG:RelWithDebInfo>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO}>
                             $<$<CONFIG:MinSizeRel>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL}>
                           COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_boost_dll_file_list}
                             $<$<CONFIG:Debug>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}>
                             $<$<CONFIG:Release>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}>
                             $<$<CONFIG:RelWithDebInfo>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO}>
                             $<$<CONFIG:MinSizeRel>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL}>
                           WORKING_DIRECTORY ${_boost_bin_dir} )
      endif()
    endif()
  endif()
endmacro()


function( _bb_get_boost_targets_recursive boost_targets_ boost_comp_ )
  set( _boost_comp_list ${boost_comp_} ${ARGN} )
  unset( _boost_target_list )
  foreach( _comp ${_boost_comp_list} )
    list( APPEND _boost_target_list Boost::${_comp} )
    get_target_property( _prop_value Boost::${_comp} "INTERFACE_LINK_LIBRARIES" )
    if( _prop_value )
      foreach( _lnk_lib ${_prop_value} )
        if( _lnk_lib MATCHES "^Boost::[a-z_A-Z]" )
          list( APPEND _boost_target_list ${_lnk_lib} )
        endif()
      endforeach()
    endif()
  endforeach()
  list( REMOVE_DUPLICATES _boost_target_list )
  set( ${boost_targets_} ${_boost_target_list} PARENT_SCOPE )
endfunction()


function( bb_get_boost_dlls boost_bin_dir_ boost_dll_list_release_ boost_dll_list_debug_ boost_comp_ )
  set( _boost_comp_list ${boost_comp_} ${ARGN} )
  # expand BOOST components into a list of BOOST targets including dependent boost DLLs.
  _bb_get_boost_targets_recursive( _boost_targets ${_boost_comp_list} )
  
  unset( _boost_bin_dir )
  unset( _boost_dll_list_release )
  unset( _boost_dll_list_debug )
  foreach( _tgt IN LISTS _boost_targets )
    # check for INTERFACE_LIBRARIES which must be skipped as they don't support a IMPORTED_LOCATION properties.
    get_target_property( _prop_value ${_tgt} TYPE )
    if( _prop_value )
      # message( STATUS "target ${_tgt} is ${_prop_value}" )
      if( _prop_value STREQUAL "INTERFACE_LIBRARY" )
        #message( STATUS "bb_get_boost_dlls(): skipping target ${_tgt}" )
        continue()
      endif()
    else()
      message( WARNING "target ${_tgt} does not have a TYPE property, looks like a bug." ) 
    endif()
    # query release component paths and extract the DLL file name.
    bb_boost_query_lib_component_path( _comp_paths ${_tgt} RELEASE )
    list( GET _comp_paths 0 _boost_dll_path )
    get_filename_component( _boost_dll_fname ${_boost_dll_path} NAME )
    list( APPEND _boost_dll_list_release ${_boost_dll_fname} )
    if( NOT DEFINED _boost_bin_dir )
      get_filename_component( _boost_bin_dir ${_boost_dll_path} DIRECTORY )
    endif()
    # query debug component paths and extract the DLL file name.
    bb_boost_query_lib_component_path( _comp_paths ${_tgt} DEBUG )
    list( GET _comp_paths 0 _boost_dll_path )
    get_filename_component( _boost_dll_fname ${_boost_dll_path} NAME )
    list( APPEND _boost_dll_list_debug ${_boost_dll_fname} )
  endforeach()
  set( ${boost_bin_dir_} ${_boost_bin_dir} PARENT_SCOPE )
  set( ${boost_dll_list_release_} ${_boost_dll_list_release} PARENT_SCOPE )
  set( ${boost_dll_list_debug_} ${_boost_dll_list_debug} PARENT_SCOPE )
endfunction()

function( _bb_boost_guess_lib_component_dll_path dll_path_ lib_path_ )

  #/home/rauthenberg/projects/Boost_1_63_0-c++11/lib/gcc-5.4/x86_64/libboost_system-gcc54-mt-d-1_63.lib [.dll.a]
  #                                             /bin/<toolset>/<target>/<libname>.dll
  # message( STATUS "_bb_boost_guess_lib_component_dll_path: lib_path_=${lib_path_}" )
  get_filename_component( _libname "${lib_path_}" NAME_WE )
  # message( STATUS "_bb_boost_guess_lib_component_dll_path: _libname=${_libname}" )
  get_filename_component( _libdir "${lib_path_}" DIRECTORY )
  get_filename_component( _target_dir "${_libdir}" NAME )
  # message( STATUS "_bb_boost_guess_lib_component_dll_path: _target_dir=${_target_dir}" )
  get_filename_component( _libdir_tmp "${_libdir}/.." REALPATH )
  get_filename_component( _toolset_dir "${_libdir_tmp}" NAME )
  # message( STATUS "_bb_boost_guess_lib_component_dll_path: _toolset_dir=${_toolset_dir}" )
  get_filename_component( _bin_dir "${_libdir}/../../../bin/${_toolset_dir}/${_target_dir}" REALPATH )
  #message( STATUS "_bb_boost_guess_lib_component_dll_path: _bin_dir=${_bin_dir}" )
  set( ${dll_path_} "${_bin_dir}/${_libname}.dll" PARENT_SCOPE )
endfunction()

function( bb_boost_query_lib_component_path comp_paths_ target_ )
  set( _lib_configuration "RELEASE" )
  if( ${ARGC} GREATER 3 )
    message( FATAL_ERROR "bb_boost_query_lib_component_path() called with too many arguments." )
  elseif( ${ARGC} EQUAL 3 )
    set( _lib_configuration ${ARGV2} )
  endif()
  
  get_target_property( _prop_value ${target_} "IMPORTED_LOCATION_${_lib_configuration}" )
  if( _prop_value )
    #message( STATUS "${target_}: IMPORTED_LOCATION_${_lib_configuration}=${_prop_value}" )
    set( _imported_location "${_prop_value}" )
  else()
    message( FATAL_ERROR "bb_boost_query_lib_component_path: IMPORTED_LOCATION_${_lib_configuration} not defined for target ${target_}." )
  endif()
  
  unset( _comp_paths )
  if( WIN32 )
    unset( _imported_implib )
    get_target_property( _prop_value ${target_} "IMPORTED_IMPLIB_${_lib_configuration}" )
    if( _prop_value )
      #message( STATUS "${target_}: IMPORTED_IMPLIB=${_prop_value}" )
      set( _imported_implib "${_prop_value}" )
    endif()
    if( Boost_USE_STATIC_LIBS )
      if( DEFINED _imported_implib )
        set( _comp_paths "${_imported_implib}" )
      else()
        set( _comp_paths "${_imported_location}" )
      endif()
    else()
      # shared Boost library: return -> dll_path;lib_path
      if( NOT DEFINED _imported_implib )
        # findBoost() seems to ignore DLL paths and does not define IMPORTED_IMPLIB_<config> properties.  
        set( _imported_implib "${_imported_location}" )
        _bb_boost_guess_lib_component_dll_path( _imported_location "${_imported_implib}" )
      endif()
      set( _comp_paths "${_imported_location}" "${_imported_implib}" )
    endif()
  else()
    set( _comp_paths "${_imported_location}" )
  endif()
  if( DEFINED _comp_paths )
    set( ${comp_paths_} "${_comp_paths}" PARENT_SCOPE )
  endif()
endfunction( bb_boost_query_lib_component_path )

#
# Include functions and macros to build Boost libraries.
# 
include( ${CMAKE_CURRENT_LIST_DIR}/BBuildEnvBuildBoost.cmake )


