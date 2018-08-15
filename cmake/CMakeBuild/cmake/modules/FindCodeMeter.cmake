#[[.rst:
FindCodeMeter
-------------

Find the installed CodeMeter development kit.

Result Variables 
^^^^^^^^^^^^^^^^

This module sets the following variables:

``CodeMeter_FOUND``
  True, if a CodeMeter development kit is found.
``CodeMeter_VERSION``
  Detected CodeMeter version.

Imported Targets
^^^^^^^^^^^^^^^^

This module defines the :prop_tgt:`IMPORTED` targets:

``CodeMeter::WibuCmHeaderOnly``
  This target defines a header-only library exposing the CodeMeter core API. A protected application or
  code module is required to load the installed CodeMeter runtime component explicitly.
``CodeMeter::WibuCm``
  Defines a library target referring to the CodeMeter core API.

#]]

#set( _CodeMeter_required_vars CodeMeter_DevKit_ROOT_DIR )

function( CodeMeter_getDevKitVersion version_var_ header_file_ )

  #message( STATUS "CodeMeter_get_devkit_version(): ${version_var} ${header_file}" )
  
  #define CODEMETER_VERSION 630
  set( _regex_cm_version_def "#define[\t ]+CODEMETER_VERSION[\t ]+([0-9]+)([0-9][0-9])" )
  
  file( STRINGS "${header_file_}" _cm_header_version_line REGEX "${_regex_cm_version_def}" )
  # message( STATUS "codemeter version line: '${_cm_header_version_line}'" )
  if( "${_cm_header_version_line}" MATCHES "${_regex_cm_version_def}")
     # set(Boost_${v} "${CMAKE_MATCH_1}")
    # message( STATUS "cm version (major.minor): ${CMAKE_MATCH_1} ${CMAKE_MATCH_2}" )
    set( ${version_var_} "${CMAKE_MATCH_1}.${CMAKE_MATCH_2}" PARENT_SCOPE )
  endif()
endfunction( CodeMeter_getDevKitVersion )


function( CodeMeter_findRuntimeFile runtime_file_ runtime_dir_root_ runtime_type_ )
  if( CMAKE_HOST_WIN32 OR MINGW )
    set( _runtime_file_list "${runtime_dir_root_}/windows/CodeMeterRuntime.exe" ) 
  elseif( CMAKE_HOST_APPLE )
    file( GLOB _runtime_file_list "${runtime_dir_root_}/macosx/x86_64/CmRuntimeUser*.dmg" )
  elseif( CMAKE_HOST_UNIX )
    bb_get_linux_pkg_fmt( _pkg_fmt )
    bb_get_target_arch( _target_arch )
    if( ${runtime_type_} STREQUAL "runtime_lite" )
      set( _runtime_lite_suffix "-lite" )      
    endif()
    if( _pkg_fmt STREQUAL "deb" )
      file( GLOB _runtime_file_list "${runtime_dir_root_}/linux/${_target_arch}/codemeter${_runtime_lite_suffix}_[1-9]*.deb" )
    else()
      file( GLOB _runtime_file_list "${runtime_dir_root_}/linux/${_target_arch}/CodeMeter${_runtime_lite_suffix}-[1-9]*.rpm" )
    endif()
  endif()
  # 
  list( LENGTH _runtime_file_list _runtime_file_list_len )
  # message( STATUS "${_runtime_file_list} ${_runtime_file_list_len}" )
  if( _runtime_file_list_len EQUAL 1 )
    list( GET _runtime_file_list 0 _runtime_file )
  elseif( _runtime_file_list_len LESS 1 )
    set( ${runtime_file_} "${runtime_file_}-NOTFOUND" PARENT_SCOPE )
    return()
  else()
    message( FATAL_ERROR "FindCodeMeter: unexpected number of CodeMeter runtime files found, please contact technical support." )
  endif()
  # message( STATUS "_runtime_file: ${_runtime_file}" )
  if( EXISTS "${_runtime_file}" )
    if( CodeMeter_ENABLE_RUNTIME_VERSION_CHECKS )
      CodeMeter_checkRuntimeVersion( ${CodeMeter_VERSION} "${_runtime_file}" )
    endif()
    set( ${runtime_file_} "${_runtime_file}" PARENT_SCOPE )
  else()
    set( ${runtime_file_} "${runtime_file_}-NOTFOUND" PARENT_SCOPE )
  endif()
endfunction( CodeMeter_findRuntimeFile )

macro( CodeMeter_addRuntimeTarget target_ runtime_dir_ runtime_type_ )
  CodeMeter_findRuntimeFile( _cm_runtime_file "${runtime_dir_}" ${runtime_type_} )
  list( APPEND CodeMeter_tmp_vars "_cm_runtime_file" )
  if( _cm_runtime_file )
    #message( STATUS "FindCodeMeter: runtime file=${_cm_runtime_file}" )
    add_custom_target( ${target_} ${CMAKE_COMMAND} -E  make_directory ${CMAKE_SOURCE_DIR}/deploy
                       COMMAND ${CMAKE_COMMAND} -E copy ${_cm_runtime_file} ${CMAKE_SOURCE_DIR}/deploy
                       VERBATIM )
    if( CodeMeter_DEBUG )                       
      message( STATUS "Creating custom target ${target_}" )
    endif()
  else()
    message( FATAL_ERROR "FindCodeMeter: CodeMeter runtime redistributable cannot be found in ${runtime_dir_}" )
  endif()
endmacro()

function( CodeMeter_checkRuntimeVersion cmdevkit_version_ cm_runtime_file_ )
  if( CMAKE_HOST_WIN32 OR MINGW )
    # cm runtime filename comes without a version number: CodeMeterRuntime.exe
    # Workaround: linux filename.
    unset( _cm_rt_basenm )
    get_filename_component( _cm_rt_dir "${cm_runtime_file_}" DIRECTORY )
    set( _cm_rt_linux_dir "${_cm_rt_dir}/../linux/x86_64" )
    if( EXISTS "${_cm_rt_linux_dir}" )
      file( GLOB _cm_rt_basenm_list RELATIVE ${_cm_rt_linux_dir} "${_cm_rt_linux_dir}/*amd64.deb" )
      #message( STATUS "CodeMeter_checkRuntimeVersion(): analyzing file ${_cm_rt_basenm_list} to guess version" )
      # In general, more than one filename may be returned but a single filename is needed below.
      list( LENGTH _cm_rt_basenm_list _cm_rt_basenm_list_len )
      if( _cm_rt_basenm_list_len GREATER_EQUAL 1 )
        list( GET _cm_rt_basenm_list 0 _cm_rt_basenm )
      endif()
    endif()
  else()
    #message( STATUS "FindCodeMeter: checking codemeter runtime version" )
    get_filename_component( _cm_rt_basenm "${cm_runtime_file_}" NAME )
  endif()
  if( DEFINED _cm_rt_basenm )
    # message( STATUS "FindCodeMeter: checking codemeter runtime version using ${_cm_rt_basenm}" )
    # e.g. codemeter_6.40.2402.501_amd64.deb
    #      codemeter-lite_6.40.2402.501_amd64.deb
    #      CodeMeter-6.40.2402-501.x86_64.rpm
    #      CodeMeter-lite-6.40.2402-501.x86_64.rpm
    #      
    #      CmRuntimeUser_6.40.2402.501.dmg
    if( _cm_rt_basenm MATCHES "^[^0-9]+([0-9]+\\.[0-9]+)" )
      #message( STATUS "CodeMeter runtime version <major.minor> extracted: ${CMAKE_MATCH_1}" )
      set( _cm_rt_version ${CMAKE_MATCH_1} )
      if( _cm_rt_version VERSION_GREATER ${cmdevkit_version_} )
        message( FATAL_ERROR "\
        
  CodeMeter runtime intended for deployment is newer than the CodeMeter development kit ${cmdevkit_version_}
  installed. Such a combination is disabled by default, please check the SVN externals,
  the CodeMeter development kit installed or contact technical support for further assistance.
"              )   
      endif()
    endif()
  endif()
endfunction()

# -
# end of functions and macros
# -

if( CodeMeter_DEBUG )
  message( STATUS "${CMAKE_CURRENT_LIST_FILE}: starting ..." )
endif()
unset( CodeMeter_tmp_vars )
if( NOT DEFINED CodeMeter_DISABLE_VERSION_CHECKS )
  set( CodeMeter_DISABLE_VERSION_CHECKS FALSE )
endif()
if( ( NOT DEFINED CodeMeter_ENABLE_RUNTIME_VERSION_CHECKS ) OR CodeMeter_DISABLE_VERSION_CHECKS )
  set( CodeMeter_ENABLE_RUNTIME_VERSION_CHECKS FALSE )
endif()


if( CMAKE_HOST_WIN32 )
  bb_get_program_files_x86( _CodeMeter_progfiles_x86 )
  find_path( CodeMeter_DevKit_INCLUDE_DIR CodeMeter.h PATHS "${_CodeMeter_progfiles_x86}/CodeMeter/DevKit" PATH_SUFFIXES include NO_DEFAULT_PATH )
	list( APPEND CodeMeter_tmp_vars "_CodeMeter_progfiles_x86" )
elseif( CMAKE_HOST_APPLE )
  find_path( CodeMeter_DevKit_INCLUDE_DIR CodeMeter.h PATHS "/Applications/WIBU-SYSTEMS DevKit/CodeMeter" PATH_SUFFIXES include NO_DEFAULT_PATH )
elseif( CMAKE_HOST_UNIX )
  if( MINGW )
    find_path( CodeMeter_DevKit_INCLUDE_DIR CodeMeter.h PATHS /usr/include NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH )
  else()
    find_path( CodeMeter_DevKit_INCLUDE_DIR CodeMeter.h PATHS /usr/include NO_DEFAULT_PATH )
  endif()
endif()

mark_as_advanced( CodeMeter_DevKit_INCLUDE_DIR CodeMeter_LIBWIBUCM ) # hide cache variable

if( CodeMeter_DevKit_INCLUDE_DIR )
  CodeMeter_getDevKitVersion( CodeMeter_VERSION "${CodeMeter_DevKit_INCLUDE_DIR}/CodeMeter.h" )
  # message( STATUS "CodeMeter Development Kit: ${CodeMeter_VERSION}" )
  #message( STATUS "CodeMeter include files found at: ${CodeMeter_DevKit_INCLUDE_DIR}" )
  
  # Create CodeMeter_DevKit_ROOT_DIR
  get_filename_component( CodeMeter_DevKit_ROOT_DIR "${CodeMeter_DevKit_INCLUDE_DIR}/.." REALPATH )

  if( NOT TARGET CodeMeter::WibuCmHeaderOnly )
    
    if( CMAKE_HOST_WIN32 )
      set( CodeMeter_INCLUDE_DIRS "${CodeMeter_DevKit_INCLUDE_DIR}" )
      if( NOT MINGW )
        # CodeMeter DevKit does not come with MinGW compliant import libraries and therefore CodeMeter_LIBRARY_DIRS is not
        # defined for MinGW.
        set( CodeMeter_LIBRARY_DIRS "${CodeMeter_DevKit_ROOT_DIR}/lib" )
      endif()    
    elseif( CMAKE_HOST_APPLE )
      set( CodeMeter_INCLUDE_DIRS "${CodeMeter_DevKit_INCLUDE_DIR}" )
    endif()  
  
    add_library( CodeMeter::WibuCmHeaderOnly INTERFACE IMPORTED )
    set_target_properties( CodeMeter::WibuCmHeaderOnly PROPERTIES INTERFACE_COMPILE_DEFINITIONS NO_AUTO_LINKING_CODEMETER )
    if( CodeMeter_INCLUDE_DIRS )
      # message( STATUS "inserting cm include dir: ${CodeMeter_INCLUDE_DIRS}" )                      
      set_target_properties( CodeMeter::WibuCmHeaderOnly PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CodeMeter_INCLUDE_DIRS}" )
    endif()
    
    if( NOT MINGW )
      # WibuCmLib -> link at build time to CodeMeter; this won't work for mingw due to missing import libraries.
      if( WIN32 )
        add_library( CodeMeter::WibuCm SHARED IMPORTED )
        if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
          set_target_properties( CodeMeter::WibuCm PROPERTIES IMPORTED_IMPLIB "${CodeMeter_LIBRARY_DIRS}/WibuCm64.lib" )
        else()
          set_target_properties( CodeMeter::WibuCm PROPERTIES IMPORTED_IMPLIB "${CodeMeter_LIBRARY_DIRS}/WibuCm32.lib" )
        endif()
      elseif( APPLE )
        # WibuCmMacX
        find_library( CodeMeter_LIBWIBUCM WibuCmMacX )
        if( CodeMeter_LIBWIBUCM )
          #message( STATUS "FindCodeMeter: CodeMeter_LIBWIBUCM=${CodeMeter_LIBWIBUCM}" )
          add_library( CodeMeter::WibuCm SHARED IMPORTED )
          set_target_properties( CodeMeter::WibuCm PROPERTIES IMPORTED_LOCATION "${CodeMeter_LIBWIBUCM}/WibuCmMacX" )
        endif()
      else()
        # Linux host system assumed
        find_library( CodeMeter_LIBWIBUCM NAMES libwibucm.so libwibucmlin64.so )
        if( CodeMeter_LIBWIBUCM )
          #message( STATUS "FindCodeMeter: CodeMeter_LIBWIBUCM=${CodeMeter_LIBWIBUCM}" )      
          add_library( CodeMeter::WibuCm SHARED IMPORTED )
          set_target_properties( CodeMeter::WibuCm PROPERTIES IMPORTED_LOCATION "${CodeMeter_LIBWIBUCM}" )
        endif()
      endif()
      if( TARGET CodeMeter::WibuCm AND CodeMeter_INCLUDE_DIRS )
        # message( STATUS "inserting cm include dir: ${CodeMeter_INCLUDE_DIRS}" )                      
        set_target_properties( CodeMeter::WibuCm PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CodeMeter_INCLUDE_DIRS}" )
      endif() 
    endif()
    
    foreach( _comp ${CodeMeter_FIND_COMPONENTS} )
      set( CodeMeter_${_comp}_FOUND FALSE )
      #message( STATUS "Processing component ${_comp}" )
      if( _comp MATCHES "^(runtime|runtime_lite)$" )
        set( CodeMeter_${_comp}_FOUND FALSE )
        set( _cm_runtime_dirs "${CMAKE_SOURCE_DIR}/CodeMeterRuntime" "${CMAKE_SOURCE_DIR}/../${PROJECT_NAME}-CodeMeterSoftware/CodeMeterRuntime" )
        list( APPEND CodeMeter_tmp_vars "_cm_runtime_dirs" )
        foreach( _cm_rtl_dir IN LISTS _cm_runtime_dirs )
          if( EXISTS "${_cm_rtl_dir}" )
            CodeMeter_addRuntimeTarget( CodeMeterCopyRuntime "${_cm_rtl_dir}" ${_comp} )
            set( CodeMeter_${_comp}_FOUND TRUE )
            break()
          endif()
        endforeach() 
        if( NOT CodeMeter_${_comp}_FOUND )
          if( CodeMeter_FIND_REQUIRED_${_comp}  )
            message( WARNING "FindCodeMeter: requested component ${_comp} not found in ${_cm_runtime_dirs}." )
          endif()
        endif()
      endif()    
    endforeach()       
  endif()
endif()

if( DEFINED CodeMeter_tmp_vars )
  # Dispose any temporary variables avoiding unwanted pollution of the calling namespace.
  list( REMOVE_DUPLICATES CodeMeter_tmp_vars )
  foreach( _v ${CodeMeter_tmp_vars} )
    # message( STATUS "findCodeMeter: unsetting ${_v}" )
    unset( ${_v} )
  endforeach()
  unset( CodeMeter_tmp_vars )
endif()

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args( CodeMeter REQUIRED_VARS CodeMeter_DevKit_ROOT_DIR CodeMeter_DevKit_INCLUDE_DIR VERSION_VAR CodeMeter_VERSION HANDLE_COMPONENTS )

if( CodeMeter_DEBUG )
  message( STATUS "${CMAKE_CURRENT_LIST_FILE}: leaving." )
endif()

