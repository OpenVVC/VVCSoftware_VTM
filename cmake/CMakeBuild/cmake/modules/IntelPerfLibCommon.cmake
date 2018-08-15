#[[.rst:
IntelPerfLibCommon
------------------

Internal utility functions used by Intel Performance Library find modules.

#]]


function( IntelPerfLib_getRedistDir redist_dir_ root_dir_ perflib_id_ )
  
  if( NOT EXISTS "${root_dir_}" )
    message( FATAL_ERROR "Directory '${root_dir_}' does not exist." )
  endif()
  
  # 
  string( TOLOWER ${perflib_id_} _perflib_id_lc )
  
  # On Windows we may run into trouble if a path includes ".." and therefore 
  # the get_filename_component() logic.
  get_filename_component( _root_dir_parent "${root_dir_}" DIRECTORY )

  if( WIN32 )
    if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
      set( _redist_dir "${_root_dir_parent}/redist/intel64_win/${_perflib_id_lc}" )
    else()
      set( _redist_dir "${_root_dir_parent}/redist/ia32_win/${_perflib_id_lc}" )
    endif()
  elseif( APPLE )
    set( _redist_dir "${_root_dir_parent}/${_perflib_id_lc}/lib" )
  elseif( UNIX )
    if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
      set( _redist_dir "${_root_dir_parent}/${_perflib_id_lc}/lib/intel64_lin" )
    else()
      set( _redist_dir "${_root_dir_parent}/${_perflib_id_lc}/lib/ia32_lin" )
    endif()
  else()
    message( FATAL_ERROR "IntelPerfLib_getRedistDir() has not support for the actual platform/configuration." )
  endif()

  if( NOT EXISTS "${_redist_dir}" )
    message( FATAL_ERROR "Directory '${_redist_dir}' does not exist." )
  else()
    set( ${redist_dir_} "${_redist_dir}" PARENT_SCOPE )
  endif()
endfunction()


function( IntelPerfLib_getVersion version_var_ header_file_ perflib_id_ )

  if( NOT EXISTS "${header_file_}" )
    message( FATAL_ERROR "header file '${header_file_}' does not exist." )
  endif()
  
  unset( _version_list )
  string( TOUPPER ${perflib_id_} _perflib_id_uc )
  if( ${_perflib_id_uc} STREQUAL "IPP" )
    set( _regex_version_def "^#define[\t ]+${_perflib_id_uc}_VERSION_(MAJOR|MINOR|UPDATE)[\t ]+([0-9]+)" )
  elseif( ${_perflib_id_uc} MATCHES "^(MKL|DAAL)$" )
    set( _regex_version_def "^#define[\t ]+__INTEL_${_perflib_id_uc}_(_|MINOR__|UPDATE__)[\t ]+([0-9]+)" )
  endif()
  
  file( STRINGS "${header_file_}" _header_version_lines REGEX "${_regex_version_def}" )
  # message( STATUS "extracted header lines: ${_header_version_lines}" )
  foreach( _line ${_header_version_lines} )
    # message( STATUS "processing header line: ${_line}" )
    if( _line MATCHES "${_regex_version_def}" )
      # message( STATUS "${CMAKE_MATCH_1} -> ${CMAKE_MATCH_2}" )
      list( APPEND _version_list "${CMAKE_MATCH_2}" )
    endif()
  endforeach()
  #message( STATUS "_version_list: ${_version_list}" )
  if( DEFINED _version_list )
    # Compose a doted separated version string
    string( REPLACE ";" "." _version_str "${_version_list}" )
    #message( STATUS "_version_str: ${_version_str}" )
  else()
    message( FATAL_ERROR "IntelPerfLib_getVersion() failed to extract version information, please contact the maintainer for further support." )
  endif()
  set( ${version_var_} "${_version_str}" PARENT_SCOPE )
endfunction()


# Returns a list of installations paths holding the specified performance library category.
function( IntelPerfLib_getRootDirPaths root_dir_paths_ version_list_ perflib_id_ )
  string( TOLOWER ${perflib_id_} _perflib_id_lc )
  unset( _version_list )
  unset( _root_dir_paths )
  
  if( WIN32 )
    set( _perflib_top_dir "C:/Program Files (x86)/IntelSWTools" )
    set( _perflib_platform_dir "windows" )
  elseif( APPLE )
    set( _perflib_top_dir "/opt/intel" )
    set( _perflib_platform_dir "mac" )
  else()
    set( _perflib_top_dir "/opt/intel" )
    set( _perflib_platform_dir "linux" )
  endif()
  
  if( EXISTS "${_perflib_top_dir}" )
    file( GLOB _perflib_inst_dir_list "${_perflib_top_dir}/compilers_and_libraries_*.*.*" )
  
    foreach( _pth IN LISTS _perflib_inst_dir_list )
      if( ${_perflib_id_lc} STREQUAL "daal" )
        set( _version_header_file "${_pth}/${_perflib_platform_dir}/${_perflib_id_lc}/include/${_perflib_id_lc}/services/library_version_info.h" )
      elseif( ${_perflib_id_lc} STREQUAL "ipp" )
        set( _version_header_file "${_pth}/${_perflib_platform_dir}/${_perflib_id_lc}/include/${_perflib_id_lc}version.h" )
      elseif( ${_perflib_id_lc} STREQUAL "mkl" )
        set( _version_header_file "${_pth}/${_perflib_platform_dir}/${_perflib_id_lc}/include/${_perflib_id_lc}_version.h" )
      else()
        set( _version_header_file "${_pth}/${_perflib_platform_dir}/${_perflib_id_lc}/include/${_perflib_id_lc}_version.h" )
      endif()
      if( EXISTS "${_version_header_file}" )
        IntelPerfLib_getVersion( _version_str "${_version_header_file}" ${perflib_id_} )
        #message( STATUS "${_pth}: found ${perflib_id_} ${_version_str}" )
        list( APPEND _version_list ${_version_str} )
        list( APPEND _root_dir_paths "${_pth}/${_perflib_platform_dir}/${_perflib_id_lc}" )
      endif()
    endforeach()
  endif()
  if( ( DEFINED _root_dir_paths ) AND ( DEFINED _version_list ) )
    set( ${version_list_} "${_version_list}" PARENT_SCOPE )
    set( ${root_dir_paths_} "${_root_dir_paths}" PARENT_SCOPE )
  else()
    set( ${root_dir_paths_} "${root_dir_paths_}-NOTFOUND" PARENT_SCOPE )
  endif()
endfunction() 


# Optional arguments: <version> [EXACT]
function( IntelPerfLib_selectRootDir root_dir_ version_ root_dir_paths_ version_list_ )
  set( _find_exact FALSE )
  set( _find_version LATEST )
  
  if( ARGC GREATER 4 )
    # One or more optional arguments specified.
    if( ARGC EQUAL 5 )
      set( _find_version ${ARGV4} )
    elseif( ARGC EQUAL 6 )
      set( _find_version ${ARGV4} )
      if( ${ARGV5} STREQUAL "EXACT" )
        set( _find_exact TRUE )
      else()
         message( FATAL_ERROR "IntelPerfLib_selectRootDir() argument=${ARGV5} not understood." )
      endif()      
    else()
      message( FATAL_ERROR "IntelPerfLib_selectRootDir() received too many arguments." )
    endif()
  endif()

  # Build a temporary path map associating available version strings with root paths.
  set( _index 0 )
  foreach( _v IN LISTS version_list_ )
    list( GET root_dir_paths_ ${_index} _root_dir )
    #message( STATUS "IntelPerfLib_selectRootDir(): processing ${_root_dir} ${_v}" )
    set( _root_pth_${_v} "${_root_dir}" )
    math( EXPR _index "${_index} + 1" )
  endforeach()

  # Use the standard version sorter to get a list in descending order
  bb_version_list_sort( _version_list_sorted ${version_list_} )
  unset( _root_dir )
  if( _find_version STREQUAL "LATEST" )
    list( GET _version_list_sorted 0 _version )
    #message( STATUS "IntelPerfLib_selectRootDir(): ${_version} selected" )
    set( _root_dir "${_root_pth_${_version}}" ) 
  else()
    # Two possibilities:
    #   - exact version match
    #   - selected version is greater or equal compared to the requested version    
    if( _find_exact )
      foreach( _v IN LISTS _version_list_sorted )
        if( _v VERSION_EQUAL ${_find_version} )
          set( _root_dir "${_root_pth_${_v}}" )
          set( _version ${_v} )
          break()
        endif()
      endforeach()     
    else()
      foreach( _v IN LISTS _version_list_sorted )
        if( _v VERSION_GREATER_EQUAL ${_find_version} )
          set( _root_dir "${_root_pth_${_v}}" )
          set( _version ${_v} )
          break()
        endif()
      endforeach()
    endif()
  endif()
  if( DEFINED _root_dir )
    set( ${root_dir_} "${_root_dir}" PARENT_SCOPE )
    set( ${version_}  "${_version}"  PARENT_SCOPE )
  else()
    set( ${root_dir_} "${root_dir_}-NOTFOUND" PARENT_SCOPE )
  endif()
endfunction()


