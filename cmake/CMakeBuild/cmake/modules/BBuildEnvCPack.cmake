# BBuildEnvCPack.cmake
#


function( bb_get_default_cpack_generator cpack_gen_ )
  unset( _cpack_gen )
  if( CMAKE_HOST_APPLE )
    set( _cpack_gen "TGZ" )
  elseif( CMAKE_HOST_WIN32 )
    set( _cpack_gen "ZIP" )
  elseif( CMAKE_HOST_UNIX )
    bb_get_linux_pkg_fmt( _pkg_fmt )  # either deb or rpm
    string( TOUPPER ${_pkg_fmt} _cpack_gen )
  else()
    message( FATAL_ERROR "bb_get_default_cpack_generator(): not implemented on current platform yet." )  
  endif()
  if( DEFINED _cpack_gen )
    set( ${cpack_gen_} "${_cpack_gen}" PARENT_SCOPE )
  endif()
endfunction( bb_get_default_cpack_generator )


function( bb_get_default_package_file_name pkg_file_name_ cpack_gen_ pkg_name_ pkg_version_ )
  
  bb_get_os_version( _os_version )
  if( _os_version MATCHES "^([0-9]+)\\.([0-9]+).*" )
    # Use <major>.<minor> on all systems.
    set( _os_version_major_minor "${CMAKE_MATCH_1}.${CMAKE_MATCH_2}" )
  endif()

  bb_get_target_arch( _target_arch )  # x86_64

  set( _re_cpack_archive_fmts "^(TGZ|ZIP|7Z)$" ) 
  if( CMAKE_HOST_WIN32 )
    set( _pkg_file_name "${pkg_name_}-${pkg_version_}-windows-${_os_version_major_minor}-${_target_arch}" )    
    if( ${cpack_gen_} MATCHES "${_re_cpack_archive_fmts}" )
      set( ${pkg_file_name_} ${_pkg_file_name} PARENT_SCOPE )
    elseif( ${cpack_gen_} STREQUAL "DEB" )
      set( ${pkg_file_name_} "${_pkg_file_name}.deb" PARENT_SCOPE )
    elseif( ${cpack_gen_} STREQUAL "RPM" )
      set( ${pkg_file_name_} "${_pkg_file_name}.rpm" PARENT_SCOPE )      
    else()
      message( FATAL_ERROR "bb_get_default_package_file_name(): CPACK_GENERATOR ${cpack_gen_} not supported yet." )
    endif()     
  elseif( CMAKE_HOST_APPLE )
    set( _pkg_file_name "${pkg_name_}-${pkg_version_}-macosx-${_os_version_major_minor}" )
    if( ${cpack_gen_} MATCHES "${_re_cpack_archive_fmts}" )
      set( ${pkg_file_name_} ${_pkg_file_name} PARENT_SCOPE )
    elseif( ${cpack_gen_} STREQUAL "DEB" )
      set( ${pkg_file_name_} "${_pkg_file_name}.deb" PARENT_SCOPE )
    elseif( ${cpack_gen_} STREQUAL "RPM" )
      set( ${pkg_file_name_} "${_pkg_file_name}.rpm" PARENT_SCOPE )
    else()
      message( FATAL_ERROR "bb_get_default_package_file_name(): CPACK_GENERATOR ${cpack_gen_} not supported yet." )
    endif()
  else()
    bb_get_linux_distro_name( _distro_name )
    if( ${ARGC} GREATER 4 )
      set( _pkg_arch "${ARGV4}" )
    else()
      bb_get_linux_pkg_arch( _pkg_arch )
    endif()
    if( ${cpack_gen_} STREQUAL "DEB" )
      set( ${pkg_file_name_} "${pkg_name_}_${pkg_version_}_${_distro_name}-${_os_version_major_minor}_${_pkg_arch}.deb" PARENT_SCOPE )
    elseif( ${cpack_gen_} STREQUAL "RPM" )
      set( ${pkg_file_name_} "${pkg_name_}-${pkg_version_}-${_distro_name}-${_os_version_major_minor}.${_pkg_arch}.rpm" PARENT_SCOPE )
    elseif( ${cpack_gen_} MATCHES "${_re_cpack_archive_fmts}" )
      set( ${pkg_file_name_} "${pkg_name_}-${pkg_version_}-${_distro_name}-${_os_version_major_minor}-${_target_arch}" PARENT_SCOPE )
    else()
      message( FATAL_ERROR "bb_get_default_package_file_name(): CPACK_GENERATOR ${cpack_gen_} not supported yet." )
    endif()  
  endif()

endfunction( bb_get_default_package_file_name )

# Optional arguments: SRC
function( bb_get_default_sdk_filename sdk_filename_ sdk_basename_ version_h_file_ )

  set( _make_src_sdk_filename OFF ) 

  if( ${ARGC} GREATER 4 )
    message( FATAL_ERROR "bb_get_default_sdk_filename(): called with too many arguments." )
  elseif( ${ARGC} EQUAL 4 )
    if( NOT ${ARGV3} STREQUAL "SRC" )
      message( FATAL_ERROR "bb_get_default_sdk_filename(): 4th argument ${ARGV3} not understood, expected SRC." )
    endif()
    set( _make_src_sdk_filename ON )   
  endif()
  
  bb_get_version_from_h_file( _version ${version_h_file_} )
  
  if( _make_src_sdk_filename )
    if( CMAKE_HOST_WIN32 )
      set( _host_system_suffix "windows" )  
    elseif( CMAKE_HOST_APPLE )
      set( _host_system_suffix "macosx" )
    else()
      set( _host_system_suffix "linux" )
    endif()
    set( ${sdk_filename_} "${sdk_basename_}-${_version}-src-${_host_system_suffix}" PARENT_SCOPE )
  else()
  
    bb_get_os_version( _os_version )
    if( _os_version MATCHES "^([0-9]+)\\.([0-9]+).*" )
      # Use <major>.<minor> on all systems.
      set( _os_version_major_minor "${CMAKE_MATCH_1}.${CMAKE_MATCH_2}" )
    endif()
  
    bb_get_target_arch( _target_arch )  # x86_64 or x86
  
    _bb_get_cxx_compiler_version_major_minor( _compiler_version_major_minor )
    
    if( MINGW ) 
      set( _compiler_suffix "gcc-mingw-${_compiler_version_major_minor}" )
    elseif( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" )
      set( _compiler_suffix "gcc-${_compiler_version_major_minor}" )
    elseif( CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" )
      set( _compiler_suffix "msvc-${_compiler_version_major_minor}" )
    elseif( CMAKE_CXX_COMPILER_ID MATCHES "^(AppleClang|Clang)$" )
      set( _compiler_suffix "clang-${_compiler_version_major_minor}" )
    elseif( CMAKE_CXX_COMPILER_ID STREQUAL "Intel" )
      set( _compiler_suffix "intel-${_compiler_version_major_minor}" )
    elseif( CMAKE_CXX_COMPILER_ID STREQUAL "Cray" )
      set( _compiler_suffix "cray-${_compiler_version_major_minor}" )            
    endif()
    
    if( CMAKE_HOST_WIN32 )
      set( _host_system_suffix "windows-${_os_version_major_minor}" )  
    elseif( CMAKE_HOST_APPLE )
      set( _host_system_suffix "macosx-${_os_version_major_minor}" )
    else()
      bb_get_linux_distro_name( _distro_name )
      set( _host_system_suffix "${_distro_name}-${_os_version_major_minor}" )
    endif()
    set( ${sdk_filename_} "${sdk_basename_}-${_version}-${_host_system_suffix}-${_compiler_suffix}-${_target_arch}" PARENT_SCOPE )
  endif()
  
endfunction( bb_get_default_sdk_filename )

macro( bb_find_inno_setup inno_setup_cmd_ )
  bb_get_program_files_x86( _inno_setup_progfiles_x86_dir )
  find_program( ${inno_setup_cmd_} "iscc" PATHS "${_inno_setup_progfiles_x86_dir}/Inno Setup 5" )
  unset( _inno_setup_progfiles_x86_dir )
endmacro()
