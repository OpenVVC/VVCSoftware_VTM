#[[.rst:
BBuildEnvBuildBoost
-------------------

Functions and macros to build Boost libraries from source.

#]]


function( bb_boost_build_bjam bjam_prog_ boost_root_ )
  # Depending on the Boost version the b2 source files are stored in different directories.
  set( _boost_tools_dir "${boost_root_}/tools" )
  set( _bjam_src_dir_list "${_boost_tools_dir}/build/src/engine" )
  list( APPEND _bjam_src_dir_list "${_boost_tools_dir}/build/v2/engine" )
  list( APPEND _bjam_src_dir_list "${_boost_tools_dir}/build/v2/engine/src" )
  list( APPEND _bjam_src_dir_list "${_boost_tools_dir}/jam/src" )

  unset( _bjam_src_dir )
  foreach( _dir IN LISTS _bjam_src_dir_list )
    if( EXISTS "${_dir}" )
      set( _bjam_src_dir "${_dir}" )
      break()
    endif()
  endforeach()

  if( NOT DEFINED _bjam_src_dir )
    message( FATAL_ERROR "bb_boost_build_bjam(): cannot find b2 source directory inside ${boost_root_}, please contact technical support." )
  endif()

  if( EXISTS "${_bjam_src_dir}/bootstrap" )
    file( REMOVE_RECURSE "${_bjam_src_dir}/bootstrap" )
  endif()

  if( CMAKE_HOST_UNIX )
    set( _build_script_launcher "/bin/sh" )
    set( _build_script_cmd_line "${_bjam_src_dir}/build.sh" )
  elseif( CMAKE_HOST_WIN32 )
    set( _build_script_cmd_line "/c" )
    find_program( BB_SHELL_PROG "cmd.exe" )
    #message( STATUS "cmd.exe found:  BB_SHELL_PROG=${BB_SHELL_PROG}" )
    bb_file( TO_NATIVE_PATH "${BB_SHELL_PROG}" _build_script_launcher )
    bb_file( TO_NATIVE_PATH "${_bjam_src_dir}/build.bat" _build_script )

    string( APPEND _build_script_cmd_line " ${_build_script}" )
    
    if( CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" )
      if( MSVC_VERSION VERSION_GREATER_EQUAL 1910 )
        # VS 2017 updated cl to 19.11.x, 19.12.x which is also mapped to msvc-14.1 by Boost.Build/1.66.0
        set( _bjam_toolset "vc141" )
      elseif( MSVC_VERSION VERSION_EQUAL 1900 )
        set( _bjam_toolset "vc14" )
      elseif( MSVC_VERSION VERSION_EQUAL 1800 )
        set( _bjam_toolset "vc12" )
      elseif( MSVC_VERSION VERSION_EQUAL 1700 )
        set( _bjam_toolset "vc11" )
      elseif( MSVC_VERSION VERSION_EQUAL 1600 )
        set( _bjam_toolset "vc10" )
      else()
        message( FATAL_ERROR "bb_boost_build_bjam(): unsupported MSVC version ${MSVC_VERSION} found, please contact technical support." )
      endif()
      string( APPEND _build_script_cmd_line " ${_bjam_toolset}" )
    endif()    
  endif()

  separate_arguments( _build_script_args NATIVE_COMMAND "${_build_script_cmd_line}" )
  message( STATUS "Launching: ${_build_script_launcher} ${_build_script_cmd_line}" )
  execute_process( COMMAND "${_build_script_launcher}" ${_build_script_args} WORKING_DIRECTORY "${_bjam_src_dir}" RESULT_VARIABLE _retv )

  if( _retv EQUAL 0 )
    if( CMAKE_HOST_APPLE )
      set( _bjam_bin_dir "${_bjam_src_dir}/bin.macosxx86_64" )
    elseif( CMAKE_HOST_UNIX )
      set( _bjam_bin_dir "${_bjam_src_dir}/bin.linuxx86_64" )
    elseif( CMAKE_HOST_WIN32 )
      set( _bjam_bin_dir "${_bjam_src_dir}/bin.ntx86" )
    endif()

    unset( _bjam_prog )
    if( CMAKE_HOST_WIN32 )
      set( _bjam_prog_list "b2.exe" "bjam.exe" )
    else()
       set( _bjam_prog_list "b2" "bjam" )
    endif()
    foreach( _prog IN LISTS _bjam_prog_list )
      if( EXISTS "${_bjam_bin_dir}/${_prog}" )
        set( _bjam_prog "${_bjam_bin_dir}/${_prog}" )
        break()
      endif()
    endforeach()
    
    if( ( DEFINED _bjam_prog ) AND ( EXISTS "${_bjam_prog}" ) )
      set( ${bjam_prog_} "${_bjam_prog}" PARENT_SCOPE )
    else()
      set( ${bjam_prog_} "${bjam_prog_}-NOTFOUND" PARENT_SCOPE )
    endif()
  else()
    set( ${bjam_prog_} "${bjam_prog_}-NOTFOUND" PARENT_SCOPE )
    message( STATUS "bjam build failed: retv=${_retv}" )
  endif()
  
  if( EXISTS "${_bjam_src_dir}/bootstrap" )
    file( REMOVE_RECURSE "${_bjam_src_dir}/bootstrap" )
  endif()
endfunction( bb_boost_build_bjam )


function( bb_boost_build_libs boost_root_ boost_lib_dir_ )

  set( _dry_run FALSE )
  set( _rebuild FALSE )

  #set( _dry_run TRUE )
  #set( _rebuild TRUE )

  if( NOT EXISTS "${boost_root_}" )
    message( FATAL_ERROR "Boost root directory ${boost_root_} does not exist." )
  endif()

  if( NOT EXISTS "${boost_root_}/boost/version.hpp" )
    message( FATAL_ERROR "Directory ${boost_root_} does not seem to be a Boost root directory." )
  endif()
  
  if( NOT _rebuild )
    if( EXISTS "${boost_lib_dir_}" )
      return()
    endif()
  endif()

  _bb_get_boost_version( _boost_version "${boost_root_}/boost/version.hpp" )
  message( STATUS "bb_boost_build_libs(): found ${_boost_version} in ${boost_root_}" )
    
  # Pick up the user's home directory
  bb_get_home_dir( _home_dir )
  set( _user_config_file "${_home_dir}/user-config.jam" )
  
  if( EXISTS "${_user_config_file}" )
    message( FATAL_ERROR "Found ${_user_config_file} which will interfere with the Boost build process." )
  endif() 

  set( _tmp_dirs "${boost_root_}/bin.v2" "${boost_root_}/tmp" )
  # The next temporary directory exists in 1.58.0 and later.
  list( APPEND _tmp_dirs "${boost_root_}/libs/config/checks/architecture/bin" )

  #message( STATUS "bb_boost_build_libs(): CMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}" )

  file( RELATIVE_PATH _lib_dir_rel "${boost_root_}" "${boost_lib_dir_}" )
  #message( STATUS "bb_boost_build_libs(): lib_dir_rel=${_lib_dir_rel}" )

  string( REGEX REPLACE "^lib/" "bin/" _bin_dir_rel "${_lib_dir_rel}" )
  #message( STATUS "bb_boost_build_libs(): bin_dir_rel=${_bin_dir_rel}" )

  set( _boost_bin_dir "${boost_root_}/${_bin_dir_rel}" )

  unset( _user_config_content )
  _bb_boost_compose_b2_cmd_line( _b2_cmd_line _user_config_content ${boost_root_} ${_lib_dir_rel} ${_boost_version} )

  if( DEFINED _user_config_content )
    _bb_boost_write_user_config_file( ${_user_config_file} "${_user_config_content}" )
  endif()

  if( _dry_run )
    if( CMAKE_HOST_WIN32 )
      file( TO_NATIVE_PATH "$ENV{USERPROFILE}/bin/PrintArgsTest.exe" _b2_prog )
    else()
      set( _b2_prog "$ENV{HOME}/bin/PrintArgsTest" )
    endif()
  else()
    # Build b2 / bjam
    bb_boost_build_bjam( _b2_prog "${boost_root_}" )
  endif()
  
  if( NOT _dry_run )
    # Remove any existing Windows DLLs
    if( EXISTS "${_boost_bin_dir}" )
      message( STATUS "Removing ${_boost_bin_dir} ... " )
      file( REMOVE_RECURSE "${_boost_bin_dir}" )
    endif()
  endif()
    
  # Remove any temporary directories inside the Boost source tree. 
  _bb_boost_remove_tmp_dirs( ${_tmp_dirs} ) 

  separate_arguments( _b2_args UNIX_COMMAND "${_b2_cmd_line}" )
  message( STATUS "Launching: ${_b2_prog} ${_b2_cmd_line}" )
  if( EXISTS "${_b2_prog}" )
    # In dry run mode _b2_prog may not exist.
    execute_process( COMMAND "${_b2_prog}" ${_b2_args} WORKING_DIRECTORY "${boost_root_}" RESULT_VARIABLE _retv )
    
    if( NOT _retv EQUAL 0 )
      message( WARNING "At least some Boost libraries failed to build." )
    endif()
  endif()
  
  # Get rid of the temporary user-config.jam as soon as possible.
  if( EXISTS "${_user_config_file}" )
    file( REMOVE "${_user_config_file}" )
  endif()  
  
  if( MSVC OR MINGW )
    _bb_boost_relocate_win_dlls( "${boost_root_}" "${boost_lib_dir_}" "${_boost_bin_dir}" )
  endif()
    
  # Remove any temporary directories inside the Boost source tree.
  _bb_boost_remove_tmp_dirs( ${_tmp_dirs} ) 

  # Get rid of the temporary b2 or bjam inside the Boost source tree.
  _bb_boost_remove_bjam( "${_b2_prog}" )

  # List all libraries 
  message( STATUS "Enumerating Boost components in ${boost_lib_dir_}:" )
  unset( _boost_comp_list )
  _bb_boost_list_components( _boost_comp_list "${boost_lib_dir_}" )
  if( DEFINED _boost_comp_list )
    foreach( _comp IN LISTS _boost_comp_list )
      message( STATUS "  ${_comp}" )
    endforeach()
  endif()

  message( STATUS "b2 command line: ${_b2_prog} ${_b2_cmd_line}" )
  if( DEFINED _user_config_content )
    message( STATUS "<user-config.jam>" )
    foreach( _line  IN LISTS _user_config_content )
      message( STATUS "${_line} ;" )
    endforeach()
    message( STATUS "</user-config.jam>" )
  endif()
  message( STATUS "Finished building Boost: ${boost_root_}" )
endfunction( bb_boost_build_libs )

function( _bb_get_boost_version boost_version_str_ boost_version_file_ )

  # Extract Boost_VERSION and Boost_LIB_VERSION from version.hpp
  set(Boost_VERSION 0)
  set(Boost_LIB_VERSION "")
  file(STRINGS "${boost_version_file_}" _boost_VERSION_HPP_CONTENTS REGEX "#define BOOST_(LIB_)?VERSION ")
  set(_Boost_VERSION_REGEX "([0-9]+)")
  set(_Boost_LIB_VERSION_REGEX "\"([0-9_]+)\"")
  foreach(v VERSION LIB_VERSION)
    if("${_boost_VERSION_HPP_CONTENTS}" MATCHES "#define BOOST_${v} ${_Boost_${v}_REGEX}")
      set(Boost_${v} "${CMAKE_MATCH_1}")
    endif()
  endforeach()
  unset(_boost_VERSION_HPP_CONTENTS)

  math(EXPR Boost_MAJOR_VERSION "${Boost_VERSION} / 100000")
  math(EXPR Boost_MINOR_VERSION "${Boost_VERSION} / 100 % 1000")
  math(EXPR Boost_SUBMINOR_VERSION "${Boost_VERSION} % 100")

  set( ${boost_version_str_} "${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}.${Boost_SUBMINOR_VERSION}" PARENT_SCOPE )
endfunction()

function( _bb_boost_list_components comp_list_ boost_lib_dir_ )
  if( NOT EXISTS "${boost_lib_dir_}" )
    message( STATUS "bb_boost_list_components(): library directory ${boost_lib_dir_} does not exist." )
    return()
  endif()
  
  if( MSVC )
    file( GLOB _lib_file_list RELATIVE "${boost_lib_dir_}" "${boost_lib_dir_}/*.lib" ) 
  else()
    file( GLOB _lib_file_list RELATIVE "${boost_lib_dir_}" "${boost_lib_dir_}/*.a" )
  endif()
  
  if( NOT _lib_file_list )
    # No library files found 
    #message( STATUS "bb_boost_list_components(): no library files found in ======== ====== ====== ===== ======= ==========> ${boost_lib_dir_}" )
    #message( STATUS "bb_boost_list_components(): no library files found in ======== ====== ====== ===== ======= ==========>" )
    return()
  endif()
  
  # message( STATUS "_lib_file_list: ${_lib_file_list}" )
  
  unset( _boost_comp_list )
  
  foreach( _lib_file IN LISTS _lib_file_list )
    # _lib_file = libboost_thread-gcc54-mt-1_65_1.a
    if( _lib_file MATCHES "^libboost_([^-]+)" )
      list( APPEND _boost_comp_list ${CMAKE_MATCH_1} )
    endif()
  endforeach()
  if( DEFINED _boost_comp_list )
    list( REMOVE_DUPLICATES _boost_comp_list )
    list( SORT _boost_comp_list )
    #message( STATUS "_boost_comp_list: ${_boost_comp_list}" )
    set( ${comp_list_} ${_boost_comp_list} PARENT_SCOPE )
  endif()
endfunction()


#
# Linux/native:
# /home/rauthenberg/bin/b2 -j16 --layout=versioned --prefix=tmp/dist cxxflags=-std=c++11 variant=release,debug link=static,shared install toolset=gcc --libdir=lib/gcc-5.4/x86_64 cflags=-fPIC
# Linux/mingw/x86_64:
# /home/rauthenberg/bin/b2 -j16 --layout=versioned --prefix=tmp/dist cxxflags=-std=c++11 variant=release,debug link=static,shared install toolset=gcc --libdir=lib/gcc-mingw-5.3/x86_64 target-os=windows threadapi=win32 --without-locale --without-python
# Linux/mingw/x86:
# /home/rauthenberg/bin/b2 -j16 --layout=versioned --prefix=tmp/dist cxxflags=-std=c++11 variant=release,debug link=static,shared install toolset=gcc --libdir=lib/gcc-mingw-5.3/x86 target-os=windows threadapi=win32 --without-locale --without-python
#
# MacOSX/native: requires user-config.jam
# /Users/rauthenberg/bin/b2 -j8 --layout=versioned --prefix=tmp/dist cxxflags=-ftemplate-depth=256 -std=c++11 linkflags=-headerpad_max_install_names -mmacosx-version-min=10.12 variant=release,debug link=static,shared install toolset=clang --libdir=lib/clang-9.0/x86_64 cflags=-mmacosx-version-min=10.12
#

# Internal function to compose the b2 command line to build Boost libraries.
function( _bb_boost_compose_b2_cmd_line cmd_line_ user_config_content_ boost_root_ lib_dir_rel_ boost_version_ )

  unset( _user_config_content )
  unset( _cxxflags )
  unset( _compileflags )
  unset( _linkflags )
  unset( _rc )
  unset( _bb_toolset_version_major_minor )
  
  set( _without_python TRUE )
  set( _without_mpi TRUE )
  
  # cmake_host_system_information( RESULT _num_cores QUERY NUMBER_OF_PHYSICAL_CORES )
  cmake_host_system_information( RESULT _num_cores QUERY NUMBER_OF_LOGICAL_CORES )

  set( _b2_cmd_line "-j${_num_cores} --layout=versioned --prefix=tmp/dist --libdir=${lib_dir_rel_} variant=release,debug link=static,shared" )

  # Figure out the Boost toolset
  _bb_get_cxx_compiler_version_major_minor( _bb_toolset_version_major_minor )
  if( MSVC )
    # Pick up the MSVC toolset argument supported by b2; e.g. toolset=msvc-14.1, toolset=msvc-14.0, ... 
    bb_get_boost_toolset_subdir( _bb_boost_toolset )
  elseif( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" )
    # -> b2 toolset=gcc ...
    set( _bb_boost_toolset "gcc" )
  elseif( CMAKE_CXX_COMPILER_ID MATCHES "^(AppleClang|Clang)$" )
    # -> b2 toolset=clang ...
    set( _bb_boost_toolset "clang" )
  endif()

  if( NOT MSVC )
    unset( _cxx_std )
    _bb_boost_get_cxx_std( _cxx_std "${boost_root_}" )
    if( DEFINED _cxx_std )
      if( ${boost_version_} VERSION_GREATER_EQUAL 1.66.0 )
        string( APPEND _b2_cmd_line " cxxstd=${_cxx_std}" )
      else()
        list( APPEND _cxxflags "-std=c++${_cxx_std}" )
      endif()
    endif()
  endif()

  if( CMAKE_CROSSCOMPILING )
    if( CMAKE_HOST_APPLE )
      # todo: ios
      message( FATAL_ERROR "No cross compile support on this platform yet, please contact technical support." )
    elseif( CMAKE_HOST_UNIX )
      if( MINGW )
        # MinGW cross compiling
        string( APPEND _b2_cmd_line " --without-locale target-os=windows threadapi=win32" )
        
        # CMAKE_CXX_COMPILER: /usr/bin/x86_64-w64-mingw32-g++-posix
        get_filename_component( _cxx_compiler_basename "${CMAKE_CXX_COMPILER}" NAME )
        # Figure out the compiler prefix to be able to determine the resource compiler
        if( _cxx_compiler_basename MATCHES "^(.+)-g\\+\\+.*$" )
          set( _mingw_tool_prefix ${CMAKE_MATCH_1} )
        else()
          message( FATAL_ERROR "Internal error: cannot determine MinGW tool prefix given ${_cxx_compiler_basename}" )
        endif()
        get_filename_component( _cxx_compiler_dir "${CMAKE_CXX_COMPILER}" DIRECTORY )
        #message( STATUS "<rc>${_cxx_compiler_dir}/${_mingw_tool_prefix}-windres" )
        set( _rc "${_cxx_compiler_dir}/${_mingw_tool_prefix}-windres" )
        set( _user_config_content "using ${_bb_boost_toolset} : ${_bb_toolset_version_major_minor} : ${CMAKE_CXX_COMPILER}" )
      else()
        message( FATAL_ERROR "Unsupported cross compiler detected, please contact technical support." )
      endif()
    else()
      message( FATAL_ERROR "No cross compile support on this platform yet, please contact technical support." )
    endif()    
  else()
    # ----------------
    # Native compiling
    # ----------------
    if( CMAKE_HOST_WIN32 )
      if( MINGW )
        string( APPEND _b2_cmd_line " --without-locale" )
        set( _user_config_content "using ${_bb_boost_toolset} : ${_bb_toolset_version_major_minor} : ${CMAKE_CXX_COMPILER}" )
      endif()
    elseif( CMAKE_HOST_APPLE )
      list( APPEND _cxxflags "-ftemplate-depth=256" )
      list( APPEND _linkflags "-headerpad_max_install_names" )
      # Add -mmacosx-version-min=<major>.<minor> in case the platform sdk is newer than the host OS.
      bb_get_os_version( _os_version )
      if( _os_version MATCHES "^([0-9]+\\.[0-9]+)" )
        set( _os_version_major_minor ${CMAKE_MATCH_1} )
        list( APPEND _compileflags "-mmacosx-version-min=${_os_version_major_minor}" )
        list( APPEND _linkflags "-mmacosx-version-min=${_os_version_major_minor}" )
      endif()
      # clang-darwin.jam does not setup -isysroot <path> which is essential to find header files, libs, development tools. 
      bb_get_isysroot( _macosx_isysroot )
      list( APPEND _compileflags "-isysroot" ${_macosx_isysroot} )
      # clang-darwin.jam does -dumpversion which emits 4.2 and not --version.
      set( _user_config_content "using ${_bb_boost_toolset} : : ${CMAKE_CXX_COMPILER}" )
    elseif( CMAKE_HOST_UNIX )
      # Pass -fPIC
      list( APPEND _compileflags "-fPIC" )
      set( _user_config_content "using ${_bb_boost_toolset} : ${_bb_toolset_version_major_minor} : ${CMAKE_CXX_COMPILER}" )
    endif()
  endif()
  
  
  if( ( DEFINED _cxxflags ) OR ( DEFINED _compileflags ) OR ( DEFINED _linkflags ) OR ( DEFINED _rc ) )
    if( DEFINED _user_config_content )
      string( APPEND _user_config_content " :" )
    else()
      message( FATAL_ERROR "Logic error in composing user-config.jam. Please contact technical support." )
    endif()
    
    foreach( _build_flags cxxflags compileflags linkflags rc )
      if( DEFINED _${_build_flags} )
        #message( STATUS "_bb_boost_compose_b2_cmd_line(): ${_build_flags}=${_${_build_flags}}" )
        # Flags consisting of multiple arguments must be space concatenated and quoted.
        list( LENGTH _${_build_flags} _len )
        if( _len GREATER 1 )
          string( REPLACE ";" " " _build_flags_args "${_${_build_flags}}" )
          string( APPEND _user_config_content " <${_build_flags}>\"${_build_flags_args}\"" )
        else()
          string( APPEND _user_config_content " <${_build_flags}>${_${_build_flags}}" )
        endif()
      endif()
    endforeach()
  endif()

  string( APPEND _b2_cmd_line " toolset=${_bb_boost_toolset}" )
  
  if( ${boost_version_} VERSION_GREATER_EQUAL 1.66.0 )
    if( MSVC OR MINGW )
      if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
        string( APPEND _b2_cmd_line " address-model=64" )
      else()
        string( APPEND _b2_cmd_line " address-model=32" )
      endif()
    endif()
  else()
    if( MSVC AND ( CMAKE_SIZEOF_VOID_P EQUAL 8 ) )
      string( APPEND _b2_cmd_line " address-model=64" )
    endif()
  endif()
  
  string( APPEND _b2_cmd_line " install" )
 
  if( _without_python )
    string( APPEND _b2_cmd_line " --without-python" )
  endif()
  
  if( _without_mpi )
    string( APPEND _b2_cmd_line " --without-mpi" )
  endif()  
 
  set( ${cmd_line_} ${_b2_cmd_line} PARENT_SCOPE )

  if( DEFINED _user_config_content )
    set( ${user_config_content_} "${_user_config_content}" PARENT_SCOPE )
  endif()

endfunction( _bb_boost_compose_b2_cmd_line )


function( _bb_boost_write_user_config_file user_config_file_ user_config_content_ )
  set( _file_content "#<CMakeBuild> - automatically generated to build Boost libraries.\n" )
  
  foreach( _line IN LISTS user_config_content_ )
    string( APPEND _file_content "${_line} ;\n" )
  endforeach()
  
  file( WRITE  ${user_config_file_} "${_file_content}" )
endfunction()


function( _bb_boost_get_cxx_std cxx_std_ boost_root_ )
  message( STATUS "_bb_boost_get_cxx_std(): boost_root=${boost_root_}" )
  # Guess the c++ standard the Boost libraries should be compiled for given the Boost root directory.
  if( "${boost_root_}" MATCHES "-c\\+\\+([0-9]+)$" )
    set( ${cxx_std_} ${CMAKE_MATCH_1} PARENT_SCOPE )
  else()
    # What's the default?
    ##if( DEFINED CMAKE_CXX_STANDARD )
    ##else()
    set( ${cxx_std_} "11" PARENT_SCOPE )
    ##endif()
  endif() 
endfunction()


function( _bb_boost_relocate_win_dlls boost_root_ boost_lib_dir_ boost_bin_dir_ )
  if( NOT EXISTS "${boost_bin_dir_}" )
    file( MAKE_DIRECTORY "${boost_bin_dir_}" )
  endif()
  file( GLOB _boost_dlls "${boost_lib_dir_}/*.dll" )
  foreach( _dll IN LISTS _boost_dlls )
    #message( STATUS "bb_boost_relocated_win_dlls(): dll=${_dll}" )
    get_filename_component( _dll_basename "${_dll}" NAME )
    file( RENAME "${_dll}" "${boost_bin_dir_}/${_dll_basename}" ) 
  endforeach()
  if( MSVC )
    file( GLOB_RECURSE _boost_pdbs "${boost_root_}/bin.v2/libs/boost*.pdb" )
    foreach( _pdb IN LISTS _boost_pdbs )
       #message( STATUS "bb_boost_relocated_win_dlls(): found ${_pdb}" )
       get_filename_component( _pdb_basename "${_pdb}" NAME )
       file( RENAME "${_pdb}" "${boost_bin_dir_}/${_pdb_basename}" ) 
    endforeach()
  endif()
endfunction()


function( _bb_boost_remove_tmp_dirs dir1_ )
  set( _tmp_dirs ${dir1_} ${ARGN} )
  foreach( _dir IN LISTS _tmp_dirs )
    if( EXISTS "${_dir}" )
      message( STATUS "Removing ${_dir} ... " )
      file( REMOVE_RECURSE "${_dir}" )
      if( CMAKE_HOST_WIN32 )
        if( EXISTS "${_dir}" )
          message( STATUS "Retrying to remove ${_dir} ... " )
          file( REMOVE_RECURSE "${_dir}" )
        endif()
      endif()
    endif()
  endforeach()
endfunction()


function( _bb_boost_remove_bjam b2_prog_ )
  if( EXISTS "${b2_prog_}" )
    get_filename_component( _b2_dir "${b2_prog_}" DIRECTORY )
    get_filename_component( _b2_basename "${b2_prog_}" NAME_WE )
    # Extra check to make sure this is really b2 or bjam.
    if( _b2_basename MATCHES "^(b2|bjam)$" )
      # Get rid of temporary b2 and/or bjam including its directory.
      message( STATUS "Removing ${_b2_dir}" )
      file( REMOVE_RECURSE "${_b2_dir}" )
    endif()
  endif()
endfunction()


