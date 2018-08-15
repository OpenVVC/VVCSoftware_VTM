# findVLC.cmake
#
#

function( VLC_getRootDir root_dir_ )
  unset( _root_dir )
  if( CMAKE_HOST_WIN32 ) 
    # Assumes a native MinGW
    if( CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$" )
      if( DEFINED ENV{VLC_ROOT64} )
        set( _root_dir "$ENV{VLC_ROOT64}" )
      else()
        set( _root_dir "C:/Program Files/VideoLAN/VLC" )
      endif()
    else()
      if( DEFINED ENV{VLC_ROOT32} )
        set( _root_dir "$ENV{VLC_ROOT32}" )
      else()
        set( _root_dir "C:/Program Files \(x86\)/VideoLAN/VLC" )
      endif()         
    endif()
  elseif( APPLE )
    set( _root_dir /Applications/VLC.app/Contents/MacOS )
  elseif( CMAKE_HOST_UNIX )
    if( MINGW )
      if( DEFINED ENV{VLC_ROOT_WINDOWS} )
       set( _vlc_root_windows $ENV{VLC_ROOT_WINDOWS} )
      else()
        set( _vlc_root_windows $ENV{HOME}/vlc-windows )
      endif()
      if( CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64)$" )
        set( _root_dir ${_vlc_root_windows}/x86_64/vlc )
      else()
        set( _root_dir ${_vlc_root_windows}/x86/vlc )
      endif()
    else()
      # native VLC build: system VLC or home brewed build.
      if( DEFINED ENV{VLC_ROOT} )
        set( _root_dir $ENV{VLC_ROOT} )
      else()
        set( _root_dir /usr )
      endif()
    endif()
  endif()

  if( DEFINED _root_dir AND EXISTS "${_root_dir}" )
    set( ${root_dir_} "${_root_dir}" PARENT_SCOPE )
  else()
    set( ${root_dir_} "${root_dir_}-NOTFOUND" PARENT_SCOPE )
  endif()

endfunction( VLC_getRootDir )

function( VLC_getIncludeDir include_dir_ root_dir_ )
  unset( _include_dir )
  if( MINGW )
    set( _include_dir ${root_dir_}/sdk/include ) 
  elseif( APPLE )
    set( _include_dir ${root_dir_}/include )
  elseif( CMAKE_HOST_UNIX )
    if( ${root_dir_} STREQUAL "/usr" )
      set( _include_dir ${root_dir_}/include )
    else()
      set( _include_dir ${root_dir_}/include ) # or ${root_dir_)/sdk/include ?
    endif()      
  endif()
  
  if( DEFINED _include_dir AND EXISTS "${_include_dir}/vlc/libvlc_version.h" )
    set( ${include_dir_} "${_include_dir}" PARENT_SCOPE )
  else()
    set( ${include_dir_} "${include_dir_}-NOTFOUND" PARENT_SCOPE )
  endif()  
endfunction()

function( VLC_getVersion version_var_ header_file_ )
  # set( _regex_vlc_version_def "^#[ ]*define[\t ]+LIBVLC_VERSION_(MAJOR|MINOR|REVISION|EXTRA)[\t ]+[(]([0-9]+)" )
  set( _regex_vlc_version_def "^#[ ]*define[\t ]+LIBVLC_VERSION_(MAJOR|MINOR|REVISION)[\t ]+[(]([0-9]+)" )
  file( STRINGS "${header_file_}" _header_version_lines REGEX "${_regex_vlc_version_def}" )
  #message( STATUS "extracted header lines: ${_header_version_lines}" )
  unset( _vlc_version_list )
  foreach( _line ${_header_version_lines} )
    if( _line MATCHES "${_regex_vlc_version_def}" )
      list( APPEND _vlc_version_list "${CMAKE_MATCH_2}" )
    endif()
  endforeach()
  #message( STATUS "_vlc_version_list: ${_vlc_version_list}" )
  if( DEFINED _vlc_version_list )
    # Compose a doted separated version string
    string( REPLACE ";" "." _vlc_version_str "${_vlc_version_list}" )
  else()
    message( FATAL_ERROR "VLC_getVersion() failed to extract version information, please contact the maintainer for further support." )
  endif()
  set( ${version_var_} "${_vlc_version_str}" PARENT_SCOPE )
endfunction()


#message( STATUS "findVlc.cmake: starting" )
set( _VLC_FOUND FALSE )

if( NOT DEFINED VLC_ROOT_DIR )
  # Standard search for VLC root if the user does not give a hint.
  VLC_getRootDir( VLC_ROOT_DIR )
endif()

#message( STATUS "FindVLC: VLC_ROOT_DIR: ${VLC_ROOT_DIR}" )
if( VLC_ROOT_DIR )

  VLC_getIncludeDir( _vlc_include_dir "${VLC_ROOT_DIR}" )
  if( _vlc_include_dir )   
    set( _VLC_FOUND TRUE )
  endif()
endif()

if( _VLC_FOUND )
  VLC_getVersion( VLC_VERSION "${_vlc_include_dir}/vlc/libvlc_version.h" )
  if( NOT TARGET VLC::vlccore )
    set( VLC_INCLUDE_DIRS "${_vlc_include_dir}/vlc" "${_vlc_include_dir}/vlc/plugins" )
    set( _vlc_interface_compile_definitions "__PLUGIN__;_FILE_OFFSET_BITS=64;_REENTRANT;_THREAD_SAFE" )
  
    add_library( VLC::vlccore SHARED IMPORTED )
    set_target_properties( VLC::vlccore PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${VLC_INCLUDE_DIRS}" 
                                                   INTERFACE_COMPILE_DEFINITIONS "${_vlc_interface_compile_definitions}" )
      
    if( MINGW ) 
      set_target_properties( VLC::vlccore PROPERTIES IMPORTED_LOCATION ${VLC_ROOT_DIR}/libvlccore.dll 
                                                     IMPORTED_IMPLIB   ${VLC_ROOT_DIR}/sdk/lib/libvlccore.lib )  
    elseif( APPLE )
      set_target_properties( VLC::vlccore PROPERTIES IMPORTED_LOCATION ${VLC_ROOT_DIR}/lib/libvlccore.dylib ) 
    elseif( CMAKE_HOST_UNIX )
      if( VLC_ROOT_DIR STREQUAL "/usr" )          
        find_library( VLC_LIBVLCCORE libvlccore.so )
      else()
        set( VLC_LIBVLCCORE ${VLC_ROOT_DIR}/lib/libvlccore.so )
      endif()
      if( VLC_LIBVLCCORE )
        set_target_properties( VLC::vlccore PROPERTIES IMPORTED_LOCATION ${VLC_LIBVLCCORE} ) 
      endif()
    endif()
  endif()
else()
  # Let find_package know that VLC development files are not available. 
  unset( VLC_ROOT_DIR )
endif()

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args( VLC REQUIRED_VARS VLC_ROOT_DIR VERSION_VAR VLC_VERSION )

# Disposal of temporary stuff.
unset( _VLC_FOUND )
unset( _vlc_interface_compile_definitions )
unset( _vlc_include_dir )

#message( STATUS "Leaving FindVLC.cmake" )

