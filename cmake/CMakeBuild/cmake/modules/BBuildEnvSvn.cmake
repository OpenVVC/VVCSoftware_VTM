#[[.rst:
BBuildEnvSvn
------------

The BBuildEnvSvn provides out-of-tree checkout and update support for SVN repositories.

Provided Macros
^^^^^^^^^^^^^^^

::

  bb_svn_co_external_dir(<checkout_dir> <url> [REV <numeric_rev> | <HEAD>])

Macro bb_svn_co_external_dir checkouts SVN ``url`` to ``checkout_dir`` if the checkout directory does not exist or updates it to 
the specified tag or branch.  A revision argument is required to track a remote branch or use a specific revision.

#]]

if( CMAKE_VERSION VERSION_GREATER_EQUAL 3.10 )
  include_guard( GLOBAL )
endif()

if( NOT DEFINED BBuildEnvSvn_DEBUG_LEVEL )
  set( BBuildEnvSvn_DEBUG_LEVEL 0 )
endif()

macro( bb_svn_dbg_msg msg )
  if( BBuildEnvSvn_DEBUG_LEVEL GREATER 0 )
    message( STATUS "${msg}" )
  endif()
endmacro()


macro( bb_svn_co_external_dir checkout_dir_ svn_url_ )
  
  bb_svn_dbg_msg( "bb_svn_co_external_dir(): entering: ${ARGV}" )
  
  if( NOT Subversion_FOUND )
    find_package( Subversion MODULE REQUIRED )
  endif()
  
  bb_svn_co_external_dir_helper( ${checkout_dir_} ${svn_url_} ${ARGN} )

  bb_svn_dbg_msg( "bb_svn_co_external_dir(): leaving" )

endmacro()


function( _bb_svn_co_external_dir_parse_args svn_rev_ arg1 )
  
  unset( _svn_rev )
  
  set( _optional_args ${arg1} ${ARGN} )
  set( _optional_args_input ${_optional_args} )
  
  bb_svn_dbg_msg( "_bb_svn_co_external_dir_parse_args(): processing optional arguments: ${_optional_args}" )
  
  # Possible optional arguments: REV HEAD | nnnn 
  list( LENGTH _optional_args  _num_optional_args )
  
  # Search for REV <value>
  list( FIND _optional_args REV _pos_keyword )
  if( NOT _pos_keyword EQUAL -1 )
    math( EXPR _pos_value "${_pos_keyword} + 1" )
    if( _num_optional_args LESS_EQUAL ${_pos_value} )
      message( FATAL_ERROR "Optional argument REV must be followed by a numeric value or HEAD." )
    else()
      list( GET _optional_args ${_pos_value} _svn_rev )
      set( ${svn_rev_} ${_svn_rev} PARENT_SCOPE )
    endif()
    list( REMOVE_AT _optional_args ${_pos_keyword} ${_pos_value} )
    list( LENGTH _optional_args  _num_optional_args )
    if( _num_optional_args EQUAL 0 )
      return()
    endif()
  endif()

  if( _num_optional_args GREATER 0 )
    message( FATAL_ERROR "_bb_svn_co_external_dir_parse_args(): optional arguments ${_optional_args_input} not understood." ) 
  endif()  
endfunction()  
   

function( bb_svn_co_external_dir_helper checkout_dir_ svn_url_ )
  
  unset( _svn_rev )
  unset( _svn_peg_rev )
  
  if( ARGC GREATER 2 )
    _bb_svn_co_external_dir_parse_args( _svn_rev ${ARGN} )
    
    if( DEFINED _svn_rev )
      set( _svn_peg_rev "@${_svn_rev}" )
      #message( STATUS "_svn_peg_rev=${_svn_peg_rev}" )
    endif()
  endif()
  
  set( _svn_options "--non-interactive" "--ignore-externals" )
  
  # Normalize checkout directory.
  get_filename_component( checkout_dir_ "${checkout_dir_}" REALPATH )
    
  if( NOT EXISTS ${checkout_dir_} )
    
    execute_process( COMMAND
        ${Subversion_SVN_EXECUTABLE} ${_svn_options} co ${svn_url_}${_svn_peg_rev} ${checkout_dir_}
        ERROR_VARIABLE Subversion_svn_co_error
        RESULT_VARIABLE Subversion_svn_co_result
        OUTPUT_STRIP_TRAILING_WHITESPACE )
    if( NOT Subversion_svn_co_result EQUAL 0 )
      message( FATAL_ERROR "Command \"${Subversion_SVN_EXECUTABLE} co \" failed with output:\n${Subversion_svn_co_error}" )
    endif()
  else()
    # Do we have to switch?
    Subversion_WC_INFO( ${checkout_dir_} _bb_svn_info )
    bb_svn_dbg_msg( "_bb_svn_co_external_dir_helper(): ${_bb_svn_info_WC_URL}" )
    bb_svn_dbg_msg( "_bb_svn_co_external_dir_helper(): ${_bb_svn_info_WC_REVISION}" )
    
    if( NOT _bb_svn_info_WC_URL STREQUAL ${svn_url_} )
      execute_process( COMMAND
          ${Subversion_SVN_EXECUTABLE} ${_svn_options} sw ${svn_url_}${_svn_peg_rev} ${checkout_dir_}
          ERROR_VARIABLE Subversion_svn_co_error
          RESULT_VARIABLE Subversion_svn_co_result
          OUTPUT_STRIP_TRAILING_WHITESPACE )
      if( NOT Subversion_svn_co_result EQUAL 0 )
        message( FATAL_ERROR "Command \"${Subversion_SVN_EXECUTABLE} sw \" failed with output:\n${Subversion_svn_co_error}" )
      endif()
    elseif( ( DEFINED _svn_rev ) AND ( NOT ${_svn_rev} STREQUAL ${_bb_svn_info_WC_REVISION} ) )
      # Same URL but different revision requested -> update required.
      # Note HEAD nether compares equal to the all numeric WC_REVISION.
      execute_process( COMMAND
          ${Subversion_SVN_EXECUTABLE} ${_svn_options} -r ${_svn_rev} up ${checkout_dir_}
          ERROR_VARIABLE Subversion_svn_co_error
          RESULT_VARIABLE Subversion_svn_co_result
          OUTPUT_STRIP_TRAILING_WHITESPACE )
      if( NOT Subversion_svn_co_result EQUAL 0 )
        message( FATAL_ERROR "Command \"${Subversion_SVN_EXECUTABLE} up \" failed with output:\n${Subversion_svn_co_error}" )
      endif()            
    endif()        
  endif()
endfunction()
