#[[.rst:
BBuildEnvGit
------------

The BBuildEnvGit provides out-of-tree checkout and update support for GIT repositories.

Provided Macros
^^^^^^^^^^^^^^^

::

  bb_git_co_external_dir(<checkout_dir> <tag> [GIT_REPOSITORY <url>] [GIT_REMOTE_NAME <remote>] [GIT_CONFIG <key=value>])

Macro bb_git_co_external_dir clones ``url`` to ``checkout_dir`` if the checkout directory does not exist or updates it to the
specified tag or branch.  Remote tracking branches are updated using fetch and rebase. If this behavior is not intended, use a local branch. 

#]]

if( CMAKE_VERSION VERSION_GREATER_EQUAL 3.10 )
  include_guard( GLOBAL )
endif()

if( NOT DEFINED BBuildEnvGit_DEBUG_LEVEL )
  set( BBuildEnvGit_DEBUG_LEVEL 0 )
endif()

macro( bb_git_dbg_msg msg )
  if( BBuildEnvGit_DEBUG_LEVEL GREATER 0 )
    message( STATUS "${msg}" )
  endif()
endmacro()


macro( bb_git_co_external_dir checkout_dir_ git_tag_ )
  
  #message( STATUS "bb_git_co_external_dir(): entering: ${ARGV}" )
  bb_git_dbg_msg( "bb_git_co_external_dir(): entering: ${ARGV}" )
  
  if( NOT Git_FOUND )
    find_package( Git MODULE REQUIRED )
  endif()
  bb_git_co_external_dir_helper( "${GIT_EXECUTABLE}" ${checkout_dir_} ${git_tag_} ${ARGN} )

  bb_git_dbg_msg( "bb_git_co_external_dir(): leaving." )

endmacro()



function( _bb_git_co_external_dir_parse_args git_repo_ git_remote_name_ git_clone_options_ arg1 )
  
  unset( _git_repo )
  unset( _git_remote_name )
  unset( _git_clone_options )
  unset( _git_config_options )

  if( DEFINED ${git_clone_options_} )
    set( _git_clone_options "${${git_clone_options_}}" )
  endif()
  
  set( _optional_args ${arg1} ${ARGN} )
  set( _optional_args_input ${_optional_args} )
  
  bb_git_dbg_msg( "_bb_git_co_external_dir_parse_args(): processing optional arguments: ${_optional_args}" )
  
  # Possible optional arguments: GIT_REPOSITORY <url>,  GIT_REMOTE_NAME <name>, GIT_PROGRESS <bool>, GIT_CONFIG <key=value> ...
  list( LENGTH _optional_args  _num_optional_args )
  
  # Search for GIT_REPOSITORY <value>
  list( FIND _optional_args GIT_REPOSITORY _pos_keyword )
  if( NOT _pos_keyword EQUAL -1 )
    math( EXPR _pos_value "${_pos_keyword} + 1" )
    if( _num_optional_args LESS_EQUAL ${_pos_value} )
      message( FATAL_ERROR "Optional argument GIT_REPOSITORY must be followed by a value." )
    else()
      list( GET _optional_args ${_pos_value} _git_repo )
      set( ${git_repo_} ${_git_repo} PARENT_SCOPE )
    endif()
    list( REMOVE_AT _optional_args ${_pos_keyword} ${_pos_value} )
    list( LENGTH _optional_args  _num_optional_args )
  endif()
    
  # Search for GIT_REMOTE_NAME <value>
  list( FIND _optional_args GIT_REMOTE_NAME _pos_keyword )
  if( NOT _pos_keyword EQUAL -1 )
    math( EXPR _pos_value "${_pos_keyword} + 1" )
    if( _num_optional_args LESS_EQUAL ${_pos_value} )
      message( FATAL_ERROR "Optional argument GIT_REMOTE_NAME must be followed by a value." )
    else()
      list( GET _optional_args ${_pos_value} _git_remote_name )
      set( ${git_remote_name_} ${_git_remote_name} PARENT_SCOPE )
    endif()
    list( REMOVE_AT _optional_args ${_pos_keyword} ${_pos_value} )
    list( LENGTH _optional_args  _num_optional_args )
  endif()
  
  # Search for GIT_PROGRESS <bool>
  list( FIND _optional_args GIT_PROGRESS _pos_keyword )
  if( NOT _pos_keyword EQUAL -1 )
    math( EXPR _pos_value "${_pos_keyword} + 1" )
    if( _num_optional_args LESS_EQUAL ${_pos_value} )
      message( FATAL_ERROR "Optional argument GIT_PROGRESS must be followed by a boolean value." )
    else()
      # Found GIT_PROGRESS, remove it from git_clone_options
      if( DEFINED _git_clone_options )
        list( REMOVE_ITEM _git_clone_options "--progress" )
      endif()
      list( GET _optional_args ${_pos_value} _git_progress )
      if( _git_progress )
        list( APPEND _git_clone_options "--progress" )
      endif()
    endif()
    list( REMOVE_AT _optional_args ${_pos_keyword} ${_pos_value} )
    list( LENGTH _optional_args  _num_optional_args )
  endif()
  
  # Search for GIT_CONFIG <key=value> ... 
  list( FIND _optional_args GIT_CONFIG _pos_keyword )
  if( NOT _pos_keyword EQUAL -1 )
    math( EXPR _pos_value "${_pos_keyword} + 1" )
    if( _num_optional_args LESS_EQUAL ${_pos_value} )
      message( FATAL_ERROR "Optional argument GIT_CONFIG must be followed by one or more <key=value> expressions." )
    else()
      list( REMOVE_AT _optional_args ${_pos_keyword} )
      set( _pos_value ${_pos_keyword} )
      math( EXPR _num_optional_args "${_num_optional_args} - 1" )
      while( _pos_value LESS ${_num_optional_args} )
        list( GET _optional_args ${_pos_value} _git_config_value )
        if( _git_config_value MATCHES "^GIT_" )
          # Looks like another argument keyword, bail out here.
          break()
        endif()
        list( APPEND _git_config_options "--config" "${_git_config_value}" )
        list( REMOVE_AT _optional_args ${_pos_value} )
        math( EXPR _num_optional_args "${_num_optional_args} - 1" )
      endwhile()
      if( DEFINED _git_config_options )
        #set( ${git_config_options_} "${_git_config_options}" PARENT_SCOPE )
        list( APPEND _git_clone_options "${_git_config_options}" )
      endif()      
    endif()
  endif()
  
  if( _num_optional_args GREATER 0 )
    message( FATAL_ERROR "_bb_git_co_external_dir_parse_args(): optional arguments ${_optional_args_input} not understood." ) 
  endif()
  
  if( DEFINED _git_clone_options )
    set( ${git_clone_options_} "${_git_clone_options}" PARENT_SCOPE )
  endif()
endfunction()


function( bb_git_co_external_dir_helper git_EXECUTABLE_ checkout_dir_ git_tag_ )
  
  bb_git_dbg_msg( "bb_git_co_external_dir_helper(): entering ${ARGV}" )
  
  set( git_remote_name "origin" )
  unset( git_repo )
  unset( git_submodules )
  unset( git_options )
  unset( git_clone_options )  

  set( git_clone_options "--progress" )

  if( ARGC GREATER 3 )        
    _bb_git_co_external_dir_parse_args( git_repo git_remote_name git_clone_options ${ARGN} ) 
    message( STATUS "git_repo=${git_repo} git_remote_name=${git_remote_name} git_clone_options=${git_clone_options}" )
    #unset( git_clone_options )
  endif()
  
  # Normalize checkout directory.
  get_filename_component( checkout_dir_ "${checkout_dir_}" REALPATH )
  
  #bb_git_dbg_msg( "bb_git_co_external_dir_helper(): normalized checkout dir: ${checkout_dir_}" )
  
  get_filename_component( _checkout_dir_parent "${checkout_dir_}" DIRECTORY )
  get_filename_component( _checkout_dir_base "${checkout_dir_}" NAME )
  
  
  if( NOT EXISTS "${checkout_dir_}" )
    
    if( NOT DEFINED git_repo )
      message( FATAL_ERROR "checkout directory '${checkout_dir_}' does not exists and no GIT repository specified to clone from." ) 
    endif()
    
    if( NOT EXISTS "${_checkout_dir_parent}" )
      file( MAKE_DIRECTORY "${_checkout_dir_parent}" )
    endif()
    
    execute_process(
      COMMAND "${git_EXECUTABLE_}" ${git_options} clone ${git_clone_options} --origin "${git_remote_name}" "${git_repo}" ${_checkout_dir_base}
      WORKING_DIRECTORY "${_checkout_dir_parent}"
      RESULT_VARIABLE error_code
      )    
    if( error_code )
      message(FATAL_ERROR "Failed to clone repository: '${git_repo}'")
    endif()
     
    execute_process(
      COMMAND "${git_EXECUTABLE_}" ${git_options} checkout ${git_tag_} --
      WORKING_DIRECTORY "${checkout_dir_}"
      RESULT_VARIABLE error_code
      )
    if( error_code )
      message(FATAL_ERROR "Failed to checkout tag: '${git_tag_}'")
    endif()
    
  else()
    _bb_do_gitupdate( "${git_EXECUTABLE_}" ${git_tag_} ${git_remote_name} "${checkout_dir_}" )
  endif()  
endfunction()



function( _bb_do_gitupdate git_EXECUTABLE git_tag git_remote_name work_dir )

  bb_git_dbg_msg( "_bb_do_gitupdate(): entering git_tag=${git_tag} work_dir=${work_dir}" )

  execute_process(
    COMMAND "${git_EXECUTABLE}" rev-list --max-count=1 HEAD
    WORKING_DIRECTORY "${work_dir}"
    RESULT_VARIABLE error_code
    OUTPUT_VARIABLE head_sha
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  if(error_code)
    message(FATAL_ERROR "Failed to get the hash for HEAD in: '${work_dir}'")
  endif()
  
  execute_process(
    COMMAND "${git_EXECUTABLE}" show-ref ${git_tag}
    WORKING_DIRECTORY "${work_dir}"
    OUTPUT_VARIABLE show_ref_output
    )
  # If a remote ref is asked for, which can possibly move around,
  # we must always do a fetch and checkout.
  if( "${show_ref_output}" MATCHES "remotes" )
    set(is_remote_ref 1)
  else()
    set(is_remote_ref 0)
  endif()
  
  # Tag is in the form <remote>/<tag> (i.e. origin/master) we must strip
  # the remote from the tag.
  if( "${show_ref_output}" MATCHES "refs/remotes/${git_tag}")
    string(REGEX MATCH "^([^/]+)/(.+)$" _unused "${git_tag}")
    set(git_remote "${CMAKE_MATCH_1}")
    set(git_tag "${CMAKE_MATCH_2}")
  else()
    set(git_remote "${git_remote_name}")
    set(git_tag "${git_tag}")
  endif()
  
  bb_git_dbg_msg( "git_remote=${git_remote} git_tag=${git_tag} is_remote_ref=${is_remote_ref}" )
  
  # This will fail if the tag or branch does not exist (it probably has not been fetched
  # yet).
  execute_process(
    COMMAND "${git_EXECUTABLE}" rev-list --max-count=1 ${git_tag}
    WORKING_DIRECTORY "${work_dir}"
    RESULT_VARIABLE error_code
    OUTPUT_VARIABLE tag_sha
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )

  if( error_code )
    set( _git_tag_unknown TRUE )
    message( STATUS "git rev-list failed for ${git_tag}" )
  else()
    set( _git_tag_unknown FALSE )
  endif()

  if( error_code OR is_remote_ref OR ( NOT ( "${tag_sha}" STREQUAL "${head_sha}" ) ) )
    
    bb_git_dbg_msg( "Executing git fetch in: '${work_dir}'" )
    execute_process(
      COMMAND "${git_EXECUTABLE}" fetch
      WORKING_DIRECTORY "${work_dir}"
      RESULT_VARIABLE error_code
      )
    if(error_code)
      message(FATAL_ERROR "Failed to fetch in: '${work_dir}'")
    endif()

    if( _git_tag_unknown )
      if( "${git_tag}" MATCHES "^tags/" )
        #message( STATUS "git_tag=${git_tag} looks like a tag, ignoring any remote branches." )
        set( _rm_branches "" )
      else()  
        execute_process(
          COMMAND "${git_EXECUTABLE}" branch -r 
          WORKING_DIRECTORY "${work_dir}"
          RESULT_VARIABLE error_code
          
          OUTPUT_VARIABLE _rm_branches
          OUTPUT_STRIP_TRAILING_WHITESPACE      
          )
        if(error_code)
          message(FATAL_ERROR "git branch -r failed in: '${work_dir}'")
        endif()
        
        bb_git_dbg_msg( "Discovered remote branches: '${_rm_branches}'" )
      endif()
      
      if( "${_rm_branches}" MATCHES "${git_remote_name}/${git_tag}" )
        bb_git_dbg_msg( "Found remote branch ${git_tag}" )
        
        execute_process(
          COMMAND "${git_EXECUTABLE}" checkout -b ${git_tag} ${git_remote_name}/${git_tag}
          WORKING_DIRECTORY "${work_dir}"
          RESULT_VARIABLE error_code
          )
        if(error_code)
          message(FATAL_ERROR "Failed to checkout: '${git_tag}'")
        endif()
      else()
        # Unknown tag ??? 
        bb_git_dbg_msg( "Trying to checkout ${git_tag} -  previous fetch may have picked it up." )
        execute_process(
          COMMAND "${git_EXECUTABLE}" checkout ${git_tag}
          WORKING_DIRECTORY "${work_dir}"
          RESULT_VARIABLE error_code
          )
        if(error_code)
          message(FATAL_ERROR "Failed to checkout tag: '${git_tag}'")
        endif()               
      endif()
    else()  
      # tag_sha is valid -> deal with remote tracking branches first. 
      if( is_remote_ref )
        bb_git_dbg_msg( "${git_tag} refers to a remote tracking branch." )
        # check head whether it is on the tracking branch.
        execute_process(
          COMMAND "${git_EXECUTABLE}" status -b --porcelain
          WORKING_DIRECTORY "${work_dir}"
          RESULT_VARIABLE error_code
          OUTPUT_VARIABLE _git_status
          OUTPUT_STRIP_TRAILING_WHITESPACE
          )
        if(error_code)
          message(FATAL_ERROR "Failed to get status: '${work_dir}'")
        endif()        
                
        bb_git_dbg_msg( "Retrieved status of '${work_dir}': '${_git_status}'" )
        if( NOT ( _git_status MATCHES "^## ${git_tag}\\.\\.\\." ) )
          # Change to the tracking branch, head points to something else.
          execute_process(
            COMMAND "${git_EXECUTABLE}" checkout ${git_tag}
            WORKING_DIRECTORY "${work_dir}"
            RESULT_VARIABLE error_code
            )
          if(error_code)
            message(FATAL_ERROR "Failed to checkout tag: '${git_tag}'")
          endif()
          
          # Updating head_sha to reflect the previous change which is needed by rebase. 
          execute_process(
            COMMAND "${git_EXECUTABLE}" rev-list --max-count=1 HEAD
            WORKING_DIRECTORY "${work_dir}"
            RESULT_VARIABLE error_code
            OUTPUT_VARIABLE head_sha
            OUTPUT_STRIP_TRAILING_WHITESPACE
            )
          if(error_code)
            message(FATAL_ERROR "Failed to get the hash for HEAD")
          endif()          
        endif()
        
        bb_git_dbg_msg( "Updating remote tracking branch using rebase ... " )
        _bb_do_gitrebase( "${git_EXECUTABLE}" ${head_sha} ${git_tag} ${git_remote} "${work_dir}" )
        
      else()
        # Local branch or tag and tag_hash != head_hash. 
        execute_process(
          COMMAND "${git_EXECUTABLE}" checkout ${git_tag}
          WORKING_DIRECTORY "${work_dir}"
          RESULT_VARIABLE error_code
          )
        if(error_code)
          message(FATAL_ERROR "Failed to checkout tag: '${git_tag}'")
        endif()
      endif()
    endif()
  else()
    # Local branch or tag and tag_hash==head_hash. The checkout is needed when switching from a commit with detached head to a branch 
    # pointing to the same commit.
    execute_process(
      COMMAND "${git_EXECUTABLE}" checkout ${git_tag}
      WORKING_DIRECTORY "${work_dir}"
      RESULT_VARIABLE error_code
      )
    if(error_code)
      message(FATAL_ERROR "Failed to checkout tag: '${git_tag}'")
    endif()  
    #message( STATUS "_bb_do_gitupdate(): ${git_tag} is up-to-date." )
  endif()
  
  bb_git_dbg_msg( "_bb_do_gitupdate(): leaving." )
  
endfunction()



function( _bb_do_gitrebase git_EXECUTABLE head_sha git_tag git_remote work_dir )
  bb_git_dbg_msg( "_bb_do_gitrebase(): entering: ${ARGV}" )

  if(NOT GIT_VERSION_STRING VERSION_LESS 1.7.6)
    set(git_stash_save_options --all --quiet )
  else()
    set(git_stash_save_options --quiet)
  endif()

  # Check if stash is needed
  execute_process(
    COMMAND "${git_EXECUTABLE}" status --porcelain
    WORKING_DIRECTORY "${work_dir}"
    RESULT_VARIABLE error_code
    OUTPUT_VARIABLE repo_status
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  if(error_code)
    message(FATAL_ERROR "Failed to get the status")
  endif()
  string(LENGTH "${repo_status}" need_stash)

  # If not in clean state, stash changes in order to be able to be able to
  # perform git pull --rebase
  if(need_stash)
    bb_git_dbg_msg( "_bb_do_gitrebase(): need stash!!" ) 
    execute_process(
      COMMAND "${git_EXECUTABLE}" stash save ${git_stash_save_options}
      WORKING_DIRECTORY "${work_dir}"
      RESULT_VARIABLE error_code
      )
    if(error_code)
      message(FATAL_ERROR "Failed to stash changes in: '${work_dir}'")
    endif()
  endif()

  # Pull changes from the remote branch
  bb_git_dbg_msg( "Trying git rebase ${git_remote}/${git_tag}" )
  execute_process(
    COMMAND "${git_EXECUTABLE}" rebase ${git_remote}/${git_tag}
    WORKING_DIRECTORY "${work_dir}"
    RESULT_VARIABLE error_code
    )
  if(error_code)
    # Rebase failed: Restore previous state.
    message( STATUS "Rebase failed -> trying to restore previous state ..." )
    execute_process(
      COMMAND "${git_EXECUTABLE}" rebase --abort
      WORKING_DIRECTORY "${work_dir}"
    )
    if(need_stash)
      execute_process(
        COMMAND "${git_EXECUTABLE}" stash pop --index --quiet
        WORKING_DIRECTORY "${work_dir}"
        )
    endif()
    message(FATAL_ERROR "\nFailed to rebase in: '${work_dir}'.\nYou will have to resolve the conflicts manually.")
  endif()

  if(need_stash)
    bb_git_dbg_msg( "Trying to recover stashed changes." ) 
    execute_process(
      COMMAND "${git_EXECUTABLE}" stash pop --index --quiet
      WORKING_DIRECTORY "${work_dir}"
      RESULT_VARIABLE error_code
      )
    if(error_code)
      # Stash pop --index failed: Try again dropping the index
      execute_process(
        COMMAND "${git_EXECUTABLE}" reset --hard --quiet
        WORKING_DIRECTORY "${work_dir}"
        RESULT_VARIABLE error_code
        )
      execute_process(
        COMMAND "${git_EXECUTABLE}" stash pop --quiet
        WORKING_DIRECTORY "${work_dir}"
        RESULT_VARIABLE error_code
        )
      if(error_code)
        # Stash pop failed: Restore previous state.
        execute_process(
          COMMAND "${git_EXECUTABLE}" reset --hard --quiet ${head_sha}
          WORKING_DIRECTORY "${work_dir}"
        )
        execute_process(
          COMMAND "${git_EXECUTABLE}" stash pop --index --quiet
          WORKING_DIRECTORY "${work_dir}"
        )
        message(FATAL_ERROR "\nFailed to unstash changes in: '${work_dir}'.\nYou will have to resolve the conflicts manually.")
      endif()
    endif()
  endif()
  bb_git_dbg_msg( "_bb_do_gitrebase(): leaving." )
endfunction()
