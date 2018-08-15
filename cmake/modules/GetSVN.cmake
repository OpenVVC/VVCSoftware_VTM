if( GENERATE_DUMMY )
  file( WRITE svnrevision.h.in "\n" )
else()
  include( FindSubversion )

  if( SUBVERSION_FOUND )
    # Copied from FindSubversion: need to change to fix the SEND_ERROR problem

    # the subversion commands should be executed with the C locale, otherwise
    # the message (which are parsed) may be translated, Alex
    set( _Subversion_SAVED_LC_ALL "$ENV{LC_ALL}" )
    set( ENV{LC_ALL} C )

    execute_process( COMMAND ${Subversion_SVN_EXECUTABLE} info ${SOURCE_DIR}
      OUTPUT_VARIABLE NEXT_WC_INFO
      ERROR_VARIABLE Subversion_svn_info_error
      RESULT_VARIABLE Subversion_svn_info_result
      OUTPUT_STRIP_TRAILING_WHITESPACE )

    if( NOT ${Subversion_svn_info_result} EQUAL 0 )
      message( "${SOURCE_DIR} is not a subversion directory" )
      file( WRITE svnrevision.h.in "\n" )
    else()
      string( REGEX REPLACE "^(.*\n)?URL: ([^\n]+).*"
        "\\2" NEXT_WC_URL "${NEXT_WC_INFO}" )
      string( REGEX REPLACE "^(.*\n)?Repository Root: ([^\n]+).*"
        "\\2" NEXT_WC_ROOT "${NEXT_WC_INFO}" )
      string( REGEX REPLACE "^(.*\n)?Revision: ([^\n]+).*"
        "\\2" NEXT_WC_REVISION "${NEXT_WC_INFO}" )
      string( REGEX REPLACE "^(.*\n)?Last Changed Author: ([^\n]+).*"
        "\\2" NEXT_WC_LAST_CHANGED_AUTHOR "${NEXT_WC_INFO}" )
      string( REGEX REPLACE "^(.*\n)?Last Changed Rev: ([^\n]+).*"
        "\\2" NEXT_WC_LAST_CHANGED_REV "${NEXT_WC_INFO}" )
      string( REGEX REPLACE "^(.*\n)?Last Changed Date: ([^\n]+).*"
        "\\2" NEXT_WC_LAST_CHANGED_DATE "${NEXT_WC_INFO}" )
      
      string( REGEX REPLACE "^(.*)?:(.*)"
        "\\2" NEXT_WC_URL_PATH "${NEXT_WC_URL}" )
      string( REGEX REPLACE "^(.*)?:(.*)"
        "\\2" NEXT_WC_ROOT_PATH "${NEXT_WC_ROOT}" )
      
      file( RELATIVE_PATH NEXT_WC_RELATIVE_URL "${NEXT_WC_ROOT_PATH}" "${NEXT_WC_URL_PATH}" )
        
      execute_process( COMMAND ${Subversion_SVN_EXECUTABLE} status ${SOURCE_DIR}
        OUTPUT_VARIABLE NEXT_WC_STATUS
        ERROR_VARIABLE Subversion_svn_status_error
        RESULT_VARIABLE Subversion_svn_status_result
        OUTPUT_STRIP_TRAILING_WHITESPACE )
      
      if( ${Subversion_svn_status_result} EQUAL 0 )
        if( NEXT_WC_STATUS )
          set( NEXT_WC_STATUS "M" )
        else()
          set( NEXT_WC_STATUS "" )
        endif()
      endif()
        
      # write a file with the SVNVERSION define
      file( WRITE  svnrevision.h.in "#define SVNREVISION    \"${NEXT_WC_LAST_CHANGED_REV}${NEXT_WC_STATUS}\"\n" )
      file( APPEND svnrevision.h.in "#define SVNRELATIVEURL \"${NEXT_WC_RELATIVE_URL}\"\n" )
    endif()

    # restore the previous LC_ALL
    set( ENV{LC_ALL} ${_Subversion_SAVED_LC_ALL} )
  else()
    message( "Subversion executable not found!" )
    file( WRITE svnrevision.h.in "\n" )
  endif()
endif()

# copy the file to the final header only if the version changes
# reduces needless rebuilds
execute_process( COMMAND ${CMAKE_COMMAND} -E copy_if_different svnrevision.h.in svnrevision.h )
