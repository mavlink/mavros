find_package(Python3 COMPONENTS Interpreter)

function(em_expand context_in context_out em_file_in file_out)
  assert_file_exists("${context_in}" "input file for context missing")
  assert_file_exists("${em_file_in}" "template file missing")
  message(DEBUG "configure_file(${context_in}, ${context_out})")
  configure_file(${context_in} ${context_out} @ONLY)
  assert_file_exists("${context_out}" "context file was not generated correctly")

  stamp(${em_file_in})

  # create directory if necessary
  get_filename_component(_folder_out ${file_out} PATH)
  if(NOT IS_DIRECTORY ${_folder_out})
    file(MAKE_DIRECTORY ${_folder_out})
  endif()

  message(DEBUG "Evaluate template '${em_file_in}' to '${file_out}' (with context from '${context_out}')")
  execute_process(COMMAND
    ${Python3_EXECUTABLE}
    -m em
    --raw-errors
    -F ${context_out}
    -o ${file_out}
    ${em_file_in}
    RESULT_VARIABLE _res)

  if(NOT _res EQUAL 0)
    message(FATAL_ERROR "em.py return error code ${_res}")
  endif()
endfunction()

