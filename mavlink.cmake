# generate MAVLink headers from message defenitions

find_package(PythonInterp)

set(MAVLINK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/mavlink")
set(MAVGEN_PY "${MAVLINK_DIR}/pymavlink/generator/mavgen.py")

# mavlink generation
macro(generate_mavlink version definitions)
    foreach(definition ${definitions})
        set(targetName ${definition}-v${version})
	set(definitionAbsPath ${MAVLINK_DIR}/message_definitions/v${version}/${definition})
        message(STATUS "processing: ${definitionAbsPath}")
        add_custom_command( 
            OUTPUT ${targetName}-stamp
	    COMMAND ${PYTHON_EXECUTABLE} ${MAVGEN_PY} --lang=C --wire-protocol=${version}
	    --output=${CATKIN_DEVEL_PREFIX}/include/mavlink/v${version} ${definitionAbsPath}
            COMMAND touch ${targetName}-stamp
	    DEPENDS ${definitionAbsPath} ${MAVGEN_PY}
            )
        add_custom_target(${targetName} ALL DEPENDS ${targetName}-stamp)
    endforeach()
endmacro()

