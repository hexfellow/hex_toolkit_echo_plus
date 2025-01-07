cmake_minimum_required(VERSION 3.0.2)
project(hex_toolkit_echo_plus)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir config/ros1 launch/ros1 urdf)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

file(GLOB SCRIPT_FILES ${PROJECT_NAME}/*)
foreach(SCRIPT ${SCRIPT_FILES})
    if(IS_DIRECTORY ${SCRIPT})
        get_filename_component(last_dir ${SCRIPT} NAME)
        install(DIRECTORY ${PROJECT_NAME}/${last_dir}/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${PROJECT_NAME}/${last_dir}
            PATTERN "__pycache__" EXCLUDE
        )
    elseif(${SCRIPT} MATCHES "__init__\\.py$")
        install(FILES ${SCRIPT}
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${PROJECT_NAME}
        )
    elseif(${SCRIPT} MATCHES "\\.py$")
        install(PROGRAMS ${SCRIPT}
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${PROJECT_NAME}
        )
    endif()
endforeach()