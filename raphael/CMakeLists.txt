cmake_minimum_required(VERSION 3.0.2)
project(raphael)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/five_seconds scripts/raphael_node
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
