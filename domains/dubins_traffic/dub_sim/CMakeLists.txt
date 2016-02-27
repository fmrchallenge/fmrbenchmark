cmake_minimum_required(VERSION 2.8.3)
project(dub_sim)
catkin_package()

install(DIRECTORY launch examples
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python (PROGRAMS src/genroad.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
