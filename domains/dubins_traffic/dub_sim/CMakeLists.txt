cmake_minimum_required(VERSION 2.8.3)
project(dub_sim)
catkin_package()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
