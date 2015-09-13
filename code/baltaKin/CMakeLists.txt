SET(PROJECTNAME baltaKin)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

SET(ADD_IPOPT TRUE CACHE BOOL "Do you want to add IPOPT library?")
IF(ADD_IPOPT)
   FIND_PACKAGE(IPOPT)
   IF(NOT IPOPT_FOUND)
      MESSAGE("IPOPT not found! Keep on building...")
   ENDIF(NOT IPOPT_FOUND)
ENDIF(ADD_IPOPT)

# this define has to be checked
IF(LINUX)
   ADD_DEFINITIONS(-DHAVE_CONFIG_H)
ENDIF(LINUX)

SET(folder_source src/iKinFwd.cpp src/iKinInv.cpp)
SET(folder_header include/iCub/iKinFwd.h include/iCub/iKinInv.h)

IF(IPOPT_FOUND)
   SET(folder_source ${folder_source} src/iKinIpOpt.cpp)
   SET(folder_header ${folder_header} include/iCub/iKinIpOpt.h)
ENDIF(IPOPT_FOUND)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include ${IPOPT_INC_DIRS})

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})


