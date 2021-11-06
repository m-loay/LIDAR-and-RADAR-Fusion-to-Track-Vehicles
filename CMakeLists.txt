cmake_minimum_required(VERSION 3.12 FATAL_ERROR)
project(KalmanFilter)
# Set Paths
set(KCONFIG_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/_generated)
set(KFLIB_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/kflib)
set(HIGH_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/highway)
set(KFH_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/kfheader)
set(TOOLS_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/tools)
set(PLOT_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/plotter)
set(KFAPP_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/kfApp)
set(UKFAPP_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/ukfApp)
set(TEST_CASES_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/testCases)

# Use simulation  testing mode or Unit testing
option (USE_TEST "Use Test Mode" OFF )
option (USE_UNIT_TESTING "Use Unit testing Mode" ON)
option (USE_EKF "Use Extnded Kalman Filter APP" OFF)  
option (USE_UKF "Use Unscented Kalman Filter" ON) 
option (USE_HIGHWAY "Use High way Mode" OFF)       

#find the pacakges required for eigen and save directory
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})
list(APPEND EXTRA_LIBS Eigen3::Eigen)

if(USE_HIGHWAY)
    find_package(PCL 1.10.1 REQUIRED)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
endif()

# configure a header file to pass some of the CMake settings
# to the source code
make_directory("${KCONFIG_INCLIDE_DIR}/_generated")
configure_file ("${PROJECT_SOURCE_DIR}/KalmanConfig.h.in"
                "${KCONFIG_INCLIDE_DIR}/KalmanConfig.h")
                  
# #set build type to debug and C++ standard to 11
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 11) 

# add the binary tree to the search path for include files
# so that we will find KalmanConfig.h
include_directories ("${PROJECT_BINARY_DIR}")
include_directories ("${KCONFIG_INCLIDE_DIR}")
include_directories ("${KFH_INCLIDE_DIR}")
include_directories ("${KFLIB_INCLIDE_DIR}")
include_directories ("${TOOLS_INCLIDE_DIR}")
include_directories ("${PLOT_INCLIDE_DIR}")
include_directories ("${KFAPP_INCLIDE_DIR}")
include_directories ("${TEST_CASES_INCLIDE_DIR}")

if(USE_HIGHWAY)
    #Add Highway sub-directory
    add_subdirectory(${HIGH_INCLIDE_DIR})
    list(APPEND EXTRA_LIBS highway)
    list(APPEND EXTRA_INCLUDES "${HIGH_INCLIDE_DIR}")

    #Add Tools sub-directory
    add_subdirectory(${TOOLS_INCLIDE_DIR})
    list(APPEND EXTRA_LIBS tools)
    list(APPEND EXTRA_INCLUDES "${TOOLS_INCLIDE_DIR}")

    #Add Kalman Filter library sub-directory
	list(APPEND EXTRA_INCLUDES "${KFLIB_INCLIDE_DIR}")
	
	#Add Plotter sub-directory
	add_subdirectory(${PLOT_INCLIDE_DIR})
	list(APPEND EXTRA_LIBS plotter)
	list(APPEND EXTRA_INCLUDES "${PLOT_INCLIDE_DIR}")

    #compile the source code and link it with external libraries
    add_executable(ukf_highway main.cpp)
    target_include_directories(ukf_highway PUBLIC ${EXTRA_INCLUDES})
    target_link_libraries (ukf_highway PUBLIC ${EXTRA_LIBS} ${PCL_LIBRARIES})	

elseif (USE_TEST)
	message("Use Test Mode")

	if(USE_EKF)
		#Add kfApp sub-directory
		add_subdirectory(${KFAPP_INCLIDE_DIR})
		list(APPEND EXTRA_LIBS kfApp)
		list(APPEND EXTRA_INCLUDES "${KFAPP_INCLIDE_DIR}")
	elseif(USE_UKF)
		#Add UkfApp sub-directory
		add_subdirectory(${UKFAPP_INCLIDE_DIR})
		list(APPEND EXTRA_LIBS ukfApp)
		list(APPEND EXTRA_INCLUDES "${UKFAPP_INCLIDE_DIR}")
	else()
		message(FATAL_ERROR "There is no App Selected")
	endif()

    #Add Tools sub-directory
    add_subdirectory(${TOOLS_INCLIDE_DIR})
    list(APPEND EXTRA_LIBS tools)
    list(APPEND EXTRA_INCLUDES "${TOOLS_INCLIDE_DIR}")

    #Add Kalman Filter library sub-directory
	list(APPEND EXTRA_INCLUDES "${KFLIB_INCLIDE_DIR}")
	
	#Add Plotter sub-directory
	add_subdirectory(${PLOT_INCLIDE_DIR})
	list(APPEND EXTRA_LIBS plotter)
	list(APPEND EXTRA_INCLUDES "${PLOT_INCLIDE_DIR}")

    #compile the source code and link it with external libraries
    add_executable(kf main.cpp)
    target_include_directories(kf PUBLIC ${EXTRA_INCLUDES})
    target_link_libraries (kf PUBLIC ${EXTRA_LIBS} ${PCL_LIBRARIES})	


elseif(USE_UNIT_TESTING)

	message("Use unit testing Mode")
	enable_testing()

	#Find Gtest lib and include the directories
	find_package(GTest REQUIRED)
	include_directories(${GTEST_INCLUDE_DIRS})

	#Add Testcases sub-directory
	add_subdirectory(${TEST_CASES_INCLIDE_DIR})
	list(APPEND EXTRA_LIBS testCases)
	list(APPEND EXTRA_INCLUDES "${TEST_CASES_INCLIDE_DIR}")

	#Add kfApp sub-directory
	add_subdirectory(${KFAPP_INCLIDE_DIR})
	list(APPEND EXTRA_LIBS kfApp)
	list(APPEND EXTRA_INCLUDES "${KFAPP_INCLIDE_DIR}")

	#Add Kalman Filter sub-directory
	list(APPEND EXTRA_INCLUDES "${KFLIB_INCLIDE_DIR}")

	#Add Tools sub-directory
	add_subdirectory(${TOOLS_INCLIDE_DIR})
	list(APPEND EXTRA_LIBS tools)
	list(APPEND EXTRA_INCLUDES "${TOOLS_INCLIDE_DIR}")

	#compile the source code and link it with external libraries
	add_executable(gtest main_test.cpp)
	target_include_directories(gtest PUBLIC ${EXTRA_INCLUDES})
	target_link_libraries (gtest ${GTEST_BOTH_LIBRARIES} ${EXTRA_LIBS} Eigen3::Eigen)
	add_test(NAME gtest COMMAND gtest)
endif()

