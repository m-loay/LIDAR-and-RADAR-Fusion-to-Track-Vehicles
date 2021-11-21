cmake_minimum_required(VERSION 3.12 FATAL_ERROR)
project(KalmanFilter)
add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)
# Set Paths
set(KCONFIG_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/src/_generated)
set(KFLIB_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/src/kflib)
set(HIGH_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/src/highway)
set(KFH_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/src/kfheader)
set(TOOLS_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/src/tools)
set(PLOT_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/src/plotter)
set(KFAPP_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/src/kfApp)
set(UKFAPP_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/src/ukfApp)
set(TEST_CASES_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/src/testCases)


# Use simulation, testing or unit test and the mode
option (USE_TEST "Use Test Mode" ON )
option (USE_UNIT_TESTING "Use Unit testing Mode" OFF)
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
configure_file ("${PROJECT_SOURCE_DIR}/src/KalmanConfig.h.in"
                "${KCONFIG_INCLIDE_DIR}/KalmanConfig.h")

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
    add_executable(ukf_highway src/main.cpp)
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
    add_executable(kf src/main.cpp)
    target_include_directories(kf PUBLIC ${EXTRA_INCLUDES})
    target_link_libraries (kf PUBLIC ${EXTRA_LIBS} ${PCL_LIBRARIES} plotter)


elseif(USE_UNIT_TESTING)

	message("Use unit testing Mode")
	enable_testing()

	#set build type to debug and C++ standard to 11 for coverage
	set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE ON)
	add_definitions(-fprofile-arcs -ftest-coverage)
	set(GCOV_LINK_FLAGS "-lgcov --coverage")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCOV_LINK_FLAGS}")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g --coverage -fprofile-arcs -ftest-coverage")

	# Create OBJECT_DIR variable
	set(OBJECT_DIR ${CMAKE_BINARY_DIR}/CMakeFiles/gtest.dir/src)
	message("-- Object files will be output to: ${OBJECT_DIR}")

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
	add_executable(gtest src/main_test.cpp)
	target_include_directories(gtest PUBLIC ${EXTRA_INCLUDES})
	target_link_libraries (gtest ${EXTRA_LIBS} ${GTEST_BOTH_LIBRARIES} Eigen3::Eigen)
	#target_link_libraries (gtest PUBLIC ${EXTRA_LIBS} ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread Eigen3::Eigen)

	# Create the gcov target. Run coverage tests with 'make gcov'
	# Create the gcov target. Run coverage tests with 'make gcov'
	add_custom_target(gcov
			COMMAND mkdir -p gcoverage
			COMMAND ${CMAKE_MAKE_PROGRAM} test
			WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

	add_custom_command(TARGET gcov
			COMMAND echo "=================== GCOV ===================="
			COMMAND gcov -b ${CMAKE_SOURCE_DIR}/src/*.cpp -o ${OBJECT_DIR}
			COMMAND echo "-- Source diretorie: ${CMAKE_SOURCE_DIR}/src/"
			COMMAND echo "-- Coverage files have been output to ${CMAKE_BINARY_DIR}/gcoverage"
			WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/gcoverage)
	add_dependencies(gcov gtest)

	add_test(output_test ${CMAKE_CURRENT_BINARY_DIR}/gtest)
	#target_link_libraries(gtest PUBLIC ${GTEST_BOTH_LIBRARIES} ${EXTRA_LIBS})
	#target_link_libraries (gtest PUBLIC ${EXTRA_LIBS} ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread Eigen3::Eigen)

endif()

