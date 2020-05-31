'''
Created on Apr 9, 2018

@author: modys
'''


###################################################### Includes ########################################################
import os
import shutil
import platform
import subprocess
import argparse
########################################################################################################################


################################################## Common Functions ####################################################

#----------------------------------------------------------------------------------------------------------------------#
'''
/**
 * @brief _clean ,Helper function to erase build folders.
 *
 * @param[in]  dest_path
 *  The destination path to the build folder.
 *
 * @param[in]  dest_folder
 *  The build folder name.
 *
 *  @return  None.
 */
'''
def _clean (dest_path,dest_folder):
    #construct build folder path
    if(dest_path == './'):
        folder_full_path = dest_folder
    else:
        folder_full_path = dest_path+'/'+dest_folder

    print("---------Clean------------")

    ## Try to remove tree; if failed show an error using try...except on screen
    try:
        shutil.rmtree(folder_full_path)
    except OSError as e:
        print ("Error: %s - %s." % (e.filename,e.strerror))
#----------------------------------------------------------------------------------------------------------------------#

#----------------------------------------------------------------------------------------------------------------------#    
'''
/**
 * @brief _build ,Helper function to build using cmake.
 *
 * @param[in]  cmake_flag
 *  The cmake flag is used in cmake to generate the correct makefile.
 *
 * @param[in]  dest_path
 *  The destination path to the build folder.
 *
 * @param[in]  dest_folder
 *  The build folder name.
 *
 *  @return  None.
 */
'''
def _build (cmake_flags,dest_path,dest_folder):
    #construct build folder path
    if(dest_path == './'):
        folder_full_path = dest_folder
    else:
        folder_full_path = dest_path+'/'+dest_folder

    if cmake_flags:
        cmake_flag_list = []
        cmake_flag_list = cmake_flags.split(",")
        cmake_flags = ""
        for flag in cmake_flag_list:
            cmake_flags +=' -D'+ flag + '=1'
    
    # Create a new directory
    if not os.path.exists(folder_full_path):
        print("---------Create New Build Directory------------")
        print(folder_full_path)
        os.makedirs(folder_full_path)
        
    print("---------Build------------")
      
    if(platform.system() == 'Windows'):
        print("---------Build Windows------------")
        print("%s is ON"%(cmake_flags))
        cmd = 'cd'+' '+folder_full_path
        cmd =  cmd + ' & cmake '+cmake_flags+' cmake .. -G "MinGW Makefiles" & mingw32-make'+' -j8'
        print("cmd = %s"%cmd)
        subprocess.call(cmd,shell=True)
            
    elif (platform.system()=='Linux'):
        print("---------Build Linux--------------")
        print("%s is ON"%(cmake_flags))
        cmd = 'cd'+' '+os.path.abspath(folder_full_path)
        cmd = cmd + '&& cmake '+cmake_flags+' cmake .. && make'
        print("cmd = %s"%cmd)
        subprocess.call(cmd,shell=True)
            
    else:
        print("Platform Unknown")
#----------------------------------------------------------------------------------------------------------------------#

#----------------------------------------------------------------------------------------------------------------------#
'''
/**
 * @brief _run ,Helper function to rrun the output and build the ocde if needed.
 *
 * @param[in]  cmake_flag
 *  The cmake flag is used in cmake to generate the correct makefile.
 *
 * @param[in]  dest_path
 *  The destination path to the build folder.
 *
 * @param[in]  dest_folder
 *  The build folder name.
 *
 * @param[in]  app_name
 *  The output of build process.
 *
 *  @return  None.
 */
'''
def _run(cmake_flag,dest_path,dest_folder,run_folder,app_name,extraParam=None,test_report=None):
    _build(cmake_flag,dest_path,dest_folder)

    if(extraParam is None):
        extraParam = ''
    if(run_folder is not ''):
        run_folder = '/'+run_folder

    print("---------Run Application------------")
    #construct build folder path
    if(dest_path == './'):
        folder_full_path = dest_folder+run_folder
    else:
        folder_full_path = dest_path+'/'+dest_folder+run_folder

    if(platform.system() == 'Windows'):
        print("---------Run App on Windows(Not Yet Done)------------")
        cmd = 'cd'+' '+folder_full_path+ ' & '+app_name+'.exe'+' '+extraParam
        print("cmd = %s"%cmd)
        subprocess.call(cmd,shell=True)
        if(cmake_flag is 'USE_UNIT_TESTING'):
            cmd = test_report
            print("cmd = %s"%cmd)
            subprocess.call(cmd,shell=True)
            
    elif (platform.system()=='Linux'):
        print("---------Run App on Linux--------------")
        cmd = 'cd'+' '+os.path.abspath(folder_full_path)+' && ./'+app_name+' '+extraParam
        print("cmd = %s"%cmd)
        subprocess.call(cmd,shell=True)
            
    else:
        print("Platform Unknown")
#----------------------------------------------------------------------------------------------------------------------#


######################################################## Main ##########################################################

if __name__ == '__main__':
    pass

#------------------------------------------------ Utility Options -----------------------------------------------------#
print("--------Project Build System--------------")
print("clean --> Cleans the project")
print("build --> builds the project according to platform type")
print("run --> runs the project")
print("sim_mode --> Use Simulation Mode")
print("test_mode_1 --> Use Test Mode data set 1")
print("test_mode_2 --> Use Test Mode data set 2")
print("unit_test_mode --> Use Unit Test Mode")


parser = argparse.ArgumentParser()
parser.add_argument("--clean", help="Cleans the project",action="store_true")
parser.add_argument("--build", help="builds the project according to platform type",action="store_true")
parser.add_argument("--run", help="runs the project",action="store_true")
parser.add_argument("--sim_mode", help="Use Simulation Mode",action="store_true")
parser.add_argument("--test_mode_1", help="Use Test Mode data set 1",action="store_true")
parser.add_argument("--test_mode_2", help="Use Test Mode data set 2",action="store_true")
parser.add_argument("--unit_test_mode", help="Use Unit Test Mode",action="store_true")
parser.add_argument("--ekf_app", help="Extnded Kalman Filter APP",action="store_true")
parser.add_argument("--ukf_app", help="Unscented Kalman Filter",action="store_true")
parser.add_argument("--highway", help="Use highway Mode",action="store_true")
parser.add_argument("--gen_doc", help="runs the documentaion",action="store_true")

args = parser.parse_args()
#----------------------------------------------------------------------------------------------------------------------#

#-------------------------------------------------- Temp Variables ----------------------------------------------------#
folder_path = './'
folder      = 'debug'
run_folder  = ''
app_name    = 'kf'
cmake_flag  = ''      
extraParam  = None
data_path   = '../data/' 
data_set_1  = 'sample-laser-radar-measurement-data-1.txt'
data_set_2  = 'sample-laser-radar-measurement-data-2.txt'
output_folder_test = 'gtest'
doc_folder = 'doc'
test_report = ''
#----------------------------------------------------------------------------------------------------------------------#


#-------------------------------------------------- States Machine ----------------------------------------------------#

#--------------------------------------------------- USE_HIGHWAY -------------------------------------------------------#
if args.highway:
    cmake_flag = ''
    cmake_flag  = 'USE_HIGHWAY'
    folder_path = './'

#--------------------------------------------------- USE_TEST_1 -------------------------------------------------------#
if args.test_mode_1:
    cmake_flag = ''
    if args.ekf_app:
        cmake_flag  = 'USE_TEST,USE_EKF'

    if args.ukf_app:
        cmake_flag  = 'USE_TEST,USE_UKF'

    folder_path = './'
    extraParam  = data_path+data_set_1

#--------------------------------------------------- USE_TEST_2 -------------------------------------------------------#
if args.test_mode_2:
    cmake_flag = ''
    if args.ekf_app:
        cmake_flag  = 'USE_TEST,USE_EKF'

    if args.ukf_app:
        cmake_flag  = 'USE_TEST,USE_UKF'

    folder_path = './'
    extraParam  = data_path+data_set_2

#----------------------------------------------- USE_UNIT_TESTING -----------------------------------------------------#
if args.unit_test_mode:
    folder_path = './'
    app_name    = 'gtest'
    cmake_flag  = 'USE_UNIT_TESTING'
    extraParam  = ' --gtest_output=xml:report-file.xml'
    test_report = 'python gtest2html.py '+folder+'/'+\
                  '/report-file.xml '+doc_folder+'/TestReport'+app_name+'.html'


print(extraParam)
if args.clean:
    _clean(folder_path, folder)
 
if args.build:
    _build(cmake_flag,folder_path,folder)
    
if args.run:
    _run(cmake_flag, folder_path,folder,run_folder,app_name,extraParam,test_report)

#----------------------------------------------- USE_UNIT_TESTING -----------------------------------------------------#
if args.gen_doc:    
    print("------Generate the documents---------")
    subprocess.call('cd doc/doxygen_config & doxygen Doxyfile',shell=True)
    subprocess.call('cd doc/'+'KF_LLD'+'/html & index.html',shell=True)
#----------------------------------------------------------------------------------------------------------------------#

########################################################################################################################