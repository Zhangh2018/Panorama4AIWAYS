import os
import shutil


#Copy directory function
def copytree(src, dst, symlinks=False, ignore=None):
    for item in os.listdir(src):
        s = os.path.join(src, item)
        d = os.path.join(dst, item)
        if os.path.isdir(s):
            shutil.copytree(s, d, symlinks, ignore)
        else:
            shutil.copy2(s, d)

#Setup some variables
evbcrosstcf = os.getcwd() + "/evb.cross.cmake"
rootDirPath = os.getcwd()
sdk_root_dir = os.environ.get('SDK_ROOT')

#Create build folder for openvx
if os.path.exists("openvx-build"):
    shutil.rmtree(rootDirPath + "/openvx-build", ignore_errors=True)
os.mkdir("openvx-build")


openvx_out_dir = os.getcwd() + "/openvx-build"
os.chdir(openvx_out_dir)
if not os.path.exists("temp_arm"):
    os.mkdir("temp_arm")
os.chdir("temp_arm")

#Build openvx in debug mode and release mode
print "Compiling openvx in " + os.getcwd()
installPrefix = "\"" + openvx_out_dir + "/install/Linux/arm/debug\""
os.system("cmake ../../openvx_sample -G \"Unix Makefiles\" -DCMAKE_TOOLCHAIN_FILE=\"" + evbcrosstcf + "\" -DCMAKE_INSTALL_PREFIX=" + installPrefix +" -DCMAKE_BUILD_TYPE=Debug")
os.system("cmake --build . --config Debug --target install")
installPrefix = "\"" + openvx_out_dir + "/install/Linux/arm/release\""
os.system("cmake ../../openvx_sample -G \"Unix Makefiles\" -DCMAKE_TOOLCHAIN_FILE=\"" + evbcrosstcf + "\" -DCMAKE_INSTALL_PREFIX=" + installPrefix +" -DCMAKE_BUILD_TYPE=Release")
os.system("cmake --build . --config Release --target install")

os.chdir(rootDirPath)
if os.path.exists("ovx"):
    shutil.rmtree(rootDirPath + "/ovx", ignore_errors=True)
os.mkdir("ovx")
os.chdir("ovx")
os.mkdir("include")
shutil.rmtree(rootDirPath + "/openvx-build/temp_arm/", ignore_errors=True)

copytree(rootDirPath + "/openvx-build/install/Linux/", rootDirPath + "/ovx", symlinks=False, ignore=None)
copytree(rootDirPath + "/ovx/arm/debug/include", rootDirPath + "/ovx/include/", symlinks=False, ignore=None)
shutil.rmtree(rootDirPath + "/ovx/arm/debug/include", ignore_errors=True)
shutil.rmtree(rootDirPath + "/ovx/arm/release/include", ignore_errors=True)

os.system("sudo cp -r " + rootDirPath +"/ovx " + sdk_root_dir + "/3rdparty/")