import os
import shutil
import tarfile

def copytree(src, dst, symlinks=False, ignore=None):
    for item in os.listdir(src):
        s = os.path.join(src, item)
        d = os.path.join(dst, item)
        if os.path.isdir(s):
            shutil.copytree(s, d, symlinks, ignore)
        else:
            shutil.copy2(s, d)

evbcrosstcf = os.getcwd() +"/evb.cross.cmake"
rootDirPath = os.getcwd()
sdk_root_dir = os.environ.get('SDK_ROOT')

# Create folder to build
os.chdir(rootDirPath)
if os.path.exists("googletest-build"):
    shutil.rmtree(rootDirPath + "/googletest-build", ignore_errors=True)
os.mkdir("googletest-build")
os.chdir("googletest-build")

if not os.path.exists("arm"):
    os.mkdir("arm")
os.chdir("arm")

# Start build googletest debug mode
if not os.path.exists("debug"):
    os.mkdir("debug")
os.chdir("debug")
print "Compiling googletest " + os.getcwd()
os.system("cmake ../../../googletest-master -G \"Unix Makefiles\" -DCMAKE_TOOLCHAIN_FILE=\"" + evbcrosstcf + "\" -DCMAKE_BINARY_DIRECTORY=\".\" -DTHREADS_PTHREAD_ARG=2 -DCMAKE_BUILD_TYPE=Debug")
os.system("cmake --build . --config Debug")

#Build googletest release mode
os.chdir("..")
if not os.path.exists("release"):
    os.mkdir("release")
os.chdir("release")
os.system("cmake ../../../googletest-master -G \"Unix Makefiles\" -DCMAKE_TOOLCHAIN_FILE=\"" + evbcrosstcf + "\" -DCMAKE_BINARY_DIRECTORY=\".\" -DTHREADS_PTHREAD_ARG=2 -DCMAKE_BUILD_TYPE=Release")
os.system("cmake --build . --config Release")

os.chdir(rootDirPath)
if os.path.exists("googletest"):
    shutil.rmtree(rootDirPath + "/googletest", ignore_errors=True)
os.mkdir("googletest")
os.chdir("googletest")
gtest_dir = os.getcwd()

if not os.path.exists("arm"):
    os.mkdir("arm")
os.chdir("arm")

if not os.path.exists("debug"):
   os.mkdir("debug")
if not os.path.exists("release"):
   os.mkdir("release")

#Collect all necessary file
os.chdir(gtest_dir)
os.mkdir("include")
shutil.copy2(rootDirPath+"/googletest-build/arm/debug/lib/libgmockd.a", gtest_dir+"/arm/debug")
shutil.copy2(rootDirPath+"/googletest-build/arm/debug/lib/libgmock_maind.a", gtest_dir+"/arm/debug")
shutil.copy2(rootDirPath+"/googletest-build/arm/debug/lib/libgtestd.a", gtest_dir+"/arm/debug")
shutil.copy2(rootDirPath+"/googletest-build/arm/debug/lib/libgtest_maind.a", gtest_dir+"/arm/debug")
shutil.copy2(rootDirPath+"/googletest-build/arm/release/lib/libgmock.a", gtest_dir+"/arm/release")
shutil.copy2(rootDirPath+"/googletest-build/arm/release/lib/libgmock_main.a", gtest_dir+"/arm/release")
shutil.copy2(rootDirPath+"/googletest-build/arm/release/lib/libgtest.a", gtest_dir+"/arm/release")
shutil.copy2(rootDirPath+"/googletest-build/arm/release/lib/libgtest_main.a", gtest_dir+"/arm/release")
copytree(rootDirPath + "/googletest-master/googletest/include/", gtest_dir + "/include", symlinks=False, ignore=None)

os.chdir(rootDirPath)
os.system("sudo cp -r " + rootDirPath +"/googletest " + sdk_root_dir + "/3rdparty/")
