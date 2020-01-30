SET S32V234_SDK_ROOT=%~dp0
IF EXIST %S32V234_SDK_ROOT%..\compilers\gcc-6.3.1-linaro-Earmv7-eabi\i686-mingw32\bin SET PATH=%S32V234_SDK_ROOT%..\compilers\gcc-6.3.1-linaro-Earmv7-eabi\i686-mingw32\bin;%PATH%
IF EXIST %S32V234_SDK_ROOT%..\compilers\gcc-6.3.1-linaro-Earmv8-eabi\i686-mingw32\bin SET PATH=%S32V234_SDK_ROOT%..\compilers\gcc-6.3.1-linaro-Earmv8-eabi\i686-mingw32\bin;%PATH%
IF EXIST %S32V234_SDK_ROOT%..\compilers\gcc-6.3.1-linaro-Xarmv8-linux\i686-mingw32\bin SET PATH=%S32V234_SDK_ROOT%..\compilers\gcc-6.3.1-linaro-Xarmv8-linux\i686-mingw32\bin;%PATH%

set PATH=%S32V234_SDK_ROOT%..\msys32\mingw32\bin;%S32V234_SDK_ROOT%..\msys32\mingw64\bin;%PATH%

echo %PATH%

..\msys32\msys2_shell.cmd -here -full-path
