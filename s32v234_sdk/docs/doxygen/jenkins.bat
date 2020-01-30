:: require doxygen 1.8.6 + MiKTeX 2.9 + gs9.16 + perl

cd %APEX_FULL_WORKSPACE%

@SET CAD_HOME=c:\tools

:: Configure Doxygen.
@SET PATH=%PATH%;%CAD_HOME%\doxygen\bin

:: Configure TexLive.
@SET PATH=%PATH%;%CAD_HOME%\MiKTeX 2.9\miktex\bin

:: Configure Ghostscript.
@SET PATH=%PATH%;%CAD_HOME%\gs\gs9.16\bin

:: Configure Perl.
@SET PATH=%PATH%;%CAD_HOME%\perl\bin

:: Generate for ApexCV
cd %APEX_FULL_WORKSPACE%
cd s32v234_sdk\docs\doxygen\ApexCV_Base
call ApexCV_Base_doxymake.bat

cd %APEX_FULL_WORKSPACE%
cd s32v234_sdk\docs\doxygen\ApexCV_Pro
call ApexCV_Pro_doxymake.bat

:: Generate for ACF
cd %APEX_FULL_WORKSPACE%
cd s32v234_sdk/docs/doxygen/acf
call acf_doxymake.bat

cd %APEX_FULL_WORKSPACE%
cd s32v234_sdk\docs\apex\apex_cv\ApexCV_Base\latex
call make.bat
move refman.pdf ..\UG-10328-01-07_APEX-CV_Base_Library.pdf

cd %APEX_FULL_WORKSPACE%
cd s32v234_sdk\docs\apex\apex_cv\ApexCV_Pro\latex
call make.bat
move refman.pdf ..\UG-10328-02-08_APEX-CV_Pro_Library.pdf

cd %APEX_FULL_WORKSPACE%
cd s32v234_sdk/docs/doxygen/acf/ACF_DOXYGEN_OUTPUT/latex
call make.bat
move refman.pdf ..\UG-10267-03-01-02_ACF.pdf

:: @robocopy /E %APEX_FULL_WORKSPACE%\Documentation \\qub28-hws1\workspace\verif\sw2\SW_REG_BUILD_%APEX_BUILD_NUMBER%_%APEX_BUILD_ID%\jenkins\Documentation

@exit 0
