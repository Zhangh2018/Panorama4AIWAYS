cd ApexCV_Base
call ApexCV_Base_doxymake.bat
cd ..\ApexCV_Pro
call ApexCV_Pro_doxymake.bat
cd ..\..\apex\apex_cv\ApexCV_Base\latex
call make.bat
copy refman.pdf ..\..\UG-10328-01-11_APEX-CV_Base_Library.pdf
cd ..\..\ApexCV_Pro\latex
call make.bat
copy refman.pdf ..\..\UG-10328-02-13_APEX-CV_Pro_Library.pdf
cd ..\..\..\..\doxygen