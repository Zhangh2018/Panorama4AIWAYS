echo off
cls
pushd.
set appdir=.


set TEST_LIST=ApexCV_Pro_doxyfile.dox

FOR %%X IN (%TEST_LIST%) DO (
   echo ********************Doxygen Build: %%X*************************
   doxygen %%X
)

popd
goto END


:END
pause
