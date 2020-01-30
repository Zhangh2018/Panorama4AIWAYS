##############################################################################
#
#  CONTENT
#    Build instructions for google-test library
#
##############################################################################

  1) Download googletest
  ----------------------------------
    - Download googletest-master.zip from https://github.com/google/googletest
    - Put evb.cross.cmake file and gtest.py at the same directory with googletest-master

  2) Modify some file to compatible with your system:
  ----------------------------------
    - Edit "COMPILER_FOLDER_NAME" in evb.cross.cmake up to your compiler folder
     
  3) Build google-test libraries
  -------------------------
    - Run gtest.py
