#!/bin/sh

SDK_ROOT := $(call path_relative_to,$(CURDIR),$(CURR_SDK_ROOT))

doxygen_acf()
{
	echo "###############################################################################"
	echo "#                          Doxygen ACF file                                   #"
	echo "###############################################################################"
	cd $SDK_ROOT/docs/doxygen/acf
	doxygen acf.dox
	cd ACF_DOXYGEN_OUTPUT/latex
	yes r | make all
	mv refman.pdf UG-10267-04-07-ACF_Reference_Guide.pdf
	cp UG-10267-04-07-ACF_Reference_Guide.pdf $SDK_ROOT/docs/apex/acf
	cd $SDK_ROOT
}

doxygen_ApexCV_Base()
{
	echo "###############################################################################"
	echo "#                          Doxygen ApexCV Base file                           #"
	echo "###############################################################################"
	cd $SDK_ROOT/docs/doxygen/ApexCV_Base
	doxygen ApexCV_Base_doxyfile.dox
	cd $SDK_ROOT/docs/apex/apex_cv/ApexCV_Base/latex
	yes r | make all
	mv refman.pdf UG-10328-01-11_APEX-CV_Base_Library.pdf
	cp UG-10328-01-11_APEX-CV_Base_Library.pdf $SDK_ROOT/docs/apex/apex_cv
	cd $SDK_ROOT
}

doxygen_ApexCV_Pro()
{
	echo "###############################################################################"
	echo "#                          Doxygen ApexCV Pro file                            #"
	echo "###############################################################################"
	cd $SDK_ROOT/docs/doxygen/ApexCV_Pro
	doxygen ApexCV_Pro_doxyfile.dox
	cd $SDK_ROOT/docs/apex/apex_cv/ApexCV_Pro/latex
	yes r | make all
	mv refman.pdf UG-10328-02-13_APEX-CV_Pro_Library.pdf
	cp UG-10328-02-13_APEX-CV_Pro_Library.pdf $SDK_ROOT/docs/apex/apex_cv
	cd $SDK_ROOT
}

doxygen_Apex_DNN()
{
	echo "###############################################################################"
	echo "#                          Doxygen Apex DNN file                              #"
	echo "###############################################################################"
	cd $SDK_ROOT/docs/doxygen/ApexDNN_User
	doxygen ApexDNN_User_doxyfile.dox
	cd $SDK_ROOT/docs/apex/apex_dnn/ApexDNN_User/latex
	yes r | make all
	mv refman.pdf APEX-DNN_User.pdf
	cp APEX-DNN_User.pdf $SDK_ROOT/docs/apex/apex_dnn
	cd $SDK_ROOT
}

doxygen_Apex_APU()
{
	echo "###############################################################################"
	echo "#                          Doxygen Apex APU file                              #"
	echo "###############################################################################"
	echo " - Copy 2 file: APU2.dox and APU2_vec.dox to ipp-APU2\L-2016.09-2\win32\designs\APU2\lib"
	echo " - Copy ../images/logo.png into same folder as ipp-APU2\L-2016.09-2\win32\designs\APU2\lib"
	echo " - Go to ipp-APU2\L-2016.09-2\win32\designs\APU2\lib"
	echo " - Run: mkdir doc"
	echo " - Run: doxygen APU2.dox"
	echo " - Produced html will be written into doc\html\instructionset"
}

fhelp()
{
	echo "###############################################################################"
	echo "#                          Please enter again !!!                             #"
	echo "###############################################################################"
	
    echo "  ./doxygen.sh: Generate all docs "
    echo "  ./doxyden.sh -m acf: gen acf docs "
    echo "  ./doxyden.sh -m apexcv_base: gen apexcv_base docs "
    echo "  ./doxyden.sh -m apexcv_pro: gen apencv_pro docs "
    echo "  ./doxyden.sh -m apex_dnn: gen dnn docs "
    echo "  ./doxyden.sh -m apex_apu: gen apu docs "
    exit 0
}

if [ $# -eq "0" ]
then
	echo "###############################################################################"
	echo "#                          Doxygen Apex All file                              #"
	echo "###############################################################################"
	doxygen_acf
	doxygen_ApexCV_Base
	doxygen_ApexCV_Pro
	doxygen_Apex_APU
	doxygen_Apex_DNN

elif [ $# -eq "2" ]
then
	while [ -n "$1" ]
	do
		echo "$1"
		case "$1" in
			-m)
				if [ "$2" = "dnn" ]
				then
					echo "Doxygen file DNN"
					doxygen_Apex_DNN
				elif [ "$2" = "apu" ]
				then
					echo "Doxygen file APU"
					doxygen_Apex_APU
				elif [ "$2" = "acf" ]
				then
					echo "Doxygen file ACF"
					doxygen_acf
				elif [ "$2" = "apexcv_base" ]
				then
					echo "Doxygen file APEXCV_BASE"
					doxygen_ApexCV_Base
				elif [ "$2" = "apexcv_pro" ]
				then
					echo "Doxygen file APEXCV_PRO"
					doxygen_ApexCV_Pro
				else
					fhelp
				fi
				break
				;;

			*)
				fhelp
				break
				;;
	esac
	done
else
	fhelp
fi
exit 0