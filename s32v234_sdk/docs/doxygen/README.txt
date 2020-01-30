Toolchain used to create APEX doxygen based documentations
==========================================================

Prerequisities:
-----------------
Installation:
	Ubuntu 1604 or Ubuntu 1804.
	doxygen-1.8.11. (sudo apt-get install doxygen)
	textlive (sudo apt-get install texlive-full )

Usage
-----------------
	Run file make_apex_pdfs.sh
	Generate all docs (acf, apex-cv, dnn, apu)
	run file:
		./make_apex_pdfs.sh
		Output save in directory SDK_ROOT/docs/apex/<module name>
	Only generate
	run file:
		./make_apex_pdfs.sh [Parameter 1] [Parameter 2]
		Parameters
		+ Parameter 1 - m: module
		+ Parameter 2: module name
		 acf -> gen acf docs
		 apexcv_base -> gen apexcv_base docs
		 apexcv_pro -> gen apexcv_pro docs
		 apex_dnn -> gen apex_dnn docs
		 apex_apu -> gen apex_apu docs
		 * -> help
		 example: ./make_apex_pdfs.sh -m acf



