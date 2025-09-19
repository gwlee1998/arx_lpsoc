TEMPLATE_FILE = ${RVX_DEV_UTIL_HOME}/makefile_template/Makefile.vta.template
include ${RVX_UTIL_HOME}/template_makefile.mh

IMPORT_DIR_NAME = imported
TARGET_DIR_NAME = src
MANUAL_DIR_NAME = manual

IMPORT_DIR = ${CURDIR}/${IMPORT_DIR_NAME}
TARGET_DIR = ${CURDIR}/${TARGET_DIR_NAME}
MANUAL_DIR = ${CURDIR}/${MANUAL_DIR_NAME}

VTA_NAME=Test.DefaultPynqConfig
VTA_FILE=${IMPORT_DIR}/${VTA_NAME}.sv

update: import porting

import:

porting: split overwrite_manual

clean_porting:
	rm -rf ./${TARGET_DIR_NAME}

split: clean_porting
	mkdir ./${TARGET_DIR_NAME}
	python3 ${RVX_DEV_UTIL_HOME}/module_spliter.py -i ${VTA_FILE} -o ${TARGET_DIR}
	rm -rf ${TARGET_DIR}/Test.v
	rm -rf ${TARGET_DIR}/VTAHost*.v
	rm -rf ${TARGET_DIR}/VTAMem*.v
	rm -rf ${TARGET_DIR}/VTASim.v

overwrite_manual:
	-cd ${MANUAL_DIR} && cp -r * ${TARGET_DIR}
