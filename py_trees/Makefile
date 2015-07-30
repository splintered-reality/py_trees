#############################################################################################
# Build Documentation
#############################################################################################

NAME=$(notdir $(CURDIR))
LOCAL_DIR = $(shell pwd)/doc/html/
VERSION   = $(shell catkin_package_version .)
REMOTE_DIR:=files@files.yujin.com:shared/documentation/${NAME}/${VERSION}
REMOTE_SSH:=files@files.yujin.com

docs:
	rosdoc_lite .

dude:
	@echo ${VERSION}
	@echo ${NAME}

upload:
	ssh ${REMOTE_SSH} 'mkdir -p shared/documentation/${NAME}/${VERSION}; rm -f shared/documentation/${NAME}/latest; ln -sf ${VERSION} shared/documentation/${NAME}/latest'
	rsync --delete --recursive --links --verbose ${LOCAL_DIR} ${REMOTE_DIR}
