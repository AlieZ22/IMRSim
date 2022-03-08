NAM_PROJ = imrsim
DIR_UTIL = $(NAM_PROJ)_util
DIR_KMOD = $(NAM_PROJ)_kmod
FIL_TAR  = $(NAM_PROJ).tgz
EXE_UTIL = $(NAM_PROJ)_util
EXE_KMOD = dm-$(NAM_PROJ).ko

all: util kmod
tar: all
	tar -czvf $(FIL_TAR) -C $(DIR_UTIL) $(EXE_UTIL) \
	    -C $(PWD)/$(DIR_KMOD) $(EXE_KMOD) 
install: kmod
util:
	cd $(DIR_UTIL) && make
kmod:
	cd $(DIR_KMOD) && make
clean:
	rm -f $(FIL_TAR)
	cd $(DIR_UTIL) && make clean
	cd $(DIR_KMOD) && make clean
install:
	cd $(DIR_KMOD) && make modules_install
