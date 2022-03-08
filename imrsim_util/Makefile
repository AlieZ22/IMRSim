NAM_PROJ = imrsim
EXE_UTIL = $(NAM_PROJ)_util
DIR_KMOD = ../$(NAM_PROJ)_kmod
MY_TEST_FILE = testInterface

all:mytestfile utilfile

utilfile:$(EXE_UTIL).c $(DIR_KMOD)/$(NAM_PROJ)_types.h  $(DIR_KMOD)/$(NAM_PROJ)_ioctl.h
	$(CC) -o $(EXE_UTIL)  $(EXE_UTIL).c -I$(DIR_KMOD) -g

mytestfile:$(MY_TEST_FILE).c $(DIR_KMOD)/$(NAM_PROJ)_types.h  $(DIR_KMOD)/$(NAM_PROJ)_ioctl.h
	$(CC) -o $(MY_TEST_FILE)  $(MY_TEST_FILE).c -I$(DIR_KMOD) -g

clean:
	rm -f $(EXE_UTIL) *.o
	rm -f $(MY_TEST_FILE) *.o
