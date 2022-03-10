CC = g++
GCC = gcc
LD = g++

DECODERSRC = ../decoder/
DECODERINC = ../decoder/
MAINSRC = ./
MAININC = ./

INC = -I$(DECODERINC) -I$(MAININC)
SOURCES = $(wildcard $(DECODERSRC)*.cpp $(DECODERSRC)*.c $(MAINSRC)*.cpp)
OBJ_C = $(patsubst %.c, %.o, $(SOURCES))
OBJ = $(patsubst %.cpp, %.o, $(OBJ_C))
TARGET = UserDecoderLib.so

OPTION = -DNOT_OUTPUT_INNER_FILE
CFLAGS = -fpermissive -shared -fPIC -fvisibility=hidden $(OPTION)

#links
all:$(OBJ)
	$(CC) $(OBJ) $(CFLAGS) -o $(TARGET)
	@rm -rf $(OBJ)
	
#compile
%.o: %.cpp
	$(CC) $(INC) -c $(CFLAGS) $< -o $@
	

.PHONY:clean
clean:
	@echo "Remove linked and compiled files......"
	rm -rf $(OBJ) $(TARGET)