
dirs = $(shell ls controllers)

WEBOTS_HOME=/usr/local/webots

VPATH = JavinoInC/lib

.PHONY: all clean

libjavino.a: 
	git clone https://github.com/chon-group/JavinoInC.git
	make -C JavinoInC all

all: libjavino.a
	make WEBOTS_HOME="$(WEBOTS_HOME)" -C controllers/four_wheels_collision_avoidance all
	make WEBOTS_HOME="$(WEBOTS_HOME)" -C controllers/target all
	
clean:
	make WEBOTS_HOME="$(WEBOTS_HOME)" -C controllers/four_wheels_collision_avoidance clean
	make WEBOTS_HOME="$(WEBOTS_HOME)" -C controllers/target clean
