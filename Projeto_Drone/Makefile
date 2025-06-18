# Diretórios dos controladores
dirs = $(shell ls controllers)

# Não define um valor padrão — você deve passar WEBOTS_HOME ao rodar pois os lugares mudam de instalação para instalação
# Exemplo: make WEBOTS_HOME=/var/lib/snapd/snap/webots/current/usr/share/webots

VPATH = JavinoInC/lib

.PHONY: all clean

# Clona e compila a biblioteca Javino
libjavino.a: 
	git clone https://github.com/chon-group/JavinoInC.git
	make -C JavinoInC all

# Compila os controladores
all: libjavino.a
	make WEBOTS_HOME="$(WEBOTS_HOME)" -C controllers/Target all
	make WEBOTS_HOME="$(WEBOTS_HOME)" -C controllers/Drone_Controller_Test all

# Limpa os controladores
clean:
	make WEBOTS_HOME="$(WEBOTS_HOME)" -C controllers/Target clean
	make WEBOTS_HOME="$(WEBOTS_HOME)" -C controllers/Drone_Controller_Test clean
