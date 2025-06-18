# Caminho dos headers e da lib Javino
INCLUDE = -I../../JavinoInC/include
LIBRARIES = -L../../JavinoInC/lib -ljavino

# Prepara a variável WEBOTS_HOME_PATH escapando espaços (caso existam)
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH := $(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))

# Inclui o Makefile padrão do Webots
include $(WEBOTS_HOME_PATH)/resources/Makefile.include