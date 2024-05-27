# Ensure minimal size by default
DEVELHELP ?= 0
APPLICATION = tingml-paho-mqtt
BOARD ?= esp32-wroom-32

USEPKG += tflite-micro

# TensorFlow-Lite crashes on M4/M7 CPUs when FPU is enabled, so disable it by
# default for now
DISABLE_MODULE += cortexm_fpu
USEMODULE += gesture
FEATURES_REQUIRED += cpp
FEATURES_REQlsUIRED = periph_i2c
FEATURES_OPTIONAL = periph_i2c_reconfigure
EXTERNAL_MODULE_DIRS += external_modules
WIFI_SSID ?= "EmNets-301"
WIFI_PASS ?= "eagle402"

# use ESP WiFi in station mode as network interface
USEMODULE += esp_wifi
USEMODULE += cpp
USEMODULE += periph_i2c
USEMODULE += ztimer
ifneq (,$(DEFAULT_MQTT_CLIENT_ID))
  CFLAGS += -DDEFAULT_MQTT_CLIENT_ID=\"$(DEFAULT_MQTT_CLIENT_ID)\"
endif
ifneq (,$(DEFAULT_MQTT_USER))
  CFLAGS += -DDEFAULT_MQTT_USER=\"$(DEFAULT_MQTT_USER)\"
endif
ifneq (,$(DEFAULT_MQTT_PWD))
  CFLAGS += -DDEFAULT_MQTT_PWD=\"$(DEFAULT_MQTT_PWD)\"
endif
USEPKG += paho-mqtt
USEMODULE += netdev_default
USEMODULE += shell_cmds_default
USEMODULE += ps

# use ESP WiFi in station mode as network interface
USEMODULE += esp_wifi
USEMODULE += cpp
USEMODULE += periph_i2c

USEMODULE += shell_cmds_default
USEMODULE += shell_cmd_i2c_scan
USEMODULE += xtimer
USEMODULE += ps
USEMODULE += netdev_default
USEPKG += paho-mqtt
RIOTBASE ?= $(CURDIR)/../../..
# As there is an 'Kconfig' we want to explicitly disable Kconfig by setting
# the variable to empty
SHOULD_RUN_KCONFIG ?=

# paho-mqtt depends on TCP support, choose which stacks you want
GNRC_IPV6 ?= 0
LWIP_IPV4 ?= 1
LWIP_IPV6 ?= 0
include Makefile.lwip
include Makefile.gnrc

USEMODULE += sock_async_event
USEMODULE += sock_ip
USEMODULE += sock_udp
USEMODULE += sock_tcp

USEMODULE += ztimer
USEMODULE += ztimer_msec
USEMODULE += xtimer

include $(RIOTBASE)/Makefile.include

ifneq (,$(filter arch_esp,$(FEATURES_USED)))
  CFLAGS += -DWIFI_SSID=\"$(WIFI_SSID)\"
  CFLAGS += -DWIFI_PASS=\"$(WIFI_PASS)\"
endif
