APPLICATION = imu
BOARD ?= esp32-wroom-32

FEATURES_REQlsUIRED = periph_i2c
FEATURES_OPTIONAL = periph_i2c_reconfigure
FEATURES_REQUIRED += cpp
RIOTBASE ?= $(CURDIR)/../../..

USEMODULE += shell_cmds_default
USEMODULE += shell_cmd_i2c_scan
USEMODULE += xtimer

# avoid running Kconfig by default
SHOULD_RUN_KCONFIG ?=

EXTERNAL_BOARD_DIRS += $(CURDIR)/../external_board_dirs/esp-ci-boards

include $(RIOTBASE)/Makefile.include

