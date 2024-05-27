# Configure nrf52dk as default board
BOARD ?= esp32-wroom-32
APPLICATION = nimble_l2cap_server

# load the default test environment
RIOTBASE ?= $(CURDIR)/../../..

# RIOT modules used in for this test
USEMODULE += xtimer
USEMODULE += bluetil_ad
USEMODULE += ztimer
# Get the shared NimBLE and test configuration from the backend server
include nimble.inc.mk

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
CFLAGS += -DLOG_LEVEL=LOG_ALL
CFLAGS += -DDEBUG_ASSERT_VERBOSE

include $(RIOTBASE)/Makefile.include
