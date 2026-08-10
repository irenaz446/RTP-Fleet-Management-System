# ══════════════════════════════════════════════════════════════
# Vehicle Fleet RTP System — Makefile
#
# Targets:
#   all          Build server (x86) + bbg-arm (ARM for BBG)
#   server       Build RTP fleet server (C++, x86)
#   bbg          Build BBG gateway (C, x86 — local testing)
#   bbg-arm      Build BBG gateway for BeagleBone Green (ARM)
#   clean        Remove bin/
#
# STM32: build in STM32CubeIDE (same main.c as V1, 1 car)
# ══════════════════════════════════════════════════════════════

CXX        := g++
CC         := gcc
CC_ARM     := arm-linux-gnueabihf-gcc

CXXFLAGS   := -std=c++17 -Wall -Wextra -g -O2 -I./common
CFLAGS     := -Wall -Wextra -g -O2 -I./common
CFLAGS_ARM := -Wall -Wextra -g -O2 -I./common

LIBS       := -lpthread -lm
SQLIBS     := -lsqlite3

BBG_SRC    := bbg/rtp_bbg.c
BBG_HDR    := bbg/rtp_bbg.h common/common.h

SERVER_SRC := server/server_main.cpp server/Database.cpp
SERVER_HDR := server/RtpReceiver.hpp server/RtpHeader.hpp \
              server/TelemetryParser.hpp server/TripManager.hpp \
              server/TripQueue.hpp server/ViolationDetector.hpp \
              server/Database.hpp \
              common/common.h common/Logger.hpp common/Config.hpp

ARM_CC_FOUND := $(shell which $(CC_ARM) 2>/dev/null)

.PHONY: all server bbg bbg-arm clean

all: server bbg-arm
	@echo ""
	@echo "=== Build complete ==="
	@echo "  bin/rtp_server     — run on PC:  ./bin/rtp_server config/rtp_fleet.cfg"
ifdef ARM_CC_FOUND
	@echo "  bin/rtp_bbg_arm    — copy to BBG:"
	@echo "    scp bin/rtp_bbg_arm debian@192.168.7.2:~/rtp_bbg"
	@echo "    sudo ./rtp_bbg <SERVER_IP> 5004"
else
	@echo "  bin/rtp_bbg_arm    — SKIPPED (install gcc-arm-linux-gnueabihf)"
endif
	@echo "  STM32              — same project as V1, build in STM32CubeIDE"

server: bin/rtp_server

bin/rtp_server: $(SERVER_SRC) $(SERVER_HDR) | bin
	$(CXX) $(CXXFLAGS) -o $@ $(SERVER_SRC) $(LIBS) $(SQLIBS)
	@echo "[OK] bin/rtp_server"

bbg: bin/rtp_bbg

bin/rtp_bbg: $(BBG_SRC) $(BBG_HDR) | bin
	$(CC) $(CFLAGS) -o $@ $(BBG_SRC) $(LIBS)
	@echo "[OK] bin/rtp_bbg  (x86)"

bbg-arm:
ifdef ARM_CC_FOUND
	$(MAKE) bin/rtp_bbg_arm
else
	@echo "[SKIP] rtp_bbg_arm — install gcc-arm-linux-gnueabihf first"
endif

bin/rtp_bbg_arm: $(BBG_SRC) $(BBG_HDR) | bin
	$(CC_ARM) $(CFLAGS_ARM) -o $@ $(BBG_SRC) $(LIBS)
	@echo "[OK] bin/rtp_bbg_arm  (ARM for BBG)"

bin:
	mkdir -p bin

clean:
	rm -rf bin
	@echo "[OK] cleaned"
