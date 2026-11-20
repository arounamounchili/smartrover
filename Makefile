HOST_TARGET  := $(shell rustc -vV | sed -n 's|host: ||p')
BAUD         ?= 115200
FLASH_BEFORE ?= no-reset
PORT         ?=
FLASH_FLAGS  := $(strip $(if $(PORT),--port $(PORT),) --before $(FLASH_BEFORE) -B $(BAUD))
UV           ?= uv
BLENDER      ?= $(or $(shell command -v blender 2>/dev/null),/Applications/Blender.app/Contents/MacOS/Blender)
CAD_PREVIEW  ?= /tmp/smartrover_preview

.PHONY: help fmt fmt-check test dashboard check clippy build build-sta flash flash-sta cad-verify cad-export cad-render ci require-wifi-env

help:
	@echo "SmartRover development targets:"
	@echo "  make fmt         Format Rust code with rustfmt"
	@echo "  make fmt-check   Verify Rust formatting without modifying files"
	@echo "  make test        Run Rust host unit tests and dashboard unit tests"
	@echo "  make dashboard   Run dashboard tests and rebuild dashboard/dashboard.html.gz"
	@echo "  make check       Check ESP32-S3 firmware (SoftAP and STA)"
	@echo "  make clippy      Run clippy on ESP32-S3 firmware and host tests (-D warnings)"
	@echo "  make build       Build release ESP32-S3 firmware"
	@echo "  make build-sta   Build release ESP32-S3 firmware in STA mode (requires WIFI_SSID & WIFI_PASS)"
	@echo "  make flash       Build and flash release firmware via espflash (PORT=..., BAUD=$(BAUD), FLASH_BEFORE=$(FLASH_BEFORE))"
	@echo "  make flash-sta   Build and flash STA firmware (requires WIFI_SSID & WIFI_PASS)"
	@echo "  make cad-verify  Verify parametric 3D CAD geometry and assembly clearances (requires uv)"
	@echo "  make cad-export  Regenerate SmartRover STLs/STEPs into 3d-printing/ (requires uv)"
	@echo "  make cad-render  Render studio-lit 3D previews in Blender (requires Blender & uv)"
	@echo "  make ci          Run full local verification suite"

fmt:
	cargo fmt

fmt-check:
	cargo fmt --check

test:
	cargo test --lib --target $(HOST_TARGET) -Z build-std=std,panic_abort
	cd dashboard && npm test

dashboard:
	cd dashboard && npm test && npm run compress && gzip -dc dashboard.html.gz | diff -u - dist/index.html

check:
	env -u WIFI_SSID -u WIFI_PASS cargo check --release
	WIFI_SSID="CI_Test_SSID" WIFI_PASS="CI_Test_Password" cargo check --release

clippy:
	env -u WIFI_SSID -u WIFI_PASS cargo clippy --release -- -D warnings
	cargo clippy --lib --profile test --target $(HOST_TARGET) -Z build-std=std,panic_abort -- -D warnings

build:
	cargo build --release

require-wifi-env:
	@test -n "$(WIFI_SSID)" && test -n "$(WIFI_PASS)" || (echo "Error: WIFI_SSID and WIFI_PASS must be set" >&2; exit 1)

build-sta: require-wifi-env
	cargo build --release

flash:
	cargo run --release -- $(FLASH_FLAGS)

flash-sta: require-wifi-env flash

cad-verify:
	$(UV) run --locked --project mechanical python mechanical/cad/verify_cad.py

cad-export:
	$(UV) run --locked --project mechanical python mechanical/cad/export_and_render.py

cad-render:
	$(UV) run --locked --project mechanical python mechanical/cad/export_and_render.py --preview-dir $(CAD_PREVIEW)
	"$(BLENDER)" --background --python mechanical/cad/render_blender.py -- $(CAD_PREVIEW) 3d-printing/renders

ci: fmt-check test
	cd dashboard && npm run build && gzip -dc dashboard.html.gz | diff -u - dist/index.html
	$(MAKE) clippy check
	env -u WIFI_SSID -u WIFI_PASS cargo build --release

