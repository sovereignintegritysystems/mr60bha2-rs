# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [0.1.0] - 2026-04-14

### Added

- `mr60bha2-proto`: Zero-allocation frame parser for all 9 known frame types (`no_std`-compatible)
- `mr60bha2d`: Async streaming daemon (UART to Unix socket, aggregated JSON snapshots at ~8 Hz)
- systemd service unit with security hardening
- Cross-compilation support via `cross` (aarch64-unknown-linux-gnu)
