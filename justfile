set dotenv-load

target := env_var_or_default("TARGET", "aarch64-unknown-linux-gnu")
host := env_var_or_default("DEPLOY_HOST", "root@raspberrypi.local")

# Build release for target architecture
build:
    cross build --target {{target}} --release

# Run tests
test:
    cargo test --all

# Clippy lint
lint:
    cargo clippy --all-targets -- -D warnings

# Format check
fmt:
    cargo fmt --check

# Deploy binary + config to remote host via scp
deploy: build
    scp target/{{target}}/release/mr60bha2d {{host}}:/usr/local/bin/
    scp config/mr60bha2d.toml {{host}}:/etc/mr60bha2d.toml
    scp deploy/mr60bha2d.service {{host}}:/etc/systemd/system/
    scp deploy/mr60bha2d.tmpfiles {{host}}:/etc/tmpfiles.d/mr60bha2d.conf
    ssh {{host}} "systemd-tmpfiles --create && systemctl daemon-reload && systemctl enable --now mr60bha2d"

# Stream radar data from remote host (for testing)
stream:
    ssh {{host}} "socat - UNIX-CONNECT:/run/mr60bha2/radar.sock"
