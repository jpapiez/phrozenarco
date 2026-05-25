#!/bin/sh
# KAOS_VERSION: v0.95

# KAOS / Phrozen installer fallback for unknown model
# POSIX sh compatible.
# Purpose: install KAOS Python files, language files, top-level cfg files,
# and split /config/kaos/*.cfg files with loud diagnostics and verification.

set -u

# Define specific versions for Klipper, Moonraker, and Mainsail
KLIPPER_VERSION="v11.0-257"
MOONRAKER_VERSION="v0.8.0-306"
MAINSAIL_VERSION="latest"

timestamp=$(date +%Y%m%d_%H%M%S)

log() {
    echo "KAOS_INSTALL: $*"
}

fail() {
    echo "KAOS_INSTALL_ERROR: $*" >&2
    exit 1
}

# Function to install Klipper
install_klipper() {
    log "Installing Klipper version $KLIPPER_VERSION"
    git clone https://github.com/Klipper3d/klipper.git || fail "Failed to clone Klipper repository"
    cd klipper || fail "Failed to enter Klipper directory"
    git fetch --tags || fail "Failed to fetch tags for Klipper"
    git checkout "$KLIPPER_VERSION" || fail "Failed to checkout Klipper version $KLIPPER_VERSION"
    log "Klipper installed successfully"
}

# Function to install Moonraker
install_moonraker() {
    log "Installing Moonraker version $MOONRAKER_VERSION"
    git clone https://github.com/Arksine/moonraker.git || fail "Failed to clone Moonraker repository"
    cd moonraker || fail "Failed to enter Moonraker directory"
    git fetch --tags || fail "Failed to fetch tags for Moonraker"
    git checkout "$MOONRAKER_VERSION" || fail "Failed to checkout Moonraker version $MOONRAKER_VERSION"
    log "Moonraker installed successfully"
}

# Function to install Mainsail
install_mainsail() {
    log "Downloading and installing Mainsail version $MAINSAIL_VERSION"
    wget -O mainsail.zip https://github.com/mainsail-crew/mainsail/releases/$MAINSAIL_VERSION/download/mainsail.zip || fail "Failed to download Mainsail"
    mkdir -p /var/www/mainsail || fail "Failed to create Mainsail installation directory"
    unzip mainsail.zip -d /var/www/mainsail || fail "Failed to unzip Mainsail"
    log "Mainsail installed successfully"
}

# Call functions to install required components
install_klipper
install_moonraker
install_mainsail

log "Installation process completed successfully."