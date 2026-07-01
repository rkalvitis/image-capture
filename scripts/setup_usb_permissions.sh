#!/usr/bin/env bash
# Grants passwordless sudo for 'pixi run record' so the Orbbec camera node
# can claim the USB device without prompting for a password every session.
#
# Run once:  sudo bash scripts/setup_usb_permissions.sh
#
# Why root is needed:
#   macOS attaches its kernel UVC driver to the Femto Mega at boot.
#   libusb (used internally by pyorbbecsdk) must detach that driver to open
#   the device — which requires root.  This script adds a narrow sudoers rule
#   so that only the pixi binary on this machine can be run as root without a
#   password, scoped to this specific user.

set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "ERROR: run this script with sudo" >&2
  exit 1
fi

# Resolve the calling (non-root) user so we don't lock the rule to root.
REAL_USER="${SUDO_USER:-$USER}"
if [[ "$REAL_USER" == "root" ]]; then
  echo "ERROR: could not determine the real (non-root) user." >&2
  echo "  Run this script directly (NOT via 'sudo pixi run'):" >&2
  echo "    sudo bash scripts/setup_usb_permissions.sh" >&2
  exit 1
fi

REAL_HOME="$(dscl . -read /Users/"$REAL_USER" NFSHomeDirectory 2>/dev/null | awk '{print $2}')"
[[ -z "$REAL_HOME" ]] && REAL_HOME="/Users/$REAL_USER"

# Find pixi. Check known install locations first and validate with -x — a bare
# `which pixi` in a login shell can print "pixi not found" to stdout, so we must
# not trust its raw output. Fall back to the user's login-shell resolution.
PIXI=""
for c in "$REAL_HOME/.pixi/bin/pixi" /opt/homebrew/bin/pixi /usr/local/bin/pixi; do
  if [[ -x "$c" ]]; then PIXI="$c"; break; fi
done
if [[ -z "$PIXI" ]]; then
  RESOLVED="$(su - "$REAL_USER" -c 'command -v pixi' 2>/dev/null || true)"
  [[ -n "$RESOLVED" && -x "$RESOLVED" ]] && PIXI="$RESOLVED"
fi

if [[ -z "$PIXI" || ! -x "$PIXI" ]]; then
  echo "ERROR: pixi binary not found (looked in $REAL_HOME/.pixi/bin and Homebrew paths)" >&2
  exit 1
fi

RULE_FILE="/etc/sudoers.d/orbbec_camera"
RULE="$REAL_USER ALL=(root) NOPASSWD: $PIXI"

echo "$RULE" | visudo -cf - >/dev/null 2>&1 \
  || { echo "ERROR: sudoers rule failed visudo validation" >&2; exit 1; }

echo "$RULE" > "$RULE_FILE"
chmod 440 "$RULE_FILE"

echo "Done. Installed: $RULE_FILE"
echo "Rule: $RULE"
echo ""
echo "Recording sessions no longer require a password:"
echo "  pixi run record"
