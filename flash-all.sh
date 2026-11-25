#!/usr/bin/env bash
set -uo pipefail

# Baked into the "Firmware revision" sensor so a stale speaker is visible.
FW_REV="$(git rev-parse --short=7 HEAD)$(git diff --quiet HEAD -- || echo -dirty)"

run() {
  local tag="$1" yaml="$2"
  esphome -s fw_rev "$FW_REV" run "$yaml" --no-logs 2>&1 | sed "s/^/[$tag] /"
  return "${PIPESTATUS[0]}"
}

run vpe  home-assistant-voice.va-direct.yaml & vpe_pid=$!
run atom atom-echo-s3r.va-direct.yaml       & atom_pid=$!

wait "$vpe_pid";  vpe_rc=$?
wait "$atom_pid"; atom_rc=$?

status() { [ "$1" -eq 0 ] && echo "✓" || echo "✗"; }

echo
echo "$(status "$vpe_rc") vpe (home-assistant-voice.va-direct.yaml)"
echo "$(status "$atom_rc") atom (atom-echo-s3r.va-direct.yaml)"

[ "$vpe_rc" -eq 0 ] && [ "$atom_rc" -eq 0 ]
