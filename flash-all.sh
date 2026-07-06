#!/usr/bin/env bash
set -uo pipefail

run() {
  local tag="$1" yaml="$2"
  esphome run "$yaml" --no-logs 2>&1 | sed "s/^/[$tag] /"
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
