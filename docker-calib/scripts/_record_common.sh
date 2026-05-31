#!/usr/bin/env bash
# Shared helpers for the ROS-bag record scripts. Source this, don't run it.
#
#   source "$(dirname "${BASH_SOURCE[0]}")/_record_common.sh"
#
# The empty-bag failure mode (see git history): record was started while the
# sensor/camera publishers were not up, so the bag captured 0 messages and only
# surfaced as a 5 KB dead file at Kalibr time. These helpers make that fail
# loudly and early instead.

# wait_for_topic <topic> [timeout_s]
#   Returns 0 if at least one message arrives within timeout, non-zero otherwise.
#   Subscribes BEST_EFFORT/VOLATILE, which is QoS-compatible with both
#   best-effort sensor publishers (gscam2, rslidar_sdk) and reliable ones, so it
#   won't false-negative on a QoS mismatch.
wait_for_topic() {
    local topic="$1" t="${2:-10}"
    timeout "$t" ros2 topic echo --once \
        --qos-reliability best_effort --qos-durability volatile \
        "$topic" >/dev/null 2>&1
}

# preflight_topics <timeout_s> <topic> [topic...]
#   Checks each topic in turn; prints ok / NO DATA. Returns non-zero if any
#   topic produced no data.
preflight_topics() {
    local t="$1"; shift
    local missing=0 topic
    for topic in "$@"; do
        printf '  %-26s ' "$topic"
        if wait_for_topic "$topic" "$t"; then
            echo "ok"
        else
            echo "NO DATA"
            missing=1
        fi
    done
    return $missing
}

# assert_bag_nonempty <bag_dir>
#   Warns loudly and returns non-zero if the recorded bag has 0 messages.
assert_bag_nonempty() {
    local dir="$1" n
    n=$(ros2 bag info "$dir" 2>/dev/null \
        | grep -oE 'Messages:[[:space:]]*[0-9]+' | grep -oE '[0-9]+' | head -1)
    if [ -z "$n" ] || [ "$n" -eq 0 ]; then
        echo "!!! WARNING: '$dir' recorded 0 messages -- the bag is EMPTY." >&2
        echo "!!! The sensors/camera were not publishing. Nothing to calibrate." >&2
        return 1
    fi
    echo "Recorded $n messages -> $dir"
}
