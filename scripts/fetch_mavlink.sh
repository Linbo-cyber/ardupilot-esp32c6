#!/bin/bash
# Fetch MAVLink C headers (common dialect)
set -e
MAVLINK_DIR="main/mavlink"
if [ -d "$MAVLINK_DIR/common" ]; then
    echo "MAVLink headers already exist"
    exit 0
fi
echo "Downloading MAVLink C headers..."
mkdir -p "$MAVLINK_DIR"
TMPDIR=$(mktemp -d)
curl -sL "https://github.com/mavlink/c_library_v2/archive/refs/heads/master.tar.gz" -o "$TMPDIR/mavlink.tar.gz"
tar xzf "$TMPDIR/mavlink.tar.gz" -C "$TMPDIR"
cp -r "$TMPDIR/c_library_v2-master/common" "$MAVLINK_DIR/"
cp "$TMPDIR/c_library_v2-master/mavlink_types.h" "$MAVLINK_DIR/"
cp "$TMPDIR/c_library_v2-master/mavlink_helpers.h" "$MAVLINK_DIR/"
cp "$TMPDIR/c_library_v2-master/mavlink_get_info.h" "$MAVLINK_DIR/"
cp "$TMPDIR/c_library_v2-master/mavlink_sha256.h" "$MAVLINK_DIR/"
cp "$TMPDIR/c_library_v2-master/mavlink_conversions.h" "$MAVLINK_DIR/"
cp "$TMPDIR/c_library_v2-master/protocol.h" "$MAVLINK_DIR/"
cp "$TMPDIR/c_library_v2-master/checksum.h" "$MAVLINK_DIR/"
rm -rf "$TMPDIR"
echo "MAVLink headers installed to $MAVLINK_DIR"
