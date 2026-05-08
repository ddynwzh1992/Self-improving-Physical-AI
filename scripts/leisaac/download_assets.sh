#!/bin/bash
# Download LeIsaac kitchen scene assets from LightwheelAI/leisaac releases
# These USD files are too large for git (~63MB total)

set -e

RELEASE_URL="https://github.com/LightwheelAI/leisaac/releases/download/v0.1.0"
ASSET_DIR="${1:-/tmp/leisaac_assets}"

echo "Downloading LeIsaac assets to: $ASSET_DIR"
mkdir -p "$ASSET_DIR"

# Download kitchen scene with oranges (70MB zip -> ~40MB USD + textures)
if [ ! -f "$ASSET_DIR/kitchen_with_orange.zip" ]; then
    echo "Downloading kitchen_with_orange.zip..."
    wget -q --show-progress -O "$ASSET_DIR/kitchen_with_orange.zip" \
        "$RELEASE_URL/kitchen_with_orange.zip"
else
    echo "kitchen_with_orange.zip already exists, skipping."
fi

# Download SO-101 follower robot USD (23MB)
if [ ! -f "$ASSET_DIR/so101_follower.usd" ]; then
    echo "Downloading so101_follower.usd..."
    wget -q --show-progress -O "$ASSET_DIR/so101_follower.usd" \
        "$RELEASE_URL/so101_follower.usd"
else
    echo "so101_follower.usd already exists, skipping."
fi

# Extract kitchen scene
if [ ! -d "$ASSET_DIR/kitchen_with_orange" ]; then
    echo "Extracting kitchen scene..."
    cd "$ASSET_DIR"
    unzip -q kitchen_with_orange.zip
fi

# Fix permissions (needed for Docker container access)
chmod -R a+r "$ASSET_DIR"

echo ""
echo "Assets ready:"
echo "  Kitchen scene: $ASSET_DIR/kitchen_with_orange/scene.usd"
echo "  SO-101 robot:  $ASSET_DIR/so101_follower.usd"
echo ""
echo "Total size:"
du -sh "$ASSET_DIR"
