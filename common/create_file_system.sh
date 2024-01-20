#!/bin/sh

# Configure file system paths
ROOT="/var/aoi"

mkdir -p $ROOT
mkdir -p "$ROOT/mysql"

# Simulation assets
mkdir -p "$ROOT/assets"
mkdir -p "$ROOT/assets/images" 

# Production templates
mkdir -p "$ROOT/templates"
mkdir -p "$ROOT/templates/images"

cp -r ./common/simulation_assets/ "$ROOT/assets/images"
cp ./common/templates/index.json "$ROOT/index.json"
touch "$ROOT/index-lock.json"