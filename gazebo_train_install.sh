#!/bin/bash

# Gazebo Train Installation Script for ROS 2
# Author: Luigi Ferraioli 
# Usage: ./gazebo_train_install.sh

#set -e  # Stop script on first error

echo "🚂 Starting Gazebo Train simulation setup..."

# Step 1: Update package lists
#echo "Updating system packages..."
#sudo apt update

# Step 2: Install dependencies from requirements.txt
if [ ! -f "requirements.txt" ]; then
     echo "requirements.txt not found in the current directory."
fi

echo "Installing packages from requirements.txt..."
while IFS= read -r package || [[ -n "$package" ]]; do
    # Skip comments and empty lines
    [[ "$package" =~ ^#.*$ || -z "$package" ]] && continue
    echo "➡️ Installing $package..."
    sudo apt install -y "$package"
done < requirements.txt

# Step 3: Build the ROS 2 workspace
echo "Building ROS 2 workspace..."
colcon build --symlink-install

# Step 4: Source the environment
echo "Sourcing setup.bash..."
source install/setup.bash

echo "✅ Installation complete! You can now run your Gazebo train simulation."
