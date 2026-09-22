#!/usr/bin/env bash
source "$HOME/miniforge3/etc/profile.d/conda.sh"
conda activate lerobot
export HF_HOME="$HOME/project/lerobot/data/hf"
cd "$HOME/project/lerobot"
