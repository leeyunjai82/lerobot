#!/usr/bin/env bash
source "$HOME/miniforge3/etc/profile.d/conda.sh"
conda activate arm-lab
export HF_HOME="$HOME/project/arm-lab/data/hf"
cd "$HOME/project/arm-lab"
