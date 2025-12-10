#!/usr/bin/env bash


yes | bash scripts/update_submodule.sh
rm -- scripts/update_submodule.sh


exec ./launch_chffrplus.sh
