#!/usr/bin/env bash

echo "[make_venv] make the venv"
python -m venv .venv
source .venv/bin/activate

echo "[make_venv] install websockets"
pip install websockets

deactivate 

echo "[make_venv] finished"



