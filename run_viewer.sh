#!/bin/bash
./build/viewer/flux-viewer --connect ws://127.0.0.1:8765/telemetry &
disown
