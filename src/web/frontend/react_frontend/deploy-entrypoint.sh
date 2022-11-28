#!/bin/bash
echo "start skript"
cd  /workspace/src
npm install -g serve
npm ci
npm run build
serve -s build