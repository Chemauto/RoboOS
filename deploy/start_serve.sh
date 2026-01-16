#!/bin/bash

/home/dora/miniforge3/envs/roboOS/bin/python -m vllm.entrypoints.openai.api_server \
  --model /home/dora/.cache/huggingface/hub/models--BAAI--RoboBrain2.0-3B/snapshots/711b343d4d08d00d3def489b3db7008c103852ef \
  --host 0.0.0.0 \
  --port 4567 \
  --served-model-name robobrain \
  --gpu-memory-utilization 0.7 \
  --max-model-len 10000 \
  --max-num-seqs 64 \
  --trust-remote-code \
  --enable-chunked-prefill \
  --enable-auto-tool-choice \
  --tool-call-parser hermes \
  --chat-template /home/dora/RoboOS/deploy/templates/tool_chat_template_hermes.jinja
