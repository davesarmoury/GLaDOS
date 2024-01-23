# GLaDOS

## LLM

    git clone --depth=1 https://github.com/dusty-nv/jetson-containers
    cd jetson-containers
    ./run.sh --workdir=/opt/llama.cpp/bin $(./autotag llama_cpp) /bin/bash -c  './server --model $(huggingface-downloader TheBloke/openchat_3.5-GGUF/openchat_3.5.Q4_K_S.gguf) --n-gpu-layers 999 --threads $(nproc) -c 2048'

## RIVA

Look here [https://docs.nvidia.com/deeplearning/riva/user-guide/docs/quick-start-guide.html](https://docs.nvidia.com/deeplearning/riva/user-guide/docs/quick-start-guide.html)

    ngc registry resource download-version nvidia/riva/riva_quickstart_arm64:2.13.1
    cd riva_quickstart_arm64_v2.13.1
    bash riva_init.sh

    # See NeMo information for what to do here

    riva_start.sh

## ASR

Check device number

    python3 transcribe_mic.py --input-device=4

## TTS

Pair bluetooth speaker
See [https://developer.nvidia.com/embedded/learn/tutorials/connecting-bluetooth-audio](https://developer.nvidia.com/embedded/learn/tutorials/connecting-bluetooth-audio)

    python3 glados_talk.py --voice=GLaDOS --play-audio

## Everything Else

    roslaunch glados_bringup bringup.launch
