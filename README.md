# GLaDOS

## LLM

    git clone --depth=1 https://github.com/dusty-nv/jetson-containers
    cd jetson-containers
    ./run.sh --workdir=/opt/llama.cpp/bin $(./autotag llama_cpp) /bin/bash -c  './server --model $(huggingface-downloader TheBloke/openchat_3.5-GGUF/openchat_3.5.Q4_K_S.gguf) --n-gpu-layers 999 --threads $(nproc) -c 2048'

## Voice

Look here [https://docs.nvidia.com/deeplearning/riva/user-guide/docs/quick-start-guide.html](https://docs.nvidia.com/deeplearning/riva/user-guide/docs/quick-start-guide.html)

### Training Your Own Voice

Install Jupyter Notebook

    pip install notebook

Run jupyter notebook and open [Train_NeMo.ipynb](https://github.com/davesarmoury/GLaDOS/blob/main/TrainNemo/Train_NeMo.ipynb).  Uncommenting the install lines in the first cell *should* install everything needed, but that isn't guaranteed.  Run one cell at a time until everything is trained.  This should be run on a computer with a decent GPU, not a Jetson. 

### Setup NGC

You will first need to setup [NGC](https://org.ngc.nvidia.com/setup).  Setup the CLI and also login to nvcr.io in Docker

### Using the GLaDOS Voice from Python

You can use the models directly from Python.  

### Using the GLaDOS Voice with RIVA

#### Riva Files

You can download the Riva files from [HERE](https://huggingface.co/DavesArmoury/GLaDOS_TTS) and skip this step, or generate your own using the commands below.  These commands need to be don on the computer that trained the models, not the computer that will be running them (ie. Jetson).  **Note**: All of the filepaths here are for my computer.  Just create empty folders to start.  

    pip3 install whl
    pip3 install nemo2riva==2.13.1

    nemo2riva --out hifigan.riva hifigan.nemo --key tlt_encode
    nemo2riva --out fastpitch.riva fastpitch.nemo --key tlt_encode

#### Deploy Models

Transfer the .riva files from the step above (or you downloaded) to whatever machine will be running the voice.  These commands are run on the Riva machine.  **Note**: All of the filepaths here are for my computer.  Just create empty folders to start.  

    docker run --gpus all -it --rm \
        -v /home/davesarmoury/RIVA/artifacts:/servicemaker-dev \
        -v /home/davesarmoury/RIVA/riva_repo:/data \
        --entrypoint="/bin/bash" \
        nvcr.io/nvidia/riva/riva-speech:2.13.1-servicemaker-l4t-aarch64

    riva-build speech_synthesis \
        /servicemaker-dev/glados.rmir:tlt_encode \
        /servicemaker-dev/glados_fastpitch.riva:tlt_encode \
        /servicemaker-dev/glados_hifigan.riva:tlt_encode \
        --voice_name=GLaDOS \
        --sample_rate 22050

    riva-deploy /servicemaker-dev/glados.rmir:tlt_encode /data/models

    exit

These commands will create a bunch of folders inside the "artifacts" directory.

#### Riva Setup

Then, get the Quickstart for Riva.  This example is for Jetson (arm64), but will change for different architectures

    ngc registry resource download-version nvidia/riva/riva_quickstart_arm64:2.13.1
    cd riva_quickstart_arm64_v2.13.1

Edit the config.sh file and change *service_enabled_nlp* and *service_enabled_nmt* to false.  This isn't necessary, but will speed things up if you aren't using them.  Then, download all of the models with the command below

    bash riva_init.sh

Take the folders that were created in the previous step and copy them into the *riva_quickstart_arm64_v2.13.1/model_repository/models*.  That should be all you need to setup Riva.  Run it with the command below

    riva_start.sh

#### Using Riva TTS

To verify that everything is working properly, use the riva python clients.  Follow the install instructions [HERE](https://github.com/nvidia-riva/python-clients?tab=readme-ov-file#installation)

Once that is setup, run the tts.py file under riva/client.  Pass in "--voice GLaDOS" as parameters and whatever string you want her to say.  You can choose to save the audio as a file, or play it directly.

## ASR

Check device number

    python3 transcribe_mic.py --input-device=4

## TTS

Pair bluetooth speaker
See [https://developer.nvidia.com/embedded/learn/tutorials/connecting-bluetooth-audio](https://developer.nvidia.com/embedded/learn/tutorials/connecting-bluetooth-audio)

    python3 glados_talk.py --voice=GLaDOS --play-audio

## Everything Else

    roslaunch glados_bringup bringup.launch
