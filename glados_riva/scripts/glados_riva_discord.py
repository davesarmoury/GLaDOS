#!/usr/bin/env python3

import discord
from discord.ext import commands
import os
import re
import num2words
import argparse
import time
import wave
from pathlib import Path
import string
import riva.client
from riva.client.argparse_utils import add_connection_argparse_parameters

TOKEN = os.getenv('DISCORD_TOKEN')
SERVER = os.getenv('DISCORD_SERVER')
CHANNEL = "glados_riva"

save_dir = "/home/davesarmoury/discord/"

intents = discord.Intents.default()
intents.message_content = True
client = commands.Bot(command_prefix='>', intents=intents)

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="A speech synthesis via Riva AI Services. You HAVE TO provide at least one of arguments "
        "`--output`, `--play-audio`, `--list-devices`, `--output-device`.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--voice",
        help="A voice name to use. If this parameter is missing, then the server will try a first available model "
        "based on parameter `--language-code`.",
    )
    parser.add_argument("-o", "--output", type=Path, help="Output file .wav file to write synthesized audio.")
    parser.add_argument(
        "--play-audio",
        action="store_true",
        help="Whether to play input audio simultaneously with transcribing. If `--output-device` is not provided, "
        "then the default output audio device will be used.",
    )
    parser.add_argument("--list-devices", action="store_true", help="List output audio devices indices.")
    parser.add_argument("--output-device", type=int, help="Output device to use.")
    parser.add_argument("--language-code", default='en-US', help="A language of input text.")
    parser.add_argument(
        "--sample-rate-hz", type=int, default=44100, help="Number of audio frames per second in synthesized audio.")
    parser.add_argument(
        "--stream",
        action="store_true",
        help="If this option is set, then streaming synthesis is applied. Streaming means that audio is yielded "
        "as it gets ready. If `--stream` is not set, then a synthesized audio is returned in 1 response only when "
        "all text is processed.",
    )
    parser = add_connection_argparse_parameters(parser)
    args = parser.parse_args()

    if args.output is not None:
        args.output = args.output.expanduser()
    if args.list_devices or args.output_device or args.play_audio:
        import riva.client.audio_io
    return args

def replace_numbers(text):
    return re.sub(r"(\d+)", lambda x: num2words.num2words(int(x.group(0))), text)

@client.event
async def on_message(message):
    if message.author == client.user:
        return

    if message.channel.name != CHANNEL:
        return

    cleaned_msg = message.clean_content
    if client.user in message.mentions:
        for u in message.mentions:
            cleaned_msg = cleaned_msg.replace("@" + u.display_name, "")


        cleaned_msg = cleaned_msg.replace('\n', ' ')          # Remove line-breaks
        cleaned_msg = replace_numbers(cleaned_msg)            # Make digits into text
        cleaned_msg = re.sub(' +', ' ', cleaned_msg)          # Unnecessary white space
        
        if len(cleaned_msg > 400):
            await message.reply("I can't say that.  The current character limit is 400.  Note: numbers are expanded into text")
        else:
            sound_stream = None
            try:
                if args.output_device is not None or args.play_audio:
                    sound_stream = riva.client.audio_io.SoundCallBack(
                        args.output_device, nchannels=nchannels, sampwidth=sampwidth, framerate=args.sample_rate_hz
                    )

                text = cleaned_msg.rstrip().lstrip()
                if len(text) > 0:
                    start = time.time()

                    resp = service.synthesize(text, args.voice, args.language_code, sample_rate_hz=args.sample_rate_hz)
                    stop = time.time()
                    print(cleaned_msg + " ( " + str(stop - start) + "s )")

                    if sound_stream is not None:
                        sound_stream(resp.audio)

                    filename = save_dir + str(message.author.display_name) + "_" + str(start) + "_" + text.translate(str.maketrans('', '', string.punctuation)) + ".wav"
                    filename = filename.replace(" ", "_")
                    out_f = wave.open(filename, 'wb')
                    out_f.setnchannels(nchannels)
                    out_f.setsampwidth(sampwidth)
                    out_f.setframerate(args.sample_rate_hz)
                    out_f.writeframesraw(resp.audio)
                    out_f.close()

            finally:
                if sound_stream is not None:
                    sound_stream.close()

            file = discord.File(filename)
            await message.reply(content=message.clean_content, file=file)

if not os.path.exists(save_dir):
    os.mkdir(save_dir)

args = parse_args()
if args.list_devices:
    riva.client.audio_io.list_output_devices()
    exit()

auth = riva.client.Auth(args.ssl_cert, args.use_ssl, args.server)
service = riva.client.SpeechSynthesisService(auth)
nchannels = 1
sampwidth = 2
sound_stream, out_f = None, None

client.run(TOKEN)