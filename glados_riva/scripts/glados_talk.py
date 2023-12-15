#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

import argparse
import time
import wave
from pathlib import Path
import string
import riva.client
from riva.client.argparse_utils import add_connection_argparse_parameters
import rospy
from std_msgs.msg import String

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
    if args.output is None and not args.play_audio and args.output_device is None and not args.list_devices:
        parser.error(
            f"You have to provide at least one of arguments: `--play-audio`, `--output-device`, `--output`, "
            f"`--list-devices`."
        )
    if args.output is not None:
        args.output = args.output.expanduser()
    if args.list_devices or args.output_device or args.play_audio:
        import riva.client.audio_io
    return args

def callback(msg):
    global TTS
    TTS = msg.data

def main() -> None:
    global TTS
    TTS = None

    args = parse_args()
    if args.list_devices:
        riva.client.audio_io.list_output_devices()
        return

    rospy.init_node('riva_tts', anonymous=True)
    rospy.Subscriber("speak", String, callback)

    auth = riva.client.Auth(args.ssl_cert, args.use_ssl, args.server)
    service = riva.client.SpeechSynthesisService(auth)
    nchannels = 1
    sampwidth = 2
    sound_stream, out_f = None, None
    try:
        if args.output_device is not None or args.play_audio:
            sound_stream = riva.client.audio_io.SoundCallBack(
                args.output_device, nchannels=nchannels, sampwidth=sampwidth, framerate=args.sample_rate_hz
            )

        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if TTS != None:
                text_list = [TTS]
                
                if len(TTS) >= 400:
                    text_list = TTS.split(".")
                
                for t in text_list:
                    text = t.rstrip().lstrip()
                    if len(text) < 1:
                        continue

                    rospy.loginfo("Generating audio for request...")
                    rospy.loginfo("< " + text + " >")
                    start = time.time()

                    resp = service.synthesize(text, args.voice, args.language_code, sample_rate_hz=args.sample_rate_hz)
                    stop = time.time()
                    rospy.loginfo("Time spent: " + str(stop - start) + "s")

                    if sound_stream is not None:
                        sound_stream(resp.audio)

                    filename = "/home/davesarmoury/" + str(start) + "_" + text.translate(str.maketrans('', '', string.punctuation)) + ".wav"
                    filename = filename.replace(" ", "_")
                    out_f = wave.open(filename, 'wb')
                    out_f.setnchannels(nchannels)
                    out_f.setsampwidth(sampwidth)
                    out_f.setframerate(args.sample_rate_hz)
                    out_f.writeframesraw(resp.audio)
                    out_f.close()

                TTS = None

            rate.sleep()
    finally:
        if sound_stream is not None:
            sound_stream.close()


if __name__ == '__main__':
    main()
