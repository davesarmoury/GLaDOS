#!/usr/bin/env python3

import requests
from tqdm import tqdm
import shutil
import os

blocklist = ["potato", "_ding_"]

audio_dir = 'audio'
if os.path.exists(audio_dir):
    print("Deleting previously downloaded audio")
    shutil.rmtree(audio_dir)

os.mkdir(audio_dir)

r = requests.get("https://theportalwiki.com/wiki/GLaDOS_voice_lines")

split_text = r.text.split()
urls = []

for chunk in split_text:
    if "https:" in chunk and ".wav" in chunk:
        url = chunk.replace('"', '').replace("href=", '')
        if url not in urls:
            for s in blocklist:
                if s in url:
                    break
            else:
                urls.append(url)

print("Found " + str(len(urls)) + " urls")

for url in tqdm(urls):
    filename = url[url.rindex("/")+1:]

    response = requests.get(url)
    open(os.path.join(audio_dir, filename), "wb").write(response.content)
