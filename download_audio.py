#!/usr/bin/env python3

import requests
from multiprocessing import cpu_count
from multiprocessing.pool import ThreadPool
import shutil
import os
from bs4 import BeautifulSoup
import soundfile as sf
import string
import json

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

blocklist = ["potato", "_ding_", "00_part1_entry-6"]
audio_dir = 'audio'
download_threads = 64

def prep():
    if os.path.exists(audio_dir):
        print("Deleting previously downloaded audio")
        shutil.rmtree(audio_dir)

    os.mkdir(audio_dir)

def remove_punctuation(str):
    return str.translate(str.maketrans('', '', string.punctuation))
    
def audio_duration(fn):
    f = sf.SoundFile(fn)
    return f.frames / f.samplerate

def download_file(args):
    url, filename = args[0], args[1]

    try:
        response = requests.get(url)
        open(os.path.join(audio_dir, filename), "wb").write(response.content)
        return filename, True
    except:
        return filename, False

def download_parallel(args):
    results = ThreadPool(download_threads).imap_unordered(download_file, args)
    for result in results:
        if result[1]:
            print(bcolors.OKGREEN + "[" + u'\u2713' + "] " + bcolors.ENDC + result[0])
        else:
            print(bcolors.FAIL + "[" + u'\u2715' + "] " + bcolors.ENDC + result[0])

def main():
    r = requests.get("https://theportalwiki.com/wiki/GLaDOS_voice_lines")

    urls = []
    filenames = []
    texts = []

    soup = BeautifulSoup(r.text, 'html.parser')
    for link_item in soup.find_all('a'):
        url = link_item.get("href", None)
        if url:
            if "https:" in url and ".wav" in url:
                list_item = link_item.find_parent("li")
                ital_item = list_item.find_all('i')
                if ital_item:
                    text = ital_item[0].text
                    text = text.replace('"', '')
                    filename = url[url.rindex("/")+1:]

                    if "[" not in text and "]" not in text:
                        if url not in urls:
                            for s in blocklist:
                                if s in url:
                                    break
                            else:
                                urls.append(url)
                                filenames.append(filename)
                                texts.append(text)

    print("Found " + str(len(urls)) + " urls")

    args = zip(urls, filenames)

    prep()
    download_parallel(args)

    #{"audio_filepath": "audio/nada_lily_21_haggard_0316.wav", 
    #"text": "awake ye kings", 
    #"duration": 1.3, 
    #"text_no_preprocessing": "\u201cAwake, ye kings,\u201d", 
    #"text_normalized": "\"Awake, ye kings,\""}
    outFile=open(os.path.join(audio_dir, "manifest.json"), 'w')
    for i in range(len(urls)):
        item = {}
        text = texts[i]
        filename = filenames[i]
        item["audio_filepath"] = filename
        item["text_normalized"] = text
        item["text_no_preprocessing"] = text
        item["text"] = text.lower()
        item["duration"] = audio_duration(os.path.join(audio_dir, filename))
        outFile.write(json.dumps(item, ensure_ascii=True, sort_keys=True) + "\n")
 
    outFile.close()

main()
