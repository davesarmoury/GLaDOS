#!/usr/bin/env python3

import requests
from multiprocessing import cpu_count
from multiprocessing.pool import ThreadPool
import shutil
import os

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

blocklist = ["potato", "_ding_"]

audio_dir = 'audio'
if os.path.exists(audio_dir):
    print("Deleting previously downloaded audio")
    shutil.rmtree(audio_dir)

os.mkdir(audio_dir)

def download_file(args):
    url, filename = args[0], args[1]

    try:
        response = requests.get(url)
        open(os.path.join(audio_dir, filename), "wb").write(response.content)
        return filename, True
    except:
        return filename, False

def download_parallel(args):
    cpus = cpu_count()
    results = ThreadPool(cpus - 1).imap_unordered(download_file, args)
    for result in results:
        if result[1]:
            print(bcolors.OKGREEN + "[" + u'\u2713' + "] " + bcolors.ENDC + result[0])
        else:
            print(bcolors.FAIL + "[" + u'\u2715' + "] " + bcolors.ENDC + result[0])

def main():
    global blocklist
    r = requests.get("https://theportalwiki.com/wiki/GLaDOS_voice_lines")

    split_text = r.text.split()
    urls = []
    filenames = []

    for chunk in split_text:
        if "https:" in chunk and ".wav" in chunk:
            url = chunk.replace('"', '').replace("href=", '')
            if url not in urls:
                for s in blocklist:
                    if s in url:
                        break
                else:
                    urls.append(url)
                    filenames.append(url[url.rindex("/")+1:])

    print("Found " + str(len(urls)) + " urls")

    args = zip(urls, filenames)

    download_parallel(args)

main()