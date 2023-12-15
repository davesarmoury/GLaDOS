import discord
from discord.ext import commands
import os
import re
import num2words

TOKEN = os.getenv('DISCORD_TOKEN')
SERVER = os.getenv('DISCORD_SERVER')
CHANNEL = "glados_riva"

intents = discord.Intents.default()
intents.message_content = True
client = commands.Bot(command_prefix='>', intents=intents)

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
        await message.reply(cleaned_msg)
    
client.run(TOKEN)