import os
import time

from ntgcalls import InputMode
from telethon import TelegramClient

from pytgcalls import idle
from pytgcalls import PyTgCalls
from pytgcalls.types.input_stream import AudioStream
from pytgcalls.types.input_stream import Stream

# Login information
app = TelegramClient(
    'pytgcalls2',   # Session name
    api_id=24157867,
    api_hash='',
)

call_py = PyTgCalls(app)
call_py.start()
file = './input.raw'
while not os.path.exists(file):
    time.sleep(0.125)
call_py.join_group_call(
    4087109494, # Chat ID of group chat
    Stream(
        AudioStream(
            input_mode=InputMode.File,
            path=file,
        ),
    ),
)
idle()