from telethon import TelegramClient

from pytgcalls import idle
from pytgcalls import PyTgCalls
from pytgcalls.media_devices import MediaDevices
from pytgcalls.types import CaptureAudioDevice

app = TelegramClient(
    "pytgcalls2",  # Session name
    api_id=24157867,
    api_hash="690948a57648db1f77ce473cc303db38",
)

call_py = PyTgCalls(app)
call_py.start()
call_py.join_group_call(
    4087109494,
    CaptureAudioDevice(
        MediaDevices.get_audio_devices()[2],
    ),
)
idle()

# print(MediaDevices.get_audio_devices())
