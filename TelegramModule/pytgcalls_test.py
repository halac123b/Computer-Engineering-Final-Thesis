import os
import asyncio

import pytgcalls

import telethon

# EDIT VALUES
# API_HASH = ''
# API_ID = 28509031

API_HASH = ''
API_ID = 24157867


CHAT_ID = '-4087109494'
INPUT_FILENAME = 'input.raw'
OUTPUT_FILENAME = 'output.raw'

CLIENT_TYPE = pytgcalls.GroupCallFactory.MTPROTO_CLIENT_TYPE.TELETHON

tele_client = telethon.TelegramClient(
    'pytgcalls2',
    API_ID,
    API_HASH
).start()

group_call = pytgcalls.GroupCallFactory(tele_client, CLIENT_TYPE)\
        .get_file_group_call(INPUT_FILENAME, OUTPUT_FILENAME)

@tele_client.on(telethon.events.NewMessage)
async def join_handler(event):
    chat = await event.get_chat()
    await group_call.start(chat.id)

# tele_client.loop.run_until_complete(join_handler())
tele_client.run_until_disconnected()

# from telethon import functions

# async def bruh():
#     async with tele_client as client:
#         result = await client(functions.messages.GetFullChatRequest(
#             chat_id=1656467350
#         ))
#         print("hafa")

# tele_client.loop.run_until_complete(bruh())

# async def main(client):
#     # # its for Pyrogram
#     # await client.start()
#     # while not client.is_connected:
#     #     await asyncio.sleep(1)

#     # you can pass init filenames in the constructor
#     group_call = pytgcalls.GroupCallFactory(client, CLIENT_TYPE)\
#         .get_file_group_call(INPUT_FILENAME, OUTPUT_FILENAME)
#     await group_call.start(CHAT_ID)

#     # to change audio file you can do this:
#     # group_call.input_filename = 'input2.raw'

#     # to change output file:
#     # group_call.output_filename = 'output2.raw'

#     # to restart play from start:
#     # group_call.restart_playout()

#     # to stop play:
#     # group_call.stop_playout()

#     # same with output (recording)
#     # .restart_recording, .stop_output

#     # to mute yourself:
#     # await group_call.set_is_mute(True)

#     # to leave a VC
#     # await group_call.stop()

#     # to rejoin
#     # await group_call.reconnect()

#     # await pyrogram.idle()


# if __name__ == '__main__':
#     tele_client = telethon.TelegramClient(
#         'pytgcalls',
#         API_ID,
#         API_HASH
#     )

#     # set your client (Pyrogram or Telethon)
#     main_client = tele_client

#     #loop = asyncio.get_event_loop()
#     with main_client:
#         main_client.loop.run_until_complete(main(main_client))