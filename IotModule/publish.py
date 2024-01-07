# Publishes an incrementing value to a feed

import os
import time
import dotenv

# Import Adafruit IO REST client.
from Adafruit_IO import Client, Feed

# holds the count for the feed
run_count = 0

dotenv.load_dotenv()

# Set to your Adafruit IO key and username
ADAFRUIT_IO_KEY = os.getenv("ADAFRUIT_IO_KEY")
ADAFRUIT_IO_USERNAME = os.getenv("ADAFRUIT_IO_USERNAME")

# Create an instance of the REST client.
aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

# Create a new feed named 'counter'
# New feed must have unique name, not exist in your account already
# feed = Feed(name="counter")
# response = aio.create_feed(feed)

while True:
    print("sending count: ", run_count)
    run_count += 1
    send_data = f"LIVING ROOM {run_count}"
    # Publish data to feed
    aio.send_data("buzzer", send_data)
    # Adafruit IO is rate-limited for publishing
    # so we'll need a delay for calls to aio.send_data()
    time.sleep(3)
