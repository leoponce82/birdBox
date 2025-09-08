#!/usr/bin/env python3
"""Simple MQTT subscriber for testing Uno R4 publishes.

Connects to a local broker and prints any messages published on the
`unoR4/random` topic.
"""

import json
import paho.mqtt.client as mqtt

BROKER = "10.60.245.204"  # replace with broker IP if not local
TOPIC = "birdBox/unoR4/sensors"


def on_connect(client, userdata, flags, rc):
    """Subscribe to the topic once connected."""
    print(f"Connected with result code {rc}")
    client.subscribe(TOPIC)


def on_message(client, userdata, msg):
    """Print incoming MQTT messages as JSON."""
    payload = json.loads(msg.payload.decode())
    print(f"{msg.topic}: {payload}")


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, 1883, 60)
client.loop_forever()

