#!/usr/bin/env python3
"""Simple MQTT subscriber for testing Uno R4 publishes.

Connects to a local broker and prints any messages published on the
`unoR4/random` topic.
"""

import paho.mqtt.client as mqtt

BROKER = "localhost"  # replace with broker IP if not local
TOPIC = "unoR4/random"


def on_connect(client, userdata, flags, rc):
    """Subscribe to the topic once connected."""
    print(f"Connected with result code {rc}")
    client.subscribe(TOPIC)


def on_message(client, userdata, msg):
    """Print incoming MQTT messages."""
    print(f"{msg.topic}: {msg.payload.decode()}")


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, 1883, 60)
client.loop_forever()

