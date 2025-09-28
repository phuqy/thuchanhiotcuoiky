#!/usr/bin/env python3
"""
Test MQTT Command Script
Test sending commands to ESP32 simulator
"""

import paho.mqtt.client as mqtt
import json
import time

# MQTT Configuration
BROKER_HOST = "broker.hivemq.com"
BROKER_PORT = 1883
TOPIC_NAMESPACE = "demo/room1"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("✅ Connected to MQTT broker")
        # Send test command to toggle light
        command = {"light": "toggle"}
        topic = f"{TOPIC_NAMESPACE}/device/cmd"
        payload = json.dumps(command)
        
        print(f"🔄 Sending command: {payload}")
        client.publish(topic, payload)
        
        time.sleep(2)
        
        # Send another command to toggle fan
        command = {"fan": "toggle"}
        payload = json.dumps(command)
        print(f"🔄 Sending command: {payload}")
        client.publish(topic, payload)
        
        # Disconnect after sending
        time.sleep(1)
        client.disconnect()
    else:
        print(f"❌ Failed to connect: {rc}")

def on_publish(client, userdata, mid):
    print(f"✅ Command published (message ID: {mid})")

if __name__ == "__main__":
    print("🚀 MQTT Command Test Starting...")
    
    # Create MQTT client
    client = mqtt.Client(client_id=f"test_commander_{int(time.time())}")
    client.on_connect = on_connect
    client.on_publish = on_publish
    
    # Connect and run
    print(f"🔄 Connecting to {BROKER_HOST}...")
    client.connect(BROKER_HOST, BROKER_PORT, 60)
    client.loop_forever()
    
    print("✅ Test completed!")