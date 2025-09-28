#!/usr/bin/env python3
"""
Manual MQTT Test Client
Send manual commands to test the IoT system
"""

import json
import paho.mqtt.client as mqtt

# Configuration
MQTT_BROKER = "broker.hivemq.com"
MQTT_PORT = 1883
TOPIC_NS = "demo/room1"

def send_command(command_dict):
    """Send a single command to the device"""
    client = mqtt.Client()
    
    try:
        print(f"🔄 Connecting to {MQTT_BROKER}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        
        topic = f"{TOPIC_NS}/device/cmd"
        payload = json.dumps(command_dict)
        
        result = client.publish(topic, payload, qos=1)
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print(f"✅ Sent command: {payload}")
        else:
            print(f"❌ Failed to send command")
            
        client.disconnect()
        
    except Exception as e:
        print(f"❌ Error: {e}")

def main():
    print("🎛️ Manual MQTT Command Tester")
    print("=" * 40)
    
    commands = [
        {"light": "toggle"},
        {"fan": "on"},
        {"light": "off", "fan": "toggle"},
        {"light": "on"},
        {"fan": "off"}
    ]
    
    for i, cmd in enumerate(commands, 1):
        print(f"\n{i}. Sending: {json.dumps(cmd)}")
        send_command(cmd)
        
        input("Press Enter to continue...")
    
    print("\n✅ All test commands sent!")

if __name__ == "__main__":
    main()