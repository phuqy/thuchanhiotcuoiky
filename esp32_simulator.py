#!/usr/bin/env python3
"""
ESP32 IoT Device Simulator
Simulates an ESP32 device publishing sensor data and receiving commands via MQTT
"""

import json
import time
import random
import threading
from datetime import datetime
import paho.mqtt.client as mqtt

# Configuration
MQTT_BROKER = "broker.hivemq.com"
MQTT_PORT = 1883
TOPIC_NS = "demo/room1"
DEVICE_ID = "esp32_simulator"
FIRMWARE_VERSION = "sim-1.0.0"

# Device state
device_state = {
    "light": "off",
    "fan": "off", 
    "online": True
}

# MQTT client
client = mqtt.Client(client_id=f"{DEVICE_ID}_{int(time.time())}")

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"✅ Connected to MQTT broker: {MQTT_BROKER}")
        
        # Subscribe to command topic
        cmd_topic = f"{TOPIC_NS}/device/cmd"
        client.subscribe(cmd_topic, qos=1)
        print(f"📡 Subscribed to: {cmd_topic}")
        
        # Publish initial online status
        publish_online_status(True)
        
        # Publish initial device state
        publish_device_state()
        
    else:
        print(f"❌ Failed to connect to MQTT broker, code: {rc}")

def on_message(client, userdata, msg):
    try:
        topic = msg.topic
        payload = msg.payload.decode('utf-8')
        print(f"📥 Received [{topic}]: {payload}")
        
        if topic == f"{TOPIC_NS}/device/cmd":
            handle_device_command(payload)
            
    except Exception as e:
        print(f"❌ Error handling message: {e}")

def handle_device_command(payload):
    """Handle device control commands"""
    try:
        cmd = json.loads(payload)
        state_changed = False
        
        # Handle light command
        if "light" in cmd:
            light_cmd = cmd["light"]
            if light_cmd == "on":
                device_state["light"] = "on"
                state_changed = True
            elif light_cmd == "off":
                device_state["light"] = "off"
                state_changed = True
            elif light_cmd == "toggle":
                device_state["light"] = "off" if device_state["light"] == "on" else "on"
                state_changed = True
            
            print(f"💡 Light: {device_state['light'].upper()}")
        
        # Handle fan command
        if "fan" in cmd:
            fan_cmd = cmd["fan"]
            if fan_cmd == "on":
                device_state["fan"] = "on"
                state_changed = True
            elif fan_cmd == "off":
                device_state["fan"] = "off"
                state_changed = True
            elif fan_cmd == "toggle":
                device_state["fan"] = "off" if device_state["fan"] == "on" else "on"
                state_changed = True
            
            print(f"🌀 Fan: {device_state['fan'].upper()}")
        
        # Publish updated device state immediately
        if state_changed:
            publish_device_state()
            
    except json.JSONDecodeError as e:
        print(f"❌ Invalid JSON command: {e}")

def publish_sensor_data():
    """Publish simulated sensor data"""
    topic = f"{TOPIC_NS}/sensor/state"
    
    # Generate fake sensor readings
    temp_c = round(20.0 + random.uniform(-3, 8), 1)  # 17-28°C
    hum_pct = round(50.0 + random.uniform(-15, 25), 1)  # 35-75%
    lux = random.randint(50, 300)  # 50-300 lux
    
    data = {
        "ts": int(time.time()),
        "temp_c": temp_c,
        "hum_pct": hum_pct,
        "lux": lux
    }
    
    payload = json.dumps(data)
    result = client.publish(topic, payload, qos=0)
    
    if result.rc == mqtt.MQTT_ERR_SUCCESS:
        print(f"🌡️  Sensor: {temp_c}°C, {hum_pct}%, {lux}lux")
    else:
        print(f"❌ Failed to publish sensor data")

def publish_device_state():
    """Publish device state (retained)"""
    topic = f"{TOPIC_NS}/device/state"
    
    # Simulate WiFi RSSI
    rssi = random.randint(-70, -40)  # -70 to -40 dBm
    
    data = {
        "ts": int(time.time()),
        "light": device_state["light"],
        "fan": device_state["fan"],
        "rssi": rssi,
        "fw": FIRMWARE_VERSION
    }
    
    payload = json.dumps(data)
    result = client.publish(topic, payload, qos=1, retain=True)
    
    if result.rc == mqtt.MQTT_ERR_SUCCESS:
        print(f"📊 Device state: Light={device_state['light']}, Fan={device_state['fan']}, RSSI={rssi}dBm")
    else:
        print(f"❌ Failed to publish device state")

def publish_online_status(online):
    """Publish online status (retained)"""
    topic = f"{TOPIC_NS}/sys/online"
    
    data = {"online": online}
    payload = json.dumps(data)
    result = client.publish(topic, payload, qos=1, retain=True)
    
    if result.rc == mqtt.MQTT_ERR_SUCCESS:
        print(f"🟢 Online status: {online}")
    else:
        print(f"❌ Failed to publish online status")

def sensor_publisher():
    """Background thread to publish sensor data every 3 seconds"""
    while True:
        if client.is_connected():
            publish_sensor_data()
        time.sleep(3)

def heartbeat_publisher():
    """Background thread to publish device state every 15 seconds"""
    while True:
        if client.is_connected():
            publish_device_state()
        time.sleep(15)

def main():
    print("🚀 ESP32 IoT Device Simulator Starting...")
    print(f"📡 MQTT Broker: {MQTT_BROKER}:{MQTT_PORT}")
    print(f"🏠 Topic Namespace: {TOPIC_NS}")
    print(f"🆔 Device ID: {DEVICE_ID}")
    print("─" * 50)
    
    # Setup MQTT callbacks
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Set Last Will Testament
    lwt_topic = f"{TOPIC_NS}/sys/online"
    lwt_payload = json.dumps({"online": False})
    client.will_set(lwt_topic, lwt_payload, qos=1, retain=True)
    
    try:
        # Connect to broker
        print(f"🔄 Connecting to {MQTT_BROKER}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        
        # Start background threads
        sensor_thread = threading.Thread(target=sensor_publisher, daemon=True)
        heartbeat_thread = threading.Thread(target=heartbeat_publisher, daemon=True)
        
        sensor_thread.start()
        heartbeat_thread.start()
        
        # Start MQTT loop
        client.loop_start()
        
        print("✅ Simulator running! Press Ctrl+C to stop")
        print("─" * 50)
        
        # Keep main thread alive
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n🛑 Shutting down simulator...")
        
        # Publish offline status
        publish_online_status(False)
        time.sleep(1)  # Wait for message to be sent
        
        client.loop_stop()
        client.disconnect()
        print("👋 Goodbye!")

if __name__ == "__main__":
    main()