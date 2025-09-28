#!/usr/bin/env python3
"""
Flutter App Simulator - IoT Device Controller
Simulates the Flutter mobile app for controlling IoT devices
"""

import json
import time
import paho.mqtt.client as mqtt

# Configuration
MQTT_BROKER = "broker.hivemq.com"
MQTT_PORT = 1883
TOPIC_NS = "demo/room1"
CLIENT_ID = f"flutter_simulator_{int(time.time())}"

# Global state
device_state = {
    "light": "unknown",
    "fan": "unknown",
    "rssi": 0,
    "fw": "unknown",
    "online": False
}

client = mqtt.Client(client_id=CLIENT_ID)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"✅ Connected to MQTT broker: {MQTT_BROKER}")
        
        # Subscribe to device state and online status
        device_topic = f"{TOPIC_NS}/device/state"
        online_topic = f"{TOPIC_NS}/sys/online"
        
        client.subscribe(device_topic, qos=1)
        client.subscribe(online_topic, qos=1)
        
        print(f"📡 Subscribed to: {device_topic}")
        print(f"📡 Subscribed to: {online_topic}")
        
    else:
        print(f"❌ Failed to connect to MQTT broker, code: {rc}")

def on_message(client, userdata, msg):
    try:
        topic = msg.topic
        payload = msg.payload.decode('utf-8')
        
        if topic == f"{TOPIC_NS}/device/state":
            handle_device_state(payload)
        elif topic == f"{TOPIC_NS}/sys/online":
            handle_online_status(payload)
            
    except Exception as e:
        print(f"❌ Error handling message: {e}")

def handle_device_state(payload):
    """Update device state from MQTT message"""
    try:
        data = json.loads(payload)
        
        device_state["light"] = data.get("light", "unknown")
        device_state["fan"] = data.get("fan", "unknown") 
        device_state["rssi"] = data.get("rssi", 0)
        device_state["fw"] = data.get("fw", "unknown")
        
        print(f"📱 Device State Updated:")
        print(f"   💡 Light: {device_state['light'].upper()}")
        print(f"   🌀 Fan: {device_state['fan'].upper()}")
        print(f"   📡 RSSI: {device_state['rssi']} dBm")
        print(f"   💿 Firmware: {device_state['fw']}")
        
    except json.JSONDecodeError as e:
        print(f"❌ Invalid device state JSON: {e}")

def handle_online_status(payload):
    """Update online status from MQTT message"""
    try:
        data = json.loads(payload)
        device_state["online"] = data.get("online", False)
        
        status = "🟢 ONLINE" if device_state["online"] else "🔴 OFFLINE"
        print(f"📱 Device Status: {status}")
        
    except json.JSONDecodeError as e:
        print(f"❌ Invalid online status JSON: {e}")

def send_command(device, action):
    """Send control command to device"""
    if not client.is_connected():
        print("❌ Not connected to MQTT broker")
        return False
    
    topic = f"{TOPIC_NS}/device/cmd"
    command = {device: action}
    payload = json.dumps(command)
    
    result = client.publish(topic, payload, qos=1)
    
    if result.rc == mqtt.MQTT_ERR_SUCCESS:
        print(f"📤 Sent command: {device} -> {action}")
        return True
    else:
        print(f"❌ Failed to send command")
        return False

def print_menu():
    """Print control menu"""
    print("\n" + "="*50)
    print("🎛️  IoT Device Controller (Flutter Simulator)")
    print("="*50)
    
    status = "🟢 ONLINE" if device_state["online"] else "🔴 OFFLINE"
    print(f"Device Status: {status}")
    
    if device_state["online"]:
        print(f"💡 Light: {device_state['light'].upper()}")
        print(f"🌀 Fan: {device_state['fan'].upper()}")
        print(f"📡 Signal: {device_state['rssi']} dBm")
        print(f"💿 Firmware: {device_state['fw']}")
    
    print("\nControls:")
    print("1. Toggle Light")
    print("2. Turn Light ON")
    print("3. Turn Light OFF") 
    print("4. Toggle Fan")
    print("5. Turn Fan ON")
    print("6. Turn Fan OFF")
    print("7. Show Device Info")
    print("q. Quit")
    print("-" * 50)

def main():
    print("🚀 Flutter App Simulator Starting...")
    print(f"📡 MQTT Broker: {MQTT_BROKER}:{MQTT_PORT}")
    print(f"🏠 Topic Namespace: {TOPIC_NS}")
    print(f"🆔 Client ID: {CLIENT_ID}")
    
    # Setup MQTT callbacks
    client.on_connect = on_connect
    client.on_message = on_message
    
    try:
        # Connect to broker
        print(f"🔄 Connecting to {MQTT_BROKER}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        
        # Start MQTT loop in background
        client.loop_start()
        
        # Wait for connection
        time.sleep(2)
        
        print("✅ Controller ready!")
        
        # Interactive control loop
        while True:
            print_menu()
            choice = input("Enter your choice: ").strip().lower()
            
            if choice == 'q':
                break
            elif choice == '1':
                send_command("light", "toggle")
            elif choice == '2':
                send_command("light", "on")
            elif choice == '3':
                send_command("light", "off")
            elif choice == '4':
                send_command("fan", "toggle")
            elif choice == '5':
                send_command("fan", "on")
            elif choice == '6':
                send_command("fan", "off")
            elif choice == '7':
                print(f"\n📊 Current Device Info:")
                print(f"   Status: {'🟢 Online' if device_state['online'] else '🔴 Offline'}")
                print(f"   Light: {device_state['light'].upper()}")
                print(f"   Fan: {device_state['fan'].upper()}")
                print(f"   RSSI: {device_state['rssi']} dBm")
                print(f"   Firmware: {device_state['fw']}")
                input("\nPress Enter to continue...")
            else:
                print("❌ Invalid choice. Please try again.")
            
            # Small delay to see command effects
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        pass
    
    print("\n🛑 Shutting down controller...")
    client.loop_stop()
    client.disconnect()
    print("👋 Goodbye!")

if __name__ == "__main__":
    main()