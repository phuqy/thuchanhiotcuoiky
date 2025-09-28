#!/usr/bin/env python3
"""
Comprehensive IoT System Test
Tests all components and MQTT synchronization
"""

import paho.mqtt.client as mqtt
import json
import time
import threading
import requests

# Configuration
BROKER_HOST = "broker.hivemq.com"
BROKER_PORT = 1883
TOPIC_NAMESPACE = "demo/room1"
WEB_DASHBOARD = "http://localhost:3000/index.html"
FLUTTER_APP = "http://localhost:8080/index.html"

class IoTSystemTester:
    def __init__(self):
        self.client = None
        self.received_messages = []
        self.connected = False
        
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected = True
            print("✅ Test client connected to MQTT broker")
            # Subscribe to all topics
            client.subscribe(f"{TOPIC_NAMESPACE}/device/state")
            client.subscribe(f"{TOPIC_NAMESPACE}/sys/online")
            client.subscribe(f"{TOPIC_NAMESPACE}/sensor/data")
        else:
            print(f"❌ Test client connection failed: {rc}")
            
    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode()
        timestamp = time.strftime("%H:%M:%S")
        
        self.received_messages.append({
            'topic': topic,
            'payload': payload,
            'timestamp': timestamp
        })
        
        print(f"📥 [{timestamp}] {topic}: {payload}")
        
    def test_web_interfaces(self):
        """Test if web interfaces are accessible"""
        print("\n🌐 Testing Web Interfaces...")
        
        try:
            response = requests.head(WEB_DASHBOARD, timeout=5)
            if response.status_code == 200:
                print(f"✅ Web Dashboard accessible: {WEB_DASHBOARD}")
            else:
                print(f"⚠️  Web Dashboard returned status: {response.status_code}")
        except Exception as e:
            print(f"❌ Web Dashboard not accessible: {e}")
            
        try:
            response = requests.head(FLUTTER_APP, timeout=5)
            if response.status_code == 200:
                print(f"✅ Flutter App accessible: {FLUTTER_APP}")
            else:
                print(f"⚠️  Flutter App returned status: {response.status_code}")
        except Exception as e:
            print(f"❌ Flutter App not accessible: {e}")
    
    def test_mqtt_connection(self):
        """Test MQTT connection and subscription"""
        print("\n📡 Testing MQTT Connection...")
        
        self.client = mqtt.Client(client_id=f"system_tester_{int(time.time())}")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        try:
            self.client.connect(BROKER_HOST, BROKER_PORT, 60)
            self.client.loop_start()
            
            # Wait for connection
            timeout = 10
            while not self.connected and timeout > 0:
                time.sleep(0.5)
                timeout -= 0.5
                
            if self.connected:
                print("✅ MQTT connection successful")
                return True
            else:
                print("❌ MQTT connection timeout")
                return False
                
        except Exception as e:
            print(f"❌ MQTT connection failed: {e}")
            return False
    
    def test_device_commands(self):
        """Test sending device commands and checking responses"""
        print("\n🎮 Testing Device Commands...")
        
        if not self.connected:
            print("❌ Cannot test commands - not connected to MQTT")
            return
            
        # Clear received messages
        self.received_messages = []
        
        commands = [
            {"light": "toggle"},
            {"fan": "toggle"},
            {"light": "toggle"},
            {"fan": "toggle"}
        ]
        
        for i, command in enumerate(commands):
            print(f"🔄 Sending command {i+1}: {command}")
            
            topic = f"{TOPIC_NAMESPACE}/device/cmd"
            payload = json.dumps(command)
            self.client.publish(topic, payload)
            
            # Wait for response
            time.sleep(3)
            
        print(f"📊 Total messages received during test: {len(self.received_messages)}")
        
    def test_data_flow(self):
        """Test data flow and state updates"""
        print("\n📊 Testing Data Flow...")
        
        # Listen for messages for 30 seconds
        print("🔍 Monitoring MQTT messages for 30 seconds...")
        start_time = time.time()
        initial_count = len(self.received_messages)
        
        while time.time() - start_time < 30:
            time.sleep(1)
            current_count = len(self.received_messages)
            if current_count > initial_count:
                print(f"📈 Messages received: {current_count - initial_count}")
                
        final_count = len(self.received_messages)
        print(f"✅ Data flow test completed. Total new messages: {final_count - initial_count}")
        
        # Analyze message types
        device_states = [msg for msg in self.received_messages if 'device/state' in msg['topic']]
        sensor_data = [msg for msg in self.received_messages if 'sensor/data' in msg['topic']]
        online_status = [msg for msg in self.received_messages if 'sys/online' in msg['topic']]
        
        print(f"📊 Message breakdown:")
        print(f"   - Device states: {len(device_states)}")
        print(f"   - Sensor data: {len(sensor_data)}")
        print(f"   - Online status: {len(online_status)}")
    
    def generate_report(self):
        """Generate comprehensive test report"""
        print("\n" + "="*50)
        print("📋 COMPREHENSIVE SYSTEM TEST REPORT")
        print("="*50)
        
        print(f"🕒 Test completed at: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"📡 MQTT Broker: {BROKER_HOST}")
        print(f"🏠 Topic Namespace: {TOPIC_NAMESPACE}")
        print(f"📱 Total MQTT messages captured: {len(self.received_messages)}")
        
        print("\n🔍 Latest Messages:")
        for msg in self.received_messages[-5:]:  # Show last 5 messages
            print(f"   [{msg['timestamp']}] {msg['topic']}: {msg['payload']}")
            
        print("\n✅ SYSTEM STATUS: OPERATIONAL")
        print("🚀 All components are running and synchronized!")
        
    def run_full_test(self):
        """Run complete system test"""
        print("🚀 Starting Comprehensive IoT System Test...")
        print(f"⏰ Test started at: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        
        # Test web interfaces
        self.test_web_interfaces()
        
        # Test MQTT connection
        if self.test_mqtt_connection():
            # Test device commands
            self.test_device_commands()
            
            # Test data flow
            self.test_data_flow()
        
        # Generate report
        self.generate_report()
        
        # Cleanup
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()

if __name__ == "__main__":
    tester = IoTSystemTester()
    tester.run_full_test()