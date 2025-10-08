# -*- coding: utf-8 -*-
"""
MQTT to PostgreSQL Bridge

This script listens to an MQTT broker for specific topics,
parses the JSON data, and inserts it into a PostgreSQL database.
"""

import paho.mqtt.client as mqtt
import psycopg2
import json
import os
import time
from datetime import datetime

# =============================================================================
# CONFIGURATION - THAY ĐỔI CÁC THÔNG SỐ NÀY
# =============================================================================

# MQTT Broker Configuration (Phải giống với ESP32 và Web)
MQTT_BROKER_HOST = "broker.hivemq.com"
MQTT_BROKER_PORT = 1883
TOPIC_NS = "lab/test_esp32" # <<-- QUAN TRỌNG: Phải khớp với firmware của bạn

# PostgreSQL Database Configuration
DB_HOST = "localhost"
DB_PORT = "5432" # Mặc định của PostgreSQL
DB_NAME = "iot_data" # Tên database bạn vừa tạo
DB_USER = "postgres" # Tên user của bạn (thường là 'postgres')
DB_PASSWORD = "123123" # <<-- THAY MẬT KHẨU POSTGRESQL CỦA BẠN

# =============================================================================

# Topics to subscribe to
SENSOR_TOPIC = f"{TOPIC_NS}/sensor/state"
COMMAND_TOPIC = f"{TOPIC_NS}/device/cmd"

def connect_db():
    """Establishes a connection to the PostgreSQL database."""
    while True:
        try:
            print("Connecting to PostgreSQL database...")
            conn = psycopg2.connect(
                host=DB_HOST,
                port=DB_PORT,
                dbname=DB_NAME,
                user=DB_USER,
                password=DB_PASSWORD
            )
            print("PostgreSQL connection successful!")
            return conn
        except psycopg2.OperationalError as e:
            print(f"Database connection failed: {e}")
            print("Retrying in 5 seconds...")
            time.sleep(5)

def on_connect(client, userdata, flags, rc):
    """Callback for when the client connects to the MQTT broker."""
    if rc == 0:
        print("Connected to MQTT Broker!")
        print(f"Subscribing to topic: {SENSOR_TOPIC}")
        client.subscribe(SENSOR_TOPIC)
        print(f"Subscribing to topic: {COMMAND_TOPIC}")
        client.subscribe(COMMAND_TOPIC)
    else:
        print(f"Failed to connect to MQTT, return code {rc}\n")

def on_message(client, userdata, msg):
    """Callback for when a message is received from the MQTT broker."""
    db_conn = userdata['db_conn']
    try:
        payload = msg.payload.decode('utf-8')
        print(f"Received message on topic '{msg.topic}': {payload}")
        
        # Parse the JSON payload
        data = json.loads(payload)
        
        # Get a cursor
        cursor = db_conn.cursor()

        # Check the topic and insert data accordingly
        if msg.topic == SENSOR_TOPIC:
            temp = data.get('temp_c')
            hum = data.get('hum_pct')
            
            if temp is not None and hum is not None:
                sql = "INSERT INTO sensor_data (temperature, humidity) VALUES (%s, %s);"
                cursor.execute(sql, (temp, hum))
                print(f"Inserted into sensor_data: temp={temp}, hum={hum}")

        elif msg.topic == COMMAND_TOPIC:
            # A command can have 'light' or 'fan' key
            device = next(iter(data)) # Get the first key ('light' or 'fan')
            command = data[device]
            
            sql = "INSERT INTO control_logs (device, command) VALUES (%s, %s);"
            cursor.execute(sql, (device, command))
            print(f"Inserted into control_logs: device={device}, command={command}")
            
        # Commit the transaction
        db_conn.commit()
        cursor.close()

    except json.JSONDecodeError:
        print(f"Error: Could not decode JSON from payload: {payload}")
    except (Exception, psycopg2.Error) as error:
        print(f"Error while inserting to PostgreSQL: {error}")
        # Rollback in case of error
        db_conn.rollback()

if __name__ == '__main__':
    # Establish database connection
    db_connection = connect_db()

    # Set up MQTT client
    client = mqtt.Client(client_id=f"db_bridge_{os.getpid()}")
    client.user_data_set({'db_conn': db_connection}) # Pass db connection to callbacks
    client.on_connect = on_connect
    client.on_message = on_message

    # Connect to MQTT broker
    try:
        client.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT, 60)
    except Exception as e:
        print(f"Could not connect to MQTT broker: {e}")
        exit(1)

    # Start the network loop to process messages
    try:
        print("Starting MQTT loop...")
        client.loop_forever()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Clean up
        client.disconnect()
        if db_connection:
            db_connection.close()
            print("Database connection closed.")
