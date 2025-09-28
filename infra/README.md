# Infrastructure Setup

## MQTT Broker Setup (Mosquitto)

### Installation

**Ubuntu/Debian:**
```bash
sudo apt update
sudo apt install mosquitto mosquitto-clients
```

**Windows:**
Download from https://mosquitto.org/download/

**Docker:**
```bash
docker run -it -p 1883:1883 -p 9001:9001 -v $(pwd)/mosquitto.conf:/mosquitto/config/mosquitto.conf eclipse-mosquitto
```

### Configuration

1. Copy `mosquitto.conf` to appropriate location:
   - Linux: `/etc/mosquitto/mosquitto.conf`
   - Windows: `C:\Program Files\mosquitto\mosquitto.conf`

2. Create password file:
```bash
# Create user for read-only access (Web)
sudo mosquitto_passwd -c /etc/mosquitto/passwd user1
# Enter password: pass1

# Create user for control access (App) - optional
sudo mosquitto_passwd /etc/mosquitto/passwd control_user
# Enter password: control_pass
```

3. Start broker:
```bash
# Linux (systemd)
sudo systemctl start mosquitto
sudo systemctl enable mosquitto

# Manual start
mosquitto -c /etc/mosquitto/mosquitto.conf

# Windows
net start mosquitto
```

### Testing Connection

**Test TCP connection:**
```bash
# Subscribe (terminal 1)
mosquitto_sub -h 192.168.1.10 -p 1883 -u user1 -P pass1 -t "lab/room1/sensor/state"

# Publish test message (terminal 2)
mosquitto_pub -h 192.168.1.10 -p 1883 -u user1 -P pass1 -t "lab/room1/sensor/state" -m '{"ts":1695890000,"temp_c":25.5,"hum_pct":60,"lux":150}'
```

**Test WebSocket connection:**
Use browser developer tools or online MQTT WebSocket client at `ws://192.168.1.10:9001`

### Firewall Configuration

**Ubuntu/Linux:**
```bash
sudo ufw allow 1883/tcp
sudo ufw allow 9001/tcp
```

**Windows:**
Open Windows Defender Firewall → Allow ports 1883 and 9001

### Logs and Troubleshooting

**Check logs:**
```bash
# Linux
sudo tail -f /var/log/mosquitto/mosquitto.log

# Check service status
sudo systemctl status mosquitto
```

**Common issues:**
- Permission denied: Check password file permissions
- Connection refused: Verify broker is running and ports are open
- WebSocket connection fails: Ensure WebSocket listener is configured correctly

### Production Notes

- Use TLS/SSL certificates for secure connections
- Configure proper ACL (Access Control Lists) for topic-based permissions
- Set up monitoring and log rotation
- Consider clustering for high availability
- Backup persistence database regularly