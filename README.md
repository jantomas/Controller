# Hexapod Controller

A comprehensive software controller for a hexapod robot prototype running on Raspberry Pi 5 with Hailo-8L AI accelerator.

## Features

- **Multi-Gait Locomotion**: Support for Tripod, Wave, Ripple, and Metachronal gait patterns
- **AI-Powered Vision**: YOLOv8 object detection and terrain classification on Hailo-8L (13 TOPS)
- **Semi-Autonomous Operation**: Intelligent decision-making with operator confirmation for high-risk actions
- **IoT Integration**: Azure IoT Hub for cloud management and LoRaWAN for long-range communication
- **Comprehensive Telemetry**: Activity logging, sensor data collection, and opportunistic cloud sync
- **Flexible Servo Control**: Support for PCA9685 (I2C) and Pololu Maestro 18-Channel USB controllers
- **Mock Mode**: Full hardware simulation for development and debugging without physical hardware

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Hexapod.Host                             │
│                   (Main Application Entry)                       │
├─────────────────────────────────────────────────────────────────┤
│  Hexapod.Autonomy   │  Hexapod.Communication  │  Hexapod.Telemetry│
│  - Decision Engine  │  - Azure IoT Hub        │  - Activity Logger│
│  - Confirmation     │  - LoRaWAN              │  - Telemetry      │
│  - Mission Planner  │                         │  - Upload         │
├─────────────────────────────────────────────────────────────────┤
│     Hexapod.Vision      │    Hexapod.Movement   │ Hexapod.Sensors │
│  - Hailo Inference      │  - Inverse Kinematics │  - GPS           │
│  - Object Detection     │  - Gait Patterns      │  - IMU           │
│  - Terrain Classifier   │  - Servo Control      │  - Distance      │
├─────────────────────────────────────────────────────────────────┤
│                        Hexapod.Core                             │
│  - Events & EventBus  │  - State Machine  │  - Configuration    │
│  - Models             │  - Enumerations   │                     │
└─────────────────────────────────────────────────────────────────┘
```

## Hardware Requirements

### Primary Platform
- **Raspberry Pi 5** (8GB recommended)
- **Hailo-8L** AI accelerator module (13 TOPS)
- **64-bit Raspberry Pi OS** (Bookworm or later)

### Sensors
- **IMU**: BNO055 or MPU-9250 (I2C)
- **GPS**: u-blox NEO-M9N (UART)
- **Distance**: 6x VL53L1X ToF sensors (I2C)
- **Touch**: 6x FSR402 force sensors (GPIO)
- **Power**: INA219 power monitor (I2C)

### Actuators
- **Servos**: 18x digital servos (3 per leg × 6 legs)
- **PWM Controller** (choose one):
  - **PCA9685**: I2C-based PWM controller (default)
  - **Pololu Maestro 18-Channel**: USB serial servo controller (recommended)

### Communication
- **LoRaWAN**: SX1262 module (UART)
- **WiFi**: Built-in or USB adapter for Azure IoT Hub

## Building

### Prerequisites

#### Linux (Raspberry Pi / Ubuntu)
```bash
# Install .NET 8 SDK
wget https://dot.net/v1/dotnet-install.sh
chmod +x dotnet-install.sh
./dotnet-install.sh --channel 8.0

# Install Hailo runtime (on Raspberry Pi)
sudo apt-get install hailo-all
```

#### Windows
```powershell
# Install .NET 8 SDK via winget
winget install Microsoft.DotNet.SDK.8

# Or download from https://dot.net/download
# Hailo runtime not required for development (use Mock Mode)
```

### Build for Development

#### Windows
```powershell
cd Controller
dotnet restore
dotnet build
```

#### Linux
```bash
cd Controller
dotnet restore
dotnet build
```

### Run in Mock Mode (No Hardware Required)

Mock mode allows full development and debugging without physical hardware:

#### Windows (PowerShell)
```powershell
# Run with Development environment (mock mode enabled by default)
$env:DOTNET_ENVIRONMENT = "Development"
dotnet run --project src/Hexapod.Host/Hexapod.Host.csproj

# Or inline
dotnet run --project src/Hexapod.Host/Hexapod.Host.csproj --environment Development
```

#### Windows (Command Prompt)
```cmd
set DOTNET_ENVIRONMENT=Development
dotnet run --project src/Hexapod.Host/Hexapod.Host.csproj
```

#### Linux
```bash
export DOTNET_ENVIRONMENT=Development
dotnet run --project src/Hexapod.Host/Hexapod.Host.csproj
```

Mock mode simulates:
- 🎮 All 18 servo channels with position tracking
- 🛰️ GPS with configurable position and drift
- 📐 IMU with orientation and acceleration
- 🔋 Battery with simulated drain
- 📏 6 distance sensors (ToF)
- 👆 6 touch/force sensors
- ☁️ Azure IoT Hub telemetry (logged locally)
- 📡 LoRaWAN communication

### Build for Deployment

```bash
# Publish for Raspberry Pi 5 (ARM64 Linux)
dotnet publish src/Hexapod.Host/Hexapod.Host.csproj \
    -c Release \
    -r linux-arm64 \
    --self-contained true \
    -o ./publish
```

## Deployment

### 1. Prepare Raspberry Pi

```bash
# Create directories
sudo mkdir -p /opt/hexapod/models
sudo mkdir -p /var/lib/hexapod/telemetry
sudo mkdir -p /var/log/hexapod
sudo mkdir -p /etc/hexapod/certs

# Set permissions
sudo chown -R $USER:$USER /opt/hexapod
sudo chown -R $USER:$USER /var/lib/hexapod
sudo chown -R $USER:$USER /var/log/hexapod
```

### 2. Copy Application

```bash
# Copy from build machine
scp -r ./publish/* pi@hexapod:/opt/hexapod/
scp ./models/* pi@hexapod:/opt/hexapod/models/
```

### 3. Configure

Edit `/opt/hexapod/appsettings.json`:

```json
{
  "Hexapod": {
    "DeviceId": "hexapod-001",
    \"Hardware\": {
      \"ServoControllerType\": \"PololuMaestro\",
      \"MaestroServo\": {
        \"SerialPort\": \"/dev/ttyACM0\",
        \"BaudRate\": 115200
      }
    },
    \"Communication\": {
      \"IoTHub\": {
        \"HostName\": \"your-hub.azure-devices.net\",
        \"DeviceId\": \"hexapod-001\"
      }
    }
  }
}
```

#### Servo Controller Options

| Controller | Config Value | Serial Port (Linux) | Serial Port (Windows) |
|------------|--------------|--------------------|-----------------------|
| PCA9685 (I2C) | `\"Pca9685\"` | N/A (uses I2C) | N/A |
| Pololu Maestro | `\"PololuMaestro\"` | `/dev/ttyACM0` | `COM3` |

### 4. Install as Service

```bash
sudo nano /etc/systemd/system/hexapod.service
```

```ini
[Unit]
Description=Hexapod Controller Service
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/opt/hexapod
ExecStart=/opt/hexapod/Hexapod.Host
Restart=always
RestartSec=10
Environment=DOTNET_ENVIRONMENT=Production

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable hexapod
sudo systemctl start hexapod
```

## Operation Modes

### Semi-Autonomous (Default)
- Autonomous navigation and obstacle avoidance
- Operator confirmation required for high-risk decisions
- Configurable risk threshold

### Autonomous
- Full autonomous operation
- Only critical situations require operator intervention
- Best for well-mapped environments

### Remote Control
- Direct operator control
- All autonomous features disabled
- Emergency mode fallback

## Azure IoT Hub Integration

### Device Provisioning

The controller supports Azure Device Provisioning Service (DPS) with X.509 certificates:

```bash
# Generate device certificate
openssl req -new -x509 -days 365 \
    -keyout device.key -out device.crt \
    -subj "/CN=hexapod-001"

# Create PFX
openssl pkcs12 -export -out device.pfx \
    -inkey device.key -in device.crt
```

### Direct Methods

| Method | Description |
|--------|-------------|
| `SwitchMode` | Change operation mode |
| `EmergencyStop` | Immediate stop |
| `RequestStatus` | Get current status |
| `StartMission` | Begin mission execution |
| `Ping` | Connectivity check |

### Device Twin Properties

```json
{
  "reported": {
    "firmwareVersion": "1.0.0",
    "currentMode": "SemiAutonomous",
    "batteryLevel": 85,
    "position": { "lat": 47.6062, "lon": -122.3321 }
  },
  "desired": {
    "targetMode": "Autonomous",
    "missionId": "patrol-001"
  }
}
```

## LoRaWAN Communication

### Message Types

- **Position Update**: GPS coordinates (10 bytes)
- **Status Report**: Battery, mode, state, flags (8 bytes)
- **Alert**: Emergency notifications

### Configuration

The controller uses OTAA (Over-the-Air Activation) for LoRaWAN network join.

## AI Models

### Object Detection (YOLOv8n)
- Input: 640x640 RGB
- Output: Bounding boxes, class labels, confidence scores
- Classes: Person, vehicle, animal, obstacle, hazard

### Terrain Classification (MobileNetV3-Small)
- Input: 224x224 RGB
- Output: Terrain type probabilities
- Classes: Paved, Grass, Gravel, Sand, Dirt, Rough, Stairs, Slope, Water

## Safety Features

- **Emergency Stop**: Immediate halt on critical conditions
- **Tilt Protection**: Auto-stop when exceeding tilt threshold
- **Geofencing**: Boundary enforcement for operational area
- **Low Battery Protection**: Safe shutdown sequence
- **Watchdog Timer**: Auto-recovery from system hangs

## Troubleshooting

### Common Issues

**Hailo not detected**
```bash
# Check Hailo status
hailortcli fw-control identify

# Verify module loaded
lsmod | grep hailo
```

**Pololu Maestro not detected**
```bash
# Check USB devices (Linux)
ls -la /dev/ttyACM*

# Check serial port permissions
sudo usermod -a -G dialout $USER
# Log out and back in after adding to group

# Windows: Check COM port in Device Manager
```

**GPS no fix**
```bash
# Check GPS data
cat /dev/ttyAMA0
```

**I2C devices not found**
```bash
# Scan I2C bus
i2cdetect -y 1
```

### Logs

```bash
# View service logs
journalctl -u hexapod -f

# View application logs
tail -f /var/log/hexapod/hexapod-*.log
```

## License

MIT License - See LICENSE file for details.
