# Hexapod Controller - Software Architecture

## Overview

The Hexapod Controller is a sophisticated software system designed for autonomous and semi-autonomous operation of a six-legged robotic platform. The system prioritizes energy efficiency, reliable navigation, and safe operation through intelligent decision-making with operator oversight.

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              HEXAPOD CONTROLLER                                   │
├─────────────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
│  │   Vision    │  │   Sensors   │  │  Movement   │  │  Autonomy   │             │
│  │   Module    │  │   Module    │  │   Module    │  │   Module    │             │
│  │             │  │             │  │             │  │             │             │
│  │ • Camera    │  │ • GPS       │  │ • Gait Ctrl │  │ • Decision  │             │
│  │ • Hailo AI  │  │ • IMU       │  │ • IK Solver │  │ • Risk Eval │             │
│  │ • YOLOv8n   │  │ • Distance  │  │ • Path Plan │  │ • Operator  │             │
│  │ • MobileNet │  │ • Touch     │  │ • Energy    │  │   Confirm   │             │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘             │
│         │                │                │                │                     │
│  ┌──────┴────────────────┴────────────────┴────────────────┴──────┐             │
│  │                         CORE CONTROLLER                         │             │
│  │                                                                 │             │
│  │  • State Machine        • Event Bus         • Task Scheduler    │             │
│  │  • System Coordinator   • Error Handler     • Power Manager     │             │
│  └─────────────────────────────┬───────────────────────────────────┘             │
│                                │                                                 │
│  ┌─────────────────────────────┴───────────────────────────────────┐             │
│  │                      COMMUNICATION LAYER                         │             │
│  │                                                                 │             │
│  │  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐       │             │
│  │  │   LoRaWAN    │    │  Azure IoT   │    │  Telemetry   │       │             │
│  │  │   Gateway    │    │     Hub      │    │   Service    │       │             │
│  │  └──────────────┘    └──────────────┘    └──────────────┘       │             │
│  └─────────────────────────────────────────────────────────────────┘             │
└─────────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              AZURE CLOUD                                         │
├─────────────────────────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐         │
│  │  IoT Hub     │  │   Storage    │  │   Stream     │  │   Digital    │         │
│  │              │──│   Account    │──│   Analytics  │──│   Twins      │         │
│  └──────────────┘  └──────────────┘  └──────────────┘  └──────────────┘         │
└─────────────────────────────────────────────────────────────────────────────────┘
```

## Hardware Platform

### Processing Unit
- **Raspberry Pi 5**: Main controller running .NET 8 on Linux ARM64
- **Hailo-8L**: AI accelerator (13 TOPS) for real-time inference

### Recommended Sensors

| Sensor Type | Recommended Model | Purpose | Interface |
|-------------|------------------|---------|-----------|
| **Camera** | Raspberry Pi Camera Module 3 | Object detection, navigation | CSI |
| **IMU** | BNO055 / MPU-9250 | Orientation, acceleration | I2C |
| **GPS** | u-blox NEO-M9N | Geolocation | UART |
| **Distance** | VL53L1X (ToF) × 6 | Obstacle detection | I2C |
| **Touch** | FSR402 × 6 | Ground contact sensing | ADC |
| **Current** | INA219 | Power monitoring | I2C |

### Communication
- **LoRaWAN**: Long-range, low-power communication via SX1262 module
- **WiFi**: High-bandwidth local communication (when available)

### Servo Controllers

| Controller | Interface | Features | Use Case |
|------------|-----------|----------|----------|
| **PCA9685** | I2C | 16-channel PWM, 12-bit resolution | Simple setups, low latency |
| **Pololu Maestro 18** | USB Serial | 18 channels, per-servo speed/accel limits, calibration | Recommended for hexapod |

#### Pololu Maestro Configuration
```json
{
  \"Hardware\": {
    \"ServoControllerType\": \"PololuMaestro\",
    \"MaestroServo\": {
      \"SerialPort\": \"/dev/ttyACM0\",
      \"BaudRate\": 115200,
      \"DeviceNumber\": 12,
      \"DefaultMinPwmUs\": 500,
      \"DefaultMaxPwmUs\": 2500
    }
  }
}
```

## AI/ML Models for Hailo-8L

### Recommended Pre-trained Models

| Model | Purpose | Size | Performance on Hailo-8L |
|-------|---------|------|------------------------|
| **YOLOv8n** | Object detection | 3.2M params | ~60 FPS |
| **MobileNetV3-Small** | Image classification | 2.5M params | ~120 FPS |
| **EfficientNet-Lite0** | Feature extraction | 4.7M params | ~80 FPS |
| **PoseNet** | Terrain analysis | 2.5M params | ~90 FPS |

### Model Selection Rationale

1. **YOLOv8 Nano**: Best balance of accuracy and speed for obstacle/object detection
2. **MobileNetV3**: Efficient classification for terrain type identification
3. **EfficientNet-Lite**: General feature extraction with good accuracy
4. **PoseNet**: Useful for analyzing terrain slopes and surface characteristics

## Software Modules

### 1. Hexapod.Core
Core abstractions, interfaces, state machine, and event system.

### 2. Hexapod.Movement
- Inverse kinematics solver for 18-DOF hexapod
- Energy-efficient gait patterns (tripod, wave, ripple)
- Path planning and obstacle avoidance
- Terrain adaptation algorithms
- **Servo Controllers**:
  - `Pca9685ServoController`: I2C-based PWM control
  - `PololuMaestroServoController`: USB serial with Compact Protocol
  - `MockServoController`: Simulated servos for development

### 3. Hexapod.Sensors
- Unified sensor abstraction layer
- GPS positioning with Kalman filtering
- IMU sensor fusion (complementary/Madgwick filter)
- Distance sensor array management
- Touch/force sensing for ground contact
- **Mock Implementations**: Full simulation for all sensors (GPS, IMU, Power, Distance, Touch)

### 4. Hexapod.Vision
- Camera capture and preprocessing
- Hailo-8L inference integration
- Object detection pipeline (YOLOv8)
- Scene understanding and classification

### 5. Hexapod.Communication
- LoRaWAN protocol implementation with automatic WiFi fallback
- Azure IoT Hub device client with DPS support
- Message queuing and prioritization
- Connection resilience and failover
- **Mock Services**: Simulated IoT Hub and LoRaWAN for offline development

### 6. Hexapod.Autonomy
- Decision-making engine
- Risk assessment and classification
- Operator confirmation workflow
- Mission planning and execution
- Emergency mode handling

### 7. Hexapod.Telemetry
- Activity logging and buffering
- Metric collection and aggregation
- Opportunistic data upload
- Local storage management

## Operation Modes

### 1. Autonomous Mode
Full autonomous operation with predefined mission parameters.

### 2. Semi-Autonomous Mode (Default)
- Autonomous navigation with operator confirmation for:
  - Irreversible actions (crossing boundaries, entering restricted areas)
  - High-risk maneuvers (steep terrain, water crossings)
  - Uncertain situations (unrecognized obstacles, path ambiguity)

### 3. Remote Control Mode
Direct operator control for emergency situations or complex scenarios.

### 4. Safe Mode
Minimal operation mode when critical errors occur or communication is lost.

## Energy Efficiency Strategies

1. **Adaptive Gait Selection**: Switch gaits based on terrain and speed requirements
2. **Sleep Scheduling**: Power down unused sensors and modules
3. **Batch Processing**: Aggregate data for efficient transmission
4. **Edge AI**: Process locally to minimize communication overhead
5. **Predictive Planning**: Optimize routes for minimal energy consumption

## Azure IoT Integration

### Device Twin Properties
```json
{
  "desired": {
    "operationMode": "semi-autonomous",
    "missionParameters": { ... },
    "sensorConfiguration": { ... }
  },
  "reported": {
    "batteryLevel": 85,
    "currentPosition": { "lat": 0.0, "lon": 0.0 },
    "systemStatus": "operational",
    "lastTelemetryUpload": "2026-01-07T10:00:00Z"
  }
}
```

### Direct Methods
- `SwitchMode`: Change operation mode
- `EmergencyStop`: Immediate halt
- `UpdateMission`: Modify mission parameters
- `RequestStatus`: Get detailed status report

### Telemetry Topics
- `sensors/gps`: Position updates
- `sensors/imu`: Orientation data
- `movement/gait`: Gait status
- `vision/detections`: Object detections
- `system/power`: Power metrics
- `autonomy/decisions`: Decision logs

## Security Considerations

1. **TLS 1.3** for all Azure communications
2. **X.509 certificates** for device authentication
3. **Secure boot** on Raspberry Pi
4. **Encrypted local storage** for sensitive data
5. **Command signature verification** for remote commands

## Development Guidelines

1. Use dependency injection throughout
2. Implement circuit breakers for external services
3. Log at appropriate levels (structured logging)
4. Unit test all business logic
5. Integration test sensor interfaces
6. Document all public APIs

## Mock Mode for Development

The system includes comprehensive mock implementations for development without hardware:

### Configuration (`appsettings.Development.json`)
```json
{
  "Hexapod": {
    "MockMode": {
      "Enabled": true,
      "MockServos": true,
      "MockSensors": true,
      "MockCommunication": true,
      "MockVision": true,
      "VerboseLogging": true,
      "SimulateNoise": true
    }
  }
}
```

### Mock Components

| Component | Mock Class | Features |
|-----------|------------|----------|
| **Servo Controller** | `MockServoController` | Position tracking, events, simulated delays |
| **GPS** | `MockGpsSensor` | Configurable position, drift simulation |
| **IMU** | `MockImuSensor` | Orientation, acceleration, calibration |
| **Power** | `MockPowerSensor` | Battery drain simulation, voltage tracking |
| **Distance** | `MockDistanceSensorArray` | 6 ToF sensors, obstacle simulation |
| **Touch** | `MockTouchSensorArray` | Ground contact, gait testing |
| **IoT Hub** | `MockAzureIoTHubService` | Telemetry logging, method simulation |
| **LoRaWAN** | `MockLoRaWanService` | Message logging, downlink simulation |

### Running in Mock Mode
```bash
# Environment variable enables Development config
DOTNET_ENVIRONMENT=Development dotnet run
```
