# Backend API

Simple Express server handling sensor data uploads from dog tracking devices.

## Setup

```bash
npm install
npm start
```

## Endpoints

### POST /api/motion
Uploads motion sensor readings.

**Body:**
```json
{
  "deviceId": "device-123",
  "data": { /* sensor payload */ }
}
```

### POST /api/heart
Uploads heart rate sensor readings.

**Body:**
```json
{
  "deviceId": "device-123",
  "bpm": 80,
  "ir": 12345
}
```

### POST /api/sound
Uploads audio recordings.

**Form Data:**
- `deviceId`: ID of the device
- `audio`: sound file

Each request automatically registers the device ID and tracks which sensor types are in use. Registration info is stored in `devices.json`.
