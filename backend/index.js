const express = require('express');
const fs = require('fs');
const path = require('path');
const multer = require('multer');

const app = express();
const upload = multer({ dest: 'uploads/' });
app.use(express.json());

const devicesFile = path.join(__dirname, 'devices.json');

function readDevices() {
  try {
    return JSON.parse(fs.readFileSync(devicesFile));
  } catch (e) {
    return {};
  }
}

function writeDevices(data) {
  fs.writeFileSync(devicesFile, JSON.stringify(data, null, 2));
}

function registerDevice(deviceId, sensor) {
  const devices = readDevices();
  if (!devices[deviceId]) {
    devices[deviceId] = { sensors: [] };
  }
  if (!devices[deviceId].sensors.includes(sensor)) {
    devices[deviceId].sensors.push(sensor);
  }
  writeDevices(devices);
}

app.post('/api/motion', (req, res) => {
  const { deviceId, data } = req.body;
  if (!deviceId || !data) {
    return res.status(400).json({ error: 'deviceId and data required' });
  }
  registerDevice(deviceId, 'motion');
  return res.json({ status: 'motion data received' });
});

app.post('/api/heart', (req, res) => {
  const { deviceId, bpm, ir } = req.body;
  if (!deviceId || bpm === undefined || ir === undefined) {
    return res.status(400).json({ error: 'deviceId, bpm and ir required' });
  }
  registerDevice(deviceId, 'heart');
  return res.json({ status: 'heart data received' });
});

app.post('/api/sound', upload.single('audio'), (req, res) => {
  const { deviceId } = req.body;
  if (!deviceId || !req.file) {
    return res.status(400).json({ error: 'deviceId and audio file required' });
  }
  registerDevice(deviceId, 'sound');
  return res.json({ status: 'sound file received' });
});

const port = process.env.PORT || 3000;
app.listen(port, () => {
  console.log(`Server listening on port ${port}`);
});
