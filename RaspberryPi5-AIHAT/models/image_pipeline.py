import json, time
import degirum as dg
from picamera2 import Picamera2
import cv2
import numpy as np
import paho.mqtt.client as mqtt
import serial  # <-- Added for USB serial

# ==== SERIAL CONFIGURATION ====
# Replace '/dev/ttyUSB0' with the port your ESP32 shows up as (check with ls /dev/ttyUSB*)
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # wait for ESP32 reset

# — MQTT CONFIGURATION —
broker = '172.20.10.2'
port = 1883
topic = 'bbox/all_centers'
client = mqtt.Client('pi_yolo_pub')
client.connect(broker, port)
client.loop_start()
time.sleep(1)

# — Load YOLO model —
model_yolo = dg.load_model(
    model_name='better--640x640_quant_hailort_hailo8_1',
    inference_host_address='@local',
    zoo_url='/home/pi/my_hailo_zoo'
)

# — Load SC-Depth model and custom postprocessor —
model_name = "scdepthv3--256x320_quant_hailort_hailo8_1"
model_depth = dg.load_model(
    model_name=model_name,
    inference_host_address="@local",
    zoo_url="degirum/hailo"
)
mi = model_depth.model_info

# — Define SC-Depth postprocessor —
def make_depth_postprocessor(mi):
    def dequantize(info):
        zp = info['quantization']['zero']
        scale = info['quantization']['scale']
        return (info['data'].astype(np.float32) - zp) * scale

    def sigmoid(x):
        return 1 / (1 + np.exp(-x))

    class DepthResults(dg.postprocessor.InferenceResults):
        color_map = cv2.COLORMAP_VIRIDIS
        use_scdepth = True
        normalize_results = False

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            info = self._inference_results[0]
            data = info['data']
            if data.dtype.kind in ('u','i'):
                data = dequantize(info)
            data = data.squeeze(0)

            H, W = mi.InputH[0], mi.InputW[0]
            if data.shape[:2] != (H, W):
                data = data.reshape((H, W, -1))
            if self.image is not None:
                h_img, w_img = self.image.shape[:2]
                data = cv2.resize(data, (w_img, h_img), interpolation=cv2.INTER_LINEAR)
                data = data[np.newaxis, ..., :]

            if DepthResults.use_scdepth:
                data = 1 / (sigmoid(data) * 10 + 0.009)
            if DepthResults.normalize_results:
                dm = data.squeeze(0)
                data = ((dm - dm.min()) / (dm.max() - dm.min()))[np.newaxis, ..., :]

            info['data'] = data

        def _convert_depth_to_image(self, depth_map):
            dm = depth_map.squeeze(0)
            if not DepthResults.normalize_results:
                dm = (dm - dm.min()) / (dm.max() - dm.min())
            img = (dm * 255).astype(np.uint8)
            return cv2.applyColorMap(img, DepthResults.color_map)

        @property
        def image_overlay(self):
            dm = self._inference_results[0]['data']
            return self._convert_depth_to_image(dm)

    return DepthResults

model_depth.custom_postprocessor = make_depth_postprocessor(mi)

# — Initialize camera —
picam2 = Picamera2()
cfg = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(cfg)
picam2.start()

print("Running YOLO and Depth inference. Press 'q' to exit.")
frame_count = 0
depth_map = None  # Updated every 3rd frame

while True:
    frame = picam2.capture_array()  # 640x480 RGB frame

    # --- YOLO Inference ---
    result_y = model_yolo(frame)
    overlay_y = result_y.image_overlay.copy()
    detections = []

    for det in result_y.results:
        x1, y1, x2, y2 = det['bbox']
        cls = det.get('label', det.get('class', ''))
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

        depth_val = None
        if depth_map is not None:
            # Scale center to depth map size
            scale_x = depth_map.shape[2] / 640  # width
            scale_y = depth_map.shape[1] / 480  # height
            dx = int(cx * scale_x)
            dy = int(cy * scale_y)
            # Clamp indices to avoid overflow
            dx = min(max(dx, 0), depth_map.shape[2] - 1)
            dy = min(max(dy, 0), depth_map.shape[1] - 1)
            depth_val = float(depth_map[0, dy, dx])
            depth_str = f"{depth_val:.2f}cm"
        else:
            depth_str = "N/A"

        # Draw bounding box center and label
        cv2.circle(overlay_y, (cx, cy), 5, (255, 0, 0), -1)
        cv2.putText(overlay_y, f"{cls} {depth_str}", (cx, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        detections.append({
            "class": cls,
            "cx": cx,
            "cy": cy,
            "depth": round(depth_val, 4) if depth_val is not None else None
        })

    if detections:
        payload = json.dumps(detections)
        ser.write((payload + "\n").encode('utf-8'))  # <-- newline terminator is important
        client.publish(topic, payload, qos=1)
        

        
    cv2.imshow("YOLO Detection", overlay_y)

    # --- SC-Depth every 3rd frame ---
    if frame_count % 3 == 0:
        frame_depth = cv2.resize(frame, (320, 256), interpolation=cv2.INTER_LINEAR)
        res_depth = model_depth(frame_depth)
        overlay_d = res_depth.image_overlay
        depth_map = res_depth.results[0]['data']  # Shape: (1, 256, 320)

        disp = cv2.resize(overlay_d, (640, 480))
        cv2.imshow("Depth Overlay", disp)

    frame_count += 1
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
picam2.stop()
client.loop_stop()
client.disconnect()
ser.close()
cv2.destroyAllWindows()
