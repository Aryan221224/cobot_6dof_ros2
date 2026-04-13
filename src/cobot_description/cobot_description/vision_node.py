#!/usr/bin/env python3
"""vision_node.py - Vision and perception layer for the cobot"""
import rclpy, json, time, math, threading
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
try:
    import cv2, numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

class VisionNode(Node):
    KNOWN_SIZES = {
        'well_plate_96': (0.127, 0.086),
        'eppendorf_tube': (0.011, 0.040),
        'petri_dish_100': (0.100, 0.015),
        'aruco_marker_50mm': (0.050, 0.050),
    }
    COLOUR_TARGETS = {
        'blue_tube': ([100, 80, 80], [130, 255, 255]),
        'red_marker': ([0, 120, 120], [10, 255, 255]),
        'white_plate': ([0, 0, 200], [180, 30, 255]),
        'yellow_tip': ([20, 100, 100], [35, 255, 255]),
    }
    FOCAL_LENGTH_PX = 800.0
    SENSOR_WIDTH_PX = 1280
    SENSOR_HEIGHT_PX = 720

    def __init__(self):
        super().__init__('vision_node')
        self.declare_parameter('use_camera', False)
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('detect_mode', 'labware')
        self.declare_parameter('sim_fps', 10.0)
        self.declare_parameter('show_window', False)
        
        self.use_camera = self.get_parameter('use_camera').value
        self.cam_idx = self.get_parameter('camera_index').value
        self.detect_mode = self.get_parameter('detect_mode').value
        self.sim_fps = self.get_parameter('sim_fps').value
        self.show_window = self.get_parameter('show_window').value
        
        if not CV2_AVAILABLE:
            self.get_logger().warn('OpenCV not installed')
        
        self.pub_detection = self.create_publisher(String, '/vision/detection', 10)
        self.pub_target = self.create_publisher(Point, '/vision/target_position', 10)
        self.pub_status = self.create_publisher(String, '/vision/status', 10)
        
        self.frame_count = 0
        self.fps = 0.0
        self.t_last_fps = time.time()
        self.cap = None
        self.detections = []
        self.sim_objects = [
            {'name': 'well_plate_96', 'x': 0.32, 'y': 0.18, 'z': 0.05, 'color': (200, 200, 200), 'size': (127, 86)},
            {'name': 'eppendorf_tube', 'x': 0.28, 'y': 0.22, 'z': 0.08, 'color': (80, 160, 220), 'size': (11, 40)},
            {'name': 'eppendorf_tube', 'x': 0.31, 'y': 0.22, 'z': 0.08, 'color': (80, 220, 130), 'size': (11, 40)},
            {'name': 'petri_dish_100', 'x': 0.40, 'y': 0.15, 'z': 0.04, 'color': (220, 220, 240), 'size': (100, 15)},
        ]
        
        self.perception_thread = threading.Thread(target=self.perception_loop, daemon=True)
        self.perception_thread.start()
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info(f'Vision node started | mode={self.detect_mode}')
    
    def init_camera(self):
        if not CV2_AVAILABLE:
            return False
        self.cap = cv2.VideoCapture(self.cam_idx)
        if not self.cap.isOpened():
            self.get_logger().warn(f'Camera {self.cam_idx} not available')
            self.use_camera = False
            return False
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.SENSOR_WIDTH_PX)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.SENSOR_HEIGHT_PX)
        self.get_logger().info(f'Camera {self.cam_idx} opened')
        return True
    
    def perception_loop(self):
        if self.use_camera:
            self.init_camera()
        interval = 1.0 / self.sim_fps
        while rclpy.ok():
            t0 = time.time()
            if self.use_camera and self.cap and CV2_AVAILABLE:
                ret, frame = self.cap.read()
                if ret:
                    self.process_frame(frame)
            else:
                self.simulate_detections()
            elapsed = time.time() - t0
            sleep_t = max(0.0, interval - elapsed)
            time.sleep(sleep_t)
            self.frame_count += 1
            now = time.time()
            if now - self.t_last_fps >= 2.0:
                self.fps = self.frame_count / (now - self.t_last_fps)
                self.frame_count = 0
                self.t_last_fps = now
    
    def process_frame(self, frame):
        detections = []
        if self.detect_mode == 'colour':
            detections = self.detect_colour(frame)
        elif self.detect_mode == 'aruco':
            detections = self.detect_aruco(frame)
        elif self.detect_mode == 'labware':
            detections = self.detect_labware(frame)
        self.detections = detections
        self.publish_detections(detections)
        if self.show_window:
            annotated = self.annotate_frame(frame, detections)
            cv2.imshow('Cobot Vision', annotated)
            cv2.waitKey(1)
    
    def detect_colour(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections = []
        for name, (lo, hi) in self.COLOUR_TARGETS.items():
            mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 500:
                    M = cv2.moments(cnt)
                    if M['m00'] > 0:
                        cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
                        x, y, w, h = cv2.boundingRect(cnt)
                        pos = self.pixel_to_3d(cx, cy, w, name)
                        detections.append({'label': name, 'confidence': 0.85, 'pixel': [cx, cy], 'bbox': [x, y, w, h], 'position_m': pos})
        return detections
    
    def detect_aruco(self, frame):
        if not CV2_AVAILABLE:
            return []
        detections = []
        try:
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            aruco_params = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
            corners, ids, _ = detector.detectMarkers(frame)
            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    c = corners[i][0]
                    cx, cy = int(c[:, 0].mean()), int(c[:, 1].mean())
                    side_px = float(np.linalg.norm(c[0] - c[1]))
                    pos = self.pixel_to_3d(cx, cy, side_px, 'aruco_marker_50mm')
                    detections.append({'label': f'aruco_{marker_id}', 'confidence': 0.99, 'pixel': [cx, cy], 'position_m': pos, 'marker_id': int(marker_id)})
        except Exception as e:
            self.get_logger().debug(f'ArUco detection error: {e}')
        return detections
    
    def detect_labware(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)
        circles = cv2.HoughCircles(gray_blur, cv2.HOUGH_GRADIENT, dp=1.2, minDist=20, param1=50, param2=30, minRadius=5, maxRadius=60)
        detections = []
        if circles is not None:
            circles = np.round(circles[0, :]).astype(int)
            for x, y, r in circles:
                pos = self.pixel_to_3d(x, y, r * 2, 'eppendorf_tube')
                detections.append({'label': 'circular_labware', 'confidence': 0.78, 'pixel': [int(x), int(y)], 'radius_px': int(r), 'position_m': pos})
        return detections
    
    def pixel_to_3d(self, px, py, size_px, obj_name):
        known = self.KNOWN_SIZES.get(obj_name, (0.05, 0.05))
        known_m = known[0]
        if size_px > 0:
            Z = (known_m * self.FOCAL_LENGTH_PX) / size_px
        else:
            Z = 0.5
        X = (px - self.SENSOR_WIDTH_PX / 2) * Z / self.FOCAL_LENGTH_PX
        Y = (py - self.SENSOR_HEIGHT_PX / 2) * Z / self.FOCAL_LENGTH_PX
        return {'x': round(X, 4), 'y': round(Y, 4), 'z': round(Z, 4)}
    
    def annotate_frame(self, frame, detections):
        out = frame.copy()
        for det in detections:
            if 'bbox' in det:
                x, y, w, h = det['bbox']
                cv2.rectangle(out, (x, y), (x+w, y+h), (0, 255, 100), 2)
            if 'pixel' in det:
                cx, cy = det['pixel']
                cv2.circle(out, (cx, cy), 4, (0, 255, 0), -1)
        return out
    
    def simulate_detections(self):
        t = time.time()
        detections = []
        for obj in self.sim_objects:
            noise = 0.002
            pos = {
                'x': obj['x'] + noise * math.sin(t * 3.7),
                'y': obj['y'] + noise * math.cos(t * 2.3),
                'z': obj['z'] + noise * 0.5 * math.sin(t * 1.1),
            }
            conf = 0.90 + 0.08 * math.sin(t * 0.5 + len(obj['name']))
            detections.append({
                'label': obj['name'],
                'confidence': round(min(0.99, conf), 3),
                'position_m': {k: round(v, 4) for k, v in pos.items()},
                'source': 'simulation'
            })
        self.detections = detections
        self.publish_detections(detections)
    
    def publish_detections(self, detections):
        if not detections:
            return
        msg = String()
        msg.data = json.dumps({
            'timestamp': time.time(),
            'frame': self.frame_count,
            'detections': detections,
            'detect_mode': self.detect_mode
        })
        self.pub_detection.publish(msg)
        best = max(detections, key=lambda d: d.get('confidence', 0))
        if 'position_m' in best:
            pt = Point()
            pt.x = float(best['position_m']['x'])
            pt.y = float(best['position_m']['y'])
            pt.z = float(best['position_m']['z'])
            self.pub_target.publish(pt)
    
    def publish_status(self):
        status = {
            'fps': round(self.fps, 1),
            'mode': self.detect_mode,
            'source': 'camera' if self.use_camera else 'simulation',
            'cv2': CV2_AVAILABLE,
            'n_detections': len(self.detections),
            'detections': [d.get('label') for d in self.detections],
        }
        msg = String()
        msg.data = json.dumps(status)
        self.pub_status.publish(msg)
        self.get_logger().info(f'Vision | {round(self.fps,1)} fps | {len(self.detections)} objects')
    
    def __del__(self):
        if self.cap:
            self.cap.release()
        if CV2_AVAILABLE and self.show_window:
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
