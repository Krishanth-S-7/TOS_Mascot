import cv2 as cv
import mediapipe as mp
import serial
import pyrealsense2 as rs
import numpy as np
import time
import math

# Arduino serial port
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

# Mediapipe Face Mesh
myfacemesh = mp.solutions.face_mesh
facemesh = myfacemesh.FaceMesh(max_num_faces=1)  # allow multiple faces
mpDraw = mp.solutions.drawing_utils

# ---------- Intel RealSense setup ----------
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # RGB
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)   # Depth

# Start streaming
pipeline.start(config)
                                                                  
ptime = 0
last_cmd = None
tracked_bbox = None  # bounding box of the currently tracked face
last_face_time = time.time()
face_start_time  = None  # ✅ track last time a face was seen

# ---------------- Helper function ----------------
def compute_iou(boxA, boxB):
    """Compute IoU between two bboxes (x1,y1,x2,y2)."""
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])

    interArea = max(0, xB - xA) * max(0, yB - yA)
    if interArea == 0:
        return 0.0

    boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])

    iou = interArea / float(boxAArea + boxBArea - interArea)
    return iou

try:
    while True:
        # Get frameset
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame:
            continue

        # Convert RealSense image to numpy array
        img = np.asanyarray(color_frame.get_data())

        imgrgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        results = facemesh.process(imgrgb)

        face_to_track = None
        
        h, w = img.shape[:2]

        # print(face_start_time, time.time())

  
        if results.multi_face_landmarks:
            # if face_start_time is None:
            #     face_start_time = time.time()
            
            last_face_time = time.time()  # ✅ reset timer when face is seen
            candidates = []
            # Collect all faces with bounding boxes + landmarks
            for facelms in results.multi_face_landmarks:
                xs = [lm.x * w for lm in facelms.landmark]
                ys = [lm.y * h for lm in facelms.landmark]
                x_min, x_max = int(min(xs)), int(max(xs))
                y_min, y_max = int(min(ys)), int(max(ys))
                bbox = (x_min, y_min, x_max, y_max)

                candidates.append((bbox, facelms))

            # If we already have a tracked face, find best IoU match
            if tracked_bbox is not None:
                best_iou = 0
                best_face = None
                for bbox, facelms in candidates:
                    iou = compute_iou(tracked_bbox, bbox)
                    if iou > best_iou:
                        best_iou = iou
                        best_face = (bbox, facelms)

                if best_iou > 0.3:
                    face_to_track = best_face
                else:
                    face_to_track = max(candidates, key=lambda x: (x[0][2]-x[0][0])*(x[0][3]-x[0][1]))
            else:
                face_to_track = max(candidates, key=lambda x: (x[0][2]-x[0][0])*(x[0][3]-x[0][1]))

            
        # ✅ If no face for >3 seconds → stop motors
        elif time.time() - last_face_time > 3:
            face_start_time = None
            if last_cmd != 'C':
                arduino.write(b'C')
                print("No face detected >3s → STOP")
                last_cmd = 'C'

        if face_to_track:
            bbox, facelms = face_to_track
            tracked_bbox = bbox

            mpDraw.draw_landmarks(img, facelms, myfacemesh.FACEMESH_TESSELATION)

            # compute mean (cx, cy)
            xs = [lm.x * w for lm in facelms.landmark]
            ys = [lm.y * h for lm in facelms.landmark]
            cx, cy = int(sum(xs) / len(xs)), int(sum(ys) / len(ys))
            cx = max(0, min(cx, w - 1))
            cy = max(0, min(cy, h - 1))


            # Get depth value at face center
            depth = None
            if depth_frame:
                depth = depth_frame.get_distance(cx, cy)

            # thresholds
            left = w // 3
            right = 2 * left
            depth_lower  = 0.6
            depth_upper = 1
            pwm_max = 200
            k = 0.6 # Diff for LR, FB?

            if cx < left:
                pwm = int(pwm_max*(1-math.exp(-k*(left- cx))))
                cmd = f"L,{pwm}\n"
            elif cx > right:
                pwm = int(pwm_max*(1-math.exp(-k*(cx- right))))
                cmd = f"R,{pwm}\n"
            else:
                
                
                if depth>depth_upper:
                    pwm = int(pwm_max*(1-math.exp(-k*(depth- depth_upper))))
                    cmd = f"F,{pwm}\n"   
                    
                elif depth < depth_lower:
                    pwm = int(pwm_max*(1-math.exp(-k*(depth_lower- depth))))
                    cmd = f"B,{pwm}"



                else:
                    cmd = "C, 0"
                    if face_start_time is None:
                        face_start_time = time.time()
                    
                    elif time.time() - face_start_time>3:
                        cmd = "S,0" 
                        
            

            


            if cmd != last_cmd:
                if pwm!= 60:
                    arduino.write(cmd.encode())
                    print(f"Sent command: {cmd}")  # debug print
                    last_cmd = cmd

            # Draw center point and bbox
            cv.circle(img, (cx, cy), 5, (0, 255, 0), -1)
            cv.rectangle(img, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (255, 0, 0), 2)

            if depth:
                cv.putText(img, f"Depth: {depth:.2f}m", (cx, cy - 10),
                           cv.FONT_HERSHEY_PLAIN, 1, (0, 255, 255), 2)

        # FPS calculation
        ctime = time.time()
        fps = 1 / (ctime - ptime) if ctime != ptime else 0
        cv.putText(img, f"FPS: {int(fps)}", (30, 50),
                   cv.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 2)
        ptime = ctime

        cv.imshow("RealSense Video", img)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv.destroyAllWindows()
