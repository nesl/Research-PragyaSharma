import cv2
from ultralytics import YOLO
import numpy as np
import pandas as pd
import time

directory = "/Users/pragyasharma/Documents/GitHub/Research-PragyaSharma/pybullet_scripts_personal_copy/mpc_python-master/mpc_pybullet_demo/"
save_file = directory+"yolov8_inference_latency.csv"

videos = ['traffic_low.mp4', 'traffic_mod.mp4', 'traffic_high.mp4']

df_columns = ['Low env complexity', 'Mod env complexity', 'High env complexity']
df = pd.DataFrame(columns=df_columns)

low_inf = []
mod_inf = []
high_inf = []

for v in videos:
    inf_speed = []
    cap = cv2.VideoCapture(v)
    model = YOLO("yolov8m.pt")

    start_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model(frame) #, device="mps"
        result = results[0]
        bboxes = np.array(result.boxes.xyxy.cpu(), dtype="int")
        classes = np.array(result.boxes.cls.cpu(), dtype="int")
        # print("Speed:", result.speed['inference'])
        inf_speed.append(result.speed['inference'])
        # print("inf_speed: ", inf_speed)
        for cls, bbox in zip(classes, bboxes):
            (x, y, x2, y2) = bbox
            cv2.rectangle(frame, (x, y), (x2, y2), (0, 0, 225), 2)
            cv2.putText(frame, str(cls), (x, y - 5), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 225), 2)

        cv2.imshow("Img", frame)
        key = cv2.waitKey(1)
        if key == 27:
            break

        if time.time()-start_time > 10:
            break
    
    if 'low' in v:
        low_inf = inf_speed[6:]
        # print(low_inf)
    elif 'mod' in v:
        mod_inf = inf_speed
    elif 'high' in v:
        high_inf = inf_speed
    cap.release()
    cv2.destroyAllWindows()

inf_data = {'Low env complexity': low_inf, 'Mod env complexity': mod_inf, 'High env complexity': high_inf}
df = pd.concat([df, pd.DataFrame([inf_data])], ignore_index=True)
df.to_csv(save_file, encoding='utf-8', index=True)