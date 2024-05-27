import cv2
import numpy as np
import dlib
from collections import deque
import time

class Node:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    @classmethod
    def instance(cls):
        return cls.__new__(cls)

    def handle(self, svm):
        raise NotImplementedError

class CameraReadNode(Node):
    def __init__(self, width, height, top_margin, bottom_margin):
        self.top_margin = top_margin
        self.bottom_margin = bottom_margin

        # 두 개의 USB 카메라를 사용합니다.
        self.left_cap = cv2.VideoCapture(2)  # 첫 번째 카메라
        self.right_cap = cv2.VideoCapture(4)  # 두 번째 카메라

        self.left_cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.left_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.right_cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.right_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def handle(self, svm):
        return1, svm.left_frame = self.left_cap.read()
        return2, svm.right_frame = self.right_cap.read()
        
        if return1 and return2:
            svm.left_cropped_frame = svm.left_frame[self.top_margin:-self.bottom_margin, :]
            svm.left_cropped_frame = cv2.cvtColor(svm.left_cropped_frame, cv2.COLOR_BGR2GRAY)
        else:
            svm.change_state(FailedNode.instance()) 
            return
        if not return1 or not return2:
            svm.change_state(ResultSenderNode.instance())
            return
        if svm.tracker_available:
            svm.change_state(RoiFaceInFrameNode.instance())
            return
        else:
            svm.change_state(DetectFaceInFrameNode.instance())
            return

class DetectFaceInFrameNode(Node):
    def __init__(self):
        pass

    def handle(self, svm):
        faces = svm.detector(svm.left_cropped_frame, 1)
        if faces:
            face = faces[0]
            x = face.left()
            y = face.top()
            w = face.right() - x
            h = face.bottom() - y
            
            svm.mk_face_bbox(x,y,w,h)
            svm.mk_roi_bbox(bbox=svm.face_bbox, roi_box_pad=(50,50))
            svm.mk_eyes_center()
            if svm.mk_template_bbox(template_box_size=(60,120)):
                svm.change_state(InitTrackerNode.instance())
                return
        svm.change_state(FailedNode.instance())
        return

class InitTrackerNode(Node):
    def handle(self, svm):
        svm.tracker_available = True
        svm.tracker = cv2.legacy.TrackerMedianFlow_create() 
        svm.tracker.init(svm.left_cropped_frame, svm.template_bbox)
        svm.count = 0
        svm.change_state(TemplateMatchingNode.instance())
        return

class UpdateTrackerNode(Node):
    def __init__(self, width, height, top_margin, bottom_margin, max_count=40):
        self.height = height - top_margin - bottom_margin
        self.width = width
        self.max_count = max_count

    def handle(self, svm):
        svm.count += 1
        if svm.count > self.max_count:
            svm.change_state(FailedNode.instance())
            return
        success, bbox = svm.tracker.update(svm.left_cropped_frame)
        svm.face_bbox = None
        if not success:
            svm.change_state(FailedNode.instance())
            return
        x, y, w, h = map(int, bbox)
        if x < 5 or y < 5 or x > self.width-5 or y > self.height-5:
            svm.change_state(FailedNode.instance())
            return
        svm.template_bbox = (x, y, w, h)
        svm.mk_roi_bbox(svm.template_bbox, (50, 50)) 
        svm.eyes_center = (x + w // 2, y + h // 2)
        svm.change_state(TemplateMatchingNode.instance())
        return

class TemplateMatchingNode(Node):
    def __init__(self, width, height, top_margin, bottom_margin, matching_rate):
        self.top_margin = top_margin
        self.matching_rate = matching_rate

    def handle(self, svm):
        x, y, w, h = svm.template_bbox
        template_image = svm.left_cropped_frame[y:y + h, x:x + w]
        
        target_image = svm.right_frame[y + self.top_margin:y + self.top_margin + h, :]
        target_image = cv2.cvtColor(target_image, cv2.COLOR_BGR2GRAY)
        res = cv2.matchTemplate(target_image, template_image, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

        if svm.DEBUG:
            # DEBUG
            cv2.rectangle(target_image, max_loc, (max_loc[0] + w, max_loc[1] + h), (255, 255, 0), 2)
            cv2.imshow("target_image", target_image)
            cv2.imshow("template_image", template_image)

        if max_val < self.matching_rate:
            svm.result = None
            svm.change_state(ResultSenderNode.instance())
            return

        cx2 = max_loc[0] + w // 2
        svm.distance = cx2 - svm.eyes_center[0]
        svm.change_state(FilterNode.instance())
        return

class FilterNode(Node):
    def __init__(self, queue_size=10, smoothing_factor=0.4, EFL=4, sensor_width_mm=1.5, baseline_m=0.05, px_width=640, px_height=480):
        self.f_px = (EFL / sensor_width_mm) * px_width
        self.baseline_m = baseline_m
        self.cx = px_width / 2
        self.cy = px_height / 2
        
        self.distance_values = deque(maxlen=queue_size)
        self.smoothing_factor = smoothing_factor
        self.filtered_distance = None
        self.move_distance = 0
        self.average_distance = 0
        
        self.handle = self.initial_handle

    def initial_handle(self, svm):
        self.handle = self.regular_handle

        self.distance_values.append(svm.distance)
        self.filtered_distance = svm.distance
        
        svm.filtered_distance = self.filtered_distance
        filtered_Z = (self.f_px * self.baseline_m) / abs(self.filtered_distance) if self.filtered_distance != 0 else 0

        Z_m = filtered_Z
        X_m = (svm.eyes_center[0] - self.cx) * filtered_Z / self.f_px
        Y_m = (svm.eyes_center[1] - self.cy) * filtered_Z / self.f_px

        svm.result = [X_m, Y_m, Z_m]
        svm.change_state(ResultSenderNode.instance())

    def regular_handle(self, svm):
        self.distance_values.append(svm.distance)

        if len(self.distance_values) == self.distance_values.maxlen:
            values_array = np.array(self.distance_values)
            kth_smallest = 3
            kth_largest = len(values_array) - 3
            partitioned_array = np.partition(values_array, kth_smallest)
            trimmed_array_small = partitioned_array[kth_smallest:]
            partitioned_array = np.partition(trimmed_array_small, kth_largest - kth_smallest - 1)
            trimmed_array = partitioned_array[:kth_largest - kth_smallest]
            self.average_distance = np.mean(trimmed_array)
            self.move_distance = self.average_distance - self.filtered_distance
        self.filtered_distance += self.move_distance * self.smoothing_factor
                
        svm.filtered_distance = self.filtered_distance
        filtered_Z = (self.f_px * self.baseline_m) / abs(self.filtered_distance) if self.filtered_distance != 0 else 0

        Z_m = filtered_Z
        X_m = (svm.eyes_center[0] - self.cx) * filtered_Z / self.f_px
        Y_m = (svm.eyes_center[1] - self.cy) * filtered_Z / self.f_px

        svm.result = [X_m, Y_m, Z_m]
        svm.change_state(ResultSenderNode.instance())

class RoiFaceInFrameNode(Node):
    def __init__(self):
        pass

    def handle(self, svm):
        roi_x, roi_y, roi_w, roi_h = svm.roi_bbox
        roi_frame = svm.left_cropped_frame[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]

        faces = svm.detector(roi_frame, 1)

        if faces:
            face = faces[0]
            
            x = face.left() + roi_x
            y = face.top() + roi_y
            w = face.right() - face.left()
            h = face.bottom() - face.top()

            svm.mk_face_bbox(x, y, w, h)
            svm.mk_roi_bbox(bbox=svm.face_bbox, roi_box_pad=(50, 50))
            svm.mk_eyes_center()
            if svm.mk_template_bbox(template_box_size=(60, 120)):
                svm.change_state(InitTrackerNode.instance())
            else:
                svm.change_state(FailedNode.instance())
        else:
            svm.change_state(UpdateTrackerNode.instance())

class DebugNode(Node):
    def __init__(self, width, height, top_margin, bottom_margin):
        self.width = width
        self.height = height
        self.top_margin = top_margin
        self.bottom_margin = bottom_margin
        self.data_view = None

    def draw_label(self, img, text, pos, bg_color, text_color, font, font_scale, thickness):
        text_size, _ = cv2.getTextSize(text, font, font_scale, thickness)
        text_w, text_h = text_size
        cv2.rectangle(img, pos, (pos[0] + text_w, pos[1] - text_h - 3), bg_color, -1)
        cv2.putText(img, text, (pos[0], pos[1] - 3), font, font_scale, text_color, thickness)

    def handle(self, svm):
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 2
        color_text = (255, 255, 255)
        color_bg = (0, 0, 0)
        if svm.result is not None:
            X_m, Y_m, Z_m = svm.result
            scale = 200
            self.data_view = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            EFL = 1.57
            sensor_width_mm = 4.54
            baseline_m = 0.1
            px_width = 640
            f_px = (EFL / sensor_width_mm) * px_width
            ori_Z = (f_px * baseline_m) / abs(svm.distance) if svm.distance != 0 else 0
            cv2.circle(self.data_view, (int(ori_Z * scale), 280), 10, (255, 200, 50), -1) 
            self.draw_label(self.data_view, f"original Z: {ori_Z:.4f}", (int(ori_Z * scale), 280), (255, 200, 50), color_text, font, 0.5, thickness)

            cv2.circle(self.data_view, (int(Z_m * scale), 200), 10, (255, 0, 0), -1) 
            self.draw_label(self.data_view, f"filtered Z: {Z_m:.4f}", (int(Z_m * scale), 200), (255, 0, 0), color_text, font, 0.5, thickness)
            
            cv2.circle(self.data_view, (int(X_m * scale) + 320, 200), 10, (0, 0, 255), -1)  
            self.draw_label(self.data_view, f"X : {X_m:.4f}", (int(X_m * scale) + 320, 200), (0, 0, 255), color_text, font, 0.5, thickness)
            
            cv2.circle(self.data_view, (320, int(Y_m * scale) + 240), 10, (0, 255, 0), -1) 
            self.draw_label(self.data_view, f"Y : {Y_m:.4f}", (320, int(Y_m * scale) + 240), (0, 255, 0), color_text, font, 0.51, thickness)

            self.draw_label(self.data_view, f"X : {X_m:.4f}", (20, 40), color_bg, color_text, font, 1, thickness)
            self.draw_label(self.data_view, f"Y : {Y_m:.4f}", (20, 80), color_bg, color_text, font, 1, thickness)
            self.draw_label(self.data_view, f"Z-original : {ori_Z:.4f}", (20, 120), color_bg, color_text, font, 1, thickness)
            self.draw_label(self.data_view, f"Z-filtered : {Z_m:.4f}", (20, 160), color_bg, color_text, font, 1, thickness)
            self.draw_label(self.data_view, f"distance : {svm.distance:.4f}", (220, 40), color_bg, color_text, font, 1, thickness)
            self.draw_label(self.data_view, f"filtered : {svm.filtered_distance:.4f}", (220, 80), color_bg, color_text, font, 1, thickness)
        else:
            self.data_view = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        cv2.imshow("data_view", self.data_view)
        debug_frame = svm.left_frame[self.top_margin:-self.bottom_margin, :]

        self.draw_label(debug_frame, "tracking cnt", (20, 30), color_bg, color_text, font, 1, thickness)
        self.draw_label(debug_frame, str(svm.count), (20, 80), color_bg, color_text, font, 2, thickness)

        if svm.roi_bbox is not None:
            x, y, w, h = svm.roi_bbox
            color_bg = (255, 255, 0)
            color_text = (0, 0, 0)
            cv2.rectangle(debug_frame, (x, y), (x + w, y + h), color_bg, 2)
            self.draw_label(debug_frame, "ROI", (x, y), color_bg, color_text, font, font_scale, 1)

        if svm.face_bbox is not None:
            x, y, w, h = svm.face_bbox
            color_bg =  (0, 255, 0)
            color_text = (0, 0, 0)
            cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            self.draw_label(debug_frame, "Face", (x, y), color_bg, color_text, font, font_scale, 1)

        if svm.template_bbox is not None:
            x, y, w, h = svm.template_bbox
            color_bg = (0, 255, 255)
            color_text = (0, 0, 0)
            cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
            self.draw_label(debug_frame, "Template", (x, y), color_bg, color_text, font, font_scale, 1)

        if svm.eyes_center is not None:
            x, y = svm.eyes_center
            color_bg = (0, 0, 255)
            color_text = (0, 0, 0)
            cv2.circle(debug_frame, (x, y), 20, (0, 0, 255), -1)
            self.draw_label(debug_frame, "Eyes", (x - 10, y - 10), color_bg, color_text, font, font_scale, 1)

        cv2.imshow("DEBUG", debug_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            svm.running = False

        svm.change_state(CameraReadNode.instance())

class FailedNode(Node):
    def handle(self, svm):
        svm.tracker_available = False
        svm.face_bbox = None
        svm.roi_bbox = None
        svm.template_bbox = None
        svm.eyes_center = None
        svm.founded_target = None

        svm.count = 0
        svm.result = None
        svm.change_state(ResultSenderNode.instance())

class ResultSenderNode(Node):
    def __init__(self):
        self.fps = 0
        self.last_time = time.time()

    def format_list(self, my_list):
        if my_list is None:
            return '0/0/0/0'
        else:
            formatted_list = '/'.join(['{:.4f}'.format(item) for item in my_list])
            return f'1/{formatted_list}'

    def handle(self, svm):
        current_time = time.time()
        time_diff = current_time - self.last_time
        self.fps = 1 / time_diff
        self.last_time = current_time
        print(f"FPS: {self.fps:.2f}, 거리 결과 >>>", svm.result)
        
        if svm.DEBUG:
            svm.change_state(DebugNode.instance())
        else:
            svm.change_state(CameraReadNode.instance())

class StereoVisionMachine:
    def __init__(self, initial_state, width, height, top_margin, bottom_margin, debug=False):
        self.width = width
        self.height = height
        self.top_margin = top_margin
        self.bottom_margin = bottom_margin

        self.state = initial_state
        self.running = True

        self.left_frame = None
        self.right_frame = None
        self.left_cropped_frame = None
        self.right_cropped_frame = None

        self.tracker = None
        self.tracker_available = False
        self.face_bbox = None
        self.roi_bbox = None
        self.template_bbox = None
        self.eyes_center = None

        self.count = 0
        self.result = None

        self.distance = None
        self.founded_target = None

        self.filtered_distance = None
        self.ori_Z = None
        self.filtered_Z = None

        self.DEBUG = debug
        self.detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor('/home/sang/catkin_ws/src/golf_bot/test_module/shape_predictor_68_face_landmarks.dat')

    def mk_face_bbox(self, x, y, w, h):
        self.face_bbox = (x, y, w, h)

    def mk_roi_bbox(self, bbox, roi_box_pad):
        x, y, w, h = bbox
        rbox_w, rbox_h = roi_box_pad
        img_width, img_height = self.left_cropped_frame.shape[1], self.left_cropped_frame.shape[0]

        init_roi_x = x - rbox_w
        init_roi_y = y - rbox_h
        init_roi_w = w + rbox_w * 2
        init_roi_h = h + rbox_h * 2

        adjusted_roi_x = max(init_roi_x, 0)
        adjusted_roi_y = max(init_roi_y, 0)

        adjusted_roi_w = min(init_roi_w, img_width - adjusted_roi_x)
        adjusted_roi_h = min(init_roi_h, img_height - adjusted_roi_y)

        if adjusted_roi_x == 0 and x < rbox_w:
            adjusted_roi_w -= (rbox_w - x)
        if adjusted_roi_y == 0 and y < rbox_h:
            adjusted_roi_h -= (rbox_h - y)

        self.roi_bbox = [adjusted_roi_x, adjusted_roi_y, adjusted_roi_w, adjusted_roi_h]

    def mk_eyes_center(self):
        x, y, w, h = self.face_bbox
        face_rect_dlib = dlib.rectangle(left=x, top=y, right=x+w, bottom=y+h)
        landmarks = self.predictor(self.left_cropped_frame, face_rect_dlib)
        left_eye_pts = np.array([[landmarks.part(n).x, landmarks.part(n).y] for n in range(36, 42)])
        right_eye_pts = np.array([[landmarks.part(n).x, landmarks.part(n).y] for n in range(42, 48)])
        left_eye_center = left_eye_pts.mean(axis=0)
        right_eye_center = right_eye_pts.mean(axis=0)
        self.eyes_center = np.mean([left_eye_center, right_eye_center], axis=0).astype(int)

    def mk_template_bbox(self, template_box_size):
        cx, cy = self.eyes_center
        w, h = template_box_size
        
        height, width = self.left_cropped_frame.shape[:2]

        if cx < 2 or cy < 2 or cx > width-2 or cy > height-2:
            return False

        max_w = min(cx, width - cx) * 2
        max_h = min(cy, height - cy) * 2
    
        adjusted_w = min(w, max_w)
        adjusted_h = min(h, max_h)
        x_start = cx - adjusted_w // 2
        y_start = cy - adjusted_h // 2
        self.template_bbox = (x_start, y_start, adjusted_w, adjusted_h)
        return True

    def mk_template_bbox2(self, template_box_size):
        fx, fy, fw, fh = self.face_bbox
        cx, cy = self.eyes_center

        new_w = max(fw - 10, 1)
        new_h = fh + 20

        height, width = self.left_cropped_frame.shape[:2]

        if cx < 0 or cy < 0 or cx > width or cy > height:
            return False

        x_start = max(cx - new_w // 2, 0)
        y_start = max(cy - new_h // 2, 0)

        if x_start + new_w > width:
            x_start = width - new_w
        if y_start + new_h > height:
            y_start = height - new_h

        self.template_bbox = (x_start, y_start, new_w, new_h)
        return True

    def change_state(self, state):
        self.state = state

    def run(self):
        while self.running:
            self.state.handle(self)

if __name__ == "__main__":
    crop_values = (640, 480, 50, 1)
    CameraReadNode(*crop_values)
    UpdateTrackerNode(*crop_values, max_count=40)
    TemplateMatchingNode(*crop_values, matching_rate=0.5)
    DebugNode(*crop_values)
    FilterNode(queue_size=10, smoothing_factor=0.4, EFL=1.57, sensor_width_mm=4.54, baseline_m=0.1, px_width=640, px_height=480)
    ResultSenderNode()

    init_state = CameraReadNode.instance()
    stereoVision = StereoVisionMachine(init_state, *crop_values, debug=True)
    stereoVision.run()
    cv2.destroyAllWindows()