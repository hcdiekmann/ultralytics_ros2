
import cv2
import torch
import random
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from cv_bridge import CvBridge

from ultralytics import YOLO
from ultralytics.tracker import BOTSORT, BYTETracker
from ultralytics.tracker.trackers.basetrack import BaseTrack
from ultralytics.yolo.utils import IterableSimpleNamespace, yaml_load
from ultralytics.yolo.utils.checks import check_requirements, check_yaml

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import Detection2DArray
from std_srvs.srv import SetBool


class Yolov8Node(Node):
    # Node constructor
    def __init__(self) -> None:
        super().__init__("yolov8_node")

        # node params
        self.declare_parameter("model", "yolov8s.pt")
        model = self.get_parameter(
            "model").get_parameter_value().string_value

        self.declare_parameter("tracker", "bytetrack.yaml")
        tracker = self.get_parameter(
            "tracker").get_parameter_value().string_value

        self.declare_parameter("device", "cpu")
        device = self.get_parameter(
            "device").get_parameter_value().string_value

        self.declare_parameter("conf_threshold", 0.5)
        self.conf_threshold = self.get_parameter(
            "conf_threshold").get_parameter_value().double_value
        
        self.declare_parameter("iou_threshold", 0.7)
        self.iou_threshold = self.get_parameter(
            "iou_threshold").get_parameter_value().double_value

        self.declare_parameter("enable", True)
        self.enable = self.get_parameter(
            "enable").get_parameter_value().bool_value
        
        self.declare_parameter("input_image_topic", "image_raw")
        input_image_topic = self.get_parameter(
            "input_image_topic").get_parameter_value().string_value
        
        self.declare_parameter("show_inference_image", True)
        self.show_inference_image = self.get_parameter(
            "show_inference_image").get_parameter_value().bool_value

        self._class_to_color = {}
        self.cv_bridge = CvBridge()
        self.tracker = self.create_tracker(tracker)
        self.yolo = YOLO(model)
        self.yolo.fuse()
        self.yolo.to(device)

        # topci publishers & subscribers
        self._detect_pub = self.create_publisher(Detection2DArray, "detections", 10)
        self._infer_pub = self.create_publisher(Image, "inference_image", 10)
        self._tracking_pub = self.create_publisher(Image, "tracking_image", 10)
        self._image_sub = self.create_subscription(
            Image, input_image_topic, self.image_cb,
            qos_profile_sensor_data
        )

        # services & clients
        self._srv = self.create_service(SetBool, "enable", self.enable_cb)


    def create_tracker(self, tracker_yaml) -> BaseTrack:

        TRACKER_MAP = {"bytetrack": BYTETracker, "botsort": BOTSORT}
        check_requirements("lap")  # for linear_assignment

        tracker = check_yaml(tracker_yaml)
        config = IterableSimpleNamespace(**yaml_load(tracker))

        assert config.tracker_type in ["bytetrack", "botsort"], \
            f"Only support 'bytetrack' and 'botsort' for now, but got '{config.tracker_type}'"
        tracker = TRACKER_MAP[config.tracker_type](args=config, frame_rate=1)
        return tracker


    def enable_cb(self,
                  req: SetBool.Request,
                  res: SetBool.Response
                  ) -> SetBool.Response:
        self.enable = req.data
        res.success = True
        return res


    def image_cb(self, msg: Image) -> None:

        if self.enable:
            # record start time
            fps_start_t = time.perf_counter()

            # convert to cv image & predict
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg)

            infer_start_t = time.perf_counter()
            results = self.yolo.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                show=self.show_inference_image,
                mode="track"
            )
            end_time = time.perf_counter()
            infer_time = (end_time - infer_start_t) * 1000
            
            # visualize the results on the frame
            annotated_image = results[0].plot()

            # record the end time and calculate FPS
            fps = 1.0 / (end_time - fps_start_t)
            cv2.putText(annotated_image, "FPS: {:.2f}".format(fps), (10, 30), 
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            # cv2.putText(annotated_image, "Inference Time: {:.2f} ms".format(infer_time), (10, 60), 
            # cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

            # get bounding boxes for tracking
            det = results[0].boxes.cpu().numpy()
            if len(det) > 0:
                im0s = self.yolo.predictor.batch[2]
                im0s = im0s if isinstance(im0s, list) else [im0s]

                tracks = self.tracker.update(det, im0s[0])
                if len(tracks) > 0:
                    results[0].update(boxes=torch.as_tensor(tracks[:, :-1]))

            # create detections msg
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header

            results = results[0].cpu()

            for b in results.boxes:

                label = self.yolo.names[int(b.cls)]
                score = float(b.conf)

                if score < self.conf_threshold:
                    continue

                detection = Detection2D()

                box = b.xywh[0]

                # get boxes values
                detection.bbox.center.position.x = float(box[0])
                detection.bbox.center.position.y = float(box[1])
                detection.bbox.size_x = float(box[2])
                detection.bbox.size_y = float(box[3])

                # create track id
                track_id = ""
                if not b.id is None:
                    track_id = str(int(b.id))
                detection.id = track_id

                # get hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = label
                hypothesis.hypothesis.score = score
                detection.results.append(hypothesis)

                # draw boxes for debugging
                if label not in self._class_to_color:
                    r = random.randint(0, 255)
                    g = random.randint(0, 255)
                    b = random.randint(0, 255)
                    self._class_to_color[label] = (r, g, b)
                color = self._class_to_color[label]

                min_pt = (round(detection.bbox.center.position.x - detection.bbox.size_x / 2.0),
                          round(detection.bbox.center.position.y - detection.bbox.size_y / 2.0))
                max_pt = (round(detection.bbox.center.position.x + detection.bbox.size_x / 2.0),
                          round(detection.bbox.center.position.y + detection.bbox.size_y / 2.0))
                cv2.rectangle(cv_image, min_pt, max_pt, color, 2)

                label = "{} ({}) ({:.3f})".format(label, str(track_id), score)
                pos = (min_pt[0] + 5, min_pt[1] + 25)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(cv_image, label, pos, font,
                            1, color, 1, cv2.LINE_AA)

                # append msg
                detections_msg.detections.append(detection)

            # publish detections and dbg image
            self._detect_pub.publish(detections_msg)
            self._infer_pub.publish(self.cv_bridge.cv2_to_imgmsg(annotated_image, encoding=msg.encoding))
            self._tracking_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image, encoding=msg.encoding))


def main():
    rclpy.init()
    node = Yolov8Node()
    rclpy.spin(node)
    rclpy.shutdown()
