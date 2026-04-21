# Vision Node Plan

## Goal

Build a ROS 2 vision node that reads frames from the host camera, runs YOLO-based detection, publishes structured detection results, and is easy for students to use from `robot/main.py`.

This phase targets the current working custom full YOLOv4 model already present in `Project-NUEVO/YOLOv4`.

This phase does not include model retraining, tiny-model conversion, or a complete face/gender classifier.

## Critical Gaps / Questions to Resolve

Resolved for this phase:

**Camera Initialization & Robustness**
- The node should behave like `rplidar_c1_node`: if `/dev/video10` is missing at startup, log a clear warning and retry until the camera appears.
- If the camera disappears during runtime, the node should return to the reconnect loop instead of exiting.
- Camera settings should be parameters, not hardcoded assumptions. Resolution, frame rate target, and device path should all be configurable.
- Model load failure is a startup configuration error and should fail the node.
- Camera open/read failure is a runtime availability problem and should retry.

**Docker Dependencies**
- Add `python3-opencv` explicitly in the Docker image even though it is currently present through transitive ROS dependencies.
- Do not add `Pillow` for this phase.
- Do not add `scipy` for this phase.

**Model Asset Availability**
- The current YOLO assets must be available inside the ROS runtime container.
- Runtime model files should not depend on the untracked top-level `YOLOv4/` directory being visible in Docker by accident.
- Keep `yolov4.cfg` and `obj.names` in `ros2_ws/src/vision/data/` and install them with the `vision` package.
- Do not track YOLO weights in Git. Students should copy their local weights to `ros2_ws/src/vision/data/yolov4.weights`.
- The default Docker runtime should load weights from `ros2_ws/src/vision/data/yolov4.weights`.

**Timestamp Handling**
- Use ROS node time when a frame is successfully captured.
- Do not pretend to have a camera-native hardware timestamp through OpenCV for this phase.

**Frame Rate Mismatch**
- The node should use a single synchronous inference loop.
- No frame queueing is needed for this phase.
- If inference is slower than the camera frame rate, intermediate camera frames are effectively dropped and the publish rate becomes inference-limited.

**Error Recovery**
- If inference fails on one frame, log the error and continue with the next frame.
- Only unrecoverable configuration problems should terminate the node.

**Observability Without Debug Images**
- Add lightweight runtime logging:
  - model loaded
  - camera connected/disconnected
  - frame size
  - inference latency
  - detection count
- Use throttled logging where needed so normal runtime stays readable.

Deferred for later:

**Testing Strategy**
- Camera-less testing can be improved later by allowing a video-file input path, but this is not required for the first implementation pass.

**Hot-Reload Parameters**
- `confidence_threshold`, `nms_threshold`, and similar inference parameters require a node restart for this phase.

## Scope For This Phase

Implement:

- a `vision` ROS 2 runtime node in `ros2_ws/src/vision`
- direct camera capture with OpenCV from `/dev/video10`
- YOLO detection using the existing custom full YOLOv4 assets
- a generic detection message in `bridge_interfaces`
- a ROS topic that publishes all detections in a student-friendly format
- robot-side helper methods so student code can use detections without raw ROS subscription code
- code structure for ROI-based post-processing
- traffic-light color analysis structure and student TODO path

Defer:

- model retraining
- YOLOv4-tiny or alternative model migration
- face detection with a working model
- gender classification implementation
- training documentation
- custom-object training workflow documentation

## Current Assets We Will Use

The current working traffic model configuration lives in `ros2_ws/src/vision/data`:

- `ros2_ws/src/vision/data/yolov4.cfg`
- `ros2_ws/src/vision/data/obj.names`

The weights file is not tracked by Git. Place it manually at:

- `ros2_ws/src/vision/data/yolov4.weights`

Current classes in `obj.names`:

- `crosswalk`
- `speedlimit`
- `stop`
- `trafficlight`

This means the first working runtime supports traffic-related detections only.

## High-Level Architecture

```text
/dev/video10
    |
    v
vision_node.py
    |
    +--> capture frame with cv2
    +--> run YOLO detector
    +--> run ROI post-processing for selected classes
    +--> publish /vision/detections
    |
    v
robot.py helper methods
    |
    v
student main.py decision logic
```

## Package Responsibilities

### `vision`

Owns:

- camera capture
- model loading
- frame inference loop
- ROI post-processing
- detection publishing
- node parameters

Does not own:

- model training
- long-term object tracking
- robot behavior logic

### `bridge_interfaces`

Owns the shared vision messages so other packages can consume detections without importing `vision` internals.

### `robot`

Owns student-facing helpers that expose detections in a simple API.

## Detection Message Design

Add the following messages to `bridge_interfaces`.

### `VisionDetection.msg`

```text
string class_name
float32 confidence
uint32 x
uint32 y
uint32 width
uint32 height
string[] attribute_names
string[] attribute_values
float32[] attribute_scores
```

### `VisionDetectionArray.msg`

```text
std_msgs/Header header
uint32 image_width
uint32 image_height
VisionDetection[] detections
```

## Why This Message Shape

It covers the current and future use cases without adding many task-specific messages.

Examples:

- traffic light:
  - `class_name="trafficlight"`
  - `attribute_names=["color"]`
  - `attribute_values=["red"]`

- stop sign:
  - `class_name="stop"`

- future face result:
  - `class_name="face"`
  - `attribute_names=["gender"]`
  - `attribute_values=["female"]`

- future custom object:
  - `class_name="my_object"`

This keeps the message stable while allowing new attributes later.

## ROS Topic Contract

Primary output topic:

- `/vision/detections` using `bridge_interfaces/msg/VisionDetectionArray`

Initial behavior:

- publish the current frame's detections
- publish an empty array when no objects are detected
- use the frame timestamp in the header

Not included in this phase:

- debug image topic
- compressed image topic
- tracking IDs

## Runtime Flow

The node loop should look like this:

1. Open camera `/dev/video10` with OpenCV
2. Read a frame
3. Convert frame into YOLO input blob
4. Run the detector
5. Decode class IDs, confidences, and boxes
6. Apply NMS
7. Build generic detection messages
8. For supported classes, crop ROI and enrich attributes
9. Publish `VisionDetectionArray`

## Reuse Strategy From Existing YOLO Code

We will reuse logic from `Project-NUEVO/YOLOv4`, but not run the old scripts directly.

### Reuse

- helper functions from `YOLOv4/yolo_utils.py`
- class loading
- blob conversion
- YOLO forward pass helper logic
- NMS helper logic
- drawing and debug patterns where useful
- traffic-light example structure from `YOLOv4/task2.py`

### Do Not Reuse Directly

- the old top-level app loop in `YOLOv4/main.py`
- `Picamera2`
- CUDA-only setup assumptions
- video recording flow

The ROS node must own the runtime loop and use `/dev/video10`.

## ROI Post-Processing Design

The node should support a consistent pattern:

1. Detect object with YOLO
2. Crop the detection bounding box as ROI
3. Run class-specific analysis on the ROI
4. Attach results as attributes on the generic detection message

### Traffic Light

Detection source:

- current YOLO model detects `trafficlight`

ROI analysis:

- crop the traffic-light box
- inspect color regions inside the crop
- set one of:
  - `red`
  - `green`
  - `other`

Implementation approach:

- include a clean helper function for traffic-light color analysis
- preserve student-facing TODO structure inspired by `YOLOv4/task2.py`

### Face

This phase will not ship a working face detector because the current model does not contain a `face` class.

What we will include:

- code structure for a future face ROI pipeline
- clear placeholder for a second detector or model profile
- comments or notes describing where student logic would run after face ROI extraction

What we will not include:

- a working face model
- gender classification logic
- a SciPy runtime dependency unless needed later

## Model Configuration Strategy

The node should load model assets from configurable paths instead of hardcoding one model forever.

Initial default configuration:

- cfg: `share/vision/data/yolov4.cfg`
- weights: `ros2_ws/src/vision/data/yolov4.weights`
- names: `share/vision/data/obj.names`

This prepares the system for later swapping to:

- YOLOv4-tiny
- another custom YOLO model
- a future face-capable model

## Proposed Node Parameters

The node should expose parameters similar to:

- `camera_device`
- `model_cfg_path`
- `model_weights_path`
- `class_names_path`
- `confidence_threshold`
- `nms_threshold`
- `publish_rate_hz`
- `enable_traffic_light_color`

These parameters should be sufficient for normal runtime changes without touching code.

## Robot Integration Plan

The student should not need to subscribe directly to `/vision/detections`.

Add a subscriber and helper methods in `robot.py` so student code can do things like:

- `robot.get_detections()`
- `robot.has_detection("stop")`
- `robot.get_detection_attribute("trafficlight", "color")`

This follows the existing style already used for other sensors.

## Student Experience Goal

Student `main.py` should be able to ask simple questions about detections and make decisions from them.

Example usage target:

```python
if robot.has_detection("stop"):
    robot.stop()

light_color = robot.get_detection_attribute("trafficlight", "color")
if light_color == "red":
    robot.stop()
```

## Docker And Runtime Integration

During implementation, the Docker runtime will need to build the `vision` package as part of normal startup.

This plan assumes:

- camera frames come from `/dev/video10`
- the container already has access to that device through the current host-camera pipeline
- the Docker image explicitly includes `python3-opencv`

This plan does not add another ROS image source.

## Phase Boundaries

### Done When

This phase is complete when:

- the `vision` node runs inside the ROS 2 runtime container
- it opens `/dev/video10`
- it loads the existing full YOLOv4 custom model
- it publishes structured detections on `/vision/detections`
- traffic-light detections can carry a color attribute
- `robot.py` can expose simple detection helper methods
- a student can use the detection results from `main.py`

### Not Required For Completion

- tiny-model support
- model retraining
- face detection working end-to-end
- gender classification
- custom training instructions

## Pending Decisions

None that block implementation for this phase.

Later decisions, outside this phase:

- which future model replaces the current full YOLOv4 weights
- whether face detection becomes part of the main detector or a separate model
- whether to add a debug-image output topic
