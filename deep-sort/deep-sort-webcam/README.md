## Installation

Start with [**Python>=3.8**](https://www.python.org/) environment.

If you want to run the YOLOv8, YOLO-NAS or YOLOX examples:

```
git clone https://github.com/mikel-brostrom/yolo_tracking.git
cd yolo_tracking
pip install -v -e .
```

but if you only want to import the tracking modules you can simply:

```
pip install boxmot
```

## YOLOv8 | YOLO-NAS | YOLOX examples

<details>
<summary>Tracking</summary>

<details>
<summary>Yolo models</summary>



```bash
$ python examples/track.py --yolo-model yolov8n       # bboxes only
  python examples/track.py --yolo-model yolo_nas_s    # bboxes only
  python examples/track.py --yolo-model yolox_n       # bboxes only
                                        yolov8n-seg   # bboxes + segmentation masks
                                        yolov8n-pose  # bboxes + pose estimation

```

  </details>

<details>
<summary>Tracking methods</summary>

```bash
$ python examples/track.py --tracking-method deepocsort
                                             strongsort
                                             ocsort
                                             bytetrack
                                             botsort
```

</details>

<details>
<summary>Tracking sources</summary>

Tracking can be run on most video formats

```bash
$ python examples/track.py --source 0                               # webcam
                                    img.jpg                         # image
                                    vid.mp4                         # video
                                    path/                           # directory
                                    path/*.jpg                      # glob
                                    'https://youtu.be/Zgi9g1ksQHc'  # YouTube
                                    'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP stream
```

</details>

<details>
<summary>Select ReID model</summary>

Some tracking methods combine appearance description and motion in the process of tracking. For those which use appearance, you can choose a ReID model based on your needs from this [ReID model zoo](https://kaiyangzhou.github.io/deep-person-reid/MODEL_ZOO). These model can be further optimized for you needs by the [reid_export.py](https://github.com/mikel-brostrom/yolo_tracking/blob/master/boxmot/deep/reid_export.py) script

```bash
$ python examples/track.py --source 0 --reid-model lmbn_n_cuhk03_d.pt               # lightweight
                                                   osnet_x0_25_market1501.pt
                                                   mobilenetv2_x1_4_msmt17.engine
                                                   resnet50_msmt17.onnx
                                                   osnet_x1_0_msmt17.pt
                                                   clip_market1501.pt               # heavy
                                                   clip_vehicleid.pt
                                                   ...
```

</details>

<details>
<summary>Filter tracked classes</summary>

By default the tracker tracks all MS COCO classes.

If you want to track a subset of the classes that you model predicts, add their corresponding index after the classes flag,

```bash
python examples/track.py --source 0 --yolo-model yolov8s.pt --classes 16 17  # COCO yolov8 model. Track cats and dogs, only
```

[Here](https://tech.amikelive.com/node-718/what-object-categories-labels-are-in-coco-dataset/) is a list of all the possible objects that a Yolov8 model trained on MS COCO can detect. Notice that the indexing for the classes in this repo starts at zero

</details>

<details>
<summary>MOT compliant results</summary>

Can be saved to your experiment folder `runs/track/exp*/` by

```bash
python examples/track.py --source ... --save-mot
```

</details>

</details>

<details>
<summary>Evaluation</summary>

Evaluate a combination of detector, tracking method and ReID model on standard MOT dataset or you custom one by

```bash
$ python3 examples/val.py --yolo-model yolo_nas_s.pt --reid-model osnetx1_0_dukemtcereid.pt --tracking-method deepocsort --benchmark MOT16
                          --yolo-model yolox_n.pt    --reid-model osnet_ain_x1_0_msmt17.pt  --tracking-method ocsort     --benchmark MOT17
                          --yolo-model yolov8s.pt    --reid-model lmbn_n_market.pt          --tracking-method strongsort --benchmark <your-custom-dataset>
```

</details>

<details>
<summary>Evolution</summary>

We use a fast and elitist multiobjective genetic algorithm for tracker hyperparameter tuning. By default the objectives are: HOTA, MOTA, IDF1. Run it by

```bash
$ python examples/evolve.py --tracking-method strongsort --benchmark MOT17 --n-trials 100  # tune strongsort for MOT17
                            --tracking-method ocsort     --benchmark <your-custom-dataset> --objective HOTA # tune ocsort for maximizing HOTA on your custom tracking dataset
```

The set of hyperparameters leading to the best HOTA result are written to the tracker's config file.

</details>


## Custom object detection model tracking example

<details>
<summary>Minimalistic</summary>

```python
import cv2
import numpy as np
from pathlib import Path

from boxmot import DeepOCSORT


tracker = DeepOCSORT(
    model_weights=Path('osnet_x0_25_msmt17.pt'), # which ReID model to use
    device='cuda:0',
    fp16=False,
)

vid = cv2.VideoCapture(0)

while True:
    ret, im = vid.read()

    # substitute by your object detector, output has to be N X (x, y, x, y, conf, cls)
    dets = np.array([[144, 212, 578, 480, 0.82, 0],
                    [425, 281, 576, 472, 0.56, 65]])

    tracks = tracker.update(dets, im) # --> (x, y, x, y, id, conf, cls, ind)
```

</details>


<details>
<summary>Complete</summary>

```python
import cv2
import numpy as np
from pathlib import Path

from boxmot import DeepOCSORT


tracker = DeepOCSORT(
    model_weights=Path('osnet_x0_25_msmt17.pt'), # which ReID model to use
    device='cuda:0',
    fp16=True,
)

vid = cv2.VideoCapture(0)
color = (0, 0, 255)  # BGR
thickness = 2
fontscale = 0.5

while True:
    ret, im = vid.read()

    # substitute by your object detector, input to tracker has to be N X (x, y, x, y, conf, cls)
    dets = np.array([[144, 212, 578, 480, 0.82, 0],
                    [425, 281, 576, 472, 0.56, 65]])

    tracks = tracker.update(dets, im) # --> (x, y, x, y, id, conf, cls, ind)

    xyxys = tracks[:, 0:4].astype('int') # float64 to int
    ids = tracks[:, 4].astype('int') # float64 to int
    confs = tracks[:, 5]
    clss = tracks[:, 6].astype('int') # float64 to int
    inds = tracks[:, 7].astype('int') # float64 to int

    # in case you have segmentations or poses alongside with your detections you can use
    # the ind variable in order to identify which track is associated to each seg or pose by:
    # segs = segs[inds]
    # poses = poses[inds]
    # you can then zip them together: zip(tracks, poses)

    # print bboxes with their associated id, cls and conf
    if tracks.shape[0] != 0:
        for xyxy, id, conf, cls in zip(xyxys, ids, confs, clss):
            im = cv2.rectangle(
                im,
                (xyxy[0], xyxy[1]),
                (xyxy[2], xyxy[3]),
                color,
                thickness
            )
            cv2.putText(
                im,
                f'id: {id}, conf: {conf}, c: {cls}',
                (xyxy[0], xyxy[1]-10),
                cv2.FONT_HERSHEY_SIMPLEX,
                fontscale,
                color,
                thickness
            )

    # show image with bboxes, ids, classes and confidences
    cv2.imshow('frame', im)

    # break on pressing q
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows()
```

</details>

<details>
<summary>Tiled inference</summary>
  
```py
from sahi import AutoDetectionModel
from sahi.predict import get_sliced_prediction
import cv2
import numpy as np
from pathlib import Path
from boxmot import DeepOCSORT


tracker = DeepOCSORT(
    model_weights=Path('osnet_x0_25_msmt17.pt'), # which ReID model to use
    device='cpu',
    fp16=False,
)

detection_model = AutoDetectionModel.from_pretrained(
    model_type='yolov8',
    model_path='yolov8n.pt',
    confidence_threshold=0.5,
    device="cpu",  # or 'cuda:0'
)

vid = cv2.VideoCapture(0)
color = (0, 0, 255)  # BGR
thickness = 2
fontscale = 0.5

while True:
    ret, im = vid.read()

    # get sliced predictions
    result = get_sliced_prediction(
        im,
        detection_model,
        slice_height=256,
        slice_width=256,
        overlap_height_ratio=0.2,
        overlap_width_ratio=0.2
    )
    num_predictions = len(result.object_prediction_list)
    dets = np.zeros([num_predictions, 6], dtype=np.float32)
    for ind, object_prediction in enumerate(result.object_prediction_list):
        dets[ind, :4] = np.array(object_prediction.bbox.to_xyxy(), dtype=np.float32)
        dets[ind, 4] = object_prediction.score.value
        dets[ind, 5] = object_prediction.category.id

    tracks = tracker.update(dets, im) # --> (x, y, x, y, id, conf, cls, ind)

    if tracks.shape[0] != 0:

        xyxys = tracks[:, 0:4].astype('int') # float64 to int
        ids = tracks[:, 4].astype('int') # float64 to int
        confs = tracks[:, 5].round(decimals=2)
        clss = tracks[:, 6].astype('int') # float64 to int
        inds = tracks[:, 7].astype('int') # float64 to int

        # print bboxes with their associated id, cls and conf
        for xyxy, id, conf, cls in zip(xyxys, ids, confs, clss):
            im = cv2.rectangle(
                im,
                (xyxy[0], xyxy[1]),
                (xyxy[2], xyxy[3]),
                color,
                thickness
            )
            cv2.putText(
                im,
                f'id: {id}, conf: {conf}, c: {cls}',
                (xyxy[0], xyxy[1]-10),
                cv2.FONT_HERSHEY_SIMPLEX,
                fontscale,
                color,
                thickness
            )

    # show image with bboxes, ids, classes and confidences
    cv2.imshow('frame', im)

    # break on pressing q
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows()
```

</details>

