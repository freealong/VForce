import os
import sys
print(sys.version)
sys.argv = ['.']

import numpy as np
import tensorflow as tf
print("All modules imported")

SCRIPT_PATH = os.path.dirname(os.path.realpath(__file__))
PROJECT_PATH = os.path.join(SCRIPT_PATH, os.pardir)

detection_graph = tf.Graph()

with detection_graph.as_default():
    sess = tf.Session(graph=detection_graph)

def load_image_into_numpy_array(image):
    (im_width, im_height) = image.size
    return np.array(image.getdata()).reshape(
        (im_height, im_width, 3)).astype(np.uint8)

# load a Tensofflow model into memory
def load_model(model):
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        print("Detector:: load model from ", model)
        with tf.gfile.GFile(model, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
        print("Detector:: load model successfully")

def detect(image_np, num=5, min_score=0.85):
    print('Detector: run detecting...')
    width = image_np.shape[1]
    height = image_np.shape[0]
    res = detect_with_rotation(image_np, num=num, min_score=min_score)
    if len(res) > 1:
        return res
    # rotation 180
    print("Detector: rotation 180")
    boxes = []
    for box in res:
        boxes.append(box)
    image_np = np.rot90(image_np, 2);
    rotated_res = detect_with_rotation(image_np, num=num, min_score=min_score)
    for box in rotated_res:
        boxes.append([width-box[2]+1, height-box[3]+1, width-box[0]+1, height-box[1]+1])
    return boxes

def detect_with_rotation(image_np, num=5, min_score=0.9):
    width = image_np.shape[1]
    height = image_np.shape[0]
    image_np_expanded = np.expand_dims(image_np, axis=0)
    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
    # Each box represents a part of the image where a particular object was detected.
    boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
    # Each score represent how level of confidence for each of the objects.
    # Score is shown on the result image, together with the class label.
    scores = detection_graph.get_tensor_by_name('detection_scores:0')
    classes = detection_graph.get_tensor_by_name('detection_classes:0')
    num_detections = detection_graph.get_tensor_by_name('num_detections:0')
    # Actual detection.
    (boxes, scores, classes, num_detections) = sess.run(
    [boxes, scores, classes, num_detections],
    feed_dict={image_tensor: image_np_expanded})
    res = []
    for i in range(min(boxes.size, num)):
        if scores[0][i] < min_score:
            return res
        raw_box =  list(boxes[0][i])
        box = [round(raw_box[1]*width), round(raw_box[0]*height), round(raw_box[3]*width), round(raw_box[2]*height)]
        # make sure x in [1, width], y in [1, height]
        box[0] = max(box[0], 1)
        box[1] = max(box[1], 1)
        box[2] = min(box[2], width)
        box[3] = min(box[3], height)
        res.append(box)
    return res;

if __name__ == '__main__':
    import timeit
    from PIL import Image
    apps_root = os.path.join(PROJECT_PATH, 'apps')
    image = Image.open(os.path.join(apps_root, 'data', 'color_pad1.png'))
    image_np = load_image_into_numpy_array(image)
    load_model(os.path.join(apps_root, 'pick_pad', 'model', 'frcnn.pb'))
    start = timeit.default_timer()
    box = detect(image_np)
    stop = timeit.default_timer()
    print("detection time: ", stop - start)
    print(box)
