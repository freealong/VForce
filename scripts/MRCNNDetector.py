import os
import sys
print(sys.version)
sys.argv = ['.']

# @TODO: Remove some depenences
sys.path.append('/home/yongqi/projects/Mask_RCNN')
import wpif
import model as modellib
print("All module imported")

SCRIPT_PATH = os.path.dirname(os.path.realpath(__file__))
PROJECT_PATH = os.path.join(SCRIPT_PATH, os.pardir)

class InferenceConfig(wpif.WPIFConfig):
    # Set batch size to 1 since we'll be running inference on
    # one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1

config = InferenceConfig()
#config.display()

# Create model object in inference mode.
model = modellib.MaskRCNN(mode="inference", config=config, model_dir='')

# Load weights
def load_model(model_weights):
    print("Detector: load model from ", model_weights)
    model.load_weights(model_weights, by_name=True)
    print("Detector: load model successfully")

def detect(image_np, max_num=5, min_score=0.9):
    r = model.single_detect(image_np, min_score)
    results = []
    for i in range(min(max_num, r['scores'].shape[0])):
        res = []
        roi = r['rois'][i]
        res.append([roi[1], roi[0], roi[3] - roi[1], roi[2] - roi[0]])
        res.append(r['masks'][i])
        res.append(r['class_ids'][i])
        results.append(res)
    return results

if __name__ == '__main__':
    apps_root = os.path.join(PROJECT_PATH, 'apps')
    load_model(os.path.join(apps_root, 'pick_pad', 'model', 'mrcnn.h5'))
    import skimage.io
    image = skimage.io.imread(os.path.join(apps_root, 'data', 'color_pad1.png'))
    results = detect(image)
    print(len(results))
