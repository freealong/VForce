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
PROJECT_PATH = os.path.join(SCRIPT_PATH, os.pardir, os.pardir, os.pardir)
MODEL_PATH = os.path.join(PROJECT_PATH, "model", "mask_rcnn.h5")

class InferenceConfig(wpif.WPIFConfig):
    # Set batch size to 1 since we'll be running inference on
    # one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1

config = InferenceConfig()
#config.display()

# Create model object in inference mode.
model = modellib.MaskRCNN(mode="inference", config=config, model_dir='')

# Load weights trained on MS-COCO
model.load_weights(MODEL_PATH, by_name=True)
print("Load weights finished")

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
    import skimage.io
    image = skimage.io.imread(os.path.join(PROJECT_PATH, 'build', 'test_data', 'color1.png'))
    results = detect(image)
    print(len(results))
