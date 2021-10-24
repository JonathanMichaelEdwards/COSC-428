import torch
import torch.nn as nn
import torch.optim as optim
import time
import os
import numpy as np
import requests
import io
import hashlib
import urllib
from copy import deepcopy

from PIL import Image
from torch.utils.data import Dataset, DataLoader
from torchvision import models, transforms
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from engine import train_one_epoch, evaluate
import utils

from label_studio.ml import LabelStudioMLBase
from label_studio.ml.utils import get_single_tag_keys, get_choice, is_skipped


device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')


IMAGE_SIZE = 224
IMAGE_TRANSFORMS = transforms.Compose([
    transforms.Resize(IMAGE_SIZE),
    transforms.CenterCrop(IMAGE_SIZE),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])
IMAGE_CACHE_DIR = os.path.join(os.path.dirname(__file__), 'image-cache')
os.makedirs(IMAGE_CACHE_DIR, exist_ok=True)


def get_transformed_image(url):
    # Prepare the images for loading into the neural network.
    is_local_file = url.startswith('/data')
    if is_local_file:
        # filename, dir_path = url.split('/data/')[1].split('?d=')
        dir_path, filename = os.path.split('labelstudio/' + url.split('/data/')[1])
        dir_path = str(urllib.parse.unquote(dir_path))
        filepath = os.path.join(dir_path, filename)
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Cannot find {filepath} in {os.getcwd}")
        with open(filepath, mode='rb') as f:
            image = Image.open(f).convert('RGB')
    else:
        cached_file = os.path.join(IMAGE_CACHE_DIR, hashlib.md5(url.encode()).hexdigest())
        if os.path.exists(cached_file):
            with open(cached_file, mode='rb') as f:
                image = Image.open(f).convert('RGB')
        else:
            r = requests.get(url, stream=True)
            r.raise_for_status()
            with io.BytesIO(r.content) as f:
                image = Image.open(f).convert('RGB')
            with io.open(cached_file, mode='wb') as fout:
                fout.write(r.content)
    return IMAGE_TRANSFORMS(image)

class ObjectDetectorDataset(Dataset):
    # This class handles the data on behalf of the neural network. The two key 
    # functions here are __getitem__() and __len__(). These must be implemented
    # to allow the neural network to load in the data. The specifics of what
    # the __getitem__() function returns depends on the network you are using.
    def __init__(self, image_urls, image_classes, image_boxes, box_areas, image_ids, model_class_names):
        self.classes = list(set(model_class_names)) # Remove any duplicate class names.
        self.class_to_label = {c: i for i, c in enumerate(self.classes)}  # Create a mapping from index to class name.

        self.image_ids = image_ids
        self.images, self.labels, self.boxes, self.box_areas = [], [], [], []
        # Generate the data that the dataset will serve up on calls to the __getitem__() function.
        for image_url, image_class, image_box, box_areas in zip(image_urls, image_classes, image_boxes, box_areas):
            try:
                image = get_transformed_image(image_url)
            except Exception as exc:
                print(exc)
                continue
            self.labels.append([])
            self.boxes.append([])
            self.box_areas.append([])
            self.images.append(image)
            for class_, box, box_area in zip(image_class, image_box, box_areas):
                self.labels[-1].append(self.class_to_label[class_])
                self.boxes[-1].append(box)
                self.box_areas[-1].append(box_area)

    def __getitem__(self, index):
        # Return an item for the neural network to train on.
        target = {}
        target['boxes'] = torch.as_tensor(self.boxes[index],  dtype=torch.float32)
        target['labels'] = torch.as_tensor(self.labels[index], dtype=torch.int64)
        target['image_id'] = torch.as_tensor(self.image_ids[index], dtype=torch.int64)
        target['area'] = torch.as_tensor(self.box_areas[index], dtype=torch.float32)
        target['iscrowd'] = torch.zeros((len(self.boxes),), dtype=torch.int64)

        return self.images[index], target

    def __len__(self):
        return len(self.images)

class ObjectDetector(object):
    # This is the wrapper for the neural network.
    def __init__(self, num_classes, freeze_extractor=False, model_path=None):
        self.model = models.detection.fasterrcnn_resnet50_fpn(pretrained=True)

        if freeze_extractor:
            print('Transfer learning with a fixed ConvNet feature extractor')
            for param in self.model.parameters():
                param.requires_grad = False
        else:
            print('Transfer learning with a full ConvNet finetuning')

        in_features = self.model.roi_heads.box_predictor.cls_score.in_features
        self.model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes) 
        self.model = self.model.to(device)

        self.optimizer = optim.SGD(self.model.parameters(), lr=0.001, momentum=0.9)

        self.scheduler = optim.lr_scheduler.StepLR(self.optimizer, step_size=5, gamma=0.1)

    def save(self, path):
        torch.save(self.model.state_dict(), path)

    def load(self, path):
        self.model.load_state_dict(torch.load(path))
        self.model.to(device)

    def predict(self, image_urls):
        images = torch.stack([get_transformed_image(url) for url in image_urls])
        images = images.to(device)

        with torch.no_grad():
            self.model.eval()
            predictions = self.model(images)

        return predictions

    def train(self, dataloader, num_epochs=5):
        for epoch in range(num_epochs):
            train_one_epoch(self.model, self.optimizer, dataloader, device, epoch,
            print_freq=10)
            self.scheduler.step()

        return self.model


class ObjectDetectorAPI(LabelStudioMLBase):
    # This is the class through which LabelStudio interacts with the neural network.
    def __init__(self, freeze_extractor=False, **kwargs):
        super(ObjectDetectorAPI, self).__init__(**kwargs)
        self.from_name, self.to_name, self.value, self.classes = get_single_tag_keys(
            self.parsed_label_config, 'RectangleLabels', 'Image')

        self.freeze_extractor = freeze_extractor
        if self.train_output:
            print("Loading trained model.")
            self.classes = self.train_output['classes']
            self.model = ObjectDetector(len(self.classes), freeze_extractor)
            self.model.load(self.train_output['model_path'])
        else:
            self.model = ObjectDetector(len(self.classes), freeze_extractor)

    def reset_model(self):
        self.model = ObjectDetector(len(self.classes), self.freeze_extractor)

    def predict(self, tasks, **kwargs):
        # This is used to assist with annotating data in the case of a partially
        # trained network.
        # This class returns a prediction dictionary. This needs to be changed depending
        # on the kind of network you're training, and the data you're working with.
        # In this case, we're training a RectangleLabels object.
        # The following URL may help if you're trying to change this https://labelstud.io/guide/export.html
        image_urls = [task['data'][self.value] for task in tasks]
        image_ids = [task['id'] for task in tasks]

        raw_prediction = self.model.predict(image_urls)
        predictions = []

        for image in range(len(image_urls)):
            labels = raw_prediction[image]["labels"].tolist()
            scores = raw_prediction[image]["scores"].tolist()
            boxes =  raw_prediction[image]["boxes"].tolist()
            result = []
            for i in range(len(labels)):
                predicted_label = self.classes[labels[i]]
                result.append({
                    'from_name': self.from_name,
                    'to_name': self.to_name,
                    'id': image_ids[image],
                    "type": "rectanglelabels",
                    'score': float(scores[i]),
                    "value": {
                        "height":boxes[i][3] - boxes[i][1],
                        "width":boxes[i][2] - boxes[i][0],
                        "rotation": 0,
                        "x":boxes[i][0],
                        "y":boxes[i][1],
                        "rectanglelabels": [predicted_label],
                    }
                })
            predictions.append({'result': deepcopy(result)})

        return predictions

    def fit(self, completions, workdir=None, batch_size=4, num_epochs=30, **kwargs):
        # This function is called when the "Start Training" button is selected
        # in the LabelStudio GUI frontend. It loads the model (trained, if available)
        # and then begins or continues the training process as appropriate.
        # NOTE: If the ML backend process is closed after training, LabelStudio
        # will lose track of the trained model as it is only stored in RAM. It is
        # up to you to change this if it is relevant to your project.
        print("BATCH SIZE", batch_size)
        image_urls, image_classes, image_boxes, box_areas, image_ids = [], [], [], [], []
        print('Collecting completions...')
        for completion in completions:
            if is_skipped(completion):
                continue
            image_urls.append(completion['data'][self.value])
            image_classes.append([])
            image_boxes.append([])
            box_areas.append([])
            image_ids.append(completion['id'])

            for result in completion['completions'][0]['result']:
                x0 =      result['value']['x']
                y0 =      result['value']['y']
                x1 = x0 + result['value']['width']
                y1 = y0 + result['value']['height']
                box_areas[-1].append(result['value']['width'] * result['value']['height'])
                image_boxes[-1].append([x0, y0, x1, y1])
                image_classes[-1].append(result['value']['rectanglelabels'][0])

        print('Creating dataset...')
        dataset = ObjectDetectorDataset(image_urls, image_classes, image_boxes, box_areas, image_ids, self.classes)
        dataloader = DataLoader(dataset, shuffle=True, batch_size=batch_size, collate_fn=utils.collate_fn)

        print('Train model...')
        self.reset_model()
        self.model.train(dataloader, num_epochs=num_epochs)

        print('Save model...')
        model_path = os.path.join(workdir, 'model.pt')
        self.model.save(model_path)

        self.train_output = {}
        self.train_output['model_path'] = model_path
        self.train_output['classes'] = dataset.classes

        return {'model_path': model_path, 'classes': dataset.classes}
