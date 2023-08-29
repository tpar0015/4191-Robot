"""
Author: Colin La
Date modified: 23/8/23
Module containing code to interface detector 
"""

# Import Libraries
import numpy as np
import pandas as pd
import cv2
import matplotlib.pyplot as plt
import imutils
from PIL import Image

# Grounding DINO needs image as tensor and text prompt
# Grounding DINO returns bounding boxes
import os, sys
sys.path.append(os.path.join(os.getcwd(), "GroundingDINO"))

import argparse
import copy

from IPython.display import display
from PIL import Image #, ImageDraw, ImageFont
from torchvision.ops import box_convert

# Grounding DINO
import GroundingDINO.groundingdino.datasets.transforms as T
from GroundingDINO.groundingdino.models import build_model
from GroundingDINO.groundingdino.util import box_ops
from GroundingDINO.groundingdino.util.slconfig import SLConfig
from GroundingDINO.groundingdino.util.utils import clean_state_dict, get_phrases_from_posmap
from GroundingDINO.groundingdino.util.inference import annotate, load_image, predict
import supervision as sv
import numpy as np
import requests
import torch
from io import BytesIO
from huggingface_hub import hf_hub_download

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


# Loading model, only needs to run once
def load_model_hf(repo_id, filename, ckpt_config_filename, device='cpu'):
    cache_config_file = hf_hub_download(repo_id=repo_id, filename=ckpt_config_filename)

    args = SLConfig.fromfile(cache_config_file)
    args.device = device
    model = build_model(args)

    cache_file = hf_hub_download(repo_id=repo_id, filename=filename)
    checkpoint = torch.load(cache_file, map_location=device)
    log = model.load_state_dict(clean_state_dict(checkpoint['model']), strict=False)
    print("Model loaded from {} \n => {}".format(cache_file, log))
    _ = model.eval()
    return model

ckpt_repo_id = "ShilongLiu/GroundingDINO"
ckpt_filenmae = "groundingdino_swinb_cogcoor.pth"
ckpt_config_filename = "GroundingDINO_SwinB.cfg.py"


groundingdino_model = load_model_hf(ckpt_repo_id, ckpt_filenmae, ckpt_config_filename, device)

# detect object using grounding DINO
def detect(image, text_prompt, model, box_threshold = 0.3, text_threshold = 0.25, device="cuda"):
  boxes, logits, phrases = predict(
      model=model,
      image=image,
      caption=text_prompt,
      box_threshold=box_threshold,
      text_threshold=text_threshold,
      device=device
  )

  annotated_frame = annotate(image_source=image_source, boxes=boxes, logits=logits, phrases=phrases)
  annotated_frame = annotated_frame[...,::-1] # BGR to RGB
  return annotated_frame, boxes

import groundingdino.datasets.transforms as T
def load_image(image_path: str):
    transform = T.Compose(
        [
            #T.RandomResize([800], max_size=1333),
            T.ResizeDebug((504, 378)),
            T.ToTensor(),
            T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ]
    )
    image_source = Image.open(image_path).convert("RGB")
    image = np.asarray(image_source)
    image_transformed, _ = transform(image_source, None)
    return image, image_transformed

def get_bbox(img_path):
    _, image = load_image(img_path)

    annotated_frame, detected_boxes = detect(image, text_prompt="green", model=groundingdino_model, device=device)
    return annotated_frame, detected_boxes




##############################################################################
# Basic set up for transforming coordinates 
bounding_box = 1
image_coords = bounding_box
camera_to_image = 1 # Transformation matrix for camera to iamge
cam_coords = image_coords / camera_to_image
world_to_camera = 1# Transformation matrix for world to camera
world_coords = cam_coords / world_to_camera




# Bounding boxes as [x_centre,y_centre,width,height] 
# Module to use camera matrix to estimate poses 
def estimate_pose():
    camera_matrix = camera_matrix
    focal_length = camera_matrix[0][0]
    target_dimensions = []

    # Measure obstacles 
    obstacle_dimensions = [0.075448, 0.074871, 0.071889]

    target_dimensions.append(obstacle_dimensions)

    target_pose_dict = {}
    for waypoint in way_points.keys():
            
        robot_pose = current_robot_pose # completed_img_dict[target_num]['robot'] 

        for i in range(len(way_points[waypoint]['target'])):
            box = bounding_box
            true_height = target_dimensions[waypoint-1][2]
            im_x=640

            dist=focal_length*true_height/box[3]
            y_cam=-(box[0]-im_x/2)*dist/focal_length
            x_cam=dist
            th=robot_pose[2]
            rob_y=robot_pose[1]
            rob_x=robot_pose[0]
            x_world=x_cam*np.cos(th)-y_cam*np.sin(th)
            y_world=x_cam*np.sin(th)+y_cam*np.cos(th)
            x_world=rob_x+x_world
            y_world=rob_y+y_world

            target_pose={'y': (y_world).item(), 'x': (x_world).item()}

    return target_pose



# Detector Class
class Detector: 
    def __init__(self, ckpt, gpu_use=False)
        self.model = 
        if torch.cuda.torch.cuda.device_count() > 0 and use_gpu:
            self.use_gpu = True
            self.model = self.model.cuda()
        else:
            self.use_gpu = False
        self.load_weights(ckpt)




# Load image 
filename = "foo.jpg" 
image=cv2.imread(filename)
image=imutils.resize(image,width=530,height=350)

# Functions
def get_colour(image, x=0, y=0):
    """
    Returns red, green, or blue based on single pixel coordinate value
    """
    height, width, _ = image.shape
    if x == 0 or y == 0:
        x = int(width / 2)
        y = int(height / 2)
    
    hue_val = image[y, x][0]

    if hue_val < 5:
        colour = "red"
    elif hue_val < 78:
        colour = "green"
    elif hue_val < 131:
        colour = "blue"
    else:
        colour = "red"
        
    return colour


get_colour(image)