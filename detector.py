"""
Author: Colin La
Date modified: 30/8/23
Module containing code to interface detector 
Code used and running from: https://github.com/IDEA-Research/GroundingDINO/tree/60d796825e1266e56f7e4e9e00e88de662b67bd3 
"""

# Instructions for installing
"""
git clone https://github.com/IDEA-Research/GroundingDINO.git
cd GroundingDINO/
pip install -e .

mkdir weights
cd weights
wget -q https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth
cd ..

## IF NOT DONE ALREADY
mkdir data
# Have all images saved in data folder

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

from GroundingDINO.groundingdino.util.inference import load_image, predict, annotate#, load_model
import glob


def detect_obstacle(img_name, prompt="obstacle", box_thresh=0.3, text_thresh=0.25):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    def load_model(model_config_path, model_checkpoint_path, cpu_only=False):
        args = SLConfig.fromfile(model_config_path)
        args.device = "cuda" if not cpu_only else "cpu"
        model = build_model(args)
        checkpoint = torch.load(model_checkpoint_path, map_location="cpu")
        load_res = model.load_state_dict(clean_state_dict(checkpoint["model"]), strict=False)
        print(load_res)
        _ = model.eval()
        return model

    model = load_model("GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py", "GroundingDINO/weights/groundingdino_swint_ogc.pth",cpu_only=True)
    #model = model.to(device)

    # Set image name
    #img_name = "g_bl_ca_2.jpg"
    img_path = f"data/{img_name}"

    # Add prompt and thresholds
    #prompt = "obstacle"
    #box_thresh = 0.35
    #text_thresh = 0.25

    # Load image
    image_source, image = load_image(img_path)

    # Run model
    boxes, logits, phrases = predict(
        model=model,
        image=image,
        caption=prompt,
        box_threshold=box_thresh,
        text_threshold=text_thresh,
        device = device
    )
    """
    # Boxes gives the bounding boxes 
    # Object 1, Object 2, Object 3
    tensor([[0.1101, 0.1870, 0.2189, 0.2451],
        [0.5630, 0.4205, 0.2442, 0.2291],
        [0.5469, 0.2940, 0.1496, 0.0888]])

    # Logits gives the probabilities
    tensor([0.5044, 0.5221, 0.3507])

    # Phrases gives the text detections from the text prompts
    """
    annotated_frame = annotate(image_source=image_source, boxes=boxes, logits=logits, phrases=phrases)

    # Save annotated image
    cv2.imwrite(f"data/labelled/{img_name}", annotated_frame)
    return boxes, image.shape


detect_obstacle("g_bl_ca_2.jpg")
#detect_obstacle("sample_arena.png", prompt="red sticker", box_thresh=0.4)

## Function to run on all images
def detect_all_images(prompt, box_thresh, text_thresh):
    "Gives bounding boxes for all objects in an image following file path data/{imgs}.jpg"
    for img_path_s in glob.glob("data/*.jpg"):
        img_path = img_path_s.replace(os.sep, '/')
        image_source, image = load_image(img_path)

        boxes, logits, phrases = predict(
            model=model,
            image=image,
            caption=prompt,
            box_threshold=box_thresh,
            text_threshold=text_thresh,
            device = device
        )


        annotated_frame = annotate(image_source=image_source, boxes=boxes, logits=logits, phrases=phrases)
        cv2.imwrite(f"data/labelled/{img_path_s[5:]}", annotated_frame)



### Colour detection
'''
# Load image 
filename = "foo.jpg" 
image=cv2.imread(filename)
image=imutils.resize(image,width=530,height=350)
'''
# Functions
def get_colour(filename, x=0, y=0):
    """
    Returns red, green, or blue based on single pixel coordinate value
    """
    image=cv2.imread(filename)
    # Resize image for easier use
    image=imutils.resize(image,width=530,height=350) 

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
