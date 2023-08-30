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

    annotated_frame = annotate(image_source=image_source, boxes=boxes, logits=logits, phrases=phrases)
    # Save annotated image
    cv2.imwrite(f"data/labelled/{img_name}", annotated_frame)


detect_obstacle("g_bl_ca_2.jpg")

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
'''