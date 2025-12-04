#!/usr/bin/env python3
# ocr_utils.py (TrOCR Version)
#
# OCR utility module using Microsoft's TrOCR (VisionEncoderDecoderModel).
#
# Usage:
#   from anafi_ros_nodes.ocr_utils import OCRProcessor
#   ocr = OCRProcessor()
#   text, pre_img = ocr.recognize_model(image)

import cv2
import time
import numpy as np
import torch
from transformers import TrOCRProcessor, VisionEncoderDecoderModel
from PIL import Image

class OCRProcessor:
    """
    OCR processor using Microsoft's TrOCR.
    """

    def __init__(
        self,
        model_name: str = "microsoft/trocr-base-printed",
        use_gpu: bool = True,
        preprocess: bool = True,
        logger=None
    ):
        self.logger = logger
        self.preprocess = preprocess
        self.device = torch.device("cuda" if use_gpu and torch.cuda.is_available() else "cpu")

        # Load TrOCR
        try:
            self.processor = TrOCRProcessor.from_pretrained(model_name)
            self.model = VisionEncoderDecoderModel.from_pretrained(model_name).to(self.device)
            self._log(f"TrOCR loaded: {model_name}")
        except Exception as e:
            self._log(f"Failed to load TrOCR model: {e}", error=True)
            raise RuntimeError(e)

    def _log(self, msg, error=False):
        if self.logger:
            if error:
                self.logger.error(msg)
            else:
                self.logger.info(msg)
        else:
            print(msg)

    # --------------------------------------------------------
    # Very light preprocessing: grayscale + OTSU threshold
    # --------------------------------------------------------
    def _preprocess_image(self, image: np.ndarray) -> np.ndarray:
        return image

    # --------------------------------------------------------
    # Main OCR function using TrOCR
    # --------------------------------------------------------
    def recognize_model(self, image: np.ndarray):
        """
        Performs OCR using TrOCR.
        Returns: (text, preprocessed_image)
        """
        if image is None or image.size == 0:
            return "", image

        # Preprocessing
        pre_img = image
        if self.preprocess:
            pre_img = self._preprocess_image(image)

        # Convert to PIL
        pil_img = Image.fromarray(pre_img)

        # Run TrOCR
        try:
            pixel_values = self.processor(pil_img, return_tensors="pt").pixel_values.to(self.device)
            start_time = time.time()

            with torch.no_grad():
                generated_ids = self.model.generate(pixel_values)
                
            
            end_time = time.time()
            infer_ms = (end_time - start_time) * 1000  # ms
            text = self.processor.batch_decode(generated_ids, skip_special_tokens=True)[0]
            text = text.strip()

            self._log(f"TrOCR inference time: {infer_ms:.2f} ms")
            return text, pre_img

        except Exception as e:
            self._log(f"TrOCR inference failed: {e}", error=True)
            return "", pre_img


# --------------------------------------------------------
# Simple bbox cropping utilities
# --------------------------------------------------------
def crop_from_bbox(image: np.ndarray, bbox: dict, padding: int = 0) -> np.ndarray:
    """Crop a region from an image using bbox dictionary."""
    if image is None or image.size == 0:
        return None

    h, w = image.shape[:2]

    if 'x1' in bbox:
        x1, y1 = int(bbox['x1']), int(bbox['y1'])
        x2, y2 = int(bbox['x2']), int(bbox['y2'])
    else:
        return None

    # Apply padding
    x1 = max(0, x1 - padding)
    y1 = max(0, y1 - padding)
    x2 = min(w, x2 + padding)
    y2 = min(h, y2 + padding)

    if x2 <= x1 or y2 <= y1:
        return None

    return image[y1:y2, x1:x2].copy()


def crop_from_xyxy(image: np.ndarray, x1: int, y1: int, x2: int, y2: int, padding: int = 5) -> np.ndarray:
    """Crop using xyxy coordinates."""
    return crop_from_bbox(
        image,
        {'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2},
        padding
    )
