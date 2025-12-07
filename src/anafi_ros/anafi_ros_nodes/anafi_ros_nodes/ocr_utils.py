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
import re
import numpy as np
import torch
from collections import Counter
from threading import Lock
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
    # Calculate confidence score from generation outputs
    # --------------------------------------------------------
    def _calculate_confidence(self, outputs) -> float:
        """
        Calculate average confidence score from token probabilities.
        
        Args:
            outputs: Generation outputs with scores
            
        Returns:
            Confidence score as percentage (0-100)
        """
        if not hasattr(outputs, 'scores') or outputs.scores is None or len(outputs.scores) == 0:
            return 0.0
        
        # Get probabilities for each generated token
        token_probs = []
        for i, score in enumerate(outputs.scores):
            # Apply softmax to get probabilities
            probs = torch.softmax(score, dim=-1)
            # Get the probability of the selected token
            selected_token = outputs.sequences[0, i + 1]  # +1 because sequences includes start token
            token_prob = probs[0, selected_token].item()
            token_probs.append(token_prob)
        
        if not token_probs:
            return 0.0
        
        # Calculate average probability as confidence
        avg_confidence = sum(token_probs) / len(token_probs) * 100
        return avg_confidence

    # --------------------------------------------------------
    # Preprocess: find and crop white screen region
    # --------------------------------------------------------
    def _preprocess_image(self, image: np.ndarray, 
                          white_threshold: int = 200,
                          min_area_ratio: float = 0.05) -> np.ndarray:
        """
        Find the largest white/bright region and crop it.
        
        Args:
            image: BGR image (numpy array)
            white_threshold: Minimum pixel value to consider as white (0-255)
            min_area_ratio: Minimum area ratio of white region to total image
            
        Returns:
            Cropped white region, or original image if not found
        """
        if image is None or image.size == 0:
            return image
        
        # Convert to grayscale
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
        
        # Threshold to find white regions
        _, binary = cv2.threshold(gray, white_threshold, 255, cv2.THRESH_BINARY)
        
        # Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return image
        
        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Check minimum area
        total_area = image.shape[0] * image.shape[1]
        if area < total_area * min_area_ratio:
            return image
        
        # Get bounding rectangle
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        # Crop tighter by shrinking the bounding box slightly (remove border artifacts)
        shrink = 10  # pixels to shrink from each side
        img_h, img_w = image.shape[:2]
        x1 = min(img_w, x + shrink)
        y1 = min(img_h, y + shrink)
        x2 = max(0, x + w - shrink)
        y2 = max(0, y + h - shrink)
        
        # Ensure valid crop region
        if x2 <= x1 or y2 <= y1:
            # Fallback to original bounding box if shrink is too aggressive
            x1, y1, x2, y2 = x, y, x + w, y + h
        
        return image[y1:y2, x1:x2].copy()

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
                # Generate with scores for confidence calculation
                outputs = self.model.generate(
                    pixel_values,
                    return_dict_in_generate=True,
                    output_scores=True
                )
                generated_ids = outputs.sequences
                
            end_time = time.time()
            infer_ms = (end_time - start_time) * 1000  # ms
            text = self.processor.batch_decode(generated_ids, skip_special_tokens=True)[0]
            text = text.strip()

            # Calculate confidence score from token probabilities
            confidence = self._calculate_confidence(outputs)
            
            self._log(f"[OCR] Result: '{text}' | Confidence: {confidence:.2f}% | Time: {infer_ms:.2f}ms")
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


def find_white_region(image: np.ndarray, 
                       white_threshold: int = 200,
                       min_area_ratio: float = 0.05) -> tuple:
    """
    Find the largest white/bright region in an image.
    
    Args:
        image: BGR image (numpy array)
        white_threshold: Minimum pixel value to consider as white (0-255)
        min_area_ratio: Minimum area ratio of white region to total image
        
    Returns:
        (x1, y1, x2, y2) bounding box of the white region, or None if not found
    """
    if image is None or image.size == 0:
        return None
    
    # Convert to grayscale
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image.copy()
    
    # Threshold to find white regions
    _, binary = cv2.threshold(gray, white_threshold, 255, cv2.THRESH_BINARY)
    
    # Find contours
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None
    
    # Find the largest contour
    largest_contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest_contour)
    
    # Check minimum area
    total_area = image.shape[0] * image.shape[1]
    if area < total_area * min_area_ratio:
        return None
    
    # Get bounding rectangle
    x, y, w, h = cv2.boundingRect(largest_contour)
    
    return (x, y, x + w, y + h)


def crop_white_region(image: np.ndarray, 
                       x1: int, y1: int, x2: int, y2: int,
                       white_threshold: int = 200,
                       min_area_ratio: float = 0.05,
                       padding: int = 5) -> np.ndarray:
    """
    First crop the detection bbox, then find and crop only the white screen region.
    
    Args:
        image: Full BGR image
        x1, y1, x2, y2: Detection bounding box coordinates
        white_threshold: Minimum pixel value to consider as white (0-255)
        min_area_ratio: Minimum area ratio of white region to cropped area
        padding: Padding around the final crop
        
    Returns:
        Cropped image of the white region, or the original bbox crop if no white region found
    """
    # First crop the detection bbox
    bbox_crop = crop_from_xyxy(image, x1, y1, x2, y2, padding=0)
    
    if bbox_crop is None or bbox_crop.size == 0:
        return None
    
    # Find white region within the cropped area
    white_bbox = find_white_region(bbox_crop, white_threshold, min_area_ratio)
    
    if white_bbox is None:
        # No white region found, return original crop with padding
        return crop_from_xyxy(image, x1, y1, x2, y2, padding=padding)
    
    # Crop the white region from the bbox crop
    wx1, wy1, wx2, wy2 = white_bbox
    h, w = bbox_crop.shape[:2]
    
    # Apply padding
    wx1 = max(0, wx1 - padding)
    wy1 = max(0, wy1 - padding)
    wx2 = min(w, wx2 + padding)
    wy2 = min(h, wy2 + padding)
    
    if wx2 <= wx1 or wy2 <= wy1:
        return crop_from_xyxy(image, x1, y1, x2, y2, padding=padding)
    
    return bbox_crop[wy1:wy2, wx1:wx2].copy()


# --------------------------------------------------------
# Batch OCR Buffer Manager
# --------------------------------------------------------
class OCRBufferManager:
    """
    Manages a buffer of images for batch OCR processing.
    
    Features:
    - Collects images into a buffer
    - Processes batch when buffer is full or timeout occurs
    - Filters results by confidence (>=90%)
    - Removes non-alphabet characters
    - Returns the most frequent (mode) result
    """
    
    def __init__(
        self,
        ocr_processor: OCRProcessor,
        buffer_size: int = 10,
        timeout_sec: float = 4.0,
        confidence_threshold: float = 90.0,
        logger=None
    ):
        """
        Args:
            ocr_processor: OCRProcessor instance for running OCR
            buffer_size: Maximum number of images before batch processing
            timeout_sec: Seconds after first image before forcing batch process
            confidence_threshold: Minimum confidence (%) to include result
            logger: Optional ROS logger
        """
        self.ocr = ocr_processor
        self.buffer_size = buffer_size
        self.timeout_sec = timeout_sec
        self.confidence_threshold = confidence_threshold
        self.logger = logger
        
        # Buffer state
        self._buffer = []  # List of (image, preprocessed_image) tuples
        self._lock = Lock()
        self._first_add_time = None
        
    def _log(self, msg, error=False):
        if self.logger:
            if error:
                self.logger.error(msg)
            else:
                self.logger.info(msg)
        else:
            print(msg)
    
    def add_image(self, image: np.ndarray) -> tuple:
        """
        Add an image to the buffer.
        
        Returns:
            (should_process: bool, result: str or None)
            - If should_process is True and result is not None, batch was processed
            - If should_process is False, image was added to buffer
        """
        if image is None or image.size == 0:
            return False, None
        
        with self._lock:
            # Check if buffer was empty (for timeout tracking)
            if len(self._buffer) == 0:
                self._first_add_time = time.time()
            
            # Add image to buffer
            self._buffer.append(image)
            self._log(f"[OCR Buffer] Added image, buffer size: {len(self._buffer)}/{self.buffer_size}")
            
            # Check if we should process
            should_process = False
            
            # Condition 1: Buffer is full
            if len(self._buffer) >= self.buffer_size:
                self._log(f"[OCR Buffer] Buffer full, triggering batch process")
                should_process = True
            
            # Condition 2: Timeout since first image
            elif self._first_add_time is not None:
                elapsed = time.time() - self._first_add_time
                if elapsed >= self.timeout_sec:
                    self._log(f"[OCR Buffer] Timeout ({elapsed:.2f}s), triggering batch process")
                    should_process = True
            
            if should_process:
                result = self._process_batch_locked()
                return True, result
            
            return False, None
    
    def force_process(self) -> str:
        """Force process the current buffer regardless of size/timeout."""
        with self._lock:
            if len(self._buffer) == 0:
                return None
            return self._process_batch_locked()
    
    def check_timeout(self) -> tuple:
        """
        Check if timeout has occurred and process if needed.
        Call this periodically to handle timeout-based processing.
        
        Returns:
            (processed: bool, result: str or None)
        """
        with self._lock:
            if len(self._buffer) == 0:
                return False, None
            
            if self._first_add_time is not None:
                elapsed = time.time() - self._first_add_time
                if elapsed >= self.timeout_sec:
                    self._log(f"[OCR Buffer] Timeout check triggered ({elapsed:.2f}s)")
                    result = self._process_batch_locked()
                    return True, result
            
            return False, None
    
    def _process_batch_locked(self) -> str:
        """
        Process all images in buffer and return mode result.
        Must be called with lock held.
        """
        if len(self._buffer) == 0:
            return None
        
        self._log(f"[OCR Buffer] Processing batch of {len(self._buffer)} images")
        
        # Run OCR on each image and collect results with confidence
        results = []  # List of (text, confidence)
        
        for img in self._buffer:
            text, confidence = self._run_single_ocr(img)
            if text and confidence >= self.confidence_threshold:
                # Clean text: keep only alphabets
                cleaned = self._clean_text(text)
                if cleaned:
                    results.append((cleaned, confidence))
                    self._log(f"[OCR Buffer] Valid result: '{cleaned}' ({confidence:.2f}%)")
                else:
                    self._log(f"[OCR Buffer] Filtered (no alpha): '{text}' ({confidence:.2f}%)")
            elif text:
                self._log(f"[OCR Buffer] Filtered (low conf): '{text}' ({confidence:.2f}%)")
        
        # Clear buffer
        self._buffer.clear()
        self._first_add_time = None
        
        if not results:
            self._log("[OCR Buffer] No valid results after filtering")
            return None
        
        # Find mode (most frequent result)
        texts = [r[0] for r in results]
        counter = Counter(texts)
        mode_text, mode_count = counter.most_common(1)[0]
        
        self._log(f"[OCR Buffer] Mode result: '{mode_text}' (count: {mode_count}/{len(results)})")
        return mode_text
    
    def _run_single_ocr(self, image: np.ndarray) -> tuple:
        """
        Run OCR on a single image and return (text, confidence).
        """
        if image is None or image.size == 0:
            return "", 0.0
        
        # Preprocessing
        pre_img = image
        if self.ocr.preprocess:
            pre_img = self.ocr._preprocess_image(image)
        
        # Convert to PIL
        pil_img = Image.fromarray(pre_img)
        
        try:
            pixel_values = self.ocr.processor(pil_img, return_tensors="pt").pixel_values.to(self.ocr.device)
            
            with torch.no_grad():
                outputs = self.ocr.model.generate(
                    pixel_values,
                    return_dict_in_generate=True,
                    output_scores=True
                )
                generated_ids = outputs.sequences
            
            text = self.ocr.processor.batch_decode(generated_ids, skip_special_tokens=True)[0]
            text = text.strip()
            
            confidence = self.ocr._calculate_confidence(outputs)
            
            return text, confidence
            
        except Exception as e:
            self._log(f"OCR inference failed: {e}", error=True)
            return "", 0.0
    
    @staticmethod
    def _clean_text(text: str) -> str:
        """Remove all non-alphabet characters from text."""
        return re.sub(r'[^a-zA-Z]', '', text)
    
    def get_buffer_size(self) -> int:
        """Return current buffer size."""
        with self._lock:
            return len(self._buffer)
    
    def clear(self):
        """Clear the buffer without processing."""
        with self._lock:
            self._buffer.clear()
            self._first_add_time = None
