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


def validate_white_screen(image: np.ndarray, 
                          min_white_ratio: float = 0.5,
                          white_threshold: int = 200) -> tuple:
    """
    Validate if the cropped image contains sufficient white area.
    
    Args:
        image: BGR image (cropped detection region)
        min_white_ratio: Minimum ratio of white pixels required (0.0-1.0)
        white_threshold: Pixel value threshold to consider as white (0-255)
        
    Returns:
        (is_valid: bool, white_ratio: float)
    """
    if image is None or image.size == 0:
        return False, 0.0
    
    # Convert to grayscale
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    
    # Count white pixels
    white_pixels = np.sum(gray > white_threshold)
    total_pixels = gray.size
    
    if total_pixels == 0:
        return False, 0.0
    
    white_ratio = white_pixels / total_pixels
    is_valid = white_ratio >= min_white_ratio
    
    return is_valid, white_ratio


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
        self._is_processing = False  # Flag to block new images during processing
        
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
            (should_process: bool, question: str or None, answer: str or None)
            - If should_process is True and answer is not None, batch was processed
            - question is the original expression (e.g., "3×5")
            - answer is the calculated result (e.g., "15")
            - If should_process is False, image was added to buffer
        """
        if image is None or image.size == 0:
            return False, None, None
        
        # Skip if currently processing a batch
        if self._is_processing:
            self._log("[OCR Buffer] Skipping - batch processing in progress")
            return False, None, None
        
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
                question, answer = self._process_batch_locked()
                return True, question, answer
            
            return False, None, None
    
    def force_process(self) -> tuple:
        """Force process the current buffer regardless of size/timeout.
        
        Returns:
            (question: str or None, answer: str or None)
        """
        with self._lock:
            if len(self._buffer) == 0:
                return None, None
            return self._process_batch_locked()
    
    def check_timeout(self) -> tuple:
        """
        Check if timeout has occurred and process if needed.
        Call this periodically to handle timeout-based processing.
        
        Returns:
            (processed: bool, question: str or None, answer: str or None)
        """
        with self._lock:
            if len(self._buffer) == 0:
                return False, None, None
            
            if self._first_add_time is not None:
                elapsed = time.time() - self._first_add_time
                if elapsed >= self.timeout_sec:
                    self._log(f"[OCR Buffer] Timeout check triggered ({elapsed:.2f}s)")
                    question, answer = self._process_batch_locked()
                    return True, question, answer
            
            return False, None, None
    
    def _process_batch_locked(self) -> tuple:
        """
        Process all images in buffer and return mode result.
        Must be called with lock held.
        
        Returns:
            (question: str or None, answer: str or None)
            - question is the original expression (e.g., "3×5")
            - answer is the calculated result (e.g., "15")
        """
        self._is_processing = True
        if len(self._buffer) == 0:
            return None, None
        
        self._log(f"[OCR Buffer] Processing batch of {len(self._buffer)} images")
        
        # Run OCR on each image and collect results with confidence
        # Store (raw_text, cleaned/answer, confidence) tuples
        results = []
        
        for img in self._buffer:
            text, confidence = self._run_single_ocr(img)
            if text and confidence >= self.confidence_threshold:
                # Detect type and clean accordingly
                is_expr = self._is_expression(text)
                if is_expr:
                    # For expressions: question = cleaned expr, answer = calculated
                    cleaned_expr = self._clean_expression(text)
                    answer = self.evaluate_expression(text)
                    if answer:
                        results.append((cleaned_expr, answer, confidence))
                        self._log(f"[OCR Buffer] Valid expr: '{cleaned_expr}' = '{answer}' ({confidence:.2f}%)")
                    else:
                        self._log(f"[OCR Buffer] Filtered (failed eval): '{text}' ({confidence:.2f}%)")
                else:
                    # For text: question = cleaned text, answer = same
                    cleaned = self._clean_text(text)
                    if cleaned:
                        results.append((cleaned, cleaned, confidence))
                        self._log(f"[OCR Buffer] Valid text: '{cleaned}' ({confidence:.2f}%)")
                    else:
                        self._log(f"[OCR Buffer] Filtered (empty after clean): '{text}' ({confidence:.2f}%)")
            elif text:
                self._log(f"[OCR Buffer] Filtered (low conf): '{text}' ({confidence:.2f}%)")
        
        # Clear buffer
        self._buffer.clear()
        self._first_add_time = None
        
        if not results:
            self._log("[OCR Buffer] No valid results after filtering")
            self._is_processing = False
            return None, None
        
        # Find mode based on answer (most frequent result)
        answers = [r[1] for r in results]
        counter = Counter(answers)
        mode_answer, mode_count = counter.most_common(1)[0]
        
        # Find the corresponding question for the mode answer
        mode_question = None
        for q, a, _ in results:
            if a == mode_answer:
                mode_question = q
                break
        
        self._log(f"[OCR Buffer] Mode result: '{mode_question}' = '{mode_answer}' (count: {mode_count}/{len(results)})")
        self._is_processing = False
        return mode_question, mode_answer
    
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
    def _is_expression(text: str) -> bool:
        """
        Determine if the text is a mathematical expression.
        
        Expression criteria (any of):
        - Contains only digits (single number like "5", "123")
        - Contains digits and operators (+, -, *, x, X, ×, ÷, /, =)
        
        Returns:
            True if expression, False if text
        """
        # Check if it's purely numeric (single number)
        cleaned = re.sub(r'[^0-9+\-*/×÷=xX]', '', text)
        if cleaned and cleaned.isdigit():
            return True
        
        # Check if it has digits and operators
        has_digit = bool(re.search(r'\d', text))
        has_operator = bool(re.search(r'[+\-*/×÷=xX]', text))
        return has_digit and has_operator
    
    @staticmethod
    def _clean_text(text: str) -> str:
        """Remove all non-alphabet characters from text (for text mode)."""
        return re.sub(r'[^A-Z]', '', text)
    
    @staticmethod
    def _clean_expression(text: str) -> str:
        """
        Clean and normalize a mathematical expression.
        
        - Keep only digits and operators
        - Normalize multiplication symbols (x, X, *) to ×
        - Normalize division symbols (/) to ÷
        - Single digit numbers only
        
        Returns:
            Cleaned expression string
        """
        # First, normalize multiplication symbols
        text = re.sub(r'[xX*]', '×', text)
        # Normalize division
        text = re.sub(r'/', '÷', text)
        # Keep only digits and operators (+, -, ×, ÷, =)
        cleaned = re.sub(r'[^0-9+\-×÷=]', '', text)
        return cleaned
    
    def _parse_expression(self, expr: str) -> list:
        """
        Parse a mathematical expression with multi-digit numbers.
        
        Supports: +, -, ×, ÷
        Valid formats:
        - Single number: "5" -> [5]
        - Two numbers + one operator: "3+5" -> [3, '+', 5]
        - Three numbers + two operators: "3+5×2" -> [3, '+', 5, '×', 2]
        
        Args:
            expr: Cleaned expression string
            
        Returns:
            List of tokens [num, op, num, ...] or None if parsing fails
        """
        original_expr = expr
        # Remove '=' if present
        expr = expr.replace('=', '')
        
        if len(expr) == 0:
            self._log(f"[OCR] Parse failed (empty): '{original_expr}'")
            return None
        
        # Tokenize: split into numbers and operators
        tokens = []
        current_num = ""
        
        for char in expr:
            if char.isdigit():
                current_num += char
            elif char in '+\-×÷':
                if current_num:
                    tokens.append(int(current_num))
                    current_num = ""
                tokens.append(char)
            else:
                # Invalid character
                self._log(f"[OCR] Parse failed (invalid char '{char}'): '{original_expr}'")
                return None
        
        # Don't forget the last number
        if current_num:
            tokens.append(int(current_num))
        
        # Validate token structure: should be [num] or [num, op, num] or [num, op, num, op, num] ...
        if len(tokens) == 0:
            self._log(f"[OCR] Parse failed (no tokens): '{original_expr}'")
            return None
        
        # Check pattern: odd positions should be numbers, even positions should be operators
        for i, token in enumerate(tokens):
            if i % 2 == 0:  # Should be number
                if not isinstance(token, int):
                    self._log(f"[OCR] Parse failed (expected number at pos {i}): '{original_expr}'")
                    return None
            else:  # Should be operator
                if token not in ['+', '-', '×', '÷']:
                    self._log(f"[OCR] Parse failed (expected operator at pos {i}): '{original_expr}'")
                    return None
        
        # Must end with a number (odd length)
        if len(tokens) % 2 == 0:
            self._log(f"[OCR] Parse failed (incomplete expression): '{original_expr}'")
            return None
        
        return tokens
    
    @staticmethod
    def _calculate(num1: int, op: str, num2: int) -> int:
        """
        Calculate the result of a simple expression.
        
        Args:
            num1: First operand
            op: Operator (×, ÷, +, -)
            num2: Second operand
            
        Returns:
            Result as integer, or None if invalid
        """
        try:
            if op == '+':
                return num1 + num2
            elif op == '-':
                return num1 - num2
            elif op == '×':
                return num1 * num2
            elif op == '÷':
                if num2 == 0:
                    return None
                return num1 // num2  # Integer division
            else:
                return None
        except Exception:
            return None
    
    def _evaluate_tokens(self, tokens: list) -> int:
        """
        Evaluate a list of tokens respecting operator precedence.
        
        Precedence: ×, ÷ before +, -
        
        Args:
            tokens: List like [num, op, num, op, num, ...]
            
        Returns:
            Calculated result as integer
        """
        if len(tokens) == 1:
            return tokens[0]
        
        # First pass: handle × and ÷ (higher precedence)
        i = 0
        while i < len(tokens):
            if i < len(tokens) and tokens[i] in ['×', '÷']:
                op = tokens[i]
                num1 = tokens[i - 1]
                num2 = tokens[i + 1]
                result = self._calculate(num1, op, num2)
                # Replace num1, op, num2 with result
                tokens = tokens[:i-1] + [result] + tokens[i+2:]
                # Don't increment i, check same position again
            else:
                i += 1
        
        # Second pass: handle + and - (lower precedence)
        i = 0
        while i < len(tokens):
            if i < len(tokens) and tokens[i] in ['+', '-']:
                op = tokens[i]
                num1 = tokens[i - 1]
                num2 = tokens[i + 1]
                result = self._calculate(num1, op, num2)
                tokens = tokens[:i-1] + [result] + tokens[i+2:]
            else:
                i += 1
        
        return tokens[0] if tokens else None
    
    def evaluate_expression(self, expr: str) -> str:
        """
        Parse and evaluate a mathematical expression.
        
        Supports:
        - Single number: "5" -> "5"
        - Simple expression: "3×5" -> "15"
        - Complex expression: "3+5×2" -> "13" (with precedence)
        
        Args:
            expr: Expression string (e.g., "3×5=", "7+2", "3+5×2")
            
        Returns:
            Calculated result as string, or None if failed
        """
        cleaned = self._clean_expression(expr)
        tokens = self._parse_expression(cleaned)
        
        if tokens is None:
            self._log(f"[OCR] Failed to parse expression: '{expr}' -> '{cleaned}'")
            return None
        
        # Build expression string for logging
        expr_str = ''.join(str(t) for t in tokens)
        
        # Evaluate
        result = self._evaluate_tokens(tokens.copy())
        
        if result is None:
            self._log(f"[OCR] Failed to calculate: '{expr_str}'")
            return None
        
        self._log(f"[OCR] Expression: {expr_str} = {result}")
        return str(result)
    
    def _clean_result(self, text: str) -> str:
        """
        Clean OCR result based on content type (text vs expression).
        
        Args:
            text: Raw OCR text
            
        Returns:
            Cleaned text appropriate for its type
        """
        if self._is_expression(text):
            return self.evaluate_expression(text)
        else:
            return self._clean_text(text)
    
    def get_buffer_size(self) -> int:
        """Return current buffer size."""
        with self._lock:
            return len(self._buffer)
    
    def clear(self):
        """Clear the buffer without processing."""
        with self._lock:
            self._buffer.clear()
            self._first_add_time = None
