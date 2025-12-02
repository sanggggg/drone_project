#!/usr/bin/env python3
# ocr_utils.py
#
# OCR utility module for text recognition from images.
# Uses Tesseract OCR for text extraction.
#
# Usage:
#     from anafi_ros_nodes.ocr_utils import OCRProcessor
#     ocr = OCRProcessor()
#     text = ocr.recognize(image)

import cv2
import numpy as np

# Optional: Tesseract OCR
try:
    import pytesseract
    TESSERACT_AVAILABLE = True
except ImportError:
    TESSERACT_AVAILABLE = False


class OCRProcessor:
    """
    OCR processor using Tesseract for text recognition.
    """
    
    def __init__(
        self,
        lang: str = 'eng',
        psm: int = 10,
        oem: int = 3,
        whitelist: str = '',
        preprocess: bool = True,
        logger=None
    ):
        """
        Initialize OCR processor.
        
        Args:
            lang: Tesseract language code (e.g., 'eng', 'kor', 'eng+kor')
            psm: Page segmentation mode
                 0 = OSD only
                 1 = Automatic with OSD
                 3 = Fully automatic (default)
                 6 = Assume uniform block of text
                 7 = Single text line
                 8 = Single word
                 10 = Single character
                 11 = Sparse text
                 13 = Raw line
            oem: OCR Engine mode
                 0 = Legacy only
                 1 = Neural nets LSTM only
                 2 = Legacy + LSTM
                 3 = Default (based on availability)
            whitelist: Character whitelist (e.g., '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ')
            preprocess: Whether to apply preprocessing to improve OCR accuracy
            logger: Optional ROS2 logger for logging messages
        """
        self.lang = lang
        self.psm = psm
        self.oem = oem
        self.whitelist = whitelist
        self.preprocess = preprocess
        self.logger = logger
        
        if not TESSERACT_AVAILABLE:
            self._log_error("Tesseract (pytesseract) is not installed. Install with: pip install pytesseract")
            self._log_error("Also ensure tesseract-ocr is installed: sudo apt install tesseract-ocr")
    
    def _log_info(self, msg: str):
        if self.logger:
            self.logger.info(msg)
    
    def _log_warn(self, msg: str):
        if self.logger:
            self.logger.warn(msg)
    
    def _log_error(self, msg: str):
        if self.logger:
            self.logger.error(msg)
    
    def is_available(self) -> bool:
        """Check if Tesseract OCR is available."""
        return TESSERACT_AVAILABLE
    
    def _build_config(self) -> str:
        """Build Tesseract configuration string."""
        config_parts = [
            f'--psm {self.psm}',
            f'--oem {self.oem}'
        ]
        
        if self.whitelist:
            config_parts.append(f'-c tessedit_char_whitelist={self.whitelist}')
        
        return ' '.join(config_parts)

    def extract_paper_region(self, image: np.ndarray) -> np.ndarray:
        """
        이진화(Thresholding)된 이미지를 기반으로 가장 큰 흰색 영역(종이)을 찾아 Crop합니다.
        Edge Detection을 따로 하지 않고 색상 대비를 이용하므로 빠릅니다.
        """
        # if image is None or image.size == 0:
        #     return image

        # # 1. 그레이스케일 변환
        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # # 2. 이진화 (핵심 단계)
        # # 배경과 종이를 분리합니다. (종이=255/흰색, 배경=0/검은색)
        # # cv2.THRESH_OTSU: 배경과 종이 사이의 적절한 밝기 값을 자동으로 찾습니다.
        # _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # 3. 빈 공간 메우기 (Morphology Close) - 중요! ★
        # 종이 안에 글자(검은색)가 있으면 구멍 뚫린 것으로 인식될 수 있습니다.
        # 이를 막기 위해 흰색 영역을 살짝 팽창시켜 '한 덩어리'로 만듭니다.
        kernel = np.ones((7, 7), np.uint8)
        binary_closed = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)

        # 4. 외곽선 찾기 (이진화된 이미지 기준)
        contours, _ = cv2.findContours(binary_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return image

        # 5. 가장 큰 영역 찾기
        largest_cnt = max(contours, key=cv2.contourArea)
        
        # 6. 노이즈 필터링 (너무 작은 영역은 무시)
        img_area = image.shape[0] * image.shape[1]
        if cv2.contourArea(largest_cnt) < img_area * 0.05:
            return image

        # 7. 좌표 구해서 자르기
        x, y, w, h = cv2.boundingRect(largest_cnt)

        # 패딩 (너무 타이트하게 잘리지 않도록 여유 공간 확보 or 제거)
        # 안쪽으로 살짝 들어가서 자르고 싶다면 padding을 양수(+)로
        padding = 5
        x = max(0, x + padding)
        y = max(0, y + padding)
        w = max(0, w - 2*padding)
        h = max(0, h - 2*padding)
        
        if w <= 0 or h <= 0:
            return image
        
        # 원본 이미지에서 해당 좌표만큼 잘라냄
        cropped = image[y:y+h, x:x+w].copy()
        
        # (디버깅용) 이진화가 잘 됐는지 확인하고 싶다면 아래 주석 해제하여 저장
        # cv2.imwrite('debug_binary_mask.jpg', binary_closed)
        
        return cropped

    def img_to_whiteblack(self, image:np.ndarray) -> np.ndarray:
        """
        Preprocess image to improve OCR accuracy.
        Uses simple binary thresholding for black text on light background.
        
        Args:
            image: Input BGR image
            
        Returns:
            Preprocessed binary image (black text on white background)
        """
        if image is None or image.size == 0:
            return image
        
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
        
        # Simple binary threshold for black text
        # Black text (dark) becomes 0, background (light) becomes 255
        # Then invert so text is white (255) on black (0) - better for Tesseract
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        # binary = binary[300:700,700:1500]
        
        return binary

    def clear_border_noise(self, image: np.ndarray) -> np.ndarray:
        """
        Crop된 이미지 내부에서 가장자리 노이즈(그림자)를 제거하고,
        화면 중앙에 있는 가장 유력한 텍스트 하나만 남깁니다.
        """
        if image is None or image.size == 0:
            return image

        h, w = image.shape[:2]
        
        # 1. 흑백 반전 (배경:검정, 글자:흰색) -> 컨투어를 찾기 위해
        # 텍스트가 검은색이므로 반전시켜야 '물체'로 인식됨
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
            
        _, binary_inv = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)

        # 2. 컨투어(덩어리) 찾기
        contours, _ = cv2.findContours(binary_inv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return image

        # 3. 중앙에 가장 가까운 덩어리 찾기
        center_x, center_y = w // 2, h // 2
        best_cnt = None
        min_dist = float('inf')

        valid_contours = []

        for cnt in contours:
            # 너무 작은 점(노이즈)은 무시
            if cv2.contourArea(cnt) < 50: 
                continue
                
            # 덩어리의 중심점 계산
            M = cv2.moments(cnt)
            if M["m00"] == 0: continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # 이미지 정중앙과의 거리 계산
            dist = np.sqrt((cx - center_x)**2 + (cy - center_y)**2)
            
            # 덩어리 크기(면적)도 고려 (너무 작은 건 중앙에 있어도 무시)
            # 여기서는 단순히 "거리"가 가장 가까운 놈을 선택
            if dist < min_dist:
                min_dist = dist
                best_cnt = cnt

        # 4. 결과 그리기 (깨끗한 흰 도화지에 중앙 덩어리만 다시 그림)
        if best_cnt is not None:
            # 흰색 배경(255) 생성
            clean_img = np.full((h, w), 255, dtype=np.uint8)
            # 선택된 덩어리만 검은색(0)으로 그림
            cv2.drawContours(clean_img, [best_cnt], -1, (0), thickness=cv2.FILLED)
            return clean_img
        else:
            return image
    
    def _preprocess_image(self, image: np.ndarray) -> np.ndarray:
        img = self.img_to_whiteblack(image)
        img = self.extract_paper_region(img)
        img = self.clear_border_noise(img)        
        return img
    
    def recognize(self, image: np.ndarray) -> str:
        """
        Recognize text from an image.
        
        Args:
            image: Input BGR or grayscale image
            preprocess: Override default preprocessing setting
            
        Returns:
            Recognized text string (stripped of whitespace)
        """
        preprocessed_img = self._preprocess_image(image)

        if not TESSERACT_AVAILABLE:
            self._log_error("Tesseract not available")
            return ""
        
        if image is None or image.size == 0:
            self._log_warn("Empty image provided to OCR")
            return ""
        
        try:
            # Build config and run OCR
            config = self._build_config()
            text = pytesseract.image_to_string(preprocessed_img, lang=self.lang, config=config)
            
            return text.strip(), preprocessed_img
            
        except Exception as e:
            self._log_error(f"OCR failed: {e}")
            return "", None
    
    def recognize_single_char(self, image: np.ndarray) -> str:
        """
        Recognize a single character from an image.
        Uses PSM 10 (single character mode).
        
        Args:
            image: Input image containing a single character
            
        Returns:
            Recognized character
        """
        if not TESSERACT_AVAILABLE:
            return ""
        
        if image is None or image.size == 0:
            return ""
        
        try:
            processed = self._preprocess_image(image)
            
            config = f'--psm 10 --oem {self.oem}'
            if self.whitelist:
                config += f' -c tessedit_char_whitelist={self.whitelist}'
            
            text = pytesseract.image_to_string(processed, lang=self.lang, config=config)
            return text.strip()
            
        except Exception as e:
            self._log_error(f"Single char OCR failed: {e}")
            return ""
    
    def recognize_with_confidence(self, image: np.ndarray) -> list:
        """
        Recognize text with confidence scores.
        
        Args:
            image: Input image
            
        Returns:
            List of dicts with 'text', 'confidence', 'bbox' keys
        """
        if not TESSERACT_AVAILABLE:
            return []
        
        if image is None or image.size == 0:
            return []
        
        try:
            processed = self._preprocess_image(image) if self.preprocess else image
            config = self._build_config()
            
            # Get detailed data
            data = pytesseract.image_to_data(
                processed, lang=self.lang, config=config, output_type=pytesseract.Output.DICT
            )
            
            results = []
            n_boxes = len(data['text'])
            
            for i in range(n_boxes):
                text = data['text'][i].strip()
                conf = int(data['conf'][i])
                
                # Skip empty text or low confidence
                if text and conf > 0:
                    results.append({
                        'text': text,
                        'confidence': conf,
                        'bbox': {
                            'x': data['left'][i],
                            'y': data['top'][i],
                            'width': data['width'][i],
                            'height': data['height'][i]
                        }
                    })
            
            return results
            
        except Exception as e:
            self._log_error(f"OCR with confidence failed: {e}")
            return []


def crop_from_bbox(image: np.ndarray, bbox: dict, padding: int = 0) -> np.ndarray:
    """
    Crop a region from an image using bounding box coordinates.
    
    Args:
        image: Input image (BGR or grayscale)
        bbox: Dictionary with 'x1', 'y1', 'x2', 'y2' or 'x', 'y', 'width', 'height'
        padding: Extra padding around the crop (pixels)
        
    Returns:
        Cropped image region
    """
    if image is None or image.size == 0:
        return None
    
    h, w = image.shape[:2]
    
    # Handle different bbox formats
    if 'x1' in bbox and 'y1' in bbox:
        x1, y1 = int(bbox['x1']), int(bbox['y1'])
        x2, y2 = int(bbox['x2']), int(bbox['y2'])
    elif 'x' in bbox and 'y' in bbox:
        x1, y1 = int(bbox['x']), int(bbox['y'])
        x2 = x1 + int(bbox.get('width', 0))
        y2 = y1 + int(bbox.get('height', 0))
    else:
        return None
    
    # Apply padding and clip to image bounds
    x1 = max(0, x1 - padding)
    y1 = max(0, y1 - padding)
    x2 = min(w, x2 + padding)
    y2 = min(h, y2 + padding)
    
    # Validate crop region
    if x2 <= x1 or y2 <= y1:
        return None
    
    return image[y1:y2, x1:x2].copy()


def crop_from_xyxy(image: np.ndarray, x1: int, y1: int, x2: int, y2: int, padding: int = 0) -> np.ndarray:
    """
    Crop a region from an image using xyxy coordinates.
    
    Args:
        image: Input image
        x1, y1: Top-left corner
        x2, y2: Bottom-right corner
        padding: Extra padding around the crop
        
    Returns:
        Cropped image region
    """
    return crop_from_bbox(image, {'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2}, padding)
