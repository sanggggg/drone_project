import { useState, useEffect, useCallback, useRef } from 'react';
import * as ROSLIB from 'roslib';

interface UseROSOptions {
  url?: string;
  autoConnect?: boolean;
}

interface UseROSReturn {
  ros: ROSLIB.Ros | null;
  isConnected: boolean;
  error: string | null;
  connect: () => void;
  disconnect: () => void;
}

export function useROS(options: UseROSOptions = {}): UseROSReturn {
  const { url = 'ws://localhost:9090', autoConnect = true } = options;
  
  const [ros, setRos] = useState<ROSLIB.Ros | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const rosRef = useRef<ROSLIB.Ros | null>(null);

  const connect = useCallback(() => {
    if (rosRef.current) {
      rosRef.current.close();
    }

    const newRos = new ROSLIB.Ros({ url });
    rosRef.current = newRos;

    newRos.on('connection', () => {
      console.log('Connected to ROSBridge at', url);
      setIsConnected(true);
      setError(null);
      setRos(newRos);
    });

    newRos.on('error', () => {
      console.error('ROSBridge connection error');
      setError('Connection error. Is rosbridge running?');
      setIsConnected(false);
    });

    newRos.on('close', () => {
      console.log('Disconnected from ROSBridge');
      setIsConnected(false);
    });
  }, [url]);

  const disconnect = useCallback(() => {
    if (rosRef.current) {
      rosRef.current.close();
      rosRef.current = null;
      setRos(null);
      setIsConnected(false);
    }
  }, []);

  useEffect(() => {
    if (autoConnect) {
      connect();
    }
    return () => {
      disconnect();
    };
  }, [autoConnect, connect, disconnect]);

  return { ros, isConnected, error, connect, disconnect };
}

// Hook for subscribing to a topic
interface UseTopicOptions {
  ros: ROSLIB.Ros | null;
  topicName: string;
  messageType: string;
  enabled?: boolean;
}

export function useTopic<T>({ ros, topicName, messageType, enabled = true }: UseTopicOptions) {
  const [message, setMessage] = useState<T | null>(null);
  const [lastUpdate, setLastUpdate] = useState<Date | null>(null);
  const topicRef = useRef<ROSLIB.Topic<T> | null>(null);

  useEffect(() => {
    if (!ros || !enabled) {
      return;
    }

    const topic = new ROSLIB.Topic<T>({
      ros,
      name: topicName,
      messageType,
    });

    topicRef.current = topic;

    const callback = (msg: T) => {
      setMessage(msg);
      setLastUpdate(new Date());
    };

    topic.subscribe(callback);

    return () => {
      topic.unsubscribe();
      topicRef.current = null;
    };
  }, [ros, topicName, messageType, enabled]);

  return { message, lastUpdate };
}

// Message type for compressed images
interface CompressedImageMessage {
  format: string;
  data: string;
}

// Hook for compressed image topics (ANAFI camera)
export function useCompressedImageTopic(ros: ROSLIB.Ros | null, topicName: string, enabled = true) {
  const [imageData, setImageData] = useState<string | null>(null);
  const [lastUpdate, setLastUpdate] = useState<Date | null>(null);
  const topicRef = useRef<ROSLIB.Topic<CompressedImageMessage> | null>(null);

  useEffect(() => {
    if (!ros || !enabled) {
      return;
    }

    const topic = new ROSLIB.Topic<CompressedImageMessage>({
      ros,
      name: topicName,
      messageType: 'sensor_msgs/CompressedImage',
    });

    topicRef.current = topic;

    const callback = (msg: CompressedImageMessage) => {
      // Convert base64 to data URL
      const dataUrl = `data:image/${msg.format};base64,${msg.data}`;
      setImageData(dataUrl);
      setLastUpdate(new Date());
    };

    topic.subscribe(callback);

    return () => {
      topic.unsubscribe();
      topicRef.current = null;
    };
  }, [ros, topicName, enabled]);

  return { imageData, lastUpdate };
}

// Message type for Image messages
interface ImageMessage {
  header: {
    seq?: number;
    stamp: {
      secs: number;
      nsecs: number;
    };
    frame_id: string;
  };
  height: number;
  width: number;
  encoding: string;
  is_bigendian: number;
  step: number;
  data: string | number[]; // base64 string (from ROSBridge) or uint8 array
}

// Hook for Image topics (sensor_msgs/Image)
export function useImageTopic(ros: ROSLIB.Ros | null, topicName: string, enabled = true) {
  const [imageData, setImageData] = useState<string | null>(null);
  const [lastUpdate, setLastUpdate] = useState<Date | null>(null);
  const topicRef = useRef<ROSLIB.Topic<ImageMessage> | null>(null);

  useEffect(() => {
    if (!ros || !enabled) {
      return;
    }

    const topic = new ROSLIB.Topic<ImageMessage>({
      ros,
      name: topicName,
      messageType: 'sensor_msgs/Image',
    });

    topicRef.current = topic;

    const callback = (msg: ImageMessage) => {
      try {
        // Debug: Log first message to understand data format
        if (!topicRef.current || (topicRef.current as any)._debugLogged) {
          // Skip debug after first message
        } else {
          console.log('Image message received:', {
            width: msg.width,
            height: msg.height,
            encoding: msg.encoding,
            step: msg.step,
            dataType: typeof msg.data,
            dataLength: typeof msg.data === 'string' ? msg.data.length : msg.data?.length,
            dataPreview: typeof msg.data === 'string' ? msg.data.substring(0, 50) : 'array',
          });
          (topicRef.current as any)._debugLogged = true;
        }
        
        // ROSLIB sends data as base64 string, convert to Uint8Array
        let imageBytes: Uint8Array;
        
        if (typeof msg.data === 'string') {
          // Base64 string from ROSBridge
          const binaryString = atob(msg.data);
          imageBytes = new Uint8Array(binaryString.length);
          for (let i = 0; i < binaryString.length; i++) {
            imageBytes[i] = binaryString.charCodeAt(i);
          }
        } else {
          // Already a number array
          imageBytes = new Uint8Array(msg.data);
        }
        
        // Validate dimensions
        if (msg.width <= 0 || msg.height <= 0) {
          console.warn('Invalid image dimensions:', msg.width, msg.height);
          return;
        }
        
        // Create canvas to render the image
        const canvas = document.createElement('canvas');
        canvas.width = msg.width;
        canvas.height = msg.height;
        const ctx = canvas.getContext('2d');
        
        if (!ctx) {
          console.error('Failed to get canvas context');
          return;
        }
        
        // Create ImageData from the raw pixel data
        const imageData = ctx.createImageData(msg.width, msg.height);
        const pixels = imageData.data;
        
        // Handle different encodings
        if (msg.encoding === 'bgr8') {
          // BGR8: Convert BGR to RGB, handle step field for row padding
          const bytesPerPixel = 3;
          const expectedDataSize = msg.height * msg.step;
          
          if (imageBytes.length < expectedDataSize) {
            console.warn(`Image data size mismatch: expected ${expectedDataSize}, got ${imageBytes.length}`);
          }
          
          for (let row = 0; row < msg.height; row++) {
            const rowOffset = row * msg.step;
            const pixelOffset = row * msg.width * 4;
            
            for (let col = 0; col < msg.width; col++) {
              const srcIdx = rowOffset + col * bytesPerPixel;
              const dstIdx = pixelOffset + col * 4;
              
              if (srcIdx + 2 < imageBytes.length) {
                pixels[dstIdx] = imageBytes[srcIdx + 2];     // R (from B)
                pixels[dstIdx + 1] = imageBytes[srcIdx + 1]; // G
                pixels[dstIdx + 2] = imageBytes[srcIdx];     // B (from R)
                pixels[dstIdx + 3] = 255;                     // A
              }
            }
          }
        } else if (msg.encoding === 'rgb8') {
          // RGB8: Already in correct format
          const bytesPerPixel = 3;
          const expectedDataSize = msg.height * msg.step;
          
          if (imageBytes.length < expectedDataSize) {
            console.warn(`Image data size mismatch: expected ${expectedDataSize}, got ${imageBytes.length}`);
          }
          
          for (let row = 0; row < msg.height; row++) {
            const rowOffset = row * msg.step;
            const pixelOffset = row * msg.width * 4;
            
            for (let col = 0; col < msg.width; col++) {
              const srcIdx = rowOffset + col * bytesPerPixel;
              const dstIdx = pixelOffset + col * 4;
              
              if (srcIdx + 2 < imageBytes.length) {
                pixels[dstIdx] = imageBytes[srcIdx];         // R
                pixels[dstIdx + 1] = imageBytes[srcIdx + 1]; // G
                pixels[dstIdx + 2] = imageBytes[srcIdx + 2]; // B
                pixels[dstIdx + 3] = 255;                     // A
              }
            }
          }
        } else if (msg.encoding === 'mono8') {
          // MONO8: Grayscale
          const expectedDataSize = msg.height * msg.step;
          
          if (imageBytes.length < expectedDataSize) {
            console.warn(`Image data size mismatch: expected ${expectedDataSize}, got ${imageBytes.length}`);
          }
          
          for (let row = 0; row < msg.height; row++) {
            const rowOffset = row * msg.step;
            const pixelOffset = row * msg.width * 4;
            
            for (let col = 0; col < msg.width; col++) {
              const srcIdx = rowOffset + col;
              const dstIdx = pixelOffset + col * 4;
              
              if (srcIdx < imageBytes.length) {
                const val = imageBytes[srcIdx];
                pixels[dstIdx] = val;     // R
                pixels[dstIdx + 1] = val; // G
                pixels[dstIdx + 2] = val; // B
                pixels[dstIdx + 3] = 255; // A
              }
            }
          }
        } else {
          console.warn(`Unsupported image encoding: ${msg.encoding}`);
          return;
        }
        
        ctx.putImageData(imageData, 0, 0);
        const dataUrl = canvas.toDataURL('image/png');
        setImageData(dataUrl);
        setLastUpdate(new Date());
      } catch (error) {
        console.error('Error processing image message:', error, msg);
      }
    };

    topic.subscribe(callback);

    return () => {
      topic.unsubscribe();
      topicRef.current = null;
    };
  }, [ros, topicName, enabled]);

  return { imageData, lastUpdate };
}

// Hook for throttled Image topics (sensor_msgs/Image) - saves bandwidth
// Uses ROSLIB.Topic's built-in throttle_rate for rate limiting
export function useThrottledImageTopic(
  ros: ROSLIB.Ros | null,
  topicName: string,
  enabled = true,
  throttleMs = 1000  // Default: 1 update per second
) {
  const [imageData, setImageData] = useState<string | null>(null);
  const [lastUpdate, setLastUpdate] = useState<Date | null>(null);
  const topicRef = useRef<ROSLIB.Topic<ImageMessage> | null>(null);

  useEffect(() => {
    if (!ros || !enabled) {
      return;
    }

    const topic = new ROSLIB.Topic<ImageMessage>({
      ros,
      name: topicName,
      messageType: 'sensor_msgs/Image',
      throttle_rate: throttleMs,  // ROSLIB handles throttling
    });

    topicRef.current = topic;

    const callback = (msg: ImageMessage) => {
      try {
        // ROSLIB sends data as base64 string, convert to Uint8Array
        let imageBytes: Uint8Array;
        
        if (typeof msg.data === 'string') {
          const binaryString = atob(msg.data);
          imageBytes = new Uint8Array(binaryString.length);
          for (let i = 0; i < binaryString.length; i++) {
            imageBytes[i] = binaryString.charCodeAt(i);
          }
        } else {
          imageBytes = new Uint8Array(msg.data);
        }
        
        if (msg.width <= 0 || msg.height <= 0) {
          return;
        }
        
        const canvas = document.createElement('canvas');
        canvas.width = msg.width;
        canvas.height = msg.height;
        const ctx = canvas.getContext('2d');
        
        if (!ctx) {
          return;
        }
        
        const imageDataObj = ctx.createImageData(msg.width, msg.height);
        const pixels = imageDataObj.data;
        
        if (msg.encoding === 'bgr8') {
          const bytesPerPixel = 3;
          for (let row = 0; row < msg.height; row++) {
            const rowOffset = row * msg.step;
            const pixelOffset = row * msg.width * 4;
            for (let col = 0; col < msg.width; col++) {
              const srcIdx = rowOffset + col * bytesPerPixel;
              const dstIdx = pixelOffset + col * 4;
              if (srcIdx + 2 < imageBytes.length) {
                pixels[dstIdx] = imageBytes[srcIdx + 2];     // R
                pixels[dstIdx + 1] = imageBytes[srcIdx + 1]; // G
                pixels[dstIdx + 2] = imageBytes[srcIdx];     // B
                pixels[dstIdx + 3] = 255;                     // A
              }
            }
          }
        } else if (msg.encoding === 'rgb8') {
          const bytesPerPixel = 3;
          for (let row = 0; row < msg.height; row++) {
            const rowOffset = row * msg.step;
            const pixelOffset = row * msg.width * 4;
            for (let col = 0; col < msg.width; col++) {
              const srcIdx = rowOffset + col * bytesPerPixel;
              const dstIdx = pixelOffset + col * 4;
              if (srcIdx + 2 < imageBytes.length) {
                pixels[dstIdx] = imageBytes[srcIdx];
                pixels[dstIdx + 1] = imageBytes[srcIdx + 1];
                pixels[dstIdx + 2] = imageBytes[srcIdx + 2];
                pixels[dstIdx + 3] = 255;
              }
            }
          }
        } else if (msg.encoding === 'mono8') {
          for (let row = 0; row < msg.height; row++) {
            const rowOffset = row * msg.step;
            const pixelOffset = row * msg.width * 4;
            for (let col = 0; col < msg.width; col++) {
              const srcIdx = rowOffset + col;
              const dstIdx = pixelOffset + col * 4;
              if (srcIdx < imageBytes.length) {
                const val = imageBytes[srcIdx];
                pixels[dstIdx] = val;
                pixels[dstIdx + 1] = val;
                pixels[dstIdx + 2] = val;
                pixels[dstIdx + 3] = 255;
              }
            }
          }
        } else {
          return; // Unsupported encoding
        }
        
        ctx.putImageData(imageDataObj, 0, 0);
        const dataUrl = canvas.toDataURL('image/png');
        setImageData(dataUrl);
        setLastUpdate(new Date());
      } catch (error) {
        console.error('Error processing throttled image:', error);
      }
    };

    topic.subscribe(callback);

    return () => {
      topic.unsubscribe();
      topicRef.current = null;
    };
  }, [ros, topicName, enabled, throttleMs]);

  return { imageData, lastUpdate };
}
