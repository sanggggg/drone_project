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
