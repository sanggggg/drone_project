import React from 'react';
import { DataCard } from './DataCard';
import './CameraFeed.css';

interface CameraFeedProps {
  imageData: string | null;
  lastUpdate: Date | null;
  title?: string;
  topic?: string;
}

export const CameraFeed: React.FC<CameraFeedProps> = ({
  imageData,
  lastUpdate,
  title = 'ANAFI Camera',
  topic = '/anafi/camera/image/compressed',
}) => {
  return (
    <DataCard
      title={title}
      icon="📷"
      topic={topic}
      lastUpdate={lastUpdate}
      className="camera-feed-card"
    >
      <div className="camera-feed">
        {imageData ? (
          <div className="camera-feed__container">
            <img
              src={imageData}
              alt="Drone Camera Feed"
              className="camera-feed__image"
            />
            <div className="camera-feed__overlay">
              <div className="camera-feed__crosshair">
                <div className="camera-feed__crosshair-h" />
                <div className="camera-feed__crosshair-v" />
                <div className="camera-feed__crosshair-center" />
              </div>
              <div className="camera-feed__corner camera-feed__corner--tl" />
              <div className="camera-feed__corner camera-feed__corner--tr" />
              <div className="camera-feed__corner camera-feed__corner--bl" />
              <div className="camera-feed__corner camera-feed__corner--br" />
            </div>
            <div className="camera-feed__status">
              <span className="camera-feed__rec">● LIVE</span>
            </div>
          </div>
        ) : (
          <div className="camera-feed__no-signal">
            <div className="camera-feed__static" />
            <span className="camera-feed__no-signal-text">NO SIGNAL</span>
            <span className="camera-feed__waiting">Waiting for camera feed...</span>
          </div>
        )}
      </div>
    </DataCard>
  );
};

