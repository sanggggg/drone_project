import React from 'react';
import { DataCard } from './DataCard';
import type { Odometry } from '../types/ros-messages';
import './OdometryDisplay.css';

interface OdometryDisplayProps {
  odom: Odometry | null;
  lastUpdate: Date | null;
}

// Convert quaternion to euler angles (roll, pitch, yaw)
function quaternionToEuler(q: { x: number; y: number; z: number; w: number }) {
  const sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  const cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  const roll = Math.atan2(sinr_cosp, cosr_cosp);

  const sinp = 2 * (q.w * q.y - q.z * q.x);
  const pitch = Math.abs(sinp) >= 1 ? (Math.sign(sinp) * Math.PI) / 2 : Math.asin(sinp);

  const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  const yaw = Math.atan2(siny_cosp, cosy_cosp);

  return {
    roll: (roll * 180) / Math.PI,
    pitch: (pitch * 180) / Math.PI,
    yaw: (yaw * 180) / Math.PI,
  };
}

export const OdometryDisplay: React.FC<OdometryDisplayProps> = ({ odom, lastUpdate }) => {
  const pos = odom?.pose?.pose?.position;
  const orient = odom?.pose?.pose?.orientation;
  const euler = orient ? quaternionToEuler(orient) : null;

  const formatValue = (val: number | undefined, decimals = 2) => {
    if (val === undefined || val === null) return '---';
    return val.toFixed(decimals);
  };

  return (
    <DataCard
      title="Crazyflie Odom"
      icon="🚁"
      topic="/cf/odom"
      lastUpdate={lastUpdate}
    >
      <div className="odometry">
        <div className="odometry__data">
          <div className="odometry__section">
            <h4 className="odometry__section-title">Position (m)</h4>
            <div className="odometry__grid">
              <div className="odometry__item">
                <span className="odometry__label">X</span>
                <span className="odometry__value odometry__value--x">{formatValue(pos?.x)}</span>
              </div>
              <div className="odometry__item">
                <span className="odometry__label">Y</span>
                <span className="odometry__value odometry__value--y">{formatValue(pos?.y)}</span>
              </div>
              <div className="odometry__item">
                <span className="odometry__label">Z</span>
                <span className="odometry__value odometry__value--z">{formatValue(pos?.z)}</span>
              </div>
            </div>
          </div>

          <div className="odometry__divider" />

          <div className="odometry__section">
            <h4 className="odometry__section-title">Orientation (°)</h4>
            <div className="odometry__grid">
              <div className="odometry__item">
                <span className="odometry__label">R</span>
                <span className="odometry__value">{formatValue(euler?.roll, 1)}</span>
              </div>
              <div className="odometry__item">
                <span className="odometry__label">P</span>
                <span className="odometry__value">{formatValue(euler?.pitch, 1)}</span>
              </div>
              <div className="odometry__item">
                <span className="odometry__label">Y</span>
                <span className="odometry__value">{formatValue(euler?.yaw, 1)}</span>
              </div>
            </div>
          </div>
        </div>

        <div className="odometry__visual">
          <div className="odometry__crosshair">
            <div
              className="odometry__position-dot"
              style={{
                transform: `translate(${(pos?.x || 0) * 20}px, ${-(pos?.y || 0) * 20}px)`,
              }}
            />
            <div className="odometry__axis odometry__axis--x" />
            <div className="odometry__axis odometry__axis--y" />
          </div>
          <span className="odometry__altitude">Z: {formatValue(pos?.z)}m</span>
        </div>
      </div>
    </DataCard>
  );
};
