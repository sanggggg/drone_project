import React from 'react';
import type { Odometry } from '../types/ros-messages';
import './DroneStatusCompact.css';

interface DroneStatusCompactProps {
  cfOdom: Odometry | null;
  cfLastUpdate: Date | null;
  anafiState: string | null;
  anafiLastUpdate: Date | null;
}

function quaternionToYaw(q: { x: number; y: number; z: number; w: number }) {
  const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  return (Math.atan2(siny_cosp, cosy_cosp) * 180) / Math.PI;
}

const anafiStateColors: Record<string, string> = {
  LANDED: '#6a8090',
  HOVERING: '#00ff88',
  FLYING: '#00ff88',
  TAKINGOFF: '#00d4ff',
  LANDING: '#ffaa00',
  EMERGENCY: '#ff4466',
  UNKNOWN: '#4a5a6a',
};

export const DroneStatusCompact: React.FC<DroneStatusCompactProps> = ({
  cfOdom,
  cfLastUpdate,
  anafiState,
  anafiLastUpdate,
}) => {
  const pos = cfOdom?.pose?.pose?.position;
  const orient = cfOdom?.pose?.pose?.orientation;
  const yaw = orient ? quaternionToYaw(orient) : null;

  const formatVal = (v: number | null | undefined, d = 2) =>
    v != null ? v.toFixed(d) : '---';

  const state = anafiState?.toUpperCase() || 'UNKNOWN';
  const stateColor = anafiStateColors[state] || anafiStateColors.UNKNOWN;

  const cfConnected = cfLastUpdate && (Date.now() - cfLastUpdate.getTime()) < 3000;
  const anafiConnected = anafiLastUpdate && (Date.now() - anafiLastUpdate.getTime()) < 3000;

  return (
    <div className="drone-compact">
      {/* Crazyflie */}
      <div className="drone-compact__item">
        <div className="drone-compact__header">
          <span className="drone-compact__icon">🚁</span>
          <span className="drone-compact__name">Crazyflie</span>
          <span className={`drone-compact__status ${cfConnected ? 'online' : 'offline'}`} />
        </div>
        <div className="drone-compact__data">
          <div className="drone-compact__field">
            <span className="drone-compact__label">X</span>
            <span className="drone-compact__value drone-compact__value--x">{formatVal(pos?.x)}</span>
          </div>
          <div className="drone-compact__field">
            <span className="drone-compact__label">Y</span>
            <span className="drone-compact__value drone-compact__value--y">{formatVal(pos?.y)}</span>
          </div>
          <div className="drone-compact__field">
            <span className="drone-compact__label">Z</span>
            <span className="drone-compact__value drone-compact__value--z">{formatVal(pos?.z)}</span>
          </div>
          <div className="drone-compact__field">
            <span className="drone-compact__label">Yaw</span>
            <span className="drone-compact__value">{formatVal(yaw, 0)}°</span>
          </div>
        </div>
      </div>

      <div className="drone-compact__divider" />

      {/* ANAFI */}
      <div className="drone-compact__item">
        <div className="drone-compact__header">
          <span className="drone-compact__icon">🦅</span>
          <span className="drone-compact__name">ANAFI</span>
          <span className={`drone-compact__status ${anafiConnected ? 'online' : 'offline'}`} />
        </div>
        <div className="drone-compact__state">
          <span
            className="drone-compact__state-badge"
            style={{ '--state-color': stateColor } as React.CSSProperties}
          >
            {state}
          </span>
        </div>
      </div>
    </div>
  );
};

