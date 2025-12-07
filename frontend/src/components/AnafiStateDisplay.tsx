import React from 'react';
import { DataCard } from './DataCard';
import './AnafiStateDisplay.css';

interface AnafiStateDisplayProps {
  state: string | null;
  lastUpdate: Date | null;
}

const stateStyles: Record<string, { color: string; icon: string }> = {
  LANDED: { color: '#6a8090', icon: '⬇' },
  MOTOR_RAMPING: { color: '#ffaa00', icon: '⟳' },
  USER_TAKEOFF: { color: '#00d4ff', icon: '↑' },
  TAKINGOFF: { color: '#00d4ff', icon: '↗' },
  HOVERING: { color: '#00ff88', icon: '◎' },
  FLYING: { color: '#00ff88', icon: '✈' },
  LANDING: { color: '#ffaa00', icon: '↓' },
  EMERGENCY: { color: '#ff4466', icon: '⚠' },
  UNKNOWN: { color: '#4a5a6a', icon: '?' },
};

export const AnafiStateDisplay: React.FC<AnafiStateDisplayProps> = ({ state, lastUpdate }) => {
  const normalizedState = state?.toUpperCase() || 'UNKNOWN';
  const style = stateStyles[normalizedState] || stateStyles.UNKNOWN;

  return (
    <DataCard
      title="ANAFI State"
      icon="🦅"
      topic="/anafi/drone/state"
      lastUpdate={lastUpdate}
      variant="warning"
    >
      <div className="anafi-state">
        <div
          className="anafi-state__badge"
          style={{ '--state-color': style.color } as React.CSSProperties}
        >
          <span className="anafi-state__icon">{style.icon}</span>
        </div>
        <div className="anafi-state__info">
          <span className="anafi-state__value" style={{ color: style.color }}>
            {normalizedState}
          </span>
          <span className="anafi-state__label">Flight State</span>
        </div>

        <div className="anafi-state__indicators">
          <div
            className={`anafi-state__indicator ${
              ['HOVERING', 'FLYING'].includes(normalizedState) ? 'active' : ''
            }`}
          >
            <span className="anafi-state__indicator-label">Air</span>
          </div>
          <div
            className={`anafi-state__indicator ${normalizedState === 'LANDED' ? 'active' : ''}`}
          >
            <span className="anafi-state__indicator-label">Gnd</span>
          </div>
          <div
            className={`anafi-state__indicator anafi-state__indicator--warning ${
              normalizedState === 'EMERGENCY' ? 'active' : ''
            }`}
          >
            <span className="anafi-state__indicator-label">Err</span>
          </div>
        </div>
      </div>
    </DataCard>
  );
};
