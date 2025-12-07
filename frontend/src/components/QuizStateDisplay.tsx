import React from 'react';
import { DataCard } from './DataCard';
import type { QuizState } from '../types/ros-messages';
import './QuizStateDisplay.css';

interface QuizStateDisplayProps {
  state: string | null;
  lastUpdate: Date | null;
}

const stateConfig: Record<QuizState, { color: string; label: string; icon: string }> = {
  UNINIT: { color: '#6a7080', label: 'Uninitialized', icon: '○' },
  IDLE: { color: '#00d4ff', label: 'Idle', icon: '◇' },
  DETECTING: { color: '#ffaa00', label: 'Detecting', icon: '◎' },
  DRAWING: { color: '#aa66ff', label: 'Drawing', icon: '✧' },
  FINISH: { color: '#00ff88', label: 'Finished', icon: '◆' },
  UNKNOWN: { color: '#ff4466', label: 'Unknown', icon: '?' },
};

export const QuizStateDisplay: React.FC<QuizStateDisplayProps> = ({ state, lastUpdate }) => {
  const normalizedState = (state?.toUpperCase() || 'UNKNOWN') as QuizState;
  const config = stateConfig[normalizedState] || stateConfig.UNKNOWN;

  return (
    <DataCard
      title="Quiz State"
      icon="🎯"
      topic="/quiz/state"
      lastUpdate={lastUpdate}
      variant="primary"
    >
      <div className="quiz-state">
        <div className="quiz-state__visual" style={{ '--state-color': config.color } as React.CSSProperties}>
          <span className="quiz-state__icon">{config.icon}</span>
          <div className="quiz-state__ring" />
          <div className="quiz-state__ring quiz-state__ring--outer" />
        </div>
        <div className="quiz-state__info">
          <span className="quiz-state__value" style={{ color: config.color }}>
            {normalizedState}
          </span>
          <span className="quiz-state__label">{config.label}</span>
        </div>
        <div className="quiz-state__flow">
          {(['UNINIT', 'IDLE', 'DETECTING', 'DRAWING', 'FINISH'] as QuizState[]).map((s, i) => (
            <React.Fragment key={s}>
              <div
                className={`quiz-state__step ${normalizedState === s ? 'active' : ''} ${
                  ['UNINIT', 'IDLE', 'DETECTING', 'DRAWING', 'FINISH'].indexOf(normalizedState) > i ? 'completed' : ''
                }`}
                style={{ '--step-color': stateConfig[s].color } as React.CSSProperties}
              >
                <span className="quiz-state__step-icon">{stateConfig[s].icon}</span>
              </div>
              {i < 4 && <div className="quiz-state__connector" />}
            </React.Fragment>
          ))}
        </div>
      </div>
    </DataCard>
  );
};
