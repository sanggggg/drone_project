import React, { useState, useEffect } from 'react';
import { DataCard } from './DataCard';
import './QuizStatsDisplay.css';

interface QuizStatsDisplayProps {
  currentState: string | null;
  operationTimeoutMin?: number;
  correctCount: number;
  lastUpdate: Date | null;
}

export const QuizStatsDisplay: React.FC<QuizStatsDisplayProps> = ({
  currentState,
  operationTimeoutMin = 5,
  correctCount,
  lastUpdate,
}) => {
  const [elapsedTime, setElapsedTime] = useState(0);
  const [startTime, setStartTime] = useState<number | null>(null);

  const state = currentState?.toUpperCase() || 'UNKNOWN';
  const totalSeconds = operationTimeoutMin * 60;

  // Start timer when entering IDLE state
  useEffect(() => {
    if (state === 'IDLE' && startTime === null) {
      setStartTime(Date.now());
      setElapsedTime(0);
    } else if (state === 'UNINIT' || state === 'FINISH') {
      setStartTime(null);
      setElapsedTime(0);
    }
  }, [state, startTime]);

  // Update elapsed time every second
  useEffect(() => {
    if (startTime === null) return;

    const interval = setInterval(() => {
      const elapsed = Math.floor((Date.now() - startTime) / 1000);
      setElapsedTime(Math.min(elapsed, totalSeconds));
    }, 1000);

    return () => clearInterval(interval);
  }, [startTime, totalSeconds]);

  const remainingSeconds = Math.max(0, totalSeconds - elapsedTime);
  const minutes = Math.floor(remainingSeconds / 60);
  const seconds = remainingSeconds % 60;

  const progressPercent = totalSeconds > 0 ? (elapsedTime / totalSeconds) * 100 : 0;
  const isLowTime = remainingSeconds < 60;
  const isRunning = startTime !== null && state !== 'FINISH';

  return (
    <DataCard
      title="Quiz Stats"
      icon="📊"
      lastUpdate={lastUpdate}
      variant="warning"
    >
      <div className="quiz-stats">
        <div className="quiz-stats__timer">
          <div className="quiz-stats__timer-label">Remaining Time</div>
          <div className={`quiz-stats__timer-value ${isLowTime ? 'low' : ''} ${!isRunning ? 'paused' : ''}`}>
            {String(minutes).padStart(2, '0')}:{String(seconds).padStart(2, '0')}
          </div>
          <div className="quiz-stats__progress">
            <div
              className={`quiz-stats__progress-bar ${isLowTime ? 'low' : ''}`}
              style={{ width: `${100 - progressPercent}%` }}
            />
          </div>
        </div>

        <div className="quiz-stats__divider" />

        <div className="quiz-stats__counter">
          <div className="quiz-stats__counter-label">Correct Answers</div>
          <div className="quiz-stats__counter-value">{correctCount}</div>
        </div>
      </div>
    </DataCard>
  );
};

