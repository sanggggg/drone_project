import React from 'react';
import { DataCard } from './DataCard';
import './QuizAnswerDisplay.css';

interface QuizAnswerDisplayProps {
  answer: string | null;
  lastUpdate: Date | null;
}

const answerConfig: Record<string, { trajectory: string; icon: string; color: string }> = {
  '1': { trajectory: 'Figure 8', icon: '∞', color: '#00ff88' },
  '2': { trajectory: 'Vertical A', icon: 'A', color: '#aa66ff' },
};

export const QuizAnswerDisplay: React.FC<QuizAnswerDisplayProps> = ({ answer, lastUpdate }) => {
  const config = answer ? answerConfig[answer] : null;

  return (
    <DataCard
      title="Quiz Answer"
      icon="💡"
      topic="/quiz/answer"
      lastUpdate={lastUpdate}
      variant="success"
    >
      <div className="quiz-answer">
        {answer ? (
          <>
            <div
              className="quiz-answer__badge"
              style={{ '--answer-color': config?.color || '#64b5f6' } as React.CSSProperties}
            >
              <span className="quiz-answer__number">{answer}</span>
            </div>
            <div className="quiz-answer__details">
              <span className="quiz-answer__trajectory-label">Trajectory:</span>
              <span className="quiz-answer__trajectory">
                <span className="quiz-answer__icon">{config?.icon}</span>
                {config?.trajectory || 'Unknown'}
              </span>
            </div>
          </>
        ) : (
          <div className="quiz-answer__empty">
            <span className="quiz-answer__waiting-icon">?</span>
            <span className="quiz-answer__waiting-text">Awaiting Detection</span>
          </div>
        )}
      </div>
    </DataCard>
  );
};
