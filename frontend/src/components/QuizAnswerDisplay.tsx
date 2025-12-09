import React from 'react';
import { DataCard } from './DataCard';
import './QuizAnswerDisplay.css';

interface QuizAnswerDisplayProps {
  question: string | null;
  answer: string | null;
  currentState: string | null;
  lastUpdate: Date | null;
}

export const QuizAnswerDisplay: React.FC<QuizAnswerDisplayProps> = ({ question, answer, lastUpdate }) => {
  const hasData = question || answer;

  return (
    <DataCard
      title="Quiz Q&A"
      icon="💡"
      topic="/quiz/question + /quiz/answer"
      lastUpdate={lastUpdate}
      variant="success"
    >
      <div className="quiz-answer">
        {hasData ? (
          <div className="quiz-answer__content">
            {/* Question Display */}
            <div className="quiz-answer__row">
              <span className="quiz-answer__label">Q:</span>
              <span className="quiz-answer__question">{question || '---'}</span>
            </div>
            {/* Answer Display */}
            <div className="quiz-answer__row">
              <span className="quiz-answer__label">A:</span>
              <span className="quiz-answer__answer">{answer || '---'}</span>
            </div>
          </div>
        ) : (
          <div className="quiz-answer__empty">
            <span className="quiz-answer__waiting-icon">?</span>
            <span className="quiz-answer__waiting-text">No Detection Yet</span>
          </div>
        )}
      </div>
    </DataCard>
  );
};
