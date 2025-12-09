import React, { useState, useEffect } from 'react';
import * as ROSLIB from 'roslib';
import { useTopic, useThrottledImageTopic } from '../hooks/useROS';
import { useBeepSound } from '../hooks/useBeepSound';
import type { StringMsg } from '../types/ros-messages';
import './TeamPanel.css';

interface TeamPanelProps {
  teamName: string;
  ros: ROSLIB.Ros | null;
  isConnected: boolean;
  error: string | null;
  onConnect: () => void;
  onDisconnect: () => void;
  accentColor: string;
  operationTimeoutMin?: number;
  cameraTopic?: string;
  cameraThrottleMs?: number;
}

export const TeamPanel: React.FC<TeamPanelProps> = ({
  teamName,
  ros,
  isConnected,
  error,
  onConnect,
  onDisconnect,
  accentColor,
  operationTimeoutMin = 5,
  cameraTopic = '/yolo/image',
  cameraThrottleMs = 1000,  // 1 FPS by default to save bandwidth
}) => {
  // State tracking
  const [correctCount, setCorrectCount] = useState(0);
  const [prevState, setPrevState] = useState<string | null>(null);
  const [displayedQuestion, setDisplayedQuestion] = useState<string | null>(null);
  const [displayedAnswer, setDisplayedAnswer] = useState<string | null>(null);
  const [detectingStartTime, setDetectingStartTime] = useState<Date | null>(null);
  
  // Timer state
  const [elapsedTime, setElapsedTime] = useState(0);
  const [startTime, setStartTime] = useState<number | null>(null);

  // Subscribe to topics
  const { message: quizState } = useTopic<StringMsg>({
    ros,
    topicName: '/quiz/state',
    messageType: 'std_msgs/String',
    enabled: isConnected,
  });

  const { message: quizAnswer, lastUpdate: quizAnswerUpdate } = useTopic<StringMsg>({
    ros,
    topicName: '/quiz/answer',
    messageType: 'std_msgs/String',
    enabled: isConnected,
  });

  const { message: quizQuestion, lastUpdate: quizQuestionUpdate } = useTopic<StringMsg>({
    ros,
    topicName: '/quiz/question',
    messageType: 'std_msgs/String',
    enabled: isConnected,
  });

  const { message: quizBeep, lastUpdate: quizBeepUpdate } = useTopic<StringMsg>({
    ros,
    topicName: '/quiz/beep',
    messageType: 'std_msgs/String',
    enabled: isConnected,
  });

  // Camera feed (throttled to save bandwidth)
  const { imageData: cameraImage, lastUpdate: cameraUpdate } = useThrottledImageTopic(
    ros,
    cameraTopic,
    isConnected,
    cameraThrottleMs
  );

  // Use beep sound hook
  useBeepSound({
    beepType: quizBeep?.data || null,
    lastUpdate: quizBeepUpdate,
    enabled: isConnected,
  });

  const currentState = quizState?.data || null;
  const state = currentState?.toUpperCase() || 'UNKNOWN';
  const totalSeconds = operationTimeoutMin * 60;

  // Track state changes to count correct answers
  useEffect(() => {
    if (currentState !== prevState) {
      // If transitioning from DRAWING to IDLE, increment correct count
      if (prevState === 'DRAWING' && currentState === 'IDLE') {
        setCorrectCount((c) => c + 1);
      }
      // Reset count on UNINIT
      if (currentState === 'UNINIT') {
        setCorrectCount(0);
        setDisplayedQuestion(null);
        setDisplayedAnswer(null);
      }
      
      // Track when DETECTING state starts
      if (currentState === 'DETECTING') {
        setDetectingStartTime(new Date());
        setDisplayedQuestion(null);
        setDisplayedAnswer(null);
      } else {
        setDetectingStartTime(null);
      }
      
      setPrevState(currentState);
    }
  }, [currentState, prevState]);

  // Show question/answer that come after entering DETECTING state
  useEffect(() => {
    if (currentState === 'DETECTING' && detectingStartTime) {
      if (quizQuestion?.data && quizQuestionUpdate && quizQuestionUpdate >= detectingStartTime) {
        setDisplayedQuestion(quizQuestion.data);
      }
    }
  }, [quizQuestion, quizQuestionUpdate, currentState, detectingStartTime]);

  useEffect(() => {
    if (currentState === 'DETECTING' && detectingStartTime) {
      if (quizAnswer?.data && quizAnswerUpdate && quizAnswerUpdate >= detectingStartTime) {
        setDisplayedAnswer(quizAnswer.data);
      }
    }
  }, [quizAnswer, quizAnswerUpdate, currentState, detectingStartTime]);

  // Timer logic
  useEffect(() => {
    if (state === 'IDLE' && startTime === null) {
      setStartTime(Date.now());
      setElapsedTime(0);
    } else if (state === 'UNINIT' || state === 'FINISH') {
      setStartTime(null);
      setElapsedTime(0);
    }
  }, [state, startTime]);

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

  const isDetecting = currentState === 'DETECTING';
  const hasQuizData = displayedQuestion || displayedAnswer;

  return (
    <div className="team-panel" style={{ '--accent-color': accentColor } as React.CSSProperties}>
      {/* Header */}
      <div className="team-panel__header">
        <h2 className="team-panel__name">{teamName}</h2>
        <div className="team-panel__connection">
          <span className={`team-panel__status-dot ${isConnected ? 'connected' : 'disconnected'}`} />
          <span className="team-panel__status-text">
            {isConnected ? 'Connected' : error || 'Disconnected'}
          </span>
          <button
            className="team-panel__connect-btn"
            onClick={isConnected ? onDisconnect : onConnect}
          >
            {isConnected ? 'Disconnect' : 'Connect'}
          </button>
        </div>
      </div>

      {/* Stats Row */}
      <div className="team-panel__stats-row">
        {/* Game State */}
        <div className={`team-panel__stat-card team-panel__state-card team-panel__state-card--${state.toLowerCase()}`}>
          <div className="team-panel__stat-content">
            <span className="team-panel__stat-value team-panel__state-value">{state}</span>
            <span className="team-panel__stat-label">State</span>
          </div>
        </div>

        {/* Correct Count */}
        <div className="team-panel__stat-card">
          <div className="team-panel__stat-icon">✓</div>
          <div className="team-panel__stat-content">
            <span className="team-panel__stat-value">{correctCount}</span>
            <span className="team-panel__stat-label">Correct</span>
          </div>
        </div>

        {/* Timer */}
        <div className="team-panel__stat-card team-panel__timer">
          <div className="team-panel__stat-icon">⏱</div>
          <div className="team-panel__stat-content">
            <span className={`team-panel__stat-value ${isLowTime ? 'low-time' : ''} ${!isRunning ? 'paused' : ''}`}>
              {String(minutes).padStart(2, '0')}:{String(seconds).padStart(2, '0')}
            </span>
            <span className="team-panel__stat-label">Remaining</span>
          </div>
          <div className="team-panel__timer-bar">
            <div
              className={`team-panel__timer-progress ${isLowTime ? 'low-time' : ''}`}
              style={{ width: `${100 - progressPercent}%` }}
            />
          </div>
        </div>
      </div>

      {/* Camera Feed */}
      <div className="team-panel__camera">
        <div className="team-panel__camera-container">
          {cameraImage ? (
            <>
              <img
                src={cameraImage}
                alt="Camera Feed"
                className="team-panel__camera-image"
              />
              <div className="team-panel__camera-overlay">
                <div className="team-panel__camera-corner team-panel__camera-corner--tl" />
                <div className="team-panel__camera-corner team-panel__camera-corner--tr" />
                <div className="team-panel__camera-corner team-panel__camera-corner--bl" />
                <div className="team-panel__camera-corner team-panel__camera-corner--br" />
              </div>
              <div className="team-panel__camera-status">
                <span className="team-panel__camera-live">● LIVE</span>
              </div>
            </>
          ) : (
            <div className="team-panel__camera-no-signal">
              <div className="team-panel__camera-static" />
              <span className="team-panel__camera-no-signal-icon">📷</span>
              <span className="team-panel__camera-no-signal-text">NO SIGNAL</span>
            </div>
          )}
        </div>
        {cameraUpdate && (
          <span className="team-panel__camera-timestamp">
            {cameraUpdate.toLocaleTimeString()}
          </span>
        )}
      </div>

      {/* Combined Quiz Display (Question = Answer) */}
      <div className="team-panel__quiz-section">
        <div className="team-panel__section-header">
          <span className="team-panel__section-icon">🧮</span>
          <span className="team-panel__section-title">Detected Quiz</span>
          <span className={`team-panel__state-indicator ${state.toLowerCase()}`}>{state}</span>
        </div>
        <div className="team-panel__quiz-content">
          {isDetecting ? (
            hasQuizData ? (
              <div className="team-panel__quiz-equation">
                <span className="team-panel__quiz-question">
                  {displayedQuestion || '?'}
                </span>
                <span className="team-panel__quiz-equals">=</span>
                <span className={`team-panel__quiz-answer ${displayedAnswer ? 'has-answer' : ''}`}>
                  {displayedAnswer || '?'}
                </span>
              </div>
            ) : (
              <div className="team-panel__quiz-waiting">
                <span className="team-panel__quiz-waiting-icon">?</span>
                <span className="team-panel__quiz-waiting-text">Awaiting Detection...</span>
              </div>
            )
          ) : (
            <div className="team-panel__quiz-idle">
              <span className="team-panel__quiz-idle-text">Not in detection mode</span>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};
