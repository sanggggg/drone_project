import React, { useCallback, useState } from 'react';
import * as ROSLIB from 'roslib';
import './CommandPanel.css';

interface CommandPanelProps {
  ros: ROSLIB.Ros | null;
  isConnected: boolean;
  currentState: string | null;
}

const MOCK_ANSWERS = [
  { value: '1', label: '1' },
  { value: '2', label: '2' },
];

export const CommandPanel: React.FC<CommandPanelProps> = ({ ros, isConnected, currentState }) => {
  const [showMockDropdown, setShowMockDropdown] = useState(false);
  const [ocrEnabled, setOcrEnabled] = useState(false);

  const publishCommand = useCallback((command: string) => {
    if (!ros || !isConnected) {
      console.warn('ROS not connected');
      return;
    }

    const topic = new ROSLIB.Topic<{ data: string }>({
      ros,
      name: '/quiz/command',
      messageType: 'std_msgs/String',
    });

    topic.publish({ data: command });
    console.log(`Published command: ${command}`);
  }, [ros, isConnected]);

  const publishMockAnswer = useCallback((answer: string) => {
    if (!ros || !isConnected) {
      console.warn('ROS not connected');
      return;
    }

    const topic = new ROSLIB.Topic<{ data: string }>({
      ros,
      name: '/quiz/answer',
      messageType: 'std_msgs/String',
    });

    topic.publish({ data: answer });
    console.log(`Published mock answer: ${answer}`);
    setShowMockDropdown(false);
  }, [ros, isConnected]);

  const toggleOcr = useCallback(() => {
    if (!ros || !isConnected) {
      console.warn('ROS not connected');
      return;
    }

    const newState = !ocrEnabled;
    
    // Publish to /anafi/yolo/ocr_enable topic (Bool)
    const topic = new ROSLIB.Topic<{ data: boolean }>({
      ros,
      name: '/anafi/yolo/ocr_enable',
      messageType: 'std_msgs/Bool',
    });

    topic.publish({ data: newState });
    setOcrEnabled(newState);
    console.log(`OCR ${newState ? 'ENABLED' : 'DISABLED'}`);
  }, [ros, isConnected, ocrEnabled]);

  const state = currentState?.toUpperCase() || 'UNKNOWN';

  // Button states based on current quiz state
  const canSetup = state === 'UNINIT' || state === 'UNKNOWN';
  const canDetect = state === 'IDLE';
  const canOcr = state === 'DETECTING';  // OCR available during DETECTING
  const canMockAnswer = state === 'DETECTING';
  const canAnswerCorrect = state === 'DRAWING';
  const canFinish = ['IDLE', 'DETECTING', 'DRAWING'].includes(state);
  const canEmergency = state !== 'FINISH';

  // Reset OCR state when leaving DETECTING
  React.useEffect(() => {
    if (state !== 'DETECTING' && ocrEnabled) {
      setOcrEnabled(false);
    }
  }, [state, ocrEnabled]);

  return (
    <div className="command-panel">
      <h3 className="command-panel__title">
        <span className="command-panel__icon">⌘</span>
        Command Panel
      </h3>
      
      <div className="command-panel__buttons">
        <button
          className={`cmd-btn cmd-btn--setup ${canSetup ? '' : 'disabled'}`}
          onClick={() => publishCommand('start')}
          disabled={!isConnected || !canSetup}
        >
          <span className="cmd-btn__icon">▶</span>
          <span className="cmd-btn__label">Setup</span>
          <span className="cmd-btn__hint">Takeoff</span>
        </button>

        <button
          className={`cmd-btn cmd-btn--detect ${canDetect ? '' : 'disabled'}`}
          onClick={() => publishCommand('detect')}
          disabled={!isConnected || !canDetect}
        >
          <span className="cmd-btn__icon">◎</span>
          <span className="cmd-btn__label">Detect</span>
          <span className="cmd-btn__hint">Start</span>
        </button>

        <button
          className={`cmd-btn cmd-btn--ocr ${canOcr ? (ocrEnabled ? 'active' : '') : 'disabled'}`}
          onClick={toggleOcr}
          disabled={!isConnected || !canOcr}
        >
          <span className="cmd-btn__icon">{ocrEnabled ? '🔍' : '📷'}</span>
          <span className="cmd-btn__label">{ocrEnabled ? 'OCR On' : 'OCR'}</span>
          <span className="cmd-btn__hint">{ocrEnabled ? 'Active' : 'Scan'}</span>
        </button>

        <div className="cmd-btn-wrapper">
          <button
            className={`cmd-btn cmd-btn--mock ${canMockAnswer ? '' : 'disabled'}`}
            onClick={() => setShowMockDropdown(!showMockDropdown)}
            disabled={!isConnected || !canMockAnswer}
          >
            <span className="cmd-btn__icon">🎲</span>
            <span className="cmd-btn__label">Mock</span>
            <span className="cmd-btn__hint">Answer</span>
          </button>
          
          {showMockDropdown && canMockAnswer && (
            <div className="mock-dropdown">
              <div className="mock-dropdown__header">Select Answer</div>
              {MOCK_ANSWERS.map((ans) => (
                <button
                  key={ans.value}
                  className="mock-dropdown__item"
                  onClick={() => publishMockAnswer(ans.value)}
                >
                  {ans.label}
                </button>
              ))}
              <button
                className="mock-dropdown__cancel"
                onClick={() => setShowMockDropdown(false)}
              >
                Cancel
              </button>
            </div>
          )}
        </div>

        <button
          className={`cmd-btn cmd-btn--correct ${canAnswerCorrect ? '' : 'disabled'}`}
          onClick={() => publishCommand('answer_correct')}
          disabled={!isConnected || !canAnswerCorrect}
        >
          <span className="cmd-btn__icon">✓</span>
          <span className="cmd-btn__label">Correct</span>
          <span className="cmd-btn__hint">OK</span>
        </button>

        <button
          className={`cmd-btn cmd-btn--finish ${canFinish ? '' : 'disabled'}`}
          onClick={() => publishCommand('finish')}
          disabled={!isConnected || !canFinish}
        >
          <span className="cmd-btn__icon">⏹</span>
          <span className="cmd-btn__label">Finish</span>
          <span className="cmd-btn__hint">Land</span>
        </button>

        <button
          className={`cmd-btn cmd-btn--emergency ${canEmergency ? '' : 'disabled'}`}
          onClick={() => publishCommand('emergency')}
          disabled={!isConnected || !canEmergency}
        >
          <span className="cmd-btn__icon">⚠</span>
          <span className="cmd-btn__label">E-STOP</span>
          <span className="cmd-btn__hint">Emergency</span>
        </button>
      </div>
    </div>
  );
};
