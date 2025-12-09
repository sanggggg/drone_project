import { useState, useEffect } from 'react';
import { useROS, useTopic } from './hooks/useROS';
import { useBeepSound } from './hooks/useBeepSound';
import {
  ConnectionStatus,
  QuizStateDisplay,
  QuizAnswerDisplay,
  QuizStatsDisplay,
  DroneStatusCompact,
  CommandPanel,
} from './components';
import type { StringMsg, Odometry } from './types/ros-messages';
import './App.css';

function App() {
  const { ros, isConnected, error, connect, disconnect } = useROS({
    url: 'ws://localhost:9090',
    autoConnect: true,
  });

  // Track correct answers (increments when transitioning from DRAWING to IDLE)
  const [correctCount, setCorrectCount] = useState(0);
  const [prevState, setPrevState] = useState<string | null>(null);

  // Quiz State topic
  const { message: quizState, lastUpdate: quizStateUpdate } = useTopic<StringMsg>({
    ros,
    topicName: '/quiz/state',
    messageType: 'std_msgs/String',
    enabled: isConnected,
  });

  // Track state changes to count correct answers
  const currentState = quizState?.data || null;
  useEffect(() => {
    if (currentState !== prevState) {
      // If transitioning from DRAWING to IDLE, increment correct count
      if (prevState === 'DRAWING' && currentState === 'IDLE') {
        setCorrectCount((c) => c + 1);
      }
      // Reset count on UNINIT
      if (currentState === 'UNINIT') {
        setCorrectCount(0);
      }
      setPrevState(currentState);
    }
  }, [currentState, prevState]);

  // Quiz Question topic
  const { message: quizQuestion, lastUpdate: quizQuestionUpdate } = useTopic<StringMsg>({
    ros,
    topicName: '/quiz/question',
    messageType: 'std_msgs/String',
    enabled: isConnected,
  });

  // Quiz Answer topic
  const { message: quizAnswer, lastUpdate: quizAnswerUpdate } = useTopic<StringMsg>({
    ros,
    topicName: '/quiz/answer',
    messageType: 'std_msgs/String',
    enabled: isConnected,
  });

  // Crazyflie Odometry topic
  const { message: cfOdom, lastUpdate: cfOdomUpdate } = useTopic<Odometry>({
    ros,
    topicName: '/cf/odom',
    messageType: 'nav_msgs/Odometry',
    enabled: isConnected,
  });

  // ANAFI State topic
  const { message: anafiState, lastUpdate: anafiStateUpdate } = useTopic<StringMsg>({
    ros,
    topicName: '/anafi/drone/state',
    messageType: 'std_msgs/String',
    enabled: isConnected,
  });

  // Quiz Beep topic
  const { message: quizBeep, lastUpdate: quizBeepUpdate } = useTopic<StringMsg>({
    ros,
    topicName: '/quiz/beep',
    messageType: 'std_msgs/String',
    enabled: isConnected,
  });

  // Sound test state
  const [testBeepType, setTestBeepType] = useState<string | null>(null);
  const [testLastUpdate, setTestLastUpdate] = useState<Date | null>(null);

  // Use beep sound hook - combines ROS topic with manual test
  useBeepSound({
    beepType: testBeepType || quizBeep?.data || null,
    lastUpdate: testLastUpdate || quizBeepUpdate,
    enabled: true,
  });

  const playTestSound = (type: string) => {
    setTestBeepType(type);
    setTestLastUpdate(new Date());
    // Reset test state after a short delay
    setTimeout(() => {
      setTestBeepType(null);
      setTestLastUpdate(null);
    }, 1000);
  };

  return (
    <div className="app">
      <div className="app__background">
        <div className="app__grid-overlay" />
        <div className="app__glow app__glow--1" />
        <div className="app__glow app__glow--2" />
      </div>

      <header className="app__header">
        <div className="app__logo">
          <span className="app__logo-icon">◈</span>
          <div className="app__logo-text">
            <h1>DRONE CONTROL</h1>
            <span className="app__logo-subtitle">Quiz Demo System</span>
          </div>
        </div>
        <ConnectionStatus
          isConnected={isConnected}
          error={error}
          onConnect={connect}
          onDisconnect={disconnect}
        />
      </header>

      <main className="app__main">
        <div className="app__dashboard">
          {/* Top Row: Quiz State + Answer + Stats */}
          <div className="dashboard__row dashboard__row--quiz">
            <QuizStateDisplay
              state={currentState}
              lastUpdate={quizStateUpdate}
            />
            <QuizAnswerDisplay
              question={quizQuestion?.data || null}
              answer={quizAnswer?.data || null}
              currentState={currentState}
              lastUpdate={quizAnswerUpdate || quizQuestionUpdate}
            />
            <QuizStatsDisplay
              currentState={currentState}
              operationTimeoutMin={5}
              correctCount={correctCount}
              lastUpdate={quizStateUpdate}
            />
          </div>

          {/* Middle Row: Drone Status + Commands */}
          <div className="dashboard__row dashboard__row--controls">
            <DroneStatusCompact
              cfOdom={cfOdom}
              cfLastUpdate={cfOdomUpdate}
              anafiState={anafiState?.data || null}
              anafiLastUpdate={anafiStateUpdate}
            />
            <CommandPanel
              ros={ros}
              isConnected={isConnected}
              currentState={currentState}
            />
          </div>
        </div>
      </main>

      {/* Sound Test Panel */}
      <div style={{
        position: 'fixed',
        bottom: '80px',
        right: '20px',
        background: 'rgba(0,0,0,0.95)',
        border: '2px solid #00ff88',
        borderRadius: '12px',
        padding: '16px',
        zIndex: 9999,
        display: 'flex',
        flexDirection: 'column',
        gap: '8px',
        boxShadow: '0 0 20px rgba(0,255,136,0.3)',
      }}>
        <span style={{ color: '#00ff88', fontWeight: 'bold', fontSize: '14px', marginBottom: '4px' }}>🔊 Sound Test</span>
        <button 
          onClick={() => playTestSound('start')}
          style={{ padding: '8px 16px', background: '#2563eb', color: 'white', border: 'none', borderRadius: '6px', cursor: 'pointer', fontWeight: 'bold', fontSize: '12px' }}
        >
          ▶ START
        </button>
        <button 
          onClick={() => playTestSound('end')}
          style={{ padding: '8px 16px', background: '#dc2626', color: 'white', border: 'none', borderRadius: '6px', cursor: 'pointer', fontWeight: 'bold', fontSize: '12px' }}
        >
          ■ END
        </button>
        <button 
          onClick={() => playTestSound('correct')}
          style={{ padding: '8px 16px', background: '#16a34a', color: 'white', border: 'none', borderRadius: '6px', cursor: 'pointer', fontWeight: 'bold', fontSize: '12px' }}
        >
          ✓ CORRECT
        </button>
        <button 
          onClick={() => playTestSound('default')}
          style={{ padding: '8px 16px', background: '#6b7280', color: 'white', border: 'none', borderRadius: '6px', cursor: 'pointer', fontWeight: 'bold', fontSize: '12px' }}
        >
          ● DEFAULT
        </button>
      </div>

      <footer className="app__footer">
        <span>Drone Project © 2025</span>
        <span className="app__footer-divider">|</span>
        <span>ROS2 Humble</span>
      </footer>
    </div>
  );
}

export default App;
