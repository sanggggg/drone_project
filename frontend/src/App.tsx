import { useState } from 'react';
import { useROS, useTopic, useCompressedImageTopic } from './hooks/useROS';
import {
  ConnectionStatus,
  QuizStateDisplay,
  QuizAnswerDisplay,
  QuizStatsDisplay,
  DroneStatusCompact,
  CameraFeed,
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

  // ANAFI Camera (compressed image)
  const { imageData: anafiCamera, lastUpdate: anafiCameraUpdate } = useCompressedImageTopic(
    ros,
    '/anafi/camera/image/compressed',
    isConnected
  );

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
              answer={quizAnswer?.data || null}
              lastUpdate={quizAnswerUpdate}
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

          {/* Bottom Row: Camera */}
          <div className="dashboard__row dashboard__row--camera">
            <CameraFeed
              imageData={anafiCamera}
              lastUpdate={anafiCameraUpdate}
            />
          </div>
        </div>
      </main>

      <footer className="app__footer">
        <span>Drone Project © 2025</span>
        <span className="app__footer-divider">|</span>
        <span>ROS2 Humble</span>
      </footer>
    </div>
  );
}

export default App;
