import { useState, useEffect, useRef } from 'react';
import { useROS, useTopic } from './hooks/useROS';
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

  // Quiz Beep topic
  const { message: quizBeep, lastUpdate: quizBeepUpdate } = useTopic<StringMsg>({
    ros,
    topicName: '/quiz/beep',
    messageType: 'std_msgs/String',
    enabled: isConnected,
  });

  // Audio context for beep sounds
  const audioContextRef = useRef<AudioContext | null>(null);

  // Initialize audio context on mount
  useEffect(() => {
    if (typeof window !== 'undefined' && !audioContextRef.current) {
      audioContextRef.current = new (window.AudioContext || (window as any).webkitAudioContext)();
    }
    return () => {
      if (audioContextRef.current) {
        audioContextRef.current.close();
        audioContextRef.current = null;
      }
    };
  }, []);

  // Play beep sound based on message type
  useEffect(() => {
    if (!quizBeep?.data || !audioContextRef.current) {
      return;
    }

    const beepType = quizBeep.data.toLowerCase();
    const ctx = audioContextRef.current;

    // Resume audio context if suspended (required for user interaction)
    if (ctx.state === 'suspended') {
      ctx.resume();
    }

    const playBeep = (frequency: number, duration: number, type: 'tone' | 'sweep' = 'tone') => {
      const oscillator = ctx.createOscillator();
      const gainNode = ctx.createGain();

      oscillator.connect(gainNode);
      gainNode.connect(ctx.destination);

      oscillator.frequency.value = frequency;
      oscillator.type = 'sine';

      if (type === 'sweep') {
        // Sweep from low to high frequency
        oscillator.frequency.setValueAtTime(frequency * 0.7, ctx.currentTime);
        oscillator.frequency.exponentialRampToValueAtTime(frequency * 1.3, ctx.currentTime + duration);
      }

      gainNode.gain.setValueAtTime(0, ctx.currentTime);
      gainNode.gain.linearRampToValueAtTime(0.3, ctx.currentTime + 0.01);
      gainNode.gain.exponentialRampToValueAtTime(0.01, ctx.currentTime + duration);

      oscillator.start(ctx.currentTime);
      oscillator.stop(ctx.currentTime + duration);
    };

    switch (beepType) {
      case 'start':
        // High-pitched short beep
        playBeep(800, 0.15);
        break;
      case 'end':
        // Medium-pitched short beep
        playBeep(600, 0.15);
        break;
      case 'correct':
        // Rising tone (success sound)
        playBeep(500, 0.3, 'sweep');
        break;
      default:
        // Default beep
        playBeep(400, 0.1);
    }
  }, [quizBeep, quizBeepUpdate]);

  // Test beep sounds - plays each type every 1 second (for testing)
  // Set TEST_BEEP_ENABLED to true to enable
  const TEST_BEEP_ENABLED = false; // Change to true to test beep sounds
  useEffect(() => {
    if (!TEST_BEEP_ENABLED || !audioContextRef.current) {
      return;
    }

    const beepTypes: Array<'start' | 'end' | 'correct'> = ['start', 'end', 'correct'];
    let currentIndex = 0;

    const ctx = audioContextRef.current;

    // Resume audio context if suspended
    if (ctx.state === 'suspended') {
      ctx.resume();
    }

    const playBeep = (frequency: number, duration: number, type: 'tone' | 'sweep' = 'tone') => {
      const oscillator = ctx.createOscillator();
      const gainNode = ctx.createGain();

      oscillator.connect(gainNode);
      gainNode.connect(ctx.destination);

      oscillator.frequency.value = frequency;
      oscillator.type = 'sine';

      if (type === 'sweep') {
        oscillator.frequency.setValueAtTime(frequency * 0.7, ctx.currentTime);
        oscillator.frequency.exponentialRampToValueAtTime(frequency * 1.3, ctx.currentTime + duration);
      }

      gainNode.gain.setValueAtTime(0, ctx.currentTime);
      gainNode.gain.linearRampToValueAtTime(0.3, ctx.currentTime + 0.01);
      gainNode.gain.exponentialRampToValueAtTime(0.01, ctx.currentTime + duration);

      oscillator.start(ctx.currentTime);
      oscillator.stop(ctx.currentTime + duration);
    };

    const playTestBeep = () => {
      const beepType = beepTypes[currentIndex];
      console.log(`[TEST] Playing beep: ${beepType}`);

      switch (beepType) {
        case 'start':
          playBeep(800, 0.15);
          break;
        case 'end':
          playBeep(600, 0.15);
          break;
        case 'correct':
          playBeep(500, 0.3, 'sweep');
          break;
      }

      currentIndex = (currentIndex + 1) % beepTypes.length;
    };

    // Play immediately, then every 1 second
    playTestBeep();
    const interval = setInterval(playTestBeep, 1000);

    return () => {
      clearInterval(interval);
    };
  }, [TEST_BEEP_ENABLED]);

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
