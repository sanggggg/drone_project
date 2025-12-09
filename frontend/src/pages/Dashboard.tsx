import { useROS } from '../hooks/useROS';
import { TeamPanel } from '../components/TeamPanel';
import './Dashboard.css';

function Dashboard() {
  // Team A connection (port 9090)
  const teamA = useROS({
    url: 'ws://localhost:9090',
    autoConnect: true,
  });

  // Team B connection (port 9091)
  const teamB = useROS({
    url: 'ws://localhost:9091',
    autoConnect: true,
  });

  return (
    <div className="dashboard">
      <div className="dashboard__background">
        <div className="dashboard__grid-overlay" />
        <div className="dashboard__glow dashboard__glow--a" />
        <div className="dashboard__glow dashboard__glow--b" />
      </div>

      <header className="dashboard__header">
        <div className="dashboard__logo">
          <span className="dashboard__logo-icon">◈</span>
          <div className="dashboard__logo-text">
            <h1>QUIZ DASHBOARD</h1>
            <span className="dashboard__logo-subtitle">Dual Team Monitor</span>
          </div>
        </div>
        <div className="dashboard__connection-summary">
          <span className={`dashboard__summary-dot ${teamA.isConnected ? 'connected' : ''}`} />
          <span className="dashboard__summary-label">Team A</span>
          <span className="dashboard__summary-divider">|</span>
          <span className={`dashboard__summary-dot ${teamB.isConnected ? 'connected' : ''}`} />
          <span className="dashboard__summary-label">Team B</span>
        </div>
      </header>

      <main className="dashboard__main">
        <div className="dashboard__split">
          <TeamPanel
            teamName="Team A"
            ros={teamA.ros}
            isConnected={teamA.isConnected}
            error={teamA.error}
            onConnect={teamA.connect}
            onDisconnect={teamA.disconnect}
            accentColor="#00ff88"
            operationTimeoutMin={5}
          />
          <div className="dashboard__divider" />
          <TeamPanel
            teamName="Team B"
            ros={teamB.ros}
            isConnected={teamB.isConnected}
            error={teamB.error}
            onConnect={teamB.connect}
            onDisconnect={teamB.disconnect}
            accentColor="#aa66ff"
            operationTimeoutMin={5}
          />
        </div>
      </main>

      <footer className="dashboard__footer">
        <span>Drone Project © 2025</span>
        <span className="dashboard__footer-divider">|</span>
        <span>ROS2 Humble</span>
        <span className="dashboard__footer-divider">|</span>
        <a href="/" className="dashboard__back-link">← Back to Control</a>
      </footer>
    </div>
  );
}

export default Dashboard;

