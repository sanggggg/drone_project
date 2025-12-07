import React from 'react';
import './ConnectionStatus.css';

interface ConnectionStatusProps {
  isConnected: boolean;
  error: string | null;
  onConnect: () => void;
  onDisconnect: () => void;
}

export const ConnectionStatus: React.FC<ConnectionStatusProps> = ({
  isConnected,
  error,
  onConnect,
  onDisconnect,
}) => {
  return (
    <div className="connection-status">
      <div className={`status-indicator ${isConnected ? 'connected' : 'disconnected'}`}>
        <span className="status-dot" />
        <span className="status-text">
          {isConnected ? 'ROSBRIDGE CONNECTED' : 'DISCONNECTED'}
        </span>
      </div>
      {error && <div className="connection-error">{error}</div>}
      <button
        className={`connection-btn ${isConnected ? 'disconnect' : 'connect'}`}
        onClick={isConnected ? onDisconnect : onConnect}
      >
        {isConnected ? 'Disconnect' : 'Connect'}
      </button>
    </div>
  );
};

