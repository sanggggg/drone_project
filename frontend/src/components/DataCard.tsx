import React, { type ReactNode } from 'react';
import './DataCard.css';

interface DataCardProps {
  title: string;
  icon?: ReactNode;
  topic?: string;
  lastUpdate?: Date | null;
  children: ReactNode;
  variant?: 'default' | 'primary' | 'warning' | 'success';
  className?: string;
}

export const DataCard: React.FC<DataCardProps> = ({
  title,
  icon,
  topic,
  lastUpdate,
  children,
  variant = 'default',
  className = '',
}) => {
  const getTimeSince = (date: Date | null) => {
    if (!date) return 'No data';
    const seconds = Math.floor((new Date().getTime() - date.getTime()) / 1000);
    if (seconds < 2) return 'Just now';
    if (seconds < 60) return `${seconds}s ago`;
    return `${Math.floor(seconds / 60)}m ago`;
  };

  return (
    <div className={`data-card data-card--${variant} ${className}`}>
      <div className="data-card__header">
        <div className="data-card__title-row">
          {icon && <span className="data-card__icon">{icon}</span>}
          <h3 className="data-card__title">{title}</h3>
        </div>
        {topic && <span className="data-card__topic">{topic}</span>}
      </div>
      <div className="data-card__content">{children}</div>
      {lastUpdate !== undefined && (
        <div className="data-card__footer">
          <span className="data-card__update-time">{getTimeSince(lastUpdate)}</span>
        </div>
      )}
    </div>
  );
};

