// ROS Message Types for the Drone Project

export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface Header {
  seq?: number;
  stamp: {
    secs: number;
    nsecs: number;
  };
  frame_id: string;
}

export interface Pose {
  position: Vector3;
  orientation: Quaternion;
}

export interface Twist {
  linear: Vector3;
  angular: Vector3;
}

export interface PoseWithCovariance {
  pose: Pose;
  covariance: number[];
}

export interface TwistWithCovariance {
  twist: Twist;
  covariance: number[];
}

export interface Odometry {
  header: Header;
  child_frame_id: string;
  pose: PoseWithCovariance;
  twist: TwistWithCovariance;
}

export interface StringMsg {
  data: string;
}

export interface CompressedImage {
  header: Header;
  format: string;
  data: string; // base64 encoded
}

export type QuizState = 'UNINIT' | 'IDLE' | 'DETECTING' | 'DRAWING' | 'FINISH' | 'UNKNOWN';

export interface DroneStatus {
  connected: boolean;
  state: string;
  lastUpdate: Date | null;
}

