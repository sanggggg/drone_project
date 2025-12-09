import { useEffect, useRef } from 'react';

interface UseBeepSoundOptions {
  beepType: string | null;
  lastUpdate: Date | null;
  enabled?: boolean;
}

export function useBeepSound({ beepType, lastUpdate, enabled = true }: UseBeepSoundOptions) {
  const audioContextRef = useRef<AudioContext | null>(null);
  const lastProcessedUpdate = useRef<Date | null>(null);

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
    if (!enabled || !beepType || !audioContextRef.current || !lastUpdate) {
      return;
    }

    // Prevent duplicate plays for the same update
    if (lastProcessedUpdate.current && lastUpdate <= lastProcessedUpdate.current) {
      return;
    }
    lastProcessedUpdate.current = lastUpdate;

    const type = beepType.toLowerCase();
    const ctx = audioContextRef.current;

    // Resume audio context if suspended (required for user interaction)
    if (ctx.state === 'suspended') {
      ctx.resume();
    }

    // Create a powerful, attention-grabbing beep with multiple oscillators
    const playPowerfulBeep = (
      baseFreq: number,
      duration: number,
      volume: number = 0.8,
      waveType: OscillatorType = 'square'
    ) => {
      // Main oscillator
      const osc1 = ctx.createOscillator();
      const gain1 = ctx.createGain();
      osc1.connect(gain1);
      gain1.connect(ctx.destination);
      osc1.frequency.value = baseFreq;
      osc1.type = waveType;

      // Harmonic oscillator for richness
      const osc2 = ctx.createOscillator();
      const gain2 = ctx.createGain();
      osc2.connect(gain2);
      gain2.connect(ctx.destination);
      osc2.frequency.value = baseFreq * 2; // Octave up
      osc2.type = 'sine';

      // Sub bass for punch
      const osc3 = ctx.createOscillator();
      const gain3 = ctx.createGain();
      osc3.connect(gain3);
      gain3.connect(ctx.destination);
      osc3.frequency.value = baseFreq * 0.5; // Octave down
      osc3.type = 'sine';

      const now = ctx.currentTime;

      // Aggressive attack, medium release envelope
      gain1.gain.setValueAtTime(0, now);
      gain1.gain.linearRampToValueAtTime(volume, now + 0.005);
      gain1.gain.setValueAtTime(volume, now + duration * 0.6);
      gain1.gain.exponentialRampToValueAtTime(0.01, now + duration);

      gain2.gain.setValueAtTime(0, now);
      gain2.gain.linearRampToValueAtTime(volume * 0.4, now + 0.005);
      gain2.gain.exponentialRampToValueAtTime(0.01, now + duration * 0.8);

      gain3.gain.setValueAtTime(0, now);
      gain3.gain.linearRampToValueAtTime(volume * 0.3, now + 0.01);
      gain3.gain.exponentialRampToValueAtTime(0.01, now + duration * 0.5);

      osc1.start(now);
      osc2.start(now);
      osc3.start(now);
      osc1.stop(now + duration);
      osc2.stop(now + duration);
      osc3.stop(now + duration);
    };

    // Multi-beep pattern for important events
    const playAlertPattern = (baseFreq: number, count: number, interval: number) => {
      for (let i = 0; i < count; i++) {
        setTimeout(() => {
          playPowerfulBeep(baseFreq, 0.15, 0.9, 'square');
        }, i * interval);
      }
    };

    // Rising sweep for success
    const playSuccessFanfare = () => {
      const now = ctx.currentTime;
      const duration = 0.4;

      const osc = ctx.createOscillator();
      const gain = ctx.createGain();
      osc.connect(gain);
      gain.connect(ctx.destination);
      osc.type = 'sawtooth';

      // Dramatic rising sweep
      osc.frequency.setValueAtTime(400, now);
      osc.frequency.exponentialRampToValueAtTime(1200, now + duration * 0.7);
      osc.frequency.setValueAtTime(1200, now + duration);

      gain.gain.setValueAtTime(0, now);
      gain.gain.linearRampToValueAtTime(0.7, now + 0.01);
      gain.gain.setValueAtTime(0.7, now + duration * 0.7);
      gain.gain.exponentialRampToValueAtTime(0.01, now + duration);

      osc.start(now);
      osc.stop(now + duration);

      // Add a bright high-frequency chime at the end
      setTimeout(() => {
        playPowerfulBeep(1400, 0.2, 0.6, 'sine');
      }, 250);
    };

    switch (type) {
      case 'start':
        // Double beep - attention grabbing start signal
        playAlertPattern(880, 2, 150);
        break;
      case 'end':
        // Triple descending beep - clear end signal
        playPowerfulBeep(700, 0.15, 0.85, 'square');
        setTimeout(() => playPowerfulBeep(550, 0.15, 0.85, 'square'), 120);
        setTimeout(() => playPowerfulBeep(400, 0.25, 0.85, 'square'), 240);
        break;
      case 'correct':
        // Triumphant rising fanfare
        playSuccessFanfare();
        break;
      default:
        // Single powerful beep
        playPowerfulBeep(600, 0.2, 0.75, 'square');
    }
  }, [beepType, lastUpdate, enabled]);
}

