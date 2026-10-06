/* Each browser owns its audio output and acknowledgement independently. */
class ProtectiveStopAlarm {
  constructor() {
    this.panel = document.querySelector('#protective-stop-alarm');
    this.status = document.querySelector('#alarm-status');
    this.enableButton = document.querySelector('#alarm-enable');
    this.testButton = document.querySelector('#alarm-test');
    this.silenceButton = document.querySelector('#alarm-silence');
    this.volume = document.querySelector('#alarm-volume');
    this.context = null;
    this.source = null;
    this.testSource = null;
    this.enabled = false;
    this.active = false;
    this.silenced = false;
    this.statusKnown = false;
    this.error = '';
    this.AudioContext = window.AudioContext || window.webkitAudioContext;
    try {
      const saved = localStorage.getItem('am-operator-alarm-volume');
      if (saved !== null && Number.isFinite(Number(saved))) {
        this.volume.value = Math.max(0, Math.min(100, Number(saved)));
      }
    } catch (_) { /* Audio still works if storage is unavailable. */ }
    this.enableButton.addEventListener('click', () => {
      if (this.enabled && this.context?.state === 'running') {
        this.enabled = false;
        this.stopTest();
        this.sync();
      } else {
        this.enable();
      }
    });
    this.testButton.addEventListener('click', () => this.test());
    this.silenceButton.addEventListener('click', () => {
      if (this.active) this.silenced = true;
      this.sync();
    });
    this.volume.addEventListener('input', () => {
      if (this.gain) this.gain.gain.setTargetAtTime(this.level(), this.context.currentTime, 0.03);
      try { localStorage.setItem('am-operator-alarm-volume', this.volume.value); } catch (_) {}
      this.render();
    });
    this.render();
  }

  level() { return Number(this.volume.value) / 100 * 0.6; }

  async enable() {
    try {
      if (!this.context) {
        this.context = new this.AudioContext();
        this.gain = this.context.createGain();
        this.gain.gain.value = this.level();
        this.gain.connect(this.context.destination);
        this.buffer = this.makeKlaxon();
        this.context.onstatechange = () => this.sync();
      }
      // Invoked directly from a click so browser autoplay policy permits resume.
      await this.context.resume();
      this.enabled = this.context.state === 'running';
      this.error = this.enabled ? '' : 'Audio blocked — click Enable alarm sound again.';
      this.sync();
      return this.enabled;
    } catch (_) {
      this.error = 'Audio unavailable — check browser sound settings and try again.';
      this.enabled = false;
      this.sync();
      return false;
    }
  }

  makeKlaxon() {
    // A rising/falling horn with strong harmonics, followed by a short pause.
    // Loop a complete audio buffer so the horn needs no JavaScript timer.
    const rate = this.context.sampleRate;
    const buffer = this.context.createBuffer(1, Math.ceil(rate * 2.8), rate);
    const samples = buffer.getChannelData(0);
    let phase = 0;
    for (let i = 0; i < samples.length; i++) {
      const t = i / rate;
      if (t >= 2.1) continue;
      const sweep = t < 0.35 ? t / 0.35 : t < 1.45 ? 1 : (2.1 - t) / 0.65;
      phase += 2 * Math.PI * (155 + 255 * sweep) / rate;
      const envelope = Math.min(1, t / 0.035, (2.1 - t) / 0.08);
      const rasp = Math.sin(phase) + 0.45 * Math.sin(2 * phase) + 0.25 * Math.sin(3 * phase);
      samples[i] = envelope * rasp / 1.7 * (0.88 + 0.12 * Math.sin(2 * Math.PI * 28 * t));
    }
    return buffer;
  }

  play(loop) {
    const source = this.context.createBufferSource();
    source.buffer = this.buffer;
    source.loop = loop;
    source.connect(this.gain);
    source.start();
    return source;
  }

  stop(source) {
    if (source) {
      source.onended = null;
      source.stop();
      source.disconnect();
    }
  }

  stopTest() {
    this.stop(this.testSource);
    this.testSource = null;
  }

  async test() {
    if (!await this.enable()) return;
    if (this.active && !this.silenced) return; // The actual alarm is already playing.
    this.stopTest();
    this.testSource = this.play(false);
    this.testSource.onended = () => {
      this.testSource.disconnect();
      this.testSource = null;
    };
  }

  update(dashboard) {
    this.statusKnown = !!dashboard?.safety_mode && (dashboard.safety_mode_available
      ?? (dashboard.available === true && !dashboard.stale));
    if (this.statusKnown) {
      const active = dashboard.safety_mode === 'PROTECTIVE_STOP';
      if (!active) this.silenced = false;
      this.active = active;
    }
    // Missing/stale data cannot acknowledge or clear a confirmed stop.
    this.sync();
  }

  sync() {
    const shouldPlay = this.enabled && this.active && !this.silenced;
    if (shouldPlay && !this.source) {
      this.stopTest();
      this.source = this.play(true);
    } else if (!shouldPlay && this.source) {
      this.stop(this.source);
      this.source = null;
    }
    this.render();
  }

  render() {
    const ready = this.enabled && this.context?.state === 'running';
    let text = ready ? 'Alarm sound ready in this browser.' : 'Click Enable alarm sound in each browser.';
    if (!this.AudioContext) text = 'This browser does not support alarm audio.';
    else if (this.error) text = this.error;
    if (this.active) {
      text = `PROTECTIVE STOP — ${this.silenced ? 'alarm silenced in this browser' : ready ? 'alarm sounding' : 'enable sound in this browser'}.`;
      if (this.error) text += ` ${this.error}`;
    }
    if (Number(this.volume.value) === 0) text += ' Volume is muted.';
    if (!this.statusKnown) text += ' Waiting for live arm safety status.';
    if (this.status.textContent !== text) this.status.textContent = text;
    this.panel.classList.toggle('alarm-active', this.active);
    this.enableButton.textContent = ready ? 'Disable alarm sound' : 'Enable alarm sound';
    this.enableButton.disabled = !this.AudioContext;
    this.testButton.disabled = !this.AudioContext;
    this.silenceButton.disabled = !this.active || this.silenced;
  }
}
