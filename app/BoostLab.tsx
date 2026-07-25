"use client";

import { useCallback, useEffect, useMemo, useRef, useState } from "react";

type Mode = "open" | "p" | "pi" | "pid";
type HistoryPoint = { t: number; voltage: number; target: number; duty: number };
type Settings = {
  mode: Mode;
  kp: number;
  ki: number;
  kd: number;
  target: number;
  input: number;
  load: number;
  pwm: boolean;
  adc: boolean;
  feedback: boolean;
  protection: boolean;
};

type Simulation = {
  running: boolean;
  fault: string;
  time: number;
  voltage: number;
  measuredVoltage: number;
  reference: number;
  duty: number;
  integrator: number;
  previousError: number;
  derivative: number;
  inputCurrent: number;
  loadOverride: number | null;
  inputOverride: number | null;
  disturbanceUntil: number;
  history: HistoryPoint[];
};

const clamp = (value: number, minimum: number, maximum: number) =>
  Math.max(minimum, Math.min(maximum, value));

const initialSimulation = (): Simulation => ({
  running: false,
  fault: "",
  time: 0,
  voltage: 0,
  measuredVoltage: 0,
  reference: 0,
  duty: 0,
  integrator: 0,
  previousError: 0,
  derivative: 0,
  inputCurrent: 0,
  loadOverride: null,
  inputOverride: null,
  disturbanceUntil: 0,
  history: [],
});

const codeFiles = [
  { label: "boost_control.c", path: "./code/boost_control.c" },
  { label: "boost_control.h", path: "./code/boost_control.h" },
  { label: "main.c", path: "./code/main.c" },
];

export function BoostLab() {
  const [settings, setSettings] = useState<Settings>({
    mode: "pi",
    kp: 0.018,
    ki: 0.8,
    kd: 0.001,
    target: 24,
    input: 12,
    load: 24,
    pwm: true,
    adc: true,
    feedback: true,
    protection: true,
  });
  const settingsRef = useRef(settings);
  const simulationRef = useRef<Simulation>(initialSimulation());
  const [snapshot, setSnapshot] = useState<Simulation>(initialSimulation());
  const [status, setStatus] = useState("确认控制链路后启动");
  const [activeCode, setActiveCode] = useState(codeFiles[0]);
  const [source, setSource] = useState("正在读取源码...");
  const [copied, setCopied] = useState(false);

  const ready = settings.pwm && settings.adc && settings.feedback;

  useEffect(() => {
    settingsRef.current = settings;
    if (!settings.pwm || !settings.adc || !settings.feedback) {
      const simulation = simulationRef.current;
      if (simulation.running) {
        simulation.running = false;
        simulation.duty = 0;
        setStatus("控制链路断开，PWM 已停止");
        setSnapshot({ ...simulation, history: [...simulation.history] });
      }
    }
  }, [settings]);

  useEffect(() => {
    let cancelled = false;
    fetch(activeCode.path)
      .then((response) => response.text())
      .then((text) => {
        if (!cancelled) setSource(text);
      })
      .catch(() => {
        if (!cancelled) setSource("源码读取失败，请刷新页面重试。");
      });
    return () => {
      cancelled = true;
    };
  }, [activeCode]);

  const trip = useCallback((reason: string) => {
    const simulation = simulationRef.current;
    if (!settingsRef.current.protection || simulation.fault) return;
    simulation.fault = reason;
    simulation.running = false;
    simulation.duty = 0;
    simulation.integrator = 0;
    setStatus(`${reason}，PWM 已锁止`);
  }, []);

  useEffect(() => {
    let frame = 0;
    let lastFrame = performance.now();
    let accumulator = 0;
    let lastPaint = 0;

    const step = (dt: number) => {
      const simulation = simulationRef.current;
      const control = settingsRef.current;
      simulation.time += dt;

      if (
        simulation.disturbanceUntil > 0 &&
        simulation.time >= simulation.disturbanceUntil
      ) {
        simulation.loadOverride = null;
        simulation.inputOverride = null;
        simulation.disturbanceUntil = 0;
        setStatus("扰动结束，观察恢复过程");
      }

      const inputVoltage = simulation.inputOverride ?? control.input;
      const loadResistance = simulation.loadOverride ?? control.load;

      if (simulation.running && !simulation.fault) {
        const rampStep = 18 * dt;
        simulation.reference += clamp(
          control.target - simulation.reference,
          -rampStep,
          rampStep,
        );

        const error = simulation.reference - simulation.measuredVoltage;
        const feedForward =
          simulation.reference > inputVoltage
            ? clamp(1 - inputVoltage / simulation.reference, 0.02, 0.8)
            : 0.02;
        let command = control.mode === "open" ? 0.42 : feedForward;

        if (control.mode !== "open") {
          command += control.kp * error;

          if (control.mode === "pi" || control.mode === "pid") {
            const nextIntegral =
              simulation.integrator + control.ki * error * dt;
            const nextCommand = command + nextIntegral;
            const blocked =
              (nextCommand >= 0.8 && error > 0) ||
              (nextCommand <= 0.02 && error < 0);
            if (!blocked) {
              simulation.integrator = clamp(nextIntegral, -0.5, 0.5);
            }
            command += simulation.integrator;
          }

          if (control.mode === "pid") {
            const rawDerivative =
              (error - simulation.previousError) / dt;
            simulation.derivative =
              0.88 * simulation.derivative + 0.12 * rawDerivative;
            command += control.kd * simulation.derivative;
          }
        }

        simulation.previousError = error;
        const dutyTarget = clamp(command, 0.02, 0.8);
        simulation.duty += clamp(
          dutyTarget - simulation.duty,
          -1.8 * dt,
          1.8 * dt,
        );

        const outputCurrent =
          simulation.voltage / Math.max(loadResistance, 1);
        const power = simulation.voltage * outputCurrent;
        simulation.inputCurrent =
          power / Math.max(inputVoltage * 0.9, 0.1);

        if (inputVoltage < 7.4) trip("输入欠压");
        if (simulation.voltage > 28.2) trip("输出过压");
        if (simulation.inputCurrent > 4.2) trip("输入过流");
      }

      const activeDuty =
        simulation.running && control.pwm ? simulation.duty : 0;
      const idealOutput =
        inputVoltage / Math.max(1 - activeDuty, 0.15);
      const outputCurrent =
        simulation.voltage / Math.max(loadResistance, 1);
      const loss =
        0.32 * outputCurrent +
        0.045 * simulation.inputCurrent * simulation.inputCurrent;
      const plantTarget = simulation.running
        ? Math.max(inputVoltage, idealOutput - loss)
        : 0;
      const timeConstant = simulation.running ? 0.11 : 0.07;
      simulation.voltage +=
        ((plantTarget - simulation.voltage) * dt) / timeConstant;
      simulation.voltage = clamp(simulation.voltage, 0, 40);

      if (control.adc) {
        simulation.measuredVoltage =
          simulation.voltage + 0.025 * Math.sin(simulation.time * 190);
      }

      if (simulation.time % 0.02 < dt) {
        simulation.history.push({
          t: simulation.time,
          voltage: simulation.voltage,
          target: control.target,
          duty: simulation.duty,
        });
        if (simulation.history.length > 400) simulation.history.shift();
      }
    };

    const animate = (now: number) => {
      const realDelta = Math.min((now - lastFrame) / 1000, 0.05);
      lastFrame = now;
      const simulation = simulationRef.current;

      if (simulation.running || simulation.voltage > 0.02) {
        accumulator += realDelta * 2.2;
        let guard = 0;
        while (accumulator >= 0.001 && guard < 180) {
          step(0.001);
          accumulator -= 0.001;
          guard += 1;
        }
      }

      if (now - lastPaint > 60) {
        setSnapshot({
          ...simulation,
          history: [...simulation.history],
        });
        lastPaint = now;
      }
      frame = requestAnimationFrame(animate);
    };

    frame = requestAnimationFrame(animate);
    return () => cancelAnimationFrame(frame);
  }, [trip]);

  const startOrPause = () => {
    const simulation = simulationRef.current;
    if (!ready || simulation.fault) return;

    simulation.running = !simulation.running;
    if (simulation.running && simulation.time === 0) {
      simulation.voltage = settings.input;
      simulation.measuredVoltage = settings.input;
      simulation.reference = settings.input;
    }
    setStatus(
      simulation.running
        ? settings.protection
          ? "控制器运行中"
          : "运行中，硬件保护未接入"
        : "仿真已暂停",
    );
    setSnapshot({ ...simulation, history: [...simulation.history] });
  };

  const reset = () => {
    const simulation = initialSimulation();
    simulationRef.current = simulation;
    setSnapshot(simulation);
    setStatus("确认控制链路后启动");
  };

  const clearFault = () => {
    const simulation = simulationRef.current;
    if (!simulation.fault) {
      setStatus("当前没有锁存故障");
      return;
    }
    simulation.fault = "";
    simulation.integrator = 0;
    simulation.duty = 0;
    setStatus("故障已清除，需要手动重新启动");
    setSnapshot({ ...simulation, history: [...simulation.history] });
  };

  const disturbLoad = () => {
    const simulation = simulationRef.current;
    if (!simulation.running) return;
    simulation.loadOverride = Math.max(8, settings.load * 0.65);
    simulation.inputOverride = null;
    simulation.disturbanceUntil = simulation.time + 1.5;
    setStatus("负载电阻降低，观察电压跌落和恢复");
  };

  const disturbInput = () => {
    const simulation = simulationRef.current;
    if (!simulation.running) return;
    simulation.inputOverride = Math.max(7, settings.input - 4.5);
    simulation.loadOverride = null;
    simulation.disturbanceUntil = simulation.time + 1.2;
    setStatus("输入电压暂时降低，观察前馈和闭环响应");
  };

  const chart = useMemo(() => {
    const width = 900;
    const height = 280;
    const left = 44;
    const right = 38;
    const top = 16;
    const bottom = 28;
    const plotWidth = width - left - right;
    const plotHeight = height - top - bottom;
    const end = snapshot.history.at(-1)?.t ?? 8;
    const start = Math.max(0, end - 8);
    const visible = snapshot.history.filter((point) => point.t >= start);
    const x = (time: number) =>
      left + ((time - start) / 8) * plotWidth;
    const voltageY = (voltage: number) =>
      top + plotHeight - clamp(voltage / 32, 0, 1) * plotHeight;
    const dutyY = (duty: number) =>
      top + plotHeight - clamp(duty, 0, 1) * plotHeight;
    const path = (
      getter: (point: HistoryPoint) => number,
      scale: (value: number) => number,
    ) =>
      visible
        .map(
          (point, index) =>
            `${index ? "L" : "M"}${x(point.t).toFixed(1)},${scale(
              getter(point),
            ).toFixed(1)}`,
        )
        .join(" ");

    return {
      voltage: path((point) => point.voltage, voltageY),
      target: path((point) => point.target, voltageY),
      duty: path((point) => point.duty, dutyY),
    };
  }, [snapshot.history]);

  const copySource = async () => {
    await navigator.clipboard.writeText(source);
    setCopied(true);
    window.setTimeout(() => setCopied(false), 1200);
  };

  const setNumber = (
    key: "kp" | "ki" | "kd" | "target" | "input" | "load",
    value: number,
  ) => setSettings((current) => ({ ...current, [key]: value }));

  const trackingError =
    (snapshot.running ? snapshot.reference : settings.target) -
    snapshot.measuredVoltage;

  return (
    <main>
      <header className="site-header">
        <div>
          <p className="eyebrow">STM32G431 / DIGITAL POWER</p>
          <h1>Boost Control Lab</h1>
        </div>
        <div className="header-actions">
          <a className="button secondary" href="#source">
            查看代码
          </a>
          <a
            className="button primary"
            href="./downloads/stm32g431-boost-control.zip"
            download
          >
            下载代码包
          </a>
        </div>
      </header>

      <section className="lab-shell" aria-label="Boost PI PID 调试台">
        <aside className="control-panel">
          <div className="section-heading">
            <h2>控制链路</h2>
            <span className={ready ? "ready" : "not-ready"}>
              {ready
                ? settings.protection
                  ? "链路完整"
                  : "缺少保护"
                : "未就绪"}
            </span>
          </div>

          <div className="stage-grid">
            {[
              ["pwm", "PWM 定时器"],
              ["adc", "ADC 采样"],
              ["feedback", "反馈闭环"],
              ["protection", "硬件保护"],
            ].map(([key, label]) => (
              <label className="stage-toggle" key={key}>
                <input
                  type="checkbox"
                  checked={settings[key as keyof Settings] as boolean}
                  onChange={(event) =>
                    setSettings((current) => ({
                      ...current,
                      [key]: event.target.checked,
                    }))
                  }
                />
                <span>{label}</span>
              </label>
            ))}
          </div>

          <div className="flow-line" aria-label="控制信号流">
            <span>PI/PID</span><i>→</i><span>PWM</span><i>→</i>
            <span>Boost</span><i>→</i><span>ADC</span>
          </div>

          <div className="control-group">
            <label htmlFor="mode">控制器</label>
            <select
              id="mode"
              value={settings.mode}
              onChange={(event) =>
                setSettings((current) => ({
                  ...current,
                  mode: event.target.value as Mode,
                }))
              }
            >
              <option value="open">开环</option>
              <option value="p">P</option>
              <option value="pi">PI</option>
              <option value="pid">PID</option>
            </select>
          </div>

          <Slider
            label="Kp"
            value={settings.kp}
            display={settings.kp.toFixed(3)}
            minimum={0}
            maximum={0.08}
            step={0.001}
            onChange={(value) => setNumber("kp", value)}
          />
          <Slider
            label="Ki"
            value={settings.ki}
            display={settings.ki.toFixed(2)}
            minimum={0}
            maximum={4}
            step={0.05}
            disabled={settings.mode === "open" || settings.mode === "p"}
            onChange={(value) => setNumber("ki", value)}
          />
          <Slider
            label="Kd"
            value={settings.kd}
            display={settings.kd.toFixed(4)}
            minimum={0}
            maximum={0.01}
            step={0.0002}
            disabled={settings.mode !== "pid"}
            onChange={(value) => setNumber("kd", value)}
          />

          <div className="divider" />
          <Slider
            label="目标电压"
            value={settings.target}
            display={`${settings.target.toFixed(1)} V`}
            minimum={16}
            maximum={28}
            step={0.5}
            onChange={(value) => setNumber("target", value)}
          />
          <Slider
            label="输入电压"
            value={settings.input}
            display={`${settings.input.toFixed(1)} V`}
            minimum={7}
            maximum={16}
            step={0.5}
            onChange={(value) => setNumber("input", value)}
          />
          <Slider
            label="负载电阻"
            value={settings.load}
            display={`${settings.load.toFixed(0)} Ω`}
            minimum={8}
            maximum={40}
            step={1}
            onChange={(value) => setNumber("load", value)}
          />

          <div className="control-actions">
            <button
              className="button primary"
              type="button"
              disabled={!ready || Boolean(snapshot.fault)}
              onClick={startOrPause}
            >
              {snapshot.running ? "暂停" : snapshot.time > 0 ? "继续" : "启动"}
            </button>
            <button className="button secondary" type="button" onClick={reset}>
              复位
            </button>
          </div>
        </aside>

        <div className="scope-panel">
          <div className="scope-toolbar">
            <div>
              <span className={`state-dot ${snapshot.fault ? "fault" : snapshot.running ? "running" : ""}`} />
              <strong>
                {snapshot.fault
                  ? "FAULT"
                  : snapshot.running
                    ? `${settings.mode.toUpperCase()} 闭环运行`
                    : "等待启动"}
              </strong>
            </div>
            <div className="scope-actions">
              <button type="button" onClick={disturbLoad}>负载突加</button>
              <button type="button" onClick={disturbInput}>输入跌落</button>
              <button type="button" onClick={clearFault}>清除故障</button>
            </div>
          </div>

          <div className="metric-row">
            <Metric label="输出电压" value={`${snapshot.voltage.toFixed(2)} V`} />
            <Metric label="占空比" value={`${(snapshot.duty * 100).toFixed(1)}%`} />
            <Metric label="跟踪误差" value={`${trackingError.toFixed(2)} V`} />
            <Metric label="输入电流" value={`${snapshot.inputCurrent.toFixed(2)} A`} />
          </div>

          <div className="scope-chart">
            <svg viewBox="0 0 900 280" preserveAspectRatio="none" role="img" aria-label="输出电压和占空比变化曲线">
              {[0, 8, 16, 24, 32].map((value) => {
                const y = 16 + 236 - (value / 32) * 236;
                return (
                  <g key={value}>
                    <line className="grid-line" x1="44" y1={y} x2="862" y2={y} />
                    <text className="axis-label" x="36" y={y + 4} textAnchor="end">{value}</text>
                  </g>
                );
              })}
              {[0, 2, 4, 6, 8].map((value) => {
                const x = 44 + (value / 8) * 818;
                return (
                  <g key={value}>
                    <line className="grid-line" x1={x} y1="16" x2={x} y2="252" />
                    <text className="axis-label" x={x} y="272" textAnchor="middle">{value - 8}s</text>
                  </g>
                );
              })}
              <path className="target-line" d={chart.target} />
              <path className="voltage-line" d={chart.voltage} />
              <path className="duty-line" d={chart.duty} />
              <text className="axis-label" x="8" y="16">V</text>
              <text className="axis-label" x="892" y="16" textAnchor="end">D</text>
            </svg>
          </div>

          <div className="chart-legend">
            <span><i className="legend voltage" />输出电压</span>
            <span><i className="legend target" />目标电压</span>
            <span><i className="legend duty" />占空比</span>
          </div>

          <div className={`status-bar ${snapshot.fault ? "fault" : ""}`}>
            <span>{status}</span>
            <code>t = {snapshot.time.toFixed(2)} s</code>
          </div>
        </div>
      </section>

      <section className="practice-band">
        <div>
          <p className="eyebrow">PRACTICE SEQUENCE</p>
          <h2>四步练习</h2>
        </div>
        <ol>
          <li><strong>01</strong><span>默认 PI 启动，观察软启动斜坡</span></li>
          <li><strong>02</strong><span>切到 P，观察负载下的稳态误差</span></li>
          <li><strong>03</strong><span>提高 Kp、Ki，主动制造过冲和振荡</span></li>
          <li><strong>04</strong><span>注入负载和输入扰动，验证保护</span></li>
        </ol>
      </section>

      <section id="source" className="source-section">
        <div className="source-header">
          <div>
            <p className="eyebrow">FIRMWARE SOURCE</p>
            <h2>STM32G431 控制代码</h2>
          </div>
          <div className="source-actions">
            <button className="button secondary" type="button" onClick={copySource}>
              {copied ? "已复制" : "复制当前文件"}
            </button>
            <a className="button primary" href="./downloads/stm32g431-boost-control.zip" download>
              下载完整代码包
            </a>
          </div>
        </div>
        <div className="code-browser">
          <div className="code-tabs" role="tablist" aria-label="源码文件">
            {codeFiles.map((file) => (
              <button
                key={file.path}
                role="tab"
                aria-selected={activeCode.path === file.path}
                className={activeCode.path === file.path ? "active" : ""}
                onClick={() => setActiveCode(file)}
                type="button"
              >
                {file.label}
              </button>
            ))}
          </div>
          <pre><code>{source}</code></pre>
        </div>
        <p className="source-note">
          示例参数默认禁止功率级启动。接入硬件前需根据分压、采样电阻、运放增益和功率器件重新标定。
        </p>
      </section>

      <footer>
        <span>Boost Control Lab</span>
        <span>教学平均模型 · 不替代硬件保护与电路验证</span>
      </footer>
    </main>
  );
}

function Slider({
  label,
  value,
  display,
  minimum,
  maximum,
  step,
  disabled = false,
  onChange,
}: {
  label: string;
  value: number;
  display: string;
  minimum: number;
  maximum: number;
  step: number;
  disabled?: boolean;
  onChange: (value: number) => void;
}) {
  return (
    <label className={`slider-row ${disabled ? "disabled" : ""}`}>
      <span><b>{label}</b><output>{display}</output></span>
      <input
        type="range"
        min={minimum}
        max={maximum}
        step={step}
        value={value}
        disabled={disabled}
        onChange={(event) => onChange(Number(event.target.value))}
      />
    </label>
  );
}

function Metric({ label, value }: { label: string; value: string }) {
  return (
    <div className="metric">
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}
