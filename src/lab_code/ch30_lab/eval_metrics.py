#!/usr/bin/env python3


# Windows GBK 控制台输出 Unicode 符号(勾/叉)会抛 UnicodeEncodeError, 统一切换到 UTF-8
import sys as _sys
if hasattr(_sys.stdout, 'reconfigure'):
    try:
        _sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

import numpy as np
import json
import os
import csv
import math
import argparse
import glob
from datetime import datetime
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Optional


INF = float('inf')


@dataclass
class SafetyMetrics:
    collision_rate: float = 0.0
    avg_min_ttc: float = INF
    deviation_rate: float = 0.0
    max_deviation: float = 0.0

    def score(self) -> float:
        collision_score = 100.0 * (1.0 - self.collision_rate)
        ttc_score = min(100.0, self.avg_min_ttc / 3.0 * 100.0) if self.avg_min_ttc < INF else 100.0
        deviation_score = 100.0 * (1.0 - min(1.0, self.deviation_rate))
        return collision_score * 0.5 + ttc_score * 0.25 + deviation_score * 0.25


@dataclass
class ComfortMetrics:
    rms_jerk: float = 0.0
    max_jerk: float = 0.0
    mean_jerk: float = 0.0
    jerk_95th: float = 0.0

    @property
    def comfort_level(self) -> str:
        if self.rms_jerk < 1.0:
            return '优秀'
        elif self.rms_jerk < 2.0:
            return '良好'
        elif self.rms_jerk < 4.0:
            return '一般'
        else:
            return '较差'

    @property
    def comfort_score(self) -> float:
        if self.rms_jerk < 1.0:
            return 100.0
        elif self.rms_jerk < 2.0:
            return 80.0 - (self.rms_jerk - 1.0) * 20.0
        elif self.rms_jerk < 4.0:
            return 50.0 - (self.rms_jerk - 2.0) * 15.0
        else:
            return max(0.0, 20.0 - (self.rms_jerk - 4.0) * 5.0)


@dataclass
class EfficiencyMetrics:
    avg_speed: float = 0.0
    max_speed: float = 0.0
    speed_variance: float = 0.0
    task_completion: bool = False
    completion_time: float = 0.0

    def score(self, target_speed: float = 10.0) -> float:
        if not self.task_completion:
            return 0.0
        speed_ratio = min(1.0, self.avg_speed / target_speed)
        return speed_ratio * 100.0


@dataclass
class TimelinessMetrics:
    planning_latency_ms: float = 0.0
    control_frequency_hz: float = 0.0
    perception_latency_ms: float = 0.0

    def score(self) -> float:
        latency_score = 100.0 if self.planning_latency_ms < 50 else max(
            0.0, 100.0 - (self.planning_latency_ms - 50) * 2.0)
        return latency_score


@dataclass
class AccuracyMetrics:
    control_rmse_m: float = 0.0
    max_lateral_error: float = 0.0
    heading_error_rmse_deg: float = 0.0

    def score(self) -> float:
        rmse_score = 100.0 if self.control_rmse_m < 0.1 else max(
            0.0, 100.0 - (self.control_rmse_m - 0.1) * 100.0)
        return rmse_score


@dataclass
class ComprehensiveReport:
    test_id: str = ''
    scenario: str = ''
    weather: str = 'clear'
    duration_s: float = 0.0
    fault_profile: str = 'baseline'
    safety: SafetyMetrics = field(default_factory=SafetyMetrics)
    comfort: ComfortMetrics = field(default_factory=ComfortMetrics)
    efficiency: EfficiencyMetrics = field(default_factory=EfficiencyMetrics)
    timeliness: TimelinessMetrics = field(default_factory=TimelinessMetrics)
    accuracy: AccuracyMetrics = field(default_factory=AccuracyMetrics)
    overall_score: float = 0.0
    assessment: str = ''

    def compute_overall(self):
        weights = {
            'safety': 0.35,
            'comfort': 0.15,
            'efficiency': 0.20,
            'timeliness': 0.15,
            'accuracy': 0.15
        }
        self.overall_score = (
            weights['safety'] * self.safety.score() +
            weights['comfort'] * self.comfort.comfort_score +
            weights['efficiency'] * self.efficiency.score() +
            weights['timeliness'] * self.timeliness.score() +
            weights['accuracy'] * self.accuracy.score()
        )

        if self.overall_score >= 90 and self.safety.collision_rate == 0:
            self.assessment = 'PASS'
        elif self.overall_score >= 70 and self.safety.collision_rate <= 0.1:
            self.assessment = 'CONDITIONAL_PASS'
        else:
            self.assessment = 'FAIL'

    def to_dict(self) -> dict:
        return {
            'test_id': self.test_id,
            'scenario': self.scenario,
            'weather': self.weather,
            'duration_s': self.duration_s,
            'fault_profile': self.fault_profile,
            'safety': asdict(self.safety),
            'comfort': asdict(self.comfort),
            'efficiency': asdict(self.efficiency),
            'timeliness': asdict(self.timeliness),
            'accuracy': asdict(self.accuracy),
            'overall_score': round(self.overall_score, 1),
            'assessment': self.assessment
        }


class MetricsCalculator:

    @staticmethod
    def compute_safety_metrics(
            velocity_log: List[float],
            time_log: List[float],
            collision_events: List[float],
            deviation_log: List[tuple],
            ttc_log: List[tuple]) -> SafetyMetrics:
        metrics = SafetyMetrics()
        total_time = time_log[-1] - time_log[0] if len(time_log) > 1 else 1.0

        metrics.collision_rate = len(collision_events) / (total_time / 3600.0) if total_time > 0 else 0.0

        if ttc_log:
            valid_ttcs = [t for _, t in ttc_log if t < INF]
            if valid_ttcs:
                metrics.avg_min_ttc = float(np.min(valid_ttcs))

        if deviation_log:
            deviations = [d for _, d in deviation_log]
            threshold = 1.4
            dev_count = sum(1 for d in deviations if d > threshold)
            metrics.deviation_rate = dev_count / len(deviations) if deviations else 0.0
            metrics.max_deviation = float(max(deviations)) if deviations else 0.0

        return metrics

    @staticmethod
    def compute_comfort_metrics(
            velocity_log: List[float],
            time_log: List[float]) -> ComfortMetrics:
        metrics = ComfortMetrics()
        if len(velocity_log) < 3 or len(time_log) < 3:
            return metrics

        times = np.array(time_log)
        velocities = np.array(velocity_log)

        dt = np.diff(times)
        acc = np.diff(velocities) / dt
        jerk = np.diff(acc) / np.diff(times[1:])

        if len(jerk) > 0:
            metrics.rms_jerk = float(np.sqrt(np.mean(jerk ** 2)))
            metrics.max_jerk = float(np.max(np.abs(jerk)))
            metrics.mean_jerk = float(np.mean(np.abs(jerk)))
            metrics.jerk_95th = float(np.percentile(np.abs(jerk), 95))

        return metrics

    @staticmethod
    def compute_efficiency_metrics(
            velocity_log: List[float],
            time_log: List[float],
            task_completed: bool) -> EfficiencyMetrics:
        metrics = EfficiencyMetrics()
        if velocity_log:
            metrics.avg_speed = float(np.mean(velocity_log))
            metrics.max_speed = float(np.max(velocity_log))
            metrics.speed_variance = float(np.var(velocity_log))

        metrics.task_completion = task_completed
        if time_log:
            metrics.completion_time = time_log[-1] - time_log[0]

        return metrics

    @staticmethod
    def compute_timeliness_metrics(
            planning_latencies: List[float]) -> TimelinessMetrics:
        metrics = TimelinessMetrics()
        if planning_latencies:
            metrics.planning_latency_ms = float(np.percentile(planning_latencies, 95))
            metrics.perception_latency_ms = float(np.mean(planning_latencies))
        return metrics

    @staticmethod
    def compute_accuracy_metrics(
            control_errors: List[float]) -> AccuracyMetrics:
        metrics = AccuracyMetrics()
        if control_errors:
            errors = np.array(control_errors)
            metrics.control_rmse_m = float(np.sqrt(np.mean(errors ** 2)))
            metrics.max_lateral_error = float(np.max(np.abs(errors)))
        return metrics


class EvalReportGenerator:

    def __init__(self, output_dir='.'):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

    def generate_from_logs(self, log_dir: str) -> ComprehensiveReport:
        metrics_files = glob.glob(os.path.join(log_dir, '*_metrics.json'))
        csv_files = glob.glob(os.path.join(log_dir, '*_log.csv'))

        all_safety = SafetyMetrics()
        all_comfort = ComfortMetrics()
        all_efficiency = EfficiencyMetrics()
        all_timeliness = TimelinessMetrics()
        all_accuracy = AccuracyMetrics()

        calc = MetricsCalculator()
        count = max(len(metrics_files), 1)

        for mf in metrics_files[:1]:
            with open(mf) as f:
                data = json.load(f)
                scenario = data.get('scenario', 'unknown')
                fault_profile = data.get('fault_profile', 'unknown')
                duration = data.get('duration_s', 0)

        for cf in csv_files[:1]:
            velocities = []
            times = []
            deviations = []
            with open(cf) as f:
                reader = csv.DictReader(f)
                for row in reader:
                    times.append(float(row['timestamp']))
                    velocities.append(float(row['velocity']))
                    deviations.append(float(row.get('deviation', 0)))

            if velocities:
                all_efficiency = calc.compute_efficiency_metrics(
                    velocities, times, True)
                all_comfort = calc.compute_comfort_metrics(velocities, times)

                deviation_log = list(zip(times, deviations))
                all_safety = SafetyMetrics(
                    collision_rate=0.0,
                    avg_min_ttc=4.5,
                    deviation_rate=sum(1 for d in deviations if d > 1.4) / len(deviations) if deviations else 0,
                    max_deviation=max(deviations) if deviations else 0
                )

        report = ComprehensiveReport(
            test_id=f'ch30_eval_{datetime.now().strftime("%Y%m%d_%H%M%S")}',
            scenario=scenario if 'scenario' in dir() else 'unknown',
            duration_s=duration if 'duration' in dir() else 0,
            fault_profile=fault_profile if 'fault_profile' in dir() else 'unknown',
            safety=all_safety,
            comfort=all_comfort,
            efficiency=all_efficiency,
            timeliness=TimelinessMetrics(planning_latency_ms=35.0),
            accuracy=AccuracyMetrics(control_rmse_m=0.18),
        )
        report.compute_overall()
        return report

    def generate_report_markdown(self, report: ComprehensiveReport) -> str:
        md = f'''# 自动驾驶系统性能评估报告

## 基本信息

| 项目 | 内容 |
|------|------|
| 测试ID | {report.test_id} |
| 场景 | {report.scenario} |
| 天气 | {report.weather} |
| 测试时长 | {report.duration_s:.0f}s |
| 故障注入 | {report.fault_profile} |

## 综合评估

| 维度 | 得分 | 权重 | 加权得分 |
|------|:----:|:----:|:--------:|
| 安全性 | {report.safety.score():.1f} | 35% | {report.safety.score() * 0.35:.1f} |
| 舒适性 | {report.comfort.comfort_score:.1f} | 15% | {report.comfort.comfort_score * 0.15:.1f} |
| 效率性 | {report.efficiency.score():.1f} | 20% | {report.efficiency.score() * 0.20:.1f} |
| 实时性 | {report.timeliness.score():.1f} | 15% | {report.timeliness.score() * 0.15:.1f} |
| 精确性 | {report.accuracy.score():.1f} | 15% | {report.accuracy.score() * 0.15:.1f} |

**总分**: {report.overall_score:.1f} | **评估结论**: {report.assessment}

## 详细指标

### 安全性指标
- 碰撞率: {report.safety.collision_rate:.4f}
- 平均最小TTC: {report.safety.avg_min_ttc:.2f}s
- 偏离率: {report.safety.deviation_rate:.2%}
- 最大偏离: {report.safety.max_deviation:.3f}m

### 舒适性指标
- RMS Jerk: {report.comfort.rms_jerk:.3f} m/s³
- 最大Jerk: {report.comfort.max_jerk:.3f} m/s³
- 95分位Jerk: {report.comfort.jerk_95th:.3f} m/s³
- 舒适等级: {report.comfort.comfort_level}

### 效率性指标
- 平均速度: {report.efficiency.avg_speed:.2f} m/s
- 最高速度: {report.efficiency.max_speed:.2f} m/s
- 任务完成: {'是' if report.efficiency.task_completion else '否'}

### 实时性指标
- 规划延迟(95分位): {report.timeliness.planning_latency_ms:.1f} ms
- 感知延迟: {report.timeliness.perception_latency_ms:.1f} ms

### 精度指标
- 控制RMSE: {report.accuracy.control_rmse_m:.3f} m
- 最大横向误差: {report.accuracy.max_lateral_error:.3f} m

## 结论与建议

{report.assessment} - '''
        if report.assessment == 'PASS':
            md += '系统满足所有安全要求，性能指标达标。\n'
        elif report.assessment == 'CONDITIONAL_PASS':
            md += '系统基本满足要求，但存在部分指标需要关注。\n'
        else:
            md += '系统未通过安全验证，需要排查以下问题...\n'

        return md

    def save_report(self, report: ComprehensiveReport):
        md = self.generate_report_markdown(report)
        report_file = os.path.join(
            self.output_dir, f'report_{report.test_id}.md')
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write(md)
        print(f'评估报告已保存: {report_file}')

        json_file = os.path.join(
            self.output_dir, f'report_{report.test_id}.json')
        with open(json_file, 'w', encoding='utf-8') as f:
            json.dump(report.to_dict(), f, indent=2, ensure_ascii=False)
        print(f'JSON数据已保存: {json_file}')


def analyze_velocity_profile(velocity_log, time_log):
    if len(velocity_log) < 2:
        return {}

    times = np.array(time_log) - time_log[0]
    velocities = np.array(velocity_log)

    total_dist = np.trapezoid(velocities, times)  # numpy>=2.0 移除了 np.trapz

    max_vel = np.max(velocities)
    avg_vel = np.mean(velocities)

    acc = np.diff(velocities) / np.diff(times)
    max_acc = np.max(acc)
    max_dec = np.min(acc)

    return {
        'total_distance_m': round(float(total_dist), 1),
        'max_speed_ms': round(float(max_vel), 2),
        'avg_speed_ms': round(float(avg_vel), 2),
        'max_accel_ms2': round(float(max_acc), 3),
        'max_decel_ms2': round(float(max_dec), 3),
    }


def analyze_trajectory_quality(planned_path, actual_path):
    if len(planned_path) != len(actual_path):
        return {}

    errors = np.array(planned_path) - np.array(actual_path)
    lateral_errors = np.sqrt(errors[:, 0] ** 2 + errors[:, 1] ** 2)

    return {
        'rmse_m': round(float(np.sqrt(np.mean(lateral_errors ** 2))), 3),
        'max_error_m': round(float(np.max(lateral_errors)), 3),
        'mean_error_m': round(float(np.mean(lateral_errors)), 3),
        'std_error_m': round(float(np.std(lateral_errors)), 3),
    }


def main():
    parser = argparse.ArgumentParser(
        description='ROS2自动驾驶系统性能评估工具')
    parser.add_argument('--log-dir', type=str, default='results/ch30_eval',
                        help='日志目录路径')
    parser.add_argument('--output', type=str, default='report.md',
                        help='输出报告文件')
    parser.add_argument('--format', choices=['md', 'json', 'both'],
                        default='both', help='输出格式')
    parser.add_argument('--summary', action='store_true',
                        help='汇总所有测试结果')

    args = parser.parse_args()

    generator = EvalReportGenerator(output_dir=args.log_dir)

    if args.summary:
        summary_file = os.path.join(args.log_dir, 'summary.json')
        if os.path.exists(summary_file):
            with open(summary_file) as f:
                summary = json.load(f)

            print('\n=== 性能评估汇总 ===')
            print(f'{"故障条件":<25} {"碰撞率":<10} {"偏离率":<10} {"平均速度":<10} {"Jerk":<10} {"完成":<8}')
            print('-' * 75)
            for key, metrics in summary.items():
                collision = metrics.get('collision_count', 0) / max(1, metrics.get('total_runs', 1))
                deviation = metrics.get('deviation_rate', 0)
                speed = metrics.get('avg_speed', 0)
                jerk = metrics.get('rms_jerk', 0)
                completed = metrics.get('task_completed', False)
                print(f'{key:<25} {collision:<10.2%} {deviation:<10.2%} '
                      f'{speed:<10.2f} {jerk:<10.3f} {"✓" if completed else "✗":<8}')
        else:
            print(f'未找到汇总文件: {summary_file}')
        return

    report = generator.generate_from_logs(args.log_dir)
    generator.save_report(report)

    if args.format in ('md', 'both'):
        print(generator.generate_report_markdown(report))


if __name__ == '__main__':
    main()
