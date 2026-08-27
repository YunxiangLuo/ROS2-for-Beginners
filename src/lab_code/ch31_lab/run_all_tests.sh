#!/bin/bash
# =============================================================================
# 第31章 综合项目：城区自动驾驶 - 测试套件
#
# 运行所有测试：单元测试、集成测试、回归测试
# 用法:
#   ./run_all_tests.sh                    # 运行全部测试
#   ./run_all_tests.sh --unit             # 仅单元测试
#   ./run_all_tests.sh --integration      # 仅集成测试
#   ./run_all_tests.sh --quick            # 快速测试（跳过集成测试）
#   ./run_all_tests.sh --coverage         # 生成覆盖率报告
#   ./run_all_tests.sh --list             # 列出所有测试
#   ./run_all_tests.sh --report           # 仅生成报告（不运行）
# =============================================================================

set -euo pipefail

# ── 颜色 ──
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
NC='\033[0m'

# ── 配置 ──
ROS_WS="${ROS_WS:-$HOME/ros2_course_ws}"
REPORT_DIR="${REPORT_DIR:-./test_reports}"
CARLA_ENABLED=false
COVERAGE=false
QUICK_MODE=false
LIST_ONLY=false
REPORT_ONLY=false
RUN_UNIT=true
RUN_INTEGRATION=true

# ── 解析参数 ──
while [[ $# -gt 0 ]]; do
    case "$1" in
        --unit)         RUN_INTEGRATION=false; shift ;;
        --integration)  RUN_UNIT=false; shift ;;
        --quick)        QUICK_MODE=true; RUN_INTEGRATION=false; shift ;;
        --coverage)     COVERAGE=true; shift ;;
        --list)         LIST_ONLY=true; shift ;;
        --report)       REPORT_ONLY=true; shift ;;
        --help|-h)      echo "用法: $0 [--unit|--integration|--quick|--coverage|--list|--report]"; exit 0 ;;
        *)              echo "未知参数: $1"; exit 1 ;;
    esac
done

mkdir -p "$REPORT_DIR"

# ── 工具函数 ──
print_banner() {
    echo -e "${CYAN}╔══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║   ${WHITE}第31章 综合项目 测试套件${NC}"
    printf "${CYAN}║   ${WHITE}%s${NC}\n" "$(date '+%Y-%m-%d %H:%M:%S')"
    echo -e "${CYAN}╚══════════════════════════════════════════════════════════╝${NC}"
    echo
}

print_result() {
    local name=$1
    local status=$2
    local detail=$3

    if [ "$status" -eq 0 ]; then
        echo -e "  ${GREEN}[PASS]${NC} $name"
    else
        echo -e "  ${RED}[FAIL]${NC} $name"
        echo -e "        ${detail:+${YELLOW}→ $detail${NC}}"
    fi
}

# ── 检查依赖 ──
check_dependencies() {
    echo -e "${BLUE}[SETUP] 检查测试依赖...${NC}"

    local missing=0

    # Python测试框架
    for pkg in pytest pytest-cov numpy opencv-python; do
        if python3 -c "import $(echo $pkg | tr '-' '_')" 2>/dev/null; then
            echo -e "  ${GREEN}[OK]${NC} $pkg"
        else
            echo -e "  ${YELLOW}[MISS]${NC} $pkg"
            missing=1
        fi
    done

    # ROS2
    if [ -n "${ROS_DISTRO:-}" ]; then
        echo -e "  ${GREEN}[OK]${NC} ROS2 $ROS_DISTRO"
    elif [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
        echo -e "  ${GREEN}[OK]${NC} ROS2 jazzy (auto-loaded)"
    else
        echo -e "  ${YELLOW}[WARN]${NC} ROS2 not found (集成测试将跳过)"
        RUN_INTEGRATION=false
    fi

    if [ $missing -eq 1 ]; then
        echo -e "${YELLOW}[提示] 安装依赖: pip install pytest pytest-cov numpy opencv-python${NC}"
    fi

    # CARLA
    if python3 -c "import carla" 2>/dev/null; then
        CARLA_ENABLED=true
        echo -e "  ${GREEN}[OK]${NC} CARLA Python API"
    else
        echo -e "  ${YELLOW}[WARN]${NC} CARLA Python API未安装 (CARLA相关测试将跳过)"
    fi
}

# ── 收集测试列表 ──
collect_tests() {
    echo -e "\n${BLUE}[INFO] 可用的测试:${NC}"

    local test_dir="./test"
    if [ -d "$test_dir" ]; then
        for f in "$test_dir"/test_*.py; do
            if [ -f "$f" ]; then
                local name=$(basename "$f" .py)
                local desc=$(head -20 "$f" | grep -E '"""|测试|验证' | head -1 | tr -d '"#' | xargs)
                echo -e "  ${CYAN}${name}${NC}"
                [ -n "$desc" ] && echo -e "    → $desc"
            fi
        done
    fi
}

# ── 运行单元测试 ──
run_unit_tests() {
    echo -e "\n${BLUE}═══════════════════════════════════════════════${NC}"
    echo -e "${BLUE}  单元测试${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════${NC}"

    local test_dir="./test"
    if [ ! -d "$test_dir" ]; then
        echo -e "${YELLOW}[SKIP] 测试目录不存在: $test_dir${NC}"
        return 0
    fi

    local test_files=("$test_dir"/test_*.py)
    if [ ${#test_files[@]} -eq 0 ]; then
        echo -e "${YELLOW}[SKIP] 未找到测试文件${NC}"
        return 0
    fi

    local total=0
    local passed=0
    local failed=0

    for test_file in "${test_files[@]}"; do
        local test_name=$(basename "$test_file" .py)

        # 跳过集成的测试（在quick模式）
        if [ "$QUICK_MODE" = true ] && echo "$test_name" | grep -q "integration"; then
            echo -e "  ${YELLOW}[SKIP]${NC} $test_name (快速模式)"
            continue
        fi

        echo -e "\n${CYAN}[TEST] ${test_name}${NC}"

        local cmd="python3 -m pytest $test_file -v --tb=short"
        if [ "$COVERAGE" = true ]; then
            cmd="$cmd --cov=../ --cov-report=term --cov-report=html:$REPORT_DIR/coverage"
        fi

        set +e
        output=$($cmd 2>&1)
        exit_code=$?
        set -e

        total=$((total + 1))

        if [ $exit_code -eq 0 ] || [ $exit_code -eq 5 ]; then
            # exit 5 表示无测试被选中（pytest行为）
            if echo "$output" | grep -q "passed"; then
                echo "$output" | grep -E "PASSED|FAILED|ERROR" | sed 's/^/    /'
                print_result "$test_name" 0
                passed=$((passed + 1))
            elif echo "$output" | grep -q "no tests ran"; then
                echo -e "  ${YELLOW}[SKIP]${NC} $test_name (无测试函数)"
            else
                print_result "$test_name" 0
                passed=$((passed + 1))
            fi
        else
            echo "$output" | tail -20 | sed 's/^/    /'
            print_result "$test_name" 1 "exit code $exit_code"
            failed=$((failed + 1))
        fi

        # 保存详细日志
        echo "$output" > "$REPORT_DIR/${test_name}.log"
    done

    echo
    echo -e "${WHITE}单元测试结果: ${passed}/${total} 通过, ${failed} 失败${NC}"

    return $failed
}

# ── 运行集成测试 ──
run_integration_tests() {
    echo -e "\n${BLUE}═══════════════════════════════════════════════${NC}"
    echo -e "${BLUE}  集成测试${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════${NC}"

    if [ ! -f "${ROS_WS}/install/setup.bash" ]; then
        echo -e "${YELLOW}[SKIP] ROS2工作空间未编译${NC}"
        return 0
    fi

    source "${ROS_WS}/install/setup.bash"

    if [ "$CARLA_ENABLED" = false ]; then
        echo -e "${YELLOW}[SKIP] CARLA未安装，跳过集成测试${NC}"
        return 0
    fi

    local integration_tests=(
        "test_integration"
    )

    local total=0
    local passed=0
    local failed=0

    for test_name in "${integration_tests[@]}"; do
        local test_file="./test/${test_name}.py"
        if [ ! -f "$test_file" ]; then
            echo -e "  ${YELLOW}[SKIP]${NC} $test_name (文件不存在)"
            continue
        fi

        echo -e "\n${CYAN}[INTEGRATION] ${test_name}${NC}"
        echo -e "${YELLOW}[INFO] 集成测试需要CARLA服务器在运行${NC}"
        echo -e "${YELLOW}[INFO] 确保已启动: ./CarlaUE4.sh${NC}"

        local cmd="python3 -m pytest $test_file -v --tb=long -x --timeout=120"
        set +e
        output=$($cmd 2>&1)
        exit_code=$?
        set -e

        total=$((total + 1))

        if [ $exit_code -eq 0 ]; then
            print_result "$test_name" 0
            passed=$((passed + 1))
        else
            echo "$output" | tail -30 | sed 's/^/    /'
            print_result "$test_name" 1 "集成测试失败"
            failed=$((failed + 1))
        fi

        echo "$output" > "$REPORT_DIR/${test_name}.log"
    done

    echo
    echo -e "${WHITE}集成测试结果: ${passed}/${total} 通过, ${failed} 失败${NC}"

    return $failed
}

# ── 静态代码检查 ──
run_lint() {
    echo -e "\n${BLUE}═══════════════════════════════════════════════${NC}"
    echo -e "${BLUE}  静态代码检查${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════${NC}"

    local lint_passed=0

    # pylint
    if command -v pylint &>/dev/null; then
        echo -e "${CYAN}[LINT] pylint 检查...${NC}"
        local lint_targets=(./main_pipeline.py)
        for module in carla_sensor_driver perception_node localization_node \
            planning_node control_node safety_monitor_node; do
            if [ -d "./$module" ]; then
                lint_targets+=("./$module/")
            fi
        done
        set +e
        pylint --exit-zero --reports=n \
            "${lint_targets[@]}" \
            > "$REPORT_DIR/pylint_report.txt" 2>&1
        local score=$(grep -oP 'rated at \K[\d.]+' "$REPORT_DIR/pylint_report.txt" | tail -1)
        set -e
        if [ -n "$score" ] && (( $(echo "$score >= 8.0" | bc -l) )); then
            echo -e "  ${GREEN}[PASS]${NC} pylint: $score"
        else
            echo -e "  ${YELLOW}[WARN]${NC} pylint: ${score:-N/A} (目标: ≥8.0)"
            lint_passed=1
        fi
    else
        echo -e "  ${YELLOW}[SKIP]${NC} pylint未安装"
    fi

    # flake8
    if command -v flake8 &>/dev/null; then
        echo -e "${CYAN}[LINT] flake8 检查...${NC}"
        local lint_targets=(./main_pipeline.py)
        for module in carla_sensor_driver perception_node localization_node \
            planning_node control_node safety_monitor_node; do
            if [ -d "./$module" ]; then
                lint_targets+=("./$module/")
            fi
        done
        set +e
        flake8 \
            --max-line-length=120 \
            --extend-ignore=E402 \
            "${lint_targets[@]}" \
            > "$REPORT_DIR/flake8_report.txt" 2>&1
        local flake8_exit=$?
        set -e
        if [ $flake8_exit -eq 0 ]; then
            echo -e "  ${GREEN}[PASS]${NC} flake8: 无错误"
        else
            local errors=$(wc -l < "$REPORT_DIR/flake8_report.txt")
            echo -e "  ${YELLOW}[WARN]${NC} flake8: ${errors}个错误"
            lint_passed=1
        fi
    else
        echo -e "  ${YELLOW}[SKIP]${NC} flake8未安装"
    fi

    return $lint_passed
}

# ── 生成测试报告 ──
generate_report() {
    echo -e "\n${BLUE}[REPORT] 生成测试报告...${NC}"

    local report_file="$REPORT_DIR/test_report_$(date +%Y%m%d_%H%M%S).html"

    # 汇总结果
    {
        echo "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        echo "<title>第31章 综合项目 - 测试报告</title>"
        echo "<style>
            body { font-family: 'Segoe UI', Arial, sans-serif; margin: 40px; background: #f5f5f5; }
            h1 { color: #2c3e50; border-bottom: 3px solid #3498db; padding-bottom: 10px; }
            h2 { color: #34495e; margin-top: 30px; }
            .summary { background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
            .pass { color: #27ae60; font-weight: bold; }
            .fail { color: #e74c3c; font-weight: bold; }
            .warn { color: #f39c12; font-weight: bold; }
            table { width: 100%; border-collapse: collapse; margin: 10px 0; }
            th, td { padding: 10px; text-align: left; border-bottom: 1px solid #ddd; }
            th { background: #3498db; color: white; }
            tr:hover { background: #f0f0f0; }
            .log { background: #2c3e50; color: #ecf0f1; padding: 15px; border-radius: 5px; overflow-x: auto; }
            .timestamp { color: #95a5a6; font-size: 0.9em; }
        </style></head><body>"

        echo "<h1>📊 第31章 综合项目测试报告</h1>"
        echo "<p class='timestamp'>生成时间: $(date '+%Y-%m-%d %H:%M:%S')</p>"

        echo "<div class='summary'>"
        echo "<h2>测试概要</h2>"
        echo "<table>"
        echo "<tr><th>类别</th><th>总数</th><th>通过</th><th>失败</th><th>通过率</th></tr>"

        local unit_passed=0
        local unit_total=0
        local unit_failed=0
        for log in "$REPORT_DIR"/test_*.log; do
            [ -f "$log" ] || continue
            if grep -qE "passed|PASSED" "$log" 2>/dev/null; then
                unit_passed=$((unit_passed + 1))
            elif grep -qE "failed|FAILED|ERROR" "$log" 2>/dev/null; then
                unit_failed=$((unit_failed + 1))
            fi
            unit_total=$((unit_total + 1))
        done

        local pass_rate=0
        [ $unit_total -gt 0 ] && pass_rate=$((unit_passed * 100 / unit_total))

        echo "<tr><td>单元测试</td><td>$unit_total</td><td class='pass'>$unit_passed</td><td class='fail'>$unit_failed</td><td>$pass_rate%</td></tr>"

        echo "</table>"

        echo "<h2>模块延迟统计</h2>"
        echo "<table><tr><th>模块</th><th>平均延迟</th><th>目标</th><th>状态</th></tr>"
        for module in "感知:perception:50ms" "定位:localization:20ms" "规划:planning:50ms" "控制:control:10ms" "安全:safety:10ms"; do
            IFS=':' read -r name key target <<< "$module"
            echo "<tr><td>$name</td><td>—</td><td>$target</td><td class='warn'>待运行</td></tr>"
        done
        echo "</table>"

        echo "</div>"

        # 各测试详情
        echo "<h2>测试详情</h2>"
        for log in "$REPORT_DIR"/test_*.log; do
            [ -f "$log" ] || continue
            local basename=$(basename "$log" .log)
            echo "<h3>$basename</h3>"
            echo "<pre class='log'>"
            tail -50 "$log" | sed 's/</\&lt;/g; s/>/\&gt;/g'
            echo "</pre>"
        done

        echo "</body></html>"
    } > "$report_file"

    echo -e "${GREEN}[REPORT] 报告已生成: $report_file${NC}"
}

# ── 主流程 ──
main() {
    print_banner

    if [ "$LIST_ONLY" = true ]; then
        collect_tests
        exit 0
    fi

    if [ "$REPORT_ONLY" = true ]; then
        generate_report
        exit 0
    fi

    check_dependencies

    local exit_code=0

    # 静态检查
    run_lint || exit_code=$?

    # 单元测试
    if [ "$RUN_UNIT" = true ]; then
        run_unit_tests || exit_code=$?
    fi

    # 集成测试
    if [ "$RUN_INTEGRATION" = true ]; then
        run_integration_tests || exit_code=$?
    fi

    # 生成报告
    generate_report

    echo
    echo -e "${BLUE}═══════════════════════════════════════════════${NC}"
    if [ $exit_code -eq 0 ]; then
        echo -e "${GREEN}  所有测试通过!${NC}"
    else
        echo -e "${RED}  部分测试失败 (exit code: $exit_code)${NC}"
        echo -e "${YELLOW}  详情请查看: $REPORT_DIR/${NC}"
    fi
    echo -e "${BLUE}═══════════════════════════════════════════════${NC}"

    exit $exit_code
}

main
