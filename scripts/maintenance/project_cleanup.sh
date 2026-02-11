#!/bin/bash
# -*- coding: utf-8 -*-
################################################################################
# Claudia项目自动化清理脚本
#
# 功能: 清理废弃文件、归档开发文档、优化项目结构
# 作者: Claude Code
# 日期: 2025-10-31
# 版本: 1.0
#
# 使用方法:
#   bash scripts/maintenance/project_cleanup.sh --mode=safe    # 安全模式（仅缓存清理）
#   bash scripts/maintenance/project_cleanup.sh --mode=full    # 完整清理（包括归档）
#   bash scripts/maintenance/project_cleanup.sh --mode=preview # 预览模式（不执行）
################################################################################

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 项目根目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "$PROJECT_ROOT"

# 时间戳
TIMESTAMP=$(date '+%Y%m%d_%H%M%S')
BACKUP_DIR="${PROJECT_ROOT}/backups/cleanup_${TIMESTAMP}"
REPORT_FILE="${PROJECT_ROOT}/logs/cleanup_report_${TIMESTAMP}.log"

# 默认模式
MODE="${1:-safe}"
if [[ "$MODE" == --mode=* ]]; then
    MODE="${MODE#--mode=}"
fi

# 统计变量
DELETED_FILES=0
DELETED_DIRS=0
ARCHIVED_FILES=0
FREED_SPACE=0

################################################################################
# 辅助函数
################################################################################

print_header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

# 记录到日志文件
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" >> "$REPORT_FILE"
}

# 创建备份
create_backup() {
    local source="$1"
    local backup_path="${BACKUP_DIR}/${source}"

    if [[ -e "$source" ]]; then
        mkdir -p "$(dirname "$backup_path")"
        cp -r "$source" "$backup_path"
        log "Backed up: $source -> $backup_path"
    fi
}

# 计算文件/目录大小（MB）
get_size_mb() {
    local path="$1"
    if [[ -e "$path" ]]; then
        du -sm "$path" 2>/dev/null | cut -f1
    else
        echo "0"
    fi
}

################################################################################
# Phase 1: Python缓存清理
################################################################################

cleanup_python_cache() {
    print_header "Phase 1: 清理Python缓存文件"

    local cache_count=0
    local cache_size=0

    # 统计缓存大小
    cache_size=$(find . -type d -name "__pycache__" -exec du -sm {} + 2>/dev/null | awk '{sum+=$1} END {print sum}')
    cache_size=${cache_size:-0}

    print_info "发现Python缓存大小: ${cache_size} MB"

    if [[ "$MODE" != "preview" ]]; then
        # 删除 __pycache__ 目录
        while IFS= read -r dir; do
            if [[ -n "$dir" ]]; then
                rm -rf "$dir"
                ((cache_count++))
                log "Deleted: $dir"
            fi
        done < <(find . -type d -name "__pycache__" 2>/dev/null)

        # 删除 .pyc 文件
        find . -type f -name "*.pyc" -delete 2>/dev/null

        # 删除 pytest 缓存
        find . -type d -name ".pytest_cache" -exec rm -rf {} + 2>/dev/null || true

        DELETED_DIRS=$((DELETED_DIRS + cache_count))
        FREED_SPACE=$((FREED_SPACE + cache_size))

        print_success "清理了 ${cache_count} 个缓存目录，释放 ${cache_size} MB"
    else
        print_info "[预览] 将清理 ${cache_count} 个缓存目录"
    fi

    log "Phase 1 completed: Cleaned ${cache_count} cache directories"
}

################################################################################
# Phase 2: 归档开发文档
################################################################################

archive_development_docs() {
    print_header "Phase 2: 归档开发过程文档"

    local archive_dir="${PROJECT_ROOT}/docs/archive/development"

    # 保留的核心文档
    local keep_docs=(
        "GO2_SUPPORTED_ACTIONS.md"
        "CORRECT_LLM_ARCHITECTURE.md"
        "guides/LED_USAGE_GUIDE.md"
        "guides/LED_SYSTEM_VERIFICATION_REPORT.md"
    )

    # 创建归档目录
    if [[ "$MODE" != "preview" ]]; then
        mkdir -p "$archive_dir"
    fi

    # 需要归档的文档模式
    local archive_patterns=(
        "BRAIN_OPTIMIZATION_REPORT.md"
        "COMPLETE_API_ANALYSIS.md"
        "COMPLEX_ACTION_SEQUENCE_RESEARCH.md"
        "DANCE_SELECTION_GUIDE.md"
        "ENHANCED_JAPANESE_COMMANDER.md"
        "FINAL_*.md"
        "FIX_REPORT_*.md"
        "HARDWARE_FIX_REPORT.md"
        "HYBRID_ARCHITECTURE_*.md"
        "INTERACTIVE_JAPANESE_COMMANDER.md"
        "LLM_FRAMEWORK_ANALYSIS.md"
        "NETWORK_CONFIG_FIX.md"
        "NEW_BRAIN_ARCHITECTURE_SUCCESS.md"
        "PHASE2_OPTIMIZATION_REPORT.md"
        "PRODUCTION_DEPLOYMENT_SUCCESS.md"
        "PROMPT_*.md"
        "REAL_*.md"
        "SDK_LIMITATION_ANALYSIS.md"
        "SPORT_MODE_*.md"
        "STATE_MANAGEMENT_FIX.md"
        "TASK11_OPTIMIZATION_SUMMARY.md"
        "TASKMASTER_UPDATE_COMPLETE_*.md"
    )

    local archived=0

    for pattern in "${archive_patterns[@]}"; do
        for doc in docs/${pattern}; do
            if [[ -f "$doc" ]]; then
                local basename=$(basename "$doc")
                print_info "归档: $doc"

                if [[ "$MODE" != "preview" ]]; then
                    create_backup "$doc"
                    mv "$doc" "${archive_dir}/"
                    log "Archived: $doc -> ${archive_dir}/${basename}"
                fi

                ((archived++))
                ((ARCHIVED_FILES++))
            fi
        done
    done

    # 归档子目录中的文档
    for subdir in ai_models deployment troubleshooting tasks; do
        if [[ -d "docs/${subdir}" ]]; then
            local subdir_size=$(get_size_mb "docs/${subdir}")
            print_info "归档子目录: docs/${subdir}/ (${subdir_size} MB)"

            if [[ "$MODE" != "preview" ]]; then
                mkdir -p "${archive_dir}/${subdir}"
                create_backup "docs/${subdir}"

                # 移动所有文件
                if [[ -n "$(ls -A docs/${subdir}/*.md 2>/dev/null)" ]]; then
                    mv docs/${subdir}/*.md "${archive_dir}/${subdir}/" 2>/dev/null || true
                fi

                # 如果子目录为空，删除它
                if [[ -z "$(ls -A docs/${subdir} 2>/dev/null)" ]]; then
                    rmdir "docs/${subdir}" 2>/dev/null || true
                fi

                log "Archived directory: docs/${subdir} -> ${archive_dir}/${subdir}"
            fi

            ((archived += $(find docs/${subdir} -name "*.md" 2>/dev/null | wc -l)))
        fi
    done

    print_success "归档了 ${archived} 个开发文档"
    log "Phase 2 completed: Archived ${archived} development documents"
}

################################################################################
# Phase 3: 清理根目录废弃文档
################################################################################

cleanup_root_deprecated_docs() {
    print_header "Phase 3: 清理根目录废弃文档"

    local deprecated_docs=(
        "README_FINAL_SOLUTION.md"
        "README_PERFORMANCE_OPTIMIZATION.md"
    )

    local cleaned=0

    for doc in "${deprecated_docs[@]}"; do
        if [[ -f "$doc" ]]; then
            local doc_size=$(get_size_mb "$doc")
            print_info "删除: $doc (${doc_size} MB)"

            if [[ "$MODE" != "preview" ]]; then
                create_backup "$doc"
                rm "$doc"
                log "Deleted: $doc"
            fi

            ((cleaned++))
            ((DELETED_FILES++))
        fi
    done

    print_success "清理了 ${cleaned} 个根目录废弃文档"
    log "Phase 3 completed: Cleaned ${cleaned} deprecated root docs"
}

################################################################################
# Phase 4: 清理临时文件和测试结果
################################################################################

cleanup_temp_files() {
    print_header "Phase 4: 清理临时文件和测试结果"

    local cleaned=0

    # 清理测试结果JSON
    for result_file in test_results_*.json; do
        if [[ -f "$result_file" ]]; then
            print_info "删除测试结果: $result_file"

            if [[ "$MODE" != "preview" ]]; then
                rm "$result_file"
                log "Deleted: $result_file"
            fi

            ((cleaned++))
            ((DELETED_FILES++))
        fi
    done

    # 清理空的备份目录
    if [[ -d "backups" ]]; then
        local backup_count=$(find backups -type f 2>/dev/null | wc -l)
        if [[ "$backup_count" -eq 0 ]]; then
            print_info "删除空的backups目录"
            if [[ "$MODE" != "preview" ]]; then
                rm -rf backups
                log "Deleted: backups/ (empty)"
            fi
            ((DELETED_DIRS++))
        fi
    fi

    # 清理空的docker目录
    if [[ -d "docker" ]]; then
        local docker_count=$(find docker -type f 2>/dev/null | wc -l)
        if [[ "$docker_count" -eq 0 ]]; then
            print_info "删除空的docker目录"
            if [[ "$MODE" != "preview" ]]; then
                rm -rf docker
                log "Deleted: docker/ (empty)"
            fi
            ((DELETED_DIRS++))
        fi
    fi

    print_success "清理了 ${cleaned} 个临时文件"
    log "Phase 4 completed: Cleaned ${cleaned} temporary files"
}

################################################################################
# Phase 5: 可选大型目录清理
################################################################################

cleanup_large_dirs() {
    print_header "Phase 5: 大型目录清理（可选）"

    print_warning "以下大型目录可以清理，但需要手动确认："

    # node_modules
    if [[ -d "node_modules" ]]; then
        local size=$(get_size_mb "node_modules")
        print_info "node_modules/: ${size} MB (可通过 npm install 恢复)"
    fi

    # cyclonedds源码
    if [[ -d "cyclonedds" ]]; then
        local size=$(get_size_mb "cyclonedds")
        print_info "cyclonedds/: ${size} MB (DDS源码，非运行时必需)"
    fi

    print_warning "这些目录不会自动清理，请根据需要手动删除"
    log "Phase 5: Large directories reported (manual cleanup required)"
}

################################################################################
# Phase 6: Git清理建议
################################################################################

suggest_git_cleanup() {
    print_header "Phase 6: Git清理建议"

    print_info "检测到以下git标记删除的目录:"

    local git_deleted=(
        ".clinerules"
        ".github/instructions"
        ".roo"
        ".serena"
        ".trae"
        ".windsurf"
    )

    local found=0
    for dir in "${git_deleted[@]}"; do
        if [[ -d "$dir" ]]; then
            print_info "  - $dir (标记删除但未清理)"
            ((found++))
        fi
    done

    if [[ "$found" -gt 0 ]]; then
        print_warning "建议执行以下命令完成git清理:"
        echo "  git add -u          # 提交已删除的文件"
        echo "  git clean -fd       # 清理未跟踪的文件和目录"
    else
        print_success "没有发现需要清理的git标记目录"
    fi

    log "Phase 6: Git cleanup suggestions provided"
}

################################################################################
# 生成清理报告
################################################################################

generate_report() {
    print_header "清理报告"

    local report_path="${PROJECT_ROOT}/docs/cleanup_report_${TIMESTAMP}.md"

    cat > "$report_path" << EOF
# Claudia项目清理报告

**执行时间**: $(date '+%Y-%m-%d %H:%M:%S')
**执行模式**: ${MODE}
**备份目录**: ${BACKUP_DIR}

## 清理统计

- **删除文件数**: ${DELETED_FILES}
- **删除目录数**: ${DELETED_DIRS}
- **归档文档数**: ${ARCHIVED_FILES}
- **释放空间**: ${FREED_SPACE} MB

## 执行的操作

### Phase 1: Python缓存清理
- 清理了所有 \`__pycache__/\` 目录
- 删除了所有 \`.pyc\` 文件
- 清理了 pytest 缓存

### Phase 2: 文档归档
- 归档路径: \`docs/archive/development/\`
- 归档文档数: ${ARCHIVED_FILES}

### Phase 3: 废弃文档清理
- 删除了根目录的废弃README文件

### Phase 4: 临时文件清理
- 清理了测试结果JSON文件
- 删除了空的临时目录

## 备份信息

所有被修改或删除的文件都已备份到:
\`${BACKUP_DIR}\`

## 回滚方法

如需恢复文件:
\`\`\`bash
cp -r ${BACKUP_DIR}/<path> <original_path>
\`\`\`

## 验证建议

清理完成后，建议执行以下验证:
\`\`\`bash
# 运行测试
python3 test/run_tests.py

# 验证LLM brain
./start_production_brain.sh

# 检查文档结构
tree docs/ -L 2

# 验证TaskMaster
ls -lh .taskmaster/tasks/tasks.json
\`\`\`

---
*生成时间: $(date '+%Y-%m-%d %H:%M:%S')*
EOF

    print_success "清理报告已生成: ${report_path}"
    log "Report generated: ${report_path}"
}

################################################################################
# 主执行流程
################################################################################

main() {
    print_header "Claudia项目自动化清理脚本 v1.0"

    echo "项目根目录: ${PROJECT_ROOT}"
    echo "执行模式: ${MODE}"
    echo "时间戳: ${TIMESTAMP}"
    echo ""

    # 创建日志目录
    mkdir -p "${PROJECT_ROOT}/logs"

    # 创建备份目录（除了preview模式）
    if [[ "$MODE" != "preview" ]]; then
        mkdir -p "$BACKUP_DIR"
        log "Cleanup started - Mode: ${MODE}"
    fi

    # 根据模式执行清理
    case "$MODE" in
        safe)
            print_info "安全模式: 仅清理缓存和临时文件"
            cleanup_python_cache
            cleanup_temp_files
            suggest_git_cleanup
            ;;
        full)
            print_info "完整模式: 执行所有清理操作"
            cleanup_python_cache
            archive_development_docs
            cleanup_root_deprecated_docs
            cleanup_temp_files
            cleanup_large_dirs
            suggest_git_cleanup
            ;;
        preview)
            print_info "预览模式: 显示将要执行的操作（不实际执行）"
            cleanup_python_cache
            archive_development_docs
            cleanup_root_deprecated_docs
            cleanup_temp_files
            cleanup_large_dirs
            suggest_git_cleanup
            ;;
        *)
            print_error "未知模式: ${MODE}"
            echo "可用模式: safe, full, preview"
            exit 1
            ;;
    esac

    # 生成报告
    if [[ "$MODE" != "preview" ]]; then
        generate_report
    fi

    print_header "清理完成"

    echo "统计信息:"
    echo "  - 删除文件: ${DELETED_FILES}"
    echo "  - 删除目录: ${DELETED_DIRS}"
    echo "  - 归档文档: ${ARCHIVED_FILES}"
    echo "  - 释放空间: ${FREED_SPACE} MB"

    if [[ "$MODE" != "preview" ]]; then
        echo ""
        echo "备份位置: ${BACKUP_DIR}"
        echo "日志文件: ${REPORT_FILE}"
    fi

    print_success "项目清理成功完成！"
}

# 执行主函数
main "$@"
