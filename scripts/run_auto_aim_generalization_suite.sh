#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

RUN_TAG="$(date +%Y%m%d_%H%M%S)"
BASE_OUT_DIR_DEFAULT="$ROOT_DIR/logs/auto-aim-generalization/$RUN_TAG"
BASE_OUT_DIR="${AUTO_AIM_SUITE_OUT_DIR:-$BASE_OUT_DIR_DEFAULT}"
mkdir -p "$BASE_OUT_DIR"

AUTOS_DEFAULT=(
  "Along Alliance Moving Shot"
  "brag"
)

if [[ -n "${AUTO_AIM_SUITE_AUTOS:-}" ]]; then
  IFS='|' read -r -a AUTOS <<< "${AUTO_AIM_SUITE_AUTOS}"
else
  AUTOS=("${AUTOS_DEFAULT[@]}")
fi

echo "Running auto-aim generalization suite"
echo "Output directory: $BASE_OUT_DIR"
echo "Autos (${#AUTOS[@]}):"
for auto in "${AUTOS[@]}"; do
  echo "  - $auto"
done
echo ""

RESULTS_FILE="$BASE_OUT_DIR/results_table.md"
{
  echo "| auto | exit | summary | p90_descent_miss_in | max_descent_miss_in | descent_within_ideal_fraction | mean_signed_descent_miss_in | mean_along_velocity_miss_m | mean_abs_cross_velocity_miss_m | p90_moving_aim_error_deg |"
  echo "|---|---:|---|---:|---:|---:|---:|---:|---:|---:|"
} > "$RESULTS_FILE"

overall_exit=0
for auto in "${AUTOS[@]}"; do
  slug="$(echo "$auto" | tr '[:upper:]' '[:lower:]' | tr -cs 'a-z0-9' '_' | sed 's/^_//; s/_$//')"
  out_dir="$BASE_OUT_DIR/$slug"
  mkdir -p "$out_dir"

  cmd_exit=0
  AUTO_AIM_AUTO_NAME="$auto" \
  AUTO_AIM_OUT_DIR="$out_dir" \
  ./scripts/run_along_alliance_auto_aim_diag.sh || cmd_exit=$?

  summary_file="$(ls -1t "$out_dir"/auto_aim_summary_*.txt 2>/dev/null | head -n 1 || true)"
  if [[ -z "$summary_file" ]]; then
    summary_file="$(ls -1t "$out_dir"/along_alliance_moving_shot_summary_*.txt 2>/dev/null | head -n 1 || true)"
  fi

  if [[ -z "$summary_file" ]]; then
    summary_file="(missing)"
    p90_in="NaN"
    max_in="NaN"
    within_ideal="NaN"
    mean_signed_in="NaN"
    mean_along_m="NaN"
    mean_cross_m="NaN"
    p90_aim_deg="NaN"
    overall_exit=1
  else
    p90_in="$(awk -F= '/^p90_descent_miss_in=/{print $2}' "$summary_file")"
    max_in="$(awk -F= '/^max_descent_miss_in=/{print $2}' "$summary_file")"
    within_ideal="$(awk -F= '/^descent_within_ideal_fraction=/{print $2}' "$summary_file")"
    mean_signed_in="$(awk -F= '/^mean_signed_descent_miss_in=/{print $2}' "$summary_file")"
    mean_along_m="$(awk -F= '/^mean_along_velocity_miss_m=/{print $2}' "$summary_file")"
    mean_cross_m="$(awk -F= '/^mean_abs_cross_velocity_miss_m=/{print $2}' "$summary_file")"
    p90_aim_deg="$(awk -F= '/^p90_moving_aim_error_deg=/{print $2}' "$summary_file")"
    if [[ "$cmd_exit" -ne 0 ]]; then
      overall_exit=1
    fi
  fi

  {
    echo "| $auto | $cmd_exit | $summary_file | $p90_in | $max_in | $within_ideal | $mean_signed_in | $mean_along_m | $mean_cross_m | $p90_aim_deg |"
  } >> "$RESULTS_FILE"
done

echo ""
echo "Generalization summary:"
cat "$RESULTS_FILE"
echo ""
echo "Saved: $RESULTS_FILE"

exit "$overall_exit"
