"""
High-quality debugging logic for grasp height issues.
"""

import numpy as np
import cv2
import logging
import time
from pathlib import Path
from typing import Optional, Dict, Tuple, List
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class DepthEstimationStep:
    """Single step in depth estimation process."""
    step_name: str
    input_data: Dict
    output_value: float
    confidence: float
    details: Dict


@dataclass
class GraspDebugInfo:
    """Comprehensive debug information for a single grasp attempt."""
    timestamp: float
    grasp_location: Tuple[int, int]
    raw_depth_estimate: float
    final_depth_estimate: float
    table_depth_estimate: float
    object_height_estimate: float
    depth_sampling_method: str
    local_region_stats: Dict
    mask_coverage: float
    quality_at_location: float
    depth_estimation_steps: List[DepthEstimationStep]
    grid_analysis: Dict
    potential_issues: List[str]


class GraspHeightDebugger:
    """Debug grasp height estimation issues."""

    def __init__(self, output_dir: str = "grasp_debug"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        self.debug_history = []
        self.enabled = True

    def debug_grasp_height_detailed(self,
                                    grasp_location: Tuple[int, int],
                                    depth_image: np.ndarray,
                                    object_mask: np.ndarray,
                                    quality_map: np.ndarray,
                                    local_depth_estimate: float,
                                    global_depth_estimate: float,
                                    grid_cell_mins: List[float] = None,
                                    median_of_mins: float = None) -> Optional[GraspDebugInfo]:
        """Generate comprehensive debug info for grasp height."""

        if not self.enabled:
            return None

        try:
            u, v = grasp_location
            timestamp = time.time()

            # Extract local region around grasp
            radius = 20
            v0, v1 = max(
                0, v - radius), min(depth_image.shape[0], v + radius + 1)
            u0, u1 = max(
                0, u - radius), min(depth_image.shape[1], u + radius + 1)

            local_depth = depth_image[v0:v1, u0:u1]
            local_mask = object_mask[v0:v1, u0:u1]

            # Analyze depth distribution
            all_depths = local_depth[local_depth > 0]
            object_depths = local_depth[local_mask & (local_depth > 0)]
            table_depths = local_depth[~local_mask & (local_depth > 0)]

            # Estimate table depth from border regions
            border_radius = 5
            border_mask = np.zeros_like(depth_image, dtype=bool)
            border_mask[:border_radius, :] = True
            border_mask[-border_radius:, :] = True
            border_mask[:, :border_radius] = True
            border_mask[:, -border_radius:] = True
            border_depths = depth_image[border_mask & (depth_image > 0)]
            table_depth = float(np.median(border_depths)
                                ) if border_depths.size > 0 else 0.6

            # Calculate object height
            object_height = table_depth - \
                local_depth_estimate if table_depth > local_depth_estimate else 0

            # Determine sampling method used
            method = "median_of_minimums" if len(
                object_depths) >= 5 else "fallback"

            # Local region statistics
            local_stats = {
                'region_size': local_depth.shape,
                'all_depths_count': len(all_depths),
                'object_depths_count': len(object_depths),
                'table_depths_count': len(table_depths),
                'min_depth': float(all_depths.min()) if len(all_depths) > 0 else 0,
                'max_depth': float(all_depths.max()) if len(all_depths) > 0 else 0,
                'mean_depth': float(all_depths.mean()) if len(all_depths) > 0 else 0,
                'object_mean_depth': float(object_depths.mean()) if len(object_depths) > 0 else 0,
                'table_mean_depth': float(table_depths.mean()) if len(table_depths) > 0 else 0
            }

            # Analyze depth estimation steps
            depth_steps = self._analyze_depth_estimation_steps(
                local_depth, local_mask, grid_cell_mins, median_of_mins,
                global_depth_estimate, local_depth_estimate)

            # Analyze 3x3 grid method specifically
            grid_analysis = self._analyze_grid_method(
                local_depth, local_mask, grid_cell_mins)

            # Create debug info (before identify_potential_issues to avoid forward reference)
            debug_info = GraspDebugInfo(
                timestamp=timestamp,
                grasp_location=grasp_location,
                raw_depth_estimate=global_depth_estimate,
                final_depth_estimate=local_depth_estimate,
                table_depth_estimate=table_depth,
                object_height_estimate=object_height,
                depth_sampling_method=method,
                local_region_stats=local_stats,
                mask_coverage=float(object_mask.sum() / object_mask.size),
                quality_at_location=float(quality_map[v, u]),
                depth_estimation_steps=depth_steps,
                grid_analysis=grid_analysis,
                potential_issues=[]  # Will be populated next
            )

            # Identify potential issues
            potential_issues = self._identify_potential_issues(
                debug_info, local_depth_estimate, table_depth, object_height,
                local_stats, grid_analysis)
            debug_info.potential_issues = potential_issues

            # Save debug images
            self._save_debug_images(debug_info, local_depth,
                                    local_mask, depth_image, object_mask)

            # Log key findings
            self._log_debug_summary(debug_info)

            # Store in history
            self.debug_history.append(debug_info)

            return debug_info

        except Exception as e:
            logger.error(f"Debug grasp height failed: {e}")
            import traceback
            logger.debug(traceback.format_exc())
            return None

    def _save_debug_images(self, debug_info: GraspDebugInfo,
                           local_depth: np.ndarray, local_mask: np.ndarray,
                           full_depth: np.ndarray, full_mask: np.ndarray):
        """Save visual debug information with enhanced analysis."""
        timestamp_str = f"{int(debug_info.timestamp * 1000)}"

        # Save local region
        cv2.imwrite(str(self.output_dir / f"local_depth_{timestamp_str}.png"),
                    (local_depth * 1000).astype(np.uint16))
        cv2.imwrite(str(self.output_dir / f"local_mask_{timestamp_str}.png"),
                    (local_mask * 255).astype(np.uint8))

        # Create enhanced visualizations
        self._create_grid_overlay_image(
            debug_info, local_depth, local_mask, timestamp_str)
        self._create_depth_comparison_image(
            debug_info, local_depth, local_mask, timestamp_str)
        self._create_issue_highlight_image(
            debug_info, local_depth, local_mask, timestamp_str)

        # Save full images with grasp location marked
        full_depth_marked = (full_depth * 1000).astype(np.uint16).copy()
        cv2.circle(full_depth_marked, debug_info.grasp_location, 10, 65535, 2)
        cv2.imwrite(str(self.output_dir / f"full_depth_marked_{timestamp_str}.png"),
                    full_depth_marked)

        full_mask_marked = (full_mask * 255).astype(np.uint8).copy()
        cv2.circle(full_mask_marked, debug_info.grasp_location, 10, 255, 2)
        cv2.imwrite(str(self.output_dir / f"full_mask_marked_{timestamp_str}.png"),
                    full_mask_marked)

    def _create_grid_overlay_image(self, debug_info: GraspDebugInfo,
                                   local_depth: np.ndarray, local_mask: np.ndarray,
                                   timestamp_str: str):
        """Create image showing 3x3 grid overlay on depth data."""
        # Normalize depth for visualization
        depth_vis = cv2.normalize(local_depth, None, 0, 255, cv2.NORM_MINMAX)
        depth_vis = depth_vis.astype(np.uint8)
        depth_vis = cv2.cvtColor(depth_vis, cv2.COLOR_GRAY2BGR)

        # Draw 3x3 grid
        h, w = local_depth.shape
        grid_size = 3
        h_cell = max(1, h // grid_size)
        w_cell = max(1, w // grid_size)

        for i in range(grid_size + 1):
            y = i * h_cell
            cv2.line(depth_vis, (0, y), (w, y), (0, 255, 0), 1)
        for j in range(grid_size + 1):
            x = j * w_cell
            cv2.line(depth_vis, (x, 0), (x, h), (0, 255, 0), 1)

        # Mark grasp location
        center_u, center_v = debug_info.grasp_location
        local_center_u = center_u - (center_u - 8)  # Adjust for local region
        local_center_v = center_v - (center_v - 8)
        cv2.circle(depth_vis, (local_center_u, local_center_v),
                   5, (0, 0, 255), -1)

        cv2.imwrite(
            str(self.output_dir / f"grid_overlay_{timestamp_str}.png"), depth_vis)

    def _create_depth_comparison_image(self, debug_info: GraspDebugInfo,
                                       local_depth: np.ndarray, local_mask: np.ndarray,
                                       timestamp_str: str):
        """Create comparison showing different depth estimation methods."""
        # Create side-by-side comparison
        h, w = local_depth.shape

        # Method 1: Simple median
        obj_depths = local_depth[local_mask & (local_depth > 0)]
        simple_median = float(np.median(obj_depths)) if len(
            obj_depths) > 0 else 0

        # Method 2: Grid median of minimums (if available)
        grid_median = debug_info.grid_analysis.get("median_of_mins", 0)

        # Method 3: 5th percentile
        percentile_5th = float(np.percentile(
            obj_depths, 5)) if len(obj_depths) > 0 else 0

        # Create visualization
        comparison_img = np.zeros((h, w * 3, 3), dtype=np.uint8)

        # Normalize each method result
        methods = [
            ("Simple Median", simple_median, 0),
            ("Grid Median", grid_median, w),
            ("5th Percentile", percentile_5th, w * 2)
        ]

        for method_name, depth_value, x_offset in methods:
            # Create color-coded depth map
            depth_diff = np.abs(local_depth - depth_value)
            depth_diff_norm = cv2.normalize(
                depth_diff, None, 0, 255, cv2.NORM_MINMAX)
            depth_diff_colored = cv2.applyColorMap(
                depth_diff_norm.astype(np.uint8), cv2.COLORMAP_JET)

            comparison_img[:, x_offset:x_offset+w] = depth_diff_colored

            # Add text label
            cv2.putText(comparison_img, f"{method_name}: {depth_value:.3f}m",
                        (x_offset + 5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imwrite(
            str(self.output_dir / f"depth_comparison_{timestamp_str}.png"), comparison_img)

    def _create_issue_highlight_image(self, debug_info: GraspDebugInfo,
                                      local_depth: np.ndarray, local_mask: np.ndarray,
                                      timestamp_str: str):
        """Create image highlighting potential issues."""
        # Start with depth visualization
        depth_vis = cv2.normalize(local_depth, None, 0, 255, cv2.NORM_MINMAX)
        depth_vis = cv2.cvtColor(
            depth_vis.astype(np.uint8), cv2.COLOR_GRAY2BGR)

        # Highlight issues
        if "table surface" in str(debug_info.potential_issues):
            # Highlight areas close to table depth
            table_depth = debug_info.table_depth_estimate
            close_to_table = np.abs(local_depth - table_depth) < 0.02
            # Red for table-like depths
            depth_vis[close_to_table] = [0, 0, 255]

        if "few object depths" in str(debug_info.potential_issues).lower():
            # Highlight areas with sparse object data
            obj_depths = local_depth[local_mask & (local_depth > 0)]
            if len(obj_depths) < 10:
                depth_vis[local_mask] = [255, 255, 0]  # Yellow for sparse data

        # Add issue text overlay
        y_offset = 20
        for issue in debug_info.potential_issues:
            cv2.putText(depth_vis, issue[:40], (5, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            y_offset += 15

        cv2.imwrite(
            str(self.output_dir / f"issues_highlighted_{timestamp_str}.png"), depth_vis)

    def _log_debug_summary(self, debug_info: GraspDebugInfo):
        """Log key debug findings (only if issues detected)."""
        height_mm = debug_info.object_height_estimate * 1000

        # Only log if there are critical issues
        critical_issues = [
            issue for issue in debug_info.potential_issues
            if "height" in issue.lower() or "table" in issue.lower() or "mask" in issue.lower()
        ]

        if critical_issues or height_mm < 5:
            u, v = debug_info.grasp_location
            logger.warning(f"🔍 Grasp issue at ({u},{v}):")
            logger.warning(f"  Object height: {height_mm:.1f}mm")
            logger.warning(
                f"  Table: {debug_info.table_depth_estimate:.3f}m | Object: {debug_info.final_depth_estimate:.3f}m")
            logger.warning(f"  Method: {debug_info.depth_sampling_method}")

            if critical_issues:
                for issue in critical_issues:
                    logger.warning(f"  ⚠️  {issue}")

    def _analyze_depth_estimation_steps(self, local_depth: np.ndarray,
                                        local_mask: np.ndarray,
                                        grid_cell_mins: List[float],
                                        median_of_mins: float,
                                        global_depth: float,
                                        final_depth: float) -> List[DepthEstimationStep]:
        """Analyze each step of the depth estimation process."""
        steps = []

        # Step 1: Raw local region extraction
        obj_depths = local_depth[local_mask & (local_depth > 0)]
        steps.append(DepthEstimationStep(
            step_name="local_region_extraction",
            input_data={"region_size": local_depth.shape, "mask_coverage": float(
                local_mask.sum() / local_mask.size)},
            output_value=float(obj_depths.mean()) if len(
                obj_depths) > 0 else 0,
            confidence=min(1.0, len(obj_depths) / 50.0),
            details={"object_depth_count": len(
                obj_depths), "all_depth_count": len(local_depth[local_depth > 0])}
        ))

        # Step 2: Grid method analysis
        if grid_cell_mins is not None:
            steps.append(DepthEstimationStep(
                step_name="grid_median_of_minimums",
                input_data={"cell_mins": grid_cell_mins,
                            "cell_count": len(grid_cell_mins)},
                output_value=median_of_mins or 0,
                confidence=min(1.0, len(grid_cell_mins) / 9.0),
                details={"grid_cell_mins": grid_cell_mins,
                         "median_value": median_of_mins}
            ))

        # Step 3: Final selection
        steps.append(DepthEstimationStep(
            step_name="final_depth_selection",
            input_data={"global_depth": global_depth,
                        "local_estimate": final_depth},
            output_value=final_depth,
            confidence=1.0,
            details={"method_used": "grid_median" if grid_cell_mins else "fallback"}
        ))

        return steps

    def _analyze_grid_method(self, local_depth: np.ndarray,
                             local_mask: np.ndarray,
                             grid_cell_mins: List[float]) -> Dict:
        """Analyze the 3x3 grid median-of-minimums method specifically."""
        if grid_cell_mins is None:
            return {"method_used": False, "reason": "No grid data provided"}

        # Simulate the grid method to understand what happened
        h, w = local_depth.shape
        grid_size = 3
        h_cell = max(1, h // grid_size)
        w_cell = max(1, w // grid_size)

        grid_analysis = {
            "method_used": True,
            "grid_size": grid_size,
            "cell_dimensions": (h_cell, w_cell),
            "total_cells": grid_size * grid_size,
            "cells_with_data": len(grid_cell_mins),
            "cell_mins": grid_cell_mins,
            "median_of_mins": float(np.median(grid_cell_mins)) if grid_cell_mins else 0,
            "min_cell_value": float(min(grid_cell_mins)) if grid_cell_mins else 0,
            "max_cell_value": float(max(grid_cell_mins)) if grid_cell_mins else 0,
            "cell_variance": float(np.var(grid_cell_mins)) if grid_cell_mins else 0
        }

        # Analyze each cell
        cell_details = []
        for i in range(grid_size):
            for j in range(grid_size):
                cell = local_depth[i *
                                   h_cell:(i+1)*h_cell, j*w_cell:(j+1)*w_cell]
                cell_m = local_mask[i *
                                    h_cell:(i+1)*h_cell, j*w_cell:(j+1)*w_cell]
                cell_obj = cell[cell_m & (cell > 0)]

                cell_info = {
                    "cell_id": f"{i},{j}",
                    "cell_size": cell.shape,
                    "object_pixels": len(cell_obj),
                    "min_depth": float(np.min(cell_obj)) if len(cell_obj) > 0 else None,
                    "mean_depth": float(np.mean(cell_obj)) if len(cell_obj) > 0 else None,
                    "depth_range": float(np.max(cell_obj) - np.min(cell_obj)) if len(cell_obj) > 0 else 0
                }
                cell_details.append(cell_info)

        grid_analysis["cell_details"] = cell_details
        return grid_analysis

    def _identify_potential_issues(self, debug_info: GraspDebugInfo,
                                   local_depth: float, table_depth: float,
                                   object_height: float, local_stats: Dict,
                                   grid_analysis: Dict) -> List[str]:
        """Identify potential issues with the depth estimation."""
        issues = []

        # Check object height
        if object_height < 0.005:  # 5mm
            issues.append(
                "Object height < 5mm - likely grasping table surface")

        # Check depth difference from table
        depth_diff = abs(local_depth - table_depth)
        if depth_diff < 0.015:  # 15mm
            issues.append(
                f"Grasp depth very close to table depth (diff: {depth_diff*1000:.1f}mm)")

        # Check grid method effectiveness
        if grid_analysis.get("method_used", False):
            if grid_analysis["cells_with_data"] < 5:
                issues.append("Grid method used but <5 cells had data")
            if grid_analysis["cell_variance"] > 0.05:  # 50mm variance
                issues.append(
                    "High variance in grid cell minimums - inconsistent depth")

        # Check mask coverage
        if debug_info.mask_coverage < 0.1:
            issues.append("Low object mask coverage - may be missing object")

        # Check depth sampling quality
        if local_stats["object_depths_count"] < 10:
            issues.append(
                "Few object depths in local region - unreliable estimate")

        return issues

    def generate_actionable_insights(self, debug_info: GraspDebugInfo) -> Dict[str, str]:
        """Generate specific, actionable recommendations based on debug analysis."""
        insights = {}

        # Analyze object height issue
        if debug_info.object_height_estimate < 0.01:
            insights["height_issue"] = f"CRITICAL: Object height only {debug_info.object_height_estimate*1000:.1f}mm. Robot will grasp table surface."
            insights["height_fix"] = "Increase depth offset in grasp execution or improve object mask segmentation"

        # Analyze grid method effectiveness
        if debug_info.grid_analysis.get("method_used", False):
            cell_count = debug_info.grid_analysis["cells_with_data"]
            if cell_count < 5:
                insights["grid_issue"] = f"Grid method used but only {cell_count}/9 cells had data"
                insights["grid_fix"] = "Increase local region radius or improve object mask coverage"

            variance = debug_info.grid_analysis["cell_variance"]
            if variance > 0.05:
                insights["variance_issue"] = f"High depth variance ({variance*1000:.1f}mm) across grid cells"
                insights["variance_fix"] = "Object may have complex geometry - consider using simple median instead"

        # Analyze depth sampling quality
        obj_depths_count = debug_info.local_region_stats["object_depths_count"]
        if obj_depths_count < 10:
            insights["sampling_issue"] = f"Only {obj_depths_count} object depths in local region"
            insights["sampling_fix"] = "Increase local region radius or check object mask accuracy"

        # Analyze depth difference from table
        depth_diff = abs(debug_info.final_depth_estimate -
                         debug_info.table_depth_estimate)
        if depth_diff < 0.015:
            insights["table_proximity"] = f"Grasp depth {depth_diff*1000:.1f}mm from table - may be too low"
            insights["table_fix"] = "Add safety margin to grasp depth or improve object height estimation"

        return insights

    def create_debug_report(self, debug_info: GraspDebugInfo) -> str:
        """Create a comprehensive debug report."""
        insights = self.generate_actionable_insights(debug_info)

        report = f"""
=== GRASP HEIGHT DEBUG REPORT ===
Timestamp: {debug_info.timestamp}
Grasp Location: {debug_info.grasp_location}

DEPTH ESTIMATION ANALYSIS:
- Final depth estimate: {debug_info.final_depth_estimate:.3f}m
- Table depth estimate: {debug_info.table_depth_estimate:.3f}m
- Object height estimate: {debug_info.object_height_estimate:.3f}m
- Method used: {debug_info.depth_sampling_method}

GRID METHOD ANALYSIS:
- Cells with data: {debug_info.grid_analysis.get('cells_with_data', 'N/A')}/9
- Median of minimums: {debug_info.grid_analysis.get('median_of_mins', 0):.3f}m
- Cell variance: {debug_info.grid_analysis.get('cell_variance', 0)*1000:.1f}mm

POTENTIAL ISSUES:
{chr(10).join(f"- {issue}" for issue in debug_info.potential_issues)}

ACTIONABLE INSIGHTS:
{chr(10).join(f"- {key}: {value}" for key, value in insights.items())}

DEBUG IMAGES SAVED:
- grid_overlay_{int(debug_info.timestamp*1000)}.png
- depth_comparison_{int(debug_info.timestamp*1000)}.png
- issues_highlighted_{int(debug_info.timestamp*1000)}.png
"""
        return report


# Global debugger instance
_grasp_debugger = GraspHeightDebugger()


def debug_grasp_height(grasp_location: Tuple[int, int],
                       depth_image: np.ndarray,
                       object_mask: np.ndarray,
                       quality_map: np.ndarray,
                       local_depth_estimate: float,
                       global_depth_estimate: float,
                       grid_cell_mins: List[float] = None,
                       median_of_mins: float = None) -> Optional[GraspDebugInfo]:
    """Convenience function for debugging grasp height."""
    if not _grasp_debugger.enabled:
        return None

    return _grasp_debugger.debug_grasp_height_detailed(
        grasp_location, depth_image, object_mask, quality_map,
        local_depth_estimate, global_depth_estimate, grid_cell_mins, median_of_mins
    )


def enable_grasp_debug(enabled: bool = True):
    """Enable or disable grasp height debugging."""
    _grasp_debugger.enabled = enabled
