"""
Calibration validation and quality assessment tools.
"""
import numpy as np
import matplotlib.pyplot as plt
import logging
from typing import List, Tuple, Dict, Any, Optional
from pathlib import Path

logger = logging.getLogger(__name__)


class CalibrationValidator:
    """Validates hand-eye calibration quality and provides visualization."""

    def __init__(self, config):
        """Initialize calibration validator."""
        self.config = config

    def validate_calibration_quality(
        self,
        R_gripper2base: List[np.ndarray],
        t_gripper2base: List[np.ndarray],
        R_target2cam: List[np.ndarray],
        t_target2cam: List[np.ndarray],
        H_cam2tcp: np.ndarray,
        reprojection_errors: List[float]
    ) -> Dict[str, Any]:
        """
        Validate the quality of hand-eye calibration.

        Args:
            R_gripper2base: List of rotation matrices (gripper to base)
            t_gripper2base: List of translation vectors (gripper to base)
            R_target2cam: List of rotation matrices (target to camera)
            t_target2cam: List of translation vectors (target to camera)
            H_cam2tcp: Hand-eye transformation matrix (camera to TCP)
            reprojection_errors: List of reprojection errors per pose

        Returns:
            Dictionary containing validation results and metrics
        """
        logger.info("Validating calibration quality...")

        # Extract rotation and translation from hand-eye matrix
        R_cam2tcp = H_cam2tcp[:3, :3]
        t_cam2tcp = H_cam2tcp[:3, 3]

        # 1. Reprojection error analysis
        reproj_metrics = self._analyze_reprojection_errors(reprojection_errors)

        # 2. Consistency error analysis
        consistency_metrics = self._analyze_consistency_errors(
            R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, H_cam2tcp
        )

        # 3. Pose diversity analysis
        diversity_metrics = self._analyze_pose_diversity(
            R_gripper2base, t_gripper2base)

        # 4. Overall quality assessment
        overall_quality = self._assess_overall_quality(
            reproj_metrics, consistency_metrics, diversity_metrics
        )

        validation_results = {
            'reprojection_metrics': reproj_metrics,
            'consistency_metrics': consistency_metrics,
            'diversity_metrics': diversity_metrics,
            'overall_quality': overall_quality,
            'is_valid': overall_quality['is_valid'],
            'quality_score': overall_quality['quality_score']
        }

        logger.info(
            f"Calibration validation complete. Quality score: {overall_quality['quality_score']:.3f}")
        return validation_results

    def _analyze_reprojection_errors(self, reprojection_errors: List[float]) -> Dict[str, Any]:
        """Analyze reprojection errors."""
        if not reprojection_errors:
            return {'mean_error': float('inf'), 'max_error': float('inf'), 'std_error': float('inf')}

        errors = np.array(reprojection_errors)

        metrics = {
            'mean_error': float(np.mean(errors)),
            'max_error': float(np.max(errors)),
            'min_error': float(np.min(errors)),
            'std_error': float(np.std(errors)),
            'median_error': float(np.median(errors)),
            'p95_error': float(np.percentile(errors, 95)),
            'is_acceptable': np.mean(errors) < self.config.max_reprojection_error
        }

        return metrics

    def _analyze_consistency_errors(
        self,
        R_gripper2base: List[np.ndarray],
        t_gripper2base: List[np.ndarray],
        R_target2cam: List[np.ndarray],
        t_target2cam: List[np.ndarray],
        H_cam2tcp: np.ndarray
    ) -> Dict[str, Any]:
        """Analyze calibration consistency errors."""
        R_cam2tcp = H_cam2tcp[:3, :3]
        t_cam2tcp = H_cam2tcp[:3, 3]

        consistency_errors = []

        for i in range(len(R_gripper2base)):
            # Build transformation matrices
            A = np.eye(4)
            A[:3, :3] = R_gripper2base[i]
            A[:3, 3] = t_gripper2base[i]

            B = np.eye(4)
            B[:3, :3] = R_target2cam[i]
            B[:3, 3] = t_target2cam[i]

            X = H_cam2tcp

            # Compute consistency error: ||AX - XB||_F
            error_matrix = A @ X - X @ B
            frobenius_error = np.linalg.norm(error_matrix, 'fro')
            consistency_errors.append(frobenius_error)

        errors = np.array(consistency_errors)

        metrics = {
            'mean_error': float(np.mean(errors)),
            'max_error': float(np.max(errors)),
            'min_error': float(np.min(errors)),
            'std_error': float(np.std(errors)),
            'median_error': float(np.median(errors)),
            'is_acceptable': np.mean(errors) < self.config.max_consistency_error
        }

        return metrics

    def _analyze_pose_diversity(self, R_gripper2base: List[np.ndarray], t_gripper2base: List[np.ndarray]) -> Dict[str, Any]:
        """Analyze pose diversity for calibration quality."""
        if len(R_gripper2base) < 2:
            return {'diversity_score': 0.0, 'is_diverse': False}

        # Analyze position diversity
        positions = np.array(t_gripper2base)
        position_distances = []

        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                distance = np.linalg.norm(positions[i] - positions[j])
                position_distances.append(distance)

        # Analyze orientation diversity
        orientations = []
        for R in R_gripper2base:
            # Convert rotation matrix to axis-angle representation
            from scipy.spatial.transform import Rotation as R_scipy
            r = R_scipy.from_matrix(R)
            axis_angle = r.as_rotvec()
            orientations.append(axis_angle)

        orientations = np.array(orientations)
        orientation_distances = []

        for i in range(len(orientations)):
            for j in range(i + 1, len(orientations)):
                distance = np.linalg.norm(orientations[i] - orientations[j])
                orientation_distances.append(distance)

        # Calculate diversity scores
        position_diversity = np.mean(
            position_distances) if position_distances else 0.0
        orientation_diversity = np.mean(
            orientation_distances) if orientation_distances else 0.0

        # Combined diversity score (normalized)
        diversity_score = min(
            1.0, (position_diversity + orientation_diversity) / 2.0)

        metrics = {
            'position_diversity': float(position_diversity),
            'orientation_diversity': float(orientation_diversity),
            'diversity_score': float(diversity_score),
            'is_diverse': diversity_score > 0.3  # Threshold for good diversity
        }

        return metrics

    def _assess_overall_quality(
        self,
        reproj_metrics: Dict[str, Any],
        consistency_metrics: Dict[str, Any],
        diversity_metrics: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Assess overall calibration quality."""
        # Weighted quality score
        reproj_weight = 0.4
        consistency_weight = 0.4
        diversity_weight = 0.2

        # Normalize scores (lower is better for errors)
        reproj_score = max(
            0, 1 - reproj_metrics['mean_error'] / self.config.max_reprojection_error)
        consistency_score = max(
            0, 1 - consistency_metrics['mean_error'] / self.config.max_consistency_error)
        diversity_score = diversity_metrics['diversity_score']

        overall_score = (
            reproj_weight * reproj_score +
            consistency_weight * consistency_score +
            diversity_weight * diversity_score
        )

        # Determine if calibration is valid
        is_valid = (
            reproj_metrics['is_acceptable'] and
            consistency_metrics['is_acceptable'] and
            diversity_metrics['is_diverse'] and
            overall_score > 0.7
        )

        return {
            'quality_score': float(overall_score),
            'is_valid': bool(is_valid),
            'reproj_score': float(reproj_score),
            'consistency_score': float(consistency_score),
            'diversity_score': float(diversity_score),
            'recommendations': self._generate_recommendations(
                reproj_metrics, consistency_metrics, diversity_metrics, overall_score
            )
        }

    def _generate_recommendations(
        self,
        reproj_metrics: Dict[str, Any],
        consistency_metrics: Dict[str, Any],
        diversity_metrics: Dict[str, Any],
        overall_score: float
    ) -> List[str]:
        """Generate recommendations for improving calibration."""
        recommendations = []

        if not reproj_metrics['is_acceptable']:
            recommendations.append(
                "Reprojection errors are too high. Check camera calibration and lighting.")

        if not consistency_metrics['is_acceptable']:
            recommendations.append(
                "Consistency errors are too high. Ensure poses are stable and well-distributed.")

        if not diversity_metrics['is_diverse']:
            recommendations.append(
                "Pose diversity is insufficient. Generate more varied robot poses.")

        if overall_score < 0.7:
            recommendations.append(
                "Overall quality is low. Consider re-running calibration with more poses.")

        if not recommendations:
            recommendations.append("Calibration quality is good!")

        return recommendations

    def create_validation_plots(
        self,
        reprojection_errors: List[float],
        consistency_errors: List[float],
        save_path: Optional[str] = None
    ) -> None:
        """Create validation plots."""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle('Hand-Eye Calibration Validation', fontsize=16)

        # Plot 1: Reprojection errors
        axes[0, 0].plot(reprojection_errors, 'bo-', markersize=4)
        axes[0, 0].axhline(y=self.config.max_reprojection_error, color='r', linestyle='--',
                           label=f'Threshold ({self.config.max_reprojection_error:.1f}px)')
        axes[0, 0].set_title('Reprojection Errors')
        axes[0, 0].set_xlabel('Pose Index')
        axes[0, 0].set_ylabel('Error (pixels)')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)

        # Plot 2: Consistency errors
        axes[0, 1].plot(consistency_errors, 'ro-', markersize=4)
        axes[0, 1].axhline(y=self.config.max_consistency_error, color='r', linestyle='--',
                           label=f'Threshold ({self.config.max_consistency_error:.3f})')
        axes[0, 1].set_title('Consistency Errors')
        axes[0, 1].set_xlabel('Pose Index')
        axes[0, 1].set_ylabel('Error (Frobenius norm)')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)

        # Plot 3: Error histograms
        axes[1, 0].hist(reprojection_errors, bins=10, alpha=0.7,
                        label='Reprojection', color='blue')
        axes[1, 0].set_title('Error Distribution')
        axes[1, 0].set_xlabel('Error Value')
        axes[1, 0].set_ylabel('Frequency')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)

        # Plot 4: Error statistics
        reproj_stats = [np.mean(reprojection_errors), np.std(
            reprojection_errors), np.max(reprojection_errors)]
        consistency_stats = [np.mean(consistency_errors), np.std(
            consistency_errors), np.max(consistency_errors)]

        x = np.arange(3)
        width = 0.35

        axes[1, 1].bar(x - width/2, reproj_stats, width,
                       label='Reprojection', alpha=0.7)
        axes[1, 1].bar(x + width/2, consistency_stats,
                       width, label='Consistency', alpha=0.7)
        axes[1, 1].set_title('Error Statistics')
        axes[1, 1].set_xlabel('Statistic')
        axes[1, 1].set_ylabel('Value')
        axes[1, 1].set_xticks(x)
        axes[1, 1].set_xticklabels(['Mean', 'Std', 'Max'])
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            logger.info(f"Validation plots saved to {save_path}")

        plt.show()

    def generate_calibration_report(
        self,
        validation_results: Dict[str, Any],
        H_cam2tcp: np.ndarray,
        save_path: Optional[str] = None
    ) -> str:
        """Generate a detailed calibration report."""
        report = []
        report.append("=" * 60)
        report.append("HAND-EYE CALIBRATION REPORT")
        report.append("=" * 60)
        report.append("")

        # Overall quality
        overall = validation_results['overall_quality']
        report.append(f"Overall Quality Score: {overall['quality_score']:.3f}")
        report.append(
            f"Calibration Valid: {'YES' if overall['is_valid'] else 'NO'}")
        report.append("")

        # Reprojection metrics
        reproj = validation_results['reprojection_metrics']
        report.append("REPROJECTION ERRORS:")
        report.append(f"  Mean Error: {reproj['mean_error']:.3f} pixels")
        report.append(f"  Max Error: {reproj['max_error']:.3f} pixels")
        report.append(f"  Std Error: {reproj['std_error']:.3f} pixels")
        report.append(
            f"  Acceptable: {'YES' if reproj['is_acceptable'] else 'NO'}")
        report.append("")

        # Consistency metrics
        consistency = validation_results['consistency_metrics']
        report.append("CONSISTENCY ERRORS:")
        report.append(f"  Mean Error: {consistency['mean_error']:.6f}")
        report.append(f"  Max Error: {consistency['max_error']:.6f}")
        report.append(f"  Std Error: {consistency['std_error']:.6f}")
        report.append(
            f"  Acceptable: {'YES' if consistency['is_acceptable'] else 'NO'}")
        report.append("")

        # Diversity metrics
        diversity = validation_results['diversity_metrics']
        report.append("POSE DIVERSITY:")
        report.append(
            f"  Position Diversity: {diversity['position_diversity']:.3f}")
        report.append(
            f"  Orientation Diversity: {diversity['orientation_diversity']:.3f}")
        report.append(
            f"  Overall Diversity Score: {diversity['diversity_score']:.3f}")
        report.append(
            f"  Sufficient Diversity: {'YES' if diversity['is_diverse'] else 'NO'}")
        report.append("")

        # Hand-eye transformation matrix
        report.append("HAND-EYE TRANSFORMATION MATRIX (Camera to TCP):")
        report.append("Rotation Matrix:")
        for i in range(3):
            report.append(
                f"  [{H_cam2tcp[i,0]:8.4f} {H_cam2tcp[i,1]:8.4f} {H_cam2tcp[i,2]:8.4f}]")
        report.append("Translation Vector:")
        report.append(
            f"  [{H_cam2tcp[0,3]:8.4f} {H_cam2tcp[1,3]:8.4f} {H_cam2tcp[2,3]:8.4f}]")
        report.append("")

        # Recommendations
        report.append("RECOMMENDATIONS:")
        for i, rec in enumerate(overall['recommendations'], 1):
            report.append(f"  {i}. {rec}")
        report.append("")

        report.append("=" * 60)

        report_text = "\n".join(report)

        if save_path:
            with open(save_path, 'w') as f:
                f.write(report_text)
            logger.info(f"Calibration report saved to {save_path}")

        return report_text
