#!/usr/bin/env python3
"""
Script to extract joint positions from JSON analysis and apply Unity→Robot coordinate transformation.
"""

import json
import sys
import os


def apply_coordinate_transform(x_unity, y_unity, z_unity):
    """
    Apply coordinate transformation from Unity space to Robot space.

    Unity coordinates (Y-up, Z-forward): (x_unity, y_unity, z_unity)
    Robot coordinates (Z-up, X-forward): (z_unity, x_unity, -y_unity)

    Args:
        x_unity: X coordinate in Unity space
        y_unity: Y coordinate in Unity space  
        z_unity: Z coordinate in Unity space

    Returns:
        tuple: (x_robot, y_robot, z_robot)
    """
    x_robot = z_unity
    y_robot = x_unity
    z_robot = -y_unity

    return x_robot, y_robot, z_robot


def load_json_data(json_file_path):
    """Load and parse the JSON analysis file."""
    with open(json_file_path, 'r') as f:
        data = json.load(f)

    # Get the skeleton data (should be under skeleton ID "67")
    skeleton_id = "67"
    if skeleton_id not in data:
        raise ValueError(f"Skeleton ID {skeleton_id} not found in JSON data")

    return data[skeleton_id]


def extract_and_transform_joints(json_data):
    """Extract joint positions and apply coordinate transformation."""
    transformed_joints = {}

    for joint_name, joint_data in json_data.items():
        # Extract Unity coordinates
        x_unity = joint_data['x']
        y_unity = joint_data['y']
        z_unity = joint_data['z']

        # Apply transformation to robot coordinates
        x_robot, y_robot, z_robot = apply_coordinate_transform(
            x_unity, y_unity, z_unity)

        transformed_joints[joint_name] = {
            'x': x_robot,
            'y': y_robot,
            'z': z_robot,
            'original_unity': (x_unity, y_unity, z_unity)
        }

        print(f"{joint_name}: Unity({x_unity:.3f}, {y_unity:.3f}, {z_unity:.3f}) → Robot({x_robot:.3f}, {y_robot:.3f}, {z_robot:.3f})")

    return transformed_joints


def generate_config_code(transformed_joints):
    """Generate the config file code with transformed coordinates."""

    # Define joint categories (same as in original config)
    joint_categories = {
        'CORE_BODY_JOINTS': ['PELVIS', 'SPINE_1', 'SPINE_2', 'SPINE_3', 'NECK'],
        'HEAD_JOINTS': ['NOSE', 'LEFT_EYE', 'RIGHT_EYE', 'LEFT_EAR', 'RIGHT_EAR'],
        'SHOULDER_JOINTS': ['LEFT_SHOULDER', 'RIGHT_SHOULDER', 'LEFT_CLAVICLE', 'RIGHT_CLAVICLE'],
        'ARM_JOINTS': ['LEFT_ELBOW', 'RIGHT_ELBOW', 'LEFT_WRIST', 'RIGHT_WRIST'],
        'HAND_JOINTS': ['LEFT_HAND_INDEX_1', 'LEFT_HAND_MIDDLE_4', 'LEFT_HAND_PINKY_1', 'LEFT_HAND_THUMB_4',
                        'RIGHT_HAND_INDEX_1', 'RIGHT_HAND_MIDDLE_4', 'RIGHT_HAND_PINKY_1', 'RIGHT_HAND_THUMB_4'],
        'LEG_JOINTS': ['LEFT_HIP', 'RIGHT_HIP', 'LEFT_KNEE', 'RIGHT_KNEE', 'LEFT_ANKLE', 'RIGHT_ANKLE'],
        'FOOT_JOINTS': ['LEFT_HEEL', 'RIGHT_HEEL', 'LEFT_BIG_TOE', 'RIGHT_BIG_TOE', 'LEFT_SMALL_TOE', 'RIGHT_SMALL_TOE']
    }

    config_lines = []

    # Generate each category
    for category_name, joint_names in joint_categories.items():
        config_lines.append(
            f"# {category_name.split('_')[0].title()} {category_name.split('_')[1].lower()} joints ({category_name.split('_')[2].lower() if len(category_name.split('_')) > 2 else ''})")
        if 'CORE' in category_name:
            config_lines.append("# Most stable joints for path planning")
        elif 'HEAD' in category_name:
            config_lines.append("# Good for head tracking")
        elif 'SHOULDER' in category_name:
            config_lines.append("# Good for arm tracking")
        elif 'ARM' in category_name:
            config_lines.append("# Moderate stability")
        elif 'HAND' in category_name:
            config_lines.append("# Least stable, use with caution")
        elif 'LEG' in category_name:
            config_lines.append("# Good for lower body tracking")
        elif 'FOOT' in category_name:
            config_lines.append("# Moderate stability")

        config_lines.append(f"{category_name} = {{")

        for joint_name in joint_names:
            if joint_name in transformed_joints:
                pos = transformed_joints[joint_name]
                comment = ""
                if joint_name == 'LEFT_ELBOW':
                    comment = "  # Most stable arm joint"
                elif joint_name == 'LEFT_WRIST':
                    comment = "  # Most stable hand joint"

                config_lines.append(
                    f"    '{joint_name}': ConfigJointData('{joint_name}', {pos['x']:.3f}, {pos['y']:.3f}, {pos['z']:.3f}),{comment}")
            else:
                print(
                    f"Warning: Joint {joint_name} not found in transformed data")

        config_lines.append("}")
        config_lines.append("")

    return "\n".join(config_lines)


def main():
    """Main function to process JSON and generate config."""
    json_file = "skeleton_data/skeleton_analysis_20251018_024220.json"

    if not os.path.exists(json_file):
        print(f"Error: JSON file {json_file} not found")
        return

    print("🔄 Loading JSON data...")
    json_data = load_json_data(json_file)

    print(f"\n📊 Found {len(json_data)} joints in JSON data")

    print("\n🔄 Applying coordinate transformation...")
    print("Unity (Y-up, Z-forward) → Robot (Z-up, X-forward)")
    print("Transform: (x,y,z) → (z,x,-y)")
    print("-" * 80)

    transformed_joints = extract_and_transform_joints(json_data)

    print(f"\n✅ Transformed {len(transformed_joints)} joints")

    print("\n📝 Generating config code...")
    config_code = generate_config_code(transformed_joints)

    # Save to file
    output_file = "transformed_joint_config.txt"
    with open(output_file, 'w') as f:
        f.write(config_code)

    print(f"\n✅ Config code saved to {output_file}")
    print("\nNext step: Replace the joint definitions in src/config/path_planning.py with this transformed data")


if __name__ == "__main__":
    main()
