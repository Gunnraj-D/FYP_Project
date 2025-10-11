"""
Script to update all imports from 'config.config' to 'config'
"""
import re
from pathlib import Path

def update_imports_in_file(file_path: Path) -> bool:
    """Update imports in a single file. Returns True if changes were made."""
    try:
        content = file_path.read_text(encoding='utf-8')
        original_content = content
        
        # Pattern to match: from config.config import ...
        # Replace with: from config import ...
        content = re.sub(
            r'from config\.config import',
            r'from config import',
            content
        )
        
        if content != original_content:
            file_path.write_text(content, encoding='utf-8')
            return True
        return False
    except Exception as e:
        print(f"Error processing {file_path}: {e}")
        return False

def main():
    src_dir = Path('src')
    
    # Find all Python files
    python_files = list(src_dir.rglob('*.py'))
    
    updated_count = 0
    skipped_files = ['config_legacy_backup.py', '__pycache__']
    
    for file_path in python_files:
        # Skip backup and cache files
        if any(skip in str(file_path) for skip in skipped_files):
            continue
            
        if update_imports_in_file(file_path):
            print(f"✅ Updated: {file_path.relative_to(src_dir)}")
            updated_count += 1
    
    print(f"\n🎉 Updated {updated_count} files")
    print("✅ All imports now use 'from config import ...'")

if __name__ == '__main__':
    main()

