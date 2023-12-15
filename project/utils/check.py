import os

def rename_images_kitti(folder_path):
    # Check if the provided path is a directory
    if not os.path.isdir(folder_path):
        print("Provided path is not a directory.")
        return

    # List all files in the directory
    files = os.listdir(folder_path)
    img_files = [f for f in files if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp'))]

    # Sort image files for consistency
    img_files.sort()

    # Rename images to KITTI format
    for i, img_file in enumerate(img_files):
        extension = os.path.splitext(img_file)[1]
        new_name = f"{i:06d}{extension}"  # Format filenames to KITTI convention (e.g., 000000.png)

        # Construct old and new paths
        old_path = os.path.join(folder_path, img_file)
        new_path = os.path.join(folder_path, new_name)

        # Rename the file
        try:
            os.rename(old_path, new_path)
            print(f"Renamed '{img_file}' to '{new_name}'")
        except Exception as e:
            print(f"Error renaming '{img_file}': {e}")

# Replace 'folder_path' with the path to your image folder
folder_path = './bagtoimg/project_data/image_1/'
rename_images_kitti(folder_path)
