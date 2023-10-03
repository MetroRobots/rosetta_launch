from ros2launch.api import get_share_file_path_from_package, launch_a_launch_file


def main():
    path = get_share_file_path_from_package(package_name='donatello', file_name='05-arg.launch.py')
    launch_arguments = ['pizza_type:=extra_cheese']

    launch_a_launch_file(
        launch_file_path=path,
        # Note: launch_file_arguments is required!
        launch_file_arguments=launch_arguments,
    )


if __name__ == '__main__':
    main()
