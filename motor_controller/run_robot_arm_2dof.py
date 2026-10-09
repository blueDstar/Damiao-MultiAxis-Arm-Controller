"""Launch the 3D robot-arm GUI using the same local CAN packages as multi_motor."""

from run_multi_motor import prepare_local_packages


if __name__ == "__main__":
    prepare_local_packages()
    from robot_arm_2dof.gui import main

    main()
