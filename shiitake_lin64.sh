ctest -S /home/srit/src/rtk/rtk-dashboard/shiitake_suse_gcc.cmake -V
ctest -S /home/srit/src/rtk/rtk-dashboard/shiitake_suse_gcc_cuda40.cmake -V
ctest -S /home/srit/src/rtk/rtk-dashboard/shiitake_suse_gcc_cuda32.cmake -V
ctest -S /home/srit/src/rtk/rtk-dashboard/shiitake_suse_gcc_opencl.cmake -V
ctest -S /home/srit/src/rtk/rtk-dashboard/shiitake_suse_intel.cmake -V
ctest -S /home/srit/src/rtk/rtk-dashboard/shiitake_style.cmake -V
ctest -S /home/srit/src/rtk/rtk-dashboard/shiitake_doxygen.cmake -V
rsync -a --delete            /home/srit/src/rtk/dashboard_tests/RTK-Doxygen/Doxygen/html \
    ssh.creatis.insa-lyon.fr:/home/srit/src/rtk/dashboard_tests/RTK-Doxygen/Doxygen
