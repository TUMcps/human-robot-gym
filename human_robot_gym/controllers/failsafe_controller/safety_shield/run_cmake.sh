mkdir build 
cd build
sudo cmake -DPYTHON_EXECUTABLE:FILEPATH=$CONDA_PREFIX/bin/python \
    -DPYTHON_INCLUDE_DIR=$CONDA_PREFIX/include/python3.8/ \
    -DPYTHON_LIBRARY:FILEPATH=$CONDA_PREFIX/lib/libpython3.8.so \
    ..