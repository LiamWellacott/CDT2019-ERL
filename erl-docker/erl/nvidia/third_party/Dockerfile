FROM erl/nvidia:cudagl-cudnn

RUN apt update && \
    apt install -y libflann-dev libopenblas-base libopenblas-dev

RUN apt install -y python-pip

RUN pip install -U pip setuptools numpy
RUN pip install -U tensorflow-gpu

# Build dlib with cuda support

RUN mkdir tools && \
    cd tools && \
    git clone https://github.com/davisking/dlib.git && \
    cd dlib && \
    git submodule init && \
    git submodule update

RUN cd tools/dlib && \
    mkdir build && cd build && \
    cmake -D DLIB_USE_CUDA=1 -D USE_AVX_INSTRUCTIONS=1 .. && \
    cmake --build . --config Release && \
    make install

RUN cd tools/dlib && \
    python setup.py install


# For some reason dlib changes permission of cmake and makes it unusable.
RUN rm -rf cmake

# Install cmake 3.15.5
RUN git clone https://gitlab.kitware.com/cmake/cmake.git && \
    cd cmake && git checkout tags/v3.15.5 && \
    ./bootstrap --parallel=8 && \
    make -j8 && \
    make install
