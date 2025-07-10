FROM ghcr.io/aica-technology/ros2-ws:v2.0.5-jazzy AS dependencies

COPY --from=ghcr.io/aica-technology/control-libraries:v9.2.0 / /

RUN sudo apt-get update && sudo apt-get install -y libpoco-dev

WORKDIR /source
RUN git clone --recursive https://github.com/Swiss-Battery-Technology-Center/libfranka
RUN cd libfranka && git checkout 5721223445f0c53ef9ec8ee1bea5ee3fc93679d3 && git submodule update && mkdir build
WORKDIR /source/libfranka/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. && cmake --build . && make -j && sudo make install -j && sudo ldconfig

WORKDIR ${HOME}
RUN sudo rm -rf /source

FROM dependencies AS development

RUN sudo apt-get update && sudo apt-get install -y clangd clang-format jq


FROM dependencies AS runtime

COPY --chown=${USER} ./source ./
RUN cd franka_lightweight_interface && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make \
  && sudo make install && sudo ldconfig
WORKDIR ${HOME}
USER ${USER}

ENTRYPOINT /bin/bash
