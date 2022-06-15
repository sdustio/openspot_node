FROM ghcr.io/sdustio/openspot:dev

RUN set -eux; \
    mkdir -p /tmp/serial; \
    curl -LsSf -o serial.tar.gz https://github.com/sdustio/serial/archive/refs/tags/v2.0.0.tar.gz; \
    tar --gzip --extract --directory /tmp/serial --strip-components=1 --file serial.tar.gz; \
    rm serial.tar.gz; \
    cd /tmp/serial; \
    cmake -DCMAKE_BUILD_TYPE:STRING=Release -B ./build; \
    cmake --build ./build --target install --config Release; \
    cd /; \
    rm -rf /tmp/serial;

COPY spotng.deb /tmp/
RUN dpkg -i /tmp/spotng.deb