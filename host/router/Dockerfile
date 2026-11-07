# syntax=docker/dockerfile:1
# Native build tools even when producing the Pi's ARM64 image (no QEMU needed).
FROM --platform=$BUILDPLATFORM rust:1.93.0-bookworm AS build
ARG TARGETARCH
ARG ZENOH_REV=2687c51352121f006e3a603ce07925a8ad0b295c
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc-aarch64-linux-gnu libc6-dev-arm64-cross git ca-certificates python3 && rm -rf /var/lib/apt/lists/*
WORKDIR /src
RUN git init && git remote add origin https://github.com/eclipse-zenoh/zenoh.git && \
    git fetch --depth 1 origin "$ZENOH_REV" && git checkout --detach FETCH_HEAD && \
    test "$(git rev-parse HEAD)" = "$ZENOH_REV"
COPY patch_pipeline.py /build/patch_pipeline.py
RUN python3 /build/patch_pipeline.py io/zenoh-transport/src/common/pipeline.rs
# Run the real pipeline regression on the build host, then cross-compile only
# the two transports the serial router needs. Do not upgrade the wire protocol.
RUN --mount=type=cache,target=/usr/local/cargo/registry \
    --mount=type=cache,target=/src/target \
    cargo test --locked -p zenoh-transport --no-default-features \
      --features test,transport_tcp,transport_serial --lib tx_pipeline_congestion_clears_without_qos && \
    case "$TARGETARCH" in arm64) target=aarch64-unknown-linux-gnu;; amd64) target=x86_64-unknown-linux-gnu;; *) exit 1;; esac && \
    rustup target add "$target" && \
    CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc \
      cargo build --locked --release -p zenohd --no-default-features \
        --features zenoh/transport_tcp,zenoh/transport_serial --target "$target" && \
    cp "target/$target/release/zenohd" /build/picoros-zenohd

FROM debian:bookworm-slim AS runtime
COPY --from=build /build/picoros-zenohd /usr/local/bin/picoros-zenohd
COPY --from=build /src/LICENSE /src/NOTICE.md /usr/share/doc/picoros-zenohd/
COPY patch_pipeline.py /usr/share/doc/picoros-zenohd/patch_pipeline.py
LABEL org.opencontainers.image.source="https://github.com/eclipse-zenoh/zenoh" \
      org.opencontainers.image.revision="2687c51352121f006e3a603ce07925a8ad0b295c" \
      turtlebot.zenoh.fix="2842-no-qos-congestion-stage-index"
ENTRYPOINT ["/usr/local/bin/picoros-zenohd"]
CMD ["--config", "/config/serial-router.json5"]
