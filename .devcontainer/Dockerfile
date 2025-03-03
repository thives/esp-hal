FROM alpine:latest

RUN apk add clang lld curl

#RUN apt -y update && apt install -y --no-install-recommends perl

#RUN apt update && export DEBIAN_FRONTEND=noninteractive && apt -y install --no-install-recommends \
#    protobuf-compiler

ENV CARGO_MAKE_DEFAULT_PROFILE="production"

ADD https://musl.cc/aarch64-linux-musl-cross.tgz /tmp/aarch64-linux-musl-cross.tgz
RUN tar -xf /tmp/aarch64-linux-musl-cross.tgz -C /opt
