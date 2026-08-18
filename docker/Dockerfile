# syntax=docker/dockerfile:1

ARG TINYNAV_BASE
FROM ${TINYNAV_BASE} AS installer

USER root
COPY runtime_files.tsv /opt/umi-vio-src/runtime_files.tsv
COPY scripts/ /opt/umi-vio-src/scripts/
COPY src/ /opt/umi-vio-src/src/
RUN bash /opt/umi-vio-src/scripts/install.sh /opt/umi-runtime

FROM ${TINYNAV_BASE}
COPY --from=installer /opt/umi-runtime/ /tinynav/tool/umi/
WORKDIR /tinynav
