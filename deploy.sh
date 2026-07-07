#!/usr/bin/env bash
set -euo pipefail

# =============================================================================
# deploy.sh — install umi_vio_converter into the tinynav container. RUN ON HOST.
#
# Workflow:
#   1. rsync this folder to the H200 host (docker lives there), e.g.
#        rsync -av umi_vio_converter/ h1:/data/shared/tools/umi_vio_converter/
#   2. on the host:  bash deploy.sh deploy [container]
#   3. validate one episode (see README), then:  bash deploy.sh finalize [container]
#   Rollback anytime:  bash deploy.sh rollback [container]
#
# Overwrites only 3 files (codec/reader/merge); backs each up to *.pre_tactile_bak
# inside the container AND snapshots the whole image before touching anything.
# Never commits :latest until you run `finalize`.
# =============================================================================

CMD="${1:-deploy}"
CONTAINER="${2:-${VIO_CONTAINER_NAME:-tinynav_flatbuffer}}"
IMAGE_REPO="${VIO_IMAGE_REPO:-tinynav_flatbuffer_saved}"
DST="/tinynav/tool/umi"
SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# files that OVERWRITE existing container files (need backup)
OVERWRITE=(flatbuffer_codec.py flatbuffer_reader.py merge_sqlite_mcap.py)
# brand-new files (no backup needed)
NEWFILES=(backfill_tactile.py umi_vio_converter.py umi_vio_converter.sh _build_map_one_sensor.sh)

dex() { docker exec "${CONTAINER}" bash -lc "$*"; }

require_container() {
  docker inspect "${CONTAINER}" >/dev/null 2>&1 || { echo "No such container: ${CONTAINER}" >&2; exit 1; }
}

do_deploy() {
  require_container
  local stamp; stamp="$(date +%Y%m%d-%H%M%S)"
  echo "==> Snapshot image ${IMAGE_REPO}:backup-${stamp}"
  docker commit "${CONTAINER}" "${IMAGE_REPO}:backup-${stamp}"

  echo "==> Back up originals inside container (once)"
  for f in "${OVERWRITE[@]}"; do
    dex "[ -f ${DST}/${f} ] && { [ -f ${DST}/${f}.pre_tactile_bak ] || cp -p ${DST}/${f} ${DST}/${f}.pre_tactile_bak; } || true"
  done

  echo "==> Copy patched + new files into ${CONTAINER}:${DST}"
  for f in "${OVERWRITE[@]}"; do
    docker cp "${SELF_DIR}/patched/${f}" "${CONTAINER}:${DST}/${f}"
  done
  for f in "${NEWFILES[@]}"; do
    docker cp "${SELF_DIR}/${f}" "${CONTAINER}:${DST}/${f}"
  done
  dex "chmod +x ${DST}/umi_vio_converter.sh ${DST}/_build_map_one_sensor.sh"
  dex "python3 -c 'import ast,sys; [ast.parse(open(f).read()) for f in [\"${DST}/flatbuffer_codec.py\",\"${DST}/flatbuffer_reader.py\",\"${DST}/merge_sqlite_mcap.py\",\"${DST}/backfill_tactile.py\",\"${DST}/umi_vio_converter.py\"]]; print(\"syntax_ok\")'"

  echo
  echo "Deployed (live in container, NOT committed to :latest)."
  echo "Next: validate one episode, then:  bash $0 finalize ${CONTAINER}"
}

do_rollback() {
  require_container
  echo "==> Restore originals from *.pre_tactile_bak in ${CONTAINER}:${DST}"
  for f in "${OVERWRITE[@]}"; do
    dex "[ -f ${DST}/${f}.pre_tactile_bak ] && cp -p ${DST}/${f}.pre_tactile_bak ${DST}/${f} && echo restored ${f} || echo 'no backup for ${f} (left as-is)'"
  done
  echo "New files left in place (harmless); remove manually if desired."
}

do_finalize() {
  require_container
  echo "==> Commit ${IMAGE_REPO}:latest from validated container"
  docker commit "${CONTAINER}" "${IMAGE_REPO}:latest"
  echo "Committed ${IMAGE_REPO}:latest"
}

case "${CMD}" in
  deploy)   do_deploy ;;
  rollback) do_rollback ;;
  finalize) do_finalize ;;
  *) echo "Usage: $0 [deploy|rollback|finalize] [container]" >&2; exit 2 ;;
esac
