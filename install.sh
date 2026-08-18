#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DESTINATION="${1:-/tinynav/tool/umi}"
MANIFEST="${REPO_ROOT}/runtime_files.tsv"

[[ "${DESTINATION}" == /* ]] || {
  echo "destination must be an absolute path: ${DESTINATION}" >&2
  exit 2
}
[[ -f "${MANIFEST}" ]]

install -d -m 0755 "${DESTINATION}"
destinations_file="$(mktemp)"
trap 'rm -f "${destinations_file}"' EXIT
python_files=()
shell_files=()

while IFS=$'\t' read -r source_path destination mode; do
  [[ -z "${source_path}" || "${source_path}" == \#* ]] && continue
  [[ "${source_path}" != /* && "${destination}" != */* ]]
  [[ "${mode}" =~ ^0[67][0-7][0-7]$ ]]
  [[ -f "${REPO_ROOT}/${source_path}" ]]
  if grep -Fxq "${destination}" "${destinations_file}"; then
    echo "duplicate destination in manifest: ${destination}" >&2
    exit 3
  fi
  printf '%s\n' "${destination}" >> "${destinations_file}"
  install -m "${mode}" "${REPO_ROOT}/${source_path}" "${DESTINATION}/${destination}"
  [[ "${destination}" == *.py ]] && python_files+=("${DESTINATION}/${destination}")
  [[ "${destination}" == *.sh ]] && shell_files+=("${DESTINATION}/${destination}")
done < "${MANIFEST}"

for script in "${shell_files[@]}"; do
  bash -n "${script}"
done
python3 -c \
  'import ast, pathlib, sys; [ast.parse(pathlib.Path(p).read_text()) for p in sys.argv[1:]]' \
  "${python_files[@]}"

printf 'INSTALLED destination=%s files=%s\n' \
  "${DESTINATION}" "$(wc -l < "${destinations_file}" | tr -d ' ')"
