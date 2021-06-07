#!/usr/bin/env bash

# This is inspired from
# https://github.com/envoyproxy/envoy/blob/master/ci/run_clang_tidy.sh

set -eo pipefail

ALPHA_DIR=${ALPHA_DIR:-$(realpath "$(dirname "$BASH_SOURCE")"/../..)}
#LLVM_PREFIX=${LLVM_PREFIX:-$(llvm-config --prefix)}

# Quick syntax check of .clang-tidy.
clang-tidy -dump-config >/dev/null 2>clang-tidy-config-errors.txt
if [[ -s clang-tidy-config-errors.txt ]]; then
  cat clang-tidy-config-errors.txt
  rm clang-tidy-config-errors.txt
  exit 1
fi
rm clang-tidy-config-errors.txt

echo "Generating compilation database..."
${ALPHA_DIR}/scripts/code/gen_compilation_database.py
ln -sf ${ALPHA_DIR}/.clang-tidy $(bazel info output_base)/

function filter_excludes() {
  grep -v "_test\.cc$" | grep -v "_benchmark\.cc$"
}

if [[ ${RUN_FULL_CLANG_TIDY} == 1 ]]; then
  echo "Running full clang-tidy..."
  ${LLVM_PREFIX}/share/clang/run-clang-tidy.py $@
elif [[ ${RUN_PR_CLANG_TIDY} == 1 ]]; then
  echo "Running clang-tidy-diff against master branch..."
  git diff -U0 remotes/origin/master | filter_excludes |
    ${LLVM_PREFIX}/share/clang/clang-tidy-diff.py -p1 $@
else
  echo "Running clang-tidy-diff against previous commit..."
  git diff -U0 HEAD^ | filter_excludes |
    ${LLVM_PREFIX}/share/clang/clang-tidy-diff.py -p1 $@
fi
