#!/usr/bin/env bash
# BSD 3-Clause License
#
# Copyright (c) 2026, Nicola Pedrocchi
# All rights reserved.
#
# See the project license for redistribution terms and warranty disclaimer.
#
# Builds Pinocchio (the version pinned in .repos) and thor_math with the same
# flags and installs both into one prefix. Nothing is installed elsewhere: the
# system Pinocchio and the one of ROS are not touched, and only the programs
# that look into the prefix (CMAKE_PREFIX_PATH) use this build.
#
# Run with --help for the options.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat <<EOF
Usage: scripts/build_thor.sh [options]

Builds Pinocchio and thor_math with the same flags into one prefix.

Where:
  --prefix DIR          install prefix (default: $REPO_ROOT/install/<arch>)
  --build-dir DIR       build directory (default: $REPO_ROOT/build/deps-<arch>)
  --deps-dir DIR        where the Pinocchio sources are fetched
                        (default: $REPO_ROOT/deps)
  --dep-prefix DIR      extra prefix where the dependencies of Pinocchio
                        (Eigen, Boost, urdfdom) are found, e.g. /opt/ros/jazzy;
                        repeatable

What:
  --arch FLAGS          architecture flags, e.g. "-march=x86-64-v3"; default
                        none (portable build). They change the layout of Eigen
                        objects: Pinocchio, thor_math and every program using
                        them must share them. thor_math exports them with its
                        CMake target, so programs that link it get them.
  --native              same as --arch -march=native: fastest on this machine,
                        not portable to other CPUs
  --extra-flags FLAGS   other compiler flags for both, e.g.
                        "-fno-semantic-interposition -fno-plt"
  --build-type TYPE     CMake build type (default: Release)
  --template-instantiation on|off
                        Pinocchio precompiled template instances (default:
                        off: builds in seconds instead of minutes, measured as
                        fast at run time)
  --no-urdf             Pinocchio without the URDF parser (no urdfdom); the
                        thor_math tests and examples are not built

How:
  --toolchain FILE      CMake toolchain file, for cross compilation (e.g.
                        VxWorks); tests are built but not run
  --generator NAME      CMake generator (default: Ninja if available)
  --jobs N              parallel jobs (default: from the physical memory,
                        $JOB_MEMORY_MB_DEFAULT MB per job, $RESERVED_MEMORY_MB_DEFAULT MB left to the system)
  --tests               build and run the thor_math tests
  --clean               remove the build directory first
  --dry-run             print the commands without running them
  -h, --help            this help

Examples:
  scripts/build_thor.sh                                   # portable
  scripts/build_thor.sh --arch -march=x86-64-v3 --tests   # AVX2/FMA
  scripts/build_thor.sh --toolchain vxworks.cmake --arch "-march=..."

Then build your program against the prefix:
  cmake -S app -B build/app -DCMAKE_PREFIX_PATH=<prefix>
EOF
}

JOB_MEMORY_MB_DEFAULT=3072
RESERVED_MEMORY_MB_DEFAULT=4096

prefix=""
build_dir=""
deps_dir="$REPO_ROOT/deps"
dep_prefixes=()
arch=""
extra_flags=""
build_type="Release"
template_instantiation="OFF"
urdf="ON"
toolchain=""
generator=""
jobs=""
run_tests=0
clean=0
dry_run=0

die() { echo "build_thor.sh: $*" >&2; exit 1; }
need_arg() { [[ $# -ge 2 && -n "$2" ]] || die "$1 needs a value"; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --prefix)      need_arg "$@"; prefix="$2"; shift 2 ;;
    --build-dir)   need_arg "$@"; build_dir="$2"; shift 2 ;;
    --deps-dir)    need_arg "$@"; deps_dir="$2"; shift 2 ;;
    --dep-prefix)  need_arg "$@"; dep_prefixes+=("$2"); shift 2 ;;
    --arch)        need_arg "$@"; arch="$2"; shift 2 ;;
    --native)      arch="-march=native"; shift ;;
    --extra-flags) need_arg "$@"; extra_flags="$2"; shift 2 ;;
    --build-type)  need_arg "$@"; build_type="$2"; shift 2 ;;
    --template-instantiation)
      need_arg "$@"
      case "$2" in
        on|ON)   template_instantiation="ON" ;;
        off|OFF) template_instantiation="OFF" ;;
        *) die "--template-instantiation takes on or off" ;;
      esac
      shift 2 ;;
    --no-urdf)     urdf="OFF"; shift ;;
    --toolchain)   need_arg "$@"; toolchain="$(realpath "$2")"; shift 2 ;;
    --generator)   need_arg "$@"; generator="$2"; shift 2 ;;
    --jobs)        need_arg "$@"; jobs="$2"; shift 2 ;;
    --tests)       run_tests=1; shift ;;
    --clean)       clean=1; shift ;;
    --dry-run)     dry_run=1; shift ;;
    -h|--help)     usage; exit 0 ;;
    *) die "unknown option $1 (see --help)" ;;
  esac
done

[[ -z "$toolchain" || -f "$toolchain" ]] || die "toolchain file not found: $toolchain"
[[ -z "$jobs" || "$jobs" =~ ^[1-9][0-9]*$ ]] || die "--jobs takes a positive integer"

# A name for this configuration, used in the default directories, so that
# builds with different flags never mix.
if [[ -n "$arch" ]]; then
  arch_tag="$(echo "$arch" | sed -E 's/-march=//g; s/[^A-Za-z0-9._-]+/_/g; s/^_+|_+$//g')"
else
  arch_tag="portable"
fi
[[ -n "$toolchain" ]] && arch_tag="$arch_tag-$(basename "$toolchain" .cmake)"
prefix="$(realpath -m "${prefix:-$REPO_ROOT/install/$arch_tag}")"
build_dir="$(realpath -m "${build_dir:-$REPO_ROOT/build/deps-$arch_tag}")"
deps_dir="$(realpath -m "$deps_dir")"

# Parallel jobs from the physical memory: a translation unit that includes
# Pinocchio peaks at a few GB, and too many of them freeze the machine.
if [[ -z "$jobs" ]]; then
  mem_mb=$(awk '/^MemTotal:/ { print int($2 / 1024) }' /proc/meminfo 2>/dev/null || echo 0)
  cpus=$(nproc 2>/dev/null || echo 1)
  jobs=$(( (mem_mb - RESERVED_MEMORY_MB_DEFAULT) / JOB_MEMORY_MB_DEFAULT ))
  (( jobs > cpus )) && jobs=$cpus
  (( jobs < 1 )) && jobs=1
fi

if [[ -z "$generator" ]] && command -v ninja > /dev/null; then
  generator="Ninja"
fi

run() {
  echo "+ $*"
  (( dry_run )) || "$@"
}

# Pinocchio version and URL from .repos (the same as the CI).
read -r pin_url pin_version < <(awk '
  /^  pinocchio:/ { inside = 1; next }
  /^  [^ ]/       { inside = 0 }
  inside && $1 == "url:"     { url = $2 }
  inside && $1 == "version:" { version = $2 }
  END { print url, version }' "$REPO_ROOT/.repos")
[[ -n "$pin_url" && -n "$pin_version" ]] || die "pinocchio not found in $REPO_ROOT/.repos"
pin_src="$deps_dir/pinocchio"
pin_patch="$REPO_ROOT/.github/patches/pinocchio-boost-system.patch"

flags="$(echo "$arch $extra_flags" | xargs)"
common_args=(-DCMAKE_BUILD_TYPE="$build_type" -DCMAKE_INSTALL_PREFIX="$prefix")
[[ -n "$generator" ]] && common_args+=(-G "$generator")
[[ -n "$toolchain" ]] && common_args+=(-DCMAKE_TOOLCHAIN_FILE="$toolchain")
dep_path="$(IFS=';'; echo "${dep_prefixes[*]-}")"

echo "Pinocchio $pin_version and thor_math"
echo "  prefix:     $prefix"
echo "  build:      $build_dir"
echo "  flags:      ${flags:-(none)}"
echo "  jobs:       $jobs${generator:+, $generator}"
[[ -n "$toolchain" ]] && echo "  toolchain:  $toolchain"
[[ -n "$dep_path" ]] && echo "  deps from:  $dep_path"
if [[ -n "${ROS_DISTRO:-}" && -z "$dep_path" ]]; then
  echo "  note: ROS $ROS_DISTRO is sourced; its packages may be picked up as dependencies." >&2
fi

if (( clean )); then
  run rm -rf "$build_dir"
fi

# --- Pinocchio sources --------------------------------------------------------
if [[ ! -d "$pin_src/.git" ]]; then
  run mkdir -p "$deps_dir"
  run git clone --depth 1 --branch "$pin_version" --recurse-submodules --shallow-submodules \
    "$pin_url" "$pin_src"
else
  current="$(git -C "$pin_src" describe --tags --exact-match 2>/dev/null || echo unknown)"
  [[ "$current" == "$pin_version" ]] ||
    die "$pin_src is at $current, .repos asks for $pin_version (remove it or check it out)"
fi
# Boost.System is header-only in every supported release (see the CI).
if (( dry_run )); then
  echo "+ git -C $pin_src apply $pin_patch   (unless already applied)"
elif git -C "$pin_src" apply --check "$pin_patch" 2> /dev/null; then
  run git -C "$pin_src" apply "$pin_patch"
elif ! git -C "$pin_src" apply --reverse --check "$pin_patch" 2> /dev/null; then
  die "cannot apply $pin_patch to $pin_src"
fi

# --- Pinocchio ----------------------------------------------------------------
# Pinocchio sets its own install RPATH ($ORIGIN); USE_LINK_PATH adds the
# directories of dependencies outside the system paths (e.g. --dep-prefix).
run cmake -S "$pin_src" -B "$build_dir/pinocchio" "${common_args[@]}" \
  -DCMAKE_CXX_FLAGS="$flags" \
  -DCMAKE_PREFIX_PATH="$dep_path" \
  -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON \
  -DBUILD_TESTING=OFF -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_BENCHMARK=OFF -DBUILD_UTILS=OFF \
  -DBUILD_WITH_COLLISION_SUPPORT=OFF -DBUILD_WITH_URDF_SUPPORT="$urdf" \
  -DENABLE_TEMPLATE_INSTANTIATION="$template_instantiation"
run cmake --build "$build_dir/pinocchio" --parallel "$jobs"
run cmake --install "$build_dir/pinocchio"

# --- thor_math ----------------------------------------------------------------
# The architecture flags go to THOR_MATH_ARCH_FLAGS, which exports them to the
# users of the target; the other flags stay private to this build.
examples="$urdf"
tests="OFF"
(( run_tests )) && [[ "$urdf" == "ON" ]] && tests="ON"
run cmake -S "$REPO_ROOT/thor_math" -B "$build_dir/thor_math" "${common_args[@]}" \
  -DCMAKE_CXX_FLAGS="$extra_flags" \
  -DTHOR_MATH_ARCH_FLAGS="$arch" \
  -DCMAKE_PREFIX_PATH="$prefix${dep_path:+;$dep_path}" \
  -Dpinocchio_DIR="$prefix/lib/cmake/pinocchio" \
  -DCMAKE_INSTALL_RPATH="$prefix/lib" \
  -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON \
  -DBUILD_EXAMPLES="$examples" \
  -DBUILD_UNIT_TESTS="$tests" \
  -DBUILD_INTEGRATION_TESTS="$tests"
run cmake --build "$build_dir/thor_math" --parallel "$jobs"
if (( run_tests )); then
  if [[ "$urdf" == "OFF" ]]; then
    echo "Tests skipped: they load URDF models (--no-urdf)."
  elif [[ -n "$toolchain" ]]; then
    echo "Tests built but not run: cross compilation (--toolchain)."
  else
    run ctest --test-dir "$build_dir/thor_math" --output-on-failure
  fi
fi
run cmake --install "$build_dir/thor_math"

# --- record of the configuration ----------------------------------------------
info="$prefix/share/thor_math/build-info.txt"
if (( dry_run )); then
  echo "+ write $info"
else
  mkdir -p "$(dirname "$info")"
  {
    echo "built by scripts/build_thor.sh on $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "thor_core commit:   $(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)$(git -C "$REPO_ROOT" diff --quiet 2>/dev/null || echo ' (modified)')"
    echo "pinocchio:          $pin_version (template instantiation $template_instantiation, URDF $urdf)"
    echo "build type:         $build_type"
    echo "architecture flags: ${arch:-(none)}"
    echo "extra flags:        ${extra_flags:-(none)}"
    echo "toolchain:          ${toolchain:-(native)}"
    echo "compiler:           $(grep -m1 '^CMAKE_CXX_COMPILER:' "$build_dir/thor_math/CMakeCache.txt" | cut -d= -f2)"
  } > "$info"
fi

cat <<EOF

Installed into $prefix. Use it with:
  cmake -S <your app> -B <build> -DCMAKE_PREFIX_PATH=$prefix
EOF
if [[ -n "$arch" ]]; then
  echo "Programs that link thor_math::thor_math get '$arch' from its target."
  echo "Other code exchanging Eigen objects with them must use the same flags."
fi
