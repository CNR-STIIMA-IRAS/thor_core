# thor_core

[![build](https://github.com/CNR-STIIMA-IRAS/thor_core/actions/workflows/build.yml/badge.svg)](https://github.com/CNR-STIIMA-IRAS/thor_core/actions/workflows/build.yml)
[![License: BSD-3-Clause](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](LICENSE)
![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-E95420?logo=ubuntu&logoColor=white)
![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white)
![Ubuntu 26.04](https://img.shields.io/badge/Ubuntu-26.04-E95420?logo=ubuntu&logoColor=white)

Core libraries for THOR.

## Packages

- `thor_math`: math utilities built on Eigen and Pinocchio, including `ThorQP`, an MPC for manipulators that scales the speed of a nominal trajectory and keeps a safe distance from dynamic obstacles (spheres or capsules) through a control barrier function.

## Dependencies

Source dependencies are listed in [.repos](.repos) and can be fetched with [vcstool](https://github.com/dirk-thomas/vcstool):

```bash
mkdir -p deps && vcs import deps < .repos
```

System packages: `libeigen3-dev`, `libboost-all-dev`, `liburdfdom-dev`.

With rosdep, system dependencies declared in [thor_math/package.xml](thor_math/package.xml) can be installed automatically:

```bash
rosdep install --from-paths [src_of_your_ws] --ignore-src -r -y 
```

Depending on your ROS distribution, `pinocchio` may not be available as a rosdep key; it can be built from [.repos](.repos), or with the script below.

## Build

```bash
cmake -S thor_math -B build/thor_math
cmake --build build/thor_math -j
```

### Pinocchio and thor_math from source, optionally optimized

[scripts/build_thor.sh](scripts/build_thor.sh) builds the Pinocchio pinned in [.repos](.repos) and `thor_math` with the same flags, and installs both into one prefix (by default `install/<arch>`). Nothing else on the machine changes: the system or ROS Pinocchio stays as it is, and only the programs pointed at the prefix use this build. It needs Eigen, Boost and, unless `--no-urdf`, urdfdom.

```bash
scripts/build_thor.sh --tests                                   # portable
scripts/build_thor.sh --arch -march=x86-64-v3 --tests           # AVX2/FMA
scripts/build_thor.sh --dep-prefix /opt/ros/jazzy --tests       # urdfdom from ROS
scripts/build_thor.sh --toolchain <file>.cmake --arch "<flags>" # cross compilation
cmake -S <your app> -B <build> -DCMAKE_PREFIX_PATH=install/x86-64-v3
```

- **Architecture flags must match.** Flags such as `-march=x86-64-v3` change the layout of Eigen objects, so Pinocchio, `thor_math` and the application must share them, or the program crashes. The script builds both libraries with the same flags, and `thor_math` exports them with its CMake target (`THOR_MATH_ARCH_FLAGS`), so applications that link `thor_math::thor_math` get them too.
- **Parallel jobs** are sized by the physical memory (3 GB per job, 4 GB left to the system), because a translation unit that includes Pinocchio peaks at a few GB.
- **Pinocchio without precompiled template instances** (`--template-instantiation off`, the default) builds in seconds instead of minutes, and is as fast at run time.
- The configuration is recorded in `<prefix>/share/thor_math/build-info.txt`. `scripts/build_thor.sh --help` lists all the options; measured gains of the build options are in [thor_math/docs/api.md](thor_math/docs/api.md) (section 3.1).

## Example

[thor_math/examples/cbf_mpc_example.cpp](thor_math/examples/cbf_mpc_example.cpp) runs the MPC with the control barrier function on a UR10 covered by spheres, among a walking person (a vertical capsule for the body, a sphere for the hand) and a static fixture. Run it again with `--no-cbf` to compare, or with `--continuous-cbf` to use the continuous-time CBF of the paper instead of the discrete-time one along the horizon:

```bash
cmake -S thor_math -B build/thor_math -DBUILD_EXAMPLES=ON
cmake --build build/thor_math -j
./build/thor_math/thor_math_cbf_mpc_example
./build/thor_math/thor_math_cbf_mpc_example --no-cbf
./build/thor_math/thor_math_cbf_mpc_example --continuous-cbf
./build/thor_math/thor_math_cbf_mpc_example --linearizations 4 --damping 0.7 --tolerance 0.5
```

## Documentation

The API reference, with the formulation of the MPC and of the control barrier function, is in [thor_math/docs/api.md](thor_math/docs/api.md). The public headers are documented with Doxygen; to generate the HTML reference in `build/thor_math/docs/html`:

```bash
cmake -S thor_math -B build/thor_math -DBUILD_DOCS=ON
cmake --build build/thor_math --target thor_math_docs
```

Changes are listed in [thor_math/CHANGELOG.md](thor_math/CHANGELOG.md).

## License

BSD 3-Clause, see [LICENSE](LICENSE).
