# thor_core

[![build](https://github.com/CNR-STIIMA-IRAS/thor_core/actions/workflows/build.yml/badge.svg)](https://github.com/CNR-STIIMA-IRAS/thor_core/actions/workflows/build.yml)
[![License: BSD-3-Clause](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](LICENSE)
![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-E95420?logo=ubuntu&logoColor=white)
![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white)
![Ubuntu 26.04](https://img.shields.io/badge/Ubuntu-26.04-E95420?logo=ubuntu&logoColor=white)

Core libraries for THOR.

## Packages

- `thor_math`: math utilities built on Eigen and Pinocchio.

## Dependencies

Source dependencies are listed in [dep.repos](dep.repos) and can be fetched with [vcstool](https://github.com/dirk-thomas/vcstool):

```bash
mkdir -p deps && vcs import deps < dep.repos
```

System packages: `libeigen3-dev`, `libboost-all-dev`, `liburdfdom-dev`.

With rosdep, system dependencies declared in [thor_math/package.xml](thor_math/package.xml) can be installed automatically:

```bash
rosdep install --from-paths [src_of_your_ws] --ignore-src -r -y 
```

Depending on your ROS distribution, `pinocchio` may not be available as a rosdep key; it can be built from [dep.repos](dep.repos).

## Build

```bash
cmake -S thor_math -B build/thor_math
cmake --build build/thor_math -j
```

## Example

[thor_math/examples/cbf_mpc_example.cpp](thor_math/examples/cbf_mpc_example.cpp) runs the MPC with the control barrier function on a UR10 covered by spheres, among a walking person (torso and hand spheres) and a static fixture. Run it again with `--no-cbf` to compare:

```bash
cmake -S thor_math -B build/thor_math -DBUILD_EXAMPLES=ON
cmake --build build/thor_math -j
./build/thor_math/thor_math_cbf_mpc_example
./build/thor_math/thor_math_cbf_mpc_example --no-cbf
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
