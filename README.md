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

`rdyn_core` is not a rosdep key and, depending on your ROS distribution, neither is `pinocchio`: build them from [dep.repos](dep.repos).

## Build

```bash
cmake -S thor_math -B build/thor_math
cmake --build build/thor_math -j
```

## License

BSD 3-Clause, see [LICENSE](LICENSE).
