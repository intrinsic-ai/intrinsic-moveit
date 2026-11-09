# robot_meshes_vendor

Vendored third-party Universal Robots visual and collision mesh assets used by [`robot_hardware_descriptions`](../../robot_hardware_descriptions/README.md) (`package://robot_meshes_vendor/meshes/...`).

## Directory Structure

```text
meshes/
├── ur3e/
│   ├── visual/      # UR3e visual DAE meshes + BSD-3-Clause LICENSE
│   └── collision/   # UR3e collision STL meshes (base.stl + *.wrapped.1cm.stl) + BSD-3-Clause LICENSE
└── ur5e/
    ├── visual/      # UR5e visual DAE meshes + BSD-3-Clause LICENSE
    └── collision/   # UR5e collision STL meshes (base.stl + *.wrapped.1cm.stl) + BSD-3-Clause LICENSE
```

## Provenance & Licensing

* Vendored from `intrinsic-core/third_party/universal_robots/{ur3e,ur5e}/`.
* Distributed under **BSD-3-Clause**; each subdirectory retains its upstream `LICENSE` file.
