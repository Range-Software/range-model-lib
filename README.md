# Range FEA Model Library

`range-model-lib` is the core data-model layer of the Range FEA suite. It defines every entity that describes a finite-element problem — geometry, mesh, physics setup, solver configuration, and results — and provides the serialisation, mesh-generation, and post-processing routines that operate on them.

## Responsibilities

**Geometry and mesh**
- `RNode`, `RElement` — nodes and elements with shape-function evaluation, Jacobian computation, and isoparametric coordinate mapping
- `RPoint`, `RLine`, `RSurface`, `RVolume` — entity groups that own sets of elements and carry boundary/initial conditions and material assignments
- `RTriangle`, `RTetrahedron`, `RSegment` — geometric primitives used by the mesh generator and ray-tracing subsystems
- `RMeshGenerator`, `RMeshInput`, `RMeshSetup` — interface to the TetGen tetrahedral mesher
- `RShapeGenerator` — parametric shape creation

**Physics setup**
- `RBoundaryCondition`, `RInitialCondition`, `REnvironmentCondition` — typed conditions attached to entity groups, each carrying time-dependent `RConditionComponent` values
- `RMaterial`, `RMaterialProperty` — material database with per-property interpolation
- `RTimeSolver`, `RModalSetup`, `REigenValueSolverConf`, `RMatrixSolverConf` — solver configuration objects

**Results and post-processing**
- `RVariable`, `RVariableData` — per-node or per-element result fields
- `RScalarField`, `RVectorField`, `RStreamLine`, `RIso`, `RCut` — derived post-processing datasets
- `RMonitoringPoint`, `RMonitoringPointManager` — time-history probes at arbitrary spatial locations
- `RViewFactorMatrix` — radiation view-factor storage and I/O

**Sparse linear algebra**
- `RSparseVector<T>`, `RSparseMatrix` — sorted sparse-vector and sparse-matrix types used by all solvers for global system assembly

**File I/O**
- `RModel` serialises the complete problem (mesh + setup + results) to the native `.rml` binary format
- Import: `.msh` (Gmsh), `.stl`, `.raw`
- `RFileManager`, `RFileHeader`, `RSaveFile` — file versioning and format management

## Central class

`RModel` inherits from `RProblem` (physics-type registry) and `RResults` (result-variable storage). It owns the full mesh (`nodes`, `elements`), all entity groups, all conditions, materials, solver settings, and post-processing datasets. All solvers in `range-solver-lib` receive a pointer to an `RModel` instance and read/write through its API.

## Dependencies

| Dependency | Role |
|---|---|
| `range-base-lib` | Math primitives, logging, vectors, matrices |
| `Qt6::Core`, `Qt6::Gui` | QString, QVector, file I/O, GUI-facing data types |

## Build

Built as a static library via CMake as part of the top-level Range FEA build:

```bash
cmake -S src -B build-Release
cmake --build build-Release --parallel -- range-model-lib
```
