## Version 1.2.0

### Improvements

#### Problem setup

- **RProblemTaskItem** added `cvgValue` (default `1e-5`): a task group stops
  iterating once every task converges below it; zero or less runs all
  `nIterations` as before. Older models load with the default, so the early
  exit applies to them too
- **RAcousticSetup** new class, part of **RProblemSetup**, selecting transient
  or harmonic acoustic analysis and holding the frequency sweep and the sound
  pressure level reference pressure
- **RProblem** acoustics is back in `R_PROBLEM_ALL`, so the application offers it
- **REigenValueSolverConf** methods `Arnoldi` and `Rayleigh` renamed to
  `SubspaceIteration` and `InversePowerIteration` to match the algorithms used

#### Materials

- **RMaterial** added JSON file format support
- **RMaterial::isFluid()** new: gas and liquid states are fluids, solids are not;
  an unspecified state (every shipped database material) is a fluid if it has a
  dynamic viscosity. The heat solver uses it to leave fluid domains to the fluid
  heat solver
- **RMaterialProperty** added *Acoustic damping factor*; *Speed of sound* is
  offered for acoustics as an alternative to deriving it from the modulus of
  elasticity and density
- **RMaterialProperty** properties can be optional per problem type
  (`getOptionalProblemTypeMask()`); `getRequiredTypes()` returns only those a
  solver truly needs, so acoustics accepts either speed of sound or modulus of
  elasticity

#### Boundary and initial conditions

- **RBoundaryCondition** added *Acoustic impedance*; *Absorbing boundary* gained
  an absorption coefficient and no longer applies to volumes; acoustic
  *Velocity* may also be applied to points and lines
- **RBoundaryCondition** *Forced convection* gained *Fluid temperature*, used
  only where the fluid heat solver provides no temperature, so surfaces
  bordering no meshed fluid can still be convected
- **RBoundaryCondition::getDefaultComponentValue()** new. *Forced* and *Natural
  convection* start with dry air properties at 20 degrees Celsius (density,
  dynamic viscosity, heat capacity, thermal conductivity, ideal gas expansion
  coefficient); other conditions keep their variable's initial value
- **RBoundaryCondition** added an *explicit local direction* flag; when set, the
  stored local direction replaces the one derived from the entity geometry.
  Existing models load with it clear
- **RInitialCondition** added *Velocity potential / s*, setting the initial
  acoustic pressure of a transient analysis
- **RInitialCondition** *Velocity* no longer offered for stress problems (never
  read by the stress solver)

#### Results and display

- **RVariable** added acoustic absorption coefficient, damping factor,
  impedance, intensity, phase, sound pressure level and imaginary velocity
  potential
- **RVariable** individual stress components and *Force* are now results of
  stress analysis, not only modal analysis
- **RModel::write()** numbers result records of a harmonic acoustic analysis by
  swept frequency
- **REntityGroupData::findVariablesByDisplayType()** new, returning all
  variables with a display type, not just the first
- **REntityGroupData::clearVariableDisplayType()** new, removing a display type
  from all variables of an entity, since only one variable can be displayed as
  scalar or applied as displacement
- **RModel::findVariablePositionByDisplayType()** new, resolving an entity's
  display type to a variable the model actually holds
- Class **RFileManager** changed to namespace **RFileUtils**

### Bug fixes

- **RSparseMatrix::mlt()** indexed the vector by storage position instead of
  column and accumulated onto the existing result, giving wrong products for
  any matrix not fully populated; this broke the eigenvalue solver and thus
  modal analysis
- Electrostatic variable units corrected: **Relative permittivity** `C^2` to
  `N/A` (dimensionless), **Charge density** `C` to `C/m^3`, **Electric energy**
  `J` to `J/m^3`, **Joule heat** `W` to `W/m^3`. Units only affect display and
  solver log statistics; no stored data changes
- **Magnetic field** variable registered under magnetostatics instead of
  electrostatics, which never computes it
- **RModel::findVariableByDisplayType()** skips display entries for variables
  the model does not hold (left over from a previously solved problem type),
  which previously hid a valid second entry
- **RConditionComponent** *enabled* flag was uninitialised on default
  construction; it now defaults to true, as model files assume
- **RModel::breakIntersectedElements()**:
  - no longer asserts in `RElement::setNodeId()` when removing coincident
    duplicate point elements; they are removed with degenerated elements
  - ignores intersection points that are already vertices of the element, which
    broke nothing and made models with shared vertices find the same
    intersections on every iteration
  - geometric tolerance is now `1e-9` of the bounding box diagonal, clamped to
    a thousandth of the shortest distance between two element nodes and floored
    at the former fixed `2.2e-14`, so results no longer depend on model units;
    the value is logged
  - stops when no element could be broken or the intersected element count is
    unchanged for three iterations (both logged), and logs elements broken per
    iteration
  - node merge loop no longer underflows its index on a model with no nodes

### File format

- Model file version raised to **1.3.1**. Files from 1.3.0 on carry the acoustic
  setup and the boundary condition explicit local direction flag; files newer
  than 1.3.0 carry the task group `cvgValue`. Older files still load, with
  defaults for missing fields

---

## Version 1.0.1

### Improvements

- Added unit tests based on QTest framework

### Bug fixes

- **RTriangle::findLongestEdgeLength():** now returns the longest edge.

---

## Version 1.0.0

### Improvements

- **RSparseVector::addValue():** linear search (`std::find`) and full re-sort on every
  new entry replaced with binary search (`std::lower_bound`) and a single sorted
  insert, reducing per-call complexity from O(n log n) to O(log n + n) while
  keeping the vector sorted.
- **RSparseVector::addVector():** loop of per-entry `addValue()` calls replaced with a
  two-pointer O(n+m) sorted merge; eliminates the dominant cost in the per-thread
  matrix reduction loop in all solvers.
- **RSparseVector/RSparseMatrix:** added APIs to zero numeric values while
  preserving sparse indexes, add values by cached sparse position, and merge
  values for matrices with identical sparsity patterns. These support reusable
  sparse matrix patterns in solver assembly.
- **RSparseMatrix::findColumnPosition():** lookup now delegates to the sorted sparse
  vector binary search instead of copying row indexes and scanning them linearly.
- **RShapeGenerator::generateArrow():** hardcoded 8-point octagonal cone replaced
  with a parametric cos/sin loop over 16 sides, doubling cone smoothness while
  reducing code size.

### Bug fixes

- **RSurface::findAverageNormal():** now weights each element's normal by its area instead of averaging all normals equally.
