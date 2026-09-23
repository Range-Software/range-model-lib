## Version 1.3.1

### Improvements

- **RProblemTaskItem** carries a `cvgValue` next to `nIterations`. A task group
  ends its iterations as soon as every task in it reports a convergence below
  this value, instead of always running the full count. Zero or less runs all of
  them, which is the former behaviour
- The value defaults to `1e-5`. A model saved by an earlier version carries no
  such field and picks up that default on load, so the early exit applies to
  existing models as well
- The model file format version was raised to **1.3.1**. The new field is read
  only from files newer than 1.3.0, so models written by earlier versions still
  load
- **RMaterial::isFluid()** tells a fluid from a solid. A gas or a liquid state
  is a fluid and a solid state is not; a material whose state is not specified -
  every material in the shipped database - is a fluid when it carries a dynamic
  viscosity. The heat solver uses it to leave fluid domains to the fluid heat
  solver

### Bug fixes

- Four electrostatic variables carry the units they are actually in.
  **Relative permittivity** was labelled `C^2` although it is a dimensionless
  multiplier of the vacuum permittivity, and is now `N/A` like the emissivity
  and the Poisson ratio; **Charge density** was labelled `C` although the solver
  reads it as a volumetric density, and is now `C/m^3`
- **Electric energy** and **Joule heat** are `J/m^3` and `W/m^3` rather than `J`
  and `W`. Both have always been densities - the energy density of the field and
  the dissipation density a heat task integrates over the element - and the
  labels were the only thing saying otherwise
- The units are used for display and in the statistics the solver log prints.
  They are not written to a model or a material file, so no stored data changes
- The **Magnetic field** result variable is registered against the magneto-statics
  problem type rather than electro-statics. It reached the results and monitoring
  point lists only because a magnetostatic task always brings an electrostatic
  one with it, and it was offered by an electro-statics task which never
  computes it
- **RModel::breakIntersectedElements()** no longer stops with an assertion while
  removing duplicate elements. A duplicate is marked by folding its second node
  onto its first, which a point element has no second node for, so two point
  elements left coincident by the node merge before it failed the
  `position < nodeIDs.size()` assertion of `RElement::setNodeId()`. Duplicate
  point elements are collected and removed with the degenerated ones instead
- An intersection point which already is a vertex of the element it was found on
  is no longer collected as a point to break that element with. Such a point can
  not break anything - it was added as a node, produced no new element and was
  merged away again on every iteration - which is why a model whose bodies meet
  at shared vertices kept finding the same intersections for as many iterations
  as it was given. The comment above the check had described this all along
- The geometric tolerance is relative to the size of the model rather than a
  fixed hundred machine epsilons. An absolute `2.2e-14` is at or below the
  rounding error of the intersection arithmetic for anything but a model of unit
  size, and it made the outcome depend on the units the model was built in. It is
  now one part in `1e9` of the bounding box diagonal, clamped to a thousandth of
  the shortest distance between two nodes of an element so that it can not weld
  geometry which is meant to be there, and floored at the former value. The value
  in force is logged
- The iteration stops when it stops making progress. It ran its whole iteration
  count whenever any intersection point was found, whether or not anything could
  be broken. It now stops when no element could be broken, and when the number of
  intersected elements has not changed for three iterations; both say so in the
  log, and the number of elements actually broken is reported per iteration
- The loop merging near and duplicate nodes counts down from the node count
  rather than from the last node index. `getNNodes()-1` underflows on a model
  with no nodes, and the `i >= oldNNodes` test can never fail when `oldNNodes` is
  zero, so the index wrapped around instead of the loop ending

## Version 1.2.0

### Improvements

- **RMaterial** added support for JSON file format
- Class **RFileManager** changed to namespace **RFileUtils**
- **RAcousticSetup** new class selecting between a transient and a harmonic
  acoustic analysis and holding the frequency sweep and the sound pressure
  level reference pressure. It is part of **RProblemSetup** and is read and
  written by **RFileIO**
- **RMaterialProperty** added *Acoustic damping factor*; *Speed of sound* is now
  offered for acoustic problems, as an alternative to deriving the speed of
  sound from the modulus of elasticity and the density
- **RMaterialProperty** properties can now be marked optional per problem type
  (`getOptionalProblemTypeMask()`), and `getRequiredTypes()` returns only the
  properties a solver genuinely needs. Acoustics uses it to accept either a
  speed of sound or a modulus of elasticity without reporting the other as
  missing
- **RBoundaryCondition** added *Acoustic impedance*; *Absorbing boundary* gained
  an absorption coefficient component and no longer applies to volumes; the
  acoustic *Velocity* boundary condition may now be applied to points and lines
  as well as surfaces
- **RInitialCondition** added *Velocity potential / s*, which sets the initial
  acoustic pressure of a transient analysis
- **RVariable** added the acoustic absorption coefficient, damping factor,
  impedance, intensity, phase and sound pressure level, plus the imaginary part
  of the velocity potential
- **RProblem** acoustics is part of `R_PROBLEM_ALL` again, so it is offered by
  the application
- **RModel::write()** numbers result records by the swept frequency of a
  harmonic acoustic analysis

- **RVariable** the individual stress components and the *Force* result are
  reported as results of a stress analysis, not only of a modal analysis
- **RInitialCondition** *Velocity* is no longer offered for stress problems -
  the stress solver never read it

- **RBoundaryCondition** *Forced convection* gained a *Fluid temperature*
  component. The heat solver takes the temperature from the fluid heat solver
  wherever it has one and uses this value only where it has none, so a surface
  bordering no meshed fluid can still be convected
- **RBoundaryCondition::getDefaultComponentValue()** new, returning the value a
  component of a given condition is created with. *Forced convection* and
  *Natural convection* start out holding the properties of dry air at 20 degrees
  Celsius - density, dynamic viscosity, heat capacity, thermal conductivity and
  the ideal gas expansion coefficient - so only the flow and the geometry are
  left to enter. Every other condition keeps the initial value of its variable
- **REntityGroupData::findVariablesByDisplayType()** new, returning every
  variable marked with a display type rather than only the first
- **REntityGroupData::clearVariableDisplayType()** new, removing a display type
  from every variable of an entity. Only one variable at a time can be displayed
  as scalar or applied as displacement, so assigning one has to clear the others
- **RModel::findVariablePositionByDisplayType()** new, resolving what an entity
  displays to a variable the model actually holds

### Bug fixes

- **RModel::findVariableByDisplayType()** returned nothing when the first
  variable an entity marks for display is missing from the model, even with a
  second, valid one marked as well. Entity groups keep display data for
  variables of a previously solved problem type, and such a left over entry hid
  the valid one. The lookup now skips variables the model does not hold
- **RConditionComponent** the *enabled* flag was left uninitialised on default
  construction. It is now true, which is what every existing model file assumes
- **RSparseMatrix::mlt()** indexed the multiplied vector by the storage position
  of a value inside its row instead of by the column the value belongs to, so
  the product was wrong for any matrix which is not fully populated. It also
  left whatever the result vector held before in place, adding the product on
  top of it. This is the operation the eigen value solver builds its iteration
  on, which is why the modal analysis results were wrong
- **REigenValueSolverConf** the `Arnoldi` and `Rayleigh` methods were renamed to
  `SubspaceIteration` and `InversePowerIteration`, matching the algorithms the
  solver now uses
- **RBoundaryCondition** added an *explicit local direction* flag. When it is
  set, the stored local direction replaces the one a solver would otherwise
  derive from the geometry of the entity. Existing models read back with the
  flag clear, so their behaviour is unchanged

### File format

- Model file version raised to **1.3.0**. Models written by this version carry
  the acoustic setup and the explicit local direction flag of a boundary
  condition; models written by an earlier version are still read, and both fall
  back to their defaults

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
