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
