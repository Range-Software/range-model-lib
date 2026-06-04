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
