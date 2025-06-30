#ifndef RML_TRIANGULATE_H
#define RML_TRIANGULATE_H

#include <vector>

#include <rbl_utils.h>

#include "rml_node.h"
#include "rml_element.h"

std::vector<RElement> RTriangulateNodes(const std::vector<RNode> &nodes, bool removeZeroSized, double zeroSize = RConstants::eps);

#endif /* RML_TRIANGULATE_H */
