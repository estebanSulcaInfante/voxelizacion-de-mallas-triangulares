#include "parallel/voxelizer_omp.h"
#include "common/geometry.h"

#include <iostream>

static bool childBoundaryRequiresFlipImpl(OctreeNode* node, SparseOctree& octree);
static void flipInsideStateImpl(OctreeNode* node);

void propagateInsideOutside(SparseOctree& octree) {
    int h = octree.height;

    std::cout << "  Algoritmo 4: propagacion I/E" << std::endl;

    int chainsProcessed = 0;
    for (auto* leaf : octree.levels[0]) {
        if (leaf->xNeighborNeg == nullptr) {
            chainsProcessed++;
            OctreeNode* curr = leaf;
            while (curr->xNeighborPos != nullptr) {
                OctreeNode* next = curr->xNeighborPos;
                if (hasActiveBoundaryX(curr->SG)) {
                    flipAllSGVoxels(next->SG);
                }
                curr = next;
            }
        }
    }
    std::cout << "    Fase 1: " << chainsProcessed << " cadenas procesadas" << std::endl;

    for (int level = 1; level <= h; ++level) {
        for (auto* node : octree.levels[level]) {
            node->flip = 0;
            node->inside = 0;

            if (childBoundaryRequiresFlipImpl(node, octree)) {
                node->flip = 1;
            }
        }

        for (auto* head : octree.levels[level]) {
            if (head->xNeighborNeg == nullptr) {
                OctreeNode* curr = head;
                while (curr->xNeighborPos != nullptr) {
                    OctreeNode* next = curr->xNeighborPos;
                    if (curr->flip == 1) {
                        next->flip = 1 - next->flip;
                        next->inside = 1 - next->inside;
                    }
                    curr = next;
                }
            }
        }

        std::cout << "    Fase 2, nivel " << level << ": " << octree.levels[level].size() << " nodos" << std::endl;
    }

    for (int level = h; level >= 1; --level) {
        for (auto* node : octree.levels[level]) {
            if (node->inside == 1) {
                for (int j = 0; j < 8; ++j) {
                    if (node->children[j]) {
                        flipInsideStateImpl(node->children[j]);
                    }
                }
            }
        }
    }

    std::cout << "    Fase 3: propagacion descendente completa" << std::endl;
}

static bool childBoundaryRequiresFlipImpl(OctreeNode* node, SparseOctree& octree) {
    (void)octree;
    for (int j = 0; j < 8; ++j) {
        if ((j & 1) == 1) {
            OctreeNode* child = node->children[j];
            if (child) {
                if (child->level == 0) {
                    if (hasActiveBoundaryX(child->SG)) {
                        return true;
                    }
                } else {
                    if (child->flip == 1) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

static void flipInsideStateImpl(OctreeNode* node) {
    if (node->level == 0) {
        flipAllSGVoxels(node->SG);
    } else {
        node->inside = 1 - node->inside;
    }
}
