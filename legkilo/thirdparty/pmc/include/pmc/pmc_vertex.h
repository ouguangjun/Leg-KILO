/**
 ============================================================================
 Name        : Parallel Maximum Clique (PMC) Library
 Author      : Ryan A. Rossi   (rrossi@purdue.edu)
 Description : A general high-performance parallel framework for computing
               maximum cliques. The library is designed to be fast for large
               sparse graphs.

 Copyright (C) 2012-2013, Ryan A. Rossi, All rights reserved.

 Please cite the following paper if used:
   Ryan A. Rossi, David F. Gleich, Assefaw H. Gebremedhin, Md. Mostofa
   Patwary, A Fast Parallel Maximum Clique Algorithm for Large Sparse Graphs
   and Temporal Strong Components, arXiv preprint 1302.6256, 2013.

 See http://ryanrossi.com/pmc for more information.
 ============================================================================
 */

#ifndef PMC_VERTEX_H_
#define PMC_VERTEX_H_

#include "pmc/pmc_utils.h"
#include "pmc_debug_utils.h"

#include <vector>
#include <omp.h>

namespace pmc {
    class Vertex {
        private:
            int id, b;
        public:
            Vertex(int vertex_id, int bound): id(vertex_id), b(bound) {}

            void set_id(int vid) noexcept { id = vid; }
            int get_id() const noexcept { return id; }

            void set_bound(int value) noexcept { b = value; }
            int get_bound() const noexcept { return b; }
    };

    static bool decr_bound(Vertex v,  Vertex u) {
        return (v.get_bound() > u.get_bound());
    }
    static bool incr_bound(Vertex v,  Vertex u) {
        return (v.get_bound() < u.get_bound());
    };

    inline static void print_mc_info(const std::vector<int>& C_max, double sec) {
        DEBUG_PRINTF("*** [pmc: thread %i", omp_get_thread_num() + 1);
        DEBUG_PRINTF("]   current max clique = %i", C_max.size());
        DEBUG_PRINTF(",  time = %i sec\n", get_time() - sec);
    };
};
#endif
