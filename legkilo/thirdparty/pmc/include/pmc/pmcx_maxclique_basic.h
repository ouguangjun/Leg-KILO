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

#ifndef PMCX_MAXCLIQUE_BASIC_H_
#define PMCX_MAXCLIQUE_BASIC_H_

#include "pmc_graph.h"
#include "pmc_input.h"
#include "pmc_utils.h"
#include "pmc_vertex.h"

#include <string>
#include <vector>

namespace pmc {

    class pmcx_maxclique_basic {
        public:
            std::vector<int>* bound;
            std::vector<int>* order;
            std::vector<int>* degree;
            int param_ub;
            int ub;
            int lb;
            double time_limit;
            double sec;
            double wait_time;
            bool not_reached_ub;
            bool time_expired_msg;
            bool decr_order;

            std::string vertex_ordering;
            int edge_ordering;
            int style_bounds;
            int style_dynamic_bounds;

            int num_threads;

            void initialize() {
                vertex_ordering = "deg";
                edge_ordering = 0;
                style_bounds = 0;
                style_dynamic_bounds = 0;
                not_reached_ub = true;
                time_expired_msg = true;
                decr_order = false;
            }

            void setup_bounds(input& params) {
                lb = params.lb;
                ub = params.ub;
                param_ub = params.param_ub;
                if (param_ub == 0)
                    param_ub = ub;
                time_limit = params.time_limit;
                wait_time = params.remove_time;
                sec = get_time();

                num_threads = params.threads;
            }


            pmcx_maxclique_basic(pmc_graph& G, input& params) {
                bound = G.get_kcores();
                order = G.get_kcore_ordering();
                setup_bounds(params);
                initialize();
                vertex_ordering = params.vertex_search_order;
                decr_order = params.decreasing_order;
            }

            ~pmcx_maxclique_basic() {};

            int search(pmc_graph& G, std::vector<int>& sol);

            void branch(
                    std::vector<long long>& vs,
                    std::vector<int>& es,
                    std::vector<Vertex> &P,
                    std::vector<short>& ind,
                    std::vector<int>& C,
                    std::vector<int>& C_max,
                    std::vector< std::vector<int> >& colors,
                    const bool_vector& pruned,
                    int& mc);


            int search_dense(pmc_graph& G, std::vector<int>& sol);

            void branch_dense(
                    std::vector<long long>& vs,
                    std::vector<int>& es,
                    std::vector<Vertex> &P,
                    std::vector<short>& ind,
                    std::vector<int>& C,
                    std::vector<int>& C_max,
                    std::vector< std::vector<int> >& colors,
                    bool_vector& pruned,
                    int& mc,
                    std::vector<bool_vector>& adj);

    };
};

#endif
