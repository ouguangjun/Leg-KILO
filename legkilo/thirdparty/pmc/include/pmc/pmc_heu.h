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

#ifndef PMC_HEU_H_
#define PMC_HEU_H_

#include "pmc/pmc_bool_vector.h"
#include "pmc_graph.h"
#include "pmc_input.h"
#include "pmc_utils.h"
#include "pmc_vertex.h"

#include <string>
#include <vector>

namespace pmc {

    class pmc_heu {
        public:
            std::vector<int> const* E;
            std::vector<long long> const* V;
            std::vector<int>* K;
            std::vector<int>* order;
            std::vector<int>* degree;
            double sec;
            int ub;
            std::string strat;

            int num_threads;

            pmc_heu(pmc_graph& G, input& params) {
                K = G.get_kcores();
                order = G.get_kcore_ordering();
                ub = params.ub;
                strat = params.heu_strat;
                num_threads = params.threads;
                initialize();
            }

            pmc_heu(pmc_graph& G, int tmp_ub) {
                K = G.get_kcores();
                order = G.get_kcore_ordering();
                ub = tmp_ub;
                strat = "kcore";
                initialize();
            }

            inline void initialize() {
                sec = get_time();
                srand (time(NULL));
            };

            int strategy(std::vector<int>& P);
            void set_strategy(std::string s) { strat = s; }
            int compute_heuristic(int v);

            static bool desc_heur(Vertex v,  Vertex u) {
                return (v.get_bound() > u.get_bound());
            }

            static bool incr_heur(Vertex v,  Vertex u) {
                return (v.get_bound() < u.get_bound());
            }

            int search(const pmc_graph& graph, std::vector<int>& C_max);
            int search_cores(const pmc_graph& graph, std::vector<int>& C_max, int lb);
            int search_bounds(const pmc_graph& graph, std::vector<int>& C_max);

            inline void branch(std::vector<Vertex>& P, int sz,
                    int& mc, std::vector<int>& C, bool_vector& ind);

            void print_info(const std::vector<int>& C_max) const;
    };
};
#endif
