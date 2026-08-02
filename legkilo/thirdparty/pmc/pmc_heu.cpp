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

#include "pmc/pmc_bool_vector.h"
#include "pmc/pmc_debug_utils.h"
#include "pmc/pmc_heu.h"

#include <algorithm>

using namespace pmc;


void pmc_heu::branch(std::vector<Vertex>& P, int sz,
        int& mc, std::vector<int>& C, bool_vector& ind) {

    if (!P.empty()) {

        const int u = P.back().get_id();
        P.pop_back();

        for (long long j = (*V)[u]; j < (*V)[u + 1]; j++)  ind[(*E)[j]] = true;

        std::vector<Vertex> R;
        R.reserve(P.size());

        std::copy_if(P.begin(), P.end(), std::back_inserter(R),
                     [this, mc, &ind](const Vertex &v) -> bool {
                       return ind[v.get_id()] && (*K)[v.get_id()] > mc;
                     });

        for (long long j = (*V)[u]; j < (*V)[u + 1]; j++)  ind[(*E)[j]] = false;

        const int mc_prev = mc;
        branch(R, sz + 1, mc, C, ind);

        if (mc > mc_prev)  C.push_back(u);

        P.clear();
    }
    else if (sz > mc)
        mc = sz;
    return;
}

int pmc_heu::search_bounds(const pmc_graph& G, std::vector<int>& C_max) {
    V = &G.get_vertices();
    E = &G.get_edges();

    std::vector<int> C;
    std::vector<Vertex> P;

    C_max.reserve(ub);
    C.reserve(ub);
    P.reserve(G.get_max_degree()+1);

    bool_vector ind(G.num_vertices(), false);

    bool found_ub = false;
    int mc = 0;

    #pragma omp parallel for schedule(dynamic) \
        shared(G, mc, C_max, found_ub) \
        private(P, C) firstprivate(ind) \
        num_threads(num_threads)
    for (int i = G.num_vertices()-1; i >= 0; --i) {
        bool found_ub_local = false;
        #pragma omp atomic read acquire
        found_ub_local = found_ub;

        if (found_ub_local) {
            continue;
        }

        const int v = (*order)[i];

        int mc_prev, mc_cur;
        {
            #pragma omp critical
            mc_prev = mc_cur = mc;
        }

        if ((*K)[v] > mc_cur) {
            for (long long j = (*V)[v]; j < (*V)[v + 1]; j++)
                if ((*K)[(*E)[j]] > mc_cur)
                    P.emplace_back((*E)[j], compute_heuristic((*E)[j]));

            if (P.size() > mc_cur) {
                std::sort(P.begin(), P.end(), incr_heur);
                branch(P, 1 , mc_cur, C, ind);

                if (mc_cur >= ub) {
                    #pragma omp atomic write release
                    found_ub = true;
                }

                if (mc_cur > mc_prev) {
                    C.push_back(v);

                    #pragma omp critical
                    if (mc_cur > mc) {
                        mc = mc_cur;

                        std::swap(C_max, C);
                        print_info(C_max);
                    }
                }
            }
            C.clear();
        }
    }
    DEBUG_PRINTF("[pmc heuristic]\t mc = %i\n", mc);
    return mc;
}


int pmc_heu::compute_heuristic(int v) {
    if (strat == "kcore_deg") 	return (*K)[v] * (*degree)[v];
    else if (strat == "deg")    return (*degree)[v];
    else if (strat == "kcore") 	return (*K)[v];
    else if (strat == "rand")  	return rand() % (*V).size();
    else if (strat == "var")    return (*K)[v] * ((int)(*degree)[v]/(*K)[v]);
    return v;
}


int pmc_heu::search_cores(const pmc_graph& G, std::vector<int>& C_max, int lb) {
    std::vector <int> C;
    std::vector<Vertex> P;

    C_max.reserve(ub);
    C.reserve(ub);
    P.reserve(G.get_max_degree()+1);

    bool_vector ind(G.num_vertices(), false);

    int mc = lb;

    int lb_idx = 0;
    for (int i = G.num_vertices()-1; i >= 0; i--) {
        const int v = (*order)[i];
        if ((*K)[v] == lb)   lb_idx = i;
    }

    #pragma omp parallel for schedule(dynamic) \
        shared(G, mc, C_max) \
        private(P, C) firstprivate(ind) \
        num_threads(num_threads)
    for (int i = lb_idx; i < G.num_vertices(); i++) {
        const int v = (*order)[i];

        int mc_prev, mc_cur;
        {
            #pragma omp critical
            mc_prev = mc_cur = mc;
        }

        if ((*K)[v] > mc_cur) {
            for (long long j = (*V)[v]; j < (*V)[v + 1]; j++)
                if ((*K)[(*E)[j]] > mc_cur)
                    P.emplace_back((*E)[j], compute_heuristic((*E)[j]));

            if (P.size() > mc_cur) {
                std::sort(P.begin(), P.end(), incr_heur);
                branch(P, 1 , mc_cur, C, ind);

                if (mc_cur > mc_prev) {
                    C.push_back(v);

                    #pragma omp critical
                    if (mc_cur > mc) {
                        mc = mc_cur;

                        std::swap(C_max, C);
                        print_info(C_max);
                    }
                }
            }
            C.clear();
        }
    }
    DEBUG_PRINTF("[search_cores]\t mc = %i\n", mc);
    return mc;
}


int pmc_heu::search(const pmc_graph& G, std::vector<int>& C_max) {
    return search_bounds(G, C_max);
}


void pmc_heu::print_info(const std::vector<int>& C_max) const {
    DEBUG_PRINTF("*** [pmc heuristic: thread %i", omp_get_thread_num() + 1);
    DEBUG_PRINTF("]   current max clique = %i", C_max.size());
    DEBUG_PRINTF(",  time = %i sec\n", get_time() - sec);
}

