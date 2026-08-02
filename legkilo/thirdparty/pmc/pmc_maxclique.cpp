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

#include "pmc/pmc_debug_utils.h"
#include "pmc/pmc_maxclique.h"

#include <cstring>

using namespace std;
using namespace pmc;

int pmc_maxclique::search(pmc_graph& G, vector<int>& sol) {

    vertices = &G.get_vertices();
    edges = &G.get_edges();
    degree = G.get_degree();
    bool_vector pruned(G.num_vertices());
    int mc = lb, i = 0, u = 0;

    // initial pruning
    int lb_idx = G.initial_pruning(G, pruned, lb);

    // set to worst case bound of cores/coloring
    vector<Vertex> P, T;
    P.reserve(G.get_max_degree()+1);
    T.reserve(G.get_max_degree()+1);

    vector<int> C, C_max;
    C.reserve(G.get_max_degree()+1);
    C_max.reserve(G.get_max_degree()+1);

    // order verts for our search routine
    vector<Vertex> V;   V.reserve(G.num_vertices());
    G.order_vertices(V,G,lb_idx,lb,vertex_ordering,decr_order);

    vector<short> ind(G.num_vertices(),0);

    #pragma omp parallel for schedule(dynamic) \
        shared(pruned, G, T, V, mc, C_max) firstprivate(ind) private(u, P, C) num_threads(num_threads)
    for (i = 0; i < (V.size()) - (mc-1); ++i) {
        if (G.time_left(C_max,sec,time_limit,time_expired_msg)) {

            u = V[i].get_id();
            if ((*bound)[u] > mc) {
                P.push_back(V[i]);
                for (long long j = (*vertices)[u]; j < (*vertices)[u + 1]; ++j)
                    if (!pruned[(*edges)[j]])
                        if ((*bound)[(*edges)[j]] > mc)
                            P.push_back(Vertex((*edges)[j], (*degree)[(*edges)[j]]));

                if (P.size() > mc) {
                    branch(P, ind, C, C_max, pruned, mc);
                }
                P = T;
            }
            pruned[u] = true;
        }
    }

    sol.resize(mc);
    for (int i = 0; i < C_max.size(); i++)  sol[i] = C_max[i];
    G.print_break();
    return sol.size();
}




void pmc_maxclique::branch(
        vector<Vertex> &P,
        vector<short>& ind,
        vector<int>& C,
        vector<int>& C_max,
        const bool_vector& pruned,
        int& mc) {

    // stop early if ub is reached
    if (not_reached_ub) {
        while (P.size() > 0) {
            // terminating condition
            if (C.size() + P.size() > mc) {
                int v = P.back().get_id();   C.push_back(v);

                vector<Vertex> R;   R.reserve(P.size());
                for (long long j = (*vertices)[v]; j < (*vertices)[v + 1]; j++)   ind[(*edges)[j]] = 1;

                // intersection of N(v) and P - {v}
                for (int k = 0; k < P.size() - 1; k++)
                    if (ind[P[k].get_id()])
                        if (!pruned[P[k].get_id()])
                            if ((*bound)[P[k].get_id()] > mc)
                                R.push_back(P[k]);

                for (long long j = (*vertices)[v]; j < (*vertices)[v + 1]; j++)  ind[(*edges)[j]] = 0;

                if (R.size() > 0) {
                    branch(R, ind, C, C_max, pruned, mc);
                }
                else if (C.size() > mc) {
                    // obtain lock
                    #pragma omp critical (update_mc)
                    if (C.size() > mc) {
                        // ensure updated max is flushed
                        mc = C.size();
                        C_max = C;
                        print_mc_info(C,sec);
                        if (mc >= param_ub) {
                            not_reached_ub = false;
                            DEBUG_PRINTF("[pmc: upper bound reached]  omega = %i\n", mc);
                        }
                    }

                }
                // backtrack and search another branch
                R.clear();
                C.pop_back();
            }
            else return;
            P.pop_back();
        }
    }
}






/**
 * Dense graphs: we use ADJ matrix + CSC Representation
 * ADJ:	  O(1) edge lookups
 * CSC:	  O(1) time to compute degree
 *
 */

int pmc_maxclique::search_dense(pmc_graph& G, vector<int>& sol) {

    vertices = &G.get_vertices();
    edges = &G.get_edges();
    degree = G.get_degree();
    auto adj = G.adj;

    bool_vector pruned(G.num_vertices());
    int mc = lb, i = 0, u = 0;

    // initial pruning
    int lb_idx = G.initial_pruning(G, pruned, lb, adj);

    // set to worst case bound of cores
    vector<Vertex> P, T;
    P.reserve(G.get_max_degree()+1);
    T.reserve(G.get_max_degree()+1);

    vector<int> C, C_max;
    C.reserve(G.get_max_degree()+1);
    C_max.reserve(G.get_max_degree()+1);

    // order verts for our search routine
    vector<Vertex> V;    V.reserve(G.num_vertices());
    G.order_vertices(V,G,lb_idx,lb,vertex_ordering,decr_order);

    vector<short> ind(G.num_vertices(),0);

    #pragma omp parallel for schedule(dynamic) \
        shared(pruned, G, adj, T, V, mc, C_max) firstprivate(ind) private(u, P, C) num_threads(num_threads)
    for (i = 0; i < (V.size()) - (mc-1); ++i) {
        if (G.time_left(C_max,sec,time_limit,time_expired_msg)) {

            u = V[i].get_id();
            if ((*bound)[u] > mc) {
                P.push_back(V[i]);
                for (long long j = (*vertices)[u]; j < (*vertices)[u + 1]; ++j)
                    if (!pruned[(*edges)[j]])
                        if ((*bound)[(*edges)[j]] > mc)
                            P.push_back(Vertex((*edges)[j], (*degree)[(*edges)[j]]));

                if (P.size() > mc) {
                    branch_dense(P, ind, C, C_max, pruned, mc, adj);
                }
                P = T;
            }
            pruned[u] = true;
            for (long long j = (*vertices)[u]; j < (*vertices)[u + 1]; j++) {
                adj[u][(*edges)[j]] = false;
                adj[(*edges)[j]][u] = false;
            }
        }
    }

    sol.resize(mc);
    for (int i = 0; i < C_max.size(); i++)  sol[i] = C_max[i];
    G.print_break();
    return sol.size();
}



void pmc_maxclique::branch_dense(
        vector<Vertex> &P,
        vector<short>& ind,
        vector<int>& C,
        vector<int>& C_max,
        bool_vector& pruned,
        int& mc,
        std::vector<bool_vector>& adj) {

    // stop early if ub is reached
    if (not_reached_ub) {
        while (P.size() > 0) {
            // terminating condition
            if (C.size() + P.size() > mc) {
                int v = P.back().get_id();   C.push_back(v);
                vector<Vertex> R;    R.reserve(P.size());

                for (int k = 0; k < P.size() - 1; k++)
                    // indicates neighbor AND pruned
                    if (adj[v][P[k].get_id()])
                        if ((*bound)[P[k].get_id()] > mc)
                            R.push_back(P[k]);

                if (R.size() > 0) {
                    branch_dense(R, ind, C, C_max, pruned, mc, adj);
                }
                else if (C.size() > mc) {
                    // obtain lock
                    #pragma omp critical (update_mc)
                    if (C.size() > mc) {
                        // ensure updated max is flushed
                        mc = C.size();
                        C_max = C;
                        print_mc_info(C,sec);
                        if (mc >= param_ub) {
                            not_reached_ub = false;
                            DEBUG_PRINTF("[pmc: upper bound reached]  omega = %i\n", mc);
                        }
                    }

                }
                // backtrack and search another branch
                R.clear();
                C.pop_back();
            }
            else return;
            P.pop_back();
        }
    }
}
