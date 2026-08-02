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
#include "pmc/pmc_graph.h"
#include "pmc/pmc_utils.h"

#include <algorithm>
#include <iostream>

using namespace std;
using namespace pmc;

int pmc_graph::initial_pruning(pmc_graph& G, bool_vector& pruned, int lb) {
    int lb_idx = 0;
    for (int i = G.num_vertices()-1; i >= 0; i--) {
        if (kcore[kcore_order[i]] == lb)  lb_idx = i;
        if (kcore[kcore_order[i]] <= lb)  pruned[kcore_order[i]] = 1;
    }

    double sec = get_time();
    DEBUG_PRINTF("[pmc: initial k-core pruning]  before pruning: |V| = %i, |E| = %i\n", G.num_vertices(), G.num_edges());
    G.reduce_graph(pruned);
    DEBUG_PRINTF("[pmc: initial k-core pruning]  after pruning:  |V| = %i, |E| = %i\n", G.num_vertices() - lb_idx, G.num_edges());
    DEBUG_PRINTF("[pmc]  initial pruning took %i sec\n", get_time()-sec);

    G.update_degrees();
    G.degree_bucket_sort(true); // largest to smallest degree

    return lb_idx;
}


int pmc_graph::initial_pruning(pmc_graph& G, bool_vector& pruned, int lb, std::vector<bool_vector>& adj) {
    int lb_idx = 0;
    for (int i = G.num_vertices()-1; i >= 0; i--) {
        if (kcore[kcore_order[i]] == lb)  lb_idx = i;
        if (kcore[kcore_order[i]] <= lb) {
            pruned[kcore_order[i]] = 1;
            for (long long j = vertices[kcore_order[i]]; j < vertices[kcore_order[i] + 1]; j++) {
                adj[kcore_order[i]][edges[j]] = false;
                adj[edges[j]][kcore_order[i]] = false;
            }
        }
    }

    double sec = get_time();
    DEBUG_PRINTF("[pmc: initial k-core pruning]  before pruning: |V| = %i, |E| = %i\n", G.num_vertices(), G.num_edges());
    G.reduce_graph(pruned);
    DEBUG_PRINTF("[pmc: initial k-core pruning]  after pruning:  |V| = %i, |E| = %i\n", G.num_vertices() - lb_idx, G.num_edges());
    DEBUG_PRINTF("[pmc]  initial pruning took %i sec\n", get_time()-sec);

    G.update_degrees();
    G.degree_bucket_sort(true);

    return lb_idx;
}


void pmc_graph::order_vertices(vector<Vertex> &V, pmc_graph &G,
        int &lb_idx, int &lb, string vertex_ordering, bool decr_order) {

    srand (time(NULL));
    int u = 0, val = 0;
    for (int k = lb_idx; k < G.num_vertices(); k++) {
        if (degree[kcore_order[k]] >= lb - 1) {
            u = kcore_order[k];

            if (vertex_ordering == "deg")
                val = vertices[u + 1] - vertices[u];
            else if (vertex_ordering == "kcore")
                val = kcore[u];
            else if (vertex_ordering == "kcore_deg")
                val = degree[u] * kcore[u];
            else if (vertex_ordering == "rand")
                val = rand() % vertices.size();
            // neighbor degrees
            else if (vertex_ordering == "dual_deg") {
                val = 0;
                for (long long j = vertices[u]; j < vertices[u + 1]; j++) {
                    val = val + G.vertex_degree(edges[j]);
                }
            }
            // neighbor degrees
            else if (vertex_ordering == "dual_kcore") {
                val = 0;
                for (long long j = vertices[u]; j < vertices[u + 1]; j++) {
                    val = val + kcore[edges[j]];
                }
            }
            else  val = vertices[u + 1] - vertices[u];
            V.push_back(Vertex(u,val));
        }
    }
    if (decr_order)
        std::sort(V.begin(), V.end(), decr_bound);
    else
        std::sort(V.begin(), V.end(), incr_bound);
}


/**
 * Reduce the graph by removing the pruned vertices
 *   + Systematically speeds algorithm up by reducing the neighbors as more vertices are searched
 *
 * The algorithm below is for parallel maximum clique finders and has the following features:
 *   + Thread-safe, since local copy of vertices/edges are passed in..
 *   + Pruned is a shared variable, but it is safe, since only reads/writes can occur, no deletion
 */
void pmc_graph::reduce_graph(
        vector<long long>& vs,
        vector<int>& es,
        const bool_vector& pruned,
        pmc_graph& G) {

    int num_vs = vs.size();

    vector<long long> V(num_vs,0);
    vector<int> E;
    E.reserve(es.size());

    int start = 0;
    for (int i = 0; i < num_vs - 1; i++) {
        start = E.size();
        if (!pruned[i]) { //skip these V_local...
            for (long long j = vs[i]; j < vs[i + 1]; j++ ) {
                if (!pruned[es[j]])
                    E.push_back(es[j]);
            }
        }
        V[i] = start;
        V[i + 1] = E.size();
    }
    vs = V;
    es = E;

    // compute k-cores and share bounds: ensure operation completed by single process
    #pragma omp single nowait
    {
        cout << ">>> [pmc: thread " << omp_get_thread_num() + 1 << "]" <<endl;
        G.induced_cores_ordering(vs,es);
    }
    V.clear();
    E.clear();
}


void pmc_graph::print_info(vector<int> &C_max, double &sec) {
    cout << "*** [pmc: thread " << omp_get_thread_num() + 1;
    cout << "]   current max clique = " << C_max.size();
    cout << ",  time = " << get_time() - sec << " sec" <<endl;
}


void pmc_graph::print_break() {
    DEBUG_PRINTF("-----------------------------------------------------------------------\n");
}

bool pmc_graph::time_left(vector<int> &C_max, double sec, double time_limit, bool &time_expired_msg) {
    if ((get_time() - sec) > time_limit) {
        if (time_expired_msg) {
            DEBUG_PRINTF("\n### Time limit expired, terminating search. ###\n");
            DEBUG_PRINTF("Size: %i\n", C_max.size());
            print_max_clique(C_max);
            time_expired_msg = false;
        }
        return false;
    }
    return true;
}

void pmc_graph::graph_stats(pmc_graph& G, int& mc, int id, double &sec) {
    cout << "[pmc: bounds updated - thread " << omp_get_thread_num() + 1 << "]  ";
    cout << "time = " << get_time() - sec << " sec, ";
    cout << "|V| = " << (G.num_vertices() - id);
    cout << " (" << id << " / " << G.num_vertices();
    cout << "), |E| = " << G.num_edges();
    cout << ", w = " << mc;
    cout << ", p = " << G.density();
    cout << ", d_min = " << G.get_min_degree();
    cout << ", d_avg = " << G.get_avg_degree();
    cout << ", d_max = " << G.get_max_degree();
    cout << ", k_max = " << G.get_max_core();
    cout <<endl;
}
