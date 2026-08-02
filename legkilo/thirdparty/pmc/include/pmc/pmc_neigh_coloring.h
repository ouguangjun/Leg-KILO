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

#ifndef PMC_NEIGH_COLORING_H_
#define PMC_NEIGH_COLORING_H_

#include "pmc/pmc_bool_vector.h"
#include "pmc_vertex.h"

#include <vector>

namespace pmc {

    // sequential dynamic greedy coloring and sort
    static void neigh_coloring_bound(
            std::vector<long long>& vs,
            std::vector<int>& es,
            std::vector<Vertex> &P,
            std::vector<short>& ind,
            std::vector<int>& C,
            std::vector< std::vector<int> >& colors,
            int& mc) {

        int j = 0, u = 0, k = 1, k_prev = 0;
        int max_k = 1;
        int min_k = mc - C.size() + 1;
        colors[1].clear();   colors[2].clear();

        for (int w=0; w < P.size(); w++) {
            u = P[w].get_id();
            k = 1, k_prev = 0;

            for (long long h = vs[u]; h < vs[u + 1]; h++)  ind[es[h]] = 1;

            while (k > k_prev) {
                k_prev = k;
                for (int i = 0; i < colors[k].size(); i++) {
                    if (ind[colors[k][i]]) {
                        k++;
                        break;
                    }
                }
            }

            for (long long h = vs[u]; h < vs[u + 1]; h++)  ind[es[h]] = 0;

            if (k > max_k) {
                max_k = k;
                colors[max_k+1].clear();
            }

            colors[k].push_back(u);
            if (k < min_k) {
                P[j].set_id(u);
                j++;
            }
        }

        if (j > 0)  P[j-1].set_bound(0);
        if (min_k <= 0)  min_k = 1;

        for (k = min_k; k <= max_k; k++)
            for (int w = 0; w < colors[k].size(); w++) {
                P[j].set_id(colors[k][w]);
                P[j].set_bound(k);
                j++;
            }
    }

    // sequential dynamic greedy coloring and sort
    static void neigh_coloring_dense(
            std::vector<Vertex> &P,
            std::vector<int>& C,
            std::vector< std::vector<int> >& colors,
            int& mc,
            const std::vector<bool_vector>& adj) {

        int j = 0, u = 0, k = 1, k_prev = 0;
        int max_k = 1;
        int min_k = mc - C.size() + 1;

        colors[1].clear();   colors[2].clear();

        for (int w=0; w < P.size(); w++) {
            u = P[w].get_id();
            k = 1, k_prev = 0;

            while (k > k_prev) {
                k_prev = k;
                for (int i = 0; i < colors[k].size(); i++) { //use directly, sort makes it fast!
                    if (adj[u][colors[k][i]]) {
                        k++;
                        break;
                    }
                }
            }

            if (k > max_k) {
                max_k = k;
                colors[max_k+1].clear();
            }

            colors[k].push_back(u);
            if (k < min_k) {
                P[j].set_id(u);
                j++;
            }
        }

        if (j > 0)  P[j-1].set_bound(0);
        if (min_k <= 0)  min_k = 1;

        for (k = min_k; k <= max_k; k++)
            for (int w = 0; w < colors[k].size(); w++) {
                P[j].set_id(colors[k][w]);
                P[j].set_bound(k);
                j++;
            }
    }
}
#endif
