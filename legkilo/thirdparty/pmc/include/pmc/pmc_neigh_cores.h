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

#ifndef PMC_NEIGH_CORES_H_
#define PMC_NEIGH_CORES_H_

#include "pmc_vertex.h"

#include <vector>

namespace pmc {

    static void neigh_cores_bound(
            std::vector<long long>& vs,
            std::vector<int>& es,
            std::vector<Vertex> &P,
            std::vector<short>& ind,
            int& mc) {

        int n = P.size() + 1;

        // lookup table
        std::vector<int> newids_to_actual(n, 0);
        std::vector<int> vert_order(n,0);
        std::vector<int> deg(n,0);
        std::vector<int> pos(n,0);

        // lookup table for neighbors
        for (int v = 1; v < n; v++) ind[P[v-1].get_id()] = 1;

        // compute degrees of induced neighborhood
        int md = 0, x, u;
        for (int v = 1; v < n; v++) { 	// for each v in P
            u = P[v-1].get_id();
            x = 0;
            for (long long j=vs[u]; j<vs[u+1]; j++) { //induced degree
                if (ind[es[j]]) x++;
            }
            deg[v] = x;
            if (deg[v] > md)  md = deg[v];
        }

        int md_end = md+1;
        std::vector<int> bin(md_end,0);
        for (int v = 1; v < n; v++) bin[deg[v]]++;

        int start = 1, num = 0;
        for (int d=0; d < md_end; d++) { //for each deg, set bin to be the pos of the first vertex of that degree
            num = bin[d];
            bin[d] = start;
            start = start + num;
        }


        for (int v=1; v<n; v++) {
            pos[v] = bin[deg[v]];

            //view this step as relabeling the vertices
            vert_order[pos[v]] = v;
            ind[P[v-1].get_id()] = v; 		   		// set bit for actual vertex id
            newids_to_actual[v] = P[v-1].get_id();
            bin[deg[v]]++;
        }

        for (int d=md; d > 1; d--)  bin[d] = bin[d-1];
        bin[0] = 1;


        int v_newid, v_actual, u_newid, du, pu, pw, w;
        long long j = 0;
        for (int i = 1; i < n; i++) {  							// neighborhood K-cores
            v_newid = vert_order[i]; 							//relabeled id
            v_actual = newids_to_actual[v_newid]; 				// real id
            for (j = vs[v_actual]; j<vs[v_actual+1]; j++) {
                if (ind[es[j]] > 0) { 							// find common induced neighbors of k

                    u_newid = ind[es[j]];
                    if (deg[u_newid] > deg[v_newid]) {
                        du = deg[u_newid];
                        pu = pos[u_newid];
                        pw = bin[du];
                        w = vert_order[pw];
                        if (u_newid != w) {
                            pos[u_newid] = pw;
                            vert_order[pu] = w;
                            pos[w] = pu;
                            vert_order[pw] = u_newid;
                        }
                        bin[du]++;   deg[u_newid]--;
                    }
                }
            }
        }

        // reset index
        for (int v=1; v<n; v++) ind[P[v-1].get_id()] = 0;


        // neighborhood core pruning and sorting
        int id = 0, prune_vert = 0;
        for (int i = n-1; i > 0; --i) {
            u = vert_order[i];
            if (deg[u]+1 > mc) {
                P[id].set_bound(deg[u]);
                P[id].set_id(newids_to_actual[u]);
                id++;
            }
            else prune_vert++;
        }

        // remove pruned verts from P
        for (int i = 0; i < prune_vert; i++)
            P.pop_back();
    }


    static void neigh_cores_tight(
            std::vector<long long>& vs,
            std::vector<int>& es,
            std::vector<Vertex> &P,
            std::vector<short>& ind,
            int& mc) {

        int n = P.size() + 1;

        // lookup table
        std::vector<int> newids_to_actual(n, 0);
        std::vector<int> vert_order(n,0);
        std::vector<int> deg(n,0);
        std::vector<int> pos(n,0);


        // lookup table for neighbors
        for (int v = 1; v < n; v++) ind[P[v-1].get_id()] = 1;

        // compute degrees of induced neighborhood
        int md = 0, x, u;
        for (int v = n-1; v >= 1; v--) {
            u = P[v-1].get_id();
            x = 0;
            for (long long j=vs[u]; j<vs[u+1]; j++) { //induced degree
                if (ind[es[j]]) x++;
            }
            if (x >= mc) {
                deg[v] = x;
                if (deg[v] > md)  md = deg[v];
            }
            else {
                deg[v] = 0;
                ind[P[v-1].get_id()] = 0;
            }
        }

        int md_end = md+1;
        std::vector<int> bin(md_end,0);
        for (int v = 1; v < n; v++) bin[deg[v]]++;

        int start = 1, num = 0;
        for (int d=0; d < md_end; d++) { //for each deg, set bin to be the pos of the first vertex of that degree
            num = bin[d];
            bin[d] = start;
            start = start + num;
        }


        for (int v=1; v<n; v++) {
            if (deg[v] > 0) {
                pos[v] = bin[deg[v]];

                //view this step as relabeling the vertices
                vert_order[pos[v]] = v;
                ind[P[v-1].get_id()] = v; 		   		// set bit for actual vertex id
                newids_to_actual[v] = P[v-1].get_id();
                bin[deg[v]]++;
            }
            else
                ind[P[v-1].get_id()] = 0;
        }

        for (int d=md; d > 1; d--)  bin[d] = bin[d-1];
        bin[0] = 1;


        int v_newid, v_actual, u_newid, du, pu, pw, w;
        long long j = 0;
        for (int i = 1; i < n; i++) {  							// neighborhood K-cores
            v_newid = vert_order[i];
            if (deg[v_newid] > 0) {
                //relabeled id
                v_actual = newids_to_actual[v_newid]; 				// real id
                for (j = vs[v_actual]; j<vs[v_actual+1]; j++) {
                    if (ind[es[j]] > 0) { 							// find common induced neighbors of k

                        u_newid = ind[es[j]];
                        if (deg[u_newid] > deg[v_newid]) {
                            du = deg[u_newid];
                            pu = pos[u_newid];
                            pw = bin[du];
                            w = vert_order[pw];
                            if (u_newid != w) {
                                pos[u_newid] = pw;
                                vert_order[pu] = w;
                                pos[w] = pu;
                                vert_order[pw] = u_newid;
                            }
                            bin[du]++;   deg[u_newid]--;
                        }
                    }
                }
            }
        }

        // reset index
        for (int v=1; v<n; v++) ind[P[v-1].get_id()] = 0;


        // neighborhood core pruning and sorting
        int id = 0, prune_vert = 0;
        for (int i = n-1; i > 0; --i) {
            u = vert_order[i];
            if (deg[u]+1 > mc) {
                P[id].set_bound(deg[u]);
                P[id].set_id(newids_to_actual[u]);
                id++;
            }
            else prune_vert++;
        }

        // remove pruned verts from P
        for (int i = 0; i < prune_vert; i++)
            P.pop_back();
    }
}


#endif
