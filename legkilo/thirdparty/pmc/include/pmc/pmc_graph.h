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

#ifndef PMC_GRAPH_H_
#define PMC_GRAPH_H_

#include "pmc/pmc_bool_vector.h"
#include "pmc_vertex.h"

#include <map>
#include <string>
#include <vector>

namespace pmc {
    class pmc_graph {
        private:
            // helper functions
            void read_mtx(const std::string& filename);
            void read_edges(const std::string& filename);
            void read_metis(const std::string& /*filename*/) {}

        public:
            std::vector<int> edges;
            std::vector<long long> vertices;
            std::vector<int> degree;
            int min_degree;
            int max_degree;
            double avg_degree;
            bool is_gstats;
            std::string fn;
            std::vector<bool_vector> adj;

            // constructor
            pmc_graph(const std::string& filename);
            pmc_graph(bool graph_stats, const std::string& filename);
            pmc_graph(const std::string& filename, bool make_adj);
            pmc_graph(std::vector<long long> vs, std::vector<int> es) {
                edges = std::move(es);
                vertices = std::move(vs);
                vertex_degrees();
            }
            pmc_graph(long long nedges, const int *ei, const int *ej, int offset);
            pmc_graph(const std::map<int, std::vector<int>>& v_map);

            // destructor
            ~pmc_graph();

            void read_graph(const std::string& filename);
            void create_adj();
            void reduce_graph(const bool_vector& pruned);
            void reduce_graph(
                    std::vector<long long>& vs,
                    std::vector<int>& es,
                    const bool_vector& pruned);

            int num_vertices() const noexcept { return vertices.size() - 1; }
            int num_edges() const noexcept { return edges.size()/2; }
            const std::vector <long long>& get_vertices() const noexcept { return vertices; }
            const std::vector<int>& get_edges() const noexcept { return edges; }
            std::vector<int>* get_degree(){ return &degree; }
            std::vector<int> get_edges_array() { return edges; }
            std::vector<long long> get_vertices_array() { return vertices; };
            std::vector<long long> e_v, e_u, eid;

            int vertex_degree(int v) const noexcept { return vertices[v] - vertices[v+1]; }
            long long first_neigh(int v) const noexcept { return vertices[v]; }
            long long last_neigh(int v) const noexcept { return vertices[v+1]; }

            void sum_vertex_degrees();
            void vertex_degrees();
            void update_degrees();
            void update_degrees(bool flag);
            void update_degrees(bool_vector& pruned, int& mc);
            double density() const noexcept { return (double)num_edges() / (num_vertices() * (num_vertices() - 1.0) / 2.0); }
            int get_max_degree() const noexcept { return max_degree; }
            int get_min_degree() const noexcept { return min_degree; }
            double get_avg_degree() const noexcept { return avg_degree; }

            void initialize();
            std::string get_file_extension(const std::string& filename);
            void basic_stats(double sec);
            void bound_stats(int alg);

            // vertex sorter
            void compute_ordering(std::vector<int>& bound, std::vector<int>& order);
            void compute_ordering(std::string degree, std::vector<int>& order);
            // edge sorters
            void degree_bucket_sort();
            void degree_bucket_sort(bool desc);

            int max_core;
            std::vector<int> kcore;
            std::vector<int> kcore_order;
            std::vector<int>* get_kcores() { return &kcore; }
            std::vector<int>* get_kcore_ordering() { return &kcore_order; }
            int get_max_core() const noexcept { return max_core; }
            void update_kcores(const bool_vector& pruned);

            void compute_cores();
            void induced_cores_ordering(
                    std::vector<long long>& V,
                    std::vector<int>& E);

            // clique utils
            int initial_pruning(pmc_graph& G, bool_vector& pruned, int lb);
            int initial_pruning(pmc_graph& G, bool_vector& pruned, int lb, std::vector<bool_vector>& adj);
            void order_vertices(std::vector<Vertex> &V, pmc_graph &G,
                    int &lb_idx, int &lb, std::string vertex_ordering, bool decr_order);

            void print_info(std::vector<int> &C_max, double &sec);
            void print_break();
            bool time_left(std::vector<int> &C_max, double sec,
                    double time_limit, bool &time_expired_msg);
            void graph_stats(pmc_graph& G, int& mc, int id, double &sec);

            void reduce_graph(
                    std::vector<long long>& vs,
                    std::vector<int>& es,
                    const bool_vector& pruned,
                    pmc_graph& G);

            bool clique_test(pmc_graph& G, std::vector<int> C);
    };

}
#endif
