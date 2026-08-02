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

#ifndef PMC_INPUT_H_
#define PMC_INPUT_H_

#include "pmc_utils.h"

#include <iostream>
#include <omp.h>
#include <string>
#include <unistd.h>

namespace pmc {
class input {
    public:
        // instance variables
        int algorithm;
        int threads;
        int experiment;
        int lb;
        int ub;
        int param_ub;
        int adj_limit;
        double time_limit;
        double remove_time;
        bool graph_stats;
        bool verbose;
        bool help;
        bool MCE;
        bool decreasing_order;
        std::string heu_strat;
        std::string format;
        std::string graph;
        std::string output;
        std::string edge_sorter;
        std::string vertex_search_order;

        input() {
            // default values
            algorithm = 0;
            threads = omp_get_max_threads();
            experiment = 0;
            lb = 0;
            ub = 0;
            param_ub = 0;
            adj_limit = 20000;
            time_limit = 60 * 60; 			// max time to search
            remove_time = 4.0; 				// time to wait before reducing graph
            verbose = false;
            graph_stats = false;
            help = false;
            MCE = false;
            decreasing_order = false;
            heu_strat = "kcore";
            vertex_search_order = "deg";
            format = "mtx";
            graph = "data/sample.mtx";
            output = "";
            std::string edge_sorter = "";

            // both off, use default alg
            if (heu_strat == "0" && algorithm == -1)
                algorithm = 0;

            if (threads <= 0) threads = 1;
        }

        input(int argc, char *argv[]) {
            // default values
            algorithm = 0;
            threads = omp_get_max_threads();
            experiment = 0;
            lb = 0;
            ub = 0;
            param_ub = 0;
            adj_limit = 20000;
            time_limit = 60 * 60; 			// max time to search
            remove_time = 4.0; 				// time to wait before reducing graph
            verbose = false;
            graph_stats = false;
            help = false;
            MCE = false;
            decreasing_order = false;
            heu_strat = "kcore";
            vertex_search_order = "deg";
            format = "mtx";
            graph = "data/sample.mtx";
            output = "";
            std::string edge_sorter = "";

            int opt;
            while ((opt=getopt(argc,argv,"i:t:f:u:l:o:e:a:r:w:h:k:dgsv")) != EOF) {
                switch (opt) {
                    case 'a':
                        algorithm = atoi(optarg);
                        if (algorithm > 9) MCE = true;
                        break;
                    case 't':
                        threads = atoi(optarg);
                        break;
                    case 'f':
                        graph = optarg;
                        break;
                    case 's':
                        graph_stats = true;
                        break;
                    case 'u':
                        param_ub = atoi(optarg); // find k-clique fast
                        lb = 2;                  // skip heuristic
                        break;
                    case 'k':
                        param_ub = atoi(optarg);
                        lb = param_ub-1;
                        break;
                    case 'l':
                        lb = atoi(optarg);
                        break;
                    case 'h':
                        heu_strat = optarg;
                        break;
                    case 'v':
                        verbose = 1;
                        break;
                    case 'w':
                        time_limit = atof(optarg) * 60;  // convert minutes to seconds
                        break;
                    case 'r':
                        remove_time = atof(optarg);
                        break;
                    case 'e':
                        edge_sorter = optarg;
                        break;
                    case 'o':
                        vertex_search_order = optarg;
                        break;
                    case 'd':
                        // direction of which vertices are ordered
                        decreasing_order = true;
                        break;
                    case '?':
                        usage(argv[0]);
                        break;
                    default:
                        usage(argv[0]);
                        break;
                }
            }

            // both off, use default alg
            if (heu_strat == "0" && algorithm == -1)
                algorithm = 0;

            if (threads <= 0) threads = 1;

            if (!fexists(graph.c_str())) {
                usage(argv[0]);
                exit(-1);
            }

            FILE* fin = fopen(graph.c_str(), "r+t");
            if (fin == NULL) {
                usage(argv[0]);
                exit(-1);
            }
            fclose(fin);

            std::cout << "\n\nFile Name ------------------------ " << graph.c_str() << std::endl;
            if (!fexists(graph.c_str()) ) {
                std::cout << "File not found!" << std::endl;
                return;
            }
            std::cout << "workers: " << threads <<std::endl;
            omp_set_num_threads(threads);
        }

};
} // namespace pmc
#endif
